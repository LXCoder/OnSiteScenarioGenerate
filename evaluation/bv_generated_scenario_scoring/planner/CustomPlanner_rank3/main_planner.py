#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import lib
import math

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches
import itertools
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from planner.CustomPlanner_rank3.local_utils import normalize_angle
from planner.CustomPlanner_rank3.routing import LaneGraph, AStarRoute
from planner.CustomPlanner_rank3.planner_components.entites import Ego, Obstacle, RefPoint
from planner.CustomPlanner_rank3.planner_components.path_optimizer import PathOptimizer
from planner.CustomPlanner_rank3.planner_components.speed_optimizer import SpeedQPOptimizer
from planner.CustomPlanner_rank3.planner_components.cubic_spline import CubicSpline2D
from planner.CustomPlanner_rank3.scenario_config import BaseConfig, LocalRoadConfig, HighwayConfig


class PublicRoadPlanner(PlannerBase):
    def __init__(self):
        self.origin = None
        self.destination = None
        self.road_info = None
        self.lane_graph = None
        self.scenario_name = None
        self.route = None
        self.route_cost = -1
        self.ego = None
        self.has_init = False
        self.need_warm = True
        self.warm_start_time = 0
        self.planner_start_time = None
        self.scenario_type = None
        self.config = None
        self.fallback_mode = False

        self.obs_st_boundaries = {}

    def init(self, scenario_dict):
        # print("----------------------------EM Planner INIT----------------------------")
        # print(scenario_dict)
        # print("----------------------------------------------------------------")
        self.origin = scenario_dict['task_info']['startPos']
        self.destination = scenario_dict['task_info']['targetPos']
        self.road_info = parse_opendrive(scenario_dict['source_file']['xodr'])
        self.scenario_name = scenario_dict['name']
        self.scenario_type = self.select_scenario_parameters()

        self.lane_graph = LaneGraph(self.road_info, self.origin, self.destination, self.scenario_name)
        self.lane_graph.run()
        route_alg = AStarRoute(self.lane_graph)
        self.route, self.route_cost = route_alg.calculate(True)
        if not self.route:
            self.fallback_mode = True
            self.has_init = False
            return
        # self.lane_graph.plot_route(self.route, self.origin, self.destination)
        self.ego = Ego(self.route,
                       [self.lane_graph.topo_graph[lane_id]["entity"].center_vertices for lane_id in self.route], self.lane_graph.topo_graph)

        self.path_s_dense = np.arange(self.config.path_s_start, self.config.path_s_dense_end,
                                      self.config.delta_path_s_dense)
        self.path_s_sparse = np.arange(self.config.path_s_dense_end, self.config.path_s_sparse_end,
                                       self.config.delta_path_s_sparse)
        self.path_s = np.concatenate([self.path_s_dense, self.path_s_sparse], axis=0)

        self.speed_t_dense = np.arange(self.config.speed_t_start, self.config.speed_t_dense_end,
                                       self.config.delta_speed_t_dense)
        self.speed_t_sparse = np.arange(self.config.speed_t_dense_end, self.config.speed_t_sparse_end,
                                        self.config.delta_speed_t_sparse)
        self.speed_t = np.concatenate([self.speed_t_dense, self.speed_t_sparse], axis=0)
        self.warm_start_time = self.config.warm_time

    def act(self, observation: Observation):
        if self.fallback_mode:
            v = float(getattr(observation.ego_info, "v", 0.0))
            if v > 8.0:
                return [-2.0, 0.0]
            if v > 2.0:
                return [-0.5, 0.0]
            return [0.0, 0.0]

        # 在每一个时间t初始化或者更新observation信息
        cur_t = float(observation.test_info.get("t", 0.0))
        if cur_t <= 1e-6 or self.has_init == False:
            self.ego.init(observation)
            self.has_init = True
            # 兼容 warmup=1 / warmup=31：暖启动时长按“规划器首次接管时刻”计算，而不是按全局t=0起算。
            if self.planner_start_time is None:
                self.planner_start_time = cur_t
                self.warm_start_time = self.planner_start_time + float(self.config.warm_time)

        else:
            self.ego.update(observation)

        # 根据当前红绿灯状态判断是否要停车，如果是红灯的话就停车
        if observation.light_info == "red" or observation.light_info == "yellow":
            self.warm_start_time += observation.test_info["dt"]
            return [-3.0 if observation.ego_info.v != 0 else 0, 0]

        # 判断优化是否需要暖启动，如果需要暖启动的话会relax一些约束条件
        if cur_t > self.warm_start_time:
            self.need_warm = False

        # 正式开始优化
        s_condition, d_condition = self.ego.to_frenet()
        ego_ref_points = self.reconstruct_ref_points()

        # 路径优化
        obstacles = self.create_obstacles(observation.object_info, ego_ref_points, self.ego.cur_ref_curve, observation.test_info["dt"])
        path_x_list, path_y_list = self.optimize_path_curve(ego_ref_points, d_condition, obstacles, self.need_warm, observation)

        path_ref_points, path_ref_curve, path_s_list = self.create_path_spine_curve(path_x_list, path_y_list)

        # 速度优化
        obstacles = self.create_obstacles(observation.object_info, path_ref_points, path_ref_curve, observation.test_info["dt"])
        # TODO: 障碍物轨迹+dt
        acc, self.obs_st_boundaries = self.optimize_speed_curve_qp(
            obstacles, path_x_list, path_y_list, path_s_list,
            observation
        )

        return self.generate_actions(acc, path_ref_curve, observation.ego_info.v, self.ego.cartesian_state.theta, observation.test_info["dt"])

    def select_scenario_parameters(self):
        """
        根据scenario名称确定场景类型，选择适应的参数
        :return:
        """
        if "highway" in self.scenario_name or "follow" in self.scenario_name or "cutin" in self.scenario_name or "lanechanging" in self.scenario_name:
            self.scenario_type = 0
            self.config = HighwayConfig()
        else:
            self.scenario_type = 1
            self.config = LocalRoadConfig()

    def reconstruct_ref_points(self):
        """
        根据车辆当前位置重新塑造参考线
        :return:
        """
        ref_points = []
        for s_interval in self.path_s:
            cur_s = self.ego.cur_rs + s_interval
            cur_x, cur_y = self.ego.cur_ref_curve.calc_position(cur_s)
            ref_points.append(RefPoint(
                rx=cur_x, ry=cur_y, rs=cur_s,
                rtheta=self.ego.cur_ref_curve.calc_theta(cur_s),
                rkappa=self.ego.cur_ref_curve.calc_curvature(cur_s),
                rdkappa=self.ego.cur_ref_curve.calc_dcurvature(cur_s)
            ))
        return ref_points


    def create_obstacles(self, obj_info, path_points, fitting_curves, dt):
        obstacles = {}
        for obj_type, obses in obj_info.items():
            for obs_key, obs_value in obses.items():
                obstacles[obs_key] = Obstacle(
                    obs_key,
                    obs_value.x, obs_value.y,
                    obs_value.v, obs_value.a, obs_value.yaw,
                    obs_value.length, obs_value.width,
                    obj_type, dt)

        for obs_key in obstacles.keys():
            obstacles[obs_key].match_points(path_points, fitting_curves)  ### 同样match障碍物与参考轨迹
            obstacles[obs_key].make_cartesian_state()  # 创建TrajPoint,为转换坐标系做准备
            obstacles[obs_key].convert_cartesian_to_frenet()  # 创建TrajPoint,为转换坐标系做准备

        return obstacles

    def optimize_path_curve(self, ref_points, d_condition, obstacles, warming=False, observation=None):
        truncated_left_bounds = np.array(self.ego.cur_drivable_boundaries["l"](self.path_s + ref_points[0].rs)).reshape(-1, 1) - self.ego.width / 2
        truncated_right_bounds = -np.array(self.ego.cur_drivable_boundaries["r"](self.path_s + ref_points[0].rs)).reshape(-1, 1) + self.ego.width / 2
        truncated_ref_points = ref_points
        path_opt = PathOptimizer(self.config,
            truncated_right_bounds, truncated_left_bounds,
            truncated_ref_points, self.ego.cur_ref_curve,
            self.path_s, obstacles, self.ego.cartesian_state.v,
            d_condition[0], d_condition[1], d_condition[2],
            ego_width=self.ego.width, ego_length=self.ego.length,
            warming=warming
        )

        path_opt.set_constraints()
        path_opt.set_optional_dynamic_obstacle_constraints(self.obs_st_boundaries, obstacles)
        path_opt.add_objective()
        res, status = path_opt.solve()

        # 如果加入动态障碍物的优化没有解，说明动态障碍物把路堵死了，去掉动态障碍物重新解一遍
        if status != "solved":
            path_opt.reset()
            path_opt.set_constraints()
            path_opt.add_objective()
            res, status = path_opt.solve()

        # for fragment case 5_103_straight_in_adjacent_left_106
        if status != "solved":
            # print("Warning, all obstacles have been omitted. It maybe caused by the red light.")
            path_opt.reset()
            path_opt.set_constraints(True)
            path_opt.add_objective()
            res, status = path_opt.solve()

        # for fragment case 4_75_straight_straight_76
        if status != "solved":
            # print("Warning, the path planning boundary may excceed the real boundary.")
            truncated_left_bounds = truncated_left_bounds + 2
            truncated_right_bounds = truncated_right_bounds - 2
            path_opt = PathOptimizer(self.config,
                                     truncated_right_bounds, truncated_left_bounds,
                                     truncated_ref_points, self.ego.cur_ref_curve,
                                     self.path_s, obstacles, self.ego.cartesian_state.v,
                                     d_condition[0], d_condition[1], d_condition[2],
                                     ego_width=self.ego.width, ego_length=self.ego.length,
                                     warming=warming
                                     )
            path_opt.reset()
            path_opt.set_constraints()
            path_opt.add_objective()
            res, status = path_opt.solve()


        if status != "solved":
            # print("Warning, act the default path planning, which returns the a straight line betweeen cur point and final destination.")
            final_x = (self.destination[0][0] + self.destination[1][0]) / 2
            final_y = (self.destination[0][1] + self.destination[1][1]) / 2
            return (np.array([observation.ego_info.x,  final_x]),
                    np.array([observation.ego_info.y,  final_y]))

        xy = path_opt.to_xy()

        # if observation.test_info["t"] >= 3.1:
        #     self.plot_path(xy, res, obstacles, truncated_ref_points)

        return xy

    def create_path_spine_curve(self, path_x_list, path_y_list):
        sp = CubicSpline2D(path_x_list, path_y_list)
        rs = np.arange(0, sp.origin_s[-1] + 1, 0.1)  # [m] distance of each interpolated points
        rx, ry = sp.calc_position(rs)
        rtheta = sp.calc_theta(rs)
        rkappa = sp.calc_curvature(rs)
        rdkappa = sp.calc_dcurvature(rs)
        ref_points = []
        for i in range(len(rs)):
            ref_points.append(RefPoint(rx=rx[i], ry=ry[i], rs=rs[i],
                                       rtheta=rtheta[i],
                                       rkappa=rkappa[i],
                                       rdkappa=rdkappa[i]))

        return ref_points, sp, sp.origin_s

    def optimize_speed_curve_qp(self, obstacles, path_x, path_y, path_s, observation=None):
        obs_st_boundaries = self.get_st_boundaries(obstacles, path_x, path_y, path_s, self.speed_t)
        obs_name, all_decisions, decision_num = self.get_st_decisions(obs_st_boundaries)
        valid_result = None
        valid_cost = 1e10

        # if observation.test_info["t"] >= 0.0:
        #     print(1)
        #     self.plot_st_obstacles(obs_name, obs_st_boundaries, self.ego.cartesian_state.v, self.ego.cartesian_state.a)


        for decision in all_decisions:
            speed_opt = SpeedQPOptimizer(self.config,
                self.speed_t, self.ego.cartesian_state.v, self.ego.cartesian_state.a,
                self.config.speed_s_start, self.config.speed_s_end,
                obs_name, decision, obs_st_boundaries, self.need_warm)
            speed_opt.set_constraints()
            speed_opt.add_objective()
            speed_opt.add_aux_obj_cons_for_collision()
            res, status = speed_opt.solve()
            if status == "solved" or status == "solved inaccurate":
                if res.info.obj_val < valid_cost:
                    valid_result = res.x
                    valid_cost = res.info.obj_val

        # 无解的情况，减小t的gap
        if valid_cost == 1e10:
            shrinked_index = int(self.config.speed_t_dense_end / self.config.delta_speed_t_dense)
            self.speed_t = self.speed_t[:shrinked_index]
            for key in obs_st_boundaries.keys():
                obs_st_boundaries[key] = obs_st_boundaries[key][:, :shrinked_index]

            for decision in all_decisions:
                speed_opt = SpeedQPOptimizer(self.config,
                    self.speed_t, self.ego.cartesian_state.v, self.ego.cartesian_state.a,
                    self.config.speed_s_start, self.config.speed_s_end,
                    obs_name, decision, obs_st_boundaries, self.need_warm)
                speed_opt.set_constraints()
                speed_opt.add_objective()
                speed_opt.add_aux_obj_cons_for_collision()
                res, status = speed_opt.solve()
                if status == "solved" or status == "solved inaccurate":
                    if res.info.obj_val < valid_cost:
                        valid_result = res.x
                        valid_cost = res.info.obj_val

        # 如果还是没有解的话，这里用IDM代替一下
        if valid_cost == 1e10:
            return self.generate_acc_from_IDM(self.ego.cartesian_state.v, self.ego.cartesian_state.a, obstacles), obs_st_boundaries

        acc = valid_result[len(self.speed_t) * 2 + 1]
        self.speed_t = np.concatenate([self.speed_t_dense, self.speed_t_sparse], axis=0)
        return acc, obs_st_boundaries


    def generate_acc_from_IDM(self, prev_speed, prev_acc, obstacles):
        min_headway = 1e11
        leading_car = None
        for car in self.obs_st_boundaries:
            if self.obs_st_boundaries[car][0][0] != None and self.obs_st_boundaries[car][0][0] < min_headway:
                min_headway = self.obs_st_boundaries[car][0][0]
                leading_car = car
        if leading_car == None:
            a_new = prev_acc
        else:
            v_ego = prev_speed
            a_max = 5.0
            v_max = 120 / 3.6
            delta_IDM = 4
            s_IDM = min_headway
            v_leading = obstacles[leading_car].v
            delta_v = v_ego - v_leading
            b_IDM = 1.75  # Comfortable deceleration, typically around 1.5 to 2.0 m/s²
            s_min = 2.5  # Minimum gap, typically around 2 to 4 meters.
            T_desired = 2  # Desired time headway, usually between 1.0 to 2.0 seconds.
            s_desired = s_min + v_ego * T_desired + v_ego * delta_v / (2 * (a_max * b_IDM) ** 0.5)
            a_new = min(5.0, max(-5.0, a_max * (1 - (v_ego / v_max) ** delta_IDM - (s_desired / s_IDM) ** 2)))
        return a_new


    def generate_actions(self, acc, path_curve, prev_speed, prev_theta, dt):
        rs = prev_speed * dt
        wheel_target = normalize_angle(path_curve.calc_theta(rs) - prev_theta) / 1.7 / max(prev_speed, 1e-3) / dt * self.ego.length
        return [min(max(acc, -8), 8), min(max(wheel_target, -self.config.max_rot), self.config.max_rot)]

    def get_st_boundaries(self, obstacles, path_x, path_y, path_s, speed_t):
        """
        获取障碍物的st边界
        :return:
        """
        obs_st_boundaries = {}
        for obstacle in obstacles.values():
            bounds = obstacle.get_precise_st_boundaries(self.ego, path_x, path_y, path_s, speed_t)
            obs_st_boundaries[obstacle.name] = bounds
        return obs_st_boundaries

    def get_st_decisions(self, obs_st_boundaries):
        obs_name = []
        raw_decisions = []
        for obstacle, bounds in obs_st_boundaries.items():
            # 如果所有的head bounds 都是1e6，说明没有碰撞风险，就不添加障碍物了
            if (bounds[0] == 1e6).all():
                continue

            # 如果没有的话，说明有碰撞风险
            else:
                # 如果从0时刻起就有碰撞风险，那么只能是跟车，不可能超车
                if bounds[0, 0] != 1e6:
                    obs_name.append(obstacle)
                    raw_decisions.append([0])

                # 如果没有的话，就存在两种操作，要么跟车，要么超车
                else:
                    obs_name.append(obstacle)
                    raw_decisions.append([0, 1])

        total_comb = np.prod([len(each) for each in raw_decisions])
        final_decisions = [each for each in itertools.product(*raw_decisions)]
        return obs_name, final_decisions, total_comb


    def plot_st_obstacles(self, obs_name, obs_st_boundaries, prev_speed, prev_acc):
        fig = plt.figure()
        ax = plt.subplot(111)
        for key, value in obs_st_boundaries.items():
            if key not in obs_name:
                continue

            for i in range(len(self.speed_t) - 1):
                if value[0, i] == 1e6:
                    continue

                corner_y = value[0, i]
                corner_x = self.speed_t[i]
                length_y = value[1, i] - value[0, i]
                length_x = self.speed_t[i + 1] - self.speed_t[i]

                ax.add_patch(patches.Rectangle((corner_x, corner_y-0.5), length_x, length_y+0.5))

        upper_bound = prev_speed * self.speed_t + 0.5 * 5 * self.speed_t ** 2
        t_stop = np.clip(self.speed_t, a_min=0, a_max=-prev_speed / -5)
        lower_bound = prev_speed * t_stop + 0.5 * -5 * t_stop ** 2
        plt.plot(self.speed_t, upper_bound)
        plt.plot(self.speed_t, lower_bound)
        # plt.ylim(0, 40)
        # plt.xlim(0, 5)
        plt.show()

    def plot_path(self, xy, res, obstacles, truncated_ref_points):
        fig = plt.figure()
        ax = plt.subplot(311)
        plt.plot(self.path_s, res.x[: len(res.x) // 3])
        for obstacle in obstacles.values():
            if obstacle.mode == "dynamic":
                continue

            s_lower = int(obstacle.frenet_state[0][0] - obstacle.length / 2) - \
                      truncated_ref_points[0].rs
            s_upper = (obstacle.frenet_state[0][0] + obstacle.length / 2) - \
                      truncated_ref_points[0].rs

            # 如果当前障碍物不在路上，就说明对行驶没有影响
            if obstacle.frenet_state[1][0] > 3 or obstacle.frenet_state[1][0] < -3:
                continue


            l = obstacle.frenet_state[1][0] - obstacle.width / 2
            u = obstacle.frenet_state[1][0] + obstacle.width / 2

            corner_y = l
            corner_x = s_lower
            length_y = obstacle.width
            length_x = obstacle.length

            ax.add_patch(patches.Rectangle((corner_x, corner_y), length_x, length_y))
        plt.subplot(312)
        plt.plot(self.path_s, res.x[len(res.x) // 3: len(res.x) // 3 * 2])
        plt.subplot(313)
        plt.plot(self.path_s, res.x[len(res.x) // 3 * 2:])
        plt.show()

        fig = plt.figure()
        plt.plot(xy[0], xy[1], marker="o")
        plt.plot([each.rx for each in truncated_ref_points], [each.ry for each in truncated_ref_points])
        plt.show()
        return xy
