#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import lib
import math

import numpy as np
import matplotlib.cm as cm
from matplotlib import pyplot as plt
from matplotlib import patches
import itertools
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from planner.CustomPlanner_rank3.routing import LaneGraph, AStarRoute
from planner.CustomPlanner_rank3.planner_components.entites import Ego, Obstacle, RefPoint
from planner.CustomPlanner_rank3.planner_components.path_optimizer import PathOptimizer
from planner.CustomPlanner_rank3.planner_components.speed_optimizer import SpeedDPOptimizer, SpeedQPOptimizer, SpeedDPOptimizerV2, SpeedNLPOptimizer
from planner.CustomPlanner_rank3.local_utils import normalize_angle
from planner.CustomPlanner_rank3.planner_components.cubic_spline import CubicSpline2D

MAX_SPEED = 120 / 3.6
MAX_ACCELERATION = 30
MAX_CURVATURE = 30


class PublicRoadPlanner(PlannerBase):
    def __init__(self):
        self.origin = None
        self.destination = None
        self.road_info = None
        self.lane_graph = None
        self.scenario_name = None
        self.route = None
        self.reference_line = None
        self.route_cost = -1
        self.ego = None
        self.has_init = False
        self.scenario_type = None

        # sample information
        self.delta_path_s_dense = 0.1
        self.delta_path_s_sparse = 0.5
        self.delta_speed_s_dense = 0.1
        self.delta_speed_s_sparse = 0.5
        self.delta_speed_t_dense = 0.1
        self.delta_speed_t_sparse = 0.5
        self.path_s_dense = np.arange(0, 5, self.delta_path_s_dense)
        self.path_s_sparse = np.arange(5, 30, self.delta_path_s_sparse)
        self.path_s = np.concatenate([self.path_s_dense, self.path_s_sparse], axis=0)
        self.speed_s_dense = np.arange(0, 5, self.delta_speed_s_dense)
        self.speed_s_sparse = np.arange(5, 40, self.delta_speed_s_sparse)
        self.speed_s = np.concatenate([self.speed_s_dense, self.speed_s_sparse], axis=0)
        self.speed_t_dense = np.arange(0, 2, self.delta_speed_t_dense)
        self.speed_t_sparse = np.arange(2, 5, self.delta_speed_t_sparse)
        self.speed_t = np.concatenate([self.speed_t_dense, self.speed_t_sparse], axis=0)
        self.speed_a = np.arange(-5.0, 5.1, 0.5)

        self.obs_st_boundaries = {}

    def init(self, scenario_dict):
        print("----------------------------Public Road Planner INIT----------------------------")
        print(scenario_dict)
        print("----------------------------------------------------------------")
        self.origin = scenario_dict['task_info']['startPos']
        self.destination = scenario_dict['task_info']['targetPos']
        self.road_info = parse_opendrive(scenario_dict['source_file']['xodr'])
        self.scenario_name = scenario_dict['name']
        self.lane_graph = LaneGraph(self.road_info, self.origin, self.destination, self.scenario_name)
        self.lane_graph.run()
        route_alg = AStarRoute(self.lane_graph)
        self.route, self.route_cost = route_alg.calculate(True)
        # self.reference_line = self.lane_graph.generate_reference_line(self.route)
        print(self.route)
        print(self.route_cost)
        # self.lane_graph.plot_route(self.route, self.origin, self.destination)
        self.ego = Ego(self.route,
                       [self.lane_graph.topo_graph[lane_id]["entity"].center_vertices for lane_id in self.route], self.lane_graph.topo_graph)

    def act(self, observation: Observation):
        print(observation.test_info["t"])
        if observation.test_info["t"] == 0 or self.has_init == False:
            self.ego.init(observation)
            self.has_init = True

        else:
            self.ego.update(observation)

        s_condition, d_condition = self.ego.to_frenet()
        ego_ref_points = self.reconstruct_ref_points()

        obstacles = self.create_obstacles(observation.object_info, ego_ref_points, self.ego.cur_ref_curve)
        if observation.test_info["t"] >= 1.8:
            print(1)
            # path_x_list, path_y_list = self.optimize_path_curve(ego_ref_points, d_condition, observation.ego_info.v, obstacles,
            #                                                     warming=True if observation.test_info["t"] <= 2 else False,
            #                                                      is_plot=True)
            # np.savez("%.02f.npz" % observation.test_info["t"], x=np.array(path_x_list), y=np.array(path_y_list),
            #          ref_x=[each.rx for each in ego_ref_points], ref_y=[each.ry for each in ego_ref_points],
            #          ego_pos=np.array([observation.ego_info.x, observation.ego_info.y]),
            #          ego_next_pos=np.array([self.ego.cartesian_state.x, self.ego.cartesian_state.y]),
            #          ref_theta_0=ego_ref_points[0].rtheta)

        path_x_list, path_y_list = self.optimize_path_curve(ego_ref_points, d_condition, observation.ego_info.v, obstacles,
                                                            True if observation.test_info["t"] <= 2 else False)

        path_ref_points, path_ref_curve, path_s_list = self.create_path_spine_curve(path_x_list, path_y_list)

        obstacles = self.create_obstacles(observation.object_info, path_ref_points, path_ref_curve)

        if observation.test_info["t"] >= 3.4:
            print(2)
            # (dp_
        # (dp_s_point, dp_p_point, dp_speed_cost), self.obs_st_boundaries = self.optimize_speed_curve(
        #     obstacles, observation.ego_info.v, observation.ego_info.a,
        #     path_x_list, path_y_list, path_s_list)
        # (dp_s_point, dp_p_point, dp_speed_cost), self.obs_st_boundaries = self.optimize_speed_curveV2(
        #     obstacles, observation.ego_info.v, observation.ego_info.a,
        #     path_x_list, path_y_list, path_s_list)
        (dp_s_point, dp_p_point, dp_speed_cost), self.obs_st_boundaries = self.optimize_speed_curve_qp(
            obstacles, self.ego.cartesian_state.v, self.ego.cartesian_state.a,
            path_x_list, path_y_list, path_s_list
        )
        # (dp_s_point, dp_p_point, dp_speed_cost), self.obs_st_boundaries = self.optimize_speed_curve_nlp(
        #     obstacles, self.ego.cartesian_state.v, self.ego.cartesian_state.a,
        #     path_x_list, path_y_list, path_s_list
        # )
        print(dp_s_point)
        if np.sum(dp_s_point) == 0:
            print(2)
        print(observation.test_info["t"])
        # if (observation.test_info["t"] * 10) % 3 == 0:
        #     fig = plt.figure()
        #     plt.plot(path_x_list, path_y_list)
        #     plt.plot([each.rx for each in rectified_refline[:30]], [each.ry for each in rectified_refline[:30]])
        #     plt.show()
        return self.generate_actions(dp_s_point, dp_p_point, path_ref_curve, observation.ego_info.v, self.ego.cartesian_state.theta,
                                     self.ego.cartesian_state.x, self.ego.cartesian_state.y, observation.test_info["dt"])

    def select_scenario_parameters(self):
        """
        根据scenario名称确定场景类型，选择适应的参数
        :return:
        """
        if "highway" in self.scenario_name or "follow" in self.scenario_name:
            self.scenario_type = 0
        else:
            self.scenario_type = 1

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


    def create_obstacles(self, obj_info, path_points, fitting_curves):
        obstacles = {}
        for obj_type, obses in obj_info.items():
            for obs_key, obs_value in obses.items():
                obstacles[obs_key] = Obstacle(
                    obs_key,
                    obs_value.x, obs_value.y,
                    obs_value.v, obs_value.a, obs_value.yaw,
                    obs_value.length, obs_value.width,
                    obj_type)

        for obs_key in obstacles.keys():
            obstacles[obs_key].match_points(path_points, fitting_curves)  ### 同样match障碍物与参考轨迹
            obstacles[obs_key].make_cartesian_state()  # 创建TrajPoint,为转换坐标系做准备
            obstacles[obs_key].convert_cartesian_to_frenet()  # 创建TrajPoint,为转换坐标系做准备

        return obstacles

    def optimize_path_curve(self, ref_points, d_condition, prev_speed, obstacles, warming=False, is_plot=False):
        # truncated_ref_points = self.ego.cur_ref_points[ref_point_index:]
        truncated_left_bounds = -np.array(self.ego.cur_drivable_boundaries["l"](self.path_s + ref_points[0].rs)).reshape(-1, 1) + self.ego.width / 2
        truncated_right_bounds = np.array(self.ego.cur_drivable_boundaries["r"](self.path_s + ref_points[0].rs)).reshape(-1, 1) - self.ego.width / 2
        truncated_ref_points = ref_points
        path_opt = PathOptimizer(
            truncated_left_bounds, truncated_right_bounds,
            truncated_ref_points, self.ego.cur_ref_curve, prev_speed,
            d_condition[0], d_condition[1], d_condition[2],
            0, 0, 0, obstacles,
            self.delta_path_s_dense, self.delta_path_s_sparse, self.path_s,
            ego_width=self.ego.width, ego_length=self.ego.length,
            warming=warming
        )
        # path_opt = PathOptimizer(
        #     np.ones([len(self.ego.cur_ref_points), 1]) * -3, np.ones([len(self.ego.cur_ref_points), 1]) * 3,
        #     ref_points, self.ego.cur_ref_curve, prev_speed,
        #     d_condition[0], d_condition[1], d_condition[2],
        #     0, 0, 0, obstacle, ego_width=self.ego.width, ego_length=self.ego.length,
        # )
        # 先按照有动态障碍物的规划解一遍，如果出现无解的情况那说明确实不能绕车而行，就老老实实按照跟车的走法走

        path_opt.set_constraints()
        path_opt.set_optional_dynamic_obstacle_constraints(self.obs_st_boundaries, obstacles)
        path_opt.add_objective()
        res, status = path_opt.solve()

        if status != "solved":
            path_opt.reset()
            path_opt.set_constraints()
            path_opt.add_objective()
            res, status = path_opt.solve()

        xy = path_opt.to_xy()
        if is_plot:
            fig = plt.figure()
            ax = plt.subplot(311)
            plt.plot(self.path_s, res.x[: len(res.x) // 3])
            for obstacle in obstacles.values():
                if obstacle.mode == "dynamic":
                    continue

                s_lower = int(obstacle.frenet_state[0][0] - obstacle.length / 2) - \
                          ref_points[0].rs
                s_upper = (obstacle.frenet_state[0][0] + obstacle.length / 2) - \
                          ref_points[0].rs

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

    def optimize_speed_curve(self, obstacles, prev_speed, prev_acc, path_x, path_y, path_s):
        speed_opt = SpeedDPOptimizer(self.ego, obstacles, self.speed_s, self.speed_t,
                                     prev_speed, prev_acc, path_x, path_y, path_s)
        st_boundaries = speed_opt.get_st_boundaries()
        res = speed_opt.solve()
        return res, st_boundaries

    def optimize_speed_curveV2(self, obstacles, prev_speed, prev_acc, path_x, path_y, path_s):
        speed_opt = SpeedDPOptimizerV2(self.ego, obstacles, self.speed_a, self.speed_t,
                                     prev_speed, prev_acc, path_x, path_y, path_s)
        st_boundaries = speed_opt.get_st_boundaries()
        res = speed_opt.solve()
        return res, st_boundaries

    def optimize_speed_curve_qp(self, obstacles, prev_speed, prev_acc, path_x, path_y, path_s, visualization=False):
        obs_st_boundaries = self.get_st_boundaries(obstacles, path_x, path_y, path_s, self.speed_t)
        obs_name, all_decisions, decision_num = self.get_st_decisions(obs_st_boundaries)
        valid_result = None
        valid_cost = 1e10
        if visualization:
            self.plot_st_obstacles(obs_name, obs_st_boundaries, self.ego.cartesian_state.v, self.ego.cartesian_state.a)

        for decision in all_decisions:
            speed_opt = SpeedQPOptimizer(
                self.speed_t, self.speed_s,
                     prev_speed, prev_acc,
                     0, self.ego.cartesian_state.v, self.ego.cartesian_state.a,
                     obs_name, decision, obs_st_boundaries)
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
            self.speed_t = self.speed_t[:20]
            for key in obs_st_boundaries.keys():
                obs_st_boundaries[key] = obs_st_boundaries[key][:, :20]
            for decision in all_decisions:
                speed_opt = SpeedQPOptimizer(
                    self.speed_t, self.speed_s,
                    prev_speed, prev_acc,
                    0, self.ego.cartesian_state.v, self.ego.cartesian_state.a,
                    obs_name, decision, obs_st_boundaries)
                speed_opt.set_constraints()
                speed_opt.add_objective()
                speed_opt.add_aux_obj_cons_for_collision()
                res, status = speed_opt.solve()
                if status == "solved" or status == "solved inaccurate":
                    if res.info.obj_val < valid_cost:
                        valid_result = res.x
                        valid_cost = res.info.obj_val


        if visualization:
            fig = plt.figure()
            ax = plt.subplot(311)
            plt.plot(self.speed_t, valid_result[: len(self.speed_t)])
            for key, value in obs_st_boundaries.items():
                if key not in obs_name:
                    continue

                for i in range(len(self.speed_t) - 1):
                    if value[0, i] == 1e6:
                        continue

                    corner_y = value[0, i]
                    corner_x = self.speed_t[i]
                    length_y = value[1, i] - value[0, i]
                    length_x = self.speed_t[i+1] - self.speed_t[i]

                    ax.add_patch(patches.Rectangle((corner_x, corner_y), length_x, length_y))

            plt.subplot(312)
            plt.plot(self.speed_t, valid_result[len(self.speed_t): len(self.speed_t) * 2])
            plt.subplot(313)
            plt.plot(self.speed_t, valid_result[len(self.speed_t) * 2: len(self.speed_t) * 3])
            plt.show()

        self.speed_t = np.concatenate([self.speed_t_dense, self.speed_t_sparse], axis=0)
        # return (valid_result[:len(self.speed_t)], self.speed_t, valid_cost), obs_st_boundaries
        return (valid_result, self.speed_t, valid_cost), obs_st_boundaries

    def optimize_speed_curve_nlp(self, obstacles, prev_speed, prev_acc, path_x, path_y, path_s, visualization=False):
        obs_st_boundaries = self.get_st_boundaries(obstacles, path_x, path_y, path_s, self.speed_t)
        obs_name, all_decisions, decision_num = self.get_st_decisions(obs_st_boundaries)
        valid_result = None
        valid_cost = 1e10

        for decision in all_decisions:
            speed_opt = SpeedNLPOptimizer(
                self.speed_t, self.speed_s,
                     prev_speed, prev_acc,
                     0, self.ego.cartesian_state.v, self.ego.cartesian_state.a,
                     obs_name, decision, obs_st_boundaries)
            speed_opt.set_constraints()
            speed_opt.add_objective()
            res, status = speed_opt.solve()
            if status == "solved":
                if res["primal objective"] < valid_cost:
                    valid_result = res["x"]

        return (valid_result[:len(self.speed_t)], self.speed_t, valid_cost), obs_st_boundaries


    def generate_actions(self, speed_points_s, speed_points_t, path_curve, prev_speed, prev_theta, cur_x, cur_y, dt):
        delta_t = (speed_points_t[1] - speed_points_t[0])
        delta_s = (speed_points_s[1] - speed_points_s[0])
        # acc = (delta_s / delta_t - prev_speed) / delta_t
        acc = speed_points_s[2 * len(self.speed_t) + 1]
        rs = prev_speed * dt
        x, y = path_curve.calc_position(rs)
        # dis_to_center =
        # wheel_target = normalize_angle(path_curve.calc_theta(rs) - prev_theta) * 1.7 / max(prev_speed, 1) / 0.1
        wheel_target = normalize_angle(path_curve.calc_theta(rs) - prev_theta) / 1.7 / max(prev_speed, 1e-3) / dt * self.ego.length
        # wheel_target = normalize_angle(np.arctan2(y - cur_y, x - cur_x) - prev_theta) / 1.7 / max(prev_speed, 1e-3) / dt * self.ego.length
        # wheel_target = np.arctan(wheel_target)
        return [min(max(acc, -8), 8), min(max(wheel_target, -0.698), 0.698)]

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
        plt.ylim(0, 40)
        plt.xlim(0, 5)
        plt.show()