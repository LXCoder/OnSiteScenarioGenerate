#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import lib
import math
import numpy as np
import pandas as pd
import matplotlib.cm as cm
from matplotlib import pyplot as plt


from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from planner.CustomPlanner_rank3.routing import LaneGraph, AStarRoute
from planner.CustomPlanner_rank3.planner_components.entites import Ego, Obstacle
from planner.CustomPlanner_rank3.planner_components.sampler import LatitudeSampler, LongitudeSampler, LongitudeLatitudeCombiner
from planner.CustomPlanner_rank3.local_utils import detect_collision, calculate_distances, normalize_angle, predict_obstacles_traj, frenet_to_cartesian

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


        # sample information
        self.d_sample_space = [0]  # 纵向采样距离
        self.s_sample_space = [20]  # 横向采样距离
        self.time_range = np.arange(1, 4, 1)
        self.merge_time_range = np.arange(0, 3, 0.1)
        self.cruise_speed_range = np.arange(0, 120 / 3.6 + 1, 120 / 3.6 / 7)

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
        self.ego = Ego(self.route, [self.lane_graph.topo_graph[lane_id]["entity"].center_vertices for lane_id in self.route])

    def act(self, observation: Observation):
        if observation.test_info["t"] == 0:
            self.ego.init(observation)
        else:
            self.ego.update(observation)

        s_condition, d_condition = self.ego.to_frenet()
        obstacles = self.create_obstacles(observation.object_info, self.ego.cur_refline, self.ego.cur_fitting_curve)
        lat_curves, lon_curves = self.sample_SL_ST_curves(s_condition, d_condition, obstacles)
        merged_curves, lat_costs = self.merge_SL_ST_curves(lat_curves, lon_curves)
        new_merged_curves, final_costs = self.evaluate_curves(merged_curves, obstacles, lat_costs)
        # self.plot_cartesian(observation, obstacles)
        # self.plot_frenet([s_condition, d_condition], obstacles)
        self.plot_lat_curves([s_condition, d_condition], lat_curves)
        self.plot_lon_curves([s_condition, d_condition], lon_curves)
        # self.plot_lon_lat_curves(new_merged_curves, topk=3)

        # ego_acc = min(max(new_merged_curves[0][0].a, -3), 3)
        ego_acc = (new_merged_curves[0][1].v - new_merged_curves[0][0].v) / 0.1
        # print(ego_acc)
        if observation.test_info["t"] == 5.2:
            print(1)
        print("Time: %.02f, Current ego x, y, yaw, v, a are %.02f, %.02f, %.02f, %.02f, %.02f" % (
            observation.test_info["t"],
            observation.ego_info.x, observation.ego_info.y,
            normalize_angle(observation.ego_info.yaw), observation.ego_info.v, observation.ego_info.a))

        print("Time: %.02f, Predicted x, y, yaw, v, a are %.02f, %.02f, %.02f, %.02f, %.02f" % (
            observation.test_info["t"],
            new_merged_curves[0][1].x, new_merged_curves[0][1].y,
            normalize_angle(new_merged_curves[0][1].theta), new_merged_curves[0][1].v, new_merged_curves[0][1].a))

        wheel_target = normalize_angle((new_merged_curves[0][1].theta - new_merged_curves[0][0].theta)) * 1.7 / 0.1 / observation.ego_info.v * 4
        return [ego_acc, wheel_target]

    def alg(self, ego_info, obj_info):
        # 新建障碍物
        pass

    def create_obstacles(self, obj_info, path_points, fitting_curves):
        obstacles = []
        for obj_type, obses in obj_info.items():
            for obs_key, obs_value in obses.items():
                obstacles.append(Obstacle(
                    obs_key,
                    obs_value.x, obs_value.y,
                    obs_value.v, obs_value.a, obs_value.yaw,
                    obs_value.length, obs_value.width,
                    obj_type))

        for i in range(len(obstacles)):
            obstacles[i].match_points(path_points, fitting_curves)  ### 同样match障碍物与参考轨迹
            obstacles[i].make_cartesian_state()  # 创建TrajPoint,为转换坐标系做准备
            obstacles[i].convert_cartesian_to_frenet()  # 创建TrajPoint,为转换坐标系做准备

        return obstacles

    def sample_SL_ST_curves(self, s_init_condition, d_init_condition, obstacles):
        lat_sampler = LatitudeSampler(self.d_sample_space, self.s_sample_space)
        lat_sampler.update_init(d_init_condition)
        lat_samples = lat_sampler.sample()
        lat_curves = lat_sampler.fit_curves(lat_samples)

        lon_sampler = LongitudeSampler(self.time_range, self.cruise_speed_range)
        lon_sampler.update_init(s_init_condition)
        lon_samples = lon_sampler.sample_cruise() + lon_sampler.sample_obstacles(obstacles, s_init_condition[0])
        # lon_samples = lon_sampler.sample_cruise()
        lon_curves = lon_sampler.fit_curves(lon_samples)

        return lat_curves, lon_curves

    def merge_SL_ST_curves(self, lat_curves, lon_curves):
        merged_curves = []
        lat_costs = []
        print("Total SL curves: ", len(lat_curves))
        print("Total ST curves: ", len(lon_curves))
        print("Total combined curves", len(lat_curves) * len(lon_curves))
        combiner = LongitudeLatitudeCombiner(self.merge_time_range)
        for i, lat_curve in enumerate(lat_curves):
            for j, lon_curve in enumerate(lon_curves):
                combiner.update_curves(lat_curve, lon_curve)
                merged_curve, lat_cost = combiner.combine(self.ego.cur_refline, self.ego.cur_fitting_curve)
                merged_curves.append(merged_curve)
                lat_costs.append(lat_cost)
        return merged_curves, lat_costs

    def evaluate_curves(self, traj_groups, obstacles, lat_costs):
        speed_vio, acc_vio, cur_vio = self.check_violation(traj_groups)
        non_vio_traj_groups = []
        non_col_traj_groups = []
        non_vio_lat_costs = []
        non_col_lat_costs = []
        obs_dis = []
        final_costs = []
        vio = speed_vio + acc_vio + cur_vio
        for i in range(len(traj_groups)):
            if vio[i] == 0:
                non_vio_traj_groups.append(traj_groups[i])
                non_vio_lat_costs.append(lat_costs[i])
        print("Remained curve number after violation check is ", len(non_vio_traj_groups))

        group_obs_dis = self.check_collision_violation_loss(non_vio_traj_groups, obstacles)

        # 这里先把碰撞当成硬约束，如果出现无解的情况再变成软约束
        for i in range(len(non_vio_traj_groups)):
            if group_obs_dis[i] != -1:
                non_col_traj_groups.append(non_vio_traj_groups[i])
                non_col_lat_costs.append(non_vio_lat_costs[i])
                obs_dis.append(group_obs_dis[i])
        print("Remained curve number after collision check is ", len(non_col_traj_groups))


        des_dis = self.check_destination_loss(non_col_traj_groups)

        for i in range(len(non_col_traj_groups)):
            # final_costs.append(obs_dis[i] + des_dis[i] * 2)
            final_costs.append(0.3 * obs_dis[i] + des_dis[i] * 2)

        final_costs = np.array(final_costs)
        argsort_cost = np.argsort(final_costs)

        return [non_col_traj_groups[i] for i in argsort_cost], final_costs[argsort_cost]


    def check_violation(self, traj_groups):
        group_speed_vio = []
        group_acceleration_vio = []
        group_curvature_vio = []
        for i, traj_points in enumerate(traj_groups):
            group_speed_vio.append(self._check_speed_violation(traj_points))
            group_acceleration_vio.append(self._check_acceleration_violation(traj_points))
            group_curvature_vio.append(self._check_curvature_violation(traj_points))

        return np.array(group_speed_vio), np.array(group_acceleration_vio), np.array(group_acceleration_vio)

    @staticmethod
    def _check_speed_violation(traj_points):
        if any([traj_point.v > MAX_SPEED for traj_point in traj_points[:5]]):
            return 1
        else:
            return 0

    @staticmethod
    def _check_acceleration_violation(traj_points):
        if any([(traj_point.a > MAX_ACCELERATION or traj_point.a < -MAX_ACCELERATION) for traj_point in traj_points[:5]]):
            return 1
        else:
            return 0

    @staticmethod
    def _check_curvature_violation(traj_points):
        if any([traj_point.kappa > MAX_CURVATURE for traj_point in traj_points[:5]]):
            return 1
        else:
            return 0

    def check_collision_violation_loss(self, traj_groups, obstacles):
        group_obs_dis = []
        obstacles_traj = predict_obstacles_traj(obstacles, self.merge_time_range)

        for traj_points in traj_groups:
            total_obs_dis = 0
            is_collide = False
            for obstacle in obstacles:
                obs_dis, is_collide = detect_collision(traj_points, obstacle, self.ego, obstacles_traj[obstacle.name])
                if is_collide:
                    break

                total_obs_dis += obs_dis

            if not is_collide:
                group_obs_dis.append(total_obs_dis)
            else:
                group_obs_dis.append(-1)
        return group_obs_dis

    def check_destination_loss(self, traj_groups):
        group_des_dis = []
        for traj_points in traj_groups:
            min_dis_index, min_dis = calculate_distances(self.destination, [(traj_point.x, traj_point.y) for traj_point in traj_points])
            group_des_dis.append(min_dis)

        return group_des_dis

    def plot_cartesian(self, observation, obstacles=None, reference_line_flag=True, obstacle_flag=True):
        fig = plt.figure()
        plt.scatter([observation.ego_info.x, self.ego.cur_matched_point.rx],
                    [observation.ego_info.y, self.ego.cur_matched_point.ry], s=50, color="tab:orange")
        plt.quiver(
            [observation.ego_info.x],
            [observation.ego_info.y],
            [np.cos(observation.ego_info.yaw)],
            [np.sin(observation.ego_info.yaw)], scale_units='x', scale=1, width=1e-3)


        if reference_line_flag:
            plt.plot([each.rx for each in self.ego.cur_refline], [each.ry for each in self.ego.cur_refline], "xb")
            plt.plot(*list(zip(*[self.ego.cur_fitting_curve.calc_position(each.rs) for each in self.ego.cur_refline])),
                     "-r")

        if obstacle_flag:
            plt.scatter([each.x for each in obstacles], [each.y for each in obstacles], marker='o')
            plt.quiver(
                [each.x for each in obstacles],
                [each.y for each in obstacles],
                [np.cos(each.theta) for each in obstacles],
                [np.sin(each.theta) for each in obstacles], scale_units='x', scale=1, width=1e-3)


        plt.xlim(3520, 3580)
        plt.ylim(14080, 14140)
        plt.show()


    def plot_frenet(self, ego_frenet, obstacles):
        fig = plt.figure()
        ax = plt.subplot()
        ax.yaxis.set_ticks_position('left')
        ax.spines['left'].set_position(('data', 0))

        plt.scatter([each.frenet_state[1][0] for each in obstacles], [each.frenet_state[0][0] for each in obstacles])
        plt.quiver(
            [each.frenet_state[1][0] for each in obstacles],
            [each.frenet_state[0][0] for each in obstacles],
            [each.frenet_state[1][1] for each in obstacles],
            [each.frenet_state[0][1] for each in obstacles], scale_units='x', scale=1, width=1e-3)
        plt.scatter([ego_frenet[1][0]], [ego_frenet[0][0]])
        plt.quiver(
            [ego_frenet[1][0]],
            [ego_frenet[0][0]],
            [ego_frenet[1][1]],
            [ego_frenet[1][0]], scale_units='x', scale=1, width=1e-3)
        plt.show()

    def plot_lat_curves(self, ego_frenet, lat_curves):
        fig = plt.figure()

        plt.scatter([ego_frenet[1][0]], [ego_frenet[0][0]])

        for each in lat_curves:
            plt.plot(each.evaluate(0, np.arange(0, 80, 1)), np.arange(0, 80, 1) + ego_frenet[0][0])
        plt.xlim(-20, 20)
        plt.show()

    def plot_lon_curves(self, ego_frenet, lon_curves):
        fig = plt.figure()
        plt.subplot(221)
        for each in lon_curves:
            plt.plot(np.arange(0, 20, 0.2), each.evaluate(0, np.arange(0, 20, 0.2)))
        plt.ylim(0, 120)

        plt.subplot(222)
        for each in lon_curves:
            plt.plot(np.arange(0, 20, 0.2), each.evaluate(1, np.arange(0, 20, 0.2)))
        plt.ylim(0, 120 / 3.6)

        plt.subplot(223)
        for each in lon_curves:
            plt.plot(np.arange(0, 20, 0.2), each.evaluate(2, np.arange(0, 20, 0.2)))
        plt.ylim(0, 120 / 3.6)

        plt.subplot(224)
        for each in lon_curves:
            plt.plot(np.arange(0, 20, 0.2), each.evaluate(3, np.arange(0, 20, 0.2)))
        plt.ylim(0, 120 / 3.6)
        plt.show()

    def plot_lon_lat_curves(self, merged_curves, topk=0):
        fig = plt.figure()
        if topk == 0:
            for merged_curve in merged_curves:
                plt.plot([each.x for each in merged_curve], [each.y for each in merged_curve])
        else:

            for i, merged_curve in enumerate(merged_curves[:topk]):
                c = cm.bwr(i / topk, 1)
                plt.plot([each.x for each in merged_curve], [each.y for each in merged_curve], color=c)
        # plt.xlim(3520, 3580)
        # plt.ylim(14080, 14140)
        plt.show()