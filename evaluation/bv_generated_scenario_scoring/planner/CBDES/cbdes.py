#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib

import numpy as np
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from typing import List, Tuple
from .readmap import parse_opendrive_cbdes
# ====================================================TP======================================================
# from .TPlib.utils.map_utils import MapTPInfo
# from .TPlib.utils.traj_utils import History
# from .TPlib.utils.model_utils import STFData
# from .TPlib.model.model_for_predict import AllVehicleTrajPredict
# from .TPlib.utils.utils import reconstruct_forcast_traj

from .TP_rule.traj_predict import predict_rule
from .TP_rule.utils.TP_rule_utils import History_rule
# ====================================================TP======================================================

# ====================================================GP======================================================
from .GPlib.globe_plan import GPInfo
# ====================================================GP======================================================

# ====================================================LP======================================================
from .LPlib.local_plan import LPInfo
# ====================================================LP======================================================

# ====================================================CTL======================================================
from .Control.cal_control import get_control
from .Control.cbf_onsite import VehicleCBF, VehicleModel
# ====================================================CTL======================================================

class CBDES(PlannerBase):
    def __init__(self, a_bound=5.0, exv=40, t=1.2, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
        """跟idm模型有关的模型参数
        :param a_bound: 本车加速度绝对值的上下界
        :param exv: 期望速度
        :param t: 反应时间
        :param a: 起步加速度
        :param b: 舒适减速度
        :param gama: 加速度指数
        :param s0: 静止安全距离
        :param s1: 与速度有关的安全距离选择参数
        """
        '''
        self.a_bound = a_bound
        self.exv = exv
        self.t = t
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_ = 0
        '''
        # ====================================================TP======================================================
        # self.mapTP = MapTPInfo()
        # self.history_traj = History()
        self.history_traj_rule = History_rule()
        # print("__init__ finish")
        # self.predictor = AllVehicleTrajPredict()
        # ====================================================TP======================================================

    def init(self, scenario_dict):
        # print("---------------------------CBDES INIT---------------------------") # KAI
        # print(scenario_dict) # KAI
        # print("----------------------------------------------------------------") # KAI
        self.road_data, name, self.road_with_stopline, junctions = parse_opendrive_cbdes(scenario_dict['source_file']['xodr'])
        self.has_light_info = False
        self.task_info = scenario_dict['task_info']
        self.vehicleModel = None
        self.frame = 0
        self.scenario_type = scenario_dict['type']
        self.K = 9.064169222036606e-05
        self.L = 2.6
        self.miu = 0.85
        self.lots_junctions = False
        self.ramp_junction = False
        self.slow_down = False
        # ====================================================TP======================================================
        # self.mapTP.getMapTPInfo(self.road_data, scenario_dict, name)
        # print(self.mapTP.lane.shape)
        # ====================================================TP======================================================

        # ====================================================GP======================================================
        self.globe_planner = GPInfo(self.task_info, self.road_data)
        
        # ====================================================GP======================================================
        
        if name in ['highway','highD','highwaymerge']:
            self.map_type = "highway"
            self.safe_miu = self.miu * 0.63 ###
            # if name in ['highwaymerge']:
            #     self.ramp_junction_Id = []
            #     ramp_roadId = None
            #     ramp_sectionId = None
            #     ramp_laneId = None
            #     for discretelane in self.road_data.discretelanes:
            #         if len(discretelane.successor) > 0:
            #             if 'None' in discretelane.successor[0]:
            #                 ramp_Id = discretelane.lane_id.split('.')
            #                 ramp_roadId = int(ramp_Id[0])
            #                 ramp_sectionId = int(ramp_Id[1])
            #                 ramp_laneId = int(ramp_Id[2])  
            #                 self.ramp_point = discretelane.center_vertices[-1]                      
            #                 break
            #     self.ramp_junction, self.ramp_junction_Id = self.get_all_ramp_Id(junctions, ramp_roadId, ramp_sectionId, ramp_laneId)
            # self.planner = zzh()
            # self.to_order, self.num_change, self.direction, self.lane_id, self.bias = self.planner.getEgoOrder_highway(self.task_info, self.road_data)
        
        elif name in ['intersection']:
            self.map_type = "intersection"
            self.safe_miu = self.miu * 0.7 # 0.6
            self.has_light_info = True

        elif name in ['','SinD','roundabout']:
            if scenario_dict['source_file']['json'] == '':
                self.map_type = "roundabout"
                self.safe_miu = self.miu * 0.63 ###

                if len(junctions) > 1 and self.scenario_type == 'REPLAY':
                    self.lots_junctions = True
                    # into_points = []
                    # self.roadid_out_roundabout = []
                    # self.roadid_into_roundabout = []
                    # self.road_into_roundabout = []
                    # self.road_into_junction = {}

                    # for discretelane in self.road_data.discretelanes:
                    #     if len(discretelane.predecessor) == 0:
                    #         self.road_into_roundabout.append([int(discretelane.lane_id.split('.')[0]), discretelane.center_vertices[-1], 0])
                    #         self.roadid_into_roundabout.append(int(discretelane.lane_id.split('.')[0]))
                    #         into_points.append(discretelane.center_vertices[-1])
                    #     elif len(discretelane.successor) == 0 or 'None' in discretelane.predecessor[0]:
                    #         self.roadid_out_roundabout.append(int(discretelane.lane_id.split('.')[0]))
                    #     else:
                    #         continue
                
                    # self.center_point = np.mean(np.array(into_points), axis=0)
                    

                    # for item in self.road_into_roundabout:
                    #     # print(item)
                    #     item[2] = np.arctan2(item[1][1] - self.center_point[1], item[1][0] - self.center_point[0])
                    # self.road_into_roundabout = sorted(self.road_into_roundabout, key=lambda item: item[2])

                    # for junction in junctions:
                    #     incomingRoadId = None
                    #     junctionRoadId = []
                    #     for connection in junction.connections:
                    #         junctionRoadId.append(connection.connectingRoad)
                    #         if connection.incomingRoad in self.roadid_into_roundabout:
                    #             incomingRoadId = connection.incomingRoad
                    #     if incomingRoadId is not None:
                    #         self.road_into_junction[str(incomingRoadId)] = junctionRoadId
   

                # self.planner = KTQControl()
                # self.planedLanes, self.AllLanesDict, self.rampFlag, self.roadStructureDict = self.planner.ktq_getEgoTrajectory(self.task_info, self.road_data) 
            
            else:
                self.map_type = "intersection"
                self.has_light_info = True
                self.safe_miu = self.miu * 0.7 ###
                # with open(scenario_dict['source_file']['json'], 'r') as read_f:
                #     self.light_info = json.load(read_f)
                # self.planner = CCYControl()
                # self.ScenarioInfo = ScenarioExtract()
                # self.TrajectoryList = self.ScenarioInfo.getEgoTrajectory(self.task_info, self.road_data)

        elif name in ['NDS_ramp']:
            self.map_type = 'ramp'
            self.safe_miu = self.miu * 0.65 ###
            # self.planner = CMCControl()
            # self.planedLanes, self.AllLanesDict, self.rampFlag, self.roadStructureDict = self.planner.cmc_getEgoTrajectory(self.task_info, self.road_data)    

        elif name in ['mixed']:
            if scenario_dict['source_file']['json'] == '':
                self.map_type = "mixed"
                self.safe_miu = self.miu * 0.65 ###
            else:
                self.map_type = "intersection"
                self.has_light_info = True
                self.safe_miu = self.miu * 0.7 ###
        
        else:
            self.map_type = 'unknown'
            self.safe_miu = self.miu * 0.63 ###
            if scenario_dict['source_file']['json'] != '':  
                self.has_light_info = True  
                # with open(scenario_dict['source_file']['json'], 'r') as read_f:
                #     self.light_info = json.load(read_f)

        # print("Init finish")
    
    # ====================================================TP======================================================
    # def predict_trajectories(self, observation: Observation, vehicle_interest_list):
    #     self.history_traj.update(observation)
    #     self.history_traj.get_all_vehicle_feature(observation, vehicle_interest_list)
    #     if len(self.history_traj.feature) > 0:
    #         stf_data = STFData(self.mapTP,self.history_traj)
    #         history_data = stf_data.collate_data
    #         predictions = self.predictor.predict(history_data)
    #         forecasted_trajectories = self.predictor.renormalized(history_data, predictions)
    #     else:
    #         forecasted_trajectories = {}
    #     return forecasted_trajectories

    def predict_trajectories_rule(self, observation: Observation, vehicle_interest_list):
        self.history_traj_rule.update(observation)
        self.history_traj_rule.get_all_vehicle_feature(observation, vehicle_interest_list)
        if len(self.history_traj_rule.feature) > 0:
            forecasted_trajectories = predict_rule(self.history_traj_rule)
        else:
            forecasted_trajectories = {}
        return forecasted_trajectories
    # ====================================================TP======================================================

    # ====================================================LP======================================================
    def local_planning(self, observation: Observation, scene_type:str):
        # print(scene_type)
        # print(f"slow_down: {self.slow_down}")
        local_planner = LPInfo(self.globe_planner, scene_type, observation, self.scenario_type, self.slow_down, self.lots_junctions)
        # print(f"TP_in_front: {local_planner.TP_in_front}")
        # print(local_planner.state,local_planner.object_id)
        local_planner.max_speed = local_planner.get_max_speed()
        # print("max_speed:",local_planner.max_speed)
        is_lane_change = local_planner.is_changelane()
        is_lane_change_ = is_lane_change
        # print(f"is_lane_change: {is_lane_change}")
        if is_lane_change:
            nearest_front_id, nearest_front_v, nearest_front_frenet_y = local_planner.get_front_car_change_lane(local_planner.TP_in_front,local_planner.TP_v_in_front,
                                                                      local_planner.ego_lane_id,local_planner.ego_frenet_y)
            nearest_behind_id, nearest_behind_v, nearest_behind_frenet_y = local_planner.get_behind_car_change_lane(local_planner.TP_in_behind,local_planner.TP_v_in_behind,
                                                                      local_planner.ego_lane_id,local_planner.ego_frenet_y)
            if nearest_front_id is not None:
                front_dis = nearest_front_frenet_y - local_planner.ego_frenet_y 
                if front_dis < 0.3*local_planner.state[0,3] + 2.5*(local_planner.state[0,3]-nearest_front_v):
                    is_lane_change_ = False
            if nearest_behind_id is not None:
                behind_dis = local_planner.ego_frenet_y - nearest_behind_frenet_y
                if behind_dis < 0.3*nearest_behind_v + 2.5*(nearest_behind_v - local_planner.state[0,3]):
                    is_lane_change_ = False
        # print(f"is_lane_change_: {is_lane_change_}")
        if is_lane_change_:
            target_paths, target_frenet_y, target_v = local_planner.change_lane(observation)
        else:
            target_paths, target_frenet_y, target_v = local_planner.keep_lane_paths(observation, is_lane_change)
        vehicle_interest_list = local_planner.TP_in_front + local_planner.TP_in_behind
        forecasted_trajectories_rule = self.predict_trajectories_rule(observation, vehicle_interest_list)
        # forecasted_trajectories = self.predict_trajectories(observation, vehicle_interest_list)
        # print("t: ", observation.test_info["t"], " frame: ", self.frame) # KAI
        # path, v = local_planner.get_safe_path(target_paths,target_frenet_y, target_v, TP_in_front, TP_in_behind, forecasted_trajectories)
        
        # 兼容 warmup=1 / warmup=31：以“规划器内部步数”判断是否进入预测安全路径，
        # 避免 warmup=31 时首个 act 因全局 t 已大于 0.1 而误走非首帧分支。
        if self.frame > 1:
            path, v, final_flag = local_planner.get_safe_path(target_paths,target_frenet_y, target_v, local_planner.TP_in_front, local_planner.TP_in_behind, forecasted_trajectories_rule, observation)
            # if self.scenario_type == "REPLAY" and scene_type == "roundabout" and self.lots_junctions == True:
            #     merge_safe_flag = self.is_merge_safe(local_planner)
            #     if not merge_safe_flag:
            #         path, v = local_planner.safe_merge()
            #         # path, v = target_paths[-3], target_v[-3]
            #         path, v = target_paths[-3], -0.5 * observation.ego_info.v - 1
        else:
            path, v = target_paths[0], target_v[0]
            final_flag = False

        
        # new_forecasted_trajectories = reconstruct_forcast_traj(forecasted_trajectories, dt_observed=0.1, 
        #                                                        dt_target=observation.test_info["dt"], time=3)
        # print("finish reconstruct_forcast_traj")
        return forecasted_trajectories_rule, path, v, is_lane_change, local_planner.ego_lane_id, final_flag
    # ====================================================LP======================================================

    # ====================================================Control======================================================
    def get_obj_type(self, obj_name: str) -> str:
        if "car" in obj_name:
            return "vehicle"
        elif "bicycle" in obj_name:
            return "bicycle"
        elif "pedestrian" in obj_name:
            return "pedestrian"
        else:
            return None

    def head_on_judge(self, angle1: float, angle2: float) -> bool:
        delta_angle = np.abs(angle1 - angle2)
        if delta_angle > np.pi * 0.75 and delta_angle < np.pi * 1.25:
            return True
        else:
            return False
        
    def same_dir_judge(self, ego_yaw, ego_width, obj_yaw, obj_width, lat_bias) -> bool:
        delta_angle = np.abs(ego_yaw - obj_yaw)
        if delta_angle > np.pi * 0.03 and delta_angle < np.pi * 1.97:
            return False
        else:
            # lat_bias = np.abs(- (obj_x - ego_x) * np.sin(ego_yaw) + (obj_y - ego_y) * np.cos(ego_yaw))
            if lat_bias < (ego_width + obj_width) / 2.0 + 0.1:
                return False
            else:
                return True
    
    def singal_stop_check(self, light, stop_position, path, ego_x, ego_y, ego_yaw, ego_v) -> bool:
        if light == "green":
            return False
        
        else:
            preview_position = path[-1]
            preview_diff_x = preview_position[0] - ego_x
            preview_diff_y = preview_position[1] - ego_y
            preview_local_x = preview_diff_x * np.cos(ego_yaw) + preview_diff_y * np.sin(ego_yaw)
            stop_diff_x = stop_position[0] - ego_x
            stop_diff_y = stop_position[1] - ego_y
            stop_local_x = stop_diff_x * np.cos(ego_yaw) + stop_diff_y * np.sin(ego_yaw)

            if ego_v < 10.0:
                stop_dis = 0.1
            else:
                stop_dis = 0.1 + ego_v - 10.0
            
            if light == "red":
                if preview_local_x > stop_local_x and stop_local_x > stop_dis:
                    return True
                else:
                    return False
            
            elif light == "yellow":
                if preview_local_x > stop_local_x * 2.0 and stop_local_x > stop_dis * 1.5:
                    return True
                else:
                    return False  

            else:
                return False      

    def final_check(self, v, a, steer, delta, a_upper_limit, a_lower_limit):
        v = np.max([v, 1.0])
        # a_limit_1 = 9.78 * self.safe_miu - np.abs(steer * v ** 2 / (self.K * v ** 2 + 1) / self.L) - np.abs(delta) * 0.6 * v - 0.5 * 0.3 * 1.6 * 1.206 * v ** 2 / 1134 
        a_limit_1 = 9.78 * self.safe_miu - np.abs(steer * v ** 2 / (self.K * v ** 2 + 1) / self.L + delta * 0.6 * v) - 0.5 * 0.3 * 1.6 * 1.206 * v ** 2 / 1134 
        a_upper_limit_new = np.min([a_upper_limit, a_limit_1])
        a_lower_limit_new = np.max([a_lower_limit, - a_limit_1])
        return np.clip(a, a_lower_limit_new, a_upper_limit_new)
    
    def final_check_roundabout(self, v, a, steer, delta, a_upper_limit, a_lower_limit, steer_upper_limit, steer_lower_limit, observation):
        if a > 3:
            steer_new = np.clip(steer * 3 / 4, steer_lower_limit, steer_upper_limit)
            # steer_new = np.clip(0, steer_lower_limit, steer_upper_limit)
            delta_new = (steer_new - observation.ego_info.rot) / observation.test_info["dt"]
        elif a < -2.5:
            steer_new = np.clip(steer * 1 / 2, steer_lower_limit, steer_upper_limit)
            # steer_new = np.clip(0, steer_lower_limit, steer_upper_limit)
            delta_new = (steer_new - observation.ego_info.rot) / observation.test_info["dt"]
        else:
            steer_new = steer
            delta_new = delta
        v = np.max([v, 1.0])
        a_limit_1 = 9.78 * self.safe_miu - np.abs(steer_new * v ** 2 / (self.K * v ** 2 + 1) / self.L) - np.abs(delta_new) * 0.6 * v - 0.5 * 0.3 * 1.6 * 1.206 * v ** 2 / 1134 
        # print(a_limit_1)
        a_upper_limit_new = np.min([a_upper_limit, a_limit_1])
        a_lower_limit_new = np.max([a_lower_limit, - a_limit_1])
        return np.clip(a, a_lower_limit_new, a_upper_limit_new), steer_new
    # ====================================================Control======================================================


    def act(self, observation: Observation):
        # print("####act####")
        # print(f"observation {observation}")
        # ob = self.observation_trans(observation)
        
        # print("\n") # KAI
        # return None

        # if observation.test_info["t"] > 3.6:
        #     print(f"debug return")
        #     return None
        # ====================================================GP======================================================
        if self.frame < 1e-3:
            self.vehicleModel = VehicleModel(observation.ego_info)
            self.globe_planner.glob_plan(observation, self.road_with_stopline)  
        self.frame = self.frame + 1    
        # print("globe path:",self.globe_planner.path) # KAI
        # ====================================================GP======================================================
        # ====================================================TP&LP======================================================
        forecasted_trajectories = {}
        preview_path = None
        forecasted_trajectories, preview_path, preview_v, is_lane_change, ego_lane_id, final_flag = self.local_planning(observation, self.map_type)
        # print("path==None:",preview_path==None)
        # print(type(preview_path),len(preview_path))
        # print(f"forecasted_trajectories\n {forecasted_trajectories}")
        # print(f"new_forecasted_trajectories\n {new_forecasted_trajectories}")
        # print(f"preview_path\n {preview_path}")
        # print(f"v\n {preview_v}")
        # ====================================================TP&LP======================================================

        # ====================================================Control======================================================
        # print("###control###")
        action = [0.0, 0.0]
        
        singal_stop_flag = False
        if self.has_light_info:
            if ego_lane_id in self.globe_planner.stopline_id:
                stop_lane_index = self.globe_planner.stopline_id.index(ego_lane_id)
                if not self.globe_planner.is_turn_right[stop_lane_index]:
                    light = observation.light_info
                    singal_stop_flag = self.singal_stop_check(light, self.globe_planner.stopline_position[stop_lane_index], preview_path, 
                                                              observation.ego_info.x, observation.ego_info.y, observation.ego_info.yaw, observation.ego_info.v)
            # print(f"singal_stop_flag: {singal_stop_flag}")
        
        # path trans
        if singal_stop_flag:
            preview_point = self.globe_planner.stopline_position[stop_lane_index]
            dis_to_stopline = np.sqrt((observation.ego_info.x - preview_point[0])**2 + (observation.ego_info.y - preview_point[1])**2)
            preview_v = observation.ego_info.v * 0.8 - 1.2
            if dis_to_stopline < 5.0 + observation.ego_info.v * 0.4:
                if preview_v < dis_to_stopline:
                    preview_v = dis_to_stopline * 1.2 - 0.4 - observation.ego_info.v * 0.4
                else:
                    preview_v = dis_to_stopline * 0.8 - 0.8 - observation.ego_info.v * 0.8 
            else:
                if preview_v < 3.0:
                    preview_v = 3.0
                # preview_v = np.max([observation.ego_info.v * 0.8 - 1.2, dis_to_stopline * 0.7 - 0.1])
        else:
            preview_point = preview_path[10]
        
        if self.map_type == 'roundabout' and preview_v < 0.1:
            preview_v = - observation.ego_info.v - 2.5
        # print(f"preview point {preview_point}")
        # print(f"preview_v {preview_v}") # KAI
        # print(f"x,y,v: {observation.ego_info.x}, {observation.ego_info.y}, {observation.ego_info.v}")
        # calculate control 
        action, a_lower_limit, a_upper_limit, steer_lower_limit, steer_upper_limit = get_control(observation.ego_info.v, observation.ego_info.x, 
                                                                                                  observation.ego_info.y, observation.ego_info.yaw, 
                                                                                                  preview_point[0], preview_point[1], preview_v, 
                                                                                                  observation.ego_info.a, observation.ego_info.rot, 
                                                                                                  observation.ego_info.length, observation.test_info["dt"])

        origin_accel = action[0]
        origin_steer = action[1]
        # print(f"action_ref {action}") # KAI
        # print(observation.object_info)
        # print(f"forecasted_trajectories_keys: {forecasted_trajectories.keys()}")
        # cbf calculates
        if len(forecasted_trajectories) > 1e-3 and not singal_stop_flag:
            obj_list = []
            for key in forecasted_trajectories.keys():
                
                # EDIT: delete prefix
                if "car" in key:
                    new_key = key.replace("car", "")
                elif "bicycle" in key:
                    new_key = key.replace("bicycle", "")
                elif "pedestrian" in key:
                    new_key = key.replace("pedestrian", "")
                else:
                    continue
                # if self.scenario_type == "REPLAY":
                #     new_key = key
                # else:
                #     if "car" in key:
                #         new_key = key.replace("car", "")
                #     elif "bicycle" in key:
                #         new_key = key.replace("bicycle", "")
                #     elif "pedestrian" in key:
                #         new_key = key.replace("pedestrian", "")
                #     else:
                #         continue
                
                obj_type = self.get_obj_type(key)
                obj_param = observation.object_info[obj_type][new_key]
                lat_bias = np.abs(- (obj_param.x - observation.ego_info.x) * np.sin(observation.ego_info.yaw) + (obj_param.y - observation.ego_info.y) * np.cos(observation.ego_info.yaw))
                if obj_type != "vehicle" or self.head_on_judge(observation.ego_info.yaw, obj_param.yaw):
                    continue
                elif self.same_dir_judge(observation.ego_info.yaw, observation.ego_info.width, obj_param.yaw, obj_param.width, lat_bias):
                    continue
                elif obj_param.v < 0.4 and (self.map_type == "mixed" or lat_bias > 5.0):
                    continue
                else:
                    # print(obj_param)
                    obj_list.append([obj_param, key])
            
            if len(obj_list) < 1:
                # if abs(action[1]) < 0.01:
                #     action[1] = 0.0
                delta = (action[1] - observation.ego_info.rot) / observation.test_info["dt"]
                # if self.map_type == "roundabout":
                #     a_new, steer_new = self.final_check_roundabout(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit, steer_upper_limit, steer_lower_limit, observation)
                #     action = [a_new, steer_new]
                #     # a_new = self.final_check(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit)
                #     # action = [a_new, action[1]]
                # else:
                #     a_new = self.final_check(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit)
                #     action = [a_new, action[1]]
                # if observation.ego_info.v + a_new * observation.test_info["dt"] < 0.03:
                #     a_new = (0.03 - observation.ego_info.v) / observation.test_info["dt"]
                # print('finish act')
                a_new = self.final_check(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit)
                action = [a_new, action[1]]
                self.slow_down = (action[0] < origin_accel - 1) or (a_new < -1.0)
                # return [a_new, action[1]]
                # print(f"action_final: {action}")
                return action

            state = np.zeros((self.vehicleModel.n_dim, 1))
            state[self.vehicleModel.X_GLOBAL, 0] = observation.ego_info.x
            state[self.vehicleModel.Y_GLOBAL, 0] = observation.ego_info.y
            state[self.vehicleModel.V_LON, 0] = observation.ego_info.v
            state[self.vehicleModel.PHI, 0] = observation.ego_info.yaw

            action_ref = np.zeros((self.vehicleModel.n_control, 1))
            action_ref[self.vehicleModel.A_LON, 0] = action[0]
            action_ref[self.vehicleModel.TAN_STEER, 0] = action[1]

            if is_lane_change:
                alpha = 2.0
            else:
                alpha = 0.5

            # if self.map_type == 'highway':
            #     Q = np.diag([0.05, 100.0])
            # else:
            #     Q = np.diag([0.05, 20.0])

            cbf = VehicleCBF(self.vehicleModel, obj_list, observation.test_info["dt"], alpha)
            action = cbf.solove(state, observation.ego_info.length, observation.ego_info.width, 
                                action_ref, a_lower_limit, a_upper_limit, steer_lower_limit, steer_upper_limit)
        # ====================================================Control======================================================
        
        '''
        if self.map_type == "highway":
            if observation.test_info['t'] < 1e-3:
                self.EgoVehicleInfo = self.planner.getStateInit_highway(ob, self.lane_id)
        
            preview_point, self.EgoVehicleInfo, self.num_change, VehicleInfo = self.planner.getDecision_highway(self.EgoVehicleInfo, ob, self.to_order, self.num_change, self.direction, self.road_data, self.bias)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
            action = self.planner.act_highway(self.EgoVehicleInfo, preview_point, VehicleInfo)

        elif self.map_type == "intersection":
            action = self.planner.act(ob, self.TrajectoryList)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。

        elif self.map_type == 'ramp':
            a1, a2, rampFlag = self.planner.cmc_getRampFlag(ob, self.task_info, self.roadStructureDict, self.planedLanes, self.AllLanesDict)
            acc, steer, fIndex, ego = self.planner.cmc_act(ob, self.task_info, self.planedLanes, self.AllLanesDict, rampFlag)
            action = [acc, steer]

        elif self.map_type == 'roundabout':
            a1, a2, rampFlag = self.planner.ktq_getRampFlag(ob, self.task_info, self.roadStructureDict, self.planedLanes, self.AllLanesDict)
            #print("rampFlag"+str(rampFlag)+"*******************************************************")
            acc, steer, fIndex, ego = self.planner.ktq_act(ob, self.task_info, self.planedLanes, self.AllLanesDict, rampFlag)
            action = [acc, steer]
        '''
        # return [self.deside_acc(state), 0]
        # if abs(action[1]) < 0.01:
        #     action[1] = 0.0
        if final_flag or self.map_type == 'highway':
            action[1] = origin_steer
        delta = (action[1] - observation.ego_info.rot) / observation.test_info["dt"]
        # if self.map_type == "roundabout":
        #     a_new, steer_new = self.final_check_roundabout(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit, steer_upper_limit, steer_lower_limit, observation)
        #     action = [a_new, steer_new]
        #     # a_new = self.final_check(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit)
        #     # action = [a_new, action[1]]
        # else:
        #     a_new = self.final_check(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit)
        #     action = [a_new, action[1]]
        # if observation.ego_info.v + a_new * observation.test_info["dt"] < 0.03:
        #     a_new = (0.03 - observation.ego_info.v) / observation.test_info["dt"]
        # print('finish act')
        a_new = self.final_check(observation.ego_info.v, action[0], action[1], delta, a_upper_limit, a_lower_limit)
        action = [a_new, action[1]]
        self.slow_down = (action[0] < origin_accel - 1) or (a_new < -1.0)
        # print(f"action_final: {action}")
        # return [a_new, action[1]]        
        return action
    
    '''
    def observation_trans(self, observation: Observation) -> dict:
        # 加载主车信息
        frame = pd.DataFrame(
            vars(observation.ego_info),
            columns=['x', 'y', 'v', 'yaw', 'length', 'width'], 
            index=['ego']
        )
        # 加载背景要素状态信息
        for obj_type in observation.object_info:
            for obj_name, obj_info in observation.object_info[obj_type].items():
                sub_frame = pd.DataFrame(vars(obj_info), columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[obj_name])
                frame = pd.concat([frame, sub_frame])
        # state = frame.to_numpy()
        vehicle_info = frame.to_dict(orient='index')

        ob = dict()
        ob['vehicle_info'] = vehicle_info
        ob['test_setting'] = observation.test_info
        if self.map_type == "intersection":
            ob['light_info'] = self.light_info[str(np.around(observation.test_info['t'], 3))]
        
        return ob

    def deside_acc(self, state: pd.DataFrame) -> float:
        v, fv, dis_gap, direction = self.getInformFront(state)
        # print(v, fv, dis_gap,direction)
        # print(state)
        if dis_gap < 0:
            a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        else:
            # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.t)
            self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
                v - fv) / 2 / (self.a * self.b) ** 0.5
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))
        # 对加速度进行约束
        a_idm = np.clip(a_idm, -self.a_bound, 1e7)
        # print(v,fv,dis_gap,a_idm,self.s_)
        # print(state,v,fv,dis_gap,a_idm)
        return a_idm

    def getInformFront(self, state: pd.DataFrame) -> Tuple[float, float, float, float]:
        # direction = np.sign(state[0,2])
        if state[0, 3] < np.pi / 2 or state[0, 3] > np.pi * 3 / 2:
            direction = 1.0
        else:
            direction = -1.0
        state[:,0] = state[:,0]*direction
        # state[:,2] = state[:,2]*direction
        ego = state[0,:]
        v, fv, dis_gap = ego[2], -1, -1
        
        # 在本车前侧
        x_ind = ego[0] < state[:,0]
        y_ind = (np.abs(ego[1] - state[:,1])) < ((ego[5] + state[:,5])/2)
        ind = x_ind & y_ind
        if ind.sum() > 0:
            state_ind = state[ind,:]
            front = state_ind[(state_ind[:,0]-ego[0]).argmin(),:]
            # print(front)
            fv = front[2]
            dis_gap = front[0] - ego[0] - (ego[4] + front[4])/2
        if dis_gap > 100:
            dis_gap = -1
            fv = -1
        return v, fv, dis_gap, direction
    '''

    '''
    roundabout : (11->1) -> 2 -> 13 -> (14->8) -> 9 -> 10
        0 -> [21 25]    5 -> [33 34]    18 -> [47 52]        3 -> [37]                     7 -> [58 62]
    11->1 -> [20 23] -> 2 -> [29 31] -> 13 -> [45 50] -> 14->8 -> [38] -> 9 -> [40 42] -> 10 -> [56 61] -> 11->1
                     -> 15           -> 16            -> 6                             -> 12            -> 4
    '''
    def roundabout_interest(self, roadId: int, ego_x: float, ego_y: float) -> Tuple[bool, List[int]]:
        interest_road_list = []
        if roadId in self.roadid_out_roundabout:
            return False, interest_road_list, None
        
        elif roadId in self.roadid_into_roundabout:
            idx = 0
            for ind in range(len(self.road_into_roundabout)):
                item = self.road_into_roundabout[ind]
                if roadId == item[0]:
                    idx = ind
                    break
            
            interest_road_list.append(self.road_into_roundabout[idx-1][0])
            interest_road_list += self.road_into_junction[str(self.road_into_roundabout[idx-1][0])]
            interest_road_list.append(self.road_into_roundabout[idx-2][0])
            interest_road_list += self.road_into_junction[str(self.road_into_roundabout[idx-2][0])]

            return True, interest_road_list, self.road_into_roundabout[idx-1][1]
        
        else:
            rad = np.arctan2(self.center_point[1] - ego_y, self.center_point[0] - ego_x)
            
            flag = 0
            for idx in range(len(self.road_into_roundabout)):
                if rad < self.road_into_roundabout[idx][2]:
                    flag = 1
                    break
            if flag == 0:
                idx = 0
            
            if idx == len(self.road_into_roundabout) - 1:
                interest_road_list.append(self.road_into_roundabout[idx][0])
                interest_road_list += self.road_into_junction[str(self.road_into_roundabout[idx][0])]
                interest_road_list.append(self.road_into_roundabout[0][0])
                interest_road_list += self.road_into_junction[str(self.road_into_roundabout[0][0])]

                return True, interest_road_list, self.road_into_roundabout[idx][1]
            else:
                interest_road_list.append(self.road_into_roundabout[idx][0])
                interest_road_list += self.road_into_junction[str(self.road_into_roundabout[idx][0])]
                interest_road_list.append(self.road_into_roundabout[idx+1][0])
                interest_road_list += self.road_into_junction[str(self.road_into_roundabout[idx+1][0])]

                return True, interest_road_list, self.road_into_roundabout[idx][1]

        # if roadId in [15, 16, 6, 12, 4]:
        #     return False, []
        
        # elif roadId in [11, 1, 21, 25, 20, 23]:
        #     return True, [5, 18]
        # elif roadId in [2, 33, 34, 29, 31]:
        #     return True, [18, 3]
        # elif roadId in [13, 47, 52, 45, 50]:
        #     return True, [3, 7]
        # elif roadId in [14, 8, 37, 38, 9, 40, 42]:
        #     return True, [7, 0]
        # elif roadId in [10, 58, 62, 56, 61]:
        #     return True, [0, 5]

        # elif roadId in [0]:
        #     return True, [11, 1, 7, 10, 9, 3]
        # elif roadId in [7]:
        #     return True, [10, 9, 3, 14, 8]
        # elif roadId in [3]:
        #     return True, [14, 8, 18, 13, 5]
        # elif roadId in [18]:
        #     return True, [13, 5, 2, 0]
        # elif roadId in [5]:
        #     return True, [2, 0, 11, 1, 7]

        # else:
        #     return False, []

    def get_merge_car(self,local_planner:LPInfo):
        road_id = int(local_planner.ego_lane_id.split('.')[0])
        ego_x = float(local_planner.state[0,0])
        ego_y = float(local_planner.state[0,1])
        is_merge, merge_road, merge_point = self.roundabout_interest(road_id,ego_x,ego_y)
        merge_car = []
        if is_merge:
            for i in range(len(local_planner.state)):
                id = local_planner.object_id[i]
                SV_lane_id = int(local_planner.get_SV_lane(local_planner.state[i,0],local_planner.state[i,1]).split('.')[0])
                if SV_lane_id == merge_road[0]:
                    merge_car.append(id)
          
        return merge_car, merge_point
    
    def get_merge_distance(self,local_planner:LPInfo,x,y):
        position = np.array([x, y])
        KD_tree = local_planner.globe_info.all_road_info["road_kd_tree"]
        distance, nearest_index  = KD_tree.query(position)
        lane_id = local_planner.globe_info.all_road_info["road_ids_array"][nearest_index]
        distance = 0.0
        points = local_planner.globe_info.all_road_info["road_points_array"]
        for i in range(nearest_index, len(local_planner.globe_info.all_road_info["road_ids_array"])-1):
            if local_planner.globe_info.all_road_info["road_ids_array"][i] != lane_id:
                break
            distance += ((points[i,0]-points[i+1,0])**2+(points[i,1]-points[i+1,1])**2)**0.5
        return distance
    
    def is_merge_safe(self,local_planner:LPInfo):
        is_merge_safe = True
        merge_car, merge_point = self.get_merge_car(local_planner)
        # print(merge_car)
        for id in merge_car:
            index = local_planner.object_id.index(id)
            # distance = self.get_merge_distance(local_planner,local_planner.state[index,0],local_planner.state[index,1])
            distance = ((local_planner.state[index,0] - merge_point[0])**2+(local_planner.state[index,1]-merge_point[1])**2)**0.5
            v = local_planner.state[index,3]
            # ego_distance = self.get_merge_distance(local_planner,local_planner.state[0,0],local_planner.state[0,1])
            ego_distance = ((local_planner.state[0,0] - merge_point[0])**2+(local_planner.state[0,1]-merge_point[1])**2)**0.5
            ego_v = local_planner.state[0,3]
            a = 1.4
            b = 2.0
            v_min = 0.0
            if distance < ego_distance:
                gap = ego_distance - distance 
                if gap < a*ego_v + b*(ego_v-v) and v > v_min:
                    is_merge_safe = False
                    break
            else:
                gap = distance - ego_distance
                if gap < a*v + b*(v-ego_v) and v > v_min:
                    is_merge_safe = False
                    break
        return is_merge_safe
    
    def get_all_ramp_Id(self, junctions, ramp_roadId, ramp_sectionId, ramp_laneId):
        if len(junctions) == 0 or ramp_roadId is None or ramp_sectionId is None or ramp_laneId is None:
            return False, []

        for junction in junctions:
            incomingRoadId = None
            junctionLaneId = []

            for connection in junction.connections:
                if connection.connectingRoad != ramp_roadId:
                    continue
                for laneLink in connection.laneLinks:
                    junctionLaneId.append(laneLink.toId)

                if ramp_laneId in junctionLaneId:
                    incomingRoadId = connection.incomingRoad
                    break
            
        if incomingRoadId is None:
            return False, []

        all_ramp_lane_Id = []
        for junction in junctions:            
            for connection in junction.connections:
                if connection.connectingRoad != ramp_roadId:
                    continue
                if connection.incomingRoad != incomingRoadId:
                    continue
                for laneLink in connection.laneLinks:
                    all_ramp_lane_Id.append(laneLink.toId)
            
        all_ramp_Id = []
        for discretelane in self.road_data.discretelanes:
            Id = discretelane.lane_id.split('.')
            roadId = int(Id[0])
            sectionId = int(Id[1])
            laneId = int(Id[2])

            if roadId == ramp_roadId and sectionId <= ramp_sectionId and laneId in all_ramp_lane_Id:   
                all_ramp_Id.append(discretelane.lane_id)

        return True, all_ramp_Id      
