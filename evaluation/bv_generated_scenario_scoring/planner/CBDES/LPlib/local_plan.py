from ..GPlib.globe_plan import GPInfo
from utils.observation import Observation
from scipy.spatial import KDTree
import numpy as np
from scipy.interpolate import interp1d
from math import *


class IDM:
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
        self.a_bound = a_bound
        self.exv = exv
        self.t = t
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_front = 0
        self.s_behind = 0
        


class LPInfo:
    def __init__(self, globe_info:GPInfo, scene_type, observation:Observation, scenario_type:str, slow_down, lots_junctions):
        self.scene_type = scene_type
        self.globe_info = globe_info
        self.scenario_type = scenario_type
        self.lots_junctions = lots_junctions
        self.interset_area_width = 30
        if scene_type=='highway':
            self.interset_area_front = 100
            self.interset_area_behind = 50
        else:
            self.interset_area_front = 50
            self.interset_area_behind = 20

        self.idm = IDM()
        self.t = 0.1
        self.num_step = 30
        self.plan_time = 3
        if scene_type=='highway':
            self.safe_distance_car = 3.75
            self.safe_distance_bicycle = 1.5
            self.safe_distance_pedestrain = 5.0
        elif scene_type=='mixed':
            self.safe_distance_car = 3.75
            self.safe_distance_bicycle = 1.5
            self.safe_distance_pedestrain = 5.0
        elif scene_type=='roundabout':
            self.safe_distance_car = 5.5 # 4.5
            self.safe_distance_bicycle = 1.5
            self.safe_distance_pedestrain = 1.0
        elif scene_type=='intersection':
            self.safe_distance_car = 5.5
            self.safe_distance_bicycle = 1.5
            self.safe_distance_pedestrain = 1.0
        else:
            self.safe_distance_car = 4.0
            self.safe_distance_bicycle = 1.5
            self.safe_distance_pedestrain = 1.0
        self.kd_tree_return_nums = 10
        self.no_safe_path_factor = 0.01
        self.speed_th = 13.0
        if scene_type=='highway':
            self.max_speed = 33.0
        elif scene_type == 'mixed':
            self.max_speed = 20.0
        else:
            self.max_speed = 9.0

        self.nearest_car_v = 1 * self.max_speed
        self.nearest_car_dis = 1 * self.interset_area_front
        self.exv_num = 10


        self.state, self.object_id = self.trans_observation(observation)
        self.relative_coordinates_rotated = self.transform_to_ego_coordinate(self.state)
        self.targetPos_bais = self.get_targetPos_bais()
        self.TP_in_front, self.TP_v_in_front, self.TP_in_behind, self.TP_v_in_behind = self.get_interest_car()
        self.ego_lane_id, self.ego_frenet_y, self.ego_nearest_index = self.get_EV_frenet(self.state)

        if self.scenario_type == "REPLAY":
            self.exv_list = [1.2, 1.1, 1.0, 0.8, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0]
        else:
            self.exv_list = [1.0, 0.8, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0] # 降序
        
        if self.scene_type == "intersection":
            self.exv_list = [1.1, 1.0, 0.8, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0]
        # elif self.scene_type == "roundabout":
        #     self.exv_list = [1.0,0.8,0.6,0.5,0.4,0.3,0.2,0.1,0.05,0]

        if self.scene_type == "intersection" and abs(observation.ego_info.rot) > 0.075:
            self.exv_list = [0.9, 0.8, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0] 
        elif self.scene_type == "roundabout" and slow_down:
            self.exv_list = [0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0]
        # elif self.scene_type == "roundabout" and self.ego_lane_id in self.globe_info.path[-1]:
        #     self.exv_list = [1.2, 1.0, 0.8, 0.6,0.5,0.4,0.3,0.2,0.1,0.05,0] 
        
    

    def trans_observation(self, observation:Observation):
       
        state = np.empty((0, 6))
        object_id = []
        ego_state = np.array([observation.ego_info.x,observation.ego_info.y,observation.ego_info.yaw,
                              observation.ego_info.v,observation.ego_info.width,observation.ego_info.length])
        state = np.vstack([state, ego_state])
        object_id.append("ego")
        for object_type, state_dict in observation.object_info.items():
            for id, objcet_state in state_dict.items():
                obj_state = np.array([objcet_state.x,objcet_state.y,objcet_state.yaw,
                              objcet_state.v,objcet_state.width,objcet_state.length])
                state = np.vstack([state, obj_state])
                if object_type == "vehicle" and (not id.startswith("car")):
                    new_id = "car" + id
                    object_id.append(new_id)
                elif object_type == "bicycle" and (not id.startswith("bicycle")):
                    new_id = "bicycle" + id
                    object_id.append(new_id)
                elif object_type == "pedestrian" and (not id.startswith("pedestrian")):
                    new_id = "pedestrian" + id
                    object_id.append(new_id)
                else:
                    object_id.append(id)
        # print("object_id:",object_id)
        return state, object_id

    def get_targetPos_bais(self):
        left_point = self.globe_info.glob_road_info["left_boundary_array"][self.globe_info.targetPos_index]
        right_point = self.globe_info.glob_road_info["right_boundary_array"][self.globe_info.targetPos_index]
        target_point = (self.globe_info.targetPos[0]+self.globe_info.targetPos[1])/2
        ratio = self.point_projection(left_point, right_point, target_point)
        if ratio < 0.375:
            return "left"
        elif ratio > 0.625:
            return "right"
        else:
            return "center"


    def get_max_speed(self):
        if self.scene_type == "ramp" and len(self.state) > 1:
            SV_state = self.state[1:]
            max_state = np.max(SV_state,axis=0)
            max_speed = float(max_state[3])
            if max_speed > self.speed_th  :
                return 33.0
            else:
                return 9.0
        else:
            return self.max_speed


    def get_EV_frenet(self, state):
        
        ego_x = state[0][0]
        ego_y = state[0][1]
        ego_position = np.array([ego_x, ego_y])
        KD_tree = self.globe_info.glob_road_info["road_kd_tree"]
        distance, nearest_index  = KD_tree.query(ego_position)
        ego_lane_id = self.globe_info.glob_road_info["road_ids_array"][nearest_index]
        ego_frenet_y = self.globe_info.glob_road_info["frenet_s_array"][nearest_index]
        # print(f"ego_lane_id: {ego_lane_id}")
        return ego_lane_id, ego_frenet_y, nearest_index
    
    def get_SV_lane(self, x, y):
        
        position = np.array([x, y])
        KD_tree = self.globe_info.all_road_info["road_kd_tree"]
        distance, nearest_index  = KD_tree.query(position)
        lane_id = self.globe_info.all_road_info["road_ids_array"][nearest_index]
        return lane_id
    
    def get_SV_frenet(self, x, y):
       
        position = np.array([x, y])
        KD_tree = self.globe_info.glob_road_info["road_kd_tree"]
        distance, nearest_index  = KD_tree.query(position)
        SV_lane_id = self.globe_info.glob_road_info["road_ids_array"][nearest_index]
        SV_frenet_y = self.globe_info.glob_road_info["frenet_s_array"][nearest_index]
        return SV_lane_id, SV_frenet_y
    
    def transform_to_ego_coordinate(self, state):
        ego_x, ego_y, ego_yaw, _, _, _ = state[0]  # 提取自车的信息
        relative_coordinates = state[:, :2] - np.array([ego_x, ego_y])  # 计算所有车辆相对于自车的相对坐标
        # 将相对坐标转换为自车坐标系
        rotation_matrix = np.array([
            [np.cos(ego_yaw), np.sin(ego_yaw)],
            [-np.sin(ego_yaw), np.cos(ego_yaw)]
        ])
        relative_coordinates_rotated = np.dot(relative_coordinates, rotation_matrix.T)
        return relative_coordinates_rotated
    
    def get_interest_car(self):
        # # print("finish transform_to_ego_coordinate")
        TP_in_front, TP_v_in_front = self.check_TP_in_front(self.state, self.relative_coordinates_rotated, self.object_id)
        # # print("finish check_TP_in_front")
        TP_in_behind, TP_v_in_behind = self.check_TP_in_behind(self.state, self.relative_coordinates_rotated, self.object_id)
        return TP_in_front, TP_v_in_front, TP_in_behind, TP_v_in_behind

    def check_TP_in_front(self,state, relative_coordinates, object_id):
        
        TP_in_front = []
        TP_v_in_front = []
       
        for i in range(1, len(relative_coordinates)):
            if object_id[i].startswith("car"):
                is_front = (relative_coordinates[i][0]>=0 and relative_coordinates[i][0]<=self.interset_area_front and 
                            relative_coordinates[i][1]>=-self.interset_area_width and relative_coordinates[i][1]<=self.interset_area_width)
                            # and (np.pi/180*170 < abs(state[i][2]-state[0][2]) < np.pi/180*190)) 
            else:
                is_front = (relative_coordinates[i][0]>=0 and relative_coordinates[i][0]<=self.interset_area_front and 
                            relative_coordinates[i][1]>=-self.interset_area_width and relative_coordinates[i][1]<=self.interset_area_width
                            )   
            if is_front:
                TP_in_front.append(object_id[i])
                TP_v_in_front.append(float(state[i][3]))
        return TP_in_front, TP_v_in_front
    
    def check_TP_in_behind(self,state, relative_coordinates, object_id):
        
        TP_in_behind = []
        TP_v_in_behind = []
        for i in range(1, len(relative_coordinates)):
            is_behind = (relative_coordinates[i][0]<=0 and relative_coordinates[i][0]>=-self.interset_area_behind and 
                        relative_coordinates[i][1]>=-self.interset_area_width and relative_coordinates[i][1]<=self.interset_area_width)
                        # and abs(state[i][2]-state[0][2])<np.pi/2)       
            if is_behind:
                TP_in_behind.append(object_id[i])
                TP_v_in_behind.append(float(state[i][3]))
        return TP_in_behind,TP_v_in_behind
    

    def has_lane_change(self, ego_lane_id, SV_lane_id):
        
        ego_index = self.globe_info.path.index(ego_lane_id)
        SV_index = self.globe_info.path.index(SV_lane_id)
        path = self.globe_info.path
        for i in range(ego_index, SV_index):
            if path[i].split('.')[0] == path[i+1].split('.')[0]:
                return True
        return False

    def get_front_car(self, TP_in_front, TP_v_in_front, ego_lane_id, ego_frenet_y):
       
        nearest_car_id = None
        nearest_car_v = None
        nearest_car_frenet_y = None
        min_distace = float('inf')
        for i in range(len(TP_in_front)):
            vehicle_id = TP_in_front[i]
            vehicle_v = TP_v_in_front[i]
            # if (vehicle_id.startswith("car") or vehicle_id.startswith("bicycle")) and vehicle_v>0:
            if self.scene_type=='highway' or self.scene_type=='mixed':
                v_min = 0
            else:
                v_min = -1
            if vehicle_id.startswith("car") and vehicle_v>v_min:
                index = self.object_id.index(vehicle_id)
                x = float(self.state[index][0])
                y = float(self.state[index][1])
                v = float(self.state[index][3])
                lane_id = self.get_SV_lane(x,y)
                if lane_id in self.globe_info.path:
                    # # print("get_SV_lane")
                    is_lane_change = self.has_lane_change(ego_lane_id, lane_id)
           
                    # # print("has_lane_change")
                    if not is_lane_change:
                        SV_lane_id, SV_frenet_y = self.get_SV_frenet(x, y)
                        # # print("get_SV_frenet")     
                        ego_index = self.globe_info.path.index(ego_lane_id)
                        SV_index = self.globe_info.path.index(SV_lane_id)  
                        if SV_index >= ego_index and 0 < SV_frenet_y - ego_frenet_y < min_distace:
                            nearest_car_id = vehicle_id
                            # nearest_car_v = observation.object_info['vehicle'][vehicle_id].v
                            nearest_car_v = v
                            nearest_car_frenet_y = SV_frenet_y
                            min_distace = SV_frenet_y - ego_frenet_y
                  
        return nearest_car_id, nearest_car_v, nearest_car_frenet_y
    
    def get_front_car_change_lane(self, TP_in_front, TP_v_in_front, ego_lane_id, ego_frenet_y):
        
        nearest_car_id = None
        nearest_car_v = None
        nearest_car_frenet_y = None
        min_distace = float('inf')
        for i in range(len(TP_in_front)):
            vehicle_id = TP_in_front[i]
            vehicle_v = TP_v_in_front[i]
            # if (vehicle_id.startswith("car") or vehicle_id.startswith("bicycle")) and vehicle_v>0:
            if vehicle_id.startswith("car") and vehicle_v>0:
                index = self.object_id.index(vehicle_id)
                x = float(self.state[index][0])
                y = float(self.state[index][1])
                v = float(self.state[index][3])
                lane_id = self.get_SV_lane(x,y)
                if lane_id in self.globe_info.path:
                    # # print("get_SV_lane")
                    is_lane_change = self.has_lane_change(ego_lane_id, lane_id)
                    # # print("has_lane_change")
                    if is_lane_change:
                        SV_lane_id, SV_frenet_y = self.get_SV_frenet(x, y)
                        # # print("get_SV_frenet")       
                        if 0 < SV_frenet_y - ego_frenet_y < min_distace:
                            nearest_car_id = vehicle_id
                            # nearest_car_v = observation.object_info['vehicle'][vehicle_id].v
                            nearest_car_v = v
                            nearest_car_frenet_y = SV_frenet_y
                            min_distace = SV_frenet_y - ego_frenet_y
                  
        return nearest_car_id, nearest_car_v, nearest_car_frenet_y

    def get_behind_car(self, TP_in_behind, TP_v_in_behind, ego_lane_id, ego_frenet_y):
        
        nearest_car_id = None
        nearest_car_v = None
        nearest_car_frenet_y = None
        min_distace = float('inf')
        for i in range(len(TP_in_behind)):
            vehicle_id = TP_in_behind[i]
            vehicle_v = TP_v_in_behind[i]
            # if (vehicle_id.startswith("car") or vehicle_id.startswith("bicycle")) and vehicle_v>0:
            if vehicle_id.startswith("car") and vehicle_v>0:
                index = self.object_id.index(vehicle_id)
                x = float(self.state[index][0])
                y = float(self.state[index][1])
                v = float(self.state[index][3])
                lane_id = self.get_SV_lane(x,y)
                if lane_id in self.globe_info.path:
                    # # print("get_SV_lane")
                    is_lane_change = self.has_lane_change(ego_lane_id, lane_id)
                    # # print("has_lane_change")
                    if not is_lane_change:
                        SV_lane_id, SV_frenet_y = self.get_SV_frenet(x, y)
                        # # print("get_SV_frenet")
                        if 0 < ego_frenet_y - SV_frenet_y < min_distace:
                            nearest_car_id = vehicle_id
                            # nearest_car_v = observation.object_info['vehicle'][vehicle_id].v
                            nearest_car_v = v
                            nearest_car_frenet_y = SV_frenet_y
                            min_distace = SV_frenet_y - ego_frenet_y
        return nearest_car_id, nearest_car_v, nearest_car_frenet_y
    
    def get_behind_car_change_lane(self, TP_in_behind, TP_v_in_behind, ego_lane_id, ego_frenet_y):
        
        nearest_car_id = None
        nearest_car_v = None
        nearest_car_frenet_y = None
        min_distace = float('inf')
        for i in range(len(TP_in_behind)):
            vehicle_id = TP_in_behind[i]
            vehicle_v = TP_v_in_behind[i]
            # if (vehicle_id.startswith("car") or vehicle_id.startswith("bicycle")) and vehicle_v>0:
            if vehicle_id.startswith("car") and vehicle_v>0:
                index = self.object_id.index(vehicle_id)
                x = float(self.state[index][0])
                y = float(self.state[index][1])
                v = float(self.state[index][3])
                lane_id = self.get_SV_lane(x,y)
                if lane_id in self.globe_info.path:
                    # # print("get_SV_lane")
                    is_lane_change = self.has_lane_change(ego_lane_id, lane_id)
                    # # print("has_lane_change")
                    if is_lane_change:
                        SV_lane_id, SV_frenet_y = self.get_SV_frenet(x, y)
                        # # print("get_SV_frenet")
                        if 0 < ego_frenet_y - SV_frenet_y < min_distace:
                            nearest_car_id = vehicle_id
                            # nearest_car_v = observation.object_info['vehicle'][vehicle_id].v
                            nearest_car_v = v
                            nearest_car_frenet_y = SV_frenet_y
                            min_distace = SV_frenet_y - ego_frenet_y
        return nearest_car_id, nearest_car_v, nearest_car_frenet_y

    def decide_acc(self, ego_v, ego_frenet_y, nearest_car_v, nearest_car_frenet_y):
        
        v = max(ego_v, 0) # very important, ego_v可能是很小的负数，会导致计算爆炸
        fv = nearest_car_v
        dis_gap = nearest_car_frenet_y - ego_frenet_y - 5
        self.idm.s_front = self.idm.s0 + self.idm.s1 * (v / (self.idm.exv+1e-6)) ** 0.5 + self.t * v + v * (
                v - fv) / 2 / (self.idm.a * self.idm.b) ** 0.5
        self.idm.s_front = max(self.idm.s_front,self.idm.s0)
            # 求解本车加速度
        # # print("idm:", v, fv, dis_gap, self.idm.s_)
        a_idm = self.idm.a * (1 - (v / (self.idm.exv+1e-6)) ** self.idm.gama - ((self.idm.s_front / (dis_gap+1e-6)) ** 2))
        a_idm = np.clip(a_idm, -self.idm.a_bound, self.idm.a_bound)
        return a_idm
        
    def simulation_idm(self,init_ego_v, init_ego_frenet_y, init_nearest_car_v, init_nearest_car_frenet_y):
        
        accelerations = []
        ego_v = []
        frenet_y_values = [init_ego_frenet_y]
        v, ego_frenet_y, nearest_car_v, nearest_car_frenet_y = init_ego_v, init_ego_frenet_y, init_nearest_car_v, init_nearest_car_frenet_y
        for step in range(self.num_step):
            acceleration = self.decide_acc(v, ego_frenet_y, nearest_car_v, nearest_car_frenet_y)
            
            v += acceleration * self.t  # 更新速度
            v = max(v,0)
            ego_frenet_y += v * self.t  # 更新frenet_y位置
            nearest_car_frenet_y += nearest_car_v * self.t  # 更新前车 frenet_y 位置
            # # print("simulation:", acceleration, v)
            accelerations.append(acceleration)
            frenet_y_values.append(ego_frenet_y)
            ego_v.append(v)
        return frenet_y_values[-1], ego_v[-1]
    
    def get_target_point_index(self, target_point_frenet_y, ego_nearest_index):
        
        # print(target_point_frenet_y,self.globe_info.glob_road_info["frenet_s_array"][ego_nearest_index],self.globe_info.glob_road_info["frenet_s_array"][ego_nearest_index+1])
        target_point_index = ego_nearest_index + 1
        KD_tree = self.globe_info.glob_road_info["frenet_kd_tree"]
        frenet_y = np.atleast_2d(target_point_frenet_y)
        distance, nearest_index  = KD_tree.query(frenet_y, self.kd_tree_return_nums)
        nearest_index_list = []
        for i in range(nearest_index.shape[1]):
            if distance[0][i] == distance[0][0]:
                nearest_index_list.append(int(nearest_index[0][i]))
        nearest_index_list.sort()
        for index in nearest_index_list:
            if index > ego_nearest_index:
                target_point_index = index
                break
        return target_point_index
    
    def interpolate_curve(self, points):
        
        if len(points) == 1:
            points_ = np.tile(points,(30,1))
        else:
            points_ = points
        x = points_[:, 0]  # 获取 x 值
        y = points_[:, 1]  # 获取 y 值
        dt_observed = self.plan_time / (len(x) - 1)
        dt_target = self.plan_time / self.num_step
        timestamps_observed = np.arange(0, dt_observed * len(x), dt_observed)
        timestamps_observed = timestamps_observed[:len(x)]
        timestamps_observed[-1] = self.plan_time
        timestamps_target = np.arange(0, dt_target*(self.num_step+1), dt_target)
        f_x = interp1d(timestamps_observed, x, kind='linear',axis=0)
        f_y = interp1d(timestamps_observed, y, kind='linear',axis=0)
        x_target = f_x(timestamps_target)
        y_target = f_y(timestamps_target)
        new_points = np.column_stack((x_target, y_target))  # 将新的 (x, y) 坐标堆叠成数组
        new_points = new_points[1:]
        return new_points
    
    def point_projection(self, left_point, right_point, target_point):

        segment_length = np.linalg.norm(right_point - left_point)
        if float(segment_length) < 1e-3:
            return 0.0
        vector_to_left = target_point - left_point

        vector_to_right = target_point - right_point

        segment_vector = right_point - left_point
        projection_length = np.dot(vector_to_left, segment_vector) / np.dot(segment_vector, segment_vector)
        projection_point = left_point + projection_length * segment_vector

        distance_to_left = np.linalg.norm(projection_point - left_point)

        ratio = distance_to_left / segment_length

        vector1 = target_point - left_point
        vector2 = right_point - left_point

        dot_product = np.dot(vector1, vector2)
        if dot_product < 0:
            ratio = -ratio
        return float(ratio)

    def interpolate_ego(self, left_boundary_points, right_boundary_points, ego_position):
       
        proportion = self.point_projection(left_boundary_points[0], right_boundary_points[0], ego_position)
        ego_points = (right_boundary_points - left_boundary_points) * proportion + left_boundary_points
        return ego_points
    
    def interpolate_two_curve(self, ego_points, ref_points):
        
        traj = []
        length = len(ego_points)
        for i in range(length):
            point = (ref_points[i]-ego_points[i])/(length-1)*i+ego_points[i]
            traj.append(point)
        return np.array(traj)

    def get_target_paths(self,ego_nearest_index,target_point_index, ego_position):
        
        # print(ego_nearest_index,target_point_index)
        center_points = self.globe_info.glob_road_info["road_points_array"][int(ego_nearest_index):int(target_point_index+1)]
        left_boundary_points = self.globe_info.glob_road_info["left_boundary_array"][int(ego_nearest_index):int(target_point_index+1)]
        right_boundary_points = self.globe_info.glob_road_info["right_boundary_array"][int(ego_nearest_index):int(target_point_index+1)]
        # print(left_boundary_points, left_boundary_points, right_boundary_points)
        center_points_interpolate = self.interpolate_curve(center_points)
        # print(f"center_points_interpolate: {center_points_interpolate}")
        left_boundary_points_interpolate = self.interpolate_curve(left_boundary_points)
        # print(f"left_boundary_points_interpolate: {left_boundary_points_interpolate}")
        right_boundary_points_interpolate = self.interpolate_curve(right_boundary_points)
        # print(f"right_boundary_points_interpolate: {right_boundary_points_interpolate}")
        ego_points = self.interpolate_ego(left_boundary_points_interpolate, right_boundary_points_interpolate, ego_position)
        left_points_interpolate = (center_points_interpolate+left_boundary_points_interpolate)/2
        right_points_interpolate = (center_points_interpolate+right_boundary_points_interpolate)/2
        target_paths = []
        center_path = self.interpolate_two_curve(ego_points, center_points_interpolate)
        # print("自车位置：",ego_points[0], ego_position)
        target_paths.append(center_path)
        left_path = self.interpolate_two_curve(ego_points,left_points_interpolate)
        # print("interpolate_two_curve")
        target_paths.append(left_path)
        right_path = self.interpolate_two_curve(ego_points,right_points_interpolate)
        target_paths.append(right_path)
       
        return target_paths
    
    def keep_lane_paths(self, observation:Observation, is_lane_change:bool):
        
        nearest_car_id, nearest_car_v, nearest_car_frenet_y = self.get_front_car(self.TP_in_front, self.TP_v_in_front, self.ego_lane_id, self.ego_frenet_y)
        # print("finish get_front_car")
        nearest_car_b_id, nearest_car_b_v, nearest_car_b_frenet_y = self.get_behind_car(self.TP_in_behind, self.TP_v_in_behind, self.ego_lane_id, self.ego_frenet_y)
        if nearest_car_b_id is not None:
            self.max_speed = max(self.max_speed,nearest_car_b_v)
        if len(self.TP_in_behind) > 0 and self.scene_type!='highway':
            self.max_speed = max(self.max_speed,max(self.TP_v_in_behind)*1.1)
        if nearest_car_id == None:
            # nearest_car_v = -1
            # nearest_car_frenet_y = ego_frenet_y - 1
            nearest_car_v = self.nearest_car_v
            nearest_car_frenet_y = self.ego_frenet_y + self.nearest_car_dis
        target_path_muti_exv = []
        target_frenet_y_muti_exv = []
        target_v_muti_exv = []
        
        # center_points_muti_exv = []
        # for i in range(self.exv_num):
        #     self.idm.exv = self.max_speed / self.exv_num * (self.exv_num - i)
        for exv_proportion in self.exv_list:
            self.idm.exv = self.max_speed * exv_proportion
            # print("exv:",self.idm.exv)
            target_frenet_y, target_v = self.simulation_idm(observation.ego_info.v, self.ego_frenet_y, nearest_car_v, nearest_car_frenet_y)
            # print("finish simulation_idm")
            target_point_index =self.get_target_point_index(target_frenet_y, self.ego_nearest_index)
            # print("finish get_target_point_index")
            # print("target_point_index:",target_point_index)
            if is_lane_change:
                for i in range(self.ego_nearest_index,len(self.globe_info.glob_road_info["road_points_array"])):
                    if self.globe_info.glob_road_info["road_ids_array"][i] != self.ego_lane_id:
                        target_point_index = min(i-1,target_point_index)
                        break
            # print("target_point_index:",target_point_index)
            target_paths = self.get_target_paths(self.ego_nearest_index,target_point_index, self.state[0][:2])
            # print("finish get_target_paths")
            target_path_muti_exv.extend(target_paths)
            target_frenet_y_muti_exv.extend([target_frenet_y]*len(target_paths))
            target_v_muti_exv.extend([target_v]*len(target_paths))
            # center_points_muti_exv.append(center_points)
        return target_path_muti_exv, target_frenet_y_muti_exv, target_v_muti_exv#, TP_in_front, TP_in_behind

    def check_collision(self, traj_group, id_list, single_traj, safety_distance, obj_type, observation):
       
        
        traj_num = traj_group.shape[0]
        for ind in range(traj_num):
            obj_traj = traj_group[ind, :, :]
            distances = np.linalg.norm(obj_traj - single_traj, axis=1)
            collision_ind = np.where(distances < safety_distance)[0]
            
            if len(collision_ind) > 0:
                id = id_list[ind]
                index = self.object_id.index(id)
                ego_length = observation.ego_info.length
                ego_width = observation.ego_info.width
                obj_length = float(self.state[index][5])
                obj_width = float(self.state[index][4])
                
                for c_ind in collision_ind:
                    if c_ind == 0:
                        ego_yaw = observation.ego_info.yaw
                        obj_yaw = float(self.state[index][2])
                    else:
                        ego_yaw = np.arctan2(single_traj[c_ind, 1] - single_traj[c_ind - 1, 1], single_traj[c_ind, 0] - single_traj[c_ind - 1, 0])
                        obj_yaw = np.arctan2(obj_traj[c_ind, 1] - obj_traj[c_ind - 1, 1], obj_traj[c_ind, 0] - obj_traj[c_ind - 1, 0])
                    
                    diff_y = obj_traj[c_ind, 1] - single_traj[c_ind, 1]
                    diff_x = obj_traj[c_ind, 0] - single_traj[c_ind, 0]
                    y_bias = - diff_x * np.sin(ego_yaw) + diff_y * np.cos(ego_yaw)
                    if np.abs(y_bias) < (ego_width + obj_width) / 2.0 + safety_distance / 10:
                        # # print(f"id: {id}, ego width: {ego_width}, obj width: {obj_width}, y bias: {y_bias}, final point: {single_traj[-1]}")
                        return True
            else:
                continue

        return False

    def get_safe_path(self,target_paths:list,target_frenet_y:list, target_v:list, TP_in_front, TP_in_behind, forecasted_trajectories, observation):
        
        final_flag = False
        interest_path_car = []
        interest_car = []
        interest_path_bicycle = []
        interest_bicycle = []
        interest_path_pedestrain = []
        interest_pedestrain = []
        for id in forecasted_trajectories:
            # if id in TP_in_front or id in vehicles_in_behind:
            if id in TP_in_front:
                if id.startswith("car"):
                    index = np.argmax(forecasted_trajectories[id]["prob"])
                    interest_path_car.append(forecasted_trajectories[id]["future"][index])
                    interest_car.append(id)
                elif id.startswith("bicycle"):
                    index = np.argmax(forecasted_trajectories[id]["prob"])
                    interest_path_bicycle.append(forecasted_trajectories[id]["future"][index])
                    interest_bicycle.append(id)
                else:
                    index = np.argmax(forecasted_trajectories[id]["prob"])
                    interest_path_pedestrain.append(forecasted_trajectories[id]["future"][index])
                    interest_pedestrain.append(id)
              
        
        
        # # print("for finish")
        final_path = None
        final_v = None
        final_index = None
        frenet_y_max = 0
        if len(interest_path_car) + len(interest_path_bicycle) + len(interest_path_pedestrain) == 0:
            final_index = np.argmax(target_frenet_y)
            final_path = target_paths[final_index]
            final_v = target_v[final_index]
            frenet_y_max = target_frenet_y[final_index]
        else:         
            collision_car = False
            collision_bicycle = False
            collision_pedestrain = False
            for i in range(len(target_paths)):
                if self.scene_type == "roundabout" and self.lots_junctions and self.scenario_type == "REPLAY":
                    if i % 3 != 0:
                        continue
                path = target_paths[i]
                frenet_y = target_frenet_y[i]
                v = target_v[i]
                if len(interest_path_car) > 0:
                    obj_type = "vehicle"
                    interest_path_car_array = np.stack(interest_path_car)
                    collision_car = self.check_collision(interest_path_car_array, interest_car, path, self.safe_distance_car, obj_type, observation)
                if len(interest_path_bicycle) > 0:
                    obj_type = "bicycle"
                    interest_path_bicycle_array = np.stack(interest_path_bicycle)
                    collision_bicycle = self.check_collision(interest_path_bicycle_array, interest_bicycle, path, self.safe_distance_bicycle, obj_type, observation)
                if len(interest_path_pedestrain) > 0:
                    obj_type = "pedestrian"
                    interest_path_pedestrain_array = np.stack(interest_path_pedestrain)
                    collision_pedestrain = self.check_collision(interest_path_pedestrain_array, interest_pedestrain, path, self.safe_distance_pedestrain, obj_type, observation)
                collision = collision_car or collision_bicycle or collision_pedestrain
                if not collision and frenet_y > frenet_y_max:
                    final_path = path
                    final_v = v
                    final_index = i
                    frenet_y_max = frenet_y
            # print(f"final_index: {final_index}")
        if final_v == None:
            # print("\033[91mNo safe path\033[0m")
            final_x = self.state[0][0] + self.no_safe_path_factor * cos(self.state[0][2])
            final_y = self.state[0][1] + self.no_safe_path_factor * sin(self.state[0][2])
            final_path = [[final_x,final_y] for _ in range(30)]
            final_path = np.array(final_path)
            final_v = 0
        elif frenet_y_max >= self.globe_info.targetPos_frenet_s:
            final_flag = True
            trj_num_for_one_v = int(len(target_paths) / len(self.exv_list))
            index = (final_index // trj_num_for_one_v) * trj_num_for_one_v

            if self.targetPos_bais == "left":
                final_path = target_paths[index+1]
                final_v = target_v[index+1]
            elif self.targetPos_bais == "right":
                final_path = target_paths[index+2]
                final_v = target_v[index+2]
            else:
                final_path = target_paths[index]
                final_v = target_v[index]
        else:
            hhhhhh = 666666
            # print("frenet_y_max:",frenet_y_max,"self.globe_info.targetPos_frenet_s:",self.globe_info.targetPos_frenet_s)
        return final_path, final_v, final_flag
    



    def get_neighbor_lane_front_car(self, TP_in_front, observation:Observation, ego_lane_id, ego_frenet_y):
       
        nearest_car_id = None
        nearest_car_v = None
        nearest_car_frenet_y = None
        min_distace = float('inf')
        for vehicle_id in TP_in_front:
            if vehicle_id.startswith("car"):
                index = self.object_id.index(vehicle_id)
                x = float(self.state[index][0])
                y = float(self.state[index][1])
                v = float(self.state[index][3])
                lane_id = self.get_SV_lane(x,y)
                
                if lane_id in self.globe_info.path:
                    ego_lane_index = self.globe_info.path.index(ego_lane_id)
                    front_car_lane_index = self.globe_info.path.index(lane_id)
                    if front_car_lane_index - ego_lane_index == 1:
                        SV_lane_id, SV_frenet_y = self.get_SV_frenet(x, y)
                        # # print("get_SV_frenet")
                        if 0 < SV_frenet_y - ego_frenet_y < min_distace:
                            nearest_car_id = vehicle_id
                            # nearest_car_v = observation.object_info['vehicle'][vehicle_id].v
                            nearest_car_v = v
                            nearest_car_frenet_y = SV_frenet_y
                            min_distace = SV_frenet_y - ego_frenet_y
        return nearest_car_id, nearest_car_v, nearest_car_frenet_y

    def get_target_point_lanechange_index(self, target_point_frenet_y, ego_lane_id):
       
        target_point_index = None
        KD_tree = self.globe_info.glob_road_info["frenet_kd_tree"]
        frenet_y = np.atleast_2d(target_point_frenet_y)
        distance, nearest_index  = KD_tree.query(frenet_y, self.kd_tree_return_nums)
        # print(type(nearest_index),len(nearest_index))
        nearest_index_list = []
        for i in range(nearest_index.shape[1]):
            if distance[0][i] == distance[0][0]:
                nearest_index_list.append(int(nearest_index[0][i]))
      
        for i in range(len(nearest_index_list)):
            # print("for")
            index = min(nearest_index_list[i],len(self.globe_info.glob_road_info["road_ids_array"])-1)
            target_point_lane_id = self.globe_info.glob_road_info["road_ids_array"][index]
            # print(target_point_lane_id)
            target_point_lane_id_index = self.globe_info.path.index(target_point_lane_id)
            # print(target_point_lane_id_index)
            ego_lane_id_index = self.globe_info.path.index(ego_lane_id)
            # print(ego_lane_id_index)
            if target_point_lane_id_index - ego_lane_id_index >= 1:
                target_point_index = min(nearest_index_list[i],len(self.globe_info.glob_road_info["road_ids_array"])-1)
                # print(target_point_index)
                break
        # print(target_point_index)
        return target_point_index

    def get_ego_neighbor_point_index(self, ego_frenet_y, ego_lane_id):
       
        target_point_index = None
        KD_tree = self.globe_info.glob_road_info["frenet_kd_tree"]
        frenet_y = np.atleast_2d(ego_frenet_y)
        distance, nearest_index  = KD_tree.query(frenet_y, self.kd_tree_return_nums)
        nearest_index_list = []
        for i in range(nearest_index.shape[1]):
            if distance[0][i] == distance[0][0]:
                nearest_index_list.append(int(nearest_index[0][i]))

        for i in range(len(nearest_index_list)):
            target_point_lane_id = self.globe_info.glob_road_info["road_ids_array"][nearest_index_list[i]]
            target_point_lane_id_index = self.globe_info.path.index(target_point_lane_id)
            ego_lane_id_index = self.globe_info.path.index(ego_lane_id)
            if target_point_lane_id_index - ego_lane_id_index == 1:
                target_point_index = nearest_index_list[i]
                break
                
        return target_point_index
                    
    def get_target_lanechange_paths(self, ego_nearest_index, target_point_index, ego_neighbor_index, target_point_neighbor_inex, ego_position):
        center_points = self.globe_info.glob_road_info["road_points_array"][int(ego_nearest_index):int(target_point_index+1)]
        left_boundary_points = self.globe_info.glob_road_info["left_boundary_array"][int(ego_nearest_index):int(target_point_index+1)]
        right_boundary_points = self.globe_info.glob_road_info["right_boundary_array"][int(ego_nearest_index):int(target_point_index+1)]
        center_points_interpolate = self.interpolate_curve(center_points)
        left_boundary_points_interpolate = self.interpolate_curve(left_boundary_points)
        right_boundary_points_interpolate = self.interpolate_curve(right_boundary_points)
        # print(left_boundary_points_interpolate,right_boundary_points_interpolate)
        ego_points = self.interpolate_ego(left_boundary_points_interpolate, right_boundary_points_interpolate, ego_position)
        # print("ego_path:",ego_points)
        neighbor_center_points = self.globe_info.glob_road_info["road_points_array"][int(ego_neighbor_index):int(target_point_neighbor_inex+1)]
        neighbor_left_boundary_points = self.globe_info.glob_road_info["left_boundary_array"][int(ego_neighbor_index):int(target_point_neighbor_inex+1)]
        neighbor_right_boundary_points = self.globe_info.glob_road_info["right_boundary_array"][int(ego_neighbor_index):int(target_point_neighbor_inex+1)]
        center_points_interpolate = self.interpolate_curve(neighbor_center_points)
        # print("neighbor center:",center_points_interpolate)
        left_boundary_points_interpolate = self.interpolate_curve(neighbor_left_boundary_points)
        right_boundary_points_interpolate = self.interpolate_curve(neighbor_right_boundary_points)
        left_points_interpolate = (center_points_interpolate+left_boundary_points_interpolate)/2
        right_points_interpolate = (center_points_interpolate+right_boundary_points_interpolate)/2
        target_paths = []
        center_path = self.interpolate_two_curve(ego_points, center_points_interpolate)
        # print("center path:",center_path)
        target_paths.append(center_path)
        left_path = self.interpolate_two_curve(ego_points,left_points_interpolate)
        target_paths.append(left_path)
        right_path = self.interpolate_two_curve(ego_points,right_points_interpolate)
        target_paths.append(right_path)
        return target_paths

    def change_lane(self,observation:Observation):
       
        
        nearest_car_id, nearest_car_v, nearest_car_frenet_y = self.get_front_car_change_lane(self.TP_in_front, self.TP_v_in_front, self.ego_lane_id, self.ego_frenet_y)
        # # print("get_front_car")
        nearest_car_b_id, nearest_car_b_v, nearest_car_b_frenet_y = self.get_behind_car_change_lane(self.TP_in_behind, self.TP_v_in_behind, self.ego_lane_id, self.ego_frenet_y)
        if nearest_car_b_id is not None and self.scene_type=='highway':
            self.max_speed = max(self.max_speed,nearest_car_b_v)
        if len(self.TP_in_behind) > 0 and self.scene_type!='highway':
            self.max_speed = max(self.max_speed,max(self.TP_v_in_behind)*1.1)
        if nearest_car_id == None:
            # nearest_car_v = -1
            # nearest_car_frenet_y = ego_frenet_y - 1
            nearest_car_v = self.nearest_car_v
            nearest_car_frenet_y = self.ego_frenet_y + self.nearest_car_dis

        target_path_muti_exv = []
        target_frenet_y_muti_exv = []
        target_v_muti_exv = []
        # for i in range(self.exv_num):
        #     self.idm.exv = self.max_speed / self.exv_num * (self.exv_num - i)
        for exv_proportion in self.exv_list:
            self.idm.exv = self.max_speed * exv_proportion
            target_frenet_y, target_v = self.simulation_idm(observation.ego_info.v, self.ego_frenet_y, nearest_car_v, nearest_car_frenet_y)
            # print(f"exc: {self.idm.exv}, target: {target_v}, near v: {nearest_car_v}, near y: {nearest_car_frenet_y}, ego y: {ego_frenet_y}")
            # print("simulation_idm")
            target_point_index = self.get_target_point_index(target_frenet_y, self.ego_nearest_index)
            # print("get_target_point_index")
            target_point_neighbor_index =self.get_target_point_lanechange_index(target_frenet_y, self.ego_lane_id)
            # print("get_target_point_lanechange_index")
            ego_neighbor_index = self.get_ego_neighbor_point_index(self.ego_frenet_y, self.ego_lane_id)
            if target_point_index > ego_neighbor_index:
                for i in range(self.ego_nearest_index,len(self.globe_info.glob_road_info["road_points_array"])):
                    if self.globe_info.glob_road_info["road_ids_array"][i] != self.ego_lane_id:
                        target_point_index = i-1
                        break
            
            target_paths = self.get_target_lanechange_paths(self.ego_nearest_index, target_point_index, ego_neighbor_index, target_point_neighbor_index, self.state[0][:2])
            # print("get_target_lanechange_paths")
            target_path_muti_exv.extend(target_paths)
            target_frenet_y_muti_exv.extend([target_frenet_y]*len(target_paths))
            target_v_muti_exv.extend([target_v]*len(target_paths))
        return target_path_muti_exv, target_frenet_y_muti_exv, target_v_muti_exv#, TP_in_front, TP_in_behind



    def is_changelane(self):
        
        # self.ego_lane_id, self.ego_frenet_y, self.ego_nearest_index =self.get_EV_frenet(self.state)
        path = self.globe_info.path
        index = path.index(self.ego_lane_id)
        if index < len(path) - 1:
            if path[index].split('.')[0] == path[index+1].split('.')[0] and path[index].split('.')[1] == path[index+1].split('.')[1]:
                return True
            else:
                return False
        return False
    
    def safe_merge(self):
        final_x = self.state[0][0] + self.no_safe_path_factor * cos(self.state[0][2]) 
        final_y = self.state[0][1] + self.no_safe_path_factor * sin(self.state[0][2]) 
        final_path = [[final_x,final_y] for _ in range(30)]
        final_path = np.array(final_path)
        final_v = 0
        return final_path,final_v