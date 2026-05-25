import numpy as np
from scipy.spatial import KDTree
from utils.observation import Observation

class GPInfo:
    def __init__(self, task_info, road_data):
        self.road_data = road_data
        self.task_info = task_info
        self.radius_factor = 1.5
        self.nearest_distance = 5.0
        self.safe_distance_car = 3.75
        self.targetPos = np.array(task_info['targetPos'])
        self.startPos = np.array(task_info['startPos'])
        self.allRoadKDTree = {}
        self.all_road_info = self.AllRoadKDTree(road_data)
        self.startLaneIDs = self.getRoadID(self.startPos)
        self.targetLaneIDs = self.getRoadID((self.targetPos[0]+self.targetPos[1])/2)
        self.road_structure = self.getRoadStruct(road_data)
        self.paths = []
        self.path = None
        self.glob_road_info = None
        self.targetPos_frenet_s, self.targetPos_index = -1.0, -1
        self.stopline_id = []
        self.stopline_position = []
        self.is_turn_right = []


    def glob_plan(self,observation: Observation, road_with_stopline):
        self.paths = self.find_paths(self.startLaneIDs, self.targetLaneIDs, self.road_structure)
        self.path = self.get_feasible_path(self.paths, observation)
        self.glob_road_info = self.globRoadKDTree(self.road_data, self.path)
        self.targetPos_frenet_s, self.targetPos_index = self.get_target_frenet_s((self.targetPos[0]+self.targetPos[1])/2)
        self.stopline_id = self.get_stop_line_id(road_with_stopline)
        self.stopline_position = self.get_stop_line_position(self.stopline_id)
        self.is_turn_right = self.check_right_turn(self.stopline_id)

    def AllRoadKDTree(self, road_data):
        
        road_points = []  # 存储所有路点的数组
        road_ids = []  # 存储所有路点对应的道路 ID 的数组

        for discrete_lane in road_data.discretelanes:
            lane_id = discrete_lane.lane_id
            center_vertices = discrete_lane.center_vertices

            # 将当前道路的路点添加到路点数组中
            road_points.extend(center_vertices)

            # 将当前道路的道路 ID 添加到道路 ID 数组中（长度与路点数组相同）
            road_ids.extend([lane_id] * len(center_vertices))

        # 将路点数组和道路 ID 数组转换为 NumPy 数组
        road_points_array = np.array(road_points)
        road_ids_array = np.array(road_ids)

        # 使用所有路点数组构建 KD 树
        all_road_kd_tree = KDTree(road_points_array)
        all_road_info = {
            "road_points_array": road_points_array,
            "road_ids_array": road_ids_array,
            "road_kd_tree": all_road_kd_tree
        }

        return all_road_info
    
    def getRoadID(self, position):
        
        
        # distances, nearest_indices = self.all_road_info["road_kd_tree"].query(position, self.radius)
        
        distance, nearest_index  = self.all_road_info["road_kd_tree"].query(position)
        indices = self.all_road_info["road_kd_tree"].query_ball_point(position, r=distance*self.radius_factor)
        # nearest_road_id = self.all_road_info["road_ids_array"][nearest_index]
        nearest_road_ids = self.all_road_info["road_ids_array"][indices]
        nearest_road_ids = np.unique(nearest_road_ids)
        return nearest_road_ids
    
    def get_adjacent_lane(self, current_lane_str):
        
        adjacent_lane = []
        parts = current_lane_str.split('.')
        lane_index = int(parts[2])
        adjacent_lane_index1 = lane_index - 1  # 获取相邻车道的索引
        adjacent_lane_str1 = f"{parts[0]}.{parts[1]}.{adjacent_lane_index1}.{parts[3]}"
        adjacent_lane_index2 = lane_index + 1  # 获取相邻车道的索引
        adjacent_lane_str2 = f"{parts[0]}.{parts[1]}.{adjacent_lane_index2}.{parts[3]}"
        adjacent_lane.append(adjacent_lane_str1)
        adjacent_lane.append(adjacent_lane_str2)
        return adjacent_lane

    def getRoadStruct(self,road_data):
        
        road_structure = {}
        for discrete_lane in road_data.discretelanes:
            road_structure[discrete_lane.lane_id] = discrete_lane.successor
        for ID in road_structure:
            adjacent_lane = self.get_adjacent_lane(ID)
            if adjacent_lane[0] in road_structure:
                road_structure[ID].append(adjacent_lane[0])
            if adjacent_lane[1] in road_structure:
                road_structure[ID].append(adjacent_lane[1])
        return road_structure

    
    def find_path(self, start_road_id, end_road_id, road_structure):
       
        visited = set()  # 用于记录已经访问过的道路，避免重复访问
        path = []
        visited.add(start_road_id)
        # 定义递归函数来搜索路径
        def dfs(current_road_id):
            # 将当前道路添加到路径中
            path.append(current_road_id)
            # print("path:",path)
            # 如果当前道路就是终点路，则返回路径
            if current_road_id == end_road_id:
                return True

            # 遍历当前道路的后继道路
            for next_road_id in road_structure.get(current_road_id, []):
                # 如果后继道路没有被访问过，则递归地搜索路径
                if next_road_id not in visited:
                    visited.add(next_road_id)
                    if dfs(next_road_id):
                        return True

            # 如果无法找到可行路径，则将当前道路从路径中移除，返回上一级继续搜索
            path.pop()
            return False
        # 开始搜索路径
        if dfs(start_road_id):
            return path
        else:
            return None
    
    def find_paths(self, start_road_ids, end_road_ids, road_structure):
        paths = []
        for end_road_id in end_road_ids:
            for start_road_id in start_road_ids:
                path = self.find_path(start_road_id, end_road_id, road_structure)
                if path:
                    paths.append(path)
        return paths

    def get_nearest_TP(self, observation: Observation):
        nearest_distance = float('inf')
        nearest_TP_id = None
        nearest_TP_type = None
        ego_x = observation.ego_info.x
        ego_y = observation.ego_info.y
        for TP_type,TP_dict in observation.object_info.items():
            for id,object in TP_dict.items():
                x = object.x
                y = object.y
                distance = ((ego_x-x)**2+(ego_y-y)**2)**0.5
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_TP_type = TP_type
                    nearest_TP_id = id
        return nearest_TP_type, nearest_TP_id, nearest_distance


    def get_feasible_path(self, paths, observation: Observation):
        nearest_TP_type, nearest_TP_id, nearest_distance = self.get_nearest_TP(observation)
        final_path = []
        if nearest_distance > self.nearest_distance:
            final_path = paths[-1]
        else:
            x = observation.object_info[nearest_TP_type][nearest_TP_id].x
            y = observation.object_info[nearest_TP_type][nearest_TP_id].y
            position = np.array([x,y])
            distance, nearest_index  = self.all_road_info["road_kd_tree"].query(position)
            nearest_road_id = self.all_road_info["road_ids_array"][nearest_index]
            for i in range(len(paths)):
                if nearest_road_id != paths[len(paths)-1-i][0]:
                    final_path = paths[len(paths)-1-i]
                    break
            # for path in paths:
            #     if nearest_road_id != path[0]:
            #         final_path = path
            #         break
            if len(final_path) == 0:
                final_path = paths[-1]
        return final_path



    def globRoadKDTree(self, road_data, path):
        
        if path:
            road_points = [np.ndarray(shape=(1,))] * len(path)
            left_boundary = [np.ndarray(shape=(1,))] * len(path)
            right_boundary = [np.ndarray(shape=(1,))] * len(path)
            road_ids = [[] for _ in range(len(path))]
            for discrete_lane in road_data.discretelanes:
                lane_id = discrete_lane.lane_id
                if lane_id in path:
                    index = path.index(lane_id)
                    center_vertices = discrete_lane.center_vertices
                    left_vertices = discrete_lane.left_vertices
                    right_vertices = discrete_lane.right_vertices
                    # 将当前道路的路点添加到路点数组中
                    road_points[index] = center_vertices
                    left_boundary[index] = left_vertices
                    right_boundary[index] = right_vertices
                    road_ids[index] = [lane_id] * len(center_vertices)
            frenet_s = []
            for i in range(len(road_points)):
                road_points_s = [0.0]
                distances = np.linalg.norm(road_points[i][1:] - road_points[i][:-1], axis=1)
                for j in range(len(distances)):
                    road_points_s.append(float(distances[j]+road_points_s[-1]))
                frenet_s.append(np.array(road_points_s))
                # print("***************************")
                # print(len(road_points[i]),len(frenet_s[i]))
            for i in range(1,len(path)):
                if path[i].split('.')[0] == path[i-1].split('.')[0] and path[i].split('.')[1] == path[i-1].split('.')[1]:
                    frenet_s[i] = frenet_s[i-1]
                else:
                    frenet_s[i] = frenet_s[i] + frenet_s[i-1][-1]
            
            road_points_array = np.vstack(road_points)
            left_boundary_array = np.vstack(left_boundary)
            right_boundary_array = np.vstack(right_boundary)
            road_ids_flatten = []
            for id in road_ids:
                road_ids_flatten.extend(id)
            road_ids_array = np.array(road_ids_flatten)
            frenet_s_array = np.concatenate(frenet_s)
            # print(len(frenet_s_array))
            frenet_s_array_2d = frenet_s_array.reshape(-1,1)
            globle_road_kd_tree = KDTree(road_points_array)
            # print(type(frenet_s_array),frenet_s_array.shape,frenet_s_array)
            global_frenet_kd_tree = KDTree(frenet_s_array_2d)
            globle_road_info = {
                "road_points_array": road_points_array,
                "left_boundary_array": left_boundary_array,
                "right_boundary_array": right_boundary_array,
                "road_ids_array": road_ids_array,
                "frenet_s_array": frenet_s_array,
                "road_kd_tree": globle_road_kd_tree,
                "frenet_kd_tree": global_frenet_kd_tree
            }
            return globle_road_info
        else:
            return None

    def get_target_frenet_s(self,target_point):
        distance, nearest_index  = self.glob_road_info["road_kd_tree"].query(target_point)
        target_point_frenet_s = self.glob_road_info["frenet_s_array"][nearest_index]
        return target_point_frenet_s, nearest_index
    
    def get_stop_line_id(self, road_with_stopline):
        stop_line_road_id = []
        for stopline in road_with_stopline:
            for i in range(1,len(self.path)+1):
                road_id = int(self.path[-i].split('.')[0])
                if road_id == stopline:
                    stop_line_road_id.append(self.path[-i])
                    break
        return stop_line_road_id
    
    def get_stop_line_position(self, stop_line_road_id):
        stop_line_road_position = []
        for id in stop_line_road_id:
            indices = np.where(self.glob_road_info["road_ids_array"]==id)
            row_indices = indices[0]
            last_row_index = int(np.max(row_indices))
            A = self.glob_road_info["road_points_array"][last_row_index - 1]
            B = self.glob_road_info["road_points_array"][last_row_index]
            AB = B - A 
            length = np.linalg.norm(AB)
            unit_vector = AB / length
            C = B + unit_vector * self.safe_distance_car
            stop_line_road_position.append(B)
        return stop_line_road_position

    def check_right_turn(self, stop_line_road_id):
        is_turn_right = []
        for id in stop_line_road_id: 
            index = self.path.index(id)
            if index == len(self.path) - 1:
                is_turn_right.append(False)
            else:
                next_id = self.path[index+1]
                indices = np.where(self.glob_road_info["road_ids_array"]==next_id)
                row_indices = indices[0]
                first_row_index = int(row_indices[0])
                last_row_index = int(np.max(row_indices))
                start_line = self.glob_road_info["road_points_array"][first_row_index+1]-self.glob_road_info["road_points_array"][first_row_index]
                end_line = self.glob_road_info["road_points_array"][last_row_index]-self.glob_road_info["road_points_array"][last_row_index-1]
                start_rad = np.arctan2(start_line[1], start_line[0])
                
                if start_rad < 0:
                    start_rad += 2*np.pi
                end_rad = np.arctan2(end_line[1], end_line[0])
                if end_rad < 0:
                    end_rad += 2*np.pi
                yaw_diff = (end_rad - start_rad) % (2*np.pi)
                if 7/6*np.pi < yaw_diff < 11/6*np.pi:
                    is_turn_right.append(True)
                else:
                    is_turn_right.append(False)
        return is_turn_right
