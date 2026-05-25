import json
import os
import numpy as np
import math
from shapely.geometry import LineString
from shapely.geometry import Polygon

json_flie = r'D:/opendrive_tj/tj_road_dict.json'

# 射线法实现判断一个点是否在一个多边形（路段）内
def in_or_out(point, rangelist):
    """
        判断车是否在某一车道内
    Args:
        point: 车的中心点坐标
        rangelist: 路段顺时针的定点

    Returns:
        True: 在多边形内
        False: 不在多边形内
    """
    # 排除不在范围内的点
    r_x = []
    r_y = []
    for p in rangelist:
        r_x.append(p[0])
        r_y.append(p[1])
    max_x = max(r_x)
    max_y = max(r_y)
    min_x = min(r_x)
    min_y = min(r_y)
    if (point[0] > max_x or point[0] < min_x or point[1] > max_y or point[1] < min_y):
        return False
    # 构建各边
    s_list = []
    for i in range(0, len(rangelist)):
        s_list_x = []
        if i != len(rangelist)-1:
            s_list_x.append(rangelist[i])
            s_list_x.append(rangelist[i+1])
            s_list.append(s_list_x)
        else:
            s_list_x.append(rangelist[i])
            s_list_x.append(rangelist[0])
            s_list.append(s_list_x)
    # 判断射线与各边的交点个数以及交点是否在边
    common = 0
    for i in range(0, len(s_list)):
        if s_list[i][1][0] - s_list[i][0][0] == 0:
            if point[0] == s_list[i][0][0]:
                common += 1
        else:
            y = ((s_list[i][1][1] - s_list[i][0][1]) / (s_list[i][1][0]-s_list[i][0][0])) * (point[0]-s_list[i][0][0]) + s_list[i][0][1]
            # 考虑交点在边的情况
            if y == point[1]:
                p = True
            elif y > point[1] and y <= max(s_list[i][0][1], s_list[i][1][1]):
                common += 1
    # 判断in or out
    if common % 2 != 0 or p:
        return True
    else:
        return False

# 计算指标
class cal_ego_param:
    def __init__(self, opendrive_dict, ego_info, si_car_info, min_dis_id, angle_threshold, length, width):
        self.opendrive_dict = opendrive_dict  # opendrive_dict: dict 对应的opendrive地图在指标计算中的数据
        self.ego_info = ego_info  # ego_info: dict  累计的ego车的所有数据 list
        self.si_car_info = si_car_info  # other_car_info: 累计的仿真车的所有数据 list
        self.min_dis_id = min_dis_id  # min_dis_id： 当前帧离ego车20m以内的车
        self.lateral_spacing_time = 0  # 横向间距违规时间
        self.four_wheels_on_road_time = 0  # 四轮在路违规时间
        self.car_reverse_count = 0  # 逆向行驶违规次数
        self.compacted_line_travel_count = 0  # 压实线行驶违规次数
        self.driving_in_designated_lane_time = 0  # 未按规定车道行驶违规时间
        self.stop_at_the_stop_line_count = 0  # 压停止线停车次数
        self.driving_in_restricted_area_flag = False  # 在禁行区禁行标志位
        self.angle_variation_time = 0  # 角度变化违规时间
        self.angle_threshold = angle_threshold  # 设置的角度阈值,用于计算角度变化违规时间
        self.drivable_space_type = ['driving', 'onRamp', 'entry', 'biking']  # 可行驶区域的类型，后续如再有类似匝道的类型，可以添加进去
        # 只有同时满足两帧数据以上才能计算
        # if len(self.si_car_info) >= 1 and len(self.ego_info) >= 2:
        if len(self.ego_info) >= 2:
            self.cal_flag = True  # 计算误差的标志位
        else:
            self.cal_flag = False
        self.ego_center_station = [-1000, -1000, None, -1000]  # ego车所在的路段 ego车所在的车道 ego车所在车道的类型 ego车所在具体车道位置
        self.ego_corner_station = [[-1000, -1000, None, -1000], [-1000, -1000, None, -1000],
                                   [-1000, -1000, None, -1000], [-1000, -1000, None, -1000]]  # ego车所有角点所在的路段 车道 车道的类型 具体车道位置
        self.ego_corner_station_flag = False  # 用于判断ego车四个角点的数据在有数据的基础上，是否在地图中
        self.length = 1.5  # ego车长度
        self.width = 1.5  # ego车宽度
        self.ego_bound = []  # ego车的四个角点
        # 获取所有的交叉口数据
        self.all_junction_info = self.opendrive_dict['all_junction_info']
        self.connectingRoad_info_list = []
        for i in range(len(self.all_junction_info)):
            every_junction_info_dict = self.all_junction_info[i]
            for keys, values in every_junction_info_dict.items():
                if keys == 'connection_info':
                    every_connectingRoad = every_junction_info_dict[keys]
                    for j in range(len(every_connectingRoad)):
                        every_connectingRoad_id = every_connectingRoad[j][1]
                        self.connectingRoad_info_list.append(every_connectingRoad_id)


    # 计算横向间距违规时间
    def cal_lateral_spacing_error(self):
        # 通过获取与ego车距离20m（可以调整）范围内的其他车的id得到对应的坐标值
        # 与ego车的x轴数据作差比较获取最后的结果
        single_lateral_spacing_time = 0
        if self.cal_flag:
            # last_si_info = self.si_car_info[-2]
            si_info = self.si_car_info
            last_ego_info = self.ego_info[-2]
            ego_info = self.ego_info[-1]
            for i in range(len(self.min_dis_id)):
                for svI in si_info:
                    if self.min_dis_id[i] == svI['id']:
                        if abs(svI['x'] - ego_info['x']) < 1:
                            single_lateral_spacing_time = ego_info['simutime'] - last_ego_info['simutime']
        self.lateral_spacing_time += single_lateral_spacing_time
        return self.lateral_spacing_time

    # 计算角度变化违规时间
    def angle_variation_time(self):
        # 获取ego前后两帧的数据，得到对应的角度数据作差，与设置好的阈值进行比较
        # 这里需要注意的是传过来的ego车的角度要变成角度制
        single_angle_variation_time = 0
        last_ego_info = self.ego_info[-2]
        ego_info = self.ego_info[-1]
        if self.cal_flag:
            if abs(ego_info['angle'] - last_ego_info['angle']) > self.angle_threshold:
                single_angle_variation_time = ego_info['simutime'] - last_ego_info['simutime']
        self.angle_variation_time += single_angle_variation_time
        return self.angle_variation_time

    # 将y轴正轴顺时钟方向的角转为x正轴逆时针方向的角
    def convert_clockwise_to_counterclockwise(self, angle) -> float:
        counterclockwise_angle = (90 - angle) % 360
        return counterclockwise_angle

    # 计算ego车的四个坐标点数据
    def calculate_rectangle_vertices(self):
        # 获取车的四个角点位置
        ego_info = self.ego_info[-1]
        ego_x = ego_info['x']
        ego_y = ego_info['y']  # tessng传过来的数据与opendrive相差一个负值
        angle = ego_info['angle']
        # 将角度值转为弧度制
        # 转为x轴正轴逆时针方向的角
        angle = self.convert_clockwise_to_counterclockwise(angle)
        angle = math.radians(angle)
        dx = self.length / 2
        dy = self.width / 2

        # 计算矩形角点相对于中心点的坐标偏移量
        vertex_offsets = [
            (-dx * math.cos(angle) - dy * math.sin(angle), -dx * math.sin(angle) + dy * math.cos(angle)),  # 左上角
            (-dx * math.cos(angle) + dy * math.sin(angle), -dx * math.sin(angle) - dy * math.cos(angle)),  # 左下角
            (dx * math.cos(angle) + dy * math.sin(angle), dx * math.sin(angle) - dy * math.cos(angle)),  # 右上角
            (dx * math.cos(angle) - dy * math.sin(angle), dx * math.sin(angle) + dy * math.cos(angle))  # 右下角
        ]

        # 计算矩形顶点的绝对坐标
        # 矩形的四个角点坐标，依次是左上、左下、右下、右上
        ego_bound = [(ego_x + offset[0], ego_y + offset[1]) for offset in vertex_offsets]
        return ego_bound

    # 计算ego车在当前哪个位置用于其他指标的计算
    def cal_ego_center_in_road_lane_id(self):
        # 通过射线法，获取ego车中心点的坐标具体的位置，具体的道路，具体的车道，具体的路段
        road_map = self.opendrive_dict['all_road_dict']
        ego_info = self.ego_info[-1]
        ego_x = ego_info['x']
        ego_y = ego_info['y']  # tessng传过来的数据与opendrive相差一个负值
        car_center = [ego_x, ego_y]
        for i in range(len(road_map)):
            center_vertices = road_map[i]['center_vertices']
            left_vertices = road_map[i]['left_vertices']
            right_vertices = road_map[i]['right_vertices']
            for j in range(len(center_vertices)):
                if j + 1 < len(center_vertices):
                    every_polygon_xy = [left_vertices[j][0:2], right_vertices[j][0:2], right_vertices[j + 1][0:2], left_vertices[j + 1][0:2]]
                    narrow_judgment_result = in_or_out(car_center, every_polygon_xy)
                    if narrow_judgment_result:
                        self.ego_center_station = [road_map[i]['road_id'], road_map[i]['lane_id'], road_map[i]['type'], j]
                        pass
        return self.ego_center_station

    # 获取ego车四个角点所在的位置
    def cal_ego_corner_in_road_lane_id(self):
        # 通过射线法，获取ego车四个角点的坐标具体的位置，具体的道路，具体的车道，具体的路段
        road_map = self.opendrive_dict['all_road_dict']
        self.ego_bound = self.calculate_rectangle_vertices()
        for i in range(len(road_map)):
            center_vertices = road_map[i]['center_vertices']
            left_vertices = road_map[i]['left_vertices']
            right_vertices = road_map[i]['right_vertices']
            for j in range(len(center_vertices)):
                if j + 1 < len(center_vertices):
                    every_polygon_xy = [left_vertices[j][0:2], right_vertices[j][0:2],
                                        right_vertices[j + 1][0:2], left_vertices[j + 1][0:2]]
                    # 后期可以根据中心点的坐标获取的道路信息获取一遍获取周围的连接段数据进行计算，减少耗时
                    if len(self.ego_bound) != 0:
                        # 矩形的四个角点坐标，依次是左上、左下、右下、右上
                        narrow_judgment_result_0 = in_or_out(self.ego_bound[0], every_polygon_xy)
                        if narrow_judgment_result_0:
                            self.ego_corner_station[0] = [road_map[i]['road_id'], road_map[i]['lane_id'], road_map[i]['type'], j]
                            pass
                        narrow_judgment_result_1 = in_or_out(self.ego_bound[1], every_polygon_xy)
                        if narrow_judgment_result_1:
                            self.ego_corner_station[1] = [road_map[i]['road_id'], road_map[i]['lane_id'], road_map[i]['type'], j]
                            pass
                        narrow_judgment_result_2 = in_or_out(self.ego_bound[2], every_polygon_xy)
                        if narrow_judgment_result_2:
                            self.ego_corner_station[2] = [road_map[i]['road_id'], road_map[i]['lane_id'], road_map[i]['type'], j]
                            pass
                        narrow_judgment_result_3 = in_or_out(self.ego_bound[3], every_polygon_xy)
                        if narrow_judgment_result_3:
                            self.ego_corner_station[3] = [road_map[i]['road_id'], road_map[i]['lane_id'], road_map[i]['type'], j]
                            pass
        if self.ego_corner_station != [[-1000, -1000, None, -1000], [-1000, -1000, None, -1000],
                                       [-1000, -1000, None, -1000], [-1000, -1000, None, -1000]]:
            self.ego_corner_station_flag = True
        return self.ego_corner_station

    # 计算是否在交叉口范围内
    def cal_is_in_junction(self):
        if self.cal_flag:
            self.ego_center_station = self.cal_ego_center_in_road_lane_id()
            # print(self.ego_center_station)
            if self.ego_center_station != [-1000, -1000, None, -1000]:
                if self.ego_center_station[0] in self.connectingRoad_info_list:
                    return True
                else:
                    return False

    # 计算四轮在路违规时间
    def cal_four_wheels_on_road_time(self):
        # 通过ego车四个角点的状态，判断是否在道路上
        # 不在路上的状态有两种：角点的状态为None，或者不为driving
        single_four_wheels_on_road_time = 0
        if self.cal_flag:
            # 需要额外判断一次是否在交叉口范围内，如果在交叉口范围内，则豁免驶出车道的情况
            if self.cal_is_in_junction():
                return 0
            last_ego_info = self.ego_info[-2]
            ego_info = self.ego_info[-1]
            self.ego_corner_station = self.cal_ego_corner_in_road_lane_id()
            outWheelNum = 0
            # print(self.ego_corner_station)
            if self.ego_corner_station == [[-1000, -1000, None, -1000], [-1000, -1000, None, -1000],
                                           [-1000, -1000, None, -1000], [-1000, -1000, None, -1000]]:
                # print( self.ego_corner_station_flag)
                # if self.ego_corner_station_flag == True:
                single_four_wheels_on_road_time = ego_info['simutime'] - last_ego_info['simutime']
            else:
                for i in range(len(self.ego_corner_station)):
                    # if self.ego_corner_station[i] == [-1000, -1000, None, -1000] or self.ego_corner_station[i][2] != 'driving' or self.ego_corner_station[i][2] != 'onramp':
                    if self.ego_corner_station[i] == [-1000, -1000, None, -1000] or self.ego_corner_station[i][2] not in self.drivable_space_type:
                        outWheelNum += 1
                # 至少有3个轮子都在外面了才算是驶出车道
                if outWheelNum > 2:
                    single_four_wheels_on_road_time = ego_info['simutime'] - last_ego_info['simutime']
            if single_four_wheels_on_road_time:
                # print("驶出行车道", self.ego_corner_station)
                pass
        self.four_wheels_on_road_time += single_four_wheels_on_road_time
        # print(self.four_wheels_on_road_time)
        return self.four_wheels_on_road_time

    # onsite弧度制转换为tess角度制的函数
    # 首先第一步是要把弧度转换为角度
    # 第二步把转换后的角度变成正北的
    def angleToArc(self, value):
        angle_clockwise = value * (math.pi / 180)
        # 将正东为0逆时针增大的角度转换为以正北为0顺时针增大的角度
        # 将弧度减去π/2，以调整起始点为正东
        angle_counterclockwise = angle_clockwise - math.pi / 2
        # 取模2π，确保角度在0到2π之间
        angle_counterclockwise = angle_counterclockwise % (2 * math.pi)
        return angle_counterclockwise

    # 首先第一步是要把弧度转换为角度
    # 第二步把转换后的角度变成正北的
    def arcToAngle(self, value):
        angle_east = value * (180 / math.pi)
        # 将正东为0逆时针增大的角度转换为以正北为0顺时针增大的角度
        angle_north = (90 - angle_east) % 360
        return angle_north

    # 计算逆向行驶违规次数
    def cal_car_reverse_count(self):
        # 获取ego车中心点的状态后查找离线文件得到对应的车道的角度
        # 对比ego车的角度与车道的角度，是否超过阈值
        # 这里需要注意两者的角度是否都是同一种单位
        single_car_reverse_count = 0
        if self.cal_flag:
            # 需要额外判断一次是否在交叉口范围内，如果在交叉口范围内，则豁免驶出车道的情况
            if self.cal_is_in_junction():
                return 0
            road_map = self.opendrive_dict['all_road_dict']
            last_ego_info = self.ego_info[-2]
            ego_info = self.ego_info[-1]
            # angle = self.angleToArc(ego_info['angle'])
            angle = ego_info['angle']
            self.ego_center_station = self.cal_ego_center_in_road_lane_id()
            # print(self.ego_center_station)
            if self.ego_center_station != [-1000, -1000, None, -1000]:
                for i in range(len(road_map)):
                    road_id = road_map[i]["road_id"]
                    road_angle = road_map[i]["lane_angle"]
                    if len(road_angle) > self.ego_center_station[3]:
                        road_angle = road_angle[self.ego_center_station[3]]
                        # print(road_angle)
                        # 这如果是按照同济测试场就需要设置反向，平时也一直是负的所以也没事
                        if self.ego_center_station[1] > 0:
                            road_angle = (road_angle + np.pi) % 2 * np.pi
                            # print(road_angle)
                        road_angle = self.arcToAngle(road_angle)
                        if self.ego_center_station[0] == road_id:
                            # print("当前路段角度",road_angle, "当前车辆角度",angle)
                            # # road_angle = road_angle / np.pi * 180
                            # angle_thresh = (abs(angle - road_angle) + 2 * np.pi) % (2 * np.pi)
                            # angle_thresh = abs(angle - road_angle)
                            # if angle_thresh > np.pi/2:
                            # print(angle, road_angle, angle_thresh)
                            if self.are_angles_close(road_angle, angle) > 90:
                                # print("驶入对向车道")
                                single_car_reverse_count = 1
        self.car_reverse_count += single_car_reverse_count
        return self.car_reverse_count

    # 判断两个角度是否相近
    def are_angles_close(self, anglePos, angleCar):
        if abs(angleCar - anglePos) > 180:
            return 360 - abs(angleCar - anglePos)
        else:
            return abs(angleCar - anglePos)

    # 计算压实线行驶违规次数
    def cal_compacted_line_travel_count(self):
        # 第一步获取ego中心点坐标的状态（用于获取车在哪个道路和哪个车道及对应的路段某一块多边形内）和四个角点的数据（用于与多边形的左右两侧比较是否压实线）
        # 第二步根据中心点所在的车道，按照opendrive车道从左到右（从大到小）获取所在车道左右两侧线型情况，如是实线或者双实线则与四个角点进行比较获取最终的结果
        if self.cal_flag:
            # 需要额外判断一次是否在交叉口范围内，如果在交叉口范围内，则豁免驶出车道的情况
            if self.cal_is_in_junction():
                return 0
            single_compacted_line_travel_count = 0
            road_map = self.opendrive_dict['all_road_dict']
            self.ego_center_station = self.cal_ego_center_in_road_lane_id()
            self.ego_bound = self.calculate_rectangle_vertices()
            shapely_poly = Polygon(self.ego_bound)
            self.ego_corner_station = self.cal_ego_corner_in_road_lane_id()
            road_mark_info = self.opendrive_dict['all_road_mark']
            for i in range(len(road_map)):
                if road_map[i]['road_id'] == self.ego_center_station[0] and road_map[i]['lane_id'] == \
                        self.ego_center_station[1]:
                    left_vertices_1 = road_map[i]['left_vertices'][self.ego_center_station[3]][0:2]
                    left_vertices_2 = road_map[i]['left_vertices'][self.ego_center_station[3] + 1][0:2]
                    left_points = [left_vertices_1, left_vertices_2]
                    right_vertices_1 = road_map[i]['right_vertices'][self.ego_center_station[3]][0:2]
                    right_vertices_2 = road_map[i]['right_vertices'][self.ego_center_station[3] + 1][0:2]
                    right_points = [right_vertices_1, right_vertices_2]
                    for j in range(len(road_mark_info)):
                        if self.ego_center_station[0] == road_mark_info[j]["road_id"]:
                            road_mark_list = road_mark_info[j]['road_mark_list']
                            if self.ego_center_station[1] > 0:
                                for m in range(len(road_mark_list['left'])):
                                    # 处理参考线左侧车道的左侧线型情况
                                    if self.ego_center_station[1] == road_mark_list['left'][m][2]:
                                        if road_mark_list['left'][m][3] == 'solid' or road_mark_list['left'][m][3] == 'solid solid':
                                            shapely_left_line = LineString(left_points)
                                            intersection_left_line = list(
                                                shapely_poly.intersection(shapely_left_line).coords)
                                            if intersection_left_line:
                                                single_compacted_line_travel_count = 1
                                    # 处理参考线左侧车道的右侧线型情况
                                    if (self.ego_center_station[1] - 1) == 0:
                                        if road_mark_list['center']:
                                            if road_mark_list['center'][0][3] == 'solid' or road_mark_list['center'][0][3] == 'solid solid':
                                                shapely_right_line = LineString(right_points)
                                                intersection_right_line = list(
                                                    shapely_poly.intersection(shapely_right_line).coords)
                                                if intersection_right_line:
                                                    single_compacted_line_travel_count = 1
                                    else:
                                        if (self.ego_center_station[1] - 1) == road_mark_list['left'][m][2]:
                                            if road_mark_list['left'][m][3] == 'solid' or road_mark_list['left'][m][3] == 'solid solid':
                                                shapely_right_line = LineString(right_points)
                                                intersection_right_line = list(
                                                    shapely_poly.intersection(shapely_right_line).coords)
                                                if intersection_right_line:
                                                    single_compacted_line_travel_count = 1
                            else:
                                for m in range(len(road_mark_list['right'])):
                                    # 处理参考线右侧车道的右侧线型情况
                                    if self.ego_center_station[1] == road_mark_list['right'][m][2]:
                                        if road_mark_list['right'][m][3] == 'solid' or road_mark_list['right'][m][3] == 'solid solid':
                                            shapely_right_line = LineString(right_points)
                                            intersection_right_line = list(
                                                shapely_poly.intersection(shapely_right_line).coords)
                                            if intersection_right_line:
                                                single_compacted_line_travel_count = 1
                                    # 处理参考线右侧车道的左侧线型情况
                                    if (self.ego_center_station[1] + 1) == 0:
                                        if road_mark_list['center']:
                                            if road_mark_list['center'][-1][3] == 'solid' or \
                                                    road_mark_list['center'][-1][3] == 'solid solid':
                                                shapely_left_line = LineString(left_points)
                                                intersection_left_line = list(
                                                    shapely_poly.intersection(shapely_left_line).coords)
                                                if intersection_left_line:
                                                    single_compacted_line_travel_count = 1
                                    else:
                                        if (self.ego_center_station[1] - 1) == road_mark_list['right'][m][2]:
                                            if road_mark_list['right'][m][3] == 'solid' or \
                                                    road_mark_list['right'][m][3] == 'solid solid':
                                                shapely_left_line = LineString(left_points)
                                                intersection_left_line = list(
                                                    shapely_poly.intersection(shapely_left_line).coords)
                                                if intersection_left_line:
                                                    single_compacted_line_travel_count = 1
        self.compacted_line_travel_count += single_compacted_line_travel_count
        return self.compacted_line_travel_count

    # 计算未按规定车道行驶违规时间
    def cal_driving_in_designated_lane_time(self):
        # 找到ego车中心点的数据，查找离线数据中对应车段的限制数据，如果不是None，则说明没有车道限制
        single_driving_in_designated_lane_time = 0
        if self.cal_flag:
            all_line_type = self.opendrive_dict['all_lane_access']
            last_ego_info = self.ego_info[-2]
            ego_info = self.ego_info[-1]
            self.ego_center_station = self.cal_ego_center_in_road_lane_id()
            if self.ego_center_station != [-1000, -1000, None, -1000]:
                for i in range(len(all_line_type)):
                    every_line_type = all_line_type[i]
                    road_id = every_line_type[0][0][0]

                    if road_id == self.ego_center_station[0]:
                        if self.ego_center_station[1] > 0:
                            road_info = every_line_type[0]
                        else:
                            road_info = every_line_type[1]
                        for m in range(len(road_info)):
                            lane_id = road_info[m][1]
                            if lane_id == self.ego_center_station[1]:
                                lane_access = road_info[m][2]
                                if lane_access != "None":
                                    single_driving_in_designated_lane_time = ego_info['simutime'] - last_ego_info['simutime']
        self.driving_in_designated_lane_time += single_driving_in_designated_lane_time
        return self.driving_in_designated_lane_time

    # 计算压停止线停车次数
    def cal_stop_at_the_stop_line_count(self):
        # 找到离线数据中所有的停止线数据
        # 与ego车的四个角点比较有交点，则说明压了停止线
        if self.cal_flag:
            single_stop_at_the_stop_line_count = 0
            ego_info = self.ego_info[-1]
            if ego_info['speed'] < 1e-3:
                stop_line = self.opendrive_dict['all_stop_line']
                self.ego_bound = self.calculate_rectangle_vertices()
                shapely_poly = Polygon(self.ego_bound)
                for i in range(len(stop_line)):
                    every_stop_line = stop_line[i]
                    length = len(every_stop_line)
                    for j in range(0, length, 2):
                        every_line_points = every_stop_line[j: j + 2]
                        shapely_line = LineString(every_line_points)
                        intersection_line = list(shapely_poly.intersection(shapely_line).coords)
                        if intersection_line:
                            single_stop_at_the_stop_line_count = 1
            self.stop_at_the_stop_line_count += single_stop_at_the_stop_line_count
        return self.stop_at_the_stop_line_count

    # 计算在禁行区禁行标志位
    def cal_driving_in_restricted_area_flag(self):
        # 获取ego车中心点与四个角点的数据，获取其所在的车道上车道类型
        # 如果不是driving类型，则说明在禁行区
        if self.cal_flag:
            ego_info = self.ego_info[-1]
            if ego_info['speed'] < 1e-3:
                self.ego_corner_station = self.cal_ego_corner_in_road_lane_id()
                self.ego_center_station = self.cal_ego_center_in_road_lane_id()
                # if self.ego_center_station[2] == 'driving' or self.ego_center_station[2] == 'onramp':
                if self.ego_center_station[2] in self.drivable_space_type:
                    for i in range(len(self.ego_corner_station)):
                        # if self.ego_corner_station[i][2] == 'driving' or self.ego_center_station[2] == 'onramp':
                        if self.ego_corner_station[i][2] in self.drivable_space_type:
                            self.driving_in_restricted_area_flag = False
                        else:
                            self.driving_in_restricted_area_flag = True
                else:
                    self.driving_in_restricted_area_flag = True
        return self.driving_in_restricted_area_flag
