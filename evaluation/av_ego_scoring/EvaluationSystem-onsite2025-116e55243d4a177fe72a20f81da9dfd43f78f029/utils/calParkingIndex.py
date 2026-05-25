import math
import os
import time

import pandas as pd
import numpy as np
from utils.others import makeZeroDataByParkingError, makePerfectData, makeZeroData

# def load_trajectory(file_path):
#     df = pd.read_csv(file_path, header=0)
#     df.columns.values[0] = 'timestamp'  # 将第一列的名称设为 'timestamp'
#     trajectory = []
#     for index, row in df.iterrows():
#         trajectory.append({
#             'timestamp': row['timestamp'],
#             'x_ego': row['x_ego'],
#             'y_ego': row['y_ego'],
#             'yaw_ego': row['yaw_ego'],
#             'length_ego': row['length_ego'],
#             'width_ego': row['width_ego']
#         })
#     return trajectory

def load_trajectory(file_path):
    df = pd.read_csv(file_path, header=0)
    df.columns.values[0] = 'timestamp'  # 将第一列的名称设为 'timestamp'
    # 获取表头
    headers = df.columns.tolist()
    # 将每行数据转为字典并添加到列表
    trajectory = df[headers].to_dict(orient='records')
    return trajectory

# 判断点是否在一个多边形范围内
def is_point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

# 判断每个角都在区域内
def are_rectangles_nested(rect1, rect2):
    return all(is_point_in_polygon(point, rect2) for point in rect1)

# 是否在边界内了，是返回1不是0
def is_vehicle_in_area(vehicle_x, vehicle_y, yaw, length, width, rect_points):
    corners = calculate_rectangle_vertices(length, width, yaw, [vehicle_x, vehicle_y])
    return are_rectangles_nested(corners, rect_points)

# 计算车辆的边界，四个角的坐标
def calculate_rectangle_vertices(length, width, angle, currentPos) -> list:
    angle = convert_clockwise_to_counterclockwise(angle)
    angle = math.radians(angle)
    dx = length / 2
    dy = width / 2
    vertex_offsets = [
        (-dx * math.cos(angle) - dy * math.sin(angle), -dx * math.sin(angle) + dy * math.cos(angle)),  # 左上角
        (-dx * math.cos(angle) + dy * math.sin(angle), -dx * math.sin(angle) - dy * math.cos(angle)),  # 左下角
        (dx * math.cos(angle) + dy * math.sin(angle), dx * math.sin(angle) - dy * math.cos(angle)),  # 右上角
        (dx * math.cos(angle) - dy * math.sin(angle), dx * math.sin(angle) + dy * math.cos(angle))   # 右下角
    ]
    vertices = [(currentPos[0] + offset[0], currentPos[1] + offset[1]) for offset in vertex_offsets]
    return vertices

def calculate_crash(svVertices, avVertices):

    # 分离轴定理计算是否相交
    rect1 = np.array(avVertices)  # 第一个矩形的四个顶点坐标
    rect2 = np.array(svVertices)  # 第二个矩形的四个顶点坐标
    overlap = rectangles_overlap(rect1, rect2)
    if overlap:
        return True
    else:
        return False

def rectangles_overlap(rect1, rect2):
    """判断两个矩形是否存在重叠部分"""
    # 将每个矩形表示为包含顶点和边的列表
    rect1_edges = [(rect1[i], rect1[(i + 1) % len(rect1)] - rect1[i]) for i in range(len(rect1))]
    rect2_edges = [(rect2[i], rect2[(i + 1) % len(rect2)] - rect2[i]) for i in range(len(rect2))]

    # 检查每个矩形的边是否为分离轴
    for edge in rect1_edges + rect2_edges:
        axis = np.array([-edge[1][1], edge[1][0]])  # 计算边的垂直向量作为分离轴
        min1, max1 = project_rect(rect1, axis)
        min2, max2 = project_rect(rect2, axis)
        if max1 < min2 or max2 < min1:  # 如果投影没有重叠，返回False
            return False
    return True

def project_rect(rect, axis):
    """在分离轴上投影矩形，并返回投影的最小值和最大值"""
    min_proj = np.dot(rect[0], axis)
    max_proj = min_proj
    for point in rect[1:]:
        proj = np.dot(point, axis)
        min_proj = min(min_proj, proj)
        max_proj = max(max_proj, proj)
    return min_proj, max_proj

def convert_clockwise_to_counterclockwise(angle) -> float:
    return (90 - angle) % 360

def disPoints(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def make_other_car_list(dataAll):
    other_cars_data = {}
    # 遍历数据并分类
    for key, value in dataAll.items():
        if "_" in key and "_ego" not in key:
            sceneList = key.split("_")
            if len(sceneList) > 1:
                # 提取编号，放入对应编号车的字典
                car_id = sceneList[1]
                if car_id not in other_cars_data:
                    other_cars_data[car_id] = {}
                if np.isnan(float(value)):
                    other_cars_data[car_id][sceneList[0]] = 0
                else:
                    other_cars_data[car_id][sceneList[0]] = value
    return other_cars_data

def check_vehicle_stopped_in_area(trajectory, rectangle_points, stop_threshold, stop_time_threshold):
    # stop_threshold = 0.1
    # stop_time_threshold = 5  # 停稳时间阈值，单位为秒
    stop_time = 0  # 当前连续停稳时间
    last_time = None  # 上一个时间戳
    vehicle_stopped = False
    sv_list = make_sv_list()

    start_parking_time = trajectory[0]['timestamp']

    for i in range(len(trajectory)):
        vehicle_x = trajectory[i]['x_ego']
        vehicle_y = trajectory[i]['y_ego']
        yaw = trajectory[i]['yaw_ego']
        length = trajectory[i]['length_ego']
        width = trajectory[i]['width_ego']
        avVertices = calculate_rectangle_vertices(length, width, yaw, [vehicle_x, vehicle_y])

        other_cars_data = make_other_car_list(trajectory[i])

        # 对所有的障碍物车辆进行判断，是否碰撞，撞到任何一辆车都视为结束，分数为0
        for key, value in other_cars_data.items():
            length_sv = value['length']
            width_sv = value['width']
            yaw_sv = value['yaw']
            currentPos_sv = [value['x'], value['y']]
            svVertices = calculate_rectangle_vertices(length_sv, width_sv, yaw_sv, currentPos_sv)

            # 如果发生了碰撞直接return 0
            if calculate_crash(svVertices, avVertices):
                # print('crash!!!!!!')
                return 0

        # 先判断是否进到这个停车的区域内了，如果有任何一个角不在边界内，也不算是进去（表示压线了）
        if is_vehicle_in_area(vehicle_x, vehicle_y, yaw, length, width, rectangle_points):
            if i > 0:
                previous_x = trajectory[i - 1]['x_ego']
                previous_y = trajectory[i - 1]['y_ego']
                distance = disPoints(vehicle_x, vehicle_y, previous_x, previous_y)

                # 如果车辆在区域内且与前一位置的距离小于阈值
                if distance < stop_threshold:
                    if last_time is not None:
                        # 计算时间差（秒）
                        time_diff = trajectory[i]['timestamp'] - last_time
                        stop_time += time_diff
                    last_time = trajectory[i]['timestamp']
                else:
                    stop_time = 0  # 如果车辆移动，重置停稳时间
                    last_time = trajectory[i]['timestamp']
            else:
                last_time = trajectory[i]['timestamp']  # 初始化第一个时间

            if stop_time >= stop_time_threshold:
                vehicle_stopped = True
                break  # 找到停稳的状态，结束循环

        # 如果三分钟还没有停进去，那就直接判断为失败了，加5秒是为了给最后停车要停够5秒
        if trajectory[i]['timestamp'] - start_parking_time > 180 + stop_time_threshold:
            return 0

    return 1 if vehicle_stopped else 0  # 返回结果


def make_rectangle_points_list():
    rectangle_points1 = [[-592.992187265738 , -174.22277831302776],[-598.9416591733268 , -173.35403325335633],[-598.5780209277807 , -170.8864162979757],[-592.6285505593507 , -171.75516112821774]]
    rectangle_points2 = [[-592.6080253188004 , -171.74076237582344],[-598.5574972701935 , -170.87201755832749],[-598.1938591256139 , -168.40440057887093],[-592.2443887145514 , -169.2731451676456]]
    rectangle_points3 = [[-592.2353612128776 , -169.26706537473765],[-598.1848332052864 , -168.39832079091983],[-597.8211951583108 , -165.93070378809543],[-591.8717247050636 , -166.799448143192]]
    rectangle_points4 = [[-591.8810935606122 , -166.8072333195833],[-597.8305655953125 , -165.93848895882184],[-597.4669276417086 , -163.47087193262965],[-591.5174571461708 , -164.33961606608608]]
    rectangle_points5 = [[-591.5222265430009 , -164.336308618121],[-597.4716986168585 , -163.46756448254024],[-597.108060758273 , -160.99994743298024],[-591.15859022358 , -161.86869134125592]]
    rectangle_points_list = [rectangle_points1,rectangle_points2,rectangle_points3,rectangle_points4,rectangle_points5]
    return rectangle_points_list

def make_sv_list():
    sv1 = {'x_1': -594.2659627, 'y_1': -162.6752182, 'v_1': 0, 'a_1': 0, 'yaw_1': 278.307647705078, 'width_1': 1.812, 'length_1': 4.03}
    sv2 = {'x_1': -595.3131924, 'y_1': -170.0853173, 'v_1': 0, 'a_1': 0, 'yaw_1': 278.307647705078, 'width_1': 1.958, 'length_1': 4.395}
    sv_list = [sv1, sv2]
    return sv_list

def startCalParkingIndex(code, avName, caseID, taskID, startTime, endTime, sceneName, csv_file_path):
    # return makePerfectData(code, avName, caseID, taskID, startTime, endTime, sceneName)

    if os.path.exists(csv_file_path):
        # 文件存在，读取数据
        pass
    else:
        # 文件不存在，输出错误信息
        print("错误：文件路径无效，返回默认值")
        return makeZeroData(code, avName, caseID, taskID, startTime, endTime, sceneName)

    # 从csv中拿到轨迹
    trajectory = load_trajectory(csv_file_path)
    # 在这里改停车点信息
    # 测试用，场景5车在停车点内的
    # rectangle_points = [[(-1222.381825863831, -402.8547537619264), (-1226.1134075289922, -404.2953433204866),
    #                     (-1223.9525231911518, -409.89271581822845), (-1220.2209415259906, -408.45212625966826)]]
    # 测试用，场景5车不在停车点内的
    # rectangle_points = [(-1232.381825863831, -402.8547537619264), (-1236.1134075289922, -404.2953433204866),
    #                     (-1233.9525231911518, -409.89271581822845), (-1230.2209415259906, -408.45212625966826)]
    rectangle_points_list = make_rectangle_points_list()
    stop_threshold = 0
    stop_time_threshold = 0  # 停稳时间阈值，单位为秒
    # 将轨迹和停车区信息穿进去判断是否正常停车，结果为1是正常停车，0是未正常停车
    result = 0
    for rectangle_points_i in rectangle_points_list:
        result = check_vehicle_stopped_in_area(trajectory, rectangle_points_i, stop_threshold, stop_time_threshold)
        if result:
            break
    # 最后根据result的结果调用other的分数并返回，1表示停进去了，调用满分
    if result:
        return makePerfectData(code, avName, caseID, taskID, startTime, endTime, sceneName)
    else:
        return makeZeroDataByParkingError(code, avName, caseID, taskID, startTime, endTime, sceneName)

# # 示例用法
# csv_file_path = 'C:/Users/USER/Desktop/Projects/demo/onsiteChangan/2b68ee0d-8453-11ef-8347-00163e21f0fc_1_1567_0.csv'
# trajectory = load_trajectory(csv_file_path)
# a = time.time()
# print(time.time())
# rectangle_points = [(-1222.381825863831, -402.8547537619264), (-1226.1134075289922, -404.2953433204866), (-1223.9525231911518, -409.89271581822845), (-1220.2209415259906, -408.45212625966826)]
# for i in range(14):
#     result = check_vehicle_stopped_in_area(trajectory, rectangle_points, 1,5)
# print(time.time())
# b = time.time()
# print(b-a)
# print("Result:", result)


