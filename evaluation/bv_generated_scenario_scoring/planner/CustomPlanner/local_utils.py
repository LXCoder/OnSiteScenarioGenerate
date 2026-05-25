# import geopandas as gpd
import numpy as np
import math
from shapely.geometry import LineString
from planner.CustomPlanner.planner_components.essentials import RefPoint, TrajPoint

# def convert_opendrive_to_geopandas(open_drive_info):
#     geom = []
#     id = []
#     color = []
#     for lane in open_drive_info.discretelanes:
#         geom.append(LineString(lane.center_vertices))
#         id.append(lane.lane_id)
#         color.append("tab:red" if lane.lane_id == "2.1.-3.-1.0" else ("tab:green" if lane.lane_id == '2.1.-2.-1.0' else "tab:blue"))
#
#     return gpd.GeoDataFrame({"geometry": geom, "id": id, "color": color})

def collect_map_sample_points(open_drive_info):
    res = []
    for lane in open_drive_info.discretelanes:
        res.extend(lane.center_vertices.tolist())

    return res

def normalize_angle(angle_rad):
    # to normalize an angle to [-pi, pi]
    a = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a = a + 2.0 * math.pi
    return a - math.pi

def calculate_distances(points, points_set):
    if not isinstance(points[0], list):
        points = np.array([points] * len(points_set))
    else:
        points = np.array([np.mean(points, axis=0)] * len(points_set))

    points_set = np.array(points_set)
    distances = cal_euclidean_distance(points, points_set)
    return np.argmin(distances), np.min(distances)


def cal_euclidean_distance(point1, point2):
    return np.linalg.norm(point1 - point2, axis=1)

def cal_manhattan_distance(point1, point2):
    return np.mean(np.abs(point1 - point2))


def NormalizeAngle(angle_rad):
    # to normalize an angle to [-pi, pi]
    a = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a = a + 2.0 * math.pi
    return a - math.pi

def cartesian_to_frenet(path_point, traj_point):
    ''' from Cartesian to Frenet coordinate, to the matched path point
    copy Apollo cartesian_frenet_conversion.cpp'''
    rx, ry, rs, rtheta, rkappa, rdkappa = path_point.rx, path_point.ry, path_point.rs, \
                                          path_point.rtheta, path_point.rkappa, path_point.rdkappa
    x, y, v, a, theta, kappa = traj_point.x, traj_point.y, traj_point.v, \
                               traj_point.a, traj_point.theta, traj_point.kappa

    s_condition = np.zeros(3)
    d_condition = np.zeros(3)

    dx = x - rx
    dy = y - ry

    cos_theta_r = math.cos(rtheta)
    sin_theta_r = math.sin(rtheta)

    cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
    d_condition[0] = math.copysign(math.sqrt(dx ** 2 + dy ** 2), cross_rd_nd)

    delta_theta = theta - rtheta
    tan_delta_theta = math.tan(delta_theta)
    cos_delta_theta = math.cos(delta_theta)

    one_minus_kappa_r_d = 1 - rkappa * d_condition[0]
    d_condition[1] = one_minus_kappa_r_d * tan_delta_theta

    kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]

    d_condition[2] = -kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d / (cos_delta_theta ** 2) * \
                     (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa)

    s_condition[0] = rs
    s_condition[1] = v * cos_delta_theta / one_minus_kappa_r_d

    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
    s_condition[2] = (a * cos_delta_theta - s_condition[1] ** 2 * \
                      (d_condition[1] * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d

    return s_condition, d_condition


def frenet_to_cartesian(path_point, s_condition, d_condition):
    ''' from Frenet to Cartesian coordinate
    copy Apollo cartesian_frenet_conversion.cpp'''
    rx, ry, rs, rtheta, rkappa, rdkappa = path_point.rx, path_point.ry, path_point.rs, \
                                          path_point.rtheta, path_point.rkappa, path_point.rdkappa
    if math.fabs(rs - s_condition[0]) >= 1.0e-6:
        pass
        # print("the reference point s and s_condition[0] don't match")

    cos_theta_r = math.cos(rtheta)
    sin_theta_r = math.sin(rtheta)

    x = rx - sin_theta_r * d_condition[0]
    y = ry + cos_theta_r * d_condition[0]

    one_minus_kappa_r_d = 1 - rkappa * d_condition[0]
    tan_delta_theta = d_condition[1] / one_minus_kappa_r_d
    delta_theta = math.atan2(d_condition[1], one_minus_kappa_r_d)
    cos_delta_theta = math.cos(delta_theta)
    theta = NormalizeAngle(delta_theta + rtheta)

    kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]
    kappa = ((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta ** 2 / one_minus_kappa_r_d \
             + rkappa) * cos_delta_theta / one_minus_kappa_r_d

    d_dot = d_condition[1] * s_condition[1]
    v = math.sqrt((one_minus_kappa_r_d * s_condition[1]) ** 2 + d_dot ** 2)

    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
    a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta + s_condition[1] ** 2 / cos_delta_theta * \
        (d_condition[1] * delta_theta_prime - kappa_r_d_prime)

    return [x, y, v, a, theta, kappa]

def collision_test_rough(point, obs, ego, obs_traj, time_index):
    """
    粗略的检测是否车辆与障碍物发生朋友，这里将障碍物简单看成一个圆形
    :param point:
    :param obs:
    :param ego:
    :return:
    """
    if isinstance(point, RefPoint):
        dis = math.sqrt((point.rx - obs_traj[time_index][0]) ** 2 + (point.ry - obs_traj[time_index][1]) ** 2)
    elif isinstance(point, TrajPoint):
        dis = math.sqrt((point.x - obs_traj[time_index][0]) ** 2 + (point.y - obs_traj[time_index][1]) ** 2)
    else:
        raise Exception("Not valid data")
    max_veh = max(ego.length, ego.width)
    max_obs = max(obs.length, obs.width)
    return dis - (max_veh + max_obs) / 2

# 碰撞检测 (这部分参考apollo代码)
def collision_test(point, obs, ego, obs_traj, time_index):
    shift_x = obs_traj[time_index][0] - point.x
    shift_y = obs_traj[time_index][1] - point.y

    cos_v = math.cos(point.theta)
    sin_v = math.sin(point.theta)
    cos_o = math.cos(obs.theta)
    sin_o = math.sin(obs.theta)
    half_l_v = ego.length / 2
    half_w_v = ego.width / 2
    half_l_o = obs.length / 2
    half_w_o = obs.width / 2

    dx1 = cos_v * ego.length / 2
    dy1 = sin_v * ego.length / 2
    dx2 = sin_v * ego.width / 2
    dy2 = -cos_v * ego.width / 2
    dx3 = cos_o * obs.length / 2
    dy3 = sin_o * obs.length / 2
    dx4 = sin_o * obs.width / 2
    dy4 = -cos_o * obs.width / 2

    # 使用分离轴定理进行碰撞检测
    return ((abs(shift_x * cos_v + shift_y * sin_v) <=
             abs(dx3 * cos_v + dy3 * sin_v) + abs(dx4 * cos_v + dy4 * sin_v) + half_l_v)
            and (abs(shift_x * sin_v - shift_y * cos_v) <=
                 abs(dx3 * sin_v - dy3 * cos_v) + abs(dx4 * sin_v - dy4 * cos_v) + half_w_v)
            and (abs(shift_x * cos_o + shift_y * sin_o) <=
                 abs(dx1 * cos_o + dy1 * sin_o) + abs(dx2 * cos_o + dy2 * sin_o) + half_l_o)
            and (abs(shift_x * sin_o - shift_y * cos_o) <=
                 abs(dx1 * sin_o - dy1 * cos_o) + abs(dx2 * sin_o - dy2 * cos_o) + half_w_o))

def detect_collision(traj_points, obstacle, ego, obstacle_trajectory):
    """
    判断车辆轨迹是否会和障碍物发生碰撞
    :param traj_points: 车辆轨迹
    :param obstacle: 障碍物信息
    :return: 障碍物距离，是否会碰撞 如果没有碰撞则返回（dis_mean, False） 与障碍物的凭据距离，不碰撞；如果碰撞了则返回 （-1，True）
    """
    dis_sum = 0
    for i, point in enumerate(traj_points):
        if isinstance(point, RefPoint):  # 如果是原来路径点，就只按圆形计算。因为每点的车辆方向难以获得
            # 计算障碍物和车辆的距离，如果<0说明有碰撞风险
            if collision_test_rough(point, obstacle, ego, obstacle_trajectory, i) > 0:
                continue
            return -1, True
        else:
            dis = collision_test_rough(point, obstacle, ego, obstacle_trajectory, i)
            dis_sum += dis
            if dis > 0:
                continue
            # 对于车辆与障碍物是否碰撞 ColliTestRough不足以(将两者视为圆形) 要用更准确的ColliTest检测是否碰撞
            if collision_test(point, obstacle, ego, obstacle_trajectory, i):
                # print("不满足实际碰撞检测")
                return -1, True

    dis_mean = dis_sum / len(traj_points)
    return dis_mean, False

def predict_obstacles_traj(obstacles, time_range):
    obstacles_traj = {}
    for obstacle in obstacles:
        obstacles_traj[obstacle.name] = []
        for t in time_range:
            displacement = obstacle.v *  t + 0.5 * obstacle.a * t ** 2
            obstacles_traj[obstacle.name].append((
                obstacle.x + displacement * math.cos(obstacle.theta),
                obstacle.y + displacement * math.sin(obstacle.theta)))
    return obstacles_traj