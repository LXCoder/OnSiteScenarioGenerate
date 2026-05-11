import os
import xml.etree.ElementTree as ET
import json
import math
import random
import numpy as np
from glob import glob
from Utils.Constant import START_TIME_THRESHOLD, MAX_TESSNG_VEHICLES


def _calculate_average_speed(path, times):
    """根据轨迹点和时间戳计算平均速度"""
    if len(path) < 2 or times[-1] <= times[0]:
        return 10.0

    total_dist = sum(
        math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        for i in range(len(path) - 1)
    )
    duration = times[-1] - times[0]
    return total_dist / duration if duration > 0 else 10.0


def _parse_trajectory_vertices(traj_elem, start_time=START_TIME_THRESHOLD):
    """提取坐标路径并过滤掉早于 start_time 的点"""
    path, times = [], []
    for vertex in traj_elem.iter("Vertex"):
        t = float(vertex.get("time"))
        if t < start_time:
            continue

        pos = vertex.find("Position/WorldPosition")
        if pos is not None:
            path.append([float(pos.get("x")), float(pos.get("y"))])
            times.append(t)
    return path, times


def _get_trajectory_element(action_elem):
    """获取 Trajectory 节点"""
    traj = action_elem.find(".//Trajectory")
    if traj is None:
        traj = action_elem.find(".//TrajectoryRef/Trajectory")
    return traj


def _douglas_peucker(points, epsilon):
    """道格拉斯-普克算法实现轨迹抽稀
    points: 原始轨迹点列表 [[x1, y1], [x2, y2], ...]
    epsilon: 距离阈值（单位通常为米）。值越大，抽稀越狠。
    """
    if len(points) < 3:
        return points

    points = np.array(points)
    start_point = points[0]
    end_point = points[-1]

    # 计算所有点到首尾连线的垂直距离
    # 向量化计算提高效率
    if np.allclose(start_point, end_point):  # 环路处理
        dists = np.linalg.norm(points[1:-1] - start_point, axis=1)
    else:
        line_vec = end_point - start_point
        line_unit_vec = line_vec / np.linalg.norm(line_vec)
        p_vec = points[1:-1] - start_point
        dist_parallel = np.dot(p_vec, line_unit_vec)
        dist_perpendicular = np.linalg.norm(
            p_vec - np.outer(dist_parallel, line_unit_vec), axis=1
        )
        dists = dist_perpendicular

    max_dist = np.max(dists)
    index = np.argmax(dists) + 1

    # 如果最大距离大于阈值，则递归处理
    if max_dist > epsilon:
        left_results = _douglas_peucker(points[: index + 1].tolist(), epsilon)
        right_results = _douglas_peucker(points[index:].tolist(), epsilon)
        return left_results[:-1] + right_results
    else:
        return [points[0].tolist(), points[-1].tolist()]


def get_smart_waypoints(points, epsilon=0.2, min_points=3):
    # 1. 使用 DP 算法提取特征点
    simplified = _douglas_peucker(points, epsilon)  # 调用之前给你的 DP 函数

    # 2. 如果点数已经达标，直接返回
    if len(simplified) >= min_points:
        return simplified

    # 3. 如果是直线导致点数不足，进行中点插值
    # 针对你之前那种只有 2 个点的情况
    while len(simplified) < min_points:
        new_points = [simplified[0]]
        for i in range(len(simplified) - 1):
            p1 = np.array(simplified[i])
            p2 = np.array(simplified[i + 1])
            # 在两点之间插入几何中心点
            mid_point = ((p1 + p2) / 2).tolist()
            new_points.append(mid_point)
            new_points.append(p2.tolist())
        # 去重并更新（保持顺序）
        simplified = []
        [simplified.append(p) for p in new_points if p not in simplified]

    return simplified


def parse_xosc(file_path):
    """解析单个 xosc 文件并确保 ego 在 JSON 顶端"""
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except Exception as e:
        print(f"Error parsing {file_path}: {e}")
        return None

    all_vehicles = {}

    # 1. 遍历所有轨迹节点提取数据
    for maneuver_group in root.iter("ManeuverGroup"):
        actors = [
            actor.get("entityRef")
            for actor in maneuver_group.findall(".//Actors/EntityRef")
        ]
        if not actors:
            continue

        for follow_action in maneuver_group.findall(".//FollowTrajectoryAction"):
            traj_elem = _get_trajectory_element(follow_action)
            if traj_elem is None:
                continue

            path, times = _parse_trajectory_vertices(traj_elem)
            if not path:
                continue

            avg_speed = _calculate_average_speed(path, times)

            for actor_name in actors:
                is_ego = "Ego" in actor_name
                vehicle_key = "ego" if is_ego else actor_name.lower()
                all_vehicles[vehicle_key] = {
                    "path": path,
                    # "path": get_smart_waypoints(path, 0.3, 3),
                    "speed": round(avg_speed, 2),
                    "color": "#14c704" if is_ego else "#3498db",
                    "control": "model",
                }

    if not all_vehicles:
        return None

    tess_control_count = random.randint(0, int(MAX_TESSNG_VEHICLES))
    vehicle_keys = list(all_vehicles.keys())
    selected_keys = random.sample(
        vehicle_keys, min(tess_control_count, len(vehicle_keys))
    )
    for key in selected_keys:
        if key == "ego":
            continue
        all_vehicles[key]["control"] = "tessng"
        all_vehicles[key]["path"] = get_smart_waypoints(
            all_vehicles[key]["path"], 0.5, 3
        )
        print(f"path is: {all_vehicles[key]['path']}")

    # 2. 重新排序：确保 ego 排在字典的最前面
    ordered_vehicles = {}
    if "ego" in all_vehicles:
        ordered_vehicles["ego"] = all_vehicles.pop("ego")
    ordered_vehicles.update(all_vehicles)

    return {"vehicles": ordered_vehicles}


def main():
    input_dir = "Data/select800"
    output_dir = "Data/converted_scenes"

    os.makedirs(output_dir, exist_ok=True)

    xosc_files = glob(os.path.join(input_dir, "**/*.xosc"), recursive=True)
    print(f"Found {len(xosc_files)} .xosc files.")

    for xosc_path in xosc_files:
        scenario_name = os.path.basename(os.path.dirname(xosc_path))
        print(f"Processing {scenario_name}...")

        data = parse_xosc(xosc_path)
        if data:
            output_path = os.path.join(output_dir, f"{scenario_name}.json")
            with open(output_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)


if __name__ == "__main__":
    main()
