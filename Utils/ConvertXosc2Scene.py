import os
import xml.etree.ElementTree as ET
import json
import math
from glob import glob
from Utils.Constant import START_TIME_THRESHOLD


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
                    "speed": round(avg_speed, 2),
                    "color": "#14c704" if is_ego else "#3498db",
                }

    if not all_vehicles:
        return None

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

