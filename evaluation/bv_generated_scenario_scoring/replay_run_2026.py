#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
回放评测主入口（2026版）。

核心职责：
1) 解析 raw xosc/xodr 或 pkl workspace 场景；
2) 统一执行背景指标与主车接管评测；
3) 调用 score_2026.py 输出可交付分数文件。
"""

from __future__ import print_function

import argparse
import copy
import csv
import json
import logging
import multiprocessing
import os
import re
import shutil
import sys
import time
from datetime import datetime
from collections import defaultdict
from utils.recorder import Recorder
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from utils.functions import check_action, detectCollision, is_point_inside_rect, updateEgoPos
import importlib.util
import xml.etree.ElementTree as ET

import numpy as np

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)


TYPE_TO_ID = {
    "car": 0,
    "pedestrian": 1,
    "bicycle": 2,
}

# 2026 口径固定策略：主车 warmup 与 ego 轨迹接管帧均固定为 31。
FIXED_WARMUP = 31
FIXED_EGO_FRAMES = "31"


def _enforce_fixed_replay_policy(warmup, ego_frames, warn_prefix=""):
    """统一约束 replay 固定策略，避免不同入口出现分歧。"""
    normalized_warmup = FIXED_WARMUP
    req_warmup = _safe_float(warmup, None)
    if req_warmup is not None and int(req_warmup) != FIXED_WARMUP:
        print("[WARNING] %swarmup=%s is ignored; forced to %s." % (warn_prefix, str(warmup), FIXED_WARMUP))

    normalized_ego_frames = FIXED_EGO_FRAMES
    req_ego_frames = str(ego_frames or "").strip().lower()
    if req_ego_frames and req_ego_frames != FIXED_EGO_FRAMES:
        print(
            "[WARNING] %sego-frames=%s is ignored; forced to %s."
            % (warn_prefix, str(ego_frames), FIXED_EGO_FRAMES)
        )
    return normalized_warmup, normalized_ego_frames


class CustomReplayController:
    def __init__(self, warmup: int = 31, visualize: bool = False):
        self._agent_type = ["vehicle", "pedestrian", "bicycle"]
        self.observation = Observation()
        self.warmup = warmup
        self.visualize = visualize
        if self.visualize:
            from utils.visualizer import Visualizer

            self.visualizer = Visualizer()

    def init(self, scenario_info):
        self.t = -1
        self.n_agent, self.n_frame, _ = scenario_info.additional_info["positions"].shape
        self.ego_index = scenario_info.additional_info["ids"].index("Ego")

        self.scenario_info = scenario_info
        if self.visualize:
            self.visualizer.live_init(scenario_info, parse_opendrive(scenario_info.source_file["xodr"]))
        self.observation.update_test_info(dt=self.scenario_info.task_info["dt"])
        self.update_ego()

    def get_observation(self):
        return copy.deepcopy(self.observation)

    def update_frame(self):
        self.observation.erase_object_info()
        for i in range(self.n_agent):
            if i != self.ego_index and self.scenario_info.additional_info["valid_mask"][i, self.t]:
                self.observation.update_object_info(
                    category=self._agent_type[self.scenario_info.additional_info["types"][i]],
                    obj_name=self.scenario_info.additional_info["ids"][i],
                    x=self.scenario_info.additional_info["positions"][i, self.t, 0].item(),
                    y=self.scenario_info.additional_info["positions"][i, self.t, 1].item(),
                    v=self.scenario_info.additional_info["velocities"][i, self.t].item(),
                    a=self.scenario_info.additional_info["accelerations"][i, self.t].item(),
                    yaw=self.scenario_info.additional_info["headings"][i, self.t].item(),
                    length=self.scenario_info.additional_info["shapes"][i, 0].item(),
                    width=self.scenario_info.additional_info["shapes"][i, 1].item(),
                )
        self.observation.update_test_info(end=self._update_end_status())
        if self.visualize:
            self.visualizer.live_update(self.get_observation())

    def update_ego(self, action=None):
        self.t += 1
        self.observation.update_test_info(t=round(self.t * self.scenario_info.task_info["dt"], 2))
        if action:
            updateEgoPos(action, self.scenario_info.task_info["dt"], self.observation.ego_info)
        else:
            self.observation.update_ego_info(
                x=self.scenario_info.additional_info["positions"][self.ego_index, self.t, 0].item(),
                y=self.scenario_info.additional_info["positions"][self.ego_index, self.t, 1].item(),
                v=self.scenario_info.additional_info["velocities"][self.ego_index, self.t].item(),
                a=self.scenario_info.additional_info["accelerations"][self.ego_index, self.t].item(),
                yaw=self.scenario_info.additional_info["headings"][self.ego_index, self.t].item(),
                length=self.scenario_info.additional_info["shapes"][self.ego_index, 0].item(),
                width=self.scenario_info.additional_info["shapes"][self.ego_index, 1].item(),
            )

    def _update_end_status(self):
        if self.t >= (self.n_frame - 1):
            return 2

        if self.observation.test_info["t"] > 0.5:
            collide_info = detectCollision(self.observation.ego_info, self.observation.object_info)
            if collide_info:
                return 3

        if is_point_inside_rect(
            self.scenario_info.task_info["targetPos"],
            [self.observation.ego_info.x, self.observation.ego_info.y],
        ):
            return 1

        return -1


class Tester:
    def __init__(self, input_dir, output_dir, logger=None):
        self.root_dir = os.path.dirname(os.path.abspath(__file__))
        self.input_dir = input_dir
        self.output_dir = output_dir
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.logger = logger
        self.setup_logger(output_dir)

    def setup_logger(self, output_dir, level=logging.DEBUG):
        if self.logger is not None:
            return self.logger

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        logger = logging.getLogger("file")
        logger.setLevel(level)
        if logger.handlers:
            logger.handlers = []

        log_file = os.path.join(output_dir, "test_info.log")
        file_handler = logging.FileHandler(log_file, "a", "utf-8")
        file_handler.setLevel(level)
        formatter = logging.Formatter("%(asctime)s - %(levelname)8s - %(message)s", "%Y-%m-%d %H:%M:%S")
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        self.logger = logger
        return logger

    def replay_test(self, config, scene, cur_planner, output_path):
        controller = CustomReplayController(config["warmup"], config["visualize"])
        recorder = Recorder()
        action = [float("nan"), float("nan")]

        controller.init(scene)

        planner = cur_planner()
        planner.init(scene.format())

        while True:
            controller.update_frame()
            recorder.record(action, controller.get_observation())

            if controller.observation.test_info["end"] != -1:
                recorder.output(output_path)
                break

            if controller.t >= controller.warmup - 1:
                action = planner.act(controller.get_observation())
                ego_action = check_action(
                    dt=scene.task_info["dt"],
                    prev_v=controller.observation.ego_info.v,
                    prev_action=[controller.observation.ego_info.a, controller.observation.ego_info.rot],
                    new_action=action,
                )
                controller.update_ego(ego_action)
            else:
                controller.update_ego()

CAR_ALIASES = {
    "car", "truck", "bus", "van", "pickup", "suv", "trailer",
}
PEDESTRIAN_ALIASES = {
    "pedestrian", "person", "walker",
}
BICYCLE_ALIASES = {
    "bicycle", "bike", "motorbike", "motorcycle", "moped",
}

DEFAULT_SHAPE = {
    "car": (4.5, 1.8),
    "pedestrian": (0.8, 0.6),
    "bicycle": (1.8, 0.6),
}

FLOAT_PATTERN = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"


def _safe_float(value, default=None):
    try:
        return float(value)
    except Exception:
        return default


def _normalize_name(name):
    if not name:
        return ""
    return re.sub(r"[^a-z0-9]+", "", name.lower())


def _find_ego_name(entity_names):
    if not entity_names:
        return ""
    for name in entity_names:
        if _normalize_name(name) == "ego":
            return name
    for name in entity_names:
        if "ego" in _normalize_name(name):
            return name
    return ""


def _canonical_category(raw_category):
    key = (raw_category or "").strip().lower()
    if key in CAR_ALIASES:
        return "car"
    if key in PEDESTRIAN_ALIASES:
        return "pedestrian"
    if key in BICYCLE_ALIASES:
        return "bicycle"
    return "car"


def _extract_comment_section(raw_text, section_name):
    pattern = r"\[" + re.escape(section_name) + r"\]\s*(.*?)(?:-->|$)"
    m = re.search(pattern, raw_text, flags=re.S | re.I)
    if not m:
        return ""
    return m.group(1).strip()


def _parse_ego_initial_state(raw_text):
    section = _extract_comment_section(raw_text, "Initial State")
    if not section:
        return None

    kv = {}
    for key, value in re.findall(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(" + FLOAT_PATTERN + r")", section):
        kv[key.lower()] = float(value)

    speed = kv.get("v_init", kv.get("v"))
    x0 = kv.get("x_init", kv.get("x"))
    y0 = kv.get("y_init", kv.get("y"))
    heading = kv.get("heading_init", kv.get("h_init", kv.get("yaw_init", kv.get("h"))))

    if speed is None or x0 is None or y0 is None or heading is None:
        values = [float(x) for x in re.findall(FLOAT_PATTERN, section)]
        if len(values) >= 4:
            speed, x0, y0, heading = values[0], values[1], values[2], values[3]

    if speed is None or x0 is None or y0 is None or heading is None:
        return None

    return {
        "speed": float(speed),
        "x": float(x0),
        "y": float(y0),
        "heading": float(heading),
    }


def _parse_goal(raw_text, xml_root=None):
    section = _extract_comment_section(raw_text, "Driving Task")
    if section:
        x_match = re.search(
            r"x_target\s*=\s*\(\s*(" + FLOAT_PATTERN + r")\s*,\s*(" + FLOAT_PATTERN + r")\s*\)",
            section,
            flags=re.I,
        )
        y_match = re.search(
            r"y_target\s*=\s*\(\s*(" + FLOAT_PATTERN + r")\s*,\s*(" + FLOAT_PATTERN + r")\s*\)",
            section,
            flags=re.I,
        )
        if x_match and y_match:
            x1, x2 = float(x_match.group(1)), float(x_match.group(2))
            y1, y2 = float(y_match.group(1)), float(y_match.group(2))
            return [min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)]

        values = [float(x) for x in re.findall(FLOAT_PATTERN, section)]
        if len(values) >= 4:
            x1, x2, y1, y2 = values[0], values[1], values[2], values[3]
            return [min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)]

    # Fallback: parse native OpenSCENARIO goal from ReachPositionCondition.
    if xml_root is not None:
        for reach in xml_root.findall(".//ReachPositionCondition"):
            tol = _safe_float(reach.get("tolerance"), 0.0)
            wp = reach.find(".//WorldPosition")
            if wp is None:
                continue
            x = _safe_float(wp.get("x"), None)
            y = _safe_float(wp.get("y"), None)
            if x is None or y is None:
                continue
            return [float(x - tol), float(x + tol), float(y - tol), float(y + tol)]

    return None


def _parse_stop_trigger_time(root):
    values = []
    for sim_cond in root.findall(".//Storyboard//StopTrigger//SimulationTimeCondition"):
        value = _safe_float(sim_cond.get("value"), None)
        if value is not None:
            values.append(float(value))
    if not values:
        return None
    return max(values)


def _parse_entities(root):
    agents = []
    for idx, obj in enumerate(root.findall("./Entities/ScenarioObject")):
        name = obj.get("name", "").strip()
        if not name:
            continue

        vehicle = obj.find("./Vehicle")
        pedestrian = obj.find("./Pedestrian")

        if vehicle is not None:
            raw_cat = vehicle.get("vehicleCategory", "car")
            dims = vehicle.find("./BoundingBox/Dimensions")
        elif pedestrian is not None:
            raw_cat = "pedestrian"
            dims = pedestrian.find("./BoundingBox/Dimensions")
        else:
            raw_cat = "pedestrian"
            dims = obj.find(".//Dimensions")

        cat = _canonical_category(raw_cat)
        default_length, default_width = DEFAULT_SHAPE[cat]
        length = _safe_float(dims.get("length"), default_length) if dims is not None else default_length
        width = _safe_float(dims.get("width"), default_width) if dims is not None else default_width

        agents.append(
            {
                "idx": idx,
                "name": name,
                "category": cat,
                "length": float(length),
                "width": float(width),
            }
        )

    return agents


def _resolve_entity_name(entity_ref, alias_map):
    if not entity_ref:
        return ""
    if entity_ref in alias_map:
        return alias_map[entity_ref]
    return alias_map.get(_normalize_name(entity_ref), "")


def _extract_actor_entity_ref(act):
    refs = act.findall("./ManeuverGroup/Actors/EntityRef")
    if not refs:
        refs = act.findall(".//Actors/EntityRef")
    if not refs:
        return ""
    return refs[0].get("entityRef", "").strip()


def _extract_points_from_act(act):
    points = []
    for vertex in act.findall(".//FollowTrajectoryAction//Vertex"):
        t = _safe_float(vertex.get("time"), None)
        world_pos = vertex.find(".//WorldPosition")
        if t is None or world_pos is None:
            continue
        x = _safe_float(world_pos.get("x"), None)
        y = _safe_float(world_pos.get("y"), None)
        heading = _safe_float(world_pos.get("h"), None)
        if heading is None:
            heading = _safe_float(world_pos.get("yaw"), None)
        if x is None or y is None or heading is None:
            continue
        points.append((float(t), float(x), float(y), float(heading)))
    return points


def _deduplicate_points(points):
    if not points:
        return []
    by_time = {}
    for t, x, y, h in points:
        by_time[round(float(t), 6)] = (float(x), float(y), float(h))
    return [(t, v[0], v[1], v[2]) for t, v in sorted(by_time.items(), key=lambda x: x[0])]


def _parse_trajectories(root, entity_names):
    alias_map = {}
    for name in entity_names:
        alias_map[name] = name
        alias_map[_normalize_name(name)] = name

    traj = {name: [] for name in entity_names}
    for act in root.findall(".//Storyboard//Story//Act"):
        entity_ref = _extract_actor_entity_ref(act)
        entity_name = _resolve_entity_name(entity_ref, alias_map)
        if not entity_name:
            continue
        traj[entity_name].extend(_extract_points_from_act(act))

    for name in traj:
        traj[name] = _deduplicate_points(traj[name])
    return traj


def _resample_points(points, num_frames, dt):
    valid_mask = np.zeros(num_frames, dtype=bool)
    positions = np.zeros((num_frames, 2), dtype=np.float32)
    headings = np.zeros(num_frames, dtype=np.float32)

    if not points:
        return valid_mask, positions, headings

    arr = np.array(points, dtype=np.float64)
    times = arr[:, 0]
    xs = arr[:, 1]
    ys = arr[:, 2]
    hs = arr[:, 3]

    if len(times) == 1:
        idx = int(round(times[0] / dt))
        if 0 <= idx < num_frames:
            valid_mask[idx] = True
            positions[idx, 0] = xs[0]
            positions[idx, 1] = ys[0]
            headings[idx] = np.mod(hs[0], 2 * np.pi)
        return valid_mask, positions, headings

    frame_times = np.arange(num_frames, dtype=np.float64) * dt
    inside = (frame_times >= times[0] - 1e-6) & (frame_times <= times[-1] + 1e-6)
    valid_mask[inside] = True
    if np.any(inside):
        t_valid = frame_times[inside]
        positions[inside, 0] = np.interp(t_valid, times, xs).astype(np.float32)
        positions[inside, 1] = np.interp(t_valid, times, ys).astype(np.float32)
        headings[inside] = np.mod(np.interp(t_valid, times, np.unwrap(hs)), 2 * np.pi).astype(np.float32)

    return valid_mask, positions, headings


def _build_ego_fallback_track(ego_init, goal, num_frames, dt, mode):
    valid_mask = np.zeros(num_frames, dtype=bool)
    positions = np.zeros((num_frames, 2), dtype=np.float32)
    headings = np.zeros(num_frames, dtype=np.float32)

    if ego_init is None or num_frames <= 0:
        return valid_mask, positions, headings

    x0 = float(ego_init.get("x", 0.0))
    y0 = float(ego_init.get("y", 0.0))
    h0 = float(np.mod(ego_init.get("heading", 0.0), 2 * np.pi))
    v0 = max(0.0, float(ego_init.get("speed", 0.0)))

    if mode == "min2":
        valid_mask[0] = True
        positions[0] = [x0, y0]
        headings[0] = h0
        if num_frames > 1:
            step = max(v0 * dt, 1e-3)
            x1 = x0 + step * np.cos(h0)
            y1 = y0 + step * np.sin(h0)
            valid_mask[1] = True
            positions[1] = [x1, y1]
            headings[1] = h0
        return valid_mask, positions, headings

    if mode == "to_goal" and goal is not None and len(goal) >= 4:
        gx = 0.5 * (float(goal[0]) + float(goal[1]))
        gy = 0.5 * (float(goal[2]) + float(goal[3]))
        valid_mask[:] = True
        if num_frames == 1:
            positions[0] = [x0, y0]
            headings[0] = h0
            return valid_mask, positions, headings

        alpha = np.linspace(0.0, 1.0, num_frames, dtype=np.float32)
        positions[:, 0] = x0 + (gx - x0) * alpha
        positions[:, 1] = y0 + (gy - y0) * alpha

        direction_heading = h0
        if abs(gx - x0) > 1e-6 or abs(gy - y0) > 1e-6:
            direction_heading = float(np.mod(np.arctan2(gy - y0, gx - x0), 2 * np.pi))
        headings[:] = direction_heading
        return valid_mask, positions, headings

    # raw/default: keep a constant ego state for all available frames.
    valid_mask[:] = True
    positions[:, 0] = x0
    positions[:, 1] = y0
    headings[:] = h0
    return valid_mask, positions, headings


def _build_single_point_track(num_frames, x, y, heading):
    valid_mask = np.zeros(num_frames, dtype=bool)
    positions = np.zeros((num_frames, 2), dtype=np.float32)
    headings = np.zeros(num_frames, dtype=np.float32)
    if num_frames <= 0:
        return valid_mask, positions, headings
    valid_mask[0] = True
    positions[0] = [float(x), float(y)]
    headings[0] = np.mod(float(heading), 2 * np.pi)
    return valid_mask, positions, headings


def _force_ego_frame_policy(
    ego_points,
    ego_valid_mask,
    ego_positions,
    ego_headings,
    ego_init,
    goal,
    num_frames,
    dt,
    ego_mode,
    ego_frames,
):
    # 统一 Ego 帧策略：
    # - 当前仅允许 31 帧策略，确保与比赛/评测口径一致。
    # - 优先用 xosc 轨迹；缺失时再用注释区初始状态进行兜底补齐。
    policy = str(ego_frames or "31").strip().lower()

    if policy in ("auto", "31"):
        target_frames = min(31, int(num_frames))
        vm_src = np.zeros(target_frames, dtype=bool)
        pos_src = np.zeros((target_frames, 2), dtype=np.float32)
        hdg_src = np.zeros(target_frames, dtype=np.float32)

        if ego_points:
            vm_src, pos_src, hdg_src = _resample_points(ego_points, target_frames, dt)
        elif ego_init is not None:
            vm_src, pos_src, hdg_src = _build_ego_fallback_track(
                ego_init=ego_init,
                goal=goal,
                num_frames=target_frames,
                dt=dt,
                mode=ego_mode,
            )
        else:
            idx = np.where(ego_valid_mask)[0]
            if idx.size > 0:
                j = int(idx[0])
                vm_src, pos_src, hdg_src = _build_single_point_track(
                    num_frames=target_frames,
                    x=float(ego_positions[j, 0]),
                    y=float(ego_positions[j, 1]),
                    heading=float(ego_headings[j]),
                )

        idx = np.where(vm_src)[0]
        if idx.size <= 0:
            if ego_init is not None:
                vm_src, pos_src, hdg_src = _build_single_point_track(
                    num_frames=target_frames,
                    x=float(ego_init.get("x", 0.0)),
                    y=float(ego_init.get("y", 0.0)),
                    heading=float(ego_init.get("heading", 0.0)),
                )
                idx = np.array([0], dtype=np.int64)
            else:
                return ego_valid_mask, ego_positions, ego_headings

        first = int(idx[0])
        cur_x = float(pos_src[first, 0])
        cur_y = float(pos_src[first, 1])
        cur_h = float(hdg_src[first])

        # 把前 target_frames 帧强制整理成连续有效轨迹（即使原始点有缺口）。
        vm = np.zeros(num_frames, dtype=bool)
        pos = np.zeros((num_frames, 2), dtype=np.float32)
        hdg = np.zeros(num_frames, dtype=np.float32)

        for i in range(target_frames):
            if i < target_frames and vm_src[i]:
                cur_x = float(pos_src[i, 0])
                cur_y = float(pos_src[i, 1])
                cur_h = float(hdg_src[i])
            vm[i] = True
            pos[i, 0] = cur_x
            pos[i, 1] = cur_y
            hdg[i] = np.mod(cur_h, 2 * np.pi)
        return vm, pos, hdg

    raise ValueError("Invalid ego_frames policy: %s (only 31 is supported)" % ego_frames)


def _convert_single_scene_from_xosc(scene_name, xosc_path, sim_freq=10, ego_mode="raw", ego_frames="31"):
    # 把单个 xosc 场景转换成 replay 所需的统一结构（ids/types/positions/valid_mask...）。
    with open(xosc_path, "r") as f:
        raw_text = f.read()
    root = ET.fromstring(raw_text)

    goal = _parse_goal(raw_text, root)
    if goal is None:
        raise ValueError("Cannot parse [Driving Task] comment: %s" % xosc_path)

    agents = _parse_entities(root)
    if not agents:
        raise ValueError("No ScenarioObject found: %s" % xosc_path)

    names = [a["name"] for a in agents]
    ego_name = _find_ego_name(names)
    if not ego_name:
        raise ValueError("No Ego object in xosc: %s" % xosc_path)
    ego_idx = names.index(ego_name)

    traj = _parse_trajectories(root, names)
    ego_init = _parse_ego_initial_state(raw_text)

    max_time = 0.0
    for name in names:
        points = traj.get(name, [])
        if points:
            max_time = max(max_time, points[-1][0])
    # 优先对齐 StopTrigger 时长：
    # 某些场景的轨迹点会被截断，但仿真总时长仍应与原始场景一致。
    stop_time = _parse_stop_trigger_time(root)
    if stop_time is not None:
        max_time = max(max_time, float(stop_time))

    dt = 1.0 / float(sim_freq)
    num_frames = int(max_time * sim_freq + 1e-6) + 1
    # 31 帧策略下，至少保证可回放 31 帧，避免 warmup/接管前提前耗尽。
    if str(ego_frames).strip().lower() == "31":
        num_frames = max(num_frames, 31)
    num_frames = max(1, num_frames)

    num_agents = len(agents)
    ids = [None] * num_agents
    types = np.zeros(num_agents, dtype=np.uint8)
    shapes = np.zeros((num_agents, 2), dtype=np.float32)
    predict_mask = np.zeros(num_agents, dtype=bool)
    valid_mask = np.zeros((num_agents, num_frames), dtype=bool)
    positions = np.zeros((num_agents, num_frames, 2), dtype=np.float32)
    headings = np.zeros((num_agents, num_frames), dtype=np.float32)

    # 逐 agent 生成统一时序张量；背景车后续由 planner 预测掩码决定是否参与预测。
    for agent in agents:
        idx = agent["idx"]
        name = agent["name"]
        is_ego = (name == ego_name)
        ids[idx] = "Ego" if is_ego else name
        types[idx] = TYPE_TO_ID[agent["category"]]
        shapes[idx] = np.array([agent["length"], agent["width"]], dtype=np.float32)
        predict_mask[idx] = (not is_ego) and (types[idx] == TYPE_TO_ID["car"])

        points = traj.get(name, [])
        if points:
            vm, pos, hdg = _resample_points(points, num_frames, dt)
            valid_mask[idx] = vm
            positions[idx] = pos
            headings[idx] = hdg

    ego_valid_count = int(np.sum(valid_mask[ego_idx]))
    # Ego 没有可用轨迹时，使用注释区初始状态做兜底轨迹。
    if ego_valid_count <= 0:
        if ego_init is None:
            raise ValueError("Ego has no trajectory and no init comment: %s" % xosc_path)
        vm, pos, hdg = _build_ego_fallback_track(
            ego_init=ego_init,
            goal=goal,
            num_frames=num_frames,
            dt=dt,
            mode=ego_mode,
        )
        valid_mask[ego_idx] = vm
        positions[ego_idx] = pos
        headings[ego_idx] = hdg

    vm, pos, hdg = _force_ego_frame_policy(
        ego_points=traj.get(ego_name, []),
        ego_valid_mask=valid_mask[ego_idx],
        ego_positions=positions[ego_idx],
        ego_headings=headings[ego_idx],
        ego_init=ego_init,
        goal=goal,
        num_frames=num_frames,
        dt=dt,
        ego_mode=ego_mode,
        ego_frames=ego_frames,
    )
    valid_mask[ego_idx] = vm
    positions[ego_idx] = pos
    headings[ego_idx] = hdg

    return {
        "scene_name": scene_name,
        "sim_freq": int(sim_freq),
        "sim_duration": int(num_frames),
        "goal": [round(float(v), 6) for v in goal],
        "ids": ids,
        "types": types,
        "shapes": shapes,
        "predict_mask": predict_mask,
        "valid_mask": valid_mask,
        "positions": positions,
        "headings": headings,
    }


def _discover_raw_scene_dirs(raw_scene_dir, require_xodr=False):
    # 兼容两种选手提交结构：
    # 1) 平铺结构：raw_scene_dir/scenario_xxx.xosc
    # 2) 子目录结构：raw_scene_dir/scenario_xxx/scenario_xxx.xosc
    # 选手提交目录只强制要求 xosc；xodr 可以来自 GT 目录或公共 --map-xodr。
    # GT 目录如果通过 require_xodr=True 调用，则要求同一目录或同名文件存在 xodr。
    if not os.path.isdir(raw_scene_dir):
        return []

    scene_items = []
    for name in sorted(os.listdir(raw_scene_dir)):
        if name.startswith('.'):
            continue
        cur = os.path.join(raw_scene_dir, name)
        low = name.lower()

        # 平铺 xosc：/scenario/A/scenario_000a0e93.xosc
        if os.path.isfile(cur) and low.endswith('.xosc'):
            if require_xodr:
                stem = os.path.splitext(cur)[0]
                same_name_xodr = stem + '.xodr'
                has_xodr = os.path.exists(same_name_xodr) or any(
                    f.lower().endswith('.xodr') for f in os.listdir(raw_scene_dir)
                )
                if not has_xodr:
                    continue
            scene_items.append(cur)
            continue

        # 子目录：/scenario/A/scenario_000a0e93/scenario_000a0e93.xosc
        if os.path.isdir(cur):
            has_xosc = any(f.lower().endswith('.xosc') for f in os.listdir(cur))
            has_xodr = any(f.lower().endswith('.xodr') for f in os.listdir(cur))
            if has_xosc and ((not require_xodr) or has_xodr):
                scene_items.append(cur)

    return scene_items


def _find_first_xosc_xodr(scene_dir):
    # scene_dir 可以是：
    # - 场景目录：xxx/scenario_001/
    # - 平铺 xosc 文件：xxx/scenario_001.xosc
    xosc_path = ""
    xodr_path = ""

    if os.path.isfile(scene_dir):
        low = scene_dir.lower()
        if low.endswith('.xosc'):
            xosc_path = scene_dir
            same_name_xodr = os.path.splitext(scene_dir)[0] + '.xodr'
            if os.path.exists(same_name_xodr):
                xodr_path = same_name_xodr
            else:
                parent = os.path.dirname(scene_dir)
                if os.path.isdir(parent):
                    for f in sorted(os.listdir(parent)):
                        if f.lower().endswith('.xodr'):
                            xodr_path = os.path.join(parent, f)
                            break
        elif low.endswith('.xodr'):
            xodr_path = scene_dir
        return xosc_path, xodr_path

    if not os.path.isdir(scene_dir):
        return xosc_path, xodr_path

    for f in sorted(os.listdir(scene_dir)):
        low = f.lower()
        fp = os.path.join(scene_dir, f)
        if low.endswith(".xosc") and not xosc_path:
            xosc_path = fp
        if low.endswith(".xodr") and not xodr_path:
            xodr_path = fp
    return xosc_path, xodr_path


def _find_first_xodr_under(root_dir):
    # 从 GT 根目录或 split 目录中寻找公共 xodr；地图相同时可复用这一份。
    if not root_dir or not os.path.isdir(root_dir):
        return ""
    for cur_root, dirnames, filenames in os.walk(root_dir):
        dirnames[:] = sorted([d for d in dirnames if not d.startswith(".")])
        for filename in sorted(filenames):
            if filename.lower().endswith(".xodr"):
                return os.path.join(cur_root, filename)
    return ""


def _resolve_map_xodr_for_scene(scene_name, gt_raw_scene_dir="", map_xodr_path=""):
    # xodr 的优先级：
    # 1) 显式传入的公共地图 --map-xodr；
    # 2) GT 同名场景目录下的 xodr；
    # 3) GT 平铺同名 xodr：gt_raw_scene_dir/scenario_xxx.xodr；
    # 4) GT 根目录中找到的第一份 xodr（适用于所有场景共用同一地图）。
    if map_xodr_path and os.path.exists(map_xodr_path):
        return os.path.abspath(map_xodr_path)

    if gt_raw_scene_dir and os.path.isdir(gt_raw_scene_dir):
        same_scene_dir = os.path.join(gt_raw_scene_dir, scene_name)
        if os.path.isdir(same_scene_dir):
            _xosc, xodr = _find_first_xosc_xodr(same_scene_dir)
            if xodr:
                return os.path.abspath(xodr)

        same_name_xodr = os.path.join(gt_raw_scene_dir, scene_name + '.xodr')
        if os.path.exists(same_name_xodr):
            return os.path.abspath(same_name_xodr)

        same_name_xosc = os.path.join(gt_raw_scene_dir, scene_name + '.xosc')
        if os.path.exists(same_name_xosc):
            _xosc, xodr = _find_first_xosc_xodr(same_name_xosc)
            if xodr:
                return os.path.abspath(xodr)

        xodr = _find_first_xodr_under(gt_raw_scene_dir)
        if xodr:
            return os.path.abspath(xodr)

    return ""


def _load_scene_info_from_raw_dir(
    scene_dir,
    scene_name,
    sim_freq=10,
    ego_mode="raw",
    ego_frames="31",
    require_xodr=False,
    fallback_xodr_path="",
):
    xosc_path, xodr_path = _find_first_xosc_xodr(scene_dir)
    if not xosc_path:
        raise RuntimeError("xosc missing in scene dir: %s" % scene_dir)

    if not xodr_path and fallback_xodr_path:
        xodr_path = os.path.abspath(fallback_xodr_path)

    if require_xodr and (not xodr_path or not os.path.exists(xodr_path)):
        raise RuntimeError("xodr missing in scene dir and no valid fallback map: %s" % scene_dir)

    scene_info = _convert_single_scene_from_xosc(
        scene_name=scene_name,
        xosc_path=xosc_path,
        sim_freq=sim_freq,
        ego_mode=ego_mode,
        ego_frames=ego_frames,
    )
    return scene_info, xosc_path, xodr_path


def _build_gt_info_for_realism(candidate_info, gt_scene_info):
    cand_ids = list(candidate_info.get("ids", []))
    gt_ids = list(gt_scene_info.get("ids", [])) if isinstance(gt_scene_info, dict) else []

    cand_predict = np.array(candidate_info.get("predict_mask", []), dtype=bool)
    cand_valid = np.array(candidate_info.get("valid_mask", []), copy=True)
    cand_pos = np.array(candidate_info.get("positions", []), copy=True)
    cand_heading = np.array(candidate_info.get("headings", []), copy=True)

    if not isinstance(gt_scene_info, dict):
        return {
            "predict_mask": cand_predict,
            "valid_mask": cand_valid,
            "positions": cand_pos,
            "headings": cand_heading,
            "note": "fallback_candidate_as_gt_missing_gt_scene",
        }

    gt_predict_raw = np.array(gt_scene_info.get("predict_mask", []), dtype=bool)
    gt_valid_raw = np.array(gt_scene_info.get("valid_mask", []), copy=True)
    gt_pos_raw = np.array(gt_scene_info.get("positions", []), copy=True)
    gt_heading_raw = np.array(gt_scene_info.get("headings", []), copy=True)

    if cand_ids == gt_ids and len(cand_ids) == gt_valid_raw.shape[0]:
        return {
            "predict_mask": gt_predict_raw,
            "valid_mask": gt_valid_raw,
            "positions": gt_pos_raw,
            "headings": gt_heading_raw,
            "note": "gt_direct_id_match",
        }

    gt_index = dict((name, idx) for idx, name in enumerate(gt_ids))
    if cand_ids and all(name in gt_index for name in cand_ids):
        idx = [gt_index[name] for name in cand_ids]
        gt_predict = gt_predict_raw[idx] if len(gt_predict_raw) == len(gt_ids) else cand_predict.copy()
        return {
            "predict_mask": np.array(gt_predict, dtype=bool),
            "valid_mask": gt_valid_raw[idx],
            "positions": gt_pos_raw[idx],
            "headings": gt_heading_raw[idx],
            "note": "gt_reordered_by_ids",
        }

    return {
        "predict_mask": cand_predict,
        "valid_mask": cand_valid,
        "positions": cand_pos,
        "headings": cand_heading,
        "note": "fallback_candidate_as_gt_due_to_id_mismatch",
    }


def _scene_has_ego_collision_before_frame(scene_info, frame_limit=31, min_time_sec=0.5):
    """
    检查场景中是否存在“主车在给定帧阈值前发生碰撞”。
    - 碰撞检测起算时间：t > min_time_sec（与回放主逻辑保持一致的0.5s保护）
    - 仅用于原始场景预过滤，不会改写原始轨迹数据。
    """
    try:
        from utils.functions import detectCollision
        from utils.observation import EgoStatus, ObjectStatus
    except Exception:
        return False, -1

    ids = list(scene_info.get("ids", []))
    if "Ego" not in ids:
        return False, -1
    ego_idx = ids.index("Ego")

    valid_mask = scene_info.get("valid_mask")
    positions = scene_info.get("positions")
    headings = scene_info.get("headings")
    shapes = scene_info.get("shapes")
    types = scene_info.get("types")
    if valid_mask is None or positions is None or headings is None or shapes is None or types is None:
        return False, -1

    try:
        nt = int(valid_mask.shape[1])
        na = int(valid_mask.shape[0])
    except Exception:
        return False, -1

    upper = min(int(frame_limit), nt)
    if upper <= 0:
        return False, -1

    dt = round(1.0 / float(scene_info.get("sim_freq", 10)), 3)
    type_to_category = {0: "vehicle", 1: "pedestrian", 2: "bicycle"}

    for t_idx in range(upper):
        cur_t = float(t_idx) * dt
        if not (cur_t > float(min_time_sec)):
            continue
        if not bool(valid_mask[ego_idx, t_idx]):
            continue

        ego = EgoStatus(
            x=float(positions[ego_idx, t_idx, 0]),
            y=float(positions[ego_idx, t_idx, 1]),
            yaw=float(headings[ego_idx, t_idx]),
            length=float(shapes[ego_idx, 0]),
            width=float(shapes[ego_idx, 1]),
            v=0.0,
            a=0.0,
            rot=0.0,
        )
        object_info = {"vehicle": {}, "bicycle": {}, "pedestrian": {}}

        for a_idx in range(na):
            if a_idx == ego_idx:
                continue
            if not bool(valid_mask[a_idx, t_idx]):
                continue
            category = type_to_category.get(int(types[a_idx]), "vehicle")
            obj_name = str(ids[a_idx]) if a_idx < len(ids) else str(a_idx)
            object_info[category][obj_name] = ObjectStatus(
                x=float(positions[a_idx, t_idx, 0]),
                y=float(positions[a_idx, t_idx, 1]),
                yaw=float(headings[a_idx, t_idx]),
                length=float(shapes[a_idx, 0]),
                width=float(shapes[a_idx, 1]),
                v=0.0,
                a=0.0,
            )

        if detectCollision(ego, object_info):
            return True, int(t_idx)

    return False, -1


def _load_raw_tasks(raw_scene_dir, warmup, scene_limit=0, ego_mode="raw", ego_frames="31", gt_raw_scene_dir="", map_xodr_path=""):
    from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
    from utils.ScenarioManager.ScenarioManagerForISG import ScenarioManagerForISG

    # 全量加载并预过滤场景，过滤原因会写入 prefilter_rows 供诊断。
    scene_dirs = _discover_raw_scene_dirs(raw_scene_dir, require_xodr=False)
    if scene_limit and scene_limit > 0:
        scene_dirs = scene_dirs[:scene_limit]

    tasks = []
    prefilter_rows = []
    removed_short_lt31_count = 0
    removed_collision_pre31_count = 0
    removed_scene_parse_failed_count = 0
    removed_missing_gt_scene_count = 0
    removed_invalid_gt_scene_count = 0
    for idx, scene_dir in enumerate(scene_dirs):
        # scene_dir 可能是场景目录，也可能是平铺的 .xosc 文件。
        # 平铺文件使用文件名去掉 .xosc 作为 scene_id。
        if os.path.isfile(scene_dir) and scene_dir.lower().endswith('.xosc'):
            scene_name = os.path.splitext(os.path.basename(scene_dir))[0]
        else:
            scene_name = os.path.basename(scene_dir.rstrip(os.sep))
        try:
            candidate_map_xodr = _resolve_map_xodr_for_scene(
                scene_name=scene_name,
                gt_raw_scene_dir=gt_raw_scene_dir,
                map_xodr_path=map_xodr_path,
            )
            scene_info, xosc_path, xodr_path = _load_scene_info_from_raw_dir(
                scene_dir=scene_dir,
                scene_name=scene_name,
                sim_freq=10,
                ego_mode=ego_mode,
                ego_frames=ego_frames,
                require_xodr=True,
                fallback_xodr_path=candidate_map_xodr,
            )
        except Exception as e:
            removed_scene_parse_failed_count += 1
            prefilter_rows.append(
                {
                    "Scene ID": scene_name,
                    "Drop Reason": "scene_parse_failed",
                    "Sim Duration Frames": "",
                    "Extra": repr(e),
                }
            )
            continue

        sim_duration = int(scene_info.get("sim_duration", 0))
        # 规则1：场景总帧数不足 31，直接剔除（无法满足固定 warmup 策略）。
        if sim_duration < 31:
            removed_short_lt31_count += 1
            prefilter_rows.append(
                {
                    "Scene ID": scene_name,
                    "Drop Reason": "scene_duration_lt31",
                    "Sim Duration Frames": sim_duration,
                    "Extra": "",
                }
            )
            continue

        # 规则2：主车在 31 帧内已与背景发生碰撞，直接剔除。
        has_collision_pre31, collision_frame = _scene_has_ego_collision_before_frame(
            scene_info,
            frame_limit=31,
            min_time_sec=0.5,
        )
        if has_collision_pre31:
            removed_collision_pre31_count += 1
            prefilter_rows.append(
                {
                    "Scene ID": scene_name,
                    "Drop Reason": "ego_collision_before_31",
                    "Sim Duration Frames": sim_duration,
                    "Extra": "collision_frame=%s" % int(collision_frame),
                }
            )
            continue

        gt_info_pack = None
        # 若提供 GT 目录，按同名场景建立 GT 对齐信息用于 realism 指标。
        if gt_raw_scene_dir:
            gt_scene_dir = os.path.join(gt_raw_scene_dir, scene_name)
            gt_scene_xosc = os.path.join(gt_raw_scene_dir, scene_name + '.xosc')
            if os.path.isdir(gt_scene_dir):
                gt_scene_item = gt_scene_dir
            elif os.path.exists(gt_scene_xosc):
                gt_scene_item = gt_scene_xosc
            else:
                removed_missing_gt_scene_count += 1
                prefilter_rows.append(
                    {
                        "Scene ID": scene_name,
                        "Drop Reason": "missing_gt_scene",
                        "Sim Duration Frames": sim_duration,
                        "Extra": gt_scene_dir + " or " + gt_scene_xosc,
                    }
                )
                continue
            try:
                gt_scene_info, _gt_xosc, _gt_xodr = _load_scene_info_from_raw_dir(
                    scene_dir=gt_scene_item,
                    scene_name=scene_name,
                    sim_freq=10,
                    ego_mode=ego_mode,
                    ego_frames=ego_frames,
                    require_xodr=True,
                    fallback_xodr_path=map_xodr_path,
                )
                gt_info_pack = _build_gt_info_for_realism(scene_info, gt_scene_info)
            except Exception as e:
                removed_invalid_gt_scene_count += 1
                prefilter_rows.append(
                    {
                        "Scene ID": scene_name,
                        "Drop Reason": "gt_scene_parse_failed",
                        "Sim Duration Frames": sim_duration,
                        "Extra": repr(e),
                    }
                )
                continue
        else:
            gt_info_pack = _build_gt_info_for_realism(scene_info, scene_info)

        valid_mask = scene_info['valid_mask'].copy()
        positions = scene_info['positions'].copy()
        headings = scene_info['headings'].copy()
        velocities, accelerations, vector_velocities = ScenarioManagerForISG._calculate_velocity_acceleration(
            valid_mask,
            positions,
            dt=round(1.0 / float(scene_info['sim_freq']), 3),
        )

        ego_idx = scene_info['ids'].index('Ego')
        # warmup 表示“从第 warmup 帧开始评测”，对应 0-based 索引为 warmup-1。
        # 这里会对边界做裁剪，避免 warmup 超过场景长度导致越界。
        start_idx = max(0, min(int(warmup) - 1, int(scene_info['sim_duration']) - 1))
        task = ScenarioInfo(
            num=idx,
            name=scene_name,
            type='REPLAY',
            source_file={
                'xodr': xodr_path,
                'xosc': xosc_path,
                'pkl': '',
                'gt': '',
                'json': '',
            },
            task_info={
                'startPos': scene_info['positions'][ego_idx, start_idx].tolist(),
                'targetPos': [[scene_info['goal'][0], scene_info['goal'][2]], [scene_info['goal'][1], scene_info['goal'][3]]],
                'waypoints': [],
                'dt': round(1.0 / float(scene_info['sim_freq']), 3),
            },
            additional_info={
                'ids': scene_info['ids'],
                'types': scene_info['types'],
                'shapes': scene_info['shapes'],
                'predict_mask': scene_info['predict_mask'],
                'valid_mask': valid_mask,
                'positions': positions,
                'headings': headings,
                'velocities': velocities,
                'accelerations': accelerations,
                'vector_velocities': vector_velocities,
                'gt_info': {
                    'predict_mask': np.array(gt_info_pack.get('predict_mask', []), copy=True),
                    'valid_mask': np.array(gt_info_pack.get('valid_mask', []), copy=True),
                    'positions': np.array(gt_info_pack.get('positions', []), copy=True),
                    'headings': np.array(gt_info_pack.get('headings', []), copy=True),
                    'note': str(gt_info_pack.get('note', '')),
                },
            }
        )
        tasks.append(task)

    class _InMemoryScenarioManager(object):
        def __init__(self, tasks):
            self.tasks = tasks
            self.tot_scene_num = len(tasks)

        @staticmethod
        def _calculate_velocity_acceleration(valid_mask, positions, dt=0.1):
            return ScenarioManagerForISG._calculate_velocity_acceleration(valid_mask, positions, dt)

    manager = _InMemoryScenarioManager(tasks)
    manager.prefilter_rows = prefilter_rows
    # 统计信息用于后续写入 run_summary / 诊断 CSV。
    manager.load_stats = {
        "input_scene_total": len(scene_dirs),
        "prefilter_removed_short_lt31_count": int(removed_short_lt31_count),
        "prefilter_removed_pre31_collision_count": int(removed_collision_pre31_count),
        "prefilter_removed_scene_parse_failed_count": int(removed_scene_parse_failed_count),
        "prefilter_removed_missing_gt_scene_count": int(removed_missing_gt_scene_count),
        "prefilter_removed_invalid_gt_scene_count": int(removed_invalid_gt_scene_count),
        "prefilter_removed_total": int(
            removed_scene_parse_failed_count
            + removed_short_lt31_count
            + removed_collision_pre31_count
            + removed_missing_gt_scene_count
            + removed_invalid_gt_scene_count
        ),
        "prefilter_kept_count": int(len(tasks)),
    }
    return manager


def _parse_planner_indices(value):
    if value is None or value == "":
        return None
    parts = [x.strip() for x in value.split(",") if x.strip()]
    if not parts:
        return None
    return [int(x) for x in parts]


def _parse_splits(value):
    if value is None or str(value).strip() == "":
        return []
    out = []
    for part in str(value).split(","):
        split = str(part).strip().upper()
        if split in ("A", "B", "C") and split not in out:
            out.append(split)
    return out


def _discover_scene_names(scenario_dir):
    scene_names = []
    for filename in sorted(os.listdir(scenario_dir)):
        if filename.startswith("."):
            continue
        if filename.endswith("_output.pkl"):
            scene_names.append(filename[:-len("_output.pkl")])
    return scene_names


def _load_single_raw_task(raw_scene_dir, scene_name, warmup, ego_mode="raw", ego_frames="31", gt_raw_scene_dir="", map_xodr_path=""):
    from utils.ScenarioManager.ScenarioInfo import ScenarioInfo
    from utils.ScenarioManager.ScenarioManagerForISG import ScenarioManagerForISG

    # 兼容：
    # 1) raw_scene_dir/scene_name/scene_name.xosc
    # 2) raw_scene_dir/scene_name.xosc
    scene_dir = os.path.join(raw_scene_dir, scene_name)
    scene_xosc = os.path.join(raw_scene_dir, scene_name + '.xosc')
    if os.path.isdir(scene_dir):
        scene_item = scene_dir
    elif os.path.exists(scene_xosc):
        scene_item = scene_xosc
    else:
        raise RuntimeError("Raw scene not found: %s or %s" % (scene_dir, scene_xosc))

    # 从选手 xosc 构造场景；xodr 可以来自选手目录、GT 同名目录或公共 --map-xodr。
    candidate_map_xodr = _resolve_map_xodr_for_scene(
        scene_name=scene_name,
        gt_raw_scene_dir=gt_raw_scene_dir,
        map_xodr_path=map_xodr_path,
    )
    scene_info, xosc_path, xodr_path = _load_scene_info_from_raw_dir(
        scene_dir=scene_item,
        scene_name=scene_name,
        sim_freq=10,
        ego_mode=ego_mode,
        ego_frames=ego_frames,
        require_xodr=True,
        fallback_xodr_path=candidate_map_xodr,
    )

    if gt_raw_scene_dir:
        gt_scene_dir = os.path.join(gt_raw_scene_dir, scene_name)
        gt_scene_xosc = os.path.join(gt_raw_scene_dir, scene_name + '.xosc')
        if os.path.isdir(gt_scene_dir):
            gt_scene_item = gt_scene_dir
        elif os.path.exists(gt_scene_xosc):
            gt_scene_item = gt_scene_xosc
        else:
            raise RuntimeError("missing_gt_scene: %s or %s" % (gt_scene_dir, gt_scene_xosc))
        gt_scene_info, _gt_xosc, _gt_xodr = _load_scene_info_from_raw_dir(
            scene_dir=gt_scene_item,
            scene_name=scene_name,
            sim_freq=10,
            ego_mode=ego_mode,
            ego_frames=ego_frames,
            require_xodr=True,
            fallback_xodr_path=map_xodr_path,
        )
        gt_info_pack = _build_gt_info_for_realism(scene_info, gt_scene_info)
    else:
        gt_info_pack = _build_gt_info_for_realism(scene_info, scene_info)

    valid_mask = scene_info['valid_mask'].copy()
    positions = scene_info['positions'].copy()
    headings = scene_info['headings'].copy()
    velocities, accelerations, vector_velocities = ScenarioManagerForISG._calculate_velocity_acceleration(
        valid_mask,
        positions,
        dt=round(1.0 / float(scene_info['sim_freq']), 3),
    )

    ego_idx = scene_info['ids'].index('Ego')
    start_idx = max(0, min(int(warmup) - 1, int(scene_info['sim_duration']) - 1))
    return ScenarioInfo(
        num=0,
        name=scene_name,
        type='REPLAY',
        source_file={
            'xodr': xodr_path,
            'xosc': xosc_path,
            'pkl': '',
            'gt': '',
            'json': '',
        },
        task_info={
            'startPos': scene_info['positions'][ego_idx, start_idx].tolist(),
            'targetPos': [[scene_info['goal'][0], scene_info['goal'][2]], [scene_info['goal'][1], scene_info['goal'][3]]],
            'waypoints': [],
            'dt': round(1.0 / float(scene_info['sim_freq']), 3),
        },
        additional_info={
            'ids': scene_info['ids'],
            'types': scene_info['types'],
            'shapes': scene_info['shapes'],
            'predict_mask': scene_info['predict_mask'],
            'valid_mask': valid_mask,
            'positions': positions,
            'headings': headings,
            'velocities': velocities,
            'accelerations': accelerations,
            'vector_velocities': vector_velocities,
            'gt_info': {
                'predict_mask': np.array(gt_info_pack.get('predict_mask', []), copy=True),
                'valid_mask': np.array(gt_info_pack.get('valid_mask', []), copy=True),
                'positions': np.array(gt_info_pack.get('positions', []), copy=True),
                'headings': np.array(gt_info_pack.get('headings', []), copy=True),
                'note': str(gt_info_pack.get('note', '')),
            },
        }
    )


def _run_scene_worker(args):
    """子进程单场景执行单元：加载场景 -> 多planner评测 -> 返回场景结果块。"""
    (
        scenario_dir,
        gt_dir,
        raw_scene_dir,
        gt_raw_scene_dir,
        use_raw_mode,
        output_dir,
        scene_name,
        warmup,
        skip_exist,
        planner_jobs,
        submitid,
        tested_scene_save_dir,
        ego_mode,
        ego_frames,
        keep_replay_csv,
        map_xodr_path,
    ) = args

    # Tester is defined in this file. Do not import replay_test_for_pkl here;
    # that legacy file can be safely removed after this patch.
    from planner import PLANNERS
    from utils.ScenarioManager.ScenarioManagerForISG import ScenarioManagerForISG
    from evaluation import run_evaluator

    config = {
        "tasks": [scene_name],
        "warmup": int(warmup),
        "skipExist": bool(skip_exist),
        "visualize": False,
        "keep_replay_csv": bool(keep_replay_csv),
    }

    if use_raw_mode:
        task = _load_single_raw_task(
            raw_scene_dir=raw_scene_dir,
            scene_name=scene_name,
            warmup=config["warmup"],
            ego_mode=ego_mode,
            ego_frames=ego_frames,
            gt_raw_scene_dir=gt_raw_scene_dir,
            map_xodr_path=map_xodr_path,
        )

        class _SingleTaskSM(object):
            def __init__(self, single_task):
                self.tasks = [single_task]
                self.tot_scene_num = 1

        sm = _SingleTaskSM(task)
    else:
        sm = ScenarioManagerForISG(scenario_dir, gt_dir, config)
        sm.tasks = [s for s in sm.tasks if s.name == scene_name]
        sm.tot_scene_num = len(sm.tasks)

    tester = Tester(scenario_dir, output_dir)

    replay_result = {}
    planner_errors = []
    tested_scene_saved_count = 0
    tested_scene_names = []
    saved_scene_names = []

    for scene in sm.tasks:
        scene_planner_result = {}
        scene_has_error = False
        for planner_idx in planner_jobs:
            cur_planner = PLANNERS[planner_idx]
            try:
                output_name = f"{scene.type}_{scene.num}_av{planner_idx}_{scene.name}_result.csv"
                output_path = os.path.join(output_dir, output_name)
                if not (config["skipExist"] and os.path.exists(output_path)):
                    tester.replay_test(config, scene, cur_planner, output_path)

                result = run_evaluator(map_file=scene.source_file["xodr"], csv_file_path=output_path)
                ability = {
                    "safe": result["AbilityDimension"]["safe"],
                    "efficiency": result["AbilityDimension"]["efficiency"],
                    "comfortable": result["AbilityDimension"]["comfortable"],
                }
                scene_planner_result[f"planner_{planner_idx}"] = ability
                if not config.get("keep_replay_csv", False) and os.path.exists(output_path):
                    try:
                        os.remove(output_path)
                    except Exception:
                        pass
            except Exception as e:
                scene_has_error = True
                tester.logger.error(
                    f"[{submitid}] pid:{os.getpid()} | Planner:{planner_idx}-{cur_planner.__name__} in {scene.num}-{scene.name} 发生错误: {repr(e)}"
                )
                # 兜底写0分：保证每个场景都有 planner_* 字段，避免 score.csv 全部缺列或全空
                scene_planner_result[f"planner_{planner_idx}"] = {
                    "safe": 0.0,
                    "efficiency": 0.0,
                    "comfortable": 0.0,
                }
                planner_errors.append(
                    {
                        "Scene ID": scene.name,
                        "Planner Idx": planner_idx,
                        "Planner Name": cur_planner.__name__,
                        "Error": repr(e),
                    }
                )

        if scene_planner_result:
            replay_result[scene.name] = scene_planner_result
            tested_scene_names.append(scene.name)
            if not scene_has_error:
                saved = _save_tested_scene_sources(scene.name, scene.source_file, tested_scene_save_dir)
                if saved:
                    tested_scene_saved_count += 1
                    saved_scene_names.append(scene.name)

    return {
        "scene_name": scene_name,
        "replay_result": replay_result,
        "planner_errors": planner_errors,
        "tested_scene_saved_count": tested_scene_saved_count,
        "tested_scene_names": tested_scene_names,
        "saved_scene_names": saved_scene_names,
    }


def _run_scene_worker_process(args, conn):
    # args[6] 才是 scene_name，args[5] 是 output_dir。
    scene_name = args[6]
    try:
        result = _run_scene_worker(args)
        result["ok"] = True
        result["error"] = ""
    except Exception as e:
        result = {
            "scene_name": scene_name,
            "replay_result": {},
            "ok": False,
            "error": repr(e),
        }
    try:
        conn.send(result)
    except Exception:
        pass
    finally:
        conn.close()


def _write_out_dynamic_diagnosis(output_dir, dynamic_results, dynamic_policy):
    score_dir = os.path.join(output_dir, "score")
    os.makedirs(score_dir, exist_ok=True)

    rows = []
    for scene_id in sorted(dynamic_results.keys()):
        item = dynamic_results.get(scene_id, {})
        if not isinstance(item, dict):
            item = {}
        rows.append(
            {
                "Scene ID": scene_id,
                "Is Out Dynamic": int(item.get("is_out_dynamic", 0)),
                "Reason Type": item.get("dynamic_reason_type", ""),
                "Reason Agent": item.get("dynamic_reason_agent", ""),
                "Reason Min": item.get("dynamic_reason_min", ""),
                "Reason Max": item.get("dynamic_reason_max", ""),
                "Acc Violation": int(item.get("dynamic_acc_violation", 0)),
                "Heading Violation": int(item.get("dynamic_heading_violation", 0)),
                "Invalid Mask Violation": int(item.get("dynamic_invalid_mask_violation", 0)),
                "Continuity Violation": int(item.get("dynamic_continuity_violation", 0)),
                "Reverse Violation": int(item.get("dynamic_reverse_violation", 0)),
            }
        )

    csv_path = os.path.join(score_dir, "out_dynamic_diagnosis.csv")
    out_count = sum(int(r["Is Out Dynamic"]) for r in rows)

    fieldnames = [
        "Scene ID",
        "Is Out Dynamic",
        "Reason Type",
        "Reason Agent",
        "Reason Min",
        "Reason Max",
        "Acc Violation",
        "Heading Violation",
        "Invalid Mask Violation",
        "Continuity Violation",
        "Reverse Violation",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    return "", csv_path, len(rows), out_count


def _write_planner_error_diagnosis(output_dir, planner_error_rows):
    score_dir = os.path.join(output_dir, "score")
    os.makedirs(score_dir, exist_ok=True)

    csv_path = os.path.join(score_dir, "planner_error_diagnosis.csv")
    fieldnames = ["Scene ID", "Planner Idx", "Planner Name", "Error"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in planner_error_rows:
            writer.writerow(
                {
                    "Scene ID": row.get("Scene ID", ""),
                    "Planner Idx": row.get("Planner Idx", ""),
                    "Planner Name": row.get("Planner Name", ""),
                    "Error": row.get("Error", ""),
                }
            )

    return csv_path


def _write_ego_test_error_scene_stats(output_dir, error_scene_rows):
    score_dir = os.path.join(output_dir, "score")
    os.makedirs(score_dir, exist_ok=True)

    csv_path = os.path.join(score_dir, "ego_test_error_scenes.csv")
    fieldnames = ["Scene ID", "Error Stage", "Error"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in error_scene_rows:
            writer.writerow(
                {
                    "Scene ID": row.get("Scene ID", ""),
                    "Error Stage": row.get("Error Stage", ""),
                    "Error": row.get("Error", ""),
                }
            )

    return csv_path


def _write_prefilter_scene_diagnosis(output_dir, prefilter_rows):
    score_dir = os.path.join(output_dir, "score")
    os.makedirs(score_dir, exist_ok=True)

    csv_path = os.path.join(score_dir, "prefilter_scene_diagnosis.csv")
    fieldnames = ["Scene ID", "Drop Reason", "Sim Duration Frames", "Extra"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in prefilter_rows:
            writer.writerow(
                {
                    "Scene ID": row.get("Scene ID", ""),
                    "Drop Reason": row.get("Drop Reason", ""),
                    "Sim Duration Frames": row.get("Sim Duration Frames", ""),
                    "Extra": row.get("Extra", ""),
                }
            )
    return csv_path


def _write_key_run_stats_csv(output_dir, run_summary):
    """
    输出关键统计CSV（单行），用于快速查看本次赛题回放核心结果。
    """
    score_dir = os.path.join(output_dir, "score")
    os.makedirs(score_dir, exist_ok=True)
    csv_path = os.path.join(score_dir, "key_run_stats.csv")

    row = {
        "Total Scenes": int(run_summary.get("folder_scene_total", 0)),
        "Out Dynamic Scenes": int(run_summary.get("out_dynamic_positive_count", 0)),
        "Worker Timeout Scenes": int(run_summary.get("worker_timeout_scene_count", 0)),
        "Scene Duration <31 Frames": int(run_summary.get("prefilter_removed_short_lt31_count", 0)),
        "Collision Before 31 Frames": int(run_summary.get("prefilter_removed_pre31_collision_count", 0)),
        "Ego Tested & Scored Scenes": int(run_summary.get("saved_scene_count", run_summary.get("tested_scene_saved_count", 0))),
    }

    fieldnames = [
        "Total Scenes",
        "Out Dynamic Scenes",
        "Worker Timeout Scenes",
        "Scene Duration <31 Frames",
        "Collision Before 31 Frames",
        "Ego Tested & Scored Scenes",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)
    return csv_path


def _save_tested_scene_sources(scene_name, scene_source, save_root_dir):
    if not save_root_dir:
        return False
    if not isinstance(scene_source, dict):
        return False

    xodr_src = scene_source.get("xodr", "")
    xosc_src = scene_source.get("xosc", "")

    if (not xodr_src or not os.path.exists(xodr_src)) and (not xosc_src or not os.path.exists(xosc_src)):
        return False

    scene_out_dir = os.path.join(save_root_dir, scene_name)
    os.makedirs(scene_out_dir, exist_ok=True)

    copied_any = False
    if xodr_src and os.path.exists(xodr_src):
        shutil.copy2(xodr_src, os.path.join(scene_out_dir, f"{scene_name}.xodr"))
        copied_any = True
    if xosc_src and os.path.exists(xosc_src):
        shutil.copy2(xosc_src, os.path.join(scene_out_dir, f"{scene_name}.xosc"))
        copied_any = True
    return copied_any


def _infer_competition_tag_from_output_dir(output_dir):
    path = os.path.abspath(output_dir).replace("\\", "/")
    for part in path.split("/"):
        up = part.upper()
        if up in {"A", "B", "C"}:
            return "赛题" + up
    m = re.search(r"(?:^|[_\-/])(A|B|C)(?:$|[_\-/])", path, flags=re.I)
    if m:
        return "赛题" + m.group(1).upper()
    return "赛题A"


def _try_plot_planner_avg_density(output_dir):
    def _write_placeholder_png(out_png_path, reason_text):
        """兜底占位图：即使无有效样本也保证产出png，便于流水线稳定。"""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(6, 4))
            ax.set_facecolor("#ffffff")
            fig.patch.set_facecolor("#ffffff")
            ax.axis("off")
            ax.text(
                0.5,
                0.62,
                "Planner Avg Score Density",
                ha="center",
                va="center",
                fontsize=13,
                color="#1f2937",
            )
            ax.text(
                0.5,
                0.48,
                str(reason_text),
                ha="center",
                va="center",
                fontsize=10,
                color="#6b7280",
            )
            out_dir = os.path.dirname(out_png_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            fig.savefig(out_png_path, dpi=180, facecolor="#ffffff", edgecolor="#ffffff")
            plt.close(fig)
            print("[INFO] planner avg density fallback png:", out_png_path)
            return out_png_path
        except Exception as ee:
            print("[WARNING] failed to generate fallback density png:", repr(ee))
            return ""

    score_csv = os.path.join(output_dir, "score", "score.csv")
    if not os.path.exists(score_csv):
        print("[WARNING] score.csv not found, skip density plot:", score_csv)
        return ""

    plot_script_path = os.path.join(os.path.dirname(__file__), "plot_planner_avg_score_density.py")
    if not os.path.exists(plot_script_path):
        print("[WARNING] plot script not found, skip density plot:", plot_script_path)
        return ""

    out_png = os.path.join(output_dir, "score", "planner_avg_score_density.png")
    tag = _infer_competition_tag_from_output_dir(output_dir)
    try:
        spec = importlib.util.spec_from_file_location("plot_planner_avg_score_density", plot_script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("failed loading plot module")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        used_font = mod.setup_chinese_font("", "")
        values = mod.load_scores(score_csv, only_tested=True)
        if values.size <= 0:
            return _write_placeholder_png(out_png, "No valid non-zero planner scores in score.csv")

        plot_kwargs = {
            "values": values,
            "output_path": out_png,
            "title": "",
            "x_label": "主车测试得分",
            "y_label": "概率密度",
            "tag_text": tag,
            "x_min": 0.0,
            "x_max": 100.0,
            "x_step": 50.0,
            "y_top": 0.03,
            "y_step": 0.005,
            # 兼容较新版本绘图脚本的主题参数；若旧版不支持，下方会自动重试无该参数调用。
            "theme": "ocean",
        }
        try:
            mod.plot_density(**plot_kwargs)
        except TypeError:
            plot_kwargs.pop("theme", None)
            mod.plot_density(**plot_kwargs)

        print("[INFO] planner avg density png:", out_png)
        print("[INFO] density samples_after_filter:", int(values.size))
        if used_font:
            print("[INFO] density font_used:", used_font)
        return out_png
    except Exception as e:
        print("[WARNING] failed to generate planner avg density:", repr(e))
        return _write_placeholder_png(out_png, "Density plot failed; fallback placeholder generated")

def _resolve_output_dir_arg(raw_scene_dir, output_dir_arg):
    """
    解析输出目录：
    - 显式传入 --output-dir 时严格使用；
    - 未传时，若 raw_scene_dir 形如 .../ego_completed_31/<split>，自动映射到 .../output31/<split>；
    - 其余情况使用默认 outputs_2026A。
    """
    explicit = str(output_dir_arg or "").strip()
    if explicit:
        return os.path.abspath(explicit)

    default_dir = os.path.join(ROOT_DIR, "new_workflow", "outputs_2026A")
    raw = str(raw_scene_dir or "").strip()
    if not raw:
        return os.path.abspath(default_dir)

    raw_abs = os.path.abspath(raw.rstrip(os.sep))
    split_name = os.path.basename(raw_abs)
    parent_name = os.path.basename(os.path.dirname(raw_abs))
    if parent_name.lower() == "ego_completed_31":
        return os.path.join(os.path.dirname(raw_abs), "output31", split_name)

    return os.path.abspath(default_dir)


def _resolve_batch_output_dir(output_dir_arg, split_raw_scene_dir, split):
    explicit = str(output_dir_arg or "").strip()
    if not explicit:
        base = _resolve_output_dir_arg(split_raw_scene_dir, "")
        base_leaf = os.path.basename(base.rstrip(os.sep)).upper()
        if base_leaf in ("A", "B", "C"):
            return os.path.join(os.path.dirname(base), split)
        return os.path.join(base, split)

    out_abs = os.path.abspath(explicit.rstrip(os.sep))
    out_leaf = os.path.basename(out_abs).upper()
    if out_leaf in ("A", "B", "C"):
        return os.path.join(os.path.dirname(out_abs), split)
    return os.path.join(out_abs, split)


def _resolve_batch_tested_scene_save_dir(tested_scene_save_dir, split):
    base = str(tested_scene_save_dir or "").strip()
    if not base:
        return ""

    base_abs = os.path.abspath(base.rstrip(os.sep))
    base_leaf = os.path.basename(base_abs).upper()
    if base_leaf in ("A", "B", "C"):
        return os.path.join(os.path.dirname(base_abs), split)
    return os.path.join(base_abs, split)


def _resolve_batch_output_root_dir(output_dir_arg, raw_root):
    explicit = str(output_dir_arg or "").strip()
    if explicit:
        out_abs = os.path.abspath(explicit.rstrip(os.sep))
        out_leaf = os.path.basename(out_abs).upper()
        if out_leaf in ("A", "B", "C"):
            return os.path.dirname(out_abs)
        return out_abs

    # No explicit output root: derive from split-style default and then strip split leaf if needed.
    sample_split_raw = os.path.join(raw_root, "A")
    base = _resolve_output_dir_arg(sample_split_raw, "")
    base_leaf = os.path.basename(base.rstrip(os.sep)).upper()
    if base_leaf in ("A", "B", "C"):
        return os.path.dirname(base)
    return base


def _infer_topic_tag(raw_scene_dir="", gt_raw_scene_dir="", score_split="", scenario_dir=""):
    """
    推断赛题标签（A/B/C）：
    1) 优先使用 --score-split；
    2) 再看 raw_scene_dir / gt_raw_scene_dir / scenario_dir 的末级目录；
    3) 最后回退为 A。
    """
    split = str(score_split or "").strip().upper()
    if split in ("A", "B", "C"):
        return split

    candidates = [raw_scene_dir, gt_raw_scene_dir, scenario_dir]
    for p in candidates:
        s = str(p or "").strip()
        if not s:
            continue
        leaf = os.path.basename(os.path.abspath(s.rstrip(os.sep))).upper()
        if leaf in ("A", "B", "C"):
            return leaf

    for p in candidates:
        s = os.path.abspath(str(p or "")).replace("\\", "/")
        m = re.search(r"(?:^|/)(A|B|C)(?:/|$)", s, flags=re.I)
        if m:
            return m.group(1).upper()

    return "A"


def _read_score_csv_rows(score_csv_path):
    fieldnames = []
    scene_rows = []
    avg_row = {}
    if not os.path.exists(score_csv_path):
        return fieldnames, scene_rows, avg_row

    with open(score_csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        for row in reader:
            scene_id = str(row.get("Scene ID", ""))
            if scene_id == "Average":
                avg_row = dict(row)
            else:
                scene_rows.append(dict(row))
    return fieldnames, scene_rows, avg_row


def _write_rows_csv(rows, csv_path, fieldnames):
    out_dir = os.path.dirname(os.path.abspath(csv_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def run_replay(
    workspace_dir,
    output_dir,
    raw_scene_dir="",
    gt_raw_scene_dir="",
    map_xodr_path="",
    warmup=31,
    scene_limit=0,
    planner_indices=None,
    num_workers=1,
    scene_timeout_sec=1800,
    skip_exist=False,
    submitid="onsite2026",
    tested_scene_save_dir="",
    ego_mode="raw",
    ego_frames="31",
    enable_test_score=False,
    gt_score_json="",
    score_split="",
    strict_gt_score=False,
    disable_intermediate_outputs=True,
    keep_replay_csv=False,
):
    # 统一 replay 策略：warmup/ego_frames 均固定为 31，确保线上线下一致。
    warmup, ego_frames = _enforce_fixed_replay_policy(warmup, ego_frames)

    try:
        from planner import PLANNERS
        from utils.ScenarioManager.ScenarioManagerForISG import ScenarioManagerForISG
        from utils.metrics.cal_comfort import get_comfort_metrics
        from utils.metrics.cal_realism import get_realism_metrics
        from utils.metrics.collision_detect import get_collision_rate
        from utils.metrics.cal_dynamics import get_dynamic_constraint
    except Exception as e:
        raise RuntimeError(
            "Failed to import replay modules. "
            "Please run in onsite Python3.6 environment. Original error: %r" % (e,)
        )

    use_raw_mode = bool(raw_scene_dir)

    # 关闭中间输出时，统一使用时间戳子目录保存日志与最终得分
    if bool(disable_intermediate_outputs):
        topic_tag = _infer_topic_tag(
            raw_scene_dir=raw_scene_dir,
            gt_raw_scene_dir=gt_raw_scene_dir,
            score_split=score_split,
            scenario_dir=os.path.join(workspace_dir, "scenario"),
        )
        timestamp_dir = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + topic_tag
        output_dir = os.path.join(output_dir, timestamp_dir)

    folder_scene_total = 0
    selected_scene_total = 0

    if use_raw_mode:
        scenario_dir = os.path.abspath(raw_scene_dir)
        gt_dir = ""
        gt_raw_dir = os.path.abspath(str(gt_raw_scene_dir or "").rstrip(os.sep)) if str(gt_raw_scene_dir or "").strip() else ""
        map_xodr_abs = os.path.abspath(str(map_xodr_path or "").rstrip(os.sep)) if str(map_xodr_path or "").strip() else ""
        if map_xodr_abs and (not os.path.exists(map_xodr_abs) or not map_xodr_abs.lower().endswith(".xodr")):
            raise RuntimeError("--map-xodr must be an existing .xodr file: %s" % map_xodr_abs)
        raw_scene_dirs = _discover_raw_scene_dirs(raw_scene_dir, require_xodr=False)
        folder_scene_total = len(raw_scene_dirs)
        all_scene_names = [
            os.path.splitext(os.path.basename(x))[0] if os.path.isfile(x) and x.lower().endswith('.xosc') else os.path.basename(x.rstrip(os.sep))
            for x in raw_scene_dirs
        ]
        if scene_limit and scene_limit > 0:
            all_scene_names = all_scene_names[:scene_limit]
        if not all_scene_names:
            raise RuntimeError("No xosc scene files or scene folders with xosc found in: %s" % raw_scene_dir)
        if gt_raw_dir and not os.path.isdir(gt_raw_dir):
            raise RuntimeError("--gt-raw-scene-dir not found: %s" % gt_raw_dir)
    else:
        scenario_dir = os.path.join(workspace_dir, "scenario")
        gt_dir = os.path.join(workspace_dir, "ground_truth")
        gt_raw_dir = ""
        map_xodr_abs = ""
        if not os.path.isdir(scenario_dir):
            raise RuntimeError("scenario dir not found: %s" % scenario_dir)
        if not os.path.isdir(gt_dir):
            raise RuntimeError("ground_truth dir not found: %s" % gt_dir)

        all_scene_names = _discover_scene_names(scenario_dir)
        folder_scene_total = len(all_scene_names)
        if not all_scene_names:
            raise RuntimeError("No *_output.pkl found in: %s" % scenario_dir)

        if scene_limit and scene_limit > 0:
            all_scene_names = all_scene_names[:scene_limit]

    selected_scene_total = len(all_scene_names)

    config = {
        "tasks": all_scene_names,
        "warmup": int(warmup),
        "skipExist": bool(skip_exist),
        "visualize": False,
        "keep_replay_csv": bool(keep_replay_csv),
        "dynamic_start_index": int(max(1, warmup)),
        "dynamic_require_valid_at_start": False,
        "dynamic_min_valid_points": 2,
        "dynamic_check_continuity": False,
        "dynamic_heading_adjacent_only": True,
    }

    if use_raw_mode:
        sm = _load_raw_tasks(
            raw_scene_dir=raw_scene_dir,
            warmup=config["warmup"],
            scene_limit=scene_limit,
            ego_mode=ego_mode,
            ego_frames=ego_frames,
            gt_raw_scene_dir=gt_raw_dir,
            map_xodr_path=map_xodr_abs,
        )
    else:
        sm = ScenarioManagerForISG(scenario_dir, gt_dir, config)
    if not sm.tasks:
        raise RuntimeError("No valid tasks loaded from workspace: %s" % workspace_dir)

    os.makedirs(output_dir, exist_ok=True)
    tester = Tester(scenario_dir, output_dir)

    prefilter_stats = {}
    prefilter_rows = []
    if use_raw_mode:
        prefilter_stats = dict(getattr(sm, "load_stats", {}) or {})
        prefilter_rows = list(getattr(sm, "prefilter_rows", []) or [])

    results_path = os.path.join(output_dir, "results.json")
    summary_path = os.path.join(output_dir, "run_summary.json")

    def _load_score_module():
        score_script_path = os.path.join(os.path.dirname(__file__), "score_2026.py")
        if not os.path.exists(score_script_path):
            # Backward compatibility with older filename
            score_script_path = os.path.join(os.path.dirname(__file__), "score_no_test_value_2026.py")
        spec = importlib.util.spec_from_file_location("score_2026", score_script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("Failed to load score script: %s" % score_script_path)
        score_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(score_module)
        return score_module

    score_module = _load_score_module()
    gt_planner_avg_map = {}
    # 读取 GT Planner Avg Score（测试有效性）
    if bool(enable_test_score):
        try:
            builder = getattr(score_module, "_build_gt_planner_avg_map_from_src", None)
            if callable(builder):
                gt_planner_avg_map = builder(
                    gt_score_json=gt_score_json,
                    split=score_split,
                )
            else:
                print("[WARNING] score module has no GT score map builder; fallback test score=0.")
        except Exception as e:
            print("[WARNING] failed to load GT Planner Avg map:", repr(e))
            gt_planner_avg_map = {}

    def _realtime_flush_outputs(result_dict, run_summary):
        # 关闭中间结果时，跳过 results.json / run_summary.json / score 目录等文件输出
        if not bool(disable_intermediate_outputs):
            with open(results_path, "w") as f:
                json.dump(dict(result_dict), f, indent=4)

        try:
            if bool(disable_intermediate_outputs):
                score_module.score_results_from_dict(
                    dict(result_dict),
                    output_dir,
                    include_test_score=bool(enable_test_score),
                    gt_planner_avg_map=gt_planner_avg_map,
                    strict_gt_score=bool(strict_gt_score),
                    score_profile="bg_only",
                    simple_output=True,
                    simple_output_dir=output_dir,
                )
            else:
                score_module.score_results(
                    results_path,
                    output_dir,
                    include_test_score=bool(enable_test_score),
                    gt_planner_avg_map=gt_planner_avg_map,
                    strict_gt_score=bool(strict_gt_score),
                )
        except TypeError:
            score_module.score_results(results_path, output_dir)

        if not bool(disable_intermediate_outputs):
            src = os.path.join(output_dir, "score", "all_scenes_summary.csv")
            dst = os.path.join(output_dir, "score", "score.csv")
            if os.path.exists(src):
                with open(src, "r", encoding="utf-8") as fsrc, open(dst, "w", encoding="utf-8", newline="") as fdst:
                    fdst.write(fsrc.read())
                try:
                    os.remove(src)
                except Exception:
                    pass

            with open(summary_path, "w") as f:
                json.dump(run_summary, f, indent=4)

    tested_scene_saved_count = 0
    tested_scene_unique = set()
    saved_scene_unique = set()

    # Step1: 先做背景指标计算（不依赖主车 planner 回放结果）。
    # 这些结果会写入每个场景基础字段，后续再叠加 planner_* 能力分。
    results = []
    results.append(get_comfort_metrics(sm, config))
    results.append(get_realism_metrics(sm, config, gt_dir))
    results.append(get_collision_rate(sm, config))
    dynamic_results = get_dynamic_constraint(sm, config)
    results.append(dynamic_results)

    result_dict = defaultdict(dict)
    for part in results:
        for scene_name, scene_value in part.items():
            result_dict[scene_name].update(scene_value)

    current_out_dynamic_count = int(sum(int(v.get("is_out_dynamic", 0)) for v in dynamic_results.values()))

    selected = None
    if planner_indices is not None:
        selected = set(planner_indices)

    planner_jobs = [idx for idx in range(len(PLANNERS)) if selected is None or idx in selected]
    worker_count = int(num_workers) if num_workers is not None else 1
    if worker_count <= 0:
        worker_count = int(os.cpu_count() or 1)

    used_planners = []
    mp_start_method = "sequential"

    run_summary = {
        "workspace_dir": workspace_dir,
        "scenario_dir": scenario_dir,
        "ground_truth_dir": gt_dir,
        "gt_raw_scene_dir": gt_raw_dir if use_raw_mode else "",
        "input_mode": "raw_xosc" if use_raw_mode else "pkl_workspace",
        "folder_scene_total": folder_scene_total,
        "selected_scene_total": selected_scene_total,
        "aligned_scene_count": len(sm.tasks),
        "output_dir": output_dir,
        "scene_count": len(sm.tasks),
        "scene_names": [s.name for s in sm.tasks],
        "warmup": int(config["warmup"]),
        "skip_exist": bool(config["skipExist"]),
        "planner_run_mode": "",
        "planner_mp_start_method": "",
        "planner_workers_requested": worker_count,
        "scene_timeout_sec": int(scene_timeout_sec),
        "planner_jobs": planner_jobs,
        "used_planners": [],
        "planner_result_scene_count": 0,
        "tested_scene_save_dir": tested_scene_save_dir,
        "tested_scene_saved_count": tested_scene_saved_count,
        "saved_scene_count": 0,
        "extracted_aligned_scene_count": len(sm.tasks),
        "out_dynamic_failed_scene_count": 0,
        "ego_tested_scene_count": 0,
        "ego_test_error_scene_count": 0,
        "ego_test_error_scene_names": [],
        "results_json": results_path,
        "test_value_enabled": bool(enable_test_score),
        "test_score_source": "gt_score_json" if bool(enable_test_score) else "disabled_zero",
        "gt_score_json": gt_score_json if bool(enable_test_score) else "",
        "score_split": score_split if bool(enable_test_score) else "",
        "strict_gt_score": bool(strict_gt_score),
        "gt_planner_avg_map_count": int(len(gt_planner_avg_map)),
        "dynamic_policy": {
            "dynamic_start_index": config["dynamic_start_index"],
            "dynamic_require_valid_at_start": config["dynamic_require_valid_at_start"],
            "dynamic_min_valid_points": config["dynamic_min_valid_points"],
            "dynamic_check_continuity": config["dynamic_check_continuity"],
            "dynamic_heading_adjacent_only": config["dynamic_heading_adjacent_only"],
        },
        "raw_ego_mode": ego_mode,
        "raw_ego_frames_policy": ego_frames,
        "prefilter_input_scene_total": int(prefilter_stats.get("input_scene_total", selected_scene_total)),
        "prefilter_removed_short_lt31_count": int(prefilter_stats.get("prefilter_removed_short_lt31_count", 0)),
        "prefilter_removed_pre31_collision_count": int(prefilter_stats.get("prefilter_removed_pre31_collision_count", 0)),
        "prefilter_removed_missing_gt_scene_count": int(prefilter_stats.get("prefilter_removed_missing_gt_scene_count", 0)),
        "prefilter_removed_invalid_gt_scene_count": int(prefilter_stats.get("prefilter_removed_invalid_gt_scene_count", 0)),
        "prefilter_removed_total": int(prefilter_stats.get("prefilter_removed_total", 0)),
        "prefilter_kept_count": int(prefilter_stats.get("prefilter_kept_count", len(sm.tasks))),
        "worker_failed_scene_count": 0,
        "worker_timeout_scene_count": 0,
        "worker_timeout_scene_names": [],
        "groundtruth_skip_worker_timeout_count": 0,
    }

    planner_error_rows = []
    ego_error_scene_set = set()
    ego_error_scene_rows = []
    worker_failed_scene_set = set()
    worker_timeout_scene_set = set()

    def _zero_planner_result_block():
        return {
            f"planner_{idx}": {"safe": 0.0, "efficiency": 0.0, "comfortable": 0.0}
            for idx in planner_jobs
        }

    def _refresh_user_counters():
        run_summary["saved_scene_count"] = len(saved_scene_unique)
        run_summary["extracted_aligned_scene_count"] = len(sm.tasks)
        run_summary["out_dynamic_failed_scene_count"] = int(
            run_summary.get("out_dynamic_positive_count", current_out_dynamic_count)
        )
        run_summary["ego_tested_scene_count"] = int(
            run_summary.get("planner_result_scene_count", 0)
        )
        run_summary["ego_test_error_scene_count"] = len(ego_error_scene_set)
        run_summary["ego_test_error_scene_names"] = sorted(list(ego_error_scene_set))
        run_summary["worker_failed_scene_count"] = len(worker_failed_scene_set)
        run_summary["worker_timeout_scene_count"] = len(worker_timeout_scene_set)
        run_summary["worker_timeout_scene_names"] = sorted(list(worker_timeout_scene_set))
        run_summary["groundtruth_skip_worker_timeout_count"] = len(worker_timeout_scene_set)

    if not bool(disable_intermediate_outputs):
        try:
            prefilter_csv = _write_prefilter_scene_diagnosis(output_dir, prefilter_rows)
            run_summary["prefilter_scene_diagnosis_csv"] = prefilter_csv
        except Exception as e:
            print(f"[WARNING] 输出预过滤场景诊断失败: {e}")

    if not bool(disable_intermediate_outputs):
        try:
            diag_json, diag_csv, diag_scene_count, diag_out_count = _write_out_dynamic_diagnosis(
                output_dir=output_dir,
                dynamic_results=dynamic_results,
                dynamic_policy=run_summary["dynamic_policy"],
            )
            run_summary["out_dynamic_diagnosis_json"] = diag_json
            run_summary["out_dynamic_diagnosis_csv"] = diag_csv
            run_summary["out_dynamic_scene_count"] = diag_scene_count
            run_summary["out_dynamic_positive_count"] = diag_out_count
            print(f"[INFO] out dynamic diagnosis: {diag_json}, {diag_csv}")
        except Exception as e:
            print(f"[WARNING] 输出 out dynamic 诊断失败: {e}")
            run_summary["out_dynamic_positive_count"] = current_out_dynamic_count

    _refresh_user_counters()
    _realtime_flush_outputs(result_dict, run_summary)

    used_planners = [{"planner_idx": idx, "planner_name": PLANNERS[idx].__name__} for idx in planner_jobs]

    # Step2: 运行主车接管评测（顺序或多进程）。
    if worker_count > 1 and len(sm.tasks) > 1:
        actual_workers = min(worker_count, len(sm.tasks))
        scene_names = [s.name for s in sm.tasks]
        worker_args = [
            (
                scenario_dir,
                gt_dir,
                raw_scene_dir,
                gt_raw_dir,
                bool(use_raw_mode),
                output_dir,
                scene_name,
                int(config["warmup"]),
                bool(config["skipExist"]),
                planner_jobs,
                submitid,
                tested_scene_save_dir,
                ego_mode,
                ego_frames,
                bool(config.get("keep_replay_csv", False)),
                map_xodr_abs,
            )
            for scene_name in scene_names
        ]
        # Linux + Python3.6 下 spawn 可能出现 SemLock._rebuild FileNotFoundError，优先使用 fork。
        try:
            ctx = multiprocessing.get_context("fork")
            mp_start_method = "fork"
        except ValueError:
            ctx = multiprocessing.get_context("spawn")
            mp_start_method = "spawn"

        pending = list(worker_args)
        running = []
        worker_outputs = []

        while pending or running:
            while pending and len(running) < actual_workers:
                cur_args = pending.pop(0)
                parent_conn, child_conn = ctx.Pipe(duplex=False)
                proc = ctx.Process(target=_run_scene_worker_process, args=(cur_args, child_conn))
                proc.daemon = False
                proc.start()
                child_conn.close()
                running.append(
                    {
                        "scene_name": cur_args[6],
                        "proc": proc,
                        "conn": parent_conn,
                        "start_ts": time.time(),
                    }
                )

            if not running:
                break

            progress = False
            for item in list(running):
                scene_name = item["scene_name"]
                proc = item["proc"]
                conn = item["conn"]
                if conn.poll(0.1):
                    try:
                        worker_outputs.append(conn.recv())
                    except Exception as e:
                        worker_outputs.append(
                            {
                                "scene_name": scene_name,
                                "replay_result": {},
                                "ok": False,
                                "error": "recv_failed: %r" % (e,),
                            }
                        )
                    try:
                        conn.close()
                    except Exception:
                        pass
                    proc.join()
                    running.remove(item)
                    progress = True
                elif not proc.is_alive():
                    exit_code = proc.exitcode
                    try:
                        conn.close()
                    except Exception:
                        pass
                    proc.join()
                    running.remove(item)
                    worker_outputs.append(
                        {
                            "scene_name": scene_name,
                            "replay_result": {},
                            "ok": False,
                            "error": "worker_exit_without_result (exitcode=%s)" % (exit_code,),
                        }
                    )
                    progress = True
                elif scene_timeout_sec and (time.time() - item.get("start_ts", time.time()) > float(scene_timeout_sec)):
                    # 超时强制回收，避免单个场景拖死整批
                    try:
                        proc.terminate()
                    except Exception:
                        pass
                    try:
                        conn.close()
                    except Exception:
                        pass
                    proc.join(timeout=2)
                    running.remove(item)
                    worker_outputs.append(
                        {
                            "scene_name": scene_name,
                            "replay_result": {},
                            "ok": False,
                            "error": "worker_timeout_exceeded (%ss)" % int(scene_timeout_sec),
                        }
                    )
                    progress = True

            if not progress:
                time.sleep(0.1)

        for item in running:
            try:
                item["conn"].close()
            except Exception:
                pass
            item["proc"].join()

        for item in sorted(worker_outputs, key=lambda x: x.get("scene_name", "")):
            if not item.get("ok", False):
                failed_scene = item.get("scene_name", "")
                err_msg = item.get("error", "")
                print(f"[ERROR] Scene worker {failed_scene} failed: {err_msg}")

                if failed_scene:
                    # worker级别报错场景：直接补齐主车接入0分，确保记录里该场景总分为0
                    result_dict[failed_scene].update(_zero_planner_result_block())
                    tested_scene_unique.add(failed_scene)
                    ego_error_scene_set.add(failed_scene)
                    worker_failed_scene_set.add(failed_scene)
                    if "worker_timeout_exceeded" in str(err_msg):
                        worker_timeout_scene_set.add(failed_scene)
                    ego_error_scene_rows.append(
                        {
                            "Scene ID": failed_scene,
                            "Error Stage": "worker",
                            "Error": str(err_msg),
                        }
                    )
                    planner_error_rows.append(
                        {
                            "Scene ID": failed_scene,
                            "Planner Idx": -1,
                            "Planner Name": "scene_worker",
                            "Error": str(err_msg),
                        }
                    )

                    run_summary["used_planners"] = used_planners
                    run_summary["tested_scene_saved_count"] = tested_scene_saved_count
                    run_summary["planner_result_scene_count"] = sum(
                        1 for v in result_dict.values() if any(k.startswith("planner_") for k in v.keys())
                    )
                    run_summary["planner_run_mode"] = "multiprocess"
                    run_summary["planner_mp_start_method"] = mp_start_method
                    _refresh_user_counters()
                    _realtime_flush_outputs(result_dict, run_summary)
                continue

            replay_result = item.get("replay_result", {})
            results.append(replay_result)
            for scene_name, scene_value in replay_result.items():
                result_dict[scene_name].update(scene_value)

            tested_scene_saved_count += int(item.get("tested_scene_saved_count", 0))
            for er in item.get("planner_errors", []):
                planner_error_rows.append(er)
                scene_id = er.get("Scene ID", "")
                if scene_id:
                    ego_error_scene_set.add(scene_id)
                    ego_error_scene_rows.append(
                        {
                            "Scene ID": scene_id,
                            "Error Stage": "planner",
                            "Error": er.get("Error", ""),
                        }
                    )
            for sn in item.get("tested_scene_names", []):
                tested_scene_unique.add(sn)
            for sn in item.get("saved_scene_names", []):
                saved_scene_unique.add(sn)

            run_summary["used_planners"] = used_planners
            run_summary["tested_scene_saved_count"] = tested_scene_saved_count
            run_summary["planner_result_scene_count"] = sum(
                1 for v in result_dict.values() if any(k.startswith("planner_") for k in v.keys())
            )
            run_summary["planner_run_mode"] = "multiprocess"
            run_summary["planner_mp_start_method"] = mp_start_method
            _refresh_user_counters()
            _realtime_flush_outputs(result_dict, run_summary)
    else:
        mp_start_method = "sequential"
        from evaluation import run_evaluator
        for scene in sm.tasks:
            replay_result = {}
            scene_has_error = False
            for planner_idx in planner_jobs:
                cur_planner = PLANNERS[planner_idx]
                try:
                    output_name = f"{scene.type}_{scene.num}_av{planner_idx}_{scene.name}_result.csv"
                    output_path = os.path.join(output_dir, output_name)
                    if not (config["skipExist"] and os.path.exists(output_path)):
                        tester.replay_test(config, scene, cur_planner, output_path)

                    result = run_evaluator(map_file=scene.source_file["xodr"], csv_file_path=output_path)
                    ability = {
                        "safe": result["AbilityDimension"]["safe"],
                        "efficiency": result["AbilityDimension"]["efficiency"],
                        "comfortable": result["AbilityDimension"]["comfortable"],
                    }
                    replay_result[f"planner_{planner_idx}"] = ability
                    if not config.get("keep_replay_csv", False) and os.path.exists(output_path):
                        try:
                            os.remove(output_path)
                        except Exception:
                            pass
                except Exception as e:
                    scene_has_error = True
                    print(f"[ERROR] Planner {cur_planner.__name__} failed in scene {scene.name}: {e}")
                    replay_result[f"planner_{planner_idx}"] = {
                        "safe": 0.0,
                        "efficiency": 0.0,
                        "comfortable": 0.0,
                    }
                    planner_error_rows.append(
                        {
                            "Scene ID": scene.name,
                            "Planner Idx": planner_idx,
                            "Planner Name": cur_planner.__name__,
                            "Error": repr(e),
                        }
                    )
                    ego_error_scene_set.add(scene.name)
                    ego_error_scene_rows.append(
                        {
                            "Scene ID": scene.name,
                            "Error Stage": "planner",
                            "Error": repr(e),
                        }
                    )

            if replay_result:
                result_dict[scene.name].update(replay_result)
                tested_scene_unique.add(scene.name)
                if not scene_has_error:
                    saved = _save_tested_scene_sources(scene.name, scene.source_file, tested_scene_save_dir)
                    if saved:
                        tested_scene_saved_count += 1
                        saved_scene_unique.add(scene.name)

            run_summary["used_planners"] = used_planners
            run_summary["tested_scene_saved_count"] = tested_scene_saved_count
            run_summary["planner_result_scene_count"] = sum(
                1 for v in result_dict.values() if any(k.startswith("planner_") for k in v.keys())
            )
            run_summary["planner_run_mode"] = "sequential"
            run_summary["planner_mp_start_method"] = mp_start_method
            _refresh_user_counters()
            _realtime_flush_outputs(result_dict, run_summary)

    planner_result_scene_count = sum(
        1 for scene_value in result_dict.values() if any(k.startswith("planner_") for k in scene_value.keys())
    )

    run_summary["planner_run_mode"] = "multiprocess" if (worker_count > 1 and len(sm.tasks) > 1) else "sequential"
    run_summary["planner_mp_start_method"] = mp_start_method
    run_summary["planner_workers_requested"] = worker_count
    run_summary["planner_jobs"] = planner_jobs
    run_summary["used_planners"] = used_planners
    run_summary["planner_result_scene_count"] = planner_result_scene_count
    run_summary["tested_scene_saved_count"] = tested_scene_saved_count
    _refresh_user_counters()

    if planner_result_scene_count == 0:
        print("[WARNING] no planner_* results found in results.json; Planner Avg Score will be 0.")

    if not bool(disable_intermediate_outputs):
        try:
            planner_err_csv = _write_planner_error_diagnosis(output_dir, planner_error_rows)
            run_summary["planner_error_diagnosis_csv"] = planner_err_csv
            run_summary["planner_error_count"] = len(planner_error_rows)
        except Exception as e:
            print(f"[WARNING] 输出 planner 错误诊断失败: {e}")

    if not bool(disable_intermediate_outputs):
        try:
            ego_err_csv = _write_ego_test_error_scene_stats(output_dir, ego_error_scene_rows)
            run_summary["ego_test_error_scenes_csv"] = ego_err_csv
        except Exception as e:
            print(f"[WARNING] 输出主车接入报错场景统计失败: {e}")

    _refresh_user_counters()
    _realtime_flush_outputs(result_dict, run_summary)

    if not bool(disable_intermediate_outputs):
        density_png = _try_plot_planner_avg_density(output_dir)
        if density_png:
            run_summary["planner_avg_score_density_png"] = density_png
            _realtime_flush_outputs(result_dict, run_summary)

    if not bool(disable_intermediate_outputs):
        try:
            key_stats_csv = _write_key_run_stats_csv(output_dir, run_summary)
            run_summary["key_run_stats_csv"] = key_stats_csv
        except Exception as e:
            print(f"[WARNING] 输出关键统计CSV失败: {e}")

    _realtime_flush_outputs(result_dict, run_summary)

    return results_path, summary_path, run_summary


def main():
    parser = argparse.ArgumentParser(description="Run 2026 replay with 4 planners")
    parser.add_argument(
        "--workspace-dir",
        default=os.path.join(ROOT_DIR, "new_workflow", "workspace_2026A_aligned"),
        help="workspace containing scenario/ and ground_truth/",
    )
    parser.add_argument(
        "--raw-scene-dir",
        default="",
        help=(
            "optional raw scene root containing per-scene xosc/xodr folders; "
            "when --splits is set, this should point to split root containing A/B/C"
        ),
    )
    parser.add_argument(
        "--gt-raw-scene-dir",
        default="",
        help=(
            "optional GT raw scene root for realism reference; "
            "layout mirrors --raw-scene-dir (scene dirs or split root)"
        ),
    )
    parser.add_argument(
        "--map-xodr",
        default="",
        help=(
            "optional common .xodr map file. Contestant raw scene folders may contain only .xosc; "
            "if omitted, xodr is resolved from the matching GT scene or the first .xodr under --gt-raw-scene-dir."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help=(
            "directory for results.json and planner csv outputs; "
            "if empty and --raw-scene-dir is .../ego_completed_31/<split>, "
            "auto use .../output31/<split>; "
            "when --splits is set, this is treated as batch output root"
        ),
    )
    parser.add_argument(
        "--splits",
        default="",
        help="comma separated splits for batch mode, e.g. A,B,C; empty means single-run mode",
    )
    parser.add_argument(
        "--scene-limit",
        type=int,
        default=0,
        help="0 means all scenes; otherwise use first N scenes",
    )
    parser.add_argument(
        "--planner-indices",
        default="",
        help="comma separated planner indices, e.g. 0,1,2,3",
    )
    parser.add_argument(
        "--num-workers",
        type=int,
        default=1,
        help="planner parallel workers; <=0 means auto(cpu_count)",
    )
    parser.add_argument(
        "--scene-timeout-sec",
        type=int,
        default=1800,
        help="per-scene worker timeout in seconds, default=1800",
    )
    parser.add_argument("--skip-exist", action="store_true", help="skip existing replay csv files")
    parser.add_argument("--submitid", default="onsite2026")
    parser.add_argument(
        "--tested-scene-save-dir",
        default="",
        help="optional directory to save xodr/xosc for scenes that are successfully tested by ego",
    )
    parser.add_argument(
        "--ego-mode",
        choices=["raw", "min2", "to_goal"],
        default="raw",
        help=(
            "raw: keep only initial ego point when no ego trajectory; "
            "min2: synthesize 2-point ego trajectory; "
            "to_goal: synthesize linear trajectory from initial state to target center"
        ),
    )
    # 默认开启测试有效性评分；需要关闭时显式传 --disable-test-score
    parser.add_argument(
        "--enable-test-score",
        action="store_true",
        help="deprecated (test score is enabled by default; use --disable-test-score to turn off)",
    )
    parser.add_argument(
        "--disable-test-score",
        action="store_true",
        help="disable Test Score computation",
    )
    parser.add_argument(
        "--gt-score-json",
        default=os.path.join("src", "gt_score_ABC.json"),
        help="path to src gt score json that contains GT Planner Avg Score (relative path preferred)",
    )
    parser.add_argument(
        "--score-split",
        default="",
        help="split key for --gt-score-json, e.g. A/B/C; empty means infer from --raw-scene-dir basename",
    )
    parser.add_argument(
        "--strict-gt-score",
        action="store_true",
        help="fail if any scene misses GT Planner Avg Score when --enable-test-score is set",
    )
    parser.add_argument(
        "--enable-intermediate-outputs",
        action="store_true",
        help="enable intermediate outputs (results.json, run_summary.json, diagnosis CSVs)",
    )
    parser.add_argument(
        "--keep-replay-csv",
        action="store_true",
        help="keep REPLAY_*_result.csv files (default: delete after scoring)",
    )
    args = parser.parse_args()

    begin = time.time()
    planner_indices = _parse_planner_indices(args.planner_indices)
    effective_warmup, effective_ego_frames = FIXED_WARMUP, FIXED_EGO_FRAMES
    score_split_override = str(args.score_split or "").strip().upper()
    enable_test_score = not bool(args.disable_test_score)
    disable_intermediate_outputs = not bool(args.enable_intermediate_outputs)
    keep_replay_csv = bool(args.keep_replay_csv)
    requested_splits = _parse_splits(args.splits)

    # 多 split 批处理模式（A/B/C）
    if requested_splits:
        raw_root = os.path.abspath(str(args.raw_scene_dir or "").rstrip(os.sep))
        if not raw_root:
            raise RuntimeError("--raw-scene-dir is required when --splits is set.")
        if not os.path.isdir(raw_root):
            raise RuntimeError("--raw-scene-dir not found: %s" % raw_root)
        gt_raw_root = os.path.abspath(str(args.gt_raw_scene_dir or "").rstrip(os.sep)) if str(args.gt_raw_scene_dir or "").strip() else ""
        if gt_raw_root and not os.path.isdir(gt_raw_root):
            raise RuntimeError("--gt-raw-scene-dir not found: %s" % gt_raw_root)

        batch_output_root = _resolve_batch_output_root_dir(args.output_dir, raw_root)
        split_results = []
        total_scene_count = 0

        for split in requested_splits:
            split_raw_scene_dir = os.path.join(raw_root, split)
            if not os.path.isdir(split_raw_scene_dir):
                raise RuntimeError("split dir not found under --raw-scene-dir: %s" % split_raw_scene_dir)

            split_output_dir = _resolve_batch_output_dir(
                output_dir_arg=args.output_dir,
                split_raw_scene_dir=split_raw_scene_dir,
                split=split,
            )
            split_tested_scene_save_dir = _resolve_batch_tested_scene_save_dir(
                tested_scene_save_dir=args.tested_scene_save_dir,
                split=split,
            )
            split_gt_raw_scene_dir = ""
            if gt_raw_root:
                candidate_gt_split_dir = os.path.join(gt_raw_root, split)
                if os.path.isdir(candidate_gt_split_dir):
                    split_gt_raw_scene_dir = candidate_gt_split_dir
                elif os.path.basename(gt_raw_root).upper() == split:
                    split_gt_raw_scene_dir = gt_raw_root
                else:
                    raise RuntimeError(
                        "split GT dir not found under --gt-raw-scene-dir: %s" % candidate_gt_split_dir
                    )
            split_score_split = score_split_override if score_split_override else split
            split_submitid = "%s_%s" % (str(args.submitid), split)

            results_path, summary_path, run_summary = run_replay(
                workspace_dir=args.workspace_dir,
                output_dir=split_output_dir,
                raw_scene_dir=split_raw_scene_dir,
                gt_raw_scene_dir=split_gt_raw_scene_dir,
                map_xodr_path=args.map_xodr,
                warmup=effective_warmup,
                scene_limit=args.scene_limit,
                planner_indices=planner_indices,
                num_workers=args.num_workers,
                scene_timeout_sec=args.scene_timeout_sec,
                skip_exist=args.skip_exist,
                submitid=split_submitid,
                tested_scene_save_dir=split_tested_scene_save_dir,
                ego_mode=args.ego_mode,
                ego_frames=effective_ego_frames,
                enable_test_score=bool(enable_test_score),
                gt_score_json=args.gt_score_json,
                score_split=split_score_split,
                strict_gt_score=bool(args.strict_gt_score),
                disable_intermediate_outputs=bool(disable_intermediate_outputs),
                keep_replay_csv=bool(keep_replay_csv),
            )

            split_results.append(
                {
                    "split": split,
                    "scene_count": int(run_summary.get("scene_count", 0)),
                    "output_dir": split_output_dir,
                    "results_json": results_path,
                    "run_summary": summary_path,
                }
            )
            total_scene_count += int(run_summary.get("scene_count", 0))
            print("[INFO] split=%s scene_count=%s" % (split, run_summary.get("scene_count", 0)))
            print("[INFO] split=%s results_json=%s" % (split, results_path))
            print("[INFO] split=%s run_summary=%s" % (split, summary_path))

        if not bool(disable_intermediate_outputs):
            merged_scene_rows = []
            merged_avg_rows = []
            merged_fields = ["Split"]
            avg_fields = ["Split"]
            for item in split_results:
                split = item.get("split", "")
                split_output_dir = item.get("output_dir", "")
                score_csv = os.path.join(split_output_dir, "score", "score.csv")
                fieldnames, scene_rows, avg_row = _read_score_csv_rows(score_csv)

                for fn in fieldnames:
                    if fn not in merged_fields:
                        merged_fields.append(fn)
                    if fn not in avg_fields:
                        avg_fields.append(fn)

                for row in scene_rows:
                    out_row = {"Split": split}
                    out_row.update(row)
                    merged_scene_rows.append(out_row)

                if avg_row:
                    out_avg = {"Split": split}
                    out_avg.update(avg_row)
                    merged_avg_rows.append(out_avg)

            if not merged_fields:
                merged_fields = ["Split", "Scene ID"]
            if not avg_fields:
                avg_fields = ["Split", "Scene ID"]

            merged_scene_csv = os.path.join(batch_output_root, "all_splits_scene_scores.csv")
            merged_avg_csv = os.path.join(batch_output_root, "all_splits_average_scores.csv")
            _write_rows_csv(merged_scene_rows, merged_scene_csv, merged_fields)
            _write_rows_csv(merged_avg_rows, merged_avg_csv, avg_fields)

            for row in merged_avg_rows:
                print(
                    "[INFO] split=%s avg_final=%s avg_test=%s avg_planner=%s"
                    % (
                        row.get("Split", ""),
                        row.get("Final Score", ""),
                        row.get("Test Score", ""),
                        row.get("Planner Avg Score", ""),
                    )
                )

            print("[INFO] merged_scene_csv:", merged_scene_csv)
            print("[INFO] merged_average_csv:", merged_avg_csv)

        print("[INFO] split_count:", len(split_results))
        print("[INFO] total_scene_count:", int(total_scene_count))
        print("[INFO] done in %.2f seconds" % (time.time() - begin))
        return

    raw_abs = os.path.abspath(str(args.raw_scene_dir or "").rstrip(os.sep))
    raw_base = os.path.basename(raw_abs).upper() if raw_abs else ""
    effective_score_split = score_split_override
    if not effective_score_split:
        if raw_base in ("A", "B", "C"):
            effective_score_split = raw_base
    effective_gt_raw_scene_dir = ""
    if str(args.gt_raw_scene_dir or "").strip():
        gt_raw_abs = os.path.abspath(str(args.gt_raw_scene_dir).rstrip(os.sep))
        if not os.path.isdir(gt_raw_abs):
            raise RuntimeError("--gt-raw-scene-dir not found: %s" % gt_raw_abs)
        if raw_base in ("A", "B", "C"):
            gt_split_candidate = os.path.join(gt_raw_abs, raw_base)
            if os.path.isdir(gt_split_candidate):
                effective_gt_raw_scene_dir = gt_split_candidate
            else:
                effective_gt_raw_scene_dir = gt_raw_abs
        else:
            effective_gt_raw_scene_dir = gt_raw_abs
    # 单次运行模式
    effective_output_dir = _resolve_output_dir_arg(args.raw_scene_dir, args.output_dir)
    results_path, summary_path, run_summary = run_replay(
        workspace_dir=args.workspace_dir,
        output_dir=effective_output_dir,
        raw_scene_dir=args.raw_scene_dir,
        gt_raw_scene_dir=effective_gt_raw_scene_dir,
        map_xodr_path=args.map_xodr,
        warmup=effective_warmup,
        scene_limit=args.scene_limit,
        planner_indices=planner_indices,
        num_workers=args.num_workers,
        scene_timeout_sec=args.scene_timeout_sec,
        skip_exist=args.skip_exist,
        submitid=args.submitid,
        tested_scene_save_dir=args.tested_scene_save_dir,
        ego_mode=args.ego_mode,
        ego_frames=effective_ego_frames,
        enable_test_score=bool(enable_test_score),
        gt_score_json=args.gt_score_json,
        score_split=effective_score_split,
        strict_gt_score=bool(args.strict_gt_score),
        disable_intermediate_outputs=bool(disable_intermediate_outputs),
        keep_replay_csv=bool(keep_replay_csv),
    )
    print("[INFO] scene_count:", run_summary["scene_count"])
    print("[INFO] warmup:", run_summary["warmup"])
    print("[INFO] test_value_enabled:", run_summary.get("test_value_enabled", False))
    if run_summary.get("test_value_enabled", False):
        print("[INFO] gt_score_json:", run_summary.get("gt_score_json", ""))
        print("[INFO] score_split:", run_summary.get("score_split", ""))
    if not bool(disable_intermediate_outputs):
        print("[INFO] results_json:", results_path)
        print("[INFO] run_summary:", summary_path)
    print("[INFO] done in %.2f seconds" % (time.time() - begin))


if __name__ == "__main__":
    main()
