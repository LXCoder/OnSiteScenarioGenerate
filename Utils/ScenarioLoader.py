"""
ScenarioLoader —— 从 Data 目录加载车辆路径 JSON 文件

JSON 文件格式：
{
    "vehicles": {
        "car_1": {
            "path": [[-637.84, 160.61], [-642.75, 154.64], [-645.15, 151.73]],
            "speed": 10.0
        },
        "car_2": {
            "path": [[-620.0, 170.0], [-625.0, 165.0], [-630.0, 158.0]],
            "speed": 12.0
        }
    }
}

用法：
    loader = ScenarioLoader("Data")
    scenarios = loader.loadAll()
    # scenarios = [
    #     {"file": "scene_01.json", "vehicles": {"car_1": {"path": [...], "speed": 10.0}, ...}},
    #     {"file": "scene_02.json", "vehicles": {"car_2": {"path": [...], "speed": 8.0}, ...}},
    # ]
"""

import os
import sys
import json
from typing import List, Dict, Any
from Utils.ConvertXosc2Scene import parse_xosc
from Utils.Constant import (
    DATA_DIR,
    TRAIN_MODE,
    USE_TEST_LOGIC,
    SCENE_TESS_MAPPING_PATH,
    FILTER_SCENES,
    NET_ROOT,
    NET_PATH,
)


def _init_scene_mapping(mapping_file: str):
    scene_to_tess_mapping = {}
    with open(mapping_file, "r", encoding="utf-8") as f:
        conf = json.load(f)
        mapping_conf = conf.get("map40_to_scene_names", {})
        keys = ["A", "B", "C", "all"]
        for map40 in mapping_conf:
            for key in keys:
                if key not in mapping_conf[map40]:
                    continue

                for scene_name in mapping_conf[map40][key]:
                    if scene_name in scene_to_tess_mapping:
                        scene_to_tess_mapping[scene_name]["type"].append(key)
                        continue

                    scene_to_tess_mapping[scene_name] = {
                        "net_name": map40,
                        "type": [key],
                    }

    return scene_to_tess_mapping


g_scene_to_tess_mapping = _init_scene_mapping(SCENE_TESS_MAPPING_PATH)


class ScenarioLoader:
    def __init__(self, dataDir: str, filter_scenes: list = []):
        scene_dir = dataDir
        if USE_TEST_LOGIC:
            scene_dir = os.path.join(dataDir, "train" if TRAIN_MODE else "test")

        self.dataDir = scene_dir
        self.filter_scenes = filter_scenes
        self.net_path = NET_PATH
        self.scenarios = self._loadAll()

    def getNetPath(self):
        return self.net_path

    def getScenarios(self):
        return self.scenarios

    def _loadAll(self) -> List[Dict[str, Any]]:
        """
        加载目录下所有 JSON 文件

        Returns:
            [{"file": "xxx.json", "vehicles": {name: {"path": [(x,y),...], "speed": float}}}, ...]
        """

        jsonFiles = []
        if USE_TEST_LOGIC:
            if not os.path.isdir(self.dataDir):
                print(f"[ScenarioLoader] 目录不存在: {self.dataDir}")
                return []

            jsonFiles = [
                f
                for f in os.listdir(self.dataDir)
                if f.endswith(".json") and f not in self.filter_scenes
            ]

        else:
            xodr_path, xosc_path = None, None
            for file_item in os.listdir(self.dataDir):
                if file_item.endswith(".xodr"):
                    xodr_path = os.path.join(self.dataDir, file_item)

                if file_item.endswith(".xosc"):
                    xosc_path = os.path.join(self.dataDir, file_item)

                if xodr_path and xosc_path:
                    jsonFiles.append(xosc_path)
                    break

        jsonFiles = sorted(jsonFiles)

        scenarios = []
        for filename in jsonFiles:
            if USE_TEST_LOGIC:
                filepath = os.path.join(self.dataDir, filename)
                scenario = self._loadFile(filepath)
            else:
                scenario = parse_xosc(filename)
                basename = os.path.basename(filename)
                tess_info = self._getTessForScene(basename.split(".")[0])
                if tess_info is None:
                    msg = f"[ScenarioLoader] 未找到场景 {basename} 对应的 TESS 路网信息"
                    print(msg)
                    sys.exit(1)
                full_net_path = os.path.join(NET_ROOT, f"{tess_info['net_name']}.tess")
                if not os.path.exists(full_net_path):
                    sys.exit(1)

                self.net_path = full_net_path

                traj = [
                    [pos[0], -pos[1]] for pos in scenario["vehicles"]["ego"]["path"]
                ]
                scenario["vehicles"]["ego"]["path"] = traj

            if scenario:
                scenario["file"] = filename
                scenarios.append(scenario)

        print(f"[ScenarioLoader] 从 {self.dataDir} 加载了 {len(scenarios)} 个场景")
        for s in scenarios:
            names = list(s["vehicles"].keys())
            print(f"  {s['file']}: {len(names)} 辆车 {names}")

        return scenarios

    def _loadFile(self, filepath: str) -> Dict[str, Any]:
        """加载单个 JSON 文件"""
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"[ScenarioLoader] 读取失败 {filepath}: {e}")
            return None

        vehicles = {}
        rawVehicles = data.get("vehicles", {})

        for name, info in rawVehicles.items():
            rawPath = info.get("path", [])
            speed = info.get("speed", 10.0)
            color = info.get("color", "#4911E4")

            # 转成 tuple 列表
            path = [(float(p[0]), float(p[1])) for p in rawPath]

            if len(path) < 2:
                print(f"[ScenarioLoader] 跳过 {name}: 路径点不足")
                continue

            vehicles[name] = {
                "path": path,
                "speed": speed,
                "color": color,
                "control": info.get("control", info.get("controlMode", "model")),
            }

        return {"vehicles": vehicles}

    def _getTessForScene(self, scene_name: str):
        """根据场景名称获取对应的 TESS 场景信息

        Args:
            scene_name: 场景名称（如 "scene_03"）

        Returns:
            包含 "net_name" 和 "type" 的字典，或 None（如果未找到）
        """
        return g_scene_to_tess_mapping.get(scene_name)


g_scenario_loader = ScenarioLoader(DATA_DIR, FILTER_SCENES)
