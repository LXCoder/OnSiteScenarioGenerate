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
import json
from typing import List, Dict, Any
from Utils.ConvertXosc2Scene import parse_xosc
from Utils.Constant import USE_TEST_LOGIC


class ScenarioLoader:

    def __init__(self, dataDir: str, filter_scenes: list = []):
        self.dataDir = dataDir
        self.filter_scenes = filter_scenes

    def loadAll(self) -> List[Dict[str, Any]]:
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
            for folder_item in os.listdir(self.dataDir):
                scene_folder = os.path.join(self.dataDir, folder_item)
                xodr_path, xosc_path = None, None
                for file_item in os.listdir(scene_folder):
                    if file_item.endswith(".xodr"):
                        xodr_path = os.path.join(scene_folder, file_item)

                    if file_item.endswith(".xosc"):
                        xosc_path = os.path.join(scene_folder, file_item)

                    if xodr_path and xosc_path:
                        jsonFiles.append(xosc_path)
                        break

        jsonFiles = sorted(jsonFiles)

        scenarios = []
        for filename in jsonFiles:
            if USE_TEST_LOGIC:
                filepath = os.path.join(self.dataDir, filename)
                scenario = self.loadFile(filepath)
            else:
                scenario = parse_xosc(filename)
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

    def loadFile(self, filepath: str) -> Dict[str, Any]:
        """加载单个 JSON 文件"""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
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
            }

        return {"vehicles": vehicles}
