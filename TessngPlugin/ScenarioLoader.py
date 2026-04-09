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


class ScenarioLoader:

    def __init__(self, dataDir: str):
        self.dataDir = dataDir

    def loadAll(self) -> List[Dict[str, Any]]:
        """
        加载目录下所有 JSON 文件

        Returns:
            [{"file": "xxx.json", "vehicles": {name: {"path": [(x,y),...], "speed": float}}}, ...]
        """
        if not os.path.isdir(self.dataDir):
            print(f"[ScenarioLoader] 目录不存在: {self.dataDir}")
            return []

        scenarios = []
        jsonFiles = sorted([f for f in os.listdir(self.dataDir) if f.endswith('.json')])

        for filename in jsonFiles:
            filepath = os.path.join(self.dataDir, filename)
            scenario = self.loadFile(filepath)
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

            # 转成 tuple 列表
            path = [(float(p[0]), float(p[1])) for p in rawPath]

            if len(path) < 2:
                print(f"[ScenarioLoader] 跳过 {name}: 路径点不足")
                continue

            vehicles[name] = {
                "path": path,
                "speed": speed,
            }

        return {"vehicles": vehicles}
