"""
MultiVehicleInference —— 多车推理管理器

灵活版：
  - 支持动态增删车辆
  - 支持从 xosc 文件批量加载路径
  - model.predict 返回的 (x, y, heading, speed) 可直接 set 给 Tessng 背景车
  - 不绑定 TessAutoPyInterface，返回通用的 dict 格式

用法：
    infer = MultiVehicleInference("tessng_dqn/model")

    # 从 xosc 加载
    infer.loadFromXosc("scenarios/scene_01.xosc")
    infer.loadFromXoscDir("scenarios/")

    # 或手动添加
    infer.addVehicle("bg_car_1", [(x0,y0), (x1,y1), ...], speed=10.0)

    # 每帧推理
    results = infer.stepAll(tessngVehicles, p2m, dt=0.1)
    # results = {"bg_car_1": {"x": ..., "y": ..., "heading": ..., "speed": ...}, ...}
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from Utils.Constant import WHEEL_BASE, MAX_SPEED


# 离散动作 (DQN使用)
ACTION_TO_CONTROL = {
    0: (0.0, -0.3),
    1: (0.0, 0.0),
    2: (0.0, 0.3),
    3: (3.0, 0.0),
    4: (-3.0, 0.0),
}



@dataclass
class VehicleAgent:
    """单辆车的运行状态"""
    name: str
    rawPath: List[Tuple[float, float]]              # 原始路径点
    smoothedPath: List[Tuple[float, float]] = field(default_factory=list)
    speed: float = 10.0
    progress: float = 0.0
    prevHeading: float = 0.0
    totalLength: float = 0.0
    alive: bool = True

    # 最新的输出状态（供外部读取）
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0


class MultiVehicleInference:

    def __init__(self, modelPath: str = None, algo: str = "PPO"):
        """
        Args:
            modelPath: DQN/PPO 模型路径（不含 .zip），传 None 则延迟加载
            algo: 模型算法 ("DQN" 或 "PPO")
        """
        self.algo = algo
        self.model = None
        self.agents: Dict[str, VehicleAgent] = {}

        if modelPath:
            self.loadModel(modelPath)

    # ============================================================
    #  模型管理
    # ============================================================

    def loadModel(self, modelPath: str):
        if self.algo == "PPO":
            from stable_baselines3 import PPO
            self.model = PPO.load(modelPath)
        else:
            from stable_baselines3 import DQN
            self.model = DQN.load(modelPath)
        print(f"[MultiInfer] 模型已加载 ({self.algo}): {modelPath}")

    # ============================================================
    #  车辆管理
    # ============================================================

    def addVehicle(
        self,
        name: str,
        path: List[Tuple[float, float]],
        speed: float = 10.0,
        smoothInterval: float = 1.0,
        color: str = "#048dc7", 
    ) -> bool:
        """
        添加一辆车

        Args:
            name: 车辆唯一标识
            path: 路径点 [(x,y), ...]
            speed: 初始速度 m/s
            smoothInterval: 平滑后的点间距（米）

        Returns:
            是否添加成功
        """
        if len(path) < 2:
            print(f"[MultiInfer] 跳过 '{name}': 路径点不足2个")
            return False

        smoothed = self._smoothPath(path, smoothInterval)
        totalLen = self._pathLength(smoothed)

        if totalLen < 1.0:
            print(f"[MultiInfer] 跳过 '{name}': 路径太短 ({totalLen:.1f}m)")
            return False

        agent = VehicleAgent(
            name=name,
            rawPath=path,
            smoothedPath=smoothed,
            speed=speed,
            totalLength=totalLen,
        )

        # 初始位置和航向
        agent.x, agent.y, agent.heading = self._posOnPath(smoothed, 0.0)
        agent.prevHeading = agent.heading

        self.agents[name] = agent
        print(f"[MultiInfer] 添加 '{name}': {len(path)} 原始点 → {len(smoothed)} 平滑点, {totalLen:.1f}m")
        return True

    def removeVehicle(self, name: str):
        self.agents.pop(name, None)

    def clearAll(self):
        self.agents.clear()

    def loadFromXosc(self, filepath: str, defaultSpeed: float = 10.0):
        """从单个 xosc 文件加载车辆路径"""
        from XoscLoader import XoscLoader
        vehicles = XoscLoader.loadFile(filepath)
        count = 0
        for name, path in vehicles.items():
            if self.addVehicle(name, path, speed=defaultSpeed):
                count += 1
        print(f"[MultiInfer] 从 {filepath} 加载了 {count} 辆车")

    def loadFromXoscDir(self, dirpath: str, defaultSpeed: float = 10.0):
        """从目录下所有 xosc 文件加载车辆路径"""
        from XoscLoader import XoscLoader
        vehicles = XoscLoader.loadDir(dirpath)
        count = 0
        for name, path in vehicles.items():
            if self.addVehicle(name, path, speed=defaultSpeed):
                count += 1
        print(f"[MultiInfer] 从 {dirpath} 共加载 {count} 辆车")

    # ============================================================
    #  每帧推理
    # ============================================================

    def stepAll(
        self,
        tessngVehicles=None,
        p2m=None,
        dt: float = 0.1,
    ) -> Dict[str, dict]:
        """
        所有存活车辆各自推理一步

        Returns:
            {name: {"x": float, "y": float, "heading": float, "speed": float}}
            x/y 已做 y 反转（y=-y），heading 是 Tessng 导航系角度。
            可直接用来 set 给 Tessng 背景车。
        """
        if self.model is None:
            raise RuntimeError("模型未加载，请先调 loadModel()")

        results = {}

        for name, agent in self.agents.items():
            if not agent.alive:
                continue

            # 1. 构建 obs
            obs = self._buildObs(agent, tessngVehicles, p2m)

            # 2. 推理
            action, _ = self.model.predict(obs, deterministic=True)
            
            if self.algo == "PPO":
                # PPO 连续动作 [accel, steer]
                accel, steer = float(action[0]), float(action[1])
                print(f"[MultiInfer] {name} PPO 动作: accel={accel:.2f}, steer={steer:.2f}")
            else:
                # DQN 离散动作索引 -> [accel, steer]
                accel, steer = ACTION_TO_CONTROL[int(action)]

            # 3. 更新速度
            agent.speed += accel * dt
            agent.speed = max(0.0, min(agent.speed, MAX_SPEED))

            # 4. 推进
            # 脱离预设路径的硬绑定，使用 steer
            # 引入车辆运动学模型 (Kinematic Bicycle Model，自动驾驶中用于模拟四轮小汽车的经典单辙模型)
            # 假设小汽车轴距为 2.8 米
            yaw_rate = (agent.speed * math.tan(steer)) / WHEEL_BASE
            agent.prevHeading = agent.heading
            agent.heading = (agent.prevHeading + math.degrees(yaw_rate * dt)) % 360.0
            
            heading_rad = math.radians(agent.heading)
            agent.x += agent.speed * math.sin(heading_rad) * dt
            agent.y += agent.speed * math.cos(heading_rad) * dt

            agent.progress += agent.speed * dt

            # 5. 到终点
            if agent.progress >= agent.totalLength:
                agent.progress = agent.totalLength
                agent.alive = False

            # 6. 定位 (不再强制从路径读取坐标)
            # x, y, heading = self._posOnPath(agent.smoothedPath, agent.progress)
            # agent.x = x
            # agent.y = y
            # agent.heading = heading
            # agent.prevHeading = heading

            # 7. 输出（JSON 里 y 已是 GUI 坐标，不需要反转）
            results[name] = {
                "x": agent.x,
                "y": agent.y,
                "heading": agent.heading,
                "speed": agent.speed,
            }

        return results

    def getInitialStates(self) -> Dict[str, dict]:
        """所有车的初始状态（首帧创建用）"""
        results = {}
        for name, agent in self.agents.items():
            results[name] = {
                "x": agent.x,
                "y": agent.y,
                "heading": agent.heading,
                "speed": agent.speed,
            }
        return results

    def resetAll(self, speed: float = 10.0):
        """重置所有车到起点"""
        for name, agent in self.agents.items():
            agent.speed = speed
            agent.progress = 0.0
            agent.alive = True
            agent.x, agent.y, agent.heading = self._posOnPath(agent.smoothedPath, 0.0)
            agent.prevHeading = agent.heading
            if name.endswith("_ego") or name == "ego":
                print(f"[MultiInfer] 重置 '{name}' 到起点: x={agent.x:.1f}, y={agent.y:.1f}, heading={agent.heading:.1f}")

    @property
    def allFinished(self) -> bool:
        return len(self.agents) > 0 and all(not a.alive for a in self.agents.values())

    @property
    def aliveCount(self) -> int:
        return sum(1 for a in self.agents.values() if a.alive)

    # ============================================================
    #  观测构建
    # ============================================================

    def _buildObs(self, agent, tessngVehicles, p2m) -> np.ndarray:
        obs = np.zeros(94, dtype=np.float32)

        # 自车状态（沿路径走，近似居中）
        obs[0] = 0.5
        obs[1] = 0.5
        obs[2] = 0.5
        obs[3] = 0.5
        obs[4] = np.clip(agent.speed / MAX_SPEED, 0, 1)
        obs[5] = 0.5
        obs[6] = 0.5
        obs[7] = 0.5
        obs[8] = 0.5

        # 导航
        x, y, headingRad = self._posOnPathRad(agent.smoothedPath, agent.progress)
        for i in range(5):
            fp = min(agent.progress + (i+1) * 5.0, agent.totalLength)
            fx, fy, _ = self._posOnPathRad(agent.smoothedPath, fp)
            dx, dy = fx - x, fy - y
            cosH, sinH = math.cos(headingRad), math.sin(headingRad)
            localX = dx * cosH + dy * sinH
            localY = -dx * sinH + dy * cosH
            obs[9 + i*2] = np.clip(localX / 100.0, 0, 1)
            obs[9 + i*2 + 1] = np.clip(localY / 200.0 + 0.5, 0, 1)

        obs[19] = 0.5
        obs[20] = 0.5
        obs[21] = 0.5

        # 雷达
        for i in range(72):
            rayAngle = headingRad + i * (2 * math.pi / 72)
            minDist = 30.0

            # 检测其他 agent
            for otherName, otherAgent in self.agents.items():
                if otherName == agent.name or not otherAgent.alive:
                    continue
                dist = math.sqrt((otherAgent.x - x)**2 + (otherAgent.y - y)**2)
                if dist < minDist:
                    rayDx, rayDy = math.cos(rayAngle), math.sin(rayAngle)
                    fx, fy = x - otherAgent.x, y - otherAgent.y
                    b = 2 * (fx * rayDx + fy * rayDy)
                    c = fx*fx + fy*fy - 2.5*2.5
                    disc = b*b - 4*c
                    if disc >= 0:
                        t = (-b - math.sqrt(disc)) / 2
                        if 0 < t < minDist:
                            minDist = t

            # 检测 Tessng 车辆
            if tessngVehicles and p2m:
                for v in tessngVehicles:
                    vPos = v.pos()
                    vx, vy = p2m(vPos.x()), p2m(vPos.y())
                    dist = math.sqrt((vx - x)**2 + (vy - y)**2)
                    if 0.1 < dist < minDist:
                        rayDx, rayDy = math.cos(rayAngle), math.sin(rayAngle)
                        fx, fy = x - vx, y - vy
                        b = 2 * (fx * rayDx + fy * rayDy)
                        c = fx*fx + fy*fy - 2.5*2.5
                        disc = b*b - 4*c
                        if disc >= 0:
                            t = (-b - math.sqrt(disc)) / 2
                            if 0 < t < minDist:
                                minDist = t

            obs[22 + i] = minDist / 30.0

        return obs

    # ============================================================
    #  路径工具
    # ============================================================

    def _posOnPath(self, path, dist) -> Tuple[float, float, float]:
        """返回 (x, y, heading_tessng_deg)  JSON坐标(y=-gui_y)"""
        accumulated = 0.0
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i+1]
            segLen = math.sqrt((bx-ax)**2 + (by-ay)**2)
            if accumulated + segLen >= dist:
                ratio = (dist - accumulated) / segLen if segLen > 1e-12 else 0.0
                x = ax + ratio * (bx - ax)
                y = ay + ratio * (by - ay)
                heading = math.degrees(math.atan2(bx-ax, by-ay)) % 360.0
                return x, y, heading
            accumulated += segLen
        bx, by = path[-1]
        ax, ay = path[-2]
        heading = math.degrees(math.atan2(bx-ax, by-ay)) % 360.0
        return bx, by, heading

    def _posOnPathRad(self, path, dist) -> Tuple[float, float, float]:
        """返回 (x, y, heading_math_rad)"""
        accumulated = 0.0
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i+1]
            segLen = math.sqrt((bx-ax)**2 + (by-ay)**2)
            if accumulated + segLen >= dist:
                ratio = (dist - accumulated) / segLen if segLen > 1e-12 else 0.0
                x = ax + ratio * (bx - ax)
                y = ay + ratio * (by - ay)
                return x, y, math.atan2(by-ay, bx-ax)
            accumulated += segLen
        return path[-1][0], path[-1][1], 0.0

    @staticmethod
    def _pathLength(path) -> float:
        return sum(
            math.sqrt((path[i+1][0]-path[i][0])**2 + (path[i+1][1]-path[i][1])**2)
            for i in range(len(path)-1)
        )

    @staticmethod
    def _smoothPath(points, interval=1.0):
        if len(points) < 2:
            return list(points)

        def catmullRom(p0, p1, p2, p3, t):
            t2, t3 = t*t, t*t*t
            x = 0.5*((2*p1[0])+(-p0[0]+p2[0])*t+(2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2+(-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
            y = 0.5*((2*p1[1])+(-p0[1]+p2[1])*t+(2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2+(-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
            return (x, y)

        dense = []
        n = len(points)
        for i in range(n-1):
            p0, p1, p2, p3 = points[max(i-1,0)], points[i], points[i+1], points[min(i+2,n-1)]
            segLen = math.sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)
            numSamples = max(int(segLen/interval*3), 10)
            for j in range(numSamples):
                dense.append(catmullRom(p0, p1, p2, p3, j/numSamples))
        dense.append(points[-1])

        result = [dense[0]]
        acc = 0.0
        for i in range(1, len(dense)):
            dx = dense[i][0]-dense[i-1][0]
            dy = dense[i][1]-dense[i-1][1]
            segLen = math.sqrt(dx*dx+dy*dy)
            remaining = segLen
            prevX, prevY = dense[i-1]
            while acc + remaining >= interval:
                step = interval - acc
                ratio = step/remaining if remaining > 1e-12 else 0.0
                newX = prevX + ratio*(dense[i][0]-prevX)
                newY = prevY + ratio*(dense[i][1]-prevY)
                result.append((newX, newY))
                prevX, prevY = newX, newY
                remaining -= step
                acc = 0.0
            acc += remaining
        return result