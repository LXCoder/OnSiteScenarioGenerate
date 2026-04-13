import math

from AutoPilot.Player.BasePlayer import BasePlayer
from AutoPilot.Player.VehicleState import VehicleState
from AutoPilot.Player.Observation import Observation
from Utils.NavigationCalculator import NavigationCalculator


class TestPlayer(BasePlayer):

    def __init__(self, player_id: str = "test"):
        super().__init__(player_id)
        # 与 MySimulator 相同的预设中心线路径
        # self.centerLine = [
        #     (-637.8400268554688, 160.61000061035156),
        #     (-642.7520141601562, 154.64199829101562),
        #     (-645.147216796875, 151.72630310058594),
        #     (-649.8162231445312, 146.03147888183594),
        #     (-651.8980102539062, 143.49000549316406),
        #     (-655.1699829101562, 139.49000549316406),
        #     (-657.84, 124.38),
        #     (-658.306, 114.059),
        #     (-626.554, 84.003),
        #     (-622.084, 56.875)
        # ]
        self.centerLine = [
            (-650.047, 287.819),
            (-662.097, 272.913),
            (-680.481, 248.938),
            (-693.14, 233.513),
            (-711.580, 211.293),
            (-724.399, 194.875),
            (-733.395, 183.068),
            (-746.664, 167.212),
            (-754.873, 148.658),
            (-759.146, 129.204),
            (-758.247, 108.064),
            (-751.612, 87.486),
            (-742.391, 72.417),
            (-728.110, 54.201),
            (-714.054, 36.321),
            (-694.038, 10.683)
        ]
        
        # 将离散点平滑为密集等距路径（每1米一个点）
        self.smoothedLine = self.smoothCenterLine(self.centerLine, interval=1.0)
        
        self.dt = 0.1
        self.egoSpeed = 10.0
        self.egoProgress = 0.0
        self.accel = 0.0

    @staticmethod
    def smoothCenterLine(points, interval=1.0):
        if len(points) < 2:
            return list(points)

        # Catmull-Rom 插值
        def catmullRom(p0, p1, p2, p3, t):
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * t +
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            y = 0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * t +
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            return (x, y)

        # 生成密集点
        dense = []
        n = len(points)
        for i in range(n - 1):
            p0 = points[max(i - 1, 0)]
            p1 = points[i]
            p2 = points[i + 1]
            p3 = points[min(i + 2, n - 1)]

            # 这段的弧长估计
            segLen = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
            numSamples = max(int(segLen / interval * 3), 10)

            for j in range(numSamples):
                t = j / numSamples
                dense.append(catmullRom(p0, p1, p2, p3, t))

        dense.append(points[-1])

        # 再等距重采样
        return NavigationCalculator.sparsifyByDistance(dense, interval)

    def positionOnCenterLine(self, dist):
        """根据累计距离在平滑路径上插值"""
        cl = self.smoothedLine
        accumulated = 0.0

        for i in range(len(cl) - 1):
            ax, ay = cl[i]
            bx, by = cl[i + 1]
            segLen = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)

            if accumulated + segLen >= dist:
                remain = dist - accumulated
                ratio = remain / segLen if segLen > 1e-12 else 0.0
                x = ax + ratio * (bx - ax)
                y = ay + ratio * (by - ay)

                # 用反转后的 dy 算航向（因为 Tessng GUI y 轴向下）
                mathRad = math.atan2(-(by - ay), bx - ax)
                heading = (90.0 - math.degrees(mathRad)) % 360.0

                return x, y, heading

            accumulated += segLen

        ax, ay = cl[-2]
        bx, by = cl[-1]
        mathRad = math.atan2(-(by - ay), bx - ax)
        heading = (90.0 - math.degrees(mathRad)) % 360.0
        return bx, by, heading

    def predict(self, obs: Observation) -> None:
        """接收观测信息，更新内部决策（简单PID/目标速度跟随）"""
        target_speed = 15.0
        speed_diff = target_speed - self.egoSpeed
        
        # 简单比例控制计算加速度
        self.accel = max(-5.0, min(5.0, speed_diff * 0.5))

    def act(self) -> VehicleState:
        """输出当前帧的车辆状态"""
        # 1. 更新速度与距离
        self.egoSpeed += self.accel * self.dt
        self.egoSpeed = max(0.0, min(self.egoSpeed, 33.3)) # 限制最大速度
        
        moveDist = self.egoSpeed * self.dt
        self.egoProgress += moveDist
        
        # 2. 计算当前里程在路径上的插值位置
        if not self.smoothedLine or len(self.smoothedLine) < 2:
            return VehicleState(x=-653.0, y=-140.0, heading=0.0, speed=0.0)
            
        newX, newY, heading = self.positionOnCenterLine(self.egoProgress)
        
        # 3. 构造并返回最新的自车状态 (与 MySimulator.py 保持 y 坐标反转逻辑一致)
        return VehicleState(
            x=newX, 
            y=-newY, 
            heading=heading, 
            speed=self.egoSpeed
        )
