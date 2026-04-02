"""
YawRateCalculator —— 横摆角速度计算器

通过相邻两帧 heading 差分计算 yaw_rate。
支持多车，内部按 vehicle_id 缓存上一帧 heading。

用法：
    from Utils.YawRateCalculator import YawRateCalculator

    yawRateCalc = YawRateCalculator()

    # afterOneStep 中
    for vehicle in vehicles:
        yaw_rate = yawRateCalc.update(vehicle.id(), vehicle.angle(), dt)
"""

import math
from typing import Dict, Optional


# 归一化常量
MAX_YAW_RATE = 1.0


class YawRateCalculator:

    def __init__(self):
        # vehicle_id → 上一帧 heading（度）
        self._prevHeading: Dict[int, float] = {}

    def update(self, vehicleId: int, headingDeg: float, dt: float) -> float:
        """
        更新并返回 yaw_rate（rad/s）

        Args:
            vehicleId: 车辆 ID
            headingDeg: 当前帧 heading（度，正北0，顺时针正）
            dt: 仿真步长（秒）

        Returns:
            yaw_rate (rad/s)，正值=顺时针旋转
            首帧返回 0.0
        """
        if vehicleId not in self._prevHeading:
            self._prevHeading[vehicleId] = headingDeg
            return 0.0

        prevDeg = self._prevHeading[vehicleId]
        self._prevHeading[vehicleId] = headingDeg

        # 角度差，归一化到 [-180, 180]
        diffDeg = headingDeg - prevDeg
        while diffDeg > 180.0:
            diffDeg -= 360.0
        while diffDeg < -180.0:
            diffDeg += 360.0

        # 转弧度，除以时间
        yawRate = math.radians(diffDeg) / dt
        return yawRate

    def normalize(self, yawRate: float) -> float:
        """
        归一化 yaw_rate 到 [0, 1]

        clip((yaw_rate / (2 * MAX_YAW_RATE)) + 0.5, 0, 1)
        0.5 = 无横摆运动
        """
        return max(0.0, min(1.0, yawRate / (2.0 * MAX_YAW_RATE) + 0.5))

    def reset(self, vehicleId: Optional[int] = None):
        """清除缓存，不传 ID 则清除全部"""
        if vehicleId is None:
            self._prevHeading.clear()
        else:
            self._prevHeading.pop(vehicleId, None)
