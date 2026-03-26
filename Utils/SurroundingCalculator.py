"""
SurroundingCalculator —— 周车交互信息计算

从 Tessng vehicle 列表计算：
  - 周车前向/侧向差距（归一化）
  - 周车纵向/横向速度差（归一化）
  - 72维雷达点云（归一化）

输入约定：
  - 自车和周车坐标通过 p2m 转换为米
  - heading 为度（正北0，顺时针正）
  - speed 为 m/s（currSpeed 返回的是 km/h 还是 m/s 需确认）
  - length/width 单位为厘米，内部转米

输出约定：
  - 所有结果已归一化到 [0, 1]
"""

import math
from typing import List, Tuple, Optional
from dataclasses import dataclass, field


# 归一化常量
MAX_NAV_DIST = 100.0
MAX_LANE_WIDTH = 4.0
MAX_RELATIVE_SPEED = 40.0
MAX_RADAR_DIST = 30.0
NUM_RADAR_RAYS = 72
RADAR_ANGLE_STEP = 2 * math.pi / NUM_RADAR_RAYS

# 周车过滤距离（米）
FILTER_RADIUS = 50.0


@dataclass
class NearbyVehicleInfo:
    """单辆周车的归一化交互信息"""
    forwardGap: float = 0.5         # 前向差距，0.5=正旁边
    lateralGap: float = 0.5         # 侧向差距，0.5=正前方
    longitudinalSpeedDiff: float = 0.5  # 纵向速度差，0.5=相对静止
    lateralSpeedDiff: float = 0.5       # 横向速度差，0.5=相对静止


@dataclass
class SurroundingResult:
    """周车交互计算结果"""
    nearbyVehicles: List[NearbyVehicleInfo] = field(default_factory=list)
    radar: List[float] = field(default_factory=list)  # 72维，[0,1]


class SurroundingCalculator:

    @staticmethod
    def fromTessngVehicles(
        egoVehicle,
        allVehicles,
        p2m,
        maxNearby: int = 8,
        filterRadius: float = FILTER_RADIUS,
    ) -> SurroundingResult:
        """
        从 Tessng vehicle 列表计算周车交互信息

        Args:
            egoVehicle: 自车 IVehicle 对象
            allVehicles: 所有车辆列表（含自车）
            p2m: Tessng 的 p2m 转换函数
            maxNearby: 最多取几辆最近周车
            filterRadius: 过滤半径（米）

        Returns:
            SurroundingResult
        """
        egoPos = egoVehicle.pos()
        egoX = p2m(egoPos.x())
        egoY = p2m(egoPos.y())
        egoHeadingDeg = egoVehicle.angle()
        egoSpeed = egoVehicle.currSpeed()
        egoId = egoVehicle.id()

        egoHeadingRad = SurroundingCalculator.tessngDegToMathRad(egoHeadingDeg)

        # 收集周车原始数据
        others = []
        for v in allVehicles:
            if v.id() == egoId:
                continue
            vPos = v.pos()
            vx = p2m(vPos.x())
            vy = p2m(vPos.y())

            # 距离过滤
            dist = math.sqrt((vx - egoX) ** 2 + (vy - egoY) ** 2)
            if dist > filterRadius:
                continue

            others.append({
                "x": vx,
                "y": vy,
                "headingDeg": v.angle(),
                "speed": v.currSpeed(),
                "length": v.length() / 100.0,   # 厘米→米
                "width": v.width() / 100.0,
                "dist": dist,
            })

        # 按距离排序，取最近的 maxNearby 辆
        others.sort(key=lambda o: o["dist"])
        nearest = others[:maxNearby]

        result = SurroundingResult()

        # 1. 周车差距和速度差
        for other in nearest:
            info = SurroundingCalculator.computeRelative(
                egoX, egoY, egoHeadingRad, egoSpeed, egoHeadingDeg, other
            )
            result.nearbyVehicles.append(info)

        # 2. 72维雷达
        result.radar = SurroundingCalculator.castRadar(
            egoX, egoY, egoHeadingRad, others
        )

        return result

    @staticmethod
    def computeRelative(
        egoX: float,
        egoY: float,
        egoHeadingRad: float,
        egoSpeed: float,
        egoHeadingDeg: float,
        other: dict,
    ) -> NearbyVehicleInfo:
        """计算单辆周车的归一化交互信息"""
        info = NearbyVehicleInfo()

        # 相对位置 → 自车局部坐标
        deltaX, deltaY = SurroundingCalculator.globalToLocal(
            other["x"], other["y"],
            egoX, egoY, egoHeadingRad,
        )

        # 归一化位置
        info.forwardGap = max(0.0, min(1.0,
            deltaX / (2.0 * MAX_NAV_DIST) + 0.5))
        info.lateralGap = max(0.0, min(1.0,
            deltaY / (2.0 * MAX_LANE_WIDTH) + 0.5))

        # 相对速度 → 自车局部坐标
        otherHeadingRad = SurroundingCalculator.tessngDegToMathRad(other["headingDeg"])

        egoVx = egoSpeed * math.cos(egoHeadingRad)
        egoVy = egoSpeed * math.sin(egoHeadingRad)
        otherVx = other["speed"] * math.cos(otherHeadingRad)
        otherVy = other["speed"] * math.sin(otherHeadingRad)

        dvxGlobal = otherVx - egoVx
        dvyGlobal = otherVy - egoVy

        cosH = math.cos(egoHeadingRad)
        sinH = math.sin(egoHeadingRad)
        dvxLocal = dvxGlobal * cosH + dvyGlobal * sinH
        dvyLocal = -dvxGlobal * sinH + dvyGlobal * cosH

        # 归一化速度差
        info.longitudinalSpeedDiff = max(0.0, min(1.0,
            dvxLocal / (2.0 * MAX_RELATIVE_SPEED) + 0.5))
        info.lateralSpeedDiff = max(0.0, min(1.0,
            dvyLocal / (2.0 * MAX_RELATIVE_SPEED) + 0.5))

        return info

    @staticmethod
    def castRadar(
        egoX: float,
        egoY: float,
        egoHeadingRad: float,
        others: List[dict],
    ) -> List[float]:
        """
        72条射线雷达扫描

        从自车中心发出72条射线（每5°），检测与周车包围圆的交点。
        """
        radar = []

        for i in range(NUM_RADAR_RAYS):
            rayAngle = egoHeadingRad + i * RADAR_ANGLE_STEP
            rayDx = math.cos(rayAngle)
            rayDy = math.sin(rayAngle)

            minDist = MAX_RADAR_DIST

            for other in others:
                # 包围圆半径：取长宽较大者的一半
                radius = max(other["length"], other["width"]) / 2.0

                hit = SurroundingCalculator.rayCircleIntersect(
                    egoX, egoY, rayDx, rayDy,
                    other["x"], other["y"], radius,
                )
                if hit is not None and hit < minDist:
                    minDist = hit

            # 归一化：未命中 → 1.0
            radar.append(minDist / MAX_RADAR_DIST)

        return radar

    @staticmethod
    def rayCircleIntersect(
        ox: float, oy: float,
        dx: float, dy: float,
        cx: float, cy: float,
        r: float,
    ) -> Optional[float]:
        """射线与圆的交点检测，返回最近交点距离"""
        fx = ox - cx
        fy = oy - cy
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r

        disc = b * b - 4.0 * a * c
        if disc < 0:
            return None

        sqrtDisc = math.sqrt(disc)
        t1 = (-b - sqrtDisc) / (2.0 * a)
        t2 = (-b + sqrtDisc) / (2.0 * a)

        if t1 > 0:
            return t1
        if t2 > 0:
            return t2
        return None

    @staticmethod
    def globalToLocal(
        targetX: float, targetY: float,
        egoX: float, egoY: float,
        egoHeadingRad: float,
    ) -> Tuple[float, float]:
        """全局 → 自车局部坐标（X朝前，Y朝右）"""
        dx = targetX - egoX
        dy = targetY - egoY
        cosH = math.cos(egoHeadingRad)
        sinH = math.sin(egoHeadingRad)
        localX = dx * cosH + dy * sinH
        localY = -dx * sinH + dy * cosH
        return localX, localY

    @staticmethod
    def tessngDegToMathRad(tessngDeg: float) -> float:
        """Tessng导航系（正北0，顺时针正）→ 数学系弧度"""
        return math.radians(90.0 - tessngDeg)
