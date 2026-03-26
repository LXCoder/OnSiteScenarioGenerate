"""
NavigationCalculator —— 导航信息计算

从整条车道中心线截取自车前方 N 个 checkpoint，计算：
  - 前向差距 / 侧向差距（归一化）
  - 弯曲半径（归一化）
  - 弯曲方向（归一化）
  - 车道角度差（归一化）

输入约定：
  - 中心线坐标单位为米
  - 自车坐标单位为米
  - heading 为度（正北0，顺时针正）

输出约定：
  - 所有结果已归一化到 [0, 1]
"""

import math
from typing import List, Tuple
from dataclasses import dataclass, field


# 归一化常量
MAX_NAV_DIST = 100.0
MAX_CURVATURE_RAD = 1000.0


@dataclass
class NavigationResult:
    """导航计算结果（全部归一化到 [0, 1]）"""
    forwardGaps: List[float] = field(default_factory=list)      # 各 checkpoint 前向差距
    lateralGaps: List[float] = field(default_factory=list)      # 各 checkpoint 侧向差距
    curvatureRadius: float = 1.0    # 弯曲半径，1.0=直道
    curvatureDirection: float = 0.5 # 弯曲方向，0.5=直道，1.0=顺时针，0.0=逆时针
    laneAngleDiff: float = 0.5      # 车道角度差，0.5=无角度变化


class NavigationCalculator:

    @staticmethod
    def compute(
        egoX: float,
        egoY: float,
        egoHeadingDeg: float,
        centerLine: List[Tuple[float, float]],
        numCheckpoints: int = 5,
    ) -> NavigationResult:
        """
        从中心线截取前方 checkpoint 并计算导航信息

        Args:
            egoX, egoY: 自车坐标（米）
            egoHeadingDeg: 自车航向角（度，正北0，顺时针正）
            centerLine: 整条车道中心线 [(x,y), ...]（米）
            numCheckpoints: 截取前方几个点

        Returns:
            NavigationResult（全部归一化）
        """
        result = NavigationResult()

        # 1. 截取前方 checkpoint
        checkpoints = NavigationCalculator.extractForwardPoints(
            egoX, egoY, egoHeadingDeg, centerLine, numCheckpoints
        )

        if not checkpoints:
            return result

        # 2. 前向/侧向差距
        egoHeadingRad = NavigationCalculator.tessngDegToMathRad(egoHeadingDeg)

        for cx, cy in checkpoints:
            dx, dy = NavigationCalculator.globalToLocal(
                cx, cy, egoX, egoY, egoHeadingRad
            )
            # 前向：clip(dx / MAX_NAV_DIST, 0, 1)
            fwd = max(0.0, min(1.0, dx / MAX_NAV_DIST))
            # 侧向：clip((dy / (2 * MAX_NAV_DIST)) + 0.5, 0, 1)
            lat = max(0.0, min(1.0, dy / (2.0 * MAX_NAV_DIST) + 0.5))
            result.forwardGaps.append(fwd)
            result.lateralGaps.append(lat)

        # 3. 弯曲半径 + 方向（至少需要3个点）
        if len(checkpoints) >= 3:
            p1 = checkpoints[0]
            p2 = checkpoints[len(checkpoints) // 2]
            p3 = checkpoints[-1]

            radius, direction = NavigationCalculator.curvatureFromThreePoints(p1, p2, p3)

            # 归一化半径：clip(radius / MAX_CURVATURE_RAD, 0, 1)
            safeRadius = min(radius, MAX_CURVATURE_RAD)
            result.curvatureRadius = max(0.0, min(1.0, safeRadius / MAX_CURVATURE_RAD))

            # 归一化方向：(direction + 1) / 2
            result.curvatureDirection = (direction + 1.0) / 2.0

        # 4. 车道角度差（始末切线方向之差）
        if len(checkpoints) >= 2:
            startHeading = math.atan2(
                checkpoints[1][1] - checkpoints[0][1],
                checkpoints[1][0] - checkpoints[0][0],
            )
            endHeading = math.atan2(
                checkpoints[-1][1] - checkpoints[-2][1],
                checkpoints[-1][0] - checkpoints[-2][0],
            )
            diffRad = NavigationCalculator.normalizeAngle(endHeading - startHeading)
            # 归一化：clip((diff / π + 1) / 2, 0, 1)
            result.laneAngleDiff = max(0.0, min(1.0, (diffRad / math.pi + 1.0) / 2.0))

        return result

    @staticmethod
    def extractForwardPoints(
        egoX: float,
        egoY: float,
        egoHeadingDeg: float,
        centerLine: List[Tuple[float, float]],
        numPoints: int,
    ) -> List[Tuple[float, float]]:
        """
        从中心线中截取自车前方的 N 个点

        策略：
          1. 找到中心线上离自车最近的点（投影点）
          2. 从该点之后的点中，取前方 numPoints 个
        """
        if len(centerLine) < 2:
            return []

        # 找最近线段
        bestSeg = 0
        bestDistSq = float("inf")

        for i in range(len(centerLine) - 1):
            ax, ay = centerLine[i]
            bx, by = centerLine[i + 1]
            abx, aby = bx - ax, by - ay
            abSq = abx * abx + aby * aby
            if abSq < 1e-12:
                continue
            t = ((egoX - ax) * abx + (egoY - ay) * aby) / abSq
            t = max(0.0, min(1.0, t))
            qx = ax + t * abx
            qy = ay + t * aby
            dSq = (egoX - qx) ** 2 + (egoY - qy) ** 2
            if dSq < bestDistSq:
                bestDistSq = dSq
                bestSeg = i

        # 从投影段的下一个点开始取
        startIdx = bestSeg + 1
        endIdx = min(startIdx + numPoints, len(centerLine))

        if startIdx >= len(centerLine):
            return []

        return list(centerLine[startIdx:endIdx])

    @staticmethod
    def sparsifyByDistance(
        points: List[Tuple[float, float]],
        interval: float,
    ) -> List[Tuple[float, float]]:
        """
        将密集点序列按等距间隔稀疏化

        沿折线累计弧长，每隔 interval 米插值取一个点。

        Args:
            points: 原始密集点序列 [(x,y), ...]
            interval: 采样间隔（米）

        Returns:
            稀疏化后的点序列（第一个点保留，之后每隔 interval 取一个）
        """
        if len(points) < 2:
            return list(points)

        result = [points[0]]
        accumulated = 0.0

        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            segLen = math.sqrt(dx * dx + dy * dy)

            remaining = segLen
            prevX, prevY = points[i - 1]

            while accumulated + remaining >= interval:
                # 还需要走多远到下一个采样点
                step = interval - accumulated
                ratio = step / remaining if remaining > 1e-12 else 0.0

                # 在当前线段上插值
                newX = prevX + ratio * (points[i][0] - prevX)
                newY = prevY + ratio * (points[i][1] - prevY)
                result.append((newX, newY))

                # 更新
                prevX, prevY = newX, newY
                remaining -= step
                accumulated = 0.0

            accumulated += remaining

        return result

    @staticmethod
    def globalToLocal(
        targetX: float, targetY: float,
        egoX: float, egoY: float,
        egoHeadingRad: float,
    ) -> Tuple[float, float]:
        """
        全局坐标 → 自车局部坐标

        局部坐标系：X 轴朝前（航向方向），Y 轴朝右
        egoHeadingRad: 数学系弧度（由 tessngDegToMathRad 转换）
        """
        dx = targetX - egoX
        dy = targetY - egoY
        cosH = math.cos(egoHeadingRad)
        sinH = math.sin(egoHeadingRad)
        localX = dx * cosH + dy * sinH
        localY = -dx * sinH + dy * cosH
        return localX, localY

    @staticmethod
    def curvatureFromThreePoints(
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float],
    ) -> Tuple[float, int]:
        """
        三点拟合圆弧 → (曲率半径, 弯曲方向)

        弯曲方向：+1 顺时针, -1 逆时针, 0 直道
        """
        ax, ay = p2[0] - p1[0], p2[1] - p1[1]
        bx, by = p3[0] - p2[0], p3[1] - p2[1]

        cross = ax * by - ay * bx
        aLen = math.sqrt(ax * ax + ay * ay)
        bLen = math.sqrt(bx * bx + by * by)
        cLen = math.sqrt((p3[0] - p1[0]) ** 2 + (p3[1] - p1[1]) ** 2)

        area2 = abs(cross)
        if area2 < 1e-6:
            # 三点共线 → 直道
            return MAX_CURVATURE_RAD, 0

        radius = (aLen * bLen * cLen) / (2.0 * area2)

        # 叉积负 → 顺时针（在屏幕坐标系下）
        if cross < -1e-6:
            direction = 1
        elif cross > 1e-6:
            direction = -1
        else:
            direction = 0

        return radius, direction

    @staticmethod
    def tessngDegToMathRad(tessngDeg: float) -> float:
        """Tessng 导航系角度（正北0，顺时针正）→ 数学系弧度（正东0，逆时针正）"""
        return math.radians(90.0 - tessngDeg)

    @staticmethod
    def normalizeAngle(angle: float) -> float:
        """归一化角度到 [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle