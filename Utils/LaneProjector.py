"""
LaneProjector —— 车道投影计算

从车道中心线/边界点序列 + 自车位置，计算：
  - 边界距离（左/右）
  - 车道偏移
  - 角度差

输入约定：
  - Tessng 所有坐标（自车 pos、边界点 breakPoints）单位均为像素，需要 p2m 转米
  - fromTessngVehicle 内部自动完成 p2m 转换
  - 直接调用 project 方法时，调用方需确保所有坐标已转换为米
  - heading 单位为 **角度（degree）**，正北0度顺时针增大

输出约定：
  - 距离类结果单位为 **米**
  - 角度类结果单位为 **弧度**
"""

import math
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class LaneProjectionResult:
    """投影结果（距离单位米）"""
    dist_left: float = 0.0          # 到左边界的距离（米）
    dist_right: float = 0.0         # 到右边界的距离（米）
    lateral_offset: float = 0.0     # 相对中心线的横向偏移（米，正=偏右）
    angle_diff: float = 0.5         # 归一化角度差 [0,1]，0.5=完全平行
    angle_diff_rad: float = 0.0     # 原始角度差（弧度，[-π, π]），供调试用
    road_heading: float = 0.0       # 投影点处的车道方向（度，正北0，顺时针增大）
    proj_x: float = 0.0             # 投影点坐标（米）
    proj_y: float = 0.0
    segment_index: int = 0          # 投影落在中心线的第几段


class LaneProjector:
    """车道投影计算器"""

    # ============================================================
    #  便捷方法：直接传入 Tessng 的 vehicle 对象
    # ============================================================

    @staticmethod
    def fromTessngVehicle(vehicle, p2m) -> Optional[LaneProjectionResult]:
        """
        直接从 Tessng vehicle 对象计算投影

        自动处理 lane / laneConnector 两种情况：
          - vehicle.lane() 不为空 → 车辆在普通车道上
          - vehicle.laneConnector() 不为空 → 车辆在连接段上

        Args:
            vehicle: Tessng 的 IVehicle 对象
            p2m: Tessng 的 p2m 转换函数

        Returns:
            LaneProjectionResult，如果既不在 lane 也不在 laneConnector 则返回 None

        用法：
            from Utils.LaneProjector import LaneProjector
            from Tessng import p2m

            result = LaneProjector.fromTessngVehicle(vehicle, p2m)
        """
        pos = vehicle.pos()
        ego_x = p2m(pos.x())
        ego_y = p2m(pos.y())
        ego_heading_deg = vehicle.angle()

        # 优先取 lane，取不到则取 laneConnector
        lane_obj = vehicle.lane()
        if lane_obj is None:
            lane_obj = vehicle.laneConnector()
        if lane_obj is None:
            return None

        center, left, right = LaneProjector.extractPoints(lane_obj, p2m)

        return LaneProjector.project(
            ego_x, ego_y, ego_heading_deg,
            center, left, right,
        )

    # ============================================================
    #  核心计算方法
    # ============================================================

    @staticmethod
    def project(
        ego_x: float,
        ego_y: float,
        ego_heading_deg: float,
        center_points: List[Tuple[float, float]],
        left_points: List[Tuple[float, float]],
        right_points: List[Tuple[float, float]],
    ) -> LaneProjectionResult:
        """
        计算自车在车道上的投影

        Args:
            ego_x, ego_y: 自车坐标（米，调用方用 p2m 转好）
            ego_heading_deg: 自车航向角（度）
            center_points: 车道中心线 [(x, y), ...]（米）
            left_points: 左边界点序列（米）
            right_points: 右边界点序列（米）

        Returns:
            LaneProjectionResult（距离单位米，角度单位弧度）
        """
        result = LaneProjectionResult()

        proj_x, proj_y, seg_idx, signed_dist, road_heading_math = \
            LaneProjector.projectToPolyline(ego_x, ego_y, center_points)

        result.proj_x = proj_x
        result.proj_y = proj_y
        result.segment_index = seg_idx
        result.lateral_offset = signed_dist

        # road_heading 从 atan2 (数学系：正东0, 逆时针正)
        # 转为 Tessng 导航系 (正北0, 顺时针正)
        # 公式: tessng_deg = (90 - math_deg) % 360
        road_heading_deg = (90.0 - math.degrees(road_heading_math)) % 360.0
        result.road_heading = road_heading_deg

        _, _, _, dist_left_signed, _ = \
            LaneProjector.projectToPolyline(ego_x, ego_y, left_points)
        result.dist_left = abs(dist_left_signed)

        _, _, _, dist_right_signed, _ = \
            LaneProjector.projectToPolyline(ego_x, ego_y, right_points)
        result.dist_right = abs(dist_right_signed)

        diff_deg = ego_heading_deg - road_heading_deg
        diff_rad = math.radians(diff_deg)
        diff_rad = LaneProjector.normalizeAngle(diff_rad)
        result.angle_diff_rad = diff_rad

        result.angle_diff = max(0.0, min(1.0, (diff_rad / math.pi + 1.0) / 2.0))

        return result

    # ============================================================
    #  内部工具
    # ============================================================

    @staticmethod
    def extractPoints(lane_obj, p2m) -> Tuple[
        List[Tuple[float, float]],
        List[Tuple[float, float]],
        List[Tuple[float, float]],
    ]:
        """
        从 Tessng 的 lane 或 laneConnector 对象提取三条点序列

        两者都有 centerBreakPoints / leftBreakPoints / rightBreakPoints 方法，
        返回的 QPointF 列表坐标单位为像素，需要 p2m 转换为米。
        """
        center = [(p2m(p.x()), p2m(p.y())) for p in lane_obj.centerBreakPoints()]
        left = [(p2m(p.x()), p2m(p.y())) for p in lane_obj.leftBreakPoints()]
        right = [(p2m(p.x()), p2m(p.y())) for p in lane_obj.rightBreakPoints()]
        return center, left, right

    @staticmethod
    def projectToPolyline(
        px: float, py: float,
        polyline: List[Tuple[float, float]],
    ) -> Tuple[float, float, int, float, float]:
        """
        点到折线的最近投影

        Returns:
            (proj_x, proj_y, segment_index, signed_dist, heading)
            signed_dist: 正 = 点在折线行进方向的右侧
        """
        best_dist_sq = float("inf")
        best_proj_x = 0.0
        best_proj_y = 0.0
        best_seg = 0
        best_signed = 0.0
        best_heading = 0.0

        for i in range(len(polyline) - 1):
            ax, ay = polyline[i]
            bx, by = polyline[i + 1]

            abx = bx - ax
            aby = by - ay
            ab_sq = abx * abx + aby * aby

            if ab_sq < 1e-12:
                continue

            t = ((px - ax) * abx + (py - ay) * aby) / ab_sq
            t = max(0.0, min(1.0, t))

            qx = ax + t * abx
            qy = ay + t * aby

            dx = px - qx
            dy = py - qy
            dist_sq = dx * dx + dy * dy

            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_proj_x = qx
                best_proj_y = qy
                best_seg = i
                best_heading = math.atan2(aby, abx)

                cross = abx * (py - ay) - aby * (px - ax)
                dist = math.sqrt(dist_sq)
                if abs(cross) > 1e-12:
                    best_signed = -math.copysign(dist, cross)
                else:
                    best_signed = 0.0

        return best_proj_x, best_proj_y, best_seg, best_signed, best_heading

    @staticmethod
    def normalizeAngle(angle: float) -> float:
        """归一化角度到 [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle