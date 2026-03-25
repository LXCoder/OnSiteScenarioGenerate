import math

from PySide2.QtCore import *

from Tessng import *

from AutoPilot.PlayerManager import PlayerManager
from ExternVehicleLogicTessAuto import TessAutoPyInterface
from Utils.LaneProjector import LaneProjector
from Utils.NavigationCalculator import NavigationCalculator
from Utils.SurroundingCalculator import SurroundingCalculator


class MySimulator(QObject, PyCustomerSimulator):
    def __init__(self):
        super().__init__()
        PyCustomerSimulator.__init__(self)

        self.iface = None
        self.simIface = None
        self.netIface = None
        self.scene = None

        self.manager = PlayerManager()

        iface = tessngIFace()
        self.tessAuto = TessAutoPyInterface(iface)

        self.centerLine = [(-637.8400268554688, 160.61000061035156), (-642.7520141601562, 154.64199829101562), (-645.147216796875, 151.72630310058594), (-649.8162231445312, 146.03147888183594), (-651.8980102539062, 143.49000549316406), (-655.1699829101562, 139.49000549316406)]


    def beforeStart(self, keepOn: bool) -> None:
        self.iface = tessngIFace()
        self.simIface = self.iface.simuInterface()
        self.netIface = self.iface.netInterface()
        self.scene = self.netIface.graphicsScene()

        self.manager.load_all()

    def ref_beforeNextPoint(self, pIVehicle, ref_keepOn):
        if pIVehicle:
            self.tessAuto.vehicleUpdate(pIVehicle)

    def afterOneStep(self):
        iface = tessngIFace()
        simuInterface = iface.simuInterface()

        # 1. 先获取当前帧车辆列表
        vehicles = simuInterface.allVehiStarted()
        activeIds = {v.id() for v in vehicles}

        # 2. 清理已消失的 av 车辆（在创建新车之前）
        staleIds = self.tessAuto.alreadyLaunchedTessngIdSet - activeIds
        for staleId in staleIds:
            self.tessAuto.alreadyLaunchedTessngIdSet.discard(staleId)
            avName = self.tessAuto.tessngId2AvNameMap.pop(staleId, None)
            if avName:
                self.tessAuto.alreadyLaunchedAvNameSet.discard(avName)
                self.tessAuto.avName2TessngIdMap.pop(avName, None)
                self.tessAuto.mainVehiclePtrDict.pop(avName, None)
                self.tessAuto.avChannel2AvMsgMap.pop(avName, None)

        # 3. 再创建/更新车辆
        states = self.manager.step_all()
        self.tessAuto.setAvChannel2AvMsgMap(states)
        self.tessAuto.vehicleCreate()

        # 4. 后续的 ego 查找、导航、周车计算...
        egoVehicle = None
        for vehicle in vehicles:
            if vehicle.id() in self.tessAuto.alreadyLaunchedTessngIdSet:
                egoVehicle = vehicle
                break

        if egoVehicle is None:
            return

        # 自车运动状态
        result = LaneProjector.fromTessngVehicle(egoVehicle, p2m)
        if result:
            print(f"自车 {egoVehicle.id()}: "
                  f"左边界={result.dist_left:.2f}m "
                  f"右边界={result.dist_right:.2f}m "
                  f"偏移={result.lateral_offset:+.3f}m "
                  f"角度差={result.angle_diff:.4f}")

        # 导航信息
        sparse = NavigationCalculator.sparsifyByDistance(self.centerLine, 5.0)
        egoPos = egoVehicle.pos()
        egoX = p2m(egoPos.x())
        egoY = p2m(egoPos.y())
        navResult = NavigationCalculator.compute(
            egoX, egoY, egoVehicle.angle(), sparse, numCheckpoints=5
        )
        print(f"导航: forward={[f'{v:.2f}' for v in navResult.forwardGaps]} "
              f"弯曲半径={navResult.curvatureRadius:.2f} "
              f"弯曲方向={navResult.curvatureDirection:.2f}")

        # 周车交互信息
        surroundResult = SurroundingCalculator.fromTessngVehicles(
            egoVehicle, vehicles, p2m,
            maxNearby=8,
            filterRadius=50.0,
        )
        for i, v in enumerate(surroundResult.nearbyVehicles):
            print(f"  周车{i}: 前向={v.forwardGap:.2f} 侧向={v.lateralGap:.2f} "
                  f"纵速差={v.longitudinalSpeedDiff:.2f} 横速差={v.lateralSpeedDiff:.2f}")
        print(f"  雷达命中数: {sum(1 for r in surroundResult.radar if r < 1.0)}/72")
