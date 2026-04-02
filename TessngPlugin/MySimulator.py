import math
import os
import threading
import numpy as np

from PySide2.QtCore import *

from Tessng import *

from AutoPilot.TessngDrivingEnv import TessngDrivingEnv, ACTIONS
from AutoPilot.Player.VehicleState import VehicleState
from ExternVehicleLogicTessAuto import TessAutoPyInterface
from Utils.LaneProjector import LaneProjector
from Utils.NavigationCalculator import NavigationCalculator
from Utils.SurroundingCalculator import SurroundingCalculator
from Utils.YawRateCalculator import YawRateCalculator

# 归一化常量
MAX_LANE_WIDTH = 4.0
MAX_SPEED = 33.3
MAX_STEER_ANGLE = 0.7
MAX_ACCEL = 5.0
MAX_DECEL = -5.0

# 训练配置
TRAIN_MODE = False           # True=训练新模型  False=加载已有模型直接推理
TOTAL_TIMESTEPS = 50000
MODEL_SAVE_DIR = "tessng_dqn"


class MySimulator(QObject, PyCustomerSimulator):
    def __init__(self):
        super().__init__()
        PyCustomerSimulator.__init__(self)

        self.iface = None
        self.simIface = None
        self.netIface = None
        self.scene = None

        iface = tessngIFace()
        self.tessAuto = TessAutoPyInterface(iface)

        self.centerLine = [(-637.8400268554688, 160.61000061035156), (-642.7520141601562, 154.64199829101562), (-645.147216796875, 151.72630310058594), (-649.8162231445312, 146.03147888183594), (-651.8980102539062, 143.49000549316406), (-655.1699829101562, 139.49000549316406), (-657.84, 124.38), (-658.306, 114.059), (-626.554, 84.003), (-622.084, 56.875)]

        # 将离散点平滑为密集等距路径（每1米一个点）
        self.smoothedLine = self.smoothCenterLine(self.centerLine, interval=1.0)

        # ===== Gym 环境 =====
        self.env = TessngDrivingEnv(obs_dim=94)
        self.env.resetCallback = self.doReset
        self.env.buildObsCallback = self.buildInitialObs

        # ===== 工具 =====
        self.yawRateCalc = YawRateCalculator()
        self.dt = 0.1
        self.prevSteer = 0.0

        # ===== 沿 centerline 行驶的状态 =====
        self.egoSpeed = 10.0
        self.egoProgress = 0.0
        self.egoSegIndex = 0
        self.prevHeading = 0.0          # 上一帧航向角（度）
        self.isFirstStep = True
        self.currentControl = (0.0, 0.0)
        self.trainThread = None
        self.stepCount = 0

    # ============================================================
    #  Tessng 生命周期回调
    # ============================================================

    def beforeStart(self, keepOn: bool) -> None:
        self.iface = tessngIFace()
        self.simIface = self.iface.simuInterface()
        self.netIface = self.iface.netInterface()
        self.scene = self.netIface.graphicsScene()

        self.isFirstStep = True
        self.stepCount = 0

        # 启动训练子线程（只启动一次）
        if self.trainThread is None or not self.trainThread.is_alive():
            self.trainThread = threading.Thread(
                target=self.runTraining, daemon=True
            )
            self.trainThread.start()

    def ref_beforeNextPoint(self, pIVehicle, ref_keepOn):
        if pIVehicle:
            self.tessAuto.vehicleUpdate(pIVehicle)

    def afterOneStep(self):
        simuInterface = self.iface.simuInterface() if self.iface else tessngIFace().simuInterface()

        # 获取当前帧车辆列表
        vehicles = simuInterface.allVehiStarted()
        activeIds = {v.id() for v in vehicles}

        # 清理已消失的 av 车辆
        staleIds = self.tessAuto.alreadyLaunchedTessngIdSet - activeIds
        for staleId in staleIds:
            self.tessAuto.alreadyLaunchedTessngIdSet.discard(staleId)
            avName = self.tessAuto.tessngId2AvNameMap.pop(staleId, None)
            if avName:
                self.tessAuto.alreadyLaunchedAvNameSet.discard(avName)
                self.tessAuto.avName2TessngIdMap.pop(avName, None)
                self.tessAuto.mainVehiclePtrDict.pop(avName, None)
                self.tessAuto.avChannel2AvMsgMap.pop(avName, None)

        # 第一帧：等训练线程 reset + 拿第一个 action，创建自车
        if self.isFirstStep:
            control = self.env.onFirstStep()
            self.isFirstStep = False
            if control is not None:
                self.currentControl = control
                self.createEgo()
            return

        # 找自车
        egoVehicle = self.findEgo(vehicles)
        if egoVehicle is None:
            return

        self.stepCount += 1

        # 第一帧跳过：车辆刚创建，Tessng 还没更新到正确位置
        if self.stepCount == 1:
            obs = np.zeros(94, dtype=np.float32)
            obs[4] = np.clip(self.egoSpeed / MAX_SPEED, 0, 1)
            control = self.env.onSimuStep(obs, 0.0, False)
            if control is not None:
                self.currentControl = control
                self.applyAction(control, egoVehicle)
            return

        # 前几步打印详细信息
        if self.stepCount <= 4:
            laneDebug = LaneProjector.fromTessngVehicle(egoVehicle, p2m)
            pos = egoVehicle.pos()
            if laneDebug:
                print(f"[调试 Step{self.stepCount}] "
                      f"pos=({pos.x():.1f}, {pos.y():.1f}) "
                      f"offset={laneDebug.lateral_offset:+.2f}m "
                      f"dist_L={laneDebug.dist_left:.2f}m "
                      f"dist_R={laneDebug.dist_right:.2f}m "
                      f"speed={self.egoSpeed:.1f}m/s")

        # 构建 94 维观测
        obs = self.buildObs(egoVehicle, vehicles)

        # 计算奖励
        reward = self.computeReward(egoVehicle, vehicles)

        # 检查终止
        done = self.checkDone(egoVehicle, vehicles)

        # 累计奖励
        self.episodeReward = getattr(self, 'episodeReward', 0.0) + reward

        # 交给训练线程，拿回控制量（阻塞等待）
        control = self.env.onSimuStep(obs, reward, done)

        if control is None:
            # episode 结束，打印汇总
            self.episodeCount = getattr(self, 'episodeCount', 0) + 1
            print(f"\n[Episode {self.episodeCount}] "
                  f"步数={self.stepCount} "
                  f"距离={self.egoProgress:.1f}m "
                  f"累计奖励={self.episodeReward:.2f} "
                  f"最终速度={self.egoSpeed:.1f}m/s\n")
            self.episodeReward = 0.0
            self.isFirstStep = True
            return

        self.currentControl = control
        self.applyAction(control, egoVehicle)

        if self.stepCount % 100 == 0:
            print(f"[Step {self.stepCount}] "
                  f"动作: {self.env.currentActionName} "
                  f"reward={reward:.3f} "
                  f"speed={self.egoSpeed:.1f}m/s "
                  f"progress={self.egoProgress:.1f}m")

    # ============================================================
    #  DQN 训练（子线程）
    # ============================================================

    def runTraining(self):
        from stable_baselines3 import DQN

        os.makedirs(MODEL_SAVE_DIR, exist_ok=True)
        savePath = os.path.join(MODEL_SAVE_DIR, "model")

        if TRAIN_MODE:
            # ===== 训练模式 =====
            print("=" * 50)
            print("[训练线程] 开始 DQN 训练")
            print(f"  观测空间: {self.env.observation_space}")
            print(f"  动作空间: {self.env.action_space} ({ACTIONS})")
            print(f"  总步数: {TOTAL_TIMESTEPS}")
            print("=" * 50)

            model = DQN(
                "MlpPolicy",
                self.env,
                policy_kwargs=dict(net_arch=[256, 256]),
                learning_rate=5e-4,
                buffer_size=15000,
                learning_starts=200,
                batch_size=32,
                gamma=0.8,
                train_freq=1,
                gradient_steps=1,
                target_update_interval=50,
                verbose=1,
            )

            model.learn(total_timesteps=TOTAL_TIMESTEPS)
            model.save(savePath)
            print(f"[训练线程] 训练完成，模型已保存到 {savePath}")
        else:
            # ===== 推理模式：直接加载已有模型 =====
            print("=" * 50)
            print(f"[训练线程] 加载已有模型: {savePath}")
            print("=" * 50)

        # 推理循环（训练完或直接加载后都走这里）
        model = DQN.load(savePath, env=self.env)
        print("[训练线程] 进入推理模式...")

        while not self.env._closed:
            done = False
            obs = self.env.reset()
            while not done and not self.env._closed:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, info = self.env.step(action)

        print("[训练线程] 已退出")

    # ============================================================
    #  环境回调
    # ============================================================

    def doReset(self):
        self.yawRateCalc.reset()
        self.prevSteer = 0.0
        self.prevHeading = 0.0
        self.stepCount = 0
        self.egoSpeed = 10.0
        self.egoProgress = 0.0
        self.isFirstStep = True
        print("[MySimulator] 场景已重置")

    def buildInitialObs(self):
        return np.zeros(94, dtype=np.float32)

    # ============================================================
    #  观测构建（94维）
    # ============================================================

    def buildObs(self, egoVehicle, vehicles) -> np.ndarray:
        obs = np.zeros(94, dtype=np.float32)

        # ===== 自车运动状态 (9维: idx 0~8) =====
        laneResult = LaneProjector.fromTessngVehicle(egoVehicle, p2m)
        egoSpeed = self.egoSpeed

        if laneResult:
            obs[0] = np.clip(laneResult.dist_left / MAX_LANE_WIDTH, 0, 1)
            obs[1] = np.clip(laneResult.dist_right / MAX_LANE_WIDTH, 0, 1)
            obs[2] = np.clip(laneResult.lateral_offset / MAX_LANE_WIDTH + 0.5, 0, 1)
            obs[3] = laneResult.angle_diff

        obs[4] = np.clip(egoSpeed / MAX_SPEED, 0, 1)

        currentSteer = self.currentControl[1]
        obs[5] = np.clip(currentSteer / (2 * MAX_STEER_ANGLE) + 0.5, 0, 1)

        currentAccel = self.currentControl[0]
        obs[6] = np.clip((currentAccel - MAX_DECEL) / (MAX_ACCEL - MAX_DECEL), 0, 1)

        obs[7] = np.clip(self.prevSteer / (2 * MAX_STEER_ANGLE) + 0.5, 0, 1)
        self.prevSteer = currentSteer

        yawRate = self.yawRateCalc.update(egoVehicle.id(), egoVehicle.angle(), self.dt)
        obs[8] = self.yawRateCalc.normalize(yawRate)

        # ===== 导航信息 (13维: idx 9~21) =====
        centerLine = self.getCenterLine(egoVehicle)
        if centerLine and len(centerLine) >= 2:
            sparse = NavigationCalculator.sparsifyByDistance(centerLine, 5.0)
            egoPos = egoVehicle.pos()
            navResult = NavigationCalculator.compute(
                p2m(egoPos.x()), p2m(egoPos.y()),
                egoVehicle.angle(), sparse, numCheckpoints=5,
            )
            idx = 9
            for i in range(min(5, len(navResult.forwardGaps))):
                obs[idx + i * 2] = navResult.forwardGaps[i]
                obs[idx + i * 2 + 1] = navResult.lateralGaps[i]
            obs[19] = navResult.curvatureRadius
            obs[20] = navResult.curvatureDirection
            obs[21] = navResult.laneAngleDiff

        # ===== 雷达 (72维: idx 22~93) =====
        surroundResult = SurroundingCalculator.fromTessngVehicles(
            egoVehicle, vehicles, p2m, maxNearby=8,
        )
        for i, r in enumerate(surroundResult.radar):
            obs[22 + i] = r

        return obs

    # ============================================================
    #  奖励函数
    # ============================================================

    def computeReward(self, egoVehicle, vehicles) -> float:
        """
        奖励设计：让 DQN 学到「保持目标速度沿路行驶」

        正奖励：
          存活 +0.1/步
          速度接近目标 +0.3（15m/s 满分，偏差越大越低）
          前进距离 +0.2
          居中 +0.1

        惩罚：
          偏离车道 -0.5
          停车 -0.3
          前方太近 -1.0
        """
        reward = 0.0

        # 存活
        reward += 0.1

        # 速度：接近 15 m/s 奖励最大
        targetSpeed = 15.0
        speedRatio = 1.0 - min(abs(self.egoSpeed - targetSpeed) / targetSpeed, 1.0)
        reward += speedRatio * 0.3

        # 前进
        progressThisStep = self.egoSpeed * self.dt
        reward += min(progressThisStep / 2.0, 0.2)

        # 居中
        laneResult = LaneProjector.fromTessngVehicle(egoVehicle, p2m)
        if laneResult:
            offsetAbs = abs(laneResult.lateral_offset)
            if offsetAbs < 0.5:
                reward += 0.1
            elif offsetAbs > MAX_LANE_WIDTH / 2:
                reward -= 0.5
            else:
                reward -= (offsetAbs / MAX_LANE_WIDTH) * 0.3

        # 前方安全
        surroundResult = SurroundingCalculator.fromTessngVehicles(
            egoVehicle, vehicles, p2m, maxNearby=4,
        )
        if surroundResult.radar:
            frontRadar = surroundResult.radar[0]
            if frontRadar < 0.2:
                reward -= 1.0
            elif frontRadar < 0.5:
                reward -= (0.5 - frontRadar) * 0.5

        # 停车惩罚
        if self.egoSpeed < 0.5:
            reward -= 0.3

        # 航向平滑奖励：航向变化越小越好
        _, _, currentHeading = self.positionOnCenterLine(self.egoProgress)
        headingDiff = abs(currentHeading - self.prevHeading)
        if headingDiff > 180:
            headingDiff = 360 - headingDiff
        # 每步航向变化超过5度就惩罚，越大惩罚越重
        if headingDiff > 5.0:
            reward -= min(headingDiff / 30.0, 0.5)  # 最大惩罚 -0.5
        elif headingDiff < 2.0:
            reward += 0.05  # 平稳行驶小奖励

        # 弯道减速奖励：弯道处低速有奖励
        if headingDiff > 3.0 and self.egoSpeed < 12.0:
            reward += 0.1  # 弯道减速是聪明的
        elif headingDiff > 3.0 and self.egoSpeed > 20.0:
            reward -= 0.3  # 弯道高速是危险的

        return reward

    # ============================================================
    #  终止判断
    # ============================================================

    def checkDone(self, egoVehicle, vehicles) -> bool:
        # 超时
        if self.stepCount >= 500:
            print(f"[Done] 超时 500 步, 距离={self.egoProgress:.1f}m")
            return True

        # 到达路径末尾（成功！）
        if self.smoothedLine:
            totalLen = sum(
                math.sqrt((self.smoothedLine[i+1][0] - self.smoothedLine[i][0])**2 +
                           (self.smoothedLine[i+1][1] - self.smoothedLine[i][1])**2)
                for i in range(len(self.smoothedLine) - 1)
            )
            if self.egoProgress >= totalLen:
                print(f"[Done] 到达终点! progress={self.egoProgress:.1f}m")
                return True

        # 速度为0超过10步（卡住了）
        if self.egoSpeed < 0.1:
            self._zeroSpeedCount = getattr(self, '_zeroSpeedCount', 0) + 1
            if self._zeroSpeedCount > 10:
                print(f"[Done] 停车超过10步")
                self._zeroSpeedCount = 0
                return True
        else:
            self._zeroSpeedCount = 0

        return False

    # ============================================================
    #  动作执行
    # ============================================================

    def createEgo(self):
        """第一帧创建自车，在 centerLine 起点"""
        cl = self.centerLine
        if cl and len(cl) >= 2:
            x, y = cl[0]
            mathRad = math.atan2(-(cl[1][1] - cl[0][1]), cl[1][0] - cl[0][0])
            heading = (90.0 - math.degrees(mathRad)) % 360.0
        else:
            x, y, heading = -660.0, 22.7, 140.0

        self.egoSpeed = 10.0
        self.egoProgress = 0.0

        state = VehicleState(
            x=x, y=y, heading=heading, speed=self.egoSpeed,
        )
        self.tessAuto.setAvChannel2AvMsgMap({"ego": state})
        self.tessAuto.vehicleCreate()

    def applyAction(self, control, egoVehicle):
        """沿平滑路径行驶，动作只控制速度"""
        accel, steer = control

        if not self.smoothedLine or len(self.smoothedLine) < 2:
            return

        self.egoSpeed += accel * self.dt
        self.egoSpeed = max(0.0, min(self.egoSpeed, MAX_SPEED))

        moveDist = self.egoSpeed * self.dt
        self.egoProgress += moveDist

        newX, newY, heading = self.positionOnCenterLine(self.egoProgress)

        # 记录航向变化（用于奖励函数）
        self.prevHeading = heading

        state = VehicleState(
            x=newX, y=-newY,
            heading=heading,
            speed=self.egoSpeed,
        )
        self.tessAuto.setAvChannel2AvMsgMap({"ego": state})

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

    # ============================================================
    #  工具方法
    # ============================================================

    def findEgo(self, vehicles):
        """通过 tessAuto 的已创建车辆 ID 匹配自车"""
        for v in vehicles:
            if v.id() in self.tessAuto.alreadyLaunchedTessngIdSet:
                return v
        return None

    def getCenterLine(self, egoVehicle):
        """获取中心线（米），优先用预设路径，否则动态获取"""
        if self.centerLine:
            return self.centerLine

        laneObj = egoVehicle.lane() or egoVehicle.laneConnector()
        if laneObj is None:
            return []

        points = [(p2m(p.x()), p2m(p.y())) for p in laneObj.centerBreakPoints()]
        return points

    @staticmethod
    def smoothCenterLine(points, interval=1.0):
        """
        将离散路径点平滑为密集等距点序列

        使用 Catmull-Rom 样条插值，保证路径经过所有原始点，
        同时在点之间生成平滑的过渡曲线。

        Args:
            points: 原始离散点 [(x,y), ...]
            interval: 输出点间距（米）

        Returns:
            平滑后的密集点序列
        """
        if len(points) < 2:
            return list(points)

        # Catmull-Rom 插值
        def catmullRom(p0, p1, p2, p3, t):
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * ((2*p1[0]) +
                        (-p0[0] + p2[0]) * t +
                        (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                        (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
            y = 0.5 * ((2*p1[1]) +
                        (-p0[1] + p2[1]) * t +
                        (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                        (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
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
            segLen = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            numSamples = max(int(segLen / interval * 3), 10)

            for j in range(numSamples):
                t = j / numSamples
                dense.append(catmullRom(p0, p1, p2, p3, t))

        dense.append(points[-1])

        # 再等距重采样
        return NavigationCalculator.sparsifyByDistance(dense, interval)