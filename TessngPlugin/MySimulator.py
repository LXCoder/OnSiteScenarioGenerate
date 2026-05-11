

import math
import os
import threading
import numpy as np

from PySide2.QtCore import QObject, QPointF, Signal,QCoreApplication

from Tessng import (
    Online,
    PyCustomerSimulator,
    tessngIFace,
    p2m,
)

from AutoPilot.TessngDrivingEnv import TessngDrivingEnv, ACTIONS, ACTION_TO_CONTROL
from AutoPilot.TessngDrivingEnvPPO import TessngDrivingEnvPPO
from AutoPilot.Player.VehicleState import VehicleState
from AutoPilot.PlayerManager import PlayerManager
from ExternVehicleLogicTessAuto import TessAutoPyInterface
from Utils.LaneProjector import LaneProjector
from Utils.NavigationCalculator import NavigationCalculator
from Utils.SurroundingCalculator import SurroundingCalculator
from Utils.YawRateCalculator import YawRateCalculator
from Utils.ScenarioLoader import g_scenario_loader
from MultiVehicleInference import MultiVehicleInference
from Utils.Constant import (
    WHEEL_BASE,
    MAX_LANE_WIDTH,
    MAX_SPEED,
    MAX_STEER_ANGLE,
    MAX_ACCEL,
    MAX_DECEL,
    TRAIN_MODE,
    TOTAL_TIMESTEPS,
    RL_ALGO,
    MODEL_SAVE_DIR,
    TENSORBOARD_LOG,
    TRAIN_MAX_STEPS,
    MAX_ACC_DELTA,
    MAX_STEER_DELTA,
    REPEAT_SINGLE_SCENARIO,
    EXIT_ON_SIMULATION_STOP
)


class MySimulator(QObject, PyCustomerSimulator):
    sig_stop_simu = Signal()

    def __init__(self):
        super().__init__()
        PyCustomerSimulator.__init__(self)

        self.iface = None
        self.simIface = None
        self.netIface = None
        self.scene = None

        iface = tessngIFace()
        self.tessAuto = TessAutoPyInterface(iface)

        self.sig_stop_simu.connect(iface.simuInterface().stopSimu)

        # ===== 主车：由选手的 TestPlayer 控制 =====
        self.playerManager = PlayerManager()
        self.egoName = "ego"
        self.egoSmoothedPath = []  # 初始化为空，切换场景时动态加载

        # 获取场景数据
        self.scenarios = g_scenario_loader.getScenarios().copy()
        self.scenario_indices = []  # 用来存放洗牌后的索引队列

        # 当前场景
        self.currentScenarioIdx = 0
        self.currentVehicles = {}
        self.bgAgents = {}  # 背景车运行状态
        self.tessngBgRoutingByName = {}
        self.tessngControlledBgNames = {
            name.strip()
            for name in os.getenv("TESSNG_BG_NAMES", "").split(",")
            if name.strip()
        }

        # 主车最新状态（每帧从 TestPlayer.act() 读取）
        self.egoState = None

        # 训练用 Gym 环境
        if RL_ALGO == "PPO":
            self.env = TessngDrivingEnvPPO(obs_dim=94)
        else:
            self.env = TessngDrivingEnv(obs_dim=94)
        self.env.resetCallback = self.doReset
        self.env.buildObsCallback = self.buildInitialObs

        # 推理用管理器
        self.multiInfer = None

        # 工具
        self.yawRateCalc = YawRateCalculator()
        self.dt = 0.1
        self.prevSteer = 0.0

        # 状态
        self.isFirstStep = True
        self.currentControl = (0.0, 0.0)
        self.currentEgoControl = (0.0, 0.0)

        self.trainThread = None
        self.stepCount = 0
        self.episodeReward = 0.0
        self.episodeCount = 0

        # 多模型配置
        self.egoModel = None
        self.bgModel = None

    # ============================================================
    #  Tessng 生命周期
    # ============================================================

    def beforeStart(self, keepOn: bool) -> None:
        self.iface = tessngIFace()
        self.simIface = self.iface.simuInterface()
        self.netIface = self.iface.netInterface()
        self.scene = self.netIface.graphicsScene()

        # 加载选手
        self.playerManager.load_all()

        self.isFirstStep = True
        self.stepCount = 0

        if self.scenarios:
            self.loadScenario(0)

        if self.trainThread is None or not self.trainThread.is_alive():
            self.trainThread = threading.Thread(target=self.runTraining, daemon=True)
            self.trainThread.start()

    def ref_beforeNextPoint(self, pIVehicle, ref_keepOn):
        if pIVehicle:
            self.tessAuto.vehicleUpdate(pIVehicle)

    def afterOneStep(self):
        simuIface = self.simIface or tessngIFace().simuInterface()
        vehicles = simuIface.allVehiStarted()
        activeIds = {v.id() for v in vehicles}

        # 清理消失的车
        staleIds = self.tessAuto.alreadyLaunchedTessngIdSet - activeIds
        for staleId in staleIds:
            self.tessAuto.alreadyLaunchedTessngIdSet.discard(staleId)
            avName = self.tessAuto.tessngId2AvNameMap.pop(staleId, None)
            if avName:
                self.tessAuto.alreadyLaunchedAvNameSet.discard(avName)
                self.tessAuto.avName2TessngIdMap.pop(avName, None)
                self.tessAuto.mainVehiclePtrDict.pop(avName, None)
                self.tessAuto.avChannel2AvMsgMap.pop(avName, None)

        # ===== 每帧更新主车（选手控制） =====
        self.updateEgo()

        # 推理模式（multiInfer 已初始化）
        if self.multiInfer is not None:
            self._afterOneStepInference(vehicles)
            return

        # 训练模式（env 未关闭时才走训练逻辑）
        if not self.env._closed:
            self._afterOneStepTraining(vehicles)

    def afterStop(self):
        print("[MySimulator] 仿真结束")
        if EXIT_ON_SIMULATION_STOP:
            QCoreApplication.quit()

    def afterPause(self):
        print("[MySimulator] 仿真暂停")

    # ============================================================
    #  主车更新
    # ============================================================

    def updateEgo(self):
        """更新主车状态"""
        if TRAIN_MODE and not self.env._closed:
            # ===== 训练模式：由 RL 动作控制 Ego =====
            self._updateEgoByRL()
            self.tessAuto.vehicleCreate()
        elif not TRAIN_MODE and getattr(self, "egoModel", None) is not None:
            # ===== 推理模式：由 _afterOneStepInference 统一处理 =====
            pass
        else:
            # ===== 普通模式：由选手代码控制 =====
            states = self.playerManager.step_all()
            self.tessAuto.setAvChannel2AvMsgMap(states)
            for name, state in states.items():
                self.egoState = state
                self.egoName = name
                break
            self.tessAuto.vehicleCreate()

    def _updateEgoByRL(self):
        """根据 RL 输出的 currentControl (accel, steer) 更新 egoState"""
        if self.egoState is None:
            # 初始状态
            init_x, init_y = (
                self.egoSmoothedPath[0]
                if self.egoSmoothedPath
                else (-650.047, -287.819)
            )
            _, _, init_heading_math = self._posOnPath(self.egoSmoothedPath, 0.0)

            # _posOnPath 返回的是基于数学坐标 (Y向北) 的 atan2(dx, dy) 角度
            # 我们需要将其转换为 TESSNG 坐标系 (Y向南，北0顺时针) 下的航向角
            # 转换公式：从逆时针转顺时针，需从360减去，加上对齐偏差
            # TNG_Heading = atan2(dx, -dy)
            # 这里我们通过重新计算来确保准确性，因为 _posOnPath 内部丢失了 dy 符号
            if len(self.egoSmoothedPath) > 1:
                dx = self.egoSmoothedPath[1][0] - self.egoSmoothedPath[0][0]
                dy = self.egoSmoothedPath[1][1] - self.egoSmoothedPath[0][1]
                init_heading = math.degrees(math.atan2(dx, dy)) % 360.0
            else:
                init_heading = 0.0

            print(
                f"初始位置: ({init_x:.2f}, {init_y:.2f}), 初始航向: {init_heading:.2f}°"
            )

            # init_y 已经是数学坐标，直接赋值
            initial_speed = getattr(self, "_ego_target_speed", 15.0)
            self.egoState = VehicleState(
                x=init_x, y=init_y, heading=init_heading, speed=initial_speed
            )

        accel, steer = self.currentEgoControl

        # 运动学模型更新
        dt = self.dt
        self.egoState.speed += accel * dt
        self.egoState.speed = max(0.0, min(self.egoState.speed, MAX_SPEED))

        yaw_rate = (self.egoState.speed * math.tan(steer)) / WHEEL_BASE
        self.egoState.heading = (
            self.egoState.heading + math.degrees(yaw_rate * dt)
        ) % 360.0

        heading_rad = math.radians(self.egoState.heading)
        self.egoState.x += self.egoState.speed * math.sin(heading_rad) * dt
        # 关键修正：在数学坐标系中，向北(heading=0)为 Y 增加
        self.egoState.y += self.egoState.speed * math.cos(heading_rad) * dt

        vsMap = {self.egoName: self.egoState}
        self.tessAuto.setAvChannel2AvMsgMap(vsMap)

    # ============================================================
    #  训练模式 (Ego 训练版)
    # ============================================================

    def _afterOneStepTraining(self, vehicles):
        if self.isFirstStep:
            control = self.env.onFirstStep()
            self.isFirstStep = False
            if control is not None:
                self.currentEgoControl = control
                self.createBgVehicles()  # 背景车作为障碍物
            return

        egoVehicle, bgVehicles = self.findBgVehicle(vehicles)

        if egoVehicle is None:
            return

        self.stepCount += 1

        # 集中计算一次 Frenet 进度，供 Reward 和 Done 共用
        (
            self._current_s_ego,
            self._current_s_total,
            self._current_lateral_dist,
            expected_heading,
        ) = self._getFrenetProgress(
            self.egoSmoothedPath, p2m(egoVehicle.pos().x()), -p2m(egoVehicle.pos().y())
        )

        # 计算并保存航向角最小绝对偏差 [0, 180]
        ego_heading = self.egoState.heading if self.egoState else egoVehicle.angle()
        raw_diff = abs(ego_heading - expected_heading) % 360.0
        diff = 360.0 - raw_diff if raw_diff > 180.0 else raw_diff

        # print(f"ego heading: {ego_heading:.2f}, expected heading: {expected_heading:.2f}, diff: {diff:.2f}")
        self._current_angle_diff = 0.0 if self.isFirstStep else diff

        # 构建以 Ego 为中心的观测，并显式同步 Ego 观测派生状态
        obs, obs_info = self.buildObs(egoVehicle, vehicles)
        self.applyEgoObsInfo(obs_info)
        # 计算 Ego 的奖励
        reward = self.computeEgoReward(egoVehicle)
        done = self.checkEgoDone(egoVehicle)

        self.episodeReward += reward
        control = self.env.onSimuStep(obs, reward, done)

        if control is None:
            self.episodeCount += 1
            print(
                f"[Episode {self.episodeCount}] Ego训练: 步数={self.stepCount}, 奖励={self.episodeReward:.2f}"
            )
            self.episodeReward = 0.0
            self.isFirstStep = True
            return

        self.currentEgoControl = control
        # 背景车在此模式下可以作为 NPC 运行
        self.applyBgActions((0.0, 0.0))  # 背景车恒速或静止

    def computeEgoReward(self, egoVehicle) -> float:
        """
        Ego 训练奖励函数 (路径优先版)
        统一使用 Frenet 轨迹作为基准，并引入航向对齐奖励与动态限速
        优先级：到达终点(10.0奖励) > 轨迹居中(0.35) > 航向对齐(0.30) = 推进量(0.35) > 速度匹配(0.05)
        """
        # 1. 碰撞检测 (若碰撞，本步奖励为 0.0，并标记结束)
        ego_id = egoVehicle.id()
        all_vehis = self.simIface.allVehiStarted()
        for v in all_vehis:
            if v.id() != ego_id:
                if self._check_bbox_collision_vehi(egoVehicle, v):
                    self._is_collision = True
                    return 0.0

        # 2. 动态速度奖励 (权重 0.05 - 极低优先级)
        # 获取场景配置的基础目标速度
        base_target_v = getattr(self, "_ego_target_speed", 15.0)

        # [新增] 动态限速逻辑：根据弯道曲率半径下调期望速度
        # self._current_curvature_radius 是 [0, 1] 归一化的值，对应 [0, 1000m]
        norm_radius = getattr(self, "_current_curvature_radius", 1.0)
        real_radius = norm_radius * 1000.0

        # 物理公式：v_max = sqrt(a_lat * R)。假设舒适侧向加速度为 2.5 m/s^2
        curve_limit_v = math.sqrt(2.5 * max(5.0, real_radius))

        # 最终期望速度是场景配置与弯道物理限速的最小值
        dynamic_target_v = min(base_target_v, curve_limit_v)

        v = egoVehicle.currSpeed()
        speed_diff = abs(v - dynamic_target_v)

        # 速度奖励降低权重至 0.05
        # r_speed = 0.05 * max(0.0, (1.0 - speed_diff / 10.0))

        # 3. 轨迹居中奖励 (权重 0.35 - 最高物理权重)
        r_center = 0.0
        # 从缓存中获取由 _getFrenetProgress 计算的车辆到参考路径的横向偏差
        lateral_offset = abs(getattr(self, "_current_lateral_dist", 0.0))
        max_tolerate_offset = MAX_LANE_WIDTH / 2.0  # 约 2.0米

        if lateral_offset > max_tolerate_offset + 0.5:
            # 严重偏离规划轨迹，标记失败，居中奖励为 0
            self._is_out_of_bounds = True
        else:
            # 必须车辆有速度才给居中奖励，防止原地趴窝白嫖
            if v > 1.0:
                # 离规划轨迹越近，奖励越高
                r_center = 0.35 * max(
                    0.0, (1.0 - lateral_offset / max_tolerate_offset) ** 2
                )

        # 4. 航向对齐奖励 (权重 0.30 - 高优先级)
        angle_diff = getattr(self, "_current_angle_diff", 0.0)
        r_heading = 0.0
        if angle_diff > 45.0:
            self._is_out_of_bounds = True  # 航向偏差过大也算出界
        else:
            if v > 1.0:
                # 航向角奖励增加权重至 0.30
                r_heading = 0.30 * max(0.0, (1.0 - angle_diff / 45.0))

        # 5. 单步实际推进量奖励 (Delta Progress, 权重 0.20)
        # 直接使用已计算的缓存值
        s_ego = getattr(self, "_current_s_ego", 0.0)

        # 获取上一帧的 s_ego (若无则默认为当前值)
        prev_s_ego = getattr(self, "_prev_s_ego", s_ego)

        # 计算本帧在路径上实际前进了多少米
        delta_s = s_ego - prev_s_ego

        # 更新缓存以备下一帧使用
        self._prev_s_ego = s_ego

        r_progress = 0.0
        if delta_s > 0.0:
            # 每帧理论最大 delta_s 为 2.0 米，权重 0.30
            r_progress = 0.35 * np.clip(delta_s / 2.0, 0.0, 1.0)

        # 6. 计算单步总奖励 [0.0, 1.0]
        # total_reward = r_speed + r_center + r_heading + r_progress
        total_reward = r_center + r_heading + r_progress

        # 7. 终点大奖 (全局最高优先级)
        if getattr(self, "_is_reached_goal", False):
            total_reward += 10.0  # 给予强力正向引导
            print("  --> 获得终点大奖! (+10.0)")

        return float(total_reward)

    def checkEgoDone(self, egoVehicle) -> bool:
        if getattr(self, "_is_collision", False):
            print("[Done] Ego 发生碰撞!")
            return True
        if getattr(self, "_is_out_of_bounds", False):
            print("[Done] Ego 偏离车道!")
            return True

        # [新增] 低速卡死判定：防止在弯道极速龟爬苟活
        v = egoVehicle.currSpeed()
        if v < 1.0:
            self._ego_stuck_count = getattr(self, "_ego_stuck_count", 0) + 1
        else:
            self._ego_stuck_count = 0

        if getattr(self, "_ego_stuck_count", 0) > 50:
            print("[Done] Ego 陷入低速卡死状态!")
            return True

        # [新增] Frenet 纵向距离判定
        # 直接使用已计算的缓存值
        s_ego = getattr(self, "_current_s_ego", 0.0)
        s_total = getattr(self, "_current_s_total", 0.0)
        dist_to_end = s_total - s_ego

        if s_total > 0.0 and dist_to_end < 5.0:
            print(f"[Done] Ego 成功到达终点! (剩余距离: {dist_to_end:.2f}m)")
            self._is_reached_goal = True
            return True

        # 取消最大步数限制，由目标到达、碰撞、出界或卡死来决定结束
        # if self.stepCount >= TRAIN_MAX_STEPS:
        #     print("[Done] 达到最大步数")
        #     return True
        return False

    def _check_bbox_collision_vehi(self, v1, v2):
        p1 = v1.pos()
        p2 = v2.pos()
        return self._check_bbox_collision(
            p2m(p1.x()),
            p2m(p1.y()),
            v1.angle(),
            v1.length(),
            v1.width(),
            p2m(p2.x()),
            p2m(p2.y()),
            v2.angle(),
            v2.length(),
            v2.width(),
        )

    # ============================================================
    #  推理模式
    # ============================================================

    def _afterOneStepInference(self, vehicles):
        if not getattr(self, "_inferCreated", False):
            # 初始化 Ego 状态
            init_x, init_y = (
                self.egoSmoothedPath[0] if self.egoSmoothedPath else (0.0, 0.0)
            )
            _, _, init_heading = self._posOnPath(self.egoSmoothedPath, 0.0)
            initial_speed = getattr(self, "_ego_target_speed", 15.0)
            self.egoState = VehicleState(
                x=init_x, y=init_y, heading=init_heading, speed=initial_speed
            )
            self._ego_prev_control = (0.0, 0.0)

            vsMap = {self.egoName: self.egoState}
            # 初始化背景车状态
            for name, agent in self.bgAgents.items():
                if self._isTessngControlledBg(name):
                    self.createTessngBgVehicle(name, agent)
                    continue

                x, y, heading = self._posOnPath(agent["smoothed"], 0.0)
                agent["state"] = VehicleState(
                    x=x, y=y, heading=heading, speed=agent["speed"]
                )
                agent["prev_control"] = (0.0, 0.0)
                vsMap[name] = agent["state"]

            self.tessAuto.setAvChannel2AvMsgMap(vsMap)
            self.tessAuto.vehicleCreate()
            self._inferCreated = True
            self._inferStep = 0
            self._is_collision = False
            self.yawRateCalc.reset()
            return

        egoVehicle, bgVehicles = self.findBgVehicle(vehicles)
        if egoVehicle is None:
            return

        egoVehicle.setColor("#02f13e")
        # 集中计算一次 Ego 的 Frenet 进度（用于终止判断）
        self._current_s_ego, self._current_s_total, self._current_lateral_dist, _ = (
            self._getFrenetProgress(
                self.egoSmoothedPath,
                p2m(egoVehicle.pos().x()),
                -p2m(egoVehicle.pos().y()),
            )
        )

        vsMap = {}

        is_debug_info = self._inferStep % 100 == 0

        # 1. 控制 Ego (使用模型)
        if hasattr(self, "egoModel") and self.egoModel is not None:
            obs, obs_info = self.buildObs(egoVehicle, vehicles)
            self.applyEgoObsInfo(obs_info)
            action, _ = self.egoModel.predict(obs, deterministic=True)

            # 直接读取物理动作值并截断
            accel = np.clip(action[0], MAX_DECEL, MAX_ACCEL)
            steer = np.clip(action[1], -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

            # 同样要复刻动作平滑限制，防止推理时动作突变导致翻车
            prev_accel, prev_steer = getattr(self, "_ego_prev_control", (0.0, 0.0))
            accel = np.clip(
                accel, prev_accel - MAX_ACC_DELTA, prev_accel + MAX_ACC_DELTA
            )
            steer = np.clip(
                steer, prev_steer - MAX_STEER_DELTA, prev_steer + MAX_STEER_DELTA
            )
            self._ego_prev_control = (float(accel), float(steer))
            self.currentEgoControl = self._ego_prev_control

            self.egoState.speed = max(
                0.0, min(self.egoState.speed + float(accel) * self.dt, MAX_SPEED)
            )
            yaw_rate = (self.egoState.speed * math.tan(float(steer))) / WHEEL_BASE
            self.egoState.heading = (
                self.egoState.heading + math.degrees(yaw_rate * self.dt)
            ) % 360.0
            h_rad = math.radians(self.egoState.heading)
            self.egoState.x += self.egoState.speed * math.sin(h_rad) * self.dt
            self.egoState.y += self.egoState.speed * math.cos(h_rad) * self.dt
            vsMap[self.egoName] = self.egoState

        # 2. 控制所有背景车 (使用背景车专用模型)
        for v in bgVehicles:
            avName = self.tessAuto.tessngId2AvNameMap.get(v.id())
            agent = self.bgAgents.get(avName)
            if not agent or "state" not in agent:
                continue

            if hasattr(self, "bgModel") and self.bgModel is not None:
                obs, _ = self.buildObs(v, vehicles)
                action, _ = self.bgModel.predict(obs, deterministic=True)

                accel = np.clip(action[0], MAX_DECEL, MAX_ACCEL)
                steer = np.clip(action[1], -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

                prev_accel, prev_steer = agent["prev_control"]
                accel = np.clip(
                    accel, prev_accel - MAX_ACC_DELTA, prev_accel + MAX_ACC_DELTA
                )
                steer = np.clip(
                    steer, prev_steer - MAX_STEER_DELTA, prev_steer + MAX_STEER_DELTA
                )
                agent["prev_control"] = (float(accel), float(steer))

                st = agent["state"]
                st.speed = max(0.0, min(st.speed + float(accel) * self.dt, MAX_SPEED))
                yaw_rate = (st.speed * math.tan(float(steer))) / WHEEL_BASE
                st.heading = (st.heading + math.degrees(yaw_rate * self.dt)) % 360.0
                h_rad = math.radians(st.heading)
                st.x += st.speed * math.sin(h_rad) * self.dt
                st.y += st.speed * math.cos(h_rad) * self.dt
                vsMap[avName] = st

                if is_debug_info:
                    print(
                        f"[推理 Step {self._inferStep}] 背景车 {avName} 推理中... 当前速度: {st.speed:.2f} m/s 当前朝向: {st.heading:.2f}"
                    )

        self.tessAuto.setAvChannel2AvMsgMap(vsMap)
        if is_debug_info:
            print(
                f"[推理 Step {self._inferStep}] Ego 推理中... 当前速度: {self.egoState.speed:.2f} m/s 当前朝向: {self.egoState.heading:.2f}"
            )

        self._inferStep += 1

        # 碰撞检测
        ego_id = egoVehicle.id()
        for v in vehicles:
            if v.id() != ego_id:
                if self._check_bbox_collision_vehi(egoVehicle, v):
                    self._is_collision = True
                    break

        # 终止与重置判断
        is_reached_goal = False
        s_ego = getattr(self, "_current_s_ego", 0.0)
        s_total = getattr(self, "_current_s_total", 0.0)
        if s_total > 0.0 and (s_total - s_ego) < 5.0:
            is_reached_goal = True

        is_out_of_bounds = False
        if abs(getattr(self, "_current_lateral_dist", 0.0)) > (
            MAX_LANE_WIDTH / 2.0 + 0.5
        ):
            is_out_of_bounds = True

        if self._is_collision or is_reached_goal or is_out_of_bounds:
            if self._is_collision:
                reason = "发生碰撞"
            elif is_reached_goal:
                reason = "成功到达终点"
            elif is_out_of_bounds:
                reason = "偏离车道"
            else:
                reason = "达到最大步数"

            print(f"[推理] {reason}，重置环境。")
            self._inferCreated = False

            if REPEAT_SINGLE_SCENARIO:
                self.clearTessngBgVehicles()
                if getattr(self, "multiInfer", None):
                    self.multiInfer.resetAll()
            else:
                self.doReset()
                print("[推理] 回合结束，自动切换到下一个场景...")

    # ============================================================
    #  训练线程
    # ============================================================

    def runTraining(self):
        from Utils.Constant import EGO_MODEL_FILENAME, BG_MODEL_FILENAME

        os.makedirs(MODEL_SAVE_DIR, exist_ok=True)
        # 训练模式下默认保存路径（当前正在训练的模型）
        savePath = os.path.join(MODEL_SAVE_DIR, "model")

        if TRAIN_MODE and self.scenarios:
            numScenarios = len(self.scenarios)
            stepsPerScenario = TOTAL_TIMESTEPS // numScenarios

            print("=" * 60)
            print(f"[训练] {numScenarios} 个场景, 总步数 {TOTAL_TIMESTEPS}")
            print(f"[训练] 目标：训练 Ego (主车) 避障与居中行驶")
            print(f"[训练] 背景障碍车由 JSON 加载，保持匀速")
            print("=" * 60)
            # ... (rest of runTraining)

            if RL_ALGO == "PPO":
                from stable_baselines3 import PPO
                from datetime import datetime

                model = PPO(
                    "MlpPolicy",
                    self.env,
                    policy_kwargs=dict(net_arch=[256, 256]),
                    learning_rate=5e-5,
                    n_steps=4096,
                    batch_size=256,
                    n_epochs=5,
                    gamma=0.99,
                    clip_range=0.1,
                    target_kl=0.015,
                    ent_coef=0.03,
                    gae_lambda=0.95,
                    verbose=1,
                    tensorboard_log=f"{TENSORBOARD_LOG}/{datetime.now().strftime('%Y%m%d_%H%M%S')}",
                )
            else:
                from stable_baselines3 import DQN

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
                    tensorboard_log=TENSORBOARD_LOG,
                )

            # for i in range(numScenarios):
            #     print(f"\n[训练] === 场景 {i}: {self.scenarios[i]['file']} ===")
            #     self.switchScenario(i)
            #     total_timestaps = TOTAL_TIMESTEPS * ratio[i]
            #     model.learn(total_timesteps=total_timestaps , reset_num_timesteps=False)
            model.learn(total_timesteps=TOTAL_TIMESTEPS, reset_num_timesteps=False)
            model.save(savePath)
            print(f"\n[训练] 完成! 模型: {savePath}")

        elif not TRAIN_MODE:
            egoPath = os.path.join(MODEL_SAVE_DIR, EGO_MODEL_FILENAME)
            bgPath = os.path.join(MODEL_SAVE_DIR, BG_MODEL_FILENAME)

            print(f"[推理] 加载 Ego 模型: {egoPath}")
            print(f"[推理] 加载背景车模型: {bgPath}")

            try:
                from stable_baselines3 import PPO, DQN

                ModelClass = PPO if RL_ALGO == "PPO" else DQN

                if os.path.exists(egoPath):
                    self.egoModel = ModelClass.load(egoPath)
                    print("[推理] Ego 模型加载成功。")

                if os.path.exists(bgPath):
                    self.bgModel = ModelClass.load(bgPath)
                    print("[推理] 背景车模型加载成功。")

            except Exception as e:
                print(f"[推理] 模型加载失败: {e}")
        else:
            print("[训练] Data 目录无 JSON 文件")
            return

        # 切换到推理模式前，关闭 env 解除可能的阻塞
        self.env.close()

        # 等一小会让 afterOneStep 从阻塞中退出
        import time

        time.sleep(0.5)

        # 推理模式
        if not TRAIN_MODE:
            print("\n[推理] 初始化多车推理(备用)...")
            self.multiInfer = MultiVehicleInference(savePath, algo=RL_ALGO)
            for scenario in self.scenarios:
                prefix = scenario["file"].replace(".json", "")
                for name, info in scenario["vehicles"].items():
                    self.multiInfer.addVehicle(
                        f"{prefix}_{name}",
                        info["path"],
                        info["speed"],
                        color=info["color"],
                    )

            self._inferCreated = False
            self.isFirstStep = False  # 推理模式不需要走 onFirstStep

            # 训练线程完成，不需要保持活着
            print("[推理] 训练线程退出，推理由 afterOneStep 主线程驱动")

    # ============================================================
    #  场景管理
    # ============================================================

    def loadScenario(self, idx):
        if idx >= len(self.scenarios):
            return
        self.currentScenarioIdx = idx
        self.currentVehicles = self.scenarios[idx]["vehicles"]
        self.egoName = "ego"  # 切换场景前重置默认 Ego 名称
        self.initBgAgents()

    def switchScenario(self, idx):
        self.loadScenario(idx)

    def initBgAgents(self):
        """初始化背景车运行状态，并加载当前场景的 Ego 参考路径"""
        self.bgAgents = {}

        # 1. 尝试从场景中提取 Ego 路径
        ego_info = self.currentVehicles.get(self.egoName)

        if ego_info is None and len(self.currentVehicles) > 0:
            print(
                f"[警告] 当前场景 {self.scenarios[self.currentScenarioIdx]['file']} 缺少名为 '{self.egoName}' 的车辆定义！"
            )
            # 临时补救：以第一个车辆的路径作为 Ego 路径，并在背景车中忽略它
            fallback_name = list(self.currentVehicles.keys())[0]
            print(f"[警告] 退路策略：将使用 '{fallback_name}' 的路径作为 Ego 路径。")
            ego_info = self.currentVehicles[fallback_name]
            self.egoName = fallback_name  # 同步更改名字，防止被当成背景车

        if ego_info:
            # 核心修正：参考路径统一转为数学坐标 (Y取反)
            gui_smoothed = MultiVehicleInference._smoothPath(ego_info["path"], 1.0)
            self.egoSmoothedPath = [(p[0], -p[1]) for p in gui_smoothed]

            # 记录当前场景下 Ego 的期望速度
            self._ego_target_speed = ego_info.get("speed", 15.0)

            # 如果初始状态存在，修正它的值
            if self.egoState:
                init_x, init_y = (
                    self.egoSmoothedPath[0]
                    if self.egoSmoothedPath
                    else (-650.0, -287.0)
                )
                _, _, init_heading = self._posOnPath(self.egoSmoothedPath, 0.0)
                self.egoState.x, self.egoState.y, self.egoState.heading = (
                    init_x,
                    init_y,
                    init_heading,
                )
                self.egoState.speed = self._ego_target_speed  # 初始速度对齐期望速度

        # 2. 初始化背景车状态
        for name, info in self.currentVehicles.items():
            if name == self.egoName:
                continue  # Ego 的路径已单独处理，不计入背景车

            smoothed = MultiVehicleInference._smoothPath(info["path"], 1.0)
            totalLen = MultiVehicleInference._pathLength(smoothed)

            # 初始化起点坐标
            init_x, init_y = smoothed[0] if smoothed else (0.0, 0.0)

            self.bgAgents[name] = {
                "speed": info["speed"],
                "progress": 0.0,
                "smoothed": smoothed,
                "path": info.get("path", []),
                "totalLength": totalLen,
                "prevHeading": 0.0,
                "x": init_x,
                "y": init_y,
                "color": info.get("color"),
                "controlMode": self._bgControlMode(name, info),
            }

    # ============================================================
    #  背景车创建与控制
    # ============================================================

    def createBgVehicles(self):
        """在 Tessng 中创建背景车"""
        vsMap = {}
        for name, agent in self.bgAgents.items():
            if self._isTessngControlledBg(name):
                self.createTessngBgVehicle(name, agent)
                continue

            s = agent["smoothed"]
            if len(s) < 2:
                continue
            x, y = s[0]
            dx = s[1][0] - s[0][0]
            dy = s[1][1] - s[0][1]
            heading = math.degrees(math.atan2(dx, dy)) % 360.0
            vsMap[name] = VehicleState(x=x, y=y, heading=heading, speed=agent["speed"])
        self.tessAuto.setAvChannel2AvMsgMap(vsMap)
        self.tessAuto.vehicleCreate()

    def applyBgActions(self, control):
        """
        背景车控制分发器。
        当前训练 Ego 阶段：使用纯 NPC 路径跟随模式。
        后期接入选手模型：可在此处或 _afterOneStepTraining 中加载选手的 .zip 模型并预测动作。
        """
        vsMap = {}

        for name, agent in self.bgAgents.items():
            if self._isTessngControlledBg(name):
                continue

            s = agent["smoothed"]
            if len(s) < 2:
                continue

            # ===== NPC 模式：路径跟随 =====
            agent["progress"] += agent["speed"] * self.dt
            agent["progress"] = min(agent["progress"], agent["totalLength"])
            x, y, heading = self._posOnPath(s, agent["progress"])
            agent["x"], agent["y"], agent["heading"] = x, y, heading
            vsMap[name] = VehicleState(x=x, y=y, heading=heading, speed=agent["speed"])

        self.tessAuto.setAvChannel2AvMsgMap(vsMap)

    def _bgControlMode(self, name, info):
        """
        确定背景车的控制模式。
        """
        mode = str(info.get("control", info.get("controlMode", ""))).strip().lower()
        if name in self.tessngControlledBgNames:
            return "tessng"
        if mode in {"tessng", "native", "builtin", "built-in", "base"}:
            return "tessng"
        return "model"

    def _isTessngControlledBg(self, name):
        """判断指定背景车是否由 TESSNG single routing 控制"""
        agent = self.bgAgents.get(name)
        return bool(agent and agent.get("controlMode") == "tessng")

    def createTessngBgVehicle(self, name, agent):
        """创建 TESSNG 驱动的背景车"""
        self.removeExternalBgControl(name)

        if self.createTessngBgRouting(name, agent):
            return

        print(f"[TESSNG BG] {name} 创建 single routing 失败，跳过该车辆")

    def createTessngBgRouting(self, name, agent):
        """创建 TESSNG single routing 来控制背景车辆"""
        if name in self.tessngBgRoutingByName:
            return True

        netIface = self.netIface or tessngIFace().netInterface()
        simIface = self.simIface or tessngIFace().simuInterface()
        if not netIface or not simIface:
            return False

        smoothed = agent.get("path") or []
        if len(smoothed) < 2:
            return False

        waypoints = []
        for index, (x, y) in enumerate(smoothed):
            waypoint = self.createTessngWaypoint(
                QPointF(x, -y), index, agent.get("speed", 10.0)
            )
            if waypoint:
                waypoints.append(waypoint)

        if len(waypoints) < 2:
            print(f"[TESSNG BG] {name} 创建 routing 失败: 有效 waypoint 不足")
            return False

        first_point = QPointF(smoothed[0][0], -smoothed[0][1])
        if not self.setupTessngRoutingDispatch(first_point, waypoints):
            print(f"[TESSNG BG] {name} 创建 routing 失败: 发车点创建失败")
            return False

        param = Online.DynaSingleRoutingParam()
        param.level = "lane"
        param.lWaypointId = [wp.id() for wp in waypoints]
        param.desiredMode = int(agent.get("desiredMode", 0))
        param.name = "routing"

        routing = netIface.createSingleRouting(param, waypoints)
        if not routing:
            print(f"[TESSNG BG] {name} 创建 single routing 失败")
            return False

        self.tessngBgRoutingByName[name] = routing
        print(
            f"[TESSNG BG] {name} 使用 TESSNG single routing 控制, routing_id={routing.id()}"
        )
        return True

    def createTessngWaypoint(self, point, number, speed):
        """创建TESSNG途径点"""
        netIface = self.netIface or tessngIFace().netInterface()
        locations = netIface.locateOnCrid(point, 9)
        if not locations:
            print(f"[TESSNG BG] waypoint 无法定位: ({point.x():.2f}, {point.y():.2f})")
            return None

        param = Online.DynaWaypointParam()
        param.number = number
        param.desiredSpeed = speed
        param.desiredTime = 0
        param.bRemoveVehi = False

        lane_object = locations[0].pLaneObject
        if lane_object.isLane():
            lane = lane_object.castToLane()
            param.type = "lane"
            param.roadId = lane.link().id()
            param.dist = lane.distToStartPoint(point)
            param.laneId = lane.id()
        else:
            lane_connector = lane_object.castToLaneConnector()
            param.type = "laneconnector"
            param.roadId = lane_connector.connector().id()
            param.dist = lane_connector.distToStartPoint(point)
            param.laneId = lane_connector.fromLane().id()
            param.toLaneId = lane_connector.toLane().id()

        return netIface.createWaypoint(param)

    def setupTessngRoutingDispatch(self, first_point, waypoints):
        """设置 TESSNG single routing 的发车点和路径点

        该函数用于在TESSNG仿真环境中为single routing模式创建车辆发车点，并配置路径点参数。
        主要流程：查找道路/连接器 -> 定位车道 -> 创建发车点 -> 配置发车参数 -> 关联路径点

        Args:
            first_point: 起始点坐标，用于定位发车车道
            waypoints: 路径点列表，包含车辆行驶的完整路径信息

        Returns:
            bool: 成功返回True，失败返回False
        """

        netIface = self.netIface or tessngIFace().netInterface()
        simIface = self.simIface or tessngIFace().simuInterface()
        first_wp = waypoints[0]  # 获取第一个路径点

        # 根据第一个路径点的道路ID查找Link（路段）和Connector（连接段）
        link = netIface.findLink(first_wp.roadId())
        connector = netIface.findConnector(first_wp.roadId())
        # 既不是路段也不是连接段，无法创建发车点
        if not link and not connector:
            return False

        locations = netIface.locateOnCrid(first_point, 9)
        if not locations:
            return False

        # 获取定位到的车道对象
        lane_object = locations[0].pLaneObject
        dispatch_point = None  # 发车点对象
        lane_number = None  # 车道编号

        if link:
            lane = lane_object.castToLane()
            if not lane:
                return False
            lane_number = lane.number()
            dispatch_point = netIface.createDispatchPoint(
                link, first_wp.distToStart(), lane_number
            )
        elif connector:
            lane_connector = lane_object.castToLaneConnector()
            if not lane_connector:
                return False
            dispatch_point = netIface.createDispatchPoint(
                connector, first_wp.distToStart()
            )

        # 发车点创建失败
        if not dispatch_point:
            return False

        # 设置发车点不可见（避免在界面上显示）
        netIface.setDispatchVisable(dispatch_point, False)

        simu_time_sec = int(simIface.simuTimeIntervalWithAcceMutiples() / 1000)
        # 设置发车模式为1（立即发车模式）
        dispatch_point.setDispatchMode(1)

        # 根据dispatch_point支持的方法设置发车时间
        if hasattr(dispatch_point, "addDispatchTime"):
            if lane_number is None:
                dispatch_point.addDispatchTime(1, simu_time_sec)
            else:
                dispatch_point.addDispatchTime(1, simu_time_sec, lane_number)
        else:
            dispatch_point.addDispatchInterval(1, 20, 1)

        # 将第一个路径点关联到发车点
        first_wp.setDeparturePointId(dispatch_point.id())
        # 设置最后一个路径点为移除车辆（到达终点后自动移除）
        waypoints[-1].setRemoveVehi(True)
        return True

    def removeExternalBgControl(self, name):
        """
        根据 name 移除模型控制的背景车
        清理 TESSNG single routing 和相关映射
        """
        self.tessAuto.avChannel2AvMsgMap.pop(name, None)

        tessng_id = self.tessAuto.avName2TessngIdMap.pop(name, None)
        if tessng_id is not None:
            self.tessAuto.tessngId2AvNameMap.pop(tessng_id, None)
            self.tessAuto.alreadyLaunchedTessngIdSet.discard(tessng_id)
            self.tessAuto.alreadyLaunchedAvIdSet.discard(tessng_id)

        self.tessAuto.alreadyLaunchedAvNameSet.discard(name)
        self.tessAuto.mainVehiclePtrDict.pop(name, None)

    def clearTessngBgVehicles(self):
        """在重新创建场景之前，停止所有受TESSNG控制的后台车辆的行驶并移除它们的 routing"""
        simIface = self.simIface or tessngIFace().simuInterface()
        netIface = self.netIface or tessngIFace().netInterface()

        for name, routing in list(self.tessngBgRoutingByName.items()):
            try:
                if simIface:
                    for vehicle in simIface.getVehiclesOnSingleRouting(routing.id()):
                        print(f"[TESSNG BG] 停止 {name} (ID: {vehicle.id()}) 的行驶")
                        vehicle.vehicleDriving().stopVehicle()

                netIface.removeSingleRouting(routing)
                print(f"[TESSNG BG] 已清理 routing {name} (ID: {routing.id()})")
            except Exception as e:
                print(f"[TESSNG BG] 清理 routing 失败 {name}: {e}")
        self.tessngBgRoutingByName.clear()

    # ============================================================
    #  环境回调
    # ============================================================

    def doReset(self):
        """重置环境状态"""
        self.yawRateCalc.reset()
        self.prevSteer = 0.0
        self.stepCount = 0
        self.egoState = None  # 触发 _updateEgoByRL 中的初始位姿分配
        self._is_collision = False
        self._is_out_of_bounds = False
        self._is_reached_goal = False  # 清理终点标志
        self._prev_s_ego = 0.0  # 清理进度缓存
        self._current_lateral_dist = 0.0  # 清理横向偏差缓存
        self._current_angle_diff = 0.0  # 清理航向角偏差缓存
        self._ego_stuck_count = 0  # 清理卡死计数

        # 切换场景
        self.clearTessngBgVehicles()
        self._doSwitchScenario()

        for name, agent in self.bgAgents.items():
            info = self.currentVehicles.get(name, {})
            agent["speed"] = info.get("speed", 10.0)
            agent["progress"] = 0.0
            smoothed = agent["smoothed"]
            init_x, init_y = smoothed[0] if smoothed else (0.0, 0.0)
            agent["x"], agent["y"] = init_x, init_y

        self.isFirstStep = True
        print(f"\n[Reset] Ego 状态已重置，开始新的 Episode。")

    def _doSwitchScenario(self):
        """切换场景的核心逻辑：训练模式随机切换，推理模式顺序切换"""

        if not self.scenarios:
            return

        if TRAIN_MODE:
            # 如果队列空了，重新装填并洗牌
            if not self.scenario_indices:
                self.scenario_indices = list(range(len(self.scenarios)))
                np.random.shuffle(self.scenario_indices)  # 随机打乱顺序

            # 从队列中取出一个场景索引
            target_idx = self.scenario_indices.pop(0)
            self.switchScenario(target_idx)
        else:
            # 推理模式按顺序切换
            next_idx = self.currentScenarioIdx + 1
            if next_idx >= len(self.scenarios):
                # 结束仿真
                if self.iface.simuInterface().isRunning():
                    print("[推理] 已完成所有场景的推理，仿真结束")
                    self.sig_stop_simu.emit()

            self.switchScenario(next_idx)
            self.currentScenarioIdx = next_idx

    def buildInitialObs(self):
        return np.zeros(94, dtype=np.float32)

    def buildObs(self, vehicle, vehicles):
        """
        构建观测向量（根据主体车辆计算）, 即状态空间,
        包含 94 维特征，并返回一个字典包含 Ego 相关的额外信息供奖励函数使用
        """
        obs = np.zeros(94, dtype=np.float32)
        obs_info = {
            "curvature_radius": None,
            "current_steer": 0.0,
        }

        # 判断是 Ego 还是背景车，选择对应的路径
        egoTessngId = self.tessAuto.avName2TessngIdMap.get(self.egoName)
        is_ego = vehicle.id() == egoTessngId
        if is_ego:
            centerLine = self.egoSmoothedPath
            v_speed = self.egoState.speed if self.egoState else vehicle.currSpeed()
            control = getattr(self, "currentEgoControl", (0.0, 0.0))
            prev_steer = self.prevSteer
        else:
            avName = self.tessAuto.tessngId2AvNameMap.get(vehicle.id())
            agent = self.bgAgents.get(avName)
            centerLine = agent["smoothed"] if agent else []
            v_speed = agent["speed"] if agent else vehicle.currSpeed()
            control = agent.get("prev_control", (0.0, 0.0)) if agent else (0.0, 0.0)
            prev_steer = control[1]

        # 1. 基础状态 (5维 + 运动学)
        laneResult = LaneProjector.fromTessngVehicle(vehicle, p2m)
        if laneResult:
            obs[0] = np.clip(laneResult.dist_left / MAX_LANE_WIDTH, 0, 1)
            obs[1] = np.clip(laneResult.dist_right / MAX_LANE_WIDTH, 0, 1)
            obs[2] = np.clip((laneResult.lateral_offset / MAX_LANE_WIDTH) + 0.5, 0, 1)
            obs[3] = laneResult.angle_diff

        obs[4] = np.clip(v_speed / MAX_SPEED, 0, 1)
        obs[5] = np.clip((control[1] / (2 * MAX_STEER_ANGLE)) + 0.5, 0, 1)
        obs[6] = np.clip((control[0] - MAX_DECEL) / (MAX_ACCEL - MAX_DECEL), 0, 1)
        obs[7] = np.clip((prev_steer / (2 * MAX_STEER_ANGLE)) + 0.5, 0, 1)

        obs_info["current_steer"] = float(control[1])

        MAX_YAW_RATE = 1.0
        yawRate = self.yawRateCalc.update(vehicle.id(), vehicle.angle(), self.dt)
        obs[8] = np.clip((yawRate / (2 * MAX_YAW_RATE)) + 0.5, 0, 1)

        # 2. 导航 (13维)
        if centerLine and len(centerLine) >= 2:
            sparse = NavigationCalculator.sparsifyByDistance(centerLine, 3.0)
            vPos = vehicle.pos()
            navResult = NavigationCalculator.compute(
                p2m(vPos.x()),
                p2m(vPos.y()),
                vehicle.angle(),
                sparse,
                numCheckpoints=4,
            )
            # [新增] 缓存归一化的曲率半径，供奖励函数计算动态限速
            obs_info["curvature_radius"] = navResult.curvatureRadius

            for i in range(min(5, len(navResult.forwardGaps))):
                obs[9 + i * 2] = navResult.forwardGaps[i]
                obs[9 + i * 2 + 1] = navResult.lateralGaps[i]
            obs[19] = navResult.curvatureRadius
            obs[20] = navResult.curvatureDirection
            obs[21] = navResult.laneAngleDiff

        # 3. 周车雷达 (72维)
        surroundResult = SurroundingCalculator.fromTessngVehicles(
            vehicle, vehicles, p2m, maxNearby=8
        )
        for i, r in enumerate(surroundResult.radar):
            obs[22 + i] = r

        return obs, obs_info

    def applyEgoObsInfo(self, obs_info):
        """Apply Ego-only observation side effects after building an observation."""
        self.prevSteer = obs_info.get("current_steer", self.prevSteer)
        curvature_radius = obs_info.get("curvature_radius")
        if curvature_radius is not None:
            self._current_curvature_radius = curvature_radius

    # ============================================================
    # Start: 训练背景车的奖励函数设计（核心：鼓励干扰主车，惩罚保持理想状态）
    # 注：项目转为训练主车，这部分逻辑保留但不再使用，未来可根据需求启用
    # ============================================================
    def computeReward(self, bgVehicle) -> float:
        """奖励函数设计：鼓励背景车干扰主车，惩罚主车保持理想状态"""
        attacker_name = getattr(self, "_attacker_name", None)
        if (
            not self.bgAgents
            or self.egoState is None
            or attacker_name not in self.bgAgents
        ):
            return 0.0

        agent = self.bgAgents[attacker_name]
        s = self._prepare_state_info(agent)

        self._is_collision = False
        self._is_out_of_bounds = False
        self._is_wrong_way = False

        reward = 0.0

        # 生存奖励（核心反自杀）
        reward += self._r_survival(s)

        # 距离控制（核心驱动力）
        reward += self._r_distance(s)

        # 对抗行为（真正学习目标）
        reward += self._r_interaction(s)

        # 行为约束（防发疯）
        reward += self._r_action(agent)

        # 速度进度奖励（新增：核心驱动力）
        reward += self._r_progress(s)

        # 车道约束（渐进惩罚）
        lane_penalty, out = self._r_lane(bgVehicle)
        reward += lane_penalty
        if out:
            reward -= 0.5  # 每步持续惩罚
            self._is_out_of_bounds = True
            # return -3.0   # 强惩罚

        # 方向约束
        dir_penalty, wrong = self._r_direction(agent, s)
        reward += dir_penalty
        if wrong:
            self._is_wrong_way = True
            reward -= 0.5  # 每步持续惩罚
            # return -3.0

        # 7 碰撞（唯一成功）
        if self._check_collision(s):
            self._is_collision = True
            return +2.0

        # 8 卡死连续惩罚
        reward += self._r_stop(agent)

        # 9 提前结束惩罚（关键）
        if self.stepCount < 30:
            reward -= 0.05 * (30 - self.stepCount)

        return float(np.clip(reward, -3.0, 2.0))

    def _prepare_state_info(self, agent):
        """计算相对位置、投影距离等物理量"""
        ego = self.egoState
        egoHeadingRad = math.radians(90.0 - ego.heading)
        actual_x = agent.get("x", 0.0)
        actual_y = agent.get("y", 0.0)
        actual_heading = agent.get("heading", 0.0)

        dx, dy = actual_x - ego.x, actual_y - ego.y
        distToEgo = math.sqrt(
            (p2m(actual_x) - p2m(ego.x)) ** 2 + (p2m(actual_y) - p2m(ego.y)) ** 2
        )

        real_x, real_y, expected_heading = self._posOnPath(
            agent["smoothed"], agent["progress"]
        )

        return {
            "ego_x": ego.x,
            "ego_y": ego.y,
            "ego_heading": ego.heading,
            "ego_speed": ego.speed,
            "actual_x": actual_x,
            "actual_y": actual_y,
            "actual_heading": actual_heading,
            "forwardDist": dx * math.cos(egoHeadingRad) + dy * math.sin(egoHeadingRad),
            "lateralDist": -dx * math.sin(egoHeadingRad) + dy * math.cos(egoHeadingRad),
            "distToEgo": distToEgo,
            "speedDiff": agent["speed"] - ego.speed,
            "dx": dx,
            "dy": dy,
            "agent_speed": agent["speed"],
            "real_x": real_x,
            "real_y": real_y,
            "expected_heading": expected_heading,
        }

    def _r_survival(self, s):
        """生存奖励：鼓励保持速度，避免碰撞"""
        return 0.05 if s["agent_speed"] > 0.5 else 0.0

    def _r_distance(self, s):
        """距离控制：鼓励保持理想距离，避免过近过远"""
        d = s["distToEgo"]

        # 理想距离：5~15m
        if d < 5:
            return -0.1 * (5 - d)  # 太近反而惩罚（避免无脑撞）
        elif d < 15:
            return +0.1 * (1 - abs(d - 10) / 10)
        elif d < 30:
            return +0.02 * (1 - (d - 15) / 15)
        else:
            return -0.05

    def _r_stop(self, agent):
        """停止惩罚：鼓励持续移动，防止卡位不动"""
        # 7.5 卡死连续惩罚（不终止，只扣分）
        if agent["speed"] < 0.1:
            agent["_stuck_count_reward"] = agent.get("_stuck_count_reward", 0) + 1
            if agent["_stuck_count_reward"] > 10:
                # 超过 10 帧静止，每帧给较强的负反馈，逼迫它动起来
                return -0.1 * min(10.0, (agent["_stuck_count_reward"] - 10) / 5.0)
        else:
            agent["_stuck_count_reward"] = 0

        return 0

    def _r_progress(self, s):
        """进度奖励：鼓励进度匹配和速度提升"""
        # 计算实际速度在期望方向上的投影
        # 如果转圈时车头反了，投影就是负的，不仅不给奖还要扣分
        expected_heading = s["expected_heading"]
        actual_heading = s["actual_heading"]

        diff_rad = math.radians(actual_heading - expected_heading)
        # 投影速度 = 速率 * cos(角度差)
        projected_speed = s["agent_speed"] * math.cos(diff_rad)

        if projected_speed > 0.5:
            return 0.1 * projected_speed
        else:
            # 如果投影速度是负值（逆行或横着滑），给予严厉惩罚
            return -0.5

    def _r_interaction(self, s):
        """对抗行为奖励：鼓励背景车卡位、追击和阻挡"""
        r = 0.0

        forward = s["forwardDist"]
        lateral = abs(s["lateralDist"])
        dv = s["speedDiff"]

        # ===== 卡位（最重要）=====
        if 0 < forward < 10 and lateral < 2.0:
            r += 0.2  # 强奖励

        # ===== 追击 =====
        if forward < 0 and dv > 0:
            r += 0.05

        # ===== 阻挡 =====
        if forward > 0 and dv < 0:
            r += 0.05

        return r

    def _r_action(self, agent):
        """行为约束：限制过激的转向和加减速，鼓励平稳干扰"""
        accel, steer = self.currentControl

        r = 0.0

        # 转向惩罚（降低权重）
        r -= abs(steer) * 0.8

        # 抖动惩罚, 防止“画龙”和剧烈摆动
        prev = agent.get("prev_steer", 0.0)
        r -= abs(steer - prev) * 1.5  # 增加突变惩罚
        agent["prev_steer"] = steer

        return r

    def _r_lane(self, bgVehicle):
        """
        车道约束：
        - 轻微偏离（0~1m）：无惩罚
        - 中等偏离（1~2.5m）：0.5惩罚
        - 严重偏离（2.5~3m）：-3惩罚
        """
        lane = LaneProjector.fromTessngVehicle(bgVehicle, p2m)
        if not lane:
            return -3.0, True

        offset = abs(lane.lateral_offset)

        # 渐进惩罚（非常关键）
        if offset > 2.5:
            return -3.0, True
        elif offset > 1.0:
            return -(offset - 1.0) * 0.5, False
        else:
            return 0.0, False

    def _check_collision(self, s):
        """
        检查是否发生碰撞
        """
        return self._check_bbox_collision(
            s["actual_x"],
            s["actual_y"],
            s["actual_heading"],
            4.8,
            2.0,
            s["ego_x"],
            s["ego_y"],
            s["ego_heading"],
            4.8,
            2.0,
        )

    def _r_direction(self, agent, s):
        """
        航向约束：
        - 防止逆行 / 掉头
        - 提供连续惩罚（而不是突然 -1）
        """

        # === 1. 获取路径期望方向 ===
        expected_heading = s["expected_heading"]
        actual_heading = s["actual_heading"]

        # === 2. 计算角度差（0~180）===
        diff = abs(actual_heading - expected_heading)
        if diff > 180:
            diff = 360 - diff

        # === 3. 连续惩罚（核心）===
        # 0~5°：基本合理
        if diff < 5:
            penalty = 0.0

        # 5~45°：逐渐惩罚（线性）
        elif diff < 45:
            penalty = -(diff - 5) / 40.0 * 0.5  # 最大 -0.5

        # >45°：严重错误（快速拉满惩罚）
        else:
            penalty = -1.0

        # === 4. 判定“逆行终止”===
        # 条件：角度大 + 持续时间
        if diff > 60:
            agent["_wrong_dir_count"] = agent.get("_wrong_dir_count", 0) + 1
        else:
            agent["_wrong_dir_count"] = 0

        # 连续 5 帧严重偏离 → 判定失败
        if agent["_wrong_dir_count"] > 10:
            return -3.0, True

        return penalty, False

    def checkDone(self) -> bool:
        """
        终止判断：检查背景车是否完成任务
        """
        # 1. 发生碰撞，干扰成功
        if getattr(self, "_is_collision", False):
            print("[Done] 主攻手发生碰撞! 干扰任务圆满完成。")
            return True

        # 1.5 严重偏离车道，干扰失败（换成持续惩罚，避免自杀式逃避惩罚）
        # if getattr(self, "_is_out_of_bounds", False):
        #     print("[Done] 主攻手严重偏离车道! 干扰失败。")
        #     return True

        # 1.6 逆行或掉头，干扰失败（换成持续惩罚，避免自杀式逃避惩罚）
        # if getattr(self, "_is_wrong_way", False):
        #     print("[Done] 主攻手逆行或掉头! 干扰失败。")
        #     return True

        # 2. 达到最大步数
        if self.stepCount >= TRAIN_MAX_STEPS:
            return True

        # 3. 检查主攻手状态
        # attacker_name = getattr(self, "_attacker_name", None)
        # if attacker_name and attacker_name in self.bgAgents:
        #     agent = self.bgAgents[attacker_name]
        #     if agent["progress"] >= agent["totalLength"]:
        #         print("[Done] 主攻手到达终点。")
        #         return True

        #     if agent["speed"] < 0.1:
        #         agent["_stuck_count"] = agent.get("_stuck_count", 0) + 1
        #         if agent["_stuck_count"] > 15:
        #             print("[Done] 探测到主攻手卡死，重置。")
        #             return True
        #     else:
        #         agent["_stuck_count"] = 0

        return False

    # ============================================================
    # End: 训练背景车的奖励函数设计
    # ============================================================

    # ============================================================
    #  工具
    # ============================================================

    def _getFrenetProgress(self, path, x, y):
        """
        计算点 (x, y) 在给定路径 path 上的 Frenet 纵向投影距离 s 等信息。
        返回: (s_ego, s_total, lateral_dist, expected_heading)
        """
        if not path or len(path) < 2:
            return 0.0, 0.0, 0.0, 0.0

        min_dist = float("inf")
        best_s = 0.0
        accumulated_s = 0.0
        best_lateral = 0.0
        best_heading = 0.0

        p = np.array([x, y])

        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])

            seg_vec = p2 - p1
            seg_len = np.linalg.norm(seg_vec)
            if seg_len < 1e-6:
                continue

            # 投影比例 t
            t = np.dot(p - p1, seg_vec) / (seg_len * seg_len)
            t = max(0.0, min(1.0, t))  # 限制在线段内部

            proj_p = p1 + t * seg_vec
            dist = np.linalg.norm(p - proj_p)

            if dist < min_dist:
                min_dist = dist
                best_s = accumulated_s + t * seg_len
                best_lateral = dist

                dx, dy = seg_vec[0], seg_vec[1]
                best_heading = math.degrees(math.atan2(dx, dy)) % 360.0

            accumulated_s += seg_len

        return (
            float(best_s),
            float(accumulated_s),
            float(best_lateral),
            float(best_heading),
        )

    def findBgVehicle(self, vehicles):
        """找到主车和所有背景车"""
        egoVehicle = None
        bgVehicles = []
        egoTessngId = self.tessAuto.avName2TessngIdMap.get(self.egoName)
        for v in vehicles:
            vid = v.id()
            if vid == egoTessngId:
                egoVehicle = v
            elif vid in self.tessAuto.alreadyLaunchedTessngIdSet:
                bgVehicles.append(v)
        return egoVehicle, bgVehicles

    @staticmethod
    def _check_bbox_collision(x1, y1, heading1, l1, w1, x2, y2, heading2, l2, w2):
        """使用分离轴定理(SAT)检测两个OBB(带有朝向的矩形)是否发生碰撞"""

        def get_corners(cx, cy, heading, length, width):
            # 将 heading 转换为数学弧度 (90 - heading)
            rad = math.radians(90.0 - heading)
            cos_h = math.cos(rad)
            sin_h = math.sin(rad)
            hl = length / 2.0
            hw = width / 2.0

            # 车头方向为X轴，车身宽为Y轴
            dx1, dy1 = hl * cos_h, hl * sin_h
            dx2, dy2 = -hw * sin_h, hw * cos_h

            return [
                (cx + dx1 + dx2, cy + dy1 + dy2),
                (cx + dx1 - dx2, cy + dy1 - dy2),
                (cx - dx1 - dx2, cy - dy1 - dy2),
                (cx - dx1 + dx2, cy - dy1 + dy2),
            ]

        def get_axes(corners):
            axes = []
            for i in range(2):  # 矩形只需要相邻两条边的法向量
                p1 = corners[i]
                p2 = corners[(i + 1) % 4]
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length_edge = math.hypot(dx, dy)
                if length_edge > 1e-6:
                    axes.append((-dy / length_edge, dx / length_edge))
            return axes

        corners1 = get_corners(x1, y1, heading1, l1, w1)
        corners2 = get_corners(x2, y2, heading2, l2, w2)

        axes = get_axes(corners1) + get_axes(corners2)

        for axis in axes:
            min1, max1 = float("inf"), float("-inf")
            for p in corners1:
                proj = p[0] * axis[0] + p[1] * axis[1]
                min1, max1 = min(min1, proj), max(max1, proj)

            min2, max2 = float("inf"), float("-inf")
            for p in corners2:
                proj = p[0] * axis[0] + p[1] * axis[1]
                min2, max2 = min(min2, proj), max(max2, proj)

            if max1 < min2 or max2 < min1:
                return False  # 找到分离轴，没有碰撞

        return True  # 所有轴都有重叠，发生碰撞

    @staticmethod
    def _posOnPath(smoothed, dist):
        """
        在平滑路径上定位

        JSON 坐标约定：x 同 GUI，y = -GUI_y
        Tessng 航向角：正北0°顺时针，基于 GUI 坐标（y 向下）
        转换：heading = atan2(dx, -dy_gui)
        """
        accumulated = 0.0
        for i in range(len(smoothed) - 1):
            ax, ay = smoothed[i]
            bx, by = smoothed[i + 1]
            segLen = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
            if accumulated + segLen >= dist:
                ratio = (dist - accumulated) / segLen if segLen > 1e-12 else 0.0
                x = ax + ratio * (bx - ax)
                y = ay + ratio * (by - ay)
                dx = bx - ax
                dy = by - ay
                # 关键修正：在数学坐标系下(Y向上为北)，atan2(dx, dy) 直接符合 TNG Heading (北0, 顺时针)
                heading = math.degrees(math.atan2(dx, dy)) % 360.0
                return x, y, heading
            accumulated += segLen
        bx, by = smoothed[-1]
        ax, ay = smoothed[-2]
        dx = bx - ax
        dy = by - ay
        heading = math.degrees(math.atan2(dx, dy)) % 360.0
        return bx, by, heading
