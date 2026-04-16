"""
MySimulator —— 主车(选手) + 背景车(DQN) 集成

架构：
  主车(ego): 由 TestPlayer（选手代码）控制，通过 PlayerManager 加载
             MySimulator 只读取主车的位置/航向角，不控制它
  背景车:    由 DQN 模型控制，沿 Data/ 目录下 JSON 定义的路径行驶
             训练目标：学会干扰主车

流程：
  训练模式 → DQN 控制背景车，观测包含主车位置，奖励鼓励干扰主车
  推理模式 → 加载训练好的模型，背景车 + 主车同时运行
"""

import math
import os
import threading
import numpy as np

from PySide2.QtCore import QObject

from Tessng import (
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
from ScenarioLoader import ScenarioLoader
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
    DATA_DIR,
    RL_ALGO,
    MODEL_SAVE_DIR,
    TENSORBOARD_LOG,
    TRAIN_MAX_STEPS
)


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

        # ===== Ego 路径 (同步自 TestPlayer) =====
        self.egoCenterLine = [
            (-650.047, 287.819), (-662.097, 272.913), (-680.481, 248.938),
            (-693.14, 233.513), (-711.580, 211.293), (-724.399, 194.875),
            (-733.395, 183.068), (-746.664, 167.212), (-754.873, 148.658),
            (-759.146, 129.204), (-758.247, 108.064), (-751.612, 87.486),
            (-742.391, 72.417), (-728.110, 54.201), (-714.054, 36.321),
            (-694.038, 10.683)
        ]
        self.egoSmoothedPath = MultiVehicleInference._smoothPath(self.egoCenterLine, 1.0)

        # ===== 主车：由选手的 TestPlayer 控制 =====
        self.playerManager = PlayerManager()
        self.egoName = "ego"
        # ... rest of init ...

        # ===== 背景车：从 Data 目录 JSON 加载，由 DQN 控制 =====
        self.scenarioLoader = ScenarioLoader(DATA_DIR)
        self.scenarios = self.scenarioLoader.loadAll()

        # 当前场景
        self.currentScenarioIdx = 0
        self.currentVehicles = {}
        self.bgAgents = {}  # 背景车运行状态

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
        self.trainThread = None
        self.stepCount = 0
        self.episodeReward = 0.0
        self.episodeCount = 0

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
            self.egoState = VehicleState(x=-650.047, y=287.819, heading=140.0, speed=10.0)
        
        accel, steer = self.currentControl
        
        # 运动学模型更新
        dt = self.dt
        self.egoState.speed += accel * dt
        self.egoState.speed = max(0.0, min(self.egoState.speed, MAX_SPEED))
        
        yaw_rate = (self.egoState.speed * math.tan(steer)) / WHEEL_BASE
        self.egoState.heading = (self.egoState.heading + math.degrees(yaw_rate * dt)) % 360.0
        
        heading_rad = math.radians(self.egoState.heading)
        self.egoState.x += self.egoState.speed * math.sin(heading_rad) * dt
        self.egoState.y += self.egoState.speed * math.cos(heading_rad) * dt # 注意：此处 y 是 GUI 坐标系的逻辑

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
                self.currentControl = control
                self.createBgVehicles() # 背景车作为障碍物
            return

        egoVehicle, bgVehicles = self.findBgVehicle(vehicles)

        if egoVehicle is None:
            return

        self.stepCount += 1

        # 构建以 Ego 为中心的观测
        obs = self.buildObs(egoVehicle, vehicles)
        # 计算 Ego 的奖励
        reward = self.computeEgoReward(egoVehicle)
        done = self.checkEgoDone(egoVehicle)

        self.episodeReward += reward
        control = self.env.onSimuStep(obs, reward, done)

        if control is None:
            self.episodeCount += 1
            print(f"[Episode {self.episodeCount}] Ego训练: 步数={self.stepCount}, 奖励={self.episodeReward:.2f}")
            self.episodeReward = 0.0
            self.isFirstStep = True
            return

        self.currentControl = control
        # 背景车在此模式下可以作为 NPC 运行
        self.applyBgActions((0.0, 0.0)) # 背景车恒速或静止

    def computeEgoReward(self, egoVehicle) -> float:
        """Ego 训练奖励函数：鼓励安全、高效、居中"""
        reward = 0.0
        
        # 1. 速度奖励：鼓励接近目标速度 (如 15m/s)
        target_v = 15.0
        v = egoVehicle.currSpeed()
        reward += 0.2 * (1.0 - abs(v - target_v) / target_v)
        
        # 2. 车道居中奖励
        laneResult = LaneProjector.fromTessngVehicle(egoVehicle, p2m)
        if laneResult:
            offset = abs(laneResult.lateral_offset)
            reward -= 0.1 * (offset / MAX_LANE_WIDTH)
            if offset > MAX_LANE_WIDTH / 2:
                reward -= 2.0 # 偏离车道惩罚
                self._is_out_of_bounds = True
        
        # 3. 碰撞惩罚
        # 这里复用原有的检测逻辑，但标记为 Ego 失败
        ego_id = egoVehicle.id()
        all_vehis = self.simIface.allVehiStarted()
        for v in all_vehis:
            if v.id() != ego_id:
                if self._check_bbox_collision_vehi(egoVehicle, v):
                    self._is_collision = True
                    return -10.0 # 碰撞重罚
        
        # 4. 生存奖励
        reward += 0.01
        
        return float(reward)

    def checkEgoDone(self, egoVehicle) -> bool:
        if getattr(self, "_is_collision", False):
            print("[Done] Ego 发生碰撞!")
            return True
        if getattr(self, "_is_out_of_bounds", False):
            print("[Done] Ego 偏离车道!")
            return True
        if self.stepCount >= TRAIN_MAX_STEPS:
            print("[Done] 达到最大步数")
            return True
        return False

    def _check_bbox_collision_vehi(self, v1, v2):
        p1 = v1.pos()
        p2 = v2.pos()
        return self._check_bbox_collision(
            p2m(p1.x()), p2m(p1.y()), v1.angle(), v1.length(), v1.width(),
            p2m(p2.x()), p2m(p2.y()), v2.angle(), v2.length(), v2.width()
        )

    # ============================================================
    #  推理模式
    # ============================================================

    def _afterOneStepInference(self, vehicles):
        if not getattr(self, "_inferCreated", False):
            # 初始化 Ego 状态
            init_x, init_y = self.egoSmoothedPath[0] if self.egoSmoothedPath else (0.0, 0.0)
            _, _, init_heading = self._posOnPath(self.egoSmoothedPath, 0.0)
            self.egoState = VehicleState(x=init_x, y=init_y, heading=init_heading, speed=10.0)
            
            vsMap = {self.egoName: self.egoState}
            # 初始化背景车状态
            for name, agent in self.bgAgents.items():
                agent["progress"] = 0.0
                x, y, heading = self._posOnPath(agent["smoothed"], 0.0)
                agent["x"], agent["y"], agent["heading"] = x, y, heading
                vsMap[name] = VehicleState(x=x, y=y, heading=heading, speed=agent["speed"])
                
            self.tessAuto.setAvChannel2AvMsgMap(vsMap)
            self.tessAuto.vehicleCreate()
            self._inferCreated = True
            self._inferStep = 0
            self._is_collision = False
            self.currentControl = (0.0, 0.0)
            self.prevSteer = 0.0
            self.yawRateCalc.reset()
            return

        egoVehicle, bgVehicles = self.findBgVehicle(vehicles)
        if egoVehicle is None:
            return

        # 1. 控制 Ego (使用训练好的模型)
        if hasattr(self, "egoModel") and self.egoModel is not None:
            obs = self.buildObs(egoVehicle, vehicles)
            action, _ = self.egoModel.predict(obs, deterministic=True)
            
            if RL_ALGO == "PPO":
                accel, steer = float(action[0]), float(action[1])
            else:
                accel, steer = ACTION_TO_CONTROL[int(action)]
                
            # 更新 Ego 状态
            dt = self.dt
            self.egoState.speed += accel * dt
            self.egoState.speed = max(0.0, min(self.egoState.speed, MAX_SPEED))
            
            yaw_rate = (self.egoState.speed * math.tan(steer)) / WHEEL_BASE
            self.egoState.heading = (self.egoState.heading + math.degrees(yaw_rate * dt)) % 360.0
            
            heading_rad = math.radians(self.egoState.heading)
            self.egoState.x += self.egoState.speed * math.sin(heading_rad) * dt
            self.egoState.y += self.egoState.speed * math.cos(heading_rad) * dt
            
            self.currentControl = (accel, steer)
            
        vsMap = {self.egoName: self.egoState}

        # 2. 控制背景车
        USE_MODEL_FOR_BG = False  # 当前训练主车阶段，先不接入模型控制背景车
        
        all_bg_finished = True
        if USE_MODEL_FOR_BG and getattr(self, "multiInfer", None) is not None:
            results = self.multiInfer.stepAll(vehicles, p2m, dt=self.dt)
            for name, s in results.items():
                vsMap[name] = VehicleState(x=s["x"], y=s["y"], heading=s["heading"], speed=s["speed"])
            all_bg_finished = self.multiInfer.allFinished
        else:
            for name, agent in self.bgAgents.items():
                if agent["progress"] < agent["totalLength"]:
                    all_bg_finished = False
                    agent["progress"] += agent["speed"] * self.dt
                    agent["progress"] = min(agent["progress"], agent["totalLength"])
                    
                x, y, heading = self._posOnPath(agent["smoothed"], agent["progress"])
                agent["x"] = x
                agent["y"] = y
                agent["heading"] = heading
                
                vsMap[name] = VehicleState(x=x, y=y, heading=heading, speed=agent["speed"])

        self.tessAuto.setAvChannel2AvMsgMap(vsMap)

        self._inferStep += 1
        if self._inferStep % 100 == 0:
            print(f"[推理 Step {self._inferStep}] Ego 推理中... 当前速度: {self.egoState.speed:.2f} m/s")

        # 碰撞检测
        ego_id = egoVehicle.id()
        for v in vehicles:
            if v.id() != ego_id:
                if self._check_bbox_collision_vehi(egoVehicle, v):
                    self._is_collision = True
                    break
        
        # 终止与重置判断
        if self._is_collision or self._inferStep >= TRAIN_MAX_STEPS or all_bg_finished:
            reason = "发生碰撞" if self._is_collision else ("达到最大步数" if self._inferStep >= TRAIN_MAX_STEPS else "背景车到达终点")
            print(f"[推理] {reason}，重置环境。")
            self._inferCreated = False
            if getattr(self, "multiInfer", None):
                self.multiInfer.resetAll()

    # ============================================================
    #  训练线程
    # ============================================================

    def runTraining(self):
        os.makedirs(MODEL_SAVE_DIR, exist_ok=True)
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
                    learning_rate=3e-4,
                    n_steps=2048,
                    batch_size=64,
                    n_epochs=10,
                    gamma=0.99,
                    gae_lambda=0.95,
                    clip_range=0.2,
                    ent_coef=0.01,
                    verbose=1,
                    tensorboard_log=f"{TENSORBOARD_LOG}/{datetime.now().strftime('%Y%m%d_%H%M%S')}"
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
                    tensorboard_log=TENSORBOARD_LOG
                )

            for i in range(numScenarios):
                print(f"\n[训练] === 场景 {i}: {self.scenarios[i]['file']} ===")
                self.switchScenario(i)
                model.learn(total_timesteps=stepsPerScenario, reset_num_timesteps=False)

            model.save(savePath)
            print(f"\n[训练] 完成! 模型: {savePath}")

        elif not TRAIN_MODE:
            print(f"[推理] 加载模型: {savePath}")
            try:
                if RL_ALGO == "PPO":
                    from stable_baselines3 import PPO
                    self.egoModel = PPO.load(savePath)
                else:
                    from stable_baselines3 import DQN
                    self.egoModel = DQN.load(savePath)
                print("[推理] Ego 模型加载成功。")
            except Exception as e:
                print(f"[推理] 模型加载失败: {e}")
                self.egoModel = None
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
                        f"{prefix}_{name}", info["path"], info["speed"], color=info["color"]
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
        self.initBgAgents()

    def switchScenario(self, idx):
        self.loadScenario(idx)

    def initBgAgents(self):
        """初始化背景车运行状态"""
        self.bgAgents = {}
        for name, info in self.currentVehicles.items():
            smoothed = MultiVehicleInference._smoothPath(info["path"], 1.0)
            totalLen = MultiVehicleInference._pathLength(smoothed)

            # 初始化起点坐标
            init_x, init_y = smoothed[0] if smoothed else (0.0, 0.0)

            self.bgAgents[name] = {
                "speed": info["speed"],
                "progress": 0.0,
                "smoothed": smoothed,
                "totalLength": totalLen,
                "prevHeading": 0.0,
                "x": init_x,
                "y": init_y,
            }

    # ============================================================
    #  背景车创建与控制
    # ============================================================

    def createBgVehicles(self):
        """在 Tessng 中创建背景车"""
        vsMap = {}
        for name, agent in self.bgAgents.items():
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

    # ============================================================
    #  环境回调
    # ============================================================

    def doReset(self):
        """重置环境状态"""
        self.yawRateCalc.reset()
        self.prevSteer = 0.0
        self.stepCount = 0
        self.egoState = None # 触发 _updateEgoByRL 中的初始位姿分配
        self._is_collision = False
        self._is_out_of_bounds = False
        
        for name, agent in self.bgAgents.items():
            info = self.currentVehicles.get(name, {})
            agent["speed"] = info.get("speed", 10.0)
            agent["progress"] = 0.0
            smoothed = agent["smoothed"]
            init_x, init_y = smoothed[0] if smoothed else (0.0, 0.0)
            agent["x"], agent["y"] = init_x, init_y
            
        self.isFirstStep = True
        print(f"\n[Reset] Ego 状态已重置，开始新的 Episode。")

    def buildInitialObs(self):
        return np.zeros(94, dtype=np.float32)

    # ============================================================
    #  观测（根据主体车辆计算）
    # ============================================================

    def buildObs(self, vehicle, vehicles) -> np.ndarray:
        obs = np.zeros(94, dtype=np.float32)
        
        # 判断是 Ego 还是背景车，选择对应的路径
        egoTessngId = self.tessAuto.avName2TessngIdMap.get(self.egoName)
        if vehicle.id() == egoTessngId:
            centerLine = self.egoSmoothedPath
            v_speed = self.egoState.speed if self.egoState else vehicle.currSpeed()
        else:
            avName = self.tessAuto.tessngId2AvNameMap.get(vehicle.id())
            agent = self.bgAgents.get(avName)
            centerLine = agent["smoothed"] if agent else []
            v_speed = agent["speed"] if agent else vehicle.currSpeed()

        # 1. 基础状态 (5维 + 运动学)
        laneResult = LaneProjector.fromTessngVehicle(vehicle, p2m)
        if laneResult:
            obs[0] = np.clip(laneResult.dist_left / MAX_LANE_WIDTH, 0, 1)
            obs[1] = np.clip(laneResult.dist_right / MAX_LANE_WIDTH, 0, 1)
            obs[2] = np.clip((laneResult.lateral_offset / MAX_LANE_WIDTH) + 0.5, 0, 1)
            obs[3] = laneResult.angle_diff

        obs[4] = np.clip(v_speed / MAX_SPEED, 0, 1)
        obs[5] = np.clip((self.currentControl[1] / (2 * MAX_STEER_ANGLE)) + 0.5, 0, 1)
        obs[6] = np.clip((self.currentControl[0] - MAX_DECEL) / (MAX_ACCEL - MAX_DECEL), 0, 1)
        obs[7] = np.clip((self.prevSteer / (2 * MAX_STEER_ANGLE)) + 0.5, 0, 1)
        self.prevSteer = self.currentControl[1]

        MAX_YAW_RATE = 1.0
        yawRate = self.yawRateCalc.update(vehicle.id(), vehicle.angle(), self.dt)
        obs[8] = np.clip((yawRate / (2 * MAX_YAW_RATE)) + 0.5, 0, 1)

        # 2. 导航 (13维)
        if centerLine and len(centerLine) >= 2:
            sparse = NavigationCalculator.sparsifyByDistance(centerLine, 5.0)
            vPos = vehicle.pos()
            navResult = NavigationCalculator.compute(
                p2m(vPos.x()), p2m(vPos.y()), vehicle.angle(),
                sparse, numCheckpoints=5,
            )
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

        return obs

    # ============================================================
    #  奖励（背景车视角，鼓励干扰主车）
    # ============================================================

    def computeReward(self, bgVehicle) -> float:
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

        real_x, real_y, expected_heading = self._posOnPath(agent["smoothed"], agent["progress"])

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
            "expected_heading": expected_heading
        }

    def _r_survival(self, s):
        return 0.05 if s["agent_speed"] > 0.5 else 0.0

    def _r_distance(self, s):
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
        
    def _r_stop(self,agent):
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
        accel, steer = self.currentControl

        r = 0.0

        # 转向惩罚（降低权重）
        r -= abs(steer) * 0.8

        # 抖动惩罚, 防止“画龙”和剧烈摆动
        prev = agent.get("prev_steer", 0.0)
        r -= abs(steer - prev) * 1.5 # 增加突变惩罚
        agent["prev_steer"] = steer

        return r

    def _r_lane(self, bgVehicle):
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

    # ============================================================
    #  终止判断
    # ============================================================

    def checkDone(self) -> bool:
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
    #  工具
    # ============================================================

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
                (cx - dx1 + dx2, cy - dy1 + dy2)
            ]

        def get_axes(corners):
            axes = []
            for i in range(2): # 矩形只需要相邻两条边的法向量
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
            min1, max1 = float('inf'), float('-inf')
            for p in corners1:
                proj = p[0] * axis[0] + p[1] * axis[1]
                min1, max1 = min(min1, proj), max(max1, proj)
                
            min2, max2 = float('inf'), float('-inf')
            for p in corners2:
                proj = p[0] * axis[0] + p[1] * axis[1]
                min2, max2 = min(min2, proj), max(max2, proj)
                
            if max1 < min2 or max2 < min1:
                return False # 找到分离轴，没有碰撞
                
        return True # 所有轴都有重叠，发生碰撞

    @staticmethod
    def _posOnPath(smoothed, dist):
        """
        在平滑路径上定位

        JSON 坐标约定：x 同 GUI，y = -GUI_y
        Tessng 航向角：正北0°顺时针，基于 GUI 坐标（y 向下）
        转换：heading = atan2(dx, dy_json)
          因为 dy_gui = -dy_json，所以 atan2(dx, -dy_gui) = atan2(dx, dy_json)
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
                heading = math.degrees(math.atan2(dx, dy)) % 360.0
                return x, y, heading
            accumulated += segLen
        bx, by = smoothed[-1]
        ax, ay = smoothed[-2]
        dx = bx - ax
        dy = by - ay
        heading = math.degrees(math.atan2(dx, dy)) % 360.0
        return bx, by, heading

