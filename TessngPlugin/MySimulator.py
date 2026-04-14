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
from MultiVehicleInference import MultiVehicleInference,WHEEL_BASE

# 归一化常量
MAX_LANE_WIDTH = 4.0
MAX_SPEED = 33.3
MAX_STEER_ANGLE = 0.7
MAX_ACCEL = 7.0
MAX_DECEL = -7.0

# ===== 配置 =====
TRAIN_MODE = True
TOTAL_TIMESTEPS = 100000
DATA_DIR = "Data"

# 选择强化学习算法: "DQN" 或 "PPO"
RL_ALGO = "PPO"
MODEL_SAVE_DIR = "tessng_" + RL_ALGO.lower()


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

        # ===== 主车：由选手的 TestPlayer 控制 =====
        self.playerManager = PlayerManager()
        self.egoName = "ego"

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
    #  主车更新（每帧调用选手的 act）
    # ============================================================

    def updateEgo(self):
        """调用选手的 TestPlayer.act()，获取主车状态并更新到 Tessng"""
        states = self.playerManager.step_all()
        self.tessAuto.setAvChannel2AvMsgMap(states)
        self.tessAuto.vehicleCreate()

        # 缓存主车状态（用于背景车的观测和奖励计算）
        for name, state in states.items():
            self.egoState = state
            self.egoName = name
            break

    # ============================================================
    #  训练模式
    # ============================================================

    def _afterOneStepTraining(self, vehicles):
        if self.isFirstStep:
            control = self.env.onFirstStep()
            self.isFirstStep = False
            if control is not None:
                self.currentControl = control
                self.createBgVehicles()
            return

        # 找主车和所有背景车
        egoVehicle, bgVehicles = self.findBgVehicle(vehicles)
        # 找任意一辆背景车作为观测基准
        bgVehicle = bgVehicles[0] if bgVehicles else None
        if bgVehicle is None:
            return

        self.stepCount += 1

        if self.stepCount == 1:
            obs = np.zeros(94, dtype=np.float32)
            control = self.env.onSimuStep(obs, 0.0, False)
            if control is not None:
                self.currentControl = control
                self.applyBgActions(control)
            return

        obs = self.buildObs(bgVehicle, vehicles)
        reward = self.computeReward()
        done = self.checkDone()

        self.episodeReward += reward
        control = self.env.onSimuStep(obs, reward, done)

        if control is None:
            self.episodeCount += 1
            bgName = list(self.bgAgents.keys())[0] if self.bgAgents else "?"
            agent = self.bgAgents.get(bgName, {})
            print(
                f"[Episode {self.episodeCount}] "
                f"场景={self.scenarios[self.currentScenarioIdx]['file']} "
                f"步数={self.stepCount} "
                f"距离={agent.get('progress', 0):.1f}m "
                f"奖励={self.episodeReward:.2f}"
            )
            self.episodeReward = 0.0
            self.isFirstStep = True
            return

        self.currentControl = control
        self.applyBgActions(control)

    # ============================================================
    #  推理模式
    # ============================================================

    def _afterOneStepInference(self, vehicles):
        if not getattr(self, "_inferCreated", False):
            initStates = self.multiInfer.getInitialStates()
            vsMap = {
                name: VehicleState(
                    x=s["x"], y=s["y"], heading=s["heading"], speed=s["speed"]
                )
                for name, s in initStates.items()
            }
            self.tessAuto.setAvChannel2AvMsgMap(vsMap)
            self.tessAuto.vehicleCreate()
            self._inferCreated = True
            self._inferStep = 0
            return

        results = self.multiInfer.stepAll(vehicles, p2m, dt=self.dt)
        vsMap = {
            name: VehicleState(
                x=s["x"], y=s["y"], heading=s["heading"], speed=s["speed"]
            )
            for name, s in results.items()
        }
        self.tessAuto.setAvChannel2AvMsgMap(vsMap)

        self._inferStep += 1
        if self._inferStep % 100 == 0:
            print(
                f"[推理 Step {self._inferStep}] "
                f"背景车: {self.multiInfer.aliveCount}/{len(self.multiInfer.agents)}"
            )

        if self.multiInfer.allFinished:
            print("[推理] 所有背景车到达终点，重置")
            self.multiInfer.resetAll()
            self._inferCreated = False
            
            # [修复] 在推理模式下重置时，也要重置主车(Ego)的状态
            if hasattr(self, "playerManager"):
                self.playerManager.load_all()

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
            print(f"[训练] 每个场景 {stepsPerScenario} 步")
            print(f"[训练] 主车由选手 TestPlayer 控制")
            print(f"[训练] 背景车由 {RL_ALGO} 控制，目标：干扰主车")
            for i, s in enumerate(self.scenarios):
                print(f"  [{i}] {s['file']}: {list(s['vehicles'].keys())}")
            print("=" * 60)

            if RL_ALGO == "PPO":
                from stable_baselines3 import PPO

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
                    ent_coef=0.0,
                    verbose=1,
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
                )

            for i in range(numScenarios):
                print(f"\n[训练] === 场景 {i}: {self.scenarios[i]['file']} ===")
                self.switchScenario(i)
                model.learn(total_timesteps=stepsPerScenario, reset_num_timesteps=False)

            model.save(savePath)
            print(f"\n[训练] 完成! 模型: {savePath}")

        elif not TRAIN_MODE:
            print(f"[推理] 加载模型: {savePath}")
        else:
            print("[训练] Data 目录无 JSON 文件")
            return
        
        # 切换到推理模式前，关闭 env 解除可能的阻塞
        self.env.close()

        # 等一小会让 afterOneStep 从阻塞中退出
        import time
        time.sleep(0.5)

        # 推理模式
        print("\n[推理] 初始化多车推理...")
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
        """所有背景车使用同一个 action 推进"""
        accel, steer = control
        
        # 调试输出当前动作，排查 steer 极值问题
        # print(f"[Debug] Training Action: accel={accel:.2f}, steer={steer:.2f}")

        vsMap = {}
        for name, agent in self.bgAgents.items():
            s = agent["smoothed"]
            if len(s) < 2:
                continue
            agent["speed"] += accel * self.dt
            agent["speed"] = max(0.0, min(agent["speed"], MAX_SPEED))
            
            # 引入车辆运动学模型 (Kinematic Bicycle Model，自动驾驶中用于模拟四轮小汽车的经典单辙模型)
            # 假设小汽车轴距为 2.8 米
            yaw_rate = (agent["speed"] * math.tan(steer)) / WHEEL_BASE
            
            agent["prevHeading"] = agent.get("heading", agent["prevHeading"])
            agent["heading"] = (agent["prevHeading"] + math.degrees(yaw_rate * self.dt)) % 360.0
            
            heading_rad = math.radians(agent["heading"])
            # Tessng GUI坐标：dx = sin(heading), dy = cos(heading)
            agent["x"] += agent["speed"] * math.sin(heading_rad) * self.dt
            agent["y"] += agent["speed"] * math.cos(heading_rad) * self.dt
            
            agent["progress"] += agent["speed"] * self.dt
            agent["progress"] = min(agent["progress"], agent["totalLength"])

            vsMap[name] = VehicleState(x=agent["x"], y=agent["y"], heading=agent["heading"], speed=agent["speed"])
        self.tessAuto.setAvChannel2AvMsgMap(vsMap)

    # ============================================================
    #  环境回调
    # ============================================================

    def doReset(self):
        self.yawRateCalc.reset()
        self.prevSteer = 0.0
        self.stepCount = 0
        for name, agent in self.bgAgents.items():
            info = self.currentVehicles.get(name, {})
            agent["speed"] = info.get("speed", 10.0)
            agent["progress"] = 0.0
            
            # 重新获取起点坐标和航向
            smoothed = agent["smoothed"]
            init_x, init_y = smoothed[0] if smoothed else (0.0, 0.0)
            if len(smoothed) >= 2:
                dx = smoothed[1][0] - smoothed[0][0]
                dy = smoothed[1][1] - smoothed[0][1]
                init_heading = math.degrees(math.atan2(dx, dy)) % 360.0
            else:
                init_heading = 0.0
                
            agent["prevHeading"] = init_heading
            agent["heading"] = init_heading
            agent["x"] = init_x
            agent["y"] = init_y
        self.isFirstStep = True

        # [优化] 在重置环境时，重新加载选手。确保 Ego 在每次 Episode 都重置进度和状态。
        if hasattr(self, "playerManager"):
            self.playerManager.load_all()

    def buildInitialObs(self):
        return np.zeros(94, dtype=np.float32)

    # ============================================================
    #  观测（背景车视角，包含主车信息）
    # ============================================================

    def buildObs(self, bgVehicle, vehicles) -> np.ndarray:
        obs = np.zeros(94, dtype=np.float32)
        bgName = list(self.bgAgents.keys())[0] if self.bgAgents else None
        if bgName is None:
            return obs
        agent = self.bgAgents[bgName]

        # 背景车自身状态
        laneResult = LaneProjector.fromTessngVehicle(bgVehicle, p2m)
        if laneResult:
            obs[0] = np.clip(laneResult.dist_left / MAX_LANE_WIDTH, 0, 1)
            obs[1] = np.clip(laneResult.dist_right / MAX_LANE_WIDTH, 0, 1)
            obs[2] = np.clip(laneResult.lateral_offset / MAX_LANE_WIDTH + 0.5, 0, 1)
            obs[3] = laneResult.angle_diff

        obs[4] = np.clip(agent["speed"] / MAX_SPEED, 0, 1)
        obs[5] = np.clip(self.currentControl[1] / (2 * MAX_STEER_ANGLE) + 0.5, 0, 1)
        obs[6] = np.clip(
            (self.currentControl[0] - MAX_DECEL) / (MAX_ACCEL - MAX_DECEL), 0, 1
        )
        obs[7] = np.clip(self.prevSteer / (2 * MAX_STEER_ANGLE) + 0.5, 0, 1)
        self.prevSteer = self.currentControl[1]
        yawRate = self.yawRateCalc.update(bgVehicle.id(), bgVehicle.angle(), self.dt)
        obs[8] = self.yawRateCalc.normalize(yawRate)

        # 导航
        centerLine = agent["smoothed"]
        if centerLine and len(centerLine) >= 2:
            sparse = NavigationCalculator.sparsifyByDistance(centerLine, 5.0)
            egoPos = bgVehicle.pos()
            navResult = NavigationCalculator.compute(
                p2m(egoPos.x()),
                p2m(egoPos.y()),
                bgVehicle.angle(),
                sparse,
                numCheckpoints=5,
            )
            for i in range(min(5, len(navResult.forwardGaps))):
                obs[9 + i * 2] = navResult.forwardGaps[i]
                obs[9 + i * 2 + 1] = navResult.lateralGaps[i]
            obs[19] = navResult.curvatureRadius
            obs[20] = navResult.curvatureDirection
            obs[21] = navResult.laneAngleDiff

        # 雷达（包含主车和其他 Tessng 车辆）
        surroundResult = SurroundingCalculator.fromTessngVehicles(
            bgVehicle, vehicles, p2m, maxNearby=8
        )
        for i, r in enumerate(surroundResult.radar):
            obs[22 + i] = r

        return obs

    # ============================================================
    #  奖励（背景车视角，鼓励干扰主车）
    # ============================================================

    def computeReward(self) -> float:
        """
        计算奖励：遍历所有背景车，取其中的最高奖励作为当前步的反馈。
        这鼓励至少有一辆背景车能成功执行干扰任务。
        """
        if not self.bgAgents or self.egoState is None:
            return 0.0

        self._is_collision = False
        self._is_out_of_bounds = False
        agent_rewards = []

        egoX = self.egoState.x
        egoY = self.egoState.y
        egoSpeed = self.egoState.speed
        egoHeadingRad = math.radians(90.0 - self.egoState.heading)

        for name, agent in self.bgAgents.items():
            reward = 0.0

            # 1. 基础存活奖励
            reward += 0.05

            # 2. 相对位置与速度计算
            bgX, bgY, bgHeading = self._posOnPath(agent["smoothed"], agent["progress"])
            dx = bgX - egoX
            dy = bgY - egoY
            distToEgo = math.sqrt(
                (p2m(bgX) - p2m(egoX)) ** 2 + (p2m(bgY) - p2m(egoY)) ** 2
            )
            debug_msg = "主车位置: ({:.2f}, {:.2f}) ({:.2f}, {:.2f}), 背景车位置: ({:.2f}, {:.2f}) ({:.2f}, {:.2f}), 距离: {:.2f}m".format(
                egoX,
                egoY,
                p2m(egoX),
                p2m(egoY),
                bgX,
                bgY,
                p2m(bgX),
                p2m(bgY),
                distToEgo,
            )
            # if name == "car_2":
            #     print(debug_msg)

            # 投影到主车坐标系
            forwardDist = dx * math.cos(egoHeadingRad) + dy * math.sin(egoHeadingRad)
            lateralDist = -dx * math.sin(egoHeadingRad) + dy * math.cos(egoHeadingRad)
            speedDiff = agent["speed"] - egoSpeed

            # 3. 碰撞判定 (核心成功条件: Bounding Box 碰撞检测)
            # 假设车辆标准尺寸：长 4.8m，宽 2.0m
            is_collide = self._check_bbox_collision(
                bgX, bgY, bgHeading, 4.8, 2.0,
                egoX, egoY, self.egoState.heading, 4.8, 2.0
            )
            
            if is_collide:
                reward += 15.0  # 调低碰撞奖励，避免过拟合于单纯的碰撞而忽略过程
                self._is_collision = True
                agent_rewards.append(reward)
                continue

            # 4. 距离诱导奖励
            if distToEgo < 10.0:
                reward += 1.0 * (1.0 - distToEgo / 10.0)
            elif distToEgo < 25.0:
                reward += 0.3 * (1.0 - distToEgo / 25.0)

            # 5. 速度与位置配合奖励
            if forwardDist < 0:
                # 背景车在主车后方：追赶
                if speedDiff > 0:
                    reward += min(speedDiff / 10.0, 0.2)
                else:
                    # 在后面还比主车慢，加大惩罚
                    reward -= min(abs(speedDiff) / 5.0, 0.5)
            else:
                # 背景车在主车前方：阻挡/压车
                if speedDiff < 0:
                    # 背景车速度比主车慢，这是期望的（压车）
                    reward += min(abs(speedDiff) / 10.0, 0.3)
                else:
                    # 背景车在主车前面，且速度比主车快（逃跑），应该被惩罚
                    reward -= min(speedDiff / 5.0, 0.5)

            # 6. 卡位奖励
            if 0 < forwardDist < 15.0 and abs(lateralDist) < 2.0:
                reward += 0.5
                if egoSpeed > agent["speed"] + 1.0:
                    reward += 0.5

            # 7. 侧向挤压
            if -5.0 < forwardDist < 5.0 and 1.5 < abs(lateralDist) < 4.0:
                reward += 0.2 * (4.0 - abs(lateralDist))

            # 8. 惩罚项增强
            # 8.1 停车惩罚 (避免原地发呆，除非距离主车很远)
            if agent["speed"] < 0.5 and distToEgo > 10.0:
                reward -= 0.5
                
            # 8.2 远离主车惩罚 (如果距离太远且还在变远)
            if distToEgo > 30.0:
                reward -= 0.2  # 距离太远本身就是一个小惩罚
                if speedDiff < 0 and forwardDist > 0:
                    # 主车在后面，但背景车跑得比主车还快，导致距离拉大
                    reward -= 0.5
                elif speedDiff > 0 and forwardDist < 0:
                    # 主车在前面，但背景车跑得比主车慢，导致距离拉大
                    reward -= 0.5

            # 8.3 剧烈画龙惩罚 (防止无意义的蛇形走位)
            hDiff = abs(agent.get("heading", 0) - agent.get("prevHeading", 0))
            if hDiff > 180: hDiff = 360 - hDiff
            if hDiff > 3.0: 
                reward -= min(hDiff / 10.0, 1.0) # 增大画龙惩罚力度

            # 8.4 偏离预设路径过远惩罚 (如果偏离太多说明动作失控)
            # 真实位置与期望路径点(bgX, bgY)的距离即为偏差
            actual_x = agent.get("x", bgX)
            actual_y = agent.get("y", bgY)
            path_deviation = math.hypot(actual_x - bgX, actual_y - bgY)
            
            if path_deviation > 0.5:
                # 稍微偏离中心线，施加线性惩罚
                reward -= path_deviation * 0.5
            if path_deviation > 2.0:
                # 严重偏离（如冲出车道），施加致命重罚并结束
                reward -= 50.0
                self._is_out_of_bounds = True
                agent_rewards.append(reward)
                continue

            agent_rewards.append(reward)

        # 返回所有背景车中表现最好的那一辆的奖励
        # print(f"agent_rewards: {agent_rewards}")
        return max(agent_rewards) if agent_rewards else 0.0

    # ============================================================
    #  终止判断
    # ============================================================

    def checkDone(self) -> bool:
        # 1. 任意一辆背景车发生碰撞，干扰成功
        if getattr(self, "_is_collision", False):
            print(f"[Done] 发生碰撞! 干扰任务圆满完成。")
            return True
            
        # 1.5 任意一辆背景车严重偏离车道，干扰失败
        if getattr(self, "_is_out_of_bounds", False):
            print(f"[Done] 严重偏离车道! 干扰失败。")
            return True

        # 2. 达到最大步数
        if self.stepCount >= 500:
            return True

        # 3. 检查所有背景车的状态
        all_finished = True
        any_stuck = False

        for name, agent in self.bgAgents.items():
            # 只要有一辆车还没跑完，就不算全部结束
            if agent["progress"] < agent["totalLength"]:
                all_finished = False

            # 检查是否卡死 (由于是多车，只要有一辆车彻底卡死，可能场景就失效了)
            if agent["speed"] < 0.1:
                agent["_stuck_count"] = agent.get("_stuck_count", 0) + 1
                if agent["_stuck_count"] > 15:
                    any_stuck = True
            else:
                agent["_stuck_count"] = 0

        # 如果所有车都跑完了，或者有车卡死了，终止 Episode
        if all_finished:
            print("[Done] 所有背景车均到达终点。")
            return True
        if any_stuck:
            print("[Done] 探测到背景车卡死，重置。")
            return True

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
        # print(f"bbox: {corners1} vs {corners2}")
        
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
