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

# 归一化常量
MAX_LANE_WIDTH = 4.0
MAX_SPEED = 33.3
MAX_STEER_ANGLE = 0.7
MAX_ACCEL = 5.0
MAX_DECEL = -5.0

# ===== 配置 =====
TRAIN_MODE = True
TOTAL_TIMESTEPS = 50000
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

        # 推理模式
        if self.multiInfer is not None:
            self._afterOneStepInference(vehicles)
            return

        # 训练模式
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

        # 找任意一辆背景车作为观测基准
        bgVehicle = self.findBgVehicle(vehicles)
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

        # 推理模式
        print("\n[推理] 初始化多车推理...")
        self.multiInfer = MultiVehicleInference(savePath, algo=RL_ALGO)
        for scenario in self.scenarios:
            prefix = scenario["file"].replace(".json", "")
            for name, info in scenario["vehicles"].items():
                self.multiInfer.addVehicle(
                    f"{prefix}_{name}", info["path"], info["speed"]
                )

        self._inferCreated = False

        while not self.env._closed:
            import time

            time.sleep(1.0)

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
            self.bgAgents[name] = {
                "speed": info["speed"],
                "progress": 0.0,
                "smoothed": smoothed,
                "totalLength": totalLen,
                "prevHeading": 0.0,
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
        vsMap = {}
        for name, agent in self.bgAgents.items():
            s = agent["smoothed"]
            if len(s) < 2:
                continue
            agent["speed"] += accel * self.dt
            agent["speed"] = max(0.0, min(agent["speed"], MAX_SPEED))
            agent["progress"] += agent["speed"] * self.dt
            agent["progress"] = min(agent["progress"], agent["totalLength"])

            x, y, heading = self._posOnPath(s, agent["progress"])
            agent["prevHeading"] = heading
            vsMap[name] = VehicleState(x=x, y=y, heading=heading, speed=agent["speed"])
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
            agent["prevHeading"] = 0.0
        self.isFirstStep = True
        
        # [优化] 在重置环境时，重新加载选手。确保 Ego 在每次 Episode 都重置进度和状态。
        if hasattr(self, 'playerManager'):
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
        reward = 0.0
        self._is_collision = False  # 记录是否发生碰撞
        
        bgName = list(self.bgAgents.keys())[0] if self.bgAgents else None
        if bgName is None or self.egoState is None:
            return reward

        agent = self.bgAgents[bgName]

        # 1. 基础存活奖励 (降低权重，避免一直苟活)
        reward += 0.05

        # 2. 速度与位置计算
        egoSpeed = self.egoState.speed
        speedDiff = agent["speed"] - egoSpeed
        
        bgX, bgY, _ = self._posOnPath(agent["smoothed"], agent["progress"])
        egoX = self.egoState.x
        egoY = -self.egoState.y

        egoHeadingRad = math.radians(90.0 - self.egoState.heading)
        dx = bgX - egoX
        dy = bgY - egoY
        forwardDist = dx * math.cos(egoHeadingRad) + dy * math.sin(egoHeadingRad)
        lateralDist = -dx * math.sin(egoHeadingRad) + dy * math.cos(egoHeadingRad)

        distToEgo = math.sqrt(dx**2 + dy**2)

        # 3. 碰撞与相对距离奖励 (核心干扰逻辑升级)
        if distToEgo < 2.5:
            # 极度危险距离，考虑到车辆长宽，中心距小于2.5米基本发生物理碰撞
            reward += 50.0  # 成功造成碰撞，给予极大奖励（目标达成）
            self._is_collision = True
            return reward   # 发生碰撞后直接返回，不计入后续的惩罚

        elif distToEgo < 10.0:
            # 危险区域：非常逼近
            reward += 1.0 * (1.0 - distToEgo / 10.0)
        elif distToEgo < 25.0:
            # 潜在威胁区域
            reward += 0.3 * (1.0 - distToEgo / 25.0)

        # 4. 速度控制奖励
        if forwardDist < 0:
            # 背景车在主车后方：鼓励比主车快
            if speedDiff > 0:
                reward += min(speedDiff / 10.0, 0.2)
        else:
            # 背景车在主车前方：鼓励比主车慢 (压车)
            if speedDiff < 0:
                reward += min(abs(speedDiff) / 10.0, 0.2)

        # 5. 变道插车/卡位奖励 (利用横向偏差)
        # 如果距离主车较近且横向距离较小，说明正在阻挡主车路径
        if 0 < forwardDist < 15.0 and abs(lateralDist) < 2.0:
            # 正前方阻挡
            reward += 0.5
            # 如果主车速度大于背景车速度，说明成功压制了主车
            if egoSpeed > agent["speed"] + 1.0:
                reward += 0.5

        # 6. 侧向逼近奖励 (鼓励从侧面挤压)
        if -5.0 < forwardDist < 5.0 and 1.5 < abs(lateralDist) < 4.0:
            # 与主车并行，鼓励缩小横向距离
            reward += 0.2 * (4.0 - abs(lateralDist))

        # 7. 惩罚项
        # 停车惩罚 (避免原地发呆，除非距离主车很远)
        if agent["speed"] < 0.5 and distToEgo > 10.0:
            reward -= 0.5

        # 剧烈画龙惩罚 (防止无意义的蛇形走位)
        hDiff = (
            abs(agent["heading"] - agent["prevHeading"]) if "heading" in agent else 0
        )
        if hDiff > 180:
            hDiff = 360 - hDiff
        if hDiff > 3.0:
            reward -= min(hDiff / 15.0, 0.5)

        return reward

    # ============================================================
    #  终止判断
    # ============================================================

    def checkDone(self) -> bool:
        # 1. 成功发生碰撞，完成干扰目标，立即终止回合
        if getattr(self, '_is_collision', False):
            print(f"[Done] 背景车成功与主车发生碰撞! 终止当前 Episode.")
            return True

        if self.stepCount >= 500:
            return True
        bgName = list(self.bgAgents.keys())[0] if self.bgAgents else None
        if bgName:
            agent = self.bgAgents[bgName]
            if agent["progress"] >= agent["totalLength"]:
                return True
            if agent["speed"] < 0.1:
                self._zc = getattr(self, "_zc", 0) + 1
                if self._zc > 10:
                    self._zc = 0
                    return True
            else:
                self._zc = 0
        return False

    # ============================================================
    #  工具
    # ============================================================

    def findBgVehicle(self, vehicles):
        """找到第一辆背景车（不是主车的 Tessng 车辆）"""
        egoTessngId = self.tessAuto.avName2TessngIdMap.get(self.egoName)
        for v in vehicles:
            vid = v.id()
            if vid in self.tessAuto.alreadyLaunchedTessngIdSet and vid != egoTessngId:
                return v
        return None

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
