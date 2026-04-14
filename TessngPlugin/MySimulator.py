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
TOTAL_TIMESTEPS = 500000
DATA_DIR = "Data"

# 选择强化学习算法: "DQN" 或 "PPO"
RL_ALGO = "PPO"
MODEL_SAVE_DIR = "tessng_" + RL_ALGO.lower()
TENSORBOARD_LOG = "./tensorboard_logs/"


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
        
        # 找到指定的“主攻手”作为观测基准
        bgVehicle = None
        for v in bgVehicles:
            avName = self.tessAuto.tessngId2AvNameMap.get(v.id())
            if avName == getattr(self, "_attacker_name", None):
                bgVehicle = v
                break
                
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
        reward = self.computeReward(bgVehicle)
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
                    tensorboard_log=TENSORBOARD_LOG
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
        """所有背景车根据角色(主攻手/NPC)执行动作推进"""
        accel, steer = control
        
        # 调试输出当前动作，监控 PPO 输出的加速度和方向盘转角
        if getattr(self, "_attacker_name", None):
            print(f"[Action] {self._attacker_name} -> accel: {accel:.2f}, steer: {steer:.2f}")

        vsMap = {}
        for name, agent in self.bgAgents.items():
            s = agent["smoothed"]
            if len(s) < 2:
                continue
                
            if name == getattr(self, "_attacker_name", None):
                # ===== 主攻手：由强化学习完全接管 =====
                agent["speed"] += accel * self.dt
                agent["speed"] = max(0.0, min(agent["speed"], MAX_SPEED))
                
                # 引入车辆运动学模型 (Kinematic Bicycle Model，自动驾驶中用于模拟四轮小汽车的经典单辙模型)
                # 假设小汽车轴距为 2.8 米
                wheelbase = getattr(self, "WHEEL_BASE", 2.8)
                yaw_rate = (agent["speed"] * math.tan(steer)) / wheelbase
                
                agent["prevHeading"] = agent.get("heading", agent["prevHeading"])
                agent["heading"] = (agent["prevHeading"] + math.degrees(yaw_rate * self.dt)) % 360.0
                
                heading_rad = math.radians(agent["heading"])
                # Tessng GUI坐标：dx = sin(heading), dy = cos(heading)
                agent["x"] += agent["speed"] * math.sin(heading_rad) * self.dt
                agent["y"] += agent["speed"] * math.cos(heading_rad) * self.dt
                
                agent["progress"] += agent["speed"] * self.dt
                agent["progress"] = min(agent["progress"], agent["totalLength"])
            else:
                # ===== NPC：恒定速度巡航，严格跟随路径 =====
                agent["progress"] += agent["speed"] * self.dt
                agent["progress"] = min(agent["progress"], agent["totalLength"])

                x, y, heading = self._posOnPath(s, agent["progress"])
                agent["x"] = x
                agent["y"] = y
                agent["heading"] = heading
                agent["prevHeading"] = heading

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

        # [新增] 随机选择一辆背景车作为本回合的“主攻手”
        import random
        if self.bgAgents:
            self._attacker_name = random.choice(list(self.bgAgents.keys()))
        else:
            self._attacker_name = None
        print(f"\n[Reset] 随机分配主攻手: {self._attacker_name}")

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
        attacker_name = getattr(self, "_attacker_name", None)
        if attacker_name is None or attacker_name not in self.bgAgents:
            return obs
        agent = self.bgAgents[attacker_name]

        # 背景车自身状态
        laneResult = LaneProjector.fromTessngVehicle(bgVehicle, p2m)
        if laneResult:
            obs[0] = np.clip(laneResult.dist_left / MAX_LANE_WIDTH, 0, 1)
            obs[1] = np.clip(laneResult.dist_right / MAX_LANE_WIDTH, 0, 1)
            # 车道偏移 [-L_width/2, L_width/2] -> [0, 1]，中心为 0.5
            obs[2] = np.clip((laneResult.lateral_offset / MAX_LANE_WIDTH) + 0.5, 0, 1)
            # 角度差归一化 [0, 1]，0.5代表完全平行
            obs[3] = laneResult.angle_diff

        obs[4] = np.clip(agent["speed"] / MAX_SPEED, 0, 1)
        # 转向角: [-MAX_STEER_ANGLE, MAX_STEER_ANGLE] -> [0, 1]，0.5 是回正
        obs[5] = np.clip((self.currentControl[1] / (2 * MAX_STEER_ANGLE)) + 0.5, 0, 1)

        # 加速度：[MAX_DECEL, MAX_ACCEL] -> [0, 1]
        obs[6] = np.clip(
            (self.currentControl[0] - MAX_DECEL) / (MAX_ACCEL - MAX_DECEL), 0, 1
        )

        obs[7] = np.clip((self.prevSteer / (2 * MAX_STEER_ANGLE)) + 0.5, 0, 1)
        self.prevSteer = self.currentControl[1]

        # 角速度映射 [-MAX_YAW_RATE, MAX_YAW_RATE] -> [0, 1]
        MAX_YAW_RATE = 1.0
        yawRate = self.yawRateCalc.update(bgVehicle.id(), bgVehicle.angle(), self.dt)
        obs[8] = np.clip((yawRate / (2 * MAX_YAW_RATE)) + 0.5, 0, 1)

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

    def computeReward(self, bgVehicle) -> float:
        """
        归一化后的奖励函数：目标区间控制在 [-1, 1] 左右
        """
        attacker_name = getattr(self, "_attacker_name", None)
        if (
            not self.bgAgents
            or self.egoState is None
            or attacker_name not in self.bgAgents
        ):
            return 0.0

        # 1. 基础初始化
        agent = self.bgAgents[attacker_name]
        self._is_collision = False
        self._is_out_of_bounds = False
        self._is_wrong_way = False

        # 2. 预计算物理状态
        s = self._prepare_state_info(agent)

        # 3. 基础与进度奖励 (归一化：max ~ 0.05)
        reward = self._reward_basic_and_progress(agent)

        # 4. 碰撞判定 (核心成功条件：截断至 +1.0)
        is_collide = self._check_bbox_collision(
            s["actual_x"], s["actual_y"], s["actual_heading"], 4.8, 2.0,
            s["ego_x"], s["ego_y"], s["ego_heading"], 4.8, 2.0,
        )
        if is_collide:
            self._is_collision = True
            return 1.0 

        # 5. 交互奖励 (归一化：max ~ 0.3)
        reward += self._reward_interaction(agent, s)

        # 6. 控制平滑奖励 (包含非线性转向惩罚，防止转圈)
        reward += self._reward_control_smoothness(agent)

        # 7. 越界检测 (截断至 -1.0)
        lane_penalty, out_of_bounds = self._reward_lane_boundary(agent, bgVehicle)
        reward += lane_penalty
        if out_of_bounds:
            self._is_out_of_bounds = True
            return -1.0 

        # 8. 逆行检测 (截断至 -1.0)
        wrong_way_penalty, is_wrong = self._reward_direction(agent, s)
        reward += wrong_way_penalty
        if is_wrong:
            self._is_wrong_way = True
            return -1.0

        # 9. 状态惩罚
        reward += self._reward_static_penalties(s)

        return float(np.clip(reward, -1.0, 1.0))

    # --- 逻辑拆解子函数 ---

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
            "agent_speed":agent["speed"]
        }

    def _reward_basic_and_progress(self, agent) -> float:
        """基础活跃奖励与进度奖励"""
        r = 0.005 
        progressThisStep = agent["speed"] * self.dt
        r += min(progressThisStep / 20.0, 0.04) 
        return r

    def _reward_interaction(self, agent, s) -> float:
        """交互引导奖励 (归一化)"""
        r = 0.0
        dist = s["distToEgo"]

        # 距离诱导
        if dist < 10.0:
            r += 0.1 * (1.0 - dist / 10.0)
        elif dist < 25.0:
            r += 0.03 * (1.0 - dist / 25.0)

        # 速度与位置配合
        if s["forwardDist"] < 0:  
            r += (min(s["speedDiff"] / 100.0, 0.02) if s["speedDiff"] > 0 
                  else -min(abs(s["speedDiff"]) / 50.0, 0.05))
        else: 
            r += (min(abs(s["speedDiff"]) / 100.0, 0.03) if s["speedDiff"] < 0 
                  else -min(s["speedDiff"] / 50.0, 0.05))

        # 卡位与挤压
        if 0 < s["forwardDist"] < 15.0 and abs(s["lateralDist"]) < 2.0:
            r += 0.05 + (0.05 if s["ego_speed"] > agent["speed"] + 1.0 else 0)
        
        # 追踪瞄准
        dx_ego, dy_ego = s["ego_x"] - s["actual_x"], s["ego_y"] - s["actual_y"]
        angle_to_ego = math.degrees(math.atan2(dx_ego, dy_ego)) % 360.0
        h_err = abs(s["actual_heading"] - angle_to_ego)
        if h_err > 180: h_err = 360 - h_err

        attHeadingRad = math.radians(90.0 - s["actual_heading"])
        att_forwardDist = dx_ego * math.cos(attHeadingRad) + dy_ego * math.sin(attHeadingRad)
        if dist < 30.0 and att_forwardDist > 0:
            r += (1.0 - h_err / 45.0) * 0.05
        return r

    def _reward_control_smoothness(self, agent) -> float:
        """
        控制平滑与转向约束：
        通过平方惩罚 (steer**2) 解决持续满打方向盘转圈的问题。
        """
        r = 0.0
        accel, steer = self.currentControl
        
        # 1. 绝对转向惩罚：使用平方项，让大转角（如0.7）的惩罚远高于小转角
        r -= (steer ** 2) * 0.2
        
        # 2. 转向变化率惩罚：防止高频抖动 (steer_diff)
        prev_steer = agent.get("prev_steer", 0.0)
        steer_diff = abs(steer - prev_steer)
        if steer_diff > 0.02:
            r -= steer_diff * 0.5
        agent["prev_steer"] = steer

        # 3. 航向变化惩罚 (画龙)
        hDiff = abs(agent.get("heading", 0) - agent.get("prevHeading", 0))
        if hDiff > 180: hDiff = 360 - hDiff
        if hDiff > 3.0:
            r -= min(hDiff / 50.0, 0.1)

        return r

    def _reward_lane_boundary(self, agent, bgVehicle) -> (float, bool):
        """路径约束 (归一化)"""
        laneResult = LaneProjector.fromTessngVehicle(bgVehicle, p2m)
        if not laneResult:
            return -1.0, True

        offsetAbs = abs(laneResult.lateral_offset)
        if offsetAbs > 2.5:
            return -1.0, True

        return (-offsetAbs * 0.05) if offsetAbs > 0.5 else 0.0, False

    def _reward_direction(self, agent, s) -> (float, bool):
        """逆行约束 (归一化)"""
        _, _, expectedHeading = self._posOnPath(agent["smoothed"], agent["progress"])
        path_heading_err = abs(s["actual_heading"] - expectedHeading)
        if path_heading_err > 180: path_heading_err = 360 - path_heading_err

        if path_heading_err > 90.0:
            return -1.0, True
        elif path_heading_err > 45.0:
            return -(path_heading_err - 45.0) / 45.0 * 0.2, False
        return 0.0, False

    def _reward_static_penalties(self, s) -> float:
        """
        静态与远离惩罚 (归一化): 
        防止背景车在离主车较远时停止参与博弈，或无意义地远离主车。
        """
        r = 0.0
        # 1. 停车惩罚：离主车 10m 开外且处于静止状态
        if s["agent_speed"] < 0.5 and s["distToEgo"] > 10.0:
            r -= 0.05
            
        # 2. 远离主车惩罚 (如果距离超过 30m)
        if s["distToEgo"] > 30.0:
            r -= 0.02  # 距离过远基础惩罚
            
            # 动态检查：如果速度差导致距离进一步拉大
            if s["speedDiff"] > 0 and s["forwardDist"] > 0:
                # 背景车在前面且跑得更快 -> 越拉越远
                r -= 0.05
            elif s["speedDiff"] < 0 and s["forwardDist"] < 0:
                # 背景车在后面且跑得更慢 -> 越掉越远
                r -= 0.05
        return r

    # ============================================================
    #  终止判断
    # ============================================================

    def checkDone(self) -> bool:
        # 1. 发生碰撞，干扰成功
        if getattr(self, "_is_collision", False):
            print("[Done] 主攻手发生碰撞! 干扰任务圆满完成。")
            return True
            
        # 1.5 严重偏离车道，干扰失败
        if getattr(self, "_is_out_of_bounds", False):
            print("[Done] 主攻手严重偏离车道! 干扰失败。")
            return True

        # 1.6 逆行或掉头，干扰失败
        if getattr(self, "_is_wrong_way", False):
            print("[Done] 主攻手逆行或掉头! 干扰失败。")
            return True

        # 2. 达到最大步数
        if self.stepCount >= 500:
            return True

        # 3. 检查主攻手状态
        attacker_name = getattr(self, "_attacker_name", None)
        if attacker_name and attacker_name in self.bgAgents:
            agent = self.bgAgents[attacker_name]
            if agent["progress"] >= agent["totalLength"]:
                print("[Done] 主攻手到达终点。")
                return True

            if agent["speed"] < 0.1:
                agent["_stuck_count"] = agent.get("_stuck_count", 0) + 1
                if agent["_stuck_count"] > 15:
                    print("[Done] 探测到主攻手卡死，重置。")
                    return True
            else:
                agent["_stuck_count"] = 0

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
