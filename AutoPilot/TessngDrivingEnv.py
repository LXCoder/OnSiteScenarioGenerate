"""
TessngDrivingEnv —— 将 Tessng 仿真封装为 Gymnasium 环境

动作空间：离散（适配 DQN）
  0: LANE_LEFT   - 左变道
  1: IDLE         - 保持
  2: LANE_RIGHT  - 右变道
  3: FASTER      - 加速
  4: SLOWER      - 减速

观测空间：Box(0, 1, shape=(obs_dim,))

核心机制：
  Tessng 在主线程跑（Qt 事件循环），通过 afterOneStep 回调。
  训练脚本在子线程跑，调用 env.step(action)。
  两边通过 threading.Event 同步。
"""

import threading
import numpy as np
import gym
from gym import spaces
from typing import Optional, Dict, Any, Tuple


OBS_DIM = 94

# ===== 离散动作定义 =====
ACTIONS = {
    0: "LANE_LEFT",
    1: "IDLE",
    2: "LANE_RIGHT",
    3: "FASTER",
    4: "SLOWER",
}
NUM_ACTIONS = len(ACTIONS)

# 每个离散动作对应的 (accel, steer) 控制量
ACTION_TO_CONTROL = {
    0: (0.0, -0.3),     # LANE_LEFT:  不加速，左转
    1: (0.0, 0.0),      # IDLE:       保持
    2: (0.0, 0.3),      # LANE_RIGHT: 不加速，右转
    3: (3.0, 0.0),      # FASTER:     加速，不转
    4: (-3.0, 0.0),     # SLOWER:     减速，不转
}


class TessngDrivingEnv(gym.Env):
    """
    Gymnasium 环境封装（离散动作，适配 DQN）

    observation_space: Box(0, 1, shape=(obs_dim,))
    action_space: Discrete(5)
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, obs_dim: int = OBS_DIM):
        super().__init__()

        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Discrete(NUM_ACTIONS)

        # ===== 线程同步 =====
        self._actionReady = threading.Event()
        self._obsReady = threading.Event()
        self._resetReady = threading.Event()
        self._resetDone = threading.Event()

        # ===== 共享数据 =====
        self._action: Optional[int] = None        # 离散动作索引
        self._control: Tuple[float, float] = (0.0, 0.0)  # 对应的 (accel, steer)
        self._obs: Optional[np.ndarray] = None
        self._reward: float = 0.0
        self._done: bool = False
        self._truncated: bool = False
        self._info: Dict[str, Any] = {}

        # ===== 状态 =====
        self._needReset: bool = False
        self._closed: bool = False

        # ===== 回调钩子 =====
        self.resetCallback = None
        self.buildObsCallback = None

    # ============================================================
    #  Gymnasium 接口（训练线程调用）
    # ============================================================

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """
        训练线程调用

        Args:
            action: 离散动作索引 (0~4)
        """
        if self._closed:
            raise RuntimeError("环境已关闭")

        self._action = int(action)
        self._control = ACTION_TO_CONTROL[self._action]

        self._obsReady.clear()
        self._actionReady.set()

        self._obsReady.wait()

        return (
            self._obs.copy(),
            self._reward,
            self._done,
            self._info.copy(),
        )

    def reset(self) -> np.ndarray:
        """
        训练线程调用，触发 Tessng 场景重置

        Returns:
            obs（gym 0.21 只返回 obs）
        """
        self._needReset = True
        self._resetDone.clear()
        self._resetReady.set()

        self._resetDone.wait()

        self._done = False
        self._truncated = False

        return self._obs.copy()

    def close(self):
        self._closed = True
        self._actionReady.set()
        self._resetReady.set()

    # ============================================================
    #  Tessng 回调侧（afterOneStep 中调用）
    # ============================================================

    def onSimuStep(
        self,
        obsArray: np.ndarray,
        reward: float,
        done: bool,
        truncated: bool = False,
        info: Optional[dict] = None,
    ) -> Optional[Tuple[float, float]]:
        """
        每帧由 afterOneStep 调用

        Returns:
            (accel, steer) 控制量，或 None（环境已关闭 / episode 结束）
        """
        self._obs = np.array(obsArray, dtype=np.float32)
        self._reward = reward
        self._done = done
        self._truncated = truncated
        self._info = info or {}

        # 通知训练线程 obs 准备好了
        self._obsReady.set()

        if self._closed:
            return None

        # episode 结束时，不等 action，直接返回
        # 训练线程收到 done 后会调 reset()，由下一帧的 afterOneStep 处理
        if done:
            return None

        # 等待训练线程给出下一个 action
        self._actionReady.wait()
        self._actionReady.clear()

        if self._closed:
            return None

        return self._control

    def onFirstStep(self) -> Optional[Tuple[float, float]]:
        """仿真第一帧调用"""
        self._resetReady.wait()
        self._resetReady.clear()

        if self._needReset:
            self._needReset = False
            self._handleReset()

        self._actionReady.wait()
        self._actionReady.clear()

        return self._control

    @property
    def currentActionName(self) -> str:
        """当前动作名称"""
        if self._action is not None:
            return ACTIONS.get(self._action, "UNKNOWN")
        return "NONE"

    def _handleReset(self):
        if self.resetCallback:
            self.resetCallback()

        if self.buildObsCallback:
            self._obs = np.array(self.buildObsCallback(), dtype=np.float32)
        else:
            self._obs = np.zeros(self.observation_space.shape, dtype=np.float32)

        self._info = {}
        self._resetDone.set()