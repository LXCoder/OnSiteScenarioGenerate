"""
TessngDrivingEnvPPO —— 将 Tessng 仿真封装为 Gymnasium 环境

动作空间：连续（适配 PPO/SAC 等连续控制算法）
  action[0]: accel - 加速度 [-5.0, 5.0] m/s^2
  action[1]: steer - 转向角 [-0.7, 0.7] rad

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


class TessngDrivingEnvPPO(gym.Env):
    """
    Gymnasium 环境封装（连续动作，适配 PPO）

    observation_space: Box(0, 1, shape=(obs_dim,))
    action_space: Box(low=[-5.0, -0.7], high=[5.0, 0.7], shape=(2,))
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, obs_dim: int = OBS_DIM):
        super().__init__()

        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )
        
        # 动作空间：连续 [accel, steer]
        # 加速度: [-5.0, 5.0] m/s^2
        # 转向角: [-0.7, 0.7] rad (这里与 MySimulator 中的最大转向角对应)
        self.action_space = spaces.Box(
            low=np.array([-7.0, -0.7]), 
            high=np.array([7.0, 0.7]), 
            dtype=np.float32
        )

        # ===== 线程同步 =====
        self._actionReady = threading.Event()
        self._obsReady = threading.Event()
        self._resetReady = threading.Event()
        self._resetDone = threading.Event()

        # ===== 共享数据 =====
        self._action: Optional[np.ndarray] = None        # 连续动作 [accel, steer]
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

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """
        训练线程调用

        Args:
            action: 连续动作数组 [accel, steer]
        """
        if self._closed:
            raise RuntimeError("环境已关闭")

        # 确保动作在定义范围内
        clipped_action = np.clip(action, self.action_space.low, self.action_space.high)
        self._action = clipped_action
        
        # 转换为 Python float tuple 以供控制使用
        self._control = (float(clipped_action[0]), float(clipped_action[1]))

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
        """当前动作名称 (用于打印调试)"""
        if self._action is not None:
            return f"Accel:{self._action[0]:.2f}, Steer:{self._action[1]:.2f}"
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
