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

# 车辆参数
WHEEL_BASE = 2.8    # 轴距

# 选择强化学习算法: "DQN" 或 "PPO"
RL_ALGO = "PPO"
MODEL_SAVE_DIR = "tessng_" + RL_ALGO.lower()
TENSORBOARD_LOG = "./Log/"
TRAIN_MAX_STEPS = 400