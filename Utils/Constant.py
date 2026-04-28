# 归一化常量
MAX_LANE_WIDTH = 4.0
MAX_SPEED = 33.3
MAX_STEER_ANGLE = 0.7
MAX_ACCEL = 7.0
MAX_DECEL = -7.0
MAX_STEER_DELTA = 0.05
MAX_ACC_DELTA = 1.0

# ===== 配置 =====
TRAIN_MODE = True
TOTAL_TIMESTEPS = 4000000
# DATA_DIR = "Data/prod"
DATA_DIR = "Data"
# global filter scenes
FILTER_SCENES = [f"scene_{i:02d}.json" for i in range(3, 12)]

# NET_PATH = r"Data\scenario_0a5c9dbc.tess"
# NET_PATH = r"Data\scenario_0a6bf824.tess"
NET_PATH = r"Data\YYT_TJST_0619_unlimited.tess"


# 车辆参数
WHEEL_BASE = 2.8  # 轴距

# 选择强化学习算法: "DQN" 或 "PPO"
RL_ALGO = "PPO"
MODEL_SAVE_DIR = "tessng_" + RL_ALGO.lower()
EGO_MODEL_FILENAME = "model.zip.v8"
BG_MODEL_FILENAME = "model.zip.v6"
TENSORBOARD_LOG = "./Log/"
TRAIN_MAX_STEPS = 1000

# 是否使用测试用的场景文件加载逻辑
USE_TEST_LOGIC = True
# 设定open scenario的起始时间阈值（秒）
START_TIME_THRESHOLD = 1.0