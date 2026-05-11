import json
import os


def _get_env_str(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    value = value.strip()
    return value if value else default


def _get_env_bool(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _get_env_list(name, default):
    value = os.getenv(name)
    if value is None or not value.strip():
        return default

    value = value.strip()
    try:
        parsed = json.loads(value)
        if isinstance(parsed, list):
            return parsed
    except json.JSONDecodeError:
        pass

    return [item.strip() for item in value.split(",") if item.strip()]


# 归一化常量
MAX_LANE_WIDTH = 4.0
MAX_SPEED = 33.3
MAX_STEER_ANGLE = 0.7
MAX_ACCEL = 7.0
MAX_DECEL = -7.0
MAX_STEER_DELTA = 0.05
MAX_ACC_DELTA = 1.0

# ===== 配置 =====
TRAIN_MODE = _get_env_bool("TESSNG_TRAIN_MODE", False)
TOTAL_TIMESTEPS = int(_get_env_str("TRAIN_TOTAL_TIMESTEPS", "4000000"))
# DATA_DIR = "Data/prod"
DATA_DIR = _get_env_str("TESSNG_DATA_DIR", "Data")
# global filter scenes
FILTER_SCENES = _get_env_list(
    "TESSNG_FILTER_SCENES",
    [f"scene_{i:02d}.json" for i in range(3, 12)],
)

NET_ROOT = "Data/nets"
NET_PATH = _get_env_str("TESSNG_NET_PATH", r"Data\YYT_TJST_0619_unlimited.tess")
# NET_PATH = r"Data\scenario_0a5c9dbc.tess"
# NET_PATH = r"Data\scenario_0a6bf824.tess"
SCENE_TESS_MAPPING_PATH = _get_env_str("TESSNG_SCENE_TESS_MAPPING_PATH", "Data\map40_to_scene_names.json")


# 车辆参数
WHEEL_BASE = 2.8  # 轴距

# 选择强化学习算法: "DQN" 或 "PPO"
RL_ALGO = "PPO"
MODEL_SAVE_DIR = "tessng_" + RL_ALGO.lower()
EGO_MODEL_FILENAME = _get_env_str("TESSNG_EGO_MODEL_FILENAME", "model.zip")
BG_MODEL_FILENAME = _get_env_str("TESSNG_BG_MODEL_FILENAME", "model.zip.v6")
TENSORBOARD_LOG = "./Log/"
TRAIN_MAX_STEPS = 1000

# 是否使用测试用的场景文件加载逻辑
USE_TEST_LOGIC = _get_env_bool("TESSNG_USE_TEST_LOGIC", True)
# 设定open scenario的起始时间阈值（秒）
START_TIME_THRESHOLD = 1.0

# 是否单一场景重复执行
REPEAT_SINGLE_SCENARIO = _get_env_bool("TESSNG_REPEAT_SINGLE_SCENARIO", True)
EXIT_ON_SIMULATION_STOP = _get_env_bool("TESSNG_EXIT_ON_SIMULATION_STOP", False)
