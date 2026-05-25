import json
import logging
import os


WebUrl = "http://129.211.28.237:8082/dist/taskReport?taskID="
# 项目目录
BASEPATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 临时文件位置
TMPPATH = os.path.join(BASEPATH, "Data", "autoEvaluation", "tmp")

# 允许指标计算的最大时间，超出此时间停止计算
calc_evaluation_max_time = 60 * 3

# 抽帧，连续两帧时间差 ms
time_threshold = 300 # 每 X ms 提取一帧数据

# 启动的子仿真程序列表(必须为不重复的自然数)
simulation_indexs = [-1, 0, 1]

# redis key 键映射关系
topic_mapping = {
    "all": "TJ002AllMatchResultMiniData",  # 仿真全量轨迹数据
    "simplify": "TJSimplifyMatchResultMiniData",  # 自动驾驶车辆周边场景仿真交通流数据
    # "auto": "TJAutoMatchResultMiniData",  # 自动驾驶数据
    "auto_simu": "TJAutoMatchResultMiniData",  # 自动驾驶数据(虚拟车辆)
    "auto": "TJParTestScenData",  # 自动驾驶数据（真实主车，含周边交通流）
    "scene": "TJTestSceneData",  # 测试场景状态数据
    # 监听topic，查看是否存在接管消息

    # 监听是否被监管
    "take_over": 'TJParTestScenData',
    "start_parallel": 'TJParTestInitData',
    "ota": "TJParTestOTAData",
}

# 启用 redis 配置
REDIS_HOST = '129.211.28.237'
REDIS_PORT = 6379
REDIS_DB = 3
REDIS_PWD = "jida20231009"
REDIS_USER = "default"

# 日志文件配置
log_name = "tessng"
log_level = logging.INFO
LOG_FILE_DIR = os.path.join(BASEPATH, 'Log')


# mysql 数据库相关配置 (host 为None时不连接)
class MySQLConfig:
    host = "192.168.1.114"  # 线上环境 '10.248.7.65'
    port = 3306
    user = "root"
    pwd = "123456"
    db = "evaluationOnLine"

# class MySQLConfig:
#     host = "10.129.0.20"  # 线上环境 '10.248.7.65'
#     port = 30036
#     user = "root"
#     pwd = "Wanji@123456"
#     db = "evaluationOnLine"

# 分隔时间，当车辆离开区域超出此时间后，再进入视为一次新的测试
split_time_interval = 5
