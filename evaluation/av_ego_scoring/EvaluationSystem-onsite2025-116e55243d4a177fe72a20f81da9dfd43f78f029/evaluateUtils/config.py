import json
import os

from pyproj import Proj



# 项目目录
BASEPATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# tessng_file_path = os.path.join(BASEPATH, 'Data', "ACC_av_sv.tess")
# tessng_file_path = os.path.join(BASEPATH, 'Data', "demo.tess")
# tessng_file_path = os.path.join(BASEPATH, 'Data', "同济测试场_3.tess")
# tessng_file_path = os.path.join(BASEPATH, 'Data', "111.tess")
# tessng_file_path = os.path.join(BASEPATH, 'Data', "111_流量降低.tess")
# tessng_file_path = os.path.join(BASEPATH, 'Data', "111_发10辆.tess")
# tessng_file_path = os.path.join(BASEPATH, 'Data', "111_无路径无车.tess")
# tessng_file_path = os.path.join(BASEPATH, 'Data', "test.tess")
tessng_file_path = os.path.join(BASEPATH, 'Data', "Cyz_TJST_726.tess")
# 交规符合性目录参数
ROOT_DIR = os.path.join(BASEPATH, 'evaluateUtils', 'TrafficRule_compliance')
# DATA_DIR = os.path.join(BASEPATH, 'testData')
# SCENARIO_DIR = os.path.join(BASEPATH, 'Data', 'scenario')
# TRJ_DIR = os.path.join(BASEPATH, 'Data', 'trajectory')
# RES_DIR = os.path.join(BASEPATH, 'Data', 'result')
# KB_DIR = os.path.join(ROOT_DIR, 'KnowledgeBase')
KB_DIR = os.path.join(BASEPATH, 'testData/KnowledgeBase')
# CACHE_DIR = os.path.join(ROOT_DIR, 'cache')
SOURCE_COL = 'source'
ARTICLE_COL = 'article'
CONTENT_COL = 'content'
ROAD_TAG_COL = 'road_tag'
INFR_TAG_COL = 'infr_tag'
MAN_TAG_COL = 'man_tag'
ENV_TAG_COL = 'env_tag'
ROAD_TAG_CODE_COL = 'road_tag_code'
INFR_TAG_CODE_COL = 'infr_tag_code'
MAN_TAG_CODE_COL = 'man_tag_code'
PTC_TAG_CODE_COL = 'ptc_tag_code'
ENV_TAG_CODE_COL = 'env_tag_code'
# Scenario data segmentation configs
SCENARIO_ID_COL = 'scenario_id'
SEGMENT_ID_COL = 'segment_id'

# 自动驾驶车辆雷达探照距离，超出此距离的车辆不发送给自动驾驶车辆
av_radar_distance = 100

# 仿真精度
accuracy = 15

p = Proj('+proj=tmerc +lon_0=121.20585769414902 +lat_0=31.290823210868965 +ellps=WGS84')

# todo WCX 万集redis地址
# redisWanJIAddress = '106.120.201.126'
# redisWanJIPort = 14611

redisWanJIAddress = '202.120.189.56'
redisWanJIPort = 5010
#
# # KAFKA_HOST = "106.120.201.126"
# # KAFKA_PORT = 19359
# KAFKA_HOST = "106.120.201.000"
# KAFKA_PORT = 10000

#KAFKA_HOST = "1.15.187.167"
#KAFKA_PORT = 9092

# todo 0925畅行演示kafka服务器
# KAFKA_HOST = "202.120.189.56"
# KAFKA_PORT = 5011

# # 内部kafka的地址和端口
# KAFKA_HOST = "129.211.28.237"
# KAFKA_PORT = 29092

# topic = 'TJTestSceneData'  # 测试场景状态数据
# topic = 'TJAutoMatchResultMiniData'  # 自动驾驶数据
# topic = 'TJ002AllMatchResultMiniData'  # 仿真全量轨迹数据
# topic = 'TJSimplifyMatchResultMiniData'  # 自动驾驶车辆周边场景仿真交通流数据

# 对外的topic
#send_topic_mapping = {
#     "all": "TJ002AllMatchResultMiniData",  # 仿真全量轨迹数据
#     "simplify": "TJSimplifyMatchResultMiniData",  # 自动驾驶车辆周边场景仿真交通流数据
#     "auto": "TJAutoMatchResultMiniData",  # 自动驾驶数据
#     "scene": "TJTestSceneData",  # 测试场景状态数据
#     "evaluate": "TJTestEvaluateData",   # 仿真评价数据
# }

# 本地测试的
send_topic_mapping = {
    "all": "TJ002AllMatchResultMiniData",  # 仿真全量轨迹数据
    "simplify": "TJSimplifyMatchResultMiniData",  # 自动驾驶车辆周边场景仿真交通流数据
    "auto": "TJAutoMatchResultMiniData",  # 自动驾驶数据
    "scene": "TJTestSceneData",  # 测试场景状态数据
    "evaluate": "TJTestEvaluateData",   # 仿真评价数据
    "sceneIscore": "TJTestSceneIscore",  # 单个场景的指标数据
    "totalScore": "TJTestTotalScore",  # 全部场景的指标数据累积
}
send_interval = 0.07  # 数据发送时间间隔

# 仿真车辆类型映射表，同时需要修改车长
sv_to_real_mapping = {
    1: 1,  # 1: 客车(tess) 0: 私家车(千乘)
    4: 1,  # 4: 货车(8m) 3: 中巴车
    2: 1,  # 2: 大客车(12m) 5: 大巴车
    7: 4,   # 7: 行人
    6: 5    # 自行车
}

# tessng 与外部数据比例配置
coordinate_scale = 1
speed_scale = 1

# # TODO 谨记，因为同一场景存在多条路径，发车点最好设置在分流前，确保车辆自动分流，分流后依照诱导路径行驶/合流
# # route_data = json.load(open(os.path.join(BASEPATH, 'Data', 'av_sv_6.json')))
# # av 采用车道优先， sv采用 路径诱导
# demo_route_mapping = {
#     # 'av': route_data['av6'],
# }
#
# # 可以在此处设置仿真车辆的行驶路径
# link_route_mapping = {
#     # 'sv': route_data['sv6']
# }
#
# # 场景数据
# scene_list = ["左转遇到直行混合交通流", "无信控环岛交织", "直行遇到对向左转车流", "匝道汇入交织"]
# scene_info = {
#     "timeStamp": None,
#     "frameId": 1,
#     "count": str(len(scene_list)),
#     "data": [
#         {
#             "sceneId": index + 1,
#             "sceneNum": index + 1,
#             "sceneName": name,
#             "sceneStatus": 3,
#             "sceneStartTime": None,
#             "sceneEndTime": None,
#         } for index, name in enumerate(scene_list)
#     ]
# }
