# 这里文件里面只记录一些最基本的可变参数
# 比如不同指标的权重，不同间隙大小应该扣除的分数，等等



class Parameter:

    # def __init__(self, avgSpeed, avgSpeedKM):
    #     self.avgSpeed = avgSpeed
    #     self.avgSpeedKM = avgSpeedKM

    # 任务名称
    taskName = "0"
    # 初始地图位置
    map_file = 'testData/0_1.xodr'
    # 目标结束位置信息
    aim_place_xosc_file_path = "testData/0_2.xosc"
    # 交叉口停车线信息
    cross_line_json_file_path = "testData/0_3.json"
    # 处理后的地图位置
    all_road_info_file = 'testData/1.json'
    # 轨迹信息
    csv_file_path = 'testData/2.csv'
    # 目标路径信息
    aim_path_json_file_path = "testData/3.json"


    a = 20 # 安全分最低标准
    b = 4

    # 车辆长度
    length = 4.49
    width = 1.95

    # 计算精度
    simuAccuracy = 0.03
    # 设置时间差（秒级，0.2表示200毫秒）
    threshold_minutes = 0.5
    # 设置图表的粒度
    granularity = 2

    # 设置期望速度
    avgSpeedKM = 20
    avgSpeed = 5

    # 按次扣分
    miniScore = {'横向加速度': 5,
                 '横向加加速度': 5,
                 '纵向加速度': 5,
                 '纵向加加速度': 5,
                 '未按规定路线行驶': 1, # 碰撞和未按规定路线行驶比较特殊，不用来计算分数，只用来计算次数，如果碰撞次数有1次就直接安全分为0
                 "碰撞": 1,
                 "闯红灯": 100,}

    # onsite权重配置文件
    weightData = {
        "sceneWeightDict": {
            "free" :0.2,
            "byCrossing" :1.5,
            "sideOn" :1.2,
            "huandao": 1.5,
            "onsite": 1,
        },
        "indexWeightDict" :{
            "10000": 0.5, # 安全
            "20000": 0.2, # 舒适
            "30000": 0.3, # 效率
            "10001": 1,
            "10002": 1,
            "10003": 0.5,
            "10004": 0,
            "10005": 0,
            "10006": 1,
            "10007": 0,
            "10008": 0,
            "10009": 0.5,
            "20001": 0.2,
            "20002": 0.2,
            "20003": 0.2,
            "20004": 0.2,
            "20005": 0.2,
            "30001": 0,
            "30002": 0.5, # 速度
            "30003": 0,
            "30004": 0.5, # 任务是否完成
        },
        "sceneNameDict": {
            "free" :["自由通过","随便跑"],
            "byCrossing" :["交叉口","通过交叉口"],
            "sideOn" :['其他车辆侧向汇入汇出',"侧向汇入"],
            "huandao" :["通过环岛"],
            "onsite":["onsiteScene"]
        }
    }

    # onsite期望时间配置文件
    missonExpectTime = {
        "0_17_straight_straight_21": 12,
    }














