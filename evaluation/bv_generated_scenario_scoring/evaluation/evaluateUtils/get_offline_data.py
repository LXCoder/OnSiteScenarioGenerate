import pandas as pd
import time
import math
import json

from ..evaluateUtils.weight_process import sceneAndScoreWeightProcess
from ..evaluateUtils.standard_parameter import Parameter
from ..opendrive2tessng.map_info_generation import mapInfoGeneration


class getOfflineData():

    def __init__(self, map_file, aim_place_xosc_file_path, csv_file_path, aim_path_json_file_path, taskName):
        self.map_file = map_file
        self.aim_place_xosc_file_path = aim_place_xosc_file_path
        self.csv_file_path = csv_file_path
        self.aim_path_json_file_path = aim_path_json_file_path
        self.taskName = taskName
        self.nameKey = 'senceall'
        self.weightKey = 'taskWeight'
        self.sceneIscoreKey = "TJTestSceneIscore"  # 单个场景的实时更新指标数据
        self.totalScoreKey = "TJTestTotalScore"  # 全部场景的指标数据累积
        self.mapJsonInforDict = None
        self.aimPathJsonInforDict = None
        self.allDataTransferDict = {}
        self.signalInfo = {}

    # 地图信息生成，包括万集解析数据和终点信息解析数据
    def getMapInfo(self, map_file_path, aim_place_file_path,aim_path_json_file_path):
        map = mapInfoGeneration()
        # 获得终点信息解析数据
        map.goalPlaceInfoProcess(aim_place_file_path,aim_path_json_file_path)
        # 获得万集解析数据
        map.mapDataProcess(map_file_path)
        # 两个结束后，把地图信息都写入了self.road_dict，并且已经放在了一个json文件里面
        return map.road_dict

    # 这里把停止线信息处理成一个四方形的区域，如果在这个区域内，则算是闯红灯了
    def getCrossLineInfo(self, verticesDict):
        polygon_vertices_dict = {}
        # 这里的all_stop_line已经是一个字典了，所以这里要继续按照字典的模式做
        for key in verticesDict["all_stop_line"]:
            polygon_vertices = []
            for i in verticesDict["all_stop_line"][key]:
                for j in i:
                    point = (j[0], j[1])
                    polygon_vertices.append(point)
            polygon_vertices_dict[key] = polygon_vertices
        return polygon_vertices_dict

    def changeLimitSpeed(self):
        mapType = self.mapJsonInforDict['header_info']['name']
        list33 = ['highway', 'highwaymerge']
        list9 = ['mixed', 'intersection', 'roundabout','serial']
        if mapType in list33:
            Parameter.avgSpeed = 33
            Parameter.avgSpeedKM = 33 * 3.6
        elif mapType in list9:
            Parameter.avgSpeed = 9
            Parameter.avgSpeedKM = 9 * 3.6
        else:
            Parameter.avgSpeed = 9
            Parameter.avgSpeedKM = 9 * 3.6

    def startGetOfflineData(self, data_transfer_queue, map_path_data_queue):

        # 还是要redis的，因为key 的那几个都要加进来
        #allRedis = redis.StrictRedis(host=redisWanJIAddress, port=redisWanJIPort, db=0, password="Wanji@300552!")
        # 回头要的数据包括，指标的权重数据，预期完成时间数据，还有地图数据，还有CSV数据
        # 还要包括，场景名称数据、车辆名称数据、测试任务id
        # 单个csv文件只对应单个的场景，一个csv只有一个场景，因此不用设置场景权重

        # 后面想功能的时候再考虑这些数据的获取，现在就考虑怎么样在已经获得上述数据的前提下对后续内容进行评价

        # 路径方面就只需要以下四个1.1、1.2、2、3

        # 1-1要获取基础地图信息
        map_file_path = self.map_file
        # 1-2要获取结束位置信息
        aim_place_file_path = self.aim_place_xosc_file_path
        # 处理地图，这里传输的是两个文件的路径
        #print("开始处理地图信息")
        self.mapJsonInforDict = self.getMapInfo(map_file_path, aim_place_file_path,aim_path_json_file_path=self.aim_path_json_file_path)
        self.mapJsonInforDict["all_stop_line_points"] = self.getCrossLineInfo(self.mapJsonInforDict)
        # 根据地图改变限速信息
        #self.changeLimitSpeed()
        #print("地图信息处理完毕")

        # 2要评价的轨迹信息
        csv_file_path = self.csv_file_path
        # 3要评价的目标路径信息，这里要自己去判断是路径信息，还是信号灯信息
        aim_path_json_file_path = self.aim_path_json_file_path
        try:
            # 读取 JSON 文件
            with open(aim_path_json_file_path, "r") as aim_path_json_file:
                # 使用 json.load() 方法加载 JSON 数据
                self.aimPathJsonInforDict = json.load(aim_path_json_file)
        except:
            self.aimPathJsonInforDict = None
        # 这里要判断一下self.aimPathJsonInforDict里面的内容，如果是信号灯的内容，需要复制到信号灯文件里面，并且把self.aimPathJsonInforDict = None
        if self.aimPathJsonInforDict:
            if "waypoints" not in self.aimPathJsonInforDict:
                self.signalInfo = self.aimPathJsonInforDict
                self.aimPathJsonInforDict = None

        # 在这里把数据放在队列中，或者说在这里把一堆关键数据，包括：3个主要，其他信息：地图、任务ID、场景名称、车辆名称等信息，放在一个字典里面，然后把字典放在一个队列里面

        taskID = 0
        avName = "onsite1"
        sceneName = "onsiteScene"
        sceneTypeList = [sceneName]
        sceneExpectTime = Parameter.missonExpectTime["0_17_straight_straight_21"]
        self.allWeight = sceneAndScoreWeightProcess(Parameter.weightData, sceneTypeList)
        # 这里获得了5个字典，场景间权重字典、维度间权重字典、安全、效率、舒适指标间权重字典
        self.allWeight.weightProcess()
        allWeight = self.allWeight

        # 上面这些数据需要从各种通道中获取


        # todo 上面的东西都是要重新根据通道或者其他手段获得信息，要重做的


        # 第一帧数据先发送静态信息，地图和目标轨迹
        # 如果上面没有数据处理得到地图和轨迹信息，则是none，第二个部分判断一下是不是none就行
        mapAndPathJsonInforDict = {"mapJsonInforDict":self.mapJsonInforDict, "aimPathJsonInforDict":self.aimPathJsonInforDict}
        map_path_data_queue.put(mapAndPathJsonInforDict)
        time.sleep(0.1)

        # 计算文件有多少行，判断什么时候结束
        df = pd.read_csv(csv_file_path, encoding='utf-8')
        end_count = df.shape[0]
        enderCount = 0

        # 使用pandas逐块读取CSV文件
        chunk_size = 1  # 指定每次读取的行数

        dataAll = pd.read_csv(csv_file_path, chunksize=chunk_size)


        text = csv_file_path.split("\\")[-1]
        # for chunk in spb(dataAll, text):
        for chunk in dataAll:
            enderCount += 1
            # 每次读取一行的数据，对这一帧的数据进行 处理和评分，然后再下一行，开始新一行之前把之前的数据清空
            SV_cars_info = []
            scene_info = {}

            # 整理av的数据
            data = chunk.values
            usedTime = data[0][0]
            AV_cars_info = self.newCar_AV(data)

            carNum = int((len(data[0]) - 1-3) / 7)
            # 整理sv的数据
            for i in range(carNum - 1):
                a = (i + 1) * 7
                if not math.isnan(data[0][1 + a +3]):
                    SV = self.newCar(data, a)
                    SV_cars_info.append(SV)
            crashData = data[0][-1]

            # 整理信号灯数据
            if json.dumps(usedTime) in self.signalInfo:
                signalState = self.signalInfo[json.dumps(usedTime)]
            else:
                signalState = None

            # 整理场景数据
            if enderCount != end_count:
                scene_info = {
                    'sceneID': 1,
                    'missonExpectTime': sceneExpectTime,
                    'usedTime': usedTime,
                    'simuTime': usedTime,
                    'sceneWeight': 1,
                    'sceneState': 0,  # 0表示未完成，1表示已完成
                    'sceneAllState': 0  # 0表示未完成，1表示已完成
                }
            else:
                scene_info = {
                    'sceneID': 0,
                    'missonExpectTime': sceneExpectTime,
                    'usedTime': usedTime,
                    'simuTime': usedTime,
                    'sceneWeight': 1,
                    'sceneState': 1,  # 0表示未完成，1表示已完成
                    'sceneAllState': 1  # 0表示未完成，1表示已完成
                }

            self.allDataTransferDict = {
                "AV_cars_info": AV_cars_info,
                "SV_cars_info": SV_cars_info,
                "scene_info": scene_info,
                "taskID": taskID,
                "taskName": self.taskName,
                "avName": avName,
                "sceneName": sceneName,
                "allWeight": allWeight,
                "crashData": crashData,
                "signalState": signalState,
            }
            # todo 如果需要提高评分效率，就可以不用每帧都评价，这里使用put_nowait，但这样必定会忽略一些帧数据的评价
            data_transfer_queue.put(self.allDataTransferDict)
            # try:
            #     data_transfer_queue.put_nowait(self.allDataTransferDict)
            # except:
            #     pass
            #     print("通道已满")
            # time.sleep(0.1)

        # 循环结束后，证明评价结束，则把end放在队列里，表示进程该束了
        data_transfer_queue.put("end")


    def newCar(self, data, a):#适用于背景车加信息
        usedTime = data[0][0]
        xAV = data[0][1 + a +3]
        yAV = data[0][2 + a +3]
        v_ego = data[0][3 + a+3]
        a_ego = data[0][4 + a+3]
        yaw_ego = data[0][5 + a+3]
        width_ego = data[0][6 + a+3]
        length_ego = data[0][7 + a+3]
        cars_info = {
            'id': a/7,
            'x': xAV,
            'y': yAV,
            'simutime': usedTime,
            'speed': v_ego,
            'acce': a_ego,
            'type': 1,
            'width': width_ego,
            'length': length_ego,
            'angle': self.arcToAngle(yaw_ego),
            # 'angle': yaw_ego,
            'HeadwayFront': 0,
            'DistFront': 0,
            # 这下面两个只有av才需要，可以设置为与原始一致，av后续会单独更新，这也是为啥要先做sv的
            'realpos': [xAV, yAV],
            'speedreal': v_ego,
            'longitude': xAV,
            'latitude': yAV,
        }
        return cars_info

    def newCar_AV(self, data):#适用于主车加信息
        usedTime = data[0][0]
        xAV = data[0][3]
        yAV = data[0][4]
        v_ego = data[0][5]
        a_ego = data[0][6]
        yaw_ego = data[0][7]
        width_ego = data[0][9]
        length_ego = data[0][10]
        cars_info = {
            'id': 0,
            'x': xAV,
            'y': yAV,
            'simutime': usedTime,
            'speed': v_ego,
            'acce': a_ego,
            'type': 1,
            'width': width_ego,
            'length': length_ego,
            'angle': self.arcToAngle(yaw_ego),
            # 'angle': yaw_ego,
            'HeadwayFront': 0,
            'DistFront': 0,
            # 这下面两个只有av才需要，可以设置为与原始一致，av后续会单独更新，这也是为啥要先做sv的
            'realpos': [xAV, yAV],
            'speedreal': v_ego,
            'longitude': xAV,
            'latitude': yAV,
        }
        return cars_info





    # onsite弧度制转换为tess角度制的函数
    # 首先第一步是要把弧度转换为角度
    # 第二步把转换后的角度变成正北的
    def arcToAngle(self, value):
        angle_east = value * (180 / math.pi)
        # 将正东为0逆时针增大的角度转换为以正北为0顺时针增大的角度
        angle_north = (90 - angle_east) % 360
        return angle_north