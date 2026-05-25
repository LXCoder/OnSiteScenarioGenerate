# main函数里主要做的就是把数据读出来，然后把数据传出去
# 这样的话这里整好定义一个函数的接口，传入一些数据（轨迹数据和场景数据）到一个指标评价类里面
# 在写一个输出函数，把类里面计算出来的得分都返回出来
import queue
import json
import csv
from datetime import datetime
from ..evaluateUtils.get_offline_data import getOfflineData
from ..evaluateUtils.AutoSelfDrivingCar import SelfDrivingCar, Car, TTCCar, PETCar, np
from ..evaluateUtils.MySource.scoreStatistic import selfStateScore, interactionScore
from ..evaluateUtils.standard_parameter import Parameter
from ..evaluateUtils.diagnose_library import diagnose


class evaluationSystem():

    def __init__(self, detail_csv_path=None, score_csv_path=None):
        self.detail_csv_path = detail_csv_path
        self.score_csv_path = score_csv_path

        # 只需要实例化一次ego车辆，后面每次循环都更新这个车辆就行了
        self.Ego = SelfDrivingCar(id=0, Xpos=0, Ypos=0, speed=0, roadType=0, startPos=0)
        self.TTC_Car: TTCCar = TTCCar(id=0, Xpos=0, Ypos=0, speed=0, roadType=0)

        # sv车辆是需要存一下的，每次实例化都要存一下，同理也要pet的车
        self.SV_car_Dict = {}
        self.PET_car_Dict = {}

        # 这个还是一个大字典，每个字典对应一个车，每个车都要实例化后再存入这个列表，或者更新替换，或者创建一个新车

        # 记录一些冲突的指标
        self.conflictCarIdList = []
        self.lastConflictTime = []

        # 用来存储上一帧的数据
        self.lastSceneData = []
        self.lastAVdata = {}
        # 这里只放sv的id，主要是间隙计算的时候用
        self.svAllPosList = []

        # 下面这些是用来存放从卡夫卡拉下来的数据，形成自己的轨迹，后面计算都是依据这些来的
        self.sceneData = {}
        self.avPosList = []
        # 所有的sv的车辆轨迹列表，这里可以用明阳的建议用字典来做
        # 每个键是id是一辆车，键值是一个列表，里面存放的是self.svIPosList，每次拿到数据就要更新一遍
        self.svAllPosDict = {}
        # 每次场景结束就清空一次吧，不然测最后一个场景的时候还保留第一个场景的数据就太怪了

        # 记录场景的得分
        self.senceScoreDital = []
        self.senceScoreTotal = []
        self.totalScore = {}

        # 记录场景id，这个id不会是0，用于数据传输
        self.sceneIdNoneZero = 0

        # 最后场景全部结束要计算唯一一次
        self.allSceneEndLock = 0
        self.allSceneStartLock = 0

        # 记录场景的权重和期望时间，每次场景更新这里就更新添加一下就行
        self.senceWeight = {}
        self.missonExpectTime = {}

        # 记录任务的开始结束时间
        self.missonStartTime = 0
        self.missonEndTime = 0
        self.missonStartTimeSimu = 0
        self.missonEndTimeSimu = 0
        self.testDuration = 0


        # 静态场景信息
        self.caseID = None
        self.avName = None
        self.taskName = None
        # 静态权重信息
        self.allWeight = None

        # 要发送评分到警警这里
        self.sceneIscoreKey = "TJTestSceneIscore"  # 单个场景的实时更新指标数据
        self.totalScoreKey = "TJTestTotalScore"  # 全部场景的指标数据累积

        # 存放折线图的数据
        self.chartDataSingleScene = {}
        self.chartDataTotalScene = {}
        self.xTime = 0

        # 设置时间差
        self.threshold_minutes = Parameter.threshold_minutes
        # 设置图表的粒度
        self.granularity = Parameter.granularity

        # 这个标志要在完成静态信息收集后，就及时清空
        self.sceneMessage = None

    def changeLimitSpeed(self, mapJsonInforDict):
        mapType = mapJsonInforDict['header_info']['name']
        list33 = ['1', '2']
        list9 = ['3', '4', '5']
        if mapType in list33:
            Parameter.avgSpeed = 33
            Parameter.avgSpeedKM = 33 * 3.6
        elif mapType in list9:
            Parameter.avgSpeed = 9
            Parameter.avgSpeedKM = 9 * 3.6
        else:
            Parameter.avgSpeed = 60/3.6
            Parameter.avgSpeedKM = 60

    def startMyEvaluation(self, data_transfer_queue, map_path_data_queue):

        # 还是要redis的，因为key 的那几个都要加进来
        # allRedis = redis.StrictRedis(host=redisWanJIAddress, port=redisWanJIPort, db=0, password="Wanji@300552!")

        map_path_data = map_path_data_queue.get()
        if "mapJsonInforDict" in map_path_data and "aimPathJsonInforDict" in map_path_data:
            if map_path_data["mapJsonInforDict"]:
                mapJsonInforDict = map_path_data["mapJsonInforDict"]
            aimPathJsonInforDict = map_path_data["aimPathJsonInforDict"]
        else:
            # 最开始要读一下json的地图信息文件
            # json_file_path = "evaluateUtils/Config/all_road_dict_json.json"
            json_file_path = "evaluateUtils/Config/all_road_dict_json_OnsiteTest1.json"
            # 读取 JSON 文件
            with open(json_file_path, "r") as json_file:
                # 使用 json.load() 方法加载 JSON 数据
                mapJsonInforDict = json.load(json_file)
            aimPathJsonInforDict = None

        # 改变期望速度
        self.changeLimitSpeed(mapJsonInforDict)
        wsh=1
        verticalA=0
        verticalAPlus=0
        horizontalA=0
        horizontalAPlus=0
        turnA=0

        while True:

            allNeedData = data_transfer_queue.get()
            if allNeedData == "end":
                break

            # 在这里开始把各种数据拿出来
            scene_info = allNeedData["scene_info"]
            AV_cars_info = allNeedData["AV_cars_info"]
            SVALL_cars_info = allNeedData["SV_cars_info"]

            self.allWeight = allNeedData["allWeight"]
            self.avName = allNeedData["avName"]
            self.sceneName = allNeedData["sceneName"]
            self.caseID = allNeedData["taskID"]
            self.taskName = allNeedData["taskName"]
            crash = allNeedData["crashData"]
            signalState = allNeedData["signalState"]

            # scene_info = message.value['scene_info']
            # AV_cars_info = message.value['AV_cars_info'] # 这里是一个字典，是av车的最新一帧的状态信息
            # SVALL_cars_info = message.value['SV_cars_info'] # 这里实际上就是最新一帧所有sv的当前状态信息，有几个sv车就有几个字典，每个字典是一个车的状态信息
            # print(scene_info)
            # 拿到av车的历史轨迹数据
            # 需要判断一下是不是空数据
            if AV_cars_info:
                self.avPosList.append(AV_cars_info)
                if len(self.avPosList) > 1000:
                    del self.avPosList[0]

            # 现有的车辆
            currentSVlist = []
            # 拿到所有sv车的历史轨迹数据
            for svI in SVALL_cars_info:
                # 判断svAllPosDict中的键有没有svI的id，如果有就添加到键对应的值的列表中，没有就创建一对键值
                if svI:
                    # 这里的内容是要把所有sv车的数据，一个一个放在svAllPosDict里面，这个是一个大字典，每个大字典里面是一个列表，这个列表是这个车的所有历史轨迹数据
                    # 所以这里就是，如果车的id在字典里面，就新增，如果不在，就重新加一个，也就是svAllPosDict[svI['id']] = [svI]，svI是数据，id是id号
                    if svI['id'] in self.svAllPosDict:
                        self.svAllPosDict[svI['id']].append(svI)
                        # 这里还要判断一下列表的长度，超过几百个点就要删除第一个了
                        # 切片的方式，把最后1000条数据都存放在svAllPosDict里面，这样就始终是1000个数据了
                        # svAllPosDict = svAllPosDict[-1000:]
                        if len(self.svAllPosDict[svI['id']]) > 1000:
                            del self.svAllPosDict[svI['id']][0]
                    else:
                        self.svAllPosDict[svI['id']] = [svI]
                    currentSVlist.append(svI['id'])
            # 这里还要多加一层判断，如果传进来的车这次id少了某个车的，证明这个车在当前不在地图中了，需要吧他的轨迹删除掉
            # 消失的车辆
            self.Ego.currentSVcars = currentSVlist
            self.Ego.clearSVinforDict()
            cars_to_remove = []
            for key in self.svAllPosDict.keys():
                if key not in currentSVlist:
                    cars_to_remove.append(key)
            # 删除多余的车辆，后面还有几个地方也要删除，petcar，av里面的那个几个dict，直接在这里价格判断一次性删除
            for key in cars_to_remove:
                del self.svAllPosDict[key]
                if key in self.SV_car_Dict:
                    del self.SV_car_Dict[key]
                if key in self.PET_car_Dict:
                    del self.PET_car_Dict[key]

            # 拿到场景数据
            self.sceneData = scene_info
            self.lastSceneData.append(self.sceneData)
            if len(self.lastSceneData)>2:
                del self.lastSceneData[0]
            # last场景数据，[0]是上一帧的数据，[1]是这一帧的数据

            # 把每个经历过的场景和对应的权重存放在场景权重字典里面
            if 'sceneWeight' in self.sceneData:
                if self.sceneData['sceneID'] in self.senceWeight:
                    pass
                else:
                    self.senceWeight[self.sceneData['sceneID']] = self.sceneData['sceneWeight']
                    self.missonExpectTime[self.sceneData['sceneID']] = self.sceneData['missonExpectTime']

            # 记录开始评价的时间
            if self.sceneData['sceneID'] and 'simuTime' in self.sceneData and not self.allSceneStartLock:
                self.missonStartTime = str(datetime.now())[:23]
                self.missonStartTimeSimu = self.sceneData['simuTime']
                #print("场景开始时间",self.missonStartTime,self.missonStartTimeSimu)
                self.allSceneStartLock = 1
                # 新的一轮场景测试的开始，就可以给旧测试解锁了
                self.allSceneEndLock = 0

                # 下面的内容可以封装成一个函数，输入上面三个变量，然后输出一个当前状态的分数
            # 这里就不做所有场景的总分计算了，所有场景的总分回头用自己计算，每个场景结束的时候给他算个当前场景的总分，和乘过权重的分数

            # 第一步判断是否是最新的一帧数据，用场景和av来判断，用记录的时间
            # if lastAVdata != avPosList[-1]:
            if self.sceneData and len(self.avPosList)>2:
                # 但是第一步要先更新所有的车辆信息，包括av车辆的更新，
                renewCarState(self.Ego, self.avPosList)
                self.Ego.longitudeAndLatitude = [round(self.avPosList[-1]["longitude"],6), round(self.avPosList[-1]["latitude"],6)]
                # 在这里要做一些操作来辅助计算平均速度
                self.Ego.currentPos = self.avPosList[-1]['realpos']
                self.Ego.previousPos = self.avPosList[-2]['realpos']
                moveDistance = self.Ego.calculatePointDistance()
                self.Ego.calculateTravelDistance(moveDistance)
                self.Ego.calculateAvgSpeed(self.sceneData["simuTime"], self.sceneData["usedTime"])
                # print("上一帧的位置", self.Ego.previousPos, "这一帧的位置", self.Ego.currentPos, "起始位置",
                #       self.Ego.startPos, "移动距离", moveDistance, "总体移动距离", self.Ego.distance)
                # 把平均速度都计算好了，存放在self.sceneAvgSpeedKM里面了，而且这个数据是实时更新的
                accuracy = abs(self.avPosList[-1]['simutime'] - self.avPosList[-2]['simutime'])
                # print("仿真精度，评价一次要用多久",accuracy)
                if accuracy:
                    self.Ego.accuracy = accuracy
                else:
                    self.Ego.accuracy = 0.02
                    # 单独更新加速度
                self.Ego.calcuAcce()
                self.avPosList[-1]['acce'] = self.Ego.acce
                self.Ego.posList[-1]['acce'] = self.Ego.acce
                # 更细ego车的二维边界
                vertices = self.Ego.calculate_rectangle_vertices(self.Ego.length, self.Ego.width, self.Ego.angle)

                # 更新sv的车辆信息，有就更新，没有就新键，for循环svAllPosDict
                for carID in self.svAllPosDict:
                    if carID in self.SV_car_Dict:
                        # 更新sv这个字典的值
                        renewCarState(self.SV_car_Dict[carID], self.svAllPosDict[carID])
                    else:
                        # 创建一个新车，新加一个字典的键值
                        nowState = self.svAllPosDict[carID][-1]
                        SV_Car = Car(id=nowState['id'], Xpos=nowState['x'], Ypos=nowState['y'],
                                     speed=nowState['speed'], roadType=0)
                        renewCarState(SV_Car, self.svAllPosDict[carID])
                        self.SV_car_Dict[carID] = SV_Car
                    # 对这个车（不管是新加的还是之前的）都进行一个更新，主要是距离的更新，用来算这个车和主车的距离
                    nowState = self.svAllPosDict[carID][-1]
                    SV_Pos = [nowState['x'], nowState['y']]
                    self.Ego.recordDistanceBetweenSimulatorVehicle(nowState['id'], SV_Pos, nowState['angle'], nowState["speed"])
                    # 到这里就把所有的车辆更新了，并且存进来了
                    self.svAllPosList.append(self.svAllPosDict[carID])

                # 这一步顺便也把ttc车更新一下，实例化已经做了
                # self.TTC_Car.id = (self.Ego.findClosestSimulatorVehicle())
                self.TTC_Car.id = (self.Ego.findTTCVehicle())
                # print(self.TTC_Car.id)
                if self.TTC_Car.id:
                    for carID in self.svAllPosDict:
                        if self.TTC_Car.id == carID:
                            # 更新这个ttc车辆要计算的东西
                            nowState = self.svAllPosDict[carID][-1]
                            self.TTC_Car.currentPos = [nowState['x'], nowState['y']]
                            self.TTC_Car.speed = nowState['speed']
                            # 获取到TTC Car的车辆二维边界
                            self.TTC_Car.calculate_rectangle_vertices(Parameter.length, Parameter.width, nowState['angle'])

                # 这一步顺便把pet的车也实例化了
                PETCarIdList = (self.Ego.findCloseSimulatorVehicle())
                if PETCarIdList:
                    for carID in self.svAllPosDict:
                        if carID in PETCarIdList:
                            if carID in self.PET_car_Dict:
                                # 更新sv这个字典的值
                                renewCarState(self.PET_car_Dict[carID], self.svAllPosDict[carID])
                            else:
                                # 创建一个新PET车，新加一个字典的键值
                                nowState = self.svAllPosDict[carID][-1]
                                PET_Car = PETCar(id=nowState['id'], Xpos=nowState['x'], Ypos=nowState['y'],
                                             speed=nowState['speed'], roadType=0)
                                renewCarState(PET_Car, self.svAllPosDict[carID])
                                self.PET_car_Dict[carID] = PET_Car
                # 到这一步也是把所有pet的车辆都实例化成了petcar了


                # 第二步把拿到最新的数据利用起来，开始计算
                # 计算前要判断场景是否已经开始了
                if self.sceneData['sceneID'] or isSceneEnd(self.lastSceneData):
                    if self.sceneData['sceneID']:
                        self.sceneIdNoneZero = self.sceneData['sceneID']
                    # 计算分成三步，第一步计算无交互的基础指标，第二步计算有交互的TTC和pet和碰撞和间隙拒绝
                    # 最后一步就是判断是否存在场景或者说存在期望时间的概念，如果存在就计算时间的效率，如果不存在就不计算时间效率

                    # 这个好像是ttc要用，间隙也要用
                    senceIinteractionScore = interactionScore()
                    # 这里碰撞是直接用的他们onsite输出的可以直接拿到
                    senceIinteractionScore.calcuCrashOnsite(crash)
                    self.Ego.scoreDict['碰撞'] = senceIinteractionScore.crash
                    # 无交互的指标计算
                    avStateScoreCalcu(self.Ego, SVALL_cars_info, self.sceneIdNoneZero, mapJsonInforDict, PETCarIdList, aimPathJsonInforDict, self.sceneData['sceneID'], signalState, self.allWeight.indexWeight)


                    # 计算PET
                    # 这里要把petav也算了
                    # 这里就体现出来字典的优势了，原来需要两个for循环，现在直接用字典就解决了，petcar如果存在，则一定存在一个对应的svcar
                    for carID in self.PET_car_Dict:
                        if self.PET_car_Dict[carID].petLock == 0:
                            objCarPosList = self.PET_car_Dict[carID].posList
                            pet, petAV = self.PET_car_Dict[carID].calculate_pet(self.Ego.posList, objCarPosList)
                            if pet != 0:
                                self.PET_car_Dict[carID].petLock = 1
                        else:
                            pet = 0
                            petAV = 0

                        # 计算碰撞，下面三个注释取消，就是重新计算碰撞
                        # svVertices = self.PET_car_Dict[carID].calculate_rectangle_vertices(self.SV_car_Dict[carID].length, self.SV_car_Dict[carID].width, self.SV_car_Dict[carID].angle)
                        # avVertices = self.Ego.calculate_rectangle_vertices(self.Ego.length, self.Ego.width, self.Ego.angle)
                        # crash = self.PET_car_Dict[carID].calculate_crash(svVertices, avVertices, self.SV_car_Dict[carID].angle, self.Ego.angle, self.Ego.accuracy)

                        # 在这里把senceIselfStateScore实例化，然后用一个总体的计算函数，来计算当前帧的状态信息
                        # 不用传参不用return，都在内部记录好了
                        # if self.PET_car_Dict[carID].crashTime:
                        #     print("碰撞时间", self.PET_car_Dict[carID].crashTime, accuracy, "碰撞车辆id是", carID, "sv长",self.SV_car_Dict[carID].length, "sv宽",self.SV_car_Dict[carID].width, "sv角度",self.SV_car_Dict[carID].angle,"计算后的sv顶点坐标",svVertices)
                        #     print("主车id是", carID, "av长",self.Ego.length, "av宽",self.Ego.width, "av角度",self.Ego.angle,"计算后的sv顶点坐标",avVertices)
                        # 下面这里两个碰撞计算用哪个屏蔽就另一个，用crash = self.PET_car_Dict[carID].calculate_crash的函数计算的self.PET_car_Dict[carID].crashTime 可以用来判断碰撞，也可以用onsite自带的检查机制检查碰撞
                        # senceIinteractionScore.calcuCrash(self.PET_car_Dict[carID].crashTime, self.Ego.accuracy)
                        senceIinteractionScore.calcuCrashOnsite(crash)
                        senceIinteractionScore.calcuOverPET(pet)

                        # 后面就是把参数记录进Ego里面，用于时间的累加计算
                        self.Ego.scoreDict['碰撞'] = senceIinteractionScore.crash
                        self.Ego.scoreDict['PET'] = senceIinteractionScore.overPET
                        if petAV != 0:
                            # 有新的想法了，这里不用去计算，只需要去记录，记录以及发生pet计算的sv车辆的id，在这个大循环中把每个这样的车都找到后，在后面只用已经产生pet的sv车来计算这个gap就行了
                            # 而且这里要加入sv车辆的话要判断一下冲突点时间，这个再说
                            # 后面只需要判断这个id列表是不是空，如果不是空，就记录一下时间，然后这个时间加0.5秒，仿真时间到达这个时间加0.5秒后，才开始计算一次这个值，计算后就把id列表清空
                            self.conflictCarIdList.append(self.PET_car_Dict[carID].id)
                            self.Ego.petAV = petAV

                    # 计算间隙
                    # 计算间隙和碰撞之前要先计算pet因为，需要pet来获得哪些是冲突车辆，来筛选
                    if self.conflictCarIdList:
                        # 暂时不计算gap了
                        pass
                        # calcuGapMain(self.conflictCarIdList, self.lastConflictTime, senceIinteractionScore, self.sceneData['simuTime'], self.svAllPosList, self.Ego)
                        # 到这里已经计算出来了ego遇到的间隙，并且把间隙都存到ego里面了
                    self.svAllPosList = []
                    # 这里立刻清空一下，这一步很重要，保证只用一帧的数据来计算
                    self.conflictCarIdList = []
                    self.lastConflictTime.append(self.sceneData['usedTime'])

                    # 计算TTC
                    # 如果后面要改ttc的算法的话就要从这里进去，在Ego.processEvaluation里面改，现在的计算方式，就之间算ttc车辆和自己的距离，然后直接粗暴的用速度差相比
                    # 后面可以用其他方式来计算这个距离，从而计算ttc
                    # 这里已经把ttc车辆更新过了，就是最近的那个车，现在只需要计算
                    # 如歌self.TTC_Car.id为空，证明当前没有合适的ttc车辆，ttc值为10
                    if self.TTC_Car.id:
                        EvaluateData = self.Ego.processEvaluation(self.Ego.bound, self.TTC_Car.bound, self.TTC_Car.speed, self.TTC_Car.id)
                    else:
                        self.Ego.TTC = 10
                    # 在后面的这个函数里，会自动计算ttc 的值并且放在ego.ttc里面，后面我们自己要用的时候就会直接调用ego.ttc了
                    self.Ego.TTC_statistics(self.Ego.accuracy)
                    # 在这里先把senceIselfStateScore实例化，然后用一个总体的计算函数，来计算当前帧的状态信息
                    senceIinteractionScore = interactionScore()
                    senceIinteractionScore.calcuOverTTC(self.Ego.TTC)
                    # print(self.TTC_Car.id, self.Ego.TTC)

                    # 后面就是把参数记录进Ego里面，用于时间的累加计算
                    self.Ego.scoreDict['TTC'] = senceIinteractionScore.overTTC

                    # 真正开始计算评分
                    self.Ego.allScore_statistics(self.Ego.accuracy, self.sceneData['simuTime'])
                    #print(self.Ego.scoreAccuNumDict)

                    # 到这里算是吧所有的评分都放进来了
                    # 最后用这个函数来计算最后的结果
                    senceIscore, totallSenceIscore = self.Ego.calcuAllScore(self.sceneData['usedTime'], self.sceneData['missonExpectTime'], self.sceneIdNoneZero, self.sceneData['sceneWeight'], self.allWeight)

                    # 这里要分清楚，哪些指标是要实时全部计算的，哪些信息是要根据场景，有场景才能计算的
                    senceIscore['useTime'] = self.sceneData['usedTime']
                    senceIscore['avgSpeedNotScore'] = self.Ego.sceneAvgSpeed
                    # 拿到这些细节的分数后，还要进行一步操作，就是处理一下，sceneData['sceneState']为0表示为完成，还在测试，是1的时候才开始
                    # 场景结束后判断后续内容
                    if self.sceneData['sceneState']:
                        senceIscore['missionAccomplish'] = 1
                        a = 100
                        b = 100
                        if not self.senceScoreDital:
                            a = 1
                            b = 1
                        for iscore in self.senceScoreDital:
                            if iscore['senceID'] == self.sceneData['sceneID']:
                                pass
                            else:
                                a = 2
                        if a == 1 or a == 2:
                            self.senceScoreDital.append(senceIscore)
                            a = 0
                        for jscore in self.senceScoreTotal:
                            if jscore['senceID'] == self.sceneData['sceneID']:
                                pass
                            else:
                                b = 2
                        if b == 1 or b == 2:
                            self.senceScoreTotal.append(totallSenceIscore)
                            b = 0
                        self.sceneData['sceneID'] = 0
                        # 在这里id等于0的时候证明场景结束了，就要在ego里面把他积累的一些数值全部归零,特别是ttc和什么别的值
                        self.Ego.clearAllEgoData()
                        # 场景结束的时候还要把sv的轨迹点和冲突时间节点清零
                        self.svAllPosList = []
                        self.conflictCarIdList = []
                        self.lastConflictTime = []
                        # 场景结束还要把sv的字典清空一下，不然最后一个场景还有第一个场景的sv车的数据就很怪
                        self.svAllPosDict = {}
                        # 还有吧单场景的折线图数据清空
                        self.chartDataSingleScene = {}
                        self.xTime = 0
                        #print('场景结束')
                        #print(self.senceScoreDital)
                        #print(self.senceScoreTotal)


                    # 在这里还要把数据写成对应的格式
                    senceIscoreDetail = evaluationDetailDataProcess(senceIscore, totallSenceIscore, self.sceneData['sceneID'], self.sceneData['usedTime'], self.Ego, self.sceneData['sceneWeight'], self.sceneData['missonExpectTime'], self.allWeight)
                    senceIscoreDetail["taskID"] = self.caseID
                    senceIscoreDetail["avName"] = self.avName
                    # 首先要做的是，找出哪些数据是要处理的
                    if self.sceneData['usedTime'] > self.xTime:
                        self.chartDataSingleScene = getChartData(senceIscoreDetail, self.xTime, self.chartDataSingleScene, self.chartDataSingleScene)
                        self.xTime += self.granularity
                        self.chartDataTotalScene[self.sceneData['sceneID']] = self.chartDataSingleScene

                    # 这个就是要发送的实时数据了
                    # print("实时评价数据：", senceIscoreDetail)
                    wsh+=1
                    verticalA += abs(senceIscoreDetail['verticalA'])
                    verticalAPlus += abs(senceIscoreDetail['verticalAPlus'])
                    horizontalA += abs(senceIscoreDetail['horizontalA'])
                    horizontalAPlus += abs(senceIscoreDetail['horizontalAPlus'])
                    turnA += abs(senceIscoreDetail['turnA'])
                    # allRedis.set(sceneIscoreKey, json.dumps(senceIscoreDetail))
                    # print(message.value['timeStamp'], time.time())
                    # producer.send('TJTestSceneIscore', value=json.dumps(senceIscoreDetail).encode('utf-8'))
                    # 这里要用redis来发送数据了

            # producer.send(topic_name2, value=json.dumps(value).encode('utf-8'))

                # 最后再加一个数据，如果全部的场景都结束了'sceneAllState'会变成1，这里只需要计算一次，而且后面再也不会有了
                if 'sceneAllState'in self.sceneData and self.sceneData['sceneAllState'] and not self.allSceneEndLock:
                    # 记录场景的结束时间
                    self.missonEndTime = str(datetime.now())[:23]
                    self.missonEndTimeSimu = self.sceneData['simuTime']
                    #print("场景结束时间", self.missonEndTime, self.missonEndTimeSimu)
                    if self.missonEndTimeSimu - self.missonStartTimeSimu:
                        self.testDuration = self.missonEndTimeSimu - self.missonStartTimeSimu
                    else:
                        self.testDuration = 9999
                    self.allSceneEndLock = 1

                    totalScore_safe = 0
                    totalScore_efficiency = 0
                    totalScore_comfortable = 0
                    # 根据清除信号，等于1表示，跑完一圈了，可以进行清除车辆，并且输出所有指标了
                    for totallSenceIJustScore in self.senceScoreTotal:
                        # weight = self.senceWeight[totallSenceIscore['senceID']]
                        weight = 1
                        totalScore_safe += weight * totallSenceIJustScore['safe']
                        totalScore_efficiency += weight * totallSenceIJustScore['efficiency']
                        totalScore_comfortable += weight * totallSenceIJustScore['comfortable']

                    # self.totalScore = {'安全': totalScore_safe, '效率': totalScore_efficiency, '舒适': totalScore_comfortable}
                    totalScore = {'safe': totalScore_safe, 'efficiency': totalScore_efficiency,
                                       'comfortable': totalScore_comfortable, 'allSenseScore': self.senceScoreTotal,
                                       'allSenseScoreDital': self.senceScoreDital}

                    # 数据处理
                    outPutData = evaluationDataProcess(self.Ego, totalScore, self.missonExpectTime, self.senceWeight, self.missonStartTime, self.missonEndTime, self.testDuration, self.allWeight, self.chartDataTotalScene)

                    # 这里要用之前拿到的数据outputdata来判断一下写出哪些诊断语句库
                    # 这里专门在写一个类写诊断语句
                    diagnoseLibrary = diagnose(outPutData)
                    diagnoseLibrary.getSuggestion()
                    outPutData["diagnose"] = diagnoseLibrary.diagnoseSuggestionDict
                    diagnoseLibrary = None
                    outPutData["taskID"] = self.caseID
                    outPutData["avName"] = self.avName
                    finalKey = self.totalScoreKey + ":" + str(self.caseID)
                    road_type = mapJsonInforDict['header_info']['name']
                    #print(outPutData)
                    # allRedis.set(finalKey, json.dumps(outPutData))
                    # producer2.send('TJTestTotalScore', value=json.dumps(outPutData).encode('utf-8'))
                    # 这里要用redis来发送数据了
                    # 打开 CSV 文件进行写入
                    if self.detail_csv_path is not None:
                        with open(self.detail_csv_path, mode="a", newline="", encoding="utf-8") as file:
                            # a表示追加模式
                            # 创建 CSV writer 对象
                            csv_writer1 = csv.writer(file)
                            # result = [self.taskName, outPutData["AbilityDimension"]["safe"]*0.5, outPutData["AbilityDimension"]["efficiency"]*0.3, outPutData["AbilityDimension"]["comfortable"]*0.2,
                            #           outPutData["AbilityDimension"]["safe"]*0.5+outPutData["AbilityDimension"]["efficiency"]*0.3+outPutData["AbilityDimension"]["comfortable"]*0.2]
                            result1 = [self.taskName, outPutData["AbilityDimension"]["safe"],
                                       outPutData["AbilityDimension"]["efficiency"],
                                       outPutData["AbilityDimension"]["comfortable"],
                                       round(outPutData["AbilityDimension"]["safe"] + outPutData["AbilityDimension"][
                                           "efficiency"] + outPutData["AbilityDimension"]["comfortable"], 2),
                                       (self.taskName.split("_")[0]).lower(),road_type,
                                       1,1,0,
                                       outPutData["testDuration"],
                                       outPutData["testScene"][0]['info']['safe'][0]['time'],
                                       outPutData["testScene"][0]['info']['safe'][2]['timeRatio：'],
                                       round(outPutData["testScene"][0]['info']['safe'][1]['time'] / outPutData[
                                           "testDuration"], 2),
                                       round(outPutData["testScene"][0]['info']['safe'][3]['time'] / outPutData[
                                           "testDuration"], 2),
                                       outPutData["testScene"][0]['info']['safe'][6]['time'],
                                       totalScore['allSenseScoreDital'][0]['calMissionAccomplish'],
                                       outPutData["testScene"][0]['info']['efficiency'][2]['avgSpeed'],
                                       round(outPutData["testScene"][0]['info']['comfortable'][0]['time']/ outPutData[
                                           "testDuration"], 2),
                                       round(outPutData["testScene"][0]['info']['comfortable'][1]['time']/ outPutData[
                                           "testDuration"], 2),
                                       round(outPutData["testScene"][0]['info']['comfortable'][2]['time']/ outPutData[
                                           "testDuration"], 2),
                                       round(outPutData["testScene"][0]['info']['comfortable'][3]['time']/ outPutData[
                                           "testDuration"], 2),
                                       round(outPutData["testScene"][0]['info']['comfortable'][4]['time'] / outPutData[
                                           "testDuration"], 2)
                                       ]
                            csv_writer1.writerow(result1)

                    # 打开 CSV 文件进行写入
                    if self.score_csv_path is not None:
                        with open(self.score_csv_path, mode="a", newline="", encoding="utf-8") as file:
                            # a表示追加模式
                            # 创建 CSV writer 对象
                            csv_writer2 = csv.writer(file)
                            # result = [self.taskName, outPutData["AbilityDimension"]["safe"]*0.5, outPutData["AbilityDimension"]["efficiency"]*0.3, outPutData["AbilityDimension"]["comfortable"]*0.2,
                            #           outPutData["AbilityDimension"]["safe"]*0.5+outPutData["AbilityDimension"]["efficiency"]*0.3+outPutData["AbilityDimension"]["comfortable"]*0.2]
                            result2 = [self.taskName, outPutData["AbilityDimension"]["safe"],
                                       outPutData["AbilityDimension"]["efficiency"],
                                       outPutData["AbilityDimension"]["comfortable"],
                                       round(outPutData["AbilityDimension"]["safe"] + outPutData["AbilityDimension"][
                                           "efficiency"] + outPutData["AbilityDimension"]["comfortable"], 2)
                                       ]
                            csv_writer2.writerow(result2)

                    # 这里开始清除数据
                    # 在这里把一圈的场景跑完后，需要把所有的内容都清空一遍
                    self.claerAllData()
        # 计算输出score_detail.csv需要的加速度值
        verticalA = round(verticalA / wsh, 2)
        verticalAPlus = round(verticalAPlus / wsh, 2)
        horizontalA = round(horizontalA / wsh, 2)
        horizontalAPlus = round(horizontalAPlus / wsh, 2)
        turnA = round(turnA / wsh, 2)
        new_data = [horizontalA, horizontalAPlus, verticalA, verticalAPlus, turnA]
        # 再次打开csv,写入
        if self.detail_csv_path is not None:
            with open(self.detail_csv_path, mode='r', newline='') as file:
                reader = csv.reader(file)
                lines = list(reader)
            # 在最后一行添加新列数据
            lines[-1].extend(new_data)
            # 写回CSV文件
            with open(self.detail_csv_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(lines)

        return outPutData

    def claerAllData(self):
        # 只需要实例化一次ego车辆，后面每次循环都更新这个车辆就行了
        self.Ego = SelfDrivingCar(id=0, Xpos=0, Ypos=0, speed=0, roadType=0, startPos=0)
        self.TTC_Car: TTCCar = TTCCar(id=0, Xpos=0, Ypos=0, speed=0, roadType=0)

        # sv车辆是需要存一下的，每次实例化都要存一下，同理也要pet的车
        self.SV_car_Dict = {}
        self.PET_car_Dict = {}

        # 这个还是一个大字典，每个字典对应一个车，每个车都要实例化后再存入这个列表，或者更新替换，或者创建一个新车

        # 记录一些冲突的指标
        self.conflictCarIdList = []
        self.lastConflictTime = []

        # 用来存储上一帧的数据
        self.lastSceneData = []
        self.lastAVdata = {}
        # 这里只放sv的id，主要是间隙计算的时候用
        self.svAllPosList = []

        # 下面这些是用来存放从卡夫卡拉下来的数据，形成自己的轨迹，后面计算都是依据这些来的
        self.sceneData = {}
        self.avPosList = []
        # 所有的sv的车辆轨迹列表，这里可以用明阳的建议用字典来做
        # 每个键是id是一辆车，键值是一个列表，里面存放的是self.svIPosList，每次拿到数据就要更新一遍
        self.svAllPosDict = {}
        # 每次场景结束就清空一次吧，不然测最后一个场景的时候还保留第一个场景的数据就太怪了

        # 记录场景的得分
        self.senceScoreDital = []
        self.senceScoreTotal = []
        self.totalScore = {}

        # 记录场景id，这个id不会是0，用于数据传输
        self.sceneIdNoneZero = 0

        # 最后场景全部结束要计算唯一一次
        self.allSceneEndLock = 0
        self.allSceneStartLock = 0

        # 记录场景的权重和期望时间，每次场景更新这里就更新添加一下就行
        self.senceWeight = {}
        self.missonExpectTime = {}

        # 记录任务的开始结束时间
        self.missonStartTime = 0
        self.missonEndTime = 0
        self.missonStartTimeSimu = 0
        self.missonEndTimeSimu = 0
        self.testDuration = 0

        self.fristSceneTrueStartTimeStamp = 0
        self.isFirstSceneState = 0
        # 记录每个场景开始的时间，每次场景结束都会主动清零
        self.sceneStartTimeStamp = 0
        self.sceneState = 0
        self.sceneAllState = 0
        self.currSceneId = 0
        self.currSceneIdNoneZero = 0
        # 第一帧标记，在遇到开始的时候，把这个标记变成1，如果标记为1则记录时间为开始时间，记录后标记清零
        self.isSceneState = 0
        # 这个不用清零，他会自己变化
        self.startType = 0

        # 静态信息
        self.AVchannelName = None
        self.SVchannelName = None
        self.carChannelName = {"AVchannelName": None, "SVchannelName": None}
        # 静态场景信息
        self.totalSceneNum = 0
        self.caseID = None
        self.avName = None
        self.taskName = None
        self.sceneTypeList = []
        self.sceneExpectTimeList = []
        # 静态权重信息
        self.weightData = {}
        self.allWeight = None

        # 任务开始标记
        self.missonStart = 0  # 0表示还没有开始任务，1表示已经开始了，在任务结束后重新赋值为0

        # 储存一下av的上一帧数据，防止av结束end标记的时候，value是空的，
        self.AVdataLast = None
        self.SVdataLast = None

        # 存放折线图的数据
        self.chartDataSingleScene = {}
        self.chartDataTotalScene = {}
        self.xTime = 0

        # 用来存放车辆的数据，这个需要再开始的时候清空，但是3.7以上才有clear函数，所以要写一个清空的函数
        self.threadAV = None
        self.threadSV = None


def renewCarState(veh, posListInfo):
    veh.posList = posListInfo
    veh.realpos = posListInfo[-1]['realpos']
    veh.Xpos = posListInfo[-1]['x']
    veh.Ypos = posListInfo[-1]['y']
    veh.currentPos = [posListInfo[-1]['x'], posListInfo[-1]['y']]
    veh.speed = posListInfo[-1]['speed']
    veh.acce = posListInfo[-1]['acce']
    veh.angle = posListInfo[-1]['angle']
    veh.vehiHeadwayFront = posListInfo[-1]['HeadwayFront']
    veh.vehiDistFront = posListInfo[-1]['DistFront']

    # 这里更新车辆的长度和宽度信息，如果提供了每个车的长度宽度信息，则更新，否则就不更新了
    if "width" in posListInfo[-1] and "length" in posListInfo[-1]:
        if posListInfo[-1]["length"] and posListInfo[-1]["width"]:
            veh.length = posListInfo[-1]["length"]
            veh.width = posListInfo[-1]["width"]
        else:
            veh.length = Parameter.length
            veh.width = Parameter.width
    else:
        veh.length = Parameter.length
        veh.width = Parameter.width


def avStateScoreCalcu(veh, SVALL_cars_info, senceStartId, mapJsonInforDict, PETCarIdList, aimPathJsonInforDict,sceneID, signalState, indexWeight):
    senceIselfStateScore = selfStateScore(veh, SVALL_cars_info, senceStartId, mapJsonInforDict, PETCarIdList,
                                          aimPathJsonInforDict, sceneID, signalState, indexWeight)
    # 不用传参不用return，都在内部记录好了
    senceIselfStateScore.calcuAllScore()
    # 后面就是把参数记录进Ego里面，用于时间的累加计算
    veh.scoreDict['驶出行车道'] = senceIselfStateScore.outOfLane
    veh.scoreDict['驶入对向车道'] = senceIselfStateScore.inSubtendRoad
    veh.scoreDict['压实线'] = senceIselfStateScore.onLaneMarking
    veh.scoreDict['超速'] = senceIselfStateScore.overSpeed
    veh.scoreDict['闯红灯'] = senceIselfStateScore.breakSignal
    veh.scoreDict['纵向加加速度'] = senceIselfStateScore.overJerk
    veh.scoreDict['纵向加速度'] = senceIselfStateScore.overAcce
    veh.scoreDict['横向加加速度'] = senceIselfStateScore.overLateralJerk
    veh.scoreDict['横向加速度'] = senceIselfStateScore.overLateralAcce
    veh.scoreDict['转向角变化'] = senceIselfStateScore.unstableSteeringAngle
    # 新加四个静态计算指标：# 横向间距违规、# 未按规定车道行驶、# 停车压停止线、# 在禁行区行驶
    veh.scoreDict['横向间距'] = senceIselfStateScore.lateral_spacing_time
    veh.scoreDict['禁行区行驶'] = senceIselfStateScore.driving_in_restricted_area_flag
    veh.scoreDict['未按规定车道行驶'] = senceIselfStateScore.driving_in_designated_lane_time
    veh.scoreDict['停车压停止线'] = senceIselfStateScore.stop_at_the_stop_line_count
    veh.scoreDict['未按规定路线行驶'] = senceIselfStateScore.driving_in_designated_path
    veh.scoreDict['是否完成任务'] = senceIselfStateScore.missionAccomplish
    veh.acceAll = senceIselfStateScore.acceRead()
    # 这里是计算的指标，还有一些没有计算的指标，比如横向间距，禁行区，闯红灯等等，所以如果要后面计算新的指标，就在这里做对应的操作就行


def calcuGapMain(conflictCarIdList, lastConflictTimeList, senceIinteractionScore, simuTime, conflictCarPostList, Ego):
    # 在场景内部开始计算gap
    # 如果出现了self.conflictCarIdList有数据，证明开始冲突了，这里就要开始记录冲突时间了
    # print(self.conflictCarIdList)
    if conflictCarIdList:
        # print(self.lastConflictTime)
        # 记录的冲突时间
        if lastConflictTimeList:
            lastConflictTime = lastConflictTimeList[-1]
        else:
            lastConflictTime = 0
        # 如果当前时间大于冲突时间+0.1秒则开始计算后面的东西，保证有充足的时间来添加后面的所有有冲突的车辆
        # if lastConflictTime + 0.1 < simuTime / 1000:
        # 第一步要根据存储的conflictCarIdList，找到他们的轨迹，这一步好像在上面就可以完成，在sv车辆的时候
        # 第二步就和之前一样了
        # print(self.iSceneConflictCarPostList)
        gap = senceIinteractionScore.calcuGap(lastConflictTime, simuTime, Ego.petAV, Ego.posList,
                                              conflictCarPostList)
        Ego.gapAllList.append(gap)
        Ego.gapRecordTimeList.append(simuTime)
        Ego.calcuGapList()
        Ego.petAV = 0


def dataTrans(AV_cars_info, SVALL_cars_info, scene_info):
    pass

    # 现在就拿到数据了
    # self.avPosList是一个列表，列表里面有n个字典，每个字典是一帧的数据，最后一个数据是当前的车辆状态
    # self.svAllPosDict是一个大字典，大字典里面有m个列表，每个列表是一个车辆的数据，每个列表里面有n个字典，每个字典是一帧的数据，最后一个数据是m辆车中的第i辆当前的车辆状态
    # self.sceneData还没有想好存什么，一会用什么存什么，用字典来存，主要包括：期望用时、场景编号、当前运行时间、权重、是否已完成


def isSceneEnd(lastSceneData):
    if len(lastSceneData) > 1 and 'sceneState' in lastSceneData[-1]:
        # 0是之前的数据，-1是最后一帧的数据
        if lastSceneData[-1]['sceneState'] == 1 and lastSceneData[0]['sceneState'] == 0:
            # if lastSceneData[-1]['sceneState'] != lastSceneData[0]['sceneState']:
            return True
    return False


def isSceneNew(lastSceneData):
    if len(lastSceneData) > 2:
        # 0是之前的数据，-1是最后一帧的数据
        if lastSceneData[-1]['senceID'] != lastSceneData[0]['senceID']:
            return True
    return False


# 吧值和x轴放在对应的图表里面
def iNameChartData(iValue, x, ChartBeforeProcess):
    ChartBeforeProcess["x"].append(x)
    ChartBeforeProcess["y"].append(round(iValue, 2))
    iNameChart = ChartBeforeProcess
    return iNameChart


# 处理折线图的数据
def getChartData(data, x, chartDataBeforeProcess, chartDataSingleScene):
    chartData = {}
    # 一共6个表，横向、纵向、加速度、加加速度、ttc、转向转向角
    # TTCChart = {"x": [], "y": []}
    # TTC = data["TTC"]
    # TTCChartBeforeProcess = chartDataBeforeProcess["TTC"]
    # TTCChart = iNameChartData(TTC, x, TTCChartBeforeProcess)
    # chartData["TTC"] = TTCChart
    # 第一个表TTC
    # 第二个表verticalA，纵向加速度
    # 第三个表verticalAPlus，纵向加加速度
    # 第四个表horizontalA，横向加速度
    # 第五个表horizontalAPlus，横向加加速度
    # 第六个表turnA，转向角

    # 整个列表for循环六次
    nameList = ["TTC", "verticalA", "verticalAPlus", "horizontalA", "horizontalAPlus", "turnA"]
    for iName in nameList:
        iValue = data[iName]
        if iName not in chartDataSingleScene:
            chartData[iName] = {"x": [], "y": []}
        else:
            iChartBeforeProcess = chartDataBeforeProcess[iName]
            iChart = iNameChartData(iValue, x, iChartBeforeProcess)
            chartData[iName] = iChart

    return chartData


def evaluationDetailDataProcess(senceIscore, totallSenceIscore, senceStartId, lastTime, Ego, senceWeight,
                                missonExpectTime, allweight):
    weight = senceWeight

    if senceStartId != 0:
        senceStatus = 0
        senceScore = None
    else:
        senceStatus = 1
        senceScore = str(round(
            (totallSenceIscore['safe'] + totallSenceIscore['efficiency'] + totallSenceIscore['comfortable']) / weight,
            2)) + '/' + '100'

    data = {"senceID": senceIscore['senceID'],
            "senceStatus": senceStatus,
            "senceAccomplish": senceIscore["calMissionAccomplish"],
            "senceScore": senceScore,
            "safeScore": 100 * allweight.scoreWeight['safe'],
            "safeMinusScore": 100 * allweight.scoreWeight['safe'] - totallSenceIscore['safe'] / weight,
            # 'safeScore': 100,
            # 'safeMinusScore': 100 - totallSenceIscore['safe']/weight,
            # 这里是因为，分数计算的时候按20%计算安全，实际显示的时候按照100分来显示
            "TTC": Ego.TTC,
            "avgSpeedAll": Ego.avgSpeed,
            "avgSpeed": Ego.sceneAvgSpeed,
            "efficiencyScore": 100 * allweight.scoreWeight['efficiency'],
            "efficiencyMinusScore": 100 * allweight.scoreWeight['efficiency'] - totallSenceIscore[
                'efficiency'] / weight,
            "taskAlreadyTime": lastTime,
            "taskTime": missonExpectTime,
            "comfortableScore": 100 * allweight.scoreWeight['comfortable'],
            "comfortableMinusScore": 100 * allweight.scoreWeight['comfortable'] - totallSenceIscore[
                'comfortable'] / weight,
            "verticalA": Ego.acceAll['verticalA'],
            "verticalAPlus": Ego.acceAll['verticalAPlus'],
            "horizontalA": Ego.acceAll['horizontalA'],
            "horizontalAPlus": Ego.acceAll['horizontalAPlus'],
            "turnA": Ego.acceAll['turnA'],
            "turnAPlus": Ego.acceAll['turnA']}
    return data


def clearZero(dict):
    for key in dict:
        if not dict[key]:
            dict[key] = 0.01
        else:
            pass
    return dict


def gradeScore(a, b, c, d, score):
    grade = None
    if score > a:
        grade = "优秀"
    elif score > b:
        grade = "良好"
    elif score > c:
        grade = "一般"
    elif score > d:
        grade = "较差"
    else:
        grade = "不合格"
    return grade


def evaluationDataProcess(Ego, totalScore, missonExpectTime, senceWeight, missonStartTime, missonEndTime, testDuration,
                          allWeight, chartDataTotalScene):
    allSence = []
    num = 2
    AbilityDimension = {"safe": 0,
                        "efficiency": 0,
                        "comfortable": 0,
                        "else": 0, }  # 百分制
    clearZero(allWeight.scoreWeight)
    clearZero(allWeight.comfortableDitalWeight)
    clearZero(allWeight.safeDitalWeight)
    clearZero(allWeight.efficiencyDitalWeight)
    gradeTableDict = {}

    for i in range(len(totalScore['allSenseScore'])):
        sceneItotalScore = totalScore['allSenseScore'][i]  # 百分制了
        sceneIDitalScore = totalScore['allSenseScoreDital'][i]  # 这里的scoreDital就是senceScoreEnglish，autodriving里面最后一段代码一大堆那个
        efficiency = [{"index": "任务完成", "score": str(sceneIDitalScore['calMissionAccomplish'] * allWeight.efficiencyDitalWeight["missionAccomplish"] * allWeight.scoreWeight['efficiency']) + "/" + str(
                          allWeight.efficiencyDitalWeight["missionAccomplish"] * allWeight.scoreWeight['efficiency']),
                       "time": round(sceneIDitalScore['useTime'], num)},
                      {"index": "任务耗时",
                       "score": str(
                           min(allWeight.efficiencyDitalWeight["usedTime"] * allWeight.scoreWeight['efficiency'], round(
                               (missonExpectTime[sceneItotalScore['senceID']] / sceneIDitalScore['useTime']) *
                               allWeight.scoreWeight['efficiency'] * allWeight.efficiencyDitalWeight["usedTime"],
                               num))) + "/" + str(
                           round(allWeight.efficiencyDitalWeight["usedTime"] * allWeight.scoreWeight['efficiency'],num)),
                       "time": round(sceneIDitalScore['useTime'], num),
                       "expectTime": missonExpectTime[sceneItotalScore['senceID']],
                       "overTime": round((sceneIDitalScore['useTime'] - missonExpectTime[sceneItotalScore['senceID']]) /
                                         missonExpectTime[sceneItotalScore['senceID']], num)},
                      {"index": "平均速度", "score": str(round(
                          (min(1, sceneIDitalScore['avgSpeedNotScore'] / Parameter.avgSpeed)) *
                          allWeight.efficiencyDitalWeight["averageSpeed"] * allWeight.scoreWeight['efficiency'],
                          num)) + "/" + str(
                          allWeight.efficiencyDitalWeight["averageSpeed"] * allWeight.scoreWeight['efficiency']),
                       "avgSpeed": sceneIDitalScore['avgSpeedNotScore']}, ]

        comfortable = [
            {"index": "横向舒适度", "score": "-" + str(round(sceneIDitalScore['overLateralAcce'], num)), "type": "a",
             "time": sceneIDitalScore['overLateralAcceNum']},
            {"index": "横向舒适度", "score": "-" + str(round(sceneIDitalScore['overLateralJerk'], num)), "type": "j",
             "time": sceneIDitalScore['overLateralJerkNum']},
            {"index": "纵向舒适度", "score": "-" + str(round(sceneIDitalScore['overAcce'], num)), "type": "a",
             "time": sceneIDitalScore['overAcceNum']},
            {"index": "纵向舒适度", "score": "-" + str(round(sceneIDitalScore['overJerk'], num)), "type": "j",
             "time": sceneIDitalScore['overJerkNum']},
            {"index": "转弯舒适度", "score": "-" + str(round(sceneIDitalScore['unstableSteeringAngle'], num)),
             "type": "a", "time": round(sceneIDitalScore['unstableSteeringAngleTime'], num)},
            {"index": "转弯舒适度", "score": "0", "type": "j", "time": 0}]
        if sceneIDitalScore['crash'] > 0.05:
            crash = 100
        else:
            crash = 0
        safe = [{"index": '碰撞', "score": "-" + str(round(crash, num)), "time": round(
            sceneIDitalScore['crash'], num)},
                {"index": '驶出行车道', "score": "-" + str(round(sceneIDitalScore['outOfLane'], num)), "time": round(
                    sceneIDitalScore['outOfLane'] * sceneIDitalScore['useTime'] / (
                                allWeight.safeDitalWeight["outOfLane"] * allWeight.scoreWeight['safe']), num)},
                {"index": 'TTC', "score": "-" + str(round(sceneIDitalScore['TTC'], num)), "time": round(
                    sceneIDitalScore['TTC'] * sceneIDitalScore['useTime'] / (
                                allWeight.safeDitalWeight["TTC"] * allWeight.scoreWeight['safe']), num),
                 "timeRatio：": round(
                     sceneIDitalScore['TTC'] / (allWeight.safeDitalWeight["TTC"] * allWeight.scoreWeight['safe']),
                     num), },
                {"index": '驶入对向车道', "score": "-" + str(round(sceneIDitalScore['inSubtendRoad'], num)),
                 "time": round(sceneIDitalScore['inSubtendRoad'] * sceneIDitalScore['useTime'] / (
                             allWeight.safeDitalWeight["inSubtendRoad"] * allWeight.scoreWeight['safe']), num)},
                {"index": '压实线', "score": "-" + str(round(sceneIDitalScore['onLaneMarking'], num)), "time": round(
                    sceneIDitalScore['onLaneMarking'] * sceneIDitalScore['useTime'] / (
                                allWeight.safeDitalWeight["onLaneMarking"] * allWeight.scoreWeight['safe']), num)},
                {"index": '超速', "score": "-" + str(round(sceneIDitalScore['overSpeed'], num)), "time": round(
                    sceneIDitalScore['overSpeed'] * sceneIDitalScore['useTime'] / (
                                allWeight.safeDitalWeight["overSpeed"] * allWeight.scoreWeight['safe']), num)},
                {"index": '闯红灯', "score": "-" + str(round(sceneIDitalScore['breakSignal'], num)), "time": sceneIDitalScore['breakSignalNum']},
                {"index": "横向间距", "score": "-" + str(round(sceneIDitalScore['transverseDistance'], num)),
                 "time": round(sceneIDitalScore['transverseDistance'] * sceneIDitalScore['useTime'] / (
                             allWeight.safeDitalWeight["transverseDistance"] * allWeight.scoreWeight['safe']), num)},
                {"index": "禁行区行驶", "score": "-" + str(round(sceneIDitalScore['inForbiddenArea'], num)),
                 "time": round(sceneIDitalScore['inForbiddenArea'] * sceneIDitalScore['useTime'] / (
                             allWeight.safeDitalWeight["inForbiddenArea"] * allWeight.scoreWeight['safe']), num)},
                {"index": "未按规定车道行驶",
                 "score": "-" + str(round(sceneIDitalScore['drivingInDesignatedLane'], num)), "time": round(
                    sceneIDitalScore['drivingInDesignatedLane'] * sceneIDitalScore['useTime'] / (
                                allWeight.safeDitalWeight["drivingInDesignatedLane"] * allWeight.scoreWeight['safe']),
                    num)},
                {"index": "停车压停止线", "score": "-" + str(round(sceneIDitalScore['stopAtStopLine'], num)),
                 "time": round(sceneIDitalScore['stopAtStopLine'] * sceneIDitalScore['useTime'] / (
                             allWeight.safeDitalWeight["stopAtStopLine"] * allWeight.scoreWeight['safe']), num)}, ]
        AbilityDimensionI = {}
        AbilityDimensionI["safe"] = round(
            sceneItotalScore['safe'] / (senceWeight[sceneItotalScore['senceID']] * allWeight.scoreWeight['safe']), num)
        AbilityDimensionI["comfortable"] = round(sceneItotalScore['comfortable'] / (
                    senceWeight[sceneItotalScore['senceID']] * allWeight.scoreWeight['comfortable']), num)
        AbilityDimensionI["efficiency"] = round(sceneItotalScore['efficiency'] / (
                    senceWeight[sceneItotalScore['senceID']] * allWeight.scoreWeight['efficiency']), num)
        testSceneI = {"senceID": sceneItotalScore['senceID'],
                      "sceneNameList": allWeight.sceneTypeList[i],
                      "chartData": chartDataTotalScene[sceneItotalScore['senceID']],
                      "AbilityDimension": AbilityDimensionI,
                      "missionAccomplish": sceneIDitalScore['missionAccomplish'],
                      "senseScore": round((sceneItotalScore['safe'] + sceneItotalScore['efficiency'] + sceneItotalScore[
                          'comfortable']) / senceWeight[sceneItotalScore['senceID']], num),
                      "senseScore*senseWeight": round(
                          (sceneItotalScore['safe'] + sceneItotalScore['efficiency'] + sceneItotalScore['comfortable']),
                          num),
                      "senseAggScore": round(senceWeight[sceneItotalScore['senceID']] * 100, num),
                      "senseWeight": round(senceWeight[sceneItotalScore['senceID']] * 100, num),
                      "info": {"efficiency": efficiency, "comfortable": comfortable, "safe": safe},
                      "eventTable": sceneIDitalScore["eventTable"], }
        allSence.append(testSceneI)

        # 计算能力雷达图
        AbilityDimension["safe"] += sceneItotalScore['safe']
        AbilityDimension["efficiency"] += sceneItotalScore['efficiency']
        AbilityDimension["comfortable"] += sceneItotalScore['comfortable']

        # 计算分级表格
        safeGrade = gradeScore(98, 95, 90, 80, sceneItotalScore['safe'] / (
                    allWeight.scoreWeight['safe'] * senceWeight[sceneItotalScore['senceID']]))
        comfortableGrade = gradeScore(95, 88, 75, 60, sceneItotalScore['comfortable'] / (
                    allWeight.scoreWeight['comfortable'] * senceWeight[sceneItotalScore['senceID']]))
        efficiencyGrade = gradeScore(90, 80, 70, 60, sceneItotalScore['efficiency'] / (
                    allWeight.scoreWeight['efficiency'] * senceWeight[sceneItotalScore['senceID']]))
        gradeTableDict[allWeight.sceneTypeList[i]] = {"安全性": safeGrade, "舒适性": comfortableGrade,
                                                      "效率性": efficiencyGrade, }

    allSenseScore = 0
    for i in allSence:
        allSenseScore += i['senseScore*senseWeight']

    for key in AbilityDimension:
        if key in allWeight.scoreWeight:
            AbilityDimension[key] = round(AbilityDimension[key] , num)

    data = {'startTime': missonStartTime,
            'endTime': missonEndTime,
            'testDuration': round(testDuration, num),
            'dangerTimeProportion': round(Ego.TTC_LowTime / testDuration, num),
            'allSenseScore': round(allSenseScore, num),  # 这个是总分
            'testScene': allSence,
            "AbilityDimension": AbilityDimension,  # 这个也是总分百分制的能力雷达图
            "gradeTableDict": gradeTableDict, }  # 这个是分级的表格

    return data


def getState(redisWanJIAddress, redisWanJIPort, channelStart, scene_queue, channel_queue, state_queue):
    #print(channelStart, '线程开启')
    allRedis = redis.StrictRedis(host=redisWanJIAddress, port=redisWanJIPort, db=0, password="Wanji@300552!")
    pubsub = allRedis.pubsub()
    pubsub.subscribe(channelStart)
    for message in pubsub.listen():
        # print(message)
        if message['type'] == 'message':
            data_str = message["data"].decode("utf-8")
            listenMsg = json.loads(data_str)
            if type(listenMsg) == dict:
                if "deviceId" in listenMsg:
                    deviceId = listenMsg["deviceId"]
                    type1 = listenMsg["type"]
                    timeStamp = listenMsg["timestamp"]
                # 又该协议了，type要先判断，如果是1表示下发任务，如果是2表示下发通道
                if "type" in listenMsg:
                    # 如果任务等于1就表示找场景数据
                    if listenMsg["type"] == 1:
                        scene_queue.put(listenMsg)
                        # 同时如果监听到这个数据，就代表要重新开始一把了，需要重启后面的各个线程和清空数据
                    # 如果任务等于2就表示找通道
                    elif listenMsg["type"] == 2:
                        channel_queue.put(listenMsg)
                        if "params" in listenMsg:
                            #     temp_dict = dict()
                            #     # listenMsg: {'id': '276', 'type': 1, 'data': '{"taskType":1,"protocols":[{"type":0,"channel":"HMIResult1","params":{}},{"type":1,"channel":"TESSResult","params":{}}]}', 'timestamp': 1696904039563, 'controlChannel': 'TESSControl'}
                            data_str = listenMsg["params"]
                            data_dict = data_str
                            taskType = data_dict["taskType"]
                            state_queue.put(taskType)


def getSVdata(redisWanJIAddress, redisWanJIPort, sv_queue, carChannelName):
    allRedis = redis.StrictRedis(host=redisWanJIAddress, port=redisWanJIPort, db=0, password="Wanji@300552!")
    pubsub = allRedis.pubsub()
    pubsub.subscribe(carChannelName["SVchannelName"])
    # 这里只收一帧的数据就行，需要判断 一下是 场景数据，还是channel数据，如果是场景数据
    #print("开始监听sv车辆信息")

    # 可以这样写，使用getmessage，不会阻塞线程，如果拿不到数据就会一直while循环，这时就可以给他设置一个标志，标志为全局变量，如果这个标志改变了，就while失败，然后主进程写一个.join()就等到线程合并，就算是退出线程了，等待下次循环的时候再开启
    lastcarChannelName = carChannelName["SVchannelName"]
    a = 0
    while True:
        a += 1
        message = pubsub.get_message()
        if message:
            if message['type'] == 'message':
                data_str = message["data"].decode("utf-8")
                data_dict = json.loads(data_str)
                try:
                    sv_queue.put_nowait(data_dict)
                except:
                    if a % 5 == 0:
                        print("sv队列已满")
        # 用whileTRUE一直循环，每次拿数据都正常拿，在进入循环之前，保存了一下频道名称，如果在后面频道名称变化了，这里就直接给他break掉，就走到后面print("准备结束监听sv车辆信息")，这个线程函数也就结束了
        if carChannelName["SVchannelName"] != lastcarChannelName:
            break
    print("准备结束监听sv车辆信息")


def getAVdata(redisWanJIAddress, redisWanJIPort, av_queue, carChannelName):
    allRedis = redis.StrictRedis(host=redisWanJIAddress, port=redisWanJIPort, db=0, password="Wanji@300552!")
    pubsub = allRedis.pubsub()
    pubsub.subscribe(carChannelName["AVchannelName"])
    # 这里只收一帧的数据就行，需要判断 一下是 场景数据，还是channel数据，如果是场景数据
    print("开始监听av车辆信息")

    # 可以这样写，使用getmessage，不会阻塞线程，如果拿不到数据就会一直while循环，这时就可以给他设置一个标志，标志为全局变量，如果这个标志改变了，就while失败，然后主进程写一个.join()就等到线程合并，就算是退出线程了，等待下次循环的时候再开启
    lastcarChannelName = carChannelName["AVchannelName"]
    a = 0
    while True:
        a += 1
        message = pubsub.get_message()
        if message:
            if message['type'] == 'message':
                data_str = message["data"].decode("utf-8")
                data_dict = json.loads(data_str)
                try:
                    av_queue.put_nowait(data_dict)
                except:
                    if a % 5 == 0:
                        print("av队列已满")
        # 用whileTRUE一直循环，每次拿数据都正常拿，在进入循环之前，保存了一下频道名称，如果在后面频道名称变化了，这里就直接给他break掉，就走到后面print("准备结束监听sv车辆信息")，这个线程函数也就结束了
        if carChannelName["AVchannelName"] != lastcarChannelName:
            break
    print("准备结束监听av车辆信息")


# 清空队列的函数
def clear_queue(q):
    try:
        while True:
            q.get_nowait()
    except queue.Empty:
        pass
