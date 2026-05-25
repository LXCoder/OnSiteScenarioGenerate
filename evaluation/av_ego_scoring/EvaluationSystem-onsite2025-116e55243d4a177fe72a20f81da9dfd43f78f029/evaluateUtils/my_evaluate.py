# main函数里主要做的就是把数据读出来，然后把数据传出去
# 这样的话这里整好定义一个函数的接口，传入一些数据（轨迹数据和场景数据）到一个指标评价类里面
# 在写一个输出函数，把类里面计算出来的得分都返回出来
import collections
import os
from multiprocessing import Process, Queue
import sys
import time
import _thread
import queue

import pandas as pd
import redis
import json
import math
import threading
import csv
from datetime import datetime

from evaluateUtils.AutoSelfDrivingCar import SelfDrivingCar, Car, TTCCar, PETCar, np
from evaluateUtils.MySource.scoreStatistic import selfStateScore, interactionScore
from evaluateUtils.standard_parameter import Parameter
from evaluateUtils.config import send_interval, send_topic_mapping, av_radar_distance, redisWanJIAddress, redisWanJIPort
from evaluateUtils.diagnose_library import diagnose
from evaluateUtils.Dynamic_Check import dynamic_check


class evaluationSystem():

    def __init__(self):
        # print("taskName_3", Parameter.taskName)

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

    def changeLimitSpeed(self, sceneName):
        sceneList = []
        try:
            sceneList = sceneName.split("-")
        except:
            try:
                sceneList = sceneName.split(".")
            except:
                changan_scene_id = '1'
        if len(sceneList) == 2:
            changan_scene_id, changan_scene_name = sceneList
        elif len(sceneList) == 3:
            tongji_scene_id, changan_scene_name, changan_scene_id = sceneList
        else:
            changan_scene_id = '1'
        if changan_scene_id in Parameter.avgSpeedKMDict:
            Parameter.avgSpeedKM = Parameter.avgSpeedKMDict[changan_scene_id]
            Parameter.avgSpeed = Parameter.avgSpeedKMDict[changan_scene_id] * 3.6

        if changan_scene_id in Parameter.svSimuTimeWithoutDisturbanceDict:
            Parameter.svSimuTimeWithoutDisturbance = Parameter.svSimuTimeWithoutDisturbanceDict[changan_scene_id]


    # wth:这里是1辆AV车评价计算，外面套一层协调性计算
    def startMyEvaluation(self,
                          data_transfer,
                          originRealTime,
                          trj_path_data=None,
                          map_path_data=None,
                          affected=1,    # 表征车辆是否受到主车影响，不受主车影响（距离较远）则不进行评分计算，只更新车辆信息
                          rule_list={}    # 交规
                         ):
        mapJsonInforDict = map_path_data["mapJsonInforDict"]
        aimPathJsonInforDict = map_path_data["aimPathJsonInforDict"]
        sceneIoutPutDataDict = None

        # wth:初始化
        senceIscore = None
        totallSenceIscore = None
        if data_transfer:
            allNeedData = data_transfer
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

            if isinstance(allNeedData.get('originRealTime'), (float, int)):
                originRealTime.append(allNeedData.get('originRealTime'))

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
            # 这里还要多加一层判断，如果传进来的车这次id少了某个车的，证明这个车在当前不在地图中了，需要把他的轨迹删除掉
            # 消失的车辆
            self.Ego.currentSVcars = currentSVlist
            self.Ego.clearSVinforDict()
            cars_to_remove = []
            for key in self.svAllPosDict.keys():
                if key not in currentSVlist:
                    cars_to_remove.append(key)
            # 删除多余的车辆，后面还有几个地方也要删除，petcar，av里面的那个几个dict，直接在这里加个判断一次性删除
            for key in cars_to_remove:
                del self.svAllPosDict[key]
                if key in self.SV_car_Dict:
                    del self.SV_car_Dict[key]
                if key in self.PET_car_Dict:
                    del self.PET_car_Dict[key]

            # 拿到场景数据
            self.sceneData = scene_info
            self.lastSceneData.append(self.sceneData)
            if len(self.lastSceneData) > 2:
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
                self.allSceneStartLock = 1
                # 新的一轮场景测试的开始，就可以给旧测试解锁了
                self.allSceneEndLock = 0

            # 下面的内容可以封装成一个函数，输入上面三个变量，然后输出一个当前状态的分数
            # 这里就不做所有场景的总分计算了，所有场景的总分回头用自己计算，每个场景结束的时候给他算个当前场景的总分，和乘过权重的分数
            # 第一步判断是否是最新的一帧数据，用场景和av来判断，用记录的时间，这里最少需要3帧轨迹，因为要计算加加速度
            # if lastAVdata != avPosList[-1]:
            if self.sceneData and len(self.avPosList) > 2:
                # 但是第一步要先更新所有的车辆信息，包括av车辆的更新，
                renewCarState(self.Ego, self.avPosList)
                self.Ego.longitudeAndLatitude = [round(self.avPosList[-1]["longitude"], 6), round(self.avPosList[-1]["latitude"], 6)]
                # 把安全员介入的事情做一下
                self.Ego.scoreDict['安全员介入'] = self.avPosList[-1]['isSecurityInvolved']
                # 在这里要做一些操作来辅助计算平均速度
                self.Ego.currentPos = self.avPosList[-1]['realpos']
                self.Ego.previousPos = self.avPosList[-2]['realpos']
                moveDistance = self.Ego.calculatePointDistance()
                self.Ego.calculateTravelDistance(moveDistance)
                self.Ego.calculateAvgSpeed(self.sceneData["simuTime"], self.sceneData["usedTime"])
                # 把平均速度都计算好了，存放在self.sceneAvgSpeedKM里面了，而且这个数据是实时更新的
                accuracy = abs(self.avPosList[-1]['simutime'] - self.avPosList[-2]['simutime'])
                if accuracy:
                    self.Ego.accuracy = accuracy
                else:
                    self.Ego.accuracy = 0.02
                # 单独更新加速度
                self.Ego.calcuAcce()
                self.avPosList[-1]['acce'] = self.Ego.acce
                self.Ego.posList[-1]['acce'] = self.Ego.acce
                # print(f"self.Ego: {get_total_size(self.Ego)}")
                # 更新ego车的二维边界
                vertices = self.Ego.calculate_rectangle_vertices(self.Ego.length, self.Ego.width, self.Ego.angle)
                # 更新sv的车辆信息，有就更新，没有就新建，for循环svAllPosDict
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
                # wth: 非影响范围内的SV车只做到这一步
                if not affected:
                    return senceIscore, totallSenceIscore
                # 这一步顺便也把ttc车更新一下，实例化已经做了
                # self.TTC_Car.id = (self.Ego.findClosestSimulatorVehicle())
                self.TTC_Car.id = (self.Ego.findTTCVehicle())
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
                        else:
                            if carID in self.PET_car_Dict:
                                del self.PET_car_Dict[carID]
                # 到这一步也是把所有pet的车辆都实例化成了petcar了

                # 第二步把拿到最新的数据利用起来，开始计算
                # 计算前要判断场景是否已经开始了
                # wth:积分计算前要判断是否受AV车影响
                if self.sceneData['sceneID'] or isSceneEnd(self.lastSceneData) and affected:
                    # 计算受影响的SV车辆id
                    self.Ego.affectedSVList = self.Ego.findCloseSVVehicle()
                    if self.sceneData['sceneID']:
                        self.sceneIdNoneZero = self.sceneData['sceneID']
                    # 计算分成三步，第一步计算无交互的基础指标，第二步计算有交互的TTC和pet和碰撞和间隙拒绝
                    # 最后一步就是判断是否存在场景或者说存在期望时间的概念，如果存在就计算时间的效率，如果不存在就不计算时间效率

                    # 这个好像是ttc要用，间隙也要用
                    senceIinteractionScore = interactionScore()
                    # wth: 碰撞检测在后面PET_cars里面计算
                    # 这里碰撞是直接用的他们onsite输出的可以直接拿到
                    # senceIinteractionScore.calcuCrashOnsite(crash)
                    # self.Ego.scoreDict['碰撞'] = senceIinteractionScore.crash
                    # 无交互的指标计算
                    # 倒车相关的指标属于无交互指标，要在这里面计算额外加一个av撞施工区的计算
                    avStateScoreCalcu(self.Ego, SVALL_cars_info, self.sceneIdNoneZero, mapJsonInforDict, PETCarIdList, aimPathJsonInforDict, self.sceneData['sceneID'], signalState, self.allWeight.indexWeight)
                    # 计算PET
                    # 这里要把petav也算了
                    # 这里就体现出来字典的优势了，原来需要两个for循环，现在直接用字典就解决了，petcar如果存在，则一定存在一个对应的svcar
                    # self.PET_car_Dict记录的是最近的20米内的车辆
                    # wth：初始化碰撞次数，并在每辆PET车的计算中更新，但是crash这个变量后续并没有用到
                    crash = 0
                    for carID in self.PET_car_Dict:
                        # 这里需要加一个判断保护，防止不同帧的数据互相比较计算
                        if abs(self.PET_car_Dict[carID].curTime - self.Ego.curTime) < 0.5:
                            if self.PET_car_Dict[carID].petLock == 0:
                                objCarPosList = self.PET_car_Dict[carID].posList
                                pet, petAV = self.PET_car_Dict[carID].calculate_pet(self.Ego.posList, objCarPosList)
                                if pet != 0:
                                    self.PET_car_Dict[carID].petLock = 1
                            else:
                                pet = 0
                                petAV = 0

                            # 计算碰撞，下面三个注释取消，就是重新计算碰撞
                            svSimuTime = self.PET_car_Dict[carID].posList[-1]['simutime'] - self.PET_car_Dict[carID].posList[0]['simutime']
                            avSimuTime = self.Ego.posList[-1]['simutime'] - self.Ego.posList[0]['simutime']
                            # wth:新增豁免条件，如果发生碰撞的主车或从车在场景中出现的时间不超过1s，则不需判断碰撞
                            # wth:这里有一个单拎出来的场景
                            # if Parameter.sceneName == 'FRAGMENT_0_scenario_c09ea70e':
                            #     crashSimuTimeThreshold = 3
                            # else:
                            #     crashSimuTimeThreshold = Parameter.crashSimuTimeThreshold
                            # if svSimuTime > crashSimuTimeThreshold and avSimuTime > crashSimuTimeThreshold:
                            svVertices = self.PET_car_Dict[carID].calculate_rectangle_vertices(self.SV_car_Dict[carID].length, self.SV_car_Dict[carID].width, self.SV_car_Dict[carID].angle)
                            avVertices = self.Ego.calculate_rectangle_vertices(self.Ego.length, self.Ego.width, self.Ego.angle)
                            crash += self.PET_car_Dict[carID].calculate_crash(svVertices, avVertices, self.SV_car_Dict[carID].angle, self.Ego.angle, self.Ego.accuracy, carID)
                            # if self.PET_car_Dict[carID].calculate_crash(svVertices, avVertices, self.SV_car_Dict[carID].angle, self.Ego.angle, self.Ego.accuracy, carID):
                            #     print(f"碰撞时间：{max([i['simutime'] for i in self.Ego.posList])}秒，碰撞SV车辆：{carID}")
                            #     print(f"AV车：航向角：{math.radians((90 - self.Ego.angle) % 360)}， 位置：{self.Ego.currentPos}")
                            #     print(f"SV车：航向角：{math.radians((90 - self.PET_car_Dict[carID].angle) % 360)}， 位置：{self.PET_car_Dict[carID].currentPos}")
                            # 在这里把senceIselfStateScore实例化，然后用一个总体的计算函数，来计算当前帧的状态信息
                            # 不用传参不用return，都在内部记录好了
                            if carID not in self.PET_car_Dict:
                                continue
                            # 下面这里可以用两个碰撞计算，用哪个就注释另一个，用crash = self.PET_car_Dict[carID].calculate_crash的函数计算的self.PET_car_Dict[carID].crashTime 可以用来判断碰撞，也可以用onsite自带的检查机制检查碰撞，用哪个就注释另一个
                            senceIinteractionScore.calcuCrash(self.PET_car_Dict[carID].crashTime, self.Ego.accuracy)
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

                            # todo 在这里计算某辆车是否为主车的后车
                            # 判定方式用一个函数搞定，这里在autoselfdrving里面有一个现成的函数了check_intersection，判断辆车是否相交，用self.Ego.check_intersection,就可以用了
                            # 用这个函数的前半部分，判断辆车是否平行，区别在于，要把pet车辆当成主车传进去，这样才能判断主车是否在从车前面
                            # 在得出true或者false后，就需要考虑记录这个pet车辆的id，记录的id需要再外面进行利用
                            # 注意这里还要加一个距离判断，如果距离小于定义跟随距离，就才算是跟随车辆
                            positionSV = [self.PET_car_Dict[carID].Xpos, self.PET_car_Dict[carID].Ypos]
                            directionSV = self.PET_car_Dict[carID].angle
                            positionAV = [self.Ego.Xpos, self.Ego.Ypos]
                            directionAV = self.Ego.angle
                            # 证明id为carID的车为被干扰车辆，需要进行记录
                            # wth:补充判断，航向角相同则跳过
                            if directionSV == directionAV:
                                continue
                            if self.Ego.check_intersection(positionSV, directionSV, positionAV, directionAV, 60, carID, 0):
                                if self.Ego.calculateDistance(positionSV, positionAV) < Parameter.svDisturbanceDistance:
                                    self.Ego.disturbedSvSimuTimeDict[carID] = 0

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
                    # 如果self.TTC_Car.id为空，证明当前没有合适的ttc车辆，ttc值为10
                    if self.TTC_Car.id:
                        EvaluateData = self.Ego.processEvaluation(self.Ego.bound, self.TTC_Car.bound, self.TTC_Car.speed, self.TTC_Car.id)
                    else:
                        self.Ego.TTC = 10
                    # 在后面的这个函数里，会自动计算ttc 的值并且放在ego.ttc里面，后面我们自己要用的时候就会直接调用ego.ttc了
                    self.Ego.TTC_statistics(self.Ego.accuracy)
                    # 在这里先把senceIselfStateScore实例化，然后用一个总体的计算函数，来计算当前帧的状态信息
                    senceIinteractionScore = interactionScore()
                    senceIinteractionScore.calcuOverTTC(self.Ego.TTC)

                    # 后面就是把参数记录进Ego里面，用于时间的累加计算
                    self.Ego.scoreDict['TTC'] = senceIinteractionScore.overTTC

                    # todo 在这里计算被干扰从车已经了多久
                    # 用一个字典，key是车辆id，value是他运行的时间，这个字典就放在自动驾驶车里就行了,self.Ego.disturbedSvSimuTimeDict
                    # 运行的时间是这个车从进来这个场景开始的时间，在轨迹里应该能看到他的时间，在这个self.svAllPosDict里面，
                    # 这个字典key是id，value是列表，[0]是最新的点，[-1]是最后一个点，每个点事一个字典，字典中有一个key是"simutime",value是仿真的时刻，
                    # 用[-1]["simutime"] - [0]["simutime"]就是这个车在场景中已经存在的时间了
                    # 把这个时间记录在目标字典里和id绑定
                    # 第一步，判断这个id是否在还在self.svAllPosDict里面，因为这里有个删除机制，如果不在了会自动删除这个里面的key，如果在就证明车还在运行，就按照上面步骤更新时间
                    # 第二步更新时间
                    # 第三步好像就没有了，如果判断车不在了，那证明这个时间就是最后的了
                    # 第四步，判断这个字典里面最长用时的车辆，这个车辆就是被影响的车辆
                    # 第五步，把这个数据，被主车干扰的从车的当前运行时间，存在主车里面，这里还要额外做一步，就是判断场景是否是那个目标场景，如果不是目标场景，就不用算这个东西
                    # 第六步，后面主车就可以在那个最大的里面计算了，实时的用这个时间-目标时间，为负表示不影响，为正表示开始存在影响了
                    for key in self.Ego.disturbedSvSimuTimeDict:
                        if key in self.svAllPosDict:
                            if len(self.svAllPosDict[key]) > 1:
                                runningTime = self.svAllPosDict[key][-1]["simutime"] - self.svAllPosDict[key][0]["simutime"]
                                self.Ego.disturbedSvSimuTimeDict[key] = runningTime
                                # print(self.Ego.disturbedSvSimuTimeDict)


                    # 真正开始计算评分

                    self.Ego.allScore_statistics(self.Ego.accuracy, self.sceneData['simuTime'])

                    # 到这里算是把所有的评分都放进来了
                    # 最后用这个函数来计算最后的结果
                    senceIscore, totallSenceIscore = self.Ego.calcuAllScore(self.sceneData['usedTime'], self.sceneData['missonExpectTime'], self.sceneIdNoneZero, self.sceneData['sceneWeight'], self.allWeight, trj_path_data, affected=affected, rule_list=rule_list)

                    # 这里要分清楚，哪些指标是要实时全部计算的，哪些信息是要根据场景，有场景才能计算的
                    senceIscore['useTime'] = self.sceneData['usedTime']
                    senceIscore['avgSpeedNotScore'] = self.Ego.sceneAvgSpeedKM
                    senceIscore['penaltyPoint']['avgSpeedNotScore'] = self.Ego.sceneAvgSpeedKM
        return senceIscore, totallSenceIscore

    def clearAllData(self):
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


def scoreIntegration(startTime, endTime, weightDict, valueDict, taskID, avName):
    testDuration = 0
    dangerTime = 0
    allSenseScore = 0
    testScene = []
    AbilityDimension = {
            "safe": 0,
            "efficiency": 0,
            "comfortable": 0,
            "else": 0
        }
    gradeTableDict = {}
    diagnose = {
        "安全性方面": None,
        "舒适性方面": None,
        "交互决策方面": None,
        "建议": None
    }
    for key, value in valueDict.items():
        testDuration += value["testDuration"]
        dangerTime += value["testDuration"] * value["dangerTimeProportion"]
        allSenseScore += value["allSenseScore"] * weightDict[key]
        testSceneDictI = value["testScene"][0]
        testSceneDictI["senseScore*senseWeight"] = value["testScene"][0]["senseScore"] * weightDict[key]
        testSceneDictI["senseAggScore"] = 100 * weightDict[key]
        testSceneDictI["senseWeight"] = 100 * weightDict[key]
        testScene.append(testSceneDictI)
        AbilityDimension["safe"] += testSceneDictI["AbilityDimension"]["safe"] * weightDict[key]
        AbilityDimension["efficiency"] += testSceneDictI["AbilityDimension"]["efficiency"] * weightDict[key]
        AbilityDimension["comfortable"] += testSceneDictI["AbilityDimension"]["comfortable"] * weightDict[key]
        AbilityDimension["else"] += 0
        gradeTableDict[value["sceneName"]] = value["gradeTableDict"]
    outPutData = {
        "startTime": startTime,
        "endTime": endTime,
        "testDuration": testDuration,
        "dangerTimeProportion": dangerTime/testDuration,
        "allSenseScore": allSenseScore,
        "testScene": testScene,
        "AbilityDimension": AbilityDimension,
        "gradeTableDict": gradeTableDict,
        "taskID": taskID,
        "avName": avName
    }
    return outPutData

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
    veh.curTime = posListInfo[-1]['simutime']

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
    veh.scoreDict['压虚线'] = senceIselfStateScore.onDottedLaneMarking
    veh.scoreDict['超速'] = senceIselfStateScore.overSpeed
    veh.scoreDict['闯红灯'] = senceIselfStateScore.breakSignal
    veh.scoreDict['纵向加加速度'] = senceIselfStateScore.overJerk
    veh.scoreDict['纵向加速度'] = senceIselfStateScore.overAcce
    veh.scoreDict['横向加加速度'] = senceIselfStateScore.overLateralJerk
    veh.scoreDict['横向加速度'] = senceIselfStateScore.overLateralAcce
    veh.scoreDict['转向角变化'] = senceIselfStateScore.unstableSteeringAngle
    # 新增一个倒车指标的计算
    veh.scoreDict['倒车'] = senceIselfStateScore.reverseCar
    # 新增一个停车标志前停止的指标
    veh.scoreDict['未遵守停车标志'] = senceIselfStateScore.followStopSignal
    # 新加四个静态计算指标：# 横向间距违规、# 未按规定车道行驶、# 停车压停止线、# 在禁行区行驶
    veh.scoreDict['横向间距'] = senceIselfStateScore.lateral_spacing_time
    veh.scoreDict['禁行区行驶'] = senceIselfStateScore.driving_in_restricted_area_flag
    veh.scoreDict['未按规定车道行驶'] = senceIselfStateScore.driving_in_designated_lane_time
    veh.scoreDict['停车压停止线'] = senceIselfStateScore.stop_at_the_stop_line_count
    veh.scoreDict['未按规定路线行驶'] = senceIselfStateScore.driving_in_designated_path
    veh.scoreDict['是否完成任务'] = senceIselfStateScore.missionAccomplish
    veh.scoreDict['未经过目标点'] = senceIselfStateScore.notPassAimPoint
    veh.acceAll = senceIselfStateScore.acceRead()
    # 这里是计算的指标，还有一些没有计算的指标，比如横向间距，禁行区，闯红灯等等，所以如果要后面计算新的指标，就在这里做对应的操作就行

    # 计算与施工区的碰撞关系
    avVertices = veh.calculate_rectangle_vertices(veh.length, veh.width, veh.angle)
    for ConstructionPositionI in Parameter.ConstructionPosition:
        if veh.calculate_construction_crash(avVertices, ConstructionPositionI, veh.accuracy):
            veh.scoreDict['碰撞'] = 1


def calcuGapMain(conflictCarIdList, lastConflictTimeList, senceIinteractionScore, simuTime, conflictCarPostList, Ego):
    # 在场景内部开始计算gap
    # 如果出现了self.conflictCarIdList有数据，证明开始冲突了，这里就要开始记录冲突时间了
    if conflictCarIdList:
        # 记录的冲突时间
        if lastConflictTimeList:
            lastConflictTime = lastConflictTimeList[-1]
        else:
            lastConflictTime = 0
        # 如果当前时间大于冲突时间+0.1秒则开始计算后面的东西，保证有充足的时间来添加后面的所有有冲突的车辆
        # if lastConflictTime + 0.1 < simuTime / 1000:
        # 第一步要根据存储的conflictCarIdList，找到他们的轨迹，这一步好像在上面就可以完成，在sv车辆的时候
        # 第二步就和之前一样了
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
    ChartBeforeProcess["x"].append(round(x, 2))
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
    nameList = ["TTC", "verticalA", "verticalAPlus", "horizontalA", "horizontalAPlus", "turnA", "speed", "avgSpeed"]
    for iName in nameList:
        iValue = data[iName]
        if iName not in chartDataSingleScene:
            chartData[iName] = {"x": [], "y": []}
        else:
            iChartBeforeProcess = chartDataBeforeProcess[iName]
            iChart = iNameChartData(iValue, x, iChartBeforeProcess)
            chartData[iName] = iChart

    return chartData

# 评分拆分，按照场景吧总分拆开，一个场景对应一个总分的所有数据
# 思路就是挺简单的，就把Ego, totalScore, missonExpectTime, senceWeight, missonStartTime, missonEndTime, testDuration, allWeight, chartDataTotalScene这些数据处理一下，按照每一个场景做一个循环，然后弄个新的这一些列数据，放在evaluationDataProcess评价一次，就得到单个场景的了
def splitData(Ego, totalScore, missonExpectTime, senceWeight, missonStartTime, missonEndTime, testDuration, allWeight, chartDataTotalScene, caseID,sceneCaseIDlist,avName):
    """
    sceneCaseIDlist [:1]
    """
    # 这里面只有一个循环就是totalScore
    nameScoreDict = {}
    for i in range(len(totalScore['allSenseScore'])):
        sceneItotalScore = [totalScore['allSenseScore'][i]]  # 百分制了
        sceneIDitalScore = [totalScore['allSenseScoreDital'][i]]  # 这里的scoreDital就是senceScoreEnglish，autodriving里面最后一段代码一大堆那个，包含了那个扣分的表格“penaltyPoint”
        iScore = {'allSenseScore': sceneItotalScore, 'allSenseScoreDital':sceneIDitalScore}
        sceneIoutputData = evaluationDataProcess(Ego, iScore, missonExpectTime, senceWeight, missonStartTime, missonEndTime, testDuration, allWeight, chartDataTotalScene)
        finalSceneIoutputData = splitDataProcess(sceneIoutputData,caseID,sceneCaseIDlist[i],avName)
        sceneName = allWeight.sceneTypeList[i]
        englishSceneName = "other"
        for key, value in allWeight.sceneNameDict.items():
            if sceneName in value:
                englishSceneName = key
        finalSceneIoutputData["sceneName"] = sceneName
        finalSceneIoutputData["code"] = englishSceneName
        finalSceneIoutputData["penaltyPoint"] = sceneIDitalScore[0]["penaltyPoint"]
        # 如果场景名词是4个，那跑多便这里会被覆盖，所以这里应该用场景id，sceneIDitalScore[0]['senceID']
        # nameScoreDict[sceneIDitalScore[0]['senceID']] = finalSceneIoutputData
        nameScoreDict["1"] = finalSceneIoutputData

    return nameScoreDict

def splitDataProcess(initialData,caseID,sceneCaseID,avName):
    testScene = initialData['testScene'][0]
    sceneName = initialData['testScene'][0]["sceneNameList"]
    dangerTimeProportion = 0
    for i in testScene["info"]['safe']:
        if i['index'] == 'TTC' and i['unit'] == 's':
            dangerTimeProportion = i.get('timeRatio') or i.get('timeRatio：') or 0
    finalData = {
        'testDuration': testScene["info"]['efficiency'][0]["time"],
        'dangerTimeProportion': dangerTimeProportion,
        'allSenseScore': testScene["senseScore"],
        'testScene': initialData['testScene'], # 这里要注意一下，没有列表了，直接就是一个字典，因为一定是对应一个场景所以没有列表了
        'AbilityDimension': testScene["AbilityDimension"],
        #'gradeTableDict': initialData["gradeTableDict"][sceneName]
        # todo 这里后面注意改一下可能需要，这里是拼接的展示效果
        'gradeTableDict': initialData["gradeTableDict"][sceneName + '/' + str(initialData['testScene'][0]['senceID'])]
    }
    # 这里要用之前拿到的数据outputdata来判断一下写出哪些诊断语句库
    # 这里专门在写一个类写诊断语句
    diagnoseLibrary = diagnose(finalData)
    diagnoseLibrary.getSuggestion()
    finalData["diagnose"] = diagnoseLibrary.diagnoseSuggestionDict
    diagnoseLibrary = None
    finalData["taskID"] = caseID
    finalData["caseID"] = sceneCaseID
    finalData["avName"] = avName
    return finalData

def evaluationDetailDataProcess(senceIscore, totallSenceIscore, senceStartId, lastTime, Ego, senceWeight,
                                missonExpectTime, allweight):
    weight = senceWeight

    if senceStartId != 0:
        senceStatus = 0
        senceScore = None
    else:
        senceStatus = 1
        senceScore = str(round(
            (totallSenceIscore['safe100'] + totallSenceIscore['efficiency100'] + totallSenceIscore['comfortable100'] +
             totallSenceIscore['coordination100'] + totallSenceIscore['compliance100']), 2)) + '/' + '100'

    data = {"senceID": senceIscore['senceID'],
            "senceStatus": senceStatus,
            "senceAccomplish": senceIscore["calMissionAccomplish"],
            "senceScore": senceScore,
            "safeScore": 100 * allweight.scoreWeight['safe'],
            "safeMinusScore": 100 * allweight.scoreWeight['safe'] - totallSenceIscore['safe100'],
            # 'safeScore': 100,
            # 'safeMinusScore': 100 - totallSenceIscore['safe']/weight,
            # 这里是因为，分数计算的时候按20%计算安全，实际显示的时候按照100分来显示
            "TTC": Ego.TTC,
            "avgSpeedAll": Ego.avgSpeed,
            "avgSpeed": Ego.sceneAvgSpeed,
            "speed": Ego.speed,
            "efficiencyScore": 100 * allweight.scoreWeight['efficiency'],
            "efficiencyMinusScore": 100 * allweight.scoreWeight['efficiency'] - totallSenceIscore['efficiency100'],
            "taskAlreadyTime": lastTime,
            "taskTime": missonExpectTime,
            "comfortableScore": 100 * allweight.scoreWeight['comfortable'],
            "comfortableMinusScore": 100 * allweight.scoreWeight['comfortable'] - totallSenceIscore['comfortable100'],
            "verticalA": Ego.acceAll['verticalA'],
            "verticalAPlus": Ego.acceAll['verticalAPlus'],
            "horizontalA": Ego.acceAll['horizontalA'],
            "horizontalAPlus": Ego.acceAll['horizontalAPlus'],
            "turnA": Ego.acceAll['turnA'],
            "turnAPlus": Ego.acceAll['turnA'],
            "coordinationScore": 100 * allweight.scoreWeight['coordination'],
            "coordinationMinusScore": 100 * allweight.scoreWeight['coordination'] - totallSenceIscore['coordination100'],
            "complianceScore": 100 * allweight.scoreWeight['compliance'],
            "complianceMinusScore": 100 * allweight.scoreWeight['compliance'] - totallSenceIscore['compliance100']}
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

def checkCrashTime(eventTable, aimIndex):
    crashNum = 0
    for key in eventTable:
        if eventTable[key]["index"] == aimIndex:
            crashNum += 1
    return crashNum

def evaluationDataProcess(Ego, totalScore, missonExpectTime, senceWeight, missonStartTime, missonEndTime, testDuration,
                          allWeight, chartDataTotalScene):
    allSence = []
    num = 2
    AbilityDimension = {"safe": 0,
                        "efficiency": 0,
                        "comfortable": 0,
                        "coordination": 0,
                        "compliance": 0,
                        "else": 0, }  # 百分制
    scoreWeight = allWeight.scoreWeight
    clearZero(allWeight.scoreWeight)
    clearZero(allWeight.comfortableDitalWeight)
    clearZero(allWeight.safeDitalWeight)
    clearZero(allWeight.efficiencyDitalWeight)
    gradeTableDict = {}
    scoreTableDict = {}

    for i in range(len(totalScore['allSenseScore'])):
        sceneItotalScore = totalScore['allSenseScore'][i]  # 百分制了,totalSenceScore,autodriving里面最后一段代码，总的那个，这里面两个的sceneid都是场景的序号，是唯一的，caseid是最开始就定好的，可能是重复的
        sceneIDitalScore = totalScore['allSenseScoreDital'][i]  # 这里的scoreDital就是senceScoreEnglish，autodriving里面最后一段代码一大堆那个
        if sceneIDitalScore['useTime']:
            pass
        else:
            sceneIDitalScore['useTime'] = 0.001
        efficiency = [{"index": "任务完成", "score": str(sceneIDitalScore['calMissionAccomplish'] *
                                                         allWeight.efficiencyDitalWeight['missionAccomplish']) + "/" +
                                                     str(allWeight.efficiencyDitalWeight['missionAccomplish']),
                       "time": round(sceneIDitalScore['useTime'], num),
                       "name": "任务完成",
                       "unit": "s", },
                      {"index": "任务耗时", "name": "任务耗时",
                       "score": str(
                           round(min(allWeight.efficiencyDitalWeight["usedTime"],
                                     (missonExpectTime[sceneItotalScore['senceID']] / sceneIDitalScore['useTime']) *
                                     allWeight.efficiencyDitalWeight["usedTime"]),
                                 num)) + "/" + str(
                           round(allWeight.efficiencyDitalWeight["usedTime"], num)),
                       "time": round(sceneIDitalScore['useTime'], num),
                       "unit": "s",
                       "expectTime": missonExpectTime[sceneItotalScore['senceID']],
                       "overTime": round((sceneIDitalScore['useTime'] - missonExpectTime[sceneItotalScore['senceID']]) /
                                         missonExpectTime[sceneItotalScore['senceID']], num)},
                      {"index": "任务耗时", "name": "任务耗时", "score": str(round(
                          (min(1, sceneIDitalScore['avgSpeedNotScore'] / Parameter.avgSpeedKM)) *
                          allWeight.efficiencyDitalWeight["averageSpeed"],
                          num)) + "/" + str(
                          round(allWeight.efficiencyDitalWeight["averageSpeed"], num)),
                       "avgSpeed": sceneIDitalScore['avgSpeedNotScore'],
                       "time": sceneIDitalScore['avgSpeedNotScore'],
                       "unit": "km/h",
                       "expectSpeed": Parameter.avgSpeedKM,
                       "belowSpeed": round((Parameter.avgSpeedKM - sceneIDitalScore['avgSpeedNotScore']) /
                                           Parameter.avgSpeedKM, num)},
                      {"index": "倒车", "name": "非场景需要倒车", "score": str(round(
                          (max(0, Parameter.miniScoreMaxLimit['倒车'] - sceneIDitalScore['reverseCar'])),
                          num)) + "/" + str(
                          round(Parameter.miniScoreMaxLimit['倒车'], num)),
                       "time": round(sceneIDitalScore['reverseCar'] * sceneIDitalScore['useTime'] / (
                           (allWeight.efficiencyDitalWeight["reverseCar"])), num),
                       "unit": "s", },
                      {"index": "倒车", "name": "非场景需要倒车",
                       "score": str(round((max(0, Parameter.miniScoreMaxLimit['倒车'] -
                                               (checkCrashTime(sceneIDitalScore['eventTable'],
                                                               '倒车') * allWeight.efficiencyDitalWeight[
                                                    "reverseCar"]))), num)) + "/" + str(
                           round(Parameter.miniScoreMaxLimit['倒车'], num)),
                       "time": checkCrashTime(sceneIDitalScore['eventTable'], '倒车'),
                       "unit": "次", }, ]

        comfortable = [
            {"index": "纵向舒适度", "score": str(round(sceneIDitalScore['overJerk'], num)), "type": "j",
             "unit": "s", "name": "纵向加加速度",
             "time": round(sceneIDitalScore['overJerk'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overJerk"])), num)},
            {"index": "纵向舒适度", "score": str(round(sceneIDitalScore['overAcce'], num)), "type": "a",
             "unit": "s", "name": "纵向加速度",
             "time": round(sceneIDitalScore['overAcce'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overAcce"])), num)},
            {"index": "横向舒适度", "score": str(round(sceneIDitalScore['overLateralJerk'], num)), "type": "j",
             "unit": "s", "name": "横向加加速度",
             "time": round(sceneIDitalScore['overLateralJerk'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overLateralJerk"])), num)},
            {"index": "横向舒适度", "score": str(round(sceneIDitalScore['overLateralAcce'], num)), "type": "a",
             "unit": "s", "name": "横向加速度",
             "time": round(sceneIDitalScore['overLateralAcce'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overLateralAcce"])), num)},
            {"index": "转弯舒适度", "score": str(round(sceneIDitalScore['unstableSteeringAngle'], num)),
             "type": "c", "unit": "s", "name": "横摆角速度",
             "time": round(sceneIDitalScore['unstableSteeringAngle'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["unstableSteeringAngle"])), num)}]
        # if sceneIDitalScore['crash'] > 0:
        #     crash = 100 * allWeight.scoreWeight['safe']
        # else:
        #     crash = 0
        # if sceneIDitalScore['isSecurityInvolved'] > 0:
        #     # isSecurityInvolved = 100 * allWeight.scoreWeight['safe']
        #     isSecurityInvolved = 100
        # else:
        #     isSecurityInvolved = 0
        safe = [{"index": '碰撞', "score": str(min(100, round(sceneIDitalScore['crash'], num))),
                 "unit": "次", "name": "碰撞",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], '碰撞')},
                {"index": '安全员介入', "score": str(min(100, round(sceneIDitalScore['isSecurityInvolved'], num))),
                 "unit": "次", "name": "安全员介入",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], '安全员介入')},
                {"index": '驶出行车道', "score": str(min(100, round(sceneIDitalScore['outOfLane'], num))),
                 "unit": "s", "name": "驶出道路边界",
                 "time": round(sceneIDitalScore['outOfLane'] * sceneIDitalScore['useTime'] / (
                     allWeight.safeDitalWeight["outOfLane"]), num)},
                {"index": '驶出行车道', "score": str(min(100, round(
                    checkCrashTime(sceneIDitalScore['eventTable'], '驶出行车道') * allWeight.safeDitalWeight[
                        "outOfLane"], num))),
                 "unit": "次", "name": "驶出道路边界",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], '驶出行车道')},
                {"index": 'TTC', "score": str(round(sceneIDitalScore['TTC'], num)),
                 "unit": "s", "name": "TTC",
                 "time": round(sceneIDitalScore['TTC'] * sceneIDitalScore['useTime'] / (
                     allWeight.safeDitalWeight["TTC"]), num),
                 "timeRatio：": round(sceneIDitalScore['TTC'] / (allWeight.safeDitalWeight["TTC"]), num),
                 "timeRatio": round(sceneIDitalScore['TTC'] / (allWeight.safeDitalWeight["TTC"]), num), },
                {"index": 'TTC', "score": str(min(100, round(
                    checkCrashTime(sceneIDitalScore['eventTable'], 'TTC') * allWeight.safeDitalWeight["TTC"], num))),
                 "unit": "次", "name": "TTC",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], 'TTC')},
                {"index": "横向间距", "score": str(round(sceneIDitalScore['transverseDistance'], num)),
                 "unit": "s", "name": "横向间距",
                 "time": round(sceneIDitalScore['transverseDistance'] * sceneIDitalScore['useTime'] / (
                     allWeight.safeDitalWeight["transverseDistance"]), num)}, ]

        compliance = [{"index": '压实线', "score": str(round(sceneIDitalScore['onLaneMarking'], num)),
                       "unit": "s", "name": "压实线",
                       "time": round(sceneIDitalScore['onLaneMarking'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["onLaneMarking"]), num)},
                      {"index": '压虚线', "score": str(round(sceneIDitalScore['onDottedLaneMarking'], num)),
                       "unit": "s", "name": "压实线",
                       "time": round(sceneIDitalScore['onDottedLaneMarking'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["onDottedLaneMarking"]), num)},
                      {"index": '超速', "score": str(round(sceneIDitalScore['overSpeed'], num)),
                       "unit": "s", "name": "超出限速行驶",
                       "time": round(sceneIDitalScore['overSpeed'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["overSpeed"]), num)},
                      {"index": '超速', "score": str(min(allWeight.scoreWeight['compliance'] * 100, round(
                          checkCrashTime(sceneIDitalScore['eventTable'], '超速') * allWeight.safeDitalWeight[
                              "overSpeed"], num))),
                       "unit": "次", "name": "超出限速行驶",
                       "time": checkCrashTime(sceneIDitalScore['eventTable'], '超速')},
                      {"index": '闯红灯', "name": "闯红灯", "score": str(round(0, num)), "unit": "次", "time": 0},
                      {"index": '未遵守停车标志', "score": str(min(allWeight.scoreWeight['compliance'] * 100,
                                                                   round(sceneIDitalScore['followStopSignal'], num))),
                       "unit": "次", "name": "未停车让行",
                       "time": round(sceneIDitalScore['followStopSignalNum'], num)},
                      {"index": '驶入对向车道', "score": str(round(sceneIDitalScore['inSubtendRoad'], num)),
                       "unit": "s", "name": "驶入对向车道",
                       "time": round(sceneIDitalScore['inSubtendRoad'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["inSubtendRoad"]), num)},
                      {"index": '驶入对向车道', "score": str(min(allWeight.scoreWeight['compliance'] * 100, round(
                          checkCrashTime(sceneIDitalScore['eventTable'], '驶入对向车道') * allWeight.safeDitalWeight[
                              "inSubtendRoad"], num))),
                       "unit": "次", "name": "驶入对向车道",
                       "time": checkCrashTime(sceneIDitalScore['eventTable'], '驶入对向车道')},
                      {"index": "禁行区行驶", "score": str(round(sceneIDitalScore['inForbiddenArea'], num)),
                       "unit": "s", "name": "禁行区行驶",
                       "time": round(sceneIDitalScore['inForbiddenArea'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["inForbiddenArea"]), num)},
                      {"index": "停车压停止线", "score": str(round(sceneIDitalScore['stopAtStopLine'], num)),
                       "unit": "s", "name": "停车压停止线",
                       "time": round(sceneIDitalScore['stopAtStopLine'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["stopAtStopLine"]), num)},
                      {"index": "未按规定车道行驶",
                       "unit": "s", "name": "未按规定车道行驶",
                       "score": str(round(sceneIDitalScore['drivingInDesignatedLane'], num)),
                       "time": round(sceneIDitalScore['drivingInDesignatedLane'] * sceneIDitalScore['useTime'] /
                                     (allWeight.safeDitalWeight["drivingInDesignatedLane"]), num)},
                      {"index": "未有效泊车",
                       "unit": "次", "name": "未正常泊车",
                       "score": str(round(sceneIDitalScore.get('outOfParkingZone', 0), num)),
                       "time": round(sceneIDitalScore.get('outOfParkingZone', 0) / 100, num)}, ]

        coordination = [{"index": '干扰背景车辆通行', "name": "干扰背景车辆通行",
                         "score": str(round(sceneIDitalScore['coordination'], num)),
                         "unit": "s", "time": round(sceneIDitalScore['minCoordinationScore'], num),
                         "svSimuTimeWithoutDisturbance": round(sceneIDitalScore['svSimuTimeWithoutDisturbance'], num),
                         "maxDisturbedSvSimuTime": round(sceneIDitalScore['maxDisturbedSvSimuTime'], num)}, ]

        AbilityDimensionI = {}
        # AbilityDimensionI["safe"] = round(sceneItotalScore['safe100'] / allWeight.scoreWeight['safe'], num)
        # AbilityDimensionI["comfortable"] = round(sceneItotalScore['comfortable100'] / allWeight.scoreWeight['comfortable'], num)
        # AbilityDimensionI["efficiency"] = round(sceneItotalScore['efficiency100'] / allWeight.scoreWeight['efficiency'], num)
        # AbilityDimensionI["coordination"] = round(sceneItotalScore['coordination100'] / allWeight.scoreWeight['coordination'], num)
        # AbilityDimensionI["compliance"] = round(sceneItotalScore['compliance100'] / allWeight.scoreWeight['compliance'], num)
        AbilityDimensionI["safe"] = round(sceneItotalScore['safe100'], num)
        AbilityDimensionI["comfortable"] = round(sceneItotalScore['comfortable100'], num)
        AbilityDimensionI["efficiency"] = round(sceneItotalScore['efficiency100'], num)
        AbilityDimensionI["coordination"] = round(sceneItotalScore['coordination100'], num)
        AbilityDimensionI["compliance"] = round(sceneItotalScore['compliance100'], num)
        AbilityDimensionI["safe*weight"] = round(sceneItotalScore['safe100'], num)
        AbilityDimensionI["comfortable*weight"] = round(sceneItotalScore['comfortable100'], num)
        AbilityDimensionI["efficiency*weight"] = round(sceneItotalScore['efficiency100'],num)
        AbilityDimensionI["coordination*weight"] = round(sceneItotalScore['coordination100'],num)
        AbilityDimensionI["compliance*weight"] = round(sceneItotalScore['compliance100'],num)
        AbilityDimensionI["safeWeight"] = round(allWeight.scoreWeight['safe'], num)
        AbilityDimensionI["comfortableWeight"] = round(allWeight.scoreWeight['comfortable'], num)
        AbilityDimensionI["efficiencyWeight"] = round(allWeight.scoreWeight['efficiency'], num)
        AbilityDimensionI["coordinationWeight"] = round(allWeight.scoreWeight['coordination'], num)
        AbilityDimensionI["complianceWeight"] = round(allWeight.scoreWeight['compliance'], num)
        testSceneI = {"senceID": sceneItotalScore['senceID'],
                      "sceneNameList": allWeight.sceneTypeList[i], # 场景的名称
                      "chartData": chartDataTotalScene[sceneItotalScore['senceID']],
                      "AbilityDimension": AbilityDimensionI,
                      "missionAccomplish": sceneIDitalScore['missionAccomplish'],
                      # "senseScore": round((sceneItotalScore['safe100'] + sceneItotalScore['efficiency100'] + sceneItotalScore['comfortable100']
                      #                      + sceneItotalScore['coordination100'] + sceneItotalScore['compliance100']), num),
                      "senseScore": round(sceneItotalScore['efficiency'] ** scoreWeight['efficiency'] * (sceneItotalScore['safe'] * scoreWeight['safe'] + sceneItotalScore['coordination'] * scoreWeight['coordination']
                                                                                                   + sceneItotalScore['comfortable'] * scoreWeight['comfortable'] + sceneItotalScore['compliance'] * scoreWeight['compliance']), num),
                      "senseScore*senseWeight": round(sceneItotalScore['efficiency'] ** scoreWeight['efficiency'] * (sceneItotalScore['safe'] * scoreWeight['safe'] + sceneItotalScore['coordination'] * scoreWeight['coordination']
                                                                                                   + sceneItotalScore['comfortable'] * scoreWeight['comfortable'] + sceneItotalScore['compliance'] * scoreWeight['compliance']), num),
                      "senseAggScore": round(senceWeight[sceneItotalScore['senceID']] * 100, num),
                      "senseWeight": round(senceWeight[sceneItotalScore['senceID']] * 100, num),
                      "info": {"efficiency": efficiency, "comfortable": comfortable, "safe": safe, "compliance": compliance, "coordination": coordination},
                      "eventTable": sceneIDitalScore["eventTable"], }
        allSence.append(testSceneI)

        # 计算能力雷达图
        AbilityDimension["safe"] += sceneItotalScore['safe']
        AbilityDimension["efficiency"] += sceneItotalScore['efficiency']
        AbilityDimension["comfortable"] += sceneItotalScore['comfortable']
        AbilityDimension["coordination"] += sceneItotalScore['coordination']
        AbilityDimension["compliance"] += sceneItotalScore['compliance']

        # 计算分级表格
        safeGrade = gradeScore(98, 95, 90, 80, AbilityDimensionI["safe"])
        comfortableGrade = gradeScore(95, 88, 75, 60, AbilityDimensionI["comfortable"])
        efficiencyGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["efficiency"])
        coordinationGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["coordination"])
        complianceGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["compliance"])
        # gradeTableDict[allWeight.sceneTypeList[i]+str(sceneItotalScore['senceID'])] = {
        gradeTableDict[allWeight.sceneTypeList[i] + '/' + str(sceneItotalScore['senceID'])] = {
            "安全性": safeGrade,
            "舒适性": comfortableGrade,
            "效率性": efficiencyGrade,
            "交通协调性": coordinationGrade,
            "交规符合性": complianceGrade, }
        # scoreTableDict[allWeight.sceneTypeList[i] + str(sceneItotalScore['senceID'])] = {
        scoreTableDict[allWeight.sceneTypeList[i] + '/' + str(sceneItotalScore['senceID'])] = {
            "安全性": {'score': AbilityDimensionI["safe"], 'grade': safeGrade},
            "舒适性": {'score': AbilityDimensionI["comfortable"], 'grade': comfortableGrade},
            "效率性": {'score': AbilityDimensionI["efficiency"], 'grade': efficiencyGrade},
            "交通协调性": {'score': AbilityDimensionI["coordination"], 'grade': coordinationGrade},
            "交规符合性": {'score': AbilityDimensionI["compliance"], 'grade': complianceGrade}, }
        # todo 解决一下展示的问题
        # gradeTableDict[sceneItotalScore['senceID']] = {"安全性": safeGrade, "舒适性": comfortableGrade, "效率性": efficiencyGrade, }

    allSenseScore = 0
    for i in allSence:
        allSenseScore += i['senseScore*senseWeight']

    # for key in AbilityDimension:
    #     if key in allWeight.scoreWeight:
    #         AbilityDimension[key] = round(AbilityDimension[key] / allWeight.scoreWeight[key], num)

    if testDuration != 0:
        dangerTime = Ego.TTC_LowTime / testDuration
    else:
        dangerTime = 0

    data = {'startTime': missonStartTime,
            'endTime': missonEndTime,
            'testDuration': round(testDuration, num),
            'dangerTimeProportionPercentage': str(round(100 * dangerTime, num)) + "%",
            'dangerTimeProportion': round(dangerTime, num),
            'allSenseScore': round(allSenseScore, num),  # 这个是总分
            'testScene': allSence,
            "AbilityDimension": AbilityDimension,  # 这个也是总分百分制的能力雷达图
            "gradeTableDict": gradeTableDict, }  # 这个是分级的表格
    return data


def getState(redisWanJIAddress, redisWanJIPort, channelStart, scene_queue, channel_queue, state_queue):
    print(channelStart, '线程开启')
    allRedis = redis.StrictRedis(host=redisWanJIAddress, port=redisWanJIPort, db=0, password="Wanji@300552!")
    pubsub = allRedis.pubsub()
    pubsub.subscribe(channelStart)
    for message in pubsub.listen():
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
    print("开始监听sv车辆信息")

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


