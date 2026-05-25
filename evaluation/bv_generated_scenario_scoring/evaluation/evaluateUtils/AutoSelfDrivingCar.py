import os
import time
from typing import Tuple

import numpy as np
import math
import random
import pandas as pd
from datetime import datetime

from .MySource.HeadFile import Scenario, ScenarioLock
from .MySource.Cache import Record
from .MySource.functions import find_minimum_distance, judgeTTC, Merge
from .Config.config_analyse import Analyse

from .standard_parameter import Parameter

configScenario = Analyse(os.path.join(os.path.abspath(os.path.dirname(__file__)), 'Config', 'Scenario.json'))

desireTime = configScenario.info('desireTimeConsume')
testDesireConsume = configScenario.info('testDesireConsume')


# 普通车
# 普通车要多加一个储存自己轨迹的功能
class Car(object):
    def __init__(self, id, Xpos, Ypos, speed, roadType):
        self.id = id
        self.Xpos = Xpos
        self.Ypos = Ypos
        self.speed = speed
        self.acce = 0
        self.angle = 0
        self.vehicleType = 0
        self.roadType = roadType
        self.distance = 0
        # 里程数，要加一个当前场景的里程数
        self.sceneDistance = 0
        self.roadId = 0
        self.laneNum = None
        self.vehiHeadwayFront = 0
        self.vehiDistFront = 0
        self.previousPos = []
        self.currentPos = []
        self.longitudeAndLatitude = []
        # 路段上已经行驶的距离
        self.disInRoad = 0
        # 自己行驶过的位置
        self.posList = []
        # 用来存放下一秒自己的计算的加速度
        self.nextAcce = 0
        # 用来存放下一秒自己的换道信息
        self.nextAct = None
        # 存放自己创建时的仿真时间
        self.bronTime = 0
        # 存放当前自己所在的时间点
        self.curTime = 0
        # 记录自己车辆在指定最初的位置后，生成的路径linkid
        self.initialRouting = []
        # 最开始设置的路径，如果有就按照这个给每次的路径赋值，没有就按照上面的
        self.startSetRouting = []
        # 是否需要继续控制
        self.svControl = 1



    def isConnector(self) -> bool:
        if self.roadType == 0:
            return True
        else:
            return False

    # 存储当前帧数据
    def add_frame(self, frame_data):
        if not self.currentPos:
            self.previousPos = frame_data
        else:
            self.previousPos = self.currentPos
        self.currentPos = frame_data

    # 获取当前帧数据
    def get_current_framePos(self):
        return self.currentPos

    # 获取上一帧数据
    def get_previous_framePos(self):
        return self.currentPos

    # 判断是否在场景内部
    def judgeScenario(self, scenarioRoad):
        if self.roadId == scenarioRoad:
            return True

    # 计算里程
    def calculateTravelDistance(self, distance):
        self.distance += distance
        self.sceneDistance += distance

    # 计算前后两帧行驶的距离
    def calculatePointDistance(self):
        p1 = np.array([self.previousPos[0], self.previousPos[1]])
        p2 = np.array([self.currentPos[0], self.currentPos[1]])
        p3 = p2 - p1
        d = math.hypot(p3[0], p3[1])
        return d

    # 计算AV与SV之间的距离
    def pointDistanceBetweenSV(self, SV_Pos):
        # p1 = np.array([SV_Pos[0], SV_Pos[1]])
        # p2 = np.array([self.currentPos[0], self.currentPos[1]])
        # p3 = p2 - p1
        # d = math.hypot(p3[0], p3[1])
        x1 = self.currentPos[0]
        y1 = self.currentPos[1]
        x2 = SV_Pos[0]
        y2 = SV_Pos[1]

        d = pow((pow(x1 - x2, 2) + pow(y1 - y2, 2)), 0.5)
        return d

    # 将y轴正轴顺时钟方向的角转为x正轴逆时针方向的角
    @staticmethod
    def convert_clockwise_to_counterclockwise(angle) -> float:
        counterclockwise_angle = (90 - angle) % 360
        return counterclockwise_angle

    # 把路过的点都储存起来
    def record_posList(self, Xpos, Ypos, t, speed, acce, vehType, angle):
        current_pos = {'id': self.id, 'x': Xpos, 'y': Ypos,
                       'simutime': t,
                       'speed': speed,
                       'acce': acce,
                       'type': vehType,
                       'angle': angle,
                       'HeadwayFront': self.vehiHeadwayFront,
                       'DistFront': self.vehiDistFront}
        self.posList.append(current_pos)
        # 当list中存在至少两个点时，计算最后一个点和第一个点之间的时间差，超过60秒就删除第一个点
        if len(self.posList) > 2:
            a = self.posList[len(self.posList) - 1]['simutime'] - self.posList[0]['simutime']
            if a > 60:
                del self.posList[0]

class TTCCar(Car):
    def __init__(self, id, Xpos, Ypos, speed, roadType):
        Car.__init__(self, id, Xpos, Ypos, speed, roadType)
        self.currentPos = []
        self.bound = []
        self.speed = 0

    def calculate_rectangle_vertices(self, length, width, angle) -> list:
        """ 计算矩形四个角点坐标
        :param length:    矩形的长
        :param width:     矩形的宽
        :param angle:     矩形的旋转角度，沿y轴顺时针旋转，就是tessng的angle
        :return:  矩形的四个角点坐标，依次是左上、左下、右下、右上
        """
        # 转为x轴正轴逆时针方向的角
        angle = self.convert_clockwise_to_counterclockwise(angle)
        angle = math.radians(angle)
        dx = length / 2
        dy = width / 2

        # 计算矩形角点相对于中心点的坐标偏移量
        vertex_offsets = [
            (-dx * math.cos(angle) - dy * math.sin(angle), -dx * math.sin(angle) + dy * math.cos(angle)),  # 左上角
            (-dx * math.cos(angle) + dy * math.sin(angle), -dx * math.sin(angle) - dy * math.cos(angle)),  # 左下角
            (dx * math.cos(angle) + dy * math.sin(angle), dx * math.sin(angle) - dy * math.cos(angle)),  # 右上角
            (dx * math.cos(angle) - dy * math.sin(angle), dx * math.sin(angle) + dy * math.cos(angle))  # 右下角
        ]

        # 计算矩形顶点的绝对坐标
        vertices = [(self.currentPos[0] + offset[0], self.currentPos[1] + offset[1]) for offset in vertex_offsets]
        self.bound = vertices
        return vertices

# 实例化petcar的车都是ego周围的车，这里就要判断这些车与ego的pet了
# 这里要做的一个函数就是，判断主车与从车之间的点位的距离
class PETCar(Car):
    def __init__(self, id, Xpos, Ypos, speed, roadType):
        Car.__init__(self, id, Xpos, Ypos, speed, roadType)
        self.currentPos = []
        self.bound = []
        self.speed = 0
        self.crashTime = 0
        self.petLock = 0
        # 记录是否为第一次发生碰撞，并且记录是否为sv撞击av
        self.firstCrash = 0
        self.SVcrash = 0

    # 在写一个函数，两点之间的距离公式
    def calculateDistance(self, x1, y1, x2, y2):
        distance = pow((pow(x1 - x2, 2) + pow(y1 - y2, 2)), 0.5)
        return distance

    # 这个函数就是正反各取一次，保证不管是谁追击谁都能输出值
    def calculate_pet(self, car1: list, car2: list) -> float:
        pet1 = self.pet(car1[len(car1) - 1], car2)
        pet2 = self.pet(car2[len(car2) - 1], car1)
        # 就是这里要计算是谁撞的谁
        # sv撞的av，就用当前sv的前面的间距作为gap
        # av撞的sv，就用当前sv的后面的间距作为gap

        # car1是av，pet1有值，则证明是av装上去的，所以需要这个值，所以返回的时候额外返回pet1
        pet = max(pet1, pet2)
        return pet, pet1

    # 在写个函数，这个函数是用一个主车a的当前轨迹值，与目标车b全体轨迹比较，算距离
    def pet(self, carA: dict, carB: list) -> float:
        # 这里a车是一个字典，xyt，b车是一个列表，里面有好多字典
        carApos = [carA['x'], carA['y']]
        disMin = 999999
        timeMin = 0
        pet = 0
        # 不断的判断carb中的轨迹点与当前车辆的点之间的距离，找到最近的点，把最近的点和对应的时刻记录下来，这个时刻一定是小的
        for posI in carB:
            # print(posI)
            carBpos = [posI['x'], posI['y']]
            dis = self.calculateDistance(carApos[0], carApos[1], carBpos[0], carBpos[1])
            if dis < disMin:
                disMin = dis
                timeMin = posI['simutime']

        # print(disMin)
        if disMin < 1:
            pet = carA['simutime'] - timeMin
        return pet

    def rectangles_overlap(self, rect1, rect2):
        """判断两个矩形是否存在重叠部分"""
        # 将每个矩形表示为包含顶点和边的列表
        rect1_edges = [(rect1[i], rect1[(i + 1) % len(rect1)] - rect1[i]) for i in range(len(rect1))]
        rect2_edges = [(rect2[i], rect2[(i + 1) % len(rect2)] - rect2[i]) for i in range(len(rect2))]

        # 检查每个矩形的边是否为分离轴
        for edge in rect1_edges + rect2_edges:
            axis = np.array([-edge[1][1], edge[1][0]])  # 计算边的垂直向量作为分离轴
            min1, max1 = self.project_rect(rect1, axis)
            min2, max2 = self.project_rect(rect2, axis)
            if max1 < min2 or max2 < min1:  # 如果投影没有重叠，返回False
                return False
        return True

    def project_rect(self, rect, axis):
        """在分离轴上投影矩形，并返回投影的最小值和最大值"""
        min_proj = np.dot(rect[0], axis)
        max_proj = min_proj
        for point in rect[1:]:
            proj = np.dot(point, axis)
            min_proj = min(min_proj, proj)
            max_proj = max(max_proj, proj)
        return min_proj, max_proj

    def divide_line_segment(self, start_point, end_point, n):
        """将线段等分为n段，并返回各个等分点的坐标"""
        delta_x = (end_point[0] - start_point[0]) / n
        delta_y = (end_point[1] - start_point[1]) / n
        divided_points = [(start_point[0] + i * delta_x, start_point[1] + i * delta_y) for i in range(1, n)]
        return divided_points

    def isSVcrashAV(self, avRect, svRect):
        """
        首先对av车边界取前半部分
        然后用av的前半部分与sv全体做分离轴定理
        最后如果av前半部分与sv相撞则判断av的车速是否小于5
        如果相撞且有车速，则判定av存在碰撞责任
        svcrash=0，否则svcrash为1
        """
        # 矩形的左上角顶点
        x1, y1 = avRect[0]
        # 矩形的左下角顶点
        x2, y2 = avRect[1]
        # 矩形的右下角顶点
        x3, y3 = avRect[2]
        # 矩形的右上角顶点
        x4, y4 = avRect[3]

        rightHalf = self.divide_line_segment(avRect[0], avRect[1],5)[0]
        leftHalf = self.divide_line_segment(avRect[3], avRect[2],5)[0]

        # 四分之一部分的顶点坐标
        quarter_vertices = [(x1, y1), rightHalf, leftHalf, (x4, y4)]

        # 分离轴定理计算是否相交
        rect1 = np.array(quarter_vertices)  # 第一个矩形的四个顶点坐标
        rect2 = np.array(svRect)  # 第二个矩形的四个顶点坐标
        overlap = self.rectangles_overlap(rect1, rect2)

        # 前半部分没有撞上就表示是sv撞的av
        if not overlap:
            return 1
        else:
            # 在判断速度，如果这个时候av有停车的想法或者速度小于某个值，则也算是sv撞的av
            if self.speed < 5 and self.acce < 0:
                return 1
            else:
                return 0

    # 计算是否碰撞了，碰撞了返回1，没有碰撞返回0
    # 当前就直接按照pet来计算，回头如果有时间就按照物理边界来计算
    # 不行这个方式太不准了，如果交汇，可能很久才到交点，但是前面就剐蹭了，还是要用物理边界，但是现在实在是太恶心了，分离轴算法回头再说吧
    # 用上面的分离轴定理
    def calculate_crash(self, svVertices, avVertices, svAngle, avAngle, accuracy, carID):

        # 分离轴定理计算是否相交
        rect1 = np.array(avVertices)  # 第一个矩形的四个顶点坐标
        rect2 = np.array(svVertices)  # 第二个矩形的四个顶点坐标
        overlap = self.rectangles_overlap(rect1, rect2)

        # 最后比较是否有交集后就可以返回了
        # 要加一个谁撞谁的话就需要在这里判断，如果下面这个为1，那么就在进行一步，判断sv的两个点是否在av内部
        # 这里有个非常简单的想法，如果sv的尖尖撞到了av上，那么碰撞点一定是会在这四个边界上，完了有点想不通，算了那就还是用sv的前两个点，是否有一个在四边形之内把
        if not overlap:
            # 这里是没有撞上的
            # 如果有一瞬间没有状，就把累计时间清零
            self.crashTime = 0
            # 并且还要把self.firstCrash和self.SVcrash重置为0
            self.firstCrash = 0
            self.SVcrash = 0
            return 0
        else:
            # 这里是撞上的
            # 这里需要引入一个变量，firstcrash，如果是第一次撞击才能用这个，判断，并且如果是第一次撞击，并且sv4个点其中有一个在av内部，则引入另一个变量svcrash = 1
            # 这里引入SVcrash是证明，是谁撞得谁，如果是av撞sv才去累加accuracy，如果是sv撞av则豁免这次碰撞
            # 后面就只需要判断是否svcrash是1，如果是1就不用继续了，直接return 0
            if self.firstCrash == 0:
                # 判断是否是av撞的sv只需要判断，在碰撞的一瞬间是否av的前半部分车身与sv有交集，如果有就证明是av撞的sv
                self.SVcrash = self.isSVcrashAV(avVertices, svVertices)
                self.firstCrash = 1
            if self.SVcrash == 0:
                self.crashTime += accuracy
                return 1
            else:
                return 0

    def findMaxMinPoint(self, posList):
        xPosList = []
        yPosList = []
        for pos in posList:
            xPosList.append(pos[0])
            yPosList.append(pos[1])

        maxX = max(xPosList)
        maxY = max(yPosList)
        minX = min(xPosList)
        minY = min(yPosList)

        return maxX, maxY, minX, minY

    # 判断是否存在一个点在av内部，如果在就是返回1，四个全部不在返回0
    def isPointInside(self, AVmaxX, AVmaxY, AVminX, AVminY, SVposList):
        # SVposList中的四个点分别是左上，左下，右下，右上
        posList = [SVposList[0], SVposList[3]]
        for pos in posList:
            if AVmaxX >= pos[0] >= AVminX and AVmaxY >= pos[1] >= AVminY:
                return 1
        return 0

    def coordinateTransformation(self, pos1, pos2):
        newPos = [pos2[0] - pos1[0], pos2[1] - pos1[1]]
        return newPos

    def rotate(self, pos, angle):
        x, y = pos[0], pos[1]
        # 将角度转换为弧度
        rad_angle = math.radians(angle)
        # 计算旋转后的坐标
        new_x = x * math.cos(rad_angle) - y * math.sin(rad_angle)
        new_y = x * math.sin(rad_angle) + y * math.cos(rad_angle)
        rotatePos = [new_x, new_y]
        return rotatePos


    def calculate_rectangle_vertices(self, length, width, angle) -> list:
        """ 计算矩形四个角点坐标
        :param length:    矩形的长
        :param width:     矩形的宽
        :param angle:     矩形的旋转角度，沿y轴顺时针旋转，就是tessng的angle
        :return:  矩形的四个角点坐标，依次是左上、左下、右下、右上
        """
        # 转为x轴正轴逆时针方向的角
        angle = self.convert_clockwise_to_counterclockwise(angle)
        angle = math.radians(angle)
        dx = length / 2
        dy = width / 2

        # 计算矩形角点相对于中心点的坐标偏移量
        vertex_offsets = [
            (-dx * math.cos(angle) - dy * math.sin(angle), -dx * math.sin(angle) + dy * math.cos(angle)),  # 左上角
            (-dx * math.cos(angle) + dy * math.sin(angle), -dx * math.sin(angle) - dy * math.cos(angle)),  # 左下角
            (dx * math.cos(angle) + dy * math.sin(angle), dx * math.sin(angle) - dy * math.cos(angle)),  # 右上角
            (dx * math.cos(angle) - dy * math.sin(angle), dx * math.sin(angle) + dy * math.cos(angle))  # 右下角
        ]

        # 计算矩形顶点的绝对坐标
        vertices = [(self.currentPos[0] + offset[0], self.currentPos[1] + offset[1]) for offset in vertex_offsets]
        self.bound = vertices
        return vertices

# 自动驾驶车
class SelfDrivingCar(Car, ScenarioLock, Scenario, Record):
    def __init__(self,  id, Xpos, Ypos, speed, roadType, startPos):
        Car.__init__(self, id, Xpos, Ypos, speed, roadType)
        ScenarioLock.__init__(self)
        Scenario.__init__(self)
        Record.__init__(self)
        # 自动驾驶车额外记录起点位置
        self.startPos = startPos
        # 行驶第几圈，初始为0
        self.loop = 0
        # Ego车开始运动的时刻
        self.startTimeOneLoop = 0
        # Ego车跑完一圈的时刻
        self.endTimeOneLoop = 0
        # 跑完一圈的行程时间 m
        self.travelingTime = 0
        # 行程平均速度 m/s，乘3.6就是km/h
        self.avgSpeed = 0
        self.avgSpeedKM = 0
        # 场景行程平均速度 km/h
        self.sceneAvgSpeed = 0
        self.sceneAvgSpeedKM = 0
        # 测试车起点开始状态获取锁
        self.egoStartStatusGetLock = 0
        # 记录跑圈数的临时变量
        self.tempLoop = 1
        # 自动驾驶车得名字
        self.name = 0
        # 自动驾驶车的QPoint
        self.QPoint = 0
        # 自动驾驶车的真实位置
        self.reaallScore_statisticslpos = []
        # 场景行驶距离
        self.scenarioDistance = 0
        # 场景行驶速度
        self.scenarioTravelSpeed = 0
        # 记录场景开始时间和结束时间
        self.recordTime = {}
        #
        self.scenarioStartTimeSimu = 0
        # 记录当前的场景号
        self.scenarioNumber = 0
        # 自动驾驶车长度
        self.length = 4.49
        # 自动驾驶车宽度
        self.width = 1.95
        # 自动驾驶车速度
        self.speed = 0
        self.speedreal = 0
        # 加速度
        self.acce = 0
        # 与前车的关系
        self.vehiHeadwayFront = 0
        self.vehiDistFront = 0
        # 自动驾驶车与SV中心点的距离表{Id: distance}
        self.distance_dict = {}
        self.angle_dict = {}
        self.svPos_dict = {}
        # 这两个分别用来代表，sv的速度，还有sv和av的相对位置，sv和av是对向行驶还是追击问题，如果是追击问题则self.svAimPlace_dict等于None，如果是追击问题则等于一个列表，【av与目标交点的距离，sv与目标交点的距离】
        # 另外svAimPlace_dict要删除的更频繁，每次判断如果车辆不再是相交车辆或者跟驰车辆要及时删除
        self.svAimPlace_dict = {}
        self.svSpeed_dict = {}
        # 用来记录潜在撞车的位置信息，value是列表记录横纵坐标，目标是用来判断如果和相交车辆的交点之间有一个跟驰车辆，则就只考虑跟驰车辆
        self.potentialTTCcatCrashPoint_dict = {}
        self.currentSVcars = []
        # 车辆几何信息
        self.bound = []
        # TTC小于1的时间计数
        self.TTC_LowTime = 0
        # TTC值
        self.TTC = 10
        # PET值
        self.PET = {}
        self.petAV = 0
        # 拒绝的间隙总列表，每个冲突点可能存在多个间隙列表，最终要处理到，每个冲突点对应一个间隙列表，保存在新的列表中
        self.gapAllList = []
        self.gapList = []
        # 记录gap的时候要记录一下当前的时刻，方便后续操作使用
        self.gapRecordTimeList = []
        self.gapRecordTimeListNotALL = []
        # 测试后的评价
        self.afterTestEvaluate = {}
        # Ego车的连接段对象
        self.connector = 0
        # Ego在下一帧将被移动到的车道对象,以及对象的Id,对象的车道编号(0, 1, 2...)
        self.targetLaneObject = 0
        self.targetLaneObjectId = 0
        self.targetLaneObjectNumber = 0
        # 自动驾驶车的航向角
        self.angle = 0
        # 记录自动驾驶车的轨迹
        self.posList = []
        # 所在车道和当前link有几条车道
        self.currLaneId = 0
        self.totallLaneId = 1
        # 计算经度也需要，这样才能计算加加速度之类的东西
        self.accuracy = 0
        # 记录自己车辆在指定最初的位置后，生成的路径
        self.initialRouting = []
        # 虽然不想在这里添加额外的东西，但是这个是必需的，需要记录一下ego的字典，因为放在外面没有办法储存起来
        self.scoreDict = {'碰撞': 0,
                          '驶出行车道': 0,
                          'TTC': 0,
                          'PET': 0,
                          '驶入对向车道': 0,
                          '压实线': 0,
                          '超速': 0,
                          '闯红灯': 0,
                          '横向间距': 0,
                          '禁行区行驶': 0,
                          "未按规定车道行驶": 0,
                          "停车压停止线": 0,
                          "未按规定路线行驶": 0,
                          "是否完成任务": 0,
                          '横向加速度': 0,
                          '横向加加速度': 0,
                          '纵向加速度': 0,
                          '纵向加加速度': 0,
                          '转向角变化': 0}
        self.scoreAccuTimeDict = {'碰撞累积时间': 0,
                                  '驶出行车道累积时间': 0,
                                  'TTC累积时间': 0,
                                  'PET累积时间': 0,
                                  '驶入对向车道累积时间': 0,
                                  '压实线累积时间': 0,
                                  '超速累积时间': 0,
                                  '闯红灯累积时间': 0,
                                  '横向间距累积时间': 0,
                                  '禁行区行驶累积时间': 0,
                                  "未按规定车道行驶累积时间": 0,
                                  "停车压停止线累积时间": 0,
                                  "未按规定路线行驶累积时间": 0,
                                  "是否完成任务累积时间": 0,
                                  '横向加速度累积时间': 0,
                                  '横向加加速度累积时间': 0,
                                  '纵向加速度累积时间': 0,
                                  '纵向加加速度累积时间': 0,
                                  '转向角变化累积时间': 0}
        self.scoreAccuNumDict = {'横向加速度累积次数': 0,
                                 '横向加加速度累积次数': 0,
                                 '纵向加速度累积次数': 0,
                                 '纵向加加速度累积次数': 0,
                                 "未按规定路线行驶累积次数": 0,
                                 "闯红灯累积次数": 0,
                                 "碰撞累积次数": 0,}
        # 这个字典不只是记录状态，而是记录上次发生事件的时间，如果当前事件的时间小于上次发生事件的时间+3s，则认为是同一次事件
        self.scoreContiNumDict = {'碰撞上帧状态': 0,
                                  '驶出行车道上帧状态': 0,
                                  'TTC上帧状态': 0,
                                  'PET上帧状态': 0,
                                  '驶入对向车道上帧状态': 0,
                                  '压实线上帧状态': 0,
                                  '超速上帧状态': 0,
                                  '闯红灯上帧状态': 0,
                                  '横向间距上帧状态': 0,
                                  '禁行区行驶上帧状态': 0,
                                  "未按规定车道行驶上帧状态": 0,
                                  "停车压停止线上帧状态": 0,
                                  "未按规定路线行驶上帧状态": 0,
                                  '横向加速度上帧状态': 0,
                                  '横向加加速度上帧状态': 0,
                                  '纵向加速度上帧状态': 0,
                                  '纵向加加速度上帧状态': 0, }
        # 针对效率指标来一个额外的积分的地方
        self.scoreEfficiency = {"平均速度":0,
                                "拒绝间隙":0,
                                "行程时间":0,
                                "任务完成":0,}
        # 用来放违规事件扣分的表格，这里别忘了清零
        self.eventTable = {}

        # 存放加速度的相关数据
        self.   acceAll = {}

        # 判断是否完成了任务，如果被标记为1则一直为1
        self.calMissionAccomplish = 0

    # 计算一下真实加速度
    def calcuAcce(self):
        if len(self.posList) > 2:
            speed1 = self.posList[-1]['speedreal']  # 最新的点的速度，也就是当前速度
            speed2 = self.posList[-2]['speedreal']  # 次新的点的速度，也就是上一针的速度
            self.acce = (speed1 - speed2)/self.accuracy
            # print("当前速度",speed1,"上一针速度", speed2,"时间间隔", self.accuracy,"加速度",self.acce)
        else:
            self.acce = 0


    # 计算总行程的 平均速度，和 当前 场景的平均速度
    def calculateAvgSpeed(self, allTime, sceneTime):
        if allTime and sceneTime:
            self.avgSpeed = self.distance / allTime
            self.avgSpeedKM = round(self.avgSpeed * 3.6, 2)
            self.sceneAvgSpeedKM = round((self.sceneDistance * 3.6) / sceneTime, 2)
            self.sceneAvgSpeed = round((self.sceneDistance) / sceneTime, 2)

    # 计算当前坐标与起点间的距离，用来看是否成功跑了一圈
    def calculateDistance(self):
        p1 = np.array([self.currentPos[0], self.currentPos[1]])
        p2 = np.array([self.startPos[0], self.startPos[1]])
        p3 = p2 - p1
        d = math.hypot(p3[0], p3[1])
        return d

    # 专门计算场景行驶距离的函数
    def calculateScenarioTravelDistance(self, distance):
        self.scenarioDistance += distance
        self.scenarioDistance = round(self.scenarioDistance, 2)

    # 统计跑了几圈，初始为 1
    def drivingLoopTimes(self):
        d = self.calculateDistance()
        # print(d, "当前坐标", self.currentPos, "起始坐标", self.startPos, self.startTimeOneLoop)
        if d <= 3 and d != 0 and time.time() - self.startTimeOneLoop > 20:
            self.tempLoop = self.loop
            # 行驶圈数 + 1
            self.loop += 1
            # 获取当前圈跑完的时刻
            self.endTimeOneLoop = time.time()
            # print(self.endTimeOneLoop)
            # 获取Ego车的行程时间
            self.travelingTime = round(self.endTimeOneLoop - self.startTimeOneLoop, 2)
            # 下一圈，重置开始时间
            self.startTimeOneLoop = time.time()
            # 获取Ego车的行程平均速度
            self.avgSpeed = round(self.distance / self.travelingTime, 2)
            if self.avgSpeed < 0.1:
                self.loop -= 1
            else:
                print("行程时间", self.travelingTime, "危险时间", self.TTC_LowTime, "平均速度", self.avgSpeed, "总里程", self.distance, "行驶圈数为", self.loop, "危险时间占比", round((self.TTC_LowTime / self.travelingTime), 2))
            # 里程清零
            self.distance = 0
            # 打开记录锁
            self.egoStartStatusGetLock = 0
        else:
            pass

    # 判断点是否在一个矩形内部，用于计算是否在场景开始面域和场景结束面域
    def is_point_inside_rect(self, rect) -> bool:
        """
        Check if a point is inside a rectangle defined by two points (top-left and bottom-right).
        Returns:
        - bool: True if the point is inside the rectangle, False otherwise.
        """
        if self.currentPos:
            x, y = self.currentPos[0], -self.currentPos[1]
            x1, y1 = rect[0]
            x2, y2 = rect[1]

            if x1 <= x <= x2 and y1 <= y <= y2:
                return True
            else:
                return False

    # 计算当前场景编号以及车辆处于开始面域/结束面域
    def scenarioJudge(self, scenarioDict: dict) -> Tuple[int, int]:
        """

        Args:
            scenarioDict: 场景面域集合

        Returns: 场景编号，1: 开始面域，2: 结束面域

        """
        for scenarioNum, scenarioSection in scenarioDict.items():
            if self.is_point_inside_rect(scenarioSection['startSection']):
                # 1 means in the startSection
                # print(scenarioNum, "号场景", "正处于开始面域")
                return scenarioNum, 1
            if self.is_point_inside_rect(scenarioSection['endSection']):
                # 2 means in the endSection
                # print(scenarioNum, "号场景", "正处于结束面域")
                return scenarioNum, 2
            else:
                # Ego keep running
                continue

    # 开始了下一圈，事件清零
    def restart(self) -> bool:
        if self.tempLoop != self.loop:
            self.tempLoop = self.loop
            return True
        else:
            return False

    # 如果一圈跑完了，则清除所有记录信息
    def clearAll(self):
        if self.restart():
            self.distance = 0
            self.recordTime = {}
            self.recordLastTime = {}

    # 把路过的点都储存起来
    def record_posList(self, Xpos, Ypos, t):
        current_pos = {'id': self.id, 'x': Xpos, 'y': Ypos,
                       'simutime': t,
                       'speed': self.speed,
                       'speedreal': self.speedreal,
                       'acce': self.acce,
                       'type': self.vehicleType,
                       'angle': self.angle,
                       'HeadwayFront': self.vehiHeadwayFront,
                       'DistFront': self.vehiDistFront}
        self.posList.append(current_pos)
        # 当list中存在至少两个点时，计算最后一个点和第一个点之间的时间差，超过60秒就删除第一个点
        if len(self.posList) > 2:
            a = self.posList[len(self.posList) - 1]['simutime'] - self.posList[0]['simutime']
            if a > 120:
                del self.posList[0]

    def calculate_rectangle_vertices(self, length, width, angle) -> list:
        """ 计算矩形四个角点坐标
        :param length:    矩形的长
        :param width:     矩形的宽
        :param angle:     矩形的旋转角度，沿y轴顺时针旋转，就是tessng的angle
        :return:  矩形的四个角点坐标，依次是左上、左下、右下、右上
        """
        # 转为x轴正轴逆时针方向的角
        angle = self.convert_clockwise_to_counterclockwise(angle)
        angle = math.radians(angle)
        dx = length / 2
        dy = width / 2

        # 计算矩形角点相对于中心点的坐标偏移量
        vertex_offsets = [
            (-dx * math.cos(angle) - dy * math.sin(angle), -dx * math.sin(angle) + dy * math.cos(angle)),  # 左上角
            (-dx * math.cos(angle) + dy * math.sin(angle), -dx * math.sin(angle) - dy * math.cos(angle)),  # 左下角
            (dx * math.cos(angle) + dy * math.sin(angle), dx * math.sin(angle) - dy * math.cos(angle)),  # 右上角
            (dx * math.cos(angle) - dy * math.sin(angle), dx * math.sin(angle) + dy * math.cos(angle))  # 右下角
        ]

        # 计算矩形顶点的绝对坐标
        vertices = [(self.currentPos[0] + offset[0], self.currentPos[1] + offset[1]) for offset in vertex_offsets]
        self.bound = vertices
        return vertices

    # TTC计算
    def calculate_ttc(self, distance: float, SV_speed: float, SV_id: int) -> float:
        """
        Calculate the Time to Collision (TTC) using the constant velocity model.

        Args:
        - distance (float): Distance between the two objects.
        - relative_velocity (float): Relative velocity between the two objects.

        Returns:
        - float: Time to Collision (TTC) in seconds.
        """
        # 首先要根据self.aimplacedict来判断主车和目标sv车的情况，是相向，还是对向
        # print(SV_id, "是否对向",self.svAimPlace_dict[SV_id], "主车速度",self.speed, "从车速度",SV_speed, distance)
        if self.svAimPlace_dict[SV_id] != None:
            # 证明是对向行驶
            if self.speed != 0 and SV_speed != 0:
                timeMain = self.svAimPlace_dict[SV_id][0]/self.speed
                timeSV = self.svAimPlace_dict[SV_id][1]/SV_speed
                if timeMain > 10 and timeSV > 10:
                    return 10
                ttc = abs(timeMain - timeSV)
                # ttc = max(timeMain, timeSV)
                return ttc
            else:
                return 10
        else:
            # 证明是相同方向行驶，判断self.speed - SV_speed，如果为正，则可能能追上sv，可以计算后面的值
            relative_velocity = self.speed - SV_speed
            if relative_velocity > 0:
                ttc = distance / relative_velocity
                return ttc
            else:
                return 10

    # 统计每辆SV与AV的距离
    def recordDistanceBetweenSimulatorVehicle(self, SV_Id, SV_Pos, SV_Angle, SV_Speed) -> None:
        d = self.pointDistanceBetweenSV(SV_Pos)
        self.distance_dict[SV_Id] = round(d, 2)
        self.angle_dict[SV_Id] = SV_Angle
        self.svPos_dict[SV_Id] = SV_Pos
        self.svSpeed_dict[SV_Id] = SV_Speed
        # if SV_Id == 100011:
        #     print(self.currentPos, SV_Pos)
        #     print(100011, self.distance_dict[100011])

    # 清楚多余的sv车辆信息
    def clearSVinforDict(self):
        # 要删除额外的车辆
        cars_to_remove = []
        for key in self.distance_dict.keys():
            if key not in self.currentSVcars:
                cars_to_remove.append(key)
        for key in cars_to_remove:
            if key in self.distance_dict:
                del self.distance_dict[key]
            if key in self.angle_dict:
                del self.angle_dict[key]
            if key in self.svPos_dict:
                del self.svPos_dict[key]
            if key in self.svAimPlace_dict:
                del self.svAimPlace_dict[key]
            if key in self.svSpeed_dict:
                del self.svSpeed_dict[key]
            if key in self.potentialTTCcatCrashPoint_dict:
                del self.potentialTTCcatCrashPoint_dict[key]



    def calculate_rotation_angle(self, point1, point2):
        # 计算相对于Y轴的角度，以顺时针方向为正方向
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        angle_rad = math.atan2(delta_x, delta_y)
        # 将弧度转换为角度
        angle_deg = math.degrees(angle_rad)
        # 调整角度范围为0到180度，因为只需要知道两者是否为同道路
        angle_deg = (angle_deg + 180) % 180
        return angle_deg

    def calculate_angle_similar(self, anglePos, angleCar, rangeAngel):
        # 这里操作的角度都是x正方向为0度，逆时针旋转
        if anglePos > angleCar:
            if 360 - rangeAngel / 2 > anglePos - angleCar > rangeAngel / 2:
                a = False
            else:
                a = True
        else:
            if 360 - rangeAngel / 2 > 360 - (angleCar - anglePos) > rangeAngel / 2:
                a = False
            else:
                a = True
        return a

    # 计算两个车是否会存在相交
    def check_intersection(self, position1, direction1, position2, direction2, rangeAngel, sv_id):
        # 车辆1的位置和方向角度
        x1, y1 = position1
        direction1 = self.convert_angle(direction1)  # 车辆1的方向角度， 车1是主车

        # 车辆2的位置和方向角度
        x2, y2 = position2
        direction2 = self.convert_angle(direction2)  # 车辆2的方向角度

        # 计算斜率
        m1 = math.tan(math.radians(direction1))
        m2 = math.tan(math.radians(direction2))
        # print(sv_id,"主车角度",direction1,"sv角度",direction2)
        # 判断是否平行,有一个范围确定是否在前面，不是必须要完全相同
        if self.calculate_angle_similar(direction1, direction2, 30):
            # print("主车和从车是平行")
            # 如果是平行则需要再额外判断一步，判断从车是否在主车的正前方，在主车正前方探测角度是180，则只要在y轴上领先主车，都算是在前方，侧前方也是，如果是0，则必须x轴相同，且y轴领先，才算是在前方
            # 这里还可以再加个条件，当主车和从车相距越远，则判断正前方的条件越苛刻，离得越近，判断就会相对宽松
            dis = pow((pow(x1 - x2, 2) + pow(y1 - y2, 2)), 0.5)
            if dis > 20:
                angelFront = 10
            elif dis > 10:
                angelFront = 10
            else:
                angelFront = 20
            if self.in_front_180_degree_range(x1, y1, direction1, x2, y2, angelFront, sv_id):
                # 如果sv在主车前方10度范围内，则证明是的，否则就是平行且不再前面
                # 在这时，需要把svAimPlace_dict的这个车的标记变成None，这样后面就知道这个车和主车的关系是，sv在主车前面，并且sv和主车同向行驶
                self.svAimPlace_dict[sv_id] = None
                # 这里是跟驰的情况，则crash点是前车从车的xy坐标
                self.potentialTTCcatCrashPoint_dict[sv_id] = [x2, y2]
                return True
            return False

        # 计算交点的x坐标
        x_intersect = (m2 * x2 - m1 * x1 + y1 - y2) / (m2 - m1)

        # 计算交点的y坐标
        y_intersect = m1 * (x_intersect - x1) + y1
        # print(sv_id, "交点坐标为：", (x_intersect, y_intersect))

        # 判断目标位置是否在车辆前方180度范围内
        # 同时也要判断目标位置是否在sv车辆前方180的范围
        if self.in_front_180_degree_range(x1, y1, direction1, x_intersect, y_intersect, rangeAngel, 100) and self.in_front_180_degree_range(x2, y2, direction2, x_intersect, y_intersect, rangeAngel, sv_id):
            # # 在这时，需要把svAimPlace_dict的这个车的标记变成主车与交点坐标的距离，这样后面就知道这个车和主车的关系是，sv在主车前面，并且sv和主车对向行驶，碰撞点在距离主车多少米的地方
            disOfCrashPoint = self.pointDistanceBetweenSV([x_intersect, y_intersect])
            disOfCrashPointSV = pow((pow(x_intersect - x2, 2) + pow(y_intersect - y2, 2)), 0.5)
            self.svAimPlace_dict[sv_id] = [disOfCrashPoint,disOfCrashPointSV]
            # 这里是相交的情况，则crash点是主车和从车的交点坐标
            self.potentialTTCcatCrashPoint_dict[sv_id] = [x_intersect, y_intersect]
            return True
        else:
            return False

    def angle_between_points(self, x1, y1, x2, y2):
        """计算两点之间的方向角度"""
        # 这里计算出来的结果是，x正方向为0度，逆时针选择为正到180为止，顺时针选择180为-到-179.99为止
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        # 通过下面的可以return，x正方向为0度，逆时针旋转
        if angle > 0:
            return angle
        else:
            return 360 + angle

    def in_front_180_degree_range(self, vehicle_x, vehicle_y, vehicle_direction, target_x, target_y, rangeAngel, sv_id):
        """判断目标位置是否在车辆前方180度范围内"""
        # 计算车辆当前位置与目标位置之间的方向角度
        target_angle = self.angle_between_points(vehicle_x, vehicle_y, target_x, target_y)
        # print(sv_id, "目标点和当前位置的角度",target_angle, "当前车辆角度", vehicle_direction)
        # 调整车辆的方向角度到0到360度的范围内
        adjusted_vehicle_direction = vehicle_direction % 360
        anglePos = target_angle
        angleCar = adjusted_vehicle_direction
        if anglePos > angleCar:
            if 360 - rangeAngel / 2 > anglePos - angleCar > rangeAngel / 2:
                a = False
            else:
                a = True
        else:
            if 360 - rangeAngel / 2 > 360 - (angleCar - anglePos) > rangeAngel / 2:
                a = False
            else:
                a = True
        # print("目标位置是否在车辆前方180度范围内",a)
        return a

    def convert_angle(self, angle):
        # 将角度从以y轴正方向为0度，顺时针旋转的角度转换为以y轴负方向为0度，逆时针旋转的角度
        # anticlockwise_angle_y = (360 - angle) % 360

        # 将角度从以y轴负方向为0度，逆时针旋转的角度转换为以x轴正方向为0度，逆时针旋转的角度
        anticlockwise_angle_x = (90 - angle) % 360

        return anticlockwise_angle_x

    def chooseCorrectTTCcar(self, potentialTTCcar):
        # 首先传入进来的potentialTTCcar是一个字典，key是id，value是ttc的值
        # 其次还需要一个字典key是id，value是交点坐标或者是跟驰车辆的坐标，self.potentialTTCcatCrashPoint_dict
        # 最后还需要一个字典，self.svAimPlace_dict，这个字典就是key是id， value是列表，是撞击点距离sv和av的距离，或者none，none就表示跟驰
        # 因为只要是potentialTTCcar，则一定是跟驰或者相交，而跟驰一定在车辆行进方向，相交也一定在车辆行进方向，所以其实本质就是判断距离就行了，距离av的交点距离与跟驰车和av的距离。
        # 那其实不需要点位信息淦，只需要把跟驰的车辆距离加到self.svAimPlace_dict里面就行了，self.distance_dict正好这个也是有距离信息的
        # 目标就是首先对potentialTTCcar这个字典进行清理，把里面不合规的相交的车辆清洗出去，也就是那些交点之间有跟驰车的情况
        # print("处理之前的所有可能的ttc车辆和对应的ttc",potentialTTCcar)
        minDisFollowCarDis = 99999
        minDisFollowCarId = None
        keepList = []
        # 首先把所有跟驰的车找到，并且算出来跟驰车和主车的距离，主要是找最近的跟驰车，其他跟驰车都要去掉
        for key in self.svAimPlace_dict:
            if self.svAimPlace_dict[key] == None:
                if minDisFollowCarDis > self.distance_dict[key]:
                    minDisFollowCarId = key
                    minDisFollowCarDis = self.distance_dict[key]
        if minDisFollowCarId:
            keepList.append(minDisFollowCarId)

        # minDisForFollowCar = min(followCarDisDict, key=followCarDisDict.get)
        # 其次是for循环找出有交点的ttc车辆，选中某些车放在keeplist里面
        for key in self.svAimPlace_dict:
            if self.svAimPlace_dict[key] != None:
                # 证明是存在焦点的
                # 这里还要在加一个判断，就是，判断self.svAimPlace_dict[key]的两个值都要小于50，并且加了这个判断之后就有可能吧所有原本的潜在车辆都删除了
                if self.svAimPlace_dict[key][0] < 50 and self.svAimPlace_dict[key][1] < 50:
                    disForAV = self.svAimPlace_dict[key][0]
                    # 判断是否存在跟驰车在他们中间，如果最近的跟驰车都不在，那其他也必定不在
                    # print(key, disForAV, minDisFollowCarDis)
                    if disForAV < minDisFollowCarDis:
                        keepList.append(key)

        # 最后是删除所有不在keeplist里面的车
        # 使用字典推导式过滤出与列表共同的key和对应的value
        potentialTTCcar = {key: value for key, value in potentialTTCcar.items() if key in keepList}

        # print("处理之后的所有可能的ttc车辆和对应的ttc", potentialTTCcar)
        # 最后对比所有ttccar的ttc，最终找到minDistancePotentialTTCcarId
        if potentialTTCcar:
            minDistancePotentialTTCcarId = min(potentialTTCcar, key=potentialTTCcar.get)
        else:
            minDistancePotentialTTCcarId = None

        return minDistancePotentialTTCcarId

    # 寻找适合的ttc车辆有几个条件，首先就是要判断两个车是否存在相撞的可能，然后判断相对距离
    # 如果两个车角度小于45度
    def findTTCVehicle(self):
        # 首先要清除svAimPlace_dict，因为每一帧都要重新计算当前车辆的ttc位置关系
        self.svAimPlace_dict = {}
        potentialTTCcar = {}
        # 计算两个车辆的角度差
        # print(self.angle_dict)
        if self.angle_dict:
            for key in self.angle_dict:
                # 前方的探测范围
                rangeAngel = 60
                # 首先判断辆车是否可能存在交点,或者是否平行属于同一个车道，如果是就记录key
                if self.check_intersection(self.currentPos, self.angle, self.svPos_dict[key], self.angle_dict[key], rangeAngel, key):
                    # 在这里直接计算当前车与主车的ttc值，并且把这个值作为value放在potentialTTCcar里面
                    potentialTTCcar[key] = self.calculate_ttc(self.distance_dict[key], self.svSpeed_dict[key], key)
                    # if potentialTTCcar[key] < 3:
                        # print(key,"TTC:", potentialTTCcar[key], "是否对向",self.svAimPlace_dict[key], "主车速度",self.speed, "从车速度", self.svSpeed_dict[key])
                    # potentialTTCcar[key] = self.distance_dict[key]
            # 找到所有潜在ttc车之后就可以开始计算所以ttc值最小的车作为ttc车了
            if potentialTTCcar:
                # print(potentialTTCcar)
                # 在这里需要再加一次额外筛选，就是如果存在相交车还有跟驰车，则需要判断跟驰车是不是在相交车的交点之间，如果在则删除这辆相交车
                minDistancePotentialTTCcarId = self.chooseCorrectTTCcar(potentialTTCcar)
                # minDistancePotentialTTCcarId = min(potentialTTCcar, key=potentialTTCcar.get)
                # print(minDistancePotentialTTCcarId, self.angle_dict[minDistancePotentialTTCcarId], self.angle)
                return minDistancePotentialTTCcarId

    # 找到离Ego车最近的仿真车Id
    def findClosestSimulatorVehicle(self) -> int:
        # 距离是一个列表，那我只需要筛选这个列表中车辆小于20的就行了
        if self.distance_dict:
            minDistanceSimulatorVehicleId = min(self.distance_dict, key=self.distance_dict.get)
            return minDistanceSimulatorVehicleId

    # 找到最近20米的仿真车id
    def findCloseSimulatorVehicle(self) -> list:
        # 距离是一个列表，那我只需要筛选这个列表中车辆小于20的就行了
        # if 400008 in self.distance_dict:
        #     print(400008, self.distance_dict[400008])
        # else:
        #     print('不在列表')
        clossDistanceSimulatorVehicleIdList = [i for i in self.distance_dict if self.distance_dict[i] <= 20]
        return clossDistanceSimulatorVehicleIdList

    # 找所有车的id
    def findAllSimulatorVehicle(self) -> list:
        # 把字典中所有的键值都取出来就行了distance_dict
        allSimulatorVehicleIdList = list(self.distance_dict.keys())
        return allSimulatorVehicleIdList

        # 整体测试后的评价
    def afterTestEvaluateFunc(self) -> dict:
        # 有了行程速度和行程时间之后，才开始整体测试评价
        if self.avgSpeed and self.travelingTime:
            self.afterTestEvaluate = {"desireTravelSpeed": testDesireConsume['desireSpeed'], "dangerousTime": self.TTC_LowTime,
                                      "dangerousTimePercent": round((self.TTC_LowTime/self.travelingTime), 3) * 100, "TravelSpeed": round(self.avgSpeed * 3.6, 2),
                                      "TravelTime": self.travelingTime, "desireTravelTravelTime": self.travelingTime}
            return self.afterTestEvaluate
        # 测试没结束，则整体测试评价数据都为0
        else:
            self.afterTestEvaluate = {"desireTravelSpeed": testDesireConsume['desireSpeed'], "dangerousTime": 0, "dangerousTimePercent": 0, "TravelSpeed": 0, "TravelTime": 0}
            return self.afterTestEvaluate

    # 处理需要发送的仿真数据
    def processEvaluation(self, EgoBound, TTC_CarBound, TTC_CarSpeed, TTC_CarId) -> dict:
        """

        Args:
            EgoBound: Ego的车辆二维边界
            TTC_CarBound: TTC车辆(TESSNG Car)的二维边界
            TTC_CarSpeed: TTC车辆的速度
            batchNum: 当前TESSNG仿真批次

        Returns: 场景评价数据

        """
        # 在这里来改ttc的计算方式，self.calculate_ttc
        # 可以在这里实现新的ttc计算方式
        TTC_distance = find_minimum_distance(EgoBound, TTC_CarBound)
        self.TTC = self.calculate_ttc(distance=TTC_distance, SV_speed=TTC_CarSpeed, SV_id=TTC_CarId)
        if self.TTC:
            if self.TTC > 10:
                self.TTC = 10.0
            # 整体测试评价
            afterTestEvaluation = self.afterTestEvaluateFunc()
            if self.recordLastTime:
                recordLastTime = self.recordLastTime
                value = list(recordLastTime.values())[0]
                self.TTC = round(self.TTC, 2)
                # 如果持续时间大于0才开始考虑处理
                if value["realLastTime"] >= 0:
                    EvaluateData = {"timeStamp": str(datetime.now())[:23],
                                         "frontCrashTime": self.TTC, "frontCrashType": judgeTTC(self.TTC),
                                         "jobConsumeTime": value["realLastTime"],
                                         "jobHopeTime": value["jobHopeTime"]}
                    # 合并场景评价数据以及测试结果数据，测试结果数据要当测试全部结束后才会生成，后期测试开始和结束应该都需要有一个http接口去接收信号
                    EvaluateData = Merge(EvaluateData, afterTestEvaluation)
                else:
                    EvaluateData = {"timeStamp": str(datetime.now())[:23],
                                     "frontCrashTime": self.TTC, "frontCrashType": judgeTTC(self.TTC),
                                     "jobConsumeTime": 0, "jobHopeTime": value["jobHopeTime"]}
                    EvaluateData = Merge(EvaluateData, afterTestEvaluation)
                self.distance_dict = {}
                self.angle_dict = {}
                self.svPos_dict = {}
                self.svAimPlace_dict = {}
                self.svSpeed_dict = {}
                self.potentialTTCcatCrashPoint_dict = {}
            else:
                EvaluateData = {"timeStamp": str(datetime.now())[:23],
                                         "frontCrashTime": self.TTC, "frontCrashType": judgeTTC(self.TTC), "jobConsumeTime": 0,
                                         "jobHopeTime": 11.7}
                EvaluateData = Merge(EvaluateData, afterTestEvaluation)
            return EvaluateData
        else:
            return {}

    # 统计TTC处于危险状态的时间
    def TTC_statistics(self, accurate):
        if self.TTC:
            if self.TTC < 2:
                self.TTC_LowTime += accurate
                self.TTC_LowTime = round(self.TTC_LowTime, 2)
                # print("当前TTC小于1的总时间为", self.TTC_LowTime, "TTC值为", self.TTC)

    # 统计分数处于不合格时候的危险时间，如果记录指标的值是1，则用这个函数计算累加值
    def accuTime(self, score, accurate, key, simuTime):
        timeKey = key + '累积时间'
        numKey = key + '累积次数'
        contiKey = key + '上帧状态'
        listScore = ['横向加速度', '横向加加速度', '纵向加速度', '纵向加加速度', "未按规定路线行驶",]
        listEvent = ['碰撞','驶出行车道','TTC','驶入对向车道','压实线','超速','闯红灯','横向间距','禁行区行驶',"未按规定车道行驶","停车压停止线",]
        # 是否完成任务不在这里的计算范围内，是单独计算，但是还是累计了完成的时间
        if score:
            # print(key)
            self.scoreAccuTimeDict[timeKey] += accurate
            # self.scoreAccuTimeDict[timeKey] = round(self.scoreAccuTimeDict[timeKey], 2)
            # 这里第一次加进来，就是0.03秒，然后四舍五入一下，是0，下一次还是0.03加进来还是0啊
            self.scoreAccuTimeDict[timeKey] = self.scoreAccuTimeDict[timeKey]
            # print("当前TTC小于1的总时间为", self.TTC_LowTime, "TTC值为", self.TTC)
            # 下面这个是计数的，上面的是计时的
            # 下面的+1和+3表示的是，只要计时一次，无敌时间分别是1秒和3秒，也就是说1秒内所有帧的错误，都只能算1次，30秒的测试，最多只有30次错误，10次错误
            if key in listScore:
                if self.scoreContiNumDict[contiKey] == 0 or simuTime > self.scoreContiNumDict[contiKey] + 1:
                    # 如这个是0表示上一次的状态是正常没有违规，那么这次违规了，dict就要加一次
                    self.scoreAccuNumDict[numKey] += 1
                    # 这样标记成1以后，下次如果还违章就不会在被加1了，直到下次不违章了，就可以了重置，等等下次再位置
                    self.scoreContiNumDict[contiKey] = simuTime
            if key in listEvent:
                # 用这个字典记录上次事件的时间，如果当前事件大于上次事件+1秒，则认为是新事件了，不管是不是新事件，都要把记录的时间更新
                # 加上self.scoreContiNumDict[contiKey] == 0防止前三秒无敌
                if self.scoreContiNumDict[contiKey] == 0 or simuTime > self.scoreContiNumDict[contiKey] + 1:
                    # 现在是新的事件了，开始对eventTable进行操作
                    event = {"time": str(datetime.now())[:23], "place":self.longitudeAndLatitude, "index": key, "simuTime": simuTime,}
                    self.eventTable[len(self.eventTable)+1] = event
                    # 碰撞比较特殊还需要统计一下， self.scoreAccuNumDict[碰撞]的次数
                    if key == "碰撞" or key == "闯红灯":
                        self.scoreAccuNumDict[numKey] += 1
                    self.scoreContiNumDict[contiKey] = simuTime
                else:
                    # 还是上次事件
                    pass

        # else:
        #     if contiKey in self.scoreContiNumDict:
        #         self.scoreContiNumDict[contiKey] = 0

        # 这里如果想要把某些指标按次扣分，某些指标按时间扣分，据只需要再这里的list和上面的self.scoreAccuNumDict里面增加相应的数字就行了

    # clear每个场景结束后，清除所有的EGO的数据，主要是ttc这种之类的
    def clearAllEgoData(self):
        self.accuTimeToZero()
        self.TTC = 10
        self.sceneDistance = 0
        self.gapAllList = []
        self.gapList = []
        self.gapRecordTimeList = []
        self.gapRecordTimeListNotALL = []
        self.eventTable = {}
        self.calMissionAccomplish = 0

    # 在每个场景结束的时候，都要把这个累计的数值变成0
    def accuTimeToZero(self):
        for key in self.scoreAccuTimeDict:
            self.scoreAccuTimeDict[key] = 0
        for key in self.scoreAccuNumDict:
            self.scoreAccuNumDict[key] = 0

    # 计算列表中重复出现的时间，把列表放在一个里面，想法非常简单，只要最后加进来的数据的时间与之前加过的时间差在一秒以上，就是证明是不同的冲突点
    def calcuGapList(self):
        # 全部里面要有数值，冲突点里面要没有数值
        if not self.gapList and self.gapAllList:
            if self.gapAllList[-1]!=None:
                self.gapList.append(self.gapAllList[-1])
                self.gapRecordTimeListNotALL.append(self.gapRecordTimeList[-1])
        else:
            if abs(self.gapRecordTimeListNotALL[-1] - self.gapRecordTimeList[-1]) > 0.8:
                if self.gapAllList[-1]!=None:
                    self.gapList.append(self.gapAllList[-1])
                    self.gapRecordTimeListNotALL.append(self.gapRecordTimeList[-1])
        # self.gapList里面存放的就是每个冲突点上存在的

    # 计算效率分数，当前列表中扣的分数
    def calcuGapMiniScore(self, expectTime, missonTime, weightDict):
        # 这里的计算方式，找出self.gapList中属于对应区间的所有间隙，然后按照间隙数量乘分数来扣分，这个扣得分数还要和完成任务的时间来进行结合
        miniScoreGap = 0
        # 时间占比评价分数
        # gapPerc = 1 - weightDict["usedTime"]/100 - weightDict["averageSpeed"]/100

        if self.gapList:
            all = []
            for iList in self.gapList:
                for i in iList:
                   all.append(i)
            # 把所有间隙都合并在all里面
            score = pd.Series(all)
            se1 = pd.cut(score, [0, 3, 5, 8, 30])
            countList = se1.value_counts().sort_index()
            countdict = {'无法穿越间隙': countList[0],
                         '激进穿越间隙': countList[1],
                         '正常穿越间隙': countList[2],
                         '保守穿越间隙': countList[3],
                         '冲突点个数': len(self.gapList)}
            miniScoreGap = 5 * countList[1] + 15 * countList[2] + 35 * countList[3]
            # print(countdict,all)

        if missonTime > expectTime:
            miniScoreTime = 100 - (expectTime / missonTime) * 100
        else:
            miniScoreTime = 0

        scoreSpeed = self.sceneAvgSpeedKM / Parameter.avgSpeedKM
        if self.sceneAvgSpeedKM < Parameter.avgSpeedKM:
            miniScoreSpeed = 100 - (self.sceneAvgSpeedKM / Parameter.avgSpeedKM) * 100
        else:
            miniScoreSpeed = 0

        miniScoreMissionAccomplish = (1 - self.calMissionAccomplish) * 100

        # print("时间扣分",miniScoreTime,"速度扣分",miniScoreSpeed,"间隙扣分",miniScoreGap)
        totalMiniScore = (weightDict["usedTime"]/100) * miniScoreTime + (weightDict["gapRefuse"]/100) * miniScoreGap + miniScoreSpeed * (weightDict["averageSpeed"]/100) + miniScoreMissionAccomplish * (weightDict["missionAccomplish"]/100)
        # 用的是要减的分数还是正的分数，
        self.scoreEfficiency["行程时间"] = (weightDict["usedTime"]/100) * miniScoreTime
        self.scoreEfficiency["平均速度"] = (weightDict["averageSpeed"]/100) * miniScoreSpeed
        self.scoreEfficiency["拒绝间隙"] = (weightDict["gapRefuse"]/100) * miniScoreGap
        self.scoreEfficiency["任务完成"] = (weightDict["missionAccomplish"]/100) * miniScoreMissionAccomplish


        return totalMiniScore


    def allScore_statistics(self, accuracy, simuTime):
        for key in self.scoreDict:
            score = self.scoreDict[key]
            self.accuTime(score, accuracy, key, simuTime)


    # 这个函数调用后会返回一个列表，记录每一项的得分，并且再返回一个列表，是得分项
    # 这个函数基本上是要重写了
    # 这个函数主要是在每帧都会算一次，默认任务始终未完成，直到遇到完成的累积时间大于0，就算是任务完成
    def calcuAllScore(self, missonTime, expectTime, senceId, weight, allWeight):
        # 场景得分
        senceScore = {'场景': senceId}
        for key in self.scoreAccuTimeDict:
            time = self.scoreAccuTimeDict[key]

            # 问题在这里，他这个是用累计的时间/当前用的时间，所以会变小
            # 前0.1秒是无敌的，不会扣分
            if missonTime < 0.1:
                score = 0
            else:
                # 这里应该已100分为单位算初始成绩，然后在乘场景权重和自己的权重，这样才是正确的分数
                score = (time / missonTime) * 100
            keyIndex = key.replace('累积时间','') + '扣分'
            senceScore[keyIndex] = score

        # 到这里为止，是把所有扣分的地方都找到了，都在senceScore里面，这里面是个大字典，当前为止所有的分数都是百分制的扣分
        # 在这里进行解耦就行了
        # 找对应的需要的计算的指标，然后把他们进行权重赋值

        # 需要对闯红灯的次数判定，如果超过1则永远是1
        if self.scoreAccuNumDict["闯红灯累积次数"] > 1:
            self.scoreAccuNumDict["闯红灯累积次数"] = 1

        # 上面这里扣的分数是安装累计时间来扣分的
        # 下面写一个根据次数扣分的
        for key in self.scoreAccuNumDict:
            num = self.scoreAccuNumDict[key]
            # 每个扣分要扣多少分
            mini = Parameter.miniScore[key.replace('累积次数', '')]
            score = num * mini
            # 表示1此扣5分，如果这里扣分不一样，就要改一下，按照字典之类的，也很简单
            keyIndex = key.replace('累积次数', '') + '扣分按次'
            senceScore[keyIndex] = score


        # 维度之间的指标计算
        scoreWeight = allWeight.scoreWeight
        # 这里是拿到了所有的要算的指标
        # 后面就是要筛选出来那些是要的，哪些是不要的

        # 都可以要，self.safeDitalWeight = {}
        #         self.comfortableDitalWeight = {}
        #         self.efficiencyDitalWeight = {}
        # 需要的都在这里了

        # 计算任务是否完成，如果没有到达终点就是0，如果到达终点但是没有按路线走就是0.5，如果按照路线走到了终点就是1
        #改：只有按路线走才是1，其余都是0
        if self.scoreAccuTimeDict["是否完成任务累积时间"] > 0:
            if senceScore['未按规定路线行驶'+'扣分按次'] < 1:
                self.calMissionAccomplish = 1
            else:
                self.calMissionAccomplish = 0.5




        if missonTime <= expectTime*5:
            # 目前用的算法是比较简单的就是单纯的除一下，这是一个线性的函数，有没有不是线性的函数
            # 这里要专门写一个函数来非线性的计算效率得分，
            # 比如说，拒绝gap中存在x个15秒以上的就扣x*10分，存在y个5秒到15秒的就扣x*3分，gap全是5秒以下的就不扣分
            # 要注意的是，这里是实时指标，每帧都会在这里计算，所以如果没到冲突点这里就不会有拒绝的gap，这里就暂时不扣分，只要到冲突点，就会开始根据算法扣分了
            # 这里还要在传进来一个比例，来计算平均速度的分数，现在平均速度的得分就是设置一个期望 平均速度，实际速度与期望速度做比值，为分数
            efficiencyMiniScore = self.calcuGapMiniScore(expectTime, missonTime, allWeight.efficiencyDitalWeight)
            senceTotallScore_efficiency = 100 - efficiencyMiniScore
        else:
            senceTotallScore_efficiency = 0

        # 计算场景总分
        # if senceScore['碰撞'+'扣分'] > 0.01 or senceScore['未按规定路线行驶'+'扣分按次'] > 3:
        if senceScore['碰撞'+'扣分按次'] >= 1:
            senceTotallScore_safe_minus = 100
        else:
            # 这里可以把ttc的比值改成0.5，就是说由ttc风险扣分的感觉降低了比重，对于激进的车就会更有优势
            senceTotallScore_safe_minus = (senceScore['驶出行车道'+'扣分'] * allWeight.safeDitalWeight["outOfLane"]/100 +
                                           senceScore['TTC'+'扣分'] * allWeight.safeDitalWeight["TTC"]/100 +
                                           senceScore['驶入对向车道' + '扣分'] * allWeight.safeDitalWeight["inSubtendRoad"]/100 +
                                           senceScore['压实线' + '扣分'] * allWeight.safeDitalWeight["onLaneMarking"]/100 +
                                           senceScore['超速' + '扣分'] * allWeight.safeDitalWeight["overSpeed"]/100 +
                                           senceScore['横向间距' + '扣分'] * allWeight.safeDitalWeight["transverseDistance"]/100 +
                                           senceScore['禁行区行驶' + '扣分'] * allWeight.safeDitalWeight["inForbiddenArea"]/100 +
                                           senceScore['未按规定车道行驶' + '扣分'] * allWeight.safeDitalWeight["drivingInDesignatedLane"]/100 +
                                           senceScore['停车压停止线' + '扣分'] * allWeight.safeDitalWeight["stopAtStopLine"]/100 +
                                           senceScore['闯红灯' + '扣分按次'] * allWeight.safeDitalWeight["breakSignal"]/100)

        if senceTotallScore_safe_minus > 100:
            senceTotallScore_safe_minus = 100

        # senceTotallScore_comfortable = 100 - (
        #             senceScore['横向加速度' + '扣分按次'] * allWeight.comfortableDitalWeight["overLateralAcce"]/100 +
        #             senceScore['横向加加速度' + '扣分按次'] * allWeight.comfortableDitalWeight["overLateralJerk"]/100 +
        #             senceScore['纵向加速度' + '扣分按次'] * allWeight.comfortableDitalWeight["overAcce"]/100 +
        #             senceScore['纵向加加速度' + '扣分按次'] * allWeight.comfortableDitalWeight["overJerk"]/100 +
        #             senceScore['转向角变化' + '扣分'] * allWeight.comfortableDitalWeight["unstableSteeringAngle"]/100)

        senceTotallScore_comfortable = 100 - (
                senceScore['横向加速度' + '扣分'] * allWeight.comfortableDitalWeight["overLateralAcce"] / 100 +
                senceScore['横向加加速度' + '扣分'] * allWeight.comfortableDitalWeight["overLateralJerk"] / 100 +
                senceScore['纵向加速度' + '扣分'] * allWeight.comfortableDitalWeight["overAcce"] / 100 +
                senceScore['纵向加加速度' + '扣分'] * allWeight.comfortableDitalWeight["overJerk"] / 100 +
                senceScore['转向角变化' + '扣分'] * allWeight.comfortableDitalWeight["unstableSteeringAngle"] / 100)

        if senceTotallScore_comfortable < 0:
            senceTotallScore_comfortable = 0

        # 重点在这里增加变化，上面扣分和得分的标准都不用改，后面要加什么就再加
        # 这里要新增的就是一个判断，如果安全分数大于90，或者说是如果安全分呗扣到90以下
        senceTotallScore_safe = 100 - senceTotallScore_safe_minus
        #print(senceScore['未按规定路线行驶'+'扣分按次'])
        if senceScore['未按规定路线行驶'+'扣分按次'] > 0:###增加不按规定行驶全部为0
            senceTotallScore_safe=0
            senceTotallScore_comfortable=0
            senceTotallScore_efficiency=0
        # else:
        #     senceTotallScore_safe = 0
        #     senceTotallScore_comfortable = 0
        #     senceTotallScore_efficiency = 0


        #totalSenceScore = {'场景': senceId, '安全': senceTotallScore_safe * 0.5 * weight, '效率': senceTotallScore_efficiency * 0.3 * weight, '舒适': senceTotallScore_comfortable * 0.2 * weight}
        totalSenceScore = {'senceID': senceId,
                           'safe': senceTotallScore_safe * scoreWeight['safe'] * weight,
                           'efficiency': senceTotallScore_efficiency * scoreWeight['efficiency'] * weight,
                           'comfortable': senceTotallScore_comfortable * scoreWeight['comfortable'] * weight}

        # 这里要注意扣得全是百分制，不是百分制，
        # 这里全是计算出来的真实分数，需要乘之前的传过来的权重，才是最终的要展示出去的分数
        senceScoreEnglish = {'senceID': senceId,
                             'missionAccomplish': 0,
                             'calMissionAccomplish': self.calMissionAccomplish,
                             'crash': senceScore['碰撞扣分按次'],
                             'outOfLane': senceScore['驶出行车道扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["outOfLane"]/100,
                             'TTC': senceScore['TTC扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["TTC"]/100,
                             'PET': senceScore['PET扣分'] * scoreWeight['safe']* 0/100, # 没有pet目前
                             'inSubtendRoad': senceScore['驶入对向车道扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["inSubtendRoad"]/100,
                             'onLaneMarking': senceScore['压实线扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["onLaneMarking"]/100,
                             'overSpeed': senceScore['超速扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["overSpeed"]/100,
                             'breakSignal': senceScore['闯红灯扣分按次'] * scoreWeight['safe']* allWeight.safeDitalWeight["breakSignal"]/100,
                             'breakSignalNum': self.scoreAccuNumDict["闯红灯累积次数"],
                             "transverseDistance": senceScore['横向间距扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["transverseDistance"]/100,
                             "inForbiddenArea": senceScore['禁行区行驶扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["inForbiddenArea"]/100,
                             "drivingInDesignatedLane": senceScore['未按规定车道行驶扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["drivingInDesignatedLane"]/100,
                             "stopAtStopLine": senceScore['停车压停止线扣分'] * scoreWeight['safe']* allWeight.safeDitalWeight["stopAtStopLine"]/100,
                             'overLateralAcce': senceScore['横向加速度扣分'] * scoreWeight['comfortable'] * allWeight.comfortableDitalWeight["overLateralAcce"]/100,
                             'overLateralAcceNum': self.scoreAccuTimeDict["横向加速度累积时间"],
                             'overLateralJerk': senceScore['横向加加速度扣分'] * scoreWeight['comfortable'] * allWeight.comfortableDitalWeight["overLateralJerk"]/100,
                             'overLateralJerkNum': self.scoreAccuTimeDict["横向加加速度累积时间"],
                             'overAcce': senceScore['纵向加速度扣分'] * scoreWeight['comfortable'] * allWeight.comfortableDitalWeight["overAcce"]/100,
                             'overAcceNum': self.scoreAccuTimeDict["纵向加速度累积时间"],
                             'overJerk': senceScore['纵向加加速度扣分'] * scoreWeight['comfortable'] * allWeight.comfortableDitalWeight["overJerk"]/100,
                             'overJerkNum': self.scoreAccuTimeDict["纵向加加速度累积时间"],
                             # 'overLateralAcce': senceScore['横向加速度扣分按次'] * scoreWeight['comfortable'] *
                             #                    allWeight.comfortableDitalWeight["overLateralAcce"] / 100,
                             # 'overLateralAcceNum': self.scoreAccuNumDict["横向加速度累积次数"],
                             # 'overLateralJerk': senceScore['横向加加速度扣分按次'] * scoreWeight['comfortable'] *
                             #                    allWeight.comfortableDitalWeight["overLateralJerk"] / 100,
                             # 'overLateralJerkNum': self.scoreAccuNumDict["横向加加速度累积次数"],
                             # 'overAcce': senceScore['纵向加速度扣分按次'] * scoreWeight['comfortable'] *
                             #             allWeight.comfortableDitalWeight["overAcce"] / 100,
                             # 'overAcceNum': self.scoreAccuNumDict["纵向加速度累积次数"],
                             # 'overJerk': senceScore['纵向加加速度扣分按次'] * scoreWeight['comfortable'] *
                             #             allWeight.comfortableDitalWeight["overJerk"] / 100,
                             # 'overJerkNum': self.scoreAccuNumDict["纵向加加速度累积次数"],
                             'unstableSteeringAngle': senceScore['转向角变化扣分'] * scoreWeight['comfortable'] * allWeight.comfortableDitalWeight["unstableSteeringAngle"]/100,
                             'unstableSteeringAngleTime': self.scoreAccuTimeDict["转向角变化累积时间"],
                             "avgSpeed": self.scoreEfficiency["平均速度"] * scoreWeight['efficiency'],
                             "gapRefuse": self.scoreEfficiency["拒绝间隙"] * scoreWeight['efficiency'],
                             "timeUsed": self.scoreEfficiency["行程时间"] * scoreWeight['efficiency'], # 这些都是扣的分数

                             "eventTable":self.eventTable}
        return senceScoreEnglish, totalSenceScore



