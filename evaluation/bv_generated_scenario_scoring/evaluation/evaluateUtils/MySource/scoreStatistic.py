import math
import numpy as np
from .cal_param import cal_ego_param
from shapely.geometry import Point, Polygon

# 在这里额外加一个用于计算，横向间距，是否四轮在路，是否逆向行驶，是否压线行驶，是否未按规定车道行驶，是否压停止线
# 这些指标是需要opdrive路网数据的
# 生成处理好的opdrive路网数据都保存在了一个json文件里面，从这个里面直接拿数据，用于上面的指标计算
# 1表示当前帧超出阈值了，需要扣分，0表示在范围内按规定行驶

class selfStateScore(object):

    def __init__(self, veh, SVALL_cars_info, id, mapJsonInforDict, PETCarIdList, aimPathJsonInforDict, sceneID, signalState, indexWeight):
        self.Ego = veh
        self.SVALL_cars_info = SVALL_cars_info # 最新一帧的所有车的当前状态列表
        self.senceID = id
        self.senceEnd = sceneID # 为0的时候表示场景结束了，当前场景任务完成，如果不为0表示的是场景的id
        self.mapJsonInforDict = mapJsonInforDict
        self.aimPathJsonInforDict = aimPathJsonInforDict
        self.closeCarIdList = PETCarIdList
        # 判断哪些指标需要计算，哪些不用
        self.indexWeight = indexWeight

        # 以下指标均针对当前帧
        # 是否驶出了行车道——对应四路在路检测
        self.outOfLane = 0
        # 是否驶入对向车道
        self.inSubtendRoad = 0
        # 是否压实线
        self.onLaneMarking = 0
        # 是否超速
        self.overSpeed = 0
        # 是否闯红灯
        self.breakSignal = 0
        self.signalState = signalState
        # 加速度是否超出阈值
        self.overAcce = 0
        # 加加速度是否超出阈值
        self.overJerk = 0
        # 横向加速度是否超出阈值
        self.overLateralAcce = 0
        # 横向加加速度是否超出阈值
        self.overLateralJerk = 0
        # 转向角变化是否超出阈值
        self.unstableSteeringAngle = 0

        # 再加一些静态指标
        # 横向间距违规
        self.lateral_spacing_time = 0
        # 未按车道行驶
        self.driving_in_designated_lane_time = 0
        # 停车压停止线
        self.stop_at_the_stop_line_count = 0
        # 在禁行区行驶
        self.driving_in_restricted_area_flag = False
        # 是否按照规定路线行驶
        self.driving_in_designated_path = 0

        # 用于记录各种加速度
        self.acce = 0
        # 加加速度是否超出阈值
        self.jerk = 0
        # 横向加速度是否超出阈值
        self.lateralAcce = 0
        # 横向加加速度是否超出阈值
        self.lateralJerk = 0
        # 转向角变化是否超出阈值
        self.angle = 0

        # 设置转向角变化阈值
        self.angle_threshold = 0.5*180/math.pi

        # 是否已经完成任务，0表示没有完成，1表示已经完成了
        # 这里有两种判断，首先判断是否地图信息中是否有"goalPlaceInfo"and"goalPlaceInfo"是否有值，如果为否，就用传递的场景数据的0/1判断是否完成了比赛
        # 如果为是，则用每一帧的数据判断是否存在某一帧经过了这个位置，经过了就是1，没有经过就是0
        self.missionAccomplish = 0

    # 总体的计算函数，用于调用自己所有分数的计算函数，不返还，不传参，
    def calcuAllScore(self):
        calEgoParam = cal_ego_param(self.mapJsonInforDict, self.Ego.posList, self.SVALL_cars_info, self.closeCarIdList, self.angle_threshold, self.Ego.length,self.Ego.width)
        self.outOfLane = self.calcuOutOfLane(calEgoParam)
        self.inSubtendRoad = self.calcuInSubtendRoad(calEgoParam)
        self.onLaneMarking = self.calcuOnLaneMarking(calEgoParam)
        self.overSpeed = self.calcuOverSpeed()
        self.breakSignal = self.calBreakSignal()
        self.overAcce = self.calcuOverAcce()
        self.overJerk = self.calcuOverJerk()
        self.overLateralAcce = self.calcuOverLateralAcce()
        self.overLateralJerk = self.calcuOverLateralJerk()
        self.unstableSteeringAngle = self.calcuUnstableSteeringAngle()
        self.missionAccomplish = self.calMissionAccomplish(self.mapJsonInforDict, self.senceEnd)
        # 横向间距违规
        self.lateral_spacing_time = self.cal_lateral_spacing_time(calEgoParam)
        # 未按车道行驶
        self.driving_in_designated_lane_time = self.cal_driving_in_designated_lane_time(calEgoParam)
        # 停车压停止线
        self.stop_at_the_stop_line_count = self.cal_stop_at_the_stop_line_count(calEgoParam)
        # 在禁行区行驶
        self.driving_in_restricted_area_flag = self.cal_driving_in_restricted_area_flag(calEgoParam)
        # 是否按照规定路线行驶
        self.driving_in_designated_path = self.cal_driving_in_designated_path()


    # 基础函数计算两点之间的距离
    def disPoints(self, pos1, pos2):
        x1 = pos1[0]
        y1 = pos1[1]
        x2 = pos2[0]
        y2 = pos2[1]
        distance = pow((pow(x1 - x2, 2) + pow(y1 - y2, 2)), 0.5)
        return distance

    def calMissionAccomplish(self, mapJsonInforDict, senceEnd):
        # 首先判断地图信息中是否有"goalPlaceInfo"and"goalPlaceInfo"是否有值
        if "goalPlaceInfo" in mapJsonInforDict:
            if mapJsonInforDict["goalPlaceInfo"]:
                goalPlace = mapJsonInforDict["goalPlaceInfo"]
                # 这里之前已经处理过了，所以就是一个完整的数据了
                if self.is_point_inside_rect(goalPlace):
                    # 表示在这个区域内了
                    return 1

                else:
                    return 0
        # 没有上面的这个信息，或者没有值，证明这个是没有标准结束点，用场景信息来判断,0表示场景结束，return1
        if not senceEnd:
            return 1
        else:
            return 0

    def calInside(self, points, vertices):
        # vertices是所有的交叉口的停止线，是个字典，所以要循环判断是不是在内
        for key in vertices:
            # 生成区域
            polygon = Polygon(vertices[key])
            # 待判断的点
            point_to_check = Point(points[0], points[1])

            # 使用 Shapely 库的 within 方法判断点是否在多边形内
            is_inside = point_to_check.within(polygon)
            if is_inside:
                return is_inside
            else:
                pass
        return False

    # 根据区域，计算车辆的中心点是否在这个范围内，并且有速度，在就算是闯红灯了，这里是一帧的情况
    # 在self.mapJsonInforDict里面有一个，"all_stop_line"，这里面就是停止线信息
    def calBreakSignal(self):
        # 如果是绿灯直接表示没事，红灯就是有可能有事
        # print("当前信号灯状态", self.signalState, "是在停止线范围内",self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"]))
        if self.signalState == "red":
            # 首先把停止线的区域围成一个区域
            currentState = self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"])
            # 在计算一下上一帧的状态，如果上一帧也是在区域内，则证明已经一直在区域范围了，就不用了
            lastPos = [self.Ego.posList[-2]['x'], self.Ego.posList[-2]['y']]
            lastState = self.calInside(lastPos, self.mapJsonInforDict["all_stop_line_points"])
            # print("当前是在停止线范围内",currentState, "上一帧是在停止线范围内",lastState)
            # 所以只有在，上一帧不在范围，而这一帧在范围，并且是红灯的情况，才需要计算存在闯红灯的行为
            if not lastState and currentState:
                # print("闯红灯了")
                return 1
            else:
                return 0
        else:
            return 0

    # 判断点是否在一个矩形内部，用于计算是否在场景开始面域和场景结束面域
    def is_point_inside_rect(self, rect) -> bool:
        """
        Check if a point is inside a rectangle defined by two points (top-left and bottom-right).
        Returns:
        - bool: True if the point is inside the rectangle, False otherwise.
        """
        if self.Ego.currentPos:
            x, y = self.Ego.currentPos[0], self.Ego.currentPos[1]
            x1, y1 = rect[0]
            x2, y2 = rect[1]
            # print(x1 , x, x2)
            # print(y1 , y , y2)
            if x1 <= x <= x2 and y1 <= y <= y2:
                # print("在终点范围内")
                return True
            else:
                return False

    # 是否驶出了行车道——四轮在路检测
    def calcuOutOfLane(self, calEgoParam):
        if not self.indexWeight["10006"]:
            return 0
        # 如果在交叉口范围内，则之间不对驶出车道和对象车道和压线进行检测
        if self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"]):
            return 0
        try:
            d = calEgoParam.cal_four_wheels_on_road_time()
        except:
            print("是否驶出车道评价错误")
            d = 0
        if d:
            return 1
        else:
            return 0

    # 是否驶入对向车道
    def calcuInSubtendRoad(self, calEgoParam):
        if not self.indexWeight["10003"]:
            return 0
        # 如果在交叉口范围内，则之间不对驶出车道和对象车道和压线进行检测
        if self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"]):
            return 0
        try:
            a = calEgoParam.cal_car_reverse_count()
        except:
            print("是否驶入对向车道评价错误")
            a = 0
        if a:
            return 1
        else:
            return 0

    # 是否压实线
    def calcuOnLaneMarking(self, calEgoParam):
        if not self.indexWeight["10004"]:
            return 0
        # 如果在交叉口范围内，则之间不对驶出车道和对象车道和压线进行检测
        if self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"]):
            return 0
        try:
            d = calEgoParam.cal_compacted_line_travel_count()
        except:
            print("是否压实线评价错误")
            d = 0
        if d:
            return 1
        else:
            return 0

    # 是否超速
    def calcuOverSpeed(self):
        if not self.indexWeight["10005"]:
            return 0
        if self.Ego.speed > 40:
            return 1
        else:
            return 0

    # 横向间距违规
    def cal_lateral_spacing_time(self, calEgoParam):
        if not self.indexWeight["10007"]:
            return 0
        try:
            a = calEgoParam.cal_lateral_spacing_error()
        except:
            print("横向间距违规评价错误")
            a = 0
        if a:
            return 1
        else:
            return 0
    # 未按规定车道行驶
    def cal_driving_in_designated_lane_time(self, calEgoParam):
        if not self.indexWeight["10010"]:
            return 0
        try:
            a = calEgoParam.cal_driving_in_designated_lane_time()
        except:
            print("未按规定车道行驶评价错误")
            a = 0
        if a:
            return 1
        else:
            return 0

    # 停车压停止线
    def cal_stop_at_the_stop_line_count(self, calEgoParam):
        if not self.indexWeight["10011"]:
            return 0
        try:
            a = calEgoParam.cal_stop_at_the_stop_line_count()
        except:
            print("停车压停止线评价错误")
            a = 0
        if a:
            return 1
        else:
            return 0

    # 在禁行区行驶
    def cal_driving_in_restricted_area_flag(self, calEgoParam):
        if not self.indexWeight["10008"]:
            return 0
        # 如果在交叉口范围内，则之间不对禁行区行驶评价
        if self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"]):
            return 0
        try:
            a = calEgoParam.cal_driving_in_restricted_area_flag()
        except:
            print("在禁行区行驶评价错误")
            a = 0
        if a:
            return 1
        else:
            return 0




    # 加速度是否超出阈值
    def calcuOverAcce(self):
        self.acce = self.Ego.acce
        if self.Ego.acce > 3 or self.Ego.acce < -3:
            return 1
        else:
            return 0

    # 加加速度是否超出阈值
    def calcuOverJerk(self):
        # 加速度可以直接读，加速度和上一针的加速度可以计算加加速度，上一针的加速度在轨迹里有保存，所以只需要拿轨迹中最新保存的两个就行了
        # current_pos = {'id': self.id, 'x': Xpos, 'y': Ypos, 'simutime': t, 'speed': self.speed, 'acce': self.acce, 'type': self.vehicleType, 'angle': self.angle}
        currPos = self.Ego.posList[-1]
        lastPos = self.Ego.posList[-2]
        currAcce = currPos['acce']
        lastAcce = lastPos['acce']
        jerk = (currAcce - lastAcce) / self.Ego.accuracy
        self.jerk = jerk
        if jerk > 6 or jerk < -6:
            return 1
        else:
            return 0

    # 通用横向加速度计算，by转向半径
    def calcuLateralAcce(self, currPos, lastPos):
        # 这个就比较恶心了，要航向角来算
        pos1 = [currPos['x'], currPos['y']]
        pos2 = [lastPos['x'], lastPos['y']]
        l = self.disPoints(pos1, pos2)
        beta = (abs(currPos['angle'] - lastPos['angle']) + 360) % 360
        if math.sin(beta) == 0:
            return 0
        r = l / math.sin(beta)
        if r == 0:
            return 0
        # 这里车辆速度都是km/h，都要换成m/s，那就除以3.6
        lateralAcce = ((currPos['speed']/3.6) ** 2) / r
        return lateralAcce

    # 通用横向加速度计算，by角速度
    def calcuLateralAcceByAngle(self, currPos, lastPos):
        # 航向角来算，航线角除时间，是角速度，角速度乘速度等于横向加速度

        beta = (abs(currPos['angle'] - lastPos['angle']) + 360) % 360
        # 转变成弧度制
        beta2 = beta * np.pi / 180
        # 计算角速度
        w = beta2 / self.Ego.accuracy

        # 这里车辆速度都是km/h，都要换成m/s，那就除以3.6
        lateralAcce = (currPos['speed'] / 3.6) * w

        if self.Ego.accuracy == 0.02:
            lateralAcce = 0

        return lateralAcce

    # 横向加速度是否超出阈值
    def calcuOverLateralAcce(self):
        # 试试用航向角来算
        currPos = self.Ego.posList[-1]
        lastPos = self.Ego.posList[-2]
        lateralAcce = self.calcuLateralAcceByAngle(currPos, lastPos)
        self.lateralAcce = lateralAcce
        if lateralAcce > 0.5 or lateralAcce < -0.5:
            return 1
        else:
            return 0

    # 横向加加速度是否超出阈值
    def calcuOverLateralJerk(self):
        currPos = self.Ego.posList[-1]
        lastPos = self.Ego.posList[-2]
        currlateralAcce = self.calcuLateralAcceByAngle(currPos, lastPos)

        lastlastPos = self.Ego.posList[-3]
        lastlateralAcce = self.calcuLateralAcceByAngle(lastPos, lastlastPos)

        lateraljerk = (currlateralAcce - lastlateralAcce) / self.Ego.accuracy
        self.lateralJerk = lateraljerk
        if lateraljerk > 1 or lateraljerk < -1:
            return 1
        else:
            return 0

    # 转向角变化是否超出阈值
    # def calcuUnstableSteeringAngle(self):
    #     Pos0 = self.Ego.posList[-1]
    #     Pos1 = self.Ego.posList[-2]
    #     Pos2 = self.Ego.posList[-3]
    #     angChange0 = Pos0['angle'] - Pos1['angle']
    #     angChange1 = Pos1['angle'] - Pos2['angle']
    #
    #     stableValue = angChange0 - angChange1
    #
    #     self.angle = stableValue
    #     if stableValue > self.angle_threshold or stableValue < -self.angle_threshold:
    #         return 1
    #     else:
    #         return 0

    # 转向角变化是否超出阈值
    def calcuUnstableSteeringAngle(self):
        Pos0 = self.Ego.posList[-1]
        Pos1 = self.Ego.posList[-2]
        angChange0 = Pos0['angle'] - Pos1['angle']
        stableValue = angChange0/self.Ego.accuracy
        self.angle = stableValue
        if stableValue > self.angle_threshold or stableValue < -self.angle_threshold:
            return 1
        else:
            return 0


    # 要写一个函数把ego车的各种加速度都算出来写上
    def acceRead(self):
        acceAll = {'verticalA': self.acce,
                   'verticalAPlus': self.jerk,
                   'horizontalA': self.lateralAcce,
                   'horizontalAPlus': self.lateralJerk,
                   'turnA': self.angle}
        return acceAll

    # 计算是不是按照规定的路线进行行驶
    def cal_driving_in_designated_path(self):
        # 如果没有这个信息，直接这个永远是0，就是按规定完成任务，如果有这个信息则进行下一步判断
        if self.aimPathJsonInforDict:
            if "waypoints" in self.aimPathJsonInforDict:
                for pos in self.aimPathJsonInforDict["waypoints"].values():
                    pos2 = [self.Ego.Xpos, self.Ego.Ypos]
                    d = self.disPoints(pos, pos2)
                    # 需要多判断一下是否在最左侧车道，有点麻烦先不做了
                    if d > 5:
                        pass
                    else:
                        return 0
                # 这里返回1是因为，如果全部values都过了一遍，全都大于5，那就只能证明已经偏离跑道了
                #print(pos2)
                return 1
        else:
            return 0




# 这个好像也不需要，直接在自动驾驶车里记录就行了
class interactionScore(object):

    def __init__(self):
        # 碰撞指标
        self.crash = 0
        # TTC指标
        self.overTTC = 0
        # PET指标
        self.overPET = 0

    # 在写一个函数，两点之间的距离公式
    def calculateDistance(self, x1, y1, x2, y2):
        distance = pow((pow(x1 - x2, 2) + pow(y1 - y2, 2)), 0.5)
        return distance

    def calcuCrash(self, crashTime, accuracy):
        if crashTime > 2 * accuracy:
            self.crash = 1

    def calcuCrashOnsite(self, isCrash):
        if isCrash == 3:
            self.crash = 1
        else:
            self.crash = 0

    def calcuOverTTC(self, TTC):
        if TTC:
            if TTC < 1:
                self.overTTC = 1
            else:
                self.overTTC = 0

    def calcuOverPET(self, PET):
        if PET < 5:
            self.overPET = 1
        else:
            self.overPET = 0

    def calcuGap(self, lastConflictTime, simutime, pet, AVposList, allSVlist):
        # 现在的算法是有点问题的，重点在于如果被一个急的车流挡住了一个缓慢的车流
        # 解决办法就是，记录下上一个冲突点的时间，在计算这个冲突点的数据时把小于冲突点时间的数据都删除
        if pet != 0:
            timeList = []
            # carIdList = []
            for iSVposList in allSVlist:
                timePass = self.findPassPosTime(AVposList[len(AVposList) - 1], iSVposList)
                if timePass != 0 and timePass > lastConflictTime:
                    # 好像不需要知道id，只需要间隔就行了
                    # carIdList.append(iSVposList[0]['id'])
                    timeList.append(timePass)
            # 拿到所有车辆在这个列表的时刻，且每辆车的时刻都是唯一的，如果经过了那个地方就是有时间，没有经过就是0
            # 所有第一步是列表排序，排序的时候要带着caridlist一起排序
            timeList.append(simutime/1000)
            timeList.sort()
            # 然后用这个列表的不同元素递减，第一个值减第二个值
            gapList = []
            if len(timeList)-1>0:
                for i in range(len(timeList)-1):
                    gap = timeList[i] - timeList[i+1]
                    if gap != 0:
                        gapList.append(abs(gap))

            return gapList
        # 这里如果进不去会返回none，none会在后面被排除掉

    def findPassPosTime(self, carAV: dict, carSV: list):
        carAVpos = [carAV['x'], carAV['y']]

        disMin = 999999
        timeMin = 0
        passtime = 0
        # 不断的判断carb中的轨迹点与当前车辆的点之间的距离，找到最近的点，把最近的点和对应的时刻记录下来，这个时刻一定是小的，而且这个时刻是唯一的
        for posI in carSV:
            # print(posI)
            carBpos = [posI['x'], posI['y']]
            dis = self.calculateDistance(carAVpos[0], carAVpos[1], carBpos[0], carBpos[1])
            if dis < disMin:
                disMin = dis
                timeMin = posI['simutime']
                # 这里会返回唯一的一个值，最接近的一个点，所以下面只要不是另一个车道的都没有问题

        if disMin < 1:
            passtime = timeMin
        return passtime

    # 用来计算所有的gap，这个gap是个大列表，里面是所有冲突点的小列表
    def calcuOverGap(self, gapList):
        pass

# 在这个类里面做一些全部的计算，主要的就是计算评分，感觉这里也没有必要
class totalScore(object):
    def __init__(self, selfStateScore, interactionScore):

        # 之前两项在当前帧是否得分
        self.selfStateScore = selfStateScore
        self.interactionScore = interactionScore

        # 任务是否完成









