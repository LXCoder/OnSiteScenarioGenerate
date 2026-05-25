import math
import numpy as np
from evaluateUtils.MySource.cal_param import cal_ego_param
from shapely.geometry import Point, Polygon
from evaluateUtils.standard_parameter import Parameter

# 在这里额外加一个用于计算，横向间距，是否四轮在路，是否逆向行驶，是否压线行驶，是否未按规定车道行驶，是否压停止线
# 这些指标是需要opdrive路网数据的
# 生成处理好的opdrive路网数据都保存在了一个json文件里面，从这个里面直接拿数据，用于上面的指标计算
# 1表示当前帧超出阈值了，需要扣分，0表示在范围内按规定行驶

class selfStateScore(object):

    def __init__(self, veh, SVALL_cars_info, id, mapJsonInforDict, PETCarIdList, aimPathJsonInforDict, sceneID, signalState, indexWeight):
        self.Ego = veh
        self.SVALL_cars_info = SVALL_cars_info   # 最新一帧的所有车的当前状态列表
        self.senceID = id
        self.senceEnd = sceneID   # 为0的时候表示场景结束了，当前场景任务完成，如果不为0表示的是场景的id
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
        # 是否压虚线
        self.onDottedLaneMarking = 0
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
        # 新增一个倒车的指标
        self.reverseCar = 0
        # 新增一个停车标志的指标
        self.followStopSignal = 0


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
        # 是否发生了未按照目标点运行的情况，1表示未经过目标点，0表示经过目标点
        self.notPassAimPoint = 0


    # 总体的计算函数，用于调用自己所有分数的计算函数，不返还，不传参，
    def calcuAllScore(self):
        # todo 提升效率可以在这里self.mapJsonInforDict对这个东西进行处理，这个包含整个路网的所有关键点，后面会用到5次左右
        # 之前是每一次都把相邻的关键点都组成正方形算一下，太影响效率了
        # 现在可以直接在最开始，根据当前点位对这个数据进行处理，如果某个点和当前点位差了超过30米（3.5*8）就肯定不是需要的点位了，可以直接删除
        # 所以在这里调用一个函数，传入当前点位，和地图信息，
        # 在函数中会对all_road_dict和all_road_mark进行操作
        # 遍历所有的点，把距离当前点30米以内的点都给筛选出来，然后放在一个新的字典里面并返回
        # 输出一个新的地图信息字典，这个地图信息包含的原来的其他字段，以及新的all_road_dict和all_road_mark
        # 最后把这个新的字典传到cal_ego_param里面，就可以完成优化了
        # mapJsonInforDictProcessed = self.mapDataProcessForCalSpeed(self.mapJsonInforDict)
        mapJsonInforDictProcessed = self.mapJsonInforDict
        calEgoParam = cal_ego_param(mapJsonInforDictProcessed, self.Ego.posList, self.SVALL_cars_info, self.closeCarIdList, self.angle_threshold, self.Ego.length,self.Ego.width)
        self.outOfLane = self.calcuOutOfLane(calEgoParam)
        self.inSubtendRoad = self.calcuInSubtendRoad(calEgoParam)
        self.onLaneMarking = self.calcuOnLaneMarking(calEgoParam)
        self.onDottedLaneMarking = self.calcuOnDottedLaneMarking(calEgoParam)
        self.overSpeed = self.calcuOverSpeed()
        self.breakSignal = self.calBreakSignal()
        self.overAcce = self.calcuOverAcce()
        self.overJerk = self.calcuOverJerk()
        self.overLateralAcce = self.calcuOverLateralAcce()
        self.overLateralJerk = self.calcuOverLateralJerk()
        self.unstableSteeringAngle = self.calcuUnstableSteeringAngle()
        self.missionAccomplish = self.calMissionAccomplish(self.mapJsonInforDict, self.senceEnd)
        # 倒车指标计算
        self.reverseCar = self.calReverse()
        # 停车标志指标计算
        self.followStopSignal = self.calFollowStopSignal()
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
        # 是否经过了目标点
        self.notPassAimPoint = self.calInAimPointRange(self.senceEnd)

    def mapDataProcessForCalSpeed(self, mapJsonInforDict):
        # 判断是否算是在这个路的这些点附近的阈值
        around_threshold = 30
        currPos = [self.Ego.posList[-1]['x'], self.Ego.posList[-1]['y']]
        all_road_dict_original = mapJsonInforDict['all_road_dict']
        all_road_mark_original = mapJsonInforDict['all_road_mark']

        # 首先处理路段的信息，做一个新的all_road_dict_processed
        # 也是一个列表，每个元素是一个字典，字典表示一个路段，路段中有一个中心点坐标，左右坐标等等，都是列表，每个元素是一个经纬度+高程坐标
        # 如果这个路段里面没有被选中的中心点，就不把这个路段加进来，如果有选中的，就加进来
        all_road_dict_processed = []
        onRoadID = []
        for roadIdict in all_road_dict_original:
            centerList = roadIdict['center_vertices']
            # 准备一下新的路段的基本信息和要添加的一些点的列表
            newRoadIdict = {
                'road_id': roadIdict['road_id'],
                'lane_id': roadIdict['lane_id'],
                'type': roadIdict['type'],
                'center_vertices': [],
                'left_vertices': [],
                'right_vertices': [],
                'lane_angle': []
            }
            # 这个标记作用是，如果这个路段上有一个点和目标点比较近，那么就把这个路段标记为1，并且要把这个路段添加到输出字典里面
            choosedRoadMark = 0
            for i in range(len(centerList)):
                centerI = centerList[i]
                if self.disPoints(currPos, centerI) < around_threshold:
                    newRoadIdict['center_vertices'].append(roadIdict['center_vertices'][i])
                    newRoadIdict['left_vertices'].append(roadIdict['left_vertices'][i])
                    newRoadIdict['right_vertices'].append(roadIdict['right_vertices'][i])
                    newRoadIdict['lane_angle'].append(roadIdict['lane_angle'][i])
                    choosedRoadMark = 1
            if choosedRoadMark:
                all_road_dict_processed.append(newRoadIdict)
                onRoadID.append(roadIdict['road_id'])

        # 路段的处理工作结束了，开始对路段标记进行处理，路段标记需要依赖上面记录的onRoadID
        # all_road_mark也是一个列表，每个元素是一个字典，字典中包含roadid和一个信息列表，只需要判断roadid是否在onRoadID列表中
        all_road_mark_processed = []
        for markIdict in all_road_mark_original:
            if markIdict['road_id'] in onRoadID:
                all_road_mark_processed.append(markIdict)

        # 这样就处理完成了，把其他的也加进来就完成了
        mapJsonInforDictProcessed = {
            'all_road_dict': all_road_dict_processed,
            'all_stop_line': mapJsonInforDict['all_stop_line'],
            'all_lane_access': mapJsonInforDict['all_lane_access'],
            'all_road_mark': all_road_mark_processed,
            'all_junction_info': mapJsonInforDict['all_junction_info'],
            'header_info': mapJsonInforDict['header_info']
        }

        return mapJsonInforDictProcessed


    # 判断两点之间形成的线段是否与某个圆相交或者相切，相交返回true，不相交返回false
    def is_segment_intersect_circle(self, pos1, pos2, aimPos, r):
        # 将点转化为 NumPy 数组
        a = np.array(pos1)
        b = np.array(pos2)
        c = np.array(aimPos)

        # 计算线段的方向向量
        ab = b - a
        ac = c - a
        ab_length_squared = np.dot(ab, ab)

        # 计算 t 的值，表示点到线段的投影位置
        t = np.dot(ac, ab) / ab_length_squared

        # 限制 t 的范围在 [0, 1] 之间
        if t < 0:
            t = 0
        elif t > 1:
            t = 1

        # 计算最近点 p
        p = a + t * ab

        # 计算 p 到圆心 c 的距离
        distance_squared = np.dot(p - c, p - c)

        # 判断距离是否小于或等于半径的平方
        return distance_squared <= r ** 2

    def haveInCircleRange(self, trajectory,circle):
        # for i in range(1, len(trajectory)):
        #     # 获取前一个点和当前点的坐标以及时间
        #     x1, y1, t1 = trajectory[i - 1]
        #     x2, y2, t2 = trajectory[i]
        # if circle[0] < -1100 and circle[0] > -1101:
        #     print(166, trajectory)

        for index, i in enumerate(trajectory[:-1]):
            x1, y1, t1 = i
            x2, y2, t2 = trajectory[index + 1]

            pos1 = (x1, y1)
            pos2 = (x2, y2)
            aimPos = (circle[0], circle[1])
            r = Parameter.keyPointsRangeMin

            if self.is_segment_intersect_circle(pos1, pos2, aimPos, r):
                # print('成功通过', pos1, pos2, aimPos, r)
                return 1

        return 0


    # 计算是否经过了开始位置（车道级）的指标，如果经过了返回0，表示不用扣分，没有经过就返回1表示扣分
    # 首先需要计算是否到达开始的标志范围区域（大范围）内，如果抵达了就要判断一下是否已经有了这个抵达记录时刻
    # 如果没有抵达时刻，就记录抵达的时刻，用轨迹中的simutime来记录时间，第一次抵达后面可以不用算，直接return 0
    # 如果有抵达时刻，判断是否驶出了这个区域，没有驶出区域之前直接return 0
    # 如果既有抵达时刻，又驶出了区域，那么进行计算，看看是否经过了开始点（小范围），传入函数的参数是一串轨迹，这段轨迹是从抵达时刻到最近的时刻开始
    # 是否经过了开始点（小范围）的算法逻辑是，判断一段轨迹中，两帧两帧进行判断
    def calInAimPointRange(self, senceEnd):
        return 0
        # 判断是否进入了停车标志控制范围
        currPos = [self.Ego.posList[-1]['x'], self.Ego.posList[-1]['y']]

        if Parameter.keyPoints:
            startPointList = Parameter.keyPoints[:1]
        else:
            startPointList = []

        for postionDict in startPointList:
        # for postionDict in Parameter.keyPoints:
            # 计算点 a 到圆心的距离
            circle = [float(postionDict['x']), float(postionDict['y'])]
            distance = self.disPoints(currPos, circle)
            # 如果距离小于等于半径，说明点在圆内或圆上
            # 这里还需要再加一个判断，场景没有结束，如果场景结束了，那么就立马进入else里面，判断有没有进到getStartSignalTime
            if distance <= Parameter.keyPointsRangeMax and senceEnd:
                # 等于9999证明是第一次进入，记录一下进来的时间就行
                if self.Ego.getStartSignalTime == 9999:
                    self.Ego.getStartSignalTime = self.Ego.posList[-1]['simutime']
                return 0
            else:
                # 不在范围内，但是有初始进入的时刻，那么就证明已经开过去了，就可以判断是否停过车了
                if self.Ego.getStartSignalTime != 9999:
                    # 收集轨迹信息从getStopSignalTime这里开始
                    startSignalRangeTrajectory = []
                    # print('驶出停车标志范围的轨迹', self.Ego.posList)
                    # print('驶入停车标志范围的时间', self.Ego.getStopSignalTime)
                    for posI in self.Ego.posList:
                        # self.Ego.getStartSignalTime - 1是为了保证轨迹能够完全拿进来，多拿几个轨迹
                        if posI['simutime'] >= self.Ego.getStartSignalTime - 1:
                            pos = (posI['x'], posI['y'], posI['simutime'])
                            startSignalRangeTrajectory.append(pos)
                    # 用完判断时间之后，就需要立刻把时间复位
                    self.Ego.getStartSignalTime = 9999
                    # 保证至少有10帧轨迹在范围内，如果10帧轨迹都没有，就可以说明经过这个范围太短，来不及停车
                    if len(startSignalRangeTrajectory) > 2:
                        if not self.haveInCircleRange(startSignalRangeTrajectory, circle):
                            return 1
        return 0
        #                 # print(211, circle)
        #                 if self.haveInCircleRange(startSignalRangeTrajectory, circle):
        #                     return 0
        #                 else:
        #                     # print('没有成功通过关键点', circle)
        #                     return 1
        #             else:
        #                 return 0
        #         else:
        #             return 0
        # return 0

    # 判断车辆是否在需要停车的区域内，目前使用的方式是，用一个坐标点和半径覆盖范围判断，后面如果是需要精确到停止线的某个位置停车，这里需要改一下
    def is_within_any_circle(self, currPos, stopSignalPositionList):
        for circle in stopSignalPositionList:
            # 计算点 a 到圆心的距离
            distance = self.disPoints(currPos, circle)
            # 如果距离小于等于半径，说明点在圆内或圆上
            if distance <= Parameter.stopSignalRange:
                return 1
            # 如果点不在任何圆的范围内
        return 0

    # 计算停车标志前停车的指标
    # 首先需要计算是否到达停车标志范围区域内，如果抵达了就要判断一下是否已经有了这个抵达记录时刻
    # 如果没有抵达时刻，就记录抵达的时刻，用轨迹中的simutime来记录时间，第一次抵达后面可以不用算，直接return 0
    # 如果有抵达时刻，判断是否驶出了这个区域，没有驶出区域之前直接return 0
    # 如果既有抵达时刻，又驶出了区域，那么进行计算，看看是否停过车，传入函数的参数是一串轨迹，这段轨迹是从抵达时刻到最近的时刻开始
    # 停过车的算法是传入一串轨迹，检测这一串轨迹中是否存在几个时刻位移十分接近
    def calFollowStopSignal(self):
        # 判断是否进入了停车标志控制范围
        currPos = [self.Ego.posList[-1]['x'], self.Ego.posList[-1]['y']]

        if self.is_within_any_circle(currPos, Parameter.stopSignalPositionList):
            # 等于9999证明是第一次进入，记录一下进来的时间就行
            if self.Ego.getStopSignalTime == 9999:
                self.Ego.getStopSignalTime = self.Ego.posList[-1]['simutime']
            return 0
        else:
            # 不在范围内，但是有初始进入的时刻，那么就证明已经开过去了，就可以判断是否停过车了
            if self.Ego.getStopSignalTime != 9999:
                # 收集轨迹信息从getStopSignalTime这里开始
                stopSignalRangeTrajectory = []
                # print('驶出停车标志范围的轨迹', self.Ego.posList)
                # print('驶入停车标志范围的时间', self.Ego.getStopSignalTime)
                for posI in self.Ego.posList:
                    if posI['simutime'] > self.Ego.getStopSignalTime:
                        pos = (posI['x'], posI['y'], posI['simutime'])
                        stopSignalRangeTrajectory.append(pos)
                # 用完判断时间之后，就需要立刻把时间复位
                self.Ego.getStopSignalTime = 9999
                # 保证至少有10帧轨迹在范围内，如果10帧轨迹都没有，就可以说明经过这个范围太短，来不及停车
                if len(stopSignalRangeTrajectory) > 5:
                    if self.haveStopedCar(stopSignalRangeTrajectory, Parameter.stopCarThresholdDistance, Parameter.stopCarThresholdTime):
                        return 0
                    else:
                        return 1
                else:
                    return 0
            else:
                return 0


    # 判断轨迹中是否停过车，停过就返回1，没有就返回0
    def haveStopedCar(self, trajectory, threshold_distance, threshold_time):
        """
            判断车辆是否在轨迹中停过车

            参数:
            - trajectory: 车辆的轨迹，列表形式，包含多个 (x, y, time) 点
            - threshold_distance: 定义停车状态的距离阈值（如：0.01 表示几乎没有移动）
            - threshold_time: 停车的时间阈值（单位为秒，表示超过多少秒没有明显移动视为停车）

            返回:
            - True: 车辆曾经停车
            - False: 车辆未停车
        """
        stop_start_time = None

        for i in range(1, len(trajectory)):
            # 获取前一个点和当前点的坐标以及时间
            x1, y1, t1 = trajectory[i - 1]
            x2, y2, t2 = trajectory[i]

            # 计算两点之间的距离
            distance = self.disPoints([x1, y1], [x2, y2])

            # 判断是否几乎没有移动
            if distance < threshold_distance:
                if stop_start_time is None:
                    stop_start_time = t1  # 记录开始停车的时间
                elif t2 - stop_start_time >= threshold_time:
                    # print(f"Vehicle stopped from time {stop_start_time} to {t2}.")
                    return True
            else:
                stop_start_time = None  # 如果有移动，重置停车时间

        return False

    # 计算倒车指标
    # 倒车指标的计算可以用两帧轨迹连成的直线的角度，和车辆传进来的航向角进行对比
    # 之前约定好的是，车辆传进来的航向角是，以y轴为0，顺时针旋转，角度制
    # 这里两帧计算的角度也要转换成这个值
    def calReverse(self):
        # 不计算倒车指标
        return 0
        # 通过多个连续时刻的角度变化来平滑结果通过多个连续时刻的角度变化来平滑结果,num为选择的平滑点数量
        num = 6 # 表示5段平滑
        aimNum = 4 # 最多为num-1，表示5段里面有4段要倒车才行
        # 存储轨迹和航向角
        trajectory = []
        headings = []
        if len(self.Ego.posList) > num:
            new_list = self.Ego.posList[-num:][::-1]
            for posI in new_list:
                currPos = (posI['x'], posI['y'])
                trajectory.append(currPos)
                headings.append(posI['angle'])
            # 判断是否正在倒车，这四个点位中有num个是倒车状态才是倒车
            if self.is_reversing(trajectory, headings, aimNum):
                return 1
        return 0

    # 计算从点 (x1, y1) 到点 (x2, y2) 的位移方向角
    def calculate_angle(self, x1, y1, x2, y2):
        # 基于y轴正方向为0度，顺时针增加
        angle = math.degrees(math.atan2(x2 - x1, y2 - y1))  # 基于 y 轴计算方向角
        return angle % 360  # 将角度标准化到 [0, 360) 范围

    # 计算是否倒车
    def is_reversing(self, trajectory, headings, aimNum):
        reversingTime = 0
        for i in range(1, len(trajectory)):
            # 获取前一个点和当前点的坐标
            x1, y1 = trajectory[i]
            x2, y2 = trajectory[i - 1]

            # 计算位移方向
            movement_angle = self.calculate_angle(x1, y1, x2, y2)

            # 获取当前点的航向角
            heading_angle = headings[i]

            # 计算航向角与位移方向的角度差
            angle_diff = abs(movement_angle - heading_angle) % 360
            # print("倒车数据", movement_angle, heading_angle, x1, y1 ,x2, y2 )
            # 将角度差限制在0到180度之间
            if angle_diff > 180:
                angle_diff = 360 - angle_diff

            # 判断是否在倒车
            if angle_diff > 150:  # 如果角度差超过90度，车辆可能在倒车
                reversingTime += 1

        if reversingTime > aimNum:
            return True
        else:
            return False

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
                # if self.is_point_inside_rect(goalPlace):
                #     # 表示在这个区域内了
                #     return 1
                # print(self.is_point_inside_rect(goalPlace), self.line_intersects_rect(goalPlace))
                if self.line_intersects_rect(goalPlace):
                    # print(f"仿真时间：{self.Ego.curTime}，上一帧位置：{self.Ego.previousPos}，当前位置：{self.Ego.currentPos}, 是否经过：True")
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

    def line_intersects_rect(self, rect):
        '''
        两帧轨迹的连线是否穿过目标矩形区域
        :param rect:
        :return:
        '''
        if not self.Ego.currentPos:
            return False
        # 解构坐标
        x0, y0 = self.Ego.previousPos[0], self.Ego.previousPos[1]
        x1, y1 = self.Ego.currentPos[0], self.Ego.currentPos[1]
        rx1, ry1 = rect[0]
        rx2, ry2 = rect[1]

        # 确定矩形的边界
        x_min = min(rx1, rx2)
        x_max = max(rx1, rx2)
        y_min = min(ry1, ry2)
        y_max = max(ry1, ry2)

        # 判断点是否在矩形内
        def point_inside(x, y):
            return x_min <= x <= x_max and y_min <= y <= y_max

        # 检查端点是否在矩形内部
        if point_inside(x0, y0) or point_inside(x1, y1):
            return True

        # 检查线段与四条边是否相交
        def check_edge():
            # 左边缘 x = x_min
            if x0 == x1 == x_min:
                y_low = min(y0, y1)
                y_high = max(y0, y1)
                if y_high >= y_min and y_low <= y_max:
                    return True
            else:
                if (x0 <= x_min <= x1) or (x1 <= x_min <= x0):
                    if x1 - x0 != 0:
                        t = (x_min - x0) / (x1 - x0)
                        if 0 <= t <= 1:
                            y_intersect = y0 + t * (y1 - y0)
                            if y_min <= y_intersect <= y_max:
                                return True

            # 右边缘 x = x_max
            if x0 == x1 == x_max:
                y_low = min(y0, y1)
                y_high = max(y0, y1)
                if y_high >= y_min and y_low <= y_max:
                    return True
            else:
                if (x0 <= x_max <= x1) or (x1 <= x_max <= x0):
                    if x1 - x0 != 0:
                        t = (x_max - x0) / (x1 - x0)
                        if 0 <= t <= 1:
                            y_intersect = y0 + t * (y1 - y0)
                            if y_min <= y_intersect <= y_max:
                                return True

            # 上边缘 y = y_min
            if y0 == y1 == y_min:
                x_low = min(x0, x1)
                x_high = max(x0, x1)
                if x_high >= x_min and x_low <= x_max:
                    return True
            else:
                if (y0 <= y_min <= y1) or (y1 <= y_min <= y0):
                    if y1 - y0 != 0:
                        t = (y_min - y0) / (y1 - y0)
                        if 0 <= t <= 1:
                            x_intersect = x0 + t * (x1 - x0)
                            if x_min <= x_intersect <= x_max:
                                return True

            # 下边缘 y = y_max
            if y0 == y1 == y_max:
                x_low = min(x0, x1)
                x_high = max(x0, x1)
                if x_high >= x_min and x_low <= x_max:
                    return True
            else:
                if (y0 <= y_max <= y1) or (y1 <= y_max <= y0):
                    if y1 - y0 != 0:
                        t = (y_max - y0) / (y1 - y0)
                        if 0 <= t <= 1:
                            x_intersect = x0 + t * (x1 - x0)
                            if x_min <= x_intersect <= x_max:
                                return True
            return False
        res = check_edge()
        # print(f"目标区域：{rect}, 仿真时间：{self.Ego.curTime}，上一帧位置：{self.Ego.previousPos}，当前位置：{self.Ego.currentPos}, 是否经过：{res}")
        return res

    # 是否驶出了行车道——四轮在路检测
    def calcuOutOfLane(self, calEgoParam):
        return 0
        if not self.indexWeight["10006"]:
            return 0
        # 如果在交叉口范围内，则之间不对驶出车道和对象车道和压线进行检测
        # if self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"]):
        #     return 0
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
        # if not self.indexWeight["10003"]:
        #     return 0
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
            d = calEgoParam.cal_compacted_line_travel_count('solid')
        except:
            print("是否压实线评价错误")
            d = 0
        if d:
            return 1
        else:
            return 0

    # 是否压虚线
    def calcuOnDottedLaneMarking(self, calEgoParam):
        if not self.indexWeight["10013"]:
            return 0
        # 如果在交叉口范围内，则之间不对驶出车道和对象车道和压线进行检测
        # if self.calInside(self.Ego.currentPos, self.mapJsonInforDict["all_stop_line_points"]):
        #     return 0
        try:
            # todo 这个函数是计算压实线和双实线的，如果是虚线用的字段不一样
            d = calEgoParam.cal_compacted_line_travel_count('broken')
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
        if self.Ego.speed > Parameter.maxSpeedKM/3.6:
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
            # print(currPos, lastPos, self.Ego.accuracy)
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
        # 如果本来就是m/s那就不要动了
        lateralAcce = ((currPos['speed']) ** 2) / r
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
        # 如果本来就是m/s那就不要动了
        lateralAcce = (currPos['speed']) * w

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
        # if crashTime > 2 * accuracy:
        if crashTime > 0.01:
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









