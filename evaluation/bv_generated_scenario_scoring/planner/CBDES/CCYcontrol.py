#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import lib
import numpy as np
import pandas as pd
import math
from . import utils_cbdes
from . import getScenarioInfo


class CCYControl:
    def __init__(self, a_bound=3.0, exv=30, TimeGap=1, a=3, b=3, gama=4, s0=2.0, s1=2.0, jerk_bound=6, EmergencyAcc=15,
                 PreviewSpeedGain=0.1, PreviewDistance=2, LaneWidth=4,
                 DangerDistance=15, ShiftingDistance=15, EmergencyStopDistance=10):
        # def __init__(self, a_bound=5.0, exv=40, t=1.2, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
        """跟idm模型有关的模型参数，一定要记得调整

        :param a_bound: 本车加速度绝对值的上下界
        :param exv: 期望速度
        :param t: 反应时间
        :param a: 起步加速度
        :param b: 舒适减速度
        :param gama: 加速度指数
        :param s0: 静止安全距离
        :param s1: 与速度有关的安全距离选择参数
        """
        self.a_bound = a_bound
        self.jerk_bound = jerk_bound
        self.exv = exv
        self.TimeGap = TimeGap
        self.t = 0
        self.dt = 0
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_ = 0
        self.PreviewSpeedGain = PreviewSpeedGain
        self.PreviewDistance = PreviewDistance
        self.DangerDistance = DangerDistance
        self.ShiftingDistance = ShiftingDistance
        self.LaneWidth = LaneWidth
        self.LaneStatus = 'Middle'
        self.LastVehicleInfo = pd.DataFrame()
        self.VehicleInfo = pd.DataFrame()
        self.ArriveAtGoalPoint = False
        self.TrafficLight = 'Green'
        self.EmergencyStopDistance = EmergencyStopDistance
        self.EmergencyAcc = EmergencyAcc
        self.StoppingLinePoint = [np.inf, np.inf]
        self.TurningDirection = 'Straight'

    def act(self, observation, TrajectoryList):
        frame = pd.DataFrame()
        t = round(float(observation['test_setting']['t']), 2)  # 当前时间
        dt = round(float(observation['test_setting']['dt']), 2)  # 时间步长
        t_n = round(t + dt, 2)
        self.t = t
        self.dt = dt
        self.TrafficLight = observation['light_info']
        # 下步时间
        for key, value in observation['vehicle_info'].items():
            sub_frame = pd.DataFrame(value, columns=['x', 'y', 'v', 'a', 'yaw', 'lateral_v', 'lateral_a',
                                                     'steering', 'length', 'width',
                                                     'NearestPointOnRoad', 'IndexOnRoad', 'Distance2Road',
                                                     'DeltaThetaValue', 'VehicleStatus'], index=[key])
            '''
            if traj:
                if key != 'ego':  # 对于背景车
                    try:
                        x = traj[key][str(t)]['x']
                        y = traj[key][str(t)]['y']  # 当前位置
                        x_n = traj[key][str(t_n)]['x']
                        y_n = traj[key][str(t_n)]['y']  # 下步位置
                        sub_frame['v'] = ((x_n - x) ** 2 + (y_n - y) ** 2) ** 0.5 / dt  # 计算背景车速度
                    except KeyError:  # 缺失背景车数据
                        sub_frame['v'] = 0
            '''
            frame = pd.concat([frame, sub_frame])
        self.VehicleInfo = frame
        # Yaw Calibration
        getScenarioInfo.YawCalibration(self.VehicleInfo)
        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'FreeDriving'
        egoTrajectory = self.getVehicleInformation(TrajectoryList)
        acc = self.desire_acc(egoTrajectory)
        steering, PreviewPoint = self.desire_steering(egoTrajectory)
        self.LastVehicleInfo = self.VehicleInfo.copy()
        
        # Arriving at goal point
        if self.ArriveAtGoalPoint:
            acc = self.a_bound
            steering = 0
        # print(t)
        # print(self.TrafficLight)
        # print(self.VehicleInfo)
        return acc, steering

    def getVehicleInformation(self, TrajectoryList):
        VehicleInfo = self.VehicleInfo
        LastVehicleInfo = self.LastVehicleInfo

        # Find Lane
        VehicleCount4LaneSelection = np.zeros([len(TrajectoryList.index)])
        VehicleDistanceToLane = np.zeros([len(TrajectoryList.index), len(VehicleInfo.index)])
        for LaneIndex in TrajectoryList.index:
            _, EgoIndex, EgoDistance, _, _ = \
                self.getNearestPointIndexOnTrajectory(VehicleInfo.at['ego', 'x'], VehicleInfo.at['ego', 'y'],
                                                      VehicleInfo.at['ego', 'yaw'],
                                                      TrajectoryList.at[LaneIndex, 'Trajectory'])
            VehicleDistanceToLane[LaneIndex, 0] = EgoDistance
            for VehicleID in VehicleInfo.index:
                if VehicleID == 'ego':
                    continue
                _, VehicleIndex, VehicleDistance, _, _ = \
                    self.getNearestPointIndexOnTrajectory(VehicleInfo.at[VehicleID, 'x'],
                                                          VehicleInfo.at[VehicleID, 'y'],
                                                          VehicleInfo.at[VehicleID, 'yaw'],
                                                          TrajectoryList.at[LaneIndex, 'Trajectory'])
                VehicleDistanceToLane[LaneIndex, np.where(VehicleInfo.index == VehicleID)[0][0]] = VehicleDistance
                if VehicleIndex > (EgoIndex - 20) and VehicleDistance < self.LaneWidth / 2:
                    # if self.t == 3.8:
                    #     a = 1
                    VehicleCount4LaneSelection[LaneIndex] = VehicleCount4LaneSelection[LaneIndex] + 1
        # print(VehicleCount4LaneSelection)
        SelectLaneIndexList = np.where(VehicleCount4LaneSelection == min(VehicleCount4LaneSelection))[0]
        if len(SelectLaneIndexList) == 1:
            SelectLaneIndex = SelectLaneIndexList[0]
        else:
            if len(LastVehicleInfo.index) > 0:
                LastSelectLaneIndex = LastVehicleInfo.at['ego', 'SelectLane']
                SelectLaneIndex = SelectLaneIndexList[np.where(abs(SelectLaneIndexList - LastSelectLaneIndex) == min(
                    abs(SelectLaneIndexList - LastSelectLaneIndex)))[0][0]]
            else:
                SelectLaneIndex = SelectLaneIndexList[0]

        # 有车阻碍换道
        EgoVehicleLaneStatus = TrajectoryList.at[np.where(VehicleDistanceToLane[:, 0] == min(VehicleDistanceToLane[:, 0]))[0][0], 'LaneStatus']
        IntentionLaneStatus = TrajectoryList.at[SelectLaneIndex, 'LaneStatus']
        TuringRightFlag = (EgoVehicleLaneStatus == 'Left' and IntentionLaneStatus != 'Left') \
                          or (EgoVehicleLaneStatus == 'Middle' and IntentionLaneStatus == 'Right')
        TuringLeftFlag = (EgoVehicleLaneStatus == 'Right' and IntentionLaneStatus != 'Right') \
                          or (EgoVehicleLaneStatus == 'Middle' and IntentionLaneStatus == 'Left')
        if TuringRightFlag:
            for index in VehicleInfo.index:
                if index == 'ego':
                    continue
                DangerDistanceFlag = ((VehicleInfo.at['ego', 'x'] - VehicleInfo.at[index, 'x']) ** 2 +
                                      (VehicleInfo.at['ego', 'y'] - VehicleInfo.at[index, 'y']) ** 2) ** 0.5 < self.LaneWidth
                VehicleLaneStatus = TrajectoryList.at[np.where(VehicleDistanceToLane[:, np.where(VehicleInfo.index == index)[0][0]] ==
                                                               min(VehicleDistanceToLane[:, np.where(VehicleInfo.index == index)[0][0]]))[0][0], 'LaneStatus']
                BlockingDirectionFlag = (EgoVehicleLaneStatus == 'Left' and VehicleLaneStatus == 'Middle') or \
                    (EgoVehicleLaneStatus == 'Middle' and VehicleLaneStatus == 'Right')
                if BlockingDirectionFlag and DangerDistanceFlag:
                        SelectLaneIndex = np.where(VehicleDistanceToLane[:, 0] == min(VehicleDistanceToLane[:, 0]))[0][0]
        elif TuringLeftFlag:
            for index in VehicleInfo.index:
                if index == 'ego':
                    continue
                DangerDistanceFlag = ((VehicleInfo.at['ego', 'x'] - VehicleInfo.at[index, 'x']) ** 2 +
                                      (VehicleInfo.at['ego', 'y'] - VehicleInfo.at[index, 'y']) ** 2) ** 0.5 < self.LaneWidth
                VehicleLaneStatus = TrajectoryList.at[np.where(VehicleDistanceToLane[:, np.where(VehicleInfo.index == index)[0][0]] ==
                                                               min(VehicleDistanceToLane[:, np.where(VehicleInfo.index == index)[0][0]]))[0][0], 'LaneStatus']
                BlockingDirectionFlag = (EgoVehicleLaneStatus == 'Middle' and VehicleLaneStatus == 'Left') or \
                    (EgoVehicleLaneStatus == 'Right' and VehicleLaneStatus == 'Middle')
                if DangerDistanceFlag and BlockingDirectionFlag:
                    SelectLaneIndex = np.where(VehicleDistanceToLane[:, 0] == min(VehicleDistanceToLane[:, 0]))[0][0]

        VehicleInfo.at['ego', 'SelectLane'] = SelectLaneIndex
        ReferencePath = TrajectoryList.at[SelectLaneIndex, 'Trajectory'].copy()
        self.LaneStatus = TrajectoryList.at[SelectLaneIndex, 'LaneStatus']
        self.StoppingLinePoint = TrajectoryList.at[SelectLaneIndex, 'StoppingLinePoint']
        self.TurningDirection = TrajectoryList.at[SelectLaneIndex, 'TurningDirection']

        for index, OneVehicleData in VehicleInfo.iterrows():
            VehicleOnRoad, VehicleIndex, VehicleDistance, DeviateAngle, Vehicle2RoadAngle = \
                self.getNearestPointIndexOnTrajectory(OneVehicleData['x'], OneVehicleData['y'], OneVehicleData['yaw'],
                                                      ReferencePath)
            VehicleInfo.at[index, 'NearestPointOnRoad'] = VehicleOnRoad
            VehicleInfo.at[index, 'IndexOnRoad'] = VehicleIndex
            VehicleInfo.at[index, 'Distance2Road'] = VehicleDistance
            VehicleInfo.at[index, 'DeltaThetaValue'] = DeviateAngle
            VehicleInfo.at[index, 'Vehicle2RoadAngle'] = Vehicle2RoadAngle
            if index != 'ego':
                # print(VehicleInfo)
                ProjectionSpeed = VehicleInfo.at['ego', 'v'] * np.cos(VehicleInfo.at['ego', 'DeltaThetaValue']) - \
                                  VehicleInfo.at[index, 'v'] * np.cos(VehicleInfo.at[index, 'DeltaThetaValue'])
                InEgoTrajectoryFlag = VehicleInfo.at[index, 'Distance2Road'] < (
                            VehicleInfo.at[index, 'width'] + VehicleInfo.at['ego', 'width']) * 2 / 3
                ObstacleFlag = VehicleInfo.at[index, 'DeltaThetaValue'] < np.pi / 18 and InEgoTrajectoryFlag
                DangerDistanceFlag = ((VehicleInfo.at['ego', 'x'] - VehicleInfo.at[index, 'x']) ** 2 +
                                      (VehicleInfo.at['ego', 'y'] - VehicleInfo.at[index, 'y']) ** 2) ** 0.5 - \
                                     (VehicleInfo.at[index, 'length'] + VehicleInfo.at['ego', 'length']) / 2 < \
                                     (self.DangerDistance + self.TimeGap * abs(ProjectionSpeed))
                PassFlag = VehicleInfo.at[index, 'IndexOnRoad'] < VehicleInfo.at['ego', 'IndexOnRoad']
                Driving2RoadFlag = abs(
                    VehicleInfo.at[index, 'Vehicle2RoadAngle'] - VehicleInfo.at[index, 'yaw']) < np.pi / 3
                DriveAlongFlag = VehicleInfo.at[
                                     index, 'DeltaThetaValue'] < np.pi / 18 and not InEgoTrajectoryFlag and not Driving2RoadFlag
                # 紧急刹车
                ProjectionSpeed4Braking = ProjectionSpeed.copy()
                if ProjectionSpeed4Braking < 0:
                    ProjectionSpeed4Braking = 0
                EmergencyStopFlag = ((VehicleInfo.at['ego', 'x'] - VehicleInfo.at[index, 'x']) ** 2 +
                                     (VehicleInfo.at['ego', 'y'] - VehicleInfo.at[index, 'y']) ** 2) ** 0.5 - \
                                    (VehicleInfo.at[index, 'length'] + VehicleInfo.at['ego', 'length']) / 2 < \
                                    (ProjectionSpeed4Braking) ** 2 / (2 * self.EmergencyAcc) + self.EmergencyStopDistance
                if index in LastVehicleInfo.index:
                    if LastVehicleInfo.at[index, 'VehicleStatus'] == 'Obstacle':
                        EmergencyStopFlag = False
                if DriveAlongFlag:
                    EmergencyStopFlag = False

                CrossingPointInTrajectory = getScenarioInfo.getCords(VehicleInfo.at[index, 'x'], VehicleInfo.at[index, 'y'],
                                                                      VehicleInfo.at[index, 'yaw'], ReferencePath,
                                                                      max(ReferencePath[:, 0]) - min(ReferencePath[:, 0]),
                                                                      max(ReferencePath[:, 1]) - min(ReferencePath[:, 1]))
                EgoVehicleDistanceToCrossingPoint = ((CrossingPointInTrajectory[0] - VehicleInfo.at['ego', 'x']) ** 2 + (CrossingPointInTrajectory[1] - VehicleInfo.at['ego', 'y']) ** 2) ** 0.5
                OtherVehicleDistanceToCrossingPoint = ((CrossingPointInTrajectory[0] - VehicleInfo.at[index, 'x']) ** 2 + (CrossingPointInTrajectory[1] - VehicleInfo.at[index, 'y']) ** 2) ** 0.5
                OtherVehicleTimeToCrossingPoint = OtherVehicleDistanceToCrossingPoint / (VehicleInfo.at[index, 'v'] + 10e-5)

                a = 0.5 * self.a_bound
                b = VehicleInfo.at['ego', 'v'] + 10e-5
                c = - EgoVehicleDistanceToCrossingPoint
                delta = b ** 2 - 4 * a * c
                result = np.zeros(2)
                if delta < 0:
                    raise Exception('Equation Solving Error')
                elif delta == 0:
                    EgoVehicleTimeToCrossingPoint = (-b + math.sqrt(delta)) / 2 * a
                else:
                    result[0] = (-b + math.sqrt(delta)) / 2 * a
                    result[1] = (-b - math.sqrt(delta)) / 2 * a
                    EgoVehicleTimeToCrossingPoint = result[result > 0][0]
                EgoEarlyArriveFlag = OtherVehicleTimeToCrossingPoint - EgoVehicleTimeToCrossingPoint > self.TimeGap

                if (EgoVehicleDistanceToCrossingPoint < VehicleInfo.at['ego', 'length']) or (Driving2RoadFlag and EgoEarlyArriveFlag) or \
                        (abs(VehicleInfo.at[index, 'v']) < 0.1 and not InEgoTrajectoryFlag):
                    EmergencyAcceleration = True
                else:
                    EmergencyAcceleration = False

                if ObstacleFlag and ProjectionSpeed < 0 and DangerDistanceFlag:
                    RearEndCollisionFlag = True
                else:
                    RearEndCollisionFlag = False

                if index == 6:
                    a = 1

                if PassFlag:
                    if RearEndCollisionFlag:
                        VehicleInfo.at['ego', 'VehicleStatus'] = 'RearEndAccel'
                        VehicleInfo.at[index, 'VehicleStatus'] = 'RearEndCollision'
                    else:
                        if index in LastVehicleInfo.index:
                            if not LastVehicleInfo.at[index, 'VehicleStatus'] == 'Obstacle':
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Irrelevant'
                        else:
                            VehicleInfo.at[index, 'VehicleStatus'] = 'Irrelevant'
                else:
                    if ObstacleFlag:
                        if EmergencyStopFlag:
                            VehicleInfo.at[index, 'VehicleStatus'] = 'EmergencyObstacle'
                            if EmergencyAcceleration and VehicleInfo.at['ego', 'VehicleStatus'] != 'EmergencyStop':
                                VehicleInfo.at['ego', 'VehicleStatus'] = 'EmergencyAccel'
                            else:
                                VehicleInfo.at['ego', 'VehicleStatus'] = 'EmergencyStop'
                        else:
                            VehicleInfo.at[index, 'VehicleStatus'] = 'Obstacle'
                    elif EmergencyStopFlag and (Driving2RoadFlag or InEgoTrajectoryFlag):
                        VehicleInfo.at[index, 'VehicleStatus'] = 'EmergencyObstacle'
                        if EmergencyAcceleration and VehicleInfo.at['ego', 'VehicleStatus'] != 'EmergencyStop':
                            VehicleInfo.at['ego', 'VehicleStatus'] = 'EmergencyAccel'
                        else:
                            VehicleInfo.at['ego', 'VehicleStatus'] = 'EmergencyStop'
                    elif DangerDistanceFlag and (Driving2RoadFlag or InEgoTrajectoryFlag):
                        VehicleInfo.at[index, 'VehicleStatus'] = 'Collision'
                    elif Driving2RoadFlag:
                        VehicleInfo.at[index, 'VehicleStatus'] = 'PotentialCollision'
                    else:
                        VehicleInfo.at[index, 'VehicleStatus'] = 'Undetermined'

        # Re-Path Planning
        if 'Obstacle' in list(VehicleInfo['VehicleStatus']):
            ObstacleVehicleInfo = VehicleInfo[VehicleInfo['VehicleStatus'] == 'Obstacle']
            for index, _ in ObstacleVehicleInfo.iterrows():
                ObstacleOnRoad_X = ObstacleVehicleInfo.at[index, 'NearestPointOnRoad'][0]
                ObstacleOnRoad_Y = ObstacleVehicleInfo.at[index, 'NearestPointOnRoad'][1]
                LaneChanging_StartPoint, LaneChanging_StartPointIndex, LaneChanging_EndPoint, LaneChanging_EndPointIndex, \
                    ObstaclePoint, ObstacleShiftingDirection = \
                    self.getKeyPoints(ObstacleOnRoad_X, ObstacleOnRoad_Y, ReferencePath, ObstacleVehicleInfo, index)
                NewCurve = utils_cbdes.DoubleLaneChange(LaneChanging_StartPoint, LaneChanging_EndPoint, ObstaclePoint)
                # Replace new path
                Line1 = ReferencePath[0:LaneChanging_StartPointIndex]
                Line3 = ReferencePath[LaneChanging_EndPointIndex:]
                Line12 = np.concatenate((Line1, NewCurve))
                egoTrajectory = np.concatenate((Line12, Line3))
        else:
            egoTrajectory = ReferencePath.copy()

        return egoTrajectory

    def getKeyPoints(self, ObstacleOnRoad_X, ObstacleOnRoad_Y, egoTrajectory, ObstacleVehicleInfo, index):
        # Set Four Detect Points
        ShiftingPoint = pd.DataFrame(
            columns=['ShiftingPoint', 'PointOnRoad', 'PointIndexOnRoad', 'PointDistance2Road'],
            index=['x1', 'x2', 'y1', 'y2'])
        for ShiftingPointIndex, _ in ShiftingPoint.iterrows():
            if ShiftingPointIndex == 'x1':
                ShiftingPoint.at[ShiftingPointIndex, 'ShiftingPoint'] = \
                    [ObstacleOnRoad_X + self.ShiftingDistance, ObstacleOnRoad_Y]
            elif ShiftingPointIndex == 'x2':
                ShiftingPoint.at[ShiftingPointIndex, 'ShiftingPoint'] = \
                    [ObstacleOnRoad_X - self.ShiftingDistance, ObstacleOnRoad_Y]
            elif ShiftingPointIndex == 'y1':
                ShiftingPoint.at[ShiftingPointIndex, 'ShiftingPoint'] = \
                    [ObstacleOnRoad_X, ObstacleOnRoad_Y + self.ShiftingDistance]
            elif ShiftingPointIndex == 'y2':
                ShiftingPoint.at[ShiftingPointIndex, 'ShiftingPoint'] = \
                    [ObstacleOnRoad_X, ObstacleOnRoad_Y - self.ShiftingDistance]
            else:
                raise Exception('Shifting Point Error')
            PointOnRoad, PointIndexOnRoad, PointDistance2Road, _, _ = \
                self.getNearestPointIndexOnTrajectory(ShiftingPoint.at[ShiftingPointIndex, 'ShiftingPoint'][0],
                                                      ShiftingPoint.at[ShiftingPointIndex, 'ShiftingPoint'][1],
                                                      0, egoTrajectory)
            ShiftingPoint.at[ShiftingPointIndex, 'PointOnRoad'] = PointOnRoad
            ShiftingPoint.at[ShiftingPointIndex, 'PointIndexOnRoad'] = PointIndexOnRoad
            ShiftingPoint.at[ShiftingPointIndex, 'PointDistance2Road'] = PointDistance2Road

        # Select obstacle direction
        if (ShiftingPoint.at['x1', 'PointDistance2Road'] + ShiftingPoint.at['x2', 'PointDistance2Road']) <= \
                (ShiftingPoint.at['y1', 'PointDistance2Road'] + ShiftingPoint.at['y2', 'PointDistance2Road']):
            # Driving in from east
            if ShiftingPoint.at['x1', 'PointIndexOnRoad'] <= ShiftingPoint.at['x2', 'PointIndexOnRoad']:
                # Lane change to left
                if ObstacleVehicleInfo.at[index, 'y'] - ObstacleOnRoad_Y > (
                        self.VehicleInfo.at['ego', 'width'] + ObstacleVehicleInfo.at[index, 'width']) / 2:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Right':
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y - self.LaneWidth]
                        ObstacleShiftingDirection = '-y'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '-y_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                # Lane change to right
                else:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Left':
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y + self.LaneWidth]
                        ObstacleShiftingDirection = '+y'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '+y_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                LaneChanging_StartPoint = ShiftingPoint.at['x1', 'PointOnRoad']
                LaneChanging_StartPointIndex = ShiftingPoint.at['x1', 'PointIndexOnRoad']
                LaneChanging_EndPoint = ShiftingPoint.at['x2', 'PointOnRoad']
                LaneChanging_EndPointIndex = ShiftingPoint.at['x2', 'PointIndexOnRoad']
            # Driving in from west
            else:
                # Lane change to right
                if ObstacleVehicleInfo.at[index, 'y'] - ObstacleOnRoad_Y > (
                        self.VehicleInfo.at['ego', 'width'] + ObstacleVehicleInfo.at[index, 'width']) / 2:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Left':
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y - self.LaneWidth]
                        ObstacleShiftingDirection = '-y'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '-y_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                # Lane change to left
                else:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Right':
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y + self.LaneWidth]
                        ObstacleShiftingDirection = '+y'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '+y_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                LaneChanging_StartPoint = ShiftingPoint.at['x2', 'PointOnRoad']
                LaneChanging_StartPointIndex = ShiftingPoint.at['x2', 'PointIndexOnRoad']
                LaneChanging_EndPoint = ShiftingPoint.at['x1', 'PointOnRoad']
                LaneChanging_EndPointIndex = ShiftingPoint.at['x1', 'PointIndexOnRoad']
        else:
            # Driving in from north
            if ShiftingPoint.at['y1', 'PointIndexOnRoad'] <= ShiftingPoint.at['y2', 'PointIndexOnRoad']:
                # Lane change to right
                if ObstacleVehicleInfo.at[index, 'x'] - ObstacleOnRoad_X > (
                        self.VehicleInfo.at['ego', 'width'] + ObstacleVehicleInfo.at[index, 'width']) / 2:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Left':
                        ObstaclePoint = [ObstacleOnRoad_X - self.LaneWidth, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '-x'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '-x_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                # Lane change to left
                else:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Right':
                        ObstaclePoint = [ObstacleOnRoad_X + self.LaneWidth, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '+x'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '+x_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                LaneChanging_StartPoint = ShiftingPoint.at['y1', 'PointOnRoad']
                LaneChanging_StartPointIndex = ShiftingPoint.at['y1', 'PointIndexOnRoad']
                LaneChanging_EndPoint = ShiftingPoint.at['y2', 'PointOnRoad']
                LaneChanging_EndPointIndex = ShiftingPoint.at['y2', 'PointIndexOnRoad']
            # Driving in from south
            else:
                # Lane change to left
                if ObstacleVehicleInfo.at[index, 'x'] - ObstacleOnRoad_X > (
                        self.VehicleInfo.at['ego', 'width'] + ObstacleVehicleInfo.at[index, 'width']) / 2:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Right':
                        ObstaclePoint = [ObstacleOnRoad_X - self.LaneWidth, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '-x'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '-x_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                # Lane change to right
                else:
                    if self.LaneStatus == 'Middle' or self.LaneStatus == 'Left':
                        ObstaclePoint = [ObstacleOnRoad_X + self.LaneWidth, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '+x'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Overtaking'
                    else:
                        ObstaclePoint = [ObstacleOnRoad_X, ObstacleOnRoad_Y]
                        ObstacleShiftingDirection = '+x_block'
                        self.VehicleInfo.at['ego', 'VehicleStatus'] = 'Blocking'
                LaneChanging_StartPoint = ShiftingPoint.at['y2', 'PointOnRoad']
                LaneChanging_StartPointIndex = ShiftingPoint.at['y2', 'PointIndexOnRoad']
                LaneChanging_EndPoint = ShiftingPoint.at['y1', 'PointOnRoad']
                LaneChanging_EndPointIndex = ShiftingPoint.at['y1', 'PointIndexOnRoad']
        return LaneChanging_StartPoint, LaneChanging_StartPointIndex, LaneChanging_EndPoint, LaneChanging_EndPointIndex, ObstaclePoint, ObstacleShiftingDirection

    def getPrecedingVehicleInfo(self, egoTrajectory):
        VehicleInfo = self.VehicleInfo
        EgoVehicleX = VehicleInfo.at['ego', 'x']
        EgoVehicleY = VehicleInfo.at['ego', 'y']
        EgoVehicleYaw = VehicleInfo.at['ego', 'yaw']
        EgoVehicleLength = VehicleInfo.at['ego', 'length']
        EgoVehicleWidth = VehicleInfo.at['ego', 'width']

        # 有前车阻碍
        if VehicleInfo.at['ego', 'VehicleStatus'] == 'Blocking':
            SelectCollision = VehicleInfo['VehicleStatus'] == 'Collision'       
            SelectObstacle = (VehicleInfo['VehicleStatus'] == 'Obstacle') 
            SelectEObstacle = (VehicleInfo['VehicleStatus'] == 'EmergencyObstacle')
            CollisionVehicleInfo = VehicleInfo[(SelectCollision | SelectObstacle) | SelectEObstacle]
        else:
            CollisionVehicleInfo = VehicleInfo[VehicleInfo['VehicleStatus'] == 'Collision']

        # 信号灯引入障碍车
        GreenTLFlag = self.TrafficLight != 'red'
        TurnRightFlag = self.TurningDirection == 'Right'
        if GreenTLFlag:
            a = 1
        if not GreenTLFlag and not TurnRightFlag:
            VirtualVehicleX = self.StoppingLinePoint[0]
            VirtualVehicleY = self.StoppingLinePoint[1]
            _, VirtualVehicleIndex, Distance2Road, _, _ = \
                self.getNearestPointIndexOnTrajectory(VirtualVehicleX, VirtualVehicleY, EgoVehicleYaw, egoTrajectory)
            tempFrame = {"x": VirtualVehicleX,
                         "y": VirtualVehicleY,
                         "v": 0,
                         "a": 0,
                         "yaw": EgoVehicleYaw,
                         "lateral_v": 0,
                         "lateral_a": 0,
                         "steering": 0,
                         "length": EgoVehicleLength / 2,
                         "width": EgoVehicleWidth,
                         "NearestPointOnRoad": "",
                         "IndexOnRoad": VirtualVehicleIndex,
                         "Distance2Road": Distance2Road,
                         "DeltaThetaValue": "",
                         "VehicleStatus": "",
                         "SelectLane": "",
                         "Vehicle2RoadAngle": ""}
            frame = pd.DataFrame(data=tempFrame, index=['Virtual'])
            CollisionVehicleInfo = pd.concat([CollisionVehicleInfo, frame])
            # a = 1

        if len(CollisionVehicleInfo.index) > 0:
            NearestVehicleInfo = CollisionVehicleInfo[CollisionVehicleInfo['IndexOnRoad'] == min(CollisionVehicleInfo['IndexOnRoad'])]
            NearestVehicleInfo = NearestVehicleInfo[NearestVehicleInfo['Distance2Road'] == min(NearestVehicleInfo['Distance2Road'])]
            NearestVehicleIndex = NearestVehicleInfo.index[0]
            if VehicleInfo.at['ego', 'VehicleStatus'] == 'FreeDriving':
                if NearestVehicleIndex == 'Virtual':
                    VehicleInfo.at['ego', 'VehicleStatus'] = 'WaitingTrafficLight'
                else:
                    VehicleInfo.at['ego', 'VehicleStatus'] = 'Following'
            dis_gap = ((NearestVehicleInfo.at[NearestVehicleIndex, 'x'] - EgoVehicleX) ** 2 +
                       (NearestVehicleInfo.at[NearestVehicleIndex, 'y'] - EgoVehicleY) ** 2) ** 0.5 - \
                      (EgoVehicleLength + NearestVehicleInfo.at[NearestVehicleIndex, 'length']) / 2
            fv = NearestVehicleInfo.at[NearestVehicleIndex, 'v']
            # print('Following Vehicle', list(NearestVehicleInfo.index))
        # 无前车阻碍
        else:
            dis_gap = -1
            fv = -1
            # print('No Following Vehicle')
        if dis_gap > 100:
            dis_gap = -1
            fv = -1
        return fv, dis_gap

    def desire_acc(self, egoTrajectory):
        dt = self.dt
        t = self.t
        VehicleInfo = self.VehicleInfo
        # 紧急避撞
        if VehicleInfo.at['ego', 'VehicleStatus'] == 'EmergencyStop':
            return -self.EmergencyAcc
        elif VehicleInfo.at['ego', 'VehicleStatus'] == 'EmergencyAccel':
            return self.a_bound
        elif VehicleInfo.at['ego', 'VehicleStatus'] == 'RearEndAccel':
            return self.a_bound
        LastVehicleInfo = self.LastVehicleInfo
        fv, dis_gap = self.getPrecedingVehicleInfo(egoTrajectory)
        v = VehicleInfo.at['ego', 'v']
        if dis_gap < 0:
            a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        else:
            # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.TimeGap)
            # self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.TimeGap * v + v * (v - fv) / 2 / (self.a * self.b) ** 0.5
            self.s_ = self.s0 + max(0, self.TimeGap * v + v * (v - fv) / 2 / (self.a * self.b) ** 0.5)
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap + 1e-6)) ** 2))

        # 对加加速度进行约束
        if len(LastVehicleInfo.index) != 0:
            EgoVehicleLastTimeAcc = LastVehicleInfo.at['ego', 'a']
            jerk_idm = np.clip((a_idm - EgoVehicleLastTimeAcc) / dt, -self.jerk_bound, self.jerk_bound)
            acc1 = EgoVehicleLastTimeAcc + dt * jerk_idm
        else:
            acc1 = a_idm

        if t > 0:
            # 转弯向心加速度约束
            CurvatureData, _ = getScenarioInfo.getCurvature(egoTrajectory)
            Curvature = CurvatureData[VehicleInfo.at['ego', 'IndexOnRoad']]
            if Curvature != 0 and abs(VehicleInfo.at['ego', 'DeltaThetaValue']) > np.pi / 18:
                LastTimeVehicleV = LastVehicleInfo.at['ego', 'v']
                CurvatureBoundValue = (np.sqrt(1 / abs(Curvature)) - LastTimeVehicleV) / dt
                acc2 = np.clip(acc1, -np.inf, CurvatureBoundValue)
                # print(acc2)
            else:
                acc2 = acc1

            # HeadingEastFlag = -np.pi / 4 < LastVehicleInfo.at['ego', 'yaw'] <= np.pi / 4
            # HeadingNorthFlag = np.pi / 4 < LastVehicleInfo.at['ego', 'yaw'] <= 3 * np.pi / 4
            # HeadingWestFlag = 3 * np.pi / 4 < LastVehicleInfo.at['ego', 'yaw'] or LastVehicleInfo.at[
            #     'ego', 'yaw'] <= -3 * np.pi / 4
            # HeadingSouthFlag = -3 * np.pi / 4 < LastVehicleInfo.at['ego', 'yaw'] <= - np.pi / 4
            # if HeadingEastFlag or HeadingWestFlag:
            #     VehicleInfo.at['ego', 'lateral_v'] = (VehicleInfo.at['ego', 'y'] - LastVehicleInfo.at['ego', 'y']) / dt
            # elif HeadingNorthFlag or HeadingSouthFlag:
            #     VehicleInfo.at['ego', 'lateral_v'] = (VehicleInfo.at['ego', 'x'] - LastVehicleInfo.at['ego', 'x']) / dt
            # else:
            #     raise Exception('Ego Vehicle Heading Error')
            # if t > 0.1:
            #     VehicleInfo.at['ego', 'lateral_a'] = (VehicleInfo.at['ego', 'lateral_v'] - LastVehicleInfo.at[
            #         'ego', 'lateral_v']) / dt
            #     # print(VehicleInfo.at['ego', 'lateral_a'])
            #     # a = 1
            # 横向加速度约束
            # 横向加加速度约束
        else:
            acc2 = acc1

        acc_final = acc2
        # 对加速度进行约束
        acc_final = np.clip(acc_final, -self.a_bound, self.a_bound)
        VehicleInfo.at['ego', 'a'] = acc_final
        return acc_final

    def desire_steering(self, egoTrajectory):
        VehicleInfo = self.VehicleInfo
        EgoVehicleX = VehicleInfo.at['ego', 'x']
        EgoVehicleY = VehicleInfo.at['ego', 'y']
        EgoVehicleYaw = VehicleInfo.at['ego', 'yaw']
        EgoVehicleLength = VehicleInfo.at['ego', 'length']
        WheelBase = EgoVehicleLength / 1.7
        ActualPreviewDistance = self.PreviewSpeedGain * VehicleInfo.at['ego', 'v'] + self.PreviewDistance
        PreviewPoint = self.getPreviewPoint(EgoVehicleX, EgoVehicleY, EgoVehicleYaw, egoTrajectory)
        alpha = math.atan2(PreviewPoint[1] - EgoVehicleY, PreviewPoint[0] - EgoVehicleX) - EgoVehicleYaw
        delta = math.atan2(2 * WheelBase * np.sin(alpha), ActualPreviewDistance)
        VehicleInfo.at['ego', 'steering'] = delta
        return delta, PreviewPoint

    def getPreviewPoint(self, EgoVehicleX, EgoVehicleY, EgoVehicleYaw, egoTrajectory):
        # Find Nearest Point
        _, NearestPointIndex, _, _, _ = \
            self.getNearestPointIndexOnTrajectory(EgoVehicleX, EgoVehicleY, EgoVehicleYaw, egoTrajectory)
        if self.t == 6.5:
            a = 1
        # Find Preview Point
        PreviewPoint = np.zeros(2)
        # PreviewPointIndex = 0
        ActualPreviewDistance = self.PreviewSpeedGain * self.VehicleInfo.at['ego', 'v'] + self.PreviewDistance
        for TempPointIndex in range(NearestPointIndex, len(egoTrajectory)):
            TempPoint = egoTrajectory[TempPointIndex]
            TempDistance = ((TempPoint[0] - EgoVehicleX) ** 2 + (TempPoint[1] - EgoVehicleY) ** 2) ** 0.5
            if TempDistance > ActualPreviewDistance:
                PreviewPoint[0] = TempPoint[0]
                PreviewPoint[1] = TempPoint[1]
                # PreviewPointIndex = TempPointIndex
                break
        if abs(len(egoTrajectory) - NearestPointIndex) < 5 or np.all(PreviewPoint == [0, 0]):
            self.ArriveAtGoalPoint = True
        else:
            self.ArriveAtGoalPoint = False
        return PreviewPoint

    def getNearestPointIndexOnTrajectory(self, EgoVehicleX, EgoVehicleY, EgoVehicleYaw, egoTrajectory):
        NearestPointOnRoad = np.zeros(2)
        NearestPointIndex = 0
        NearestPointDistance = 10000
        for TempPointIndex in range(len(egoTrajectory)):
            TempPoint = egoTrajectory[TempPointIndex]
            TempDistance = ((TempPoint[0] - EgoVehicleX) ** 2 + (TempPoint[1] - EgoVehicleY) ** 2) ** 0.5
            if TempDistance < NearestPointDistance:
                NearestPointOnRoad[0] = TempPoint[0]
                NearestPointOnRoad[1] = TempPoint[1]
                NearestPointIndex = TempPointIndex
                NearestPointDistance = TempDistance
        # Find Road Yaw
        if NearestPointIndex == 0:
            StartPointIndex = 0
            EndPointIndex = 1
        else:
            StartPointIndex = NearestPointIndex - 1
            EndPointIndex = NearestPointIndex
        StartPoint = egoTrajectory[StartPointIndex]
        EndPoint = egoTrajectory[EndPointIndex]
        RoadAngle = math.atan2(EndPoint[1] - StartPoint[1], EndPoint[0] - StartPoint[0])
        RoadAngle = getScenarioInfo.AngleCalibration(RoadAngle)
        DeltaThetaValue = abs(RoadAngle - EgoVehicleYaw)
        if DeltaThetaValue > np.pi:
            DeltaThetaValue = np.pi * 2 - DeltaThetaValue
        Vehicle2RoadAngle = math.atan2(NearestPointOnRoad[1] - EgoVehicleY, NearestPointOnRoad[0] - EgoVehicleX)
        Vehicle2RoadAngle = getScenarioInfo.AngleCalibration(Vehicle2RoadAngle)
        return NearestPointOnRoad, NearestPointIndex, NearestPointDistance, DeltaThetaValue, Vehicle2RoadAngle
