#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import numpy as np
import pandas as pd
import math
import sys
#import utils
import time
from collections import defaultdict



class KTQControl():
    def __init__(self, a_bound=3.0, exv=10, t=1, a=3, b=3, gama=4, s0=2.0, s1=2.0, PreviewDistance=3,
                 DangerDistance=15, LaneWidth=4, ShiftingDistance=50):
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
        self.exv = exv
        self.t = t
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1
        self.s_ = 0
        self.PreviewDistance = PreviewDistance
        self.DangerDistance = DangerDistance
        self.LaneWidth = LaneWidth
        self.ShiftingDistance = ShiftingDistance

    def ktq_act(self, observation, task_info, planedLanes, AllLanesDict, rampFlag):
        frame = pd.DataFrame()
        for key, value in observation['vehicle_info'].items():
                sub_frame = pd.DataFrame(value,columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[key])
                frame = pd.concat([frame, sub_frame])

        state = frame.to_numpy()
        
        #a=1

        ego = state[0, :]
        ## # print("ego state: ", ego)
        goalPointX = [task_info['targetPos'][0][0], task_info['targetPos'][1][0]]
        goalPointY = [task_info['targetPos'][0][1], task_info['targetPos'][1][1]]
        allRoadPoints = []
        # lastPoint = []
        for planedLaneID in planedLanes:
            centerLine=AllLanesDict[planedLaneID].center_vertices
            for point in centerLine:
                # if ((point[0]-min(goalPointX))**2 + (point[1]-np.average(goalPointY))**2) ** 0.5 < 20:
                #     break
                allRoadPoints.append([point[0], point[1]])
                # lastPoint.append(point)
        

        # lastBut10Point = lastPoint[len(lastPoint)-10]
        # for iii in range(1000):
        #     if ego[3] < np.pi/2 or ego[3] > np.pi*3/2:
        #         allRoadPoints.append([np.mean(goalPointX)+iii/2, np.average(goalPointY) + iii/2 * (np.average(goalPointY) - lastBut10Point[1])/(np.average(goalPointX) - lastBut10Point[0])])
        #     else:
        #         allRoadPoints.append([np.mean(goalPointX) - iii / 2, np.average(goalPointY) - iii / 2 * (np.average(goalPointY) - lastBut10Point[1])/(np.average(goalPointX) - lastBut10Point[0])])
        # # print("allRoadPoints:  ", allRoadPoints)

        # file_path = "all_road_points.txt"

        # # 将数据写入文本文件
        # with open(file_path, 'w') as file:
        #     for point in allRoadPoints:
        #         file.write(f"{point[0]}, {point[1]}\n")

        # 横向控制开始，弧度制，左转为正，右转为负
        distanceFromChoosePoint=[]
        for choosePoint in allRoadPoints:
            distanceFromChoosePoint.append(((ego[0]-choosePoint[0])**2 + (ego[1]-choosePoint[1])**2) ** 0.5)
        minDistance = min(distanceFromChoosePoint)
        minIndex = distanceFromChoosePoint.index(minDistance)
        previewPointIndex = minIndex
        pre_distance = 3
        ## # print("minIndex minDistance", minIndex, minDistance)
        for i in range(minIndex,len(allRoadPoints)):
            previewPointIndex = i
            choosePoint = allRoadPoints[i]
            distance = ((ego[0]-choosePoint[0])**2 + (ego[1]-choosePoint[1])**2) ** 0.5
            if distance>pre_distance:
                break

        # previewPointIndex = min(minIndex + 10 + int(ego[2]), len(distanceFromChoosePoint)-1)
        previewPoint = allRoadPoints[previewPointIndex]
    
        anglePreview = self.ktq_calArctan(previewPoint[0]-ego[0],previewPoint[1]-ego[1])
        ## # print("anglePreview is ", anglePreview)
        angleDiff = anglePreview - ego[3]
        # ld = ((ego[0]-previewPoint[0])**2 + (ego[1]-previewPoint[1])**2) ** 0.5
        ld = pre_distance
        L = ego[4]*2
        angleDiff = np.arctan((2 * L * np.sin(angleDiff)) / ld)
        if angleDiff > np.pi:
            angleDiff = angleDiff - 2*np.pi
        elif angleDiff < -np.pi:
            angleDiff = angleDiff + 2*np.pi
        steer = angleDiff * 0.5
        # # print("steer == ", steer)
        laneChangeFlag = 0
        if np.abs(steer) > 0.01:
            laneChangeFlag = 1
        # 横向控制结束
        # 纵向控制开始

        acc, fIndex, steerTempFlag = self.ktq_deside_acc(state, rampFlag, laneChangeFlag)
        # print("acc:"+str(acc)+"******************************8")
        # 纵向控制结束


        # if steer > np.pi / 6:
        #     steer = np.pi / 6
        # elif steer < -np.pi / 6:
        #     steer = -np.pi / 6

      
        frame1 = pd.DataFrame()
        for key, value in observation['vehicle_info'].items():
            sub_frame = pd.DataFrame(value, columns=['x', 'y', 'v', 'yaw', 'length', 'width'], index=[key])
            frame1 = pd.concat([frame1, sub_frame])
        state1 = frame1.to_numpy()
        # a=1

        ego1 = state1[0, :]
        anglePreviewCheck = self.ktq_calArctan(np.mean(goalPointX) - ego1[0], np.mean(goalPointY) - ego1[1])
       
        angleDiff = anglePreviewCheck - ego1[3]
        # # print("angleDiff = ",angleDiff)
        if angleDiff > np.pi:
            angleDiff = angleDiff - 2 * np.pi
        elif angleDiff < -np.pi:
            angleDiff = angleDiff + 2 * np.pi
       
        return acc, steer, fIndex, ego1

# 这个是算角度的
    def ktq_calArctan(self, dx, dy):
        angle = np.arctan2(dy, dx)
        if angle < 0:
            angle += np.pi*2
        return angle

# laneID是a.0.-1.-1，所以roadID是a
    def ktq_getRoadIDFromLaneID(self, laneID):
        roadID = laneID.split('.')[0]
        return roadID

    def ktq_getAdjacentLanes(self, laneID, roadStructureDict):
        roadID, laneSegmentID = laneID.split(".")[:2]
        return [f"{roadID}.{laneSegmentID}.{laneInTemp}.-1" for laneInTemp in roadStructureDict[roadID][laneSegmentID]]

    def ktq_getEgoTrajectory(self, task_info, road_data):
        # print(road_data)
        goalPointX = [task_info['targetPos'][0][0], task_info['targetPos'][1][0]]
        goalPointY = [task_info['targetPos'][0][1], task_info['targetPos'][1][1]]
        # print(goalPointX,goalPointY)
        ego_vehicle_x = task_info['startPos'][0]
        ego_vehicle_y = task_info['startPos'][1]
        # print(ego_vehicle_x,ego_vehicle_y)
        
        # Find Ego Vehicle Lane
        # Find Goal Lane
        # EgoVehicleLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        # GoalLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        EgoVehicleLaneIndexDict = dict()
        GoalLaneIndexDict = dict()
        #EgoVehicleLaneDataList = list()
        #GoalLaneDataList = list()
        AllLanesDict = dict()
        
        for discrete_lane in road_data.discretelanes:
            AllLanesDict[discrete_lane.lane_id] = discrete_lane
            center_lane = discrete_lane.center_vertices
            ## # print(" Lane ID ", discrete_lane.lane_id, " x0 ", discrete_lane.center_vertices[0][0], " y0 ", discrete_lane.center_vertices[0][1])
            for point in center_lane:

                if ((point[0] - ego_vehicle_x) ** 2 + (point[1] - ego_vehicle_y) ** 2) ** 0.5 < 2.5:
                    EgoVehicleLaneIndexDict[discrete_lane.lane_id] = point
                    EgoVehicleLaneDataList = discrete_lane
                    break
                LaneWidth = 5
                if goalPointX[0] - 2 < point[0] < goalPointX[1] + 2 and goalPointY[0]-1.5 < point[1] < goalPointY[1]+1.5:
                    GoalLaneIndexDict[discrete_lane.lane_id] = point
                    GoalLaneDataList = discrete_lane
                    break
        ## # print(AllLanesDict)
        ## # print(EgoVehicleLaneDataList.successor)
        roadList = list()
        roadStructureDict = self.ktq_RoadStructure(road_data)
        ## # print("road structure   ", roadStructureDict)
        currentLane = EgoVehicleLaneDataList
        currentRoad = self.ktq_getRoadIDFromLaneID(currentLane.lane_id)
        ## # print("current lane ", currentLane.lane_id)
        goalLane = GoalLaneDataList
        goalRoad = self.ktq_getRoadIDFromLaneID(goalLane.lane_id)
        ## # print("goalLaneID ", goalLane.lane_id)
        expandableLanes = []
        expandableLanesNoDelete = []
        fatherOfLanes = dict()

        expandableLanes.extend(currentLane.successor)
        expandableLanesNoDelete.extend(currentLane.successor)
        for successorTemp in currentLane.successor:
            fatherOfLanes[successorTemp] = currentLane.lane_id

        adjacentLaneList = self.ktq_getAdjacentLanes(currentLane.lane_id, roadStructureDict)
        ## # print("adjacent lane list:  ", adjacentLaneList)
        rampFlag = 0
        for adjacentLaneID in adjacentLaneList:
            expandableLanes.append(adjacentLaneID)
            expandableLanesNoDelete.append(adjacentLaneID)
            fatherOfLanes[adjacentLaneID] = currentLane.lane_id

            #判断自车是否在匝道上或在匝道的邻道上
            endPointOfCurrentLane = AllLanesDict[currentLane.lane_id].center_vertices[len(AllLanesDict[currentLane.lane_id].center_vertices) - 1]
            endPointOfAdjacentLane = AllLanesDict[adjacentLaneID].center_vertices[len(AllLanesDict[adjacentLaneID].center_vertices) - 1]
            if ((endPointOfCurrentLane[0] - endPointOfAdjacentLane[0]) ** 2 + (endPointOfCurrentLane[1] - endPointOfAdjacentLane[1]) ** 2) ** 0.5 < 1:
                rampFlag = 1
        ## # print("expandableLanes:  ", expandableLanes)
        if(currentLane.lane_id == goalLane.lane_id):
            print("current lane is goal lane ")
        else:
            while True :

                successorLaneID = expandableLanes[0]
                aaa=1
                successorRoad = self.ktq_getRoadIDFromLaneID(successorLaneID)
                ## # print("successorRoad ", successorRoad)
                if successorLaneID == goalLane.lane_id:
                    break
                else:
                    if successorLaneID in AllLanesDict:
                        for nextSuccessorLaneID in AllLanesDict[successorLaneID].successor:
                            #nextSuccessorLane = AllLanesDict[nextSuccessorLaneID]

                            if nextSuccessorLaneID not in expandableLanesNoDelete:
                                expandableLanes.append(nextSuccessorLaneID)
                                expandableLanesNoDelete.append(nextSuccessorLaneID)
                                fatherOfLanes[nextSuccessorLaneID] = successorLaneID
                                ## # print("test flag 1", nextSuccessorLaneID)
                                adjacentLaneList = self.ktq_getAdjacentLanes(nextSuccessorLaneID, roadStructureDict)
                                ## # print(" 111 adjacent lane list:  ", adjacentLaneList)
                                for adjacentLaneID in adjacentLaneList:
                                    if adjacentLaneID not in expandableLanesNoDelete:
                                        expandableLanes.append(adjacentLaneID)
                                        expandableLanesNoDelete.append(adjacentLaneID)
                                        fatherOfLanes[adjacentLaneID] = nextSuccessorLaneID
                                ## # print("111 expandableLanes:  ", expandableLanes)
                    del expandableLanes[0]
                ## # print("flag.......")
            ## # print("fatherOfLanes ", fatherOfLanes)


            # ；开始回溯

            resultPathList = list()
            thisCheckLaneID = goalLane.lane_id
            resultPathList.append(thisCheckLaneID)

            while True:

                fatherLaneID = fatherOfLanes[thisCheckLaneID]
                resultPathList.append(fatherLaneID)

                if fatherLaneID == currentLane.lane_id:
                    break
                else:
                    thisCheckLaneID = fatherLaneID

            ## # print("resultPathList:  ", resultPathList)
            ## # print("1 resultPathList:  ", resultPathList[::-1])

            resultPathList = resultPathList[::-1]

            checkTemp = 0
            planedPathLength = len(resultPathList)

            while True:
                if planedPathLength > checkTemp + 1:
                    thisLaneID = resultPathList[checkTemp]
                    thisRoadID = self.ktq_getRoadIDFromLaneID(thisLaneID)
                    nextLaneID = resultPathList[checkTemp + 1]
                    nextRoadID = self.ktq_getRoadIDFromLaneID(nextLaneID)

                    if nextRoadID == thisRoadID and thisLaneID.split(".")[1] == nextLaneID.split(".")[1]:
                        del resultPathList[checkTemp]
                        planedPathLength = len(resultPathList)
                    else:
                        checkTemp = checkTemp + 1
                    a=1
                else:
                    break
        # print(AllLanesDict)
        # print(roadStructureDict)
        return resultPathList, AllLanesDict, rampFlag, roadStructureDict

    def ktq_getRampFlag(self, observation,  task_info, roadStructureDict, resultPathList, AllLanesDict):
        EgoVehicleLaneDataList = 0
        goalPointX = [task_info['targetPos'][0][0], task_info['targetPos'][1][0]]
        goalPointY = [task_info['targetPos'][0][1], task_info['targetPos'][1][1]]
        ego_vehicle_x = task_info['startPos'][0]
        ego_vehicle_y = task_info['startPos'][1]
        t1 = time.time()


        # Find Ego Vehicle Lane
        # Find Goal Lane
        # EgoVehicleLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        # GoalLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        EgoVehicleLaneIndexDict = dict()
        GoalLaneIndexDict = dict()
        #EgoVehicleLaneDataList = list()
        #GoalLaneDataList = list()
        
        for egoVehiclePath in resultPathList:
            discrete_lane = AllLanesDict[egoVehiclePath]
            center_lane = discrete_lane.center_vertices
            if discrete_lane.lane_id == resultPathList[len(resultPathList)-1]:
                for iii in range(500):
                    if observation['vehicle_info']['ego']['yaw'] < np.pi/2 or observation['vehicle_info']['ego']['yaw'] > np.pi/2*3:
                        center_lane = np.vstack([center_lane, [min(goalPointX) + iii / 2, np.average(goalPointY)]])
                    else:
                        center_lane = np.vstack([center_lane, [min(goalPointX) - iii / 2, np.average(goalPointY)]])
                    #center_lane.extend([min(goalPointX) + iii / 2, np.average(goalPointY)])

            ## # print(" Lane ID ", discrete_lane.lane_id, " x0 ", discrete_lane.center_vertices[0][0], " y0 ", discrete_lane.center_vertices[0][1])
            for point in center_lane:
                ## # print(" !!!! This point distance :     ", ((point[0] - ego_vehicle_x) ** 2 + (point[1] - ego_vehicle_y) ** 2) ** 0.5)
                if ((point[0] - ego_vehicle_x) ** 2 + (point[1] - ego_vehicle_y) ** 2) ** 0.5 < 20:
                    EgoVehicleLaneIndexDict[discrete_lane.lane_id] = point
                    EgoVehicleLaneDataList = discrete_lane
                    break
        
        ## # print(AllLanesDict)
        ## # print(EgoVehicleLaneDataList.successor)
        roadList = list()

        ## # print("road structure   ", roadStructureDict)

        rampFlag = 0

        if EgoVehicleLaneDataList ==0:
            return 0,0,2

        #判断当前车道和邻车道终点距离
        currentLane = EgoVehicleLaneDataList
        ## # print("current lane ", currentLane.lane_id)
        adjacentLaneList = self.ktq_getAdjacentLanes(currentLane.lane_id, roadStructureDict)
        ## # print("adjacent lane list:  ", adjacentLaneList)
        for adjacentLaneID in adjacentLaneList:

            #判断自车是否在匝道上或在匝道的邻道上
            endPointOfCurrentLane = AllLanesDict[currentLane.lane_id].center_vertices[len(AllLanesDict[currentLane.lane_id].center_vertices) - 1]
            endPointOfAdjacentLane = AllLanesDict[adjacentLaneID].center_vertices[len(AllLanesDict[adjacentLaneID].center_vertices) - 1]
            if 0.1 < ((endPointOfCurrentLane[0] - endPointOfAdjacentLane[0]) ** 2 + (endPointOfCurrentLane[1] - endPointOfAdjacentLane[1]) ** 2) ** 0.5 < 2:
                rampFlag = 1
        if len(currentLane.successor):
            #判断下一段车道和其林车道终点距离
            nextLaneID = currentLane.successor[0]
            if nextLaneID.split('.')[2] != 'None':
                ## # print("nextLaneID ", nextLaneID)
                adjacentLaneList = self.ktq_getAdjacentLanes(nextLaneID, roadStructureDict)
                ## # print("adjacent lane list:  ", adjacentLaneList)
                for adjacentLaneID in adjacentLaneList:

                    # 判断自车是否在匝道上或在匝道的邻道上
                    if nextLaneID in AllLanesDict.keys():
                        endPointOfCurrentLane = AllLanesDict[nextLaneID].center_vertices[len(AllLanesDict[nextLaneID].center_vertices) - 1]
                        endPointOfAdjacentLane = AllLanesDict[adjacentLaneID].center_vertices[len(AllLanesDict[adjacentLaneID].center_vertices) - 1]
                if 0.1 < ((endPointOfCurrentLane[0] - endPointOfAdjacentLane[0]) ** 2 + (endPointOfCurrentLane[1] - endPointOfAdjacentLane[1]) ** 2) ** 0.5 < 2:
                    rampFlag = 1

                # 判断再下一段车道和其林车道终点距离
                if currentLane.successor[0] in AllLanesDict.keys():
                    if len(AllLanesDict[currentLane.successor[0]].successor):
                        nextLaneID = AllLanesDict[currentLane.successor[0]].successor[0]
                        if nextLaneID.split('.')[2] != 'None':
                            ## # print("nextLaneID ", nextLaneID)
                            adjacentLaneList = self.ktq_getAdjacentLanes(nextLaneID, roadStructureDict)
                            ## # print("adjacent lane list:  ", adjacentLaneList)
                            for adjacentLaneID in adjacentLaneList:

                                # 判断自车是否在匝道上或在匝道的邻道上
                                endPointOfCurrentLane = AllLanesDict[nextLaneID].center_vertices[
                                    len(AllLanesDict[nextLaneID].center_vertices) - 1]
                                endPointOfAdjacentLane = AllLanesDict[adjacentLaneID].center_vertices[
                                    len(AllLanesDict[adjacentLaneID].center_vertices) - 1]
                                if 0.1 < ((endPointOfCurrentLane[0] - endPointOfAdjacentLane[0]) ** 2 + (endPointOfCurrentLane[1] - endPointOfAdjacentLane[1]) ** 2) ** 0.5 < 2:
                                    rampFlag = 1

            ## # print("#######  time 4  @@@@@@@@@   ", time.time() - t1)


        return 0, 0, rampFlag

    def ktq_RoadStructure(self, road_data):
        road_structure = {}
        for discrete_lane in road_data.discretelanes:
            road, laneSegment, lane = discrete_lane.lane_id.split('.')[:3]
            if road_structure.get(road, False):
                if road_structure[road].get(laneSegment, False):
                    road_structure[road][laneSegment].append(int(lane))
                else:
                    road_structure[road][laneSegment] =[int(lane)]
            else:
                road_structure[road]={}
                road_structure[road][laneSegment] = [int(lane)]
        return road_structure
    
    def ktq_getMapInfo(self, observation, scenario: dict):
        goalPointX = observation['test_setting']['goal']['x']
        goalPointY = observation['test_setting']['goal']['y']
        ego_vehicle_x = observation['vehicle_info']['ego']['x']
        ego_vehicle_y = observation['vehicle_info']['ego']['y']
        observation, traj = self.controller.init(scenario)
        road_data = observation.road_info


        return road_data.discretelanes

    def ktq_deside_acc(self, state, rampFlag, laneChangeFlag):
        #前方车辆的速度 fv 和与前车的距离 dis_gap 。fIndex 表示前方车辆的索引
        v, fv, dis_gap,direction,fIndex = self.ktq_getInformFront(state, rampFlag, laneChangeFlag)
        if dis_gap == -10086 or fIndex == 0:
            # print("fIndex=0")
            self.exv = 20
            a_idm = (self.exv - v) * 10 # 4很重要
            #a_idm = -20
        elif dis_gap < 3.0:  # 3很重要
            a_idm = -20
        else:
            # self.exv = fv
            # a_idm = (self.exv - v) * 0.45
            self.exv = 10

            self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
                v - fv) / 2 / (self.a * self.b) ** 0.5
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))

            desire_gap = 10 + v*v/2/15/2
            desire_gap = 10
            if fv > self.exv:
                a_idm = (desire_gap-dis_gap) * -4 + (fv - v) * 5
                a_idm = np.clip(a_idm, -20, 20)
            else:
                desire_gap = 5
                a_idm = (desire_gap - dis_gap) * -1 + (fv - v) * 5
                a_idm = np.clip(a_idm, -20, 3)
        # 对加速度进行约束
        if v == 0 and a_idm < -3:
            a_idm = -1
        steerTempFlag = 0
        if a_idm < -1 and laneChangeFlag == 1:
            steerTempFlag = 1
        if v > 50 and a_idm > 0:
            # a_idm = 0
            aaaa = 1
        return a_idm,fIndex, steerTempFlag

    def transform_to_ego_coordinate(self,state):
        """
        将其他车辆的全局坐标转换为自车坐标系下的坐标

        参数：
        state (np.array): 包含车辆状态的矩阵，每行代表一个车辆的状态，每行应包含x、y、v、yaw、height、width

        返回值：
        np.array: 包含转换后的其他车辆相对于自车的相对坐标的矩阵，每行代表一个车辆的相对坐标
        """
        ego_x, ego_y, _, ego_yaw, _, _ = state[0]  # 提取自车的信息
        relative_coordinates = state[:, :2] - np.array([ego_x, ego_y])  # 计算所有车辆相对于自车的相对坐标
        # 将相对坐标转换为自车坐标系
        rotation_matrix = np.array([
            [np.cos(ego_yaw), np.sin(ego_yaw)],
            [-np.sin(ego_yaw), np.cos(ego_yaw)]
        ])
        relative_coordinates_rotated = np.dot(relative_coordinates, rotation_matrix.T)

        return relative_coordinates_rotated

    def check_vehicles_in_front(self,state,relative_coordinates, angle_range_deg, distance_range_m):
        
        vehicles_in_front = []
        vehicles_in_far_front = []
        for i in range(len(relative_coordinates)):
            is_front = (relative_coordinates[i][0]>=-5 and relative_coordinates[i][0]<=40 and 
                        relative_coordinates[i][1]>=-15 and relative_coordinates[i][1]<=15 and 
                        abs(state[i][3]-state[0][3])<np.pi/2)
            is_far_front = (relative_coordinates[i][0]>=-10 and relative_coordinates[i][0]<=10 and 
                        relative_coordinates[i][1]>=20 and relative_coordinates[i][1]<=40 and 
                        abs(state[i][3]-state[0][3])<np.pi/2)
            
            vehicles_in_front.append(is_front)
            vehicles_in_far_front.append(is_far_front)
        vehicles_in_front[0] = False
        vehicles_in_far_front[0] = False
       
        return np.array(vehicles_in_front),np.array(vehicles_in_far_front)

    def ktq_getInformFront(self,state, rampFlag, laneChangeFlag):
        # print(state)
        # if state[0, 3] < np.pi / 2 or state[0, 3] > np.pi * 3 / 2:
        #     direction = 1.0
        # else:
        #     direction = -1.0
        # state[:,0] = state[:,0]*direction
     
        ego = state[0,:]
        v, fv, dis_gap = ego[2], -1, -10086
        
        fov = 90
        dangerous_distance = 10
        relative_coordinates = self.transform_to_ego_coordinate(state)
        vehicle_front,vehicle_far_front = self.check_vehicles_in_front(state,relative_coordinates,fov,dangerous_distance)
        # ind[0]=False
        # print("ind:"+str(ind)+"AAAAAAAAAAAAAAAAAAAAAaaaaa")
        # safeLength1 = 7
        # safeLength2 = 8
        # safeLength3 = 9
        # safePoint1 = [ego[0] + safeLength1*np.cos(ego[3]),ego[1]+safeLength1*np.sin(ego[3])]
        # safePoint2 = [ego[0] + safeLength2 * np.cos(ego[3]), ego[1] + safeLength2 * np.sin(ego[3])]
        # safePoint3 = [ego[0] + safeLength3 * np.cos(ego[3]), ego[1] + safeLength3 * np.sin(ego[3])]


        fIndex = 0
  
        # if ego[0]>127:
        #     aa=1
        if vehicle_front.sum() > 0:
            for i in range(1,len(vehicle_front)):
                if vehicle_front[i]:
                    if dis_gap == -10086:
                        dis_gap = relative_coordinates[i][0] - (state[i][4] + state[0][4])/2
                        fv = state[i][2]
                        fIndex = i
                    elif relative_coordinates[i][0] - (state[i][4] + state[0][4])/2 < dis_gap:
                        dis_gap = relative_coordinates[i][0] - (state[i][4] + state[0][4])/2
                        fv = state[i][2]
                        fIndex = i
                  

            # # print(front[0], ego[0], front[1], ego[1])
        if dis_gap > 100:
            dis_gap = -10086
            fv = -1
        return v, fv, dis_gap, vehicle_far_front.sum() > 0, fIndex
