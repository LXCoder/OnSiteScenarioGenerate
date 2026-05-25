#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import lib
import numpy as np
import pandas as pd
import math
import sys
#import utils
import time



class CMCControl():
    def __init__(self, a_bound=3.0, exv=40, t=1, a=3, b=3, gama=4, s0=2.0, s1=2.0, PreviewDistance=3,
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

    def cmc_act(self, observation, task_info, planedLanes, AllLanesDict, rampFlag):
        frame = pd.DataFrame()
        for key, value in observation['vehicle_info'].items():
                sub_frame = pd.DataFrame(value,columns=['x', 'y', 'v', 'yaw', 'length', 'width'],index=[key])
                frame = pd.concat([frame, sub_frame])

        state = frame.to_numpy()
        #a=1

        ego = state[0, :]
        #print("ego state: ", ego)
        goalPointX = [task_info['targetPos'][0][0], task_info['targetPos'][1][0]]
        goalPointY = [task_info['targetPos'][0][1], task_info['targetPos'][1][1]]
        allRoadPoints = []
        lastPoint = []
        for planedLaneID in planedLanes:
            centerLine=AllLanesDict[planedLaneID].center_vertices
            for point in centerLine:
                if ((point[0]-min(goalPointX))**2 + (point[1]-np.average(goalPointY))**2) ** 0.5 < 20:
                    break
                allRoadPoints.append([point[0], point[1]])
                lastPoint.append(point)
        lastBut10Point = lastPoint[len(lastPoint)-10]
        for iii in range(1000):
            if ego[3] < np.pi/2 or ego[3] > np.pi*3/2:
                allRoadPoints.append([np.mean(goalPointX)+iii/2, np.average(goalPointY) + iii/2 * (np.average(goalPointY) - lastBut10Point[1])/(np.average(goalPointX) - lastBut10Point[0])])
            else:
                allRoadPoints.append([np.mean(goalPointX) - iii / 2, np.average(goalPointY) - iii / 2 * (np.average(goalPointY) - lastBut10Point[1])/(np.average(goalPointX) - lastBut10Point[0])])
        #print("allRoadPoints:  ", allRoadPoints)

        # 横向控制开始，弧度制，左转为正，右转为负
        distanceFromChoosePoint=[]
        for choosePoint in allRoadPoints:
            distanceFromChoosePoint.append(((ego[0]-choosePoint[0])**2 + (ego[1]-choosePoint[1])**2) ** 0.5)
        minDistance = min(distanceFromChoosePoint)
        minIndex = distanceFromChoosePoint.index(minDistance)
        #print("minIndex minDistance", minIndex, minDistance)

        previewPointIndex = min(minIndex + 10 + int(ego[2]), len(distanceFromChoosePoint)-1)
        previewPoint = allRoadPoints[previewPointIndex]
        #print("previewPoint is ", previewPoint)

        anglePreview = self.cmc_calArctan(previewPoint[0]-ego[0],previewPoint[1]-ego[1])
        #print("anglePreview is ", anglePreview)
        angleDiff = anglePreview - ego[3]
        if angleDiff > np.pi:
            angleDiff = angleDiff - 2*np.pi
        elif angleDiff < -np.pi:
            angleDiff = angleDiff + 2*np.pi
        steer = angleDiff * 0.5
        print("steer == ", steer)
        laneChangeFlag = 0
        if np.abs(steer) > 0.01:
            laneChangeFlag = 1
        # 横向控制结束

        # 纵向控制开始

        acc, fIndex, steerTempFlag = self.cmc_deside_acc(state, rampFlag, laneChangeFlag)

        # 纵向控制结束



        if steer > np.pi / 6:
            steer = np.pi / 6
        elif steer < -np.pi / 6:
            steer = -np.pi / 6

        # if ((ego[0]-allRoadPoints[len(allRoadPoints)-1][0])**2 + (ego[1]-allRoadPoints[len(allRoadPoints)-1][1])**2) ** 0.5 < 95:
        #     acc = -15
        #     if ego[2] == 0 and acc < -3:
        #         acc = -1
        frame1 = pd.DataFrame()
        for key, value in observation['vehicle_info'].items():
            sub_frame = pd.DataFrame(value, columns=['x', 'y', 'v', 'yaw', 'length', 'width'], index=[key])
            frame1 = pd.concat([frame1, sub_frame])
        state1 = frame1.to_numpy()
        # a=1

        ego1 = state1[0, :]
        anglePreviewCheck = self.cmc_calArctan(np.mean(goalPointX) - ego1[0], np.mean(goalPointY) - ego1[1])
        print("datas are:  ", np.mean(goalPointX), ego1[0], np.mean(goalPointY), ego1[1])
        print("anglePreviewCheck = ", anglePreviewCheck)
        print("ego[3] = ", ego1[3])
        angleDiff = anglePreviewCheck - ego1[3]
        print("angleDiff = ",angleDiff)
        if angleDiff > np.pi:
            angleDiff = angleDiff - 2 * np.pi
        elif angleDiff < -np.pi:
            angleDiff = angleDiff + 2 * np.pi
        print("angleDiff = ", angleDiff)

        if np.abs(angleDiff)>np.pi/2 and ((ego1[0]-np.mean(goalPointX))**2 + (ego1[1]-np.mean(goalPointY))**2) ** 0.5 >40:
            steer = 0
            if ego1[2] > 30 and acc > 0:
                acc = 0
            print("steer = 0 !!!")

        if steerTempFlag == 1:
            #steer = 0
            if ego1[2] > 30 and acc > 0:
                #acc = 0
                aaaa = 1
            print("steer = 0 !!!")
        if rampFlag == 2:
            steer = 0
            if ego1[2] > 0:
                acc = -15
            else:
                acc = -1
        print("final command: ", acc, steer)
        return acc, steer, fIndex, ego1

    def cmc_calArctan(self, dx, dy):
        angle = 0
        if dx > 0 and dy > 0:
            angle = np.arctan(dy/dx)
        elif dx < 0 and dy > 0:
            angle = np.pi + np.arctan(dy/dx)
        elif dx < 0 and dy < 0:
            angle = np.arctan(dy / dx) + np.pi
        elif dx >0 and dy <0:
            angle = np.pi*2 + np.arctan(dy / dx)
        elif dx == 0 and dy > 0:
            angle = np.pi / 2
        elif dx == 0 and dy < 0:
            angle = np.pi /2 * 3
        elif dx < 0 and dy == 0:
            angle = np.pi
        elif dx > 0 and dy == 0:
            angle = 0
        else:
            angle = 0
        # if angle > np.pi:
        #     angle = angle - 2 * np.pi
        # elif angle < -np.pi:
        #     angle = angle + 2 * np.pi
        return angle

    def cmc_getRoadIDFromLaneID(self, laneID):
        roadID=[]
        countTemp = 0
        for charactor in laneID:
            if charactor == '.':
                break
            countTemp = countTemp + 1
        roadID = laneID[0:countTemp]

        # temp = laneID.split(".")
        # roadID = temp[0]
        # laneSegment = temp[1]
        return roadID


    def cmc_getAdjacentLanes(self, laneID, roadStructureDict):
        laneList = []
        roadID = self.cmc_getRoadIDFromLaneID(laneID)
        laneSegmentID = laneID.split(".")[1]
        for laneInTemp in roadStructureDict[roadID][laneSegmentID]:
            laneList.append(roadID + '.' + laneSegmentID + '.' + str(laneInTemp) + '.-1')
        return laneList

    def cmc_getEgoTrajectory(self, task_info, road_data):
        print(type(task_info['targetPos']),task_info['targetPos'])
        goalPointX = [task_info['targetPos'][0][0], task_info['targetPos'][1][0]]
        goalPointY = [task_info['targetPos'][0][1], task_info['targetPos'][1][1]]
        ego_vehicle_x = task_info['startPos'][0]
        ego_vehicle_y = task_info['startPos'][1]

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
            #print(" Lane ID ", discrete_lane.lane_id, " x0 ", discrete_lane.center_vertices[0][0], " y0 ", discrete_lane.center_vertices[0][1])
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
        #print(AllLanesDict)
        #print(EgoVehicleLaneDataList.successor)
        roadList = list()
        roadStructureDict = self.cmc_RoadStructure(road_data)
        #print("road structure   ", roadStructureDict)
        currentLane = EgoVehicleLaneDataList
        currentRoad = self.cmc_getRoadIDFromLaneID(currentLane.lane_id)
        #print("current lane ", currentLane.lane_id)
        goalLane = GoalLaneDataList
        goalRoad = self.cmc_getRoadIDFromLaneID(goalLane.lane_id)
        #print("goalLaneID ", goalLane.lane_id)
        expandableLanes = []
        expandableLanesNoDelete = []
        fatherOfLanes = dict()

        expandableLanes.extend(currentLane.successor)
        expandableLanesNoDelete.extend(currentLane.successor)
        for successorTemp in currentLane.successor:
            fatherOfLanes[successorTemp] = currentLane.lane_id

        adjacentLaneList = self.cmc_getAdjacentLanes(currentLane.lane_id, roadStructureDict)
        #print("adjacent lane list:  ", adjacentLaneList)
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
        #print("expandableLanes:  ", expandableLanes)
        if(currentLane.lane_id == goalLane.lane_id):
            print("current lane is goal lane ")
        else:
            while True :

                successorLaneID = expandableLanes[0]
                aaa=1
                successorRoad = self.cmc_getRoadIDFromLaneID(successorLaneID)
                #print("successorRoad ", successorRoad)
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
                                #print("test flag 1", nextSuccessorLaneID)
                                adjacentLaneList = self.cmc_getAdjacentLanes(nextSuccessorLaneID, roadStructureDict)
                                #print(" 111 adjacent lane list:  ", adjacentLaneList)
                                for adjacentLaneID in adjacentLaneList:
                                    if adjacentLaneID not in expandableLanesNoDelete:
                                        expandableLanes.append(adjacentLaneID)
                                        expandableLanesNoDelete.append(adjacentLaneID)
                                        fatherOfLanes[adjacentLaneID] = nextSuccessorLaneID
                                #print("111 expandableLanes:  ", expandableLanes)
                    del expandableLanes[0]
                #print("flag.......")
            #print("fatherOfLanes ", fatherOfLanes)


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

            #print("resultPathList:  ", resultPathList)
            #print("1 resultPathList:  ", resultPathList[::-1])

            resultPathList = resultPathList[::-1]

            checkTemp = 0
            planedPathLength = len(resultPathList)

            while True:
                if planedPathLength > checkTemp + 1:
                    thisLaneID = resultPathList[checkTemp]
                    thisRoadID = self.cmc_getRoadIDFromLaneID(thisLaneID)
                    nextLaneID = resultPathList[checkTemp + 1]
                    nextRoadID = self.cmc_getRoadIDFromLaneID(nextLaneID)

                    if nextRoadID == thisRoadID and thisLaneID.split(".")[1] == nextLaneID.split(".")[1]:
                        del resultPathList[checkTemp]
                        planedPathLength = len(resultPathList)
                    else:
                        checkTemp = checkTemp + 1
                    a=1
                else:
                    break

        return resultPathList, AllLanesDict, rampFlag, roadStructureDict

    def cmc_getRampFlag(self, observation,  task_info, roadStructureDict, resultPathList, AllLanesDict):
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

            #print(" Lane ID ", discrete_lane.lane_id, " x0 ", discrete_lane.center_vertices[0][0], " y0 ", discrete_lane.center_vertices[0][1])
            for point in center_lane:
                #print(" !!!! This point distance :     ", ((point[0] - ego_vehicle_x) ** 2 + (point[1] - ego_vehicle_y) ** 2) ** 0.5)
                if ((point[0] - ego_vehicle_x) ** 2 + (point[1] - ego_vehicle_y) ** 2) ** 0.5 < 20:
                    EgoVehicleLaneIndexDict[discrete_lane.lane_id] = point
                    EgoVehicleLaneDataList = discrete_lane
                    break

        #print(AllLanesDict)
        #print(EgoVehicleLaneDataList.successor)
        roadList = list()

        #print("road structure   ", roadStructureDict)

        rampFlag = 0

        if EgoVehicleLaneDataList ==0:
            return 0,0,2

        #判断当前车道和邻车道终点距离
        currentLane = EgoVehicleLaneDataList
        #print("current lane ", currentLane.lane_id)
        adjacentLaneList = self.cmc_getAdjacentLanes(currentLane.lane_id, roadStructureDict)
        #print("adjacent lane list:  ", adjacentLaneList)
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
                #print("nextLaneID ", nextLaneID)
                adjacentLaneList = self.cmc_getAdjacentLanes(nextLaneID, roadStructureDict)
                #print("adjacent lane list:  ", adjacentLaneList)
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
                            #print("nextLaneID ", nextLaneID)
                            adjacentLaneList = self.cmc_getAdjacentLanes(nextLaneID, roadStructureDict)
                            #print("adjacent lane list:  ", adjacentLaneList)
                            for adjacentLaneID in adjacentLaneList:

                                # 判断自车是否在匝道上或在匝道的邻道上
                                endPointOfCurrentLane = AllLanesDict[nextLaneID].center_vertices[
                                    len(AllLanesDict[nextLaneID].center_vertices) - 1]
                                endPointOfAdjacentLane = AllLanesDict[adjacentLaneID].center_vertices[
                                    len(AllLanesDict[adjacentLaneID].center_vertices) - 1]
                                if 0.1 < ((endPointOfCurrentLane[0] - endPointOfAdjacentLane[0]) ** 2 + (endPointOfCurrentLane[1] - endPointOfAdjacentLane[1]) ** 2) ** 0.5 < 2:
                                    rampFlag = 1

            #print("#######  time 4  @@@@@@@@@   ", time.time() - t1)


        return 0, 0, rampFlag

    def cmc_RoadStructure(self, road_data):
        road_structure = {}
        for discrete_lane in road_data.discretelanes:
            road = (discrete_lane.lane_id).split('.')[0]
            laneSegment = (discrete_lane.lane_id).split('.')[1]
            lane = int((discrete_lane.lane_id).split('.')[2])
            if road_structure.get(road, False):
                if road_structure[road].get(laneSegment, False):
                    road_structure[road][laneSegment].append(lane)
                else:
                    road_structure[road][laneSegment] =[lane]
            else:
                road_structure[road]={}
                road_structure[road][laneSegment] = [lane]

        return road_structure
    def cmc_getMapInfo(self, observation, scenario: dict):
        goalPointX = observation['test_setting']['goal']['x']
        goalPointY = observation['test_setting']['goal']['y']
        ego_vehicle_x = observation['vehicle_info']['ego']['x']
        ego_vehicle_y = observation['vehicle_info']['ego']['y']
        observation, traj = self.controller.init(scenario)
        road_data = observation.road_info


        return road_data.discretelanes


    def cmc_deside_acc(self, state, rampFlag, laneChangeFlag):
        v, fv, dis_gap,direction,fIndex = self.cmc_getInformFront(state, rampFlag, laneChangeFlag)
        # print(v, fv, dis_gap,direction)
        # print(state)
        print(v, fv, dis_gap)
        if dis_gap == -10086 or fIndex == 0:
            a_idm = (self.exv - v) * 5
            #a_idm = -20
        elif dis_gap < 0:
            a_idm = -20
        else:
            # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.t)
            self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (
                v - fv) / 2 / (self.a * self.b) ** 0.5
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((self.s_ / (dis_gap+1e-6)) ** 2))

            desire_gap = 10 + v*v/2/15/2
            desire_gap = 10
            if fv > 10:
                a_idm = (desire_gap-dis_gap) * -4 + (fv - v) * 5
                a_idm = np.clip(a_idm, -20, 20)
            else:
                desire_gap = 5
                a_idm = (desire_gap - dis_gap) * -1 + (fv - v) * 5
                a_idm = np.clip(a_idm, -20, 3)

        # 对加速度进行约束
        if v == 0 and a_idm < -3:
            a_idm = -1
            print('!!!!')
        # print(v,fv,dis_gap,a_idm,self.s_)
        # print(state,v,fv,dis_gap,a_idm)
        steerTempFlag = 0
        if a_idm < -1 and laneChangeFlag == 1:
            steerTempFlag = 1
        if v > 50 and a_idm > 0:
            # a_idm = 0
            aaaa = 1
        return a_idm,fIndex, steerTempFlag

    def cmc_getInformFront(self,state, rampFlag, laneChangeFlag):
        # direction = np.sign(state[0,2])
        if state[0, 3] < np.pi / 2 or state[0, 3] > np.pi * 3 / 2:
            direction = 1.0
        else:
            direction = -1.0
        state[:,0] = state[:,0]*direction
        # state[:,2] = state[:,2]*direction
        ego = state[0,:]
        v, fv, dis_gap = ego[2], -1, -10086

        lateralGap = 1.5
        backGap = 6
        if rampFlag == 1:
            lateralGap = 1.5
            # backGap = 6

        if laneChangeFlag == 1:  
            #lateralGap = 2
            backGap = 6
        
        # 在本车前侧
        x_ind = ego[0] < state[:, 0] #自车正前方
        x_ind2 = ((ego[0]-backGap) < state[:,0]) & ((ego[0]+50) > state[:,0])        #自车左方
        x_ind3 = ((ego[0]-backGap) < state[:,0]) & ((ego[0]+50) > state[:,0])        #自车右方

        yawFlag = ego[3]
        print("yawFlag   ", yawFlag)
        if yawFlag > np.pi:
            yawFlag = yawFlag - np.pi * 2
        elif yawFlag < -np.pi:
            yawFlag = yawFlag + np.pi * 2


        y_ind_0 = (np.abs(ego[1] - state[:,1])) * math.cos(yawFlag - np.pi) < ((ego[5] + state[:,5])*1/2) + 0.3*(np.abs(ego[0] - state[:,0])) * math.cos(yawFlag - np.pi)
        y_ind_1 = (np.abs(ego[0] - state[:,0])) * math.cos(yawFlag - np.pi) < ((ego[4] + state[:,4])*3/5) + backGap*1.6
        # y_ind_2 = (np.abs(ego[1] - state[:,1])) * math.cos(yawFlag - np.pi) > ((ego[5] + state[:,5])*3/5)
        y_ind = y_ind_0 & y_ind_1

        yawSafeGap = 0
        if np.abs(yawFlag) > 1:
            yawSafeGap = yawSafeGap + np.abs(yawFlag)*2
        yawSafeGap = 0

        print("yawSafeGap= ", yawSafeGap)
        y_ind2 = (ego[5]/2 + state[:,5]/2 < (state[:, 1] - ego[1]) * math.cos(yawFlag - np.pi)) \
            & ((state[:, 1] - ego[1]) * math.cos(yawFlag - np.pi) < ego[5]/2 + state[:,5]/2 +lateralGap + yawSafeGap)
        y_ind3 = (ego[5]/2 + state[:,5]/2 < (-state[:, 1] + ego[1]) * math.cos(yawFlag - np.pi)) \
            & ((-state[:, 1] + ego[1]) * math.cos(yawFlag - np.pi) < ego[5]/2 + state[:,5]/2 +lateralGap + yawSafeGap)

        ind = (x_ind & y_ind) | (x_ind2 & y_ind2) | (x_ind3 & y_ind3)
        ind[0]=False

        safeLength1 = 7
        safeLength2 = 8
        safeLength3 = 9
        safePoint1 = [ego[0] + safeLength1*np.cos(ego[3]),ego[1]+safeLength1*np.sin(ego[3])]
        safePoint2 = [ego[0] + safeLength2 * np.cos(ego[3]), ego[1] + safeLength2 * np.sin(ego[3])]
        safePoint3 = [ego[0] + safeLength3 * np.cos(ego[3]), ego[1] + safeLength3 * np.sin(ego[3])]



        fIndex = 0
        if ego[0]>127:
            aa=1
        if ind.sum() > 0:
            state_ind = state[ind,:]
            front = state_ind[(state_ind[:,0]-ego[0]).argmin(),:]
            # print(front)
            fv = front[2]
            #print("state [:,2]", state[:,2])
            fIndex = np.where(state[:,2]==fv)[0][0]

            dis_gap = front[0] - ego[0] - (ego[4] + front[4])/2
            print(front[0], ego[0], front[1], ego[1])
        if dis_gap > 100:
            dis_gap = -10086
            fv = -1
        return v, fv, dis_gap, direction, fIndex
