import numpy as np
import pandas as pd
import math


class zzh:
    def __init__(self, a_bound=3.0, exv=40, t=2, a=3, b=3, gama=4, s0=1.0, s1=1.2):
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
        self.exv = exv
        self.t = t
        self.a = a
        self.b = b
        self.gama = gama
        self.s0 = s0
        self.s1 = s1

    
    def getEgoOrder_highway(self, task_info, road_data):
        goalPointX = (task_info['targetPos'][0][0] + task_info['targetPos'][1][0]) / 2.0
        goalPointY = (task_info['targetPos'][0][1] + task_info['targetPos'][1][1]) / 2.0
        ego_vehicle_x = task_info['startPos'][0]
        ego_vehicle_y = task_info['startPos'][1]
        self.goalX = goalPointX
        self.goalY = goalPointY
        # Find Ego Vehicle Lane
        # Find Goal Lane
        # EgoVehicleLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        # GoalLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        for discrete_lane in road_data.discretelanes:
            center_lane = discrete_lane.center_vertices
            if self.locate_highway(center_lane, ego_vehicle_x, ego_vehicle_y, 2.5):
                EgoVehicleLaneData = discrete_lane

            if self.locate_highway(center_lane, goalPointX, goalPointY, 2.5, False):
                GoalLaneData = discrete_lane
            
        bias = - GoalLaneData.center_vertices[self.locate_index][1] + goalPointY
        if abs(bias) < 0.5:
            bias = 0
        
        if ego_vehicle_x > goalPointX:
            direction = np.array([-1, 0])
        else:
            direction = np.array([1, 0])

        d_lane = abs(int(EgoVehicleLaneData.lane_id.split('.')[2])) - abs(int(GoalLaneData.lane_id.split('.')[2]))
        if d_lane < 0:
            to_order = 'ToRight'
        elif d_lane > 0:
            to_order = 'ToLeft'
        else:
            to_order = 'NoChange'
        
        return to_order, abs(d_lane), direction, int(EgoVehicleLaneData.lane_id.split('.')[2]), bias
    

    def getStateInit_highway(self, observation, lane_id):
        EgoVehicleInfo = pd.DataFrame()

        EgoVehicleInfo = pd.DataFrame(
            observation['vehicle_info']['ego'],
            columns=['x', 'y', 'v', 'yaw', 'length', 'width', 'VehicleStatus', 'lane_id'], 
            index=['ego']
        )

        EgoVehicleInfo.at['ego', 'VehicleStatus'] = 'straight'
        EgoVehicleInfo.at['ego', 'lane_id'] = lane_id
        return EgoVehicleInfo

    def getDecision_highway(self, EgoVehicleInfo, observation, to_order, num_change, direction, road_data, bias):
        ego_vehicle_x = observation['vehicle_info']['ego']['x']
        ego_vehicle_y = observation['vehicle_info']['ego']['y']
        ego_vehicle_v = observation['vehicle_info']['ego']['v']
        ego_vehicle_l = observation['vehicle_info']['ego']['length']
        now_lane_id = EgoVehicleInfo.at['ego', 'lane_id']
        ego_x = ego_vehicle_x*direction[0]
        ego_y = ego_vehicle_y*direction[0]
        now_lane = self.GetLaneData(road_data, now_lane_id)

        VehicleInfo = pd.DataFrame()
        for key, value in observation['vehicle_info'].items():
            sub_frame = pd.DataFrame(value, columns=['x', 'y', 'v', 'yaw',
                                                'length', 'width', 'VehicleStatus', 'lane_id'], index=[key])
            VehicleInfo = pd.concat([VehicleInfo, sub_frame])
        
        '''
        for index, OneVehicleData in VehicleInfo.iterrows():
            VehicleInfo.at[index, 'x'] = observation['vehicle_info'][index]['x']
            VehicleInfo.at[index, 'y'] = observation['vehicle_info'][index]['y']
            VehicleInfo.at[index, 'v'] = observation['vehicle_info'][index]['v']
            VehicleInfo.at[index, 'yaw'] = observation['vehicle_info'][index]['yaw']
        '''    

        if EgoVehicleInfo.at['ego', 'VehicleStatus'] == 'turning':
            goal_lane_id = self.GetGoalLane(to_order, now_lane_id, direction)
            goal_lane = self.GetLaneData(road_data, goal_lane_id)        

            if self.locate_highway(goal_lane, ego_vehicle_x, ego_vehicle_y-bias, 2):
                EgoVehicleInfo.at['ego', 'VehicleStatus'] = 'straight'
                EgoVehicleInfo.at['ego', 'lane_id'] = goal_lane_id
            else:
                goal_X, goal_Y = self.GetPreview(goal_lane, ego_vehicle_x, ego_vehicle_y, direction, bias)
                return np.array([goal_X - ego_vehicle_x, goal_Y - ego_vehicle_y + bias]), EgoVehicleInfo, num_change, VehicleInfo

        if EgoVehicleInfo.at['ego', 'VehicleStatus'] == 'straight':
            flag = 1
            for index, OneVehicleData in VehicleInfo.iterrows():
                if index == 'ego':
                    continue
                veh_x = VehicleInfo.at[index, 'x']*direction[0]
                veh_y = VehicleInfo.at[index, 'y']*direction[0]
                veh_v = VehicleInfo.at[index, 'v']
                l = (VehicleInfo.at[index, 'length'] + ego_vehicle_l) / 2.0
                locate_thresold = 1.0 + VehicleInfo.at[index, 'width'] / 2.0
                if self.locate_highway(now_lane, VehicleInfo.at[index, 'x'], VehicleInfo.at[index, 'y']-bias, locate_thresold) and ego_x - veh_x < 0:
                    VehicleInfo.at[index, 'VehicleStatus'] = 'SameLaneF'
                    if ego_x - veh_x > -l - 2:
                        flag = 0
                elif self.locate_highway(now_lane, VehicleInfo.at[index, 'x'], VehicleInfo.at[index, 'y']-bias, locate_thresold) and ego_x - veh_x > 0:
                    VehicleInfo.at[index, 'VehicleStatus'] = 'SameLaneB'
                elif num_change == 0:
                    VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                else:
                    goal_lane_id = self.GetGoalLane(to_order, now_lane_id, direction)
                    goal_lane = self.GetLaneData(road_data, goal_lane_id)
                                          
                    if 'Right' in to_order and self.locate_highway(goal_lane, VehicleInfo.at[index, 'x'], VehicleInfo.at[index, 'y']-bias, locate_thresold):
                        if 8+l < ego_x - veh_x or ego_x - veh_x < -4-l:
                            VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                        elif ego_x - veh_x < 0:
                            if ego_x - veh_x > -2-l:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0
                            elif veh_v >= ego_vehicle_v:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            elif ego_x - veh_x < -2-l-(ego_vehicle_v-veh_v)*2:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            else:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0
                        else:
                            if ego_x - veh_x < 4+l:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0
                            elif veh_v <= ego_vehicle_v:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            elif ego_x - veh_x > 4+l-(ego_vehicle_v-veh_v)*2:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            else:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0                            
                    elif 'Left' in to_order and self.locate_highway(goal_lane, VehicleInfo.at[index, 'x'], VehicleInfo.at[index, 'y']-bias, locate_thresold):
                        if 8+l < ego_x - veh_x or ego_x - veh_x < -4-l:
                            VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                        elif ego_x - veh_x < 0:
                            if ego_x - veh_x > -1.5-l:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0
                            elif veh_v >= ego_vehicle_v:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            elif ego_x - veh_x < -1.5-l-(ego_vehicle_v-veh_v)*1.5:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            else:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0
                        else:
                            if ego_x - veh_x < 3+l:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0
                            elif veh_v <= ego_vehicle_v:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            elif ego_x - veh_x > 3+l-(ego_vehicle_v-veh_v)*1.5:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'Ignore'
                            else:
                                VehicleInfo.at[index, 'VehicleStatus'] = 'NoTurn'
                                flag = 0    
            
            if flag and num_change:
                EgoVehicleInfo.at['ego', 'VehicleStatus'] = 'turning'
                num_change -= 1
                now_lane_id = EgoVehicleInfo.at['ego', 'lane_id']
                goal_lane_id = self.GetGoalLane(to_order, now_lane_id, direction)
                goal_lane = self.GetLaneData(road_data, goal_lane_id)
                
                for index, OneVehicleData in VehicleInfo.iterrows():
                    if index == 'ego':
                        continue
                    veh_x = VehicleInfo.at[index, 'x']*direction[0]
                    if self.locate_highway(goal_lane, VehicleInfo.at[index, 'x'], VehicleInfo.at[index, 'y']-bias, locate_thresold) and ego_x - veh_x < 0:
                        VehicleInfo.at[index, 'VehicleStatus'] = 'SameLaneF'
                    elif self.locate_highway(goal_lane, VehicleInfo.at[index, 'x'], VehicleInfo.at[index, 'y']-bias, locate_thresold) and ego_x - veh_x > 0:
                        VehicleInfo.at[index, 'VehicleStatus'] = 'SameLaneB'

                goal_X, goal_Y = self.GetPreview(goal_lane, ego_vehicle_x, ego_vehicle_y, direction, bias)
                return np.array([goal_X - ego_vehicle_x, goal_Y - ego_vehicle_y + bias]), EgoVehicleInfo, num_change, VehicleInfo
            
            else:
                now_lane_id = EgoVehicleInfo.at['ego', 'lane_id']
                now_lane = self.GetLaneData(road_data, now_lane_id)

                goal_X, goal_Y = self.GetPreview(now_lane, ego_vehicle_x, ego_vehicle_y, direction, bias)
                return np.array([goal_X - ego_vehicle_x, goal_Y - ego_vehicle_y + bias]), EgoVehicleInfo, num_change, VehicleInfo

    def act_highway(self, EgoVehicleInfo, preview_point, VehicleInfo):
        EgoVehicleYaw = EgoVehicleInfo.at['ego', 'yaw']

        PrevDis = np.linalg.norm(preview_point)
        EgoVehicleLength = EgoVehicleInfo.at['ego', 'length']
        WheelBase = EgoVehicleLength / 1.7
        alpha = math.atan2(preview_point[1], preview_point[0]) - EgoVehicleYaw
        delta = math.atan2(2 * WheelBase * np.sin(alpha), PrevDis)
        a = self.desire_acc_highway(VehicleInfo)
        return a, delta

    def getInformFront_highway(self, VehicleInfo):
        EgoVehicleX = VehicleInfo.at['ego', 'x']
        EgoVehicleY = VehicleInfo.at['ego', 'y']
        EgoVehicleYaw = VehicleInfo.at['ego', 'yaw']
        EgoVehicleLength = VehicleInfo.at['ego', 'length']

        if 'SameLaneF' in list(VehicleInfo['VehicleStatus']):
            SameLineVehicleInfo = VehicleInfo[VehicleInfo['VehicleStatus'] == 'SameLaneF']
            MinDis = 9999
            for index, OneVehicleData in SameLineVehicleInfo.iterrows():
                if abs(SameLineVehicleInfo.at[index, 'x'] - EgoVehicleX) < MinDis:
                    MinDis = abs(SameLineVehicleInfo.at[index, 'x'] - EgoVehicleX)
                    NearestVehicleInfo = SameLineVehicleInfo[SameLineVehicleInfo['x'] == SameLineVehicleInfo.at[index, 'x']]
            NearestVehicleIndex = NearestVehicleInfo.index[0]
            dis_gap_f = ((NearestVehicleInfo.at[NearestVehicleIndex,'x'] - EgoVehicleX) ** 2 + (NearestVehicleInfo.at[NearestVehicleIndex,'y'] - EgoVehicleY) ** 2) ** 0.5 \
                      - (EgoVehicleLength + NearestVehicleInfo.at[NearestVehicleIndex,'length']) / 2
            fv = NearestVehicleInfo.at[NearestVehicleIndex,'v']
            #print('Following Vehicle', list(NearestVehicleInfo.index))
        else:
            dis_gap_f = -1
            fv = -1
            #print('No Following Vehicle')
        if dis_gap_f > 100:
            dis_gap_f = -1
            fv = -1
        return fv, dis_gap_f

    def getInformBehind_highway(self, VehicleInfo):
        EgoVehicleX = VehicleInfo.at['ego', 'x']
        EgoVehicleY = VehicleInfo.at['ego', 'y']
        EgoVehicleYaw = VehicleInfo.at['ego', 'yaw']
        EgoVehicleLength = VehicleInfo.at['ego', 'length']

        if 'SameLaneB' in list(VehicleInfo['VehicleStatus']):
            SameLineVehicleInfo = VehicleInfo[VehicleInfo['VehicleStatus'] == 'SameLaneB']
            MinDis = 9999
            for index, OneVehicleData in SameLineVehicleInfo.iterrows():
                if abs(SameLineVehicleInfo.at[index, 'x'] - EgoVehicleX) < MinDis:
                    MinDis = abs(SameLineVehicleInfo.at[index, 'x'] - EgoVehicleX)
                    NearestVehicleInfo = SameLineVehicleInfo[SameLineVehicleInfo['x'] == SameLineVehicleInfo.at[index, 'x']]
            NearestVehicleIndex = NearestVehicleInfo.index[0]
            dis_gap_b = ((NearestVehicleInfo.at[NearestVehicleIndex,'x'] - EgoVehicleX) ** 2 + (NearestVehicleInfo.at[NearestVehicleIndex,'y'] - EgoVehicleY) ** 2) ** 0.5 \
                      - (EgoVehicleLength + NearestVehicleInfo.at[NearestVehicleIndex,'length']) / 2
            bv = NearestVehicleInfo.at[NearestVehicleIndex,'v']
            #print('Followed Vehicle', list(NearestVehicleInfo.index))
        else:
            dis_gap_b = -1
            bv = -1
            #print('No Followed Vehicle')
        if dis_gap_b > 60:
            dis_gap_b = -1
            bv = -1
        return bv, dis_gap_b

    def desire_acc_highway(self, VehicleInfo):
        fv, dis_gap_f = self.getInformFront_highway(VehicleInfo)
        bv, dis_gap_b = self.getInformBehind_highway(VehicleInfo)
        v = VehicleInfo.at['ego', 'v']
        w = VehicleInfo.at['ego', 'width']
        if dis_gap_f < 0:
            a_idm = self.a * (1 - (v / self.exv) ** self.gama)
        elif dis_gap_b < 0 or bv <= v:
            # 求解本车与前车的期望距离
            # print(self.s0,self.s1,self.exv,v,self.t)
            # self.s_ = self.s0 + self.s1 * (v / self.exv) ** 0.5 + self.t * v + v * (v - fv) / 2 / (self.a * self.b) ** 0.5
            s_ = self.s0 + w/4 + max(0, self.t * (v - fv) + (v - fv + 0.5) ** 2 / 2 / (self.a * self.b) ** 0.5)
            # 求解本车加速度
            a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((s_ / (dis_gap_f + 1e-6)) ** 2))
        else:
            ttc = dis_gap_b / (bv -v)
            if ttc < 2:
                a_idm = 15
            else:
                s_ = self.s0 + w/4 + max(0, self.t * (v - fv) + (v - fv + 0.5) ** 2 / 2 / (self.a * self.b) ** 0.5)
                # 求解本车加速度
                a_idm = self.a * (1 - (v / self.exv) ** self.gama - ((s_ / (dis_gap_f + 1e-6)) ** 2))

        # 对加速度进行约束
        if abs(a_idm) <= 6:
            a_idm = np.clip(a_idm, -self.a_bound, self.a_bound)
        elif a_idm > 6:
            a_idm = 2*a_idm - 9
        else:
            a_idm = 2*a_idm + 9
        # print(v,fv,dis_gap,a_idm,self.s_)
        # print(state,v,fv,dis_gap,a_idm)
        return a_idm
    
    def GetRoadData(self, scenario):
        obs, traj = self.controller.init(scenario)
        road_data = obs.road_info
        return road_data
    
    def GetGoalLane(self, to_order, now_lane_id, direction):
        if 'Left' in to_order:
            goal_lane_id = now_lane_id + 1*direction[0]
        else:
            goal_lane_id = now_lane_id - 1*direction[0]
        return goal_lane_id
    
    def locate_highway(self, center_lane, X, Y, thresold, flag = True):
        lane_size = center_lane.shape[0]
        min_dis = 99999
        index = 0
        for i in range(int(lane_size/100)):
            dis = np.sqrt((center_lane[i*100][1] - Y)**2 + (center_lane[i*100][0] - X)**2)
            if dis < min_dis:
                min_dis = dis
                index = i
        
        s_i = (index-1)*100
        e_i = (index+1)*100
        if s_i < 0:
            s_i = 0
        if e_i > lane_size:
            e_i = lane_size
        
        min_x = 9999
        for i in range(s_i, e_i):
            dis = np.sqrt((center_lane[i][1] - Y)**2 + (center_lane[i][0] - X)**2)
            if dis < thresold:
                if flag is True:
                    return True
                else:
                    if min_x >= abs(center_lane[i][0] - X):
                        min_x = abs(center_lane[i][0] - X)
                    else:
                        self.locate_index = i - 1
                        return True
        
        return False
    
    def GetLaneData(self, road_data, lane_id):
        for discrete_lane in road_data.discretelanes:
            center_lane = discrete_lane.center_vertices
            if ('.'+str(lane_id)) in discrete_lane.lane_id:
                goal_lane = center_lane
                break
        return goal_lane
    
    def GetPreview(self, center_lane, X, Y, direction, bias):
        index = -1
        min_dis = 9999

        goal_dis = np.sqrt((self.goalY - Y)**2 + (self.goalX - X)**2)
        if goal_dis < 60:
            X0 = self.goalX
            Y0 = self.goalY
            prevY = 30*(Y0-Y)/(X0-X) + Y
            return X + 30*direction[0], prevY-bias*1.5
                
        for i, point in enumerate(center_lane):
            if (point[0] - X) * direction[0] <= 0:
                continue
            if abs(np.sqrt((point[1] - Y)**2 + (point[0] - X)**2) -30) < min_dis:
                min_dis = abs(np.sqrt((point[1] - Y)**2 + (point[0] - X)**2) - 30)
                index = i
        
        if index >=0 and min_dis < 2:
            return center_lane[index][0], center_lane[index][1]
        else:
            X0 = center_lane[-1][0]
            Y0 = center_lane[-1][1]
            prevY = 30*(Y0-Y)/(X0-X) + Y
            return X + 30*direction[0], prevY