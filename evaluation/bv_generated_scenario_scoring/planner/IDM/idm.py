#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import lib
import numpy as np
import pandas as pd
from planner.plannerBase import PlannerBase
from utils.observation import Observation
from utils.opendrive2discretenet import parse_opendrive
from typing import List, Tuple
import math

from lxml import etree




"""
版本 V5.5.2
更新日期 2024.5.31
更新内容:
1.  增加重叠车兼容
"""


class IDM(PlannerBase):
    def __init__(self, a_bound=4, exv=40, t=1.2, a=2.22, b=2.4, gama=4, s0=1.0, s1=2.0):
        """跟idm模型有关的模型参数
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

    def init(self, scenario_dict):
        # print("----------------------------IDM INIT----------------------------")
        # print(scenario_dict)
        # print('当前版本 V4.9')
        # print("----------------------------------------------------------------")

        # 预瞄距离设置
        self.pre_scn_distance = 2

        # 记录测试时刻
        self.time = 0

        # 记录初始加速度
        self.acc_last = 0

        # 终点坐标
        self.target_xy = None

        # 取出 type
        self.type = scenario_dict['type']
        # 取出 waypoints
        self.waypoints = scenario_dict['task_info']['waypoints']

        # 取出dt
        self.dt = scenario_dict['task_info']['dt']

        # 标记是否为单road轨迹
        self.road_single = False

        # 最近轨迹点距离
        self.nearest_point_distance = 0

        # 检测是否重叠车
        self.chongdie = [False, 0]

        # 建立全局中心线轨迹列表
        self.line_center_global_list = []
        # 建立全局道路方案列表
        self.line_center_global_plan_list = []
        if self.type != 'SERIAL':
            # 识别路网
            parsed_data = parse_opendrive(scenario_dict['source_file']['xodr'])  # 解析xodr文件
            # 读取xodr静态路网文件
            with open(scenario_dict['source_file']['xodr'], 'r', encoding='utf-8') as fh:
                root = etree.parse(fh).getroot()

            # 解析道路散点坐标
            self.luwang_point = self.luwang_point_dict(parsed_data)
            luwang_point_dict = self.luwang_point

        targetPos = scenario_dict['task_info']['targetPos']  # 取出终点范围
        startPos = scenario_dict['task_info']['startPos']  # 取出起点坐标
        self.target_xy = [((targetPos[0][0] + targetPos[1][0]) / 2),
                        ((targetPos[0][1] + targetPos[1][1]) / 2)]
        # 非无限里程道路
        if self.type != 'SERIAL':
            # 判断起点/终点坐标及所属道路
            targetPos, target_lane_id = self.locate_Pos(luwang_point_dict, targetPos)
            startPos, start_lane_id = self.locate_Pos(luwang_point_dict, startPos)
            # print("已识别到终点坐标：", targetPos, "确定终点位于道路:", target_lane_id, "已识别到起点坐标：", startPos, "确定起点位于道路:", start_lane_id)
            # if target_lane_id == None:
            #     print("终点道路所属未找到")
            # if start_lane_id == None:
            #     print("起点道路所属未找到")

        # 非无限里程道路
        if self.type != 'SERIAL':
            start_target_lane_id = []  # 起点 终点组合列表
            # 对起点终点不同的属于道路进行方案整合，如果搜寻到一条起点到终点道路即停止
            for i in target_lane_id:  # 取出终点属于的所有道路
                for j in start_lane_id:  # 取出终点属于的所有道路
                    start_lane_id_member = j[0]
                    target_lane_id_member = i[0]
                    start_lane_id_member = [int(part) for part in start_lane_id_member.split('.')]
                    target_lane_id_member = [int(part) for part in target_lane_id_member.split('.')]
                    start_target_lane_id.append([start_lane_id_member, target_lane_id_member])
            # 遍历 起点 终点 组合方案
            for i in start_target_lane_id:
                start_lane_id_member = i[0]
                target_lane_id_member = i[1]
                line_center_global = []
                if start_lane_id_member[0] != target_lane_id_member[0]:
                    # 构建路网接续字典
                    luwang_successor_dict = self.luwang_successor_dict(root)

                    # 解析道路连接方案
                    luwang_list = self.luwang_list(luwang_successor_dict)

                    # 全局中心线
                    line_center_global = self.line_center_global(start_lane_id_member, target_lane_id_member,
                                                                 luwang_list, luwang_point_dict)

                else:  # 如果起点终点在同一道路，全局路径轨迹为终点区域细分路段中心散点
                    # print('起点、终点均位于同一road')
                    self.road_single = True
                    for key, value in luwang_point_dict.items():  # 遍历道路散点字典，找到可行方案中心线
                        key = [int(part) for part in key.split('.')]
                        if key == target_lane_id_member:
                            line_center_global = value[2]

                # if line_center_global != []:  # 如果找到中心线，停止循环， 考虑后期加不同道路方案长度优劣判定
                #     break
                self.line_center_global_list.append(line_center_global)


        # 无限里程道路直接使用轨迹点作为全局轨迹
        else:
            # 将 waypoints转化并作为全局轨迹
            values_array = np.array([value for value in self.waypoints.values()])
            self.line_center_global_list.append(values_array)

        # 找到空列表的索引
        empty_indices = [
            index for index, item in enumerate(self.line_center_global_list)
            if (isinstance(item, np.ndarray) and item.size == 0) or (not isinstance(item, np.ndarray) and not item)
        ]

        # 剔除 self.line_center_global_list 中的空列表
        self.line_center_global_list = [
            item for index, item in enumerate(self.line_center_global_list)
            if index not in empty_indices
        ]
        if len(self.line_center_global_list) == 0:
            self.line_center_global_list = np.array([
                [float(startPos[0]), float(startPos[1])],
                [float(self.target_xy[0]), float(self.target_xy[1])],
            ], dtype=np.float64)
        else:
            self.line_center_global_list = self.line_center_global_list[0]


    def act(self, observation: Observation):
        # 加载主车信息
        frame = pd.DataFrame(
            vars(observation.ego_info),
            columns=['x', 'y', 'v', 'yaw', 'length', 'width'],
            index=['ego']
        )
        # 加载背景要素状态信息
        for obj_type in observation.object_info:
            for obj_name, obj_info in observation.object_info[obj_type].items():
                sub_frame = pd.DataFrame(vars(obj_info), columns=['x', 'y', 'v', 'yaw', 'length', 'width'],
                                         index=[obj_name])
                frame = pd.concat([frame, sub_frame])
        state = frame.to_numpy()

        # 记录当前测试时刻
        self.time = self.time + self.dt
        # print(192, "当前测试时刻", self.time)
        ego_yaw_now = state[0][3]
        # 当前场景其他车辆最大速度
        if state[1:, 2].any():
            exv_max_other = max(state[1:, 2])
            if exv_max_other == 0:
                exv_max_ego = 15
            else:
                exv_max_ego = exv_max_other * 1.2

            self.exv = int(exv_max_ego)
        if self.exv < 7:
            self.exv = 7  # 兼容其它车辆速度太小场景
        v, fv, fv_xy, dis_gap, direction, a_car_num, v_nearest, f_yaw = self.getInformFront(state, 1)
        v, bv, bv_xy, dis_gap_b, direction, a_car_num, v_nearest, b_yaw = self.getInformFront(state, 2)
        v, cv, cv_xy, dis_gap_c, direction, a_car_num, v_nearest, c_yaw = self.getInformFront(state, 3)

        # 计算它车与自车角度差
        if abs(ego_yaw_now - c_yaw) < np.pi:  # 防止0左右的溢出
            cv_cankao = abs(ego_yaw_now - c_yaw) < 2.5
        else:
            cv_cankao = abs(abs(ego_yaw_now- c_yaw) - np.pi * 2) < 2.5

        turn_state = self.turn_detect(state)

        if fv > 0 and dis_gap < 15:  # 取出近前车作为参考
            self.exv = 1.2 * fv
        elif cv > 1 and (dis_gap_c[0] < 2 and 0 < dis_gap_c[1] < 10) and cv_cankao:  # 取出近侧向车作为参考
            self.exv = 1.2 * cv

        # 非无限里程道路
        if self.type != 'SERIAL':
            if turn_state != False and (self.exv > 7) and v_nearest < 20:  # 有 20米后弯道
                self.exv = 7
                # print(189, '20米后有弯道')
        else:
            if turn_state != False and (self.exv > 3):  # 有 20米后弯道
                self.exv = 3
                # print(189, '20米后有弯道')

        yaw, cepian_acc = self.yaw_deside(state, observation)

        # 获取加速度
        acc = self.deside_acc(state, observation, cepian_acc)

        # 加速度溢出矫正，加速度幅度为 4.9
        acc_limit = 4.9 / (0.1 / self.dt)
        if self.acc_last < 0:  # 上一加速度为负数
            if acc > 0:  # 此时刻给一正加速
                if abs(self.acc_last - acc) > acc_limit:  # 超过幅度值
                    acc = acc_limit + self.acc_last  # 矫正
        else:  # 非负
            if acc < 0:  # 此时刻给一减加速
                if abs(self.acc_last - acc) > acc_limit:  # 超过幅度值
                    acc = self.acc_last - acc_limit

        # 动力学校验
        # 下一时刻速度
        v_next = v + acc * self.dt
        yaw_val = self.donglixue(v_next, acc)

        if abs(yaw_val) < abs(yaw):

            yaw = np.clip(yaw, -abs(yaw_val), abs(yaw_val))


        # 动力学小bug，速度不能低于0.5，否则全局一点偏转角都不能有，速度低于2的时候不能有转向操作
        if v < 2:
            yaw = 0
        if v_next < 0.5:
            if acc < 0:
                acc = (0.5 - v) / self.dt

        # 场景前三个时刻直行且加速
        if self.time / self.dt < 4:
            yaw = 0
            acc = np.clip(acc, 0, abs(acc))

        # 兼容车重叠场景
        if (self.time == self.dt) and (dis_gap != -1):
            x1, y1 = fv_xy
            x2, y2 = state[0][0], state[0][1]
            distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if distance_i < state[0][4] / 2:
                self.chongdie = [True, self.nearest_point_distance]

        if self.chongdie[0] == True and self.chongdie[1] < 2:
            if (np.pi / 2 < ego_yaw_now < np.pi * 3 / 2):
                # 场景前三个时刻直行且加速
                if self.time / self.dt < 14:
                    if self.nearest_point_distance < 4:
                        acc = 4
                    else:
                        acc = -2
                    v_next = v + acc * self.dt
                    f_yaw = self.other_yaw_yichu(ego_yaw_now, f_yaw)
                    if abs(ego_yaw_now - f_yaw) < 0.4:
                        if c_yaw != -1:
                            is_right_cexiang = self.is_right_to_ego(state, (cv_xy, ego_yaw_now))
                            if is_right_cexiang == True:
                                yaw = abs(self.donglixue(v_next, acc))
                            else:
                                yaw = - abs(self.donglixue(v_next, acc))
                        else:
                            yaw = - abs(self.donglixue(v_next, acc))
                    else:
                        yaw = 0
        return [acc, yaw]

    # 加速度控制
    def deside_acc(self, state: pd.DataFrame, observation, cepian_acc) -> float:
        # 取出最近前车信息
        v, fv, fv_xy, dis_gap, direction, a_car_num, v_nearest, f_yaw = self.getInformFront(state, 1)
        v, bv, bv_xy, dis_gap_b, direction, a_car_num, v_nearest, b_yaw = self.getInformFront(state, 2)

        # 获取当前坐标与下一步全局坐标
        ego_xy, next_global_xy, last_global_xy, nearest_global_xy = self.LineCenterGlobal(state)
        nearest_point_distance = np.sqrt(
            (ego_xy[0] - nearest_global_xy[0]) ** 2 + (ego_xy[1] - nearest_global_xy[1]) ** 2)
        self.nearest_point_distance = nearest_point_distance
        # 计算与前车的安全距离

        front_safe_distance = (v ** 2 - fv ** 2) / (2 * self.a_bound) + 3

        if fv < 1:
            front_safe_distance += 5  # 前车静止时，安全距离调大，因为自车无法完全停车
        # print(171, '最近前车', a_car_num, '距离', dis_gap, '前车速度', fv, '本车速度', v, '安全距离为', front_safe_distance)
        # 取前后车速度最大的作为本车速度参考
        other_v = max(fv, bv)

        # 无前车 or 前车距离足够 or 前车速度大
        if (((dis_gap == -1) or (((v < fv * 1.2 and dis_gap > 5) or fv > 20) and dis_gap > 1)) and ((dis_gap > front_safe_distance) or dis_gap == -1) \
                and cepian_acc == 0) and (nearest_point_distance < 8 or v < 2):
            exv = self.exv
            if other_v > v:
                if dis_gap > front_safe_distance * 1.5:  # 前车较远
                    exv = max(other_v, v)
                else:
                    exv = other_v * 1.2

            a_idm = self.a * (1 - (v / (exv + 1e-6)) ** self.gama)


            # print(264, '前车距离', dis_gap, '前车安全距离', front_safe_distance, '前车速度', fv, '后车速度', bv, '本车速度', v, '期望速度', exv, '加速度', a_idm, cepian_acc)
        else:
            # 求解本车与前车的期望距离
            self.s_ = self.s0 + v * ((v - fv) / (self.a_bound + 1e-6)) + state[0, 4] + dis_gap

            # 求解本车加速度
            a_idm = self.a * (1 - (v / (self.exv + 1e-6)) ** self.gama - ((front_safe_distance / (dis_gap + 1e-6)) ** 2))
            if dis_gap < 0:
                a_idm = - self.a_bound
            if a_idm > 0:
                a_idm = -a_idm

            # print(212, '加速度',a_idm, '前车/安全距离',dis_gap, front_safe_distance,'车速/期望车速', v, self.exv, cepian_acc, nearest_point_distance)


        # 碰撞风险检测
        deside_avoid_acc, deside_avoid_direct_yaw, turn_guidance, yaw_back = self.deside_avoid(state)
        if deside_avoid_acc:  # 触发减速避障
            a_idm = deside_avoid_acc
            # print(295, a_idm, '避撞加速度控制触发')
        # 对加速度进行约束
        a_idm = np.clip(a_idm, -self.a_bound, self.a_bound)

        # 前方车静止，满减速刹停
        if fv == 0 and a_idm < 0:
            a_idm = -10

        # 检测终点
        target_near, L = self.target_rash_deside(state)
        turn_detect = self.turn_detect(state)
        # 接近终点 and 无弯道 and 无碰撞风险
        if target_near == True and turn_detect == False and deside_avoid_acc == []:
            # 无前车 or 前车距离大于终点距离 or 前车速度大
            if dis_gap == -1 or L < dis_gap or fv > v:
                a_idm = self.a_bound
        # print(326, a_idm)

        return a_idm

    # 获得其他车辆信息
    def getInformFront(self, state: pd.DataFrame, car_target) -> Tuple[float, float, float, float]:
        """
        根据 car_target信息指定获取对象
        """
        # 初始化 car_target，1为前车，2为后车，3为侧向车
        cv, cv_xy, c_dis_gap, c_yaw = 0, 0, -1, -1
        bv, bv_xy, b_dis_gap, b_yaw = 0, 0, -1, -1

        # 计算当前轨迹角度
        # 获取当前坐标与下一步全局坐标
        ego_xy, next_global_xy, last_global_xy, nearest_global_xy = self.LineCenterGlobal(state)
        yaw_guiji_now = self.calculate_angle(nearest_global_xy, next_global_xy)

        # 横纵向初始距离置空
        distance_hengxiang_true = 0
        distance_zongxiang_true = 0

        # 判定车辆的横纵向驾驶场景，以两条45度线切割坐标系
        drving_direction = 0  # 1 为纵向 -1 为横向
        directio = 0
        # 车辆纵向驾驶
        if (np.pi * 3 / 4) > state[0, 3] > (np.pi / 4) or (np.pi * 7 / 4) > state[0, 3] > (np.pi * 5 / 4):
            # 取出航向角象限信息
            if 0 < state[0, 3] < np.pi:
                direction = 1.0
            else:
                direction = -1.0
            state[:, 1] = state[:, 1] * direction  # 所有车辆实时信息置正

            drving_direction = 1  # 1 为纵向 -1 为横向

        # 车辆横向驾驶
        else:
            # 取出航向角象限信息
            if state[0, 3] < np.pi / 2 or state[0, 3] > np.pi * 3 / 2:
                direction = 1.0
            else:
                direction = -1.0
            state[:, 0] = state[:, 0] * direction  # 所有车辆实时信息置正
            drving_direction = -1  # 1 为纵向 -1 为横向


        ego = state[0, :]  # ego车实时信息
        v, fv, dis_gap, fv_yaw = ego[2], -1, -1, -1

        # 判定与本车的前后关系
        if drving_direction == -1:  # 横向驾驶
            x_ind = ego[0] < state[:, 0]  # 判定 x轴意义上本车前侧所有车辆关系
            back_ind = ego[0] > state[:, 0]  # 判定 x轴意义上本车后侧所有车辆关系

        elif drving_direction == 1:  # 纵向驾驶
            x_ind = ego[1] < state[:, 1]  # 判定 y轴意义上本车前侧所有车辆关系
            back_ind = ego[1] > state[:, 1]  # 判定 y轴意义上本车后侧所有车辆关系

        # 判断完相对关系后，将 state信息恢复，方便后续计算
        if drving_direction == 1 and direction == -1:  # 纵向驾驶恢复
            state[:, 1] = state[:, 1] * direction
        elif drving_direction == -1 and direction == -1:
            state[:, 0] = state[:, 0] * direction  # 横向驾驶恢复

        ego_xy = [state[0][0], state[0][1]]  # 取出本车坐标
        ego_other_distance_list = []  # 建立本车与它车的横向距离列表
        ego_other_distance_zongxiang_list = []  # 建立本车与它车的纵向距离列表
        ego_yaw_now = state[0][3]

        p2p_distance_list = []
        # 取出每辆车与本车的横向距离
        for i in state[:]:
            i_ego_xy = [i[0], i[1]]

            if i_ego_xy == ego_xy:  # ego自车
                # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(i_ego_xy, ego_xy,
                                                                                                   yaw_guiji_now)
                # distance_hengxiang, distance_zongxiang, p2p_distance = 0, 0, 0
            else:  # 它车算四个边界点
                # 区分前后车
                is_back = self.is_back_to_ego(state, drving_direction, i_ego_xy)
                ego_corners_xy = self.other_car_points_locat(state[0])
                distance_hengxiang_list = []
                distance_zongxiang_list = []
                p2p_distance_linshi_list = []
                if is_back == False:  # 前车
                    # 取出它车四个边界点距离，分别计算并拿出最大值
                    corners_xy = self.other_car_points_locat(i)
                    # 分别计算四个点
                    for j in corners_xy:
                        other_ego_xy = [j[0], j[1]]
                        # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                        distance_hengxiang_guiji, distance_zongxiang_guiji, p2p_distance = self.distance_point_to_line(
                            other_ego_xy,
                            ego_xy,
                            yaw_guiji_now)
                        distance_hengxiang_ego, distance_zongxiang_ego, p2p_distance = self.distance_point_to_line(
                            other_ego_xy,
                            ego_xy,
                            ego_yaw_now)
                        # 纵向距离小于2，用自车精确角
                        if max(distance_zongxiang_guiji, distance_zongxiang_ego) < (3 + (ego[4]) / 2):
                            distance_hengxiang = distance_hengxiang_ego
                            distance_zongxiang = distance_zongxiang_ego
                        else:
                            distance_hengxiang = distance_hengxiang_guiji
                            distance_zongxiang = distance_zongxiang_guiji

                        distance_hengxiang_list.append(distance_hengxiang)
                        distance_zongxiang_list.append(distance_zongxiang)
                        p2p_distance_linshi_list.append(p2p_distance)
                else:  # 后车
                    # 分别计算四个点
                    for j in ego_corners_xy:
                        other_ego_xy = [j[0], j[1]]
                        # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                        distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(
                            other_ego_xy,
                            i_ego_xy,
                            i[3])

                        distance_hengxiang_list.append(distance_hengxiang)
                        distance_zongxiang_list.append(distance_zongxiang)
                        p2p_distance_linshi_list.append(p2p_distance)

                # 取最小值
                distance_hengxiang = min(distance_hengxiang_list)
                distance_zongxiang = min(distance_zongxiang_list)

                p2p_distance = min(p2p_distance_linshi_list)

            ego_other_distance_list.append(distance_hengxiang)
            ego_other_distance_zongxiang_list.append(distance_zongxiang)

            p2p_distance_list.append(p2p_distance)

        # 对列表进行排序
        sorted_numbers = sorted(p2p_distance_list)

        if len(sorted_numbers) == 1:  # 当前只有自车
            index_p2p_distance_list = p2p_distance_list.index(sorted_numbers[0])
        else:
            index_p2p_distance_list = p2p_distance_list.index(sorted_numbers[1])
        v_nearest = state[index_p2p_distance_list][2]


        ego_other_distance_array = np.array(ego_other_distance_list)  # 将距离列表转化为数组方便比较计算
        ego_other_zongxiang_distance_array = np.array(ego_other_distance_zongxiang_list)  # 将距离列表转化为数组方便比较计算

        y_ind = (ego_other_distance_array) < ((ego[5] / 2) + 0.2)  # 横向要留有至少0.1米距离

        front_yaw_ind = []
        for i in state[:, 3]:
            other_yaw = self.other_yaw_yichu(state[0][3], i)
            front_yaw_ind.append(abs(other_yaw - ego_yaw_now) < 0.8)  # 剔除异向车辆
            # if abs(i - ego_yaw_now) < np.pi:  # 防止0左右的溢出
            #     front_yaw_ind.append(abs(i - ego_yaw_now) < 0.8)  # 剔除异向车辆
            # else:
            #     front_yaw_ind.append(abs(abs(i - ego_yaw_now) - np.pi * 2) < 0.8)

        ind = x_ind & y_ind & front_yaw_ind  # 综合考虑横纵向因素 加一个航向角关联

        # 前车检测
        if car_target == 1:  # 获取前车
            fv_xy = False
            if ind.sum() > 0:  # 打印 True的数量
                state_ind = state[ind, :]

                closest_index = np.argmin(np.linalg.norm(state_ind[:, :2] - ego[:2], axis=1))
                front = state_ind[closest_index]  # 最近的前车信息

                fv = front[2]  # 前车速度
                fv_xy = [front[0], front[1]]
                fv_yaw = front[3]

                # 取出它车四个边界点距离，分别计算并拿出最大值
                corners_xy = self.other_car_points_locat(front)
                distance_hengxiang_list = []
                distance_zongxiang_list = []
                p2p_distance_linshi_list = []
                # 分别计算四个点
                for j in corners_xy:
                    other_ego_xy = [j[0], j[1]]
                    # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                    distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(other_ego_xy,
                                                                                                       ego_xy,
                                                                                                       yaw_guiji_now)
                    distance_hengxiang_list.append(distance_hengxiang)
                    distance_zongxiang_list.append(distance_zongxiang)
                    p2p_distance_linshi_list.append(p2p_distance)

                # 取最小值
                distance_hengxiang = min(distance_hengxiang_list)
                distance_zongxiang = min(distance_zongxiang_list)
                p2p_distance = min(p2p_distance_linshi_list)

                dis_gap = distance_zongxiang - (ego[4]) / 2  # 前向车纵向距离

            if dis_gap > 100:  # 大于100米 不做处理
                dis_gap = -1
                fv = -1

            cv = fv
            cv_xy = fv_xy
            c_dis_gap = dis_gap
            c_yaw = fv_yaw

        # 侧向车检测
        if car_target == 3:  # 侧向车检测


            # 检测横向距离 40以内的车
            cexiang_ind = (ego_other_distance_array > ((ego[5]) / 2)) & (
                        ego_other_distance_array < 4)   # 横向距离大于 0 小于 4 ，认为在本车侧向


            if cexiang_ind.sum() > 0:
                state_cexiang_ind = state[cexiang_ind, :]

                distances = np.linalg.norm(state_cexiang_ind[:, :2] - ego[:2], axis=1)
                # 找到最近点的索引
                first_closest_index = np.argmin(distances)
                if len(distances) > 1:
                    # 对数组进行排序
                    distances_sort = np.sort(distances)
                    # 找到数组中第二小的值
                    second_smallest_value = distances_sort[1]
                    # 找到原始数组中倒数第二小值（即第二小的值）的索引
                    second_closest_index = np.where(distances == second_smallest_value)[0][0]
                    # 如果第二辆车的纵向距离小于2，选取两车横向距离小的
                    # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                    first_xy = [state_cexiang_ind[first_closest_index][0], state_cexiang_ind[first_closest_index][1]]
                    second_xy = [state_cexiang_ind[second_closest_index][0], state_cexiang_ind[second_closest_index][1]]
                    distance_hengxiang_first, distance_zongxiang_first, p2p_distance_first = self.distance_point_to_line(
                        first_xy, ego_xy,
                        yaw_guiji_now)
                    distance_hengxiang_second, distance_zongxiang_second, p2p_distance_second = self.distance_point_to_line(
                        second_xy, ego_xy,
                        yaw_guiji_now)
                    # 如果纵向距离均小于2，取出横向距离最近的
                    if (ego[4]/2 < distance_zongxiang_first < (10 + ego[4]/2) and distance_zongxiang_second < (10 + ego[4]/2)):
                        if distance_hengxiang_first > distance_hengxiang_second:
                            closest_index = second_closest_index
                        else:
                            closest_index = first_closest_index
                    else:
                        closest_index = first_closest_index
                    # 如果第一车纵向已不足，优选首车
                    if distance_zongxiang_first < ego[4]/2:
                        closest_index = first_closest_index
                else:
                    closest_index = first_closest_index

                cexiang_info = state_cexiang_ind[first_closest_index]  # 最近的侧向车信息
                # print(540, cexiang_info, first_xy, second_xy, state_cexiang_ind, ego, distances, distance_zongxiang_first, distance_zongxiang_second)
                cexiang_v = cexiang_info[2]
                cexiang_xy = [cexiang_info[0], cexiang_info[1]]
                cexiang_yaw = cexiang_info[3]

                # 取出它车四个边界点距离，分别计算并拿出最大值
                corners_xy = self.other_car_points_locat(cexiang_info)
                ego_corners_xy = self.other_car_points_locat(state[0])
                distance_hengxiang_list = []
                distance_zongxiang_list = []
                p2p_distance_linshi_list = []

                # 区分前后车
                is_back = self.is_back_to_ego(state, drving_direction, cexiang_xy)
                is_right = self.is_right_to_ego(state, cexiang_xy)
                yaw_is_between = self.is_angle_between(c_yaw, ego_yaw_now - np.pi / 2, ego_yaw_now + np.pi / 2)

                distance_hengxiang_i, distance_zongxiang_i, p2p_distance_i = self.distance_point_to_line(
                    cexiang_xy, ego_xy,
                    ego_yaw_now)

                # 检查路径是否相交
                intersect, projection_point = self.does_ray_intersect_points(state, cexiang_xy, cexiang_yaw)
                # print(651, is_back, distance_zongxiang_i, drving_direction, cexiang_xy, ego_xy, intersect)
                if is_back == False:  # 前车 or 纵向已接近本车
                    c_yaw = self.other_yaw_yichu(ego_yaw_now, cexiang_yaw)
                    yaw_right = self.is_angle_between(c_yaw, ego_yaw_now, ego_yaw_now + np.pi)
                    yaw_left = self.is_angle_between(c_yaw, ego_yaw_now - np.pi, ego_yaw_now)
                    # print(670, is_right, yaw_right, yaw_left)
                    # 前车右侧 角度大于本车；前车左侧 角度小于本车
                    if (is_right == True and (yaw_right)) or (
                            is_right == False and (yaw_left)) or abs(c_yaw - ego_yaw_now) < 0.6:
                        # 分别计算四个点
                        for j in corners_xy:
                            other_ego_xy = [j[0], j[1]]
                            if intersect == True:
                                distance_hengxiang, distance_zongxiang, p2p = self.heng_zong_guiji_other(state, other_ego_xy)
                                # print(679, distance_zongxiang,distance_hengxiang)
                            else:
                                # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                                distance_hengxiang_guiji, distance_zongxiang_guiji, p2p_distance = self.distance_point_to_line(
                                    other_ego_xy,
                                    ego_xy,
                                    yaw_guiji_now)
                                distance_hengxiang_ego, distance_zongxiang_ego, p2p_distance = self.distance_point_to_line(
                                    other_ego_xy,
                                    ego_xy,
                                    ego_yaw_now)
                                # print(589, distance_hengxiang_ego, distance_zongxiang_ego, ego_xy, other_ego_xy, ego_yaw_now)
                                # 纵向距离小于2，用自车精确角
                                if max(distance_zongxiang_guiji, distance_zongxiang_ego) < (3 + (ego[4]) / 2):
                                    distance_hengxiang = distance_hengxiang_ego
                                    distance_zongxiang = distance_zongxiang_ego
                                else:
                                    distance_hengxiang = distance_hengxiang_guiji
                                    distance_zongxiang = distance_zongxiang_guiji

                            distance_hengxiang_list.append(distance_hengxiang)
                            distance_zongxiang_list.append(distance_zongxiang)

                    else:
                        # 分别计算四个点
                        for j in ego_corners_xy:
                            other_ego_xy = [j[0], j[1]]
                            # if intersect == True:
                            #     distance_hengxiang, distance_zongxiang, p2p = self.heng_zong_guiji_other(state,
                            #                                                                              other_ego_xy)
                            # else:
                            # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                            distance_hengxiang_guiji, distance_zongxiang_guiji, p2p_distance = self.distance_point_to_line(
                                other_ego_xy,
                                cexiang_xy,
                                yaw_guiji_now)
                            distance_hengxiang_ego, distance_zongxiang_ego, p2p_distance = self.distance_point_to_line(
                                other_ego_xy,
                                cexiang_xy,
                                ego_yaw_now)
                            # print(589, distance_hengxiang_ego, distance_zongxiang_ego, ego_xy, other_ego_xy, ego_yaw_now)
                            # 纵向距离小于2，用自车精确角
                            if max(distance_zongxiang_guiji, distance_zongxiang_ego) < (3 + (ego[4]) / 2):
                                distance_hengxiang = distance_hengxiang_ego
                                distance_zongxiang = distance_zongxiang_ego
                            else:
                                distance_hengxiang = distance_hengxiang_guiji
                                distance_zongxiang = distance_zongxiang_guiji

                            distance_hengxiang_list.append(distance_hengxiang)
                            distance_zongxiang_list.append(distance_zongxiang)
                else:  # 后车
                    c_yaw = self.other_yaw_yichu(ego_yaw_now, cexiang_yaw)
                    if abs(ego_yaw_now - c_yaw) < np.pi / 2:
                        # 分别计算四个点
                        for j in ego_corners_xy:
                            other_ego_xy = [j[0], j[1]]
                            # if intersect == True:
                            #     distance_hengxiang, distance_zongxiang, p2p_distance = self.heng_zong_guiji_other(state, other_ego_xy)
                            # else:
                                # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                            distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(
                                other_ego_xy,
                                cexiang_xy,
                                cexiang_yaw)

                            distance_hengxiang_list.append(distance_hengxiang)
                            distance_zongxiang_list.append(distance_zongxiang)
                    else:
                        distance_hengxiang_list.append(50)
                        distance_zongxiang_list.append(50)
                        p2p_distance_linshi_list.append(50)

                # 取最大值
                distance_hengxiang = min(distance_hengxiang_list)
                distance_zongxiang = min(distance_zongxiang_list)

                if is_back == False:
                    dis_gap_cexiang_zongxiang = distance_zongxiang - (ego[4]) / 2  # 侧向车纵向距离

                    dis_gap_cexiang_hengxiang = distance_hengxiang - (ego[5]) / 2  # 侧向车横向距离
                else:
                    dis_gap_cexiang_zongxiang = distance_zongxiang - (cexiang_info[4]) / 2  # 侧向车纵向距离

                    dis_gap_cexiang_hengxiang = distance_hengxiang - (cexiang_info[5]) / 2  # 侧向车横向距离
                cv = cexiang_v
                cv_xy = cexiang_xy
                c_dis_gap = [dis_gap_cexiang_hengxiang, dis_gap_cexiang_zongxiang]  # 距离列表，横向、纵向
                c_yaw = cexiang_yaw

        # 交叉侧向车检测
        if car_target == 4:  # 侧向车检测
            jiaocha_cecxiang_yaw_ind = []
            intersect_yaw_ind = []
            for i in state[:, ]:
                i_yaw = i[3]
                other_xy = [i[0], i[1]]
                other_yaw = self.other_yaw_yichu(state[0][3], i_yaw)
                # 向我车前进方向驶来的车辆
                jiaocha_cecxiang_yaw_ind.append(0.8 < abs(other_yaw - ego_yaw_now))  # 设置交叉侧向范围 0.8-2.6
                # 检查路径是否相交
                intersect, projection_point = self.does_ray_intersect_points(state, other_xy, i_yaw)
                intersect_yaw_ind.append(intersect)

            # 检测横向距离 40以内的车
            cexiang_ind = (ego_other_distance_array < 40)  # 横向距离大于 小于 40，认为在本车侧向
            cexiang_ind = cexiang_ind & jiaocha_cecxiang_yaw_ind & intersect_yaw_ind



            if cexiang_ind.sum() > 0:
                state_cexiang_ind = state[cexiang_ind, :]
                # print(756, state, cexiang_ind,jiaocha_cecxiang_yaw_ind,intersect_yaw_ind, state_cexiang_ind)
                closest_index = np.argmin(np.linalg.norm(state_cexiang_ind[:, :2] - ego[:2], axis=1))
                cexiang_info = state_cexiang_ind[closest_index]  # 最近的侧向车信息

                cexiang_v = cexiang_info[2]
                cexiang_xy = [cexiang_info[0], cexiang_info[1]]
                cexiang_yaw = cexiang_info[3]

                # 取出它车四个边界点距离，分别计算并拿出最大值
                corners_xy = self.other_car_points_locat(cexiang_info)
                ego_corners_xy = self.other_car_points_locat(state[0])
                distance_hengxiang_list = []
                distance_zongxiang_list = []
                p2p_distance_linshi_list = []

                # 区分前后车
                is_back = self.is_back_to_ego(state, drving_direction, cexiang_xy)
                is_right = self.is_right_to_ego(state, cexiang_xy)

                yaw_is_between = self.is_angle_between(cexiang_yaw, ego_yaw_now - np.pi / 2, ego_yaw_now + np.pi / 2)
                distance_hengxiang_i, distance_zongxiang_i, p2p_distance_i = self.distance_point_to_line(
                    cexiang_xy, ego_xy,
                    ego_yaw_now)


                if is_back == False or (distance_zongxiang_i < 3):  # 前车 or 纵向已接近本车
                    c_yaw = self.other_yaw_yichu(ego_yaw_now, cexiang_yaw)

                    yaw_right = self.is_angle_between(c_yaw, ego_yaw_now, ego_yaw_now + np.pi)
                    yaw_left = self.is_angle_between(c_yaw, ego_yaw_now - np.pi, ego_yaw_now)
                    # print(720, is_right, yaw_right, yaw_left, cexiang_xy, ego_xy)
                    # 前车右侧 角度大于本车；前车左侧 角度小于本车
                    if (is_right == True and (yaw_right)) or (
                            is_right == False and (yaw_left)):
                        # 分别计算四个点
                        for j in corners_xy:
                            other_ego_xy = [j[0], j[1]]
                            # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                            # distance_hengxiang_guiji, distance_zongxiang_guiji, p2p_distance = self.distance_point_to_line(
                            #     other_ego_xy,
                            #     ego_xy,
                            #     yaw_guiji_now)
                            # distance_hengxiang_ego, distance_zongxiang_ego, p2p_distance = self.distance_point_to_line(
                            #     other_ego_xy,
                            #     ego_xy,
                            #     ego_yaw_now)


                            # 纵向距离小于2，用自车精确角
                            # if max(distance_zongxiang_guiji, distance_zongxiang_ego) < (3 + (ego[4]) / 2):
                            #     distance_hengxiang = distance_hengxiang_ego
                            #     distance_zongxiang = distance_zongxiang_ego
                            # else:
                            #     distance_hengxiang = distance_hengxiang_guiji
                            #     distance_zongxiang = distance_zongxiang_guiji

                            distance_hengxiang, distance_zongxiang, p2p = self.heng_zong_guiji_other(state, other_ego_xy)
                            # print(847, distance_hengxiang, distance_zongxiang)
                            distance_hengxiang_list.append(distance_hengxiang)
                            distance_zongxiang_list.append(distance_zongxiang)

                    else:
                        # 分别计算四个点
                        for j in ego_corners_xy:
                            other_ego_xy = [j[0], j[1]]
                            # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                            distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(
                                other_ego_xy,
                                cexiang_xy,
                                cexiang_yaw)

                            # distance_hengxiang, distance_zongxiang, p2p = self.heng_zong_guiji_other(state, other_ego_xy)

                            distance_hengxiang_list.append(distance_hengxiang)
                            distance_zongxiang_list.append(distance_zongxiang)

                            distance_hengxiang_list.append(distance_hengxiang)
                            distance_zongxiang_list.append(distance_zongxiang)

                else:  # 后车
                    # print(765, cexiang_xy, '后车')
                    c_yaw = self.other_yaw_yichu(ego_yaw_now, cexiang_yaw)
                    if abs(ego_yaw_now - c_yaw) < np.pi / 2:
                        # 分别计算四个点
                        for j in ego_corners_xy:
                            other_ego_xy = [j[0], j[1]]
                            # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
                            distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(
                                other_ego_xy,
                                cexiang_xy,
                                cexiang_yaw)
                            # distance_hengxiang, distance_zongxiang, p2p = self.heng_zong_guiji_other(state, other_ego_xy)

                            distance_hengxiang_list.append(distance_hengxiang)
                            distance_zongxiang_list.append(distance_zongxiang)

                    else:
                        distance_hengxiang_list.append(50)
                        distance_zongxiang_list.append(50)
                        p2p_distance_linshi_list.append(50)

                # 取最小值
                distance_hengxiang = min(distance_hengxiang_list)
                distance_zongxiang = min(distance_zongxiang_list)


                if is_back == False:
                    dis_gap_cexiang_zongxiang = distance_zongxiang - (ego[4]) / 2  # 侧向车纵向距离

                    dis_gap_cexiang_hengxiang = distance_hengxiang - (ego[5]) / 2  # 侧向车横向距离
                else:
                    dis_gap_cexiang_zongxiang = distance_zongxiang - (cexiang_info[4]) / 2  # 侧向车纵向距离

                    dis_gap_cexiang_hengxiang = distance_hengxiang - (cexiang_info[5]) / 2  # 侧向车横向距离

                cv = cexiang_v
                cv_xy = cexiang_xy
                c_dis_gap = [dis_gap_cexiang_hengxiang, dis_gap_cexiang_zongxiang]  # 距离列表，横向、纵向
                c_yaw = cexiang_yaw

        # 后车检测
        if car_target == 2:

            back_ind = back_ind & y_ind & front_yaw_ind  # 综合考虑横纵向因素
            # print(327, back_ind, y_ind, front_yaw_ind, state)
            if back_ind.sum() > 0:  # 打印 True的数量
                state_ind = state[back_ind, :]
                # back = state_ind[np.abs(state_ind[:, 0] - ego[0]).argmin(), :]  # 最近的后车信息

                closest_index = np.argmin(np.linalg.norm(state_ind[:, :2] - ego[:2], axis=1))
                back = state_ind[closest_index]  # 最近的后车信息

                bv = back[2]  # 前车速度

                bv_xy = [back[0], back[1]]
                b_yaw = back[3]
                # 根据点斜式计算  横向距离 纵向距离
                distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(bv_xy, ego_xy,
                                                                                                   yaw_guiji_now)


                dis_gap_back = distance_zongxiang - (ego[4] + back[4]) / 2  # 后车距离

                cv = bv
                cv_xy = bv_xy
                c_dis_gap = dis_gap_back
                c_yaw = b_yaw

        # 改为存储它车长宽
        a_car_num = 0
        if cv_xy != 0:  # 存在目标车辆
            for i in state[1:, :]:
                if i[0] == cv_xy[0] and i[1] == cv_xy[1]:
                    a_car_num = [i[4], i[5]]
                    # print('最近前车', a_car_num, dis_gap)
                    break

        return v, cv, cv_xy, c_dis_gap, drving_direction, a_car_num, v_nearest, c_yaw

    # 定义一个基于ego车当前坐标提供全局道路中心线指引的方法
    def LineCenterGlobal(self, state):
        """
        根据当前车辆位置，在全局道路中心线中获取5个散点前/后的偏移指引
        """
        # 取出ego车当前位置
        ego_xy = [state[0][0], state[0][1]]
        # 采样点间隔参数
        sample_num = 5
        # 查看ego车在全局路径轨迹位置
        # 取出全局路径轨迹
        line_center_global = self.line_center_global_list
        # 计算每个点与查询点的距离
        distances = np.linalg.norm(ego_xy - line_center_global, axis=1)
        # 找到最近的点
        nearest_point = line_center_global[np.argmin(distances)]

        # 找到前向点作为全局道路中心线指引
        # 防止溢出

        # 修改为根据预瞄距离选取合适下一点位
        L = 0.0  # 初始化累加的路径长度 L

        # 计算前视距离，Lf = k * state.v + Lfc，这里 state.v 是车辆当前速度
        Lf = 0.1 * state[0][2] + self.pre_scn_distance

        # 最接近点距离索引
        nearest_point_index = np.argmin(distances)

        next_point_index = nearest_point_index
        # 从最近的路径点开始，向前搜索直到累加的路径长度 L 等于或超过前视距离 Lf
        while Lf > L and next_point_index < (len(distances) - 1):
            # 计算当前路径点和下一个路径点之间的距离
            x1, y1, x2, y2 = line_center_global[next_point_index][0], line_center_global[next_point_index][1], \
                line_center_global[next_point_index + 1][0], line_center_global[next_point_index + 1][1],
            distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            L += distance_i
            next_point_index += 1

        L = 0.0  # 初始化累加的路径长度 L
        last_point_index = nearest_point_index
        # 从最近的路径点开始，向后搜索直到累加的路径长度 L 等于或超过前视距离 Lf
        while Lf > L and last_point_index > 0:
            # 计算当前路径点和下一个路径点之间的距离
            x1, y1, x2, y2 = line_center_global[next_point_index][0], line_center_global[next_point_index][1], \
                line_center_global[next_point_index - 1][0], line_center_global[next_point_index - 1][1],
            distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            L += distance_i
            last_point_index -= 1

        # if (np.argmin(distances) + sample_num) <= len(distances) - 1:
        #     next_point_index = np.argmin(distances) + sample_num  # 没溢出向前取5个点
        # else:
        #     next_point_index = len(distances) - 1  # 溢出取边界点
        # if (np.argmin(distances) - sample_num) > 0:  # 没溢出向后取5个点
        #     last_point_index = np.argmin(distances) - sample_num
        # else:
        #     last_point_index = 0  # 溢出取边界点

        next_nearest_point = line_center_global[next_point_index]  # 下一采样点
        last_nearest_point = line_center_global[last_point_index]  # 上一采样点

        return ego_xy, next_nearest_point, last_nearest_point, nearest_point

    # 定义一个计算两点直接夹角的算法
    def calculate_angle(self, point_a, point_b):
        """
        计算二维空间中两点之间的正弧度值。

        参数:
        point_a -- 点A的坐标，形式为 (x1, y1)
        point_b -- 点B的坐标，形式为 (x2, y2)

        返回:
        正弧度值 -- 两点之间的角度，结果总是正数
        """
        # 计算两点之间的向量
        # dx, dy = point_b - point_a
        # 计算向量的dx和dy分量
        dx = point_b[0] - point_a[0]
        dy = point_b[1] - point_a[1]

        # 使用np.arctan2计算从点A到点B的弧度值
        # np.arctan2返回的角度范围是[-pi, pi]
        radians = np.arctan2(dy, dx)

        # 将弧度值转换为正数，范围在 [0, 2π)
        positive_radians = radians if radians >= 0 else 2 * np.pi + radians

        return positive_radians

    # yaw计算方法
    def yaw_deside(self, state, observation):
        """
        计算车辆转角yaw
        """
        # 车辆轴距
        ego_l = state[0, 4] / 1.7
        # 当前ego偏航角
        ego_yaw_now = state[0, 3]
        # 当前速度
        ego_v_now = state[0, 2]

        # 检测终点
        target_near, L = self.target_rash_deside(state)

        # 获取当前坐标与下一步全局坐标
        ego_xy, next_global_xy, last_global_xy, nearest_global_xy = self.LineCenterGlobal(state)
        nearest_point_distance = np.sqrt(
            (ego_xy[0] - nearest_global_xy[0]) ** 2 + (ego_xy[1] - nearest_global_xy[1]) ** 2)



        # 计算当前道路的回正角度
        yaw_center = self.calculate_angle(ego_xy, next_global_xy)
        yaw_center_guiji = self.calculate_angle(nearest_global_xy, next_global_xy)
        # 由于 yaw范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        yaw_center = self.other_yaw_yichu(ego_yaw_now, yaw_center)

        # 由于 yaw范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        yaw_center_guiji = self.other_yaw_yichu(ego_yaw_now, yaw_center_guiji)

        yaw_center_yuanshi = yaw_center

        # 限制角度变化
        yaw_dt_distance = 0.1 / (state[0, 4] / 4.5)  # 限制每个时刻最大侧偏距离，与车身长度关联
        yaw_center_limit = yaw_dt_distance / (self.dt) / (state[0][2] + 1e-6)
        yaw_center_limit = np.clip(yaw_center_limit, 0, 1)
        yaw_center_limit = np.arcsin(yaw_center_limit)  # 设置单 刷新时刻角度微调量只能为 1米

        yaw_center = np.clip(yaw_center, yaw_center_guiji-yaw_center_limit, yaw_center_guiji+yaw_center_limit)

        if L < 10:  # 不到10米下一点置为终点
            next_global_xy = self.target_xy
            yaw_center = self.calculate_angle(ego_xy, next_global_xy)



        # 由于 yaw范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        yaw_center = self.other_yaw_yichu(yaw_center_yuanshi, yaw_center)


        # 微调角度设置
        yaw_weitiao_canshu = 0.04 / (self.dt) / (state[0][2] + 1e-6)
        yaw_weitiao_canshu = np.clip(yaw_weitiao_canshu, 1e-6, 1)
        yaw_weitiao = np.arcsin(yaw_weitiao_canshu)  # 设置单 刷新时刻角度微调量只能为 1米

        # 防撞微调角度设置
        yaw_avoid_distance = 0.1 / (state[0, 4] / 4.5)  # 限制每个时刻最大侧偏距离，与车身长度关联
        yaw_weitiao_canshu_avoid = yaw_avoid_distance / (self.dt) / (state[0][2] + 1e-6)
        yaw_weitiao_canshu_avoid = np.clip(yaw_weitiao_canshu_avoid, 1e-6, 1)
        yaw_weitiao_avoid = np.arcsin(yaw_weitiao_canshu_avoid)  # 设置单 刷新时刻角度微调量只能为 1米

        # 车身与车道矫正 判断路心与前进方向的位置关系，左加右减
        if nearest_point_distance > 1:  # 车心与路心距离大于1时触发矫正, 并且车辆本身没有做变道操作
            # print(704, '车身矫正', next_global_xy)
            # 判定下一轨迹点相对位置，向其靠拢
            is_right = self.is_right_to_ego(state, (next_global_xy, ego_yaw_now))
            if is_right:  # 在本车右侧
                yaw_center = yaw_center - yaw_weitiao
                # print(1001, '车身右矫正', next_global_xy)
            else:  # 在本车左侧
                yaw_center = yaw_center + yaw_weitiao
                # print(1004, '车身左矫正', next_global_xy)

        # 车道内避免反复调方向，在车道中心一定范围内保持直向行驶
        if nearest_point_distance < 0.5:  # 路心的限定范围要大于 yaw_weitiaoliang
            yaw_center = self.calculate_angle(nearest_global_xy, next_global_xy)

        # 期望下一偏航角
        yaw_expect_next = yaw_center

        # 碰撞风险检测
        deside_avoid_acc, deside_avoid_direct_yaw, turn_guidance, yaw_back = self.deside_avoid(state)

        # 由于 yaw_center范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        if abs(yaw_expect_next - ego_yaw_now) > np.pi:  # 检测跳变
            if yaw_expect_next < ego_yaw_now:
                yaw_expect_next = yaw_expect_next + (np.pi * 2)
            elif yaw_expect_next > ego_yaw_now:
                ego_yaw_now = ego_yaw_now + (np.pi * 2)

        # 检查碰撞检测直接转角参数
        if deside_avoid_direct_yaw:
            if deside_avoid_direct_yaw > 0:  # 左偏
                yaw_expect_next = ego_yaw_now + yaw_weitiao_avoid
                # print(742, '防撞左偏', yaw_expect_next, yaw_weitiao_avoid)
            else:  # 右偏
                yaw_expect_next = ego_yaw_now - yaw_weitiao_avoid
                # print(864, '防撞右偏', yaw_expect_next, yaw_weitiao_avoid, ego_yaw_now)

        yaw_between = yaw_expect_next - ego_yaw_now

        # 纯轨迹跟踪公式
        # 计算前视距离，Lf = k * ego_v_now + self.pre_scn_distance，这里 ego_v_now 是车辆当前速度 self.pre_scn_distance 是前视距离
        Lf = 0.1 * ego_v_now + self.pre_scn_distance
        yaw = math.atan2(2.0 * ego_l * math.sin(yaw_between) / Lf, 1.0)
        if deside_avoid_direct_yaw:
            # print(845, '防撞调制', yaw)
            pass

        cepian_acc = 0  # 侧偏加速指引， 0 无指引； 1 已方向矫正
        # 禁止侧偏指引参数检测
        if turn_guidance == 1:  # 1 禁止右偏 2 禁止左偏
            if yaw < 0:
                yaw = math.atan2(2.0 * ego_l * math.sin(yaw_back - ego_yaw_now) / Lf, 1.0)
                cepian_acc = 0
                # print(848, '禁止右偏矫正', yaw)
        elif turn_guidance == 2:
            if yaw > 0:
                yaw = math.atan2(2.0 * ego_l * math.sin(yaw_back - ego_yaw_now) / Lf, 1.0)
                cepian_acc = 0
                # print(851, '禁止左偏矫正', yaw)

        return yaw, cepian_acc

    # 路网接续字典
    def luwang_successor_dict(self, root):
        """
        本方法用于从 xodr文件中读入所有道路的连接关系，并存储于字典 luwang_successor_dict中
        字典内容示例：
        '12': {'end': ['2', '-1'], 'start': ['1', '-1'], 'lane_id': '-1'}
        本道路road_id：{连接符：[连接道路road_id，连接细分道路lane_id]}
        """
        # 路网接续字典
        luwang_successor_dict = {}
        # 查找所有道路连接
        roads = root.findall('.//road')
        # 遍历所有道路
        for road in roads:
            road_id = road.get('id')
            # 查找当前道路的连接信息
            links = road.findall('.//link')

            # 将继承和后续道路取出
            predecessor_element_id = None
            predecessor_contact_point = None
            successor_element_id = None
            successor_contact_point = None

            # 遍历当前道路的连接信息并打印输出
            for link in links:
                # 将用于上述检查 lane_id置空
                lane_id = None

                predecessor = link.find('predecessor')
                successor = link.find('successor')
                if predecessor is not None:
                    element_id = predecessor.attrib.get('elementId')
                    contact_point = predecessor.attrib.get('contactPoint', 'N/A')

                    if (element_id is not None) and (contact_point is not 'N/A'):
                        # 记录 predecessor数据
                        predecessor_element_id = element_id
                        predecessor_contact_point = contact_point

                    #  取出细分道路的拼接信息
                    # 上溯找到父级元素 <link>
                    link = predecessor.find('..')
                    # 如果 <link> 元素有 'id' 属性，上溯找到父级元素 <lane>
                    lane = link.find('..')
                    # 避免上述到 road
                    if lane.tag == 'lane':
                        # 现在取出 <lane> 元素的 'id' 属性
                        lane_id = lane.attrib.get('id')

                    # 查询到细分路段
                    if lane_id is not None:
                        # 取出对向路 lane_id
                        object_lane_id = predecessor.attrib.get('id')

                        # 已建立链接符
                        if (predecessor_contact_point != None) and (predecessor_element_id != None):
                            # 添加细分道路信息到路网接续字典
                            if (road_id, lane_id) in luwang_successor_dict:  # 构建键值对，或者补充键值对
                                luwang_successor_dict[road_id, lane_id][predecessor_contact_point] = [
                                    predecessor_element_id]
                            else:
                                luwang_successor_dict[road_id, lane_id] = {
                                    predecessor_contact_point: [predecessor_element_id]}

                            # 指明接续道路的细分路段
                            if (road_id, lane_id) in luwang_successor_dict:
                                if predecessor_contact_point in luwang_successor_dict[road_id, lane_id]:
                                    luwang_successor_dict[road_id, lane_id][predecessor_contact_point].append(
                                        object_lane_id)

                if successor is not None:
                    element_id = successor.attrib.get('elementId')
                    contact_point = successor.attrib.get('contactPoint', 'N/A')
                    if (element_id is not None) and (contact_point is not 'N/A'):
                        # 记录 successor数据
                        successor_element_id = element_id
                        successor_contact_point = contact_point

                    #  取出细分道路的拼接信息
                    # 上溯找到父级元素 <link>
                    link = successor.find('..')
                    # 如果 <link> 元素有 'id' 属性，上溯找到父级元素 <lane>
                    lane = link.find('..')
                    # 避免上述到 road
                    if lane.tag == 'lane':
                        # 现在取出 <lane> 元素的 'id' 属性
                        lane_id = lane.attrib.get('id')

                    if lane_id is not None:
                        # 取出对向路 lane_id
                        object_lane_id = successor.attrib.get('id')

                        if (successor_contact_point != None) and (successor_element_id != None):
                            # 添加细分道路信息到路网接续字典
                            if (road_id, lane_id) in luwang_successor_dict:  # 构建键值对，或者补充键值对
                                luwang_successor_dict[road_id, lane_id][successor_contact_point] = [
                                    successor_element_id]
                            else:
                                luwang_successor_dict[road_id, lane_id] = {
                                    successor_contact_point: [successor_element_id]}

                            # 指明后续道路的细分路段
                            if (road_id, lane_id) in luwang_successor_dict:
                                if successor_contact_point in luwang_successor_dict[road_id, lane_id]:
                                    luwang_successor_dict[road_id, lane_id][successor_contact_point].append(
                                        object_lane_id)

        # 获取所有的<connection>元素
        connections = root.findall('.//junction/connection')

        # 遍历<connection>元素并提取信息
        for connection in connections:
            incoming_road = connection.get('incomingRoad')
            contactPoint = connection.get('contactPoint')
            connecting_road = connection.get('connectingRoad')
            lane_links = connection.findall('laneLink')
            # 打印连接信息
            for lane_link in lane_links:
                from_lane = lane_link.get('from')
                to_lane = lane_link.get('to')
                # if incoming_road in luwang_successor_dict:  # 如果道路未在接续字典中建立，则需要建立
                #     luwang_successor_dict[incoming_road][contactPoint] = [connecting_road, to_lane]
                # else:
                luwang_successor_dict[incoming_road, from_lane] = {contactPoint: [connecting_road, to_lane]}

        return luwang_successor_dict

    # 路网连接方案列表
    def luwang_list(self, luwang_successor_dict):
        """
        此方法用于将获取到的路网接续字典进一步解析为道路方案列表
        列表内容示例如下：
        [['2', '-1'], ['12', '-1'], ['1', '-1']]
        基于每条 road分析其前后的道路连接关系，将连接的 [road_id, lane_id]存放于列表
        """
        # luwang_list存储场景所有方案
        luwang_list = []
        # 对路网接续字典继续处理，输出全局道路接续信息，构建片段道路方案
        # 遍历路径接续外部字典

        for key, inner_dict in luwang_successor_dict.items():
            # 遍历内部字典
            lane_successor = []
            key = [key[0], str(key[1])]
            lane_successor.append(key)
            if 'end' in inner_dict:
                inner_dict_end = inner_dict['end']
                lane_successor.insert(0, inner_dict_end)
            if 'predecessor' in inner_dict:
                inner_dict_end = inner_dict['predecessor']
                lane_successor.insert(0, inner_dict_end)
            if 'start' in inner_dict:
                inner_dict_start = inner_dict['start']
                lane_successor.append(inner_dict_start)
            if 'successor' in inner_dict:
                inner_dict_start = inner_dict['successor']
                lane_successor.append(inner_dict_start)
            luwang_list.append(lane_successor)
        # 继续处理道路方案，将片段段落方案拼接
        luwang_list = self.lanes_global_plan(luwang_list)
        return luwang_list

    # 路网散点字典
    def luwang_point_dict(self, parsed_data):
        """
        此方法用于将所有道路的车道id、左、右、中心边界提取并保存于字典
        字典内容示例：

        """
        # 提取整体路网
        luwang_point_dict = {}  # 构建路网字典

        for discrete_lane in parsed_data.discretelanes:  # 遍历离散车道列表
            line_id = discrete_lane.lane_id  # 车道ID
            line_left = discrete_lane.left_vertices  # 左边界散点
            line_right = discrete_lane.right_vertices  # 右边界散点
            line_center = discrete_lane.center_vertices  # 中心线散点

            # 将车道ID作为字典的键，左边界、右边界和中心线散点组成的元组作为值
            luwang_point_dict[line_id] = (line_left, line_right, line_center)

        return luwang_point_dict

    # 起点/终点的坐标及所属道路判定
    def locate_Pos(self, luwang_point_dict, pointPos):
        """
        本方法用于定位 起点/终点坐标，并判定其所属道路
        """
        # 判断终点坐标
        # 取出终点中心坐标
        if all(isinstance(item, list) for item in pointPos):  # 终点由两点确定，起点仅为车辆质心
            pointPos = [((pointPos[0][0] + pointPos[1][0]) / 2),
                        ((pointPos[0][1] + pointPos[1][1]) / 2)]
        # 判定终点所属道路
        # 遍历所有中心散点，找寻与当前点距离最近点所属道路
        # value存放左、右、中心散点坐标
        distances_lanes_point = {}
        for key, value in luwang_point_dict.items():
            # 计算查询点与坐标数组中每个点之间的距离
            distances = np.linalg.norm(value[2] - pointPos, axis=1)
            # 找到距离最近的点的索引
            nearest_index = np.argmin(distances)
            distances_lanes_point[key] = distances[nearest_index]

        # 使用 items() 方法获取键值对列表，并用 min 函数找到最小值的键
        # min_value_key = min(distances_lanes_point.items(), key=lambda item: item[1])[0]
        min_value_list = []
        while True:
            if not distances_lanes_point:
                break
            min_value = []
            min_value.append(min(distances_lanes_point.items(), key=lambda item: item[1]))
            min_value_list.append(min_value[0])  # 存在可能属于的道路列表
            if min_value[0][1] > 4:  # 保存4米以内道路，为变道检测提供依据
                break
            # 移除最小值及其对应的键
            distances_lanes_point.pop(min_value[0][0])

        return pointPos, min_value_list

    # 生成全局路径轨迹
    def line_center_global(self, start_lane_id, target_lane_id, luwang_list, luwang_point_dict):
        """
        本方法用于基于起点和终点位置，在路网方案中搜索，并确定唯一全局路径轨迹
        """
        # 判定起点道路的接续方案
        if start_lane_id != target_lane_id:  # 判定起点终点是否为同一道路
            line_center_global = []  # 建立全局中心线轨迹列表
            start_lane_id_simple = [str(start_lane_id[0]), str(start_lane_id[2])]  # 取出起点 road_id lane_id
            target_lane_id_simple = [str(target_lane_id[0]), str(target_lane_id[2])]  # 取出终点 road_id lane_id

            luwang_list = sorted(luwang_list, key=lambda x: len(x))

            for plan in luwang_list:  # 遍历所有道路方案
                if (start_lane_id_simple in plan) and (target_lane_id_simple in plan):  # 严格搜索 road_id 与 lane_id均匹配方案
                    # line_center_global = []  # 严格匹配最优先，防止有其它道路方案存入生乱
                    line_plan = plan  # 取出可行方案
                    line_plan = line_plan[:]

                    for i in line_plan:  # 依次找出方案中心线
                        for key, value in luwang_point_dict.items():  # 遍历道路散点字典，找到可行方案中心线

                            key = [int(part) for part in key.split('.')]
                            key = [str(key[0]), str(key[2])]
                            if key == i:
                                def calculate_curvature(points):
                                    # 存储所有点的曲率
                                    curvatures = []

                                    for i in range(1, len(points) - 1):
                                        p1 = points[i - 1]
                                        p2 = points[i]
                                        p3 = points[i + 1]

                                        # 计算向量
                                        v1 = p1 - p2
                                        v2 = p3 - p2

                                        # 计算向量的夹角
                                        angle = np.arctan2(np.linalg.det([v1, v2]), np.dot(v1, v2))

                                        # 计算曲率
                                        curvature = 2 * np.sin(angle / 2) / np.linalg.norm(p3 - p1)
                                        curvatures.append(curvature)

                                    # 计算最大曲率
                                    average_max = max(curvatures)
                                    return average_max

                                curv = calculate_curvature(value[2])

                                x1, y1 = value[0][0][0],  value[0][0][1]
                                x2, y2 = value[1][0][0], value[1][0][1]
                                # 计算两点之间的距离(路宽)
                                distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                                # print(1394, key, curv, distance)
                                # 最大曲率小于0，凹段路，选择左边界为中心
                                if curv < 0 and distance > 5:
                                    line_center_global.append(value[0])  # 0 左边界 1 右边界 2 中心
                                else:
                                    line_center_global.append(value[2])  # 0 左边界 1 右边界 2 中心
                    break

            if line_center_global:
                # print(620, '道路方案为：', line_plan, '，已生成全局路径轨迹')
                self.line_center_global_plan_list.append(line_plan)

                line_center_global = np.vstack(line_center_global)  # 将中心线合并，最终得到全局行驶轨迹

        else:  # 同属一条道路，规划道路即为起点路段
            # print(627, '道路规划为：', start_lane_id)
            pass

        return line_center_global

    # 路网生成--重复路网切断
    def remove_duplicates_from_positions(self, original_list):
        """
        从重复位置切断列表，使得每个元素只出现一次。

        参数:
        original_list (list): 要处理的原始列表。

        返回:
        list: 去除重复元素后的新列表。
        """
        seen = set()  # 用于跟踪已经遇到的元素的集合
        result_list = []  # 用于存储结果的列表

        for item in original_list:
            # 将列表转换为元组，以便可以用于集合
            item_tuple = tuple(item)
            if item_tuple not in seen:
                result_list.append(item)
                seen.add(item_tuple)
            else:
                # 如果元素已经遇到，从当前位置开始切断列表
                break  # 跳出循环

        return result_list

    # 路网生成--片段路网拼接
    def lane_concat(self, unique_lists):
        """
        根据 xodr文件查询拼接关系符，对道路进行拼接组合
        """
        new_plan = []
        for i in unique_lists:
            i_cancat = False  # 无拼接路段
            for j in unique_lists:  # 查询除本路段之外，列表内是否有相接路段
                if i[0] == j[-1]:  # 有前向拼接路段
                    new_i_list = j + i[1:]
                    if new_i_list not in new_plan and new_i_list[::-1] not in new_plan:
                        # 判断是否有相同集合在列表中
                        have_same_new_i_list = False
                        for new_plan_i in new_plan:
                            if set(map(tuple, new_i_list)) == set(map(tuple, new_plan_i)):  # 如果有相同集合
                                have_same_new_i_list = True
                        if have_same_new_i_list == False:  # 没有相同集合
                            # if new_i_list[0] == new_i_list[-1]:  # 循环跳出
                            #     new_i_list = new_i_list[:-1]
                            new_i_list = self.remove_duplicates_from_positions(new_i_list)  # 重复切断
                            new_plan.append(new_i_list)
                        i_cancat = True  # 有拼接路段标识
                elif i[-1] == j[0]:  # 有后向拼接路段
                    new_i_list = i + j[1:]
                    if new_i_list not in new_plan and new_i_list[::-1] not in new_plan:
                        # 判断是否有相同集合在列表中
                        have_same_new_i_list = False
                        for new_plan_i in new_plan:
                            if set(map(tuple, new_i_list)) == set(map(tuple, new_plan_i)):  # 如果有相同集合
                                have_same_new_i_list = True
                        if have_same_new_i_list == False:
                            # if new_i_list[0] == new_i_list[-1]:  # 循环跳出
                            #     new_i_list = new_i_list[:-1]
                            new_i_list = self.remove_duplicates_from_positions(new_i_list)  # 重复切断
                            new_plan.append(new_i_list)
                        i_cancat = True
            if i not in new_plan and i[::-1] not in new_plan and i_cancat == False:  # 没有首尾相接的就直接存放
                new_plan.append(i)
            # unique_lists = new_plan
        return new_plan

    # 路网生成--循环找到全局路径方案
    def lanes_global_plan(self, list):
        """
        生成全局路径方案，找到最长路径后即止
        """
        new_plan = []
        # 去掉重复项
        for inner_list in list:
            if inner_list not in new_plan and inner_list[::-1] not in new_plan:
                # 检查是否存在父集，存在则不添加

                is_ordered_fatherset = False
                for new_plan_part in new_plan:
                    # 初始化一个索引变量
                    index = 0
                    for item in new_plan_part:
                        # 是否为子集
                        if index < len(inner_list) and item == inner_list[index]:
                            index += 1
                        if index == len(inner_list):
                            break
                    # 如果索引等于 a 的长度，则 a 是 b 的有序子集
                    is_ordered_fatherset = index == len(inner_list)
                    # 如果发现父集
                    if is_ordered_fatherset == True:
                        # 不添加子集

                        break

                # 检查是否存在子集，存在则删除子集
                is_ordered_subset = False
                for new_plan_part in new_plan:
                    # 初始化一个索引变量
                    index = 0
                    for item in inner_list:
                        # 是否为子集
                        if index < len(new_plan_part) and item == new_plan_part[index]:
                            index += 1
                        if index == len(new_plan_part):
                            break
                    # 如果索引等于 a 的长度，则 a 是 b 的有序子集
                    is_ordered_subset = index == len(new_plan_part)
                    # 如果发现子集
                    if is_ordered_subset == True:
                        # 删除子集
                        new_plan.remove(new_plan_part)

                        break

                # 没发现父集，直接添加
                if is_ordered_fatherset == False:
                    new_plan.append(inner_list)


        last_longest_sublist = None  # 循环停止符，最长道路搜索到后即停
        # 开始循环，不断叠加道路拼接
        if list:
            i = 0
            while True:
                i += 1

                new_plan = self.lane_concat(new_plan)
                longest_sublist = max(new_plan, key=len)
                new_longest_sublist = len(longest_sublist)
                if new_longest_sublist == last_longest_sublist:
                    break
                else:
                    last_longest_sublist = new_longest_sublist

        return new_plan

    # 计算点斜式，以及点到线的距离
    def distance_point_to_line(self, point_p2, point_p1, slope):
        """
        计算一个点到直线的距离，直线由直线上的一个点和斜率确定。

        参数:
        point_p2 (tuple): 不在直线上的点的坐标 (x_2, y_2)。
        point_p1 (tuple): 直线上的点的坐标 (x_1, y_1)。
        slope (float): 直线的斜率 (m)。

        返回:
        float: 点到直线的水平距离。
        float: 点到直线的纵向距离。
        """
        x1, y1 = point_p1
        x2, y2 = point_p2
        m = np.tan(slope)  # 斜率
        A = m  # 直线方程 Ax + By + C = 0 中的 A
        B = -1  # 直线方程中的 B
        C = -m * x1 + y1  # 直线方程中的 C

        # 使用点到直线的公式计算水平距离
        hengxiang_distance = abs(A * x2 + B * y2 + C) / math.sqrt(A ** 2 + B ** 2)
        # 计算两点之间的欧几里得距离
        p2p_distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        # 使用勾股定理计算点到直线的纵向距离
        zongxiang_distance = math.sqrt(p2p_distance ** 2 - hengxiang_distance ** 2)

        return hengxiang_distance, zongxiang_distance, p2p_distance

    # 变道检测算法，查询是否存在相邻车道、以及相邻车道是否具备借道条件
    def lane_change_ready(self, fv_xy):
        """
        变道条件检测算法：
        1. 查询前车当前所属道路（中心点最接近的车道）
        2. 查询是否存在相邻道路
        3. 查询相邻道路上的在途车是否存在碰撞风险（避免变道碰撞）
        4. 找出相邻道路中映射点
        5. 返回映射点作为下一运动目标
        """
        if self.type != 'SERIAL':
            # 下一映射点初始置空
            next_point_xy = []

            luwang_point = self.luwang_point  # 取出路网散点

            # 1. 查询前车当前所属道路（中心点最接近的车道）
            pointPos, min_value_list = self.locate_Pos(luwang_point, fv_xy)  # 利用 locate_pos方法 取出4米内车道列表

            # 2. 查询本车道是否有变道空间

            # 3. 查询是否存在相邻道路
            near_lane_list = []  # 构建相邻车道列表

            # 判定 可变车道类型 左车道 or 右车道
            # 判定 映射点前后10米有无它车
            # 需要根据邻车道 id信息取出映射点，用于指导自车向邻道偏移

            # 检查是否存在相邻车道
            if len(min_value_list) > 1:
                # 计算本车道道路方向
                # 根据车道 id得到该车道中心散点
                near_lane_center = luwang_point[min_value_list[0][0]][2]

                # 计算查询点与坐标数组中每个点之间的距离
                distances = np.linalg.norm(near_lane_center - pointPos, axis=1)

                # 找到距离最近的点的索引
                nearest_index = np.argmin(distances)
                nearest_point_xy = near_lane_center[nearest_index]  # 取出最接近点坐标

                # 计算本车道道路方向
                lane_yaw = self.calculate_angle(nearest_point_xy, near_lane_center[-1])

                # 检查相邻车道的同向性
                for i in range(1, len(min_value_list)):
                    # 根据车道 id得到该车道中心散点
                    near_lane_center = luwang_point[min_value_list[i][0]][2]

                    # 计算查询点与坐标数组中每个点之间的距离
                    distances = np.linalg.norm(near_lane_center - pointPos, axis=1)

                    # 找到距离最近的点的索引
                    nearest_index = np.argmin(distances)
                    nearest_point_xy = near_lane_center[nearest_index]  # 取出最接近点坐标

                    # 取出本车道id
                    lane_name_ego = [int(part) for part in min_value_list[0][0].split('.')]
                    lane_name_ego = [str(lane_name_ego[0]), str(lane_name_ego[2])]

                    # 计算相邻道路方向
                    lane_yaw_i = self.calculate_angle(nearest_point_xy, near_lane_center[-1])
                    if abs(lane_yaw_i - lane_yaw) <= (np.pi / 2):  # 判定是否同向
                        lane_name = [int(part) for part in min_value_list[i][0].split('.')]
                        lane_name = [str(lane_name[0]), str(lane_name[2])]
                        # 判断是否为 同road_id车道
                        if lane_name_ego[0] == lane_name[0]:
                            break

            else:  # 不存在相邻车道
                next_point_xy = []
        else:
            next_point_xy = []

        return next_point_xy

    # 碰撞检测算法
    def deside_avoid(self, state):
        """
        碰撞检测，
        前向碰撞检测：当前车存在前向碰撞风险时，采取 减速/变道
        侧向碰撞检测：与 10米内的最近侧向车保持 0.5米侧向间距
        """
        # 初始碰撞加速度/航向角置空
        deside_avoid_acc = []
        deside_avoid_cexiang_yaw = [100, 100]  # 初始化侧向碰撞转角
        yaw_weitiao_canshu = 0.2 / (state[0][2] * self.dt + 1e-6)
        yaw_weitiao_canshu = np.clip(yaw_weitiao_canshu, 1e-6, 1)
        yaw_weitiao = np.arcsin(yaw_weitiao_canshu)  # 设置单 刷新时刻角度微调量只能为 1米

        # 获取当前坐标与下一步全局坐标
        ego_xy, next_global_xy, last_global_xy, nearest_global_xy = self.LineCenterGlobal(state)
        yaw_guiji_now = self.calculate_angle(nearest_global_xy, next_global_xy)

        # 给出一个回正指引
        yaw_back = []

        # 初始化前后车距离
        front_car_distance = -1
        back_car_distance = -1

        turn_guidance = 0  # 变向指引，0 无指引；1 禁止右偏；2 禁止左偏，防止变向后发生碰撞，

        front_safe_distance = -1
        ego_yaw_now = state[0][3]  # 我车转角
        # 本车坐标
        ego_xy = [state[0][0], state[0][1]]

        # 前向碰撞风险检测
        # 前车数据
        v_1, fv_1, fv_1_xy, dis_gap_1, direction, a_car_num_1, v_nearest, c_yaw = self.getInformFront(state,
                                                                                                      1)  # 取出最近前车数据
        # 由于 yaw范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        c_yaw = self.other_yaw_yichu(ego_yaw_now, c_yaw)

        front_safe_distance = abs((fv_1 ** 2 - v_1 ** 2) / (2 * self.a_bound)) + 3
        # print(1149, '最近前车', a_car_num_1, '距离', dis_gap_1, '速度', fv_1)
        # 前车速度是否小于本车
        if (fv_1 < v_1 and (dis_gap_1 != -1)) or (-1 < dis_gap_1 < 3):  # (前车速度小 且前车距小于100) or 前车距离不足2米 才有碰撞风险
            # 计算与前车的安全距离
            # print(1180, '最近前车', a_car_num_1, '距离', dis_gap_1, '前车速度', fv_1, '本车速度', v_1, '安全距离为', front_safe_distance, '前车坐标', fv_1_xy, '前车角度', c_yaw)
            # 如果本车速度大于前车速度，才有碰撞风险
            if v_1 > fv_1:
                # print(1236, '最近前车', a_car_num_1, '距离', dis_gap_1, '前车速度', fv_1, '本车速度', v_1, '安全距离为',front_safe_distance)
                # 如果前车静止，借道/减速避让
                if fv_1 == 0:
                    if dis_gap_1 > 10:  # 前车距离大于10米
                        deside_avoid_acc = - self.a_bound  # 减速跟驰
                        # print(1606, '减速避障', fv_1_xy, deside_avoid_acc)
                    else:
                        # 检查变道条件
                        next_xy = self.lane_change_ready(fv_1_xy)
                        if len(next_xy) > 0:  # 存在变道条件
                            deside_avoid_xy = next_xy  # 记录避障坐标
                            # print(948, '绕道避让静止车辆', a_car_num_1, '当前车辆坐标', ego_xy, next_xy)
                        else:  # 无变道条件
                            deside_avoid_acc = - self.a_bound  # 减速跟驰
                            # print(951, '减速避让静止车辆', fv_1_xy, '当前车辆坐标', ego_xy)

                # 如果前车速度不为0，借道/减速避让
                elif dis_gap_1 < front_safe_distance:  # 车间距小于安全距离
                    # print(1255, '与车', a_car_num_1, '安全距离为', front_safe_distance, '实际距离为', dis_gap_1,'存在前向碰撞风险')
                    if dis_gap_1 > 15:  # 前车距离大于10米
                        deside_avoid_acc = - self.a_bound  # 减速跟驰
                        # print(1618, '减速避障', fv_1_xy, deside_avoid_acc)
                    else:  # 前车距离不足十米
                        # 检查变道条件
                        next_xy = self.lane_change_ready(fv_1_xy)
                        if len(next_xy) > 0:  # 存在变道条件
                            deside_avoid_xy = next_xy  # 记录避障坐标
                            # print(1142, '变道避障')
                        else:  # 无变道条件
                            if dis_gap_1 > 5 and front_safe_distance / dis_gap_1 < 2:  # 有一定距离做减速
                                deside_avoid_acc = - self.a_bound  # 减速跟驰
                                # print(1418, '减速避障', fv_1_xy, deside_avoid_acc)
                            else:  # 距离过小 强制避让
                                # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                is_right = self.is_right_to_ego(state, (fv_1_xy, ego_yaw_now))
                                if is_right:  # 在本车右侧
                                    deside_avoid_cexiang_yaw_i = [dis_gap_1, yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 左调
                                    # print(1476, '左调避让', ego_yaw_now - c_yaw)
                                    deside_avoid_acc = - self.a_bound  # 减速跟驰
                                else:  # 在本车左侧
                                    # 采取右调或者加/减速避让
                                    deside_avoid_cexiang_yaw_i = [dis_gap_1, -yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 左调
                                    # print(1660, '右调避让', fv_1_xy)
                                    deside_avoid_acc = - self.a_bound  # 减速跟驰


        # 侧向碰撞风险检测
        # 取出最近侧向车信息
        v_1, fv_3, fv_3_xy, dis_gap_3, direction, a_car_num_1, v_nearest, c_yaw = self.getInformFront(state,
                                                                                                      3)  # 取出最近侧向车数据
        # 由于 yaw范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        c_yaw = self.other_yaw_yichu(ego_yaw_now, c_yaw)
        deside_avoid_cexiang_other_yaw = -1  # 初始化侧向同它车转角
        if fv_3_xy != 0 and (dis_gap_3 != [0, 0]):  # 存在侧向车
            dis_gap_cexiang_hengxiang, dis_gap_cexiang_zongxiang = dis_gap_3[0], dis_gap_3[1]
            # print(1342, '最近侧向车', a_car_num_1, '横向距离', dis_gap_3[0], '纵向距离', dis_gap_3[1], '速度', fv_3,'坐标', fv_3_xy, '它车角度', c_yaw, '我车角度', ego_yaw_now)

            # 禁止向该侧变道检测
            if dis_gap_cexiang_hengxiang < 2:  # 如果横向不足 1米
                if dis_gap_cexiang_zongxiang < 20:  # 后车纵向距离不足 10米
                    # if abs(ego_yaw_now - c_yaw) < 0.5:  # 设置仅同向车影响变道决策
                    if direction == -1:  # 横向驾驶场景
                        if (0 <= ego_yaw_now <= np.pi / 2) or (3 / 2 * np.pi <= ego_yaw_now):  # 右向驾驶
                            if fv_3_xy[0] < ego_xy[0] or dis_gap_cexiang_zongxiang < 0:  # 后车
                                # 计算与后车的安全距离
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 > v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 0:  # (后车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, (fv_3_xy, ego_yaw_now))
                                    if is_right:
                                        # print(1186, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1225, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                            elif fv_3_xy[0] > ego_xy[0]:  # 前车
                                # 计算与前车的安全距离
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 < v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 0:  # (自车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, (fv_3_xy, ego_yaw_now))
                                    if is_right:
                                        # print(1846, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1853, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                        else:  # 左向驾驶
                            if fv_3_xy[0] > ego_xy[0] or dis_gap_cexiang_zongxiang < 0:  # 后车
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 > v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 3:  # (后车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, (fv_3_xy, ego_yaw_now))
                                    if is_right:
                                        # print(1186, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1546, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                            elif fv_3_xy[0] < ego_xy[0]:  # 前车
                                # 计算与前车的安全距离
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 < v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 0:  # (自车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, (fv_3_xy, ego_yaw_now))
                                    if is_right:
                                        # print(1888, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1895, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw

                    elif direction == 1:  # 纵向驾驶场景
                        if 0 <= ego_yaw_now <= np.pi:  # 上向驾驶
                            if fv_3_xy[1] < ego_xy[1] or dis_gap_cexiang_zongxiang < 0:  # 后车
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 > v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 0:  # (后车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, fv_3_xy)
                                    if is_right:
                                        # print(1186, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1248, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                            elif fv_3_xy[1] > ego_xy[1] or dis_gap_cexiang_zongxiang < 0:  # 前车
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 < v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 0:  # (自车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, fv_3_xy)
                                    if is_right:
                                        # print(1186, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1248, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                        else:  # 下向驾驶
                            if fv_3_xy[1] > ego_xy[1] or dis_gap_cexiang_zongxiang < 0:  # 后车
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 > v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 0:  # (后车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, fv_3_xy)
                                    if is_right:
                                        # print(1186, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1259, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                            elif fv_3_xy[1] < ego_xy[1] or dis_gap_cexiang_zongxiang < 0:  # 前车
                                change_safe_distance = abs((fv_3 ** 2 - v_1 ** 2) / (2 * self.a_bound))
                                if ((fv_3 < v_1) and (
                                        change_safe_distance > dis_gap_cexiang_zongxiang)) or dis_gap_cexiang_zongxiang < 0:  # (自车速度大,且无安全变道距离) or 纵向距离已不足
                                    # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                    is_right = self.is_right_to_ego(state, fv_3_xy)
                                    if is_right:
                                        # print(1186, '禁止右偏')
                                        turn_guidance = 1
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw
                                    else:
                                        # print(1248, '禁止左偏')
                                        turn_guidance = 2
                                        if abs(c_yaw - ego_yaw_now) < np.pi / 2:  # 异向车方向矫正
                                            yaw_back = c_yaw
                                        else:
                                            yaw_back = np.pi + c_yaw

            # 侧向微调矫正
            if dis_gap_cexiang_hengxiang < 1:  # 不到1米，微调侧偏
                # 与本车纵向距离是否不到 10米
                if (dis_gap_cexiang_zongxiang < 15 and v_1 > 25) or (dis_gap_cexiang_zongxiang < 5):
                    # 不到 10米继续检测侧向距离是否在 0.5米以上
                    if dis_gap_cexiang_hengxiang < 1:
                        # print(1177, '最近侧向车', a_car_num_1, '横向距离', dis_gap_3[0], '纵向距离', dis_gap_3[1], '速度', fv_3)
                        # 侧向碰撞风险参数
                        # 判定侧向车相对位置，向另一边微调拉开侧向距离
                        is_right = self.is_right_to_ego(state, fv_3_xy)
                        if is_right:  # 在本车右侧

                            if dis_gap_cexiang_zongxiang < 0:  # 贴近车
                                if v_1 > fv_3 and (dis_gap_1 == -1 or (dis_gap_1 > front_safe_distance)):
                                    deside_avoid_acc = self.a_bound
                                    # print(1768, '贴近车加速')
                                else:
                                    if v_1 > fv_3 / 2:  # 避免过度减速
                                        deside_avoid_acc = -self.a_bound
                                        # print(1626, '贴近车减速')
                        else:  # 在本车左侧

                            if dis_gap_cexiang_zongxiang < 0:  # 贴近车
                                if v_1 > fv_3 and (dis_gap_1 == -1 or (dis_gap_1 > front_safe_distance)):
                                    deside_avoid_acc = self.a_bound
                                    # print(1609, '贴近车加速', v_1, fv_3)
                                else:
                                    if v_1 > fv_3 / 2:  # 避免过度减速
                                        deside_avoid_acc = -self.a_bound
                                        # print(1642, '贴近车减速', v_1, fv_3)

                    # 如果 纵向距离小于0，且它车向我车侧偏，则实施避让
                    if dis_gap_cexiang_zongxiang < 0:
                        # 检测终点
                        target_near, L = self.target_rash_deside(state)
                        if (v_1 - fv_3 < 0) and abs(ego_yaw_now - c_yaw) < 0.3 and v_1 > 1 \
                                and dis_gap_cexiang_hengxiang < 1 and L > 50:  # 距离相近，自车减速
                            deside_avoid_acc = -self.a_bound
                            # print(1716, '邻车减速')
                        # 判断是否向我车侧偏
                        ego_yaw_now = state[0][3]  # 我车转角
                        other_yaw_now = c_yaw
                        # 弧度大于 0.2，判定会有后续侧向碰撞风险
                        if abs(ego_yaw_now - other_yaw_now) > 0.2:

                            # 判定侧向车相对位置，向另一边微调拉开侧向距离
                            is_right = self.is_right_to_ego(state, fv_3_xy)
                            if is_right:  # 在我车右侧，角度大于我车存在侧撞风险
                                if 0.5 > other_yaw_now - ego_yaw_now > 0.1:  # 大于 0.1弧度开始响应
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 左调  # 左调
                                    # print(1351, '左调')
                                elif 0.5 <= other_yaw_now - ego_yaw_now:  # 大于 0.5弧度 视为交叉车道来车，向来车方向调弯
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, -yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 左调  # 右调
                                    # print(1354, '右调')
                            else:
                                if -0.5 < other_yaw_now - ego_yaw_now < 0.1:
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, -yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 左调  # 右调
                                    # print(1358, '右调避让', other_yaw_now, ego_yaw_now)
                                elif -0.5 >= other_yaw_now - ego_yaw_now:  # 大于 0.5弧度 视为交叉车道来车，向来车方向调弯
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 左调  # 左调
                                    # print(1361, '左调避让', other_yaw_now, ego_yaw_now)


        # 取出最近侧向车信息
        v, fv_4, fv_4_xy, dis_gap_4, direction, a_car_num_1, v_nearest, c_yaw_4 = self.getInformFront(state,
                                                                                                      4)  # 取出最近侧向车数据
        # 它车角度矫正
        # 由于 yaw范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        c_yaw_4 = self.other_yaw_yichu(ego_yaw_now, c_yaw_4)

        if fv_4_xy != 0:  # 判定最近侧向是否为交叉来车
            dis_gap_cexiang_hengxiang, dis_gap_cexiang_zongxiang = dis_gap_4[0], dis_gap_4[1]
            # print(1578, '最近交叉侧向车', a_car_num_1, '横向距离', dis_gap_4[0], '纵向距离', dis_gap_4[1], '速度', fv_4, '坐标', fv_4_xy,  '它车角度', c_yaw_4, '我车角度', ego_yaw_now)
            # 计算和交叉侧向车有无碰撞风险
            # 综合横纵向距离，如果满加速的情况下可以冲过去，则加速
            acc_can = self.cexiang_acc_deside(state, dis_gap_cexiang_hengxiang, dis_gap_cexiang_zongxiang,
                                              v, fv_4)
            if acc_can == False:
                # print(1585, '和交叉侧向车存在碰撞风险', '横向距离', dis_gap_4[0], '纵向距离', dis_gap_4[1], '速度', fv_4,   '坐标', fv_4_xy, '它车角度', c_yaw_4, '我车角度', ego_yaw_now)
                pass
            # 与本车横向距离是否不到 10米
            if dis_gap_cexiang_hengxiang < 30:
                if dis_gap_cexiang_zongxiang < 20:  # 纵向距离小于 10
                    # 只检测前向车
                    is_back = self.is_back_to_ego(state, direction, fv_4_xy)
                    # print(1701, is_back, fv_4_xy, ego_xy)
                    if is_back == False or dis_gap_cexiang_zongxiang < 0:  # 非后向车 or 纵向距离不足
                        # 判定侧向车相对位置，向另一边微调拉开侧向距离
                        is_right = self.is_right_to_ego(state, (fv_4_xy, ego_yaw_now))
                        # print(1676, is_right, ego_yaw_now, c_yaw_4, abs(ego_yaw_now - c_yaw_4))
                        if is_right:  # 在前进方向右侧 or 横向小于0（前车位置）
                            # print(1325, abs(ego_yaw_now - c_yaw))
                            if (ego_yaw_now < c_yaw_4 or dis_gap_cexiang_hengxiang < 0) and abs(ego_yaw_now - c_yaw_4) < 2.5:  # 它车角度大，向我车驶来
                                # 具备纵向距离，且侧向车速度在小于我车5以上,
                                # 综合横纵向距离，如果满加速的情况下可以冲过去，则加速
                                acc_can = self.cexiang_acc_deside(state, dis_gap_cexiang_hengxiang,
                                                                  dis_gap_cexiang_zongxiang,
                                                                  v, fv_4)
                                # print(1950, acc_can, dis_gap_cexiang_hengxiang, dis_gap_cexiang_zongxiang, v_1, fv_4, dis_gap_1, front_safe_distance)
                                if ((((acc_can == False) and fv_4 > 1) or (dis_gap_1 < front_safe_distance * 2 and dis_gap_1 != -1)) and \
                                        dis_gap_cexiang_zongxiang > 0) or (dis_gap_cexiang_hengxiang < 0 and fv_4 > 1) \
                                        or (a_car_num_1[0] < 1.5 and a_car_num_1[0] < 1.5):
                                    if acc_can and fv_1 > 0:  # 具备加速侧向条件，速度降至前车速
                                        if v > fv_1:
                                            deside_avoid_acc = - self.a_bound  # 减速跟驰
                                            # print(1725, '减速避障', fv_4, v_1, ego_yaw_now, c_yaw_4,  dis_gap_cexiang_hengxiang)
                                    elif v > 0.3:  # 避免过度减速
                                        deside_avoid_acc = - self.a_bound  # 减速跟驰
                                        # print(1692, '减速避障', fv_4, v_1, ego_yaw_now, c_yaw_4, dis_gap_cexiang_hengxiang)

                                else:
                                    # 避免过度加速 or 纵向不足
                                    if v < fv_4 * 3:
                                        deside_avoid_acc = self.a_bound  # 加速跟驰
                                        # print(1620, '加速避障', fv_4, v)

                                # 如果横向小于 半个车长，纵向小于1，急偏
                                if (a_car_num_1[0] > 1.5 and a_car_num_1[0] > 1.5):
                                    if dis_gap_cexiang_hengxiang < (state[0][4] + 3) and dis_gap_cexiang_zongxiang < 5:
                                        deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_hengxiang, yaw_weitiao]
                                        deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                                   0] <= \
                                                                                               deside_avoid_cexiang_yaw_i[
                                                                                                   0] else deside_avoid_cexiang_yaw_i  # 左调  # 左调

                                        # print(2020, '左调', yaw_weitiao)
                            elif abs(ego_yaw_now - c_yaw_4) > 2.5 and dis_gap_cexiang_hengxiang < 1:  # 异向来车
                                def is_b_left_of_a(a, b):
                                    # 标准化弧度a和b到[0, 2π]区间内
                                    a_normalized = a % (2 * math.pi)
                                    b_normalized = b % (2 * math.pi)

                                    # 弧度a和b之间的差值
                                    delta = (b_normalized - a_normalized) % (2 * math.pi)

                                    # 如果差值小于π，则b在a的左侧
                                    if delta < math.pi:
                                        return True
                                    else:
                                        return False
                                is_b_left_of_a = is_b_left_of_a(ego_yaw_now, c_yaw_4)
                                is_right_next = self.is_right_to_ego(state, (next_global_xy, ego_yaw_now))

                                if is_b_left_of_a == True:  # 它车向本车左向驾驶 and 下一轨迹点在右
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, -yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 右调
                                    # print(2104, '右调')
                                else:
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 右调
                                    # print(1650, '左调')
                        else:  # 在前进方向左侧
                            if ((ego_yaw_now > c_yaw_4 or dis_gap_cexiang_hengxiang < 0) and abs(ego_yaw_now - c_yaw_4) < 2.5):  # 它车角度小，向我车驶来
                                # 综合横纵向距离，如果满加速的情况下可以冲过去，则加速
                                acc_can = self.cexiang_acc_deside(state, dis_gap_cexiang_hengxiang,
                                                                  dis_gap_cexiang_zongxiang, v_1, fv_4)
                                # print(1684, acc_can, dis_gap_cexiang_zongxiang, dis_gap_1, front_safe_distance)

                                # 无法加速，or 前车距离不足
                                if ((((acc_can == False) and fv_4 > 1) or (dis_gap_1 < front_safe_distance * 2 and dis_gap_1 != -1)) \
                                        and dis_gap_cexiang_zongxiang > 0) or dis_gap_cexiang_hengxiang < 0 or (a_car_num_1[0] < 1.5 and a_car_num_1[0] < 1.5):
                                    if acc_can and fv_1 > 0:  # 具备加速侧向条件，速度降至前车速
                                        if v > fv_1:
                                            deside_avoid_acc = - self.a_bound  # 减速跟驰
                                            # print(1765, '减速避障', fv_4, v_1, ego_yaw_now, c_yaw,dis_gap_cexiang_hengxiang)
                                    elif v > 1:  # 避免过度减速
                                        deside_avoid_acc = - self.a_bound  # 减速跟驰
                                        # print(1769, '减速避障', fv_4, v_1, ego_yaw_now, c_yaw,  dis_gap_cexiang_hengxiang)

                                else:
                                    if dis_gap_cexiang_hengxiang > 0:
                                        deside_avoid_acc = self.a_bound  # 加速跟驰
                                        # print(1649, '加速避障', dis_gap_1, dis_gap_4[1], dis_gap_4[0])

                                # 如果横向小于 半个车长，纵向小于1，急偏
                                if (a_car_num_1[0] > 1.5 and a_car_num_1[0] > 1.5):
                                    if dis_gap_cexiang_hengxiang < (
                                            state[0][4] / 2 + 0.2) and dis_gap_cexiang_zongxiang < 5:
                                        deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_hengxiang, -yaw_weitiao]
                                        deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                                   0] <= \
                                                                                               deside_avoid_cexiang_yaw_i[
                                                                                                   0] else deside_avoid_cexiang_yaw_i  # 右调
                                        # print(1659, '右调')

                            # 异向来车，它车向左驾驶--右调  向右驾驶--左调
                            elif abs(ego_yaw_now - c_yaw_4) > 2.5 and dis_gap_cexiang_hengxiang < 1:  # 异向来车

                                acc_can = self.cexiang_acc_deside(state, dis_gap_cexiang_hengxiang,
                                                                  dis_gap_cexiang_zongxiang, v_1, fv_4)
                                # print(2194, acc_can, dis_gap_cexiang_zongxiang, dis_gap_1, front_safe_distance)

                                # 无法加速，or 前车距离不足
                                if ((((acc_can == False) and fv_4 > 1) or (
                                        dis_gap_1 < front_safe_distance * 2 and dis_gap_1 != -1)) \
                                    and dis_gap_cexiang_zongxiang > 0) or dis_gap_cexiang_hengxiang < 0 or (
                                        a_car_num_1[0] < 1.5 and a_car_num_1[0] < 1.5):
                                    if acc_can and fv_1 > 0:  # 具备加速侧向条件，速度降至前车速
                                        if v > fv_1:
                                            deside_avoid_acc = - self.a_bound  # 减速跟驰
                                            # print(1765, '减速避障', fv_4, v_1, ego_yaw_now, c_yaw,dis_gap_cexiang_hengxiang)
                                    elif v > 1:  # 避免过度减速
                                        deside_avoid_acc = - self.a_bound  # 减速跟驰
                                        # print(1769, '减速避障', fv_4, v_1, ego_yaw_now, c_yaw,  dis_gap_cexiang_hengxiang)

                                else:
                                    if dis_gap_cexiang_hengxiang > 0:
                                        deside_avoid_acc = self.a_bound  # 加速跟驰
                                        # print(1649, '加速避障', dis_gap_1, dis_gap_4[1], dis_gap_4[0])


                                def is_b_left_of_a(a, b):
                                    # 标准化弧度a和b到[0, 2π]区间内
                                    a_normalized = a % (2 * math.pi)
                                    b_normalized = b % (2 * math.pi)

                                    # 弧度a和b之间的差值
                                    delta = (b_normalized - a_normalized) % (2 * math.pi)

                                    # 如果差值小于π，则b在a的左侧
                                    if delta < math.pi:
                                        return True
                                    else:
                                        return False

                                is_b_left_of_a = is_b_left_of_a(ego_yaw_now, c_yaw_4)

                                is_right_next = self.is_right_to_ego(state, (next_global_xy, ego_yaw_now))

                                if is_b_left_of_a == True:  # 它车向本车左向驾驶 and 下一轨迹点在右
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, -yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 右调
                                    # print(2145, '右调')
                                else:
                                    deside_avoid_cexiang_yaw_i = [dis_gap_cexiang_zongxiang, yaw_weitiao]
                                    deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                               0] <= \
                                                                                           deside_avoid_cexiang_yaw_i[
                                                                                               0] else deside_avoid_cexiang_yaw_i  # 右调
                                    # print(2152, '左调')
                        # 如果横向距离小于0 ，处于前车位置

        # 后向碰撞风险检测
        # 取出最近后车信息
        v, bv, bv_xy, dis_gap_b, direction, a_car_num_1, v_nearest, c_yaw = self.getInformFront(state, 2)  # 取出最近侧向车数据
        # 由于 yaw范围在【0， 2Π】,为变 “0.0015260841420270683 6.277”这样的巨大偏差，增加一个控制方法
        c_yaw = self.other_yaw_yichu(ego_yaw_now, c_yaw)
        if bv_xy != 0:  # 存在后车
            # print(1472, '最近后向车', a_car_num_1, '距离', dis_gap_b, '速度', bv, '坐标', bv_xy)
            # 后车速度是否大于本车
            if (bv > v and (dis_gap_b != -1)) or dis_gap_b < 0:  # 后车速度大 且后车距小于100 才有碰撞风险
                # 计算与后车的安全距离
                back_safe_distance = abs((bv ** 2 - v ** 2) / (2 * self.a_bound)) + 2
                # print(1468, '最近后车', a_car_num_1, '距离', dis_gap_b, '后车速度', bv, '本车速度', v, '安全距离为', back_safe_distance)

                # 借道/加速避让
                if dis_gap_b < back_safe_distance:  # 车间距小于安全距离
                    # print(1451, '与车', a_car_num_1, '安全距离为', back_safe_distance, '实际距离为', dis_gap_b, '存在后向碰撞风险')
                    # 优先加速避撞
                    # (无前车 or 前车在安全距离内)，and 如果后车不足2米 and 后车安全距离远大于实际距离，优先变道
                    if ((dis_gap_1 == -1) or (front_safe_distance < dis_gap_1)) and (dis_gap_b > 2) and (
                            back_safe_distance < dis_gap_b * 2):  # 无前车直接加速
                        # exv = bv * 1.5  # 后车的 1.2倍速作为新的期望速度
                        # deside_avoid_acc = self.a * (1 - (v / (exv + 1e-6)) ** self.gama - ((dis_gap_b / (back_safe_distance + 1e-6)) ** 2))
                        deside_avoid_acc = self.a_bound
                        # print(1237, '加速避让后车', deside_avoid_acc)
                    # 无法加速避让，与后车不到十米时，变道避让
                    else:
                        # 无前车 or (前车安全 or 后车距离更小)
                        if ((dis_gap_1 == -1 or (front_safe_distance < dis_gap_1 or back_safe_distance > front_safe_distance
                            or dis_gap_b < dis_gap_1)) or (dis_gap_b < 0)) and v < bv:
                            deside_avoid_acc = self.a_bound
                            # print(1391, '无加速避让后车条件', dis_gap_1, front_safe_distance, back_safe_distance, dis_gap_b)
                        if dis_gap_b < 10:  # 后车靠近才做变道处理
                            # 检查变道条件
                            next_xy = self.lane_change_ready(bv_xy)
                            if len(next_xy) > 0:  # 存在变道条件
                                deside_avoid_xy = next_xy  # 记录避障坐标
                                # print(1254, '变道避障')
                            else:  # 无变道条件
                                # 强制侧偏避让
                                # print(1257, '无变道条件')
                                # 判断是否向我车侧偏
                                ego_yaw_now = state[0][3]  # 我车转角
                                other_yaw_now = c_yaw
                                # 判定侧向车相对位置，向另一边微调拉开侧向距离
                                is_right = self.is_right_to_ego(state, (bv_xy, ego_yaw_now))
                                if is_right:  # 在我车右侧，角度大于我车存在侧撞风险
                                    if 0.5 > other_yaw_now - ego_yaw_now > 0.1:  # 大于 0.1弧度开始响应
                                        deside_avoid_cexiang_yaw_i = [dis_gap_b, yaw_weitiao]
                                        deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                                   0] <= \
                                                                                               deside_avoid_cexiang_yaw_i[
                                                                                                   0] else deside_avoid_cexiang_yaw_i  # 左调  # 左调
                                        # print(1519, '左调避让', other_yaw_now, ego_yaw_now)
                                    elif 0.5 <= other_yaw_now - ego_yaw_now:  # 大于 0.5弧度 视为交叉车道来车，向来车方向调弯
                                        deside_avoid_cexiang_yaw_i = [dis_gap_b, -yaw_weitiao]
                                        deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                                   0] <= \
                                                                                               deside_avoid_cexiang_yaw_i[
                                                                                                   0] else deside_avoid_cexiang_yaw_i  # 右调
                                        # print(2036, '右调避让', other_yaw_now, ego_yaw_now)
                                else:
                                    if -0.5 < other_yaw_now - ego_yaw_now < 0.1:
                                        deside_avoid_cexiang_yaw_i = [dis_gap_b, -yaw_weitiao]
                                        deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                                   0] <= \
                                                                                               deside_avoid_cexiang_yaw_i[
                                                                                                   0] else deside_avoid_cexiang_yaw_i  # 右调
                                        # print(1188, '右调避让', other_yaw_now, ego_yaw_now)
                                    elif -0.5 >= other_yaw_now - ego_yaw_now:  # 大于 0.5弧度 视为交叉车道来车，向来车方向调弯
                                        deside_avoid_cexiang_yaw_i = [dis_gap_b, yaw_weitiao]
                                        deside_avoid_cexiang_yaw = deside_avoid_cexiang_yaw if deside_avoid_cexiang_yaw[
                                                                                                   0] <= \
                                                                                               deside_avoid_cexiang_yaw_i[
                                                                                                   0] else deside_avoid_cexiang_yaw_i  # 左调
                                        # print(1193, '左调避让', other_yaw_now, ego_yaw_now)

        # 存在侧向碰撞转弯避让
        if deside_avoid_cexiang_yaw[0] != 100:
            deside_avoid_direct_yaw = deside_avoid_cexiang_yaw[1]
        else:
            deside_avoid_direct_yaw = []

        return deside_avoid_acc, deside_avoid_direct_yaw, turn_guidance, yaw_back

    # 超车算法
    def deside_overtake(self):
        pass

    # 动力学校验模型
    def donglixue(self, v, acc):
        '''
        根据动力学校验矩阵，输入当前车速以及加速度，返回前轮转角的上限值
        '''

        donglixue_array = np.array([
            [-3.35431236e-03, 1.42284382e-02, 2.79720280e-05, 1.63273193e-01],  # 0
            [-1.45861780e-03, 6.40035135e-03, -2.09603384e-03, 1.69082016e-01],  # 1
            [-3.33366150e-04, 1.60147989e-03, -2.88191954e-03, 2.44960124e-01],  # 2
            [-1.41726256e-03, 3.09314057e-03, 2.77103901e-02, 1.58457167e-01],  # 3
            [9.39366256e-04, 3.46103445e-04, -2.98097490e-02, 3.24328128e-01],  # 4
            [5.22101664e-03, -1.87957792e-02, -9.59731475e-02, 6.32821893e-01],  # 5
            [0.00152473, 0.00837515, -0.0091032, 0.16938518],  # 6
            [-0.00049314, -0.00027708, 0.0115196, 0.21224637],  # 7
            [0.00046576, -0.00353305, -0.00317358, 0.22933939],  # 8
            [-0.0002798, -0.00197166, 0.0188549, 0.17730063],  # 9
            [0.00237302, -0.00322483, -0.015194, 0.21238009],  # 10
            [4.94185198e-05, 1.237461e-03, 2.91569953e-02, 1.52183878e-01],  # 11
            [0.00116847, 0.00421882, 0.00830538, 0.09531109],  # 12
            [2.91266465e-05, -2.09683751e-03, 8.44367019e-03, 1.19489320e-01],  # 13
            [-0.00053552, -0.00126393, 0.02546364, 0.12794865],  # 14
            [-0.00016977, -0.00050623,  0.01302406,  0.09446553],  # 15
            [-0.00085092, -0.00179417, 0.0166201, 0.07525603],  # 16
            [-0.00016969, -0.00093033, 0.01236247, 0.07906742],  # 17
            [-0.0003017, -0.00090315, 0.01058231, 0.06936233],  # 18
            [-0.00024213, -0.00108102, 0.0104131, 0.06495766],  # 19
            [-0.00068261, -0.0013469, 0.01619044, 0.06614042],  # 20
            [-0.00026165, -0.00070753, 0.00955285, 0.05456499],  # 21
            [-8.27073842e-05, -2.16264379e-04, 6.67225402e-03, 4.61989418e-02],  # 22
            [-0.00020339, -0.00046021,  0.00690765,  0.03998663],  # 23
            [-0.00012057, -0.00014606,  0.0034589,   0.02945015],  # 24
            [-0.00014971, -0.00014764, 0.00516091, 0.03111249],  # 25
            [-0.00019656, -0.00023524, 0.00604896, 0.03163061],  # 26
            [-1.52139835e-04, -3.95941328e-04, 4.61039602e-03, 3.05562802e-02],  # 27
            [-0.00011341, -0.00030362, 0.00402034, 0.02637378],  # 28
            [5.72949467e-05, 2.66810257e-05, 2.15670581e-03, 2.20466192e-02],  # 29
            [1.86236184e-05, 1.52376355e-04, 1.77526914e-03, 1.73360257e-02],  # 30
            [-5.70192891e-05, -2.37468745e-04, 2.45279994e-03, 1.96644391e-02],  # 31
            [-1.72187768e-04, -3.58234220e-05, 4.08516639e-03, 1.71174019e-02],  # 32
            [-2.09773147e-05, -3.97103328e-04, 1.59474442e-03, 2.00487386e-02],  # 33
            [-6.03195684e-05, -2.65501634e-04,  2.43412751e-03  ,1.76076813e-02],  # 34
            [-1.04064623e-05, -2.83202183e-04,  1.41420404e-03,  1.77318536e-02],  # 35
            [-1.17317738e-04 ,-3.39793311e-05 , 3.38697504e-03 , 1.46760844e-02],  # 36
            [0.00010088, -0.0002886 , -0.00023466,  0.01594034],  # 37
            [-5.02478335e-05 ,-1.52524625e-04 , 2.56664407e-03,  1.46245884e-02],  # 38
            [0.00015589, -0.00028153, -0.00101158, 0.01524676],  # 39
            [-4.78717668e-05, -3.36432053e-04,  2.17904206e-03,  1.46610952e-02],  # 40
            [  0.00014253, -0.00044466 ,-0.00099249 , 0.01586462],  # 41
            [ 5.08623083e-05 ,-5.26626665e-04,  6.22185589e-04,  1.65864593e-02],  # 42
            [ -3.82306912e-05, -5.82176520e-04,  1.98277767e-03,  1.82367216e-02],  # 43
            [1.24903409e-04, -3.57657596e-04, -4.06527627e-05, 1.49533474e-02],  # 44
            [1.10849947e-04, -3.30658146e-04, 1.36107042e-04, 1.39265125e-02],  # 45
            [7.64803043e-05, -5.15361054e-04, 2.19006924e-04, 1.47807516e-02],  # 46
            [7.66856606e-05, -3.93434020e-04, 3.63102508e-04, 1.40967150e-02],  # 47
            [6.37102187e-05, -3.82867871e-04, 4.94344293e-04, 1.34602867e-02],  # 48
            [4.88668801e-05, -2.54185434e-04, 6.82891607e-04, 1.15585571e-02],  # 49
            [6.23755131e-05, -2.84280605e-04, 3.91796054e-04, 1.16980684e-02],  # 50
            [4.29374815e-05, -2.72001292e-04, 6.55436151e-04, 1.10657161e-02],  # 51
            [4.38688003e-05, -2.47030375e-04, 6.67808696e-04, 1.08303990e-02],  # 52
            [6.15429314e-05, -2.14123694e-04, 3.04885891e-04, 1.02355544e-02],  # 53
            [3.64986306e-05, -2.36303752e-04, 6.02388916e-04, 9.88346400e-03],  # 54
            [5.34636349e-05, -1.96147326e-04, 2.71528690e-04, 9.40682858e-03]  # 55
        ]
        )
        # 计算公式
        v_yuanshi = v
        v = round(v)  # 将速度取整
        if v > 55:
            v = 55  # 临时处理大于55速的情况
        row = donglixue_array[v]  # 取出对应行
        a, b, c, d = row[0], row[1], row[2], row[3]  # 取出系数

        yaw = a * acc ** 3 + b * acc ** 2 + c * acc + d  # 计算 yaw

        return yaw

    # 查询它车在本车前进方向的左右
    def is_right_to_ego(self, state, point):
        """
        构建矢量线，判断它车与本车的左右关系
        """
        # 获取当前坐标与下一步全局坐标
        ego_xy, next_global_xy, last_global_xy, nearest_global_xy = self.LineCenterGlobal(state)
        yaw_center_guiji = self.calculate_angle(nearest_global_xy, next_global_xy)

        x1, y1 = nearest_global_xy[0], nearest_global_xy[1]
        theta = yaw_center_guiji
        line_length = 0.1

        if type(point) == tuple:
            theta = point[1]
            point = point[0]
            x1, y1 = ego_xy[0], ego_xy[1]

        # 计算矢量线的终点 (x2, y2)
        x2 = x1 + line_length * np.cos(theta)
        y2 = y1 + line_length * np.sin(theta)

        p1 = np.array([x1, y1])
        p2 = np.array([x2, y2])

        point = np.array([point[0], point[1]])

        # 计算向量 p1 到 point 和 p1 到 p2
        vector_p1_to_point = point - p1
        vector_p1_to_p2 = p2 - p1

        # 计算叉乘的z分量
        # 在二维平面上，我们只需要计算两个向量的x和y分量的叉乘
        z_component = vector_p1_to_point[0] * vector_p1_to_p2[1] - vector_p1_to_point[1] * vector_p1_to_p2[0]

        # 如果z分量大于0，则点在p1和p2构成的线的左侧
        return z_component > 0

    # 前后车关系判定方法
    def is_back_to_ego(self, state, direction, other_car_xy):
        """
        判定目标车在本车的前后相对关系
        """
        # 初始化后车判定参数
        is_back = False
        ego_yaw_now = state[0][3]  # 自车当前角度
        ego_xy = [state[0][0], state[0][1]]
        if direction == 1:  # 纵向驾驶场景
            if 0 <= ego_yaw_now <= np.pi:  # 向上驾驶
                if other_car_xy[1] < ego_xy[1]:  # 后车
                    is_back = True
            else:  # 向下驾驶
                if other_car_xy[1] > ego_xy[1]:  # 后车
                    is_back = True
        elif direction == -1:  # 横向驾驶场景
            if (np.pi / 2) <= ego_yaw_now <= (np.pi * 3 / 2):  # 向左驾驶
                if other_car_xy[0] > ego_xy[0]:  # 后车
                    is_back = True
            else:  # 向右驾驶
                if other_car_xy[0] < ego_xy[0]:  # 后车
                    is_back = True
        return is_back

    # 实时检测 20米后有无大弯道
    def turn_detect(self, state):
        """
        拿到全局轨迹的下一20米点，判定与本车的横向距离
        """

        # 初始化
        turn_state = False
        next_point = []
        # 取出ego车当前位置
        ego_xy = [state[0][0], state[0][1]]
        # 拿到全局轨迹
        line_center_global = self.line_center_global_list
        # 计算每个点与查询点的距离
        distances = np.linalg.norm(ego_xy - line_center_global, axis=1)
        # 找到最近的点
        nearest_point = line_center_global[np.argmin(distances)]

        # 修改为根据预瞄距离选取合适下一点位
        L = 0.0  # 初始化累加的路径长度 L

        # 计算前视距离
        Lf = 30

        # 最接近点距离索引
        nearest_point_index = np.argmin(distances)



        next_point_index = nearest_point_index
        # 从最近的路径点开始，向前搜索直到累加的路径长度 L 等于或超过前视距离 Lf
        while Lf > L and next_point_index < (len(distances) - 1):
            # 计算当前路径点和下一个路径点之间的距离
            x1, y1, x2, y2 = line_center_global[next_point_index][0], line_center_global[next_point_index][1], \
                line_center_global[next_point_index + 1][0], line_center_global[next_point_index + 1][1],
            distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            L += distance_i
            next_point_index += 1

        # if next_point == []:  # 轨迹已不足20米
        #     next_point = line_center_global[-1]  # 取轨迹最后的点
        next_point = line_center_global[next_point_index]
        # 当前道路的角度
        if nearest_point_index != 0:
            last_point = line_center_global[nearest_point_index - 1]
            lane_now_yaw = self.calculate_angle(last_point, nearest_point)
        else:
            xiayi_point = line_center_global[nearest_point_index + 1]
            lane_now_yaw = self.calculate_angle(nearest_point, xiayi_point)

        # 计算横向距离
        # 根据点斜式计算  横向距离 纵向距离 以及两车直线距离
        distance_hengxiang, distance_zongxiang, p2p_distance = self.distance_point_to_line(next_point, ego_xy,
                                                                                           state[0][3])
        distance_hengxiang_lane, distance_zongxiang_lane, p2p_distance_lane = self.distance_point_to_line(next_point, ego_xy,
                                                                                           lane_now_yaw)

        # 如果横向距离大于 7米，
        if (distance_hengxiang or distance_hengxiang_lane) > 4:
            turn_state = True  # 20 米后有弯道

        return turn_state

    # 定义一个考虑角度差的横向它车半宽度映射算法
    def other_car_points_locat(self, other_car_info):
        """
        根据它车的中心点以及航向角，计算四个边界点坐标，分别同自车计算距离，取最大值
        """
        heading = other_car_info[3]  # 它车航向角
        length = other_car_info[4]
        width = other_car_info[5]
        center_x = other_car_info[0]
        center_y = other_car_info[1]
        # 将航向角从度转换为弧度
        heading_rad = heading

        # 计算角点的偏移量
        corner_offsets = [
            (-length / 2, -width / 2),  # 后左角点
            (length / 2, -width / 2),  # 后右角点
            (length / 2, width / 2),  # 前右角点
            (-length / 2, width / 2)  # 前左角点
        ]

        # 应用航向角旋转
        corners = []
        for offset in corner_offsets:
            # 应用旋转变换
            rotated_x = center_x + offset[0] * math.cos(heading_rad) - offset[1] * math.sin(heading_rad)
            rotated_y = center_y + offset[0] * math.sin(heading_rad) + offset[1] * math.cos(heading_rad)
            corners.append((rotated_x, rotated_y))

        return corners

    # 根据横纵向距离，计算交叉方向来车时，能否满加速通过
    def cexiang_acc_deside(self, state, hengxiang, zongxiang, v, cexiang_v):
        """
        输入横纵向距离，两车速度，判定能否在侧向车到达前满加速通过
        """
        a_bound = self.a_bound / 2  # 给半加速度规划，避免极端场景
        # 初始化加速参考
        acc_can = False
        # 侧向车到达时间
        t_need = hengxiang / (cexiang_v + 1e-6)
        # 纵向距离满加速通过时间
        zongxiang = abs(zongxiang) + state[0][4]   # 留一定计算余量
        # 计算判别式
        # t_can = zongxiang / v
        if v < 3:  # 有一定车速 不用加速计算
            discriminant = v ** 2 + 2 * a_bound * zongxiang

        # 使用二次公式计算时间
            t1 = (-v + math.sqrt(discriminant)) / a_bound
            t2 = (-v - math.sqrt(discriminant)) / a_bound
            t_can = abs(max(t1, t2))
        else:
            t_can = zongxiang / v


        if (t_can < (t_need / 1.3)) or (cexiang_v < 1.5 and hengxiang > 3):  # 可以抢先到达, 并行横向距离大于纵向（有的背景车太快了）
            acc_can = True
        # print(1698, t_need, t_can, hengxiang, zongxiang, cexiang_v, v, discriminant)
        return acc_can

    # 定义一个检测是否可以冲刺终点的算法
    def target_rash_deside(self, state):
        """
        如果已到终点路段，无前车或者前车距离大于终点距离，冲刺终点, 且下一目标点为终点
        """
        # 如果10米后的轨迹点与终点距离小于1，无前车/前车距离大于10米，下一轨迹点为终点，并全速冲刺

        # 拿出10米后的轨迹点
        # 初始化
        target_xy = self.target_xy
        target_near = False
        target_pass = False
        next_point = []
        # 取出ego车当前位置
        ego_xy = [state[0][0], state[0][1]]
        # 拿到全局轨迹
        line_center_global = self.line_center_global_list
        # 计算每个点与查询点的距离
        distances = np.linalg.norm(ego_xy - line_center_global, axis=1)
        distances_target = np.linalg.norm(target_xy - line_center_global, axis=1)
        # 修改为根据预瞄距离选取合适下一点位
        L = 0.0  # 初始化累加的路径长度 L

        # 最接近点距离索引
        nearest_point_index = np.argmin(distances)
        nearest_target_point_index = np.argmin(distances_target)

        next_point_index = nearest_point_index
        # 从最近的路径点开始，向前搜索直到累加的路径长度 L 等于或超过前视距离 Lf
        if next_point_index < nearest_target_point_index:
            while next_point_index < nearest_target_point_index:
                # 计算当前路径点和下一个路径点之间的距离
                x1, y1, x2, y2 = line_center_global[next_point_index][0], line_center_global[next_point_index][1], \
                    line_center_global[next_point_index + 1][0], line_center_global[next_point_index + 1][1],
                distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                L += distance_i
                next_point_index += 1
        else:
            target_pass = True  # 超过终点后，自然驾驶，因为存在终点过后无法停止的场景

        # print(2157, '相距终点距离：', L)

        if L < 30 and target_pass == False:
            target_near = True


        return target_near, L

    # 检测它车角度是否溢出方法
    def other_yaw_yichu(self, ego_yaw_now, other_yaw):
        """
        如果和本车角度同属1 4 象限，则重置它车角度
        """
        if other_yaw > np.pi * 2:
            other_yaw = other_yaw - np.pi * 2

        other_yaw_correct = other_yaw
        if np.pi / 2 < ego_yaw_now < np.pi * 3 / 2:   # 自车属于 2 3象限时不作处理
            other_yaw_correct = other_yaw
        elif 0 <= ego_yaw_now <= np.pi / 2:  # 自车是 1象限
            if np.pi * 3 / 2 <= other_yaw <= np.pi * 2:  # 它车是 4象限
                other_yaw_correct = other_yaw - np.pi * 2
        elif np.pi * 3 / 2 <= ego_yaw_now <= np.pi * 2:  # 自车是 4象限
            if 0 <= other_yaw <= np.pi / 2:  # 它车是 1象限
                other_yaw_correct = other_yaw + np.pi * 2

        return other_yaw_correct

    # 判断它车射线，是否与30米轨迹相交
    def does_ray_intersect_points(self, state, other_xy, other_yaw):

        # 取出ego车当前位置
        ego_xy = [state[0][0], state[0][1]]
        # 拿到全局轨迹
        line_center_global = self.line_center_global_list
        # 计算每个点与查询点的距离
        distances = np.linalg.norm(ego_xy - line_center_global, axis=1)
        # 找到最近的点
        nearest_point = line_center_global[np.argmin(distances)]
        # 最接近点距离索引
        nearest_point_index = np.argmin(distances)

        # 修改为根据预瞄距离选取合适下一点位
        L = 0.0  # 初始化累加的路径长度 L

        # 计算前视距离
        Lf = 30

        next_point_index = nearest_point_index
        # 从最近的路径点开始，向前搜索直到累加的路径长度 L 等于或超过前视距离 Lf
        while Lf > L and next_point_index < (len(distances) - 1):
            # 计算当前路径点和下一个路径点之间的距离
            x1, y1, x2, y2 = line_center_global[next_point_index][0], line_center_global[next_point_index][1], \
                line_center_global[next_point_index + 1][0], line_center_global[next_point_index + 1][1],
            distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            L += distance_i
            next_point_index += 1

        # 倒取5米
        # 修改为根据预瞄距离选取合适下一点位
        L_back = 0.0  # 初始化累加的路径长度 L

        # 计算前视距离
        Lf_back = 5

        next_back_point_index = nearest_point_index
        # 从最近的路径点开始，向前搜索直到累加的路径长度 L 等于或超过前视距离 Lf
        while L_back < Lf_back and next_back_point_index > 0:
            # 计算当前路径点和下一个路径点之间的距离
            x1, y1, x2, y2 = line_center_global[next_back_point_index][0], line_center_global[next_back_point_index][1], \
                line_center_global[next_back_point_index - 1][0], line_center_global[next_back_point_index - 1][1],
            distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            L_back += distance_i
            next_back_point_index -= 1

        # 取出当前点到30米后点中间的所有点
        points = line_center_global[next_back_point_index:next_point_index]

        def is_within_range(P0, theta, point, radius=1):
            """ 判断射线是否在指定半径范围内与点相交 """
            d = (np.cos(theta), np.sin(theta))

            # 射线起点到点的向量
            P0_to_point = (point[0] - P0[0], point[1] - P0[1])

            # 向量投影
            projection_length = P0_to_point[0] * d[0] + P0_to_point[1] * d[1]

            if projection_length < 0:
                return False, []  # 射线方向与点相反，不相交

            # 投影点的坐标
            projection_point = (P0[0] + projection_length * d[0], P0[1] + projection_length * d[1])

            # 判断投影点到点的距离是否小于等于指定半径
            distance = np.linalg.norm(np.array(projection_point) - np.array(point))
            return distance <= radius, projection_point

        for point in points:
            is_with_in, projection_point = is_within_range(other_xy, other_yaw, point)
            if is_with_in:
                return True, projection_point
        return False, []

    # 检测一角度是否夹在两角之间
    def is_angle_between(self, angle, start, end):
        """Check if angle is between start and end, considering circular nature."""

        def normalize_angle(angle):
            """Normalize angle to be within the range 0 to 2*pi."""
            return angle % (2 * np.pi)

        # Normalize all angles to be within 0 to 2*pi
        angle = normalize_angle(angle)
        start = normalize_angle(start)
        end = normalize_angle(end)
        if start < end:
            return start <= angle <= end
        else:  # This handles the wrap around case
            return angle >= start or angle <= end

    # 根据本车轨迹，计算它车相对横纵向距离
    def heng_zong_guiji_other(self, state, other_xy):
        """
        在轨迹中，找到里它车最近的点，计算与本车的横向距离（它车与最近点距离），纵向距离（最近点与本车距离）
        """
        # 初始化
        hengxiang_L, L = 0, 0
        # 取出ego车当前位置
        ego_xy = [state[0][0], state[0][1]]
        # 拿到全局轨迹
        line_center_global = self.line_center_global_list
        # 计算每个点与查询点的距离
        distances_other = np.linalg.norm(other_xy - line_center_global, axis=1)
        distances_ego = np.linalg.norm(ego_xy - line_center_global, axis=1)
        # 最接近点距离索引
        nearest_point_index_other = np.argmin(distances_other)
        nearest_point_index_ego = np.argmin(distances_ego)
        # 计算纵向距离
        if nearest_point_index_other > nearest_point_index_ego:
            # 修改为根据预瞄距离选取合适下一点位
            L = 0.0  # 初始化累加的路径长度 L

            next_point_index = nearest_point_index_ego
            # 从最近的路径点开始，向前搜索直到累加的路径长度 L 等于或超过前视距离 Lf
            while next_point_index < nearest_point_index_other:
                # 计算当前路径点和下一个路径点之间的距离
                x1, y1, x2, y2 = line_center_global[next_point_index][0], line_center_global[next_point_index][1], \
                    line_center_global[next_point_index + 1][0], line_center_global[next_point_index + 1][1],
                distance_i = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                L += distance_i
                next_point_index += 1
        # 找到最近的点
        nearest_point_other = line_center_global[nearest_point_index_other]
        # 计算横向距离
        x1, y1 = nearest_point_other
        x2, y2 = other_xy
        x3, y3 = ego_xy
        hengxiang_L = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        p2p = math.sqrt((x2 - x3) ** 2 + (y2 - y3) ** 2)

        return hengxiang_L, L, p2p