#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import lib

from planner.plannerBase import PlannerBase
from utils.observation import Observation

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import main_main as mm

class PublicRoadPlanner(PlannerBase):
    def __init__(self, predict=1):
        a_bound = 10  # 本车加速度绝对值的上下界
        exv_set = 10  # 期望速度(m/s)
        exv = exv_set  # 期望速度(m/s)
        t = 1.0  # 反应时间
        a = 40  # 起步加速度 # ori:2.22
        b = 0.5  # 舒适减速度
        gama = 4  # 加速度指数
        s0 = 5.0  # 静止安全距离 #ori:10
        s1 = 2.0  # 与速度有关的安全距离选择参数
        s_ = 0
        index = 0
        history_acc_dece = 0  # history_acc_dece
        aacc_dece = 1  # ori:0.8

        # 预测
        predict = predict
        dt = 2  # ori : 1
        Predict_number = 2  # ori: 1
        min_obj_pre_area = 1


        all_line = []
        ref_line = []
        lateral_safe_dis = 1.5

        xodr_map = 0

        route_cost = -1
        warm_start_time = 0
        max_r = 40
        self.mmm = mm.main_main(a_bound, exv_set, t, a, b, gama, s0, s1, s_, index, history_acc_dece, aacc_dece,
                 predict, dt, Predict_number, min_obj_pre_area,
                 all_line, ref_line, lateral_safe_dis, xodr_map,
                 route_cost, warm_start_time, max_r)

    def init(self, scenario_dict):
        self.mmm.init(scenario_dict)

    def act(self, observation: Observation):
        return self.mmm.act(observation)

