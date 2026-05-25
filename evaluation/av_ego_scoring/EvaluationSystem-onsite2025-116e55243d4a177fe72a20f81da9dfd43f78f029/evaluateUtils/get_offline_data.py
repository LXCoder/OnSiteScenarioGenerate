import logging

import numpy as np
import redis
import pandas as pd
import time
import math
import json
import os
import collections
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from evaluateUtils.config import send_interval, send_topic_mapping, av_radar_distance, redisWanJIAddress, redisWanJIPort
from evaluateUtils.weight_process import sceneAndScoreWeightProcess
from evaluateUtils.standard_parameter import Parameter
from opendrive2tessng.map_info_generation import mapInfoGeneration
from evaluateUtils.my_evaluate import evaluationSystem
from evaluateUtils import my_evaluate
from evaluateUtils.Dynamic_Check import dynamic_check
from evaluateUtils.TrafficRule_compliance.openX_extractor import run_extractor
from evaluateUtils.TrafficRule_compliance.detect_behaviors import run_RuleListConstruction

class getOfflineData():

    def __init__(self, map_file, aim_place_xosc_file_path, csv_file_path, aim_path_json_file_path, taskName, score_data_result):
        self.score_data_result = score_data_result
        self.map_file = map_file
        self.aim_place_xosc_file_path = aim_place_xosc_file_path
        self.csv_file_path = csv_file_path
        self.aim_path_json_file_path = aim_path_json_file_path
        self.taskName = taskName
        self.nameKey = 'senceall'
        self.weightKey = 'taskWeight'
        self.sceneIscoreKey = "TJTestSceneIscore"  # 单个场景的实时更新指标数据
        self.totalScoreKey = "TJTestTotalScore"  # 全部场景的指标数据累积
        self.mapJsonInforDict = None
        self.aimPathJsonInforDict = None
        self.allDataTransferDict = {}
        self.signalInfo = {}

    @staticmethod
    def _contains_scene_map_files(scene_dir):
        if not scene_dir or not os.path.isdir(scene_dir):
            return False
        files = os.listdir(scene_dir)
        has_xodr = any(str(f).lower().endswith('.xodr') for f in files)
        has_xosc = any(str(f).lower().endswith('.xosc') for f in files)
        return has_xodr and has_xosc

    def _resolve_scene_folder(self, map_root, scene_type, scene_name):
        """
        Resolve scene folder from different map layouts.
        Supports:
        - <root>/<scene_type>/<scene_name>
        - <root>/<scene_name>
        - recursive search under <root>
        """
        map_root = os.path.abspath(map_root)
        candidates = [
            os.path.join(map_root, str(scene_type), str(scene_name)),
            os.path.join(map_root, str(scene_type).lower(), str(scene_name)),
            os.path.join(map_root, str(scene_name)),
        ]
        for cand in candidates:
            if self._contains_scene_map_files(cand):
                return os.path.abspath(cand)

        for dirpath, _, _ in os.walk(map_root):
            if os.path.basename(dirpath) != str(scene_name):
                continue
            if self._contains_scene_map_files(dirpath):
                return os.path.abspath(dirpath)
        return None

    # 地图信息生成，包括万集解析数据和终点信息解析数据
    def getMapInfo(self, map_file_path, aim_place_file_path):
        map = mapInfoGeneration()
        # 获得终点信息解析数据
        map.goalPlaceInfoProcess(aim_place_file_path)
        # 获得万集解析数据
        map.mapDataProcess(map_file_path)
        # 两个结束后，把地图信息都写入了self.road_dict，并且已经放在了一个json文件里面
        return map.road_dict

    # 这里把停止线信息处理成一个四方形的区域，如果在这个区域内，则算是闯红灯了
    def getCrossLineInfo(self, verticesDict):
        polygon_vertices_dict = {}
        # 这里的all_stop_line已经是一个字典了，所以这里要继续按照字典的模式做
        for key in verticesDict["all_stop_line"]:
            polygon_vertices = []
            for i in verticesDict["all_stop_line"][key]:
                for j in i:
                    point = (j[0], j[1])
                    polygon_vertices.append(point)
            polygon_vertices_dict[key] = polygon_vertices
        return polygon_vertices_dict

    # 还原根据地图改变限速信息
    def changeLimitSpeed(self):
        mapType = self.mapJsonInforDict['header_info']['name']
        list33 = ['highway', 'highwaymerge']
        list9 = ['mixed', 'intersection', 'roundabout', 'serial']
        if mapType in list33:
            Parameter.maxSpeedKM = 33 * 3.6
            Parameter.avgSpeed = 33
            Parameter.avgSpeedKM = 33 * 3.6
        elif mapType in list9:
            Parameter.maxSpeedKM = 23 * 3.6
            Parameter.avgSpeed = 9
            Parameter.avgSpeedKM = 9 * 3.6
        else:
            Parameter.maxSpeedKM = 23 * 3.6
            Parameter.avgSpeed = 9
            Parameter.avgSpeedKM = 9 * 3.6

    def changeAffectedMetre(self):
        mapType = self.mapJsonInforDict['header_info']['name']
        list33 = ['highway', 'highwaymerge']
        list9 = ['mixed', 'intersection', 'roundabout', 'serial']
        if mapType in list33:
            Parameter.affectedMetre = 50
        elif mapType in list9:
            Parameter.affectedMetre = 200
        else:
            Parameter.affectedMetre = 50

    def startGetOfflineData(self, data_transfer_queue, map_path_data_queue):
        evaluation = evaluationSystem()
        SVevaluations = {}

        # 回头要的数据包括，指标的权重数据，预期完成时间数据，还有地图数据，还有CSV数据
        # 还要包括，场景名称数据、车辆名称数据、测试任务id
        # 单个csv文件只对应单个的场景，一个csv只有一个场景，因此不用设置场景权重

        # 1-1要获取基础地图信息
        if not Parameter.sceneType:
            map_file_path = 'testData/0_1.xodr'
        else:
            '''wth：这里是离线的逻辑'''
            map_file_folder = Parameter.map_file
            # wth: 如果是serial场景，需要先读task的json文件，指向场景名，再去读场景目录
            if Parameter.sceneType == 'serial' or Parameter.sceneType == 'SERIAL':
                with open(self.aim_path_json_file_path, "r") as aim_path_json_file:
                    # 使用 json.load() 方法加载 JSON 数据
                    self.aimPathJsonInforDict = json.load(aim_path_json_file)
                    file_name = self.aimPathJsonInforDict['map']
                    map_file_folder = os.path.join(map_file_folder, Parameter.sceneType, 'maps', file_name)
                    map_files = os.listdir(map_file_folder)
                    xodr_file = [f for f in map_files if f.endswith('.xodr')][0] if len([f for f in map_files if f.endswith('.xodr')]) > 0 else None
                    xosc_file = [f for f in map_files if f.endswith('.xosc')][0] if len([f for f in map_files if f.endswith('.xosc')]) > 0 else None
                    signal_files = [f for f in map_files if f.endswith('.json') and not f.startswith("rule_list") and not f.startswith("all_road_info_file")]
                    signal_file = signal_files[0] if len(signal_files) > 0 else None
                    map_file_path = os.path.join(map_file_folder, xodr_file)
                    tmp_aim_place_file_path = os.path.join(map_file_folder, xosc_file) if xosc_file else None
                    tmp_aim_signal_file_path = os.path.join(map_file_folder, signal_file) if signal_file else None
                    Parameter.scn_folder_name = os.path.join(Parameter.sceneType, 'maps', file_name)
            else:
                file_name = '_'.join(Parameter.sceneName.split('_')[2:])
                Parameter.scn_folder_name = os.path.join(Parameter.sceneType, file_name)
                resolved_scene_folder = self._resolve_scene_folder(map_file_folder, Parameter.sceneType, file_name)
                if resolved_scene_folder:
                    map_file_folder = resolved_scene_folder
                    try:
                        rel_scene_folder = os.path.relpath(resolved_scene_folder, Parameter.map_file)
                    except Exception:
                        rel_scene_folder = os.path.join(Parameter.sceneType, file_name)
                    Parameter.scn_folder_name = rel_scene_folder
                else:
                    map_file_folder = os.path.join(map_file_folder, Parameter.sceneType, file_name)
                map_files = os.listdir(map_file_folder)
                xodr_file = [f for f in map_files if f.endswith('.xodr')][0] if len([f for f in map_files if f.endswith('.xodr')]) > 0 else None
                xosc_file = [f for f in map_files if f.endswith('.xosc')][0] if len([f for f in map_files if f.endswith('.xosc')]) > 0 else None
                signal_files = [f for f in map_files if f.endswith('.json') and not f.startswith("rule_list") and not f.startswith("all_road_info_file")]
                signal_file = signal_files[0] if len(signal_files) > 0 else None
                map_file_path = os.path.join(map_file_folder, xodr_file)
                tmp_aim_place_file_path = os.path.join(map_file_folder, xosc_file) if xosc_file else None
                tmp_aim_signal_file_path = os.path.join(map_file_folder, signal_file) if signal_file else None
            # map_file_path = r"C:\Users\wth13\Desktop\Projects\EvaluationSystem\Data\scenario\replay\0_45_left_left_48\0_45_left_left_48.xodr"
        # 1-2要获取结束位置信息
        aim_place_file_path = tmp_aim_place_file_path or self.aim_place_xosc_file_path
        # aim_place_file_path = self.aim_place_xosc_file_path

        # 3要评价的轨迹信息
        csv_file_path = self.csv_file_path
        # 3要评价的目标路径信息，这里要自己去判断是路径信息，还是信号灯信息
        aim_path_json_file_path = self.aim_path_json_file_path

        try:
            # 读取 JSON 文件
            with open(aim_path_json_file_path, "r") as aim_path_json_file:
                # 使用 json.load() 方法加载 JSON 数据
                self.aimPathJsonInforDict = json.load(aim_path_json_file)
        except:
            self.aimPathJsonInforDict = None
        # 这里要判断一下self.aimPathJsonInforDict里面的内容，如果是信号灯的内容，需要复制到信号灯文件里面，并且把self.aimPathJsonInforDict = None
        if self.aimPathJsonInforDict:
            if "waypoints" not in self.aimPathJsonInforDict:
                self.signalInfo = self.aimPathJsonInforDict
                self.aimPathJsonInforDict = None
        # 存在信控json文件，读信控数据
        if tmp_aim_signal_file_path:
            with open(tmp_aim_signal_file_path, "r") as json_file:
                # 使用 json.load() 方法加载 JSON 数据
                self.signalInfo = json.load(json_file)

        # 处理地图，这里传输的是两个文件的路径
        # wth:这里基本没有保留原先的结构
        map_read_status = None
        try:
            json_file_path = os.path.join(os.path.dirname(map_file_path), "all_road_info_file.json")
            # 判断是否存在opendrive同级的roadjson文件，存在表示该路网已经解析过
            if os.path.exists(json_file_path):
                try:
                    with open(json_file_path, "r") as json_file:
                        self.mapJsonInforDict = json.load(json_file)
                    map_read_status = True
                    print(f"{json_file_path}文件读取成功")
                except:
                    map_read_status = False
                if not map_read_status:
                    # 文件存在但读取失败可能是有别的进程正在写，不能再重复写入
                    print(f"{json_file_path}文件读取失败，重新计算地图信息")
                    self.mapJsonInforDict = self.getMapInfo(map_file_path, aim_place_file_path)
                    self.mapJsonInforDict["all_stop_line_points"] = self.getCrossLineInfo(self.mapJsonInforDict)
            else:
                # 没有all_road_info_file文件，计算并写入
                print(f"{json_file_path}文件不存在，开始处理地图信息")
                self.mapJsonInforDict = self.getMapInfo(map_file_path, aim_place_file_path)
                self.mapJsonInforDict["all_stop_line_points"] = self.getCrossLineInfo(self.mapJsonInforDict)
                try:
                    with open(json_file_path, "w") as json_file:
                        json.dump(self.mapJsonInforDict, json_file)
                    print(f"{json_file_path}文件写入成功")
                except:
                    print(f"{json_file_path}文件写入失败")
                # 根据地图改变限速信息
                print("地图信息处理完毕")
                # 第一帧数据先发送静态信息，地图和目标轨迹
                # 如果上面没有数据处理得到地图和轨迹信息，则是none，第二个部分判断一下是不是none就行
            mapAndPathJsonInforDict = {"mapJsonInforDict": self.mapJsonInforDict,
                                       "aimPathJsonInforDict": self.aimPathJsonInforDict}
            self.changeLimitSpeed()
            self.changeAffectedMetre()
        except:
            raise ValueError(f"地图信息不存在：{map_file_path}")

        # wth:解析交规
        rule_read_status = None
        if csv_file_path:
            scn_folder_name = Parameter.scn_folder_name
            database = ['china']
            # wth:场景固定时前两步都可以提前做
            rule_list_file_path = os.path.join(Parameter.map_file, f'{scn_folder_name}/rule_list.json')
            if os.path.exists(rule_list_file_path):
                try:
                    with open(rule_list_file_path, 'r') as f:
                        rule_list = json.load(f)
                    rule_read_status = True
                    print(f"{rule_list_file_path}文件读取成功")
                except:
                    rule_read_status = False
                if not rule_read_status:
                    print(f"{rule_list_file_path}文件读取失败，重新计算交规")
                    # 文件存在但读取失败可能是有别的进程正在写，不能再重复写入
                    extractor_res = run_extractor(scn_folder_name)
                    rule_list = run_RuleListConstruction(scn_folder_name, extractor_res, database)
            else:
                print(f"{rule_list_file_path}文件不存在，开始计算交规")
                # 第一步：读取opendrive(.xodr)和openscenario(.xosc)等文件，提取测试场景中的要素
                extractor_res = run_extractor(scn_folder_name)
                # 第二步：根据场景要素检索交通法规，生成禁止性行为和强制性行为清单
                rule_list = run_RuleListConstruction(scn_folder_name, extractor_res, database)
                try:
                    with open(rule_list_file_path, 'w') as f:
                        json.dump(rule_list, f)
                    print(f"{rule_list_file_path}文件写入成功")
                except:
                    print(f"{rule_list_file_path}文件写入失败")
        else:
            rule_list = dict()
        # 2基础信息处理
        taskID = Parameter.taskID
        avName = Parameter.avName
        sceneName = Parameter.sceneName
        sceneTypeList = [sceneName]
        sceneExpectTime = Parameter.missonExpectTime["0_17_straight_straight_21"]
        self.allWeight = sceneAndScoreWeightProcess(Parameter.weightData, sceneTypeList)
        # 这里获得了5个字典，场景间权重字典、维度间权重字典、安全、效率、舒适指标间权重字典
        self.allWeight.weightProcess()
        allWeight = self.allWeight
        originRealTime = []
        # 记录每辆SV车的最终得分
        final_SVScore = collections.defaultdict(dict)

        # 在这里把数据放在队列中，或者说在这里把一堆关键数据，包括：3个主要，其他信息：地图、任务ID、场景名称、车辆名称等信息，放在一个字典里面，然后把字典放在一个队列里面
        # 计算文件有多少行，判断什么时候结束
        df = pd.read_csv(csv_file_path, encoding='utf-8')
        df = df.rename(columns={'Unnamed: 0': 'simuTime'})
        dataAll = df.sort_values('simuTime')
        end_count = df.shape[0]
        enderCount = 0
        # wth: 主车动力学校核
        t = df.iloc[:, 0].tolist()
        deltaf = df['rot_ego'].tolist()
        vx = df['v_ego'].tolist()
        ax = df['a_ego'].tolist()
        flag, reasons = dynamic_check(t, deltaf, vx, ax)
        # 未通过动力学校核，则不进行评价
        if flag == 0:
            # wth:直接写入动力学校核不通过的结果
            self.score_data_result[Parameter.taskName] = {'2': {'flag': 0, 'reasons': reasons, 'sceneNameList': Parameter.sceneName}}
            print("主车未通过动力学校核")
        # 通过动力学校核，启动评价
        else:
            print("主车通过动力学校核,开始评价")
            starTime = 0
            for index, row in dataAll.iterrows():
                if index == end_count - 1:
                    Parameter.done = True
                # enderCount += 1
                # 每次读取一行的数据，对这一帧的数据进行 处理和评分，然后再下一行，开始新一行之前把之前的数据清空
                SV_cars_info = []

                # 整理av的数据
                # data = chunk.values
                # 不能直接 dropna：否则会把同一车辆的部分字段（如 y）丢掉，导致后续 KeyError。
                # 统一缺失值补 0，保证每辆车字段结构完整。
                row = row.fillna(0)
                data = row.to_dict()
                singleCarData = collections.defaultdict(dict)
                for k, v in data.items():
                    if len(k.split('_')) <= 1:
                        continue
                    carId = k.split('_')[1]
                    key = k.split('_')[0]
                    singleCarData[carId][key] = v
                if index == 0:
                    starTime = data['simuTime']
                usedTime = data['simuTime'] - starTime
                # print(f"usedTime: {usedTime}")
                # wth：轨迹数据多出两列
                # 整理sv的数据
                SV_car_ids = list()
                for carID in singleCarData:
                    if carID == 'ego':
                        AV_cars_info = self.newCar(singleCarData[carID], carID, usedTime)
                    else:
                        # SV_cars_info.append(self.newCar(singleCarData[carID], int(''.join([i for i in carID if i.isdigit()])), usedTime))
                        SV_cars_info.append(
                            self.newCar(singleCarData[carID], carID,
                                        usedTime))
                        SV_car_ids.append(carID)
                # 这里释放掉已经不存在的SV车辆评价系统
                dele_SV_carIDs = [i for i in SVevaluations.keys() if i not in SV_car_ids]
                for carID in dele_SV_carIDs:
                    SV_evaluation = SVevaluations[carID]
                    SVevaluations.pop(carID)
                    del SV_evaluation
                # todo 为了测试先关上。后续打开直接取消注释
                # 整理信号灯数据
                if json.dumps(usedTime) in self.signalInfo:
                    signalState = self.signalInfo[json.dumps(usedTime)]
                else:
                    signalState = None

                # 整理场景数据
                if enderCount != end_count:
                    scene_info = {
                        'sceneID': 1,
                        'missonExpectTime': sceneExpectTime,
                        'usedTime': usedTime,
                        'simuTime': usedTime,
                        'sceneWeight': 1,
                        'sceneState': 0,  # 0表示未完成，1表示已完成
                        'sceneAllState': 0,  # 0表示未完成，1表示已完成
                    }
                else:
                    scene_info = {
                        'sceneID': 0,
                        'missonExpectTime': sceneExpectTime,
                        'usedTime': usedTime,
                        'simuTime': usedTime,
                        'sceneWeight': 1,
                        'sceneState': 1,  # 0表示未完成，1表示已完成
                        'sceneAllState': 1  # 0表示未完成，1表示已完成
                    }

                self.allDataTransferDict = {
                    "AV_cars_info": AV_cars_info,
                    "SV_cars_info": SV_cars_info,
                    "scene_info": scene_info,
                    "taskID": taskID,
                    "taskName": self.taskName,
                    "avName": avName,
                    "sceneName": sceneName,
                    "allWeight": allWeight,
                    "crashData": 0,
                    "signalState": signalState,
                    "originRealTime": data['simuTime'],
                }
                # wth:为每一辆SV车创建评价系统
                for car in SV_cars_info:
                    carID = car['id']
                    # wth:这里只能筛掉车辆id中包含了类型描述的
                    if 'bicycle' in carID or 'pedestrian' in carID:
                        continue
                    if carID in SVevaluations.keys():
                        continue
                    SVevaluation = evaluationSystem()
                    SVevaluations[car['id']] = SVevaluation
                # wth:计算AV车评分
                AV_senceIscore, AV_totallSenceIscore = evaluation.startMyEvaluation(self.allDataTransferDict, originRealTime, self.csv_file_path, mapAndPathJsonInforDict, affected=-1, rule_list=rule_list)
                if not AV_senceIscore:
                    continue
                # wth:除了回放型，其他类型要重新计算协调性得分
                if Parameter.sceneType not in ['replay', 'replay'.upper()]:
                    # wth:调换AV车和SV车
                    SV_score = collections.defaultdict(dict)
                    cal_task_list = collections.defaultdict(dict)
                    for car in SV_cars_info:
                        carID = car['id']
                        if 'bicycle' in carID or 'pedestrian' in carID:
                            continue
                        # 从SV_cars中取出这辆车赋值给AV_car
                        update_SV_cars_info = [car for car in SV_cars_info if car['id'] != carID]
                        update_SV_cars_info.append(AV_cars_info)
                        update_AV_cars_info = [car for car in SV_cars_info if car['id'] == carID][0]
                        allDataTransferDict = self.allDataTransferDict
                        allDataTransferDict["AV_cars_info"] = update_AV_cars_info
                        allDataTransferDict["SV_cars_info"] = update_SV_cars_info
                        if carID in evaluation.Ego.affectedSVList:
                            for k, v in allDataTransferDict.items():
                                cal_task_list[carID][k] = v
                            cal_task_list[carID]["AV_cars_info"] = update_AV_cars_info
                            cal_task_list[carID]["SV_cars_info"] = update_SV_cars_info
                        else:
                            affected = 0
                            SVevaluation = SVevaluations[carID]
                            SVevaluation.startMyEvaluation(allDataTransferDict, originRealTime, self.csv_file_path, mapAndPathJsonInforDict, affected)
                    scoreWeight = evaluation.allWeight.scoreWeight
                    fullScore = evaluation.allWeight.fullScore
                    for carID in cal_task_list:
                        SV_senceIscore, SV_totallSenceIscore = SVevaluations[carID].startMyEvaluation(cal_task_list[carID], originRealTime, self.csv_file_path, mapAndPathJsonInforDict, 1)
                        if not SV_senceIscore:
                            continue
                        totalScore = (SV_totallSenceIscore.get('efficiency')) ** scoreWeight.get('efficiency') * (SV_totallSenceIscore.get('comfortable') * 0.4 + SV_totallSenceIscore.get('safe') * 0.6)
                        SV_score[SV_totallSenceIscore.get('senceID')][carID] = totalScore
                        # 更新为每辆SV车的最新得分
                        final_SVScore[SV_totallSenceIscore.get('senceID')][carID] = totalScore
                    # wth:轨迹帧数不够，没有评分结果
                    if len(final_SVScore) > 0:
                        # wth:更新主车协调性得分
                        for sceneID, SVs in final_SVScore.items():
                            if AV_totallSenceIscore['senceID'] != sceneID:
                                continue
                            # SV车的total得分为百分制，这里需要再乘权重，场景权重为1，暂时忽略
                            coordination = (sum([v for v in SVs.values()]) / len(SVs) / (100 / fullScore['coordination']))
                            # 更新主车得分
                            AV_totallSenceIscore.update({'coordination': coordination, 'coordination100': coordination})
                    # print(final_SVScore, AV_totallSenceIscore)
                # 用更新后的AV_senceIscore, AV_totallSenceIscore整理detail
                senceIscoreDetail = my_evaluate.evaluationDetailDataProcess(AV_senceIscore, AV_totallSenceIscore, evaluation.sceneData['sceneID'],
                                                                evaluation.sceneData['usedTime'], evaluation.Ego, evaluation.sceneData['sceneWeight'], evaluation.sceneData['missonExpectTime'], evaluation.allWeight)
                senceIscoreDetail["taskID"] = evaluation.caseID
                senceIscoreDetail["avName"] = evaluation.avName
                if evaluation.sceneData['usedTime'] > evaluation.xTime:
                    evaluation.chartDataSingleScene = my_evaluate.getChartData(senceIscoreDetail, evaluation.xTime, evaluation.chartDataSingleScene,
                                                             evaluation.chartDataSingleScene)
                    evaluation.xTime = evaluation.sceneData['usedTime']
                    evaluation.chartDataTotalScene[evaluation.sceneData['sceneID']] = evaluation.chartDataSingleScene

                # wth:场景结束后判断后续内容,移到外层
                if Parameter.done:
                    AV_senceIscore['missionAccomplish'] = 1
                    a = 100
                    b = 100
                    if not evaluation.senceScoreDital:
                        a = 1
                        b = 1
                    for iscore in evaluation.senceScoreDital:
                        if iscore['senceID'] == evaluation.sceneData['sceneID']:
                            pass
                        else:
                            a = 2
                    if a == 1 or a == 2:
                        evaluation.senceScoreDital.append(AV_senceIscore)
                    for jscore in evaluation.senceScoreTotal:
                        if jscore['senceID'] == evaluation.sceneData['sceneID']:
                            pass
                        else:
                            b = 2
                    if b == 1 or b == 2:
                        evaluation.senceScoreTotal.append(AV_totallSenceIscore)
                    evaluation.sceneData['sceneID'] = 0
                    # 在这里id等于0的时候证明场景结束了，就要在ego里面把他积累的一些数值全部归零,特别是ttc和什么别的值
                    evaluation.Ego.clearAllEgoData()
                    # 场景结束的时候还要把sv的轨迹点和冲突时间节点清零
                    evaluation.svAllPosList = []
                    evaluation.conflictCarIdList = []
                    evaluation.lastConflictTime = []
                    # 场景结束还要把sv的字典清空一下，不然最后一个场景还有第一个场景的sv车的数据就很怪
                    evaluation.svAllPosDict = {}
                    # 还有把单场景的折线图数据清空
                    evaluation.chartDataSingleScene = {}
                    evaluation.xTime = 0
            # wth:结束后整理数据
            evaluation.missonEndTime = str(datetime.now())[:23]
            evaluation.missonEndTimeSimu = evaluation.sceneData['simuTime']
            # evaluation.missonEndTimeSimu = usedTime
            print("场景结束时间", evaluation.missonEndTime, evaluation.missonEndTimeSimu)
            evaluation.testDuration = evaluation.missonEndTimeSimu - evaluation.missonStartTimeSimu if evaluation.missonEndTimeSimu - evaluation.missonStartTimeSimu else 9999
            evaluation.allSceneEndLock = 1

            totalScore_safe = 0
            totalScore_efficiency = 0
            totalScore_comfortable = 0
            totalScore_coordination = 0
            totalScore_compliance = 0
            # 根据清除信号，等于1表示，跑完一圈了，可以进行清除车辆，并且输出所有指标了
            for totallSenceIJustScore in evaluation.senceScoreTotal:
                weight = 1
                totalScore_safe += weight * totallSenceIJustScore['safe']
                totalScore_efficiency += weight * totallSenceIJustScore['efficiency']
                totalScore_comfortable += weight * totallSenceIJustScore['comfortable']
                totalScore_coordination += weight * totallSenceIJustScore['coordination']
                totalScore_compliance += weight * totallSenceIJustScore['compliance']

            # self.totalScore = {'安全': totalScore_safe, '效率': totalScore_efficiency, '舒适': totalScore_comfortable}
            totalScore = {'safe': totalScore_safe,
                          'efficiency': totalScore_efficiency,
                          'comfortable': totalScore_comfortable,
                          'coordination': totalScore_coordination,
                          'compliance': totalScore_compliance,
                          'allSenseScore': evaluation.senceScoreTotal,
                          'allSenseScoreDital': evaluation.senceScoreDital}

            # 数据处理
            outPutData = my_evaluate.evaluationDataProcess(evaluation.Ego, totalScore, evaluation.missonExpectTime, evaluation.senceWeight,
                                               evaluation.missonStartTime, evaluation.missonEndTime, evaluation.testDuration, evaluation.allWeight,
                                               evaluation.chartDataTotalScene)

            # 这里要用之前拿到的数据outputdata来判断一下写出哪些诊断语句库
            # 这里专门在写一个类写诊断语句
            diagnoseLibrary = my_evaluate.diagnose(outPutData)
            diagnoseLibrary.getSuggestion()
            outPutData["diagnose"] = diagnoseLibrary.diagnoseSuggestionDict
            diagnoseLibrary = None
            outPutData["taskID"] = evaluation.caseID
            outPutData["avName"] = evaluation.avName
            finalKey = evaluation.totalScoreKey + ":" + str(evaluation.caseID)
            print(f"\033[1;32m{'本次测试结果概况如下：'}\033[0m", "测试总分：", outPutData["allSenseScore"], "分项成绩：", outPutData["AbilityDimension"])
            # print(f"\033[1;32m{'本次测试结果概况如下：'}\033[0m", "分项成绩：", outPutData["AbilityDimension"])

            # wth:只有一个场景这一步并不需要
            # 这一段是将总的场景的分切分成单场景得分
            evaluation.sceneCaseIDlist = [evaluation.caseID]
            sceneIoutPutDataDict = my_evaluate.splitData(evaluation.Ego, totalScore, evaluation.missonExpectTime, evaluation.senceWeight,
                                             evaluation.missonStartTime, evaluation.missonEndTime, evaluation.testDuration,
                                             evaluation.allWeight, evaluation.chartDataTotalScene, evaluation.caseID,
                                             evaluation.sceneCaseIDlist, evaluation.avName)
            # 这里开始清除数据
            # 在这里把一圈的场景跑完后，需要把所有的内容都清空一遍
            evaluation.clearAllData()
            # print(Parameter.taskName, "测试完成2")
            sceneIoutPutDataDict["1"]['startTime'] = min(originRealTime)
            sceneIoutPutDataDict["1"]['endTime'] = max(originRealTime)

            # wth:整理结束后才开始写入结果
            try:
                self.score_data_result[Parameter.taskName] = sceneIoutPutDataDict
            except Exception as e:
                print(f"检查数据: self.score_data_result: {len(self.score_data_result), id(self.score_data_result), type(self.score_data_result)}")
                print(f"检查数据: Parameter.taskName: {len(Parameter.taskName), id(Parameter.taskName), type(Parameter.taskName)}")
                print(f"检查数据: sceneIoutPutDataDict: {len(sceneIoutPutDataDict), id(sceneIoutPutDataDict), type(sceneIoutPutDataDict)}")
                pass



    def newCar_origin(self, data, a):
        usedTime = data[0][0]
        xAV = data[0][1 + a]
        yAV = data[0][2 + a]
        v_ego = data[0][3 + a]
        a_ego = data[0][4 + a]
        yaw_ego = data[0][5 + a]
        # wth:轨迹文件格式变化
        # width_ego = data[0][6 + a]
        # length_ego = data[0][7 + a]
        if a - 2 == 0:
            width_ego = data[0][7 + a]
            length_ego = data[0][8 + a]
        else:
            width_ego = data[0][6 + a]
            length_ego = data[0][7 + a]
        cars_info = {
            'id': (a-2)/7,
            'x': xAV,
            'y': yAV,
            'simutime': usedTime,
            'speed': v_ego,
            'acce': a_ego,
            'type': 1,
            'width': width_ego,
            'length': length_ego,
            'angle': self.arcToAngle(yaw_ego),    # 弧度制转角度制
            # 'angle': yaw_ego,  # 约定好的角度是，y轴为0顺时针增加，角度制
            'HeadwayFront': 0,
            'DistFront': 0,
            # 这下面两个只有av才需要，可以设置为与原始一致，av后续会单独更新，这也是为啥要先做sv的
            'realpos': [xAV, yAV],
            'speedreal': v_ego,
            'longitude': xAV,
            'latitude': yAV,
            'isSecurityInvolved': 0,
        }
        return cars_info

    def newCar(self, data, carID, usedTime):
        x = data.get('x', 0)
        y = data.get('y', 0)
        v = data.get('v', 0)
        a = data.get('a', 0)
        yaw = data.get('yaw', 0)
        # wth:轨迹文件格式变化
        width = data.get('width', 0)
        length = data.get('length', 0)
        cars_info = {
            'id': carID,
            'x': x,
            'y': y,
            'simutime': usedTime,
            'speed': v,
            'acce': a,
            'type': 1,
            'width': width,
            'length': length,
            'angle': self.arcToAngle(yaw),    # 弧度制转角度制
            # 'angle': yaw,  # 约定好的角度是，y轴为0顺时针增加，角度制
            'HeadwayFront': 0,
            'DistFront': 0,
            # 这下面两个只有av才需要，可以设置为与原始一致，av后续会单独更新，这也是为啥要先做sv的
            'realpos': [x, y],
            'speedreal': v,
            'longitude': x,
            'latitude': y,
            'isSecurityInvolved': 0,
        }
        numeric_fields = ["x", "y", "simutime", "speed", "acce", "width", "length", "angle", "speedreal", "longitude", "latitude"]
        result = self.ensure_numeric_fields(cars_info, numeric_fields)
        return result


    # onsite弧度制转换为tess角度制的函数
    # 首先第一步是要把弧度转换为角度
    # 第二步把转换后的角度变成正北的
    def arcToAngle(self, value):
        angle_east = (value % (math.pi * 2)) * (180 / math.pi)
        # 将正东为0逆时针增大的角度转换为以正北为0顺时针增大的角度
        angle_north = (90 - angle_east) % 360
        # print(value, angle_east, angle_north)
        return angle_north

    def ensure_numeric_fields(self, data, numeric_fields):
        """确保字典中的指定字段为数值类型，如果是字符串则转换"""
        for field in numeric_fields:
            if field in data:
                try:
                    # 尝试转换为 float（兼容整数和小数）
                    data[field] = float(data[field])
                except (ValueError, TypeError):
                    # 转换失败可以设置默认值或报错
                    data[field] = None  # 或者 raise ValueError(f"无法转换字段 {field} 为数值")
        return data
