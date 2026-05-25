#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import datetime
from pathlib import Path
from utils import functions
import pandas as pd
import csv
from multiprocessing import freeze_support

def offline_evaluation(trj_folder, map_folder):
    '''

    :param trj_folder: 轨迹csv文件目录
    :param map_folder: 场景文件总目录
    :return:
    '''
    trj_files = os.listdir(trj_folder)
    task_name = os.path.basename(trj_folder)
    if 'score_ing.csv' in trj_files:
        os.remove(os.path.join(trj_folder, 'score_ing.csv'))
    if 'score_detail_ing.csv' in trj_files:
        os.remove(os.path.join(trj_folder, 'score_detail_ing.csv'))
    trj_files = [i for i in trj_files if i.endswith('.csv') and not i.startswith('score')]
    scoreResultList = []
    detailResultList = []
    count = len(trj_files)
    number = 0
    for trj_file in trj_files:
        number += 1
        print(trj_file, f'评价进度：{number} / {count}')
        csvDataPath = os.path.join(trj_folder, trj_file)
        # IMPORTANT:
        # task_name is usually "trajectory" for all scenes under score_ego_effectiveness staging dir.
        # If we reuse it as evaluation id, TMPPATH file names can collide across parallel workers.
        # Use per-csv unique evaluation id to avoid cross-scene temp file overwrite.
        scene_stem = Path(trj_file).stem
        evaluation_id = f"{task_name}_{scene_stem}"
        evaluation_record = {
            'id': evaluation_id,
            'isDeleted': 0,
            'status': 'wait',
            'filterType': 'time',
            'csvDataPath': csvDataPath,
            'mapDataPath': map_folder
        }

        from evaluateUtils.standard_parameter import Parameter
        Parameter.done = False

        vehi_regional_score = functions.calc_evaluation_score_one(evaluation_record)
        if len(vehi_regional_score) == 0:
            raise RuntimeError("empty vehi_regional_score")

        scoreResult, detailResult = organizeResults(vehi_regional_score)
        if len(scoreResult) == 0 and len(detailResult) == 0:
            raise RuntimeError("empty score/detail rows")

        if len(scoreResult) > 0:
            scoreResultList += scoreResult
            dynamic_append(os.path.join(trj_folder, 'score_ing.csv'), scoreResult[0])
        if len(detailResult) > 0:
            detailResultList += detailResult
            dynamic_append(os.path.join(trj_folder, 'score_detail_ing.csv'), detailResult[0])
    if len(scoreResultList) == 0 and len(detailResultList) == 0:
        raise RuntimeError("all trajectories failed; no valid AV score rows generated")

    scoreResultTable = pd.DataFrame(scoreResultList)
    detailResultTable = pd.DataFrame(detailResultList)
    scoreResultTable, detailResultTable = supplement(map_folder, scoreResultTable, detailResultTable)
    average_score = {}
    for col in scoreResultTable.columns:
        average_score[col] = scoreResultTable[col].mean() if pd.api.types.is_numeric_dtype(scoreResultTable[col]) else 'Mean' if col == 'Scenario' else None
    scoreResultTable.loc[len(scoreResultTable)] = average_score
    average_score_detail = {}
    for col in detailResultTable.columns:
        average_score_detail[col] = detailResultTable[col].mean() if pd.api.types.is_numeric_dtype(detailResultTable[col]) else 'Mean' if col == 'Scenario' else None
    detailResultTable.loc[len(detailResultTable)] = average_score_detail
    scoreResultTable.to_csv(os.path.join(trj_folder, 'score.csv'))
    detailResultTable.to_csv(os.path.join(trj_folder, 'score_detail.csv'))
    # 删除过程中记录文件
    if 'score_ing.csv' in os.listdir(trj_folder):
        os.remove(os.path.join(trj_folder, 'score_ing.csv'))
    if 'score_detail_ing.csv' in os.listdir(trj_folder):
        os.remove(os.path.join(trj_folder, 'score_detail_ing.csv'))
    # # 返回给昀喆的结果
    resDict = {'安全': average_score.get('Safety', 0), '效率': average_score.get('Efficiency', 0), '舒适': average_score.get('Comfort', 0), '交通协调性': average_score.get('Coordination', 0),
           '交规符合性': average_score.get('Compliance', 0), '总分': average_score.get('Total', 0)}
    resDict = {k: round(v, 1) for k, v in resDict.items()}
    return resDict


def supplement(map_folder, score, detail):
    def _contains_scene_map_files(scene_dir):
        if not scene_dir or not os.path.isdir(scene_dir):
            return False
        files = os.listdir(scene_dir)
        has_xodr = any(str(f).lower().endswith('.xodr') for f in files)
        has_xosc = any(str(f).lower().endswith('.xosc') for f in files)
        return has_xodr and has_xosc

    def _collect_scene_names_from_map_root(map_root):
        scene_names = set()
        if not os.path.isdir(map_root):
            return []

        for name in os.listdir(map_root):
            p = os.path.join(map_root, name)
            if not os.path.isdir(p):
                continue
            if _contains_scene_map_files(p):
                scene_names.add(name)
                continue
            # one-level nested groups, e.g. A/scenario_xxx
            for sub in os.listdir(p):
                sp = os.path.join(p, sub)
                if os.path.isdir(sp) and _contains_scene_map_files(sp):
                    scene_names.add(sub)
        return sorted(scene_names)

    # 统计所有场景
    scen_folder = map_folder
    frag_folder = os.path.join(scen_folder, 'fragment')
    frag_scens = os.listdir(frag_folder) if os.path.exists(frag_folder) else []
    replay_folder = os.path.join(scen_folder, 'replay')
    replay_scens = os.listdir(replay_folder) if os.path.exists(replay_folder) else []
    serial_folder = os.path.join(scen_folder, 'serial/tasks')
    serial_scens = [i.replace('.json', '').replace('serial_', '') for i in os.listdir(serial_folder)] if os.path.exists(serial_folder) else []
    scens = [{'scenario': i, 'type': 'fragment'} for i in frag_scens] + [{'scenario': i, 'type': 'replay'} for i in replay_scens] + [{'scenario': i, 'type': 'serial'} for i in serial_scens]
    if len(scens) == 0:
        # fallback for direct scene folders (or grouped scene folders) without replay/fragment/serial hierarchy
        direct_scens = _collect_scene_names_from_map_root(scen_folder)
        scens = [{'scenario': i, 'type': 'replay'} for i in direct_scens]

    scensTable = pd.DataFrame(scens)
    all_scenario = scensTable['scenario'].tolist()
    # 补充未测试成功的场景分数
    if 'Scenario' in score.columns:
        test_scens = ['_'.join(i.split('_')[-2:]) for i in score['Scenario'].tolist()]
    else:
        test_scens = []
    supple_scens = [i for i in all_scenario if i not in test_scens]
    for scen in supple_scens:
        type = scensTable.loc[scensTable['scenario'] == scen, 'type'].tolist()[0]
        scen_name = type.upper() + "_supplement_" + scen
        new_score_row = {'Scenario': scen_name, 'Safety': 0, 'Efficiency': 0, 'Comfort': 0, 'Coordination': 0, 'Compliance': 0, 'Total': 0}
        new_detail_row = {'Scenario': scen_name, 'Safety': 0, 'Efficiency': 0, 'Comfort': 0, 'Coordination': 0, 'Compliance': 0, 'Total': 0, 'task_type': type, 'test_state': 0}
        score.loc[len(score)] = new_score_row
        detail.loc[len(detail)] = new_detail_row
    detail.fillna(value=0)
    return score, detail

# wth:逐行写入结果
def dynamic_append(filename, data_dict):
    fieldnames = list(data_dict.keys())
    exist = os.path.exists(filename)
    with open(filename, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not exist:
            writer.writeheader()
        writer.writerow(data_dict)


# wth:组织CSV文件需要的信息
def organizeResults(vehi_regional_score):
    scoreResultList = []
    detailResultList = []
    for key, vehResult in vehi_regional_score.items():
        if not vehResult:
            continue
        for scen, scenResult in vehResult.items():
            if not scenResult:
                continue
            testSceneResult = scenResult.get('testScene', [{}])[0]
            scenario = testSceneResult.get('sceneNameList', scenResult.get('sceneNameList'))
            AbilityDimension = scenResult.get('AbilityDimension', {})
            safe = AbilityDimension.get('safe', 0)
            comfortable = AbilityDimension.get('comfortable', 0)
            effeciency = AbilityDimension.get('efficiency', 0)
            coordination = AbilityDimension.get('coordination', 0)
            compliance = AbilityDimension.get('compliance', 0)
            total = testSceneResult.get('senseScore', 0)
            # total = (AbilityDimension.get('efficiency', 0) ** 0.5) * (AbilityDimension.get('safe*weight', 0) + AbilityDimension.get('comfortable*weight', 0) + AbilityDimension.get('coordination*weight', 0) + AbilityDimension.get('compliance*weight', 0))
            scoreResult = {'Scenario': scenario, 'Safety': safe, 'Efficiency': effeciency, 'Comfort': comfortable, 'Coordination': coordination, 'Compliance': compliance,
                           'Total': total}
            scoreResultList.append(scoreResult)
            task_type = scenario.split('_')[0].lower() if scenario and len(scenario.split('_')) > 0 else None
            test_state = 1
            dynamic_result = 1 if scen == '1' else scenResult.get('flag', 0)
            reason = '' if scen == '1' else scenResult.get('reasons', '')
            penaltyPoint = scenResult.get('penaltyPoint', {})
            testDuration = scenResult.get('testDuration', 1)
            crash = penaltyPoint.get('crashNum', 0)
            ttc_ratio = penaltyPoint.get('TTCAccuTime', 0) / testDuration
            outoflane_ratio = penaltyPoint.get('outOfLane', 0) / testDuration
            subtendlane_ratio = penaltyPoint.get('inSubtendRoad', 0)
            redlight = penaltyPoint.get('breakSignal', 0)
            mission_accomplish = penaltyPoint.get('calMissionAccomplish', 0)
            speed = penaltyPoint.get('avgSpeedNotScore', 0)
            horizontalA_ratio = penaltyPoint.get('overLateralAcce', 0) / testDuration
            horizontalAA_ratio = penaltyPoint.get('overLateralJerk', 0) / testDuration
            verticalA_ratio = penaltyPoint.get('overAcce', 0) / testDuration
            verticalAA_ratio = penaltyPoint.get('overJerk', 0) / testDuration
            turnA_ratio = penaltyPoint.get('unstableSteeringAngleTime', 0) / testDuration
            chartData = testSceneResult.get('chartData', {})
            horizontalA_list = chartData.get('horizontalA', {}).get('y', [0])
            horizontalA = sum(horizontalA_list) / len(horizontalA_list) if len(horizontalA_list) > 0 else 0
            horizontalAA_list = chartData.get('horizontalAPlus', {}).get('y', [0])
            horizontalAA = sum(horizontalAA_list) / len(horizontalAA_list) if len(horizontalAA_list) > 0 else 0
            verticalA_list = chartData.get('verticalA', {}).get('y', [0])
            verticalA = sum(verticalA_list) / len(verticalA_list) if len(verticalA_list) > 0 else 0
            verticalAA_list = chartData.get('verticalAPlus', {}).get('y', [0])
            verticalAA = sum(verticalAA_list) / len(verticalAA_list) if len(verticalAA_list) > 0 else 0
            turnA_list = chartData.get('turnA', {}).get('y', [0])
            turnA = sum(turnA_list) / len(turnA_list) if len(turnA_list) > 0 else 0
            detailResult = {k: v for k, v in scoreResult.items()}
            detailResult.update({'task_type': task_type, 'test_state': test_state,
                                 'dynamic_result': dynamic_result, 'reason': reason, 'testDuration': testDuration,
                                 'crash': crash, 'ttc_ratio': ttc_ratio, 'outoflane_ratio': outoflane_ratio,
                                 'subtendlane_ratio': subtendlane_ratio, 'redlight': redlight,
                                 'mission_accomplish': mission_accomplish,
                                 'speed': speed, 'horizontalA_ratio': horizontalA_ratio,
                                 'horizontalAA_ratio': horizontalAA_ratio, 'verticalA_ratio': verticalA_ratio,
                                 'verticalAA_ratio': verticalAA_ratio, 'turnA_ratio': turnA_ratio,
                                 'horizontalA': horizontalA, 'horizontalAA': horizontalAA, 'verticalA': verticalA,
                                 'verticalAA': verticalAA, 'turn_A': turnA})
            detailResultList.append(detailResult)
    return scoreResultList, detailResultList

if __name__ == "__main__":
    freeze_support()
    trj_file_folder = os.path.join(os.getcwd(), r"testData\fake collision check")
    # map_data_folder = os.path.join(os.getcwd(), r"testData\scenario")
    map_data_folder = r"D:\项目\onsite\2025B整合版"
    print(f"evaluation startTime: {datetime.datetime.now()}")
    res = offline_evaluation(trj_file_folder, map_data_folder)
    print(f"evaluation endTime: {datetime.datetime.now()}")
