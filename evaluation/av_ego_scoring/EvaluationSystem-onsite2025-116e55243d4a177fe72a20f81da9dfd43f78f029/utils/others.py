from datetime import datetime


def makeZeroData(code, avName, caseID, taskID, startTime, endTime, sceneName):
    # #打开并读取JSON文件
    # data_str = open('dataZero.json', encoding="utf-8").read()
    # dataZero = eval(data_str)

    dataZero = {
        'testDuration': 0,
        'dangerTimeProportion': 0,
        'allSenseScore': 0.0,
        'testScene': [
            {
                'senceID': 0,
                'sceneCaseID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
                'sceneNameList': '3紧密跟驰',
                'chartData': {
             'TTC': {'x': [2, 4],
                     'y': [10, 10]},
             'turnA': {'x': [2, 4],
                       'y': [0.11, 0.07]}, 'verticalA': {
                 'x': [2, 4],
                 'y': [0, 0]},
             'horizontalA': {
                 'x': [2, 4],
                 'y': [0.01, 0]},
             'verticalAPlus': {
                 'x': [2, 4],
                 'y': [0, 0]},
             'horizontalAPlus': {
                 'x': [2, 4],
                 'y': [0, 0]}},
                'AbilityDimension': {'safe': 0.0, 'comfortable': 0.0, 'efficiency': 0.0, 'coordination': 0.0,
                              'compliance': 0.0, 'safe*weight': 0.0, 'comfortable*weight': 0.0,
                              'efficiency*weight': 0.0, 'coordination*weight': 0.0, 'compliance*weight': 0.0,
                              'safeWeight': 0.4, 'comfortableWeight': 0.05, 'efficiencyWeight': 0.3,
                              'coordinationWeight': 0.15, 'complianceWeight': 0.1},
                'missionAccomplish': 0,
                'senseScore': 0.0,
                'senseScore*senseWeight': 0.0,
                'senseAggScore': 25.0,
                'senseWeight': 25.0,
                'info': {
            'efficiency': [{'index': '任务完成', 'name': '任务完成', 'score': '0/10', 'time': 0, 'unit': 's'},
                           {'index': '任务耗时', 'name': '任务耗时', 'score': '0/0', 'time': 0, 'unit': 's', 'expectTime': 9999,
                            'overTime': -1.0},
                           {'index': '任务耗时', 'name': '任务耗时', 'score': '0/10', 'avgSpeed': 0, 'unit': 'km/h',
                            'expectSpeed': 30, 'belowSpeed': 0.0},
                           {'index': '倒车', 'name': '非场景需要倒车', 'score': '0.0', 'time': 0.0, 'unit': 's'},
                           {'index': '倒车', 'name': '非场景需要倒车', 'score': '0/10', 'time': 0, 'unit': '次'}],
            'comfortable': [{'index': '横向舒适度', 'name': '横向加速度', 'score': '0.0', 'type': 'a', 'unit': 's', 'time': 0.0},
                            {'index': '横向舒适度', 'name': '横向加加速度', 'score': '0.0', 'type': 'j', 'unit': 's', 'time': 0.0},
                            {'index': '纵向舒适度', 'name': '纵向加速度', 'score': '0.0', 'type': 'a', 'unit': 's', 'time': 0.0},
                            {'index': '纵向舒适度', 'name': '纵向加加速度', 'score': '0.0', 'type': 'j', 'unit': 's', 'time': 0.0},
                            {'index': '转弯舒适度', 'name': '横摆角速度', 'score': '0.0', 'type': 'c', 'unit': 's', 'time': 0.0},],
            'safe': [{'index': '碰撞', 'name': '碰撞', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '安全员介入', 'name': '安全员介入', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '驶出行车道', 'name': '驶出道路边界', 'score': '0.0', 'unit': 's', 'time': 0.0},
                     {'index': '驶出行车道', 'name': '驶出道路边界', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': 'TTC', 'name': 'TTC', 'score': '0.0', 'unit': 's', 'time': 0.0, 'timeRatio：': 0.0, 'timeRatio': 0.0},
                     {'index': 'TTC', 'name': 'TTC', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '横向间距', 'name': '横向间距', 'score': '0.0', 'unit': 's', 'time': 0.0}],
            'compliance': [{'index': '压实线', 'name': '压实线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '压虚线', 'name': '压虚线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '超速', 'name': '超出限速行驶', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '超速', 'name': '超出限速行驶', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '闯红灯', 'name': '闯红灯', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '未遵守停车标志', 'name': '未停车让行', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '驶入对向车道', 'name': '驶入对向车道', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '驶入对向车道', 'name': '驶入对向车道', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '禁行区行驶', 'name': '禁行区行驶', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '停车压停止线', 'name': '停车压停止线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '未按规定车道行驶', 'name': '未按规定车道行驶', 'unit': 's', 'score': '0.0', 'time': 0.0},
                           {'index': '未有效泊车', 'name': '未正常泊车', 'unit': '次', 'score': '0', 'time': 0.0}], 'coordination': [
                {'index': '干扰背景车辆通行', 'name': '干扰背景车辆通行', 'score': '0', 'unit': 's', 'time': 0, 'svSimuTimeWithoutDisturbance': 30,
                 'maxDisturbedSvSimuTime': 0}]},
                'eventTable': {
            '1': {'time': '2024-09-10 16:25:08.092', 'index': '任务未完成', 'place': [0, 0], 'simuTime': 0}}
            }
        ],
        'AbilityDimension': {'safe': 0.0, 'comfortable': 0.0, 'efficiency': 0.0, 'coordination': 0.0,
                                     'compliance': 0.0,
                                     'safe*weight': 0.0, 'comfortable*weight': 0.0, 'efficiency*weight': 0.0,
                                     'coordination*weight': 0.0, 'compliance*weight': 0.0, 'safeWeight': 0.4,
                                     'comfortableWeight': 0.05, 'efficiencyWeight': 0.3, 'coordinationWeight': 0.15,
                                     'complianceWeight': 0.1},
        'gradeTableDict': {'安全性': '不合格', '舒适性': '不合格', '效率性': '不合格', '交通协调性': '不合格', '交规符合性': '不合格'},
        'diagnose': {'安全性方面': '安全性差，存在较严重违规行为，不能保证行车安全。',
                             '舒适性方面': '舒适性差，存在较多急加速/急减速/急转向等行为。',
                             '交互决策方面': '任务耗时较长、效率较低，或任务未完成。',
                             '交通协调性方面': '交通协调性差，存在较多影响其他车辆通行的行为。',
                             '交规符合性方面': '交规符合性差，存在较多违规行为。',
                             '建议': '从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，有必要提升控制规划算法，减少违规次数。 '},
        'taskID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
        'caseID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
        'avName': '8',
        'sceneName': '3紧密跟驰',
        'code': '0',
        'penaltyPoint': {'calMissionAccomplish': 0, 'crash': 0.0, 'crashNum': 1.0, 'isSecurityInvolved': 0.0,
                                 'outOfLane': 0.0, 'outOfLaneNum': 0, 'TTC': 0.0, 'TTCNum': 0, 'TTCAccuTime': 0.0,
                                 'PET': 0.0,
                                 'inSubtendRoad': 0.0, 'inSubtendRoadNum': 0, 'onLaneMarking': 0.0,
                                 'onDottedLaneMarking': 0.0,
                                 'followStopSignal': 0.0, 'followStopSignalNum': 0.0, 'overSpeed': 0.0,
                                 'overSpeedNum': 0,
                                 'breakSignal': 0.0, 'breakSignalNum': 0.0, 'transverseDistance': 0.0,
                                 'inForbiddenArea': 0.0,
                                 'drivingInDesignatedLane': 0.0, 'stopAtStopLine': 0.0, 'overLateralAcce': 0.0,
                                 'overLateralAcceNum': 0.0, 'overLateralJerk': 0.0, 'overLateralJerkNum': 0.0,
                                 'overAcce': 0.0,
                                 'overAcceNum': 0.0, 'overJerk': 0.0, 'overJerkNum': 0.0, 'unstableSteeringAngle': 0.0,
                                 'unstableSteeringAngleTime': 0.0, 'avgSpeed': 0.0, 'gapRefuse': 0, 'timeUsed': 0.0,
                                 'eventTable': {
                                     '1': {'time': '2024-08-08 10:52:02.049', 'index': '任务未完成', 'place': [0, 0],
                                           'simuTime': 0}}, 'avgSpeedNotScore': 10.58, 'useTime': 0,
                                 'reverseCar': 0.0, 'reverseCarNum': 0, 'coordination': 0, 'maxDisturbedSvSimuTime': 0,
                                 'svSimuTimeWithoutDisturbance': 30, 'minCoordinationScore': 0, 'outOfParkingZone': 0},
        'startTime': 1724987913.801,
        'endTime': 1724987960.937
    }

    dt_object = datetime.fromtimestamp(startTime)

    dataZero['code'] = code
    dataZero['avName'] = avName
    dataZero['caseID'] = caseID
    dataZero['taskID'] = taskID
    dataZero['startTime'] = startTime
    dataZero['endTime'] = endTime
    dataZero['sceneName'] = sceneName
    dataZero['testScene'][0]['sceneNameList'] = sceneName
    dataZero['testScene'][0]['sceneCaseID'] = caseID
    dataZero['testScene'][0]['eventTable']['1']['time'] = dt_object.strftime("%Y-%m-%d %H:%M:%S")
    dataZero['penaltyPoint']['eventTable']['1']['time'] = dt_object.strftime("%Y-%m-%d %H:%M:%S")


    return dataZero


def makeZeroDataByParkingError(code, avName, caseID, taskID, startTime, endTime, sceneName):
    # #打开并读取JSON文件
    # data_str = open('dataZero.json', encoding="utf-8").read()
    # dataZero = eval(data_str)

    dataZero = {
        'testDuration': 0,
        'dangerTimeProportion': 0,
        'allSenseScore': 0.0,
        'testScene': [
            {
                'senceID': 0,
                'sceneCaseID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
                'sceneNameList': '3紧密跟驰',
                'chartData': {
             'TTC': {'x': [2, 4],
                     'y': [10, 10]},
             'turnA': {'x': [2, 4],
                       'y': [0.11, 0.07]}, 'verticalA': {
                 'x': [2, 4],
                 'y': [0, 0]},
             'horizontalA': {
                 'x': [2, 4],
                 'y': [0.01, 0]},
             'verticalAPlus': {
                 'x': [2, 4],
                 'y': [0, 0]},
             'horizontalAPlus': {
                 'x': [2, 4],
                 'y': [0, 0]}},
                'AbilityDimension': {'safe': 0.0, 'comfortable': 0.0, 'efficiency': 0.0, 'coordination': 0.0,
                              'compliance': 0.0, 'safe*weight': 0.0, 'comfortable*weight': 0.0,
                              'efficiency*weight': 0.0, 'coordination*weight': 0.0, 'compliance*weight': 0.0,
                              'safeWeight': 0.4, 'comfortableWeight': 0.05, 'efficiencyWeight': 0.3,
                              'coordinationWeight': 0.15, 'complianceWeight': 0.1},
                'missionAccomplish': 0,
                'senseScore': 0.0,
                'senseScore*senseWeight': 0.0,
                'senseAggScore': 25.0,
                'senseWeight': 25.0,
                'info': {
            'efficiency': [{'index': '任务完成', 'name': '任务完成', 'score': '0/10', 'time': 0, 'unit': 's'},
                           {'index': '任务耗时', 'name': '任务耗时', 'score': '0/0', 'time': 0, 'unit': 's', 'expectTime': 9999,
                            'overTime': -1.0},
                           {'index': '任务耗时', 'name': '任务耗时', 'score': '0/10', 'avgSpeed': 0, 'unit': 'km/h',
                            'expectSpeed': 30, 'belowSpeed': 0.0},
                           {'index': '倒车', 'name': '非场景需要倒车', 'score': '0.0', 'time': 0.0, 'unit': 's'},
                           {'index': '倒车', 'name': '非场景需要倒车', 'score': '0/10', 'time': 0, 'unit': '次'}],
            'comfortable': [{'index': '横向舒适度', 'name': '横向加速度', 'score': '0.0', 'type': 'a', 'unit': 's', 'time': 0.0},
                            {'index': '横向舒适度', 'name': '横向加加速度', 'score': '0.0', 'type': 'j', 'unit': 's', 'time': 0.0},
                            {'index': '纵向舒适度', 'name': '纵向加速度', 'score': '0.0', 'type': 'a', 'unit': 's', 'time': 0.0},
                            {'index': '纵向舒适度', 'name': '纵向加加速度', 'score': '0.0', 'type': 'j', 'unit': 's', 'time': 0.0},
                            {'index': '转弯舒适度', 'name': '横摆角速度', 'score': '0.0', 'type': 'c', 'unit': 's', 'time': 0.0},],
            'safe': [{'index': '碰撞', 'name': '碰撞', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '安全员介入', 'name': '安全员介入', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '驶出行车道', 'name': '驶出道路边界', 'score': '0.0', 'unit': 's', 'time': 0.0},
                     {'index': '驶出行车道', 'name': '驶出道路边界', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': 'TTC', 'name': 'TTC', 'score': '0.0', 'unit': 's', 'time': 0.0, 'timeRatio：': 0.0, 'timeRatio': 0.0},
                     {'index': 'TTC', 'name': 'TTC', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '横向间距', 'name': '横向间距', 'score': '0.0', 'unit': 's', 'time': 0.0}],
            'compliance': [{'index': '压实线', 'name': '压实线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '压虚线', 'name': '压虚线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '超速', 'name': '超出限速行驶', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '超速', 'name': '超出限速行驶', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '闯红灯', 'name': '闯红灯', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '未遵守停车标志', 'name': '未停车让行', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '驶入对向车道', 'name': '驶入对向车道', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '驶入对向车道', 'name': '驶入对向车道', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '禁行区行驶', 'name': '禁行区行驶', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '停车压停止线', 'name': '停车压停止线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '未按规定车道行驶', 'name': '未按规定车道行驶', 'unit': 's', 'score': '0.0', 'time': 0.0},
                           {'index': '未有效泊车', 'name': '未正常泊车', 'unit': '次', 'score': '100', 'time': 1}], 'coordination': [
                {'index': '干扰背景车辆通行', 'name': '干扰背景车辆通行', 'score': '0', 'unit': 's', 'time': 0, 'svSimuTimeWithoutDisturbance': 30,
                 'maxDisturbedSvSimuTime': 0}]},
                'eventTable': {
            '1': {'time': '2024-09-10 16:25:08.092', 'index': '任务未完成', 'place': [0, 0], 'simuTime': 0}}
            }
        ],
        'AbilityDimension': {'safe': 0.0, 'comfortable': 0.0, 'efficiency': 0.0, 'coordination': 0.0,
                                     'compliance': 0.0,
                                     'safe*weight': 0.0, 'comfortable*weight': 0.0, 'efficiency*weight': 0.0,
                                     'coordination*weight': 0.0, 'compliance*weight': 0.0, 'safeWeight': 0.4,
                                     'comfortableWeight': 0.05, 'efficiencyWeight': 0.3, 'coordinationWeight': 0.15,
                                     'complianceWeight': 0.1},
        'gradeTableDict': {'安全性': '不合格', '舒适性': '不合格', '效率性': '不合格', '交通协调性': '不合格', '交规符合性': '不合格'},
        'diagnose': {'安全性方面': '安全性差，存在较严重违规行为，不能保证行车安全。',
                             '舒适性方面': '舒适性差，存在较多急加速/急减速/急转向等行为。',
                             '交互决策方面': '任务耗时较长、效率较低，或任务未完成。',
                             '交通协调性方面': '交通协调性差，存在较多影响其他车辆通行的行为。',
                             '交规符合性方面': '交规符合性差，存在较多违规行为。',
                             '建议': '从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，有必要提升控制规划算法，减少违规次数。 '},
        'taskID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
        'caseID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
        'avName': '8',
        'sceneName': '3紧密跟驰',
        'code': '0',
        'penaltyPoint': {'calMissionAccomplish': 0, 'crash': 0.0, 'crashNum': 0.0, 'isSecurityInvolved': 0.0,
                                 'outOfLane': 0.0, 'outOfLaneNum': 0, 'TTC': 0.0, 'TTCNum': 0, 'TTCAccuTime': 0.0,
                                 'PET': 0.0,
                                 'inSubtendRoad': 0.0, 'inSubtendRoadNum': 0, 'onLaneMarking': 0.0,
                                 'onDottedLaneMarking': 0.0,
                                 'followStopSignal': 0.0, 'followStopSignalNum': 0.0, 'overSpeed': 0.0,
                                 'overSpeedNum': 0,
                                 'breakSignal': 0.0, 'breakSignalNum': 0.0, 'transverseDistance': 0.0,
                                 'inForbiddenArea': 0.0,
                                 'drivingInDesignatedLane': 0.0, 'stopAtStopLine': 0.0, 'overLateralAcce': 0.0,
                                 'overLateralAcceNum': 0.0, 'overLateralJerk': 0.0, 'overLateralJerkNum': 0.0,
                                 'overAcce': 0.0,
                                 'overAcceNum': 0.0, 'overJerk': 0.0, 'overJerkNum': 0.0, 'unstableSteeringAngle': 0.0,
                                 'unstableSteeringAngleTime': 0.0, 'avgSpeed': 0.0, 'gapRefuse': 0, 'timeUsed': 0.0,
                                 'eventTable': {
                                     '1': {'time': '2024-08-08 10:52:02.049', 'index': '任务未完成', 'place': [0, 0],
                                           'simuTime': 0}}, 'avgSpeedNotScore': 10.58, 'useTime': 0,
                                 'reverseCar': 0.0, 'reverseCarNum': 0, 'coordination': 0, 'maxDisturbedSvSimuTime': 0,
                                 'svSimuTimeWithoutDisturbance': 30, 'minCoordinationScore': 0, 'outOfParkingZone': 1},
        'startTime': 1724987913.801,
        'endTime': 1724987960.937
    }

    dt_object = datetime.fromtimestamp(startTime)

    dataZero['code'] = code
    dataZero['avName'] = avName
    dataZero['caseID'] = caseID
    dataZero['taskID'] = taskID
    dataZero['startTime'] = startTime
    dataZero['endTime'] = endTime
    dataZero['sceneName'] = sceneName
    dataZero['testScene'][0]['sceneNameList'] = sceneName
    dataZero['testScene'][0]['sceneCaseID'] = caseID
    dataZero['testScene'][0]['eventTable']['1']['time'] = dt_object.strftime("%Y-%m-%d %H:%M:%S")
    dataZero['penaltyPoint']['eventTable']['1']['time'] = dt_object.strftime("%Y-%m-%d %H:%M:%S")


    return dataZero





# 整一个满分的
def makePerfectData(code, avName, caseID, taskID, startTime, endTime, sceneName):
    # #打开并读取JSON文件
    # data_str = open('dataZero.json', encoding="utf-8").read()
    # dataZero = eval(data_str)
    dataZero = {
        'testDuration': 0,
        'dangerTimeProportion': 0,
        'allSenseScore': 100,
        'testScene': [
            {
                'senceID': 0,
                'sceneCaseID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
                'sceneNameList': '3紧密跟驰',
                'chartData': {
                     'TTC': {'x': [2, 4],
                             'y': [10, 10]},
                     'turnA': {'x': [2, 4],
                               'y': [0.11, 0.07]},
                     'verticalA': {
                         'x': [2, 4],
                         'y': [0, 0]},
                     'horizontalA': {
                         'x': [2, 4],
                         'y': [0.01, 0]},
                     'verticalAPlus': {
                         'x': [2, 4],
                         'y': [0, 0]},
                     'horizontalAPlus': {
                         'x': [2, 4],
                         'y': [0, 0]}},
                'AbilityDimension': {'safe': 100.0, 'comfortable': 100.0, 'efficiency': 100, 'coordination': 100.0,
                                      'compliance': 100.0, 'safe*weight': 40.0, 'comfortable*weight': 5.0,
                                      'efficiency*weight': 30, 'coordination*weight': 15.0, 'compliance*weight': 10.0,
                                      'safeWeight': 0.4, 'comfortableWeight': 0.05, 'efficiencyWeight': 0.3,
                                      'coordinationWeight': 0.15, 'complianceWeight': 0.1},
                'missionAccomplish': 0,
                'senseScore': 100,
                'senseScore*senseWeight': 25,
                'senseAggScore': 25.0,
                'senseWeight': 25.0,
                'info': {
            'efficiency': [{'index': '任务完成', 'name': '任务完成', 'score': '10/10', 'time': 0, 'unit': 's'},
                           {'index': '任务耗时', 'name': '任务耗时', 'score': '0/0', 'time': 0, 'unit': 's', 'expectTime': 9999,
                            'overTime': -1.0},
                           {'index': '任务耗时', 'name': '任务耗时', 'score': '10/10', 'avgSpeed': 30, 'unit': 'km/h',
                            'expectSpeed': 30, 'belowSpeed': 0.0},
                           {'index': '倒车', 'name': '非场景需要倒车', 'score': '0.0', 'time': 0.0, 'unit': 's'},
                           {'index': '倒车', 'name': '非场景需要倒车', 'score': '10/10', 'time': 0, 'unit': 's'}],
            'comfortable': [{'index': '横向舒适度', 'name': '横向加速度', 'score': '0.0', 'type': 'a', 'unit': 's', 'time': 0.0},
                            {'index': '横向舒适度', 'name': '横向加加速度', 'score': '0.0', 'type': 'j', 'unit': 's', 'time': 0.0},
                            {'index': '纵向舒适度', 'name': '纵向加速度', 'score': '0.0', 'type': 'a', 'unit': 's', 'time': 0.0},
                            {'index': '纵向舒适度', 'name': '纵向加加速度', 'score': '0.0', 'type': 'j', 'unit': 's', 'time': 0.0},
                            {'index': '转弯舒适度', 'name': '横摆角速度', 'score': '0.0', 'type': 'c', 'unit': 's', 'time': 0.0},],
            'safe': [{'index': '碰撞', 'name': '碰撞', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '安全员介入', 'name': '安全员介入', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '驶出行车道', 'name': '驶出道路边界', 'score': '0.0', 'unit': 's', 'time': 0.0},
                     {'index': '驶出行车道', 'name': '驶出道路边界', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': 'TTC', 'name': 'TTC', 'score': '0.0', 'unit': 's', 'time': 0.0, 'timeRatio：': 0.0, 'timeRatio': 0.0},
                     {'index': 'TTC', 'name': 'TTC', 'score': '0.0', 'unit': '次', 'time': 0},
                     {'index': '横向间距', 'name': '横向间距', 'score': '0.0', 'unit': 's', 'time': 0.0}],
            'compliance': [{'index': '压实线', 'name': '压实线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '压虚线', 'name': '压虚线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '超速', 'name': '超出限速行驶', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '超速', 'name': '超出限速行驶', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '闯红灯', 'name': '闯红灯', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '未遵守停车标志', 'name': '未停车让行', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '驶入对向车道', 'name': '驶入对向车道', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '驶入对向车道', 'name': '驶入对向车道', 'score': '0.0', 'unit': '次', 'time': 0},
                           {'index': '禁行区行驶', 'name': '禁行区行驶', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '停车压停止线', 'name': '停车压停止线', 'score': '0.0', 'unit': 's', 'time': 0.0},
                           {'index': '未按规定车道行驶', 'name': '未按规定车道行驶', 'unit': 's', 'score': '0.0', 'time': 0.0},
                           {'index': '未有效泊车', 'name': '未正常泊车', 'unit': '次', 'score': '0', 'time': 0.0}], 'coordination': [
                {'index': '干扰背景车辆通行', 'name': '干扰背景车辆通行', 'score': '0', 'unit': 's', 'time': 0, 'svSimuTimeWithoutDisturbance': 30,
                 'maxDisturbedSvSimuTime': 0}]},
                'eventTable': {}
            }
        ],
        'AbilityDimension': {'safe': 100.0, 'comfortable': 100.0, 'efficiency': 100, 'coordination': 100.0,
                                  'compliance': 100.0, 'safe*weight': 40.0, 'comfortable*weight': 5.0,
                                  'efficiency*weight': 30, 'coordination*weight': 15.0, 'compliance*weight': 10.0,
                                  'safeWeight': 0.4, 'comfortableWeight': 0.05, 'efficiencyWeight': 0.3,
                                  'coordinationWeight': 0.15, 'complianceWeight': 0.1},
        'gradeTableDict': {'安全性': '优秀', '舒适性': '优秀', '效率性': '优秀', '交通协调性': '优秀', '交规符合性': '优秀'},
        'diagnose': {'安全性方面': '安全性好，不存在违规行为。', '舒适性方面': '舒适性好，不存在急加速/急减速/急转向等行为。',
                          '交互决策方面': '任务完成，且效率较高。',
                          '交通协调性方面': '交通协调性好，不存在影响其他车辆通行的行为。',
                          '交规符合性方面': '交规符合性好，不存在较多违规行为。',
                          '建议': '从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，算法很高效，无提升建议。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 '},
        'taskID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
        'caseID': 'ff27dd5b-667e-11ef-8347-00163e21f0fc',
        'avName': '8',
        'sceneName': '3紧密跟驰',
        'code': 0,
        'penaltyPoint': {'PET': 0.9129540068577612, 'TTC': 0, 'crash': 0, 'useTime': 0, 'avgSpeed': 0,
                              'crashNum': 0, 'overAcce': 0, 'overJerk': 0, 'timeUsed': 0, 'gapRefuse': 0, 'outOfLane': 0,
                              'overSpeed': 0, 'eventTable': {}, 'reverseCar': 0, 'TTCAccuTime': 0, 'breakSignal': 0,
                              'overAcceNum': 0, 'overJerkNum': 0, 'coordination': 0, 'inSubtendRoad': 0, 'onLaneMarking': 0,
                              'breakSignalNum': 0, 'stopAtStopLine': 0, 'inForbiddenArea': 0, 'overLateralAcce': 0,
                              'overLateralJerk': 0, 'avgSpeedNotScore': 30, 'followStopSignal': 0, 'isSecurityInvolved': 0,
                              'overLateralAcceNum': 0, 'overLateralJerkNum': 0, 'transverseDistance': 0,
                              'followStopSignalNum': 0, 'onDottedLaneMarking': 0, 'calMissionAccomplish': 1,
                              'minCoordinationScore': 0, 'unstableSteeringAngle': 0, 'maxDisturbedSvSimuTime': 0, 'outOfParkingZone': 0,
                              'drivingInDesignatedLane': 0, 'unstableSteeringAngleTime': 0, 'svSimuTimeWithoutDisturbance': 30},
        'startTime': 1724987913.801,
        'endTime': 1724987960.937
    }

    dt_object = datetime.fromtimestamp(startTime)

    dataZero['code'] = code
    dataZero['avName'] = avName
    dataZero['caseID'] = caseID
    dataZero['taskID'] = taskID
    dataZero['startTime'] = startTime
    dataZero['endTime'] = endTime
    dataZero['sceneName'] = sceneName
    dataZero['testScene'][0]['sceneNameList'] = sceneName
    dataZero['testScene'][0]['sceneCaseID'] = caseID
    # dataZero['testScene'][0]['eventTable']['1']['time'] = dt_object.strftime("%Y-%m-%d %H:%M:%S")
    # dataZero['penaltyPoint']['eventTable']['1']['time'] = dt_object.strftime("%Y-%m-%d %H:%M:%S")


    return dataZero