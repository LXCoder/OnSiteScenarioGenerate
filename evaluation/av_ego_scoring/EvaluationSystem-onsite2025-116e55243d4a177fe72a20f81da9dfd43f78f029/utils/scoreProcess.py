# -*- coding: utf-8 -*-

import json

import json
from evaluateUtils.standard_parameter import Parameter
from evaluateUtils.AutoSelfDrivingCar import SelfDrivingCar, Car, TTCCar, PETCar, np
from datetime import datetime


# 在写一个新的函数来代替原有的startScoreProcess
# 这个函数里面会判断第一次出现碰撞的场景的时间，
# 然后找这个场景后面的所有场景的id
# 然后在这些场景中都加一个碰撞的事件，碰撞次数都修改掉
# 最后调用reCalScore函数，返回新的值就行了
def startScoreProcessWithCrashCheck(vehi_regional_score, regionals, projectWeight, taskID, missonStartTime, missonEndTime, vehID):
    # 首先对vehi_regional_score原始数据进行解析，查找出现碰撞的场景，并且找到他们对应场景的开始时间
    crashSceneStartTimeDict = {}
    if vehID in vehi_regional_score:
        value_car = vehi_regional_score[vehID]
        for scene_id, value_score in value_car.items():
            for sceneI_regional_score in vehi_regional_score[vehID][scene_id]:
                sceneI_regional_score = sceneI_regional_score['1']
                for sceneI_score_info_dict in sceneI_regional_score['testScene']:
                    safe_info_list = sceneI_score_info_dict['info']['safe']
                    for sceneI_indexJ in safe_info_list:
                        if sceneI_indexJ['index'] == '碰撞':
                            if sceneI_indexJ['time'] > 0:
                                crashSceneStartTimeDict[scene_id] = {
                                    'sceneStartTime': sceneI_regional_score['startTime'],
                                    'sceneName': sceneI_regional_score['sceneName'],
                                    'crashTime': sceneI_indexJ['time'],
                                    'sceneEndTime': sceneI_regional_score['endTime'],
                                }
    # 第二步是把这个找到所有碰撞的场景，找到最早的场景，并记录这个最早出现碰撞场景的开始时间
    fristCrashSceneStartTime = 99999999999999
    fristCrashSceneID = None
    for sceneI_id, sceneI_info in crashSceneStartTimeDict.items():
        if fristCrashSceneStartTime > sceneI_info['sceneStartTime']:
            fristCrashSceneStartTime = sceneI_info['sceneStartTime']
            fristCrashSceneID = sceneI_id

    # 第三步把这个时间后面的所有场景id都找出来，并且构造一个newScore分数
    newScore = {
        vehID: {}
    }
    changeZeroSceneList = []
    if vehID in vehi_regional_score:
        value_car = vehi_regional_score[vehID]
        for scene_id, value_score in value_car.items():
            newScore[vehID][scene_id] = []
            for sceneI_regional_score in vehi_regional_score[vehID][scene_id]:
                sceneI_start_time = sceneI_regional_score['1']['startTime']
                if sceneI_start_time >= fristCrashSceneStartTime:
                    if scene_id in crashSceneStartTimeDict:
                    # if 0:
                        pass
                    else:
                        scoreDict = {
                            "time": crashSceneStartTimeDict[fristCrashSceneID]['crashTime'],
                            "unit": "s",
                            "index": "碰撞",
                            "score": "0"
                        }
                        changeDict = {
                            '1': [scoreDict]
                        }
                        newScore[vehID][scene_id].append(changeDict)
                        changeZeroSceneList.append(scene_id)

    # 整理一下newScore，把里面场景空的数据清除掉
    delList = []
    for scene_id, value_score in newScore[vehID].items():
        if not value_score:
            delList.append(scene_id)
    for scene_id in delList:
        del newScore[vehID][scene_id]

    # 到这里时可以找出来需要变成0分的场景的，那么在这里直接对0分进行修改也是可以的
    # 先调用startScoreProcess这个函数，拿到正常的分数，然后对需要改变的分数进行调整
    totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar = startScoreProcess(vehi_regional_score, regionals, projectWeight, taskID, missonStartTime,missonEndTime, vehID)

    # 然后对需要为0的场景分数进行改变
    # 先对单个场景进行处理
    for scene_id, score_value in sceneResultDataDictSplitByCar.items():
        id, index = scene_id.split("_")
        if id in changeZeroSceneList:
            sceneResultDataDictSplitByCar[scene_id] = makeZeroSceneScore(sceneResultDataDictSplitByCar[scene_id])

    # 再对总分的所有场景进行处理
    totalResultDataDictSplitByCar = makeZeroTotalScore(totalResultDataDictSplitByCar, changeZeroSceneList)

    return totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar


def makeZeroSceneScore(sceneResultDataDict):
    sceneResultDataDict['allSenseScore'] = 0
    sceneResultDataDict['testScene'][0]['AbilityDimension'] = {'safe': 0.0, 'comfortable': 0.0, 'efficiency': 0.0, 'coordination': 0.0, 'compliance': 0.0, 'safe*weight': 0.0, 'comfortable*weight': 0.0, 'efficiency*weight': 0.0, 'coordination*weight': 0.0, 'compliance*weight': 0.0, 'safeWeight': 0.4, 'comfortableWeight': 0.05, 'efficiencyWeight': 0.3, 'coordinationWeight': 0.15, 'complianceWeight': 0.1}
    sceneResultDataDict['AbilityDimension'] = {'safe': 0.0, 'comfortable': 0.0, 'efficiency': 0.0, 'coordination': 0.0, 'compliance': 0.0, 'safe*weight': 0.0, 'comfortable*weight': 0.0, 'efficiency*weight': 0.0, 'coordination*weight': 0.0, 'compliance*weight': 0.0, 'safeWeight': 0.4, 'comfortableWeight': 0.05, 'efficiencyWeight': 0.3, 'coordinationWeight': 0.15, 'complianceWeight': 0.1}
    sceneResultDataDict['testScene'][0]['senseScore'] = 0
    sceneResultDataDict['testScene'][0]['senseScore*senseWeight'] = 0
    sceneResultDataDict['gradeTableDict'] = {'安全性': '不合格', '舒适性': '不合格', '效率性': '不合格', '交通协调性': '不合格', '交规符合性': '不合格'}
    sceneResultDataDict['diagnose'] = {'安全性方面': '安全性差，存在较严重违规行为，不能保证行车安全。', '舒适性方面': '舒适性差，存在较多急加速/急减速/急转向等行为。', '交互决策方面': '任务耗时较长、效率较低，或任务未完成。', '交通协调性方面': '交通协调性差，存在较多影响其他车辆通行的行为。', '交规符合性方面': '交规符合性差，存在较多违规行为。', '建议': '从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，有必要提升控制规划算法，减少违规次数。 '}
    return sceneResultDataDict

def makeZeroTotalScore(totalResultDataDict, changeZeroSceneList):
    sceneResultDataDict = totalResultDataDict['testScene']
    allSenseScore = 0
    AbilityDimension = {'safe': 0, 'efficiency': 0, 'comfortable': 0, 'coordination': 0, 'compliance': 0, 'else': 0, 'allScore': 0}
    abilityDimension = {'safe': 0, 'efficiency': 0, 'comfortable': 0, 'coordination': 0, 'compliance': 0, 'else': 0, 'allScore': 0}
    gradeTableDict = {}
    scoreTableDict = {}
    diagnoseDict = {}
    for value in sceneResultDataDict:
        id, index = value['senceID'].split("_")
        if id in changeZeroSceneList:
            value['AbilityDimension'] = {'safe': 0.0, 'comfortable': 0.0, 'efficiency': 0.0, 'coordination': 0.0, 'compliance': 0.0, 'safe*weight': 0.0, 'comfortable*weight': 0.0, 'efficiency*weight': 0.0, 'coordination*weight': 0.0, 'compliance*weight': 0.0, 'safeWeight': 0.4, 'comfortableWeight': 0.05, 'efficiencyWeight': 0.3, 'coordinationWeight': 0.15, 'complianceWeight': 0.1}
            value['senseScore'] = 0
            value['senseScore*senseWeight'] = 0
        else:
            # id不在证明要加正常分数
            allSenseScore += value['senseScore*senseWeight']
            for key_dimension, score in AbilityDimension.items():
                if key_dimension in value['AbilityDimension']:
                    AbilityDimension[key_dimension] += value['AbilityDimension'][key_dimension] * value['senseWeight'] / 100
                    abilityDimension[key_dimension] += value['AbilityDimension'][key_dimension] * value['senseWeight'] / 100
        index_key = value['sceneNameList'] + '/' + value['senceID']
        AbilityDimensionI = value['AbilityDimension']
        safeGrade = gradeScore(98, 95, 90, 80, AbilityDimensionI["safe"])
        comfortableGrade = gradeScore(95, 88, 75, 60, AbilityDimensionI["comfortable"])
        efficiencyGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["efficiency"])
        coordinationGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["coordination"])
        complianceGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["compliance"])
        # gradeTableDict[sceneNameDict[sceneItotalScore['senceID']]+str(sceneItotalScore['senceID'])] = {
        gradeTableDict[index_key] = {
            "安全性": safeGrade,
            "舒适性": comfortableGrade,
            "效率性": efficiencyGrade,
            "交通协调性": coordinationGrade,
            "交规符合性": complianceGrade, }
        # scoreTableDict[sceneNameDict[sceneItotalScore['senceID']]+str(sceneItotalScore['senceID'])] = {
        scoreTableDict[index_key] = {
            "安全性": {'score': AbilityDimensionI["safe"], 'grade': safeGrade},
            "舒适性": {'score': AbilityDimensionI["comfortable"], 'grade': comfortableGrade},
            "效率性": {'score': AbilityDimensionI["efficiency"], 'grade': efficiencyGrade},
            "交通协调性": {'score': AbilityDimensionI["coordination"], 'grade': coordinationGrade},
            "交规符合性": {'score': AbilityDimensionI["compliance"], 'grade': complianceGrade}, }
    AbilityDimensionForDiagnose = {"AbilityDimension": AbilityDimension}
    diagnoseLibrary = diagnose(AbilityDimensionForDiagnose)
    diagnoseLibrary.getSuggestion()
    diagnoseDict = diagnoseLibrary.diagnoseSuggestionDict
    AbilityDimension['allScore'] = allSenseScore
    abilityDimension['allScore'] = allSenseScore

    # 然后是处理外层的总分，循环加一下里面的总分就行了
    totalResultDataDict['allSenseScore'] = allSenseScore
    totalResultDataDict['AbilityDimension'] = AbilityDimension
    totalResultDataDict['abilityDimension'] = abilityDimension
    totalResultDataDict['gradeTableDict'] = gradeTableDict
    totalResultDataDict['scoreTableDict'] = scoreTableDict
    totalResultDataDict['diagnose'] = diagnoseDict

    return totalResultDataDict


def abilityDimensionIProcess(sceneItotalScore, allWeight):
    num = 2
    AbilityDimensionI = {}
    AbilityDimensionI["safe"] = round(sceneItotalScore['safe100'] / allWeight.scoreWeight['safe'], num)
    AbilityDimensionI["comfortable"] = round(sceneItotalScore['comfortable100'] / allWeight.scoreWeight['comfortable'],num)
    AbilityDimensionI["efficiency"] = round(sceneItotalScore['efficiency100'] / allWeight.scoreWeight['efficiency'],num)
    AbilityDimensionI["coordination"] = round(sceneItotalScore['coordination100'] / allWeight.scoreWeight['coordination'], num)
    AbilityDimensionI["compliance"] = round(sceneItotalScore['compliance100'] / allWeight.scoreWeight['compliance'],num)
    AbilityDimensionI["safe*weight"] = round(sceneItotalScore['safe100'], num)
    AbilityDimensionI["comfortable*weight"] = round(sceneItotalScore['comfortable100'], num)
    AbilityDimensionI["efficiency*weight"] = round(sceneItotalScore['efficiency100'], num)
    AbilityDimensionI["coordination*weight"] = round(sceneItotalScore['coordination100'], num)
    AbilityDimensionI["compliance*weight"] = round(sceneItotalScore['compliance100'], num)
    AbilityDimensionI["safeWeight"] = round(allWeight.scoreWeight['safe'], num)
    AbilityDimensionI["comfortableWeight"] = round(allWeight.scoreWeight['comfortable'], num)
    AbilityDimensionI["efficiencyWeight"] = round(allWeight.scoreWeight['efficiency'], num)
    AbilityDimensionI["coordinationWeight"] = round(allWeight.scoreWeight['coordination'], num)
    AbilityDimensionI["complianceWeight"] = round(allWeight.scoreWeight['compliance'], num)
    return AbilityDimensionI

def scoreAccuDictProcess(scoreI, Ego, sceneI_regional_score):
    checkFromEventTableIndexList = ['碰撞', '安全员介入', 'TTC', '驶入对向车道', '驶出行车道', '倒车', '超速']
    # 对负数进行一下保护
    if 'time' in scoreI:
        if float(scoreI['time']) >= 0:
            pass
        else:
            scoreI['time'] = str(0)

    # 对于累积时间的进行处理
    if 'time' in scoreI:
        if scoreI['index'] in Parameter.timeIndexDict:
            newIndex = scoreI['index'] + '累积时间'
            if scoreI['unit'] == 's':
                Ego.scoreAccuTimeDict[newIndex] = float(scoreI['time'])
        # 对于累积次数的进行处理
        if scoreI['index'] in Parameter.numIndexDict:
            newIndex = scoreI['index'] + '累积次数'
            if scoreI['unit'] == '次':
                Ego.scoreAccuNumDict[newIndex] = float(scoreI['time'])
    # 对与舒适度相关的
    if 'type' in scoreI:
        if scoreI['index'] == '横向舒适度':
            if scoreI['type'] == 'a':
                Ego.scoreAccuTimeDict['横向加速度累积时间'] = float(scoreI['time'])
                Ego.scoreAccuNumDict['横向加速度累积次数'] = float(scoreI['time'])
            if scoreI['type'] == 'j':
                Ego.scoreAccuTimeDict['横向加加速度累积时间'] = float(scoreI['time'])
                Ego.scoreAccuNumDict['横向加加速度累积次数'] = float(scoreI['time'])
        if scoreI['index'] == '纵向舒适度':
            if scoreI['type'] == 'a':
                Ego.scoreAccuTimeDict['纵向加速度累积时间'] = float(scoreI['time'])
                Ego.scoreAccuNumDict['纵向加速度累积次数'] = float(scoreI['time'])
            if scoreI['type'] == 'j':
                Ego.scoreAccuTimeDict['纵向加加速度累积时间'] = float(scoreI['time'])
                Ego.scoreAccuNumDict['纵向加加速度累积次数'] = float(scoreI['time'])
        if scoreI['index'] == '转弯舒适度':
            Ego.scoreAccuTimeDict['转向角变化累积时间'] = float(scoreI['time'])
    # 对泊车相关的指标进行操作
    if 'time' in scoreI:
        if scoreI['index'] == '未有效泊车':
            Ego.scoreAccuNumDict["未有效泊车累积次数"] = float(scoreI['time'])
    # 对效率相关的指标进行操作
    # 首先在原始分数里面找到时间和平均速度算出来距离
    regional_efficiency_score = sceneI_regional_score['testScene'][0]['info']['efficiency']
    regional_time = 0
    regional_speed = 0
    for indexI in regional_efficiency_score:
        if indexI['index'] == '任务完成':
            regional_time = indexI['time']
        if indexI['index'] == '任务耗时' and indexI['unit'] == 'km/h':
            regional_speed = indexI['avgSpeed']
    regional_distance = regional_time * regional_speed / 3.6
    new_time = 9999
    if 'time' in scoreI:
        if scoreI['index'] == '任务完成':
            new_time = scoreI['time']
    new_speed = regional_distance / new_time
    new_speedKM = new_speed * 3.6
    Ego.sceneAvgSpeedKM = round(new_speedKM, 2)
    Ego.sceneAvgSpeed = round(new_speed, 2)


    # 对碰撞和安全员介入进行特殊处理，这两个是通过eventTable来做的
    # 现在里面的次数都是按照EventTable的数量来计算的，所以这里只能来对EventTable做一个整体性的处理
    # 就是先看这个指标是哪个，然后找到现有EventTable中的这个指标，把那些都删除，然后放入自己的重置的
    # 这里先把改的都放进来，后面有专门的把之前的校核后再加进来
    # 还需要在这里多做一步，如果原来Ego中已经有了eventtable，需要判断当前的这个新的指标的次数和原来eventtable的次数
    # 如果比他大就替换，如果比他小，就重新删除多的
    if scoreI['index'] in checkFromEventTableIndexList and scoreI['unit'] == '次':
        if int(scoreI['time']):
            # 把次数改大，原有表格中的数据，小于新的传进来的数据
            if checkCrashTime(Ego.eventTable, scoreI['index']) <= int(scoreI['time']):
                for i in range(int(scoreI['time'])):
                    eventTableProcess(scoreI, Ego, i, 0)
            else:
                # 改小的时候，主要就是删除，把表格这个指标置空，然后重新写进来就行了
                table_copy = Ego.eventTable.copy()
                for key, value in table_copy.items():
                    if value['index'] == scoreI['index']:
                        del Ego.eventTable[key]
                for i in range(int(scoreI['time'])):
                    eventTableProcess(scoreI, Ego, i, 0)
        else:
            # 表示，这次表格中的这个数据需要是0次
            eventTableProcess(scoreI, Ego, 0, 1)

def eventTableProcess(scoreI, Ego, i, isZero):
    if scoreI['index'] == '碰撞':
        num = '1000' + str(i + 1)
    elif scoreI['index'] == '安全员介入':
        num = '2000' + str(i + 1)
    elif scoreI['index'] == 'TTC':
        num = '3000' + str(i + 1)
    elif scoreI['index'] == '驶入对向车道':
        num = '4000' + str(i + 1)
    elif scoreI['index'] == '驶出行车道':
        num = '5000' + str(i + 1)
    elif scoreI['index'] == '倒车':
        num = '6000' + str(i + 1)
    elif scoreI['index'] == '超速':
        num = '7000' + str(i + 1)
    else:
        num = '0'
    if isZero:
        simuTime = 9999
    else:
        simuTime = 1
    Ego.eventTable[num] = {
        "time": str(datetime.now())[:23],
        "index": scoreI['index'],
        "place": [0, 0],
        "simuTime": simuTime
    }

# 输入两个事件表格，在原始表格的基础上删除一些check_eventTable中相同的元素，返回一个新的表格
# check_eventTable中有所有sumtime为0的
# 这里应该是分成3步，1把check_eventTable中为0的指标找出了，不为0的找出来；2是把不为0的和之前的进行合并；3是对合并后的表格进行为0的数据的删除
def eventTableCheck(regional_eventTable, check_eventTable):
    # 先找出来check_eventTable中所有的指标，放在一个list里面
    copy_eventTable = regional_eventTable.copy()

    checkIndexList = []
    checkZeroIndexList = []
    for key, value in check_eventTable.items():
        if value['simuTime'] == 9999:
            checkZeroIndexList.append(value['index'])
        else:
            checkIndexList.append(value['index'])

    # 是把不为0的和之前的进行合并
    for key, value in copy_eventTable.items():
        if value['index'] in checkIndexList:
            del regional_eventTable[key]
    check_eventTable.update(regional_eventTable)

    copy_eventTable_aim = check_eventTable.copy()
    # 最后把copy_eventTable_aim中的所有checkZeroIndexList中的index的删除
    for key, value in copy_eventTable_aim.items():
        if value['index'] in checkZeroIndexList:
            del check_eventTable[key]

    return check_eventTable


# 根据传入的参数重新计算分数，传入的参数是，某次测试，某个场景的某个指标的时间/次数，还有原来的分数，还有一个场景权重，三个入参，
# 根据第一个入参去改第二个分数中的某个元素，然后重新得出一个分数
# 第一步要先找到要改的指标，
# 第二步根据具体指标计算出新的penaltyPoint
# 第三步更新原有分数中的penaltyPoint中对应的那个指标
# 然后就重复第二步，第三步，把所有的指标都找到
# 这里最恶心的地方在于，要针对每个指标都写一个规则，或者说是有2套大的加一些其他，写两个列表把指标都存在里面，看进来的index是否和列表中的一致，再去找乘谁，或者替换谁
# 还要在写一个对应的字典index - 英文，index就是我前端上面的那个，英文就是我那个penaltyPoint的
# 在处理完上面的后，还要计算出来新的能力分数
# 其实这个函数也就是相当于把auto那个里面的函数calcuAllScore重新走一遍
# 那其实要不然就直接调用那个函数就行了，把一些self.dict补上，这个好像是更简单的，那就第二步计算新的penaltyPoint改成调用函数，这样可以一次性把所有要的分数都算出来
# 那么重点就是对那几个dict的改造，把指标中的数据变成dict中的数据
# 最后在调用一下下面的这个函数输出一个总的一个分的分数就行了
def reCalScore(newScore, vehi_regional_score, regionals, projectWeight, taskID, missonStartTime, missonEndTime, vehID):
    # 实例化一个自动驾驶车，用来计算分数
    Ego = SelfDrivingCar(id=0, Xpos=0, Ypos=0, speed=0, roadType=0, startPos=0)

    regionals = {}
    # 处理权重数据
    if not regionals:
        if vehID in vehi_regional_score:
            value_car = vehi_regional_score[vehID]
            for key_case, value_case in value_car.items():
                # 在这里先把场景的分数处理一下，这里key_case是“1”，“2”，“3”这种
                if key_case not in regionals:
                    regionals[int(key_case)] = {'weights': 1}

    # todo 这里是按照默认值来计算的权重，后面实际上还需要一个按照他们给的权重计算权重的
    allWeight = standardWeightProcess(Parameter.weightData['indexWeightDict'], regionals)

    # 处理Ego的积累的dict
    if vehID in vehi_regional_score:
        for scene_id, value_score in newScore.items():
            if scene_id in vehi_regional_score[vehID]:
                # 对valuescore中的指标和新的分数进行处理，valuescore是一个列表，如果更新一个指标，这里就传一个字典，两个指标就两个字典，每个字典的内容包括index、score、time、type
                sceneI_regional_score = vehi_regional_score[vehID][scene_id][0]['1']
                new_time = None
                for key, value in sceneI_regional_score['testScene'][0]['info'].items():
                    for scoreI in value:
                        scoreAccuDictProcess(scoreI, Ego, sceneI_regional_score)
                for scene_time_score_i in value_score:
                    for scoreI in scene_time_score_i['1']:
                        scoreAccuDictProcess(scoreI, Ego, sceneI_regional_score)
                        if scoreI['index'] == '任务完成':
                            new_time = scoreI['time']
                # 处理任务完成相关的内容
                Ego.calMissionAccomplish = sceneI_regional_score['penaltyPoint']['calMissionAccomplish']
                # 处理协作相关的字典
                Ego.disturbedSvSimuTimeDict[0] = sceneI_regional_score['penaltyPoint']['maxDisturbedSvSimuTime']
                # 处理效率相关的内容
                # 因为效率在里面做了，这里就要关闭
                # Ego.sceneAvgSpeedKM = sceneI_regional_score['penaltyPoint']['avgSpeedNotScore']
                # 最后需要拿到 missonTime, expectTime, senceId, weight, allWeight这些数值就行了
                if new_time == None:
                    missonTime = sceneI_regional_score['penaltyPoint']['useTime']
                else:
                    missonTime = new_time
                expectTime = 0
                for i in sceneI_regional_score['testScene'][0]['info']['efficiency']:
                    if i['index'] == '任务耗时' and i['unit'] == "s":
                        expectTime = i['expectTime']
                senceId = sceneI_regional_score['testScene'][0]['senceID']
                weight = allWeight.sceneWeight[scene_id]

                # 然后是放到原来函数里面开始
                senceIscore, totallSenceIscore = Ego.calcuAllScore(missonTime, expectTime, senceId, weight, allWeight)
                senceIscore['useTime'] = missonTime
                senceIscore['avgSpeedNotScore'] = Ego.sceneAvgSpeedKM
                senceIscore['penaltyPoint']['avgSpeedNotScore'] = Ego.sceneAvgSpeedKM
                # 从senceIscore拿出必要的东西penaltyPoint，和totallSenceIscore的总分，去更新vehi_regional_score中的分数
                # 这里需要对eventTable做一个特殊的处理，因为如果更改的目标有这个次数那么之前的eventTable中有对应次数就不能再加进来了，所以这里要分情况更新，写一个函数来做
                # 这里还需要考虑如果改动是0的话，怎么把原有的表格中对应的删除
                # 在前面找到了如果没有这个事件在eventtable里面，那么10001~70001，simuTime = 9999，找到这样的数据，记录他们的index，然后删除所有index为记录的
                # 最后把这个表格作为最新的表格记录进来
                regional_eventTable = vehi_regional_score[vehID][scene_id][0]['1']['penaltyPoint']['eventTable']
                # 在这里更新的刚刚处理过的新的penaltyPoint，也包括更改过的eventTable
                vehi_regional_score[vehID][scene_id][0]['1']['penaltyPoint'] = senceIscore['penaltyPoint']
                # 这里通过一个函数把老的regional_eventTable中和新的eventTable中相同的指标去掉，然后返回一个新的，更新
                vehi_regional_score[vehID][scene_id][0]['1']['penaltyPoint']['eventTable'] = eventTableCheck(regional_eventTable, senceIscore['penaltyPoint']['eventTable'])
                vehi_regional_score[vehID][scene_id][0]['1']['AbilityDimension'] = abilityDimensionIProcess(totallSenceIscore, allWeight)

                # 一个场景的数据处理完后，要对ego的事件列表进行处理，重置为空
                Ego.eventTable = {}

    # 在处理完所有的vehi_regional_score中需要改变的场景后，开始整合最终的输出分数
    totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar = startScoreProcess(vehi_regional_score, regionals, projectWeight, taskID,
                                                                                     missonStartTime, missonEndTime, vehID)

    # 得到这个输出的分数后不能直接使用，需要重新做一下结构
    SQLscore = {
        vehID: {}
    }
    for scene_id, sceneScore in sceneResultDataDictSplitByCar.items():
        scene_id, index = scene_id.split("_")
        SQLscore[vehID][scene_id] = []
        for i in range(int(index)+1):
            sceneScore['taskID'] = sceneScore['caseID']
            sceneScore['startTime'] = vehi_regional_score[vehID][scene_id][0]['1']['startTime']
            sceneScore['endTime'] = vehi_regional_score[vehID][scene_id][0]['1']['endTime']
            a = {'1': sceneScore}
            SQLscore[vehID][scene_id].append(a)

    return SQLscore, totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar

# 启动函数，输入多车测试的结果，以及想要的权重信息，输出基于车辆的总分和分场景分数
# 可能还有一些数据需要输入，比如测试的开始结束时间，切片（也就是某个场地区域）id和名称等，这里其实也可以不用输出，可以再数据里面读，但是要保证输入的时候是对的
# 第一步需要把权重整理成我需要的类型weightProcess
# 第二步把某个车辆中每个单场景中的关键信息提取出来包括：'penaltyPoint'以及'chartData'还有一些其他的东西提取出来，并且自定义一个'senceID'
# 第三步利用第二步提取的东西，构造五个新的dict：totalScore = {'allSenseScore'；'allSenseScoreDital'}和chartDataTotalScene
    # 其中'allSenseScore'是一个列表，每个元素都是一个字典包括'senceID'、'weight'、'safe'效率舒适分数的针对单场景的分数
    # 其中'allSenseScoreDital'是一个列表，每个元素都是一个字典，就是'penaltyPoint'，注意这两个列表的顺序要一致
    # chartDataTotalScene字典的key是'senceID'；value是'chartData'
    # 那这里还要再加二个字典caseID和sceneName，key是'senceID'；value是'caseID'和'sceneName'
    # 还要再加一个sceneWeight字典，sceneWeight字典需要自己在重新算一下值，根据切片权重，还有切片的场景数量，所有场景的权重加起来应该为1
# 第四步把上面组合的五个字典，还有一些其他信息都输入到evaluationDataProcess函数中，生成总分数，作为其中一个输出项
# 第五步把总分和权重输入到splitData函数中，生成单场景分数，作为其中一个输出项
# 第六步重复2到5步，做下一辆车
def startScoreProcess(vehi_regional_score, regionals, projectWeight, taskID, missonStartTime, missonEndTime, vehID):
    regional_records = regionals.get('EvaluationArea', [])
    regionals = {}
    for regional_record in regional_records:
        region_position = json.loads(regional_record['region'])
        regionals[regional_record['id']] = {
            'weights': regional_record['weights'],
            'position': {
                "minX": region_position['posFrom']['x'],
                "maxX": region_position['posTo']['x'],
                "minY": region_position['posTo']['y'],
                "maxY": region_position['posFrom']['y'],
            },
        }

    # 这里需要做一个regionals的验证，如果regionals是空，或者regionals没有包含所有的场景就要给个默认值
    if not regionals:
        if vehID in vehi_regional_score:
            value_car = vehi_regional_score[vehID]
            for key_case, value_case in value_car.items():
                # 在这里先把场景的分数处理一下，这里key_case是“1”，“2”，“3”这种
                if key_case not in regionals:
                    regionals[int(key_case)] = {'weights': 1}

    # 一个字典，key是车辆名称，value是totalResultData
    totalResultDataDictSplitByCar = {}
    sceneResultDataDictSplitByCar = {}
    # todo 这里做一个默认的权重，把上面的用户可以设置的权重更新后，这里就可以删除了，也可以保留当成一个默认数值，如果下面有错误，这里也不会出问题了
    allWeight = standardWeightProcess(Parameter.weightData['indexWeightDict'], regionals)
    # 得到一个类，这个类包含五个字典，四个分数权重字典，一个场景权重字典（和为1）
    # todo 这里可以用他们给的一些权重，包括场景权重和指标权重
    # allWeight = weightProcess(projectWeight, regionals, allWeight)
    # 分车辆来输出结果，下面这个for循环就是到最后了
    # for key_car, value_car in vehi_regional_score.items():
    if vehID in vehi_regional_score:
        value_car = vehi_regional_score[vehID]
        key_car = vehID
        # 去每一个车里找这个车经历的所有片区，key是片区名称，value是列表，里面是经历了几次这个片区，每次经历都是一个字典，一个字典就是一个场景分
        # 每次开始找一个车的数据的时候，sceneID重新为0
        # 构建五个新的dict，这五个字典都是针对单车的，每次新到一个车的数据都要重置一下，然后这五个字典输入到分数聚合函数evaluationDataProcess
        totalScore = {'allSenseScore': [], 'allSenseScoreDital': []}
        chartDataTotalScene = {}
        caseIdDict = {}
        sceneNameDict = {}
        sceneWeight = {} # 场景分数之和为1，key是场景id，value是那个场景的权重
        # 还需要加两个时间来计算总时间
        TTCtime = 0
        TTCtimeDict = {}
        testDuration = 0
        testDurationDict = {}
        # 还需要一个期望时间
        missonExpectTime = {}
        for key_case, value_case in value_car.items():
            # 在这里先把场景的分数处理一下，这里key_case是“1”，“2”，“3”这种
            if key_case in allWeight.sceneWeight:
                sceneWeightI = allWeight.sceneWeight[key_case] / len(value_case)
            else:
                sceneWeightI = 0.0001
            for index, sceneScoreI_initial in enumerate(value_case):
                # 找到一个场景就是sceneID+1
                sceneID = f"{key_case}_{index}"
                # 不知道为啥每一个场景分要放在一个字典里，他们的key都是“1”
                # 使用这个函数可以得到一个字典，字典里有要的信息，然后把这个字典里的信息，不断的完善要构建的5个新dict里面
                sceneIInforDict = getInforFromSingleScene(sceneScoreI_initial["1"], sceneID, allWeight, sceneWeightI)
                totalScore['allSenseScore'].append(sceneIInforDict['allSenseScore'])
                totalScore['allSenseScoreDital'].append(sceneIInforDict['allSenseScoreDital'])
                chartDataTotalScene[sceneID] = sceneIInforDict['chartDataTotalScene']
                caseIdDict[sceneID] = sceneIInforDict['caseId']
                sceneNameDict[sceneID] = sceneIInforDict['sceneName']
                sceneWeight[sceneID] = sceneWeightI
                # 计算总时间和总危险时间
                TTCtime += sceneScoreI_initial["1"]['penaltyPoint']['TTCAccuTime']
                TTCtimeDict[sceneID] = sceneScoreI_initial["1"]['penaltyPoint']['TTCAccuTime']
                testDuration += sceneScoreI_initial["1"]["testDuration"]
                testDurationDict[sceneID] = sceneScoreI_initial["1"]["testDuration"]
                # 加一个期望时间
                missonExpectTime[sceneID] = 9999

        # 到这里就把之前做的5个字典，都塞到这个函数里就行了
        outPutData = evaluationDataProcess(TTCtime, totalScore, missonExpectTime, sceneWeight, missonStartTime, missonEndTime, testDuration, allWeight, chartDataTotalScene, sceneNameDict, caseIdDict)
        diagnoseLibrary = diagnose(outPutData)
        diagnoseLibrary.getSuggestion()
        outPutData["diagnose"] = diagnoseLibrary.diagnoseSuggestionDict
        outPutData["taskID"] = taskID
        outPutData["avName"] = key_car
        outPutData['AbilityDimension']['allScore'] = outPutData['allSenseScore']
        outPutData['abilityDimension'] = outPutData['AbilityDimension']
        outPutData['testScene'] = sorted(outPutData['testScene'], key=lambda x: x['sceneNameList'])
        totalResultDataDictSplitByCar[key_car] = outPutData
        # 到这里总分就做好了，后面是拆分

        sceneIoutPutDataDict = splitData(TTCtimeDict, totalScore, missonExpectTime, sceneWeight, missonStartTime, missonEndTime, testDurationDict, allWeight, chartDataTotalScene, sceneNameDict, caseIdDict, taskID, key_car)
        sceneResultDataDictSplitByCar[key_car] = sceneIoutPutDataDict

    # 这里如果要放在数据库里面就新增一些数据库的东西，在函数中，如果不用就不用了
    return totalResultDataDictSplitByCar.get(vehID, {}), sceneResultDataDictSplitByCar.get(vehID, {})


def standardWeightProcess(indexWeight, regionals):
    scoreWeight = {}
    safeDitalWeight = {}
    comfortableDitalWeight = {}
    efficiencyDitalWeight = {}
    sceneWeight = {}

    # 先处理最简单的场景权重数据
    # 这里的场景权重是全部均分
    totalWeight = 0
    for key, value in regionals.items():
        totalWeight += float(value['weights'])
    for key, value in regionals.items():
        sceneWeight[str(key)] = float(value['weights']) / totalWeight

    # todo 场景数据也可以按照默认值来做


    scoreWeight = {'safe': indexWeight["10000"],
                    'comfortable': indexWeight["20000"],
                    'efficiency': indexWeight["30000"],
                    'coordination': indexWeight['40000'],
                    'compliance': indexWeight['50000']}

    for i in range(1, 6):
        for j in range(0, 15):
            a = i * 10000 + j
            if str(a) not in indexWeight:
                indexWeight[str(a)] = 0

    # 这里还是按照三个维度来做，其中交通协调性归属到效率里面，交规符合性归属到安全里面
    safeDitalWeight = {"crash": indexWeight["10001"],
                        "TTC": indexWeight["10002"],
                        "inSubtendRoad": indexWeight["10003"],
                        "onLaneMarking": indexWeight["10004"],
                        "overSpeed": indexWeight["10005"],
                        "outOfLane": indexWeight["10006"],
                        "transverseDistance": indexWeight["10007"],
                        "inForbiddenArea": indexWeight["10008"],
                        "breakSignal": indexWeight["10009"],
                        "drivingInDesignatedLane": indexWeight["10010"],
                        "stopAtStopLine": indexWeight["10011"],
                        "isSecurityInvolved": indexWeight["10012"],
                        "onDottedLaneMarking": indexWeight["10013"],
                        "followStopSignal": indexWeight["10014"],}
    comfortableDitalWeight = {"overLateralAcce": indexWeight["20001"],
                                "overLateralJerk": indexWeight["20002"],
                                "overAcce": indexWeight["20003"],
                                "overJerk": indexWeight["20004"],
                                "unstableSteeringAngle": indexWeight["20005"],}
    efficiencyDitalWeight = {"usedTime": indexWeight["30001"],
                              "averageSpeed": indexWeight["30002"],
                              "gapRefuse": indexWeight["30003"],
                              "missionAccomplish": indexWeight["30004"],
                              "reverseCar": indexWeight["30005"],
                              "other_missionAccomplish": indexWeight["30006"],
                              "coordination": indexWeight["40001"]}

    allWeight = sceneAndScoreWeightProcess(scoreWeight, safeDitalWeight, comfortableDitalWeight, efficiencyDitalWeight, sceneWeight)
    return allWeight

# 处理权重数据到想要的类型
# 输入明阳的权重信息，输出我的标准格式的权重信息
# 标准格式的权重信息是一个类重点是四个字典，把这4个字典组合成一个类，然后输出出去这个类allWeight就行了
def weightProcess(projectWeight, regionals, allWeight):
    scoreWeight = allWeight.scoreWeight
    safeDitalWeight = allWeight.safeDitalWeight
    comfortableDitalWeight = allWeight.comfortableDitalWeight
    efficiencyDitalWeight = allWeight.efficiencyDitalWeight
    sceneWeight = allWeight.sceneWeight

    # 先处理最简单的场景权重数据
    totalWeight = 0
    for key, value in regionals.items():
        totalWeight += float(value['weights'])
    for key, value in regionals.items():
        sceneWeight[str(key)] = float(value['weights']) / totalWeight

    # 然后开始处理数据权重
    totalClassWeight = 0
    for weightClassI in projectWeight['EvaluationClassWeights']:
        totalClassWeight += float(weightClassI['weights'])
    for weightClassJ in projectWeight['EvaluationClassWeights']:
        if weightClassJ['name'] == '安全':
            scoreWeight['safe'] = float(weightClassJ['weights']) / totalClassWeight
        if weightClassJ['name'] == '效率':
            scoreWeight['efficiency'] = float(weightClassJ['weights']) / totalClassWeight
        if weightClassJ['name'] == '舒适':
            scoreWeight['comfortable'] = float(weightClassJ['weights']) / totalClassWeight
        if weightClassJ['name'] == '交通协调':
            scoreWeight['coordination'] = float(weightClassJ['weights']) / totalClassWeight
        if weightClassJ['name'] == '交规符合':
            scoreWeight['compliance'] = float(weightClassJ['weights']) / totalClassWeight

    # 继续计算安全数据
    totalSafeWeight = 0
    for weightSafeI in projectWeight['EvaluationSafeWeights']:
        totalSafeWeight += float(weightSafeI['weights'])
    totalSafeWeight = totalSafeWeight / 100

    # TODO projectWeight['EvaluationSafeWeights'].append({"indicatorName": '安全员介入',"weights": 10,})
    for weightSafeJ in projectWeight['EvaluationSafeWeights']:
        if weightSafeJ['indicatorName'] == '是否碰撞':
            safeDitalWeight['crash'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '碰撞时间TTC':
            safeDitalWeight['TTC'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '逆向行驶':
            safeDitalWeight['inSubtendRoad'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '压实线行驶':
            safeDitalWeight['onLaneMarking'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '压虚线行驶':
            safeDitalWeight['onDottedLaneMarking'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '未遵守停车标志':
            safeDitalWeight['followStopSignal'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '超速行驶':
            safeDitalWeight['overSpeed'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '四轮在路':
            safeDitalWeight['outOfLane'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '横向间距':
            safeDitalWeight['transverseDistance'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '在禁行区禁行':
            safeDitalWeight['inForbiddenArea'] = float(weightSafeJ['weights'])
        if weightSafeJ['indicatorName'] == '安全员介入':
            safeDitalWeight['isSecurityInvolved'] = float(weightSafeJ['weights'])
        safeDitalWeight['drivingInDesignatedLane'] = 0
        safeDitalWeight['stopAtStopLine'] = 0
        safeDitalWeight['breakSignal'] = 0

        # 继续计算效率数据
    totalEfficiencyWeight = 0
    for weightEfficiencyI in projectWeight['EvaluationEfficiencyWeights']:
        totalEfficiencyWeight += float(weightEfficiencyI['weights'])
    totalEfficiencyWeight = totalEfficiencyWeight / 100
    for weightEfficiencyJ in projectWeight['EvaluationEfficiencyWeights']:
        if weightEfficiencyJ['indicatorName'] == '任务完成时间':
            efficiencyDitalWeight['missionAccomplish'] = float(weightEfficiencyJ['weights'])
        if weightEfficiencyJ['indicatorName'] == '平均速度':
            efficiencyDitalWeight['averageSpeed'] = float(weightEfficiencyJ['weights'])
        if weightEfficiencyJ['indicatorName'] == '非场景需要倒车':
            efficiencyDitalWeight['reverseCar'] = float(weightEfficiencyJ['weights'])
        if weightEfficiencyJ['indicatorName'] == '干扰背景车辆通行':
            efficiencyDitalWeight['coordination'] = float(weightEfficiencyJ['weights'])
        efficiencyDitalWeight['usedTime'] = 0

    # 继续计算舒适数据
    totalComfortWeight = 0
    for weightComfortI in projectWeight['EvaluationComfortWeights']:
        totalComfortWeight += float(weightComfortI['weights'])
    totalComfortWeight = totalComfortWeight / 100
    for weightComfortJ in projectWeight['EvaluationComfortWeights']:
        if weightComfortJ['indicatorName'] == '横向加速度':
            comfortableDitalWeight['overLateralAcce'] = float(weightComfortJ['weights'])
        if weightComfortJ['indicatorName'] == '横向急动度':
            comfortableDitalWeight['overLateralJerk'] = float(weightComfortJ['weights'])
        if weightComfortJ['indicatorName'] == '纵向加速度':
            comfortableDitalWeight['overAcce'] = float(weightComfortJ['weights'])
        if weightComfortJ['indicatorName'] == '纵向急动度':
            comfortableDitalWeight['overJerk'] = float(weightComfortJ['weights'])
        if weightComfortJ['indicatorName'] == '角速度':
            comfortableDitalWeight['unstableSteeringAngle'] = float(weightComfortJ['weights'])

    allWeight = sceneAndScoreWeightProcess(scoreWeight, safeDitalWeight, comfortableDitalWeight, efficiencyDitalWeight, sceneWeight)
    return allWeight


# 输入是单场景分数，输出是需要的一个字典，里面有需要的信息
def getInforFromSingleScene(sceneScoreI_initial, sceneID, allWeight, sceneWeightI):
    sceneIInforDict = {}
    allSenseScore = {}
    allSenseScoreDital = {}
    chartDataTotalScene = {}
    caseId = 0
    sceneName = None

    # 处理第一个数据allSenseScore
    # todo 是不是这里不用乘allWeight.scoreWeight['safe'] ，感觉之前乘过了，一会计算的时候看一下
    # 这里要注意下，safe100这种是针对场景满分一百的情况，safe100是乘过安全权重的分数，safe是针对场景的
    allSenseScore['senceID'] = sceneID
    allSenseScore['safe100'] = sceneScoreI_initial['AbilityDimension'].get('safe', 0) * allWeight.scoreWeight['safe']
    allSenseScore['efficiency100'] = sceneScoreI_initial['AbilityDimension'].get('efficiency', 0) * allWeight.scoreWeight['efficiency']
    allSenseScore['comfortable100'] = sceneScoreI_initial['AbilityDimension'].get('comfortable', 0) * allWeight.scoreWeight['comfortable']
    allSenseScore['coordination100'] = sceneScoreI_initial['AbilityDimension'].get('coordination', 0) * allWeight.scoreWeight['coordination']
    allSenseScore['compliance100'] = sceneScoreI_initial['AbilityDimension'].get('compliance', 0) * allWeight.scoreWeight['compliance']
    allSenseScore['safe'] = sceneWeightI * allSenseScore['safe100']
    allSenseScore['efficiency'] = sceneWeightI * allSenseScore['efficiency100']
    allSenseScore['comfortable'] = sceneWeightI * allSenseScore['comfortable100']
    allSenseScore['coordination'] = sceneWeightI * allSenseScore['coordination100']
    allSenseScore['compliance'] = sceneWeightI * allSenseScore['compliance100']
    allSenseScore['weight'] = sceneWeightI

    scoreWeight = allWeight.scoreWeight
    # 处理第二个数据allSenseScoreDital
    allSenseScoreDital = {
        'senceID': sceneID,
        'missionAccomplish': 0,
        'sceneEndTime': 0,
        'sceneUsedTime': 0,
        'sceneStartTime': 0,
        'useTime': sceneScoreI_initial['penaltyPoint']["useTime"],
        'avgSpeedNotScore': sceneScoreI_initial['penaltyPoint']["avgSpeedNotScore"],
        'calMissionAccomplish': sceneScoreI_initial['penaltyPoint']['calMissionAccomplish'],
        'crash': sceneScoreI_initial['penaltyPoint'].get('crashNum', 0) * allWeight.safeDitalWeight["crash"],
        'isSecurityInvolved': sceneScoreI_initial['penaltyPoint'].get('isSecurityInvolved', 0) * allWeight.safeDitalWeight["isSecurityInvolved"],
        'outOfLane': sceneScoreI_initial['penaltyPoint']['outOfLane'] * allWeight.safeDitalWeight["outOfLane"],
        'TTC': sceneScoreI_initial['penaltyPoint']['TTC'] * allWeight.safeDitalWeight["TTC"],
        'PET': sceneScoreI_initial['penaltyPoint']['PET'] * 0,  # 没有pet目前
        'inSubtendRoad': sceneScoreI_initial['penaltyPoint']['inSubtendRoad'] * allWeight.safeDitalWeight["inSubtendRoad"],
        'onLaneMarking': sceneScoreI_initial['penaltyPoint']['onLaneMarking'] * allWeight.safeDitalWeight["onLaneMarking"],
        'onDottedLaneMarking': sceneScoreI_initial['penaltyPoint'].get('onDottedLaneMarking', 0) * allWeight.safeDitalWeight["onDottedLaneMarking"],
        'overSpeed': sceneScoreI_initial['penaltyPoint']['overSpeed'] * allWeight.safeDitalWeight["overSpeed"],
        'breakSignal': sceneScoreI_initial['penaltyPoint']['breakSignal'] * allWeight.safeDitalWeight["breakSignal"],
        'breakSignalNum': sceneScoreI_initial['penaltyPoint']['breakSignalNum'],
        'followStopSignal': sceneScoreI_initial['penaltyPoint'].get('followStopSignal', 0) * allWeight.safeDitalWeight["followStopSignal"],
        'followStopSignalNum': sceneScoreI_initial['penaltyPoint'].get('followStopSignalNum', 0),
        "transverseDistance": sceneScoreI_initial['penaltyPoint']['transverseDistance'] * allWeight.safeDitalWeight["transverseDistance"],
        "inForbiddenArea": sceneScoreI_initial['penaltyPoint']['inForbiddenArea'] * allWeight.safeDitalWeight["inForbiddenArea"],
        "drivingInDesignatedLane": sceneScoreI_initial['penaltyPoint']['drivingInDesignatedLane'] * allWeight.safeDitalWeight["drivingInDesignatedLane"],
        "stopAtStopLine": sceneScoreI_initial['penaltyPoint']['stopAtStopLine'] * allWeight.safeDitalWeight["stopAtStopLine"],
        'overLateralAcce': sceneScoreI_initial['penaltyPoint']['overLateralAcce'] * allWeight.comfortableDitalWeight["overLateralAcce"],
        'overLateralAcceNum': sceneScoreI_initial['penaltyPoint']['overLateralAcceNum'],
        'overLateralJerk': sceneScoreI_initial['penaltyPoint']['overLateralJerk'] * allWeight.comfortableDitalWeight["overLateralJerk"],
        'overLateralJerkNum': sceneScoreI_initial['penaltyPoint']['overLateralJerkNum'],
        'overAcce': sceneScoreI_initial['penaltyPoint']['overAcce'] * allWeight.comfortableDitalWeight["overAcce"],
        'overAcceNum': sceneScoreI_initial['penaltyPoint']['overAcceNum'],
        'overJerk': sceneScoreI_initial['penaltyPoint']['overJerk'] * allWeight.comfortableDitalWeight["overJerk"],
        'overJerkNum': sceneScoreI_initial['penaltyPoint']['overJerkNum'],
        'unstableSteeringAngle': sceneScoreI_initial['penaltyPoint']['unstableSteeringAngle'] * allWeight.comfortableDitalWeight["unstableSteeringAngle"],
        'unstableSteeringAngleTime': sceneScoreI_initial['penaltyPoint']['unstableSteeringAngleTime'],

        "avgSpeed": sceneScoreI_initial['penaltyPoint']["avgSpeed"] * allWeight.efficiencyDitalWeight["averageSpeed"],
        "gapRefuse": sceneScoreI_initial['penaltyPoint']["gapRefuse"] * allWeight.efficiencyDitalWeight["gapRefuse"],
        "timeUsed": sceneScoreI_initial['penaltyPoint']["timeUsed"] * allWeight.efficiencyDitalWeight["usedTime"],
        'reverseCar': sceneScoreI_initial['penaltyPoint'].get("reverseCar", 0) * allWeight.efficiencyDitalWeight["reverseCar"],
        'coordination': min(allWeight.scoreWeight['coordination']*100, sceneScoreI_initial['penaltyPoint'].get("coordination", 0) * allWeight.efficiencyDitalWeight["coordination"]),
        'maxDisturbedSvSimuTime': sceneScoreI_initial['penaltyPoint'].get("maxDisturbedSvSimuTime", 0),
        'svSimuTimeWithoutDisturbance': Parameter.svSimuTimeWithoutDisturbance,
        'minCoordinationScore': sceneScoreI_initial['penaltyPoint'].get("minCoordinationScore", 0),
        'outOfParkingZone': sceneScoreI_initial['penaltyPoint'].get("outOfParkingZone", 0) * 100,

        "eventTable": sceneScoreI_initial['penaltyPoint']['eventTable'],
        "penaltyPoint": sceneScoreI_initial['penaltyPoint'],}

    # 处理第三个图表数据
    chartDataTotalScene = sceneScoreI_initial['testScene'][0]['chartData']

    # 处理第四个caseId
    caseId = sceneScoreI_initial['caseID']
    # 处理第五个sceneName
    sceneName = sceneScoreI_initial['sceneName']

    sceneIInforDict = {
        'allSenseScore': allSenseScore,
        'allSenseScoreDital': allSenseScoreDital,
        'chartDataTotalScene': chartDataTotalScene,
        'caseId': caseId,
        'sceneName': sceneName,
    }

    return sceneIInforDict

def clearZero(dict):
    for key in dict:
        if not dict[key]:
            dict[key] = 0.01
        else:
            pass
    return dict


def gradeScore(a, b, c, d, score):
    grade = None
    if score > a:
        grade = "优秀"
    elif score > b:
        grade = "良好"
    elif score > c:
        grade = "一般"
    elif score > d:
        grade = "较差"
    else:
        grade = "不合格"
    return grade

def checkCrashTime(eventTable, aimIndex):
    crashNum = 0
    for key in eventTable:
        if eventTable[key]["index"] == aimIndex:
            crashNum += 1
    return crashNum

# 改进一下之前的那个函数
def evaluationDataProcess(TTCtime, totalScore, missonExpectTime, senceWeight, missonStartTime, missonEndTime, testDuration,
                          allWeight, chartDataTotalScene, sceneNameDict, caseIdDict):
    allSence = []
    num = 2
    AbilityDimension = {"safe": 0,
                        "efficiency": 0,
                        "comfortable": 0,
                        "coordination": 0,
                        "compliance": 0,
                        "else": 0, }  # 百分制
    clearZero(allWeight.scoreWeight)
    clearZero(allWeight.comfortableDitalWeight)
    clearZero(allWeight.safeDitalWeight)
    clearZero(allWeight.efficiencyDitalWeight)
    gradeTableDict = {}
    scoreTableDict = {}

    for i in range(len(totalScore['allSenseScore'])):
        sceneItotalScore = totalScore['allSenseScore'][i]  # 百分制了
        sceneIDitalScore = totalScore['allSenseScoreDital'][i]  # 这里的scoreDital就是senceScoreEnglish，autodriving里面最后一段代码一大堆那个
        if sceneIDitalScore['useTime']:
            pass
        else:
            sceneIDitalScore['useTime'] = 0.001

        efficiency = [{"index": "任务完成", "score": str(sceneIDitalScore['calMissionAccomplish'] *
                                                         allWeight.efficiencyDitalWeight['missionAccomplish']) + "/" +
                                                     str(allWeight.efficiencyDitalWeight['missionAccomplish']),
                       "time": round(sceneIDitalScore['useTime'], num),
                       "name": "任务完成",
                       "unit": "s", },
                      {"index": "任务耗时","name": "任务耗时",
                       "score": str(
                           round(min(allWeight.efficiencyDitalWeight["usedTime"],
                                     (missonExpectTime[sceneItotalScore['senceID']] / sceneIDitalScore['useTime']) *
                                     allWeight.efficiencyDitalWeight["usedTime"]),
                                 num)) + "/" + str(
                           round(allWeight.efficiencyDitalWeight["usedTime"], num)),
                       "time": round(sceneIDitalScore['useTime'], num),
                       "unit": "s",
                       "expectTime": missonExpectTime[sceneItotalScore['senceID']],
                       "overTime": round((sceneIDitalScore['useTime'] - missonExpectTime[sceneItotalScore['senceID']]) /
                                         missonExpectTime[sceneItotalScore['senceID']], num)},
                      {"index": "任务耗时", "name": "任务耗时", "score": str(round(
                          (min(1, sceneIDitalScore['avgSpeedNotScore'] / Parameter.avgSpeedKM)) *
                          allWeight.efficiencyDitalWeight["averageSpeed"],
                          num)) + "/" + str(
                          round(allWeight.efficiencyDitalWeight["averageSpeed"], num)),
                       "avgSpeed": sceneIDitalScore['avgSpeedNotScore'],
                       "time": sceneIDitalScore['avgSpeedNotScore'],
                       "unit": "km/h",
                       "expectSpeed": Parameter.avgSpeedKM,
                       "belowSpeed": round((Parameter.avgSpeedKM - sceneIDitalScore['avgSpeedNotScore']) /
                                           Parameter.avgSpeedKM, num)},
                      {"index": "倒车", "name": "非场景需要倒车", "score": str(round(
                          (max(0, Parameter.miniScoreMaxLimit['倒车'] - sceneIDitalScore['reverseCar'])),
                          num)) + "/" + str(
                          round(Parameter.miniScoreMaxLimit['倒车'], num)),
                       "time": round(sceneIDitalScore['reverseCar'] * sceneIDitalScore['useTime'] / (
                           (allWeight.efficiencyDitalWeight["reverseCar"])), num),
                       "unit": "s", },
                      {"index": "倒车", "name": "非场景需要倒车", "score": str(round((max(0, Parameter.miniScoreMaxLimit['倒车'] -
                                                                (checkCrashTime(sceneIDitalScore['eventTable'],
                                                                                '倒车') * allWeight.efficiencyDitalWeight[
                                                                     "reverseCar"]))), num)) + "/" + str(
                          round(Parameter.miniScoreMaxLimit['倒车'], num)),
                       "time": checkCrashTime(sceneIDitalScore['eventTable'],'倒车'),
                       "unit": "次", },]

        comfortable = [
            {"index": "纵向舒适度", "score": str(round(sceneIDitalScore['overJerk'], num)), "type": "j",
             "unit": "s","name": "纵向加加速度",
             "time": round(sceneIDitalScore['overJerk'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overJerk"])), num)},
            {"index": "纵向舒适度", "score": str(round(sceneIDitalScore['overAcce'], num)), "type": "a",
             "unit": "s","name": "纵向加速度",
             "time": round(sceneIDitalScore['overAcce'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overAcce"])), num)},
            {"index": "横向舒适度", "score": str(round(sceneIDitalScore['overLateralJerk'], num)), "type": "j",
             "unit": "s","name": "横向加加速度",
             "time": round(sceneIDitalScore['overLateralJerk'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overLateralJerk"])), num)},
            {"index": "横向舒适度", "score": str(round(sceneIDitalScore['overLateralAcce'], num)), "type": "a",
             "unit": "s","name": "横向加速度",
             "time": round(sceneIDitalScore['overLateralAcce'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["overLateralAcce"])), num)},
            {"index": "转弯舒适度", "score": str(round(sceneIDitalScore['unstableSteeringAngle'], num)),
             "type": "c", "unit": "s","name": "横摆角速度",
             "time": round(sceneIDitalScore['unstableSteeringAngle'] * sceneIDitalScore['useTime'] / (
                 (allWeight.comfortableDitalWeight["unstableSteeringAngle"])), num)}]
        # if sceneIDitalScore['crash'] > 0:
        #     crash = 100 * allWeight.scoreWeight['safe']
        # else:
        #     crash = 0
        # if sceneIDitalScore['isSecurityInvolved'] > 0:
        #     # isSecurityInvolved = 100 * allWeight.scoreWeight['safe']
        #     isSecurityInvolved = 100
        # else:
        #     isSecurityInvolved = 0
        safe = [{"index": '碰撞', "score": str(min(100, round(sceneIDitalScore['crash'], num))),
                 "unit": "次","name": "碰撞",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], '碰撞')},
                {"index": '安全员介入', "score": str(min(100, round(sceneIDitalScore['isSecurityInvolved'], num))),
                 "unit": "次","name": "安全员介入",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], '安全员介入')},
                {"index": '驶出行车道', "score": str(min(100, round(sceneIDitalScore['outOfLane'], num))),
                 "unit": "s","name": "驶出道路边界",
                 "time": round(sceneIDitalScore['outOfLane'] * sceneIDitalScore['useTime'] / (
                         allWeight.safeDitalWeight["outOfLane"]), num)},
                {"index": '驶出行车道', "score": str(min(100, round(checkCrashTime(sceneIDitalScore['eventTable'], '驶出行车道') * allWeight.safeDitalWeight["outOfLane"], num))),
                 "unit": "次","name": "驶出道路边界",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], '驶出行车道')},
                {"index": 'TTC', "score": str(round(sceneIDitalScore['TTC'], num)),
                 "unit": "s","name": "TTC",
                 "time": round(sceneIDitalScore['TTC'] * sceneIDitalScore['useTime'] / (
                     allWeight.safeDitalWeight["TTC"]), num),
                 "timeRatio：": round(sceneIDitalScore['TTC'] / (allWeight.safeDitalWeight["TTC"]), num),
                 "timeRatio": round(sceneIDitalScore['TTC'] / (allWeight.safeDitalWeight["TTC"]), num), },
                {"index": 'TTC', "score": str(min(100, round(checkCrashTime(sceneIDitalScore['eventTable'], 'TTC') * allWeight.safeDitalWeight["TTC"], num))),
                 "unit": "次","name": "TTC",
                 "time": checkCrashTime(sceneIDitalScore['eventTable'], 'TTC')},
                {"index": "横向间距", "score": str(round(sceneIDitalScore['transverseDistance'], num)),
                 "unit": "s","name": "横向间距",
                 "time": round(sceneIDitalScore['transverseDistance'] * sceneIDitalScore['useTime'] / (
                     allWeight.safeDitalWeight["transverseDistance"]), num)}, ]

        compliance = [{"index": '压实线', "score": str(round(sceneIDitalScore['onLaneMarking'], num)),
                       "unit": "s","name": "压实线",
                       "time": round(sceneIDitalScore['onLaneMarking'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["onLaneMarking"]), num)},
                      {"index": '压虚线', "score": str(round(sceneIDitalScore['onDottedLaneMarking'], num)),
                       "unit": "s","name": "压实线",
                       "time": round(sceneIDitalScore['onDottedLaneMarking'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["onDottedLaneMarking"]), num)},
                      {"index": '超速', "score": str(round(sceneIDitalScore['overSpeed'], num)),
                       "unit": "s","name": "超出限速行驶",
                       "time": round(sceneIDitalScore['overSpeed'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["overSpeed"]), num)},
                      {"index": '超速', "score": str(min(allWeight.scoreWeight['compliance'] * 100, round(checkCrashTime(sceneIDitalScore['eventTable'], '超速') * allWeight.safeDitalWeight["overSpeed"], num))),
                       "unit": "次","name": "超出限速行驶",
                       "time": checkCrashTime(sceneIDitalScore['eventTable'], '超速')},
                      {"index": '闯红灯', "name": "闯红灯", "score": str(round(0, num)), "unit": "次", "time": 0},
                      {"index": '未遵守停车标志', "score": str(min(allWeight.scoreWeight['compliance'] * 100, round(sceneIDitalScore['followStopSignal'], num))),
                       "unit": "次", "name": "未停车让行",
                       "time": round(sceneIDitalScore['followStopSignalNum'], num)},
                      {"index": '驶入对向车道', "score": str(round(sceneIDitalScore['inSubtendRoad'], num)),
                       "unit": "s","name": "驶入对向车道",
                       "time": round(sceneIDitalScore['inSubtendRoad'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["inSubtendRoad"]), num)},
                      {"index": '驶入对向车道', "score": str(min(allWeight.scoreWeight['compliance'] * 100, round(checkCrashTime(sceneIDitalScore['eventTable'], '驶入对向车道') * allWeight.safeDitalWeight["inSubtendRoad"], num))),
                       "unit": "次","name": "驶入对向车道",
                       "time": checkCrashTime(sceneIDitalScore['eventTable'], '驶入对向车道')},
                      {"index": "禁行区行驶", "score": str(round(sceneIDitalScore['inForbiddenArea'], num)),
                       "unit": "s","name": "禁行区行驶",
                       "time": round(sceneIDitalScore['inForbiddenArea'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["inForbiddenArea"]), num)},
                      {"index": "停车压停止线", "score": str(round(sceneIDitalScore['stopAtStopLine'], num)),
                       "unit": "s","name": "停车压停止线",
                       "time": round(sceneIDitalScore['stopAtStopLine'] * sceneIDitalScore['useTime'] / (
                           allWeight.safeDitalWeight["stopAtStopLine"]), num)},
                      {"index": "未按规定车道行驶",
                       "unit": "s","name": "未按规定车道行驶",
                       "score": str(round(sceneIDitalScore['drivingInDesignatedLane'], num)),
                       "time": round(sceneIDitalScore['drivingInDesignatedLane'] * sceneIDitalScore['useTime'] /
                                     (allWeight.safeDitalWeight["drivingInDesignatedLane"]), num)},
                      {"index": "未有效泊车",
                       "unit": "次", "name": "未正常泊车",
                       "score": str(round(sceneIDitalScore.get('outOfParkingZone', 0), num)),
                       "time": round(sceneIDitalScore.get('outOfParkingZone', 0) / 100, num)},]

        coordination = [{"index": '干扰背景车辆通行', "name": "干扰背景车辆通行", "score": str(round(sceneIDitalScore['coordination'], num)),
                         "unit": "s", "time": round(sceneIDitalScore['minCoordinationScore'], num),
                         "svSimuTimeWithoutDisturbance": round(sceneIDitalScore['svSimuTimeWithoutDisturbance'], num),
                         "maxDisturbedSvSimuTime": round(sceneIDitalScore['maxDisturbedSvSimuTime'], num)}, ]

        AbilityDimensionI = {}
        AbilityDimensionI["safe"] = round(sceneItotalScore['safe100'] / allWeight.scoreWeight['safe'], num)
        AbilityDimensionI["comfortable"] = round(sceneItotalScore['comfortable100'] / allWeight.scoreWeight['comfortable'], num)
        AbilityDimensionI["efficiency"] = round(sceneItotalScore['efficiency100'] / allWeight.scoreWeight['efficiency'], num)
        AbilityDimensionI["coordination"] = round(sceneItotalScore['coordination100'] / allWeight.scoreWeight['coordination'], num)
        AbilityDimensionI["compliance"] = round(sceneItotalScore['compliance100'] / allWeight.scoreWeight['compliance'], num)
        AbilityDimensionI["safe*weight"] = round(sceneItotalScore['safe100'], num)
        AbilityDimensionI["comfortable*weight"] = round(sceneItotalScore['comfortable100'], num)
        AbilityDimensionI["efficiency*weight"] = round(sceneItotalScore['efficiency100'], num)
        AbilityDimensionI["coordination*weight"] = round(sceneItotalScore['coordination100'], num)
        AbilityDimensionI["compliance*weight"] = round(sceneItotalScore['compliance100'], num)
        AbilityDimensionI["safeWeight"] = round(allWeight.scoreWeight['safe'], num)
        AbilityDimensionI["comfortableWeight"] = round(allWeight.scoreWeight['comfortable'], num)
        AbilityDimensionI["efficiencyWeight"] = round(allWeight.scoreWeight['efficiency'], num)
        AbilityDimensionI["coordinationWeight"] = round(allWeight.scoreWeight['coordination'], num)
        AbilityDimensionI["complianceWeight"] = round(allWeight.scoreWeight['compliance'], num)
        testSceneI = {"senceID": sceneItotalScore['senceID'],
                      "sceneCaseID": caseIdDict[sceneItotalScore['senceID']],
                      "sceneNameList": sceneNameDict[sceneItotalScore['senceID']],
                      "chartData": chartDataTotalScene[sceneItotalScore['senceID']],
                      "AbilityDimension": AbilityDimensionI,
                      "missionAccomplish": sceneIDitalScore['missionAccomplish'],
                      "senseScore": round((sceneItotalScore['safe100'] + sceneItotalScore['efficiency100'] + sceneItotalScore['comfortable100']
                                           + sceneItotalScore['coordination100'] + sceneItotalScore['compliance100']), num),
                      "senseScore*senseWeight": round((sceneItotalScore['safe'] + sceneItotalScore['efficiency'] + sceneItotalScore['comfortable']
                                                       + sceneItotalScore['coordination'] + sceneItotalScore['compliance']),num),
                      "senseAggScore": round(senceWeight[sceneItotalScore['senceID']] * 100, num),
                      "senseWeight": round(senceWeight[sceneItotalScore['senceID']] * 100, num),
                      "info": {"efficiency": efficiency, "comfortable": comfortable, "safe": safe, "compliance": compliance, "coordination": coordination},
                      "eventTable": sceneIDitalScore["eventTable"], }
        allSence.append(testSceneI)

        # 计算能力雷达图
        AbilityDimension["safe"] += sceneItotalScore['safe']
        AbilityDimension["efficiency"] += sceneItotalScore['efficiency']
        AbilityDimension["comfortable"] += sceneItotalScore['comfortable']
        AbilityDimension["coordination"] += sceneItotalScore['coordination']
        AbilityDimension["compliance"] += sceneItotalScore['compliance']

        # 计算分级表格
        safeGrade = gradeScore(98, 95, 90, 80, AbilityDimensionI["safe"])
        comfortableGrade = gradeScore(95, 88, 75, 60, AbilityDimensionI["comfortable"])
        efficiencyGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["efficiency"])
        coordinationGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["coordination"])
        complianceGrade = gradeScore(90, 80, 70, 60, AbilityDimensionI["compliance"])
        # gradeTableDict[sceneNameDict[sceneItotalScore['senceID']]+str(sceneItotalScore['senceID'])] = {
        gradeTableDict[sceneNameDict[sceneItotalScore['senceID']] + '/' + str(sceneItotalScore['senceID'])] = {
            "安全性": safeGrade,
            "舒适性": comfortableGrade,
            "效率性": efficiencyGrade,
            "交通协调性": coordinationGrade,
            "交规符合性": complianceGrade, }
        # scoreTableDict[sceneNameDict[sceneItotalScore['senceID']]+str(sceneItotalScore['senceID'])] = {
        scoreTableDict[sceneNameDict[sceneItotalScore['senceID']] + '/' + str(sceneItotalScore['senceID'])] = {
            "安全性": {'score': AbilityDimensionI["safe"], 'grade': safeGrade},
            "舒适性": {'score': AbilityDimensionI["comfortable"], 'grade': comfortableGrade},
            "效率性": {'score': AbilityDimensionI["efficiency"], 'grade': efficiencyGrade},
            "交通协调性": {'score': AbilityDimensionI["coordination"], 'grade': coordinationGrade},
            "交规符合性": {'score': AbilityDimensionI["compliance"], 'grade': complianceGrade}, }

    allSenseScore = 0
    for i in allSence:
        allSenseScore += i['senseScore*senseWeight']

    for key in AbilityDimension:
        if key in allWeight.scoreWeight:
            AbilityDimension[key] = round(AbilityDimension[key] / allWeight.scoreWeight[key], num)

    if testDuration != 0:
        dangerTime = TTCtime / testDuration
    else:
        dangerTime = 0
    data = {'startTime': missonStartTime,
            'endTime': missonEndTime,
            'testDuration': round(testDuration, num),
            'dangerTimeProportionPercentage': str(round(100 * dangerTime, num)) + "%",
            'dangerTimeProportion': round(dangerTime, num),
            'allSenseScore': round(allSenseScore, num),  # 这个是总分
            'testScene': allSence,
            "AbilityDimension": AbilityDimension,  # 这个也是总分百分制的能力雷达图
            "gradeTableDict": gradeTableDict,
            "scoreTableDict": scoreTableDict,}  # 这个是分级的表格

    return data



def splitData(TTCtimeDict, totalScore, missonExpectTime, sceneWeight, missonStartTime, missonEndTime, testDurationDict, allWeight, chartDataTotalScene, sceneNameDict, caseIdDict, taskID, avName):
    # 这里面只有一个循环就是totalScore
    nameScoreDict = {}
    for i in range(len(totalScore['allSenseScore'])):
        sceneItotalScore = [totalScore['allSenseScore'][i]]  # 百分制了
        sceneIDitalScore = [totalScore['allSenseScoreDital'][i]]  # 这里的scoreDital就是senceScoreEnglish，autodriving里面最后一段代码一大堆那个，包含了那个扣分的表格“penaltyPoint”
        iScore = {'allSenseScore': sceneItotalScore, 'allSenseScoreDital':sceneIDitalScore}
        sceneID = sceneIDitalScore[0]['senceID']
        sceneIoutputData = evaluationDataProcess(TTCtimeDict[sceneID], iScore, missonExpectTime, sceneWeight, missonStartTime, missonEndTime, testDurationDict[sceneID], allWeight, chartDataTotalScene, sceneNameDict, caseIdDict)
        # print(sceneIoutputData)
        finalSceneIoutputData = splitDataProcess(sceneIoutputData, taskID, caseIdDict[sceneID], avName)
        sceneName = sceneNameDict[sceneID]
        englishSceneName = "other"
        for key, value in sceneNameDict.items():
            if sceneName in value:
                englishSceneName = key
        finalSceneIoutputData["sceneName"] = sceneName
        finalSceneIoutputData["code"] = englishSceneName
        finalSceneIoutputData["penaltyPoint"] = sceneIDitalScore[0]["penaltyPoint"]
        # 如果场景名词是4个，那跑多便这里会被覆盖，所以这里应该用场景id，sceneIDitalScore[0]['senceID']
        nameScoreDict[sceneID] = finalSceneIoutputData

    return nameScoreDict

def splitDataProcess(initialData,taskID,sceneCaseID,avName):
    testScene = initialData['testScene'][0]
    sceneName = initialData['testScene'][0]["sceneNameList"]
    dangerTimeProportion = 0
    for i in testScene["info"]['safe']:
        if i['index'] == 'TTC':
            dangerTimeProportion = i.get('timeRatio') or i.get('timeRatio：') or 0
    finalData = {
        'testDuration': testScene["info"]['efficiency'][0]["time"],
        'dangerTimeProportion': dangerTimeProportion,
        'allSenseScore': testScene["senseScore"],
        'testScene': initialData['testScene'], # 这里要注意一下，没有列表了，直接就是一个字典，因为一定是对应一个场景所以没有列表了
        'AbilityDimension': testScene["AbilityDimension"],
        # todo 这里后面注意改一下可能需要，这里是拼接的展示效果
        'gradeTableDict': initialData["gradeTableDict"][sceneName + '/' + str(testScene['senceID'])]
    }
    # 这里要用之前拿到的数据outputdata来判断一下写出哪些诊断语句库
    # 这里专门在写一个类写诊断语句
    diagnoseLibrary = diagnose(finalData)
    diagnoseLibrary.getSuggestion()
    finalData["diagnose"] = diagnoseLibrary.diagnoseSuggestionDict
    diagnoseLibrary = None
    finalData["taskID"] = taskID
    finalData["caseID"] = sceneCaseID
    finalData["avName"] = avName
    return finalData

class sceneAndScoreWeightProcess:

    # 记住weightData是处理过后，聚合好的一个字典，是我自己处理的
    def __init__(self, scoreWeight, safeDitalWeight, comfortableDitalWeight, efficiencyDitalWeight, sceneWeight):
        self.scoreWeight = scoreWeight
        self.safeDitalWeight = safeDitalWeight
        self.comfortableDitalWeight = comfortableDitalWeight
        self.efficiencyDitalWeight = efficiencyDitalWeight
        self.sceneWeight = sceneWeight


class diagnose:

    def __init__(self, data):
        self.userScore = data["AbilityDimension"]
        self.diagnoseSuggestionDict = {}
        self.safeDiagnosticBase = {}
        self.efficiencyDiagnosticBase = {}
        self.comfortableDiagnosticBase = {}
        self.coordinationDiagnosticBase = {}
        self.complianceDiagnosticBase = {}
        self.safeSuggestionDiagnosticBase = {}
        self.efficiencySuggestionDiagnosticBase = {}
        self.comfortableSuggestionDiagnosticBase = {}
        self.coordinationSuggestionDiagnosticBase = {}
        self.complianceSuggestionDiagnosticBase = {}
        self.suggestionDiagnosticBase = {}

    def getSuggestion(self):
        self.diagnosticStatementBase()
        safe = self.userScore["safe"]
        efficiency = self.userScore["efficiency"]
        comfortable = self.userScore["comfortable"]
        coordination = self.userScore["coordination"]
        compliance = self.userScore["compliance"]

        Suggestion = ""
        if safe < 60:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[0]
            Suggestion += self.safeSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif safe < 80:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[60]
            Suggestion += self.safeSuggestionDiagnosticBase[60]
            Suggestion += " "
        elif safe < 100:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[80]
            Suggestion += self.safeSuggestionDiagnosticBase[80]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["安全性方面"] = self.safeDiagnosticBase[100]
            Suggestion += self.safeSuggestionDiagnosticBase[100]
            Suggestion += " "

        if comfortable < 60:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[0]
            Suggestion += self.comfortableSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif comfortable < 100:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[60]
            Suggestion += self.comfortableSuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["舒适性方面"] = self.comfortableDiagnosticBase[100]
            Suggestion += self.comfortableSuggestionDiagnosticBase[100]
            Suggestion += " "

        if efficiency < 60:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[0]
            Suggestion += self.efficiencySuggestionDiagnosticBase[0]
            Suggestion += " "
        elif efficiency < 100:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[60]
            Suggestion += self.efficiencySuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["交互决策方面"] = self.efficiencyDiagnosticBase[100]
            Suggestion += self.efficiencySuggestionDiagnosticBase[100]
            Suggestion += " "

        if coordination < 60:
            self.diagnoseSuggestionDict["交通协调性方面"] = self.coordinationDiagnosticBase[0]
            Suggestion += self.coordinationSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif coordination < 100:
            self.diagnoseSuggestionDict["交通协调性方面"] = self.coordinationDiagnosticBase[60]
            Suggestion += self.coordinationSuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["交通协调性方面"] = self.coordinationDiagnosticBase[100]
            Suggestion += self.coordinationSuggestionDiagnosticBase[100]
            Suggestion += " "

        if compliance < 60:
            self.diagnoseSuggestionDict["交规符合性方面"] = self.complianceDiagnosticBase[0]
            Suggestion += self.complianceSuggestionDiagnosticBase[0]
            Suggestion += " "
        elif compliance < 100:
            self.diagnoseSuggestionDict["交规符合性方面"] = self.complianceDiagnosticBase[60]
            Suggestion += self.complianceSuggestionDiagnosticBase[60]
            Suggestion += " "
        else:
            self.diagnoseSuggestionDict["交规符合性方面"] = self.complianceDiagnosticBase[100]
            Suggestion += self.complianceSuggestionDiagnosticBase[100]
            Suggestion += " "

        self.diagnoseSuggestionDict["建议"] = Suggestion

    # 后续更新诊断语句，就在这里更新
    def diagnosticStatementBase(self):
        self.safeDiagnosticBase = {
            0: "安全性差，存在较严重违规行为，不能保证行车安全。",
            60: "安全性中等，存在严重违规行为，不能保证行车安全。",
            80: "安全性良好，存在较轻违规行为，不能保证行车安全。",
            100: "安全性好，不存在违规行为。"
        }
        self.efficiencyDiagnosticBase = {
            0: "任务耗时较长、效率较低，或任务未完成。",
            60: "任务耗时长，效率较低。",
            100: "任务完成，且效率较高。"
        }
        self.comfortableDiagnosticBase = {
            0: "舒适性差，存在较多急加速/急减速/急转向等行为。",
            60: "舒适性中等，存在急加速/急减速/急转向等行为。",
            100: "舒适性好，不存在急加速/急减速/急转向等行为。"
        }
        self.coordinationDiagnosticBase = {
            0: "交通协调性差，存在较多影响其他车辆通行的行为。",
            60: "交通协调性中等，存在影响其他车辆通行的行为。",
            100: "交通协调性好，不存在影响其他车辆通行的行为。"
        }
        self.complianceDiagnosticBase = {
            0: "交规符合性差，存在较多违规行为。",
            60: "交规符合性中等，存在较多违规行为。",
            100: "交规符合性好，不存在较多违规行为。"
        }
        self.safeSuggestionDiagnosticBase = {
            0: "从安全性方面，有必要提升控制规划算法，提升交通安全。",
            60: "从安全性方面，有必要提升控制规划算法，提升交通安全。",
            80: "从安全性方面，建议提升控制规划算法，提升交通安全。",
            100: "从安全性方面，安全性很好，无提升建议。"
        }
        self.efficiencySuggestionDiagnosticBase = {
            0: "从效率性方面，有必要提升控制算法，提升行驶效率。",
            60: "从效率性方面，建议提升控制算法，提升行驶效率。",
            100: "从效率性方面，算法很高效，无提升建议。"
        }
        self.comfortableSuggestionDiagnosticBase = {
            0: "从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。",
            60: "从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。",
            100: "从舒适性方面，舒适性好，无提升建议。"
        }
        self.coordinationSuggestionDiagnosticBase = {
            0: "从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。",
            60: "从交通协调性方面，建议提升控制算法和驾驶策略，提升协作效率。",
            100: "从交通协调性方面，交通协调性好，无提升建议。"
        }
        self.complianceSuggestionDiagnosticBase = {
            0: "从交规符合性方面，有必要提升控制规划算法，减少违规次数。",
            60: "从交规符合性方面，建议提升控制规划算法，减少违规次数。",
            100: "从交规符合性方面，交规符合性好，无提升建议。"
        }

        self.suggestionDiagnosticBase = {
            "safe": "从安全性角度出发，建议提升路径规划算法，提升决策规划算法，提升速度规划算法及车辆控制算法。",
            "efficiency": "从效率角度出发，建议提升驾驶策略，增加域控制器算法的激进程度，主动与其他车辆进行交互。",
            "comfortable": "从舒适性角度出发，建议提升速度规划算法及车辆控制算法，尽可能平滑的转向，减少反复修正转向角的次数。",
            "wellDone": "在当前测试场景下，从安全性、舒适性、效率评价，成绩优异，建议更换场景测试，或选择更多维度再次评价。"
        }


if __name__ == '__main__':
    # 打开并读取JSON文件
    # with open('demo.json', 'r') as file:
    #     vehi_regional_score = json.load(file)

    # vehi_regional_score = """{"8": {"1510": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724987960.937, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "3紧密跟驰", "startTime": 1724987913.801, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 47.14, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 47.14, "unit": "s", "index": "任务耗时", "score": "0.0/0.01", "overTime": 2.93, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "7.94/15", "avgSpeed": 10.58, "belowSpeed": 0.47, "expectSpeed": 20}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [10, 10, 8.7, 8.84, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0.11, 0.07, 0.11, 0.07, 0.07, 0.08, -0.04, -0.03, -0.05, -0.05, -0.04, -0.11, -0.06, -0.52, -0.07, -0.07, -0.08, -0.05, -0.03, -0.08, 0.08, 0.04, 0.08]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0, 0, -1, -0.83, -0.83, 0, 1, 0, 0, 0, 0, 0, -0.83, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0.01, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0, 0, -1, -0.69, -0.69, 0, 1, 0, -1, 1, 0, -0.91, -0.69, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [-0, -0, 0, 0, 0, 0, -0, -0, -0, -0, -0, 0, -0, 0.02, -0, 0, 0, 0, -0, 0, 0, -0, -0.08]}}, "eventTable": {}, "senseScore": 92.94, "senseWeight": 100, "sceneNameList": "3紧密跟驰", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 76.45, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 22.94, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 82.94}], "penaltyPoint": {"PET": 0.9129540068577612, "TTC": 0, "crash": 0, "useTime": 47.13599991798401, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 10.58, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 47.14, "allSenseScore": 92.94, "gradeTableDict": {"安全性": "优秀", "效率性": "一般", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 76.45, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 22.94, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1511": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724988005.3779998, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "4单车变道", "startTime": 1724987974.7470002, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 30.63, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 30.63, "unit": "s", "index": "任务耗时", "score": "0.0/0.01", "overTime": 1.55, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "10.16/15", "avgSpeed": 13.55, "belowSpeed": 0.32, "expectSpeed": 20}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}], "comfortable": [{"time": 7.11, "type": "a", "unit": "s", "index": "横向舒适度", "score": "1.16"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [10, 10, 10, 10, 10, 10, 10, 10, 8.23, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [-0.13, -0.11, -0.19, 0.09, 0.04, -0.01, -0.12, -0.85, -8.44, -1.13, -5.37, -7.93, -9.81, -13.91]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [0, 0, 2, 1, 2, 0, 0, 0, 1, -1, 0, 0, -0.91, 1]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [0, 0, 0.01, 0.01, 0, 0, 0.01, 0.07, 0.7, 0.09, 0.42, 0.58, 0.62, 0.94]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [0.99, 0, 1.09, -1.5, 2, -0.99, -1, 1, 1.71, -1, 0, 1, 0.08, 1.91]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [-0, -0, 0, 0, 0, -0, 0.01, 0.06, 0.1, -0.31, 0.16, -0.04, -0.03, 0.26]}}, "eventTable": {}, "senseScore": 94, "senseWeight": 100, "sceneNameList": "4单车变道", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 83.88, "safeWeight": 0.4, "comfortable": 76.8, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 25.16, "comfortable*weight": 3.84, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 82.84}], "penaltyPoint": {"PET": 0.9346740322708748, "TTC": 0, "crash": 0, "useTime": 30.63099956512451, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 25, "overLateralJerk": 0, "avgSpeedNotScore": 13.55, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 5, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 30.63, "allSenseScore": 94, "gradeTableDict": {"安全性": "优秀", "效率性": "良好", "舒适性": "一般", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 83.88, "safeWeight": 0.4, "comfortable": 76.8, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 25.16, "comfortable*weight": 3.84, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1519": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724988073.931, "diagnose": {"建议": "从安全性方面，建议提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性良好，存在较轻违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "6丁字路口左转", "startTime": 1724988058.42, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 2, "unit": "s", "index": "TTC", "score": "5.16", "timeRatio": 0.13, "timeRatio：": 0.13}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 15.51, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 15.51, "unit": "s", "index": "任务耗时", "score": "0.01/0.01", "overTime": 0.29, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "9.1/15", "avgSpeed": 12.13, "belowSpeed": 0.39, "expectSpeed": 20}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}], "comfortable": [{"time": 8.31, "type": "a", "unit": "s", "index": "横向舒适度", "score": "2.68"}, {"time": 1, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.32"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0.26, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [-11.78, -10.12, 0.15, -0.33, -0.25, -0.14, -0.11]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0, -0.48, -1, 0, -1, 0, -0.83]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0.91, 0.64, 0.01, 0.02, 0.01, 0.01, 0.01]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [-1, -0.08, -0.52, 1, -1, 1, -0.69]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0.09, 0.08, -4, 0.01, -0.01, -0.01, -0]}}, "eventTable": {"1": {"time": "2024-08-30 11:21:55.496", "index": "TTC", "place": [-1206.083212, -451.058214], "simuTime": 2.000999927520752}, "2": {"time": "2024-08-30 11:21:55.505", "index": "TTC", "place": [-1203.305829, -454.175681], "simuTime": 3.002000093460083}}, "senseScore": 85.93, "senseWeight": 100, "sceneNameList": "6丁字路口左转", "senseAggScore": 100, "AbilityDimension": {"safe": 87.09, "compliance": 100, "efficiency": 80.32, "safeWeight": 0.4, "comfortable": 40, "safe*weight": 34.84, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 24.1, "comfortable*weight": 2, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 72.93}], "penaltyPoint": {"PET": 0.935529623796815, "TTC": 0.12906969918418298, "crash": 0, "useTime": 15.510999917984009, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {"1": {"time": "2024-08-30 11:21:55.496", "index": "TTC", "place": [-1206.083212, -451.058214], "simuTime": 2.000999927520752}, "2": {"time": "2024-08-30 11:21:55.505", "index": "TTC", "place": [-1203.305829, -454.175681], "simuTime": 3.002000093460083}}, "reverseCar": 0, "TTCAccuTime": 2.002000093460083, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.536, "overLateralJerk": 0.064, "avgSpeedNotScore": 12.13, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 3, "overLateralJerkNum": 1, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 15.51, "allSenseScore": 85.93, "gradeTableDict": {"安全性": "较差", "效率性": "良好", "舒适性": "不合格", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 87.09, "compliance": 100, "efficiency": 80.32, "safeWeight": 0.4, "comfortable": 40, "safe*weight": 34.84, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 24.1, "comfortable*weight": 2, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0.13}}], "1520": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724988053.409, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时较长、效率较低，或任务未完成。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "5停车标识+车辆冲突", "startTime": 1724988006.571, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 46.84, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 46.84, "unit": "s", "index": "任务耗时", "score": "0.0/0.01", "overTime": 2.9, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "2.45/15", "avgSpeed": 3.27, "belowSpeed": 0.84, "expectSpeed": 20}, {"time": 11.8, "unit": "s", "index": "倒车", "score": "1.007899535771466"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 1.01, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.11"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [2.04, 2.82, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0.01, -0.04, -0.04, -0.01, 0, -0.01, -0.01, -0.01, 0, 0.01, 0.01, 0, 0.01, -0.01, 0, 0, 0, 0, 0.07, 0.16, -0.15, -0.24]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, -1, -3.96, -1.67, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.02]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [1, -1, -1.44, -0.56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.99, 1, 1, -1]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, -0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0, 0]}}, "eventTable": {"1": {"time": "2024-08-30 11:21:55.550", "index": "倒车", "place": [-1226.6701, -395.091781], "simuTime": 13.417999744415283}, "2": {"time": "2024-08-30 11:21:55.563", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 17.520999908447266}, "3": {"time": "2024-08-30 11:21:55.565", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 18.521999835968018}, "4": {"time": "2024-08-30 11:21:55.577", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 22.72499990463257}, "5": {"time": "2024-08-30 11:21:55.584", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 24.7260000705719}, "6": {"time": "2024-08-30 11:21:55.588", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 25.726999759674072}, "7": {"time": "2024-08-30 11:21:55.591", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 26.727999925613403}, "8": {"time": "2024-08-30 11:21:55.600", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 31.5310001373291}, "9": {"time": "2024-08-30 11:21:55.602", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 32.723999977111816}, "10": {"time": "2024-08-30 11:21:55.605", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 33.72499990463257}}, "senseScore": 86.34, "senseWeight": 100, "sceneNameList": "5停车标识+车辆冲突", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 54.82, "safeWeight": 0.4, "comfortable": 97.85, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 16.44, "comfortable*weight": 4.89, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 76.23}], "penaltyPoint": {"PET": 0.9357359408603012, "TTC": 0, "crash": 0, "useTime": 46.83799982070923, "avgSpeed": 0, "crashNum": 0, "overAcce": 5, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {"1": {"time": "2024-08-30 11:21:55.550", "index": "倒车", "place": [-1226.6701, -395.091781], "simuTime": 13.417999744415283}, "2": {"time": "2024-08-30 11:21:55.563", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 17.520999908447266}, "3": {"time": "2024-08-30 11:21:55.565", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 18.521999835968018}, "4": {"time": "2024-08-30 11:21:55.577", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 22.72499990463257}, "5": {"time": "2024-08-30 11:21:55.584", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 24.7260000705719}, "6": {"time": "2024-08-30 11:21:55.588", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 25.726999759674072}, "7": {"time": "2024-08-30 11:21:55.591", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 26.727999925613403}, "8": {"time": "2024-08-30 11:21:55.600", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 31.5310001373291}, "9": {"time": "2024-08-30 11:21:55.602", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 32.723999977111816}, "10": {"time": "2024-08-30 11:21:55.605", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 33.72499990463257}}, "reverseCar": 0.2519748839428665, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 1, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 3.27, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 46.84, "allSenseScore": 86.34, "gradeTableDict": {"安全性": "优秀", "效率性": "不合格", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 54.82, "safeWeight": 0.4, "comfortable": 97.85, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 16.44, "comfortable*weight": 4.89, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}]}}"""
    # vehi_regional_score = """{"8": {"1510": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724987960.937, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "3紧密跟驰", "startTime": 1724987913.801, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 47.14, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 47.14, "unit": "s", "index": "任务耗时", "score": "0.0/0.01", "overTime": 2.93, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "7.94/15", "avgSpeed": 10.58, "belowSpeed": 0.47, "expectSpeed": 20}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [10, 10, 8.7, 8.84, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0.11, 0.07, 0.11, 0.07, 0.07, 0.08, -0.04, -0.03, -0.05, -0.05, -0.04, -0.11, -0.06, -0.52, -0.07, -0.07, -0.08, -0.05, -0.03, -0.08, 0.08, 0.04, 0.08]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0, 0, -1, -0.83, -0.83, 0, 1, 0, 0, 0, 0, 0, -0.83, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0.01, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [0, 0, -1, -0.69, -0.69, 0, 1, 0, -1, 1, 0, -0.91, -0.69, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46], "y": [-0, -0, 0, 0, 0, 0, -0, -0, -0, -0, -0, 0, -0, 0.02, -0, 0, 0, 0, -0, 0, 0, -0, -0.08]}}, "eventTable": {}, "senseScore": 92.94, "senseWeight": 100, "sceneNameList": "3紧密跟驰", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 76.45, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 22.94, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 82.94}], "penaltyPoint": {"PET": 0.9129540068577612, "TTC": 0, "crash": 0, "useTime": 47.13599991798401, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 10.58, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 47.14, "allSenseScore": 92.94, "gradeTableDict": {"安全性": "优秀", "效率性": "一般", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 76.45, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 22.94, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1511": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724988005.3779998, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "4单车变道", "startTime": 1724987974.7470002, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 30.63, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 30.63, "unit": "s", "index": "任务耗时", "score": "0.0/0.01", "overTime": 1.55, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "10.16/15", "avgSpeed": 13.55, "belowSpeed": 0.32, "expectSpeed": 20}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}], "comfortable": [{"time": 7.11, "type": "a", "unit": "s", "index": "横向舒适度", "score": "1.16"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [10, 10, 10, 10, 10, 10, 10, 10, 8.23, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [-0.13, -0.11, -0.19, 0.09, 0.04, -0.01, -0.12, -0.85, -8.44, -1.13, -5.37, -7.93, -9.81, -13.91]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [0, 0, 2, 1, 2, 0, 0, 0, 1, -1, 0, 0, -0.91, 1]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [0, 0, 0.01, 0.01, 0, 0, 0.01, 0.07, 0.7, 0.09, 0.42, 0.58, 0.62, 0.94]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [0.99, 0, 1.09, -1.5, 2, -0.99, -1, 1, 1.71, -1, 0, 1, 0.08, 1.91]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28], "y": [-0, -0, 0, 0, 0, -0, 0.01, 0.06, 0.1, -0.31, 0.16, -0.04, -0.03, 0.26]}}, "eventTable": {}, "senseScore": 94, "senseWeight": 100, "sceneNameList": "4单车变道", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 83.88, "safeWeight": 0.4, "comfortable": 76.8, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 25.16, "comfortable*weight": 3.84, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 82.84}], "penaltyPoint": {"PET": 0.9346740322708748, "TTC": 0, "crash": 0, "useTime": 30.63099956512451, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.23195457532506297, "overLateralJerk": 0, "avgSpeedNotScore": 13.55, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 5, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 30.63, "allSenseScore": 94, "gradeTableDict": {"安全性": "优秀", "效率性": "良好", "舒适性": "一般", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 83.88, "safeWeight": 0.4, "comfortable": 76.8, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 25.16, "comfortable*weight": 3.84, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1519": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724988073.931, "diagnose": {"建议": "从安全性方面，建议提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性良好，存在较轻违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "6丁字路口左转", "startTime": 1724988058.42, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 2, "unit": "s", "index": "TTC", "score": "5.16", "timeRatio": 0.13, "timeRatio：": 0.13}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 15.51, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 15.51, "unit": "s", "index": "任务耗时", "score": "0.01/0.01", "overTime": 0.29, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "9.1/15", "avgSpeed": 12.13, "belowSpeed": 0.39, "expectSpeed": 20}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}], "comfortable": [{"time": 8.31, "type": "a", "unit": "s", "index": "横向舒适度", "score": "2.68"}, {"time": 1, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.32"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0.26, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [-11.78, -10.12, 0.15, -0.33, -0.25, -0.14, -0.11]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0, -0.48, -1, 0, -1, 0, -0.83]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0.91, 0.64, 0.01, 0.02, 0.01, 0.01, 0.01]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [-1, -0.08, -0.52, 1, -1, 1, -0.69]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14], "y": [0.09, 0.08, -4, 0.01, -0.01, -0.01, -0]}}, "eventTable": {"1": {"time": "2024-08-30 15:22:03.347", "index": "TTC", "place": [-1206.083212, -451.058214], "simuTime": 2.000999927520752}, "2": {"time": "2024-08-30 15:22:03.348", "index": "TTC", "place": [-1203.305829, -454.175681], "simuTime": 3.002000093460083}}, "senseScore": 85.93, "senseWeight": 100, "sceneNameList": "6丁字路口左转", "senseAggScore": 100, "AbilityDimension": {"safe": 87.09, "compliance": 100, "efficiency": 80.32, "safeWeight": 0.4, "comfortable": 40, "safe*weight": 34.84, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 24.1, "comfortable*weight": 2, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 72.93}], "penaltyPoint": {"PET": 0.935529623796815, "TTC": 0.12906969918418298, "crash": 0, "useTime": 15.510999917984009, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {"1": {"time": "2024-08-30 15:22:03.347", "index": "TTC", "place": [-1206.083212, -451.058214], "simuTime": 2.000999927520752}, "2": {"time": "2024-08-30 15:22:03.348", "index": "TTC", "place": [-1203.305829, -454.175681], "simuTime": 3.002000093460083}}, "reverseCar": 0, "TTCAccuTime": 2.002000093460083, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.5354909443747518, "overLateralJerk": 0.06453485727755924, "avgSpeedNotScore": 12.13, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 3, "overLateralJerkNum": 1, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 15.51, "allSenseScore": 85.93, "gradeTableDict": {"安全性": "较差", "效率性": "良好", "舒适性": "不合格", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 87.09, "compliance": 100, "efficiency": 80.32, "safeWeight": 0.4, "comfortable": 40, "safe*weight": 34.84, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 24.1, "comfortable*weight": 2, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0.13}}], "1520": [{"1": {"code": "other", "avName": "8", "caseID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "taskID": "ff27dd5b-667e-11ef-8347-00163e21f0fc", "endTime": 1724988053.409, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时较长、效率较低，或任务未完成。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "5停车标识+车辆冲突", "startTime": 1724988006.571, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": "0", "unit": "次", "index": "未遵守停车标志", "score": "0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}], "efficiency": [{"time": 46.84, "unit": "s", "index": "任务完成", "score": "15/15"}, {"time": 46.84, "unit": "s", "index": "任务耗时", "score": "0.0/0.01", "overTime": 2.9, "expectTime": 12}, {"unit": "km/h", "index": "平均速度", "score": "2.45/15", "avgSpeed": 3.27, "belowSpeed": 0.84, "expectSpeed": 20}, {"time": 11.8, "unit": "s", "index": "倒车", "score": "1.007899535771466"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 1.01, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.11"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "转弯舒适度", "score": "0.0"}, {"time": 0, "type": "j", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [2.04, 2.82, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0.01, -0.04, -0.04, -0.01, 0, -0.01, -0.01, -0.01, 0, 0.01, 0.01, 0, 0.01, -0.01, 0, 0, 0, 0, 0.07, 0.16, -0.15, -0.24]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, -1, -3.96, -1.67, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.02]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [1, -1, -1.44, -0.56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.99, 1, 1, -1]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, -0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0, 0, 0]}}, "eventTable": {"1": {"time": "2024-08-30 15:22:03.362", "index": "倒车", "place": [-1226.6701, -395.091781], "simuTime": 13.417999744415283}, "2": {"time": "2024-08-30 15:22:03.379", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 17.520999908447266}, "3": {"time": "2024-08-30 15:22:03.383", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 18.521999835968018}, "4": {"time": "2024-08-30 15:22:03.400", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 22.72499990463257}, "5": {"time": "2024-08-30 15:22:03.406", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 24.7260000705719}, "6": {"time": "2024-08-30 15:22:03.409", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 25.726999759674072}, "7": {"time": "2024-08-30 15:22:03.412", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 26.727999925613403}, "8": {"time": "2024-08-30 15:22:03.432", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 31.5310001373291}, "9": {"time": "2024-08-30 15:22:03.433", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 32.723999977111816}, "10": {"time": "2024-08-30 15:22:03.434", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 33.72499990463257}}, "senseScore": 86.34, "senseWeight": 100, "sceneNameList": "5停车标识+车辆冲突", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 54.82, "safeWeight": 0.4, "comfortable": 97.85, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 16.44, "comfortable*weight": 4.89, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 76.23}], "penaltyPoint": {"PET": 0.9357359408603012, "TTC": 0, "crash": 0, "useTime": 46.83799982070923, "avgSpeed": 0, "crashNum": 0, "overAcce": 0.02154233375435047, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {"1": {"time": "2024-08-30 15:22:03.362", "index": "倒车", "place": [-1226.6701, -395.091781], "simuTime": 13.417999744415283}, "2": {"time": "2024-08-30 15:22:03.379", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 17.520999908447266}, "3": {"time": "2024-08-30 15:22:03.383", "index": "倒车", "place": [-1226.670101, -395.102874], "simuTime": 18.521999835968018}, "4": {"time": "2024-08-30 15:22:03.400", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 22.72499990463257}, "5": {"time": "2024-08-30 15:22:03.406", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 24.7260000705719}, "6": {"time": "2024-08-30 15:22:03.409", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 25.726999759674072}, "7": {"time": "2024-08-30 15:22:03.412", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 26.727999925613403}, "8": {"time": "2024-08-30 15:22:03.432", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 31.5310001373291}, "9": {"time": "2024-08-30 15:22:03.433", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 32.723999977111816}, "10": {"time": "2024-08-30 15:22:03.434", "index": "倒车", "place": [-1226.660903, -395.102875], "simuTime": 33.72499990463257}}, "reverseCar": 0.2519748839428665, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 1, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 3.27, "followStopSignal": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 46.84, "allSenseScore": 86.34, "gradeTableDict": {"安全性": "优秀", "效率性": "不合格", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 54.82, "safeWeight": 0.4, "comfortable": 97.85, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 16.44, "comfortable*weight": 4.89, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}]}}"""
    # vehi_regional_score = """{"1": {"1549": [{"1": {"code": "1-礼让行人横穿斑马线", "avName": "1", "caseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "taskID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "endTime": 1728717405, "diagnose": {"建议": "从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，有必要提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性差，存在较严重违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时较长、效率较低，或任务未完成。", "交规符合性方面": "交规符合性差，存在较多违规行为。", "交通协调性方面": "交通协调性差，存在较多影响其他车辆通行的行为。"}, "sceneName": "1-礼让行人横穿斑马线", "startTime": 1728717405, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0.0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 0, "unit": "s", "index": "任务完成", "score": "0/10"}, {"time": 0, "unit": "s", "index": "任务耗时", "score": "0/0", "overTime": -1, "expectTime": 9999}, {"unit": "km/h", "index": "任务耗时", "score": "0/10", "avgSpeed": 0, "belowSpeed": 0, "expectSpeed": 30}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}, {"time": 0, "unit": "次", "index": "倒车", "score": "0/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 0, "chartData": {"TTC": {"x": [2, 4], "y": [10, 10]}, "turnA": {"x": [2, 4], "y": [0.11, 0.07]}, "verticalA": {"x": [2, 4], "y": [0, 0]}, "horizontalA": {"x": [2, 4], "y": [0.01, 0]}, "verticalAPlus": {"x": [2, 4], "y": [0, 0]}, "horizontalAPlus": {"x": [2, 4], "y": [0, 0]}}, "eventTable": {"1": {"time": "2024-10-12 15:16:45", "index": "任务未完成", "place": [0, 0], "simuTime": 0}}, "senseScore": 0, "sceneCaseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "senseWeight": 25, "sceneNameList": "1-礼让行人横穿斑马线", "senseAggScore": 25, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "missionAccomplish": 0, "senseScore*senseWeight": 0}], "penaltyPoint": {"PET": 0, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 47.13599991798401, "avgSpeed": 0, "crashNum": 1, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {"1": {"time": "2024-10-12 15:16:45", "index": "任务未完成", "place": [0, 0], "simuTime": 0}}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 10.58, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 0, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 0, "allSenseScore": 0, "gradeTableDict": {"安全性": "不合格", "效率性": "不合格", "舒适性": "不合格", "交规符合性": "不合格", "交通协调性": "不合格"}, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "dangerTimeProportion": 0}}], "1554": [{"1": {"code": "other", "avName": "1", "caseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "taskID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "endTime": 1728267657.516, "diagnose": {"建议": "从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，建议提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性中等，存在严重违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性中等，存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "6-丁字路口左转", "startTime": 1728267602.4220002, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 3.3, "unit": "s", "index": "TTC", "score": "0.6", "timeRatio": 0.06, "timeRatio：": 0.06}, {"time": 1, "unit": "次", "index": "TTC", "score": "10"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 32.99, "unit": "s", "index": "超速", "score": "1.5"}, {"time": 1, "unit": "次", "index": "超速", "score": "2.5"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 55.09, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 55.09, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 3.59, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "6.68/10", "avgSpeed": 10.02, "belowSpeed": 0.33, "expectSpeed": 15}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 5.48, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.5"}, {"time": 0.96, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.09"}, {"time": 7.52, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.68"}, {"time": 16.38, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "1.49"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 24.77, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54], "y": [6.09, 1.23, 1.65, 0.86, 1.79, 0.48, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 3.58, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54], "y": [-0.33, 2.5, -9.66, -6.84, -14.22, -0.07, -0.18, 0.03, -0.03, 0.02, -0.03, 0, 0.1, -0.17, -2.96, -6.06, -6.73, -0.58, 0.87, -0.62, 0.06, 0.18, -0.68, 0.72, 7.91, -4.54, -2.5]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54], "y": [0, 0, 0, 0, -3.31, -3.03, 0, 0, 0, 0, 0, 0, 0, 0, 2.49, 2.51, 0, 0, 0, 3.32, 0, -3.32, 0, -2.99, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54], "y": [0.02, 0.13, 0.52, 0.36, 0.62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0.26, 0.42, 0.04, 0.08, 0.07, 0.01, 0.02, 0.06, 0.06, 0.61, 0.33, 0.17]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54], "y": [0, 0, -5.79, 0, -10.96, 7.15, 0, 0, 0, 0, 0, 0, 0, 0, 0, -6.28, -6.63, 0, -7.23, 11.04, 8.24, -11.04, 0, -8.91, 6.23, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54], "y": [0.02, 0.01, 0.69, 0.79, -0.51, -0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, -0.04, 0.26, -0.4, -0.12, 0.15, 0.01, 0.01, 0.13, 0.16, -0.07, -0.21, -0.18]}}, "eventTable": {"1": {"time": "2024-10-12 15:16:39.679", "index": "超速", "place": [-1213.273743, -433.541572], "simuTime": 0.7839999198913574}, "2": {"time": "2024-10-12 15:16:39.716", "index": "TTC", "place": [-1210.175102, -441.772916], "simuTime": 3.254999876022339}}, "senseScore": 81.43, "senseWeight": 100, "sceneNameList": "6-丁字路口左转", "senseAggScore": 100, "AbilityDimension": {"safe": 75, "compliance": 75, "efficiency": 88.93, "safeWeight": 0.4, "comfortable": 44.92, "safe*weight": 30, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 26.68, "comfortable*weight": 2.25, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 68.67}], "penaltyPoint": {"PET": 0.99121501490577, "TTC": 0.05986133848209682, "crash": 0, "TTCNum": 1, "useTime": 55.0939998626709, "avgSpeed": 0, "crashNum": 0, "overAcce": 0.13649399208028298, "overJerk": 0.2974007891536415, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0.5988129382381373, "eventTable": {"1": {"time": "2024-10-12 15:16:39.679", "index": "超速", "place": [-1213.273743, -433.541572], "simuTime": 0.7839999198913574}, "2": {"time": "2024-10-12 15:16:39.716", "index": "TTC", "place": [-1210.175102, -441.772916], "simuTime": 3.254999876022339}}, "reverseCar": 0, "TTCAccuTime": 3.2980005741119385, "breakSignal": 0, "overAcceNum": 1, "overJerkNum": 1, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 1, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.09944821335882836, "overLateralJerk": 0.0174610646456309, "avgSpeedNotScore": 10.02, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 1, "overLateralJerkNum": 1, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 24.768999814987183, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 55.09, "allSenseScore": 81.43, "gradeTableDict": {"安全性": "不合格", "效率性": "良好", "舒适性": "不合格", "交规符合性": "一般", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 75, "compliance": 75, "efficiency": 88.93, "safeWeight": 0.4, "comfortable": 44.92, "safe*weight": 30, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 26.68, "comfortable*weight": 2.25, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1560": [{"1": {"code": "other", "avName": "1", "caseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "taskID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "endTime": 1728267461.665, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，建议提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性中等，存在较多违规行为。", "交通协调性方面": "交通协调性差，存在较多影响其他车辆通行的行为。"}, "sceneName": "2-非机动车占用前方车道", "startTime": 1728267412.655, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 27.47, "unit": "s", "index": "超速", "score": "1.4"}, {"time": 1, "unit": "次", "index": "超速", "score": "2.5"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 49.01, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 49.01, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 3.08, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "4.62/10", "avgSpeed": 11.54, "belowSpeed": 0.54, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 4.64, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.47"}, {"time": 17.02, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "1.74"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 3.61, "unit": "s", "index": "干扰背景车辆通行", "score": "7.21", "maxDisturbedSvSimuTime": 48.61, "svSimuTimeWithoutDisturbance": 45}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [10, 8.25, 4.59, 5.73, 6.12, 6.56, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9.97]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [-0.11, -0.37, 0.55, 0.24, -1.08, -0.63, 0.05, 0.15, 0.11, 0.07, 0.03, -0.04, -0.03, 0, 0.11, 0.05, 0, 0, 0.03, 0, -0.03, -0.13, -0.25, -0.03]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [0, 2.57, 0, 0, -2.99, -5.45, -2.33, -2.33, 0, 0, 0, 0, 0, 0, 0, -2.49, 0, 0, 0, 0, 0, 0, 3.28, 2.73]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [0.01, 0.04, 0.06, 0.02, 0.1, 0.04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.02, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [0, 6.61, 0, 0, -8.91, -14.85, 0.38, -5.41, 8.33, 0, 0, 11.07, -9.13, -7.53, 0, -6.22, 0, 0, -9.08, 0, 0, -4.99, 1.84, -0.64]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [-0.02, 0.07, 0.06, 0.03, 0.1, 0.1, -0.01, -0, 0, 0, 0, 0, 0, -0.01, 0.01, 0, -0.01, 0, -0, -0, -0, -0.01, 0.02, -0.01]}}, "eventTable": {"1": {"time": "2024-10-12 15:16:39.708", "index": "超速", "place": [-794.940455, -200.41349], "simuTime": 0.7730000019073486}}, "senseScore": 82.7, "senseWeight": 100, "sceneNameList": "2-非机动车占用前方车道", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 75, "efficiency": 82.05, "safeWeight": 0.4, "comfortable": 55.8, "safe*weight": 40, "coordination": 51.93, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 24.62, "comfortable*weight": 2.79, "coordinationWeight": 0.15, "coordination*weight": 7.79}, "missionAccomplish": 1, "senseScore*senseWeight": 77.7}], "penaltyPoint": {"PET": 0.9917363809127, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 49.00999999046326, "avgSpeed": 0, "crashNum": 0, "overAcce": 0.09475616845319013, "overJerk": 0.34721482903823875, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0.5605794732188925, "eventTable": {"1": {"time": "2024-10-12 15:16:39.708", "index": "超速", "place": [-794.940455, -200.41349], "simuTime": 0.7730000019073486}}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 1, "overJerkNum": 1, "coordination": 3.6050000190734863, "outOfLaneNum": 0, "overSpeedNum": 1, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 11.54, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 3.6050000190734863, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 48.605000019073486, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 45}, "testDuration": 49.01, "allSenseScore": 82.7, "gradeTableDict": {"安全性": "优秀", "效率性": "良好", "舒适性": "不合格", "交规符合性": "一般", "交通协调性": "不合格"}, "AbilityDimension": {"safe": 100, "compliance": 75, "efficiency": 82.05, "safeWeight": 0.4, "comfortable": 55.8, "safe*weight": 40, "coordination": 51.93, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 24.62, "comfortable*weight": 2.79, "coordinationWeight": 0.15, "coordination*weight": 7.79}, "dangerTimeProportion": 0}}], "1561": [{"1": {"code": "other", "avName": "1", "caseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "taskID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "endTime": 1728267565.9729998, "diagnose": {"建议": "从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，有必要提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性差，存在较严重违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时较长、效率较低，或任务未完成。", "交规符合性方面": "交规符合性差，存在较多违规行为。", "交通协调性方面": "交通协调性差，存在较多影响其他车辆通行的行为。"}, "sceneName": "3-高密度交通流紧密跟驰", "startTime": 1728267461.9970002, "testScene": [{"info": {"safe": [{"time": 1, "unit": "次", "index": "碰撞", "score": "100"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0.93, "unit": "s", "index": "TTC", "score": "0.09", "timeRatio": 0.01, "timeRatio：": 0.01}, {"time": 1, "unit": "次", "index": "TTC", "score": "10"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 55.3, "unit": "s", "index": "超速", "score": "1.33"}, {"time": 2, "unit": "次", "index": "超速", "score": "5.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 103.98, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 103.98, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 7.66, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "4.61/10", "avgSpeed": 11.52, "belowSpeed": 0.54, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 1.8, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.09"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 14.14, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.68"}, {"time": 39.64, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "1.91"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 17.94, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102], "y": [9.98, 10, 8.69, 6.98, 5.99, 6.03, 5.96, 9.43, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 2.83, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102], "y": [0.22, 0.03, 0.11, -0.38, 0.09, 0.22, 0.11, 0, -0.07, -0.03, -0.03, -0.04, -0.03, -0.18, -0.22, 0.19, 0.4, -0.03, -0.03, -0.03, -0.14, -0.07, 0.09, 0.18, 0, 0.44, 0.93, -0.33, -1.06, 0.65, -0.26, -0.76, 0.6, 0, -4.45, -5.39, 1.62, 4.07, 2.28, 1.14, -0.81, -3.29, -3.98, -3.43, -3.06, -2.64, -2.57, -3.95, -10.41, 0.32, -10.38]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102], "y": [-2.51, -3, 0, 0, -4.3, -5.01, -2.51, 0, 0, 0, 0, 0, 0, 0, 3.32, 0, -3.32, -2.94, 0, 0, 2.51, 3.3, 2.16, 3.33, 0, 0, -2.5, 0, 0, 0, 0, 0, -2.6, 0, 0, 0, -2.31, 0, 0, 0, 0, 0, 0, 0, 0, 0, -3, 2.7, 2, 0, -2.52]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102], "y": [0.02, 0, 0.01, 0.04, 0.01, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.02, 0, 0, 0, 0.01, 0, 0.01, 0.02, 0, 0.05, 0.1, 0.03, 0.1, 0.06, 0.02, 0.07, 0.04, 0, 0.24, 0.21, 0.05, 0.12, 0.06, 0.03, 0.02, 0.1, 0.12, 0.1, 0.09, 0.06, 0.02, 0.08, 0.4, 0.02, 0.55]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102], "y": [-12.58, -9.02, 0, -6.83, -9.25, -6.31, -6.28, 5.35, 0, 0, 0, 0, 0, -11.11, -0.04, 0, -11.04, -8.65, 0, 0, 6.28, 10.89, 4.64, 11.11, -11.07, 0, -6.25, 0, 0, -7.41, 0, 0, 0.68, 0, 0, 0, -5.33, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9.02, 7.3, -2.67, -6.1, -6.34]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102], "y": [0.01, -0, -0.01, -0.03, 0.02, -0.01, 0, -0.01, -0, -0, 0, -0, -0, 0.01, 0.03, 0.01, -0.01, -0, 0, 0, 0.01, 0.01, -0.01, -0.01, -0.03, 0.1, 0.14, 0.06, -0.34, -0.11, 0.02, -0.01, -0.01, -0.05, 0.27, -0.37, -0, -0.02, -0.01, 0, 0.03, 0.05, 0.05, 0.02, -0.01, 0.1, -0.1, 0.03, 0.44, -0.69, 0.54]}}, "eventTable": {"1": {"time": "2024-10-12 15:16:39.618", "index": "超速", "place": [-950.589104, -248.309244], "simuTime": 0.7639999389648438}, "2": {"time": "2024-10-12 15:16:42.430", "index": "超速", "place": [-1165.447915, -314.21009], "simuTime": 61.14599967002869}, "3": {"time": "2024-10-12 15:16:43.542", "index": "碰撞", "place": [-1233.957327, -344.296364], "simuTime": 90}, "4": {"time": "2024-10-12 15:16:43.829", "index": "TTC", "place": [-1238.93428, -351.16222], "simuTime": 96.9599997997284}}, "senseScore": 0, "senseWeight": 100, "sceneNameList": "3-高密度交通流紧密跟驰", "senseAggScore": 100, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "missionAccomplish": 1, "senseScore*senseWeight": 0}], "penaltyPoint": {"PET": 0.9961241067217018, "TTC": 0.008963608562157497, "crash": 0.07044894819848904, "TTCNum": 1, "useTime": 103.97599959373474, "avgSpeed": 0, "crashNum": 1, "overAcce": 0.13601216138377945, "overJerk": 0.3811937452929981, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0.5318727400884743, "eventTable": {"1": {"time": "2024-10-12 15:16:39.618", "index": "超速", "place": [-950.589104, -248.309244], "simuTime": 0.7639999389648438}, "2": {"time": "2024-10-12 15:16:42.430", "index": "超速", "place": [-1165.447915, -314.21009], "simuTime": 61.14599967002869}, "3": {"time": "2024-10-12 15:16:43.542", "index": "碰撞", "place": [-1233.957327, -344.296364], "simuTime": 90}, "4": {"time": "2024-10-12 15:16:43.829", "index": "TTC", "place": [-1238.93428, -351.16222], "simuTime": 96.9599997997284}}, "reverseCar": 0, "TTCAccuTime": 0.9320001602172852, "breakSignal": 0, "overAcceNum": 2, "overJerkNum": 2, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 2, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.017282836200425523, "overLateralJerk": 0, "avgSpeedNotScore": 11.52, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 1, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 17.941999673843384, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 103.98, "allSenseScore": 0, "gradeTableDict": {"安全性": "不合格", "效率性": "不合格", "舒适性": "不合格", "交规符合性": "不合格", "交通协调性": "不合格"}, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "dangerTimeProportion": 0}}], "1564": [{"1": {"code": "other", "avName": "1", "caseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "taskID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "endTime": 1728267775.974, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，建议提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性中等，存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "8-施工区避撞", "startTime": 1728267673.664, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 99.54, "unit": "s", "index": "超速", "score": "2.43"}, {"time": 1, "unit": "次", "index": "超速", "score": "2.5"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 102.31, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 102.31, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 7.53, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "7.53/10", "avgSpeed": 18.83, "belowSpeed": 0.25, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 3.23, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.16"}, {"time": 1.44, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.07"}, {"time": 7.39, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.36"}, {"time": 34.36, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "1.68"}, {"time": 0.4, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.02"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 8.56, 6.28, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100], "y": [0.29, -0.03, -0.51, 0, 0.38, 0, -0.22, -0.05, 0.22, 0.03, 0.11, 0, 0.73, 0.07, -0.11, -0.07, 0.58, 0.05, -0.63, -0.3, 0.29, 0.07, -0.11, -0.25, 0.03, 0.28, 0, 0.07, 0.07, 0.1, 0.05, -0.47, 0.61, 0.19, -0.44, -0.18, 0.13, 0.15, 0.03, -0.14, 0, 0.18, 0.27, -0.07, -0.58, 0.18, -1.52, -4.82, -10.86, -22.91]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 2.5, -2.5, 0, 0, 0, -3.32, 0, 0, 2.33, 0, 0, 0, 0, 0, 0, 0, -2.51, 0, 3.06, 0, 2.19, 2.12, 0, 0, 0, -3.33, 0, 0, 0, 0, 0, 0, 0, 0, -3.15, -2.51, 0, 0, 0, -2.92, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100], "y": [0.03, 0, 0.05, 0, 0.04, 0, 0.02, 0.01, 0.02, 0, 0.01, 0, 0.07, 0.01, 0.01, 0.01, 0.05, 0, 0.06, 0.03, 0.03, 0.01, 0.01, 0.02, 0, 0.02, 0, 0, 0, 0.01, 0, 0.05, 0.07, 0.02, 0.04, 0.02, 0.01, 0.01, 0, 0.01, 0, 0.02, 0.03, 0.01, 0.05, 0.02, 0.1, 0.26, 0.47, 0.89]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100], "y": [0, 0, 0, 0, 0, 0, 0, 0, -4.99, 6.25, -6.25, 11.07, 0, 0, -11.04, 0, 0, 5.43, 0, 0, 0, 0, 0, -8.21, 0, -6.28, 6.25, 9.35, 0, -1.55, -1.95, 0, 0, 0, -11.11, 0, -8.2, 0, 6.25, 0, 0, 0, 0, -9.95, -6.31, 11.04, 5.85, 0, -8.55, 4.96]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100], "y": [-0.01, 0.01, 0.02, -0.01, 0.02, -0.01, 0.05, -0.01, 0.02, -0.03, -0.02, -0.66, 0.16, -0.01, 0.01, -0.01, -0.32, -0.08, -0.04, -0.04, 0.03, -0, 0.03, 0.01, -0.03, 0.03, -0.01, -0.01, -0, 0.01, 0.01, -0.13, 0.02, -0.05, 0.13, -0, 0.01, -0.15, 0, 0.01, -0.01, 0.01, 0, -0.03, 0.08, -0.01, 0.19, -1.05, 0.48, -0.16]}}, "eventTable": {"1": {"time": "2024-10-12 15:16:39.494", "index": "超速", "place": [-995.084262, -386.844028], "simuTime": 0.8999998569488525}}, "senseScore": 92.74, "senseWeight": 100, "sceneNameList": "8-施工区避撞", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 75, "efficiency": 91.77, "safeWeight": 0.4, "comfortable": 54.23, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 27.53, "comfortable*weight": 2.71, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 80.46}], "penaltyPoint": {"PET": 0.99512266802522, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 102.30999994277954, "avgSpeed": 0, "crashNum": 0, "overAcce": 0.07224123032916804, "overJerk": 0.33585181901491146, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0.9728961023845208, "eventTable": {"1": {"time": "2024-10-12 15:16:39.494", "index": "超速", "place": [-995.084262, -386.844028], "simuTime": 0.8999998569488525}}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 2, "overJerkNum": 2, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 1, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.03160981534358694, "overLateralJerk": 0.014035777030012558, "avgSpeedNotScore": 18.83, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 1, "overLateralJerkNum": 1, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0.003958557048346017, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0.4049999713897705, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 102.31, "allSenseScore": 92.74, "gradeTableDict": {"安全性": "优秀", "效率性": "优秀", "舒适性": "不合格", "交规符合性": "一般", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 75, "efficiency": 91.77, "safeWeight": 0.4, "comfortable": 54.23, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 27.53, "comfortable*weight": 2.71, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1567": [{"1": {"code": "5-停车标志识别", "avName": "1", "caseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "taskID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "endTime": 1728717405, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，算法很高效，无提升建议。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务完成，且效率较高。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "5-停车标志识别", "startTime": 1728717405, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 47.14, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 47.14, "unit": "s", "index": "任务耗时", "score": "0/0", "overTime": -1, "expectTime": 9999}, {"unit": "km/h", "index": "任务耗时", "score": "10/10", "avgSpeed": 30, "belowSpeed": 0, "expectSpeed": 30}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0.0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 0, "chartData": {"TTC": {"x": [2, 4], "y": [10, 10]}, "turnA": {"x": [2, 4], "y": [0.11, 0.07]}, "verticalA": {"x": [2, 4], "y": [0, 0]}, "horizontalA": {"x": [2, 4], "y": [0.01, 0]}, "verticalAPlus": {"x": [2, 4], "y": [0, 0]}, "horizontalAPlus": {"x": [2, 4], "y": [0, 0]}}, "eventTable": {}, "senseScore": 100, "sceneCaseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "senseWeight": 25, "sceneNameList": "5-停车标志识别", "senseAggScore": 25, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 100, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 30, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 0, "senseScore*senseWeight": 25}], "penaltyPoint": {"PET": 0.9129540068577612, "TTC": 0, "crash": 0, "useTime": 47.13599991798401, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 30, "followStopSignal": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 47.14, "allSenseScore": 100, "gradeTableDict": {"安全性": "优秀", "效率性": "优秀", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 100, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 30, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1576": [{"1": {"code": "other", "avName": "1", "caseID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "taskID": "2b68ee0d-8453-11ef-8347-00163e21f0fc", "endTime": 1728267821.293, "diagnose": {"建议": "从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，建议提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性中等，存在严重违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性中等，存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "10-环岛汇入汇出", "startTime": 1728267776.3520002, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 2.91, "unit": "s", "index": "TTC", "score": "0.65", "timeRatio": 0.06, "timeRatio：": 0.06}, {"time": 1, "unit": "次", "index": "TTC", "score": "10"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 28.22, "unit": "s", "index": "超速", "score": "1.57"}, {"time": 1, "unit": "次", "index": "超速", "score": "2.5"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 44.94, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 44.94, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 2.75, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "5.27/10", "avgSpeed": 7.9, "belowSpeed": 0.47, "expectSpeed": 15}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 14.26, "type": "a", "unit": "s", "index": "横向舒适度", "score": "1.59"}, {"time": 4.13, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.46"}, {"time": 4.85, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.54"}, {"time": 8.59, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.96"}, {"time": 0.6, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.07"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [1.83, 0.06, 10, 10, 10, 0.26, 0.83, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [2.45, 0.68, 1.61, 0, -0.03, 2.98, 23.58, 7.37, -10.96, -9.83, -14.43, -13.43, -0.8, -5.09, -11.8, -19.37, -14.59, 3.84, -0.13, 6.7, 17.18, -1.15]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, -2.71, -3.33, 0, 0, 2.31, 3.26, 0, 0, 0, -2.55, 0, 0, 0, 0, -2.92, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0.11, 0.02, 0, 0, 0, 0.03, 0.69, 0.36, 0.58, 0.48, 0.56, 0.52, 0.03, 0.22, 0.57, 0.85, 0.64, 0.19, 0.01, 0.32, 0.83, 0.06]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, -0.55, -11.11, 0, 0, -2.36, 10.61, -8.38, 0, 0, -6.51, 0, 0, 0, -0.71, -8.55, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [-0.12, -0.03, -0.02, 0, 0, 0.05, -89.27, -0.15, -0.54, 0.55, -0.11, -0.27, 0, 0.17, 0.12, -20.4, -0.21, 0.11, -0.01, 0.58, -1.09, 0.1]}}, "eventTable": {"1": {"time": "2024-10-12 15:16:39.504", "index": "超速", "place": [-514.003905, -173.610514], "simuTime": 0.7009997367858887}, "2": {"time": "2024-10-12 15:16:39.620", "index": "TTC", "place": [-516.873137, -165.534711], "simuTime": 4.2709996700286865}}, "senseScore": 79.16, "senseWeight": 100, "sceneNameList": "10-环岛汇入汇出", "senseAggScore": 100, "AbilityDimension": {"safe": 75, "compliance": 75, "efficiency": 84.22, "safeWeight": 0.4, "comfortable": 27.81, "safe*weight": 30, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 25.27, "comfortable*weight": 1.39, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 65.55}], "penaltyPoint": {"PET": 0.9732093294287104, "TTC": 0.06477381521957881, "crash": 0, "TTCNum": 1, "useTime": 44.94099974632263, "avgSpeed": 0, "crashNum": 0, "overAcce": 0.1079637646835508, "overJerk": 0.19125073678535137, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0.6280011670260859, "eventTable": {"1": {"time": "2024-10-12 15:16:39.504", "index": "超速", "place": [-514.003905, -173.610514], "simuTime": 0.7009997367858887}, "2": {"time": "2024-10-12 15:16:39.620", "index": "TTC", "place": [-516.873137, -165.534711], "simuTime": 4.2709996700286865}}, "reverseCar": 0, "TTCAccuTime": 2.9110000133514404, "breakSignal": 0, "overAcceNum": 1, "overJerkNum": 1, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 1, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.3173939009862877, "overLateralJerk": 0.09183149910994234, "avgSpeedNotScore": 7.9, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 1, "overLateralJerkNum": 1, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0.013417590374427413, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0.6029999256134033, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 44.94, "allSenseScore": 79.16, "gradeTableDict": {"安全性": "不合格", "效率性": "良好", "舒适性": "不合格", "交规符合性": "一般", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 75, "compliance": 75, "efficiency": 84.22, "safeWeight": 0.4, "comfortable": 27.81, "safe*weight": 30, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 7.5, "efficiency*weight": 25.27, "comfortable*weight": 1.39, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}]}}"""
    vehi_regional_score = """{"1": {"1549": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830133.801, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "1-礼让行人横穿斑马线", "startTime": 1728830113.302, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 1, "unit": "次", "index": "未有效泊车", "score": "100"}], "efficiency": [{"time": 20.5, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 20.5, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 0.71, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "9.66/10", "avgSpeed": 24.15, "belowSpeed": 0.03, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 20}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20], "y": [-0.01, -0, 0.15, 0.67, -0.02, 0, 0.03, 0.04, -0.02, -0]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20], "y": [0, 0, 0, 0.02, 0, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20], "y": [-0, -0, 0.01, -0.02, -0, -0, 0, -0, 0, -0]}}, "eventTable": {}, "senseScore": 99.66, "senseWeight": 100, "sceneNameList": "1-礼让行人横穿斑马线", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 98.87, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 29.66, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 89.66}], "penaltyPoint": {"PET": 0.9804868484339172, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 20.499000072479248, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 24.15, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 20}, "testDuration": 20.5, "allSenseScore": 99.66, "gradeTableDict": {"安全性": "优秀", "效率性": "优秀", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 98.87, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 29.66, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1590": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830241.202, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性差，存在较多影响其他车辆通行的行为。"}, "sceneName": "4-高速公路切入变道", "startTime": 1728830191.901, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 49.3, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 49.3, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 3.11, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "9.56/10", "avgSpeed": 14.34, "belowSpeed": 0.04, "expectSpeed": 15}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0.3, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.03"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 16.9, "unit": "s", "index": "干扰背景车辆通行", "score": "15.0", "maxDisturbedSvSimuTime": 46.9, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [-0.01, 0, 0, -1.02, -2.8, -2.49, 0.56, 1.99, 3.12, 1.5, -0.41, -0.26, -0.17, -0.08, -0.67, -0.54, -0.06, -0.3, -3.75, -8.29, -9.11, -12.84, -8.62, -3.08]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [0, 0, 0, -2.09, -0.93, 0, 0, 0, 0.92, 0, 0, 0.92, 0, 0, 0, -0.93, 0, 0, 0, 0, 0, 0, 0, 0.92]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [0, 0, 0, 0.02, 0.04, 0.04, 0.01, 0.02, 0.04, 0.03, 0.01, 0.01, 0, 0, 0.02, 0.01, 0, 0.01, 0.07, 0.15, 0.16, 0.22, 0.15, 0.07]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [0, 0, 0, -0.61, -3.09, -3.09, 2.31, 0, -0.01, 0, 0, 3.07, -1.74, 0, 0, -3.09, 0, 0, 0, 0, 0, 0, 0, 0.73]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48], "y": [-0, -0, -0, 0.01, 0.01, -0.05, 0, 0, 0.02, -0.01, 0, 0.01, 0, 0, 0, -0.03, 0, 0.01, 0.02, 0.02, 0.02, 0.15, -0.29, -0.21]}}, "eventTable": {}, "senseScore": 84.53, "senseWeight": 100, "sceneNameList": "4-高速公路切入变道", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 98.53, "safeWeight": 0.4, "comfortable": 99.39, "safe*weight": 40, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 29.56, "comfortable*weight": 4.97, "coordinationWeight": 0.15, "coordination*weight": 0}, "missionAccomplish": 1, "senseScore*senseWeight": 89.5}], "penaltyPoint": {"PET": 0.9938743675253362, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 49.301000118255615, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0.006085068286580205, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 1, "coordination": 16.901000022888184, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 14.34, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 16.901000022888184, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 46.901000022888184, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 49.3, "allSenseScore": 84.53, "gradeTableDict": {"安全性": "优秀", "效率性": "优秀", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "不合格"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 98.53, "safeWeight": 0.4, "comfortable": 99.39, "safe*weight": 40, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 29.56, "comfortable*weight": 4.97, "coordinationWeight": 0.15, "coordination*weight": 0}, "dangerTimeProportion": 0}}], "1591": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830191.601, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "3-高密度交通流紧密跟驰", "startTime": 1728830166.101, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 25.5, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 25.5, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 1.12, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "7.89/10", "avgSpeed": 19.72, "belowSpeed": 0.21, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], "y": [0.05, 0.01, 0, 0, 0.01, -0, 0, -0, 0, 0, -0, 0.01]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], "y": [0.93, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], "y": [0.78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24], "y": [-0, -0, -0, 0, 0, 0, -0, 0, 0, -0, 0, 0]}}, "eventTable": {}, "senseScore": 97.89, "senseWeight": 100, "sceneNameList": "3-高密度交通流紧密跟驰", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 92.96, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 27.89, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 87.89}], "penaltyPoint": {"PET": 0.9881960737938974, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 25.5, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 19.72, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 25.5, "allSenseScore": 97.89, "gradeTableDict": {"安全性": "优秀", "效率性": "优秀", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 92.96, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 27.89, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1592": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830265.702, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "5-停车标志识别", "startTime": 1728830241.502, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 24.2, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 24.2, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 1.02, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "3.14/10", "avgSpeed": 7.86, "belowSpeed": 0.69, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0.3, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.06"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 23.9, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22], "y": [0.02, 0.01, 0, 0, 0, 0.11, 0.04, 0.01, 0.04, -0.03, -0.03]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22], "y": [-0.71, -1.85, 0, 0, 0.69, 1.39, -0.93, 0, 0.93, 0.7, 0.93]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22], "y": [-1.81, 0, 0, 0, 1.73, 3.45, -0, -3.08, 0.01, 1.74, 3.09]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22], "y": [0, -0, 0, 0, 0, 0, -0, 0, 0, 0, -0]}}, "eventTable": {}, "senseScore": 93.08, "senseWeight": 100, "sceneNameList": "5-停车标志识别", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 77.15, "safeWeight": 0.4, "comfortable": 98.76, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 23.14, "comfortable*weight": 4.94, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 83.02}], "penaltyPoint": {"PET": 0.987603307779952, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 24.200000047683716, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0.012396692220048092, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 1, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 7.86, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 23.90000009536743, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 24.2, "allSenseScore": 93.08, "gradeTableDict": {"安全性": "优秀", "效率性": "一般", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 77.15, "safeWeight": 0.4, "comfortable": 98.76, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 23.14, "comfortable*weight": 4.94, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1595": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830433.467, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "9-无信号交叉口冲突", "startTime": 1728830393.668, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 39.8, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 39.8, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 2.32, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "6.46/10", "avgSpeed": 9.69, "belowSpeed": 0.35, "expectSpeed": 15}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0.3, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.04"}, {"time": 0.6, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.08"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0.3, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.04"}, {"time": 0.3, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.04"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [10, 10, 10, 10, 10, 10, 9.02, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [1.78, -7.3, -0.06, 0.16, -0.05, 0.06, -0.01, 0.04, 0, 0, 0, 0, 0, 0, 0.01, -0.57, -3.82, -13.64, -19.31]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [0, 0, 0, 0, 0, 0, -0.93, -0.92, 0, 0, 0, 0, 0, 0.7, 0.93, 0, 0.7, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [0.05, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.05, 0.24, 0.34]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [0, 0, 0, 0, 0, 2.36, -3.09, -0.73, 0, 0, 0, 0, 0, 1.74, 0.77, -1.74, 1.74, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [-0.09, 0.12, -0.02, 0, 0, -0, -0, 0, 0, 0, 0, 0, 0, 0, -0, 0.01, 0.05, 0.13, -0.58]}}, "eventTable": {}, "senseScore": 96.27, "senseWeight": 100, "sceneNameList": "9-无信号交叉口冲突", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 88.2, "safeWeight": 0.4, "comfortable": 96.23, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 26.46, "comfortable*weight": 4.81, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 86.08}], "penaltyPoint": {"PET": 0.9924621233666832, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 39.79900002479553, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0.007537882623883953, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 1, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.007537882623883953, "overLateralJerk": 0.015075759257200836, "avgSpeedNotScore": 9.69, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 1, "overLateralJerkNum": 1, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0.007537882623883953, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0.3000001907348633, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 39.8, "allSenseScore": 96.27, "gradeTableDict": {"安全性": "优秀", "效率性": "良好", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 88.2, "safeWeight": 0.4, "comfortable": 96.23, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 26.46, "comfortable*weight": 4.81, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1597": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830293.801, "diagnose": {"建议": "从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，有必要提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性差，存在较严重违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时较长、效率较低，或任务未完成。", "交规符合性方面": "交规符合性差，存在较多违规行为。", "交通协调性方面": "交通协调性差，存在较多影响其他车辆通行的行为。"}, "sceneName": "6-合流区汇入行驶", "startTime": 1728830266.101, "testScene": [{"info": {"safe": [{"time": 1, "unit": "次", "index": "碰撞", "score": "100"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 2.6, "unit": "s", "index": "TTC", "score": "0.94", "timeRatio": 0.09, "timeRatio：": 0.09}, {"time": 1, "unit": "次", "index": "TTC", "score": "10"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 27.7, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 27.7, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 1.31, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "10/10", "avgSpeed": 17.71, "belowSpeed": -0.18, "expectSpeed": 15}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0.3, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.05"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 17.2, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26], "y": [10, 8.79, 0.88, 1.62, 1.31, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26], "y": [-0.21, -3.91, -13.05, -13.69, -10.18, -3.97, 0.64, -0.01, -0.01, 0, -0, 0, -0]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26], "y": [0, 0, -1.39, 0, 0, 0.7, 0, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26], "y": [0.01, 0.1, 0.23, 0.22, 0.16, 0.09, 0.02, 0, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26], "y": [0, 0, 1.15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26], "y": [-0, 0.08, 0.06, -0.26, -0.21, -0.1, -0.01, 0, -0, -0, -0, -0, 0]}}, "eventTable": {"1": {"time": "2024-10-13 23:07:38.714", "index": "TTC", "place": [-1202.484362, -456.605518], "simuTime": 5.700999975204468}, "2": {"time": "2024-10-13 23:07:38.981", "index": "碰撞", "place": [-1192.939084, -461.704508], "simuTime": 8.900000095367432}}, "senseScore": 0, "senseWeight": 100, "sceneNameList": "6-合流区汇入行驶", "senseAggScore": 100, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "missionAccomplish": 1, "senseScore*senseWeight": 0}], "penaltyPoint": {"PET": 0.9891335697567702, "TTC": 0.09379060674007944, "crash": 0.19133573802139656, "TTCNum": 1, "useTime": 27.700000047683716, "avgSpeed": 0, "crashNum": 1, "overAcce": 0, "overJerk": 0.010830323169669824, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {"1": {"time": "2024-10-13 23:07:38.714", "index": "TTC", "place": [-1202.484362, -456.605518], "simuTime": 5.700999975204468}, "2": {"time": "2024-10-13 23:07:38.981", "index": "碰撞", "place": [-1192.939084, -461.704508], "simuTime": 8.900000095367432}}, "reverseCar": 0, "TTCAccuTime": 2.597999811172486, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 1, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 17.71, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 17.198999881744385, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 27.7, "allSenseScore": 0, "gradeTableDict": {"安全性": "不合格", "效率性": "不合格", "舒适性": "不合格", "交规符合性": "不合格", "交通协调性": "不合格"}, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "dangerTimeProportion": 0}}], "1599": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830378.966, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "8-社会车辆随机干扰", "startTime": 1728830340.1020002, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 38.86, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 38.86, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 2.24, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "7.89/10", "avgSpeed": 19.72, "belowSpeed": 0.21, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [-0, 0, -0.06, -0.05, 0, 0.06, -0.1, -5.46, 3.05, 3.76, -0.11, -0.02, -0.38, 0.32, 0.01, -0, -0, 0.01, 0]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [0, 0, 0, 0, 0, 0, 0, 0.15, 0.08, 0.1, 0, 0, 0.01, 0.01, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38], "y": [0, -0, 0, -0, 0, 0.01, -0, 0.03, 0.18, -0.1, 0.01, -0, 0.01, -0.01, -0, -0, -0, 0, 0]}}, "eventTable": {}, "senseScore": 97.89, "senseWeight": 100, "sceneNameList": "8-社会车辆随机干扰", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 92.96, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 27.89, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 87.89}], "penaltyPoint": {"PET": 0.9922807751769352, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 38.86399984359741, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 19.72, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 38.86, "allSenseScore": 97.89, "gradeTableDict": {"安全性": "优秀", "效率性": "优秀", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 92.96, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 27.89, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1600": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830165.801, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "2-低速车辆占用前方车道", "startTime": 1728830134.103, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 31.7, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 31.7, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 1.64, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "6.98/10", "avgSpeed": 17.44, "belowSpeed": 0.3, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 45}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30], "y": [-0, 0.01, 0.01, 0, -0, 0.05, -0.01, -0, 0.09, -0.05, 0.11, 0, 0.05, 0, -0.05]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30], "y": [0, 0, 0, 0, -1.85, -0.69, 0, 0, 0, 0, 0, 0, 0, 0.7, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30], "y": [0, 0, 0, 0, -3.09, -1.74, 0, 0, 0, 0, 0, 0, 0, 0.01, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30], "y": [0, 0, 0, -0, 0, -0, -0, -0, -0, 0, 0, -0, -0, -0, 0]}}, "eventTable": {}, "senseScore": 96.98, "senseWeight": 100, "sceneNameList": "2-低速车辆占用前方车道", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 89.92, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 26.98, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 86.98}], "penaltyPoint": {"PET": 0.9874124498258, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 31.698000192642212, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 17.44, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 45}, "testDuration": 31.7, "allSenseScore": 96.98, "gradeTableDict": {"安全性": "优秀", "效率性": "良好", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 89.92, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 26.98, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1601": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830474.367, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，建议提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性中等，存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "10-环岛汇入汇出", "startTime": 1728830433.867, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 40.5, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 40.5, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 2.38, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "6.04/10", "avgSpeed": 9.06, "belowSpeed": 0.4, "expectSpeed": 15}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0.6, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.07"}, {"time": 1.5, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.19"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0.6, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.07"}, {"time": 0.6, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.07"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40], "y": [10, 10, 10, 7.44, 6.15, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40], "y": [9.45, 15.68, -0.97, -7.79, -14.65, -1.78, -12.88, -9.58, -6.6, -8.25, -8.83, -9.15, -10.45, -7.72, -0.85, 5.57, 12.45, 6.46, 1.13, -2.06]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40], "y": [0, 0, 0, -0.93, 0, 0.69, 0.93, 0, -0.11, 0, 0.93, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40], "y": [0.17, 0.27, 0.02, 0.09, 0.2, 0.01, 0.14, 0.12, 0.05, 0.07, 0.08, 0.09, 0.1, 0.08, 0.01, 0.08, 0.17, 0.09, 0.02, 0.03]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40], "y": [0, 0, 0, 3.09, 0, -0.58, 0, -2.32, -0.01, 0.36, 3.09, -2.31, 0, 0, 0, 0, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40], "y": [0.21, 0.03, -0.01, -0.06, 0.08, 0.01, 0.11, -0.23, 0.01, -4.34, 0.06, 0.06, 0.03, -0.14, -0.01, 0.06, 0.12, -0.13, -0.06, 0.03]}}, "eventTable": {}, "senseScore": 95.63, "senseWeight": 100, "sceneNameList": "10-环岛汇入汇出", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 86.8, "safeWeight": 0.4, "comfortable": 91.85, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 26.04, "comfortable*weight": 4.59, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 85.23}], "penaltyPoint": {"PET": 0.9851851875399366, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 40.5, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0.014814824233820408, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 1, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0.014814812460063416, "overLateralJerk": 0.03703703115015854, "avgSpeedNotScore": 9.06, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 1, "overLateralJerkNum": 1, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0.014814812460063416, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0.5999999046325684, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 40.5, "allSenseScore": 95.63, "gradeTableDict": {"安全性": "优秀", "效率性": "良好", "舒适性": "良好", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 86.8, "safeWeight": 0.4, "comfortable": 91.85, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 26.04, "comfortable*weight": 4.59, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1603": [{"1": {"code": "other", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728830339.801, "diagnose": {"建议": "从安全性方面，安全性很好，无提升建议。 从舒适性方面，舒适性好，无提升建议。 从效率性方面，建议提升控制算法，提升行驶效率。 从交通协调性方面，交通协调性好，无提升建议。 从交规符合性方面，交规符合性好，无提升建议。 ", "安全性方面": "安全性好，不存在违规行为。", "舒适性方面": "舒适性好，不存在急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时长，效率较低。", "交规符合性方面": "交规符合性好，不存在较多违规行为。", "交通协调性方面": "交通协调性好，不存在影响其他车辆通行的行为。"}, "sceneName": "7-施工区避撞", "startTime": 1728830294.101, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未有效泊车", "score": "0"}], "efficiency": [{"time": 45.7, "unit": "s", "index": "任务完成", "score": "10/10"}, {"time": 45.7, "unit": "s", "index": "任务耗时", "score": "0.0/0.0", "overTime": 2.81, "expectTime": 12}, {"unit": "km/h", "index": "任务耗时", "score": "7.56/10", "avgSpeed": 18.91, "belowSpeed": 0.24, "expectSpeed": 25}, {"time": 0, "unit": "s", "index": "倒车", "score": "10/10"}, {"time": 0, "unit": "次", "index": "倒车", "score": "10/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 1, "chartData": {"TTC": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]}, "turnA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [-0.09, 0.05, 0, 0, -0, -0, 0, -0, 0, -0, 0, -0, 0, -0.95, -5.57, -0.25, 3.83, 1.94, -0.17, -0, 0, -0]}, "verticalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2.08, 0, 0, 0.92, 0, 0, 0, 0]}, "horizontalA": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.03, 0.13, 0, 0.06, 0.04, 0, 0, 0, 0]}, "verticalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -5.21, 0, 0, 3.07, 0, 0, 0, 0]}, "horizontalAPlus": {"x": [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44], "y": [0, 0, 0, -0, 0, 0, 0, -0, 0, 0, 0, -0, -0, 0.04, -0.07, -0.06, 0.02, -0.07, 0, -0, -0, 0]}}, "eventTable": {}, "senseScore": 97.56, "senseWeight": 100, "sceneNameList": "7-施工区避撞", "senseAggScore": 100, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 91.88, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 27.56, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "missionAccomplish": 1, "senseScore*senseWeight": 87.56}], "penaltyPoint": {"PET": 0.9846608320684236, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 45.70000004768371, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 18.91, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 0, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 1, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 45.7, "allSenseScore": 97.56, "gradeTableDict": {"安全性": "优秀", "效率性": "优秀", "舒适性": "优秀", "交规符合性": "优秀", "交通协调性": "优秀"}, "AbilityDimension": {"safe": 100, "compliance": 100, "efficiency": 91.88, "safeWeight": 0.4, "comfortable": 100, "safe*weight": 40, "coordination": 100, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 10, "efficiency*weight": 27.56, "comfortable*weight": 5, "coordinationWeight": 0.15, "coordination*weight": 15}, "dangerTimeProportion": 0}}], "1605": [{"1": {"code": "11-垂直车位自动泊车", "avName": "1", "caseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "taskID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "endTime": 1728832061, "diagnose": {"建议": "从安全性方面，有必要提升控制规划算法，提升交通安全。 从舒适性方面，有必要提升控制算法和驾驶策略，平缓加速、减速和转向等行为，提升行驶效率。 从效率性方面，有必要提升控制算法，提升行驶效率。 从交通协调性方面，有必要提升控制算法和驾驶策略，提升协作效率。 从交规符合性方面，有必要提升控制规划算法，减少违规次数。 ", "安全性方面": "安全性差，存在较严重违规行为，不能保证行车安全。", "舒适性方面": "舒适性差，存在较多急加速/急减速/急转向等行为。", "交互决策方面": "任务耗时较长、效率较低，或任务未完成。", "交规符合性方面": "交规符合性差，存在较多违规行为。", "交通协调性方面": "交通协调性差，存在较多影响其他车辆通行的行为。"}, "sceneName": "11-垂直车位自动泊车", "startTime": 1728832061, "testScene": [{"info": {"safe": [{"time": 0, "unit": "次", "index": "碰撞", "score": "0.0"}, {"time": 0, "unit": "次", "index": "安全员介入", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶出行车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "TTC", "score": "0.0", "timeRatio": 0, "timeRatio：": 0}, {"time": 0, "unit": "次", "index": "TTC", "score": "0.0"}, {"time": 0, "unit": "s", "index": "横向间距", "score": "0.0"}], "compliance": [{"time": 0, "unit": "s", "index": "压实线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "压虚线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "超速", "score": "0.0"}, {"time": 0, "unit": "次", "index": "闯红灯", "score": "0.0"}, {"time": 0, "unit": "次", "index": "未遵守停车标志", "score": "0.0"}, {"time": 0, "unit": "s", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "次", "index": "驶入对向车道", "score": "0.0"}, {"time": 0, "unit": "s", "index": "禁行区行驶", "score": "0.0"}, {"time": 0, "unit": "s", "index": "停车压停止线", "score": "0.0"}, {"time": 0, "unit": "s", "index": "未按规定车道行驶", "score": "0.0"}, {"time": 1, "unit": "次", "index": "未有效泊车", "score": "100"}], "efficiency": [{"time": 0, "unit": "s", "index": "任务完成", "score": "0/10"}, {"time": 0, "unit": "s", "index": "任务耗时", "score": "0/0", "overTime": -1, "expectTime": 9999}, {"unit": "km/h", "index": "任务耗时", "score": "0/10", "avgSpeed": 0, "belowSpeed": 0, "expectSpeed": 30}, {"time": 0, "unit": "s", "index": "倒车", "score": "0.0"}, {"time": 0, "unit": "次", "index": "倒车", "score": "0/10"}], "comfortable": [{"time": 0, "type": "a", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "横向舒适度", "score": "0.0"}, {"time": 0, "type": "a", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "j", "unit": "s", "index": "纵向舒适度", "score": "0.0"}, {"time": 0, "type": "c", "unit": "s", "index": "转弯舒适度", "score": "0.0"}], "coordination": [{"time": 0, "unit": "s", "index": "干扰背景车辆通行", "score": "0", "maxDisturbedSvSimuTime": 0, "svSimuTimeWithoutDisturbance": 30}]}, "senceID": 0, "chartData": {"TTC": {"x": [2, 4], "y": [10, 10]}, "turnA": {"x": [2, 4], "y": [0.11, 0.07]}, "verticalA": {"x": [2, 4], "y": [0, 0]}, "horizontalA": {"x": [2, 4], "y": [0.01, 0]}, "verticalAPlus": {"x": [2, 4], "y": [0, 0]}, "horizontalAPlus": {"x": [2, 4], "y": [0, 0]}}, "eventTable": {"1": {"time": "2024-10-13 23:07:41", "index": "任务未完成", "place": [0, 0], "simuTime": 0}}, "senseScore": 0, "sceneCaseID": "3e6b4be5-8971-11ef-b5fb-00163e21f0fc", "senseWeight": 25, "sceneNameList": "11-垂直车位自动泊车", "senseAggScore": 25, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "missionAccomplish": 0, "senseScore*senseWeight": 0}], "penaltyPoint": {"PET": 0, "TTC": 0, "crash": 0, "TTCNum": 0, "useTime": 47.13599991798401, "avgSpeed": 0, "crashNum": 0, "overAcce": 0, "overJerk": 0, "timeUsed": 0, "gapRefuse": 0, "outOfLane": 0, "overSpeed": 0, "eventTable": {"1": {"time": "2024-10-13 23:07:41", "index": "任务未完成", "place": [0, 0], "simuTime": 0}}, "reverseCar": 0, "TTCAccuTime": 0, "breakSignal": 0, "overAcceNum": 0, "overJerkNum": 0, "coordination": 0, "outOfLaneNum": 0, "overSpeedNum": 0, "inSubtendRoad": 0, "onLaneMarking": 0, "reverseCarNum": 0, "breakSignalNum": 0, "stopAtStopLine": 0, "inForbiddenArea": 0, "overLateralAcce": 0, "overLateralJerk": 0, "avgSpeedNotScore": 10.58, "followStopSignal": 0, "inSubtendRoadNum": 0, "outOfParkingZone": 100, "isSecurityInvolved": 0, "overLateralAcceNum": 0, "overLateralJerkNum": 0, "transverseDistance": 0, "followStopSignalNum": 0, "onDottedLaneMarking": 0, "calMissionAccomplish": 0, "minCoordinationScore": 0, "unstableSteeringAngle": 0, "maxDisturbedSvSimuTime": 0, "drivingInDesignatedLane": 0, "unstableSteeringAngleTime": 0, "svSimuTimeWithoutDisturbance": 30}, "testDuration": 0, "allSenseScore": 0, "gradeTableDict": {"安全性": "不合格", "效率性": "不合格", "舒适性": "不合格", "交规符合性": "不合格", "交通协调性": "不合格"}, "AbilityDimension": {"safe": 0, "compliance": 0, "efficiency": 0, "safeWeight": 0.4, "comfortable": 0, "safe*weight": 0, "coordination": 0, "complianceWeight": 0.1, "efficiencyWeight": 0.3, "comfortableWeight": 0.05, "compliance*weight": 0, "efficiency*weight": 0, "comfortable*weight": 0, "coordinationWeight": 0.15, "coordination*weight": 0}, "dangerTimeProportion": 0}}]}}"""
    vehi_regional_score = json.loads(vehi_regional_score)

    regionals = {1: {'weights': '1', 'position': {'minX': -645.5327689647672, 'maxX': -588.3589088916776, 'minY': -94.73575651645658, 'maxY': -39.55633342266081}}, 2: {'weights': '1', 'position': {'minX': -773.5091418027876, 'maxX': -690.7400071620939, 'minY': -212.73994445800776, 'maxY': -156.56330287456507}}, 3: {'weights': '1', 'position': {'minX': -709.0223461389539, 'maxX': -638.8846457004545, 'minY': -25.927680730819695, 'maxY': 29.251742362976064}}}
    projectWeight = {'EvaluationSafeWeights': [{'id': 1, 'weights': 777, 'indicatorName': '是否碰撞', 'calculationFormula': '碰撞-100', 'indicatorDescription': '判断主车与障碍物是否发生碰撞，若是，则不通过。'}, {'id': 2, 'weights': 10, 'indicatorName': '碰撞时间TTC', 'calculationFormula': 'TTC危险时长/任务耗时*100%', 'indicatorDescription': '计算主车的碰撞时间， 若TTC<1s， 则认为存在碰撞风险， 累计违规时长。'}, {'id': 3, 'weights': 10, 'indicatorName': '逆向行驶', 'calculationFormula': '逆向行驶时长/任务耗时*100%', 'indicatorDescription': '检测主车是否出现逆向行驶的情况，若是，则累计违规时长。'}, {'id': 4, 'weights': 10, 'indicatorName': '压实线行驶', 'calculationFormula': '压实线行驶时长/任务耗时*100%', 'indicatorDescription': '检测主车行驶过程中是否压实线，若是，则累计违规时长。'}, {'id': 5, 'weights': 10, 'indicatorName': '超速行驶', 'calculationFormula': '超速行驶时长/任务耗时*100%', 'indicatorDescription': '检测主车行驶速度是否超过地图上的道路限速，若是，则累计违规时长。'}, {'id': 6, 'weights': 10, 'indicatorName': '四轮在路', 'calculationFormula': '四轮未在路行驶时长/任务耗时*100%', 'indicatorDescription': '判断主车四轮是否全程都在道路内，若不是，则累计违规时长。'}, {'id': 7, 'weights': 40, 'indicatorName': '横向间距', 'calculationFormula': '危险横向间距行驶时长/任务耗时*100%', 'indicatorDescription': '计算主车与周围车的横向车间距，若小于1米，则认为存在碰撞风险，累计违规时长。'}, {'id': 8, 'weights': 555, 'indicatorName': '在禁行区禁行', 'calculationFormula': '禁行区行驶-100', 'indicatorDescription': '检测主车年是否在应急车道、非机动车道、公交专用车道等禁行区域行驶，若是，则不通过。'}], 'EvaluationClassWeights': [{'id': 1, 'name': '安全', 'weights': '40'}, {'id': 2, 'name': '效率', 'weights': '59'}, {'id': 3, 'name': '舒适', 'weights': '1'}], 'EvaluationComfortWeights': [{'id': 1, 'weights': 10, 'indicatorName': '横向加速度', 'calculationFormula': '横向加速度超出阈值时长/任务耗时*100%', 'indicatorDescription': '检测规划轨迹点的横向加速度是否在合理上下限范围内[-4,4]m/s2，若不是，则累计违规时长。'}, {'id': 2, 'weights': 10, 'indicatorName': '横向急动度', 'calculationFormula': '横向急动度超出阈值时长/任务耗时*100%', 'indicatorDescription': '检测规划轨迹点的横向加速度变化率是否在合理上下限范围内[-4,4]m/s3，若不是，则累计违规时长。'}, {'id': 3, 'weights': 10, 'indicatorName': '纵向加速度', 'calculationFormula': '纵向加速度超出阈值时长/任务耗时*100%', 'indicatorDescription': '检测规划轨迹点的纵向加速度是否在合理上下限范围内[-4,4]m/s2，若不是，则累计违规时长。'}, {'id': 4, 'weights': 10, 'indicatorName': '纵向急动度', 'calculationFormula': '纵向急动度超出阈值时长/任务耗时*100%', 'indicatorDescription': '检测轨迹规划点的纵向加速度变化率是否在合理上下限范围内[-4,4]m/s3，若不是，则累计违规时长。'}, {'id': 5, 'weights': 888, 'indicatorName': '角速度', 'calculationFormula': '航向角变化超出阈值时长/任务耗时*100%', 'indicatorDescription': '检测航向角是否反复变化，若航向角频繁变化，则累计违规时长。'}], 'EvaluationEfficiencyWeights': [{'id': 1, 'weights': 111, 'indicatorName': '任务完成时间', 'calculationFormula': '期望时间/实际用时*100%', 'indicatorDescription': '按照超出期望用时的比例进行扣分，在期望用时内完成任务为满分。'}, {'id': 2, 'weights': 222, 'indicatorName': '平均速度', 'calculationFormula': '行驶里程/行驶时间', 'indicatorDescription': '计算主车在行驶全程中的平均速度。'}]}
    taskID = 1
    missonStartTime = '13:20'
    missonEndTime = '13:20'
    vehID = '1'

    change_score = {
        '1549': [
            {
                '1': [
                    {
                        "time": -10,
                        "unit": "次",
                        "index": "倒车",
                        "score": "0"
                    },
                    {
                        "time": 0,
                        "unit": "次",
                        "index": "碰撞",
                        "score": "0"
                    },
                    {
                        "time": -26,
                        "unit": "次",
                        "index": "TTC",
                        "score": "0"
                    },
                    {
                        "time": 200,
                        "unit": "s",
                        "index": "任务完成",
                        "score": "0"
                    },
                    # {
                    #     "time": 3,
                    #     "unit": "次",
                    #     "index": "未有效泊车",
                    #     "score": "0"
                    # },
                    # {
                    #     "time": 200,
                    #     "unit": "s",
                    #     "index": "纵向舒适度",
                    #     'type': 'j',
                    #     "score": "0"
                    # }
                ]
            }
        ],
    }

    sqlScore, c, d = reCalScore(change_score, vehi_regional_score, regionals, projectWeight, taskID, missonStartTime, missonEndTime, vehID)

    change_score2 = {
       '1520': [
           {
               '1': [
                   {
                       "time": -6,
                       "unit": "次",
                       "index": "TTC",
                       "score": "0"
                   },
                   {
                       "time": 2,
                       "unit": "次",
                       "index": "碰撞",
                       "score": "0"
                   },
                   {
                       "time": 6,
                       "unit": "次",
                       "index": "超速",
                       "score": "0"
                   },
                   {
                       "time": 7,
                       "unit": "次",
                       "index": "驶入对向车道",
                       "score": "0"
                   },
                   {
                       "time": 123,
                       "unit": "次",
                       "index": "未遵守停车标志",
                       "score": "0"
                   },
                   # {
                   #     "time": 2,
                   #     "unit": "s",
                   #     "index": "横向舒适度",
                   #     'type': 'a',
                   #     "score": "0"
                   # }
               ]
           }
       ],
    }

    # sqlScore2, a, b = reCalScore(change_score2, sqlScore, regionals, projectWeight, taskID, missonStartTime, missonEndTime, vehID)

    # totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar = startScoreProcessWithCrashCheck(sqlScore2, regionals, projectWeight, taskID, missonStartTime, missonEndTime, vehID)
    totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar = startScoreProcess(sqlScore, regionals, projectWeight, taskID, missonStartTime, missonEndTime, vehID)

    print(vehi_regional_score)
    # print(regionals)
    # print(projectWeight)
    print("总：", totalResultDataDictSplitByCar)
    print("分：", sceneResultDataDictSplitByCar)


