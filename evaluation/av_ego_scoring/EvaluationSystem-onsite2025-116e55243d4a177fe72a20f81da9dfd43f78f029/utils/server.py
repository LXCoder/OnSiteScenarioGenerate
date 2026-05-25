import collections
import copy
import datetime
import json
import logging
import os
import threading
import time

from flask import Response, jsonify, Flask, request, current_app
from flask_cors import CORS
from utils.SQLDB import MySQLDB

# 确保响应的内容以JSON格式返回
from utils.config import WebUrl, LOG_FILE_DIR, log_name
from utils.functions import supplement_scene_score, calc_car_regional, calc_main_task
from utils.log import setup_log
from utils.scoreProcess import startScoreProcess, startScoreProcessWithCrashCheck, reCalScore
from utils.server_config import PROJECTNAME, HTTPPORT, HTTPHOST


logger = logging.getLogger(log_name)

class JSONResponse(Response):
    default_mimetype = 'application/json'

    @classmethod
    def force_type(cls, response, environ=None):
        if isinstance(response, dict):
            response = jsonify(response)
        return super(JSONResponse, cls).force_type(response, environ)


app = Flask(__name__)
app.response_class = JSONResponse
CORS(app)


@app.errorhandler(Exception)
def exception_handler(error):
    app.logger.exception(f'程序出错')
    message = "出现意外错误，请联系开发人员！"

    return {
        "message": message,
        'status': 500,
    }


# 日志记录请求信息
@app.before_request
def before_app_request():
    app.logger.info(f'request url: {request.url}, data: {request.data}')


# 日志记录响应信息
@app.after_request
def after_app_request(response):
    app.logger.info(f"response status: {response.status}, status code: {response.status_code}")
    return response


# 获取所有的仿真记录
@app.route(f"/{PROJECTNAME}/evaluation/list/", methods=['GET'])
def get_evaluation_list():
    evaluation_id = request.args.get('taskId')
    parms = []
    if evaluation_id:
        sql = "select id, createTime, status, mainVehiId from Evaluation where isDeleted=0 and id=%s"
        parms.append(evaluation_id)
    else:
        sql = "select id, createTime, status, mainVehiId from Evaluation where isDeleted=0 order by createTime desc limit 100"
    is_ok, evaluation_records = MySQLDB.execute_sql(sql, tuple(parms))
    for record in evaluation_records:
        record['mainVehiId'] = [int(i) for i in record['mainVehiId'].split(',') if i]
    return {
        "message": 'ok',
        "status": 200,
        "data": evaluation_records,
    }


# 获取某仿真的某车测试得分
@app.route(f"/{PROJECTNAME}/taskReport/", methods=['GET'])
def get_evaluation_taskReport():
    evaluation_id = request.args['taskId']
    vehi_id = request.args['carName']

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select eva.score score, eva.createTime createTime, net.regionalWeight regionalWeight, net.projectWeight projectWeight from Evaluation eva join Network net on eva.netId=net.id where eva.id=%s and eva.isDeleted=0 and net.isDeleted=0"
    is_ok, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    result = {}
    if evaluation_record and evaluation_record[0]['score']:
        evaluation_record = evaluation_record[0]
        missonStartTime = f"{evaluation_record['createTime'].hour}: {evaluation_record['createTime'].minute}"
        missonEndTime = f"{evaluation_record['createTime'].hour}: {evaluation_record['createTime'].minute}"
        # 样例数据
        totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar = startScoreProcess(
            json.loads(evaluation_record['score']), json.loads(evaluation_record['regionalWeight']),
            json.loads(evaluation_record['projectWeight']), evaluation_id, missonStartTime, missonEndTime, str(vehi_id))

        gradeTableDict = collections.defaultdict(list)
        for name, value in sceneResultDataDictSplitByCar.items():
            scene_id, index = name.split("_")
            gradeTableDict[value['sceneName']].append(value['gradeTableDict'])
        result = {
            **totalResultDataDictSplitByCar,
            "gradeTableDict": gradeTableDict,
        }

    return {
        "message": 'ok',
        "status": 200,
        "data": result,
    }


# 获取小车的场景数据
@app.route(f"/{PROJECTNAME}/allCutRoad/", methods=['GET'])
def get_evaluation_allCarScene():
    evaluation_id = request.args['taskId']
    car_id = request.args.get('carName')

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select mainVehiId, score from Evaluation where id=%s and isDeleted=0"
    is_ok, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    result = []
    if evaluation_record:
        evaluation_record = evaluation_record[0]
        score = json.loads(evaluation_record['score'])
        for vehi_id in filter(lambda x: bool(x), evaluation_record['mainVehiId'].split(",")):
            if car_id and vehi_id != car_id:
                continue

            car_score = score.get(vehi_id, {})
            for scene_id, scene_scores in car_score.items():
                result.append(
                    {
                        "name": str(scene_id),
                        "num": len(scene_scores),
                        "id": str(scene_id),
                    }
                )
            pass

    return {
        "message": 'ok',
        "status": 200,
        "data": result,
    }


# 获取测试中的所有小车
@app.route(f"/{PROJECTNAME}/allCar/", methods=['GET'])
def get_evaluation_allCar():
    evaluation_id = request.args['taskID']

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select mainVehiId, score from Evaluation where id=%s and isDeleted=0"
    is_ok, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    result = []
    if evaluation_record:
        evaluation_record = evaluation_record[0]
        score = json.loads(evaluation_record['score'])
        for vehi_id in filter(lambda x: bool(x), evaluation_record['mainVehiId'].split(",")):
            count = 0
            car_score = score.get(vehi_id, {})
            for scene_id, scene_scores in car_score.items():
                count += len(scene_scores)
            result.append(
                {
                    "name": str(vehi_id),
                    "num": count,
                }
            )

    return {
        "message": 'ok',
        "status": 200,
        "data": result,
    }


# 获取测试的所有得分结果
@app.route(f"/{PROJECTNAME}/getList/", methods=['GET'])
def get_evaluation_getList():
    evaluation_id = request.args['taskId']
    # 根据车辆过滤
    car_id = request.args.get('carName')
    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select eva.score score, eva.mainVehiId mainVehiId, eva.createTime createTime, net.regionalWeight regionalWeight, net.projectWeight projectWeight from Evaluation eva join Network net on eva.netId=net.id where eva.id=%s and eva.isDeleted=0 and net.isDeleted=0"
    is_ok, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    result = []
    if evaluation_record:
        # 虽然是 getList，但是在这里，最多也只会有一条记录
        evaluation_record = evaluation_record[0]
        # regions = json.loads(evaluation_record.get('regionalWeight') or "{}").get('EvaluationArea', [])
        # scene_name_mapping = {str(region['id']): str(region['name']) for region in regions}

        evaluation_score = json.loads(evaluation_record['score'])
        for vehi_id in filter(lambda x: bool(x), evaluation_record['mainVehiId'].split(",")):
            if car_id is not None and car_id != vehi_id:
                continue

            car_score = evaluation_score.get(vehi_id, {})
            for scene_id, scene_scores in car_score.items():
                for index, scene_score in enumerate(scene_scores):
                    score = scene_score.get('1', {})
                    result.append(
                        {
                            "id": f"{evaluation_id}_{vehi_id}_{scene_id}_{index}",
                            "startTime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(int(score.get('startTime') or time.time()))),
                            "endTime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(int(score.get('endTime') or time.time()))),
                            "safety": score.get('gradeTableDict', {}).get('安全性', "不合格"),
                            "comfort": score.get('gradeTableDict', {}).get('舒适性', "不合格"),
                            "efficiency": score.get('gradeTableDict', {}).get('效率性', "不合格"),
                            "compliance": score.get('gradeTableDict', {}).get('交规符合性', "不合格"),
                            "coordination": score.get('gradeTableDict', {}).get('交通协调性', "不合格"),
                            "testDuration": score['testDuration'],
                            "cutName": score['sceneName'],
                            # "cutName": scene_name_mapping.get(str(scene_id), ''),
                            "case_id": scene_id,
                            "scene_id": str(scene_id),
                            "recordId": evaluation_id,
                            "senseScore": score['allSenseScore'],
                        }
                    )
                    # break  # 每个场景只取一条数据

    return {
        "message": 'ok',
        "status": 200,
        "data": {
            'count': len(result),
            'list': result,
        },
    }


# 获取某车某场景某次的得分详情
@app.route(f"/{PROJECTNAME}/getReportDetail/", methods=['GET'])
def get_evaluation_getReportDetail():
    score_id = request.args['id']
    evaluation_id, vehi_id, scene_id, index = score_id.split('_')

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select mainVehiId, score from Evaluation where id=%s and isDeleted=0 and status='end'"
    is_ok, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    result = {}
    if evaluation_record:
        evaluation_score = json.loads(evaluation_record[0]['score'])
        scene_scores = evaluation_score.get(vehi_id, {}).get(scene_id, [])
        if len(scene_scores) > int(index):
            result = scene_scores[int(index)].get('1')
            message = '查询成功'
        else:
            message = '车辆在指定场景索引下无分数'
    else:
        message = '未查询到有效记录，请稍后重试'
    return {
        "message": message,
        "status": 200,
        "data": result,
    }

# 重构某车得分
@app.route(f"/{PROJECTNAME}/score/update/", methods=['POST'])
def update_evaluation_score():
    request_data = request.json['data']
    evaluation_id, vehi_id, change_score = request_data['taskID'], request_data['name'], request_data['detail']

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select eva.score score, eva.createTime createTime, eva.other other, net.regionalWeight regionalWeight, net.projectWeight projectWeight from Evaluation eva join Network net on eva.netId=net.id where eva.id=%s and eva.isDeleted=0 and net.isDeleted=0"
    _, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    is_ok = False
    if evaluation_record and evaluation_record[0]['score']:
        evaluation_record = evaluation_record[0]
        timestamp_list = []
        car_scores = json.loads(evaluation_record['score']).get(str(vehi_id), {})
        for car_sence_scores in car_scores.values():
            for car_sence_score in car_sence_scores:
                timestamp_list.append(car_sence_score['1']['startTime'])
                timestamp_list.append(car_sence_score['1']['endTime'])

        timestamp_list = timestamp_list or [time.time()]
        missonStartTime = time.strftime("%Y-%m-%d %H:%M", time.localtime(min(timestamp_list)))
        missonEndTime = time.strftime("%Y-%m-%d %H:%M", time.localtime(max(timestamp_list)))

        newScore, _, _ = reCalScore(change_score, json.loads(evaluation_record['score']),
                                    json.loads(evaluation_record['regionalWeight']), json.loads(evaluation_record['projectWeight']), evaluation_id,
                                    missonStartTime, missonEndTime, str(vehi_id))

        other_info = json.loads(evaluation_record['other'] or "{}")
        if other_info.get('originScore', {}).get('score') is None:
            origin_score = json.loads(evaluation_record['score'])
        else:
            origin_score = other_info.get('originScore', {}).get('score')

        sql = "update Evaluation set score=%s, other=%s where id=%s"
        is_ok, _ = MySQLDB.execute_sql(sql, (
            json.dumps(newScore),
            json.dumps({
                **other_info,
                "originScore": {
                    'score': origin_score,
                    'updateTime': str(datetime.datetime.now())
                }
            }),
            evaluation_id
        ))
    return {
        "message": 'ok',
        "status": 200,
        "data": {"update": is_ok},
    }

# 恢复某车得分为原始分数
@app.route(f"/{PROJECTNAME}/score/reset/", methods=['POST'])
def reset_evaluation_score():
    score_id = request.json['id']
    evaluation_id, vehi_id, sence_id, index = score_id.split('_')
    index = int(index)

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select eva.score score, eva.createTime createTime, eva.other other, net.regionalWeight regionalWeight, net.projectWeight projectWeight from Evaluation eva join Network net on eva.netId=net.id where eva.id=%s and eva.isDeleted=0 and net.isDeleted=0"
    _, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    is_ok = False
    if evaluation_record and evaluation_record[0]['score']:
        evaluation_record = evaluation_record[0]

        # 获取原始分数
        other_info = json.loads(evaluation_record['other'] or "{}")
        if other_info.get('originScore', {}).get('score') is None:
            origin_score = json.loads(evaluation_record['score'])
        else:
            origin_score = other_info.get('originScore', {}).get('score')

        # 获取此车当前场景当前批次的原始得分
        try:
            car_sence_index_origin_score = origin_score.get(vehi_id, {}).get(sence_id, [])[index]
        except:
            car_sence_index_origin_score = {}

        # 获取当前分数
        now_score = json.loads(evaluation_record['score'])
        # 将原始得分注入到分数中
        try:
            now_score[vehi_id][sence_id][index] = car_sence_index_origin_score
        except:
            print("更新失败")
            pass

        sql = "update Evaluation set score=%s, other=%s where id=%s"
        is_ok, _ = MySQLDB.execute_sql(sql, (
            json.dumps(now_score),
            json.dumps({
                **other_info,
                "originScore": {
                    'score': origin_score,
                    'updateTime': str(datetime.datetime.now())
                }
            }),
            evaluation_id
        ))
        is_ok = True
    return {
        "message": 'ok',
        "status": 200,
        "data": {"update": is_ok},
    }


# 将mysql得分转换为redis需要的分数
@app.route(f"/{PROJECTNAME}/redisData/", methods=['GET'])
def get_evaluation_redisData():
    evaluation_id, vehi_id, sence_id, index, data_type = request.args['taskID'], request.args[
        'carName'], request.args.get('senceId'), request.args.get('index'), request.args['type']

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select eva.score score, eva.status status, eva.createTime createTime, net.regionalWeight regionalWeight, net.projectWeight projectWeight, eva.other other from Evaluation eva join Network net on eva.netId=net.id where eva.id=%s and eva.isDeleted=0 and net.isDeleted=0"
    is_ok, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    message = "ok"
    result = {}
    other = {}
    if evaluation_record and evaluation_record[0]['score']:
        evaluation_record = evaluation_record[0]
        other = evaluation_record['other']

        timestamp_list = []
        car_scores = json.loads(evaluation_record['score']).get(str(vehi_id), {})
        for car_sence_scores in car_scores.values():
            for car_sence_score in car_sence_scores:
                timestamp_list.append(car_sence_score['1']['startTime'])
                timestamp_list.append(car_sence_score['1']['endTime'])

        timestamp_list = timestamp_list or [time.time()]
        missonStartTime = time.strftime("%Y-%m-%d %H:%M", time.localtime(min(timestamp_list)))
        missonEndTime = time.strftime("%Y-%m-%d %H:%M", time.localtime(max(timestamp_list)))

        # 样例数据
        totalResultDataDictSplitByCar, sceneResultDataDictSplitByCar = startScoreProcess(
            json.loads(evaluation_record['score']), json.loads(evaluation_record['regionalWeight']),
            json.loads(evaluation_record['projectWeight']), evaluation_id, missonStartTime, missonEndTime, str(vehi_id))
        if data_type == "total":
            result = totalResultDataDictSplitByCar
        elif data_type == 'sence':
            result = sceneResultDataDictSplitByCar.get(f"{sence_id}_{index}", {})

        if evaluation_record['status'] != "done":
            message = "评价未完成"

    return {
        "message": message,
        "status": 200,
        "data": result,
        'other': other
    }

# 查询数据库记录
@app.route(f"/{PROJECTNAME}/sqlData/", methods=['GET'])
def get_evaluation_sqlData():
    evaluation_id, vehi_id, sence_id, index, data_type = request.args['taskID'], request.args.get('carName'), request.args.get('senceId'), request.args.get('index'), request.args.get('type')

    # 从数据库获取计算测试得分此次所需的所有数据
    sql = "select eva.* from Evaluation eva join Network net on eva.netId=net.id where eva.id=%s and eva.isDeleted=0 and net.isDeleted=0"
    is_ok, evaluation_record = MySQLDB.execute_sql(sql, (evaluation_id,))

    result = evaluation_record and evaluation_record[0]
    return {
        "message": 'ok',
        "status": 200,
        "data": result,
    }

# 新建路网信息表
@app.route(f"/{PROJECTNAME}/network/create/", methods=['POST'])
def create_network_record():
    network_data = request.json['data']

    uuid = MySQLDB.get_new_uuid()
    network_data = {
        "id": uuid,
        "name": network_data.get('name') or "",
        "regionalWeight": json.dumps(network_data['regionalWeight']),
        "projectWeight": json.dumps(network_data['projectWeight']),
    }
    is_ok = MySQLDB.insert_one_with_dict("Network", network_data)

    if is_ok:
        return {
            "message": '路网创建成功',
            "status": 200,
            "networkId": uuid,
        }
    else:
        return {
            "message": '路网创建失败',
            "status": 400,
        }

# 新建测试记录
@app.route(f"/{PROJECTNAME}/evaluation/create/", methods=['POST'])
def create_evaluation_record():
    request_data = request.json['data']

    uuid = MySQLDB.get_new_uuid()
    evaluation_data = {
        "id": uuid,
        "netId": request_data['netId'],
        "mainVehiId": request_data['mainVehiId'],
        "jsonDataPath": request_data.get('jsonDataPath'),
        "isChildren": request_data.get('isChildren'),
        'filterType': request_data.get('filterType', 'regional')
    }
    if "kafkaTopic" in request_data:
        evaluation_data['kafkaTopic'] = json.dumps(request_data['kafkaTopic'])

    is_ok = MySQLDB.insert_one_with_dict("Evaluation", evaluation_data)

    if is_ok:
        return {
            "message": '评价记录创建成功',
            "status": 200,
            "taskId": uuid,
            "evaluationUrl": f"{WebUrl}{uuid}",
        }
    else:
        return {
            "message": '评价记录创建失败',
            "status": 400,
        }

# 新建主测试任务, 聚合多个子任务
@app.route(f"/{PROJECTNAME}/evaluation/mainTask/create/", methods=['POST'])
def create_evaluation_record_main_task_create():
    request_data = request.json['data']
    children_ids = request_data['childrenIds']
    if not isinstance(children_ids, list):
        app.logger.error(f"子任务ID列表异常: {children_ids}")
        return {
            "message": '缺少子任务ID列表',
            "status": 200
        }

    sql = "SELECT * from Evaluation where id in %s"
    is_ok, children_evaluation_records = MySQLDB.execute_sql(sql, (tuple(children_ids),))
    if is_ok and children_evaluation_records:
        uuid = MySQLDB.get_new_uuid()
        evaluation_data = {
            "id": uuid,
            "status": "end",
            "mainVehiId": children_evaluation_records[0]["mainVehiId"],
            "netId": children_evaluation_records[0]["netId"],
            "filterType": None,
            "other": json.dumps({"childrenIds": children_ids})
        }
        is_ok = MySQLDB.insert_one_with_dict("Evaluation", evaluation_data)

        if is_ok:
            # TODO 自动返回插入id，子任务只生成单场景，主任务拼接没有的场景,通过文件获取轨迹
            # 启动子线程，等待聚合
            app.logger.warning(f"启动子线程，等待聚合: {uuid}")
            threading.Thread(target=calc_main_task, args=(uuid, children_ids)).start()
            return {
                "message": '评价记录创建成功',
                "status": 200,
                "taskId": uuid,
                "evaluationUrl": f"{WebUrl}{uuid}",
            }
        else:
            return {
                "message": '主任务创建失败',
                "status": 400,
            }
    else:
        return {
            "message": '未找到有效的子任务, 主任务创建失败',
            "status": 400,
        }

def api_server():
    logger = setup_log(os.path.join(LOG_FILE_DIR, 'server'), log_name, when="MIDNIGHT")
    app.logger = logger
    logger.info('启动程序')
    app.run(debug=False, host=HTTPHOST, port=HTTPPORT)


if __name__ == '__main__':
    api_server()
