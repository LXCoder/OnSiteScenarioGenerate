import collections
import copy
import hashlib
import json
import logging
import multiprocessing
import os
import psutil
import signal
import subprocess
import time
import pandas as pd
import requests
import csv
from queue import Queue
from kafka import KafkaConsumer, TopicPartition
from evaluateUtils.start_evaluate_utils import evaluateStarter
from utils.SQLDB import MySQLDB
from utils.config import TMPPATH, split_time_interval, log_name, time_threshold
from evaluateUtils.standard_parameter import Parameter
from utils.others import makeZeroData
from utils.calParkingIndex import startCalParkingIndex


logger = logging.getLogger(log_name)


def remove_dir(dir_path):
    """
        删除文件夹
    """
    result = False
    if os.name == "nt":
        # Windows
        if not subprocess.call(['rd', '/s', '/q', dir_path], shell=True):
            # subprocess.call 正确执行时，会返回0，否则返回错误码
            result = True
    elif os.name == "posix":
        # Linux或Mac OS
        # if not subprocess.call(['rm', '-rf', dir_path], shell=True):  # Linux/macOS
        if not subprocess.call([f"find {dir_path} | xargs rm -rf"], shell=True):  # Linux/macOS
            result = True
    return result


def calc_main_task(task_id, children_ids=None):
    logger.info(f"主任务等待计算")
    # 等待子任务计算结束，用以生成主任务数据
    if not children_ids:
        # 未提供子任务ID列表，主动检索
        sql = "SELECT * from Evaluation where id = %s"
        is_ok, evaluation_records = MySQLDB.execute_sql(sql, tuple([task_id]))
        if is_ok and evaluation_records:
            children_ids = evaluation_records[0]['other'] and json.loads(evaluation_records[0]['other'])['childrenIds']

    children_ids = children_ids or []
    while True:
        sql = "SELECT * from Evaluation where id in %s"
        is_ok, evaluation_records = MySQLDB.execute_sql(sql, (tuple(children_ids), ))
        if is_ok and all([i['status'] == 'end' for i in evaluation_records]):
            logger.info(f"所有子任务计算完成，进行聚合")
            # 所有子任务计算结束，进行主任务聚合
            score = {}
            # children_data = [json.loads(i['score'] or "{}") for i in evaluation_records]
            for evaluation_record in evaluation_records:
                value_1 = json.loads(evaluation_record['score'] or "{}")
                for vehi_id, value_2 in value_1.items():
                    score[vehi_id] = score.get(vehi_id) or {}
                    for regional_id, value_3 in value_2.items():
                        score[vehi_id][regional_id] = score[vehi_id].get(regional_id) or []
                        for value_4 in value_3:
                            score[vehi_id][regional_id].append(copy.deepcopy(value_4))

            checkok_score(score)
            # 补齐空缺场景的得分
            main_vehi_ids, regionals, network = calc_car_regional(evaluation_records[0])
            supplement_scene_score(task_id, regionals, score, main_vehi_ids)
            checkok_score(score)

            sql = "UPDATE Evaluation set score=%s where id=%s"
            MySQLDB.execute_sql(sql, tuple([json.dumps(score), task_id]))
            break
        else:
            logger.warning(f"子任务正在计算，请等待")
            time.sleep(3)

def integration_trace(evaluation_id, main_vehi_ids, regionals, trace_data, filter_type=None):
    print(f"过滤条件: {filter_type}")
    """
        分析轨迹数据文件夹，提取轨迹数据至新的csv文件中
    """
    # 记录每辆车，在每个区域的轨迹,默认是 DataFrame，同时强制ego占据前几列
    car_regional_trace = collections.defaultdict(lambda: collections.defaultdict(lambda: pd.DataFrame(columns=['x_ego', 'y_ego', 'v_ego', 'a_ego', 'yaw_ego', 'width_ego', 'length_ego', 'isSecurityInvolved'])))

    # 初始化多辆车，多个场景的 pdData
    # 初始化数据(主车在前几列)
    print(f"len(trace_data): {len(trace_data)}")
    print(f"main_vehi_ids: {main_vehi_ids}")
    for vehi_info_data in trace_data:
        simuTime = vehi_info_data['simuTime'] / 1000
        # 遍历每一辆主车，如果主车在某区域内，就需要把这帧数据进行注入
        for main_vehi_id in main_vehi_ids:
            # 提取出这帧轨迹中的主车信息，提前提取进行判断，可以避免记录大量不需要记录的帧数据

            # 确保此帧轨迹中有主车
            main_vehi_info = None
            for vehi_info in vehi_info_data['data']:
                # print(f"数据库主车id: {main_vehi_id}, 轨迹中车辆id: {vehi_info['id']}")
                if vehi_info['id'] == main_vehi_id:
                    main_vehi_info = vehi_info
                    break
            if not main_vehi_info:
                continue

            # print(f"regionals: {regionals}")
            for regional_id, regional_value in regionals.items():
                # 如果主车中有区域ID，那需要进一步过滤，确保主车轨迹属于确定区域，且后续不需要切片
                # print(f"场景id: {regional_id}, 轨迹所属场景id: {main_vehi_info.get('regionalId')}")
                if filter_type == 'regional' and regional_id != main_vehi_info.get('regionalId'):
                    # 当通过区域过滤时，必须确保当前主车轨迹包含区域ID
                    continue
                # 遍历所有的区域，进行检查
                regional_position = regional_value['position']
                if regional_position['minX'] <= main_vehi_info['x'] <= regional_position['maxX'] and regional_position['minY'] <= main_vehi_info['y'] <= regional_position['maxY']:
                    # 如果主车在此区域内，对所有数据进行记录
                    for vehi_info in vehi_info_data['data']:
                        # if (mainVehiId and vehi_info['id'] == mainVehiId) or (not mainVehiId and vehi_info['isMain']):
                        if vehi_info['id'] == main_vehi_id:
                            symbol = 'ego'
                        else:
                            symbol = str(vehi_info['id'])
                        pdData = car_regional_trace[main_vehi_id][regional_id]
                        pdData.loc[simuTime, f'x_{symbol}'] = vehi_info['x']
                        pdData.loc[simuTime, f'y_{symbol}'] = vehi_info['y']
                        pdData.loc[simuTime, f'v_{symbol}'] = vehi_info['speed']
                        pdData.loc[simuTime, f'a_{symbol}'] = vehi_info['acce']
                        pdData.loc[simuTime, f'yaw_{symbol}'] = vehi_info['angle']
                        pdData.loc[simuTime, f'width_{symbol}'] = vehi_info['width']
                        pdData.loc[simuTime, f'length_{symbol}'] = vehi_info['length']

                        # 新增安全员介入考量
                        if symbol == 'ego':
                            pdData.loc[simuTime, f'isSecurityInvolved'] = vehi_info.get('isSecurityInvolved')
                        car_regional_trace[main_vehi_id][regional_id] = pdData

    car_regional_trace_files = collections.defaultdict(lambda: collections.defaultdict(list))

    print(f"len(car_regional_trace): {len(car_regional_trace)}")
    for main_vehi_id in car_regional_trace:
        for regional_id in car_regional_trace[main_vehi_id]:
            pdData = car_regional_trace[main_vehi_id][regional_id]
            # 去除 ego 车不存在的轨迹帧
            pdData.drop(pdData[(pdData['x_ego'].isnull() | (pdData['x_ego'].isin([''])))].index, inplace=True)
            # 根据仿真时间排序(索引)
            pdData = pdData.loc[pdData.index.sort_values()]

            if filter_type == 'time':
                # 当通过时间过滤时，需要进行时间的轨迹切片
                indexs = pdData.index.values  # 时间戳列表
                # 计算相邻两行 simuTime 的差值
                pdData['diff'] = pd.DataFrame(indexs).diff().values

                # 找出差值大于 一定时间 的位置,分时间段切割轨迹
                split_indices = pdData.index[pdData['diff'] > split_time_interval].tolist()

                # 将起始位置添加到索引中
                split_indices.insert(0, indexs[0])
                if indexs[-1] not in split_indices:
                    split_indices.append(indexs[-1])

                # 根据这些位置进行切片，并重置索引,每个场景的最后一条数据删除（最后一段本不应删除，还是不参与计算了）
                slices = [pdData[split_indices[i]:split_indices[i + 1]].reset_index(drop=True)[:-1] for i in
                          range(len(split_indices) - 1)]
            else:
                print("不通过时间过滤")
                slices = [pdData]

            # 删除辅助列 'diff' 并确保每个切片的列名唯一
            for index, slice_data in enumerate(slices):
                if not os.path.exists(TMPPATH):
                    os.makedirs(TMPPATH)
                # TODO 把控时间段长度,忽略轨迹持续时间过少的仿真
                tmp_file_path = os.path.join(TMPPATH, f"{evaluation_id}_{main_vehi_id}_{regional_id}_{index}")

                # 把安全员这一列移动至最后
                isSecurityInvolved = slice_data.pop('isSecurityInvolved')
                slice_data.insert(loc=len(slice_data.columns), column='isSecurityInvolved', value=isSecurityInvolved, allow_duplicates=False)

                if slice_data.shape[0] > 2:
                    slice_data.to_csv(tmp_file_path)
                    car_regional_trace_files[main_vehi_id][regional_id].append(tmp_file_path)

    print(f"轨迹文件: {car_regional_trace_files}")
    return car_regional_trace_files


def convert_dir_to_md5(event_trace_dir, index):
    """
        提取轨迹文件夹信息至新的文件，并生成md5及详情字典，方便上传
    """
    trace_dir = os.path.join(event_trace_dir, index)
    tmp_trace_data = []
    for file_name in os.listdir(trace_dir):
        vehi_info_data = json.load(open(os.path.join(trace_dir, file_name)))
        tmp_trace_data.append(vehi_info_data)

    file_bytes = json.dumps(tmp_trace_data).encode()
    md5_value = hashlib.md5(file_bytes).hexdigest()  # md5 值
    json.dump(tmp_trace_data, open(os.path.join(event_trace_dir, md5_value), 'w'))
    description = {
        "md5": md5_value,
        'subType': 2,
        "algorithmId": str(index),
    }
    return os.path.join(event_trace_dir, md5_value), md5_value, description

def draw_score(data):
    import matplotlib.pyplot as plt

    # Extracting the keys and values
    categories = list(data.keys())
    values = list(data.values())

    # Creating the bar chart
    plt.figure(figsize=(5, 6))
    plt.bar(categories, values, color=['pink', 'green', 'orange', 'red'])

    # Adding titles and labels
    plt.title('Category Values')
    plt.xlabel('Categories')
    plt.ylabel('Values')

    # Display the bar chart
    plt.show()


def calc_car_regional(evaluation_record):
    """
    根据测试记录，搜索路网及轨迹数据，通过主车&区域进行轨迹切分
    """
    sql = "select * from Network where id=%s"
    network_id = evaluation_record['netId']
    is_ok, network = MySQLDB.execute_sql(sql, tuple([network_id]))

    network = network and network[0]
    if evaluation_record['mainVehiId'] and network and network['regionalWeight'] and network['projectWeight']:
        regional_records = json.loads(network.get('regionalWeight') or "{}").get('EvaluationArea', [])
        regionals = {}
        for regional_record in regional_records:
            region_position = json.loads(regional_record['region'])
            regionals[regional_record['id']] = {
                "name": regional_record.get('name'),
                'weights': regional_record['weights'],
                'keyPoints': regional_record.get('keyPoints', []),
                'position': {
                    "minX": min(region_position['posFrom']['x'], region_position['posTo']['x']),
                    "maxX": max(region_position['posFrom']['x'], region_position['posTo']['x']),
                    "minY": min(region_position['posFrom']['y'], region_position['posTo']['y']),
                    "maxY": max(region_position['posFrom']['y'], region_position['posTo']['y']),
                },
            }

        # regionals = {regional['id']: regional for regional in regional_records}

        main_vehi_ids = [int(i) for i in evaluation_record["mainVehiId"].split(',') if i]
        return main_vehi_ids, regionals, network
    return [], {}, None

def start(values, score_data_result):
    for k, v in values.items():
        setattr(Parameter, k, v)

    print("开始评价：", Parameter.taskName)
    starter = evaluateStarter(Parameter, score_data_result, values)
    # producer_process = starter.startEvaluation()
    data_transfer_queue = Queue(3)
    map_path_data_queue = Queue()
    starter.get_offline_data(data_transfer_queue, map_path_data_queue, values)
    print(f"{Parameter.taskName} 计算结束")


def get_kafka_trace_data(kafkaTopic):
    consumer = KafkaConsumer(
        bootstrap_servers=[f'{kafkaTopic["host"]}:{kafkaTopic["port"]}'],
        api_version=(0, 10, 1),
        auto_offset_reset='earliest',
        value_deserializer=json.loads,  # message.value 主动格式化
        consumer_timeout_ms=30000,  # 长时间消费不到数据自动停止消费者
    )

    # 注册 topic 及 分区
    topic = kafkaTopic['topic']
    tp = TopicPartition(topic, 0)
    consumer.assign([tp])

    consumer.seek_to_end(tp)
    latest_offset = consumer.position(tp)
    consumer.seek_to_beginning(tp)

    trace_data = []
    for message in consumer:
        # print(f'get new trace: {latest_offset}, {message.offset} {message.value}')
        trace_data.append(message.value)
        if message.offset >= latest_offset - 1:
            break
    return trace_data


def calc_evaluation_score_one(evaluation_record, regionals=None, main_vehi_ids=None):
    """
        收集轨迹数据，测试评价
    """
    # print(f"evaluation_record: {evaluation_record}")
    evaluation_id, json_path, csv_file_path, map_file_folder, kafkaTopic = evaluation_record.get('id'), evaluation_record.get('jsonDataPath'), evaluation_record.get('csvDataPath'), evaluation_record.get('mapDataPath'), evaluation_record.get('kafkaTopic')
    sceneName = None
    sceneType = None
    aim_path_json_file_path = None
    if json_path:
        trace_data = []
        if os.path.exists(json_path):
            print(f"开始从本地文件夹获取轨迹: {json_path}")
            for file_name in os.listdir(json_path):
                try:
                    vehi_info_data = json.load(open(os.path.join(json_path, file_name)))
                    trace_data.append(vehi_info_data)
                except:
                    logger.error(f"本地文件{os.path.join(json_path, file_name)}解析失败")
                    continue
        else:
            # 尝试从http获取数据
            try:
                print(f"开始从云文件获取轨迹: {json_path}")
                data = requests.get(json_path)
                for line in data.text.split("\n"):
                    if line:
                        try:
                            trace_data.append(json.loads(line))
                        except:
                            logger.error(f"云文件的某轨迹解析失败: {line}")
            except:
                logger.exception(f"云文件解析失败: {json_path}")
    # else:
    #     # 用 kafka 数据代替文件数据
    #     print(f"开始从kafka获取轨迹: {kafkaTopic}")
    #     kafkaTopic = json.loads(kafkaTopic)
    #     trace_data = get_kafka_trace_data(kafkaTopic)
        logger.warning(f"轨迹数据获取: {len(trace_data)}")

        # 轨迹数据排序
        trace_data = sorted(trace_data, key=lambda x: x["simuTime"])

        # 用于存储最终结果的列表
        filtered_trace_data = []
        # 遍历排序后的列表，删除时间差小于阈值的元素
        prev_time = None
        for item in trace_data:
            simuTime = int(item['simuTime'])
            if prev_time is None or (simuTime - prev_time) >= time_threshold:
                filtered_trace_data.append(item)
                prev_time = simuTime
        trace_data = filtered_trace_data

        vehi_regional_score = collections.defaultdict(lambda: collections.defaultdict(list))

        # 分析此次测试的每辆车，每个场景切分
        # TODO:这里主车id和场景暂时改为入参，方便调整计算量
        # main_vehi_ids, regionals, network = calc_car_regional(evaluation_record)
        if not (main_vehi_ids and regionals):
            # 如果主车或区域划分不存在，不计算评价
            return vehi_regional_score

        print(f"存在需评价记录，开始评价: {evaluation_record['id']}")
        # 提取轨迹信息至新的文件(会有多辆主车，需要逐个提取)
        # wth: car_regional_trace: {main_veh_id, regional_id, csv_file_path}
        car_regional_trace = integration_trace(evaluation_id, main_vehi_ids, regionals, trace_data, filter_type=evaluation_record['filterType'])
    elif csv_file_path:
        # wth: car_regional_trace: {main_veh_id, scene_name, csv_file_path}
        # 提取主车id，场景名称，主车id不存在则定为0
        sceneName = '_'.join(os.path.basename(csv_file_path).split('_')[:-1])
        sceneType = sceneName.split('_')[0]
        # 连续性场景目录结构不同
        # wth:判断sceneType目录是否存在，不存在则改为小写
        if not os.path.exists(os.path.join(map_file_folder, sceneType)):
            sceneType = sceneType.lower()
        # 连续性场景目录结构不同，读任务名称，找到任务指向的json文件
        if sceneType == 'SERIAL' or sceneType == 'serial':
            taskName = '_'.join(sceneName.split('_')[2:])
            aim_path_json_file_path = os.path.join(map_file_folder, sceneType, 'tasks', f'{taskName}.json')
        car_regional_trace = {0: {sceneName: [csv_file_path]}}
    else:
        return None
    # 统一简化：单场景内部严格串行计算。
    # 批量并发仅由外层 score_ego_effectiveness.py 的 workers 控制。
    score_data_result = {}
    # 开始测试评价
    for main_vehi_id in car_regional_trace:
        for regional_id in car_regional_trace[main_vehi_id]:
            # 根据区域ID获取场景名
            if not sceneName:
                sceneName = regionals.get(regional_id, {}).get('name') or str(regional_id)
            for index, tmp_file_path in enumerate(car_regional_trace[main_vehi_id][regional_id]):
                taskName = f"{evaluation_id}_{main_vehi_id}_{regional_id}_{index}"
                score_data_result[taskName] = None
                values = {
                    # "csv_file_path": evaluation_record.get('csv_file_path', os.path.join(TMPPATH, taskName)),
                    "csv_file_path": tmp_file_path,
                    'map_file': map_file_folder,
                    "aim_path_json_file_path": aim_path_json_file_path,
                    "taskName": taskName,
                    "taskID": str(evaluation_id),
                    "avName": str(main_vehi_id),
                    "sceneName": sceneName,
                    "sceneType": sceneType,
                    "keyPoints": regionals.get(regional_id, {}).get('keyPoints', []) if regionals else []
                }
                start(values, score_data_result)
    if regionals:
        for taskName in score_data_result.keys():
            # 为没有评分的场景提供默认值
            taskID, avName, regional_id, index = taskName.split('_')
            regional_name = regionals.get(int(regional_id), {}).get('name') or str(regional_id)
            # TODO 此时可能存在2,没有1,需要特殊处理
            score_data_result[taskName] = score_data_result[taskName] or {"1": makeZeroData(regional_name, avName, taskID, taskID, int(time.time()), int(time.time()), regional_name)}

        # 先整合同一区域内得分，并进行记录, 车辆，区域，每次经过的得分
        for vehi_regional_index, score in score_data_result.items():
            if score is not None:
                vehi_id, regional_id = [i for i in vehi_regional_index.split('_')[1:3]]
                vehi_regional_score[vehi_id][regional_id].append(score)
    else:
        vehi_regional_score = score_data_result
    # TODO 仅仅是为了万集，在此处为未参与评分的场景提供默认得分
    checkok_score(vehi_regional_score)
    # isChildren = evaluation_record.get('isChildren', 1)
    # if not isChildren:
    #     # 正常任务，进行场景信息补全
    #     print(f"进行轨迹补全: {evaluation_id}")
    #     supplement_scene_score(evaluation_id, regionals, vehi_regional_score, main_vehi_ids)
    # else:
    #     print(f"不进行轨迹补全: {evaluation_id}")

    # checkok_score(vehi_regional_score)
    # 车辆ID:场景ID:场景内得分列表
    return vehi_regional_score


def checkok_score(value_1):
    try:
        count = 0
        for vehi_id, value_2 in value_1.items():
            for regional_id, value_3 in value_2.items():
                for index, value_4 in enumerate(value_3):
                    count += 1
        print(f"总计 {count} 条数据")
    except:
        print('评分结果不完整')


def supplement_scene_score(evaluation_id, regionals, vehi_regional_score, main_vehi_ids):
    # 仅仅是为了万集，在此处为未参与评分的场景提供默认得分
    for vehi_id in main_vehi_ids:
        vehi_id = str(vehi_id)
        if vehi_id not in vehi_regional_score:
            vehi_regional_score[vehi_id] = {}

        for regional_id, regional_value in regionals.items():
            regional_id = str(regional_id)
            if regional_id not in vehi_regional_score[vehi_id]:
                vehi_regional_score[vehi_id][regional_id] = [
                    {"1": makeZeroData(regional_value['name'], str(vehi_id), str(evaluation_id), str(evaluation_id),
                                       int(time.time()), int(time.time()), regional_value['name'])}
                ]
            # 停车区特殊处理
            if regional_value['name'] == "11-垂直车位自动泊车":
                taskName = f"{evaluation_id}_{vehi_id}_{regional_id}_{0}"
                vehi_regional_score[vehi_id][regional_id] = [
                    {"1": startCalParkingIndex(regional_value['name'], str(vehi_id), str(evaluation_id),
                                               str(evaluation_id), int(time.time()), int(time.time()),
                                               regional_value['name'], os.path.join(TMPPATH, taskName))}
                ]

def calc_evaluation(evaluation_record):
    """
        计算测试记录的得分
    """
    try:
        # 此处进行计算
        vehi_regional_score = calc_evaluation_score_one(evaluation_record)
        print(f"评价结果: {bool(vehi_regional_score)}")
    except:
        vehi_regional_score = {}
        logger.exception(f"评价计算出错")

    # 数据入库,记录仿真的单次得分，后续展示时根据实时的权重分配进行整合
    evaluation_id, trace_dir, kafkaTopic = evaluation_record['id'], evaluation_record['jsonDataPath'], evaluation_record['kafkaTopic']
    MySQLDB.update_evaluation_data(
        evaluation_id,
        {
            "status": 'end',
            'score': json.dumps(vehi_regional_score)
        }
    )
    # 移除轨迹文件
    if trace_dir:
        try:
            # TODO：暂时取消移除轨迹数据的逻辑，方便测试
            remove_result = True
            # remove_result = remove_dir(trace_dir)
        except:
            remove_result = False
        logger.info(f'事件文件夹移除: {trace_dir}, 移除结果: {remove_result}')

    # TODO 清理此进程
    # control_child_processes(os.getpid(), controlSelf=True)

    # 分数展示
    # draw_score(score_data)

def control_child_processes(parent_id, sig=signal.SIGTERM, controlSelf=False):
    """
        给定父进程，控制该进程的所有子进程
    :param parent: 父进程
    :param sig: 进程信号，默认为 kill
    :param killSelf: 是否对主进程本身做同样的控制
    :return:
    """
    parent = psutil.Process(parent_id)
    children = parent.children(recursive=True)
    if controlSelf:
        children.append(parent)
    for process in children:
        try:
            process.send_signal(sig)
        except:
            logger.exception(f"处理进程发生异常")
    return True
