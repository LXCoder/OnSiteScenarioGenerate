import collections
import hashlib
import json
import logging
import multiprocessing
import os
import subprocess
import time
import traceback
import pandas as pd

from multiprocessing import Process

from kafka import KafkaConsumer, TopicPartition

from evaluateUtils.start_evaluate_utils import evaluateStarter
from utils.SQLDB import MySQLDB
from utils.config import TMPPATH, calc_evaluation_max_time, split_time_interval, log_name
from evaluateUtils.standard_parameter import Parameter


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

def integration_trace(evaluation_id, main_vehi_ids, regionals, trace_data, filter_type=None):
    """
        分析轨迹数据文件夹，提取轨迹数据至新的csv文件中
    """
    # 记录每辆车，在每个区域的轨迹,默认是 DataFrame
    car_regional_trace = collections.defaultdict(lambda : collections.defaultdict(lambda : pd.DataFrame(columns=['x_ego', 'y_ego', 'v_ego', 'a_ego', 'yaw_ego', 'width_ego', 'length_ego'])))

    # 初始化多辆车，多个场景的 pdData
    # 初始化数据(主车在前几列)
    lastSimuTime = -1
    for vehi_info_data in trace_data:
        simuTime = vehi_info_data['simuTime'] / 1000
        if simuTime - lastSimuTime < 1:
            # 如果时间过去不久(1s内)，不进行记录
            continue

        lastSimuTime = simuTime
        # 遍历每一辆主车，如果主车在某区域内，就需要把这帧数据进行注入
        for main_vehi_id in main_vehi_ids:
            # 提取出这帧轨迹中的主车信息，提前提取进行判断，可以避免记录大量不需要记录的帧数据
            main_vehi_info = None
            for vehi_info in vehi_info_data['data']:
                if vehi_info['id'] == main_vehi_id:
                    main_vehi_info = vehi_info
                    break
            if not main_vehi_info:
                # 当前帧没有指定主车，跳过
                continue
            # main_vehi_infos = [vehi_info for vehi_info in vehi_info_data['data'] if vehi_info['id'] == main_vehi_id]
            # if not main_vehi_infos:
            #     continue
            # main_vehi_info = main_vehi_infos[0]

            for regional_id, regional_value in regionals.items():
                if filter_type == 'region' and main_vehi_info.get('regionId') != regional_id:
                    # 如果过滤条件为区域过滤，且当前主车携带的区域ID不满足条件，跳过
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

    car_regional_trace_files = collections.defaultdict(lambda : collections.defaultdict(list))

    for main_vehi_id in car_regional_trace:
        for regional_id in car_regional_trace[main_vehi_id]:
            pdData = car_regional_trace[main_vehi_id][regional_id]
            # 去除 ego 车不存在的轨迹帧
            pdData.drop(pdData[(pdData['x_ego'].isnull() | (pdData['x_ego'].isin([''])))].index, inplace=True)
            # 根据仿真时间排序(索引)
            pdData = pdData.loc[pdData.index.sort_values()]

            if filter_type == 'time':
                # 如果过滤条件为时间过滤，指定时间进行分段

                # 计算相邻两行 simuTime 的差值
                # pdData['diff'] = pdData['simuTime'].diff()
                indexs = pdData.index.values  # 时间戳列表
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
                slices = [pdData]

            # 删除辅助列 'diff' 并确保每个切片的列名唯一
            for index, slice_data in enumerate(slices):
                if not os.path.exists(TMPPATH):
                    os.makedirs(TMPPATH)
                # TODO 把控时间段长度,忽略轨迹持续时间过少的仿真
                tmp_file_path = os.path.join(TMPPATH, f"{evaluation_id}_{main_vehi_id}_{regional_id}_{index}")
                pdData.to_csv(tmp_file_path)
                car_regional_trace_files[main_vehi_id][regional_id].append(tmp_file_path)

    return car_regional_trace_files

def upload_file(files, recordId, description=None):
    """
        files: 一个或多个文件
    """
    if not isinstance(files):
        files = [files]
    return True


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
        # # 需要路网，区域，权重均存在
        # sql = "select * from Regional where netId=%s and isDeleted=0"
        # regional_records = MySQLDB.execute_sql(sql, tuple(network_id, ))
        # # 提取存在的区域
        # regionals = {regional['id']: regional for regional in regional_records if
        #              regional['id'] in network['regionalWeight']}
        regional_records = json.loads(network.get('regionalWeight') or "{}").get('EvaluationArea', [])
        regionals = {}
        for regional_record in regional_records:
            region_position = json.loads(regional_record['region'])
            regionals[regional_record['id']] = {
                'weights': regional_record['weights'],
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
    return [], [], None

def wait_score_done(score_data_result):
    """
    检查各辆车的所有评价结果，针对评价结果在区域层进行合并
    """
    start_calc_time = time.time()
    while True:
        is_done = True
        # 遍历检查数据是否计算完成
        for key in score_data_result.keys():
            if score_data_result[key] is None and time.time() - start_calc_time < calc_evaluation_max_time:
                # 无指标且未达到最大计算时间
                is_done = False
                break
        if is_done:
            # 计算完成，跳出循环
            break
        else:
            # 计算未完成，继续等待
            time.sleep(10)

def start(values, score_data_result):
    for k, v in values.items():
        setattr(Parameter, k, v)

    print("taskName", Parameter.taskName)
    starter = evaluateStarter(Parameter, score_data_result, values)
    producer_process, consumer_process = starter.startEvaluation()
    # 这里是为了卡主进程
    while True:
        if not consumer_process.is_alive():  # 消费者一定会结束，不能永远 while true
            break
        time.sleep(2)
    # 子进程结束
    producer_process.terminate()
    producer_process.join()
    consumer_process.terminate()
    consumer_process.join()

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
    print(f"从kafka 获取到轨迹信息: {len(trace_data)}")
    return trace_data


def calc_evaluation_score_one(evaluation_record):
    """
        收集轨迹数据，测试评价
    """
    evaluation_id, trace_dir, kafkaTopic = evaluation_record['id'], evaluation_record['jsonDataPath'], evaluation_record['kafkaTopic']
    if trace_dir:
        print(f"开始从文件夹获取轨迹: {trace_dir}")
        trace_data = []
        for file_name in os.listdir(trace_dir):
            try:
                vehi_info_data = json.load(open(os.path.join(trace_dir, file_name)))
                trace_data.append(vehi_info_data)
            except:
                # print(f"文件{os.path.join(trace_dir, file_name)}解析失败")
                continue
    else:
        # 用 kafka 数据代替文件数据
        kafkaTopic = json.loads(kafkaTopic)
        print(f"开始从kafka获取轨迹: {kafkaTopic}")
        trace_data = get_kafka_trace_data(kafkaTopic)

    # 轨迹数据排序
    trace_data = sorted(trace_data, key=lambda x: x["simuTime"])
    print(f"获取轨迹信息: {len(trace_data)}")

    vehi_regional_score = collections.defaultdict(lambda: collections.defaultdict(list))

    main_vehi_ids, regionals, network = calc_car_regional(evaluation_record)

    if not (main_vehi_ids and regionals):
        return vehi_regional_score

    # 如果主车或区域划分存在，计算评价
    print(f"存在需评价记录，开始评价: {evaluation_record}")
    # 开始评价
    # 提取轨迹信息至新的文件(会有多辆主车，需要逐个提取)

    car_regional_trace = integration_trace(evaluation_id, main_vehi_ids, regionals, trace_data, filter_type=evaluation_record['filter_type'])

    score_data_result = multiprocessing.Manager().dict()
    # 开始测试评价
    for main_vehi_id in car_regional_trace:
        for regional_id in car_regional_trace[main_vehi_id]:
            for index, tmp_file_path in enumerate(car_regional_trace[main_vehi_id][regional_id]):
                taskName = f"{evaluation_id}_{main_vehi_id}_{regional_id}_{index}"
                score_data_result[taskName] = None
                values = {
                    "csv_file_path": os.path.join(TMPPATH, taskName),
                    "taskName": taskName,
                }
                take_trace_p = Process(target=start, args=(values, score_data_result))
                take_trace_p.start()

    # 等待计算完成
    wait_score_done(score_data_result)

    # 先整合同一区域内得分，并进行记录, 车辆，区域，每次经过的得分
    for vehi_regional_index, score in score_data_result.items():
        if score is not None:
            vehi_id, regional_id = [int(i) for i in vehi_regional_index.split('_')[1:3]]
            vehi_regional_score[vehi_id][regional_id].append(score)

    # 车辆ID:场景ID:场景内得分列表
    return vehi_regional_score

def calc_evaluation(evaluation_record):
    """
        计算测试记录的得分
    """
    try:
        vehi_regional_score = calc_evaluation_score_one(evaluation_record)
        print(f"评价结果: {vehi_regional_score}")
    except:
        vehi_regional_score = {}
        error = str(traceback.format_exc())
        logger.error(f"评价计算出错: {error}")
        print(f"评价计算出错: {error}")

    evaluation_id, trace_dir, kafkaTopic = evaluation_record['id'], evaluation_record['jsonDataPath'], evaluation_record['kafkaTopic']
    # 数据入库,记录仿真的单次得分，后续展示时根据实时的权重分配进行整合
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
            remove_result = remove_dir(trace_dir)
        except:
            remove_result = False
        logger.info(f'事件文件夹移除: {trace_dir}, 移除结果: {remove_result}')

    # 分数展示
    # draw_score(score_data)
