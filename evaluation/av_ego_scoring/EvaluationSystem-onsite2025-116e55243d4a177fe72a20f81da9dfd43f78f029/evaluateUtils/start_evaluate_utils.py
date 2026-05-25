import json
from queue import Queue
from multiprocessing import Process, Queue
# from queue import Queue

from evaluateUtils.my_evaluate import evaluationSystem
from evaluateUtils.get_offline_data import getOfflineData
from evaluateUtils.standard_parameter import Parameter


class evaluateStarter:
    def __init__(self, Parameter, score_data_result, values):
        self.score_data_result = score_data_result
        self.map_file = Parameter.map_file
        self.aim_place_xosc_file_path = Parameter.aim_place_xosc_file_path
        self.csv_file_path = Parameter.csv_file_path
        self.aim_path_json_file_path = Parameter.aim_path_json_file_path
        self.taskName = Parameter.taskName
        self.values = values

    # 消费者的函数
    def my_evaluation_channel(self, data_transfer_queue, map_path_data_queue, values):
        for k, v in values.items():
            setattr(Parameter, k, v)
        evaluation = evaluationSystem(self.score_data_result)

        evaluation.startMyEvaluation(data_transfer_queue, map_path_data_queue)
        print("停止评价系统")

    def get_offline_data(self, data_transfer_queue, map_path_data_queue, values):
        for k, v in values.items():
            setattr(Parameter, k, v)
        offlineData = getOfflineData(self.map_file, self.aim_place_xosc_file_path, Parameter.csv_file_path, self.aim_path_json_file_path, self.taskName, self.score_data_result)
        offlineData.startGetOfflineData(data_transfer_queue, map_path_data_queue)
        print("停止接受数据")


    def start_producer_process(self, data_transfer_queue, map_path_data_queue):
        # 启动生产者进程，target目标是谁就用谁获取数据
        producer_process = Process(target=self.get_offline_data, args=(data_transfer_queue, map_path_data_queue, self.values))
        producer_process.start()
        return producer_process

    def start_consumer_process(self, data_transfer_queue, map_path_data_queue):
        # 启动消费者进程
        consumer_process = Process(target=self.my_evaluation_channel, args=(data_transfer_queue, map_path_data_queue, self.values))
        consumer_process.start()
        return consumer_process


    def startEvaluation(self):
        data_transfer_queue = Queue(3)
        map_path_data_queue = Queue()

        producer_process = self.start_producer_process(data_transfer_queue, map_path_data_queue)
        # consumer_process = self.start_consumer_process(data_transfer_queue, map_path_data_queue)

        return producer_process

