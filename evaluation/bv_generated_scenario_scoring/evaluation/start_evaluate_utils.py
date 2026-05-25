import json
from queue import Queue
from multiprocessing import Process, Queue
from .evaluateUtils.my_evaluate import evaluationSystem
from .evaluateUtils.get_offline_data import getOfflineData
from .evaluateUtils.standard_parameter import Parameter


class evaluateStarter:
    def __init__(self, map_file, aim_place_xosc_file_path, csv_file_path, aim_path_json_file_path, taskName, detail_csv_path=None, score_csv_path=None):
        self.map_file = map_file
        self.aim_place_xosc_file_path = aim_place_xosc_file_path
        self.csv_file_path = csv_file_path
        self.aim_path_json_file_path = aim_path_json_file_path
        self.taskName = taskName
        self.detail_csv_path = detail_csv_path
        self.score_csv_path = score_csv_path

    # 消费者的函数
    def my_evaluation_channel(self, data_transfer_queue, map_path_data_queue, result_queue):
        evaluation = evaluationSystem(detail_csv_path=None, score_csv_path=None)
        result = evaluation.startMyEvaluation(data_transfer_queue, map_path_data_queue)
        result_queue.put(result)

    def get_offline_data(self, data_transfer_queue, map_path_data_queue):
        offlineData = getOfflineData(self.map_file, self.aim_place_xosc_file_path, self.csv_file_path, self.aim_path_json_file_path, self.taskName)
        offlineData.startGetOfflineData(data_transfer_queue, map_path_data_queue)

    def start_producer_process(self, data_transfer_queue, map_path_data_queue):
        producer_process = Process(target=self.get_offline_data, args=(data_transfer_queue, map_path_data_queue,))
        producer_process.start()
        return producer_process

    def start_consumer_process(self, data_transfer_queue, map_path_data_queue, result_queue):
        consumer_process = Process(target=self.my_evaluation_channel, args=(data_transfer_queue, map_path_data_queue, result_queue))
        consumer_process.start()
        return consumer_process

    def startEvaluation(self):
        data_transfer_queue = Queue(3)
        map_path_data_queue = Queue()
        result_queue = Queue()

        producer_process = self.start_producer_process(data_transfer_queue, map_path_data_queue)
        consumer_process = self.start_consumer_process(data_transfer_queue, map_path_data_queue, result_queue)

        return producer_process, consumer_process, result_queue