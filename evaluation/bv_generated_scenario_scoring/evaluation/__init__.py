import time
from .start_evaluate_utils import evaluateStarter

# 单个场景的评价
def run_evaluator(map_file, csv_file_path):
    starter = evaluateStarter(
        map_file=map_file,
        aim_place_xosc_file_path=None,
        csv_file_path=csv_file_path,
        aim_path_json_file_path=None,
        taskName=''
    )
    producer_process, consumer_process, result_queue = starter.startEvaluation()
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

    return result_queue.get()


if __name__ == '__main__':
    map_file = r'./testData/scenario/replay/0_1049_merge_1066\0_1049_merge_1066.xodr'
    aim_place_xosc_file_path = r'./testData/scenario/replay/0_1049_merge_1066\xxx'
    csv_file_path = r'./testData\trajectory\gt\A\REPLAY_0_av0_0_1049_merge_1066_result.csv'
    aim_path_json_file_path = r'./testData/scenario/replay/0_1049_merge_1066\xxx'
    taskName = 'REPLAY_0_av0_0_1049_merge_1066' 
    result = run_evaluator(map_file, aim_place_xosc_file_path, csv_file_path, aim_path_json_file_path, taskName)
    print(result)