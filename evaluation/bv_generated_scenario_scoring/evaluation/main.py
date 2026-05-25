# 主程序作为开启程序
import multiprocessing
import time
import csv
import os
import pandas as pd
from evaluateUtils.my_evaluate import evaluationSystem
from evaluateUtils.get_offline_data import getOfflineData
from evaluateUtils.standard_parameter import Parameter
from start_evaluate_utils import evaluateStarter
import json
from dynamicCheck.Dynamic_Check import dynamic_check
from tqdm import tqdm
import xml.etree.ElementTree as et

def get_files_by_type(root_folder):
    # 遍历每个tester_id目录
    for tester_id in os.listdir(os.path.join(root_folder, 'trajectory')):
        tester_path = os.path.join(root_folder, 'trajectory', tester_id)
        if os.path.isdir(tester_path):
            # 遍历A/B/C子目录
            for sub_dir in ['A', 'B', 'C']:
                sub_path = os.path.join(tester_path, sub_dir)
                if os.path.exists(sub_path):
                    # 创建对应的结果输出目录
                    result_dir = os.path.join(root_folder, 'result', tester_id, sub_dir)
                    os.makedirs(result_dir, exist_ok=True)
                    
                    # 创建该子目录的结果文件
                    detail_csv_path = os.path.join(result_dir, "av_safety_score_detail.csv")
                    score_csv_path = os.path.join(result_dir, "av_safety_score.csv")
                    
                    # 写入CSV文件头
                    with open(detail_csv_path, "w", newline="") as csv_file:
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow(["Scenario", "Safety", "Efficiency", "Comfort", "Total",
                                          "task_type","road_type","test_state","dynamic_result","reason","testDuration",
                                          "crash","ttc_ratio","outoflane_ratio","subtendlane_ratio","redlight",
                                          "mission_accomplish","speed",
                                          "horizontalA_ratio","horizontalAA_ratio","verticalA_ratio","verticalAA_ratio","turnA_ratio",
                                          "horizontalA","horizontalAA","verticalA","verticalAA","turn_A"])
                    
                    with open(score_csv_path, "w", newline="") as csv_file:
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow(["Scenario", "Safety", "Efficiency", "Comfort", "Total"])
                    
                    # 获取该目录下的所有轨迹文件
                    filtered_filenames = [filename for filename in os.listdir(sub_path) 
                                       if ('REPLAY' in filename or 'FRAGMENT' in filename 
                                           or 'SERIAL' in filename) and '.csv' in filename]
                    
                    # 处理每个轨迹文件
                    for filename in tqdm(filtered_filenames, desc=f'评价进度 {tester_id}/{sub_dir}:'):
                        # 拿到轨迹文件对应的轨迹名称,去掉'_result.csv'后缀
                        currentTaskName = filename[:-11]
                        #print(currentTaskName)
                        Parameter.taskName = currentTaskName
                        print(currentTaskName)
                        task_type=(currentTaskName.split("_")[0]).lower()
                        # 取得现有任务名中第3个下划线之后的部分，即场景名称
                        taskname_sce="_".join(currentTaskName.split("_")[3:])
                        print(taskname_sce)
                        fileNameDict = {}
                        # 遍历每个文件，改一下路径，在这个循环中开始评价

                        ##连续性交互测试再多加一层判断
                        if task_type!="serial":
                            currentTask_scepath = os.path.join(root_folder + '/' + 'scenario' + '/' + task_type + '/' + taskname_sce)
                        #如果没有放入轨迹对应的场景，全部0分
                            if not os.path.exists(currentTask_scepath):

                                csv_file_path = os.path.join(root_folder + '/' + 'result' + '/' + "av_safety_score_detail.csv")
                                # 写入 CSV 文件
                                with open(csv_file_path, mode="a", newline="", encoding="utf-8") as file:
                                    csv_writer = csv.writer(file)
                                    csv_writer.writerow([currentTaskName, 0, 0, 0, 0,task_type,0,
                                                         0,0,0,0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0])

                                # 创建一个csv_file_path存放结果数据
                                csv_file_path = os.path.join(root_folder + '/' + 'result' + '/' + "av_safety_score.csv")
                                # 写入 CSV 文件
                                with open(csv_file_path, mode="a", newline="", encoding="utf-8") as file:
                                    csv_writer = csv.writer(file)
                                    csv_writer.writerow([currentTaskName, 0, 0, 0, 0])

                            else:#只有需要的场景存在，才能继续进行
                                files = os.listdir(currentTask_scepath)
                                startNum = len(files) + 1
                                num = 0
                                for file_name in files:
                                    # 获取文件的扩展名（文件类型）
                                    _, file_extension = os.path.splitext(file_name)

                                    # 计数器，最后一个文件也读完的时候才能评价
                                    num += 1

                                    # 如果文件类型在指定的文件类型列表中，则将文件路径添加到列表中
                                    # 地图信息
                                    if file_extension.lower() in ['.xodr']:
                                        file_path = os.path.join(currentTask_scepath, file_name)
                                        # Parameter.map_file = file_path
                                        fileNameDict["map_file"] = file_path
                                    # 目标结束位置信息
                                    if file_extension.lower() in ['.xosc']:
                                        file_path = os.path.join(currentTask_scepath, file_name)
                                        # Parameter.aim_place_xosc_file_path = file_path
                                        fileNameDict["aim_place_xosc_file_path"] = file_path
                                    # 轨迹信息
                                    # if file_extension.lower() in ['.csv']:
                                    #     file_path = os.path.join(root, file_name)
                                    #     # Parameter.csv_file_path = file_path
                                    #     fileNameDict["csv_file_path"] = file_path
                                    # 目标路径信息
                                    if file_extension.lower() in ['.json']:
                                        file_path = os.path.join(currentTask_scepath, file_name)
                                        # Parameter.aim_path_json_file_path = file_path
                                        fileNameDict["aim_path_json_file_path"] = file_path
                                xml_file=fileNameDict["map_file"]
                                xml = et.ElementTree(file=xml_file)
                                root_xml = xml.getroot()
                                headers = root_xml.findall('header')
                                road_type=headers[0].attrib['name']

                                #轨迹信息
                                csvfile_path = os.path.join(root_folder, 'trajectory', tester_id, sub_dir, filename)
                                fileNameDict["csv_file_path"] = csvfile_path
                                output = pd.read_csv(csvfile_path)
                                if len(output)==1 or output['end'].iloc[-1]==-1:#轨迹没有输出完，认为没有完成任务，全部为0分
                                    # 写入 CSV 文件
                                    with open(detail_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                        csv_writer = csv.writer(file)
                                        csv_writer.writerow([currentTaskName, 0, 0, 0, 0,task_type,road_type,
                                                         0,0,0,0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0])

                                    # 写入 CSV 文件
                                    with open(score_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                        csv_writer = csv.writer(file)
                                        csv_writer.writerow([currentTaskName, 0, 0, 0, 0])

                                else:  #只有轨迹文件存在且完全正常输出了，才能评价
                                    t = output.iloc[:, 0].tolist()
                                    deltaf = output.iloc[:, 8].tolist()
                                    vx = output.iloc[:, 5].tolist()
                                    ax = output.iloc[:, 6].tolist()
                                    FunctionReturns = dynamic_check(t, deltaf, vx, ax)
                                    if FunctionReturns[0] == 1:  # 只有通过动力学检测，才进行评价
                                        num += 1
                                    else:
                                        # 写入 CSV 文件
                                        with open(detail_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                            csv_writer = csv.writer(file)
                                            csv_writer.writerow(
                                                [currentTaskName, 0, 0, 0, 0,task_type,road_type,
                                                 1, FunctionReturns[0], FunctionReturns[1], 0,
                                                 0, 0, 0, 0, 0,
                                                 0, 0,
                                                 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0])
                                        # 写入 CSV 文件
                                        with open(score_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                            csv_writer = csv.writer(file)
                                            csv_writer.writerow([currentTaskName, 0, 0, 0, 0])

                                file_path = os.path.join(currentTask_scepath, "xxx")
                                if "map_file" in fileNameDict:
                                    Parameter.map_file = fileNameDict["map_file"]
                                else:
                                    Parameter.map_file = file_path

                                if "aim_place_xosc_file_path" in fileNameDict:
                                    Parameter.aim_place_xosc_file_path = fileNameDict["aim_place_xosc_file_path"]
                                else:
                                    Parameter.aim_place_xosc_file_path = file_path

                                if "csv_file_path" in fileNameDict:
                                    Parameter.csv_file_path = fileNameDict["csv_file_path"]
                                else:
                                    Parameter.csv_file_path = file_path

                                if "aim_path_json_file_path" in fileNameDict:
                                    Parameter.aim_path_json_file_path = fileNameDict["aim_path_json_file_path"]
                                else:
                                    Parameter.aim_path_json_file_path = file_path


                                # 当num等于startnum的时候，证明评价需要的地图信息都准备好了
                                if num == startNum:
                                    #print(Parameter.map_file,Parameter.aim_place_xosc_file_path,Parameter.csv_file_path,Parameter.aim_path_json_file_path, Parameter.taskName)
                                    # 在这里就可以开始评价了
                                    # 这里就是主进程的开始，然后就在这里卡主他，直到当前进程结束
                                    starter = evaluateStarter(
                                        map_file=Parameter.map_file,
                                        aim_place_xosc_file_path=Parameter.aim_place_xosc_file_path,
                                        csv_file_path=Parameter.csv_file_path,
                                        aim_path_json_file_path=Parameter.aim_path_json_file_path,
                                        taskName=Parameter.taskName,
                                        detail_csv_path=detail_csv_path,
                                        score_csv_path=score_csv_path
                                    )
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
                        else:
                            # TODO: 需要从A/B/C文件夹中分别读取赛题
                            currentTask_scejsonpath = os.path.join(root_folder + '/' + 'scenario' + '/' + task_type + '/' +'tasks'+'/'+taskname_sce+'.json')
                            # 如果没有放入轨迹对应的场景，全部0分
                            if not os.path.exists(currentTask_scejsonpath):
                                csv_file_path = os.path.join(root_folder + '/' + 'result' + '/' + "av_safety_score_detail.csv")
                                # 写入 CSV 文件
                                with open(csv_file_path, mode="a", newline="", encoding="utf-8") as file:
                                    csv_writer = csv.writer(file)
                                    csv_writer.writerow([currentTaskName, 0, 0, 0, 0,task_type,0,
                                                         0,0,0,0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0])

                                # 创建一个csv_file_path存放结果数据
                                csv_file_path = os.path.join(root_folder + '/' + 'result' + '/' + "av_safety_score.csv")
                                # 写入 CSV 文件
                                with open(csv_file_path, mode="a", newline="", encoding="utf-8") as file:
                                    csv_writer = csv.writer(file)
                                    csv_writer.writerow([currentTaskName, 0, 0, 0, 0])

                            else:  # 只有需要的场景存在，才能继续进行

                                startNum =1
                                num = 0
                                 # 计数器，最后一个文件也读完的时候才能评价
                                # 如果文件类型在指定的文件类型列表中，则将文件路径添加到列表中
                                #目标路径信息
                                fileNameDict["aim_path_json_file_path"] = currentTask_scejsonpath
                                with open(currentTask_scejsonpath, 'r') as file:
                                    # 读取文件内容并解析为Python字典
                                    data_dict = json.load(file)

                                xodr_type=data_dict['map']
                                currentTask_sce_xodrpath=os.path.join(root_folder + '/' + 'scenario' + '/' + task_type + '/' +'maps'+'/'+xodr_type+'/'+'TJST.xodr')
                                # 地图信息
                                fileNameDict["map_file"] = currentTask_sce_xodrpath

                                # 轨迹信息
                                csvfile_path = os.path.join(root_folder, 'trajectory', tester_id, sub_dir, filename)
                                fileNameDict["csv_file_path"] = csvfile_path
                                output = pd.read_csv(csvfile_path)
                                if len(output) == 1 or output['end'].iloc[-1] == -1:  # 轨迹没有输出完，认为没有完成任务，全部为0分
                                    # 写入 CSV 文件
                                    with open(detail_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                        csv_writer = csv.writer(file)
                                        csv_writer.writerow([currentTaskName, 0, 0, 0, 0,task_type,task_type,
                                                             0,0,0,0,
                                                             0, 0, 0, 0, 0,
                                                             0, 0,
                                                             0, 0, 0, 0, 0,
                                                             0, 0, 0, 0, 0])

                                    # 写入 CSV 文件
                                    with open(score_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                        csv_writer = csv.writer(file)
                                        csv_writer.writerow([currentTaskName, 0, 0, 0, 0])
                                else:  # 只有轨迹文件存在且完全正常输出了，才能评价
                                    t = output.iloc[:, 0].tolist()
                                    deltaf = output.iloc[:, 8].tolist()
                                    vx = output.iloc[:, 5].tolist()
                                    ax = output.iloc[:, 6].tolist()
                                    FunctionReturns = dynamic_check(t, deltaf, vx, ax)
                                    if FunctionReturns[0] == 1 :  # 只有通过动力学检测，才进行评价
                                        num += 1
                                    else:
                                        # 写入 CSV 文件
                                        with open(detail_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                            csv_writer = csv.writer(file)
                                            csv_writer.writerow(
                                                [currentTaskName, 0, 0, 0, 0, task_type,task_type,
                                                 1, FunctionReturns[0], FunctionReturns[1], 0,
                                                 0, 0, 0, 0, 0,
                                                 0, 0,
                                                 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0])
                                        # 写入 CSV 文件
                                        with open(score_csv_path, mode="a", newline="", encoding="utf-8") as file:
                                            csv_writer = csv.writer(file)
                                            csv_writer.writerow([currentTaskName, 0, 0, 0, 0])

                                file_path = os.path.join(currentTask_scejsonpath, "xxx")
                                if "map_file" in fileNameDict:
                                    Parameter.map_file = fileNameDict["map_file"]
                                else:
                                    Parameter.map_file = file_path

                                if "aim_place_xosc_file_path" in fileNameDict:
                                    Parameter.aim_place_xosc_file_path = fileNameDict["aim_place_xosc_file_path"]
                                else:
                                    Parameter.aim_place_xosc_file_path = file_path

                                if "csv_file_path" in fileNameDict:
                                    Parameter.csv_file_path = fileNameDict["csv_file_path"]
                                else:
                                    Parameter.csv_file_path = file_path

                                if "aim_path_json_file_path" in fileNameDict:
                                    Parameter.aim_path_json_file_path = fileNameDict["aim_path_json_file_path"]

                                else:
                                    Parameter.aim_path_json_file_path = file_path

                                # 当num等于startnum的时候，证明评价需要的地图信息都准备好了
                                if num == startNum:
                                    # 在这里就可以开始评价了
                                    # 这里就是主进程的开始，然后就在这里卡主他，直到当前进程结束
                                    starter = evaluateStarter(
                                        map_file=Parameter.map_file,
                                        aim_place_xosc_file_path=Parameter.aim_place_xosc_file_path,
                                        csv_file_path=Parameter.csv_file_path,
                                        aim_path_json_file_path=Parameter.aim_path_json_file_path,
                                        taskName=Parameter.taskName,
                                        detail_csv_path=detail_csv_path,
                                        score_csv_path=score_csv_path
                                    )
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
                        pass
                    ##计算最后的平均值
                    # CSV文件路径使用当前子目录的路径
                    column_sums = {i: 0 for i in range(1, 28)}
                    column_counts = {i: 0 for i in range(1, 28)}
                    with open(detail_csv_path, 'r', newline='') as csvfile:
                        reader = csv.reader(csvfile)
                        next(reader)  # 跳过头部
                        for row in reader:
                            for i in range(1, 28):  # 从第二列开始计算
                                try:
                                    column_sums[i] += float(row[i])
                                    column_counts[i] += 1
                                except ValueError:
                                    # 如果值不是数字，则忽略它
                                    pass
                    # 计算平均值（跳过第一列）
                    average_row = ['mean'] + [column_sums[i] / column_counts[i] if column_counts[i] > 0 else 0 for i in
                                         range(1, 28)]
                    # 在CSV文件的最后添加一行平均值
                    with open(detail_csv_path, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow(average_row)

                    # 写入score.csv的平均值
                    with open(score_csv_path, mode="a", newline="", encoding="utf-8") as file:
                        csv_writer = csv.writer(file)
                        csv_writer.writerow(average_row[:5])
    # ## 计算最后的平均值
    # file_path = os.path.join(root_folder + '/' + 'result' + '/' + "av_safety_score_detail.csv")
    # column_sums = {i: 0 for i in range(1, 28)}
    # column_counts = {i: 0 for i in range(1, 28)}
    # with open(file_path, 'r', newline='') as csvfile:
    #     reader = csv.reader(csvfile)
    #     next(reader)  # 跳过头部
    #     for row in reader:
    #         for i in range(1, 28):  # 从第二列开始计算
    #             try:
    #                 column_sums[i] += float(row[i])
    #                 column_counts[i] += 1
    #             except ValueError:
    #                 pass
    # average_row = ['mean'] + [column_sums[i] / column_counts[i] if column_counts[i] > 0 else 0 for i in range(1, 28)]
    # with open(file_path, 'a', newline='') as csvfile:
    #     writer = csv.writer(csvfile)
    #     writer.writerow(average_row)
    # csv_file_path = os.path.join(root_folder + '/' + 'result' + '/' + "av_safety_score.csv")
    # with open(csv_file_path, mode="a", newline="", encoding="utf-8") as file:
    #     csv_writer = csv.writer(file)
    #     csv_writer.writerow(average_row[:5])
    
    return {"安全": average_row[1], "效率": average_row[2], "舒适": average_row[3], "总分": average_row[4]}






def start(map_file, aim_place_xosc_file_path, csv_file_path, aim_path_json_file_path, taskName):
    starter = evaluateStarter(map_file, aim_place_xosc_file_path, csv_file_path, aim_path_json_file_path, taskName)
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



if __name__ == '__main__':
    multiprocessing.freeze_support()
    root_folder_path = './testData'  # 替换为实际的根文件夹路径
    result_paths = get_files_by_type(root_folder_path)
