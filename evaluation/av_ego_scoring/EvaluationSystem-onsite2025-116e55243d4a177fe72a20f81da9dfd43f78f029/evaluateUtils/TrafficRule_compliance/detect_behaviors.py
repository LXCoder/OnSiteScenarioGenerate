from typing import List
import os
import pandas as pd
import numpy as np
import csv
import json
import re
from evaluateUtils.config import SCENARIO_ID_COL
from evaluateUtils.standard_parameter import Parameter
from evaluateUtils.TrafficRule_compliance.data import load_multi_database, load_scenario_elements, filter_scenario_tags
from evaluateUtils.TrafficRule_compliance.pipeline import run_rulelist_pipeline
from shapely.geometry import Point
from evaluateUtils.TrafficRule_compliance.get_speed_limit_function import get_lane_section_speed_limits
from evaluateUtils.TrafficRule_compliance.calculate_special_lane import parse_special_lanes
from evaluateUtils.TrafficRule_compliance.calculate_road_mark_boundary import parse_road_marks
from evaluateUtils.TrafficRule_compliance.calculate_lane_section_boundary import get_lane_section_polygons
from evaluateUtils.TrafficRule_compliance.rules_dict import rules_dict1, rules_dict2, rules_name, content_dict, rules_time1, rules_time2
from evaluateUtils.TrafficRule_compliance.rules_dict import behaviors, content_dict_2, rules_output_behaviors, rules_point
from evaluateUtils.TrafficRule_compliance.file_utils import is_lefthand_scn


def run_RuleListConstruction(
        scn_folder_name: str,
        input_scn_tag: pd.DataFrame,
        # op_fn: str,
        db_list: List[str],
        dbfile_suffix: str = '.csv',
        # language: str = 'zh',
        r2b_mode: str = 'gt',
):
    # output_path = os.path.join(CACHE_DIR, op_fn)
    # print('The rule list will be saved at:', output_path)

    # load database
    db, tag2rule_dict = load_multi_database(db_list=db_list, filename_suffix=[dbfile_suffix] * len(db_list))
    print('Database has been loaded successfully,including {} articles'.format(len(db)))

    # load input file
    scn_tag = load_scenario_elements(
        scn_tag=input_scn_tag,
    )

    print('The input file has been loaded successfully,including {scn_num} scenarios and a total of {seg_num} segments' \
          .format(scn_num=scn_tag[SCENARIO_ID_COL].nunique(),
                  seg_num=len(scn_tag))
          )

    # Filter the tag code that is not in the database to accelerate the retrieval process
    scn_tag = filter_scenario_tags(scn_tag=scn_tag, tag2rule_dict=tag2rule_dict)
    use_rule2behavior_gt = r2b_mode == 'gt'
    if use_rule2behavior_gt:
        if "commands" not in db.columns:
            raise ValueError(
                "The database does not contain the commands column, the ground truth of the behavior will not be used.")
        if "prohibitions" not in db.columns:
            raise ValueError(
                "The database does not contain the prohibitions column, the ground truth of the behavior will not be used.")

    # run the pipeline to construct the rule list
    rule_list = run_rulelist_pipeline(
        db=db,
        tag2rule_dict=tag2rule_dict,
        scn_tag=scn_tag,
        # output_path=output_path,
        # language=language,
        use_rule2behavior_gt=use_rule2behavior_gt,
    )

    # wth:在外层写
    # with open(os.path.join(Parameter.map_file, f'{scn_folder_name}', 'rule_list.json'), 'w') as f:
    #     json.dump(rule_list, f)
    # print('The rule list has been saved at:', output_path)
    return rule_list


def detect_behaviors_review(scn_folder_name: str,
                            trajectory_csv_path: str,
                            rule_list: dict,
                            # res_folder: str
                            ):
    '''
    ——————————————————————————开发思路————————————————————————————————
    1.根据已有数据，定下各种行为提取标准
    2.导入轨迹、地图
    3.定位车辆所在车道，加载检索到的交规
    4.行为提取部分
    5.评价部分(逻辑判断)
    ————————————————————————————————————————————————————————————————
    '''
    # ————————————————————————————路径输入——————————————————————
    folder_path = os.path.join(Parameter.map_file, scn_folder_name)
    # trajectory_csv_path = TRJ_DIR
    # 获取用户传入场景文件夹的名字
    entries1 = os.listdir(folder_path)
    # wth:跳过这一步检查
    # xodr_subfolders = [entry for entry in entries1 if os.path.isdir(os.path.join(folder_path, entry))]
    xodr_subfolders = entries1
    if not xodr_subfolders:
        print("警告: 请检查场景文件夹命名与xodr文件命名是否相同。")
        return None
    xodr_subfolders = [scn_folder_name]
    res_list = []
    for subfolder_name in xodr_subfolders:
        scenario_name = os.path.basename(subfolder_name)
        subfolder_path = os.path.join(Parameter.map_file, subfolder_name)
        # ！注意这里要求场景文件夹名字与xodr和xosc文件名字相同
        xodr = os.path.join(subfolder_path, f"{scenario_name}.xodr")
        map_name = scenario_name
        # wth:单个轨迹文件计算
        trajectory_path = trajectory_csv_path
        # 下面搜索对应轨迹数据 ！要求轨迹文件名与xodr文件名相同或包含xodr文件名字符
        # entries_trajectory = os.listdir(trajectory_csv_path)
        # trajectory_files = [
        #     file for file in entries_trajectory
        #     if scenario_name in file and os.path.isfile(os.path.join(trajectory_csv_path, file))
        # ]
        # if not trajectory_files:
        #     print('请检查轨迹文件命名')
        #     continue
        # else:
        #     trajectory_name = trajectory_files[0]
        #     trajectory_path = os.path.join(trajectory_csv_path, trajectory_name)

        # ————————————————————————————判断左右行场景————————————————————————————————————————————————————————

        is_lefthand = is_lefthand_scn(scn_name=scenario_name)
        if not is_lefthand:  # 如果是右行

            # ———————————————————————————数据读取——————————————————————————————————————————————————————————
            final_point = 100  # 设置初始分满分

            df = pd.read_csv(trajectory_path)
            csv.field_size_limit(500000)

            # 处理下主车车速
            df['v_ego'] = df['v_ego'] * 3.6
            # 设计一个速度对应时间帧的字典方便读取
            time_speed_only = dict(zip(df.iloc[:, 0], df['v_ego']))

            # 读取主车宽度
            ego_width = None
            try:
                ego_width = df.iloc[1, df.columns.get_loc('width_ego')]
            except:
                pass

            if not ego_width:
                ego_width = 1.66
            # 优化一下主车宽度用于后续处理
            else:
                ego_width = ego_width / 2 + 0.8

            # 设计一个只有速度的列表，速度按时间顺序排列
            speed_only = df['v_ego'].iloc[1:].tolist()
            # 设计一个位置对应时间帧的字典方便读取     {time:(x,y)}
            time_position_only = dict(zip(df.iloc[:, 0], zip(df.iloc[:, 3], df.iloc[:, 4])))
            # 设计一个航向角对应时间帧的字典方便读取
            time_yaw_only = dict(zip(df.iloc[:, 0], df['yaw_ego']))
            # 设计一个速度，加速度，航向角对应时间帧的字典用于倒车检测
            time_speed_acceleration_yaw = {time: (v, a, yaw) for time, v, a, yaw in
                                           zip(df.iloc[:, 0], df['v_ego'], df['a_ego'], df['yaw_ego'])}
            # 设计一个前轮转角和对应时间帧的字典用于变道检测
            time_rot_only = dict(zip(df.iloc[:, 0], df['rot_ego']))
            # 两帧之间的时间
            sorted_times = sorted(time_speed_only.keys())
            timestamp = sorted_times[1] - sorted_times[0]
            # 前轮转角存储
            rot_only = []
            for i in time_rot_only.keys():
                rot_only.append(time_rot_only[i])

            # 读取其他所有车辆的位置信息，格式vehicles_position_dict = {'car1':{'x':x,'y':y}}
            vehicles_position_dict = {}
            for i in range(1, len(df.columns) // 2):  # 假设每辆车有对应的 x 和 y 列
                x_col = f'x_car{i}'
                y_col = f'y_car{i}'
                vehicle_data = {}
                if x_col in df.columns and y_col in df.columns:
                    for index, row in df.iterrows():
                        time_frame = row.iloc[0]
                        x = row[x_col]
                        y = row[y_col]
                        x = float(x)
                        y = float(y)
                        time_frame = float(time_frame)
                        vehicle_data[time_frame] = {'x': x, 'y': y}
                        vehicles_position_dict[f'car{i}'] = vehicle_data
                    globals()[f'car{i}_position'] = vehicle_data

            # 转换成以主车为原点的局部坐标
            # 主车前方是y正方向，主车左侧x是正方向          存储的时候实际上是(y,x)即元组第一个数为y第二个数为x
            def global_to_local(x, y, x_origin, y_origin, theta):
                """将全局坐标转换为局部坐标"""
                # 计算相对坐标
                dx = x - x_origin
                dy = y - y_origin
                # 将角度转换为弧度
                theta_rad = theta
                x_rel = dx * np.cos(theta_rad) + dy * np.sin(theta_rad)
                y_rel = -dx * np.sin(theta_rad) + dy * np.cos(theta_rad)
                x_rel = float(x_rel)
                y_rel = float(y_rel)
                x_rel = round(x_rel, 2)
                y_rel = round(y_rel, 2)
                local_coords = (x_rel, y_rel)
                return local_coords

            # l为其他车辆总数
            l = len(vehicles_position_dict) + 1
            vehicles_position_dict_local = {}  # 格式为{car1:{time1:(y,x),time2:(y,x)},car2:{time1:(y,x)}}
            # 遍历所有车辆
            for i in range(1, l):
                if f'car{i}' in vehicles_position_dict:
                    car_position = vehicles_position_dict[f'car{i}']
                    # 遍历所有时间
                    local_position = {}
                    for j in sorted_times:
                        x = car_position[j]['x']
                        y = car_position[j]['y']
                        local_ords = global_to_local(x, y, time_position_only[j][0], time_position_only[j][1],
                                                     time_yaw_only[j])
                        local_position[j] = local_ords
                        vehicles_position_dict_local[f'car{i}'] = local_position
                    globals()[f'car{i}_local_position'] = local_position

            # 读取其他所有车辆的速度信息，vehicles_speed_dict = {'car1':{time1:speed1,time2:speed2,...}}
            vehicles_speed_dict = {}
            for i in range(1, l):
                speed_col = f'v_car{i}'
                speed_data = {}
                if speed_col in df.columns:
                    for index, row in df.iterrows():
                        time_frame = row.iloc[0]
                        speed = row[speed_col]
                        time_frame = float(time_frame)
                        speed = speed * 3.6
                        speed = round(speed, 2)
                        speed = float(speed)
                        speed_data[time_frame] = speed
                        vehicles_speed_dict[f'car{i}'] = speed_data
                    globals()[f'car{i}_speed'] = speed_data

            # 读取所有其他车辆航向角信息，vehicles_yaw_dict = {'car1':{time1:yaw1,time2:yaw2},...}
            vehicles_yaw_dict = {}
            for i in range(1, l):
                yaw_col = f'yaw_car{i}'
                yaw_data = {}
                if yaw_col in df.columns:
                    for index, row in df.iterrows():
                        time_frame = row.iloc[0]
                        yaw = row[yaw_col]
                        yaw = float(yaw)
                        time_frame = float(time_frame)
                        yaw_data[time_frame] = yaw
                        vehicles_yaw_dict[f'car{i}'] = yaw_data
                    globals()[f'car{i}_yaw'] = yaw_data

            '''
            # 计算主车横向位移
            计算相邻时刻之间的主车横向位移
            参数:
            车辆x坐标列表
            车辆y坐标列表
            headings: 车辆航向角列表(弧度)
            返回:
            local_lateral_displacements: 相邻时刻之间的局部横向位移列表
            正值：表示向左偏移
            负值：表示向右偏移
            '''

            local_lateral_displacements = {}

            for i in range(len(sorted_times) - 1):
                # 计算全局位移
                end_time = sorted_times[i + 1]
                start_time = sorted_times[i]
                dx = time_position_only[end_time][0] - time_position_only[start_time][0]
                dy = time_position_only[end_time][1] - time_position_only[start_time][1]

                # 使用上一时刻的航向角进行坐标转换
                heading = time_yaw_only[start_time]

                # 计算旋转矩阵的逆矩阵（从全局坐标转换到局部坐标）
                cos_theta = np.cos(heading)
                sin_theta = np.sin(heading)
                rotation_matrix = np.array([
                    [cos_theta, sin_theta],
                    [-sin_theta, cos_theta]
                ]).T
                # 将位移转换到局部坐标系
                local_displacement = rotation_matrix @ np.array([dx, dy])
                # 局部坐标系中的横向位移就是y分量
                lateral_displacement = local_displacement[1]
                lateral_displacement = float(lateral_displacement)
                lateral_displacement = round(lateral_displacement, 4)
                local_lateral_displacements[end_time] = lateral_displacement

            ego_lateral = [local_lateral_displacements[key] for key in sorted(local_lateral_displacements)]

            # ————————————————————————地图数据读取
            # 这个函数输入道路范围多边形和车辆的坐标，返回车辆目前所在的区域
            def point_in_lane_section(polygons, x, y):
                point = Point(x, y)
                containing_sections = []
                for lane_section_id, polygon in polygons.items():
                    if polygon.contains(point):
                        containing_sections.append(lane_section_id)
                return containing_sections

            # 存储[(section,time)]
            section_time = []
            # {time:section}
            section_time_dict = {}

            # 获取各个section的坐标范围
            section_polygon = get_lane_section_polygons(xodr)

            # 判断每个时刻在哪个section，存储在section_time和section_time_dict中
            last_section = None
            for time in sorted_times:
                x = time_position_only[time][0]
                y = time_position_only[time][1]
                containing_sections = point_in_lane_section(section_polygon, x, y)
                if containing_sections:
                    for section in containing_sections:
                        section_time.append((section, time))
                        section_time_dict[time] = section
                        last_section = section
                        break
                # else:
                #     section_time.append((last_section, time))
                #     section_time_dict[time] = last_section
                # wth:保护，不加入None
                elif last_section:
                    section_time.append((last_section, time))
                    section_time_dict[time] = last_section
                else:
                    pass

            # 将这两个按时间帧顺序排序
            section_time = sorted(section_time, key=lambda x: x[1])
            section_time_dict = {t[1]: t[0] for t in section_time}

            # 设置一个字典记录车辆每个时间段所处的区域范围
            section_behavior = {}
            for location_name, time_frame in section_time:
                if location_name not in section_behavior:
                    section_behavior[location_name] = {'开始时间': time_frame, '结束时间': time_frame}
                else:
                    section_behavior[location_name]['结束时间'] = time_frame
            # 格式{Road1:{'开始时间'：'','结束时间':''}，Road2:{...}}

            '''
            创建匹配所用的字典。键为所有经过的section，值为每个section对应的行为列表。
            ！每个经过的section都有一个列表，列表内每个元素都代表一个行为，且为0-1变量，若在这个section内有这个行为为1，没有为0
            目前一共是37个行为，所以设37个元素，元素初始都为0，若下面检测到了则修改成1
            '''
            behaviors_dict = {key: [0] * 37 for key in section_behavior.keys()}

            # 记录主车经过的section
            car_section_list = []
            for i in section_behavior:
                if i.startswith('Road'):
                    car_section_list.append(i)
            # 格式：[Road1,Road2]

            # ——————————————————————————计算道路标线坐标范围并测是否有压实线的行为————————————————————————————————————
            polygons_road_mark = parse_road_marks(xodr, road_mark_width=ego_width)
            start_time = None
            count_road_mark = 0
            previous_road_mark = None
            current_road_mark = None
            in_mark = False
            for time in sorted_times:
                x = time_position_only[time][0]
                y = time_position_only[time][1]
                containing_sections_road_mark = point_in_lane_section(polygons_road_mark, x, y)
                # 这里返回lane_section_id，如果不在Road_mark内则是空列表
                if containing_sections_road_mark:
                    current_road_mark = containing_sections_road_mark[0]
                    if not in_mark:
                        start_time = time
                        in_mark = True
                    if in_mark:
                        if current_road_mark == previous_road_mark:
                            count_road_mark += timestamp
                            if count_road_mark > 0.5:  # 如果压实线超过0.5s
                                for tup in section_time:
                                    if start_time == tup[1]:
                                        section = tup[0]
                                        behaviors_dict[section][9] = 1
                                        section_behavior[section]['9'] = start_time
                                count_road_mark = 0
                                start_time = None
                        else:
                            count_road_mark = 0
                    previous_road_mark = current_road_mark

            # ——————————————————————————计算专用车道坐标范围并检测是否有驶入专用车道的行为
            special_lane_polygons = parse_special_lanes(xodr)
            if len(special_lane_polygons) != 0:
                for time in sorted_times:
                    x = time_position_only[time][0]
                    y = time_position_only[time][1]
                    containing_sections_special = point_in_lane_section(special_lane_polygons, x, y)
                    if containing_sections_special:
                        section = section_time_dict[time]
                        behaviors_dict[section][14] = 1

            # ———————————————————————————加载Json文件，读取交规————————————————————————————————————————————

            section_rules_retrieved_dict = {}
            # wth:取消读文件
            # with open(os.path.join(CACHE_DIR, rule_list_fn), 'r', encoding='utf-8') as f:
            #     data = json.load(f)
            # 提取 "交规" 项，格式：section_rules_retrieved_dict = {section:'[content1,content2]'}
            if map_name in rule_list:
                for i in car_section_list:
                    map_section = rule_list[map_name]
                    for key in map_section:
                        if key.startswith(i):
                            section_rules_retrieved_dict[i] = map_section[key]["rules"]

            # 创建一个字典存储交规编号  格式：section_code_dict = {section1:[code1,code2],section2:[]}
            section_code_dict = {}
            for section in behaviors_dict:
                if section not in section_code_dict:
                    section_code_dict[section] = None
            for section in section_rules_retrieved_dict:
                code_list = []
                content_list = section_rules_retrieved_dict[section]
                for content in content_list:
                    content = content.replace('\r\n', '\n')
                    code = content_dict[content]
                    code_list.append(code)
                section_code_dict[section] = code_list

            # ——————————————————————————————————————————————————————————————————————————————————————
            # ——————————————————————————行为提取部分——————————————————————————————————————————————————————————————
            # 最终结果存储在section_behavior中
            # 停车模块停车位置存储在一个时间帧为键，其对应xy坐标为值的字典
            parking_position = {}

            # ——————————————————————————减速模块————————————————————————————————————
            previous_time_deceleration = None
            previous_speed_deceleration = None
            deceleration_start_time = None  # 用于标记减速开始的时间
            cumulative_time = 0  # 连续减速的时间帧数阈值(1s)
            for time in sorted_times:
                current_speed_deceleration = time_speed_only[time]
                if previous_time_deceleration is not None and previous_time_deceleration != 0:
                    if current_speed_deceleration <= previous_speed_deceleration:
                        # 当前速度小于或等于前一个速度，车辆正在减速
                        cumulative_time += timestamp
                        if deceleration_start_time is None:
                            # 记录减速开始时间
                            deceleration_start_time = time
                    else:
                        # 当前速度大于前一个速度，检查是否满足连续阈值减速
                        if cumulative_time >= 1:
                            # 满足条件，输出减速的起止时间
                            for tup in section_time:
                                if deceleration_start_time == tup[1]:
                                    section = tup[0]
                                    behaviors_dict[section][0] = 1
                                    section_behavior[section]['0'] = deceleration_start_time
                        # 重置减速计数器和开始时间
                        cumulative_time = 0
                        deceleration_start_time = None
                previous_time_deceleration = time
                previous_speed_deceleration = current_speed_deceleration
            # 循环结束后，检查是否有未输出的减速区间
            if cumulative_time >= 1:
                for tup in section_time:
                    if deceleration_start_time == tup[1]:
                        section = tup[0]
                        behaviors_dict[section][0] = 1
                        section_behavior[section]['0'] = deceleration_start_time

            # ———————————————————————————最大车速模块———————————

            # 遍历所有时间
            for time in sorted_times:
                if time in time_speed_only:
                    speed = time_speed_only[time]
                    section = section_time_dict.get(time)
                    # wth：section不存在时的兼容处理
                    if not section:
                        continue
                    if 144 <= speed:
                        behaviors_dict[section][18] = 1
                        behaviors_dict[section][19] = 1
                        behaviors_dict[section][20] = 1
                        behaviors_dict[section][21] = 1
                        behaviors_dict[section][22] = 1
                        behaviors_dict[section][23] = 1
                        behaviors_dict[section][24] = 1
                        behaviors_dict[section][25] = 1
                        section_behavior[section]['25'] = time
                        section_behavior[section]['24'] = time
                        section_behavior[section]['23'] = time
                        section_behavior[section]['22'] = time
                        section_behavior[section]['21'] = time
                        section_behavior[section]['20'] = time
                        section_behavior[section]['19'] = time
                        section_behavior[section]['18'] = time
                    elif 100 < speed < 144:
                        behaviors_dict[section][18] = 1
                        behaviors_dict[section][19] = 1
                        behaviors_dict[section][20] = 1
                        behaviors_dict[section][21] = 1
                        behaviors_dict[section][22] = 1
                        behaviors_dict[section][23] = 1
                        behaviors_dict[section][24] = 1
                        section_behavior[section]['24'] = time
                        section_behavior[section]['23'] = time
                        section_behavior[section]['22'] = time
                        section_behavior[section]['21'] = time
                        section_behavior[section]['20'] = time
                        section_behavior[section]['19'] = time
                        section_behavior[section]['18'] = time
                    elif 70 < speed < 100:
                        behaviors_dict[section][18] = 1
                        behaviors_dict[section][19] = 1
                        behaviors_dict[section][20] = 1
                        behaviors_dict[section][21] = 1
                        behaviors_dict[section][22] = 1
                        behaviors_dict[section][23] = 1
                        section_behavior[section]['23'] = time
                        section_behavior[section]['22'] = time
                        section_behavior[section]['21'] = time
                        section_behavior[section]['20'] = time
                        section_behavior[section]['19'] = time
                        section_behavior[section]['18'] = time
                    elif 60 < speed < 70:
                        behaviors_dict[section][18] = 1
                        behaviors_dict[section][19] = 1
                        behaviors_dict[section][20] = 1
                        behaviors_dict[section][21] = 1
                        behaviors_dict[section][22] = 1
                        section_behavior[section]['22'] = time
                        section_behavior[section]['21'] = time
                        section_behavior[section]['20'] = time
                        section_behavior[section]['19'] = time
                        section_behavior[section]['18'] = time
                    elif 50 < speed < 60:
                        behaviors_dict[section][18] = 1
                        behaviors_dict[section][19] = 1
                        behaviors_dict[section][20] = 1
                        behaviors_dict[section][21] = 1
                        section_behavior[section]['21'] = time
                        section_behavior[section]['20'] = time
                        section_behavior[section]['19'] = time
                        section_behavior[section]['18'] = time
                    elif 40 < speed < 50:
                        behaviors_dict[section][18] = 1
                        behaviors_dict[section][19] = 1
                        behaviors_dict[section][20] = 1
                        section_behavior[section]['20'] = time
                        section_behavior[section]['19'] = time
                        section_behavior[section]['18'] = time
                    elif 30 < speed < 40:
                        behaviors_dict[section][18] = 1
                        behaviors_dict[section][19] = 1
                        section_behavior[section]['19'] = time
                        section_behavior[section]['18'] = time
                    elif 20 < speed < 30:
                        behaviors_dict[section][18] = 1
                        section_behavior[section]['18'] = time

            # ————————————————————————————————————————————————————————————————————————————————————————

            # ——————————————————————————————————————————停车———————————————————————————————————————————

            previous_time_parking = None
            previous_speed_parking = 0
            for time in sorted_times:
                current_speed_parking = time_speed_only.get(time)
                if previous_speed_parking != 0 and current_speed_parking == 0:
                    # 接下来10个时间帧内都保持为0
                    next_time_frames = list(sorted(time_speed_only.keys()))[
                                       list(time_speed_only.keys()).index(time) + 1:
                                       min(list(time_speed_only.keys()).index(time) + 10, len(time_speed_only))]
                    for next_time in next_time_frames:
                        if time_speed_only[next_time] == 0:
                            # 记录停车位置
                            current_position_parking = time_position_only[time]
                            parking_position[time] = current_position_parking
                            for tup in section_time:
                                if time == tup[1]:
                                    section = tup[0]
                                    behaviors_dict[section][1] = 1
                                    section_behavior[section]['1'] = time
                previous_time_parking = time
                previous_speed_parking = current_speed_parking

            # ————————————————————————————————————————————————————
            # ————————————————————————————————————————转弯和掉头——————————————————————————————

            # 设定阈值和变量   目前转弯阈值为30°-150°
            start_time = None
            cumulative_delta_yaw = 0
            turn_direction = None
            turn_start_time = None
            turn_end_time = None
            last_time = None
            last_yaw_turn = None
            last_delta_yaw = 0  # 上一次的航向角改变量
            turns = []
            # 遍历字典，按时间帧顺序
            for time_frame in sorted_times[1:]:  # 从第二个时间帧开始,sorted返回以时间帧排序的列表
                current_yaw_turn = time_yaw_only[time_frame]
                if last_yaw_turn is not None:
                    current_delta_yaw = current_yaw_turn - last_yaw_turn  # 当前航向角的改变量
                    if current_delta_yaw != 0:  # 如果航向角发生变化
                        # 如果是同一方向的变化，累计改变量
                        if current_delta_yaw > 0 and last_delta_yaw >= 0:
                            cumulative_delta_yaw += current_delta_yaw
                            if start_time is None:  # 记录转弯开始时的时间帧
                                start_time = last_time  # 使用上一个时间帧
                        elif current_delta_yaw < 0 and last_delta_yaw <= 0:
                            cumulative_delta_yaw += current_delta_yaw
                            if start_time is None:  # 记录转弯开始时的时间帧
                                start_time = last_time  # 使用上一个时间帧
                        else:  # 方向改变，停止累计
                            # 判断是否满足转弯条件
                            if 0.52 <= abs(cumulative_delta_yaw) <= 2.9:
                                next_time_frames = list(sorted(time_yaw_only.keys()))[
                                                   list(time_yaw_only.keys()).index(time_frame) + 1:
                                                   min(list(time_yaw_only.keys()).index(time_frame) + 11,
                                                       len(time_yaw_only))]
                                for next_time in next_time_frames:
                                    next_change = abs(time_yaw_only[next_time] - time_yaw_only[last_time])
                                    if next_change > 0.6:
                                        break
                                else:
                                    # 满足条件，记录转弯信息
                                    turn_direction = '右转' if cumulative_delta_yaw < 0 else '左转'
                                    turn_start_time = start_time
                                    turn_end_time = time_frame
                                    for tup in section_time:
                                        if turn_start_time == tup[1]:
                                            section = tup[0]
                                            behaviors_dict[section][2] = 1
                                            turns.append((turn_start_time, turn_end_time, turn_direction))
                                            section_behavior[section]['2'] = turn_start_time
                            # 判断是否满足掉头条件
                            elif 2.9 < abs(cumulative_delta_yaw) < 4.714:
                                next_time_frames = list(sorted(time_yaw_only.keys()))[
                                                   list(time_yaw_only.keys()).index(time_frame) + 1:
                                                   min(list(time_yaw_only.keys()).index(time_frame) + 11,
                                                       len(time_yaw_only))]
                                for next_time in next_time_frames:
                                    next_change = abs(time_yaw_only[next_time] - time_yaw_only[last_time])
                                    if next_change > 0.6:
                                        break
                                else:
                                    u_turn_start_time = start_time
                                    for tup in section_time:
                                        if u_turn_start_time == tup[1]:
                                            section = tup[0]
                                            behaviors_dict[section][3] = 1
                                            section_behavior[section]['3'] = u_turn_start_time
                            start_time = None
                            # 重置累计值
                            cumulative_delta_yaw = 0
                    last_delta_yaw = current_delta_yaw
                last_yaw_turn = current_yaw_turn
                last_time = time_frame

            if start_time is not None and 0.52 <= abs(cumulative_delta_yaw) <= 2.9:
                turn_direction = '右转' if cumulative_delta_yaw < 0 else '左转'
                turn_start_time = start_time
                turn_end_time = sorted_times[-1]
                for tup in section_time:
                    if turn_start_time == tup[1]:
                        section = tup[0]
                        behaviors_dict[section][2] = 1
                        turns.append((turn_start_time, turn_end_time, turn_direction))
                        section_behavior[section]['2'] = turn_start_time

            # ———————————————————————————————————————————————————————————————————————
            # ————————————————————————————————倒车————————————————————————————————

            reversing_start_time = None
            total_frames = len(time_speed_acceleration_yaw)
            valid = False
            reverse_count = 0
            # 遍历字典中的每一帧
            for time, (speed, acceleration, yaw) in time_speed_acceleration_yaw.items():
                # 速度减小到零
                if speed == 0:
                    # 判断加速度是否为负
                    if acceleration < 0:
                        reverse_count += timestamp
                        if reverse_count > 1:
                            reversing_start_time = time
                            # 检查后续10帧是否存在,航向角变化不超过0.6
                            frame = sorted_times.index(time)  # 返回目前是第几帧
                            remaining_frames = min(10, total_frames - frame - 1)  # 后面几帧
                            valid = True
                            for i in range(1, remaining_frames + 1):
                                next_frame = frame + i
                                next_time = sorted_times[next_frame]
                                if next_time not in time_speed_acceleration_yaw:
                                    valid = False
                                    break
                                next_yaw = time_speed_acceleration_yaw[next_time][2]
                                if next_yaw is None or abs(next_yaw - yaw) > 0.6:
                                    valid = False
                                    break
                            if valid:
                                section = section_time_dict[reversing_start_time]
                                behaviors_dict[section][4] = 1
                                section_behavior[section]['4'] = reversing_start_time
                            reverse_count = 0

            # ————————————————————————————————————————————————————————————————————————————————
            # ———————————————————————————————————————变道————————————————————————————————————————

            df_lane_change = df
            df_lane_change['delta_yaw'] = df_lane_change['yaw_ego'].diff()
            # 添加时间差（秒）列
            df_lane_change['delta_time'] = df_lane_change.iloc[:, 0].diff().fillna(0)
            # 变道相关变量
            lane_change_accumulated_yaw_change = 0
            lane_change_in_continuous_change = False  # 判断是否正在持续变化
            lane_change_start_recording = False
            lane_change_direction = None
            lane_change_initial_time = None
            lane_change_initial_yaw = None
            lane_change_data_row_count = 0
            lane_changes = []

            # 变道判定阈值
            lane_change_start_threshold = 0.001  # 开始判定变道的改变量阈值（度）
            lane_change_accumulated_threshold_min = 0.069  # 变道累计变化最小阈值（度）
            lane_change_accumulated_threshold_max = 0.52  # 变道累计变化最大阈值（度）
            lane_change_data_row_threshold = 10  # 变道持续的数据行数阈值
            lane_change_yaw_difference_threshold = 0.069  # 变道后航向角差值阈值（度）
            lane_change_next_data_rows = 15  # 11-(15)变道后检测的数据行数
            all_lateral = 0

            for idx, row in df_lane_change.iterrows():
                delta_yaw = row['delta_yaw']
                current_yaw = row['yaw_ego']
                current_time = row.iloc[0]
                delta_time = row['delta_time']
                # 判断航向角变化的方向
                if delta_yaw > 0:
                    current_direction = 'positive'
                elif delta_yaw < 0:
                    current_direction = 'negative'
                else:
                    current_direction = 'zero'
                if not lane_change_in_continuous_change:
                    if abs(delta_yaw) >= lane_change_start_threshold:
                        # 开始持续变化
                        lane_change_in_continuous_change = True
                        lane_change_direction = current_direction
                        lane_change_accumulated_yaw_change = delta_yaw
                        lane_change_initial_time = current_time  # 使用实际时间戳
                        lane_change_initial_yaw = current_yaw
                        lane_change_data_row_count = 1
                    else:
                        # 未达到变道检测的起始阈值，继续
                        pass
                else:
                    if current_direction == lane_change_direction or current_direction == 'zero':
                        # 持续同方向变化或未变化
                        lane_change_accumulated_yaw_change += delta_yaw
                        lane_change_data_row_count += 1
                    else:
                        # 方向改变，停止累计
                        lane_change_end_time = current_time
                        if (lane_change_accumulated_threshold_min < abs(
                                lane_change_accumulated_yaw_change) < lane_change_accumulated_threshold_max and
                                lane_change_data_row_count >= lane_change_data_row_threshold):
                            # 满足变道条件，开始进一步判断
                            # 检查后11-15行数据的航向角是否与最初开始改变的航向角相差不超过0.07°
                            end_idx = min(idx + lane_change_next_data_rows, len(df_lane_change) - 1)
                            future_yaws = df_lane_change.loc[idx + 10:end_idx, 'yaw_ego']
                            yaw_differences = future_yaws - lane_change_initial_yaw
                            max_yaw_difference = yaw_differences.abs().max()
                            if abs(max_yaw_difference) <= lane_change_yaw_difference_threshold or abs(
                                    max_yaw_difference) > 6.2:
                                idx1 = sorted_times.index(lane_change_initial_time)
                                idx2 = sorted_times.index(lane_change_end_time)
                                for lateral in ego_lateral[idx1:idx2]:
                                    all_lateral += lateral
                                if abs(all_lateral) >= 2.5:
                                    all_lateral = 0
                                    # 判定为变道
                                    if lane_change_accumulated_yaw_change > 0:
                                        lane_change_direction_str = '向左变道'

                                    elif lane_change_accumulated_yaw_change < 0:
                                        lane_change_direction_str = '向右变道'

                                    else:
                                        lane_change_direction_str = '变道方向未知'
                                    for tup in section_time:
                                        if lane_change_initial_time == tup[1]:
                                            section = tup[0]
                                            behaviors_dict[section][5] = 1
                                            lane_change_initial_time = float(lane_change_initial_time)
                                            section_behavior[section]['5'] = lane_change_initial_time
                                            lane_changes.append(
                                                (lane_change_initial_time, lane_change_end_time,
                                                 lane_change_direction_str))

                        elif (6.21 < abs(lane_change_accumulated_yaw_change) and
                              lane_change_data_row_count >= lane_change_data_row_threshold):
                            # 满足变道条件，开始进一步判断
                            # 检查后11-15行数据的航向角是否与最初开始改变的航向角相差不超过0.07°
                            end_idx = min(idx + lane_change_next_data_rows, len(df_lane_change) - 1)
                            future_yaws = df_lane_change.loc[idx + 10:end_idx, 'yaw_ego']
                            yaw_differences = future_yaws - lane_change_initial_yaw
                            max_yaw_difference = yaw_differences.abs().max()
                            if abs(max_yaw_difference) <= lane_change_yaw_difference_threshold or abs(
                                    max_yaw_difference) > 6.21:
                                idx1 = sorted_times.index(lane_change_initial_time)
                                idx2 = sorted_times.index(lane_change_end_time)
                                for lateral in ego_lateral[idx1:idx2]:
                                    all_lateral += lateral
                                if all_lateral >= 2.5:
                                    all_lateral = 0
                                    # 判定为变道
                                    if lane_change_accumulated_yaw_change > 0:
                                        lane_change_direction_str = '向左变道'

                                    elif lane_change_accumulated_yaw_change < 0:
                                        lane_change_direction_str = '向右变道'
                                        section = section_time_dict[lane_change_initial_time]
                                        behaviors_dict[section][34] = 1
                                    else:
                                        lane_change_direction_str = '变道方向未知'
                                    for tup in section_time:
                                        if lane_change_initial_time == tup[1]:
                                            section = tup[0]
                                            behaviors_dict[section][5] = 1
                                            lane_change_initial_time = float(lane_change_initial_time)
                                            section_behavior[section]['5'] = lane_change_initial_time
                                            lane_changes.append(
                                                (lane_change_initial_time, lane_change_end_time,
                                                 lane_change_direction_str))

                        # 重置变道相关变量
                        lane_change_in_continuous_change = False
                        lane_change_accumulated_yaw_change = 0
                        lane_change_direction = None
                        lane_change_initial_time = None
                        lane_change_initial_yaw = None
                        lane_change_data_row_count = 0

            # ———————————————————————————————————————————————————————————————————————————————
            # ———————————————————————————————————————超车————————————————————————————————————————
            # 检测是否有两次换道行为时间帧在阈值以内
            overtakes = []
            time_window = 5  # 两次换道时间窗5s
            if len(lane_changes) >= 2:
                for i in range(len(lane_changes) - 1):
                    first_change = lane_changes[i]
                    second_change = lane_changes[i + 1]
                    # 判断两次换道方向是否相反，且在指定时间窗口内
                    if first_change[2] != second_change[2]:
                        if first_change[2] == '向左变道':
                            time_diff = second_change[1] - first_change[0]
                            if time_diff <= time_window:
                                # 满足条件，开始超车检测
                                start_time = first_change[0]
                                end_time = second_change[1]
                                # 遍历其他所有车辆
                                for j in vehicles_yaw_dict:
                                    try:
                                        yaw1 = vehicles_yaw_dict[j][start_time]
                                        yaw2 = time_yaw_only[start_time]
                                        yaw_difference1 = abs(yaw1 - yaw2)
                                        # 判断其他车辆与主车航向角相差是否小于0.5°
                                        if yaw_difference1 <= 0.5:
                                            try:

                                                start_y = vehicles_position_dict_local[j][start_time][0]
                                                end_y = vehicles_position_dict_local[j][end_time][0]
                                                # 判断其他车辆相对位置y是否由正到负
                                                if start_y > 0 and end_y < 0:
                                                    overtakes.append((start_time, end_time))
                                                    for tup in section_time:
                                                        if start_time == tup[1]:
                                                            section = tup[0]
                                                            pass_time = start_time
                                                            pass_time = float(pass_time)
                                                            behaviors_dict[section][7] = 1
                                                            section_behavior[section]['7'] = start_time
                                                    break
                                            except KeyError as e:
                                                continue
                                    except KeyError as e:
                                        continue

                        elif first_change[2] == '向右变道':
                            time_diff = second_change[1] - first_change[0]
                            if time_diff <= time_window:
                                # 满足条件，开始超车检测
                                start_time = first_change[0]
                                end_time = second_change[1]
                                # 遍历其他所有车辆
                                for j in vehicles_yaw_dict:
                                    try:
                                        yaw1 = vehicles_yaw_dict[j][start_time]
                                        yaw2 = time_yaw_only[start_time]
                                        yaw_difference1 = abs(yaw1 - yaw2)
                                        # 判断其他车辆与主车航向角相差是否小于30°
                                        if yaw_difference1 <= 0.5:
                                            try:
                                                start_y = vehicles_position_dict_local[j][start_time][0]
                                                end_y = vehicles_position_dict_local[j][end_time][0]
                                                # 判断其他车辆相对位置y是否由正到负
                                                if start_y > 0 and end_y < 0:
                                                    overtakes.append((start_time, end_time))
                                                    for tup in section_time:
                                                        if start_time == tup[1]:
                                                            section = tup[0]
                                                            pass_time = start_time
                                                            pass_time = float(pass_time)
                                                            behaviors_dict[section][7] = 1
                                                            section_behavior[section]['7'] = start_time
                                                    break
                                            except KeyError as e:
                                                continue
                                    except KeyError as e:
                                        continue

            # ——————————————————————————————————————前方车辆停车排队时超车————————————————————————————————————
            # 判断有无超车
            pre_car_count1 = 0
            if overtakes:
                # 遍历所有超车
                for overtake in overtakes:
                    # 遍历所有时间
                    for time in sorted_times:
                        # 在每个时间遍历其它所有车辆判断是否在前面且车速小于每小时3.6公里
                        for car in vehicles_position_dict_local:
                            if time in vehicles_position_dict_local[car]:
                                x = vehicles_position_dict_local[car][time][1]
                                y = vehicles_position_dict_local[car][time][0]
                                if 0 < y < 20:
                                    if abs(x) < 2:
                                        if car in vehicles_speed_dict:
                                            if time in vehicles_speed_dict[car]:
                                                if vehicles_speed_dict[car][time] <= 3.6:
                                                    pre_car_count1 += 1
                        # 判断是否有两辆车以上在前20m处
                        if pre_car_count1 >= 2:
                            if overtake[0] < time < overtake[1]:
                                for tup in section_time:
                                    if time == tup[1]:
                                        section = tup[0]
                                        behaviors_dict[section][31] = 1
                                        section_behavior[section]['31'] = time
                                        break
                        pre_car_count1 = 0
            # ——————————————————————————————————————前方车辆缓慢行驶时超车————————————————————————————————————
            # 判断有无超车
            pre_car_count2 = 0
            if overtakes:
                # 遍历所有超车
                for overtake in overtakes:
                    # 遍历所有时间
                    for time in sorted_times:
                        # 在每个时间遍历其它所有车辆判断是否在前面且车速小于每小时10公里
                        for car in vehicles_position_dict_local:
                            if time in vehicles_position_dict_local[car]:
                                x = vehicles_position_dict_local[car][time][1]
                                y = vehicles_position_dict_local[car][time][0]
                                if abs(x) < 2:
                                    if 0 < y < 20:
                                        if car in vehicles_speed_dict:
                                            if time in vehicles_speed_dict[car]:
                                                if vehicles_speed_dict[car][time] <= 20:
                                                    pre_car_count2 += 1
                        # 判断是否有两辆车以上在前20m处
                        if pre_car_count2 >= 2:
                            if overtake[0] < time < overtake[1]:
                                for tup in section_time:
                                    if time == tup[1]:
                                        section = tup[0]
                                        behaviors_dict[section][32] = 1
                                        section_behavior[section]['32'] = time
                                        break
                        pre_car_count2 = 0
            # ——————————————————————————————————————前车停车排队——————————————————————————————
            # 遍历所有时间
            for time in sorted_times:
                # 在每个时间遍历其它所有车辆判断是否在前面且车速小于每小时3.6公里
                for car in vehicles_position_dict_local:
                    if time in vehicles_position_dict_local[car]:
                        x = vehicles_position_dict_local[car][time][1]
                        y = vehicles_position_dict_local[car][time][0]
                        if 0 < y < 20:
                            if abs(x) < 2:
                                if car in vehicles_speed_dict:
                                    if time in vehicles_speed_dict[car]:
                                        if vehicles_speed_dict[car][time] <= 3.6:
                                            pre_car_count2 += 1
                # 判断是否有两辆车以上在前20m处
                if pre_car_count2 >= 2:
                    for tup in section_time:
                        if time == tup[1]:
                            section = tup[0]
                            behaviors_dict[section][26] = 1
                            section_behavior[section]['26'] = time
                            break
                pre_car_count2 = 0
            # ——————————————————————————————————————前车缓慢行驶——————————————————————————————

            # 遍历所有时间
            for time in sorted_times:
                # 在每个时间遍历其它所有车辆判断是否在前面且车速小于每小时20公里
                for car in vehicles_position_dict_local:
                    if time in vehicles_position_dict_local[car]:
                        x = vehicles_position_dict_local[car][time][1]
                        y = vehicles_position_dict_local[car][time][0]
                        if 0 < y < 20:
                            if abs(x) < 2:
                                if car in vehicles_speed_dict:
                                    if time in vehicles_speed_dict[car]:
                                        if vehicles_speed_dict[car][time] <= 20:
                                            pre_car_count2 += 1
                # 判断是否有两辆车以上在前20m处
                if pre_car_count2 >= 2:
                    for tup in section_time:
                        if time == tup[1]:
                            section = tup[0]
                            behaviors_dict[section][27] = 1
                            section_behavior[section]['27'] = time
                            break
                pre_car_count2 = 0

            # ————————————————————————————————————会车————————————————————————————————————

            for q in car_section_list:
                meeting_start_time = None
                meeting_time = []
                # 遍历所有车辆
                for i in range(1, l):
                    try:
                        car_position_local = vehicles_position_dict_local[f'car{i}']
                        car_yaw_local = vehicles_yaw_dict[f'car{i}']
                        # 遍历所有时间
                        for j in sorted_times[:-1]:
                            # 如果横向相对距离小于4大于2
                            if 2 < abs(car_position_local[j][1]) < 4:
                                # 如果航向角大于150°
                                if 4 > abs(time_yaw_only[j] - car_yaw_local[j]) > 2.62:
                                    # 相对距离y由正到负
                                    if 0 < car_position_local[j][0] < 5:
                                        meeting_start_time = j
                                        meeting_end_time = min(sorted_times[-1], j + 2)
                                        if car_position_local[meeting_end_time][0] < 0:
                                            meeting_time = (meeting_start_time, meeting_end_time)
                                            for tup in section_time:
                                                if meeting_start_time == tup[1]:
                                                    section = tup[0]
                                                    behaviors_dict[section][8] = 1
                                                    section_behavior[section]['8'] = time
                                            break
                    except KeyError as e:
                        continue

            # ——————————————————————————————————————————————————————————————————————————————————

            # ————————————————————————————————————————跟车距离————————————————————————————————————

            # 遍历锁车其他车辆
            follow_count = 0
            for car in vehicles_position_dict_local:
                # 遍历所有时间
                try:
                    for time in sorted_times:
                        if time in vehicles_position_dict_local[car]:
                            # 获取local x y:
                            x = vehicles_position_dict_local[car][time][1]
                            y = vehicles_position_dict_local[car][time][0]
                            # 判断是否在同车道:
                            if abs(x) <= 1.99:
                                # 小于50米
                                if 0 < y < 50:
                                    follow_count += 1
                                    if follow_count >= 5:
                                        follow_count = 0
                                        for tup in section_time:
                                            if time == tup[1]:
                                                section = tup[0]
                                                time = float(time)
                                                behaviors_dict[section][16] = 1
                                                section_behavior[section]['16'] = time
                                                break
                                        break
                                # 判断是否距离小于100米
                                elif 0 < y < 100:
                                    if time_speed_only[time] > 100:
                                        follow_count += 1
                                        if follow_count >= 5:
                                            follow_count = 0
                                            for tup in section_time:
                                                if time == tup[1]:
                                                    section = tup[0]
                                                    time = float(time)
                                                    behaviors_dict[section][17] = 1
                                                    section_behavior[section]['17'] = time
                                                    break
                                            break

                except KeyError as e:
                    continue
            # ————————————————————————————————————变更车道时影响相关车道内行驶的机动车的正常行驶——————————————————————————————————————

            # 遍历所有车辆
            for car in vehicles_position_dict_local:
                for time in sorted_times:
                    if time in vehicles_position_dict_local[car]:
                        x = vehicles_position_dict_local[car][time][1]
                        y = vehicles_position_dict_local[car][time][0]
                        # 判断是否在相邻车道
                        if -5 < x < -2:
                            if -5 < y < 0:
                                if lane_changes:
                                    for change in lane_changes:
                                        if change[0] < time < change[1]:
                                            if change[2] == '向右变道':
                                                for tup in section_time:
                                                    if time == tup[1]:
                                                        section = tup[0]
                                                        behaviors_dict[section][30] = 1
                                                        section_behavior[section]['30'] = time
                                                        break
                                                break
                        elif 2 < x < 5:
                            if -5 < y < 0:
                                if lane_changes:
                                    for change in lane_changes:
                                        if change[0] < time < change[1]:
                                            if change[2] == '向左变道':
                                                for tup in section_time:
                                                    if time == tup[1]:
                                                        section = tup[0]
                                                        behaviors_dict[section][30] = 1
                                                        section_behavior[section]['30'] = time
                                                        break
                                                break

            # ——————————————————————————————————————未避让其它车辆——————————————————————————————————————

            # 遍历所有车辆
            Begining0 = False
            for car in vehicles_position_dict_local:
                Begining0 = True
                for time in sorted_times:
                    if Begining0:
                        if time in vehicles_position_dict_local[car]:
                            x = vehicles_position_dict_local[car][time][1]
                            y = vehicles_position_dict_local[car][time][0]
                            # 车辆是否在前面
                            if 0 < y < 5:
                                # 车辆是否在附近
                                if abs(x) < 4:
                                    # 航向角相差小于150°
                                    if abs(vehicles_yaw_dict[car][time] - time_yaw_only[time]) < 2.6 or abs(
                                            vehicles_yaw_dict[car][time] - time_yaw_only[time]) > 4:
                                        if time_speed_only[time] >= 10:
                                            for tup in section_time:
                                                Begining0 = False
                                                if time == tup[1]:
                                                    section = tup[0]
                                                    behaviors_dict[section][11] = 1
                                                    section_behavior[section]['11'] = time
                                                    break

            Begining1 = False
            # 左侧来车
            for car in vehicles_position_dict_local:
                Begining1 = True
                for time in sorted_times:
                    if Begining1:
                        if time in vehicles_position_dict_local[car]:
                            x = vehicles_position_dict_local[car][time][1]
                            y = vehicles_position_dict_local[car][time][0]
                            # 车辆是否在前面
                            if -1 < y < 3.9:
                                # 车辆是否在附近
                                if 0 < x < 3.5:
                                    if abs(vehicles_yaw_dict[car][time] - time_yaw_only[time]) < 2.6 or abs(
                                            vehicles_yaw_dict[car][time] - time_yaw_only[time]) > 4:
                                        if abs(vehicles_yaw_dict[car][time] - time_yaw_only[time]) > 0.3:
                                            if time_speed_only[time] >= 10:
                                                for tup in section_time:
                                                    Begining1 = False
                                                    if time == tup[1]:
                                                        section = tup[0]
                                                        behaviors_dict[section][36] = 1
                                                        section_behavior[section]['36'] = time
                                                        break

            Begining2 = False
            # 右方来车
            # 遍历所有车辆
            for car in vehicles_position_dict_local:
                Begining2 = True
                for time in sorted_times:
                    if Begining2:
                        if time in vehicles_position_dict_local[car]:
                            x = vehicles_position_dict_local[car][time][1]
                            y = vehicles_position_dict_local[car][time][0]
                            # 车辆是否在前面
                            if 0 < y < 5:
                                # 车辆是否在附近
                                if -4 < x < 0:
                                    if abs(vehicles_yaw_dict[car][time] - time_yaw_only[time]) < 2.09 or abs(
                                            vehicles_yaw_dict[car][time] - time_yaw_only[time]) > 4:
                                        if abs(vehicles_yaw_dict[car][time] - time_yaw_only[time]) > 0.52:
                                            if time_speed_only[time] >= 10:
                                                for tup in section_time:
                                                    Begining2 = False
                                                    if time == tup[1]:
                                                        section = tup[0]
                                                        behaviors_dict[section][33] = 1
                                                        section_behavior[section]['33'] = time
                                                        break

            # ————————————————————————————————————————转弯时未避让直行车辆————————————————————————————————————
            nearing = False
            nearing_end_time = None
            nearings = {}
            # 遍历所有车辆
            for car in vehicles_position_dict_local:
                nearing = False
                for time in sorted_times:
                    if time in vehicles_position_dict_local[car]:
                        x = vehicles_position_dict_local[car][time][1]
                        y = vehicles_position_dict_local[car][time][0]
                        # 车辆是否在前面
                        if 0 < y < 6:
                            # 车辆是否在附近
                            if abs(x) < 4:
                                if not nearing:
                                    nearing = True
                                    nearing_start_time = time
                                    nearing_end_time = time
                                else:
                                    nearing_end_time = time
                                nearings[car] = (nearing_start_time, nearing_end_time)

            if turns:
                for turn in turns:
                    turn_start_time = turn[0]
                    turn_end_time = turn[1]
                    if nearings:
                        for car in nearings:
                            nearing_start_time = nearings[car][0]
                            nearing_end_time = nearings[car][1]
                            vehicle_yaw1 = vehicles_yaw_dict[car][nearing_start_time]
                            vehicle_yaw2 = vehicles_yaw_dict[car][nearing_end_time]
                            if abs(vehicle_yaw1 - vehicle_yaw2) < 0.1:
                                if turn_start_time < nearing_start_time < turn_end_time:
                                    idx1 = sorted_times.index(nearing_start_time)
                                    idx2 = sorted_times.index(nearing_end_time)
                                    if time_speed_only[nearing_start_time] > 10:
                                        section = section_time_dict[nearing_start_time]
                                        behaviors_dict[section][35] = 1
                                        section_behavior[section]['35'] = nearing_start_time
                                        break

            if turns:
                for turn in turns:
                    if turn[2] == '右转':
                        turn_start_time = turn[0]
                        turn_end_time = turn[1]
                        if nearings:
                            for car in nearings:
                                nearing_start_time = nearings[car][0]
                                nearing_end_time = nearings[car][1]
                                vehicle_yaw1 = vehicles_yaw_dict[car][nearing_start_time]
                                vehicle_yaw2 = vehicles_yaw_dict[car][nearing_end_time]
                                if vehicle_yaw2 - vehicle_yaw1 > 0.1:
                                    if turn_start_time < nearing_start_time < turn_end_time:
                                        idx1 = sorted_times.index(nearing_start_time)
                                        idx2 = sorted_times.index(nearing_end_time)
                                        if abs(vehicles_yaw_dict[car][nearing_end_time] - time_yaw_only[
                                            nearing_end_time]) > 1.57:
                                            if time_speed_only[nearing_start_time] > 10:
                                                section = section_time_dict[nearing_start_time]
                                                behaviors_dict[section][28] = 1
                                                section_behavior[section]['28'] = nearing_start_time
                                                break

            # ——————————————————————————————————————交替通行——————————————————————————————————————

            start_pre_car = None
            pre_y = None
            pre_car = None
            pre_y2 = None
            pre_car2 = None
            alternating = False
            start_alternating_time = None
            Bool = False
            alternatings = []  # 记录开始alternating时间的序号和前车
            # 遍历所有时间
            for time in sorted_times:
                # 遍历其它所有车辆判断是否在前面且车速小于每小时20公里
                for car in vehicles_position_dict_local:
                    if time in vehicles_position_dict_local[car]:
                        x = vehicles_position_dict_local[car][time][1]
                        y = vehicles_position_dict_local[car][time][0]
                        if time in vehicles_yaw_dict[car]:
                            yaw_car = vehicles_yaw_dict[car][time]
                            yaw = time_yaw_only[time]
                            if (abs(yaw_car - yaw) <= 1.57 or abs(yaw_car - yaw) > 4.71):
                                if 0 < y < 20:
                                    if abs(x) < 2:
                                        if car in vehicles_speed_dict:
                                            if time in vehicles_speed_dict[car]:
                                                if vehicles_speed_dict[car][time] <= 20:
                                                    pre_car_count2 += 1
                                                    if not pre_car:
                                                        pre_y = y
                                                        pre_car = car
                                                    else:
                                                        if y <= pre_y:
                                                            pre_car = car
                                                            pre_y = y
                # 判断是否有两辆车以上在前20m处，若有则前方正在缓慢行驶
                if pre_car_count2 >= 2:
                    # 看看主车速度是否小于10
                    if time_speed_only[time] <= 10:
                        if not alternatings:
                            idx_time = sorted_times.index(time)
                            if pre_car:
                                alternatings.append((idx_time, pre_car))
                        else:
                            for alternate in alternatings:
                                if pre_car == alternate[1]:
                                    Bool = False
                                    break
                                else:
                                    Bool = True

                            if Bool:
                                Bool = False
                                if pre_car:
                                    alternatings.append((idx_time, pre_car))
                pre_car = None

            for alternate in alternatings:
                start_time_idx = alternate[0]
                start_pre_car = alternate[1]
                for time in sorted_times[start_time_idx:-1]:
                    # 若车速大于16则认为结束缓慢行驶或停车排队
                    if time_speed_only[time] > 16:
                        # 判断下此时前车是哪辆车
                        for car in vehicles_position_dict_local:
                            if time in vehicles_position_dict_local[car]:
                                x = vehicles_position_dict_local[car][time][1]
                                y = vehicles_position_dict_local[car][time][0]
                                if time in vehicles_yaw_dict[car]:
                                    yaw_car = vehicles_yaw_dict[car][time]
                                    yaw = time_yaw_only[time]
                                    if (abs(yaw_car - yaw) <= 1.57 or abs(yaw_car - yaw) > 4.71):
                                        if 0 < y < 50:
                                            if abs(x) < 2:
                                                if not pre_car2:
                                                    pre_car2 = car
                                                    pre_y2 = y
                                                else:
                                                    if y < pre_y2:
                                                        pre_car2 = car
                                                        pre_y2 = y
                        # 前后车不一样，判定为交替通行
                        if pre_car2 != start_pre_car:
                            for tup in section_time:
                                if start_alternating_time == tup[1]:
                                    section = tup[0]
                                    behaviors_dict[section][13] = 1
                                    section_behavior[section]['13'] = time
                                    break

                        break

            # ———————————————————————————————————————结果评价———————————————————————————————————————

            # 从Json文件读取的交规编码用字典存储
            # 遍历
            # 建立一个字典存储违反的所有rules
            violated_rule = {}
            for section in behaviors_dict:
                if section not in violated_rule:
                    violated_rule[section] = []

            Violation = False
            for section in behaviors_dict:
                behaviors_list = behaviors_dict[section]
                # 读取交规编码，再遍历所有编码。从rules_dict内获得该编码对应的违规判定标准
                code_list = section_code_dict[section]
                for code in code_list:
                    code = str(code)
                    if code in rules_dict1:
                        behavior_dict = rules_dict1[code]
                        for behavior in behavior_dict:
                            variable = behavior_dict[behavior]
                            if behaviors_list[behavior] == variable:
                                Violation = True
                            else:
                                Violation = False
                                break
                        if Violation:
                            Violation = False
                            violated_rule[section].append(code)

            Violation = False
            for section in behaviors_dict:
                behaviors_list = behaviors_dict[section]
                # 读取交规编码，再遍历所有编码。从rules_dict内获得该编码对应的违规判定标准
                code_list = section_code_dict[section]
                for code in code_list:
                    code = str(code)
                    if code in rules_dict2:
                        behavior_dict = rules_dict2[code]
                        for behavior in behavior_dict:
                            variable = behavior_dict[behavior]
                            if behaviors_list[behavior] == variable:
                                Violation = True
                            else:
                                Violation = False
                                break
                        if Violation:
                            Violation = False
                            if code not in violated_rule[section]:
                                violated_rule[section].append(code)

            # 将违反的交规编号转为文本
            violated_rule2 = {}
            for section in violated_rule:
                violated_rule2[section] = []
            for section in violated_rule:
                number_list = violated_rule[section]
                if number_list:
                    for number in number_list:
                        number = int(number)
                        text = content_dict_2[number]
                        violated_rule2[section].append(text)

            '''————————————————————————————————最终输出的行为————————————————————————————————'''
            # 设计一个字典存储所有行为
            all_section_behavior = {}
            for section in behaviors_dict:
                if section not in all_section_behavior:
                    all_section_behavior[section] = []

            # 将所有行为写进这个字典
            for section in behaviors_dict:
                behavior_list = behaviors_dict[section]
                for index, value in enumerate(behavior_list):
                    if value == 1:
                        behavior_str = behaviors[f'{index}']
                        if behavior_str not in all_section_behavior[section]:
                            all_section_behavior[section].append(behavior_str)

            # 处理一下最大车速重复的问题
            for section in all_section_behavior:
                behavior_list = all_section_behavior[section]
                pattern = re.compile(r'最大车速超过每小时(\d+)公里')
                # 存储匹配的元素及其速度
                speed_elements = []
                other_elements = []
                for elem in behavior_list:
                    match = pattern.match(elem)
                    if match:
                        speed = int(match.group(1))
                        speed_elements.append((speed, elem))
                    else:
                        other_elements.append(elem)
                # 如果有匹配的元素，找到速度最大的那个
                if speed_elements:
                    # 按速度降序排序
                    speed_elements.sort(reverse=True, key=lambda x: x[0])
                    # 保留速度最大的元素
                    max_speed_elem = speed_elements[0][1]
                    # 构建最终的列表
                    behavior_list = [max_speed_elem] + other_elements
                    all_section_behavior[section] = behavior_list
                else:
                    behavior_list = behavior_list
                    all_section_behavior[section] = behavior_list

            # 处理本车道直行
            for section in all_section_behavior:
                behavior_list = all_section_behavior[section]
                if '变道' not in behavior_list:
                    if '转弯' not in behavior_list:
                        if '掉头' not in behavior_list:
                            if '倒车' not in behavior_list:
                                if '减速' not in behavior_list:
                                    if '停车' not in behavior_list:
                                        if '逆行' not in behavior_list:
                                            if '压实线' not in behavior_list:
                                                if '闯红灯' not in behavior_list:
                                                    if '交替通行' not in behavior_list:
                                                        if '驶入专用车道' not in behavior_list:
                                                            if '驶入应急车道' not in behavior_list:
                                                                behavior_list.append('在本车道直行')
                                                                all_section_behavior[section] = behavior_list

            # 进行扣分
            for section in violated_rule:
                code_list1 = violated_rule[section]
                if code_list1:
                    for code in code_list1:
                        point = rules_point[code]
                        final_point = final_point - point

            output_violations = []
            # 对“压实线行为单独处理”
            for section in all_section_behavior:
                behavior_list = all_section_behavior[section]
                if '压实线' in behavior_list:
                    final_point -= 25
                    time = section_behavior[section]['9']
                    output_section1 = section.split('_')[0]
                    output_violations.append(
                        f'{time}秒,主车在{output_section1},实施了“压道路实线”的违法行为，违反了《道路交通标志和标线第3部分》')

            if final_point < 0:
                final_point = 0

            for section in violated_rule:
                code_list1 = violated_rule[section]
                if code_list1:
                    for code in code_list1:
                        output_behavior = rules_output_behaviors[int(code)]
                        rule_name = rules_name[int(code)]
                        point = rules_point[code]
                        try:
                            rule_time = rules_time1[code]
                            time = section_behavior[section][str(rule_time)]
                        except:
                            rule_time = rules_time2[code]
                            time = section_behavior[section][str(rule_time)]
                        output_section = section.split('_')[0]
                        output_violations.append(
                            f'{time}秒,主车在{output_section},实施了“{output_behavior}”的违法行为，违反了{rule_name}，扣除{point}点分数')
            if not output_violations:
                output_violations.append('主车行驶过程中未违反交规')
            else:
                output_violations = sorted(output_violations, key=lambda x: float(x.split("秒")[0]))  # 将违规按时间排列

            # wth:组织结果并返回
            res_list.append({
                'scenario': map_name,
                'violations': '\n'.join(output_violations),
                'final_point': final_point
            })
            # # 结果保存为csv文件
            # with open(os.path.join(res_folder, f'{scenario_name}_result.csv'), 'w', newline='',
            #           encoding='utf-8') as csvfile:
            #     # 定义列名
            #     fieldnames = ['scenario', 'violations', 'final_point']
            #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            #     # 写入列名
            #     writer.writeheader()
            #     scenario = map_name
            #     point = final_point
            #     violations = '\n'.join(output_violations)
            #
            #     # 写入CSV行
            #     writer.writerow({
            #         'scenario': scenario,
            #         'violations': violations,
            #         'final_point': point
            #     })

        # ————————————————————————————————左行场景——————————————————————————————————————————
        else:  # 如果是左行
            # ———————————————————————————数据读取——————————————————————————————————————————————————————————
            final_point = 100

            df = pd.read_csv(trajectory_path)  # 轨迹数据读取
            csv.field_size_limit(500000)

            # 处理下主车车速
            df['v_ego'] = df['v_ego'] * 3.6
            # 设计一个速度对应时间帧的字典方便读取
            time_speed_only = dict(zip(df.iloc[:, 0], df['v_ego']))

            # 读取主车宽度
            ego_width = None
            try:
                ego_width = df.iloc[1, df.columns.get_loc('width_ego')]
            except:
                pass

            if not ego_width:
                ego_width = 1.66
            else:
                ego_width = ego_width / 2 + 0.8

            # 设计一个只有速度的列表，速度按时间顺序排列
            speed_only = df['v_ego'].iloc[1:].tolist()
            # 设计一个位置对应时间帧的字典方便读取     {time:(x,y)}
            time_position_only = dict(zip(df.iloc[:, 0], zip(df.iloc[:, 3], df.iloc[:, 4])))
            # 设计一个航向角对应时间帧的字典方便读取
            time_yaw_only = dict(zip(df.iloc[:, 0], df['yaw_ego']))
            # 设计一个速度，加速度，航向角对应时间帧的字典用于倒车检测
            time_speed_acceleration_yaw = {time: (v, a, yaw) for time, v, a, yaw in
                                           zip(df.iloc[:, 0], df['v_ego'], df['a_ego'], df['yaw_ego'])}
            # 设计一个前轮转角和对应时间帧的字典用于变道检测
            time_rot_only = dict(zip(df.iloc[:, 0], df['rot_ego']))
            # 两帧之间的时间
            sorted_times = sorted(time_speed_only.keys())
            timestamp = sorted_times[1] - sorted_times[0]
            # 前轮转角存储
            rot_only = []
            for i in time_rot_only.keys():
                rot_only.append(time_rot_only[i])

            # 读取其他所有车辆的位置信息
            vehicles_position_dict = {}
            for i in range(1, len(df.columns) // 2):  # 假设每辆车有对应的 x 和 y 列
                x_col = f'x_car{i}'
                y_col = f'y_car{i}'
                vehicle_data = {}
                if x_col in df.columns and y_col in df.columns:
                    for index, row in df.iterrows():
                        time_frame = row.iloc[0]
                        x = row[x_col]
                        y = row[y_col]
                        x = float(x)
                        y = float(y)
                        time_frame = float(time_frame)
                        vehicle_data[time_frame] = {'x': x, 'y': y}
                        vehicles_position_dict[f'car{i}'] = vehicle_data
                    globals()[f'car{i}_position'] = vehicle_data

            # 转换成以主车为原点的局部坐标
            # 主车前方是y正方向，主车左侧x是正方向          存储的时候(x,y)实际上x是y,y是x，即元组第一个数为y第二个数为x
            def global_to_local(x, y, x_origin, y_origin, theta):
                """将全局坐标转换为局部坐标"""
                # 计算相对坐标
                dx = x - x_origin
                dy = y - y_origin
                # 将角度转换为弧度
                theta_rad = theta
                x_rel = dx * np.cos(theta_rad) + dy * np.sin(theta_rad)
                y_rel = -dx * np.sin(theta_rad) + dy * np.cos(theta_rad)
                x_rel = float(x_rel)
                y_rel = float(y_rel)
                x_rel = round(x_rel, 2)
                y_rel = round(y_rel, 2)
                local_coords = (x_rel, y_rel)
                return local_coords

            # l为其他车辆总数
            l = len(vehicles_position_dict) + 1

            vehicles_position_dict_local = {}  # 格式为{car1:{time1:(y,x),time2:(y,x)},car2:{time1:(y,x)}}
            # 遍历所有车辆
            for i in range(1, l):
                if f'car{i}' in vehicles_position_dict:
                    car_position = vehicles_position_dict[f'car{i}']
                    # 遍历所有时间
                    local_position = {}
                    for j in sorted_times:
                        x = car_position[j]['x']
                        y = car_position[j]['y']
                        local_ords = global_to_local(x, y, time_position_only[j][0], time_position_only[j][1],
                                                     time_yaw_only[j])
                        local_position[j] = local_ords
                        vehicles_position_dict_local[f'car{i}'] = local_position
                    globals()[f'car{i}_local_position'] = local_position

            # 读取其他所有车辆的速度信息
            vehicles_speed_dict = {}
            for i in range(1, l):
                speed_col = f'v_car{i}'
                speed_data = {}
                if speed_col in df.columns:
                    for index, row in df.iterrows():
                        time_frame = row.iloc[0]
                        speed = row[speed_col]
                        time_frame = float(time_frame)
                        speed = speed * 3.6
                        speed = round(speed, 2)
                        speed = float(speed)
                        speed_data[time_frame] = speed
                        vehicles_speed_dict[f'car{i}'] = speed_data
                    globals()[f'car{i}_speed'] = speed_data

            # 读取所有其他车辆航向角信息
            vehicles_yaw_dict = {}
            for i in range(1, l):
                yaw_col = f'yaw_car{i}'
                yaw_data = {}
                if yaw_col in df.columns:
                    for index, row in df.iterrows():
                        time_frame = row.iloc[0]
                        yaw = row[yaw_col]
                        yaw = float(yaw)
                        time_frame = float(time_frame)
                        yaw_data[time_frame] = yaw
                        vehicles_yaw_dict[f'car{i}'] = yaw_data
                    globals()[f'car{i}_yaw'] = yaw_data

            # ————————————————————————地图数据读取
            def point_in_lane_section(polygons, x, y):
                point = Point(x, y)
                containing_sections = []
                for lane_section_id, polygon in polygons.items():
                    if polygon.contains(point):
                        containing_sections.append(lane_section_id)
                return containing_sections

            # 存储[(section,time)]
            section_time = []
            # {time:section}
            section_time_dict = {}

            # 获取lane_section坐标范围
            section_polygon = get_lane_section_polygons(xodr)

            # 判断每个时刻在哪个section
            last_section = None
            for time in sorted_times:
                x = time_position_only[time][0]
                y = time_position_only[time][1]
                containing_sections = point_in_lane_section(section_polygon, x, y)
                if containing_sections:
                    for section in containing_sections:
                        section_time.append((section, time))
                        section_time_dict[time] = section
                        last_section = section
                        break
                else:
                    section_time.append((last_section, time))
                    section_time_dict[time] = last_section

            # 按时间帧顺序排序
            section_time = sorted(section_time, key=lambda x: x[1])
            section_time_dict = {t[1]: t[0] for t in section_time}

            section_behavior = {}
            for location_name, time_frame in section_time:
                if location_name not in section_behavior:
                    section_behavior[location_name] = {'开始时间': time_frame, '结束时间': time_frame}
                else:
                    section_behavior[location_name]['结束时间'] = time_frame
            # 格式{Road1:{'开始时间'：'','结束时间':'','行为编码'：'发生时间'}，Road2:{...}}

            # 记录主车经过的section,格式：[Road1,Road2]无'超出地图数据'
            car_section_list = []
            for i in section_behavior:
                if i.startswith('Road'):
                    car_section_list.append(i)

            # 创建匹配所用的字典。键为所有经过的section，值为每个section对应的行为列表。
            behaviors_dict_left_hand = {key: [0] * 3 for key in section_behavior.keys()}
            behaviors_left_hand = {'0': '车速超出限速', '1': '转弯时车速超过每小时30公里', '2': '压道路实线'}

            '''超速模块'''
            # 获取地图限速
            speed_limit_map = get_lane_section_speed_limits(xodr)
            for key in speed_limit_map:
                if speed_limit_map[key] is not None:
                    speed_limit_map[key] = round(speed_limit_map[key] * 3.6)

            for time in sorted_times:
                section = section_time_dict[time]
                if section in speed_limit_map:
                    speed_limit = speed_limit_map[section]
                    if speed_limit:
                        if time_speed_only[time] >= speed_limit:
                            behaviors_dict_left_hand[section][0] = 1
                            section_behavior[section]['0'] = time

            '''转弯超速模块模块'''
            # 设定阈值和变量   目前转弯阈值为30°-150°
            start_time = None
            cumulative_delta_yaw = 0
            turn_direction = None
            turn_start_time = None
            turn_end_time = None
            last_time = None
            last_yaw_turn = None
            last_delta_yaw = 0  # 上一次的航向角改变量
            turns = []
            # 遍历字典，按时间帧顺序
            for time_frame in sorted_times[1:]:  # 从第二个时间帧开始,sorted返回以时间帧排序的列表
                current_yaw_turn = time_yaw_only[time_frame]
                if last_yaw_turn is not None:
                    current_delta_yaw = current_yaw_turn - last_yaw_turn  # 当前航向角的改变量
                    if current_delta_yaw != 0:  # 如果航向角发生变化
                        # 如果是同一方向的变化，累计改变量
                        if current_delta_yaw > 0 and last_delta_yaw >= 0:
                            cumulative_delta_yaw += current_delta_yaw
                            if start_time is None:  # 记录转弯开始时的时间帧
                                start_time = last_time  # 使用上一个时间帧
                        elif current_delta_yaw < 0 and last_delta_yaw <= 0:
                            cumulative_delta_yaw += current_delta_yaw
                            if start_time is None:  # 记录转弯开始时的时间帧
                                start_time = last_time  # 使用上一个时间帧
                        else:  # 方向改变，停止累计
                            # 判断是否满足转弯条件
                            if 0.52 <= abs(cumulative_delta_yaw) <= 2.9:
                                next_time_frames = list(sorted(time_yaw_only.keys()))[
                                                   list(time_yaw_only.keys()).index(time_frame) + 1:
                                                   min(list(time_yaw_only.keys()).index(time_frame) + 11,
                                                       len(time_yaw_only))]
                                for next_time in next_time_frames:
                                    next_change = abs(time_yaw_only[next_time] - time_yaw_only[last_time])
                                    if next_change > 0.6:
                                        break
                                else:
                                    # 满足条件，记录转弯信息
                                    turn_direction = '右转' if cumulative_delta_yaw < 0 else '左转'
                                    turn_start_time = start_time
                                    turn_end_time = time_frame
                                    if time_speed_only[turn_start_time] > 30:
                                        section = section_time_dict[turn_start_time]
                                        behaviors_dict_left_hand[section][1] = 1
                                        section_behavior[section]['1'] = turn_start_time
                                    elif time_speed_only[turn_end_time] > 30:
                                        section = section_time_dict[turn_end_time]
                                        behaviors_dict_left_hand[section][1] = 1
                                        section_behavior[section]['1'] = turn_end_time
                            start_time = None
                            # 重置累计值
                            cumulative_delta_yaw = 0
                    last_delta_yaw = current_delta_yaw
                last_yaw_turn = current_yaw_turn
                last_time = time_frame

            if start_time is not None and 0.52 <= abs(cumulative_delta_yaw) <= 2.9:
                turn_direction = '右转' if cumulative_delta_yaw < 0 else '左转'
                turn_start_time = start_time
                turn_end_time = sorted_times[-1]
                if time_speed_only[turn_start_time] > 30:
                    section = section_time_dict[turn_start_time]
                    behaviors_dict_left_hand[section][1] = 1
                    section_behavior[section]['1'] = turn_start_time
                elif time_speed_only[turn_end_time] > 30:
                    section = section_time_dict[turn_end_time]
                    behaviors_dict_left_hand[section][1] = 1
                    section_behavior[section]['1'] = turn_end_time

            '''获取标线坐标范围并检测是否有压实线行为'''
            polygons_road_mark = parse_road_marks(xodr, road_mark_width=ego_width)
            start_time = None
            count_road_mark = 0
            previous_road_mark = None
            current_road_mark = None
            in_mark = False
            for time in sorted_times:
                x = time_position_only[time][0]
                y = time_position_only[time][1]
                containing_sections_road_mark = point_in_lane_section(polygons_road_mark, x, y)
                # 这里返回lane_section_id，如果不在Road_mark内则是空列表
                if containing_sections_road_mark:
                    current_road_mark = containing_sections_road_mark[0]
                    if not in_mark:
                        start_time = time
                        in_mark = True
                    if in_mark:
                        if current_road_mark == previous_road_mark:
                            count_road_mark += 1
                            if count_road_mark == 5:
                                for tup in section_time:
                                    if start_time == tup[1]:
                                        section = tup[0]
                                        behaviors_dict_left_hand[section][2] = 1
                                        section_behavior[section]['2'] = start_time
                                count_road_mark = 0
                                start_time = None
                        else:
                            count_road_mark = 0
                    previous_road_mark = current_road_mark

            # ——————————————————————结果评价——————————————————————————————————
            output_violations = []

            # 设计一个字典存储所有行为
            all_section_behavior = {}
            for section in behaviors_dict_left_hand:
                if section not in all_section_behavior:
                    all_section_behavior[section] = []
            # 将所有行为写进这个字典
            for section in behaviors_dict_left_hand:
                behavior_list = behaviors_dict_left_hand[section]
                for index, value in enumerate(behavior_list):
                    if value == 1:
                        behavior_str = behaviors_left_hand[f'{index}']
                        if behavior_str not in all_section_behavior[section]:
                            all_section_behavior[section].append(behavior_str)

            for section in all_section_behavior:
                behavior_list = all_section_behavior[section]
                if '车速超出限速' in behavior_list:
                    final_point -= 25
                    time = section_behavior[section]['0']
                    output_section1 = section.split('_')[0]
                    output_violations.append(
                        f'{time}秒,主车在{output_section1},实施了“车速超出限速”的违法行为，违反了《中华人民共和国道路交通安全法》')

            for section in all_section_behavior:
                behavior_list = all_section_behavior[section]
                if '转弯时车速超过每小时30公里' in behavior_list:
                    final_point -= 25
                    time = section_behavior[section]['1']
                    output_section2 = section.split('_')[0]
                    output_violations.append(
                        f'{time}秒,主车在{output_section2},实施了“转弯时车速超过每小时30公里”的违法行为，违反了《中华人民共和国道路交通安全法实施条例》')

            for section in all_section_behavior:
                behavior_list = all_section_behavior[section]
                if '压道路实线' in behavior_list:
                    final_point -= 25
                    time = section_behavior[section]['2']
                    output_section3 = section.split('_')[0]
                    output_violations.append(
                        f'{time}秒,主车在{output_section3},实施了“压道路实线”的违法行为，违反了《道路交通标志和标线第3部分》')


            if not output_violations:
                output_violations.append('主车行驶过程中未违反交规')
            else:
                output_violations = sorted(output_violations, key=lambda x: float(x.split("秒")[0]))

            # wth:组织结果并返回
            res_list.append({
                    'scenario': map_name,
                    'violations': '\n'.join(output_violations),
                    'final_point': final_point
                })
            # 结果保存为csv文件
            # with open(os.path.join(res_folder, f'{scenario_name}_result.csv'), 'w', newline='',
            #           encoding='utf-8') as csvfile:
            #     # 定义列名
            #     fieldnames = ['scenario', 'violations', 'final_point']
            #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            #     # 写入列名
            #     writer.writeheader()
            #     scenario = map_name
            #     point = final_point
            #     violations = '\n'.join(output_violations)
            #
            #     # 写入CSV行
            #     writer.writerow({
            #         'scenario': scenario,
            #         'violations': violations,
            #         'final_point': point
            #     })
        resTable = pd.DataFrame(res_list)
        return resTable