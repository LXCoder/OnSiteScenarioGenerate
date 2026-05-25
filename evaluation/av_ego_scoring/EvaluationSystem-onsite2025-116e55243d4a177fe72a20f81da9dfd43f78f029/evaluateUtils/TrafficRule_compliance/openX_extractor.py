import xml.etree.ElementTree as ET
import datetime
import json
import os
import warnings
import copy
from tqdm import tqdm
from typing import Tuple, List
import pandas as pd
import networkx as nx
from evaluateUtils.TrafficRule_compliance.TagTree import TagTree
from evaluateUtils.config import ROAD_TAG_COL, INFR_TAG_COL, MAN_TAG_COL, ENV_TAG_COL
from evaluateUtils.config import SCENARIO_ID_COL, SEGMENT_ID_COL
from evaluateUtils.standard_parameter import Parameter
from evaluateUtils.TrafficRule_compliance.file_utils import is_lefthand_scn


# 提取函数
def extract_road_type(road: ET.Element, base_filename: str):
    """
    Extract road type tag, according to xodr definition.
    """
    road_type_mapping = {
        'motorway': '高速公路',
        'rural': '乡村道路',
        # 'urban': '城市道路',
        'town': '城市道路',
        'townExpressway': '城市快速路',
        'townArterial': '普通城市道路',
        'townCollector': '普通城市道路',
        'townLocal': '普通城市道路',
        # 以下在tag中没有对应的
        'bicycle': '',
        'lowSpeed': '',
        'pedestrian': '',
        'unknown': ''
    }
    road_type_elem = road.find('type')
    if road_type_elem is not None:
        road_type = road_type_elem.get('type', 'unknown')
        road_type_tag = road_type_mapping.get(road_type, '')
        return road_type_tag
    # 数据集中部分xodr缺少road type。从文件名中打补丁
    if 'highway' in base_filename:
        return '高速公路'
    return ''


def is_ramp_merge(junction_root: ET.Element, xodr_root: ET.Element) -> bool:
    """
    Judge whether a junction is a ramp merge or not.
    """
    # list all of the road in xodr_root
    all_road = xodr_root.findall('road')
    all_road_dict = {road.get('id'): road for road in all_road}
    flag = False
    connections = junction_root.findall('connection')
    for cnc in connections:
        incoming_road_id = cnc.get('incomingRoad')
        connecting_road_id = cnc.get('connectingRoad')
        if incoming_road_id is not None and connecting_road_id is not None:
            in_road = all_road_dict.get(incoming_road_id,None)
            cn_road = all_road_dict.get(connecting_road_id,None)
            if (in_road is None) or (cn_road is None):
                continue
            if is_ramp(in_road) or is_ramp(cn_road):
                flag = True
                break
    return flag


def is_ramp(road: ET.Element) -> bool:
    """
    Judge whether the road is a ramp or not.
    """
    for lane_section in road.findall('.//laneSection'):
        lane_section_s = float(lane_section.get('s'))
        left_lanes = lane_section.find('left')
        right_lanes = lane_section.find('right')
        all_lane_type = []
        if left_lanes is not None:
            all_lane_type.extend([lane.get('type') for lane in left_lanes.findall('lane')])
        if right_lanes is not None:
            all_lane_type.extend([lane.get('type') for lane in right_lanes.findall('lane')])
        if all([t in ['onRamp', 'offRamp', 'connectingRamp'] for t in all_lane_type]):
            return True
    return False


def is_roundabout(junction_root: ET.Element, cycles: List[List[str]]) -> bool:
    """
    Judge whether the junction is a roundabout or not.
    """
    roundabout_id_list = [item for sublist in cycles for item in sublist]
    connections = junction_root.findall('connection')
    for cnc in connections:
        incoming_road_id = cnc.get('incomingRoad')
        connecting_road_id = cnc.get('connectingRoad')
        if (incoming_road_id in roundabout_id_list) or (connecting_road_id in roundabout_id_list):
            return True
    return False


def extract_facility_type(road: ET.Element, xodr_root: ET.Element, cycles: List[List[str]], base_filename: str) -> str:
    """
    判断给定道路的设施类型，包括["路段","匝道","桥梁","隧道","铁路道口","停车场","出租车上下站","道路停车泊位","停靠站"]
    针对每一种设施具有不同的判断方法.
    尚未完成. 剩余类型可能要用到多种属性,当前数据集不涉及,暂时不予考虑
    """
    # 判断是不是交叉口
    # PATCH：由于xodr部分匝道的车道类型未标记为*ramp，故根据xodr_header_name的命名进行判断
    xodr_header_name = xodr_root.find('header').get('name')
    junction = road.get('junction')
    if int(junction) != -1:
        # 排除匝道汇入等非交叉口的情形
        all_junction = xodr_root.findall('junction')
        try:
            junction_root = [j for j in all_junction if j.get('id') == junction][0]
            if (not is_ramp_merge(junction_root, xodr_root)) and ('ramp' not in xodr_header_name.lower()):
                # 判断是不是环形交叉口
                if is_roundabout(junction_root, cycles):
                    return '环形交叉口'
                else:
                    return '交叉口'
        except IndexError:
            pass

    object_elem = road.find('object')
    if object_elem is not None:
        # 判断是不是隧道
        tunnel = object_elem.findall('tunnel')
        if tunnel:
            if len(tunnel) > 1:
                print('多个隧道')
            return '隧道'
        # 判断是不是桥梁
        bridge = object_elem.findall('bridge')
        if bridge:
            if len(bridge) > 1:
                print('多个桥梁')
            return '桥梁'

    # 判断是不是匝道
    for lane_section in road.findall('.//laneSection'):
        lane_section_s = float(lane_section.get('s'))
        left_lanes = lane_section.find('left')
        right_lanes = lane_section.find('right')
        all_lane_type = []
        if left_lanes is not None:
            all_lane_type.extend([lane.get('type') for lane in left_lanes.findall('lane')])
        if right_lanes is not None:
            all_lane_type.extend([lane.get('type') for lane in right_lanes.findall('lane')])
        if all([t in ['onRamp', 'offRamp', 'connectingRamp'] for t in all_lane_type]):
            return '匝道'

    return '路段'


def extract_lane_type(lane: ET.Element) -> str:
    """
    Extract lane type tag, according to xodr definition.
    """
    lane_type_mapping = {
        'driving': '机动车道',
        'sidewalk': '人行道',
        'biking': '非机动车道',
        'pedestrian': '人行横道',
        'entry': '加速车道',
        'exit': '减速车道',
        'onRamp': '机动车道',
        'offRamp': '机动车道',
        'connectingRamp': '机动车道',
        # 如下是tag中没有的
        '...': '...',
    }
    lane_type = lane.get('type', 'unknown')
    return lane_type_mapping.get(lane_type, '')


def extract_lane_access(lane: ET.Element) -> str:
    """
    判断一条车道的专属权
    """
    lane_access_mapping = {
        'bus': '公交专用车道',
        'taxi': '客车专用道',
        'emergency': '应急车道',
        'none': '无车道专属权'
    }
    access = lane.find('access')
    if access is not None:
        restricted_participant = [restriction.get('type') for restriction in access.findall('restriction')]
        is_allowed = access.get('rule')
        if is_allowed == 'allow':
            tag_name = ';'.join(
                [lane_access_mapping[r] for r in restricted_participant if r in lane_access_mapping.keys()])
            return tag_name
    return '无车道专属权'


def extract_lane_width(lane: ET.Element, lane_type: str):
    """
    判断车道宽度是否狭窄，并标记为对应的tag
    对于公路, 取阈值为3.25m(参考《公路路线设计规范JTGD20Y2017》)
    对于城市道路, 阈值同样为3.25m(《城市道路工程设计规范CJJ37Y2012》)
    """
    if lane_type != 'driving':
        return ''
    width_th = 3.25
    widths = lane.findall('width')
    if not widths:
        return ''
    for width in widths:
        # 用a的取值做近似宽度
        a = float(width.get('a', '0.0'))
        b = float(width.get('b', '0.0'))
        c = float(width.get('c', '0.0'))
        d = float(width.get('d', '0.0'))
        if not (round(b, 2) == 0.0 and round(c, 2) == 0.0 and round(d, 2) == 0.0):
            print('In lane width attributes,', b, c, d)
        if a < width_th:
            print('The lane width is less than 3.25m!')
            print(a, b, c, d)
            return '狭窄的车道'
    return '宽度正常的车道'


def get_lane_location(
        lane: ET.Element,
        same_dir_lanes: ET.Element,
        facility_type: str,
) -> str:
    """
    提取车道所处的位置，服务于判断交通标线所处的位置
    """
    if facility_type in ['交叉口', "普通交叉口", "环形交叉口"]:
        return '交叉口'
    target_lane_type = lane.get('type')
    if target_lane_type == 'parking':
        return "停车位"
    all_lanes = same_dir_lanes.findall('lane')
    # 车道越远离中心线,其ID绝对值越大
    target_lane_id = abs(int(lane.get('id')))
    all_lane_id = [abs(int(l.get('id'))) for l in all_lanes]
    all_driving_lane_id = [abs(int(l.get('id'))) for l in all_lanes if l.get('type') == 'driving']

    if target_lane_id not in all_lane_id:
        print('The lane id is NOT in the lanes!')
        return ''
    if target_lane_id == max(all_lane_id):
        return '路侧'
    if target_lane_id < max(all_driving_lane_id):
        return '路中'
    if target_lane_id == max(all_driving_lane_id):
        return '车行道边缘'
    return ''


def extract_lane_mark(lane: ET.Element, lanes: str, facility_type: str) -> str:
    """
    给定一个车道，提取该车道的交通标线信息
    """
    mark = []
    # <roadMark>元素只能（shall）用于描述外侧路标。
    for road_mark in lane.findall('roadMark'):
        color = road_mark.get('color')
        color = 'white' if color == 'standard' else color
        mark_type = road_mark.get('type')
        if color == 'white' and mark_type == 'broken':
            mark.append('白色虚线')
        elif color == 'yellow' and mark_type == 'broken':
            # 黄色虚线需要细分
            lane_location = get_lane_location(lane, lanes, facility_type)
            if lane_location == '路中':
                mark.append('路中黄色虚线')
            elif lane_location == '路侧':
                mark.append('路侧黄色虚线')
            elif lane_location == '交叉口':
                mark.append('交叉口黄色虚线')
            else:
                mark.append('黄色虚线')
        elif color == 'white' and mark_type == 'solid':
            # 白色实线需要细分
            lane_location = get_lane_location(lane, lanes, facility_type)
            if lane_location == '路中':
                mark.append('路中白色实线')
            elif lane_location == '车行道边缘':
                mark.append('车行道边缘白色实线')
            elif lane_location == '停车位':
                mark.append('停车位白色实线')
            else:
                mark.append('白色实线')

        elif color == 'yellow' and mark_type == 'solid':
            # 黄色实线需要细分
            lane_location = get_lane_location(lane, lanes, facility_type)
            if lane_location == '路中':
                mark.append('路中黄色实线')
            elif lane_location == '路侧':
                mark.append('路侧黄色实线')
            elif lane_location == '停车位':
                mark.append('停车位黄色实线')
            else:
                mark.append('黄色实线')

        elif color == 'white' and mark_type == 'broken broken':
            mark.append('双白虚线')
        elif color == 'white' and mark_type == 'solid solid':
            mark.append('双白实线')
        elif color == 'white' and mark_type == 'broken solid':
            mark.append('白色虚实线(左虚右实)')
        elif color == 'white' and mark_type == 'solid broken':
            mark.append('白色虚实线(左实右虚)')
        elif color == 'yellow' and mark_type == 'solid solid':
            mark.append('双黄实线')
        elif color == 'yellow' and mark_type == 'broken broken':
            mark.append('双黄虚线')
        elif color == 'yellow' and mark_type == 'broken solid':
            mark.append('黄色虚实线(左虚右实)')
        elif color == 'yellow' and mark_type == 'solid broken':
            mark.append('黄色虚实线(左实右虚)')
        elif color == 'blue':
            mark.append('蓝色虚(实)线')
        elif color == 'orange':
            mark.append('橙色虚(实)线')
        else:
            pass
    return ';'.join(mark)


def extract_wo_lane_mark(lane_mark: str) -> str:
    """
    在获取xodr中交通标线基础之上，进一步判断没有包含哪些交通标线。
    """
    # xodr文件中往往定义了各个车道的限速（max），因此不打"无限速标线"的tag
    wo_mark = []
    w_mark = lane_mark.split(';')
    if '路侧黄色实线' not in w_mark:
        wo_mark.append('无禁止停车标线')
    if ('路中黄色实线' not in w_mark) and \
            ('双黄实线' not in w_mark) and \
            ('黄色虚实线(左虚右实)' not in w_mark):
        wo_mark.append('无禁止左转标线')
        wo_mark.append('无禁止掉头标线')
    if not w_mark:
        wo_mark.append('无交通标线')
    return ';'.join(wo_mark)


def extract_vertical_alignment(road: ET.Element, road_type: str) -> Tuple[List[str], List[float]]:
    """
    提取道路纵断面特征相关的tag，包括：陡坡、非陡坡、上陡坡、下陡坡

    判断道路是否“陡”的核心在于找到不同道路类型对应的坡度阈值。

    如过某段出现了陡坡相关的特征，则记录并返回其s坐标，以支持后续交规检索单元的切分。
    """
    if not road_type or road_type in ["其他"]:
        return [], []
    vertical_feature = []
    vertical_cut_s = []
    # 城市道路
    if road_type in ["城市道路", "城市快速路", "普通城市道路", "小区内部道路"]:
        slope_th = 0.07
    # 公路
    elif road_type in ["公路", "高速公路", "普通公路", "山区公路", "非山区公路"]:
        slope_th = 0.08
    else:
        raise ValueError('Unknown road type:', road_type)

    elevations = road.findall('elevationProfile/elevation')
    for elevation in elevations:
        slope = float(elevation.get('b', '0.0'))
        if abs(slope) > slope_th:
            if slope > 0:
                vertical_feature.append('上陡坡')
            else:
                vertical_feature.append('下陡坡')
        else:
            vertical_feature.append('非陡坡')
        vertical_cut_s.append(float(elevation.get('s')))

    return vertical_feature, vertical_cut_s


def extract_lane_num(lanes: ET.Element) -> Tuple[str, int]:
    """
    Extract the number of lanes in left and right side of the road

    Note that only 'driving', 'entry', 'exit', 'onRamp','offRamp', 'connectingRamp', 'slipLane' are considered as driving lanes
    """

    def is_driving_lane(lane: ET.Element):
        return lane.get('type', 'unknown') in ['driving', 'entry', 'exit', 'onRamp', 'offRamp', 'connectingRamp',
                                               'slipLane']

    def lane_list2tag(lane_list: list) -> str:
        if len(lane_list) == 0:
            return ''
        elif len(lane_list) == 1:
            return '单车道'
        elif len(lane_list) == 2:
            return '双车道'
        elif len(lane_list) == 3:
            return '大于两条车道'
        else:
            return '大于两条车道;大于三条车道'

    if lanes is not None:
        driving_lanes = [lane for lane in lanes.findall('lane') if is_driving_lane(lane)]
    else:
        driving_lanes = []
    return lane_list2tag(driving_lanes), len(driving_lanes)


def extract_central_divider(lane_section: ET.Element, road_type: str) -> str:
    """
    extract the type of central divider in the road.

    注意：部分xodr中错误地/没有设置中央隔离设施，包括但不限于：高速公路、城市快速路未设置中央分隔带(center_type!=median)等问题。

    针对xodr这类缺陷，增加了以下逻辑：
    包括：
    （1）若道路类型为高速公路、城市快速路，return "中央分隔带"

    """
    if road_type in ["高速公路", "城市快速路"]:
        return "中央分隔带"

    center = lane_section.find('center')
    center_lane = center.find('lane')
    center_lane_type = center_lane.get('type')
    if center_lane_type == 'median':
        return '中央分隔带'
    center_lane_road_mark = center_lane.find('roadMark')
    if center_lane_road_mark is not None:
        return '道路中心线'
    return '无道路中央隔离设施'


def extract_time_info(environment: ET.Element) -> str:
    """
    To judge "day" or "night" by the time of day
    """
    time_of_day = environment.find('TimeOfDay')
    if time_of_day is not None:
        date_time = time_of_day.get('dateTime', '')
        if not date_time:
            return ''
        hour = int(date_time[11:13])
        if 6 <= hour < 18:
            return '白天'
        else:
            return '夜晚'
    return ''


def extract_visibility_info(environment: ET.Element) -> str:
    """
    提取能见度相关的tag
    """
    weather = environment.find('Weather')
    if weather is not None:
        visibility = float(weather.get('visibility', '1000'))
        if visibility >= 200:
            return '能见度正常'
        elif 100 <= visibility < 200:
            return '能见度小于200米'
        elif 50 <= visibility < 100:
            return '能见度小于100米'
        else:
            return '能见度小于50米'
    else:
        return ''


def extract_weather_info(environment: ET.Element) -> str:
    """
    从xosc中提取天气相关的tag
    """
    weather = environment.find('Weather')
    precipitation = weather.find('Precipitation')
    fog = weather.find('Fog')
    weather_tag = []
    if precipitation is not None:
        precipitationType = precipitation.get('precipitationType', 'none').lower()
        # intensity = float(precipitation.get('intensity'))
        if precipitationType == 'dry':
            pass
        elif precipitationType == 'rain':
            weather_tag.append('雨天')
        elif precipitationType == 'snow':
            weather_tag.append('雪天')
        else:
            print('Not pre-defined precipitationType:', precipitationType)

    if fog is not None:
        visual_range = float(fog.get('visualRange'))
        # (坑) boundingBox 指代了雾的位置,用以标注局部雾天
        # 由于目前场景地图范围较小，暂时未设计根据boundingBox的切分逻辑
        if visual_range < 200:
            weather_tag.append('雾天')

    if len(weather_tag) == 0:
        return '无极端天气'
    else:
        return ';'.join(weather_tag)


def extract_travel_period(environment: ET.Element):
    """
    提取出行时间段
    """
    time_of_day = environment.find('TimeOfDay')
    if time_of_day is not None:
        date_time = time_of_day.get('dateTime', '')
        if not date_time:
            return ''
        hour = int(date_time[11:13])
        if 7 <= hour < 9:
            return '早高峰'
        elif 17 <= hour < 19:
            return '晚高峰'
        else:
            return '平峰'
    return ''


def extract_weekday_info(environment: ET.Element) -> str:
    """
    提取工作日情况
    """
    time_of_day = environment.find('TimeOfDay')
    if time_of_day is not None:
        date_time = time_of_day.get('dateTime', '2024-01-01T00:00:00')
        date_str = date_time[:10]
        date_obj = datetime.datetime.strptime(date_str, '%Y-%m-%d')
        weekday = date_obj.weekday()

        if weekday in [0, 1, 2, 3, 4]:
            return '工作日'
        elif weekday == 5 or weekday == 6:
            return '双休日'

    return ''


def extract_main_auxiliary_road_type(road: ET.Element) -> str:
    return ''


def extract_road_direction(road: ET.Element) -> str:
    """
    Judge the given road is bidirectional or unidirectional, according to the lane ID.

    If lane ID including positive and negative values, the road is bidirectional.

    If lane ID only including positive(or negative) values, the road is unidirectional.
    """
    """
    lanes = road.findall('lanes/laneSection')
    direction = []
    for lane_section in lanes:
        for lane in lane_section.findall('.//lane'):
            lane_id = int(lane.get('id'))
            if lane_id > 0:
                direction.append('left')
            elif lane_id < 0:
                direction.append('right')
            else:
                pass
    if 'right' in direction and 'left' in direction:
        return '双向通行'
    elif 'right' in direction:
        return '单向通行'
    elif 'left' in direction:
        return '单向通行'
    else:
        return ''
    """
    # 由于数据集中大多数双向通行道路被拆分成两个road，
    # 难以判断单向or双向，
    # 此处不再判断道路通行方向
    return ''


def extract_traffic_light_info(faciality_type: str, json_data: dict) -> str:
    """
    Extract the traffic light information from the given JSON data.

    Only the color is considered here. (e.g., red, green, yellow)
    """
    traffic_light_color_mapping = {
        'red': '红灯',
        'green': '绿灯',
        'yellow': '黄灯'
    }
    traffic_light_shape_mapping = {
        'straight': '直行灯',
        'left': '左转灯',
        'right': '右转灯',
        'u_turn': '掉头灯',
        'circle': '圆形灯'
    }
    if faciality_type not in ["交叉口", "普通交叉口", "环形交叉口"]:
        return ''
    if not json_data:
        return '无交通信号灯'
    else:
        all_colors = list(set(list(json_data.values())))
        # if the traffic light does not change during the whole test,
        # return the corresponding color.
        if len(all_colors) == 1:
            return traffic_light_color_mapping[all_colors[0]]
        else:
            return '有交通信号灯'


def extract_traffic_sign_info(sign):
    """
    提取交通标志信息

    由于数据集中不包含交通标志，故直接return
    """
    # xodr文件中往往定义了各个车道的限速（max），因此不打"无限速"的tag
    return "无禁止掉头标志;无禁止左转标志;无禁止鸣喇叭标志;无交通标志"


def extract_stop_line(road: ET.Element) -> str:
    """
    Stop line(s) has NOT appeared in existing xodr file,
    therefore, we do NOT need to consider it.
    """
    return ''


def get_precessor_and_successor(road: ET.Element):
    """
    Get the predecessor and successor of a road element, according it's <link> element.
    """
    linkage = road.find('link')
    predecessor = linkage.find('predecessor') if linkage is not None else None
    successor = linkage.find('successor') if linkage is not None else None
    predecessor_type = predecessor.get('elementType') if predecessor is not None else ''
    predecessor_id = predecessor.get('elementId') if predecessor is not None else ''
    successor_type = successor.get('elementType') if successor is not None else ''
    successor_id = successor.get('elementId') if successor is not None else ''
    return predecessor_type, predecessor_id, successor_type, successor_id


def extract_forward_lane_num_change_at_road_level(
        road: ET.Element,
        link_dict: dict,
        ego_facility_type: str,
        xodr_root: ET.Element,
) -> Tuple[str, str, float, float, float, float]:
    """
    获取road连接处的车道数变化情况
    Judge the forwar lane number change, including keeping the same, increasing, or decreasing.

    Note that the left and right lanes are considered separately.
    """

    def get_target_road(target_id: str, xodr_root: ET.Element) -> ET.Element:
        for road in xodr_root.findall('road'):
            if road.get('id') == target_id:
                return road
        raise ValueError('The target road is not found!')

    def get_lane_section(road: ET.Element, s_loc: str) -> ET.Element:
        lanesecion_dict = {}
        s_list = []
        for lanesection in road.findall('.//laneSection'):
            s = float(lanesection.get('s'))
            s_list.append(s)
            lanesecion_dict[s] = lanesection
        if s_loc == 'max':
            return lanesecion_dict[max(s_list)], max(s_list)
        elif s_loc == 'min':
            return lanesecion_dict[min(s_list)], min(s_list)
        else:
            raise ValueError('Unknown direction:', s_loc)

    def get_lane_num(lanes: ET.Element) -> int:
        driving_lanes = [lane for lane in lanes.findall('lane') if lane.get('type') == 'driving']
        return len(driving_lanes)

    left_lane_forward_lane_num_change, right_lane_forward_lane_num_change = '', ''
    left_ego_s = None
    left_forward_s = None
    right_ego_s = None
    right_forward_s = None
    if ego_facility_type in ["交叉口", "普通交叉口", "环形交叉口"]:
        return left_lane_forward_lane_num_change, right_lane_forward_lane_num_change, left_ego_s, left_forward_s, right_ego_s, right_forward_s
    # road_id = road.get('id')
    predecessor_type = link_dict[road.get('id')]['predecessor_type']
    predecessor_id = link_dict[road.get('id')]['predecessor_id']
    successor_type = link_dict[road.get('id')]['successor_type']
    successor_id = link_dict[road.get('id')]['successor_id']

    # predecssor is the forward road for ego road left lanes
    if predecessor_type == 'road':
        ego_lane_section, left_ego_s = get_lane_section(road, 'min')
        forward_lane_section, left_forward_s = get_lane_section(
            get_target_road(target_id=predecessor_id, xodr_root=xodr_root), 'max')
        ego_lanes = ego_lane_section.find('left')
        forward_lanes = forward_lane_section.find('left')
        if ego_lanes is not None and forward_lanes is not None:
            ego_lane_num = get_lane_num(ego_lanes)
            forward_lane_num = get_lane_num(forward_lanes)
            if ego_lane_num == forward_lane_num:
                left_lane_forward_lane_num_change = '前方车道数不变'
            elif ego_lane_num < forward_lane_num:
                left_lane_forward_lane_num_change = '前方车道数增加'
            else:
                left_lane_forward_lane_num_change = '前方车道数减少'
    # successor is the forward road for ego road right lanes
    if successor_type == 'road':
        ego_lane_section, right_ego_s = get_lane_section(road, 'max')
        forward_lane_section, right_forward_s = get_lane_section(
            get_target_road(target_id=successor_id, xodr_root=xodr_root), 'min')
        ego_lanes = ego_lane_section.find('right')
        forward_lanes = forward_lane_section.find('right')
        if ego_lanes is not None and forward_lanes is not None:
            ego_lane_num = get_lane_num(ego_lanes)
            forward_lane_num = get_lane_num(forward_lanes)
            if ego_lane_num == forward_lane_num:
                right_lane_forward_lane_num_change = '前方车道数不变'
            elif ego_lane_num < forward_lane_num:
                right_lane_forward_lane_num_change = '前方车道数增加'
            else:
                right_lane_forward_lane_num_change = '前方车道数减少'

    if left_ego_s:
        print('left_ego_s:', left_ego_s)
    if right_ego_s:
        print('right_ego_s:', right_ego_s)
    return left_lane_forward_lane_num_change, right_lane_forward_lane_num_change, left_ego_s, left_forward_s, right_ego_s, right_forward_s


def extract_forward_lane_num_change_at_laneSection_level(
        current_lane_section_s: float,
        current_lane_num: int,
        forward_lane_num: int,
        flncr: str,  # forward_lane_num_change_road_level
        flncs: float,  # forward_lane_num_change_s
):
    """
    获取laneSection连接处的车道数变化情况
    """
    if round(current_lane_section_s, 2) == flncs:
        return flncr
    if forward_lane_num is None:
        return ''

    if current_lane_num == forward_lane_num:
        return '前方车道数不变'
    elif current_lane_num < forward_lane_num:
        return '前方车道数增加'
    else:
        return '前方车道数减少'


def cut_lane_section(
        segment_id: List[str],
        segment_tags: List[List[str]],
        horizontal_feature: List[str],
        horizontal_cut_s: List[float],
        vertical_feature: List[str],
        vertical_cut_s: List[float],
) -> List[str]:
    """
    According to the transition points (i.e., cut_s), cut segment_id into several segments.
    """
    if (not horizontal_cut_s) and (not vertical_cut_s):
        return segment_id, segment_tags
    new_segment_id = []
    new_segment_tags = []
    # 每个cut_s是对应一个新线形的起点
    cut_s_list = list(set(horizontal_cut_s + vertical_cut_s))
    cut_s_list.sort()
    current_hrz_feature = horizontal_feature[0] if horizontal_feature else ''
    current_vtc_feature = vertical_feature[0] if vertical_feature else ''
    for cut_s in cut_s_list:
        if cut_s in horizontal_cut_s:
            current_hrz_feature = horizontal_feature[horizontal_cut_s.index(cut_s)]
        if cut_s in vertical_cut_s:
            current_vtc_feature = vertical_feature[vertical_cut_s.index(cut_s)]
        for i, original_seg_id in enumerate(segment_id):
            new_id = original_seg_id + f'_s-{cut_s}'
            new_segment_id.append(new_id)
            if 'left' in original_seg_id:
                seg_tag_index = 0
                left_lane_seg_tags = copy.deepcopy(segment_tags[seg_tag_index])
                left_lane_seg_tags.extend([current_hrz_feature, current_vtc_feature])
                new_segment_tags.append(left_lane_seg_tags)
            elif 'right' in original_seg_id:
                seg_tag_index = 0 if len(segment_tags) == 1 else 1
                right_lane_seg_tags = copy.deepcopy(segment_tags[seg_tag_index])
                right_lane_seg_tags.extend([current_hrz_feature, current_vtc_feature])
                new_segment_tags.append(right_lane_seg_tags)
            else:
                raise ValueError('Irregular segemnt id. "left" or "right" must be contained in segment id !')

    return new_segment_id, new_segment_tags


def detect_cycle(xodr_root: ET.Element) -> List[List[str]]:
    """
    Find the roundabout roads from the given link dict.
    """
    # build graph(successors only!!s)
    graph = nx.DiGraph()

    def find_successors_by_junction(road_id: str, target_junction: ET.Element) -> str:
        connections = target_junction.findall('connection')
        successors = []
        for cnc in connections:
            incoming_road_id = cnc.get('incomingRoad')
            connecting_road_id = cnc.get('connectingRoad')
            if road_id == incoming_road_id:
                successors.append(connecting_road_id)
        return successors

    for road in xodr_root.findall('road'):
        road_id = road.get('id')
        _, _, successor_type, successor_id = get_precessor_and_successor(road)
        if successor_id and successor_type != 'junction':
            graph.add_edge(road_id, successor_id)
            continue
        # 仅通过successor来增加edge，忽视了road可能是双向的情况。这是因为在环形交叉口中的连接段都是单向的，因此不影响环形交叉口的判断
        if successor_type == 'junction':
            try: 
                target_junction = [j for j in xodr_root.findall('junction') if j.get('id')==successor_id][0]
            except IndexError:
                continue
            successors = find_successors_by_junction(
                road_id=road_id,
                target_junction=target_junction,
            )
            for s in successors:
                graph.add_edge(road_id, s)
    cycles = list(nx.simple_cycles(graph))
    return cycles


# TODO:这里修了ET.Element|None → ET.Element or None
def run_scn_element_extraction(xodr_root: ET.Element, xosc_root: ET.Element or None, base_filename: str,
                               json_data: dict) -> pd.DataFrame:
    """
    Given scenario xml file, split the road into several retrieval unit, and extract there elements.

    All of the elements are represented as a "tag":str.

    The purpose of splitting is to make sure the tags in a unit remain same.
    """
    all_scn_elements = pd.DataFrame(columns=['segment_id', 'tag'])
    # step 1: 提取xosc中的共性tag
    if xosc_root is not None:
        # 环境层信息
        env = xosc_root.find('.//Environment')
        # 天气
        weather_info = extract_weather_info(env)
        # 时间
        time_info = extract_time_info(env)
        # 出行时段
        travel_period = extract_travel_period(env)
        # 工作日情况
        weekday_info = extract_weekday_info(env)
        # 能见度
        visibility_info = extract_visibility_info(env)
    else:
        weather_info, time_info, travel_period, weekday_info, visibility_info = '', '', '', '', ''

    # 交通管理层
    # traffic_management_info = extract_traffic_management(xosc_root)
    traffic_management_info = '无交通警察指挥'

    # step 2 :针对每一个Road提取道路共性tag
    # 先建立道路之间的连接关系
    link_dict = {}
    for road in xodr_root.findall('road'):
        road_id = road.get('id')
        predecessor_type, predecessor_id, successor_type, successor_id = get_precessor_and_successor(road)
        link_dict[road_id] = {
            'predecessor_type': predecessor_type, 'predecessor_id': predecessor_id,
            'successor_type': successor_type, 'successor_id': successor_id
        }
    # 检测是否有环路，供后续判断交叉口是否为环形交叉口
    # detect whether there is a cycle in the road network
    cycles = detect_cycle(xodr_root)

    # 分别提取各个road的要素
    for road in xodr_root.findall('road'):
        road_id = road.get('id', 'unknown')
        # print(road_id)
        # 道路类型
        road_type = extract_road_type(road, base_filename)
        # 设施类型
        facility_type = extract_facility_type(road, xodr_root, cycles, base_filename)
        # 交通信号灯控制情况
        traffic_light = extract_traffic_light_info(facility_type, json_data)

        # 主辅路类型
        # main_auxiliary_road_type = extract_main_auxiliary_road_type(road)
        # 道路通行方向
        road_direction = extract_road_direction(road)

        # 周边设施情况
        # 暂时空缺

        # 平面线形（暂时不考虑该特征）
        # horizontal_feature,horizontal_cut_s = extract_horizontal_alignment(road,road_type,facility_type)
        horizontal_feature, horizontal_cut_s = ['正常线形'], [0.]

        # 纵断面特征
        vertical_feature, vertical_cut_s = extract_vertical_alignment(road, road_type)

        # 交通标志
        traffic_sign = extract_traffic_sign_info(road)

        # 停止线
        stop_line = extract_stop_line(road)

        # 前方车道数变化
        left_lane_forward_lane_num_change_at_road_level, \
            right_lane_forward_lane_num_change_at_road_level, \
            left_ego_s, left_forward_s, right_ego_s, right_forward_s = \
            extract_forward_lane_num_change_at_road_level(road, link_dict, facility_type, xodr_root)

        # step 3: 针对每一个laneSection提取道路特性tag
        # build the topology of the lane_section(s)
        topology = []
        for i, lane_section in enumerate(road.findall('.//laneSection')):
            lane_section_s = float(lane_section.get('s'))
            left_lanes = lane_section.find('left')
            right_lanes = lane_section.find('right')
            _, left_lane_num_int = extract_lane_num(left_lanes)
            _, right_lane_num_int = extract_lane_num(right_lanes)
            topology.append([lane_section_s, left_lane_num_int, right_lane_num_int])

        for ls_order, lane_section in enumerate(road.findall('.//laneSection')):
            """
            交通信号灯
            交通标志
            """
            lane_section_s = float(lane_section.get('s'))
            # Left
            left_lanes = lane_section.find('left')
            # Right
            right_lanes = lane_section.find('right')
            segment_id = []
            segment_tags = []
            segment_description = []

            # 每一条车道的信息(for left lanes)
            if left_lanes is not None:
                left_lane_num, left_lane_num_int = extract_lane_num(left_lanes)
                # find the forward lane_section
                forward_left_lane_num_int = None
                for i in range(len(topology)):
                    if (topology[i][0] == lane_section_s) and i > 0:
                        forward_left_lane_num_int = topology[i - 1][1]

                # 前方车道数变化
                left_forward_lane_change = extract_forward_lane_num_change_at_laneSection_level(
                    current_lane_section_s=lane_section_s,
                    current_lane_num=left_lane_num_int,
                    forward_lane_num=forward_left_lane_num_int,
                    flncr=left_lane_forward_lane_num_change_at_road_level,
                    flncs=left_ego_s,
                )

                left_lane_info = [left_lane_num, left_forward_lane_change]
                for lane in left_lanes.findall('.//lane'):
                    lane_type = extract_lane_type(lane)
                    lane_access = extract_lane_access(lane)
                    lane_width = extract_lane_width(lane, lane_type)
                    lane_mark = extract_lane_mark(lane, left_lanes, facility_type)
                    wo_lane_mark = extract_wo_lane_mark(lane_mark)
                    left_lane_info.extend([lane_type, lane_access, lane_width, lane_mark, wo_lane_mark])

                segment_id.append(f"Road-{road_id}_LaneSection-{ls_order}_left")
                segment_tags.append(left_lane_info)
            else:
                left_lane_num = 0
                left_lane_info = []

            # 每一条车道的信息(for right lanes)
            if right_lanes is not None:
                right_lane_num, right_lane_num_int = extract_lane_num(right_lanes)
                # find the forward lane_section
                forward_right_lane_num_int = None
                for i in range(len(topology)):
                    if (topology[i][0] == lane_section_s) and i < len(topology) - 1:
                        forward_right_lane_num_int = topology[i + 1][1]
                right_forward_lane_change = extract_forward_lane_num_change_at_laneSection_level(
                    current_lane_section_s=lane_section_s,
                    current_lane_num=right_lane_num_int,
                    forward_lane_num=forward_right_lane_num_int,
                    flncr=right_lane_forward_lane_num_change_at_road_level,
                    flncs=right_ego_s,
                )

                right_lane_info = [right_lane_num, right_forward_lane_change]
                for lane in right_lanes.findall('.//lane'):
                    lane_type = extract_lane_type(lane)
                    lane_access = extract_lane_access(lane)
                    lane_width = extract_lane_width(lane, lane_type)
                    lane_mark = extract_lane_mark(lane, right_lanes, facility_type)
                    wo_lane_mark = extract_wo_lane_mark(lane_mark)
                    right_lane_info.extend([lane_type, lane_access, lane_width, lane_mark, wo_lane_mark])
                segment_id.append(f"Road-{road_id}_LaneSection-{ls_order}_right")
                segment_tags.append(right_lane_info)
            else:
                right_lane_num = 0
                right_lane_info = []

            # for center lane
            central_mark = extract_lane_mark(
                lane=lane_section.find('center/lane'),
                lanes=lane_section.find('center'),
                facility_type=facility_type,
            )
            if left_lane_num and right_lane_num:
                # 道路中央隔离情况
                central_divider = extract_central_divider(lane_section, road_type)
            else:
                central_divider = ''
            # if cut_s is activated, then the segment should be further divided
            segment_id, segment_tags = cut_lane_section(
                segment_id,
                segment_tags,
                horizontal_feature,
                horizontal_cut_s,
                vertical_feature,
                vertical_cut_s,
            )
            # print(segment_id)
            add_tags = [central_divider, central_mark]
            all_scn_elements = update_res(
                all_scn_elements=all_scn_elements,
                new_segment_id=segment_id,
                new_segment_tags=segment_tags,
                add_tags=add_tags,
                target_segment_id_prefix=[f"Road-{road_id}_LaneSection-{ls_order}_"]
            )

        add_tags = [road_type, facility_type, traffic_light, road_direction, traffic_sign, stop_line]
        all_scn_elements = update_res(
            all_scn_elements=all_scn_elements,
            add_tags=add_tags,
            target_segment_id_prefix=[f"Road-{road_id}_"]
        )
    add_tags = [weather_info, time_info, travel_period, weekday_info, visibility_info, traffic_management_info]
    all_scn_elements = update_res(
        all_scn_elements=all_scn_elements,
        add_tags=add_tags,
        target_segment_id_prefix=['']
    )

    return all_scn_elements


def update_res(
        all_scn_elements: pd.DataFrame,
        new_segment_id: List[str] = [],
        new_segment_tags: List[List[str]] = [],
        add_tags: List[str] = [],
        target_segment_id_prefix: List[str] = [],
) -> pd.DataFrame:
    """
    Update the extractin results, including:
        1) add new segment id and segment tags to the all_scn_elements:pd.DataFrame
        2) add add_tags to the existing segment in all_scn_elements:pd.DataFrame
    """

    # check whether the given arguments are valid
    if len(new_segment_id) != len(new_segment_tags):
        raise ValueError('The length of new_segment_id and new_segment_tags should be the same!')
    if len(add_tags) > 0 and len(target_segment_id_prefix) == 0:
        raise ValueError('If add_tags is given, then target_segment_id_prefix should also be given!')
    for i in range(len(new_segment_id)):
        seg_id = new_segment_id[i]
        # employ string to store segment tag names
        seg_tag = ';'.join([t for t in new_segment_tags[i] if t])
        new = pd.DataFrame({'segment_id': [seg_id], 'tag': [seg_tag]})
        all_scn_elements = pd.concat([all_scn_elements, new], axis=0).reset_index(drop=True)
    if add_tags:
        add_tags = [t for t in add_tags if t]
        for i in range(len(all_scn_elements)):
            seg_id = all_scn_elements.loc[i, 'segment_id']
            # if seg_id not startswith any of the given prefix, then skip this segment
            if target_segment_id_prefix and not any([seg_id.startswith(prefix) for prefix in target_segment_id_prefix]):
                continue
            tmp = all_scn_elements.loc[i, 'tag']
            tmp += ';' + ';'.join(add_tags)
            all_scn_elements.loc[i, 'tag'] = tmp.strip(';')

    return all_scn_elements


def remove_duplicate_tags(df: pd.DataFrame) -> pd.DataFrame:
    """
    Remove duplicate tags in the dataframe's tag column
    """
    df['tag'] = df['tag'].apply(lambda x: ';'.join(list(set(x.split(';')))))
    return df


def tag_classification(df: pd.DataFrame, scenario_id: str) -> pd.DataFrame:
    """
    Classify the tags to (road,infrastructure,management,environment) four categories.

    The classification is based on the tag name, which has been pre-defined in the TagTree structure.
    """
    tree = TagTree()
    for i in range(df.shape[0]):
        tags = df.loc[i, 'tag']
        tags_lst = [tree.find_tag(t)[0] for t in tags.split(';') if t]
        road_tags, infr_tags, man_tags, env_tags = [], [], [], []
        for t in tags_lst:
            if t.code.startswith('/root/道路'):
                road_tags.append(t.name)
            elif t.code.startswith('/root/基础设施'):
                infr_tags.append(t.name)
            elif t.code.startswith('/root/交通管理'):
                man_tags.append(t.name)
            elif t.code.startswith('/root/环境'):
                env_tags.append(t.name)
            else:
                print('Unknown tag:', t.name)
        road_tags = list(set(road_tags))
        infr_tags = list(set(infr_tags))
        man_tags = list(set(man_tags))
        env_tags = list(set(env_tags))
        df.loc[i, ROAD_TAG_COL] = ';'.join(road_tags)
        df.loc[i, INFR_TAG_COL] = ';'.join(infr_tags)
        df.loc[i, MAN_TAG_COL] = ';'.join(man_tags)
        df.loc[i, ENV_TAG_COL] = ';'.join(env_tags)
    df['scenario_id'] = scenario_id
    return df[[SCENARIO_ID_COL, SEGMENT_ID_COL, ROAD_TAG_COL, INFR_TAG_COL, MAN_TAG_COL, ENV_TAG_COL]]


def scn_element_extract_pipeline(
        xodr_files: List[str],
        xosc_files: List[str],
        json_files: List[str],
        # output_folder: str,
        # merged_res_save_path: str,
        # if_save_each: bool = False,
):
    if len(xodr_files) != len(xosc_files):
        raise ValueError("XODR 文件和 XOSC 文件数量不匹配！")
        return
    all_res = pd.DataFrame(
        columns=[SCENARIO_ID_COL, SEGMENT_ID_COL, ROAD_TAG_COL, INFR_TAG_COL, MAN_TAG_COL, ENV_TAG_COL])
    # 对每个文件对进行处理
    for xodr_file, xosc_file, json_file in tqdm(zip(xodr_files, xosc_files, json_files), total=len(xodr_files)):
        if not os.path.exists(xodr_file):
            raise ValueError(f"文件路径不存在：{xodr_file}")
            continue

        # 获取文件的基本名称用于输出
        base_filename = os.path.splitext(os.path.basename(xodr_file))[0]

        # 解析 XODR 和 XOSC 文件
        xodr_tree = ET.parse(xodr_file)
        xodr_root = xodr_tree.getroot()
        xosc_tree = ET.parse(xosc_file) if xosc_file is not None else None
        xosc_root = xosc_tree.getroot() if xosc_tree is not None else None

        # load json file when it is NOT none
        if json_file is not None:
            with open(json_file, 'r', encoding='utf-8') as f:
                json_data = json.load(f)
        else:
            json_data = {}

        res = run_scn_element_extraction(xodr_root, xosc_root, base_filename, json_data)
        # drop duplicate tags
        res = remove_duplicate_tags(res)
        res = tag_classification(res, base_filename)

        # save results
        # wth:save_each取消
        # if not os.path.exists(output_folder):
        #     os.makedirs(output_folder)
        # if if_save_each:
        #     res.to_csv(f'{output_folder}/{base_filename}_tag.csv', index=False)

        all_res = pd.concat([all_res, res], axis=0).reset_index(drop=True)

        # print(f'{base_filename} Done!')
        # break

    # save merged results as csv file (note that if saving as xlsx file, the large file may be bcorrupted)
    # wth: save_merged取消
    # all_res.to_csv(merged_res_save_path, index=False)
    return all_res


def run_extractor(scn_folder_name: str,
                  # merged_res_filename: str,
                  # if_save_each: bool = False
                  ):
    # define paths
    input_folder = os.path.join(Parameter.map_file, scn_folder_name)
    if not os.path.exists(input_folder):
        raise ValueError(f"Error: The input folder '{input_folder}' does not exist!")
    # output_folder = CACHE_DIR
    # merged_res_save_path = os.path.join(output_folder, merged_res_filename)

    # wth:这里只读一个场景
    # scn_lst = os.listdir(scenerio_name)
    scn_lst = [scn_folder_name]
    xodr_files, xosc_files = [], []
    json_files = []
    for scn_folder in scn_lst:
        scn_path = os.path.join(Parameter.map_file, scn_folder_name)
        if not os.path.isdir(scn_path):
            continue
        file_lst = os.listdir(scn_path)
        xodr_s = [os.path.join(scn_path, f) for f in file_lst if f.endswith('.xodr')]
        xosc_s = [os.path.join(scn_path, f) for f in file_lst if f.endswith('.xosc')]
        json_s = [os.path.join(scn_path, f) for f in file_lst if f.endswith('.json') and not f.startswith("all_road_info_file") and not f.startswith("rule_list")]
        # skip the lefthand scenarios
        if is_lefthand_scn(scn_name=scn_folder):
            continue

        # check if ONLY One
        if len(xodr_s) > 1 or len(xosc_s) > 1:
            print(f'Error: {scn_folder} has more than one xodr or xosc file!')
            continue
        if len(json_s) > 1:
            print(f'Error: {scn_folder} has more than one json file!')
            continue
        if not xodr_s:
            print(f'Error: {scn_folder} has no xodr file!')
            continue
        if not xosc_s:
            warnings.warn(
                f'Warning: {scn_folder} has no xosc file! This will not affect the following tag extraction, but the env-related tags will not be extracted.',
                UserWarning
            )
            xosc_s = [None]
        xodr_files.append(xodr_s[0])
        xosc_files.append(xosc_s[0])
        if json_s:
            json_files.append(json_s[0])
        else:
            json_files.append(None)

    print('{} xodr files and {} xosc files to be processed...'.format(len(xodr_files), len(xosc_files)))
    # print('The extraction results will be saved at:', output_folder)

    merged_res = scn_element_extract_pipeline(
        xodr_files=xodr_files,
        xosc_files=xosc_files,
        json_files=json_files,
        # output_folder=output_folder,
        # merged_res_save_path=merged_res_save_path,
        # if_save_each=if_save_each,
    )
    # print('The merged results have been saved at:', merged_res_save_path)
    return merged_res


