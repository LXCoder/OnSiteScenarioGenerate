import os
import json
import numpy as np
from lxml import etree
from opendrive2tessng.opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2tessng.opendrive2lanelet.opendriveparser.elements.opendrive import OpenDrive
from typing import List, Dict
from opendrive2tessng.opendrive2lanelet.opendriveparser.elements.roadLanes import Lane
from opendrive2tessng.utils.convert_utils import convert_opendrive, convert_roads_info, convert_lanes_info
from scipy import special
import math


# none: 对线条无要求; broken：虚线; solid: 实线; broken broken: 双虚线；solid soild: 双实线;
road_mark_type_dict = {"others": 0, "none": 1, "broken": 2, "broken broken": 3, "solid": 4, "solid solid": 5}

class Section:
    def __init__(self, road_id, section_id, lane_ids: list):
        self.road_id = road_id
        self.id = section_id
        self._left_link = None
        self._right_link = None
        self.lane_ids = list(lane_ids or [])
        # 左右来向的车道id分别为正负， 需要根据tess的规则进行排序
        self.left_lane_ids = sorted(filter(lambda i: i > 0, self.lane_ids, ), reverse=True)
        self.right_lane_ids = sorted(filter(lambda i: i < 0, self.lane_ids, ), reverse=False)
        self.lane_mapping = {}

    @property
    def left_link(self):
        return self._left_link

    @left_link.setter
    def left_link(self, obj):
        for link_info in obj:
            link = link_info['link']
            if not link:
                continue
            lane_ids = link_info['lane_ids']
            for index, lane in enumerate(link.lanes()):
                link_info[lane_ids[index]] = lane
        self._left_link = obj

    @property
    def right_link(self):
        return self._right_link

    @right_link.setter
    def right_link(self, obj):
        for link_info in obj:
            link = link_info['link']
            # 路段创建失败时，link 为 None
            if not link:
                continue
            lane_ids = link_info['lane_ids']
            for index, lane in enumerate(link.lanes()):
                link_info[lane_ids[index]] = lane
        self._right_link = obj

    def tess_lane(self, lane_id, type):
        try:
            attr = self.left_link if lane_id > 0 else self.right_link
            link_index = -1 if type == 'from' else 0
            return attr[link_index].get(lane_id)
        except:
            return None

    def tess_link(self, lane_id, type):
        try:
            attr = self.left_link if lane_id > 0 else self.right_link
            link_index = -1 if type == 'from' else 0
            return attr[link_index]['link']
        except:
            return None

class Road:
    def __init__(self, road_id):
        self.id = road_id
        self.sections = []

    def section(self, section_id: int = None):
        if section_id is None:
            return self.sections
        else:
            for section in self.sections:
                if section.id == section_id:
                    return section

    def section_append(self, section: Section):
        self.sections.append(section)
        self.sections.sort(key=lambda i: i.id)

class Network:
    def __init__(self, opendrive: OpenDrive, file_name: str = None):
        """
            TessNg 路网对象初始化
        Args:
            opendrive:
            file_name: 纯粹的文件名称
        """
        self.file_name = file_name or ""
        self.opendrive = opendrive
        self.network_info = None
        self.xy_move = (0, 0)
        self.size = (300, 600)
        self.step = None

    def extract_network_info(self, step: int = None, filters: List[str] = None, context: Dict = None):
        """
            借助 opendrive2lanelet 开源库提取opendrive基础信息，包括路段/车道三维信息以及车道间的连接关系
        Args:
            step: 提取信息时的精度
            filters: 过滤的opendrive车道类型
            context: 上下文信息

        Returns:

        """
        step = step or 1
        self.step = step
        filters = filters or Lane.laneTypes
        opendrive = self.opendrive
        # 头信息
        header_info = {
            "date": opendrive.header.date,
            "geo_reference": opendrive.header.geo_reference,
            "name": opendrive.header.name,
        }

        # 参考线信息解析
        roads_info = convert_roads_info(opendrive, step, filters)

        # 车道点位序列不再独立计算，采用 road info 中参考线的点位
        # 车道信息解析，这一步最消耗时间，允许传入进度条
        scenario = convert_opendrive(opendrive, filters, roads_info, context)
        lanes_info = convert_lanes_info(opendrive, scenario, roads_info)
        network_info = {
            "header_info": header_info,
            "roads_info": roads_info,
            "lanes_info": lanes_info,
        }
        return network_info

    def convert_network(self, step=None, filters=None, context=None):
        """
        提取opendrive路网基础信息

        Args:
            step:
            filters:
            context:

        Returns:

        """
        try:
            self.network_info = self.extract_network_info(step, filters, context)
            header_info = self.network_info["header_info"]
            roads_info = self.network_info["roads_info"]
            lanes_info = self.network_info["lanes_info"]

            xy_limit = None
            for road_id, road_info in roads_info.items():
                # 记录 坐标点的极值 (取左右point列表无区别，只是计算方向不同)
                for section_id, points in road_info['road_points'].items():
                    for point in points['right_points']:
                        position = point['position']
                        if xy_limit is None:  # x1,x2,y1,y2
                            xy_limit = [position[0], position[0], position[1], position[1]]
                        else:
                            xy_limit[0] = min(xy_limit[0], position[0])
                            xy_limit[1] = max(xy_limit[1], position[0])
                            xy_limit[2] = min(xy_limit[2], position[1])
                            xy_limit[3] = max(xy_limit[3], position[1])
            self.xy_move = (- sum(xy_limit[:2]) / 2, - sum(xy_limit[2:]) / 2) if xy_limit else (0, 0)
            self.size = (max(abs(xy_limit[0]), abs(xy_limit[1])) * 2, max(abs(xy_limit[2]), abs(xy_limit[3])) * 2)
            print(f"路网移动参数: {self.xy_move}")

            for lane_name, lane_info in lanes_info.items():
                if not lane_info:  # 此车道只是文件中某车道的前置或者后置车道，仅仅被提及，是空信息，跳过
                    continue
                road_id = lane_info['road_id']
                section_id = lane_info['section_id']
                lane_id = lane_info['lane_id']
                if road_id not in roads_info.keys():
                    continue

                # 添加默认属性
                roads_info[road_id].setdefault('sections', {})
                roads_info[road_id]['sections'].setdefault(section_id, {})
                roads_info[road_id]['sections'][section_id].setdefault('lanes', {})
                roads_info[road_id]['sections'][section_id]["lanes"][lane_id] = lane_info

        except Exception as e:
            print(f"convert_network error: {e}")

class EulerSpiral:
    """ """

    def __init__(self, gamma):
        self._gamma = gamma

    @staticmethod
    def createFromLengthAndCurvature(length, curvStart, curvEnd):
        """Create an EulerSpiral from a given length with curveStart
        and curvEnd. This is how the OpenDrive format specifies
        EulerSpirals.

        Args:
          length: Length of EulerSpiral.
          curvStart: Curvature at start of EulerSpiral.
          curvEnd: Curvature at end of EulerSpiral.

        Returns:
          EulerSpiral: A new Clothoid.

        """
        # if length is zero, assume zero curvature
        if length == 0:
            return EulerSpiral(0)
        return EulerSpiral(1 * (curvEnd - curvStart) / length)

    def calc(self, s, x0=0, y0=0, kappa0=0, theta0=0):
        """

        Args:
          s:
          x0:  (Default value = 0)
          y0:  (Default value = 0)
          kappa0:  (Default value = 0)
          theta0:  (Default value = 0)

        Returns:

        """

        # Start
        C0 = x0 + 1j * y0

        if self._gamma == 0 and kappa0 == 0:
            # Straight line
            Cs = C0 + np.exp(1j * theta0 * s)

        elif self._gamma == 0 and kappa0 != 0:
            # Arc
            Cs = C0 + np.exp(1j * theta0) / kappa0 * (
                    np.sin(kappa0 * s) + 1j * (1 - np.cos(kappa0 * s))
            )

        else:
            # Fresnel integrals
            Cs = self._calc_fresnel_integral(s, kappa0, theta0, C0)

        # Tangent at each point
        theta = self._gamma * s ** 2 / 2 + kappa0 * s + theta0

        return (Cs.real, Cs.imag, theta)

    def _calc_fresnel_integral(self, s, kappa0, theta0, C0):
        Sa, Ca = special.fresnel(
            (kappa0 + self._gamma * s) / np.sqrt(np.pi * np.abs(self._gamma))
        )
        Sb, Cb = special.fresnel(kappa0 / np.sqrt(np.pi * np.abs(self._gamma)))

        # Euler Spiral
        Cs1 = np.sqrt(np.pi / np.abs(self._gamma)) * np.exp(
            1j * (theta0 - kappa0 ** 2 / 2 / self._gamma)
        )
        Cs2 = np.sign(self._gamma) * (Ca - Cb) + 1j * Sa - 1j * Sb

        Cs = C0 + Cs1 * Cs2

        return Cs

# 获取车道信息
def get_lane_info(lane_info, lane_list,  s, singleSide):
    for lane in lane_info.findall("lane"):
        lane_id = int(lane.get("id"))
        lane_type = lane.get("type")
        all_road_mark = lane.findall("roadMark")
        all_lane_access = lane.findall("access")
        road_mark_list = []
        if len(all_road_mark) > 0:
            for road_mark in all_road_mark:
                road_mark_sOffset = float(road_mark.get("sOffset"))
                road_mark_type = road_mark.get("type")
                road_mark_param = [road_mark_sOffset, road_mark_type]
                road_mark_list.append(road_mark_param)
        else:
            road_mark_param = [None, None, None]
            road_mark_list.append(road_mark_param)
        lane_access_list = []
        if len(all_lane_access) > 0:
            for lane_access in all_lane_access:
                lane_sOffset = float(lane_access.get("sOffset"))
                lane_rule = lane_access.get("rule")
                lane_restriction = lane_access.get("restriction")
                access_param = [lane_sOffset, lane_rule, lane_restriction]
                lane_access_list.append(access_param)
        else:
            access_param = [None, None, None]
            lane_access_list.append(access_param)
        lane_list.append([[lane_id, s, singleSide], road_mark_list, lane_access_list])
    return lane_list

# 用于获取opendrive文件中参考中心线各个参数
def parser_opendrive_new(root_node):
    """
    Args:
      root_node: xodr文件

    Returns:
        road_list: 提取的道路数据

    """
    # Only accept lxml element
    if not etree.iselement(root_node):
        raise TypeError("Argument root_node is not a xml element")

    # opendrive解析
    # 用于提取opendrive中的数据，存为可用的json文件数据，用于相关指标评价
    # 后续再需要其他参数自行提取即可，目前值提取涉及指标计算的部分
    all_road_list = []
    all_road_dict = {}
    for road in root_node.findall("road"):
        road_dict = {}
        # 获取道路的id
        road_id = int(road.get("id"))
        road_dict['road_id'] = road_id
        # 获取该道路的长度
        road_length = float(road.get("length"))
        road_dict['road_length'] = road_length
        # 获取该道路的名称
        road_name = road.get("name")
        road_dict['road_name'] = road_name

        # 获取道路的类型，限速
        road_type_list = []
        for road_type in road.findall("type"):
            # 记录的是对应在s-t坐标系下s的值（s-t坐标系），在<planView>-<geometry>元素中记录的第一个x，y的值,作为s-t的原点s-t
            # 如果s的值为d，则表示从距离s-t坐标原点的s轴的x的位置开始计算，即x+d初开始计算，利用道路参考线对应的曲线函数进行计算
            start_pos = float(road_type.get("s"))
            # 记录该道路所属的类型，如高速公路，乡村公路
            use_type = road_type.get("type")
            # 记录该道路上的最大速度（限速）和速度单位
            limit_speed = road_type.find("speed")
            if limit_speed is None:
                limit_speed_data = None
                limit_speed_unit = None
            else:
                limit_speed_data = float(limit_speed.attrib['max'])
                limit_speed_unit = limit_speed.attrib['unit']
                # limit_speed_data = float(limit_speed.max)
                # limit_speed_unit = limit_speed.unit
            road_type_list.append([start_pos, use_type, limit_speed_data, limit_speed_unit])
        road_dict['road_type_list'] = road_type_list

        # 获取该路段与那些路段连接，这里主要是考虑停止线停车的问题
        # 当前xodr文件中的<objects>没有相关停止线，所以暂时以有人行横道的位置来确认停止线
        # 思路
        # 先判断该路段中是否包含人行横道，有的话继续下面的操作
        # 人行横道的位置可以选其中一个点，然后计算与其连接的道路之间（左右两侧的最开始的两个点构成的线段和最后的两个点构成的线段，后续会解析成lanlet数据格式中有）的距离
        # 选取小距离的两个点作为停止线
        # 原因
        # 通过可视化观察xodr文件（可以用https://odrviewer.io/网站查看），发现该路段中包含人行横道的数据时，
        # 后续如果有停止线记录，则使用即可，目前不用
        # 可以通过“contactPoint”中的start和end来判断连接的点是该路段的最开始的左右两个点还是最后面两个点
        # for predecessor in road.link.predecessor:
        # 上一个连接路段
        opendrive_road_link = road.find("link")
        if opendrive_road_link is not None:
            predecessor = opendrive_road_link.find("predecessor")
            if predecessor is None:
                road_p_elementId = None
                road_p_contactPoint = None
            else:
                road_p_elementId = int(predecessor.get("elementId"))
                road_p_contactPoint = predecessor.get("contactPoint")
            predecessor_list = [road_p_elementId, road_p_contactPoint]
            road_dict['predecessor_list'] = predecessor_list

            # 下一个连接路段
            successor = opendrive_road_link.find("successor")
            if successor is None:
                road_s_elementId = None
                road_s_contactPoint = None
            else:
                road_s_elementId = int(successor.get("elementId"))
                road_s_contactPoint = successor.get("contactPoint")
            successor_list = [road_s_elementId, road_s_contactPoint]
            road_dict['successor_list'] = successor_list

            # 相邻连接段
            # neighbor暂时没有用到
        else:
            road_dict['predecessor_list'] = []
            road_dict['successor_list'] = []

        # 获取所有的道路的道路参考线
        road_reference_line_param_list = []
        for road_geometry in road.find("planView").findall("geometry"):
            s_pos = float(road_geometry.get("s"))
            startCoord = [float(road_geometry.get("x")), float(road_geometry.get("y"))]
            start_pos_x = float(road_geometry.get("x"))
            start_pos_y = float(road_geometry.get("y"))
            if road_geometry.find("line") is not None:
                hdg = float(road_geometry.get("hdg"))
                length = float(road_geometry.get("length"))
                param = ['line', s_pos, start_pos_x, start_pos_y, hdg, length]

            elif road_geometry.find("spiral") is not None:
                hdg = float(road_geometry.get("hdg"))
                length = float(road_geometry.get("length"))
                curvStart = float(road_geometry.find("spiral").get("curvStart"))
                curvEnd = float(road_geometry.find("spiral").get("curvEnd"))
                param = ['spiral', s_pos, start_pos_x, start_pos_y, hdg, length, curvStart, curvEnd]

            elif road_geometry.find("arc") is not None:
                hdg = float(road_geometry.get("hdg"))
                length = float(road_geometry.get("length"))
                curvature = float(road_geometry.find("arc").get("curvature"))
                param = ['arc', s_pos, start_pos_x, start_pos_y, hdg, length, curvature]

            elif road_geometry.find("poly3") is not None:
                hdg = float(road_geometry.get("hdg"))
                length = float(road_geometry.get("length"))
                a = float(road_geometry.find("poly3").get("a"))
                b = float(road_geometry.find("poly3").get("b"))
                c = float(road_geometry.find("poly3").get("c"))
                d = float(road_geometry.find("poly3").get("d"))
                param = ['poly3', s_pos, start_pos_x, start_pos_y, hdg, length, a, b, c, d]

            elif road_geometry.find("paramPoly3") is not None:
                hdg = float(road_geometry.get("hdg"))
                length = float(road_geometry.get("length"))
                aU = float(road_geometry.find("paramPoly3").get("aU"))
                bU = float(road_geometry.find("paramPoly3").get("bU"))
                cU = float(road_geometry.find("paramPoly3").get("cU"))
                dU = float(road_geometry.find("paramPoly3").get("dU"))
                aV = float(road_geometry.find("paramPoly3").get("aV"))
                bV = float(road_geometry.find("paramPoly3").get("bV"))
                cV = float(road_geometry.find("paramPoly3").get("cV"))
                dV = float(road_geometry.find("paramPoly3").get("dV"))
                if road_geometry.find("paramPoly3").get("pRange"):
                    if road_geometry.find("paramPoly3").get("pRange") == "arcLength":
                        pMax = float(road_geometry.get("length"))
                    else:
                        pMax = 1.0
                else:
                    pMax = 1.0
                param = ['paramPoly3', s_pos, start_pos_x, start_pos_y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV, pMax]
            else:
                param = []
                raise Exception("invalid xml")
            road_reference_line_param_list.append(param)
        road_dict['road_reference_line_param'] = road_reference_line_param_list

        # 获取所有道路的高程
        road_elevation_param_list = []
        road_elevation_profile = road.find("elevationProfile")
        if road_elevation_profile is not None:
            for elevation in road_elevation_profile.findall("elevation"):
                s_pos = float(elevation.get("s"))
                a = float(elevation.get("a"))
                b = float(elevation.get("b"))
                c = float(elevation.get("c"))
                d = float(elevation.get("d"))
                elevation_param = [s_pos, a, b, c, d]
                road_elevation_param_list.append(elevation_param)
            road_dict['road_elevation_param'] = road_elevation_param_list
        else:
            road_dict['road_elevation_param'] = []

        # 获取所有道路的倾斜角
        crossfall_param_list = []
        shape_param_list = []
        superelevation_param_list = []
        road_lateral_profile = road.find("lateralProfile")
        road_dict['road_lateral_param'] = {}
        if road_lateral_profile is not None:
            if road_lateral_profile.findall("superelevation") is not None:
                for superelevation in road_lateral_profile.findall("superelevation"):
                    a = float(superelevation.get("a"))
                    b = float(superelevation.get("b"))
                    c = float(superelevation.get("c"))
                    d = float(superelevation.get("d"))
                    s = float(superelevation.get("s"))
                    superelevation_param = ['superelevation', s, a, b, c, d]
                    superelevation_param_list.append(superelevation_param)
            road_dict['road_lateral_param']['superelevation_param'] = superelevation_param_list

            if road_lateral_profile.findall("crossfall") is not None:
                for crossfall in road_lateral_profile.findall("crossfall"):
                    a = float(crossfall.get("a"))
                    b = float(crossfall.get("b"))
                    c = float(crossfall.get("c"))
                    d = float(crossfall.get("d"))
                    side = crossfall.get("side")
                    s = float(crossfall.get("s"))
                    crossfall_param = ['crossfall', s, side, a, b, c, d]
                    crossfall_param_list.append(crossfall_param)
            road_dict['road_lateral_param']['crossfall_param'] = crossfall_param_list

            if road_lateral_profile.findall("shape") is not None:
                for shape in road_lateral_profile.findall("shape"):
                    a = float(shape.get("a"))
                    b = float(shape.get("b"))
                    c = float(shape.get("c"))
                    d = float(shape.get("d"))
                    s = float(shape.get("s"))
                    t = float(shape.get("t"))
                    shape_param = ['shape', s, t, a, b, c, d]
                    shape_param_list.append(shape_param)
            road_dict['road_lateral_param']['shape_param'] = shape_param_list

        else:
            road_dict['road_lateral_param'] = {}

        # 获取对应的车道
        lanes_center_list = []
        lanes_center_access_list = []
        lanes_left_list = []
        lanes_left_access_list = []
        lanes_right_list = []
        lanes_right_access_list = []
        lanes = road.find("lanes")
        road_dict['lanes'] = {}
        if lanes is None:
            road_dict['lanes'] = {}
            raise Exception("Road must have lanes element")
        else:
            for laneSection in road.find("lanes").findall("laneSection"):
                section_s = float(laneSection.get("s"))
                section_singleSide = laneSection.get("singleSide")
                lanes_center = laneSection.find("center")
                lanes_left = laneSection.find("left")
                lanes_right = laneSection.find("right")
                if lanes_center is None:
                    road_dict['lanes']['center'] = []
                else:
                    lanes_center_list = get_lane_info(lanes_center, lanes_center_list, section_s, section_singleSide)
                    road_dict['lanes']['center'] = lanes_center_list
                if lanes_left is None:
                    road_dict['lanes']['left'] = []
                else:
                    lanes_left_list = get_lane_info(lanes_left, lanes_left_list, section_s, section_singleSide)
                    road_dict['lanes']['left'] = lanes_left_list
                if lanes_right is None:
                    road_dict['lanes']['right'] = []
                else:
                    lanes_right_list = get_lane_info(lanes_right, lanes_right_list, section_s, section_singleSide)
                    road_dict['lanes']['right'] = lanes_right_list

        # 获取对应的车道，和停止线信息
        object_list = []
        lanes_objects = road.find("objects")
        road_dict['stop_line'] = None
        road_dict['lanes_objects'] = {}
        if lanes_objects is None:
            road_dict['lanes_objects'] = {}
            road_dict['stop_line'] = None
            # raise Exception("Road must have objects element")
        else:
            lanes_objects_len = len(lanes_objects)
            if lanes_objects_len > 0:
                for lane_object_len in lanes_objects.findall("object"):
                    object_s = float(lane_object_len.get("s"))
                    object_t = float(lane_object_len.get("t"))
                    object_type = lane_object_len.get("type")
                    object_param = [object_s, object_t, object_type]
                    object_list.append(object_param)
                    if object_type == "roadmark":
                        road_dict['stop_line'] = object_s
                road_dict['lanes_objects']['object_list'] = object_list
            else:
                road_dict['lanes_objects'] = {}
                road_dict['stop_line'] = None

        all_road_list.append(road_dict)
    all_road_dict['all_road_dict'] = all_road_list
    return all_road_dict

# 获取所有的交叉口数据
def get_junction_info(root_node):
    # Only accept lxml element
    all_junction_info = []
    junction_info_dict = {}
    if not etree.iselement(root_node):
        raise TypeError("Argument root_node is not a xml element")
    for junction in root_node.findall("junction"):
        junciton_dict = {}
        junction_id = int(junction.get("id"))
        all_connection_list = []
        for connection in junction.findall("connection"):
            connection_id = int(connection.get("id"))
            connectingRoad_id = int(connection.get("connectingRoad"))
            incomingRoad_id = int(connection.get("incomingRoad"))
            connection_dict = [connection_id, connectingRoad_id, incomingRoad_id]
            all_connection_list.append(connection_dict)
        junciton_dict['junction_id'] = junction_id
        junciton_dict['connection_info'] = all_connection_list
        all_junction_info.append(junciton_dict)
    junction_info_dict['all_junction_info'] = all_junction_info
    return all_junction_info

def main(xodr_file: str) -> Network:
    """
        初始化opendrive文件，得到原始的路网对象
    Args:
        xodr_file: 文件位置

    Returns:

    """
    with open(xodr_file, "r", encoding='utf-8') as file_in:
        root_node = etree.parse(file_in).getroot()
        opendrive = parse_opendrive(root_node)
        road_list = parser_opendrive_new(root_node)
        all_road_mark = parser_road_mark(root_node)
        all_junction_info = get_junction_info(root_node)

    file_name = os.path.splitext(os.path.split(xodr_file)[-1])[0]
    network = Network(opendrive, file_name)

    step_length = float(1)
    filters = None  # list(LANE_TYPE_MAPPING.keys())
    context = None
    network_info = network.extract_network_info(step_length, filters, context)
    header_info = network_info["header_info"]
    roads_info = network_info["roads_info"]
    lanes_info = network_info["lanes_info"]

    # 获取路段号以及车道角
    all_lane_info = []
    for keys, values in roads_info.items():
        lane_dict = {}
        lane_angle = []
        lane_dict['road_id'] = keys
        road_points = roads_info[keys]['road_points'][0]
        right_points = road_points['right_points']
        for i in range(len(right_points)):
            angles = right_points[i]['angle']
            lane_angle.append(angles)
            lane_dict['lane_angle'] = lane_angle
        all_lane_info.append(lane_dict)
    # 获取每个车道的信息
    # 后续需要额外增加的信息，再此处增加，但是首先要在lanes_info中增加，具体增加的内容可以查看lanelet.py文件中查看
    # 具体增加位置在convert_utils.py文件中convert_lanes_info函数中
    every_lane_info = []
    for keys, values in lanes_info.items():
        every_lane_dict = {}
        road_marks_list = []
        for i in range(len(all_lane_info)):
            if values['road_id'] == all_lane_info[i]['road_id']:
                every_lane_dict['road_id'] = all_lane_info[i]['road_id']
                every_lane_dict['lane_id'] = values['lane_id']
                every_lane_dict['type'] = values['type']
                every_lane_dict['center_vertices'] = values['center_vertices']
                every_lane_dict['left_vertices'] = values['left_vertices']
                every_lane_dict['right_vertices'] = values['right_vertices']
                every_lane_dict['lane_angle'] = all_lane_info[i]['lane_angle']
                every_lane_info.append(every_lane_dict)
    all_road_dict = {}
    all_road_dict['all_road_dict'] = every_lane_info
    return opendrive, all_road_dict, road_list, all_road_mark, all_junction_info, every_lane_info, header_info

# 根据道路参考线按照曲线的类型计算对应的位置
def calc_line_position(curve_type_list, ds):
    """

    Args:
      curve_type_list:

    Returns:
        x,y
        heading

    """
    # param = ['line', s_pos, start_pos_x, start_pos_y, hdg, length]
    if curve_type_list[0] == 'line':
        start_position = [curve_type_list[2], curve_type_list[3]]
        pos = start_position + np.array([ds * np.cos(curve_type_list[4]), ds * np.sin(curve_type_list[4])])
        tangent = curve_type_list[4]
        return (pos, tangent)

    # param = ['arc', s_pos, start_pos_x, start_pos_y, hdg, length, curvature]
    elif curve_type_list[0] == 'arc':
        start_position = [curve_type_list[2], curve_type_list[3]]
        c = curve_type_list[6]
        hdg = curve_type_list[4] - np.pi / 2
        a = 2 / c * np.sin(ds * c / 2)
        alpha = (np.pi - ds * c) / 2 - hdg
        dx = -1 * a * np.cos(alpha)
        dy = a * np.sin(alpha)
        pos = start_position + np.array([dx, dy])
        tangent = curve_type_list[4] + ds * curve_type_list[6]
        return (pos, tangent)

    # param = ['spiral', s_pos, start_pos_x, start_pos_y, hdg, length, curvStart, curvEnd]
    elif curve_type_list[0] == 'spiral':
        spiral = EulerSpiral.createFromLengthAndCurvature(curve_type_list[5], curve_type_list[6], curve_type_list[7])
        (x, y, t) = spiral.calc(ds, curve_type_list[2], curve_type_list[3], curve_type_list[6], curve_type_list[4],)
        return (np.array([x, y]), t)

    # param = ['poly3', s_pos, start_pos_x, start_pos_y, hdg, length, a, b, c, d]
    # Calculate new point in s_pos/t coordinate system
    elif curve_type_list[0] == 'poly3':
        coeffs = [curve_type_list[6], curve_type_list[7], curve_type_list[8], curve_type_list[9]]
        t = np.polynomial.polynomial.polyval(ds, coeffs)
        # Rotate and translate
        srot = ds * np.cos(curve_type_list[4]) - t * np.sin(curve_type_list[4])
        trot = ds * np.sin(curve_type_list[4]) + t * np.cos(curve_type_list[4])
        # Derivate to get heading change
        dCoeffs = coeffs[1:] * np.array(np.arange(1, len(coeffs)))
        tangent = np.polynomial.polynomial.polyval(ds, dCoeffs)
        return (np.array([curve_type_list[2], curve_type_list[3]]) + np.array([srot, trot]), curve_type_list[4] + tangent)

    # param = ['paramPoly3', s_pos, start_pos_x, start_pos_y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV, pMax]
    # Position
    elif curve_type_list[0] == 'paramPoly3':
        pos = (ds / curve_type_list[5]) * curve_type_list[14]

        coeffsU = [curve_type_list[6], curve_type_list[7], curve_type_list[8], curve_type_list[9]]
        coeffsV = [curve_type_list[10], curve_type_list[11], curve_type_list[12], curve_type_list[13]]

        x = np.polynomial.polynomial.polyval(pos, coeffsU)
        y = np.polynomial.polynomial.polyval(pos, coeffsV)

        xrot = x * np.cos(curve_type_list[4]) - y * np.sin(curve_type_list[4])
        yrot = x * np.sin(curve_type_list[4]) + y * np.cos(curve_type_list[4])

        # Tangent is defined by derivation
        dCoeffsU = coeffsU[1:] * np.array(np.arange(1, len(coeffsU)))
        dCoeffsV = coeffsV[1:] * np.array(np.arange(1, len(coeffsV)))

        dx = np.polynomial.polynomial.polyval(pos, dCoeffsU)
        dy = np.polynomial.polynomial.polyval(pos, dCoeffsV)

        tangent = np.arctan2(dy, dx)

        return (np.array([curve_type_list[2], curve_type_list[3]]) + np.array([xrot, yrot]), curve_type_list[4] + tangent)

    else:
        return ([], None)

# 获取人行横道上的一个点
def get_crosswalk_point(road_list):
    all_road_values_list = road_list['all_road_dict']
    crosswalk_points_temp = []
    for i in range(len(all_road_values_list)):
        crosswalk_list = []
        road_id = all_road_values_list[i]['road_id']
        road_reference_line_param = all_road_values_list[i]['road_reference_line_param']
        lanes_objects = all_road_values_list[i]['lanes_objects']

        # 获取该路段的曲线类型
        all_s_list = []
        if len(road_reference_line_param) > 0:
            for j in range(len(road_reference_line_param)):
                all_s = road_reference_line_param[j][1]
                all_s_list.append(all_s)
        # 获取人行横道的一个点，用于计算停止线
        if len(lanes_objects) > 0:
            object_list = lanes_objects['object_list']
            if len(object_list) > 0:
                for num in range(len(object_list)):
                    s_start = object_list[num][0]
                    crosswalk_list.append(s_start)
                all_index = []
                ds_list = []
                for cross_i in range(len(crosswalk_list)):
                    if len(all_s_list) == 1:
                        if all_s_list[0] <= crosswalk_list[cross_i]:
                            ds = crosswalk_list[cross_i] - all_s_list[0]
                            ds_list.append(ds)
                            all_index.append(0)
                            pos_info = calc_line_position(road_reference_line_param[0], ds)
                    else:
                        for s_i in range(len(all_s_list)):
                            if s_i + 1 < len(all_s_list):
                                if all_s_list[s_i] <= crosswalk_list[cross_i] < all_s_list[s_i + 1]:
                                    all_s_index = s_i
                                    ds = crosswalk_list[cross_i] - all_s_list[s_i]
                                    pos_info = calc_line_position(road_reference_line_param[s_i], ds)
                                    ds_list.append(ds)
                                    all_index.append(all_s_index)
                                elif crosswalk_list[cross_i] >= all_s_list[-1]:
                                    all_s_index = all_s_list.index(all_s_list[-1])
                                    ds = crosswalk_list[cross_i] - all_s_list[-1]
                                    pos_info = calc_line_position(road_reference_line_param[all_s_index], ds)
                                    ds_list.append(ds)
                                    all_index.append(all_s_index)
                    crosswalk_points_temp.append(pos_info)
        else:
            pos_info = []

            crosswalk_points_temp.append(pos_info)
    crosswalk_points = []

    for i in range(len(crosswalk_points_temp)):
        if crosswalk_points_temp[i] != []:
            crosswalk_points.append(crosswalk_points_temp[i])

    return crosswalk_points

# 根据停止线直接计算得到最终的点位列表
def get_stop_line_withStopLaneInfo(road_list, every_lane_info, all_junction_info):

    # all_stop_line_dict用于存放所有交叉口的所有停止线
    all_stop_line_dict = {}

    all_road_values_list = road_list['all_road_dict']
    # road_list的长度会比every_lane_info的短，因为every_lane_info是车道信息，前面是道路信息，一个道路可能有两个车道
    stop_line_info = {}
    # 如果路段id一致就放在一个single_road_stop_line_list的key里面
    single_road_stop_line_dict = {}
    for i in range(len(all_road_values_list)):
        if all_road_values_list[i]['stop_line'] != None:
            stop_line_info[all_road_values_list[i]["road_id"]] = all_road_values_list[i]['stop_line']
    # 这里是为了拿到stopline的信息，所以放在一个字典里就行，而且只需要做一次

    for junctionI in all_junction_info:
        junction_id = junctionI["junction_id"]
        # 下面的是用的roadid,所以这里要用connection_info中的第三个
        incomeRoadIdList = []
        for incomeRoadI in junctionI["connection_info"]:
            incomeRoadIdList.append(incomeRoadI[2])
        # 存放最终的所有的停止线坐标，一个大列表，包括很多中列表每个中列表是对应一个路，然后包括很多小列表，每个小列表是一个坐标
        all_stop_line_list = []
        for i in range(len(every_lane_info)):
            # 如果路段id在all_stop_line_list里面，证明这个车道是属于这个交叉口的
            if every_lane_info[i]["road_id"] in incomeRoadIdList:
                # 如果路段id在key里面，证明这个车道是有停止线的，就开始计算
                if every_lane_info[i]["road_id"] in stop_line_info:
                    right_point, left_point = get_single_road_stop_line(every_lane_info[i]["right_vertices"],every_lane_info[i]["left_vertices"], stop_line_info[every_lane_info[i]["road_id"]])
                    if every_lane_info[i]["road_id"] in single_road_stop_line_dict:
                        single_road_stop_line_dict[every_lane_info[i]["road_id"]].append(right_point)
                        single_road_stop_line_dict[every_lane_info[i]["road_id"]].append(left_point)
                    else:
                        single_road_stop_line_dict[every_lane_info[i]["road_id"]] = []
                        single_road_stop_line_dict[every_lane_info[i]["road_id"]].append(right_point)
                        single_road_stop_line_dict[every_lane_info[i]["road_id"]].append(left_point)

        for key in single_road_stop_line_dict:
            all_stop_line_list.append(single_road_stop_line_dict[key])
        # 一个交叉口信息对应一堆自己的停止线信息
        all_stop_line_dict[junction_id] = all_stop_line_list

    return all_stop_line_dict

def get_single_road_stop_line(right_vertices, left_vertices,distanceis):
    right = []
    left = []
    single_road_stop_line_list = []
    for i in range(len(right_vertices)):
        r = [right_vertices[i][0],right_vertices[i][1]]
        right.append(r)
        l = [left_vertices[i][0], left_vertices[i][1]]
        left.append(l)
    right_point, right_prev_point, right_next_point = find_coordinates_and_neighbors(right, distanceis)
    left_point, left_prev_point, left_next_point = find_coordinates_and_neighbors(left, distanceis)
    # center_point = [(right_point[0]+left_point[0])/2,(right_point[1]+left_point[1])/2]
    single_road_stop_line_list.append(right_point)
    single_road_stop_line_list.append(left_point)
    # single_road_stop_line_list.append(center_point)
    return right_point, left_point


# 下面是苏苏写的函数，计算距离和列表中点的位置,调用find_coordinates_and_neighbors，得到目标坐标，如果目标坐标不是给定的点则同时给出前一个坐标，和后一个坐标
def calculate_distance(point1, point2):
    return ((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2) ** 0.5

def linear_interpolation(point1, point2, target_distance):
    distance_between_points = calculate_distance(point1, point2)
    ratio = target_distance / distance_between_points
    target_x = point1[0] + ratio * (point2[0] - point1[0])
    target_y = point1[1] + ratio * (point2[1] - point1[1])
    return target_x, target_y
# 输入一个列表的坐标，和目标距离
def find_coordinates_and_neighbors(road_centerline, target_distance):
    current_distance = 0.0
    coordinates = [road_centerline[0]]  # 存储每个插值点的坐标，初始为第一个点的坐标
    prev_coordinates = None
    next_coordinates = None

    for i in range(len(road_centerline) - 1):
        current_point = road_centerline[i]
        next_point = road_centerline[i + 1]
        segment_distance = calculate_distance(current_point, next_point)
        if current_distance + segment_distance >= target_distance:
            remaining_distance = target_distance - current_distance
            target_x, target_y = linear_interpolation(current_point, next_point, remaining_distance)
            coordinates.append((target_x, target_y))
            prev_coordinates = current_point
            next_coordinates = next_point
            break

        current_distance += segment_distance
        coordinates.append(next_point)

    # 如果目标距离超过了整条道路长度，返回最后一个点
    if len(coordinates) == 1:
        coordinates.append(road_centerline[-1])
        prev_coordinates = road_centerline[-1]
        next_coordinates = None

    return coordinates[-1], prev_coordinates, next_coordinates

# # 根据道路连接段的信息获取路段上每条道路的的始末对应的左右两个点的信息
def get_stop_line(all_road_dict, road_list):
    all_road_values_list = road_list['all_road_dict']
    all_stop_line = []
    for i in range(len(all_road_values_list)):
        stop_line_info = {}
        lanes_objects = all_road_values_list[i]['lanes_objects']
        if len(lanes_objects) > 0:
            object_list = lanes_objects['object_list']
            road_id = all_road_values_list[i]['road_id']
            predecessor_list = all_road_values_list[i]['predecessor_list']
            successor_list = all_road_values_list[i]['successor_list']
            stop_line_info['road_id'] = road_id
            stop_line_info['object_list'] = object_list
            stop_line_info['predecessor_list'] = predecessor_list
            stop_line_info['successor_list'] = successor_list
            all_stop_line.append(stop_line_info)

    all_road_line_list = all_road_dict['all_road_dict']
    stop_line_list = []
    for i in range(len(all_stop_line)):
        stop_line_json = {}
        all_stop_line_point = []
        road_id = all_stop_line[i]['road_id']
        object_list = all_stop_line[i]['object_list']
        predecessor_list = all_stop_line[i]['predecessor_list']
        successor_list = all_stop_line[i]['successor_list']
        if len(object_list) > 0:
            for num in range(len(object_list)):
                all_predecessor_list = []
                all_successor_list = []
                # 获取所有可能的停止线数据
                if object_list[num][2] == "roadmark":
                    if len(predecessor_list) > 0:
                        for j in range(len(all_road_line_list)):
                            all_road_id = all_road_line_list[j]['road_id']
                            lane_id = all_road_line_list[j]['lane_id']
                            left_vertices = all_road_line_list[j]['left_vertices']
                            right_vertices = all_road_line_list[j]['right_vertices']
                            if all_road_id == predecessor_list[0]:  # and lane_id < 0:
                                if predecessor_list[1] == "end":
                                    stop_line_point = [[left_vertices[-1], right_vertices[-1]]]
                                elif predecessor_list[1] == "start":
                                    stop_line_point = [[left_vertices[0], right_vertices[0]]]
                                else:
                                    stop_line_point = [[left_vertices[0], right_vertices[0]], [left_vertices[-1], right_vertices[-1]]]

                                all_predecessor_list.append(stop_line_point)
                    else:
                        all_predecessor_list = []
                    if len(successor_list) > 0:
                        for j in range(len(all_road_line_list)):
                            all_road_id = all_road_line_list[j]['road_id']
                            lane_id = all_road_line_list[j]['lane_id']
                            left_vertices = all_road_line_list[j]['left_vertices']
                            right_vertices = all_road_line_list[j]['right_vertices']
                            if successor_list[0] == all_road_id: # and lane_id < 0:
                                if predecessor_list[1] == "end":
                                    stop_line_point = [[left_vertices[-1], right_vertices[-1]]]
                                elif predecessor_list[1] == "start":
                                    stop_line_point = [[left_vertices[0], right_vertices[0]]]
                                else:
                                    stop_line_point = [[left_vertices[0], right_vertices[0]], [left_vertices[-1], right_vertices[-1]]]
                                all_successor_list.append(stop_line_point)
                    else:
                        all_successor_list = []
        stop_line_json['road_id'] = road_id
        stop_line_json['all_predecessor_list'] = all_predecessor_list
        stop_line_json['all_successor_list'] = all_successor_list
        stop_line_list.append(stop_line_json)
    return stop_line_list

# 根据人行横道与各个停止线之间的距离获取最终的停止线
def cal_stop_line(all_stop_line, crosswalk_points):
    all_predecessor_point = []
    all_successor_point = []
    for i in range(len(all_stop_line)):
        predecessor_point_temp = []
        successor_point_temp = []
        stop_line_info = all_stop_line[i]
        predecessor_list = stop_line_info['all_predecessor_list']
        successor_list = stop_line_info['all_successor_list']

        if len(predecessor_list) > 0:
            for j in range(len(predecessor_list)):
                for m in range(len(predecessor_list[j])):
                    for n in range(len(predecessor_list[j][m])):
                        predecessor_point_temp.append([predecessor_list[j][m][n][0], predecessor_list[j][m][n][1]])
        else:
            predecessor_point_temp.append([])
        if len(successor_list) > 0:
            for j in range(len(successor_list)):
                for m in range(len(successor_list[j])):
                    for n in range(len(successor_list[j][m])):
                        successor_point_temp.append([successor_list[j][m][n][0], successor_list[j][m][n][1]])
        else:
            successor_point_temp.append([])
        all_predecessor_point.append(predecessor_point_temp)
        all_successor_point.append(successor_point_temp)
    predecessor_point = []
    predecessor_index = []
    for i in range(len(all_predecessor_point)):
        if all_predecessor_point[i][0] != []:
            predecessor_point.append([all_predecessor_point[i][0][0], all_predecessor_point[i][0][1]])
            predecessor_index.append(i)
    temp_predecessor_stop_line_info = []
    for i in range(len(crosswalk_points)):
        predecessor_distance_list_temp = []
        for j in range(len(predecessor_point)):
            predecessor_distance = np.sqrt((crosswalk_points[i][0][0] - predecessor_point[j][0]) ** 2 + (crosswalk_points[i][0][1] - predecessor_point[j][1]) ** 2)
            predecessor_distance_list_temp.append(predecessor_distance)
        try:
            min_index = predecessor_distance_list_temp.index(min(predecessor_distance_list_temp))
        except:
            continue
        min_value = min(predecessor_distance_list_temp)
        temp_predecessor_stop_line_info.append([i, min_index, min_value])

    successor_point = []
    successor_index = []
    for i in range(len(all_successor_point)):
        if all_successor_point[i][0] != []:
            successor_point.append([all_successor_point[i][0][0], all_successor_point[i][0][1]])
            successor_index.append(i)

    temp_successor_stop_line_info = []
    for i in range(len(crosswalk_points)):
        successor_distance_list_temp = []
        for j in range(len(successor_point)):
            successor_distance = np.sqrt((crosswalk_points[i][0][0] - successor_point[j][0]) ** 2 + (crosswalk_points[i][0][1] - successor_point[j][1]) ** 2)
            successor_distance_list_temp.append(successor_distance)
        min_index = successor_distance_list_temp.index(min(successor_distance_list_temp))
        min_value = min(successor_distance_list_temp)
        temp_successor_stop_line_info.append([i, min_index, min_value])

    stop_line = []
    for i in range(len(temp_predecessor_stop_line_info)):
        predecessor_dis = temp_predecessor_stop_line_info[i]
        successor_dis = temp_successor_stop_line_info[i]
        if predecessor_dis[2] >= successor_dis[2]:
            stop_line.append(all_successor_point[successor_index[successor_dis[1]]])
        else:
            stop_line.append(all_predecessor_point[predecessor_index[predecessor_dis[1]]])
    return stop_line

# 获取道路上每个车道的使用规则
def get_lane_access(road_list):
    all_road_values_list = road_list['all_road_dict']
    lane_access = []
    for i in range(len(all_road_values_list)):
        left_lane_info = []
        right_lane_info = []
        road_id = all_road_values_list[i]['road_id']
        center_lines = all_road_values_list[i]['lanes']['center']
        left_lane = all_road_values_list[i]['lanes']['left']
        right_lane = all_road_values_list[i]['lanes']['right']
        if left_lane == []:
            left_lane_id = None
            left_lane_restriction = None
            left_lane_info.append([road_id, left_lane_id, left_lane_restriction])
        else:
            for j in range(len(left_lane)):
                lane_info = left_lane[j][0]
                left_lane_id = lane_info[0]
                lane_rule = left_lane[j][2]
                for m in range(len(lane_rule)):
                    if lane_rule[m] == [None, None, None]:
                        left_lane_restriction = None
                    else:
                        for n in range(len(lane_rule[m])):
                            if lane_rule[m][n][1] == 'allow':
                                left_lane_restriction = lane_rule[m][n][2]
                            elif lane_rule[m][n][1] == 'deny':
                                left_lane_restriction = None
                            else:
                                left_lane_restriction = None
                    left_lane_info.append([road_id, left_lane_id, left_lane_restriction])

        if right_lane == []:
            right_lane_id = None
            right_lane_restriction = None
            right_lane_info.append([road_id, right_lane_id, right_lane_restriction])
        else:
            for j in range(len(right_lane)):
                lane_info = right_lane[j][0]
                right_lane_id = lane_info[0]
                lane_rule = right_lane[j][2]
                for m in range(len(lane_rule)):
                    if lane_rule[m] == [None, None, None]:
                        right_lane_restriction = None
                    else:
                        for n in range(len(lane_rule[m])):
                            if lane_rule[m][n][1] == 'allow':
                                right_lane_restriction = lane_rule[m][n][2]
                            elif lane_rule[m][n][1] == 'deny':
                                right_lane_restriction = None
                            else:
                                right_lane_restriction = None
                    right_lane_info.append([road_id, right_lane_id, right_lane_restriction])
        lane_access.append([left_lane_info, right_lane_info])
    return lane_access

# 获取道路上的road_mark
def parser_road_mark(root_node):
    # Only accept lxml element
    all_road_mark = []
    road_mark_dict = {}
    if not etree.iselement(root_node):
        raise TypeError("Argument root_node is not a xml element")
    for road in root_node.findall("road"):
        road_dict = {}
        lanes = road.find("lanes")
        road_id = int(road.get("id"))
        road_length = float(road.get("length"))
        road_dict['road_id'] = road_id
        road_dict['road_mark_list'] = {}
        if lanes is None:
            road_dict['road_mark_list'] = {}
            raise Exception("Road must have lanes element")
        else:
            for laneSection in road.find("lanes").findall("laneSection"):
                section_s = float(laneSection.get("s"))
                section_singleSide = laneSection.get("singleSide")
                lanes_center = laneSection.find("center")
                lanes_left = laneSection.find("left")
                lanes_right = laneSection.find("right")
                if lanes_center is None:
                    pass
                else:
                    center_list = get_road_mark(lanes_center, road_length)
                    road_dict['road_mark_list']['center'] = center_list

                if lanes_left is None:
                    pass
                else:
                    left_list = get_road_mark(lanes_left, road_length)
                    road_dict['road_mark_list']['left'] = left_list
                if lanes_right is None:
                    pass
                else:
                    right_list = get_road_mark(lanes_right, road_length)
                    road_dict['road_mark_list']['right'] = right_list
        all_road_mark.append(road_dict)
    road_mark_dict['all_road_mark'] = all_road_mark
    return all_road_mark

def get_road_mark(lane_info, road_length):
    # 获取所有的实线solid和虚线broken
    lane_list = []
    for lane in lane_info.findall("lane"):
        lane_id = int(lane.get("id"))
        lane_type = lane.get("type")
        all_road_mark = lane.findall("roadMark")
        all_lane_access = lane.findall("access")
        road_mark_list = []
        if len(all_road_mark) > 0:
            for road_mark in all_road_mark:
                road_mark_sOffset = float(road_mark.get("sOffset"))
                road_mark_type = road_mark.get("type")
                road_mark_param = [lane_id, road_mark_sOffset, road_mark_type]
                road_mark_list.append(road_mark_param)
        else:
            road_mark_param = [lane_id, None, None]
            road_mark_list.append(road_mark_param)
        lane_list.append(road_mark_list)

    all_soffset_list = []
    for j in range(len(lane_list)):
        soffset = []
        for m in range(len(lane_list[j])):
            if lane_list[j][m][2] == 'solid' or lane_list[j][m][2] == 'solid solid' or lane_list[j][m][2] == 'broken' or lane_list[j][m][2] == 'broken broken':
                soffset.append([j, m, lane_list[j][0][0], lane_list[j][m][2]])
        if len(soffset) == 0:
            pass
        elif len(soffset) == 1:
            if soffset[0][1] == 0:
                soffset_list = [0, road_length, soffset[0][2], soffset[0][3]]
            else:
                if len(lane_list[j]) == (soffset[0][1] + 1):
                    soffset_list = [lane_list[soffset[0][0]][soffset[0][1]][1], road_length,
                                    soffset[0][2], soffset[0][3]]
                else:
                    soffset_list = [lane_list[soffset[0][0]][soffset[0][1]][1],
                                    lane_list[soffset[0][0]][soffset[0][1] + 1][1],
                                    soffset[0][2], soffset[0][3]]
            all_soffset_list.append(soffset_list)
        else:
            for num in range(len(soffset)):
                if num == 0:
                    soffset_list = [0, lane_list[soffset[0][0]][soffset[0][1] + 1][1],
                                    soffset[num][2], soffset[num][3]]
                elif num == (len(soffset) - 1):
                    if len(lane_list[j]) == (soffset[-1][1] + 1):
                        soffset_list = [lane_list[soffset[num][0]][soffset[num][1]][1], road_length,
                                        soffset[num][2], soffset[num][3]]
                    else:
                        soffset_list = [lane_list[soffset[num][0]][soffset[num][1]][1],
                                        lane_list[soffset[num][0]][soffset[num][1] + 1][1],
                                        soffset[num][2], soffset[num][3]]
                else:
                    soffset_list = [lane_list[soffset[num][0]][soffset[num][1]][1],
                                    lane_list[soffset[num][0]][soffset[num][1] + 1][1],
                                    soffset[num][2], soffset[num][3]]
                all_soffset_list.append(soffset_list)
    return all_soffset_list

