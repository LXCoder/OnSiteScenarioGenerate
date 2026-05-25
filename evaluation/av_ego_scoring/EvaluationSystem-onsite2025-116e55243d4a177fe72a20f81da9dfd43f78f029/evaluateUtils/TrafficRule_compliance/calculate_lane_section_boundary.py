import xml.etree.ElementTree as ET
import math
from shapely.geometry import Polygon

class Geometry:
    def __init__(self, s, x, y, hdg, length, geom_type, curvature=None):
        self.s = s
        self.x = x
        self.y = y
        self.hdg = hdg
        self.length = length
        self.type = geom_type
        self.curvature = curvature

class LaneWidth:
    def __init__(self, s_offset, a, b, c, d):
        self.s_offset = s_offset
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def width_at(self, s):
        ds = s - self.s_offset
        return self.a + self.b * ds + self.c * ds**2 + self.d * ds**3

class Lane:
    def __init__(self, lane_id, lane_type):
        self.id = lane_id
        self.type = lane_type
        self.widths = []

    def add_width(self, width):
        self.widths.append(width)

    def get_width(self, s):
        applicable_widths = [w for w in self.widths if w.s_offset <= s]
        if not applicable_widths:
            return 0
        current_width = applicable_widths[-1]
        return current_width.width_at(s)

class LaneSection:
    def __init__(self, s, single_side):
        self.s = s
        self.single_side = single_side
        self.lanes = {}

    def add_lane(self, lane):
        self.lanes[lane.id] = lane

class Road:
    def __init__(self, road_id):
        self.id = road_id
        self.length = 0
        self.geometries = []
        self.lane_sections = []

    def add_geometry(self, geometry):
        self.geometries.append(geometry)

    def add_lane_section(self, lane_section):
        self.lane_sections.append(lane_section)

def parse_lane(lane_element):
    lane_id = int(lane_element.attrib['id'])
    lane_type = lane_element.attrib['type']
    lane = Lane(lane_id, lane_type)

    for width_element in lane_element.findall('width'):
        s_offset = float(width_element.attrib['sOffset'])
        a = float(width_element.attrib['a'])
        b = float(width_element.attrib.get('b', 0))
        c = float(width_element.attrib.get('c', 0))
        d = float(width_element.attrib.get('d', 0))
        width = LaneWidth(s_offset, a, b, c, d)
        lane.add_width(width)

    return lane

def parse_opendrive(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    roads = []

    for road_element in root.findall('road'):
        road = Road(road_element.attrib['id'])
        road.length = float(road_element.attrib['length'])

        # 解析几何段
        planView = road_element.find('planView')
        for geometry in planView.findall('geometry'):
            s = float(geometry.attrib['s'])
            x = float(geometry.attrib['x'])
            y = float(geometry.attrib['y'])
            hdg = float(geometry.attrib['hdg'])
            length = float(geometry.attrib['length'])

            element = list(geometry)[0]
            if element.tag == 'line':
                geom_type = 'line'
                curvature = None
            elif element.tag == 'arc':
                geom_type = 'arc'
                curvature = float(element.attrib['curvature'])
            else:
                continue

            geom = Geometry(s, x, y, hdg, length, geom_type, curvature)
            road.add_geometry(geom)

        # 解析车道信息
        lanes = road_element.find('lanes')
        for lane_section_element in lanes.findall('laneSection'):
            s = float(lane_section_element.attrib['s'])
            single_side = lane_section_element.attrib.get('singleSide', 'false') == 'true'
            lane_section = LaneSection(s, single_side)

            # 中心车道
            center = lane_section_element.find('center')
            if center is not None:
                for lane_element in center.findall('lane'):
                    lane = parse_lane(lane_element)
                    lane_section.add_lane(lane)

            # 左侧车道
            left = lane_section_element.find('left')
            if left is not None:
                for lane_element in left.findall('lane'):
                    lane = parse_lane(lane_element)
                    lane_section.add_lane(lane)

            # 右侧车道
            right = lane_section_element.find('right')
            if right is not None:
                for lane_element in right.findall('lane'):
                    lane = parse_lane(lane_element)
                    lane_section.add_lane(lane)

            road.add_lane_section(lane_section)

        roads.append(road)
    return roads

def get_position_along_geometry(geometry, s_offset):
    x0 = geometry.x
    y0 = geometry.y
    hdg = geometry.hdg

    if geometry.type == 'line':
        x = x0 + s_offset * math.cos(hdg)
        y = y0 + s_offset * math.sin(hdg)
        hdg_new = hdg
    elif geometry.type == 'arc':
        curvature = geometry.curvature
        radius = 1 / curvature
        angle = curvature * s_offset
        cx = x0 - radius * math.sin(hdg)
        cy = y0 + radius * math.cos(hdg)
        hdg_new = hdg + angle

        x = cx + radius * math.sin(hdg_new)
        y = cy - radius * math.cos(hdg_new)
    else:
        x = x0
        y = y0
        hdg_new = hdg

    return x, y, hdg_new

def compute_lane_section_polygon(road, lane_section):
    s_start = lane_section.s
    idx = road.lane_sections.index(lane_section)
    if idx + 1 < len(road.lane_sections):
        s_end = road.lane_sections[idx + 1].s
    else:
        s_end = road.length

    # 找到与 lane_section 范围重叠的几何段
    geometries = []
    for geom in road.geometries:
        geom_s_end = geom.s + geom.length
        if geom_s_end <= s_start or geom.s >= s_end:
            continue
        geometries.append(geom)

    left_boundary = []
    right_boundary = []

    lane_ids = lane_section.lanes.keys()
    if not lane_ids:
        return None

    max_left_lane_id = max([lid for lid in lane_ids if lid > 0], default=None)
    max_right_lane_id = min([lid for lid in lane_ids if lid < 0], default=None)

    # 若无左侧车道，则参考线本身为最左
    if max_left_lane_id is None:
        max_left_lane_id = 0

    # 若无右侧车道，则参考线本身为最右
    if max_right_lane_id is None:
        max_right_lane_id = 0

    for geom in geometries:
        s_geom_start = max(geom.s, s_start)
        s_geom_end = min(geom.s + geom.length, s_end)
        length = s_geom_end - s_geom_start

        num_samples = max(int(length / 0.1) + 1, 2)
        s_samples = [s_geom_start + i * (length / (num_samples - 1)) for i in range(num_samples)]

        for s in s_samples:
            ds = s - geom.s
            x_ref, y_ref, hdg_ref = get_position_along_geometry(geom, ds)

            # 累积左侧车道宽度
            total_width_left = 0
            for lane_id in range(1, max_left_lane_id + 1):
                lane = lane_section.lanes.get(lane_id)
                if lane:
                    lane_width = lane.get_width(s - lane_section.s)
                    total_width_left += lane_width

            # 累积右侧车道宽度
            total_width_right = 0
            for lane_id in range(-1, max_right_lane_id - 1, -1):
                lane = lane_section.lanes.get(lane_id)
                if lane:
                    lane_width = lane.get_width(s - lane_section.s)
                    total_width_right += lane_width

            x_left = x_ref - total_width_left * math.sin(hdg_ref)
            y_left = y_ref + total_width_left * math.cos(hdg_ref)
            left_boundary.append((x_left, y_left))

            x_right = x_ref + total_width_right * math.sin(hdg_ref)
            y_right = y_ref - total_width_right * math.cos(hdg_ref)
            right_boundary.append((x_right, y_right))

    if left_boundary and right_boundary:
        boundary = left_boundary + right_boundary[::-1]
        polygon = Polygon(boundary)
        return polygon
    else:
        return None

def get_lane_section_polygons(xodr_file):
    """
    根据输入的 OpenDRIVE 文件，返回所有 Road 的所有 LaneSection 对应的多边形。
    返回结果为字典:
    {
        "Road-{road_id}_LaneSection-{idx}": Polygon对象,
        ...
    }
    """
    roads = parse_opendrive(xodr_file)
    polygons = {}

    for road in roads:
        for idx, lane_section in enumerate(road.lane_sections):
            polygon = compute_lane_section_polygon(road, lane_section)
            section_id = f"Road-{road.id}_LaneSection-{idx}"
            if polygon is not None:
                polygons[section_id] = polygon

    return polygons
