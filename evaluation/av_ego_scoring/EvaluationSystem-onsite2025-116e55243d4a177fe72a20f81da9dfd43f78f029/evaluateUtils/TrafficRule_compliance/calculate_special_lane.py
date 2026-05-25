import xml.etree.ElementTree as ET
import math
from shapely.geometry import Polygon


'''
# 调用示例：
from this_module import parse_special_lanes
result = parse_special_lanes("your_file.xodr", step=5.0)
print(result)
'''

def parse_xodr(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    return root


def get_geometry_points(geometries, step=1.0):
    """
    改进：返回包含(s, x, y, hdg)的列表。
    s为参考线起点处累计距离，用于后续宽度插值。
    """
    points = []
    total_s = 0.0
    last_geom_end_s = 0.0
    for geom in geometries:
        s0 = float(geom.get('s'))
        x0 = float(geom.get('x'))
        y0 = float(geom.get('y'))
        hdg0 = float(geom.get('hdg'))
        length = float(geom.get('length'))

        line = geom.find('line')
        arc = geom.find('arc')

        num_steps = max(1, int(length / step))
        for i in range(num_steps + 1):
            ds = i * step
            if ds > length:
                ds = length
            s_global = s0 + ds
            if line is not None:
                # line段
                px = x0 + ds * math.cos(hdg0)
                py = y0 + ds * math.sin(hdg0)
                phdg = hdg0
            elif arc is not None:
                # arc段
                curvature = float(arc.get('curvature'))
                px = x0 + (math.sin(hdg0 + ds * curvature) - math.sin(hdg0)) / curvature
                py = y0 - (math.cos(hdg0 + ds * curvature) - math.cos(hdg0)) / curvature
                phdg = hdg0 + ds * curvature
            else:
                # 未实现其他类型，这里简单处理
                px = x0 + ds * math.cos(hdg0)
                py = y0 + ds * math.sin(hdg0)
                phdg = hdg0

            points.append((s_global, px, py, phdg))
    return points


def get_lane_sections(road):
    lanes = road.find('lanes')
    if lanes is not None:
        return lanes.findall('laneSection')
    return []


def parse_lane_widths(lane_element, default_width=3.0):
    """
    返回该lane的宽度定义列表，每个元素为(dict):
    {
      'sOffset': float,
      'a': float,
      'b': float,
      'c': float,
      'd': float
    }
    如果没有width定义，使用默认宽度作为常量值处理。
    """
    width_elems = lane_element.findall('width')
    width_segments = []
    if len(width_elems) > 0:
        for we in width_elems:
            sOffset = float(we.get('sOffset', 0.0))
            a = float(we.get('a', default_width))
            b = float(we.get('b', 0.0))
            c = float(we.get('c', 0.0))
            d = float(we.get('d', 0.0))
            width_segments.append({
                'sOffset': sOffset,
                'a': a,
                'b': b,
                'c': c,
                'd': d
            })
    else:
        # 无width定义，使用默认宽度的常量段
        width_segments.append({
            'sOffset': 0.0,
            'a': default_width,
            'b': 0.0,
            'c': 0.0,
            'd': 0.0
        })
    # 按sOffset排序
    width_segments.sort(key=lambda w: w['sOffset'])
    return width_segments


def get_width_at_s(width_segments, s_rel):
    """
    给定lane的width_segments列表和车道内相对起点的s位置s_rel，
    查找对应的width段并计算宽度。
    假设laneSection的s起点为0，对应lane width定义的sOffset基于本section起点。
    """
    # 在实际OpenDRIVE中，laneSection有起点s。
    # width segments定义相对于laneSection起点的偏移sOffset。
    # 需要根据s_rel找到合适的区间。
    # s_rel是相对于laneSection起点的距离。
    chosen = None
    for i, seg in enumerate(width_segments):
        # 如果下一个segment的sOffset比s_rel大，就用当前这个segment
        # 若没有下一个，最后一个segment继续使用
        if i + 1 < len(width_segments):
            if width_segments[i + 1]['sOffset'] > s_rel:
                chosen = seg
                break
        else:
            chosen = seg
            break
    if chosen is None:
        chosen = width_segments[-1]

    ds = s_rel - chosen['sOffset']
    w = chosen['a'] + chosen['b'] * ds + chosen['c'] * (ds ** 2) + chosen['d'] * (ds ** 3)
    return w


def lane_is_special(lane_type):
    if lane_type.startswith('special') or lane_type == 'restricted':
        return True
    return False


def compute_lane_polygon(ref_points, lane_id, lane_section, default_width=3.0):
    """
    基于动态宽度与偏移：
    1. 获取laneSection起点s
    2. 对laneSection内所有lane解析width段
    3. 对ref_points中每个点(包含s), 计算当前lane的宽度和偏移

    注：ref_points是整条road的，需要根据laneSection的起点和长度只取laneSection范围内的点。
    """
    s_section = float(lane_section.get('s', 0.0))  # laneSection起点
    # laneSection的长度未明确给出，可通过下一个laneSection的s或road总长计算。
    # 简化方法：在该section后一个laneSection的起点s - s_section = length_section
    # 若无下一个，则到road末尾。
    # 这里简单通过parent road下的laneSections获取该section在road中的终点
    parent_lanes = lane_section.getparent()
    # 在ElementTree中向上访问需要用其他方式，如果这里版本不支持，可以在外面调用时传入laneSectionIndex和下一个laneSection s
    # 为了演示，我们在调用此函数前就应已知laneSection的结束s,此处假设lane_section_end已传入或可找到。
    # 简化：假设此函数只处理当前section，需要调用方提供下个section起点或road长度以限定范围。

    # 这里先尝试找到下一个laneSection以确定结束s
    # 注：ElementTree的上级访问需要更复杂操作，这里简单传入参数解决:
    # 为演示，我们在参数列表中增加一个end_s参数来界定laneSection终点。
    # 用户请在调用此函数时传入, 或者在parse_special_lanes中计算好。
    # 这里我们先改函数签名：
    pass


def compute_lane_polygon_with_dynamic_width(ref_points, s_section_start, s_section_end, lane_id, lane_section,
                                            default_width=3.0):
    """
    动态宽度版本的计算函数。
    需要:
    - ref_points: 整条road的点列表 (s,x,y,hdg)
    - s_section_start: laneSection的起点s值
    - s_section_end: laneSection的终点s值
    - lane_id, lane_section

    输出：该laneSection中指定车道的polygon (shapely Polygon)
    """
    # 解析所有lane的width段
    lane_elems = lane_section.findall('.//lane')
    lane_width_map = {}
    for le in lane_elems:
        lid = int(le.get('id'))
        lane_width_map[lid] = parse_lane_widths(le, default_width)

    # 筛选出该laneSection范围内的参考点
    section_points = [(s, x, y, hdg) for (s, x, y, hdg) in ref_points if s_section_start <= s <= s_section_end]
    if len(section_points) < 2:
        # 若没有足够的点，则无法形成多边形
        return None

    # 对每个section_points点计算当前lane的中心offset和宽度
    # offset计算方法：
    # 正id lane向左: 假设lane_id=1在参考线左侧，其中心线 = sum(width(1..0))?
    # 更严格方式：
    # 对于s位置，对于lane_id>0: offset = sum_{i=1}^{i<lane_id} width(i,s) + width(lane_id,s)/2
    # 对于lane_id<0: offset = - ( sum_{i=-1}^{i>lane_id} width(i,s) + width(lane_id,s)/2 )
    # 在进行sum时，需要动态计算每条lane的width(s)。

    def get_lane_center_offset_at_s(lid, s_global):
        # s_rel是该点相对于laneSection起点的距离
        s_rel = s_global - s_section_start
        if lid > 0:
            # 从1到lid-1累积
            offset = 0.0
            for i in range(1, lid):
                w_i = get_width_at_s(lane_width_map[i], s_rel)
                offset += w_i
            w_lid = get_width_at_s(lane_width_map[lid], s_rel)
            offset += w_lid / 2.0
            return offset
        elif lid < 0:
            offset = 0.0
            for i in range(-1, lid, -1):
                w_i = get_width_at_s(lane_width_map[i], s_rel)
                offset -= w_i
            w_lid = get_width_at_s(lane_width_map[lid], s_rel)
            offset -= w_lid / 2.0
            return offset
        else:
            # lid=0不是真实车道
            return 0.0

    left_boundary = []
    right_boundary = []
    for (s_global, x, y, hdg) in section_points:
        s_rel = s_global - s_section_start
        lane_width = get_width_at_s(lane_width_map[lane_id], s_rel)
        t_offset = get_lane_center_offset_at_s(lane_id, s_global)

        half_w = lane_width / 2.0
        x_center = x - t_offset * math.sin(hdg)
        y_center = y + t_offset * math.cos(hdg)

        x_left = x_center - half_w * math.sin(hdg)
        y_left = y_center + half_w * math.cos(hdg)

        x_right = x_center + half_w * math.sin(hdg)
        y_right = y_center - half_w * math.cos(hdg)

        left_boundary.append((x_left, y_left))
        right_boundary.append((x_right, y_right))

    boundary_coords = left_boundary + right_boundary[::-1]
    poly = Polygon(boundary_coords)
    return poly


def parse_special_lanes(xodr_file, step=1.0):
    root = parse_xodr(xodr_file)
    lane_polygons_dict = {}

    # 首先获取road的长度和laneSection信息，方便确定每个section的结束s
    # Road长度从road标签获取: <road length="...">
    # laneSection按s排序，最后一个到road末尾
    for road in root.findall('road'):
        road_id = road.get('id', 'unknown')
        road_length = float(road.get('length', 0.0))

        planView = road.find('planView')
        if planView is None:
            continue
        geometries = planView.findall('geometry')
        ref_points = get_geometry_points(geometries, step=step)

        lane_sections = get_lane_sections(road)

        # 先将laneSections按s排序
        lane_sections_sorted = sorted(lane_sections, key=lambda ls: float(ls.get('s', 0.0)))

        for ls_index, lane_section in enumerate(lane_sections_sorted):
            s_section_start = float(lane_section.get('s', 0.0))
            # 找到section结束s
            if ls_index < len(lane_sections_sorted) - 1:
                s_section_end = float(lane_sections_sorted[ls_index + 1].get('s', 0.0))
            else:
                s_section_end = road_length

            # 检查lane
            lanes = lane_section.findall('.//lane')
            section_has_special = False
            for lane in lanes:
                lane_id = int(lane.get('id'))
                lane_type = lane.get('type', 'driving')
                if lane_is_special(lane_type):
                    poly = compute_lane_polygon_with_dynamic_width(ref_points, s_section_start, s_section_end, lane_id,
                                                                   lane_section)
                    if poly is not None:
                        dict_key = f"Road-{road_id}_LaneSection-{ls_index}"
                        lane_polygons_dict[dict_key] = poly
                        section_has_special = True
                        break

    return lane_polygons_dict

