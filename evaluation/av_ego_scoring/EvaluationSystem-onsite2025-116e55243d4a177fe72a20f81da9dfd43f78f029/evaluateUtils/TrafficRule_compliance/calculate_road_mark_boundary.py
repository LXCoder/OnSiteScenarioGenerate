import xml.etree.ElementTree as ET
import math
from shapely.geometry import LineString


def parse_road_marks(xodr_file, road_mark_width=1.66):
    """
    输入:
        xodr_file: xodr文件路径
        road_mark_width: roadmark（标线）的整体宽度(米)，默认1m

    输出:
        一个字典 { "Road-<road_id>_LaneSection-<sec_j>_Lane-<lane_id>_roadmark-<mark_i>": 多边形 }
        其中多边形是 Shapely Polygon，用于描述指定 roadmark 的空间范围
    """

    def parse_xodr(path_to_xodr_file):
        tree = ET.parse(path_to_xodr_file)
        root = tree.getroot()
        return root

    def extract_road_geometries(road_elem):
        geoms = []
        planView = road_elem.find('planView')
        for geom in planView.findall('geometry'):
            s = float(geom.get('s', 0.0))
            x = float(geom.get('x', 0.0))
            y = float(geom.get('y', 0.0))
            hdg = float(geom.get('hdg', 0.0))
            length = float(geom.get('length', 0.0))

            g_line = geom.find('line')
            g_arc = geom.find('arc')

            if g_line is not None:
                geoms.append({
                    's': s, 'x': x, 'y': y, 'hdg': hdg,
                    'length': length, 'type': 'line'
                })
            elif g_arc is not None:
                curvature = float(g_arc.get('curvature', '0.0'))
                geoms.append({
                    's': s, 'x': x, 'y': y, 'hdg': hdg,
                    'length': length, 'type': 'arc',
                    'curvature': curvature
                })
            else:
                # 未实现 spiral 等类型，默认当 line 处理
                geoms.append({
                    's': s, 'x': x, 'y': y, 'hdg': hdg,
                    'length': length, 'type': 'line'
                })
        return geoms

    def compute_refline_xyz(road_geoms, s):
        """
        在给定 s 的位置，计算参考线 X,Y,hdg
        支持 line / arc.
        """
        for gm in road_geoms:
            s0 = gm['s']
            s_len = gm['length']
            if s0 <= s <= s0 + s_len:
                ds = s - s0
                x0, y0, hdg0 = gm['x'], gm['y'], gm['hdg']

                if gm['type'] == 'line':
                    X = x0 + ds * math.cos(hdg0)
                    Y = y0 + ds * math.sin(hdg0)
                    hdg = hdg0
                    return X, Y, hdg

                elif gm['type'] == 'arc':
                    R = 1.0 / gm['curvature']  # gm['curvature'] != 0
                    hdg = hdg0 + ds / R
                    X = x0 + R * (math.sin(hdg) - math.sin(hdg0))
                    Y = y0 - R * (math.cos(hdg) - math.cos(hdg0))
                    return X, Y, hdg

        return None, None, None

    def find_lane_section_end_s(road_elem, laneSec_index):
        """
        找 laneSection 的结束 s 值：
        如果有下一个 laneSection，则用它的 s。
        否则用 road 的总长度。
        """
        all_sections = road_elem.find('lanes').findall('laneSection')
        if laneSec_index < len(all_sections) - 1:
            s_next = float(all_sections[laneSec_index + 1].get('s', '0.0'))
            return s_next
        else:
            return float(road_elem.get('length', '1000.0'))

    def get_lane_width_segments(lane_elem):
        w_elems = lane_elem.findall('width')
        segs = []
        for w in w_elems:
            sOff = float(w.get('sOffset', '0.0'))
            a = float(w.get('a', '0.0'))
            b = float(w.get('b', '0.0'))
            c = float(w.get('c', '0.0'))
            d = float(w.get('d', '0.0'))
            segs.append({
                'sOffset': sOff, 'a': a, 'b': b, 'c': c, 'd': d
            })
        segs.sort(key=lambda x: x['sOffset'])
        return segs

    def lane_width_at_s(width_segments, s_rel):
        """
        在 laneSection 的坐标系(相对于 laneSection起点)下, s_rel 处的 lane 宽度
        通过找到对应 width 段并用 a+b*ds+c*ds²+d*ds³ 计算
        """
        if not width_segments:
            return 0.0

        candidate = width_segments[-1]  # 缺省用最后一段
        for i in range(len(width_segments)):
            seg = width_segments[i]
            next_sOff = (width_segments[i + 1]['sOffset']
                         if i < len(width_segments) - 1
                         else float('inf'))
            if seg['sOffset'] <= s_rel < next_sOff:
                candidate = seg
                break

        ds = s_rel - candidate['sOffset']
        return (candidate['a']
                + candidate['b'] * ds
                + candidate['c'] * (ds ** 2)
                + candidate['d'] * (ds ** 3))

    def get_all_lane_width_maps(lane_section):
        """
        返回: { lane_id: [ {sOffset, a,b,c,d}, ...], ... }
        """
        mapping = {}
        # 不再区分 left/center/right, 直接 .//lane
        for lane_elem in lane_section.findall('.//lane'):
            lid = int(lane_elem.get('id', '0'))
            segs = get_lane_width_segments(lane_elem)
            mapping[lid] = segs
        return mapping

    def cumulative_lane_offset(lane_id, lanes_width_map, s_rel):
        """
        计算 lane_id 车道的「内侧」相对于参考线的 t 偏移。
        假设:
          - lane_id>0(左侧车道)向正t累加
          - lane_id<0(右侧车道)向负t累加
          - lane_id=0: t=0
        """
        if lane_id == 0:
            return 0.0
        offset = 0.0
        if lane_id > 0:
            for lid in range(1, lane_id):
                if lid in lanes_width_map:
                    offset += lane_width_at_s(lanes_width_map[lid], s_rel)
            return offset
        else:
            for lid in range(-1, lane_id, -1):
                if lid in lanes_width_map:
                    offset += lane_width_at_s(lanes_width_map[lid], s_rel)
            return -offset

    # -------------- 解析核心开始 --------------

    root = parse_xodr(xodr_file)
    solid_types = {"solid", "solid solid", "solid broken", "broken solid"}
    result = {}

    # 1) 遍历所有 road
    for road_i, road_elem in enumerate(root.findall('road')):
        road_geoms = extract_road_geometries(road_elem)

        # 2) 遍历该 Road 下所有 LaneSection
        lane_sections = (road_elem.find('lanes').findall('laneSection')
                         if road_elem.find('lanes') else [])
        for sec_j, lane_section in enumerate(lane_sections):
            s_section = float(lane_section.get('s', '0.0'))
            s_section_end = find_lane_section_end_s(road_elem, sec_j)

            # 构建 lane_id -> [widthSegments] 的 map
            lanes_width_map = get_all_lane_width_maps(lane_section)

            # 3) 找到所有 lane
            for lane_elem in lane_section.findall('.//lane'):
                lane_id = int(lane_elem.get('id', '0'))

                # 4) 查看 roadMark
                marks = lane_elem.findall('roadMark')
                # 按 sOffset 排序
                marks.sort(key=lambda mm: float(mm.get('sOffset', '0.0')))

                for mark_i, mark_elem in enumerate(marks):
                    rm_type = mark_elem.get('type', '')
                    if rm_type not in solid_types:
                        continue

                    sOffset = float(mark_elem.get('sOffset', '0.0'))
                    # 计算 end_s
                    if mark_i < len(marks) - 1:
                        next_sOffset = float(marks[mark_i + 1]
                                             .get('sOffset', '0.0'))
                    else:
                        next_sOffset = s_section_end - s_section

                    start_s = s_section + sOffset
                    end_s = s_section + next_sOffset
                    if end_s <= start_s:
                        continue

                    # 5) 根据 roadMark 的 side 属性决定内侧/外侧
                    mark_side = mark_elem.get('side')  # 若无则 None

                    # 离散采样
                    num_samples = 50
                    s_samples = [
                        start_s + i * (end_s - start_s) / float(num_samples)
                        for i in range(num_samples + 1)
                    ]

                    line_coords = []
                    for s_val in s_samples:
                        x_ref, y_ref, hdg = compute_refline_xyz(road_geoms, s_val)
                        if x_ref is None:
                            continue

                        s_rel = s_val - s_section
                        # 先算内侧 t
                        t_inner = cumulative_lane_offset(lane_id,
                                                         lanes_width_map,
                                                         s_rel)

                        # lane 自身宽度
                        w_lane = lane_width_at_s(lanes_width_map.get(lane_id, []),
                                                 s_rel)

                        # 若 roadMark 未指定 side => 默认 outer
                        if mark_side is None:
                            if lane_id > 0:
                                t_mark = t_inner + w_lane
                            elif lane_id < 0:
                                t_mark = t_inner - w_lane
                            else:
                                t_mark = 0.0
                        else:
                            # 如果有 side='left'/'right'
                            if mark_side.lower() == 'left':
                                t_mark = t_inner
                            elif mark_side.lower() == 'right':
                                if lane_id > 0:
                                    t_mark = t_inner + w_lane
                                elif lane_id < 0:
                                    t_mark = t_inner - w_lane
                                else:
                                    t_mark = 0.0
                            else:
                                # 未知值时默认外侧
                                if lane_id > 0:
                                    t_mark = t_inner + w_lane
                                elif lane_id < 0:
                                    t_mark = t_inner - w_lane
                                else:
                                    t_mark = 0.0

                        # 计算全局坐标
                        nx = -math.sin(hdg)
                        ny = math.cos(hdg)
                        x_line = x_ref + t_mark * nx
                        y_line = y_ref + t_mark * ny
                        line_coords.append((x_line, y_line))

                    # 生成 Shapely 多边形
                    if len(line_coords) >= 2:
                        line_str = LineString(line_coords)
                        # 这里使用可变参数 road_mark_width (半径 = road_mark_width/2)
                        poly = line_str.buffer(road_mark_width / 2.0,
                                               resolution=8,
                                               cap_style=1,
                                               join_style=1)

                        # 在 key 中含 lane_id，防止覆盖
                        key = (f"Road-{road_i}_LaneSection-{sec_j}_"
                               f"Lane-{lane_id}_roadmark-{mark_i}")
                        result[key] = poly

    return result




