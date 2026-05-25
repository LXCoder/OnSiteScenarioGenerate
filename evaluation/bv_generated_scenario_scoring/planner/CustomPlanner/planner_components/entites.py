import copy
import math
import numpy as np
from scipy.interpolate import CubicSpline
from planner.CustomPlanner.planner_components.cubic_spline import CubicSpline2D
from planner.CustomPlanner.planner_components.essentials import RefPoint, TrajPoint
from utils.opendrive2discretenet.utils import decode_road_section_lane_width_type_id, encode_road_section_lane_width_type_id
from planner.CustomPlanner.local_utils import calculate_distances, cartesian_to_frenet


class _LinearBoundary:
    def __init__(self, xs, ys):
        self.xs = np.asarray(xs, dtype=np.float64)
        self.ys = np.asarray(ys, dtype=np.float64)

    def __call__(self, s):
        arr = np.asarray(s, dtype=np.float64)
        val = np.interp(arr, self.xs, self.ys, left=self.ys[0], right=self.ys[-1])
        if np.ndim(s) == 0:
            return float(val)
        return val


def _build_safe_boundary_spline(rs_list, val_list):
    xs = np.asarray(rs_list, dtype=np.float64).reshape(-1)
    ys = np.asarray(val_list, dtype=np.float64).reshape(-1)

    if xs.size == 0 or ys.size == 0:
        xs = np.array([0.0, 1e-3], dtype=np.float64)
        ys = np.array([0.0, 0.0], dtype=np.float64)

    order = np.argsort(xs)
    xs = xs[order]
    ys = ys[order]

    keep = [0]
    for i in range(1, xs.size):
        if xs[i] - xs[keep[-1]] > 1e-8:
            keep.append(i)
    xs = xs[keep]
    ys = ys[keep]

    if xs.size == 1:
        xs = np.array([xs[0], xs[0] + 1e-3], dtype=np.float64)
        ys = np.array([ys[0], ys[0]], dtype=np.float64)

    try:
        return CubicSpline(xs, ys)
    except Exception:
        return _LinearBoundary(xs, ys)


class Ego:
    def __init__(self, route, ref_points, topo_graph):
        self.cartesian_state = None
        self.frenet_state = None
        self.matched_point = None
        self.route = route
        self.raw_ref_points = ref_points
        self.concated_ref_points = None
        self.concated_ref_curves = None
        self.concated_drivable_boundaries = None
        self.concated_bound_curves = None
        self.cur_matched_point = None
        self.cur_matched_point_index = None
        self.cur_ref_points = None
        self.cur_ref_curve = None
        self.cur_lane_indices = None
        self.cur_drivable_boundaries = None
        self.cur_lane = None
        self.cur_lane_id = None
        self.cur_time = -1
        self.last_lane_change_time = -1

        self.width = -1
        self.length = -1
        raw_concated_ref_points, concated_lane_indices = self.concat_ref_points()
        self.concated_ref_points, self.concated_ref_curves = self.create_smoothed_frenet(raw_concated_ref_points)
        self.concated_drivable_boundaries, self.concated_lane_indices = self.generate_boundaries(concated_lane_indices, topo_graph)

    def init(self, observation):
        """
        初始化车辆状态
        :return:
        """
        self.width = observation.ego_info.width
        self.length = observation.ego_info.length
        x_ = observation.ego_info.x + observation.ego_info.v * np.cos(observation.ego_info.yaw) * observation.test_info["dt"]
        y_ = observation.ego_info.y + observation.ego_info.v * np.sin(observation.ego_info.yaw) * observation.test_info["dt"]
        self.cartesian_state = TrajPoint(x=x_,y=y_,
                                         v=observation.ego_info.v, a=observation.ego_info.a,
                                         theta=observation.ego_info.yaw, kappa=0) # x, y, v, a, yaw, kappa
        self.cur_time = 0
        self.cur_lane_id = 0
        self.cur_segment_id = 0
        self.cur_lane = self.route[self.cur_lane_id]
        self.cur_ref_points = self.concated_ref_points[self.cur_segment_id]
        self.cur_ref_curve = self.concated_ref_curves[self.cur_segment_id]
        self.cur_lane_indices = self.concated_lane_indices[self.cur_segment_id]
        self.cur_drivable_boundaries = self.concated_drivable_boundaries[self.cur_segment_id]
        self.cur_matched_point, self.cur_matched_point_index, self.cur_rs = self.match_points(self.cur_ref_points, self.cur_ref_curve)
        self.frenet_state = self.to_frenet()

    def update(self, observation):
        x_ = observation.ego_info.x + observation.ego_info.v * np.cos(observation.ego_info.yaw) * observation.test_info["dt"]
        y_ = observation.ego_info.y + observation.ego_info.v * np.sin(observation.ego_info.yaw) * observation.test_info["dt"]
        self.cartesian_state = TrajPoint(x=x_, y=y_, v=observation.ego_info.v, a=observation.ego_info.a,
                                         theta=observation.ego_info.yaw, kappa=0)  # x, y, v, a, yaw, kappa
        self.cur_time = observation.test_info["t"]
        self.update_cur_lane()
        # 换道策略
        if self.check_lane_changing():
            self.cur_lane_id += 1
            self.cur_segment_id += 1
            self.cur_lane = self.route[self.cur_lane_id]
            self.cur_ref_points = self.concated_ref_points[self.cur_segment_id]
            self.cur_ref_curve = self.concated_ref_curves[self.cur_segment_id]
            self.cur_lane_indices = self.concated_lane_indices[self.cur_segment_id]
            self.cur_drivable_boundaries = self.concated_drivable_boundaries[self.cur_segment_id]
            self.last_lane_change_time = observation.test_info["t"]

        self.cur_matched_point, self.cur_matched_point_index, self.cur_rs = self.match_points(self.cur_ref_points, self.cur_ref_curve)
        self.frenet_state = self.to_frenet()

    def update_cur_lane(self):
        if self.cur_lane_id == len(self.route) - 1:
            return

        self.cur_lane = self.cur_lane_indices[self.cur_matched_point_index]
        self.cur_lane_id = self.route.index(self.cur_lane)

    def check_lane_changing(self):
        """
        判断是否要进行换道
        :return:
        """
        # 如果是route的最后一条lane，则不用判断需不需要换lane
        if self.cur_lane_id == len(self.route) - 1:
            return False

        decode_cur_lane_id = decode_road_section_lane_width_type_id(self.cur_lane)
        decode_next_lane_id = decode_road_section_lane_width_type_id(self.route[self.cur_lane_id + 1])

        # 这个条件排除连续换道的情况，当出现连续换道时，当ego靠近中心线时才可以批准下一次换道
        if self.last_lane_change_time + 0.9 >= self.cur_time:
            # TODO: 这个换道逻辑目前还是有点问题，如果还有而外的问题再继续修改逻辑
            ## 5_24_straight_straight_25 : 如果初始时刻车辆位置不在参考线上，而偏向于要换的道，就放缩条件直接换道
            if abs(self.frenet_state[1][0]) >= 0.45:
                return False

        # 如果不涉及换道问题的话，那么参考线可以直接拼接
        if decode_cur_lane_id[0] != decode_next_lane_id[0] or decode_cur_lane_id[1] != decode_next_lane_id[1]:
            return False

        else:
            return True


    def concat_ref_points(self):
        """
        将每段路的参考线拼接成一个整体。注：这个参考线仍然是分段的，在换道处会分段。所以车辆在换道处需要进行决策，换完道后需要更换参考线
        :return: 拼接后的参考线
        """
        prev_lane_id = None
        concated_ref_points = []
        concated_lane_indices = []
        temp_concated_ref_points = []
        temp_concated_lane_indices = []
        for i, lane_id in enumerate(self.route):
            if i == 0:
                prev_lane_id = lane_id
                temp_concated_lane_indices.append((0, lane_id))
                temp_concated_ref_points.extend(self.raw_ref_points[i])
                continue

            decode_prev_lane_id = decode_road_section_lane_width_type_id(prev_lane_id)
            decode_lane_id = decode_road_section_lane_width_type_id(lane_id)

            # 如果不涉及换道问题的话，那么参考线可以直接拼接
            if decode_prev_lane_id[0] != decode_lane_id[0] or decode_prev_lane_id[1] != decode_lane_id[1]:
                temp_concated_lane_indices.append((len(temp_concated_ref_points), lane_id))

                # 检查不同路段参考线的首尾是否回出现重合的问题
                if np.sqrt(np.sum(np.square(temp_concated_ref_points[-1] - self.raw_ref_points[i][0]))) > 1e-2:
                    temp_concated_ref_points.extend(self.raw_ref_points[i])
                else:
                    temp_concated_ref_points.extend(self.raw_ref_points[i][1:])
            # 如果涉及换道问题的话，就把现在已有的参考线保存，并存conccated——reflines中，并创建新的temp_concated_ref_points
            else:
                concated_ref_points.append(temp_concated_ref_points)
                concated_lane_indices.append(temp_concated_lane_indices)

                temp_concated_lane_indices = [(0, lane_id)]
                temp_concated_ref_points = []
                temp_concated_ref_points.extend(self.raw_ref_points[i])

            prev_lane_id = lane_id

        # 将最后一组refline也添加至concated_refine中
        concated_ref_points.append(temp_concated_ref_points)
        concated_lane_indices.append(temp_concated_lane_indices)
        return concated_ref_points, concated_lane_indices

    def create_boundaries_table(self, topo_graph):
        """
        创建每条lane的边界索引表
        :param topo_graph:
        :return:
        """
        res = {}
        for lane_id in self.route:
            decode_lane_id = decode_road_section_lane_width_type_id(lane_id)
            # 根据opendrive规则，上下车流向的左车道和右车道id规则不一样
            if decode_lane_id[2] > 0:
                left_decode_lane_id = decode_lane_id[2] - 1
                right_decode_lane_id = decode_lane_id[2] + 1
            else:
                left_decode_lane_id = decode_lane_id[2] + 1
                right_decode_lane_id = decode_lane_id[2] - 1

            left_lane_id = encode_road_section_lane_width_type_id(
                decode_lane_id[0], decode_lane_id[1], left_decode_lane_id, decode_lane_id[3], decode_lane_id[4])
            right_lane_id = encode_road_section_lane_width_type_id(
                decode_lane_id[0], decode_lane_id[1], right_decode_lane_id, decode_lane_id[3],
                decode_lane_id[4])

            # 如果当前lane有左车道，则把左车道的左边界当成当前行驶区域的左边界，这里我们
            # Note这里我们只向左或者右看一个车道，避免出现连续换道的情况
            if left_lane_id in topo_graph:
                left_bound = topo_graph[left_lane_id]["width"] + topo_graph[lane_id]["width"] / 2
            # 如果没有左车道，就将当前车道的左边界作为行驶区域的左边界
            else:
                left_bound = topo_graph[lane_id]["width"] / 2

            # 如果当前lane有右车道，则把右车道的左边界当成当前行驶区域的右边界
            # Note这里我们只向左或者右看一个车道，避免出现连续换道的情况
            if right_lane_id in topo_graph:
                right_bound = topo_graph[right_lane_id]["width"] + topo_graph[lane_id]["width"] / 2

            # 如果没有右车道，就将当前车道的右边界作为行驶区域的右边界
            else:
                right_bound = topo_graph[lane_id]["width"] / 2
            res[lane_id] = (left_bound, right_bound)

        return res

    def create_smoothed_frenet(self, ref_points_sets):
        """
        创建cubicspine差值拟合后的参考线轨迹
        :return: 曲线拟合后的参考线
        """
        concated_ref_points = []
        concated_ref_curves = []
        for refline in ref_points_sets:
            refline = np.array(refline)
            sp = CubicSpline2D(refline[:, 0], refline[:, 1])
            rs = np.arange(0, sp.origin_s[-1]+1, 0.2)   # [m] distance of each interpolated points

            ref_points = []
            for i_s in rs:
                ix, iy = sp.calc_position(i_s)
                ref_points.append(RefPoint(rx=ix, ry=iy, rs=i_s,
                                           rtheta=sp.calc_theta(i_s),
                                           rkappa=sp.calc_curvature(i_s),
                                           rdkappa=sp.calc_dcurvature(i_s)))

            concated_ref_points.append(ref_points)
            concated_ref_curves.append(sp)
        return concated_ref_points, concated_ref_curves


    def generate_boundaries(self, concated_lane_indices, topo_graph):
        lane_boundary_table = self.create_boundaries_table(topo_graph)
        concated_drivable_boundaries = []
        concated_ref_point_lane_indices = []

        for i in range(len(self.concated_ref_points)):
            cur = 0
            ref_point_lane_indices = []
            boundary = {"l": None, "r": None} # 分别代表左右边界的横向距离
            j = 0
            while j < len(self.concated_ref_points[i]):
                if cur+1 >= len(concated_lane_indices[i]):
                    ref_point_lane_indices.append(concated_lane_indices[i][-1][1])
                    j += 1
                    continue

                if self.concated_ref_points[i][j].rs < self.concated_ref_curves[i].origin_s[concated_lane_indices[i][cur+1][0]]:
                    j += 1
                    ref_point_lane_indices.append(concated_lane_indices[i][cur][1])
                else:
                    cur += 1


            boundary["l"] = _build_safe_boundary_spline(
                [each.rs for each in self.concated_ref_points[i]],
                [lane_boundary_table[each][0] for each in ref_point_lane_indices]
                )

            boundary["r"] = _build_safe_boundary_spline(
                [each.rs for each in self.concated_ref_points[i]],
                [lane_boundary_table[each][1] for each in ref_point_lane_indices]
                )

            concated_drivable_boundaries.append(boundary)
            concated_ref_point_lane_indices.append(ref_point_lane_indices)
        return concated_drivable_boundaries, concated_ref_point_lane_indices


    def match_points(self, ref_points, fitting_curve):
        """
        将车辆当前的位置和参考线上的最近的点做匹配，为转换成frenet坐标系做准备
        :param traj_point: 车辆当前的位置
        :param ref_points: 参考线
        :param fitting_curve: 参考线拟合出来的虚线
        :return: 匹配出来的参考线上的轨迹点
        """
        dist_min_index, dist_min = calculate_distances(
            [self.cartesian_state.x, self.cartesian_state.y],
            [[each.rx, each.ry] for each in ref_points])

        path_point_min = ref_points[dist_min_index]
        if dist_min_index == 0 or dist_min_index == len(ref_points) - 1:
            return path_point_min, dist_min_index, path_point_min.rs
        else:
            path_point_next = ref_points[dist_min_index + 1]  # 上一时刻参考点和下一时刻参考点
            path_point_last = ref_points[dist_min_index - 1]
            vec_p2t = np.array([self.cartesian_state.x - path_point_min.rx, self.cartesian_state.y - path_point_min.ry])
            vec_p2p_next = np.array(
                [path_point_next.rx - path_point_min.rx, path_point_next.ry - path_point_min.ry])
            vec_p2p_last = np.array(
                [path_point_last.rx - path_point_min.rx, path_point_last.ry - path_point_min.ry])
            if np.dot(vec_p2t, vec_p2p_next) * np.dot(vec_p2t, vec_p2p_last) >= 0:
                return path_point_min, dist_min_index, path_point_min.rs
            else:
                if np.dot(vec_p2t, vec_p2p_next) >= 0:
                    rs_inter = path_point_min.rs + np.dot(vec_p2t, vec_p2p_next / np.linalg.norm(vec_p2p_next))
                    inter_point_x, inter_point_y = fitting_curve.calc_position(rs_inter)
                    return RefPoint(
                        rx=inter_point_x, ry=inter_point_y, rs=rs_inter,
                        rtheta=fitting_curve.calc_theta(rs_inter),
                        rkappa=fitting_curve.calc_curvature(rs_inter),
                        rdkappa=fitting_curve.calc_dcurvature(rs_inter)), dist_min_index, rs_inter
                else:
                    rs_inter = path_point_min.rs - np.dot(vec_p2t, vec_p2p_last / np.linalg.norm(vec_p2p_last))
                    inter_point_x, inter_point_y = fitting_curve.calc_position(rs_inter)
                    return RefPoint(
                        rx=inter_point_x, ry=inter_point_y, rs=rs_inter,
                        rtheta=fitting_curve.calc_theta(rs_inter),
                        rkappa=fitting_curve.calc_curvature(rs_inter),
                        rdkappa=fitting_curve.calc_dcurvature(rs_inter)), dist_min_index - 1, rs_inter

    def convert_cartesian_to_frenet(self, traj_point, path_point):
        """
        将笛卡尔坐标系转换成frenet坐标系
        :param traj_point: 车辆当前位置
        :param path_point: 匹配的参考线上的最近点
        :return:
        """
        return cartesian_to_frenet(path_point, traj_point)


    def to_frenet(self):
        """
        将frenet坐标系转换成笛卡尔坐标系
        :return:
        """
        return self.convert_cartesian_to_frenet(self.cartesian_state, self.cur_matched_point)
    def get_on_lane_id(self):
        """
        获取当前车是在哪一条车道上，决定选择哪条作为参考线
        :return:
        """
        pass


class Obstacle:
    def __init__(self, name, x, y, v, a, theta, length, width, type, dt):
        self.name = name
        self.x = x
        self.y = y
        self.v = v
        self.a = a
        self.theta = theta
        self.length = length
        self.width = width
        self.type = type
        self.dt = dt
        self.mode = "static" if self.v < 0.1 else "dynamic"

        self.cartesian_state = None
        self.frenet_state = None
        self.matched_point = None
        self.corner = self.GetCorner()

    def GetCorner(self):
        cos_o = math.cos(self.theta)
        sin_o = math.sin(self.theta)
        dx3 = cos_o * self.length / 2
        dy3 = sin_o * self.length / 2
        dx4 = sin_o * self.width / 2
        dy4 = -cos_o * self.width / 2
        return [self.x - (dx3 - dx4), self.y - (dy3 - dy4)]

    def match_points(self, ref_points, fitting_curve):
        """
        将车辆当前的位置和参考线上的最近的点做匹配，为转换成frenet坐标系做准备
        :param traj_point: 车辆当前的位置
        :param ref_points: 参考线
        :param fitting_curve: 参考线拟合出来的虚线
        :return: 匹配出来的参考线上的轨迹点
        """
        dist_min_index, dist_min = calculate_distances(
            [self.x, self.y],
            [[each.rx, each.ry] for each in ref_points])

        path_point_min = ref_points[dist_min_index]
        if dist_min_index == 0 or dist_min_index == len(ref_points) - 1:
            self.matched_point = path_point_min
        else:
            path_point_next = ref_points[dist_min_index + 1]  # 上一时刻参考点和下一时刻参考点
            path_point_last = ref_points[dist_min_index - 1]
            vec_p2t = np.array([self.x - path_point_min.rx, self.y - path_point_min.ry])
            vec_p2p_next = np.array(
                [path_point_next.rx - path_point_min.rx, path_point_next.ry - path_point_min.ry])
            vec_p2p_last = np.array(
                [path_point_last.rx - path_point_min.rx, path_point_last.ry - path_point_min.ry])
            if np.dot(vec_p2t, vec_p2p_next) * np.dot(vec_p2t, vec_p2p_last) >= 0:
                self.matched_point = path_point_min
            else:
                if np.dot(vec_p2t, vec_p2p_next) >= 0:
                    rs_inter = path_point_min.rs + np.dot(vec_p2t, vec_p2p_next / np.linalg.norm(vec_p2p_next))
                    inter_point_x, inter_point_y = fitting_curve.calc_position(rs_inter)
                    self.matched_point = RefPoint(
                        rx=inter_point_x, ry=inter_point_y, rs=rs_inter,
                        rtheta=fitting_curve.calc_theta(rs_inter),
                        rkappa=fitting_curve.calc_curvature(rs_inter),
                        rdkappa=fitting_curve.calc_dcurvature(rs_inter))
                else:
                    rs_inter = path_point_min.rs - np.dot(vec_p2t, vec_p2p_last / np.linalg.norm(vec_p2p_last))
                    inter_point_x, inter_point_y = fitting_curve.calc_position(rs_inter)
                    self.matched_point = RefPoint(
                        rx=inter_point_x, ry=inter_point_y, rs=rs_inter,
                        rtheta=fitting_curve.calc_theta(rs_inter),
                        rkappa=fitting_curve.calc_curvature(rs_inter),
                        rdkappa=fitting_curve.calc_dcurvature(rs_inter))

    def make_cartesian_state(self):
        x_ = self.x + self.v * np.cos(self.theta) * self.dt
        y_ = self.y + self.v * np.sin(self.theta) * self.dt
        self.cartesian_state = TrajPoint(
                x=x_, y=y_,
                v=self.v, a=self.a,
                theta=self.theta, kappa=0)

    def convert_cartesian_to_frenet(self):
        """
        将笛卡尔坐标系转换成frenet坐标系
        :param traj_point: 车辆当前位置
        :param path_point: 匹配的参考线上的最近点
        :return:
        """
        self.frenet_state = cartesian_to_frenet(self.matched_point, self.cartesian_state)

    def get_st_boundaries(self, ego, x_range, y_range, s_range, t_range):
        """
        获取障碍物的st边界
        :param ego:  自驾车
        :param x_range: 自驾车路径x取值
        :param y_range: 自驾车路径y取值
        :param s_range: 自驾车在frenet坐标系下的s取值
        :param t_range: 时间t范围
        :return: 返回在t_range中的st边界
        """
        ego_theta = ego.cartesian_state.theta
        ego_length = ego.length
        ego_width = ego.width
        displacement = self.v * t_range + 0.5 * self.a * np.square(t_range)
        # displacement = self.v * t_range
        predict_x = self.x + displacement * np.cos(self.theta)
        predict_y = self.y + displacement * np.sin(self.theta)

        dis_matrix_x = np.subtract.outer(x_range, predict_x)
        dis_matrix_y = np.subtract.outer(y_range, predict_y)
        # dis_matrix = np.square(dis_matrix_x) + np.square(dis_matrix_y)
        #
        # obs_length = (self.length ** 2 + self.width ** 2)
        boundary = self._check_collision(ego_theta, ego_length+0.3, ego_width+0.1,
                                         self.theta, self.length+0.3, self.width+0.1,
                                         dis_matrix_x, dis_matrix_y)
        boundary_index = []

        for i in range(len(t_range)):
            for_cur = 0
            post_cur = len(x_range) - 1
            while for_cur < len(x_range):
                if boundary[for_cur, i]:
                    break
                for_cur += 1

            while post_cur >= 0:
                if boundary[post_cur, i]:
                    break
                post_cur -= 1

            if for_cur != len(x_range) or post_cur != -1:
                boundary_index.append((s_range[max(for_cur, 1)], s_range[max(post_cur, 1)]))
            # 如果找不到boundary，说明该背景车与adc的路径没有交集
            else:
                boundary_index.append((None, None))

        return list(zip(*boundary_index))

    def get_precise_st_boundaries(self, ego, x_range, y_range, s_range, t_range, obs_dt=0.1):
        """
        获取障碍物的st边界
        :param ego:  自驾车
        :param x_range: 自驾车路径x取值
        :param y_range: 自驾车路径y取值
        :param s_range: 自驾车在frenet坐标系下的s取值
        :param t_range: 时间t范围
        :return: 返回在t_range中的st边界
        """
        precise_t_range = np.arange(0, t_range[-1] + 0.1, obs_dt)
        ego_theta = ego.cartesian_state.theta
        ego_length = ego.length
        ego_width = ego.width
        # displacement = self.v * precise_t_range + 0.5 * self.a * np.square(precise_t_range)
        displacement = max(self.v + self.a * obs_dt, 0) * precise_t_range
        predict_x = self.x + displacement * np.cos(self.theta)
        predict_y = self.y + displacement * np.sin(self.theta)

        dis_matrix_x = np.subtract.outer(x_range, predict_x)
        dis_matrix_y = np.subtract.outer(y_range, predict_y)
        # dis_matrix = np.square(dis_matrix_x) + np.square(dis_matrix_y)
        #
        # obs_length = (self.length ** 2 + self.width ** 2)
        boundary = self._check_collision(ego_theta, ego_length+0.3, ego_width+0.1,
                                         self.theta, self.length+0.3, self.width+0.1,
                                         dis_matrix_x, dis_matrix_y)
        precise_boundary_index = np.ones([len(precise_t_range), 2]) * np.array([1e6, -1e6])

        for i in range(len(precise_t_range)):
            for_cur = 0
            post_cur = len(x_range) - 1
            while for_cur < len(x_range):
                if boundary[for_cur, i]:
                    break
                for_cur += 1

            while post_cur >= 0:
                if boundary[post_cur, i]:
                    break
                post_cur -= 1

            if for_cur != len(x_range) or post_cur != -1:
                precise_boundary_index[i, 0] = s_range[for_cur]
                precise_boundary_index[i, 1] = s_range[post_cur]

        original_boundary_index = np.ones([len(t_range), 2]) * np.array([1e6, -1e6])

        for i, t in enumerate(t_range):
            if i == len(t_range) - 1:
                original_boundary_index[i] = precise_boundary_index[-1]
                break
            next_t = t_range[i+1]
            original_boundary_index[i, 0] = np.min(precise_boundary_index[int(t / obs_dt): int(next_t / obs_dt), 0])
            original_boundary_index[i, 1] = np.max(precise_boundary_index[int(t / obs_dt): int(next_t / obs_dt), 1])

        # 修正轨迹，不能出现碰撞位置倒挂的情况
        for i, t in enumerate(t_range[:-1]):

            if original_boundary_index[i, 0] != 1e6 and original_boundary_index[i + 1, 0] < original_boundary_index[i, 0]:
                original_boundary_index[i + 1, 0] = original_boundary_index[i, 0]

            if original_boundary_index[i+1, 1] != -1e6 and original_boundary_index[i + 1, 1] < original_boundary_index[i, 1]:
                original_boundary_index[i + 1, 1] = original_boundary_index[i, 1]


        return original_boundary_index.T

    @staticmethod
    def _check_collision(ego_theta, ego_length, ego_width, obs_theta, obs_length, obs_width, dis_matrix_x, dis_matrix_y):
        """
        使用分离轴定理检测背景车与自驾车碰撞，返回bool值是否发生碰撞
        :param ego_theta: 自驾车航向角
        :param ego_length: 自驾车长度
        :param ego_width: 自驾车宽度
        :param obs_theta: 背景车航向角
        :param obs_length: 背景车长度
        :param obs_width: 背景车宽度
        :param dis_matrix_x: 背景车与自驾车的x距离矩阵
        :param dis_matrix_y: 背景车与自驾车y距离矩阵
        :return: 是否发生碰撞
        """
        cos_v = math.cos(ego_theta)
        sin_v = math.sin(ego_theta)
        cos_o = math.cos(obs_theta)
        sin_o = math.sin(obs_theta)
        half_l_v = ego_length / 2
        half_w_v = ego_width / 2
        half_l_o = obs_length / 2
        half_w_o = obs_width / 2

        dx1 = cos_v * ego_length / 2
        dy1 = sin_v * ego_length / 2
        dx2 = sin_v * ego_width / 2
        dy2 = -cos_v * ego_width / 2
        dx3 = cos_o * obs_length / 2
        dy3 = sin_o * obs_length / 2
        dx4 = sin_o * obs_width / 2
        dy4 = -cos_o * obs_width / 2

        # 使用分离轴定理进行碰撞检测
        return ((np.abs(dis_matrix_x * cos_v + dis_matrix_y * sin_v) <=
                 np.abs(dx3 * cos_v + dy3 * sin_v) + np.abs(dx4 * cos_v + dy4 * sin_v) + half_l_v)
                & (np.abs(dis_matrix_x * sin_v - dis_matrix_y * cos_v) <=
                     np.abs(dx3 * sin_v - dy3 * cos_v) + np.abs(dx4 * sin_v - dy4 * cos_v) + half_w_v)
                & (np.abs(dis_matrix_x * cos_o + dis_matrix_y * sin_o) <=
                     np.abs(dx1 * cos_o + dy1 * sin_o) + np.abs(dx2 * cos_o + dy2 * sin_o) + half_l_o)
                & (np.abs(dis_matrix_x * sin_o - dis_matrix_y * cos_o) <=
                     np.abs(dx1 * sin_o - dy1 * cos_o) + np.abs(dx2 * sin_o - dy2 * cos_o) + half_w_o))