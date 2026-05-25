import numpy as np
from planner.CustomPlanner_rank3.planner_components.entites import RefPoint, TrajPoint
from planner.CustomPlanner_rank3.local_utils import  frenet_to_cartesian
from planner.CustomPlanner_rank3.planner_components.polyfit import QuarticPolynomial, QuinticPolynomial

LAT_COMFORT_COST_WEIGHT = 1  # 横向舒适度
LAT_OFFSET_COST_WEIGHT = 1  # 横向偏移量

class LatitudeSampler:
    def __init__(self, d_sample_space, s_sample_space):
        self.d_sample_space = d_sample_space # 纵向采样距离
        self.s_sample_space = s_sample_space # 横向采样距离
        self.d_init_condition = None

    def update_init(self, d_init_condition):
        self.d_init_condition = d_init_condition

    def sample(self):
        samples = []
        for d_value in self.d_sample_space:
            for s_value in self.s_sample_space:
                samples.append([(d_value, 0, 0), s_value]) # 返回终点处，横向的位移，横向速度，横向加速度
        return samples

    def fit_curves(self, samples):
        curves = []
        for sample in samples:
            curves.append(QuinticPolynomial(
                self.d_init_condition[0], self.d_init_condition[1], self.d_init_condition[2],
                sample[0][0], sample[0][1], sample[0][2], sample[1]
            ))
        return curves


class LongitudeSampler:
    def __init__(self, time_range, cruise_speed_range):
        self.time_range = time_range
        self.cruise_speed_range = cruise_speed_range
        self.s_init_condition = None

    def update_init(self, s_init_condition):
        self.s_init_condition = s_init_condition

    def sample_cruise(self):
        samples = []
        for speed in self.cruise_speed_range:
            for t in self.time_range:
                samples.append([(speed, 0), t])
        return samples

    def sample_obstacles(self, obstacles, cur_s):
        """
        采样有静态，动态障碍物时候的速度曲线，在这里我们只考虑跟车，为了安全考虑暂时不考虑超车
        :param obstacles: 静态，动态障碍物
        :param cur_s: 当前背景车的frenet纵轴坐标，为了剪枝操作
        :return: 采样点集
        """
        samples = []
        for obstacle in obstacles:
            for t in self.time_range:
                # 这里预测未来背景车辆的位移就先简单处理，假设车辆承匀加速直线运动
                predicted_s = obstacle.frenet_state[0][0] + t * obstacle.frenet_state[0][1] + 0.5 * obstacle.frenet_state[0][2] * t ** 2
                # if predicted_s < cur_s: # 如果背景车在当前车的后面，则暂时不考虑，进行剪枝
                #     continue

                # 这里的3代表安全车距，终点的速度为背景车的速度，即跟车速度
                samples.append([(predicted_s - 3, obstacle.frenet_state[0][1], 0), t])

        return samples

    def sample_stop(self):
        pass

    def fit_curves(self, samples):
        curves = []
        for sample in samples:
            if len(sample[0]) == 2:
                curves.append(QuarticPolynomial(
                    self.s_init_condition[0], self.s_init_condition[1], self.s_init_condition[2],
                    sample[0][0], sample[0][1], sample[1]
                ))
            elif len(sample[0]) == 3:
                curves.append(QuinticPolynomial(
                    self.s_init_condition[0], self.s_init_condition[1], self.s_init_condition[2],
                    sample[0][0], sample[0][1], sample[0][2], sample[1]
                ))
            else:
                raise Exception("Invalid sample size.")

        return curves


class LongitudeLatitudeCombiner:
    def __init__(self, time_range):
        self.lat_curve = None
        self.lon_curve = None
        self.time_range = time_range
        self.minimal_distance = 0.1
        self.minimal_speed = 0.1

    def update_curves(self, lat_curve, lon_curve):
        self.lat_curve = lat_curve
        self.lon_curve = lon_curve

    def calculate_min_abs_distance(self, s, s_list):
        s = np.array([s] * len(s_list))
        s_list = np.array(s_list)
        distances = np.abs(s - s_list)
        return np.argmin(distances), np.min(distances)

    def match_points(self, s, ref_points, fitting_curve):
        """
        将当前采样点和参考线上的最近的点做匹配，为转换成frenet坐标系做准备
        :param traj_point: 车辆当前的位置
        :param ref_points: 参考线
        :param fitting_curve: 参考线拟合出来的虚线
        :return: 匹配出来的参考线上的轨迹点
        """
        dist_min_index, dist_min = self.calculate_min_abs_distance(s, [each.rs for each in ref_points])

        path_point_min = ref_points[dist_min_index]
        if dist_min_index == 0 or dist_min_index == len(ref_points) - 1:
            return path_point_min
        else:
            inter_point_x, inter_point_y = fitting_curve.calc_position(s)
            return RefPoint(
                rx=inter_point_x, ry=inter_point_y, rs=s,
                rtheta=fitting_curve.calc_theta(s),
                rkappa=fitting_curve.calc_curvature(s),
                rdkappa=fitting_curve.calc_dcurvature(s))


    def combine(self, ref_points, fitting_curve):
        '''
        combine long and lat traj together
        F2C function is used to output future traj points in a list to follow
        '''
        traj_points = []
        lat_comfort_cost, lat_offset_cost = 0, 0
        s0 = self.lon_curve.evaluate(0, 0)
        s_ref_max = ref_points[-1].rs
        last_s = -self.minimal_distance

        for t in self.time_range:
            s = self.lon_curve.evaluate(0, t)
            if last_s > 0.0:
                s = max(last_s, s)
            last_s = s

            s_p = max(self.minimal_speed, self.lon_curve.evaluate(1, t))
            s_pp = self.lon_curve.evaluate(2, t)

            # 如果当前距离超过了最大距离（即当前参考线的长度），则说明后面的t一定会超过最大距离，就break
            if s > s_ref_max and len(traj_points) > 1:
                break

            relative_s = s - s0

            d = self.lat_curve.evaluate(0, relative_s)
            d_p = self.lat_curve.evaluate(1, relative_s)
            d_pp = self.lat_curve.evaluate(2, relative_s)

            _lat_comfort_cost, _lat_offset_cost = self.cal_lat_cost(d, d_p, d_pp, s, s_p, s_pp)
            lat_comfort_cost += _lat_comfort_cost
            lat_offset_cost += _lat_offset_cost

            matched_point = self.match_points(s, ref_points, fitting_curve)

            s_condition = np.array([s, s_p, s_pp])
            d_condition = np.array([d, d_p, d_pp])

            traj_point = frenet_to_cartesian(matched_point, s_condition, d_condition)
            traj_points.append(TrajPoint(x=traj_point[0], y=traj_point[1],
                                         v=traj_point[2], a=traj_point[3],
                                         theta=traj_point[4], kappa=traj_point[5]))
        lat_cost = lat_comfort_cost * LAT_COMFORT_COST_WEIGHT + lat_offset_cost * LAT_OFFSET_COST_WEIGHT
        return traj_points, lat_cost

    def cal_lat_cost(self, d, d_p, d_pp, s, s_p, s_pp):
        lat_a = d_pp * s_p ** 2 + d_p * s_pp
        _lat_comfort_cost = lat_a * lat_a
        _lat_offset_cost = d * d

        return _lat_comfort_cost, _lat_offset_cost