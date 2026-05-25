import math
import osqp
import numpy as np
from scipy.sparse import csc_matrix
from scipy.linalg import ldl
from planner.CustomPlanner_rank3.planner_components.cubic_spline import CubicSpline2D

class PathOptimizer:

    def __init__(self, config, l_min, l_max, ref_points, ref_curve,
                 s_range, obstacles,
                 prev_speed, start_l, start_dl, start_ddl,
                 ego_width=2, ego_length=4, warming=False):
        """
        路径优化器
        :param l_min: 道路左边界范围
        :param l_max: 道路右边界范围
        :param ref_points: 参考线
        :param ref_curve: 参考线拟合出来的曲线
        :param prev_speed: t-1时刻的速度
        :param start_l: 开始时刻ego的横向位移
        :param start_dl: 开始时刻ego横向位移一阶导
        :param start_ddl: 开始时刻ego横向位移二阶导
        :param end_l: 开始时刻ego的横向位移（默认为0）
        :param end_dl: 终止时刻ego横向位移一阶导（默认为0）
        :param end_ddl: 终止时刻ego横向位移二阶导（默认为0）
        :param obstacles: 障碍物
        :param max_rot: 方向盘最大转角
        :param axis_distance: 默认的轴距
        :param w_l: 目标函数中，横向偏移误差权重
        :param w_dl:目标函数中，横向偏移一阶导权重
        :param w_ddl:目标函数中，横向偏移二阶导权重
        :param w_dddl:目标函数中，横向偏移三阶导权重
        :param w_end_l: 目标函数中，终止时刻偏移误差权重
        :param w_end_dl:目标函数中，终止时刻偏移误差一阶导权重
        :param w_end_ddl:目标函数中，终止时刻偏移误差二阶导权重
        :param ego_width:ego的宽度
        :param ego_length: ego的长度
        """
        self.config = config
        self.s_range = s_range
        self.s_size = len(self.s_range)
        self.l_min = l_min
        self.l_max = l_max
        self.ref_points = ref_points
        self.ref_curve = ref_curve
        self.ref_curvature = [each.rkappa for each in self.ref_points]
        self.prev_speed = prev_speed
        self.start_l = start_l
        self.start_dl = start_dl
        # self.start_ddl = start_ddl
        self.start_ddl = 0
        self.obstacles = obstacles
        self.ego_width = ego_width
        self.ego_length = ego_length

        self.dl_bound = 2.0 # 沿用apollo的参数
        self.max_rot = self.config.max_rot
        self.axis_distance = self.config.axis_distance
        self.max_curvature = math.tan(self.max_rot) / self.axis_distance
        self.max_dcurvature = self.max_curvature / max(prev_speed, 1e-12)

        self.w_l = self.config.path_w_l
        self.w_dl = self.config.path_w_dl
        self.w_ddl = self.config.path_w_ddl
        self.w_dddl = self.config.path_w_dddl
        self.w_end_l = self.config.path_w_end_l
        self.w_end_dl = self.config.path_w_end_dl
        self.w_end_ddl = self.config.path_w_end_ddl
        self.warming = warming
        self.A = None # 二次规划约束矩阵
        self.P = None # 二次规划目标函数二次矩阵
        self.q = None # 目标函数一次矩阵
        self.l = None # 约束下界
        self.u = None # 约束上界
        self.res = None # 返回结果
        self.init_matrices()

    def init_matrices(self):
        """
        初始化二次规划矩阵
        :return:
        """
        self.A = np.zeros([6 * self.s_size, 3 * self.s_size]) # 初始化二次约束矩阵
        self.P = np.zeros([3 * self.s_size, 3 * self.s_size]) # 目标函数的二次矩阵

        self.u = np.zeros([6 * self.s_size, 1]) # 初始化约束下界
        self.l = np.zeros([6 * self.s_size, 1]) # 初始化约束上界
        self.q = np.zeros([3 * self.s_size, 1])  # 初始化二次约束矩阵

    def reset(self):
        """
        初始化二次规划矩阵
        :return:
        """
        self.A = np.zeros([6 * self.s_size, 3 * self.s_size])  # 初始化二次约束矩阵
        self.P = np.zeros([3 * self.s_size, 3 * self.s_size])  # 目标函数的二次矩阵

        self.u = np.zeros([6 * self.s_size, 1])  # 初始化约束下界
        self.l = np.zeros([6 * self.s_size, 1])  # 初始化约束上界
        self.q = np.zeros([3 * self.s_size, 1])  # 初始化二次约束矩阵

    def add_objective(self):
        self.add_safety_objective()
        self.add_comfort_objective()
        self.add_destination_objective()

    def add_safety_objective(self):
        """
        关于安全性的目标函数，车辆应该行驶在道路的中央
        # TODO：如果没有q矩阵的值的话，那就是车辆应该按照参考线行驶，但是实际中道路会有障碍物，所以应该按照车辆行驶边界的中心行驶
        :return:
        """
        lane_boundary = np.array([min(each0, each1) for each0, each1 in zip(np.abs(self.l_min), np.abs(self.l_max))])
        diff_boundary = np.zeros_like(lane_boundary)
        for i in range(len(lane_boundary)):
            # 如果constraint后的右边界在道路右边界内侧，则说明，道路右侧→障碍物需要避开
            if self.l[i, 0] > -lane_boundary[i]:
                diff_boundary[i] = self.l[i, 0] + lane_boundary[i]

            # 如果constraint后的左边界在道路左边界内侧，则说明，道路左侧障碍物需要避开
            if self.u[i, 0] < lane_boundary[i]:
                diff_boundary[i] = self.u[i, 0] - lane_boundary[i]

        diff_boundary = diff_boundary.reshape(-1, 1)
        self.P[:self.s_size, :self.s_size] += np.eye(self.s_size) * self.w_l
        self.q[:self.s_size, :] += -0.5 * self.w_l * diff_boundary

    def add_comfort_objective(self):
        """
        添加舒适性目标函数约束，主要是一阶导，二阶导，三阶导尽可能平滑
        :return:
        """
        self.P[self.s_size: self.s_size * 2, self.s_size: self.s_size * 2] += np.eye(self.s_size) * self.w_dl
        self.P[self.s_size * 2: self.s_size * 3, self.s_size * 2: self.s_size * 3] += np.eye(self.s_size) * self.w_ddl

        for i in range(self.s_size - 1):
            ds = self.s_range[i+1] - self.s_range[i]
            self.P[self.s_size * 2 + i, self.s_size * 2 + i] += self.w_dddl / (ds ** 2)
            self.P[self.s_size * 2 + i + 1, self.s_size * 2 + i + 1] += self.w_dddl / (ds ** 2)
            self.P[self.s_size * 2 + i, self.s_size * 2 + i + 1] += (-1 * self.w_dddl) / (ds ** 2)
            self.P[self.s_size * 2 + i + 1, self.s_size * 2 + i] += (-1 * self.w_dddl) / (ds ** 2)

    def add_destination_objective(self):
        """
        关于终点的额外约束
        # TODO: 这里暂时不考虑终点的横向位移，默认为0，所以只有二次项
        :return:
        """
        self.P[self.s_size * 1 - 1, self.s_size * 1 - 1] += self.w_end_l
        self.P[self.s_size * 2 - 1, self.s_size * 2 - 1] += self.w_end_dl
        self.P[self.s_size * 3 - 1, self.s_size * 3 - 1] += self.w_end_ddl

    def set_constraints(self, omit_obstacle=False):
        self.set_l_constraints(omit_obstacle)
        self.set_dl_constraints()
        self.set_ddl_constraints()
        self.set_dddl_constraints()
        self.set_dcontinuity_constraints()
        self.set_start_point_constraints()

    def set_optional_dynamic_obstacle_constraints(self, st_boundaries, path_obstacles):
        for name, boundaries in st_boundaries.items():
            # 只看未来两秒内产生的碰撞区间
            non_none_lower_boundary = [each for each in boundaries[0][:4] if each is not None]
            non_none_upper_boundary = [each for each in boundaries[1][:4] if each is not None]

            if len(non_none_lower_boundary) == 0 or len(non_none_upper_boundary) == 0:
                continue

            s_lower = min(non_none_lower_boundary) - 5
            s_upper = max(non_none_upper_boundary) + 5
            s_lower = np.argmin(np.abs(s_lower - self.s_range))
            s_upper = np.argmin(np.abs(s_upper - self.s_range))
            # 如果当前障碍车的位置在参考线左侧，那道路的下界设成车的右边界
            if not path_obstacles.__contains__(name):
                continue

            if path_obstacles[name].frenet_state[1][0] > 0:
                obs_left_bound = path_obstacles[name].frenet_state[1][0] - path_obstacles[name].width / 2 - self.ego_width / 2 - 0.5
                self.u[s_lower:s_upper, :] = np.clip(obs_left_bound, a_min=-9999, a_max=self.u[s_lower:s_upper, :])

            # 如果当前障碍车的位置在参考线右侧，那道路的上界设成车的左边界
            else:
                obs_right_bound = path_obstacles[name].frenet_state[1][0] + path_obstacles[name].width / 2 + self.ego_width / 2 + 0.5
                self.l[s_lower:s_upper, :] = np.clip(obs_right_bound, a_min=self.l[s_lower:s_upper, :], a_max=9999)

        for i in range(self.s_size):
            if self.u[i, 0] < self.l[i, 0]:
                self.u[i, 0] = 0
                self.l[i, 0] = 0
    def set_l_constraints(self, omit_obstacle=False):
        """
        添加横向位移约束，1）不能超过道路行驶边界；2）不能与静态障碍物相撞
        :return:
        """
        self.set_road_boundary_constraints()
        if not omit_obstacle:
            self.set_obstacle_boundary_constraints()

    def set_road_boundary_constraints(self):
        """
        车道线边界约束
        :return:
        """
        self.A[:self.s_size, :self.s_size] = np.eye(self.s_size)
        # self.l[:self.s_size, :] = np.kron(self.l_min, np.ones([self.inter_ratio, 1]))
        # self.u[:self.s_size, :] = np.kron(self.l_max, np.ones([self.inter_ratio, 1]))
        self.l[:self.s_size, :] = self.l_min
        self.u[:self.s_size, :] = self.l_max
        # # 有的车开始的时候就在可行驶区域外，先设置一个缓冲区，让他回到车道线内
        if self.warming:
            self.l[0:30, :] = -5
            self.u[0:30, :] = 5

    def set_obstacle_boundary_constraints(self):
        """
        静态障碍物约束
        :return:
        """
        for obstacle in self.obstacles.values():
            if obstacle.mode == "dynamic":
                continue

            s_lower = obstacle.frenet_state[0][0] - obstacle.length / 2 - self.ego_length / 2 - 5 - self.ref_points[0].rs
            s_lower = max(np.argmin(np.abs(s_lower - self.s_range)), 1)
            s_upper = obstacle.frenet_state[0][0] + obstacle.length / 2 + self.ego_length / 2 + 5 - self.ref_points[0].rs
            s_upper = np.argmin(np.abs(s_upper - self.s_range))


            # 如果当前障碍物不在路上，就说明对行驶没有影响
            if obstacle.frenet_state[1][0] > 3.5 or obstacle.frenet_state[1][0] < -3.5:
                continue

            # 如果当前障碍车的位置在参考线左侧，那道路的下界设成车的右边界
            if obstacle.frenet_state[1][0] > 0:
                obs_left_bound = obstacle.frenet_state[1][0] - obstacle.width / 2 - self.ego_width / 2 - 0.3
                self.u[s_lower:s_upper, :] = np.clip(obs_left_bound, a_min=-9999, a_max=self.u[s_lower:s_upper, :])

            # 如果当前障碍车的位置在参考线右侧，那道路的上界设成车的左边界
            else:
                obs_right_bound = obstacle.frenet_state[1][0] + obstacle.width / 2 + self.ego_width / 2 + 0.3
                self.l[s_lower:s_upper, :] = np.clip(obs_right_bound, a_min=self.l[s_lower:s_upper, :], a_max=9999)


    def set_dl_constraints(self):
        """
        添加横向位移一阶导约束
        :return:
        """
        self.A[self.s_size: self.s_size * 2, self.s_size:self.s_size * 2] = np.eye(self.s_size)
        self.l[self.s_size: self.s_size * 2, :] = -self.dl_bound
        self.u[self.s_size: self.s_size * 2, :] = self.dl_bound

    def set_ddl_constraints(self):
        """
        添加横向位移二阶导约束
        :return:
        """
        self.A[self.s_size * 2: self.s_size * 3, self.s_size*2: self.s_size * 3] = np.eye(self.s_size)
        # TODO: curvature这块儿算的有点问题
        curvature_bound = np.array(
            [[max(self.max_curvature - self.ref_curvature[i], 0)] for i in range(len(self.ref_curvature))])

        # curvature_bound = np.kron(curvature_bound, np.ones([self.inter_ratio, 1]))
        self.l[self.s_size * 2: self.s_size * 3, :] = -curvature_bound
        self.u[self.s_size * 2: self.s_size * 3, :] = curvature_bound

    def set_dddl_constraints(self):
        """
        添加横向位移三阶导约束
        :return:
        """
        dcurvature_bound = np.array(
            [[self.max_dcurvature] for i in range(len(self.ref_curvature))])

        # dcurvature_bound = np.kron(dcurvature_bound, np.ones([self.inter_ratio, 1]))
        for i in range(self.s_size-1):
            self.A[self.s_size * 3 + i, self.s_size * 2 + i] = -1
            self.A[self.s_size * 3 + i, self.s_size * 2 + i + 1] = 1

        self.l[self.s_size * 3: self.s_size * 4 - 1, :] = -dcurvature_bound[:-1]
        self.u[self.s_size * 3: self.s_size * 4 - 1, :] = dcurvature_bound[:-1]

    def set_dcontinuity_constraints(self):
        """
        添加横向位移导数连续的约束
        :return:
        """
        begin_idx = self.s_size * 4 - 1
        for i in range(self.s_size - 1):
            ds = self.s_range[i + 1] - self.s_range[i]
            self.A[begin_idx + i, i] = 1
            self.A[begin_idx + i, i + 1] = -1
            self.A[begin_idx + i, self.s_size + i] = ds
            self.A[begin_idx + i, self.s_size + i + 1] = 0
            self.A[begin_idx + i, self.s_size * 2 + i] = ds ** 2 / 3
            self.A[begin_idx + i, self.s_size * 2 + i + 1] = ds ** 2 / 6

        begin_idx = self.s_size * 5 - 2
        for i in range(self.s_size - 1):
            ds = self.s_range[i + 1] - self.s_range[i]
            self.A[begin_idx + i, self.s_size + i] = 1
            self.A[begin_idx + i, self.s_size + i + 1] = -1
            self.A[begin_idx + i, self.s_size * 2 + i] = ds / 2
            self.A[begin_idx + i, self.s_size * 2 + i + 1] = ds / 2

    def set_start_point_constraints(self):
        """
        添加起始点约束
        :return:
        """
        begin_idx = self.s_size * 6 - 3
        self.A[begin_idx, 0] = 1
        self.A[begin_idx + 1, self.s_size] = 1
        self.A[begin_idx + 2, 2 * self.s_size] = 1

        self.l[begin_idx, :] = self.start_l
        self.l[begin_idx + 1, :] = self.start_dl
        self.l[begin_idx + 2, :] = self.start_ddl

        self.u[begin_idx, :] = self.start_l
        self.u[begin_idx + 1, :] = self.start_dl
        self.u[begin_idx + 2, :] = self.start_ddl

    def solve(self):
        """
        求解
        :return:
        """
        prob = osqp.OSQP()
        # 如果出现下界大于上界的情况，说明模型必然无解，优化模型需要进行放缩
        if not np.all(self.l <= self.u):
            return 1e10, "primal infeasible"

        prob.setup(csc_matrix(self.P * 2), self.q, csc_matrix(self.A), self.l, self.u, verbose=0)
        res = prob.solve()
        self.res = res.x
        return res, res.info.status

    def to_xy(self):
        """
        将路径规划由frenet坐标系转换成Cartesian坐标系
        :return:
        """
        x, y = self.ref_curve.calc_position(self.ref_points[0].rs + self.s_range)
        theta = self.ref_curve.calc_theta(self.ref_points[0].rs + self.s_range)
        cos_theta_r = np.cos(theta)
        sin_theta_r = np.sin(theta)

        x = x - sin_theta_r * self.res[:self.s_size]
        y = y + cos_theta_r * self.res[:self.s_size]

        return x, y