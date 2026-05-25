import osqp
import numpy as np
import scipy
from scipy.sparse import csc_matrix
INF = 1e9

class SpeedQPOptimizer:
    def __init__(self, config,
                 speed_t, prev_speed, prev_acc, start_s, end_s,
                 obstacle_names, obstacle_decisions, obstacle_st_boundaries, warm):
        self.config = config
        self.speed_t = speed_t
        self.cruise_speed = self.config.cruise_speed
        self.speed_limit = self.config.speed_max
        self.acc_max = self.config.acc_max
        self.acc_min = self.config.acc_min
        self.jerk_max = self.config.jerk_max
        self.jerk_min = self.config.jerk_min
        self.t_size = len(self.speed_t)

        self.prev_speed = prev_speed
        self.prev_acc = prev_acc
        self.start_s = start_s
        self.end_s = end_s

        self.obstacle_names = obstacle_names
        self.obstacle_decisions = obstacle_decisions
        self.obstacle_st_boundaries = obstacle_st_boundaries


        self.w_acc = self.config.speed_w_acc
        self.w_jerk = self.config.speed_w_jerk
        self.w_cruise = self.config.speed_w_cruise
        self.w_end_s = self.config.speed_w_end_s
        self.w_follow = self.config.speed_w_follow # 目标函数中跟车距离的权重
        self.w_overtake = self.config.speed_w_overtake  # 目标函数中超车距离的权重

        self.s_safe_overtake = max(self.prev_speed, self.config.min_overtake)  # 超车距离的安全距离
        self.s_safe_follow = max(self.prev_speed, self.config.min_follow)  # 跟车距离的安全距离

        self.warm = warm
        self.init_matrices()

    def init_matrices(self):
        """
        初始化二次规划矩阵
        :return:
        """
        self.A = np.zeros([6 * self.t_size, 3 * self.t_size]) # 初始化二次约束矩阵
        self.P = np.zeros([3 * self.t_size, 3 * self.t_size]) # 目标函数的二次矩阵

        self.u = np.zeros([6 * self.t_size, 1]) # 初始化约束下界
        self.l = np.zeros([6 * self.t_size, 1]) # 初始化约束上界
        self.q = np.zeros([3 * self.t_size, 1])  # 初始化二次约束矩阵

    def reset(self):
        """
        初始化二次规划矩阵
        :return:
        """
        self.A = np.zeros([6 * self.t_size, 3 * self.t_size])  # 初始化二次约束矩阵
        self.P = np.zeros([3 * self.t_size, 3 * self.t_size])  # 目标函数的二次矩阵

        self.u = np.zeros([6 * self.t_size, 1])  # 初始化约束下界
        self.l = np.zeros([6 * self.t_size, 1])  # 初始化约束上界
        self.q = np.zeros([3 * self.t_size, 1])  # 初始化二次约束矩阵

    def add_aux_obj_cons_for_collision(self):
        """
        获取障碍物的st边界
        :return:
        """
        total_aux_var_size = 0
        total_indices = []
        aux_A1_bmat_list = []
        aux_l1_list = []
        aux_u1_list = []
        aux_P_list = []

        for name, decision in zip(self.obstacle_names, self.obstacle_decisions):
            tail_bounds, head_bounds = self.obstacle_st_boundaries[name]
            indices = np.arange(0, self.t_size, dtype=np.int32)
            overtake_bounds = head_bounds + self.s_safe_overtake
            follow_bounds = tail_bounds - self.s_safe_follow
            # 跟车
            if decision == 0:
                valid_follow_bounds = follow_bounds[tail_bounds != 1e6]
                indices = indices[tail_bounds != 1e6]
                aux_var_size = len(valid_follow_bounds)

                temp_aux_A1 = scipy.sparse.diags(-np.ones([aux_var_size]), format="csc")
                temp_aux_l1 = -np.ones([aux_var_size, 1]) * np.inf
                temp_aux_u1 = np.expand_dims(valid_follow_bounds, -1)

                total_aux_var_size += aux_var_size
                aux_A1_bmat_list.append(temp_aux_A1)
                aux_l1_list.append(temp_aux_l1)
                aux_u1_list.append(temp_aux_u1)

                aux_P_list.append(np.ones([aux_var_size]) * self.w_follow)
                total_indices.append(indices)
            # 超车
            else:
                valid_overtake_bounds = overtake_bounds[overtake_bounds != -1e6]
                indices = indices[head_bounds != -1e6]
                aux_var_size = len(valid_overtake_bounds)

                temp_aux_A1 = scipy.sparse.diags(np.ones([aux_var_size]), format="csc")
                temp_aux_l1 = np.expand_dims(valid_overtake_bounds, -1)
                temp_aux_u1 = np.ones([aux_var_size, 1]) * np.inf

                total_aux_var_size += aux_var_size
                aux_A1_bmat_list.append(temp_aux_A1)
                aux_l1_list.append(temp_aux_l1)
                aux_u1_list.append(temp_aux_u1)

                aux_P_list.append(np.ones([aux_var_size]) * self.w_overtake)
                total_indices.append(indices)

        self.A = csc_matrix(self.A)
        self.P = csc_matrix(self.P)
        if total_aux_var_size == 0:
            return


        aux_A1 = scipy.sparse.block_diag(aux_A1_bmat_list, format="csc")
        aux_l1 = np.concatenate(aux_l1_list, axis=0)
        aux_u1 = np.concatenate(aux_u1_list, axis=0)

        aux_A2 = scipy.sparse.diags(np.ones([total_aux_var_size]), format="csc")
        aux_l2 = np.zeros([total_aux_var_size, 1])
        aux_u2 = np.ones([total_aux_var_size, 1]) * np.inf

        aux_A = scipy.sparse.vstack([aux_A1, aux_A2])
        aux_l = np.r_[aux_l1, aux_l2]
        aux_u = np.r_[aux_u1, aux_u2]
        aux_P = scipy.sparse.diags(np.concatenate(aux_P_list, axis=0), format="csc")
        aux_q = np.ones([total_aux_var_size, 1])

        total_indices = np.concatenate(total_indices, axis=0)
        aux_A_below = np.zeros((total_aux_var_size, 3 * self.t_size))
        for i, each in enumerate(total_indices):
            aux_A_below[i, each] = 1
        aux_A_below = csc_matrix(aux_A_below)
        aux_A_below = scipy.sparse.vstack([aux_A_below, aux_A_below])


        self.A = scipy.sparse.bmat([[self.A, None], [aux_A_below, aux_A]], format="csc")
        self.P = scipy.sparse.block_diag([self.P, aux_P], format="csc")



        self.l = np.r_[self.l, aux_l]
        self.u = np.r_[self.u, aux_u]
        self.q = np.r_[self.q, aux_q]

    def add_objective(self):
        self.add_comfort_objective()
        self.add_cruise_speed_objective()
        self.add_destination_objective()


    def add_comfort_objective(self):
        """
        添加舒适性目标函数约束，主要是一阶导，二阶导，三阶导尽可能平滑
        :return:
        """
        self.P[self.t_size * 2: self.t_size * 3, self.t_size * 2: self.t_size * 3] += np.eye(self.t_size) * self.w_acc

        for i in range(self.t_size - 1):
            dt = self.speed_t[i+1] - self.speed_t[i]
            self.P[self.t_size * 2 + i, self.t_size * 2 + i] += self.w_jerk / (dt ** 2)
            self.P[self.t_size * 2 + i + 1, self.t_size * 2 + i + 1] += self.w_jerk / (dt ** 2)
            self.P[self.t_size * 2 + i, self.t_size * 2 + i + 1] += (-1 * self.w_jerk) / (dt ** 2)
            self.P[self.t_size * 2 + i + 1, self.t_size * 2 + i] += (-1 * self.w_jerk) / (dt ** 2)

    def add_cruise_speed_objective(self):
        self.P[self.t_size: self.t_size * 2, self.t_size: self.t_size * 2] += np.eye(self.t_size) * self.w_cruise
        self.q[self.t_size: self.t_size * 2] += -2 * self.cruise_speed * self.w_cruise

    def add_destination_objective(self):
        """
        关于终点的额外约束
        :return:
        """
        self.P[self.t_size * 1 - 1, self.t_size * 1 - 1] += self.w_end_s
        self.q[self.t_size * 1 - 1] += -2 * self.end_s * self.w_end_s

    def set_constraints(self):
        self.set_s_constraints()
        self.set_speed_constraints()
        self.set_acc_constraints()
        self.set_jerk_constraints()
        self.set_dcontinuity_constraints()
        self.set_start_point_constraints()

    def set_s_constraints(self):
        """
        添加横向位移约束，1）不能超过道路行驶边界；2）不能与静态障碍物相撞
        :return:
        """
        self.set_road_constraints()
        self.set_obstacle_boundary_constraints()

    def set_road_constraints(self):
        """
        车道线边界约束
        :return:
        """
        self.A[:self.t_size, :self.t_size] = np.eye(self.t_size)
        self.u[:self.t_size, 0] = self.prev_speed * self.speed_t + 0.5 * self.acc_max * self.speed_t ** 2
        t_stop = np.clip(self.speed_t, a_min=0, a_max=-self.prev_speed / self.acc_min)
        self.l[:self.t_size, 0] = self.prev_speed * t_stop + 0.5 * self.acc_min * t_stop ** 2

    def set_obstacle_boundary_constraints(self):
        """
        动态障碍物约束
        :return:
        """
        for (key, value) in zip(self.obstacle_names, self.obstacle_decisions):
            # 如果value是0，说明决策是跟车决策，那么上界设为st_boundary的下界
            if value == 0:
                upper_bound = self.obstacle_st_boundaries[key][0]
                self.u[:self.t_size, 0] = np.clip(upper_bound, a_min=-9999, a_max=self.u[:self.t_size, 0])

            # 如果value是1，说明决策是超车决策，那么下界设为st_boundary的上界
            else:
                lower_bound = self.obstacle_st_boundaries[key][1]
                self.l[:self.t_size, 0] = np.clip(lower_bound, a_min=self.l[:self.t_size, 0], a_max=9999)

        for i in range(self.t_size):
            if self.u[i, 0] < self.l[i, 0]:
                self.u[i, 0] = self.l[i, 0]

    def set_speed_constraints(self):
        """
        添加横向位移一阶导约束
        :return:
        """


        self.A[self.t_size: self.t_size * 2, self.t_size:self.t_size * 2] = np.eye(self.t_size)
        self.l[self.t_size: self.t_size * 2, :] = 0
        self.u[self.t_size: self.t_size * 2, :] = self.speed_limit

        if self.warm:
            self.u[self.t_size: self.t_size + 20, 0] = self.speed_limit * 1.5 - (0.5 * self.speed_limit) / 20 * np.arange(0, 20, 1)

    def set_acc_constraints(self):
        """
        添加横向位移二阶导约束
        :return:
        """
        self.A[self.t_size * 2: self.t_size * 3, self.t_size * 2: self.t_size * 3] = np.eye(self.t_size)
        self.l[self.t_size * 2: self.t_size * 3, :] = self.acc_min
        self.u[self.t_size * 2: self.t_size * 3, :] = self.acc_max

    def set_jerk_constraints(self):
        """
        添加横向位移三阶导约束
        :return:
        """
        for i in range(self.t_size-1):
            self.A[self.t_size * 3 + i, self.t_size * 2 + i] = -1
            self.A[self.t_size * 3 + i, self.t_size * 2 + i + 1] = 1

        self.l[self.t_size * 3: self.t_size * 4 - 1, :] = self.jerk_min
        self.u[self.t_size * 3: self.t_size * 4 - 1, :] = self.jerk_max

    def set_dcontinuity_constraints(self):
        """
        添加横向位移导数连续的约束
        :return:
        """
        begin_idx = self.t_size * 4 - 1
        for i in range(self.t_size - 1):
            dt = self.speed_t[i + 1] - self.speed_t[i]
            self.A[begin_idx + i, i] = 1
            self.A[begin_idx + i, i + 1] = -1
            self.A[begin_idx + i, self.t_size + i] = dt
            self.A[begin_idx + i, self.t_size + i + 1] = 0
            self.A[begin_idx + i, self.t_size * 2 + i] = dt ** 2 / 3
            self.A[begin_idx + i, self.t_size * 2 + i + 1] = dt ** 2 / 6

        begin_idx = self.t_size * 5 - 2
        for i in range(self.t_size - 1):
            dt = self.speed_t[i + 1] - self.speed_t[i]

            self.A[begin_idx + i, self.t_size + i] = 1
            self.A[begin_idx + i, self.t_size + i + 1] = -1
            self.A[begin_idx + i, self.t_size * 2 + i] = dt / 2
            self.A[begin_idx + i, self.t_size * 2 + i + 1] = dt / 2

    def set_start_point_constraints(self):
        """
        添加起始点约束
        :return:
        """
        begin_idx = self.t_size * 6 - 3
        self.A[begin_idx, 0] = 1
        self.A[begin_idx + 1, self.t_size] = 1
        self.A[begin_idx + 2, 2 * self.t_size] = 1

        self.l[begin_idx, :] = self.start_s
        self.l[begin_idx + 1, :] = self.prev_speed
        self.l[begin_idx + 2, :] = self.prev_acc

        self.u[begin_idx, :] = self.start_s
        self.u[begin_idx + 1, :] = self.prev_speed
        self.u[begin_idx + 2, :] = self.prev_acc

    def solve(self):
        """
        求解
        :return:
        """
        prob = osqp.OSQP()
        prob.setup(self.P, self.q, self.A, self.l, self.u, verbose=0)
        res = prob.solve()
        self.res = res.x
        return res, res.info.status

