import math
import osqp
import numpy as np
import scipy
from scipy.sparse import csc_matrix
from scipy.optimize import Bounds, LinearConstraint, NonlinearConstraint
import cvxpy as cp
# from cvxopt import matrix, solvers
# from cyipopt
INF = 1e9
class SpeedDPOptimizer:
    def __init__(self, ego, obstacles,
                 speed_s, speed_t,
                 prev_speed, prev_acc, path_x, path_y, path_s):
        """
        速度规划中的动态规划
        :param ego:自驾车
        :param obstacles: 障碍物（动态or静态）
        :param prev_speed: t-1时刻的速度
        :param prev_acc: t-1时刻的加速度
        :param path_x: 路径规划 x list
        :param path_y: 路径规划 y list
        :param path_s: 路径规划 s list
        """
        self.ego = ego
        self.obstacles = obstacles
        self.prev_speed = prev_speed
        self.prev_acc = prev_acc
        self.s_size = len(speed_s)
        self.t_size = len(speed_t)
        self.speed_s = speed_s
        self.speed_t = speed_t
        self.path_x = path_x
        self.path_y = path_y
        self.path_s = path_s

        # self.s_safe_overtake = self.prev_speed # 超车距离的安全距离
        # self.s_safe_follow = self.prev_speed # 跟车距离的安全距离
        # self.s_safe_overtake = 3  # 超车距离的安全距离
        self.s_safe_overtake = max(self.prev_speed, 5)  # 超车距离的安全距离
        # self.s_safe_follow = 3  # 跟车距离的安全距离
        self.s_safe_follow = max(self.prev_speed, 5)  # 跟车距离的安全距离
        self.upper_speed_limit = 120 / 3.6 #最大车速限制
        self.lower_speed_limit = 3.6 / 3.6 #最大车速限制
        self.acc_min = -5.0 # 最小加速度限制
        self.acc_max = 5.0 # 最大加速度限制
        self.jerk_min = -30 # 最小加加速度限制
        self.jerk_max = 30 # 最大加加速度限制

        self.w_follow = 1e3 # 目标函数中跟车距离的权重
        self.w_overtake = 1e3 # 目标函数中超车距离的权重
        self.w_spatial = 1e3 # 目标函数中与终点距离的权重
        self.w_exceed_speed = 10 # 超速的权重
        self.w_lower_speed = 2.5 # 低于最低速度权重
        self.w_acc = 2.0 # 加速度权重
        self.w_deacc = 2.0 # 减速度权重
        self.w_jerk_pos = 1 # 加加速度权重
        self.w_jerk_neg = 300 # 负加加速度权重

        self.obs_st_boundaries = {} # 障碍物的st边界

    def get_st_boundaries(self):
        """
        获取障碍物的st边界
        :return:
        """
        for obstacle in self.obstacles.values():
            tail_bounds, head_bounds = obstacle.get_precise_st_boundaries(self.ego, self.path_x, self.path_y, self.path_s, self.speed_t)
            self.obs_st_boundaries[obstacle.name] = (tail_bounds, head_bounds)
        return self.obs_st_boundaries

    def get_obstacles_cost(self):
        """
        获取障碍物的碰撞或者超车，跟车产生的cost
        :return:
        """
        cost = np.zeros([self.t_size, self.s_size])

        for obstacle in self.obstacles.values():
            tail_bounds, head_bounds = self.obs_st_boundaries[obstacle.name]
            for t_index in range(self.t_size):
                # if head_bounds[t_index] is None or tail_bounds[t_index] is None:
                if head_bounds[t_index] == -1e6 or tail_bounds[t_index] == 1e6:
                    continue

                head_bound = head_bounds[t_index]
                tail_bound = tail_bounds[t_index]
                overtake_bound = head_bound + self.s_safe_overtake
                follow_bound = tail_bound - self.s_safe_follow

                for s_index in range(self.s_size):
                    # 当前距离>tail bound，<head bound 说明撞车了，cost为inf
                    if self.speed_s[s_index] >= tail_bound and self.speed_s[s_index] <= head_bound:
                        cost[t_index, s_index] += INF
                    # 当前距离>overtake bound 说明超车无碰撞风险，cost为0
                    elif self.speed_s[s_index] >= overtake_bound:
                        cost[t_index, s_index] += 0
                    # 当前距离<follow bound 说明跟车无碰撞风险，cost为0
                    elif self.speed_s[s_index] < follow_bound:
                        cost[t_index, s_index] += 0
                    # 当前距离在head bound和overtake bound之间 说明有部分碰撞风险，cost为欧氏距离
                    elif self.speed_s[s_index] < overtake_bound and self.speed_s[s_index] > head_bound:
                        cost[t_index, s_index] += self.w_overtake * (overtake_bound - self.speed_s[s_index]) ** 2
                        # 当前距离在follow bound和follow bound之间 说明有部分碰撞风险，cost为欧氏距离
                    elif self.speed_s[s_index] > follow_bound and self.speed_s[s_index] < tail_bound:
                        cost[t_index, s_index] += self.w_follow * (follow_bound - self.speed_s[s_index]) ** 2
        return cost

    def get_spatial_cost(self):
        """
        与终点距离的cost
        :return:
        """
        return self.w_spatial * np.array([np.abs((self.speed_s[-1] - self.speed_s)) for _ in self.speed_t])

    def get_transfer_cost_t3(self, s0, s1, s2, s3, dt):
        """
        状态转移函数，t >= 3
        :param s0: t-3时刻位置
        :param s1: t-2时刻位置
        :param s2: t-1时刻位置
        :param s3: t时刻位置
        :return:
        """
        # return self.get_transfer_speed_cost(s2, s3) + self.get_transfer_acc_cost_normal(s1, s2, s3) + self.get_transfer_jerk_cost_normal(s0, s1, s2, s3)
        return self.get_transfer_speed_cost(s2, s3, dt) + self.get_transfer_acc_cost_normal(s1, s2, s3, dt)

    def get_transfer_cost_t2(self, v1, s1, s2, s3, dt):
        """
        状态转移函数，t >= 2
        :param v1: t-1时刻速度
        :param s1: t-2时刻位置
        :param s2: t-1时刻位置
        :param s3: t时刻位置
        :return:
        """
        # return self.get_transfer_speed_cost(s2, s3) + self.get_transfer_acc_cost_normal(s1, s2, s3) + self.get_transfer_jerk_cost_prev_speed(v1, s1, s2, s3)
        return self.get_transfer_speed_cost(s2, s3, dt) + self.get_transfer_acc_cost_normal(s1, s2, s3, dt)

    def get_transfer_cost_t1(self, acc2, v2, s2, s3, dt):
        """
        状态转移函数，t >= 1
        :param acc2: t-1时刻加速度
        :param v2: t-1时刻速度
        :param s2: t-1时刻位置
        :param s3: t时刻位置
        :return:
        """
        # return self.get_transfer_speed_cost(s2, s3) + self.get_transfer_acc_cost_prev_speed(v2, s2, s3) + self.get_transfer_jerk_cost_prev_acc_speed(acc2, v2, s2, s3)
        return self.get_transfer_speed_cost(s2, s3, dt) + self.get_transfer_acc_cost_prev_speed(v2, s2, s3, dt)

    def get_transfer_speed_cost(self, s2, s3, dt):
        """
        状态转移时的速度cost
        :param s2:
        :param s3:
        :return:
        """
        speed = (s3 - s2) / dt
        upper_speed_det = (speed - self.upper_speed_limit) / self.upper_speed_limit
        lower_speed_det = (speed - self.lower_speed_limit) / self.lower_speed_limit
        if speed < 0:
            return INF
        else:
            if upper_speed_det > 0:
                # 针对于告诉情况的限速，如果超速了，立刻减速到限速内
                return (upper_speed_det + 1) * self.w_exceed_speed
            if lower_speed_det < 0:
                return -lower_speed_det * self.w_lower_speed
            return 0

    def get_transfer_acc_cost_normal(self, s1, s2, s3, dt):
        """
        状态转移时的加速度cost
        :return:
        """
        acc = (s3 + s1 - 2 * s2) / dt ** 2
        if acc > self.acc_max or acc < self.acc_min:
            return INF
        else:
            temp = ((self.w_deacc * acc) ** 2) / (1 + np.exp(acc - self.acc_min)) + ((self.w_acc * acc) ** 2) / (1 + np.exp(self.acc_max - acc))
            if acc > 0 and acc <= self.acc_max:
                return self.w_acc * acc ** 2 + temp
            else:
                return self.w_deacc * acc ** 2 + temp

    def get_transfer_acc_cost_prev_speed(self, v2, s2, s3, dt):
        """
        状态转移时的加速度cost
        :return:
        """
        acc = ((s3 - s2) / dt - v2) / dt
        if acc > self.acc_max or acc < self.acc_min:
            return INF
        else:
            temp = ((self.w_deacc * acc) ** 2) / (1 + np.exp(acc - self.acc_min)) + ((self.w_acc * acc) ** 2) / (1 + np.exp(self.acc_max - acc))
            if acc > 0 and acc <= self.acc_max:
                return self.w_acc * acc ** 2 + temp
            else:
                return self.w_deacc * acc ** 2 + temp

    def get_transfer_jerk_cost_normal(self, s0, s1, s2, s3, dt):
        """
        状态转移时的加加速度cost
        :return:
        """
        jerk = (s3 - 3 * s2 + 3 * s1 - s0) / dt ** 3
        if jerk > self.jerk_max or jerk < self.jerk_min:
            return INF
        else:
            if jerk > 0 and jerk <= self.jerk_max:
                return self.w_jerk_pos * jerk ** 2 * dt
            else:
                return self.w_jerk_neg * jerk ** 2 * dt

    def get_transfer_jerk_cost_prev_acc_speed(self, acc2, v2, s2, s3, dt):
        """
        状态转移时的加加速度cost
        :return:
        """
        acc3 = ((s3 - s2) / dt - v2) / dt
        jerk = (acc3 - acc2) / dt
        if jerk > self.jerk_max or jerk < self.jerk_min:
            return INF
        else:
            if jerk > 0 and jerk <= self.jerk_max:
                return self.w_jerk_pos * jerk ** 2 * dt
            else:
                return self.w_jerk_neg * jerk ** 2 * dt

    def get_transfer_jerk_cost_prev_speed(self, v1, s1, s2, s3, dt):
        """
        状态转移时的加加速度cost
        :return:
        """
        v2 = (s2 - s1) / dt
        acc2 = (v2 - v1) / dt
        v3 = (s3 - s2) / dt
        acc3 = (v3 - v2) / dt
        jerk = (acc3 - acc2) / dt
        if jerk > self.jerk_max or jerk < self.jerk_min:
            return INF
        else:
            if jerk > 0 and jerk <= self.jerk_max:
                return self.w_jerk_pos * jerk ** 2 * dt
            else:
                return self.w_jerk_neg * jerk ** 2 * dt

    def solve(self):
        # 动态规划主程序
        total_obstacle_cost = self.get_obstacles_cost()
        total_spatial_cost = self.get_spatial_cost()
        dp_st_nodes = np.zeros([self.t_size, self.s_size], dtype=np.int32)
        dp_st_costs = np.ones([self.t_size, self.s_size]) * INF

        dp_st_nodes, dp_st_costs = self.solve_t0(dp_st_nodes, dp_st_costs)
        dp_st_nodes, dp_st_costs = self.solve_t1(total_obstacle_cost, total_spatial_cost, dp_st_nodes, dp_st_costs)
        dp_st_nodes, dp_st_costs = self.solve_t2(total_obstacle_cost, total_spatial_cost, dp_st_nodes, dp_st_costs)
        dp_st_nodes, dp_st_costs = self.solve_t3(total_obstacle_cost, total_spatial_cost, dp_st_nodes, dp_st_costs)
        return self.output_st(dp_st_nodes, dp_st_costs)

    def solve_t0(self, dp_st_nodes, dp_st_costs):
        dp_st_costs[0, 0] = 0
        dp_st_costs[0, 1:] = INF
        dp_st_nodes[0, :] = 0
        return dp_st_nodes, dp_st_costs

    def solve_t1(self, total_obstacle_cost, total_spatial_cost, dp_st_nodes, dp_st_costs):
        dt = self.speed_t[1] - self.speed_t[0]
        for i in range(self.s_size):
            s3 = self.speed_s[i]
            s2 = self.speed_s[0]
            cost = self.get_transfer_cost_t1(self.prev_acc, self.prev_speed, s2, s3, dt) + total_obstacle_cost[1, i] + total_spatial_cost[1, i]
            if cost + dp_st_costs[0, 0] < dp_st_costs[1, i]:
                dp_st_costs[1, i] = cost + dp_st_costs[0, 0]
                dp_st_nodes[1, i] = 0
        return dp_st_nodes, dp_st_costs

    def solve_t2(self, total_obstacle_cost, total_spatial_cost, dp_st_nodes, dp_st_costs):
        dt = self.speed_t[2] - self.speed_t[1]
        for i in range(self.s_size):  # j 为行循环
            cur_s_index = i
            cur_t_index = 2
            for j in range(i+1):  # 遍历前一列
                pre_t_index = 1
                prepre_t_index = 0
                pre_s_index = j
                prepre_s_index = 0

                s3 = self.speed_s[cur_s_index]
                s2 = self.speed_s[pre_s_index]
                s1 = self.speed_s[prepre_s_index]

                cost = self.get_transfer_cost_t2(self.prev_speed, s1, s2, s3, dt) + total_obstacle_cost[cur_t_index, cur_s_index] + total_spatial_cost[cur_t_index, cur_s_index]
                if cost + dp_st_costs[pre_t_index, pre_s_index] < dp_st_costs[cur_t_index, cur_s_index]:
                    dp_st_costs[cur_t_index, cur_s_index] = cost + dp_st_costs[pre_t_index, pre_s_index]
                    dp_st_nodes[cur_t_index, cur_s_index] = pre_s_index

        return dp_st_nodes, dp_st_costs

    def solve_t3(self, total_obstacle_cost, total_spatial_cost, dp_st_nodes, dp_st_costs):
        for i in range(3, self.t_size):  # i 为列循环
            dt = self.speed_t[i] - self.speed_t[i-1]
            for j in range(self.s_size):  # j 为行循环
                cur_s_index = j
                cur_t_index = i
                for k in range(j+1):  # 遍历前一列
                    pre_t_index = i - 1
                    prepre_t_index = i - 2
                    preprepre_t_index = i - 3
                    pre_s_index = k
                    prepre_s_index = dp_st_nodes[pre_t_index, pre_s_index]
                    preprepre_s_index = dp_st_nodes[prepre_t_index, prepre_s_index]
                    # 计算边的代价 其中起点为pre_row,pre_col 终点为cur_row cur_col
                    s3 = self.speed_s[cur_s_index]
                    s2 = self.speed_s[pre_s_index]
                    s1 = self.speed_s[prepre_s_index]
                    s0 = self.speed_s[preprepre_s_index]

                    cost = self.get_transfer_cost_t3(s0, s1, s2, s3, dt) + \
                           total_obstacle_cost[cur_t_index, cur_s_index] + \
                           total_spatial_cost[cur_t_index, cur_s_index]\

                    if cost + dp_st_costs[pre_t_index, pre_s_index] < dp_st_costs[cur_t_index, cur_s_index]:
                        dp_st_costs[cur_t_index, cur_s_index] = cost + dp_st_costs[pre_t_index, pre_s_index]
                        dp_st_nodes[cur_t_index, cur_s_index] = pre_s_index
        return dp_st_nodes, dp_st_costs

    def output_st(self, dp_st_nodes, dp_st_costs):
        """
        将动态规划结果进行回溯输出st结果
        :param dp_st_nodes: st node表
        :param dp_st_costs: st cost表
        :return:
        """
        # 输出初始化
        dp_speed_s = np.ones(self.t_size) * np.nan
        dp_speed_t = np.ones(self.t_size) * np.nan
        # 找到dp_node_cost 上边界和右边界代价最小的节点

        min_row_cost = np.min(dp_st_costs[-1])
        min_col_cost = np.min(dp_st_costs[:, -1])
        # 如果row_cost < col_cost说明，在t_range之内该车没有行驶到终点
        if min_row_cost <= min_col_cost:
            min_row = len(dp_st_costs) - 1
            min_col = np.argmin(dp_st_costs[min_row])
        # 否则，说明在t_range之内就已经到达终点
        else:
            min_col = len(dp_st_costs[0]) - 1
            min_row = np.argmin(dp_st_costs[:, min_col])

        min_cost = dp_st_costs[min_row, min_col]

        # 反向回溯
        while min_row != 0:
            pre_col = dp_st_nodes[min_row, min_col]
            pre_row = min_row - 1
            dp_speed_s[min_row] = self.speed_s[min_col]
            dp_speed_t[min_row] = self.speed_t[min_row]
            min_row = pre_row
            min_col = pre_col

        dp_speed_s[0], dp_speed_t[0] = 0, 0
        return dp_speed_s, dp_speed_t, min_cost


class SpeedDPOptimizerV2:
    def __init__(self, ego, obstacles,
                 speed_a, speed_t,
                 prev_speed, prev_acc, path_x, path_y, path_s):
        """
        速度规划中的动态规划
        :param ego:自驾车
        :param obstacles: 障碍物（动态or静态）
        :param prev_speed: t-1时刻的速度
        :param prev_acc: t-1时刻的加速度
        :param path_x: 路径规划 x list
        :param path_y: 路径规划 y list
        :param path_s: 路径规划 s list
        """
        self.ego = ego
        self.obstacles = obstacles
        self.prev_speed = prev_speed
        self.prev_acc = prev_acc
        self.a_size = len(speed_a)
        self.t_size = len(speed_t)
        self.state_s = np.zeros([self.t_size, self.a_size])
        self.speed_a = speed_a
        self.speed_t = speed_t
        self.path_x = path_x
        self.path_y = path_y
        self.path_s = path_s

        # self.s_safe_overtake = self.prev_speed # 超车距离的安全距离
        # self.s_safe_follow = self.prev_speed # 跟车距离的安全距离
        # self.s_safe_overtake = 3  # 超车距离的安全距离
        self.s_safe_overtake = max(self.prev_speed, 5)  # 超车距离的安全距离
        # self.s_safe_follow = 3  # 跟车距离的安全距离
        self.s_safe_follow = max(self.prev_speed, 5)  # 跟车距离的安全距离
        self.upper_speed_limit = 120 / 3.6 #最大车速限制
        self.lower_speed_limit = 3.6 / 3.6 #最大车速限制
        self.acc_min = -5.0 # 最小加速度限制
        self.acc_max = 5.0 # 最大加速度限制
        self.jerk_min = -30 # 最小加加速度限制
        self.jerk_max = 30 # 最大加加速度限制

        self.w_follow = 1e3 # 目标函数中跟车距离的权重
        self.w_overtake = 1e3 # 目标函数中超车距离的权重
        self.w_spatial = 1e3 # 目标函数中与终点距离的权重
        self.w_exceed_speed = 10 # 超速的权重
        self.w_lower_speed = 2.5 # 低于最低速度权重
        self.w_acc = 2.0 # 加速度权重
        self.w_deacc = 2.0 # 减速度权重
        self.w_jerk_pos = 1 # 加加速度权重
        self.w_jerk_neg = 300 # 负加加速度权重

        self.obs_st_boundaries = {} # 障碍物的st边界
        self.head_bounds = None
        self.tail_bounds = None
        self.overtake_bounds = None
        self.follow_bounds = None
    def get_st_boundaries(self):
        """
        获取障碍物的st边界
        :return:
        """
        all_head_bounds = []
        all_tail_bounds = []
        for obstacle in self.obstacles.values():
            tail_bounds, head_bounds = obstacle.get_precise_st_boundaries(self.ego, self.path_x, self.path_y, self.path_s, self.speed_t)
            self.obs_st_boundaries[obstacle.name] = (tail_bounds, head_bounds)
            all_head_bounds.append(head_bounds)
            all_tail_bounds.append(tail_bounds)
        all_head_bounds = np.array(all_head_bounds)
        all_tail_bounds = np.array(all_tail_bounds)

        self.head_bounds = all_head_bounds
        self.tail_bounds = all_tail_bounds
        self.overtake_bounds = all_head_bounds + self.s_safe_overtake
        self.follow_bounds = all_tail_bounds - self.s_safe_follow
        return self.obs_st_boundaries

    def get_obstacles_cost(self, s):
        """
        获取障碍物的碰撞或者超车，跟车产生的cost
        :return:
        """
        cost = 0
        for i in range(len(self.head_bounds)):
            for j in range(len(self.head_bounds[i])):
                # 当前距离>tail bound，<head bound 说明撞车了，cost为inf
                if s >= self.tail_bounds[i, j] and s <= self.head_bounds[i, j]:
                    cost += INF
                # 当前距离>overtake bound 说明超车无碰撞风险，cost为0
                elif s >= self.overtake_bounds[i, j]:
                    cost += 0
                # 当前距离<follow bound 说明跟车无碰撞风险，cost为0
                elif s < self.follow_bounds[i, j]:
                    cost += 0
                # 当前距离在head bound和overtake bound之间 说明有部分碰撞风险，cost为欧氏距离
                elif s < self.overtake_bounds[i, j] and s > self.head_bounds[i, j]:
                    cost += self.w_overtake * (self.overtake_bounds[i, j] - s) ** 2
                    # 当前距离在follow bound和follow bound之间 说明有部分碰撞风险，cost为欧氏距离
                elif s > self.follow_bounds[i, j] and s < self.tail_bounds[i, j]:
                    cost += self.w_follow * (self.follow_bounds[i, j] - s) ** 2
        return cost

    def get_spatial_cost(self, s):
        """
        与终点距离的cost
        :return:
        """
        return self.w_spatial * max(40 - s, 0)


    def get_transfer_cost_t2(self, acc2, v3, acc3, dt):
        """
        状态转移函数，t >= 2
        :param v1: t-1时刻速度
        :param s1: t-2时刻位置
        :param s2: t-1时刻位置
        :param s3: t时刻位置
        :return:
        """
        return self.get_transfer_speed_cost(v3) + self.get_transfer_acc_cost(acc3) + self.get_transfer_jerk_cost(acc2, acc3, dt)

    def get_transfer_cost_t1(self, acc2, v3, acc3, dt):
        """
        状态转移函数，t >= 1
        :param acc2: t-1时刻加速度
        :param v2: t-1时刻速度
        :param s2: t-1时刻位置
        :param s3: t时刻位置
        :return:
        """
        return self.get_transfer_speed_cost(v3) + self.get_transfer_acc_cost(acc3) + self.get_transfer_jerk_cost(acc2, acc3, dt)

    def get_transfer_speed_cost(self, v3):
        """
        状态转移时的速度cost
        :param s2:
        :param s3:
        :return:
        """
        upper_speed_det = (v3 - self.upper_speed_limit) / self.upper_speed_limit
        lower_speed_det = (v3 - self.lower_speed_limit) / self.lower_speed_limit
        if v3 < 0:
            return INF
        else:
            if upper_speed_det > 0:
                # 针对于告诉情况的限速，如果超速了，立刻减速到限速内
                return (upper_speed_det + 1) * self.w_exceed_speed
            if lower_speed_det < 0:
                return -lower_speed_det * self.w_lower_speed
            return 0

    def get_transfer_acc_cost(self, acc3):
        """
        状态转移时的加速度cost
        :return:
        """
        if acc3 > self.acc_max or acc3 < self.acc_min:
            return INF
        else:
            temp = ((self.w_deacc * acc3) ** 2) / (1 + np.exp(acc3 - self.acc_min)) + ((self.w_acc * acc3) ** 2) / (1 + np.exp(self.acc_max - acc3))
            if acc3 > 0 and acc3 <= self.acc_max:
                return self.w_acc * acc3 ** 2 + temp
            else:
                return self.w_deacc * acc3 ** 2 + temp

    def get_transfer_jerk_cost(self, acc2, acc3, dt):
        """
        状态转移时的加加速度cost
        :return:
        """
        jerk = (acc3 - acc2) / dt
        if jerk > self.jerk_max or jerk < self.jerk_min:
            return INF
        else:
            if jerk > 0 and jerk <= self.jerk_max:
                return self.w_jerk_pos * jerk ** 2 * dt
            else:
                return self.w_jerk_neg * jerk ** 2 * dt

    def solve(self):
        # 动态规划主程序
        dp_at_nodes = np.zeros([self.t_size, self.a_size], dtype=np.int32)
        dp_at_costs = np.ones([self.t_size, self.a_size]) * INF
        state_s = np.zeros([self.t_size, self.a_size])
        state_v = np.zeros([self.t_size, self.a_size])
        dp_at_nodes, dp_at_costs, state_s, state_v = self.solve_t0(dp_at_nodes, dp_at_costs, state_s, state_v)
        dp_at_nodes, dp_at_costs, state_s, state_v = self.solve_t1(dp_at_nodes, dp_at_costs, state_s, state_v)
        return self.output_st(dp_at_nodes, dp_at_costs)

    def solve_t0(self, dp_at_nodes, dp_at_costs, state_s, state_v):
        idx = np.argmin(np.abs(self.speed_a - self.prev_acc))
        dp_at_costs[0] = INF
        state_s[0] = 0
        state_v[0] = self.prev_speed
        dp_at_costs[0, idx] = 0
        dp_at_nodes[0, idx] = 0
        return dp_at_nodes, dp_at_costs, state_s, state_v

    def solve_t1(self, dp_at_nodes, dp_at_costs, state_s, state_v):
        for i in range(1, self.t_size):  # i 为列循环
            dt = self.speed_t[i] - self.speed_t[i-1]
            for j in range(self.a_size):  # j 为行循环
                cur_a_index = j
                cur_t_index = i
                for k in range(self.a_size):  # 遍历前一列
                    pre_a_index = k
                    pre_t_index = i - 1
                    speed_v = max(0, dt * self.speed_a[cur_a_index] + state_v[pre_t_index, pre_a_index])
                    speed_s = speed_v * dt + state_s[pre_t_index, pre_a_index]

                    cost = self.get_transfer_cost_t1(self.speed_a[pre_a_index], speed_v, self.speed_a[cur_a_index], dt) + \
                           self.get_obstacles_cost(speed_s) + \
                           self.get_spatial_cost(speed_s)\


                    if cost + dp_at_costs[pre_t_index, pre_a_index] < dp_at_costs[cur_t_index, cur_a_index]:
                        dp_at_costs[cur_t_index, cur_a_index] = cost + dp_at_costs[pre_t_index, pre_a_index]
                        dp_at_nodes[cur_t_index, cur_a_index] = pre_a_index
                        state_s[cur_t_index, cur_a_index] = speed_s
                        state_v[cur_t_index, cur_a_index] = speed_v
        return dp_at_nodes, dp_at_costs, state_s, state_v

    def output_st(self, dp_at_nodes, dp_at_costs):
        """
        将动态规划结果进行回溯输出st结果
        :param dp_st_nodes: st node表
        :param dp_st_costs: st cost表
        :return:
        """
        # 输出初始化
        dp_speed_a = np.ones(self.t_size) * np.nan
        dp_speed_t = np.ones(self.t_size) * np.nan
        # 找到dp_node_cost 上边界和右边界代价最小的节点

        min_row = len(dp_at_costs) - 1
        while np.min(dp_at_costs[min_row]) == INF:
            min_row -= 1

        min_col = np.argmin(dp_at_costs[min_row])
        min_cost = dp_at_costs[min_row, min_col]

        # 反向回溯
        while min_row != 0:
            pre_col = dp_at_nodes[min_row, min_col]
            pre_row = min_row - 1
            dp_speed_a[min_row] = self.speed_a[min_col]
            dp_speed_t[min_row] = self.speed_t[min_row]
            min_row = pre_row
            min_col = pre_col

        return dp_speed_a, dp_speed_t, min_cost


class SpeedQPOptimizer:
    def __init__(self, speed_t, speed_s,
                 prev_speed, prev_acc,
                 start_s, start_speed, start_acc,
                 obstacle_names, obstacle_decisions, obstacle_st_boundaries):
        self.speed_t = speed_t
        self.speed_s = speed_s
        self.cruise_speed = 30 / 3.6
        self.speed_limit = 120 / 3.6
        self.t_size = len(self.speed_t)

        self.prev_speed = prev_speed
        self.prev_acc = prev_acc
        self.start_s = start_s
        self.start_speed = start_speed
        self.start_acc = start_acc

        self.obstacle_names = obstacle_names
        self.obstacle_decisions = obstacle_decisions
        self.obstacle_st_boundaries = obstacle_st_boundaries

        self.acc_max = 5
        self.acc_min = -5
        self.jerk_max = 30
        self.jerk_min = -30
        self.w_acc = 10
        self.w_jerk = 10
        self.w_cruise = 1
        self.w_end_s = 20

        self.w_follow = 20  # 目标函数中跟车距离的权重
        self.w_overtake = 20  # 目标函数中超车距离的权重

        self.s_safe_overtake = max(self.prev_speed, 5)  # 超车距离的安全距离
        # self.s_safe_follow = 3  # 跟车距离的安全距离
        self.s_safe_follow = max(self.prev_speed, 5)  # 跟车距离的安全距离

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

        self.A = csc_matrix(self.A)
        self.P = csc_matrix(self.P)
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
        self.q[self.t_size * 1 - 1] += -2 * self.speed_s[-1] * self.w_end_s

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
                self.u[:self.t_size, 0] = np.clip(self.u[:self.t_size, 0], a_min=-9999, a_max=upper_bound)

            # 如果value是1，说明决策是超车决策，那么下界设为st_boundary的上界
            else:
                lower_bound = self.obstacle_st_boundaries[key][1]
                self.l[:self.t_size, 0] = np.clip(self.l[:self.t_size, 0], a_min=lower_bound, a_max=9999)

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
        self.l[begin_idx + 1, :] = self.start_speed
        self.l[begin_idx + 2, :] = self.start_acc

        self.u[begin_idx, :] = self.start_s
        self.u[begin_idx + 1, :] = self.start_speed
        self.u[begin_idx + 2, :] = self.start_acc

    def solve(self):
        """
        求解
        :return:
        """
        prob = osqp.OSQP()
        prob.setup(self.P, self.q, self.A, self.l, self.u)
        res = prob.solve()
        self.res = res.x
        return res, res.info.status


class SpeedNLPOptimizer:
    def __init__(self, speed_t, speed_s,
                 prev_aspeed, prev_acc,
                 start_s, start_speed, start_acc,
                 obstacle_names, obstacle_decisions, obstacle_st_boundaries):
        self.speed_t = speed_t
        self.speed_s = speed_s
        self.cruise_speed = 30 / 3.6
        self.speed_limit = 120 / 3.6
        self.t_size = len(self.speed_t)

        self.prev_speed = prev_speed
        self.prev_acc = prev_acc
        self.start_s = start_s
        self.start_speed = start_speed
        self.start_acc = start_acc

        self.obstacle_names = obstacle_names
        self.obstacle_decisions = obstacle_decisions
        self.obstacle_st_boundaries = obstacle_st_boundaries

        self.acc_max = 5
        self.acc_min = -5
        self.jerk_max = 30
        self.jerk_min = -30
        self.w_acc = 10
        self.w_jerk = 10
        self.w_cruise = 1
        self.w_end_s = 20

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
        # self.P[:self.t_size * 1 - 1, :self.t_size * 1 - 1] += 1 * np.eye(self.t_size - 1)
        self.P[self.t_size * 1 - 1, self.t_size * 1 - 1] += self.w_end_s
        self.q[self.t_size * 1 - 1] += -2 * self.speed_s[-1] * self.w_end_s

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
            if key == "car2":
                continue

            # 如果value是0，说明决策是跟车决策，那么上界设为st_boundary的下界
            if value == 0:
                upper_bound = self.obstacle_st_boundaries[key][0] - 3
                self.u[:self.t_size, 0] = np.clip(self.u[:self.t_size, 0], a_min=-9999, a_max=upper_bound)

            # 如果value是1，说明决策是超车决策，那么下界设为st_boundary的上界
            else:
                lower_bound = self.obstacle_st_boundaries[key][1] + 3
                self.l[:self.t_size, 0] = np.clip(self.l[:self.t_size, 0], a_min=lower_bound, a_max=9999)

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
        self.l[begin_idx + 1, :] = self.start_speed
        self.l[begin_idx + 2, :] = self.start_acc

        self.u[begin_idx, :] = self.start_s
        self.u[begin_idx + 1, :] = self.start_speed
        self.u[begin_idx + 2, :] = self.start_acc

    def func_wrapper(self):
        def func(x):
            x = np.expand_dims(x, 1)
            return 0.5 * (np.matmul(np.matmul(x.T, self.P), x) + np.matmul(self.q.T, x))[0, 0]
        return func

    def hess_wrapper(self):
        def func(x):
            return self.P
        return func

    def jac_wrapper(self):
        def func(x):
            return (np.matmul(x.T, self.P) + self.q.T).flatten()
        return func


    def solve(self):
        """
        求解
        :return:
        """
        linear_constraint = LinearConstraint(self.A, self.l[:, 0], self.u[:, 0])
        x0 = np.zeros([self.t_size * 3])

        # res = scipy.optimize.minimize(
        #     self.func_wrapper(), x0, method='trust-constr', hess=self.hess_wrapper(),
        #     constraints=[linear_constraint],
        #     options={'verbose': 1})
        #
        x = cp.Variable(shape=(self.t_size * 3))
        prob = cp.Problem(cp.Minimize((1 / 2) * cp.quad_form(x, self.P) + self.q[:, 0].T @ x),
                          [self.A @ x <= self.u[:, 0],
                           self.A @ x >= self.l[:, 0],])
        # print(cp.__version__)
        prob.solve(verbose=True, solver="")
        # print(prob)
        # temp_G = np.r_[self.A, -self.A]
        # temp_h = np.r_[self.u, -self.l]
        #
        #
        # res = solvers.qp(matrix(self.P) * 2, matrix(self.q[:, 0]), matrix(temp_G), matrix(temp_h[:, 0]))
        # solvers.gp()
        # self.res = res["x"]
        return res, "solved" if res["status"] == "optimal" else "infeasible"

