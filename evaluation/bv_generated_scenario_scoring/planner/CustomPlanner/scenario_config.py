
class BaseConfig():

    def __init__(self):
        # 路径规划和速度规划的初始参数
        self.path_s_start = 0
        self.path_s_dense_end = 5
        self.path_s_sparse_end = 30
        self.speed_t_start = 0
        self.speed_t_dense_end = 2
        self.speed_t_sparse_end = 5
        self.speed_s_start = 0
        self.speed_s_end = 40

        self.delta_path_s_dense = 0.1
        self.delta_path_s_sparse = 0.5
        self.delta_speed_t_dense = 0.1
        self.delta_speed_t_sparse = 0.5

        # 优化暖启动的时间
        self.warm_time = 4

        # 路径规划相关参数
        self.max_rot = 0.7
        self.axis_distance = 3
        self.path_w_l = 200
        self.path_w_dl = 10000
        self.path_w_ddl = 3000
        self.path_w_dddl = 150
        self.path_w_end_l = 40
        self.path_w_end_dl = 40
        self.path_w_end_ddl = 40

        # 速度规划相关参数
        self.cruise_speed = 30 / 3.6
        self.speed_max = 140 / 3.6
        self.acc_max = 5
        self.acc_min = -5
        self.jerk_max = 30
        self.jerk_min = -30
        self.min_overtake = 5
        self.min_follow = 5
        self.speed_w_acc = 10
        self.speed_w_jerk = 10
        self.speed_w_cruise = 1
        self.speed_w_end_s = 20
        self.speed_w_follow = 20  # 目标函数中跟车距离的权重
        self.speed_w_overtake = 20  # 目标函数中超车距离的权重



class LocalRoadConfig(BaseConfig):
    def __init__(self):
        super().__init__()
        pass


class HighwayConfig(BaseConfig):
    def __init__(self):
        super().__init__()
        self.delta_path_s_dense = 0.5
        self.delta_path_s_sparse = 1
        self.delta_speed_t_dense = 0.1
        self.delta_speed_t_sparse = 0.5
        self.path_s_dense_end = 20
        self.path_s_sparse_end = 100

        self.cruise_speed = 80 / 3.6
        self.speed_s_end = 200

        self.acc_max = 5
        self.acc_min = -7

        self.speed_w_overtake = 10  # 目标函数中超车距离的权重