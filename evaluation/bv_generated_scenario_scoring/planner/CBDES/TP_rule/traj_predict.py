from .CTRV import predict_trajectory,estimate_initial_omega
from .utils.TP_rule_utils import History_rule
import numpy as np


def predict_rule(history:History_rule):
    # 提取历史数据
    # print("rule!!!!!!!!!!!!")
    future_traj = {}
    T = 3.0
    dt = 0.1
    for id, traj in history.feature.items():
        x0 = traj[-1, 0]
        y0 = traj[-1, 1]
        v0 = traj[-1, 2]
        yaw0 = traj[-1, 3]

        # 根据历史航向角计算初始角速度
        yaw_angles = traj[:, 3]
        omega = estimate_initial_omega(yaw_angles, dt)
        # print(omega)
        # 预测轨迹
        trajectory = predict_trajectory(x0, y0, v0, yaw0, omega, dt, T)
        trajectory = np.tile(trajectory, (6, 1, 1))
        prob = np.eye(6)[:, 0]
        future_traj[id] = {}
        future_traj[id]["future"] = trajectory
        future_traj[id]["prob"] = prob
    return future_traj
    