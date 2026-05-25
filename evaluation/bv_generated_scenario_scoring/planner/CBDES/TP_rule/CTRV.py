import numpy as np
# import matplotlib.pyplot as plt


def compute_yaw_rate(yaw_angles, delta_t):
    yaw_rates = np.diff(yaw_angles) / delta_t
    return yaw_rates

def estimate_initial_omega(yaw_angles, delta_t):
    # 展开角度，消除跳变
    unwrapped_yaw_angles = np.unwrap(yaw_angles)
    yaw_rates = compute_yaw_rate(unwrapped_yaw_angles, delta_t)
    return np.mean(yaw_rates)

def predict_trajectory(x0, y0, v0, psi0, omega, delta_t, T):
    num_points = int(T / delta_t)
    trajectory = np.zeros((num_points, 3))  # 每一行存储 (x, y, psi)
    trajectory[0] = [x0, y0, psi0]
    
    for i in range(1, num_points):
        x_prev, y_prev, psi_prev = trajectory[i-1]
        
        if omega != 0:
            x_new = x_prev + (v0 / omega) * (np.sin(psi_prev + omega * delta_t) - np.sin(psi_prev))
            y_new = y_prev + (v0 / omega) * (np.cos(psi_prev) - np.cos(psi_prev + omega * delta_t))
        else:
            x_new = x_prev + v0 * delta_t * np.cos(psi_prev)
            y_new = y_prev + v0 * delta_t * np.sin(psi_prev)
        
        psi_new = psi_prev + omega * delta_t
        
        trajectory[i] = [x_new, y_new, psi_new]
    
    return trajectory[:,:2]

# # 历史数据: 每行包含 [x, y, v, psi]
# history = np.array([
#     [0.0, 0.0, 1.0, 0.0],
#     [1.0, 0.1, 1.0, 0.01],
#     [2.0, 0, 1.0, 6.27],  # 15π/8 相当于 1π/8, 超过2π
#     [3.0, -0.1, 1.0, 6.27],  # 23π/8 相当于 3π/8, 超过2π
#     [4.0, 0, 1.0, 0.01]  # 2π
# ])

# # 提取历史数据
# x0 = history[-1, 0]
# y0 = history[-1, 1]
# v0 = history[-1, 2]
# psi0 = history[-1, 3]
# delta_t = 0.1  # s
# T = 10.0  # s

# # 根据历史航向角计算初始角速度
# yaw_angles = history[:, 3]
# omega = estimate_initial_omega(yaw_angles, delta_t)
# print(omega)

# # 预测轨迹
# trajectory = predict_trajectory(x0, y0, v0, psi0, omega, delta_t, T)

# # 绘制历史轨迹和预测轨迹
# plt.figure()
# plt.plot(history[:, 0], history[:, 1], 'bo-', label='History')
# plt.plot(trajectory[:, 0], trajectory[:, 1], 'ro-', label='Predicted trajectory')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.legend()
# plt.axis('equal')
# plt.title('CTRV Model Trajectory Prediction')
# plt.grid(True)
# plt.show()
