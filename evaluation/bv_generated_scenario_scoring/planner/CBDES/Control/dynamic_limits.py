import numpy as np


m = 1134
l_r = 1.56
l_f = 1.04
L = l_r + l_f
miu = 0.85
g = 9.8
k1 = -43160
k2 = -29210
K = 9.064169222036606e-05

def slid_angle_gain(
    v: float,
    ) -> float:
    return (l_r / L + m * l_f * v ** 2 / k2 / L ** 2) / (1 + K * v ** 2)

def yaw_rate_gain(
    v: float,
    ) -> float:
    return (v / L) / (1 + K * v ** 2)

def steer_dynamic_limit(
    v: float
    ) -> float:
    if v < 0.1 :
        return 0
    k_safe = 0.9
    # if v > 5:
    #     k_safe = 1 - 0.02 * v
    limit1 = np.arctan(0.02 * miu * g * L / l_r) * (1 + K * v ** 2) * k_safe
    
    gain_1  = slid_angle_gain(v)
    if np.abs(gain_1) < 0.05:
        limit2 = 1
    else:
        limit2 = np.arctan(0.02 * miu * g) / np.abs(gain_1) * k_safe

    gain_2 = yaw_rate_gain(v)
    limit3 = g * miu / v / gain_2 * k_safe

    limit4 = np.arctan(L * np.arctan(miu * g / v) / v) * k_safe
    return np.min([limit1, limit2, limit3, limit4, v * 0.1])

def delta_dynamic_limit(
    v: float,
    steer_ori: float,    
    ) -> float:
    if v < 0.1:
        return 2.0
    k_safe = 0.9
    # if v > 5:
    #     k_safe = 1 - 0.02 * v
    limit = (miu * g / v - v * np.abs(steer_ori) / L / (1 + K * v ** 2)) * L / l_r * k_safe
    return limit

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    v = np.arange(0.1, 30, 0.1)
    limit = []
    for v0 in v:
        limit.append(steer_dynamic_limit(v0))

    plt.figure()
    plt.plot(v, limit)
    plt.show()
    