from typing import Tuple, Optional, Callable, Dict, List
from .dynamic_limits import steer_dynamic_limit, delta_dynamic_limit
import numpy as np

K = 9.064169222036606e-05
L = 2.6
miu = 0.85
g = 9.8
l_r = 1.56

max_accel = 5.4
max_jerk= 36.0
max_delta = 1.35

def get_steer(
    v_lon: float,
    ego_x: float,
    ego_y: float,
    heading: float,
    preview_point_x: float,
    preview_point_y: float,
    L: float,
    dt: float,
    ) -> float:
    # print(f"ego x {ego_x}")
    # print(f"ego y {ego_y}")
    # print(f"ego v {v_lon}")
    # print(f"preview x {preview_point_x}")
    # print(f"preview y {preview_point_y}")
    diff_x = preview_point_x - ego_x
    diff_y = preview_point_y - ego_y
    x = np.abs(diff_x * np.cos(heading) + diff_y * np.sin(heading)) + L * 3 / 5 - v_lon * dt
    y = - diff_x * np.sin(heading) + diff_y * np.cos(heading)
    dis_square = x ** 2 + y ** 2

    if dis_square < L ** 2 * 1.5:
        return 0
    
    steer = 2 * L * y / dis_square / (2 - 1 / (1 + K * v_lon ** 2))
    return steer

def steer_limit(
    v_lon: float,
    steer_ori: float,
    dt: float,
    ) -> Tuple[float, float]:
    s_dynamic_limit = steer_dynamic_limit(v_lon) * 0.95
    d_dynamic_limit = delta_dynamic_limit(v_lon, steer_ori) * 0.95
    upper_limit = np.min([s_dynamic_limit, steer_ori + dt * np.min([max_delta, d_dynamic_limit])])
    lower_limit = np.max([- s_dynamic_limit, steer_ori - dt * np.min([max_delta, d_dynamic_limit])])
    return lower_limit, upper_limit

def get_accel(
    v_lon: float,
    preview_v: float,
    kp: float,
    ) -> float:
    if preview_v < v_lon:
        kp = kp + (2.0 - kp) * 0.6
    a_lon = kp * (preview_v - v_lon)
    return a_lon

def accel_limit(
    a_ori: float,
    v_lon: float,
    dt: float,
    ) -> float:
    upper_limit = np.min([a_ori + max_jerk * dt, max_accel])
    lower_limit = np.max([a_ori - max_jerk * dt, - max_accel, - v_lon / dt])
    if lower_limit > upper_limit:
        lower_limit = upper_limit 
    return lower_limit, upper_limit

def get_control(
    v_lon: float,
    ego_x: float,
    ego_y: float,
    heading: float,
    preview_point_x: float,
    preview_point_y: float,
    preview_v: float,
    a_ori: float,
    steer_ori: float,   
    length: float,
    dt: float,    
    ) -> Tuple[List[float], float, float, float, float]:
    L = length / 1.7
    steer = get_steer(v_lon, ego_x, ego_y, heading, preview_point_x, preview_point_y, L, dt)
    steer_lower_limit, steer_upper_limit = steer_limit(v_lon, steer_ori, dt)
    steer_output = np.clip(steer, steer_lower_limit, steer_upper_limit)
    
    if abs(steer) < 0.05:
        kp = 1.5
    elif abs(steer) > 0.15:
        kp = 0.5
    else:
        kp = 2.0 - abs(steer) * 10.0

    a_lon = get_accel(v_lon, preview_v, kp)
    a_lower_limit, a_upper_limit = accel_limit(a_ori, v_lon, dt)
    a_output = np.clip(a_lon, a_lower_limit, a_upper_limit)

    return [a_output, steer_output], a_lower_limit, a_upper_limit, steer_lower_limit, steer_upper_limit
