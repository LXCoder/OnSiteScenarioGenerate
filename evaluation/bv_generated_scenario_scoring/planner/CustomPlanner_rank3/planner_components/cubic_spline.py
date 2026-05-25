from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)

"""
import math
import numpy as np
import bisect

class CubicSpline2D:
    def __init__(self, x, y):
        x_arr = np.asarray(x, dtype=np.float64).reshape(-1)
        y_arr = np.asarray(y, dtype=np.float64).reshape(-1)

        if x_arr.size != y_arr.size:
            raise ValueError("x and y must have the same length")

        if x_arr.size == 0:
            x_arr = np.array([0.0, 1e-3], dtype=np.float64)
            y_arr = np.array([0.0, 0.0], dtype=np.float64)

        keep = [0]
        for i in range(1, x_arr.size):
            if math.hypot(x_arr[i] - x_arr[keep[-1]], y_arr[i] - y_arr[keep[-1]]) > 1e-6:
                keep.append(i)

        x_clean = x_arr[keep]
        y_clean = y_arr[keep]

        if x_clean.size == 1:
            x_clean = np.array([x_clean[0], x_clean[0] + 1e-3], dtype=np.float64)
            y_clean = np.array([y_clean[0], y_clean[0]], dtype=np.float64)

        self.origin_s = self.__calc_s(x_clean, y_clean)
        self.sx = CubicSpline(self.origin_s, x_clean)
        self.sy = CubicSpline(self.origin_s, y_clean)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx(s)
        y = self.sy(s)

        return x, y

    def calc_curvature(self, s):
        dx = self.sx(s, 1)
        ddx = self.sx(s, 2)
        dy = self.sy(s, 1)
        ddy = self.sy(s, 2)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2) + 1e-9)
        return k

    def calc_dcurvature(self, s):
        dx = self.sx(s, 1)
        ddx = self.sx(s, 2)
        dddx = self.sx(s, 3)
        dy = self.sy(s, 1)
        ddy = self.sy(s, 2)
        dddy = self.sy(s, 3)
        a = dx * ddy - dy * ddx
        b = dx * dddy - dy * dddx
        c = dx * ddx + dy * ddy
        d = dx * dx + dy * dy
        return (b * d - 3.0 * a * c) / (d * d * d)

    def calc_theta(self, s):
        dx = self.sx(s, 1)
        dy = self.sy(s, 1)
        yaw = np.arctan2(dy, dx)
        return yaw
