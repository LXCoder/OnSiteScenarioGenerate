"""
LEGACY 模块说明：
- 本文件保留用于历史 evaluation/main.py 链路；
- 2026 replay 主流程不再调用此文件，而使用 `utils/metrics/cal_dynamics.py`；
- 新功能请优先维护 `cal_dynamics.py`，避免双轨规则漂移。
"""

import numpy as np
import os
import pandas as pd


def dynamic_check(
        t,  # 时间序列 单位：s
        deltaf,  # 前轮转角序列 单位：rad
        vx,  # 纵向车速序列 单位：m/s
        ax,  # 纵向加速度序列 单位：m/s^2
        vx_lim=None,  # 最大车速限制 单位：km/h
        ax_lim=None,  # 最大加速度限制 单位：g
        jx_lim=None,  # 最大加加速度限制 单位：g/s
        deltaf_lim=None,  # 最大前轮转角限制 单位：°
        omegaf_lim=None,  # 最大前轮转速限制 单位：°/s
        k1=None,  # 质心侧偏角安全系数
        k2=None,  # 横摆角速度安全系数
        k3=None  # 侧倾角安全系数
):
    # 删除第一行 初始状态
    t = t[1:]
    deltaf = deltaf[1:]
    vx = vx[1:]
    ax = ax[1:]

    # =========== 执行器动力学限制 缺省参数 =================
    if vx_lim is None:
        vx_lim = 200 / 3.6  # km/h to m/s
        # 缺省最高限速120km/h
        # +-+- 放宽约束 +-+-
    if ax_lim is None:
        ax_lim = 1 * 9.8  # g to m/s^2
        # 缺省最大加速度1g=9.8 m/s^2
    if jx_lim is None:
        jx_lim = 5 * 9.8  # g/s to m/s^3
        # 缺省最大加加速度（冲击度），相当于从0到最大作用力需要 0.2s（5=1/0.2）
        # +-+- 放宽约束 +-+- 电机扭矩响应可以到20ms
    if deltaf_lim is None:
        deltaf_lim = 40 * np.pi / 180  # deg to rad
        # 最大前轮转角40度
    if omegaf_lim is None:
        omegaf_lim = 80 * np.pi / 180  # deg/s to rad/s
        # 最大前轮转动速度，相当于方向盘从中间位置转动到极限位置用时 0.5s （80=40/0.5）
        # 美国ESV转向阶跃实验方向盘转速不小于 500 deg/s （GB/T6323.2中 200 deg/s） 乘用车转向角传动比17~25，即前轮转角20~29 deg/s
        # +-+- 放宽约束 +-+-

    # =========== 整车动力学限制 缺省参数 =================
    if k1 is None:
        k1 = 1  # 质心侧偏角 安全系数
        # 安全系数越大，越保守，动力学校核越不容易通过
    if k2 is None:
        k2 = 1  # 横摆角速度 安全系数
        # 安全系数越大，越保守，动力学校核越不容易通过
    if k3 is None:
        k3 = 1  # 侧倾角 安全系数
        # 安全系数越大，越保守，动力学校核越不容易通过

    # =============== 输出参数 ==================
    # 校核通过指示
    # flag = 1 % 通过动力学校验
    # flag = 0 % 不通过动力学校验
    flag = 1

    # 不通过原因
    reasons = ''

    # ================== 数据预处理 =====================
    jx = np.diff(ax) / np.diff(t)
    omegaf = np.diff(deltaf) / np.diff(t)

    # =================执行器动力学======================
    if np.sum(np.abs(vx) > vx_lim) > 0:
        flag = 0
        #reasons += '车速超过限制；'
        reasons += 'Exceeding speed limit.'
    # if np.sum(np.abs(ax) > ax_lim) > 0:
    #     flag = 0
    #     #reasons += '加速度超过驱动系统极限；'
    #     reasons += 'Exceeding Acceleration limit.'
    # if np.sum(np.abs(jx) > jx_lim) > 0:  # len(jx) * 0.05+-+-允许5%的点可以突破+-+-
    #     flag = 0
    #     #reasons += '加速度响应速度超过驱动系统极限；'  # +-+- 超过+-+-
    #     reasons += 'Exceeding acceleration-response-time limit.'
    # if np.sum(np.abs(deltaf) > deltaf_lim) > 0:
    #     flag = 0
    #     #reasons += '前轮转角超过限制；'
    #     reasons += 'Exceeding front-wheel-steering-angle limit.'
    # if np.sum(omegaf > omegaf_lim) > 0:
    #     flag = 0
    #     #reasons += '转向系统响应速度超过极限；'
    #     reasons += 'Exceeding steering-system-response-time limit.'

    # =====================整车动力学校核==============
    # 动力学计算步长 (s) 步长越短计算越准确，计算速度越慢 +-+- 0.01 +-+-
    sample_time = 0.01

    # 初值
    d_vy1 = 0
    d_d_phi1 = 0
    v0 = vx[0]
    Rw = 0.287 / 1.01
    s0 = 0.01
    omega0 = v0 / ((1 - s0) * Rw)
    x = np.array([0, 0, 0, 0, omega0, omega0, omega0, omega0])

    # 变量大小定义
    Tend = t[-1]
    n = round(Tend / sample_time)
    dx = np.zeros_like(x)
    u = np.zeros(3)
    ys = np.zeros((n, 4))

    # +-+- 防止车速为0时的动力学计算错误 +-+-
    vx=np.array(vx)
    vx = np.where(vx <= 0.005, 0.005, vx)


    # ------------ 迭代计算整车动力学模型 ----------------
    for i in range(n):
        # 插值计算输入
        u[0] = np.interp(sample_time * (i - 1), t, deltaf)
        u[1] = np.interp(sample_time * (i - 1), t, ax)
        u[2] = np.interp(sample_time * (i - 1), t, vx)

        # 计算微分
        dx = Devs(x, u, d_vy1, d_d_phi1)

        # 更新中间量
        d_vy1 = dx[0]
        d_d_phi1 = dx[2]

        # 更新状态
        x = x + dx * sample_time

        # 计算输出
        beta = np.arctan(x[0] / u[2])
        y = [beta, x[1], x[3],u[2]]

        ys[i, :] = y

    # 质心侧偏角
    miu = 0.85
    g = 9.8
    beta_lim = np.arctan(0.02 * miu * g / k1)   # +-+-

    # 横摆角速度
    miu_y = 0.85
    vx1 = np.interp(sample_time * np.arange(1, n + 1), t, vx)
    r_lim = g * miu_y / (vx1 * k2)

    # 侧倾角
    phi_lim = 6 / 180 * np.pi / k3

    # +-+-车速低于0.1 m/s 时计算的质心侧偏角不可靠+-+-
    betas = ys[:, 0]
    vs = ys[:, 3]
    betas[vs <= 0.5] = 0

    if np.sum(np.abs(ys[:, 0]) > beta_lim) > 0:
        flag = 0
        #reasons += '质心侧偏角超过限制；'
        reasons += 'Exceeding sideslip-angle limit.'
    if np.sum(np.abs(ys[:, 1]) > r_lim) > 0:
        flag = 0
        #reasons += '横摆角速度超过限制；'
        reasons += 'Exceeding yaw-rate limit.'

    if np.sum(np.abs(ys[:, 2]) > phi_lim) > 0:
        flag = 0
        #reasons += '侧倾角超过限制；'
        reasons += 'Exceeding roll-angle limit.'

    if flag == 1:
        #reasons += '动力学校核通过！'
        reasons += 'Dynamics verification passed!'
    else:
        reasons = reasons

    # 动力学校核结果/理由输出（不需要输出可注释）
    #print(reasons)

    return flag, reasons


def Devs(x, u, d_vy1, d_d_phi1):
    # ==============整车参数================

    # B级车参数
    m = 1134  # 整车质量
    mu = 71.4 + 54.5  # 非簧载质量
    ms = m - mu  # 簧载质量

    Iz = 1343.1  # 车辆横摆转动惯量
    Ix = 440.6  # 车辆侧倾转动惯量
    Ixz = 0  # 车辆绕xz轴的转动惯量积
    lf = 1.04  # 车辆质心到前轴的距离
    L = 2.6  # 轴距
    lr = L - lf  # 车辆质心到后轴的距离
    B = 1.48  # 轮距
    hcg = 0.54  # 车辆质心离地高度
    h = 0.23  # 簧载质量质心到侧倾轴垂直距离

    kf = 4.316e4 * 2  # 前轮侧偏刚度 N/rad
    kr = 2.921e4 * 2  # 后轮侧偏刚度
    Cf = 8.952e4 * 2  # 前轮纵向刚度 N
    Cr = 6.074e4 * 2  # 后轮纵向刚度

    kphif = 384 * 180 / np.pi  # 车辆前悬架侧倾角刚度
    kphir = 251 * 180 / np.pi  # 车辆前后架侧倾角刚度
    bphif = 2000  # 车辆前悬架侧倾角阻尼
    bphir = 2000  # 车辆前后架侧倾角阻尼

    f = 0.008  # 车轮滚动阻力系数

    CD = 0.3  # 空气阻力系数
    ru = 1.206  # 空气密度
    A = 1.6  # 汽车正面迎风面积

    Iw = 0.8  # 车轮转动惯量
    Rw = 0.287  # 车轮滚动半径

    lfs = lf  # 簧载质量质心到前轴距离
    lrs = lr  # 簧载质量质心到后轴距离

    hrf = 0.165  # 前轴侧倾中心离地距离
    hrr = 0.165  # 后轴侧倾中心离地距离
    muf = 71.4  # 非簧载质量在前轴分配值
    mur = 54.5  # 非簧载质量在后轴分配值

    huf = Rw  # 前轴非簧载质量中心离地高度
    hur = Rw  # 后轴非簧载质量中心离地高度

    g = 9.8  # 重力加速度

    # ==============状态给定==================
    vy = x[0]
    r = x[1]
    d_phi = x[2]
    phi = x[3]
    Wfl = x[4]
    Wfr = x[5]
    Wrl = x[6]
    Wrr = x[7]

    deltaf_out = u[0]
    ax_out = u[1]
    vx_out = u[2]
    vx = vx_out

    # 不变参数按照常数定义，减少无用输入
    deltarl = 0
    deltarr = 0
    Tbfl = 0
    Tbfr = 0
    Tbrl = 0
    Tbrr = 0
    miu = 0.85  # 路面附着系数

    # 转角换算
    deltafl = np.arctan(np.tan(deltaf_out) / (1 - B / L / 2 * np.tan(deltaf_out)))
    deltafr = np.arctan(np.tan(deltaf_out) / (1 + B / L / 2 * np.tan(deltaf_out)))

    # 转矩换算
    Fx = m * ax_out + f * m * g + CD * A * ru * vx_out ** 2 / 2
    Tall = Fx * Rw
    Delta_T = Tall / 2 * (1 - np.cos(deltaf_out))
    Tdfl = (Tall + Delta_T) / 4
    Tdfr = Tdfl
    Tdrl = Tdfl
    Tdrr = Tdfl

    # ==============车轮纵向速度==================
    ufl = (vx - 0.5 * B * r) * np.cos(deltafl) + (vy + lf * r) * np.sin(deltafl)
    ufr = (vx + 0.5 * B * r) * np.cos(deltafr) + (vy + lf * r) * np.sin(deltafr)
    url = (vx - 0.5 * B * r) * np.cos(deltarl) + (vy - lr * r) * np.sin(deltarl)
    urr = (vx + 0.5 * B * r) * np.cos(deltarr) + (vy - lr * r) * np.sin(deltarr)
    # ==============车轮侧偏角==================
    alfafl = deltafl - np.arctan((vy + lf * r) / (vx - 0.5 * B * r))
    alfafr = deltafr - np.arctan((vy + lf * r) / (vx + 0.5 * B * r))
    alfarl = deltarl - np.arctan((vy - lr * r) / (vx - 0.5 * B * r))
    alfarr = deltarr - np.arctan((vy - lr * r) / (vx + 0.5 * B * r))
    # ==============车轮滑移率==================
    if ufl < Rw * Wfl:
        sfl = 1 - ufl / (Rw * Wfl)
    else:
        sfl = (Rw * Wfl) / ufl - 1

    if ufr < Rw * Wfr:
        sfr = 1 - ufr / (Rw * Wfr)
    else:
        sfr = (Rw * Wfr) / ufr - 1

    if url < Rw * Wrl:
        srl = 1 - url / (Rw * Wrl)
    else:
        srl = (Rw * Wrl) / url - 1

    if urr < Rw * Wrr:
        srr = 1 - urr / (Rw * Wrr)
    else:
        srr = (Rw * Wrr) / urr - 1
    # ==============中间变量==================
    ax = ax_out - vy * r
    ay = d_vy1 + vx * r
    # ==============车轮垂直载荷==================
    Fzfl = m * g * lr / 2 / L - m * ax * hcg / 2 / L - ay / B * (ms * hrf * lrs / L + muf * huf) - 1 / B * (
            kphif * phi + bphif * d_phi)
    Fzfr = m * g * lr / 2 / L - m * ax * hcg / 2 / L + ay / B * (ms * hrf * lrs / L + muf * huf) + 1 / B * (
            kphif * phi + bphif * d_phi)
    Fzrl = m * g * lf / 2 / L + m * ax * hcg / 2 / L - ay / B * (ms * hrr * lfs / L + mur * hur) - 1 / B * (
            kphir * phi + bphir * d_phi)
    Fzrr = m * g * lf / 2 / L + m * ax * hcg / 2 / L + ay / B * (ms * hrr * lfs / L + mur * hur) + 1 / B * (
            kphir * phi + bphir * d_phi)
    # ==============车轮纵向力侧向力(Dugoff轮胎模型)==================
    lambdafl = miu * Fzfl * (1 - sfl) / 2 / np.sqrt(Cf ** 2 * sfl ** 2 + kf ** 2 * np.tan(alfafl) ** 2)
    if lambdafl <= 1:
        flambdafl = (2 - lambdafl) * lambdafl
    else:
        flambdafl = 1
    Fxfl = Cf * sfl * flambdafl / (1 - sfl)
    Fyfl = kf * np.tan(alfafl) * flambdafl / (1 - sfl)

    lambdafr = miu * Fzfr * (1 - sfr) / 2 / np.sqrt(Cf ** 2 * sfr ** 2 + kf ** 2 * np.tan(alfafr) ** 2)
    if lambdafr <= 1:
        flambdafr = (2 - lambdafr) * lambdafr
    else:
        flambdafr = 1
    Fxfr = Cf * sfr * flambdafr / (1 - sfr)
    Fyfr = kf * np.tan(alfafr) * flambdafr / (1 - sfr)

    lambdarl = miu * Fzrl * (1 - srl) / 2 / np.sqrt(Cr ** 2 * srl ** 2 + kr ** 2 * np.tan(alfarl) ** 2)
    if lambdarl <= 1:
        flambdarl = (2 - lambdarl) * lambdarl
    else:
        flambdarl = 1
    Fxrl = Cr * srl * flambdarl / (1 - srl)
    Fyrl = kr * np.tan(alfarl) * flambdarl / (1 - srl)

    lambdarr = miu * Fzrr * (1 - srr) / 2 / np.sqrt(Cr ** 2 * srr ** 2 + kr ** 2 * np.tan(alfarr) ** 2)
    if lambdarr <= 1:
        flambdarr = (2 - lambdarr) * lambdarr
    else:
        flambdarr = 1
    Fxrr = Cr * srr * flambdarr / (1 - srr)
    Fyrr = kr * np.tan(alfarr) * flambdarr / (1 - srr)

    # ==============车辆八自由度运动微分方程==================
    d_vy = ms / m * h * d_d_phi1 - vx * r + 1 / m * (
            Fxfl * np.sin(deltafl) + Fxfr * np.sin(deltafr) + Fxrl * np.sin(deltarl) + Fxrr * np.sin(
        deltarr) + Fyfl * np.cos(deltafl) + Fyfr * np.cos(deltafr) + Fyrl * np.cos(deltarl) + Fyrr * np.cos(
        deltarr))
    d_r = 1 / Iz * Ixz * d_d_phi1 + 1 / Iz * (
            lf * (Fyfl * np.cos(deltafl) + Fyfr * np.cos(deltafr) + Fxfl * np.sin(deltafl) + Fxfr * np.sin(
        deltafr)) - lr * (Fyrl * np.cos(deltarl) + Fyrr * np.cos(deltarr) + Fxrl * np.sin(deltarl) + Fxrr * np.sin(
        deltarr)) + B / 2 * (Fyfl * np.sin(deltafl) + Fyrl * np.sin(deltarl) - Fyfr * np.sin(deltafr) - Fyrr * np.sin(
        deltarr)) + B / 2 * (-Fxfl * np.cos(deltafl) + Fxfr * np.cos(deltafr) - Fxrl * np.cos(deltarl) + Fxrr * np.cos(
        deltarr)))
    d_d_phi = 1 / Ix * (
            ms * g * h * phi - (bphif + bphir) * d_phi - (kphif + kphir) * phi + ms * h * (
            d_vy1 + vx * r) + Ixz * d_r)
    d_Wfl = 1 / Iw * (Tdfl - Fxfl * Rw - Tbfl)
    d_Wfr = 1 / Iw * (Tdfr - Fxfr * Rw - Tbfr)
    d_Wrl = 1 / Iw * (Tdrl - Fxrl * Rw - Tbrl)
    d_Wrr = 1 / Iw * (Tdrr - Fxrr * Rw - Tbrr)

    # ==============更新状态==================
    dx = np.array([d_vy, d_r, d_d_phi, d_phi, d_Wfl, d_Wfr, d_Wrl, d_Wrr])

    return dx

# filedir=r"C:\Users\王思涵\Desktop\新建文件夹"
# file_namelist=os.listdir(filedir)
# for filename in file_namelist:
#     csvfile_path=os.path.join(filedir+'/'+filename)
#     print(csvfile_path)
#     output = pd.read_csv(csvfile_path)
#     t = output.iloc[:, 0].tolist()
#     deltaf = output.iloc[:, 8].tolist()
#     vx = output.iloc[:, 5].tolist()
#     ax = output.iloc[:, 6].tolist()
#     FunctionReturns = dynamic_check(t, deltaf, vx, ax)
#     print(FunctionReturns)


