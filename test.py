# -*- coding: utf-8 -*-
"""
Created on Tue May 13 21:28:01 2025

@author: 陈鹏飞
"""

import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sin, cos, sqrt

# 基于Abkowitz模型的船舶旋回运动仿真
# 船舶参数为Mariner型货船，尺度比为1:100

def main():
    # 船舶参数设置
    L = 160.93      # 船长(m)
    B = 23.17       # 船宽(m)
    d = 8.23        # 吃水(m)
    Cb = 0.559      # 方形系数
    m = 18442.8     # 船舶质量(t)
    xG = -2.11      # 重心纵向位置(m)
    Iz = 107.88e4   # 绕Z轴转动惯量(t·m^2)

    # 无量纲化参数
    rho = 1.025     # 海水密度(t/m^3)
    U0 = 7.72       # 初始速度(m/s)
    L_pp = L        # 垂线间长(m)
    mass = m        # 质量(t)
    Volume = m/rho  # 排水体积(m^3)

    # 无量纲化系数
    m_prime = mass / (0.5*rho*L_pp**3)
    Iz_prime = Iz / (0.5*rho*L_pp**5)
    xG_prime = xG / L_pp

    # Abkowitz模型水动力导数
    # 纵向力系数
    X0 = -0.0075
    Xvv = 0.0030
    Xvr = -0.0002
    Xrr = 0.0003
    Xvvvv = 0.0001

    # 横向力系数
    Yv = -0.0400
    Yr = 0.0022
    Yvvv = 0.0010
    Yvvr = -0.0028
    Yvrr = 0.0011
    Yrrr = -0.0008

    # 转首力矩系数
    Nv = -0.0030
    Nr = -0.0022
    Nvvv = -0.0028
    Nvvr = 0.0015
    Nvrr = -0.0004
    Nrrr = 0.0008

    # 仿真参数设置
    t_final = 500    # 仿真总时间(s)
    dt = 0.1         # 时间步长(s)
    t = np.arange(0, t_final+dt, dt)  # 时间向量
    N = len(t)       # 仿真步数

    # 初始条件
    u = np.ones(N) * U0   # 纵向速度(m/s)
    v = np.zeros(N)       # 横向速度(m/s)
    r = np.zeros(N)       # 转首角速度(rad/s)
    psi = np.zeros(N)     # 艏向角(rad)
    x = np.zeros(N)       # 纵向位置(m)
    y = np.zeros(N)       # 横向位置(m)

    # 舵角设置(阶跃输入)
    delta = np.zeros(N)
    delta[50:] = np.radians(35)  # 35度舵角

    # 主仿真循环
    for k in range(N-1):
        # 当前状态
        U = sqrt(u[k]**2 + v[k]**2)  # 合速度
        beta = atan2(-v[k], u[k])     # 漂角
        
        # 无量纲化速度
        v_prime = v[k]/U if U != 0 else 0
        r_prime = r[k]*L_pp/U if U != 0 else 0
        
        # 计算水动力和力矩(无量纲)
        X = X0 + Xvv*v_prime**2 + Xvr*v_prime*r_prime + Xrr*r_prime**2 + Xvvvv*v_prime**4
        Y = (Yv*v_prime + Yr*r_prime + Yvvv*v_prime**3 + Yvvr*v_prime**2*r_prime + 
             Yvrr*v_prime*r_prime**2 + Yrrr*r_prime**3)
        N = (Nv*v_prime + Nr*r_prime + Nvvv*v_prime**3 + Nvvr*v_prime**2*r_prime + 
             Nvrr*v_prime*r_prime**2 + Nrrr*r_prime**3)
        
        # 转换为有量纲值
        X = 0.5*rho*L_pp**2*U**2 * X
        Y = 0.5*rho*L_pp**2*U**2 * Y
        N = 0.5*rho*L_pp**3*U**2 * N
        
        # 加入舵力
        delta_R = delta[k]  # 当前舵角
        Fx_R = 0
        Fy_R = 0.5*rho*L_pp*d*U**2 * (0.0032*delta_R + 0.00034*abs(v_prime)*delta_R - 0.00084*r_prime*delta_R)
        Mz_R = 0.5*rho*L_pp**2*d*U**2 * (-0.0011*delta_R - 0.000071*abs(v_prime)*delta_R + 0.00021*r_prime*delta_R)
        
        # 总外力
        X_total = X + Fx_R
        Y_total = Y + Fy_R
        N_total = N + Mz_R
        
        # 运动方程求解(欧拉法)
        u[k+1] = u[k] + dt*(X_total/mass + v[k]*r[k])
        v[k+1] = v[k] + dt*(Y_total/mass - u[k]*r[k])
        r[k+1] = r[k] + dt*(N_total/Iz - xG*(Y_total/mass - u[k]*r[k]))/Iz
        
        # 位置和艏向更新
        psi[k+1] = psi[k] + dt*r[k]
        x[k+1] = x[k] + dt*(u[k]*cos(psi[k]) - v[k]*sin(psi[k]))
        y[k+1] = y[k] + dt*(u[k]*sin(psi[k]) + v[k]*cos(psi[k]))

    # 结果可视化
    plt.figure(figsize=(12, 8))
    
    # 舵角输入
    plt.subplot(3, 1, 1)
    plt.plot(t, np.degrees(delta), 'b', linewidth=1.5)
    plt.ylabel('Rudder Angle (°)')
    plt.title('Rudder Input')
    plt.grid(True)

    # 船舶艏向角变化
    plt.subplot(3, 1, 2)
    plt.plot(t, np.degrees(psi), 'r', linewidth=1.5)
    plt.ylabel('Heading Angle (°)')
    plt.title('Ship Heading Angle')
    plt.grid(True)

    # 船舶转首角速度
    plt.subplot(3, 1, 3)
    plt.plot(t, np.degrees(r), 'g', linewidth=1.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Rate (°/s)')
    plt.title('Ship Yaw Rate')
    plt.grid(True)

    plt.tight_layout()

    # 船舶旋回轨迹
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, 'b', linewidth=1.5)
    plt.axis('equal')
    plt.xlabel('Longitudinal Position (m)')
    plt.ylabel('Lateral Position (m)')
    plt.title('Ship Turning Trajectory')
    plt.grid(True)

    # 船舶纵向速度变化
    plt.figure(figsize=(8, 5))
    plt.plot(t, u, 'b', linewidth=1.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Ship Longitudinal Speed')
    plt.grid(True)

    plt.show()
    
    # 增强的船舶轨迹可视化
    plt.figure(figsize=(10, 10))
    
    # 绘制轨迹线
    plt.plot(x, y, 'b-', linewidth=1.5, label='Ship Trajectory')
    
    # 标记起始点和结束点
    plt.plot(x[0], y[0], 'go', markersize=10, label='Start Point')
    plt.plot(x[-1], y[-1], 'ro', markersize=10, label='End Point')
    
    # 修改这一行，将 N 转换为整数
    for k in range(0, int(N), 100):
        dx = 30 * cos(psi[k])
        dy = 30 * sin(psi[k])
        plt.arrow(x[k], y[k], dx, dy, 
                 head_width=15, head_length=20, fc='k', ec='k')
    
    # 绘制旋回圆参考（拟合最后300个点）
    if N > 300:
        from scipy import optimize
        def circle_func(xy, xc, yc, R):
            return (xy[0]-xc)**2 + (xy[1]-yc)**2 - R**2
        
        xy = np.array([x[-300:], y[-300:]])
        xc, yc, R = optimize.leastsq(circle_func, [0, 0, 100], args=(xy))[0]
        circle = plt.Circle((xc, yc), R, color='r', fill=False, linestyle='--', label='Fitted Turning Circle')
        plt.gca().add_patch(circle)
        plt.text(xc, yc, f'R={abs(R):.1f}m', ha='center', va='center')
    
    plt.axis('equal')
    plt.xlabel('East Position (m)')
    plt.ylabel('North Position (m)')
    plt.title('Ship Turning Trajectory with Attitude Indicators\n(Rudder Angle: 35°)')
    plt.legend(loc='upper right')
    plt.grid(True)
    
    # 添加比例尺
    scale_length = 100  # 100米比例尺
    plt.plot([x[0], x[0]+scale_length], [y[0]-50, y[0]-50], 'k-', linewidth=2)
    plt.text(x[0]+scale_length/2, y[0]-70, f'{scale_length} m', ha='center')
    
    plt.show()

if __name__ == "__main__":
    main()