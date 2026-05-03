import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# 仿真参数
DT = 0.1            # 仿真步长(s)
T_MAX = 500         # 总仿真时间(s)
t = np.arange(0, T_MAX+DT, DT)  # 时间序列

# PID控制器参数
KP = 1.0
KI = 0.01
KD = 5.0

# 目标航向角(deg)
TARGET_PSI = 30.0

# 船舶模型参数（来自 KT.py 的 ShipTurningSimulator）
L = 160.0           # 船长(m)
B = 21.0            # 船宽(m)
d = 8.0             # 吃水(m)
Cb = 0.56           # 方形系数
m = Cb * L * B * d * 1.025  # 质量(吨)
K = 0.5             # 旋回性指数
T_ship = 1.0        # 追随性指数
a = 0.4             # 非线性系数
max_rudder_angle = np.radians(35)  # 最大舵角(rad)
rudder_rate = np.radians(2.5)      # 舵机速率(rad/s)

# 初始状态
psi = 0.0           # 初始航向角(rad)
r = 0.0             # 初始首摇角速度(rad/s)
delta = 0.0         # 初始舵角(rad)
u = 5.0 * 0.5144    # 初始航速(m/s，10节)
v = 0.0             # 初始横向速度(m/s)
x = 0.0             # 北向初始位置(m)
y = 0.0             # 东向初始位置(m)

# 数据记录容器
log_psi = np.zeros_like(t)
log_delta = np.zeros_like(t)
log_x = np.zeros_like(t)
log_y = np.zeros_like(t)
log_u = np.zeros_like(t)

# 控制器状态变量
integral = 0.0
prev_error = 0.0

def kt_equation(state, t, delta_cmd):
    """整合后的KT运动方程（包含PID输出的舵角指令）"""
    v, r, psi, x, y, u = state

    # 舵角限幅与速率限制（来自 KT.py 的 get_rudder_angle 逻辑）
    delta = np.clip(delta_cmd, -max_rudder_angle, max_rudder_angle)
    delta = np.clip(delta, delta - rudder_rate*DT, delta + rudder_rate*DT)

    # KT方程（来自 KT.py 的 kt_equation 方法）
    drdt = (K * delta - (1 + a * abs(r)) * r) / T_ship

    # 改进的航速变化模型
    speed_loss_turning = 0.02 * u * abs(r)
    speed_loss_rudder = 0.005 * u * delta**2
    dudt = -speed_loss_turning - speed_loss_rudder

    # 横向速度变化
    dvdt = 0.15 * r * u - 0.25 * v

    # 航向角变化
    dpsidt = r

    # 位置变化
    dxdt = u * np.cos(psi) - v * np.sin(psi)
    dydt = u * np.sin(psi) + v * np.cos(psi)

    return [dvdt, drdt, dpsidt, dxdt, dydt, dudt]   

# 主仿真循环
for i in range(len(t)):
    # 记录当前状态
    log_psi[i] = np.degrees(psi)  # 转换为度数记录
    log_delta[i] = np.degrees(delta)
    log_x[i] = x
    log_y[i] = y
    log_u[i] = u / 0.5144  # 转换为节记录

    # 计算航向误差（度数）
    error = TARGET_PSI - np.degrees(psi)

    # PID计算
    integral += error * DT
    derivative = (error - prev_error) / DT
    delta_cmd = KP*error + KI*integral + KD*derivative
    delta_cmd = np.radians(delta_cmd)  # 转换为弧度

    # 求解KT运动方程（使用 odeint 进行单步积分）
    state = [v, r, psi, x, y, u]
    new_state = odeint(kt_equation, state, [t[i], t[i]+DT], args=(delta_cmd,))[-1]
    v, r, psi, x, y, u = new_state

    prev_error = error

# 可视化
plt.figure(figsize=(12, 8))

# 舵角曲线
plt.subplot(2,2,1)
plt.plot(t, log_delta, 'b', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel('Rudder Angle (deg)')
plt.title('Rudder Command')
plt.grid(True)
plt.xlim(0, T_MAX)

# 航向响应曲线
plt.subplot(2,2,2)
plt.plot(t, log_psi, 'r', linewidth=1.5, label='Actual Heading')
plt.plot([0, T_MAX], [TARGET_PSI, TARGET_PSI], 'k--', label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Heading (deg)')
plt.title('Heading Response')
plt.grid(True)
plt.legend(loc='lower right')
plt.xlim(0, T_MAX)

# 船舶运动轨迹
plt.subplot(2,2,(3,4))
plt.plot(log_y, log_x, linewidth=1.5)
plt.plot(0, 0, 'go', markersize=10, label='Start')
plt.plot(log_y[-1], log_x[-1], 'ro', markersize=10, label='End')
plt.xlabel('East Position (m)')
plt.ylabel('North Position (m)')
plt.title('Ship Trajectory')
plt.grid(True)
plt.axis('equal')
plt.legend()

plt.tight_layout()
plt.show()