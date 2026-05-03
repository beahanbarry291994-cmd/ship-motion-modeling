import numpy as np
import matplotlib.pyplot as plt

# 仿真参数
dt = 0.1            # 仿真步长(s)
T = 500             # 总仿真时间(s)
t = np.arange(0, T+dt, dt)  # 时间序列

# 船舶模型参数 (Nomoto模型)
K = 0.5             # 回转性指数
T_ship = 50         # 响应时间常数

# PID控制器参数
Kp = 1.0
Ki = 0.01
Kd = 5.0

# 初始状态
psi = 0.0           # 初始航向角(deg)
r = 0.0             # 初始转艏角速度(deg/s)
delta = 0.0         # 初始舵角(deg)
target_psi = 30.0   # 目标航向角(deg)

# 船舶运动参数
u = 5.0             # 船速(m/s)
x = 0.0             # 北向初始位置(m)
y = 0.0             # 东向初始位置(m)

# 数据记录容器
log_psi = np.zeros_like(t)
log_delta = np.zeros_like(t)
log_x = np.zeros_like(t)
log_y = np.zeros_like(t)

# 控制器状态变量
integral = 0.0
prev_error = 0.0

# 主仿真循环
for i in range(len(t)):
    # 记录当前状态
    log_psi[i] = psi
    log_delta[i] = delta
    log_x[i] = x
    log_y[i] = y
    
    # 计算航向误差
    error = target_psi - psi
    
    # PID计算
    integral += error * dt
    derivative = (error - prev_error) / dt
    
    # 舵角指令计算
    delta_cmd = Kp*error + Ki*integral + Kd*derivative
    delta_cmd = np.clip(delta_cmd, -35, 35)  # 舵角限幅
    
    prev_error = error
    
    # 船舶运动模型 (Nomoto一阶方程)
    dr = (K * delta - r) / T_ship
    r += dr * dt
    psi += r * dt


    # 舵机动态 (一阶滞后)
    tau_rudder = 2.5
    d_delta = (delta_cmd - delta) / tau_rudder
    delta += d_delta * dt
    
    # 更新船舶位置 (北东坐标系)
    x += u * np.cos(np.deg2rad(psi)) * dt  # 北向位置
    y += u * np.sin(np.deg2rad(psi)) * dt  # 东向位置

# 可视化
plt.figure(figsize=(12, 8))

# 舵角曲线
plt.subplot(2,2,1)
plt.plot(t, log_delta, 'b', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel('Rudder Angle (deg)')
plt.title('Rudder Command')
plt.grid(True)
plt.xlim(0, T)

# 航向响应曲线
plt.subplot(2,2,2)
plt.plot(t, log_psi, 'r', linewidth=1.5, label='Actual Heading')
plt.plot([0, T], [target_psi, target_psi], 'k--', label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Heading (deg)')
plt.title('Heading Response')
plt.grid(True)
plt.legend(loc='lower right')
plt.xlim(0, T)

# 船舶运动轨迹
plt.subplot(2,2,(3,4))
plt.plot(log_y, log_x, linewidth=1.5)
plt.plot(0, 0, 'go', markersize=10, label='Start')        # 起始点
plt.plot(log_y[-1], log_x[-1], 'ro', markersize=10, label='End')  # 终止点
plt.xlabel('East Position (m)')
plt.ylabel('North Position (m)')
plt.title('Ship Trajectory')
plt.grid(True)
plt.axis('equal')
plt.legend()

plt.tight_layout()
plt.show()