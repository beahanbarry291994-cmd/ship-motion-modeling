from turtle import Turtle
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

# 旋回实验参数
turning_test = True  # 是否进行旋回实验
test_start_time = 10 # 旋回实验开始时间(s)
test_rudder_angle = 15 # 旋回实验舵角(deg)

# 数据记录容器
log_psi = np.zeros_like(t)
log_delta = np.zeros_like(t)
log_x = np.zeros_like(t)
log_y = np.zeros_like(t)
log_r = np.zeros_like(t)  # 记录转艏角速度

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
    log_r[i] = r
    
    # 旋回实验逻辑
    if turning_test and t[i] >= test_start_time:
        delta_cmd = test_rudder_angle  # 固定舵角
    else:
        # 正常PID控制逻辑
        error = target_psi - psi
        integral += error * dt
        derivative = (error - prev_error) / dt
        delta_cmd = Kp*error + Ki*integral + Kd*derivative
        prev_error = error
    
    # 舵角限幅
    delta_cmd = np.clip(delta_cmd, -35, 35)
    
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
plt.figure(figsize=(14, 10))

# 舵角曲线
plt.subplot(3,2,1)
plt.plot(t, log_delta, 'b', linewidth=1.5)
if turning_test:
    plt.axvline(x=test_start_time, color='r', linestyle='--', label='Test Start')
plt.xlabel('Time (s)')
plt.ylabel('Rudder Angle (deg)')
plt.title('Rudder Command')
plt.grid(True)
plt.xlim(0, T)
if turning_test:
    plt.legend()

# 航向响应曲线
plt.subplot(3,2,2)
plt.plot(t, log_psi, 'r', linewidth=1.5, label='Actual Heading')
if not turning_test:
    plt.plot([0, T], [target_psi, target_psi], 'k--', label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Heading (deg)')
plt.title('Heading Response')
plt.grid(True)
plt.legend(loc='lower right')
plt.xlim(0, T)

# 转艏角速度曲线
plt.subplot(3,2,3)
plt.plot(t, log_r, 'g', linewidth=1.5)
if turning_test:
    plt.axvline(x=test_start_time, color='r', linestyle='--', label='Test Start')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Rate (deg/s)')
plt.title('Yaw Rate Response')
plt.grid(True)
plt.xlim(0, T)

# 船舶运动轨迹
plt.subplot(3,2,(4,6))
plt.plot(log_y, log_x, linewidth=1.5)
plt.plot(0, 0, 'go', markersize=10, label='Start')        # 起始点
plt.plot(log_y[-1], log_x[-1], 'ro', markersize=10, label='End')  # 终止点

if turning_test:
    # 计算旋回特征参数
    test_start_idx = int(test_start_time/dt)
    max_r = np.max(log_r[test_start_idx:])
    steady_r = np.mean(log_r[int(test_start_time/dt)+100:])  # 稳态转艏速率
    
    # 标注旋回直径
    idx_180 = np.where(log_psi[test_start_idx:] >= 180)[0]
    if len(idx_180) > 0:
        idx_180 = idx_180[0] + test_start_idx
        diameter = 2*(log_y[idx_180] - log_y[test_start_idx])
        plt.plot([log_y[test_start_idx], log_y[idx_180]], 
                 [log_x[test_start_idx], log_x[idx_180]], 'm--')
        plt.text(log_y[test_start_idx], log_x[test_start_idx]+50,
                f'Diameter: {abs(diameter):.1f}m', color='m')
    
    plt.text(log_y[-1], log_x[-1], 
             f'Steady yaw rate: {steady_r:.2f} deg/s', ha='right')

plt.xlabel('East Position (m)')
plt.ylabel('North Position (m)')
plt.title('Ship Trajectory' + (' (Turning Circle Test)' if turning_test else ''))
plt.grid(True)
plt.axis('equal')
plt.legend()

plt.tight_layout()
plt.show()

# 控制台输出旋回实验结果
if turning_test:
    print("\n=== Turning Circle Test Results ===")
    print(f"Test rudder angle: {test_rudder_angle} deg")
    print(f"Maximum yaw rate: {max_r:.2f} deg/s")
    print(f"Steady-state yaw rate: {steady_r:.2f} deg/s")
    if 'diameter' in locals():
        print(f"Tactical diameter: {abs(diameter):.1f} m")
    print("Advance and transfer can be measured from the plot")