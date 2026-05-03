import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class ShipParams:
    def __init__(self):
        # 船舶主尺度
        self.L = 100.0  # 船长(m)
        self.B = 15.0   # 船宽(m)
        self.d = 5.0     # 吃水(m)
        self.Cb = 0.65   # 方形系数
        
        # 质量参数
        self.m = self.L * self.B * self.d * self.Cb * 1.025  # 质量(kg)
        self.Iz = 0.25 * self.m * self.L**2  # 绕Z轴转动惯量
        
        # 修正的水动力导数 (重点调整了Nr和Nv)
        self.Xuu = -0.05
        self.Xvv = -0.1
        self.Xvr = 0.2
        self.Xrr = -0.1
        
        self.Yv = -0.3
        self.Yr = 0.3
        self.Yvvv = 0.8
        self.Yvvr = -1.5
        self.Yvrr = 0.3
        self.Yrrr = -0.1
        
        # 修正的首摇力矩导数
        self.Nv = -0.3   # 增加绝对值
        self.Nr = -0.5   # 增加阻尼
        self.Nvvv = -0.05
        self.Nvvr = 0.2
        self.Nvrr = -0.3
        self.Nrrr = -0.02
        
        # 舵参数
        self.AR = 0.025 * self.L * self.d  # 舵面积
        self.HR = 0.5 * self.d             # 舵高
        self.xR = -0.45 * self.L           # 舵位置x坐标
        
        # 初始条件
        self.u0 = 5.0  # 初始速度(m/s)
        self.v0 = 0.0  # 初始横向速度
        self.r0 = 0.0  # 初始转首角速度

def mmg_model(state, t, ship, delta_fn):
    u, v, r, x, y, psi = state
    
    # 当前舵角
    delta = delta_fn(t)
    delta_rad = np.deg2rad(delta)
    
    # 水动力和力矩计算
    U = np.sqrt(u**2 + v**2)
    if U < 0.001:
        U = 0.001
    
    # 无量纲化参数
    v_hat = v / U
    r_hat = r * ship.L / U
    
    # 纵向力
    X = (ship.Xuu * u**2 + ship.Xvv * v**2 + ship.Xvr * v * r + 
         ship.Xrr * r**2) * 0.5 * 1.025 * U**2 * ship.L * ship.d
    
    # 横向力
    Y = (ship.Yv * v + ship.Yr * r + ship.Yvvv * v**3 / ship.L**2 + 
         ship.Yvvr * v**2 * r / ship.L + ship.Yvrr * v * r**2 + 
         ship.Yrrr * r**3 * ship.L) * 0.5 * 1.025 * U**2 * ship.L * ship.d
    
    # 转首力矩 (修正了量纲)
    N = (ship.Nv * v + ship.Nr * r + ship.Nvvv * v**3 / ship.L**2 + 
         ship.Nvvr * v**2 * r / ship.L + ship.Nvrr * v * r**2 + 
         ship.Nrrr * r**3 * ship.L) * 0.5 * 1.025 * U**2 * ship.L**2 * ship.d
    
    # 舵力计算 (修正了力矩计算)
    vR = v + ship.xR * r
    uR = u
    UR = np.sqrt(uR**2 + vR**2)
    if UR < 0.001:
        UR = 0.001
    
    alphaR = delta_rad - np.arctan2(vR, uR)
    
    f_alpha = 6.13 * ship.AR / (ship.AR + 2.25)
    FN = 0.5 * 1.025 * UR**2 * ship.AR * f_alpha * np.sin(alphaR)
    
    XR = -FN * np.sin(delta_rad)
    YR = FN * np.cos(delta_rad)
    NR = ship.xR * YR + FN * 0.25 * ship.L  # 修正的舵力矩
    
    # 总合力和力矩
    X_total = X + XR
    Y_total = Y + YR
    N_total = N + NR
    
    # 运动方程
    dudt = (X_total + ship.m * v * r) / ship.m
    dvdt = (Y_total - ship.m * u * r) / ship.m
    drdt = N_total / ship.Iz  # 首摇角速度变化率
    
    # 位置和航向变化
    dxdt = u * np.cos(psi) - v * np.sin(psi)
    dydt = u * np.sin(psi) + v * np.cos(psi)
    dpsidt = r
    
    return [dudt, dvdt, drdt, dxdt, dydt, dpsidt]

def delta_function(t):
    """舵角控制函数"""
    if t < 10:
        return 0.0
    elif t < 30:
        return 35.0 * (1 - np.exp(-(t-10)/5))
    else:
        return 35.0

def simulate_ship_movement(ship, delta_fn, t_max=350, dt=0.5):
    t = np.arange(0, t_max, dt)
    state0 = [ship.u0, ship.v0, ship.r0, 0, 0, 0]
    
    # 使用更精确的求解器
    states = odeint(mmg_model, state0, t, args=(ship, delta_fn), 
                   atol=1e-6, rtol=1e-6, mxstep=5000, hmax=1.0)
    
    u = states[:, 0]
    v = states[:, 1]
    r = np.rad2deg(states[:, 2])  # 转换为度/秒
    x = states[:, 3]
    y = states[:, 4]
    psi = np.rad2deg(states[:, 5] % (2*np.pi))
    delta = np.array([delta_fn(ti) for ti in t])
    
    return t, u, v, r, x, y, psi, delta

def plot_results(t, x, y,u,v, psi, delta, r):
    plt.figure(figsize=(16, 12))
    
    # 舵角随时间变化
    plt.subplot(3, 2, 1)
    plt.plot(t, delta, 'b-', linewidth=2)
    plt.title('Rudder Angle vs Time', fontsize=12)
    plt.xlabel('Time (s)')
    plt.ylabel('Rudder Angle (deg)')
    plt.grid(True)
    
    # 航向随时间变化
    plt.subplot(3, 2, 2)
    plt.plot(t, psi, 'r-', linewidth=2)
    plt.title('Heading Angle vs Time', fontsize=12)
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (deg)')
    plt.grid(True)
    
    # 船舶轨迹
    plt.subplot(3, 2, 3)
    plt.plot(y, x, 'g-', linewidth=2)
    plt.title('Ship Trajectory', fontsize=12)
    plt.xlabel('y Position (m)')
    plt.ylabel('x Position (m)')
    plt.axis('equal')
    plt.grid(True)
    
    # 首摇角速度 (deg/s)
    plt.subplot(3, 2, 4)
    plt.plot(t, r, 'm-', linewidth=2)
    plt.title('Yaw Rate vs Time', fontsize=12)
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Rate (deg/s)')
    plt.grid(True)
    
    # 横向速度
    plt.subplot(3, 2, 5)
    plt.plot(t, np.rad2deg(np.arctan2(v, u)), 'c-', linewidth=2)
    plt.title('Drift Angle vs Time', fontsize=12)
    plt.xlabel('Time (s)')
    plt.ylabel('Drift Angle (deg)')
    plt.grid(True)
    
    # 合速度
    plt.subplot(3, 2, 6)
    plt.plot(t, np.sqrt(u**2 + v**2), 'k-', linewidth=2)
    plt.title('Speed vs Time', fontsize=12)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    ship = ShipParams()
    t, u, v, r, x, y, psi, delta = simulate_ship_movement(ship, delta_function)
    
    # 打印最大首摇角速度
    print(f"Maximum yaw rate: {np.max(np.abs(r)):.2f} deg/s")
    
    plot_results(t, x, y,u,v, psi, delta, r)

if __name__ == "__main__":
    main()