import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class ShipTurningSimulator:
    def __init__(self):
        # 船舶参数 (以Mariner船为参考)
        self.L = 160.0  # 船长(m)
        self.B = 21.0   # 船宽(m)
        self.d = 8.0    # 吃水(m)
        self.Cb = 0.56  # 方形系数
        self.m = self.Cb * self.L * self.B * self.d * 1.025  # 质量(吨)
        
        # 无量纲化KT方程参数
        self.K = 0.5     # 旋回性指数
        self.T = 1.0     # 追随性指数
        self.a = 0.4     # 非线性系数
        
        # 初始状态
        self.u0 = 5.0 * 0.5144  # 降低初始航速为10节(约5.14 m/s)
        self.v0 = 0.0    # 初始横向速度(m/s)
        self.r0 = 0.0     # 初始首摇角速度(rad/s)
        self.psi0 = 0.0   # 初始航向角(rad)
        self.x0 = 0.0     # 初始x位置(m)
        self.y0 = 0.0    # 初始y位置(m)
        
        # 仿真参数
        self.dt = 0.1    # 时间步长(s)
        self.t_max = 60  # 最大仿真时间(s)
        
        # 舵机参数
        self.max_rudder_angle = np.radians(35)  # 最大舵角(rad)
        self.rudder_rate = np.radians(2.5)      # 舵机速率(rad/s)
        
    def get_rudder_angle(self, t):
        """舵角变化曲线"""
        # 舵角变化策略: 前30秒逐步转到目标舵角，之后保持
        if t < 30.0:
            return min(self.max_rudder_angle, self.rudder_rate * t)
        else:
            return self.max_rudder_angle
    
    def kt_equation(self, state, t):
        """
        KT方程 - 描述船舶旋回运动的微分方程
        state: [v, r, psi, x, y, u]
        t: 时间
        """
        v, r, psi, x, y, u = state
        
        # 获取当前舵角
        delta = self.get_rudder_angle(t)
        
        # KT方程
        drdt = (self.K * delta - (1 + self.a * abs(r)) * r) / self.T
        
        # 改进的航速变化模型
        # 考虑旋回阻力和舵阻力
        speed_loss_due_turning = 0.02 * u * abs(r)  # 旋回导致的航速损失
        speed_loss_due_rudder = 0.005 * u * delta**2  # 舵角导致的航速损失
        dudt = -speed_loss_due_turning - speed_loss_due_rudder
        
        # 横向速度变化 (改进模型)
        dvdt = 0.15 * r * u - 0.25 * v
        
        # 航向角变化
        dpsidt = r
        
        # 位置变化
        dxdt = u * np.cos(psi) - v * np.sin(psi)
        dydt = u * np.sin(psi) + v * np.cos(psi)
        
        return [dvdt, drdt, dpsidt, dxdt, dydt, dudt]
    
    def simulate(self):
        """执行仿真"""
        # 时间点
        t = np.arange(0, self.t_max, self.dt)
        
        # 初始状态
        state0 = [self.v0, self.r0, self.psi0, self.x0, self.y0, self.u0]
        
        # 解微分方程
        states = odeint(self.kt_equation, state0, t)
        
        # 提取结果
        v = states[:, 0]
        r = states[:, 1]
        psi = states[:, 2]
        x = states[:, 3]
        y = states[:, 4]
        u = states[:, 5]
        
        # 计算舵角变化
        delta = np.array([self.get_rudder_angle(time) for time in t])
        
        return t, x, y, psi, r, u, delta
    
    def plot_results(self, t, x, y, psi, r, u, delta):
        """绘制结果图形"""
        plt.figure(figsize=(15, 12))
        
        # 1. 船舶旋回运动轨迹图
        plt.subplot(3, 2, 1)
        plt.plot(x, y)
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.title('Ship Turning Trajectory')
        plt.grid(True)
        plt.axis('equal')
        
        # 2. 首摇角速度图
        plt.subplot(3, 2, 2)
        plt.plot(t, np.degrees(r))
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw rate (deg/s)')
        plt.title('Yaw Rate vs Time')
        plt.grid(True)
        
        # 3. 航向变化图
        plt.subplot(3, 2, 3)
        plt.plot(t, np.degrees(psi) % 360)
        plt.xlabel('Time (s)')
        plt.ylabel('Heading (deg)')
        plt.title('Heading vs Time')
        plt.grid(True)
        
        # 4. 航速变化图
        plt.subplot(3, 2, 4)
        plt.plot(t, u / 0.5144)  # 转换为节
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (knots)')
        plt.title('Speed vs Time')
        plt.grid(True)
        
        # 5. 舵角变化图
        plt.subplot(3, 2, 5)
        plt.plot(t, np.degrees(delta))
        plt.xlabel('Time (s)')
        plt.ylabel('Rudder angle (deg)')
        plt.title('Rudder Angle vs Time')
        plt.grid(True)
        
        # 6. 航速与舵角关系图
        plt.subplot(3, 2, 6)
        plt.plot(np.degrees(delta), u / 0.5144)
        plt.xlabel('Rudder angle (deg)')
        plt.ylabel('Speed (knots)')
        plt.title('Speed vs Rudder Angle')
        plt.grid(True)
        
        plt.tight_layout()
        # 保存图片到当前工作目录下，文件名为 ship_simulation.png
        plt.savefig('ship_simulation.png')
        plt.show()
    
    def run_simulation(self):
        """运行完整仿真并绘制结果"""
        t, x, y, psi, r, u, delta = self.simulate()
        self.plot_results(t, x, y, psi, r, u, delta)

if __name__ == "__main__":
    simulator = ShipTurningSimulator()
    simulator.run_simulation()