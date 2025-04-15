# -*- coding: utf-8 -*-
class Params:
    def __init__(self):
        # 定义参数及其默认值
        self.T = 0.1 # 采样时间
        self.N = 40 # 预测步长
        self.v_max = 30 # 最大速度
        self.v_desired = 5.0 # 目标速度
        self.acc_max = 1.0 # 最大加速度
        self.omega_max = 1.0 # 最大角速度
        self.wheelbase = 3.00 # 车辆轴距
        self.length = 4.79 # 车辆长度
        self.width = 2.16 # 车辆宽度
        self.safe_prolong_length = 0.1
        self.safe_prolong_width = 0.4 #1.4
        self.points_n = 20 # 路径点数量
        self.sigma_i_2 = 0.0
        self.sigma_o_2 = 0.0
        # self.sigma_x_2 = 0.1**2 # x方向噪声方差
        # self.sigma_y_2 = 0.1**2 # y方向噪声方差
        self.sigma_theta_2 = 0.0 # 角度方向噪声方差
        self.delta_0 = 0.05 # 碰撞概率
        self.description = "This class contains parameters for vehicle motion and noise."

        self.w_pos = 0.75 # 位置权重
        self.w_vel = 3.0 # 速度权重

        self.w_acc = 1.0 # 加速度权重
        self.w_omega = 4.0 # 角速度权重

        self.iteration_time = 100000 # 迭代次数
        self.acceptable_tol = 1e-6 # 可接受误差
        self.obj_change_tol = 1e-4 # 目标函数变化误差

    # def __repr__(self):
    #     # 提供一个便于查看的字符串表示形式
    #     return (f"Params(T={self.T}, N={self.N}, v_max={self.v_max}, v_desired={self.v_desired}, "
    #             f"acc_max={self.acc_max}, omega_max={self.omega_max}, wheelbase={self.wheelbase}, "
    #             f"length={self.length}, width={self.width}, points_n={self.points_n}, "
    #             f"sigma_x_2={self.sigma_x_2}, sigma_y_2={self.sigma_y_2}, sigma_theta_2={self.sigma_theta_2}, "
    #             f"description={self.description})")

# # 创建 Params 对象
# params = Params()