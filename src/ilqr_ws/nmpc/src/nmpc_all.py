# -*- coding: utf-8 -*-

import math
import casadi as ca
import numpy as np
import time
from matplotlib import pyplot as plt
from nmpc.msg import VehicleArray, Vehicle
from params import Params

import casadi as ca
import numpy as np
from scipy.linalg import sqrtm
from scipy.special import erfinv

def forward_avoidobstacle(x_start,obstacles,params):
    '''
    description: mpc traj follower solver
    args:
        T: sampling time, second
        N: prediction horizon, number of steps
        r: robot radius, default: 0.15m
    return: mpc solver
    '''

    # State variables: x, y, vel (velocity), theta (heading angle)
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    vel = ca.SX.sym('vel')
    theta = ca.SX.sym('theta')
    states = ca.vertcat(x, y, vel, theta)
    n_states = states.size()[0]

    # Control variables: acceleration (acc), yaw rate (yaw_rate)
    acc = ca.SX.sym('acc')
    yaw_rate = ca.SX.sym('yaw_rate')
    controls = ca.vertcat(acc, yaw_rate)
    n_controls = controls.size()[0]

    # 4-DOF Vehicle model dynamics
    dx = vel * ca.cos(theta) * params.T + 0.5 * acc * params.T**2 * ca.cos(theta)
    dy = vel * ca.sin(theta) * params.T + 0.5 * acc * params.T**2 * ca.sin(theta)
    dvel = acc * params.T
    dtheta = yaw_rate * params.T

    rhs = ca.vertcat(dx, dy, dvel, dtheta)

    # Define function f(states, controls) -> rhs
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    # Define decision variables for optimization
    U = ca.SX.sym('U', n_controls, params.N)    # Control input variables for N steps
    X = ca.SX.sym('X', n_states, params.N + 1)  # State variables for N+1 steps
    X_ref = ca.SX.sym('X_ref', n_states, params.N + 1)  # Reference trajectory

    Obstacles = ca.SX.sym('Obstacles', 5, len(obstacles[0]))  # Obstacles [x,y,theta,a,b,]
    # print(len(obstacles[0]))
    # Weights for the cost function
    Q = np.diag([params.w_pos, params.w_pos, params.w_vel, 0.0])  # Weight matrix for state errors
    R = np.diag([params.w_acc, params.w_omega])               # Weight matrix for control effort


    # Cost function and constraints
    obj = 0  # Initialize cost
    g = []   # Initialize constraints

    # Initial constraint: the first state equals the re ference state
    # g.append((X[:, 0] - x_start))
    # g.append((X[0, 0] - x_start[0])**2+(X[1, 0] - x_start[1])**2+(X[2, 0] - x_start[2])**2+(X[3, 0] - x_start[3])**2)
    g.append(X[0, 0] - x_start[0])
    g.append(X[1, 0] - x_start[1])
    g.append(X[2, 0] - x_start[2])
    g.append(X[3, 0] - x_start[3])

    for i in range(params.N):
        state_error = X[:, i] - X_ref[:, i]
        control_effort = U[:, i]
        obj += ca.mtimes([state_error.T, Q, state_error]) 
        obj += ca.mtimes([control_effort.T, R, control_effort])
        
        # Predict next state using the dynamics function
        x_next = f(X[:, i], U[:, i]) + X[:, i]
        # g.append(X[:, i + 1] - x_next)
        # g.append((X[0, i + 1] - x_next[0])**2+(X[1, i + 1] - x_next[1])**2+(X[2, i + 1] - x_next[2])**2+(X[3, i + 1] - x_next[3])**2)
        g.append(X[0, i + 1] - x_next[0])
        g.append(X[1, i + 1] - x_next[1])
        g.append(X[2, i + 1] - x_next[2])
        g.append(X[3, i + 1] - x_next[3])    

    # print(len(obstacles))
    for i in range(params.N):
        for j in range(len(obstacles[0])):

            x_obs = obstacles[0, j]  # 障碍物的x坐标
            y_obs = obstacles[1, j]  # 障碍物的y坐标
            theta_obs = obstacles[2, j]  # 障碍物的朝向角
            a_obs = obstacles[3, j] / 2  # 障碍物的半长轴
            b_obs = obstacles[4, j] / 2  # 障碍物的半短轴

            # 车辆的参数
            x_car = X[0, i]  # 车辆的x坐标
            y_car = X[1, i]  # 车辆的y坐标
            theta_car = X[3, i]  # 车辆的朝向角
            a_car = params.length / 2  + params.safe_prolong_length# 车辆的半长轴
            b_car = params.width / 2  + params.safe_prolong_width# 车辆的半短轴

            # # avoid Obstacle ----------------------
            # constraint = ((x_car - x_obs) * ca.cos(theta_obs) + (y_car - y_obs) * ca.sin(theta_obs))**2 / (a_obs + a_car)**2 + \
            #             ((y_car - y_obs) * ca.cos(theta_obs) - (x_car - x_obs) * ca.sin(theta_obs))**2 / (b_obs + b_car)**2 - 1
            # g.append(constraint)

            #-----------------------

            # # Chance constraint ----------------------
            #a.T*omega*(p_i-p_j) - b_io > erf-1(1-2δ)*sqrt(2*a.T*omega*(sigma_i-sigma_j)*omega.T*a_io)
            #a=(p_i-p_o)/||p_i-p_o|| 1 * 2 / 
            #omega = R_0.T*diag(1/(a0+ri)**2,1/(b0+ri)**2)*R_0, R_0 is rotation matrix of obstacle .(a0,b0) is the elipse of obstacle. 2 x 2
            #δ is a const ， 0.2；
            #sigma_i is the σ^2 of i.
            #erf
            p_i = np.array([x_car, y_car])
            p_o = np.array([x_obs, y_obs])
            a_io = (p_i - p_o) / np.linalg.norm(p_i - p_o)

            b_io = 1
            
            R_0 = np.array([[ca.cos(theta_obs), -ca.sin(theta_obs)],
                            [ca.sin(theta_obs), ca.cos(theta_obs)]])
            a0 = a_obs
            b0 = b_obs
            r_i_a = a_car
            r_i_b = b_car
            # omega_io = R_0.T*np.diag(1/(a0+r_i_a)**2,1/(b0+r_i_b)**2)*R_0 
            # omega_io = R_0.T @ np.diag([1/(a0+r_i_a)**2, 1/(b0+r_i_b)**2]) @ R_0
            omega_io = np.dot(np.dot(R_0.T, np.diag([1/(a0+r_i_a)**2, 1/(b0+r_i_b)**2])), R_0)

            omega_io_half = sqrtm(omega_io) # 2x2

            delta_0 = params.delta_0

            sigma_i_2 = params.sigma_i_2

            sigma_i_x_2 = sigma_i_2 
            sigma_i_y_2 = sigma_i_2 

            sigma_i_matrix = np.diag([sigma_i_x_2,sigma_i_y_2])

            sigma_o_2 = params.sigma_o_2

            sigma_o_x_2 = sigma_o_2 
            sigma_o_y_2 = sigma_o_2 
            sigma_o_matrix = np.diag([sigma_o_x_2,sigma_o_y_2])


            # chanceconstraint = a_io.T @ omega_io_half @ (p_i-p_o) - b_io - erfinv(1 - 2*delta_0)*ca.sqrt(2*a_io.T @ omega_io_half @ (sigma_i_2-sigma_o_2) @ omega_io_half.T @ a_io)
            chanceconstraint = ca.mtimes(ca.mtimes(a_io.T, omega_io_half), (p_i - p_o)) - b_io - erfinv(1 - 2 * delta_0) * ca.sqrt(2 * ca.mtimes(ca.mtimes(ca.mtimes(ca.mtimes(a_io.T, omega_io_half), (sigma_i_matrix + sigma_o_matrix)), omega_io_half.T), a_io))
            g.append(chanceconstraint)


            # # # print(j)
            # g.append((X[0,i]- Obstacles[0,j])*(X[0,i]- Obstacles[0,j])+(X[1,i]- Obstacles[1,j])*(X[1,i]- Obstacles[1,j]) - 16)

    # for i in range(params.N):
    #     distance = ca.sqrt((X[0,i]-X_ref[0,i])**2+(X[1,i]-X_ref[1,i])**2)
    #     g.append(distance)

    for i in range(params.N):
        p_current = X[0:2, i]

        if i < X_ref.shape[1] - 1:
            traj_direction = X_ref[0:2, i + 1] - X_ref[0:2, i]
        else:
            traj_direction = X_ref[0:2, i] - X_ref[0:2, i - 1]

        norm_traj_direction = ca.norm_2(traj_direction)
        is_large_enough = ca.if_else(norm_traj_direction > 1e-6, 1, 0)
        traj_direction = ca.if_else(is_large_enough > 0, traj_direction / norm_traj_direction, traj_direction)

        # 使用 vertcat 来创建 normal_vector
        normal_vector = ca.vertcat(-traj_direction[1], traj_direction[0])

        # 计算 offset 时确保维度一致
        offset = ca.mtimes(ca.transpose(p_current - X_ref[0:2, i]), normal_vector)
        g.append(offset)

    # Optimization variables and parameters
    opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
    # opt_params = ca.reshape(X_ref, -1, 1)
    opt_params = ca.vertcat(ca.reshape(X_ref, -1, 1), ca.reshape(Obstacles, -1, 1))

    # Nonlinear programming problem
    nlp_prob = {'f': obj, 'x': opt_variables, 'p': opt_params, 'g': ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter': params.iteration_time,
                    'ipopt.print_level': 0,
                    'print_time': 0,
                    'ipopt.acceptable_tol': params.acceptable_tol,
                    'ipopt.acceptable_obj_change_tol': params.obj_change_tol}

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    return solver


def mpc_avoidobstacle(solver,ref_traj, obstacles,params):
    '''
    description: mpc trajectory application
    args:
        acc_max: maximum linear velocity
        omega_max: maximum angular velocity
        solver: mpc solver
        ref_traj: reference trajectory, shape(3,N+1)
    return:
        u_0:[linear_velocity, angular_velocity]
    '''
    # solver = forward(x_start,obstacles,T, N)

    # lbg = 0.0  # Lower bound for equality constraints
    # ubg = 0.0  # Upper bound for equality constraints
    # g constraint
    # equation constraint : x - a == 0
    # lb_equality = [0,0,0,0]  # Lower bound for constraints
    # ub_equality = [0,0,0,0]  # Upper bound for constraints
    lb_equality = [0.0]  # Lower bound for constraints
    ub_equality = [0.0]  # Upper bound for constraints
    lbg_equality = np.tile(lb_equality, 4*(1 + params.N))
    ubg_equality = np.tile(ub_equality, 4*(1 + params.N))
    # print(lbg_equality.shape)
    # lbg = np.concatenate([lbg_equality])
    # ubg = np.concatenate([ubg_equality])
    # print(ubg)

    # # inequality constraint : x - a <= 0
    # 

    # lb_inequal = [0,0,0,-np.inf]  # Lower bound for constraints
    # ub_inequal = [np.inf,np.inf,0,np.inf]  # Upper bound for constraints
    # lb_inequal = [-1600000000000]  # Lower bound for constraints
    lb_inequal = [0.0]  # Lower bound for constraints
    ub_inequal = [np.inf]  # Upper bound for constraints
    lbg_inequal = np.tile(lb_inequal, len(obstacles[0])*params.N)
    ubg_inequal = np.tile(ub_inequal, len(obstacles[0])*params.N)
    # # # print(lbg_inequal.shape)
    # lb_inequal_offset = [-2] # right
    # ub_inequal_offset = [4] # left
    lb_inequal_offset = [0] # right
    ub_inequal_offset = [5] # left
    lbg_inequal_offset = np.tile(lb_inequal_offset, params.N)
    ubg_inequal_offset = np.tile(ub_inequal_offset, params.N)
    # # # print(lbg_inequal.shape)

    lbg = np.concatenate([lbg_equality,lbg_inequal, lbg_inequal_offset])
    ubg = np.concatenate([ubg_equality,ubg_inequal, ubg_inequal_offset])

    # lbg = np.concatenate([lbg_equality,lbg_inequal])
    # ubg = np.concatenate([ubg_equality,ubg_inequal])

    # print(ubg)



    # Control bounds
    lb_control = [-params.acc_max, -params.omega_max]
    ub_control = [params.acc_max, params.omega_max]
    lbx_controls = np.tile(lb_control, params.N)
    ubx_controls = np.tile(ub_control, params.N)

    # State bounds
    lb_state = [-np.inf, -np.inf, 0.0, -np.inf]
    ub_state = [np.inf, np.inf, params.v_max, np.inf]
    lbx_states = np.tile(lb_state, params.N + 1)
    ubx_states = np.tile(ub_state, params.N + 1)

    # varible constraint
    lbx = np.concatenate([lbx_controls, lbx_states])
    ubx = np.concatenate([ubx_controls, ubx_states])
    # print(ubx)
    
    
    # for _ in range(N):
    #     lbx.append(-acc_max)
    #     lbx.append(-omega_max)
    #     ubx.append(acc_max)
    #     ubx.append(omega_max)
    # for _ in range(N+1): # note that this is different with the method using structure
    #     lbx.append(-np.inf)
    #     lbx.append(-np.inf)
    #     lbx.append(0.0)
    #     lbx.append(-np.inf)
    #     ubx.append(np.inf)
    #     ubx.append(np.inf)
    #     ubx.append(v_max)
    #     ubx.append(np.inf)
    
    
    # for _ in range(N):
    #     lbx = lbx + [-acc_max, -omega_max, -np.inf, -np.inf, 0.0, -np.inf]
    #     ubx = ubx + [acc_max, omega_max, np.inf, np.inf, v_max, np.inf]

    # lbx = lbx + [-np.inf, -np.inf,0.0, -np.inf]
    # ubx = ubx + [np.inf, np.inf,v_max, np.inf]
    '''
    for _ in range(N):
        lbx = lbx + [-acc_max, -omega_max, -np.inf, -np.inf, -np.inf, -np.inf]
        ubx = ubx + [acc_max, omega_max, np.inf, np.inf, np.inf, np.inf]

    lbx = lbx + [-np.inf, -np.inf, -np.inf]
    ubx = ubx + [np.inf, np.inf, np.inf]
    '''

    # initial states initialization
    states = np.zeros((4,params.N+1))
    controls = np.zeros((2,params.N))
    init_states = np.concatenate((controls.T.reshape(-1,1), states.T.reshape(-1,1)))

    # c_p = ref_traj.T.reshape(-1, 1)
    c_p = np.concatenate((ref_traj.T.reshape(-1,1), obstacles.T.reshape(-1,1)))
    # solver computation
    res = solver(x0 = init_states, p = c_p, lbg = lbg, lbx = lbx, ubg = ubg, ubx = ubx)
    estimated_opt = res['x'].full() # the feedback is in the series [u0, x0, u1, x1, ...]
    
    u_m = estimated_opt[:int(2*params.N)].reshape(params.N, 2).T # (n_controls, N)
    x_m = estimated_opt[int(2*params.N):].reshape(params.N+1, 4).T #(n_states, N+1)
    
    return u_m, x_m

def mpc_api_avoidobstacle(x_start,ref_path,obstacles, params):
    x = ref_path[:, 0]
    y = ref_path[:, 1]

    # 使用五次多项式拟合
    coeffs_x = np.polyfit(np.linspace(0, 1, len(x)), x, 5)
    coeffs_y = np.polyfit(np.linspace(0, 1, len(y)), y, 5)

    # 生成 N 个点的参考轨迹
    t_values = np.linspace(0, 1, params.N+1)
    ref_traj = []

    for t in t_values:
        ix = np.polyval(coeffs_x, t)
        iy = np.polyval(coeffs_y, t)
        ref_traj.append([ix, iy, 0.0])

    ref_traj = np.array(ref_traj)

    new_col = np.ones(ref_traj.shape[0]) * params.v_desired
    new_col = np.expand_dims(new_col, axis=1)
    new_col = new_col.T
    
    ref_traj = np.insert(ref_traj, 2, new_col, axis=1)
    ref_traj = ref_traj.T
    # print(ref_traj.shape)
    obstacles = np.array(obstacles)
    obstacles = obstacles.T
    # print(obstacles.shape)

    solver = forward_avoidobstacle(x_start,obstacles,params)
    u_m, x_m = mpc_avoidobstacle(solver,ref_traj[:, :params.N + 1],obstacles, params)
    return u_m, x_m

def find_nearest_points_indices(x_start, ref_path, n):
    # 计算x_start与ref_path中每个点的欧几里得距离
    distances = np.linalg.norm(ref_path - x_start[:2], axis=1)
    
    # 找到最近的点的索引
    nearest_index = np.argmin(distances)
    
    # 选择从最近点开始的连续n个点的索引
    if nearest_index + n <= len(ref_path):
        indices = np.arange(nearest_index, nearest_index + n)
    else:
        # 如果超出范围，就选择最后的n个点
        indices = np.arange(len(ref_path) - n, len(ref_path))
    
    # 返回这些索引对应的点
    nearest_points = ref_path[indices]
    
    return nearest_points


def forward_follow(x_start,params):
    '''
    description: mpc traj follower solver
    args:
        T: sampling time, second
        N: prediction horizon, number of steps
        r: robot radius, default: 0.15m
    return: mpc solver
    '''

    # State variables: x, y, vel (velocity), theta (heading angle)
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    vel = ca.SX.sym('vel')
    theta = ca.SX.sym('theta')
    states = ca.vertcat(x, y, vel, theta)
    n_states = states.size()[0]

    # Control variables: acceleration (acc), yaw rate (yaw_rate)
    acc = ca.SX.sym('acc')
    yaw_rate = ca.SX.sym('yaw_rate')
    controls = ca.vertcat(acc, yaw_rate)
    n_controls = controls.size()[0]

    # 4-DOF Vehicle model dynamics
    dx = vel * ca.cos(theta) * params.T + 0.5 * acc * params.T**2 * ca.cos(theta)
    dy = vel * ca.sin(theta) * params.T + 0.5 * acc * params.T**2 * ca.sin(theta)
    dvel = acc * params.T
    dtheta = yaw_rate * params.T

    rhs = ca.vertcat(dx, dy, dvel, dtheta)

    # Define function f(states, controls) -> rhs
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    # Define decision variables for optimization
    U = ca.SX.sym('U', n_controls, params.N)    # Control input variables for N steps
    X = ca.SX.sym('X', n_states, params.N + 1)  # State variables for N+1 steps
    X_ref = ca.SX.sym('X_ref', n_states, params.N + 1)  # Reference trajectory

    # Weights for the cost function
    Q = np.diag([params.w_pos, params.w_pos, params.w_vel, 0.0])  # Weight matrix for state errors
    R = np.diag([params.w_acc, params.w_omega])               # Weight matrix for control effort

    # Cost function and constraints
    obj = 0  # Initialize cost
    g = []   # Initialize constraints

    # Initial constraint: the first state equals the reference state
    g.append(X[:, 0] - x_start)

    for i in range(params.N):
        state_error = X[:, i] - X_ref[:, i]
        control_effort = U[:, i]
        obj += ca.mtimes([state_error.T, Q, state_error]) 
        obj += ca.mtimes([control_effort.T, R, control_effort])
        
        # Predict next state using the dynamics function
        x_next = f(X[:, i], U[:, i]) + X[:, i]
        g.append(X[:, i + 1] - x_next)

    # Optimization variables and parameters
    opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
    opt_params = ca.reshape(X_ref, -1, 1)

    # Nonlinear programming problem
    nlp_prob = {'f': obj, 'x': opt_variables, 'p': opt_params, 'g': ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter': params.iteration_time,
                    'ipopt.print_level': 0,
                    'print_time': 0,
                    'ipopt.acceptable_tol': params.acceptable_tol,
                    'ipopt.acceptable_obj_change_tol': params.obj_change_tol}

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    return solver


def mpc_follow(solver,ref_traj, params):
    '''
    description: mpc trajectory application
    args:
        acc_max: maximum linear velocity
        omega_max: maximum angular velocity
        solver: mpc solver
        ref_traj: reference trajectory, shape(3,N+1)
    return:
        u_0:[linear_velocity, angular_velocity]
    '''
    # solver = forward(x_start,T, N)

    lbg = 0.0 # equal constraint
    ubg = 0.0 # equal constraint
    lbx = []
    ubx = []
    
    
    for _ in range(params.N):
        lbx.append(-params.acc_max)
        lbx.append(-params.omega_max)
        ubx.append(params.acc_max)
        ubx.append(params.omega_max)
    for _ in range(params.N+1): # note that this is different with the method using structure
        lbx.append(-np.inf)
        lbx.append(-np.inf)
        lbx.append(0.0)
        lbx.append(-np.inf)
        ubx.append(np.inf)
        ubx.append(np.inf)
        ubx.append(params.v_max)
        ubx.append(np.inf)
    
    
    # for _ in range(N):
    #     lbx = lbx + [-acc_max, -omega_max, -np.inf, -np.inf, 0.0, -np.inf]
    #     ubx = ubx + [acc_max, omega_max, np.inf, np.inf, v_max, np.inf]

    # lbx = lbx + [-np.inf, -np.inf,0.0, -np.inf]
    # ubx = ubx + [np.inf, np.inf,v_max, np.inf]
    '''
    for _ in range(N):
        lbx = lbx + [-acc_max, -omega_max, -np.inf, -np.inf, -np.inf, -np.inf]
        ubx = ubx + [acc_max, omega_max, np.inf, np.inf, np.inf, np.inf]

    lbx = lbx + [-np.inf, -np.inf, -np.inf]
    ubx = ubx + [np.inf, np.inf, np.inf]
    '''

    # initial states initialization
    states = np.zeros((4,params.N+1))
    controls = np.zeros((2,params.N))
    init_states = np.concatenate((controls.T.reshape(-1,1), states.T.reshape(-1,1)))

    c_p = ref_traj.T.reshape(-1, 1)
   

    # solver computation
    res = solver(x0 = init_states, p = c_p, lbg = lbg, lbx = lbx, ubg = ubg, ubx = ubx)
    estimated_opt = res['x'].full() # the feedback is in the series [u0, x0, u1, x1, ...]
    
    u_m = estimated_opt[:int(2*params.N)].reshape(params.N, 2).T # (n_controls, N)
    x_m = estimated_opt[int(2*params.N):].reshape(params.N+1, 4).T #(n_states, N+1)
    
    return u_m, x_m

def mpc_api_follow(x_start,ref_path, params):
    '''
    description: mpc path tracking
    args:
        ref_path:(5||10||15, 2)
        N:number of horizon
        T:sampling time
        v_max:linear velocity
        omega_max:angular velocity
    return:
        U0:[acc, angular_velocity]
    '''
    x = ref_path[:, 0]
    y = ref_path[:, 1]

    # 使用五次多项式拟合
    coeffs_x = np.polyfit(np.linspace(0, 1, len(x)), x, 5)
    coeffs_y = np.polyfit(np.linspace(0, 1, len(y)), y, 5)

    # 生成 N 个点的参考轨迹
    t_values = np.linspace(0, 1, params.N+1)
    ref_traj = []

    for t in t_values:
        ix = np.polyval(coeffs_x, t)
        iy = np.polyval(coeffs_y, t)
        ref_traj.append([ix, iy, 0.0])

    ref_traj = np.array(ref_traj)

    new_col = np.ones(ref_traj.shape[0]) * params.v_desired
    new_col = np.expand_dims(new_col, axis=1)
    new_col = new_col.T
    
    ref_traj = np.insert(ref_traj, 2, new_col, axis=1)
    # print(ref_traj.shape)
    ref_traj = ref_traj.T

    # declare solver
    solver = forward_follow(x_start,params)
    u_m, x_m = mpc_follow(solver,ref_traj[:, :params.N + 1], params)
    
    return u_m, x_m

def nmpc(x_start, global_ref_path,obstacles,params):

    # obstacle [x,y,theta,a,b]

    local_ref_path = find_nearest_points_indices(x_start, global_ref_path,params.points_n)  

    if(len(obstacles)> 0):
        obstacles = np.array(obstacles)

        # 计算每个障碍物与起始点之间的欧氏距离
        distances = np.sqrt((obstacles[:, 0] - x_start[0])**2 + (obstacles[:, 1] - x_start[1])**2)

        # 找到距离小于 20 米的障碍物的索引
        close_obstacles_indices = np.where(distances < 20)[0]

        # 获取这些障碍物的坐标
        close_obstacles = obstacles[close_obstacles_indices]

        # print(len(close_obstacles))
        if(len(close_obstacles) == 0):
            U,X = mpc_api_follow(x_start,local_ref_path, params)
            print("Follow")
        else:
            U,X = mpc_api_avoidobstacle(x_start,local_ref_path,close_obstacles, params)
            print("Avoid Obstacle")
    else:
        U,X = mpc_api_follow(x_start,local_ref_path, params)
        print("Follow")
        
    # for i in range(N):
    #     print("X:",X[0,i],"Y:",X[1,i],"Theta:",X[2,i],"V:",X[3,i])

    return U, X , local_ref_path
