import math
import casadi as ca
import numpy as np
import time
from matplotlib import pyplot as plt
from cubic_spline_planner import Spline2D

import casadi as ca
import numpy as np

def forward(x_start,T, N):
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
    dx = vel * ca.cos(theta) * T + 0.5 * acc * T**2 * ca.cos(theta)
    dy = vel * ca.sin(theta) * T + 0.5 * acc * T**2 * ca.sin(theta)
    dvel = acc * T
    dtheta = yaw_rate * T

    rhs = ca.vertcat(dx, dy, dvel, dtheta)

    # Define function f(states, controls) -> rhs
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    # Define decision variables for optimization
    U = ca.SX.sym('U', n_controls, N)    # Control input variables for N steps
    X = ca.SX.sym('X', n_states, N + 1)  # State variables for N+1 steps
    X_ref = ca.SX.sym('X_ref', n_states, N + 1)  # Reference trajectory

    # Weights for the cost function
    Q = np.diag([2.0, 2.0, 0.5, 0.0])  # Weight matrix for state errors
    R = np.diag([1.0, 3.0])               # Weight matrix for control effort

    # Cost function and constraints
    obj = 0  # Initialize cost
    g = []   # Initialize constraints

    # Initial constraint: the first state equals the reference state
    g.append(X[:, 0] - x_start)

    for i in range(N):
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
    opts_setting = {'ipopt.max_iter': 20,
                    'ipopt.print_level': 0,
                    'print_time': 1,
                    'ipopt.acceptable_tol': 1e-4,
                    'ipopt.acceptable_obj_change_tol': 1e-2}

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    return solver


def mpc(solver,x_start,ref_traj, T, N,v_max, acc_max, omega_max):
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
    solver = forward(x_start,T, N)

    lbg = 0.0 # equal constraint
    ubg = 0.0 # equal constraint
    lbx = []
    ubx = []
    
    
    for _ in range(N):
        lbx.append(-acc_max)
        lbx.append(-omega_max)
        ubx.append(acc_max)
        ubx.append(omega_max)
    for _ in range(N+1): # note that this is different with the method using structure
        lbx.append(-np.inf)
        lbx.append(-np.inf)
        lbx.append(0.0)
        lbx.append(-np.inf)
        ubx.append(np.inf)
        ubx.append(np.inf)
        ubx.append(v_max)
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
    states = np.zeros((4,N+1))
    controls = np.zeros((2,N))
    init_states = np.concatenate((controls.T.reshape(-1,1), states.T.reshape(-1,1)))

    c_p = ref_traj.T.reshape(-1, 1)
   

    # solver computation
    res = solver(x0 = init_states, p = c_p, lbg = lbg, lbx = lbx, ubg = ubg, ubx = ubx)
    estimated_opt = res['x'].full() # the feedback is in the series [u0, x0, u1, x1, ...]
    
    u_m = estimated_opt[:int(2*N)].reshape(N, 2).T # (n_controls, N)
    x_m = estimated_opt[int(2*N):].reshape(N+1, 4).T #(n_states, N+1)
    
    return u_m, x_m

def mpc_api(x_start,ref_path, N, T, v_max,v_desired, acc_max, omega_max):
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

    # ds = T * v_desired / 2
    
    # sp = Spline2D(x, y)
    # s = np.arange(0, sp.s[-1], ds)

    # ref_traj = []
    # for i, i_s in enumerate(s):
    #     ix, iy = sp.calc_position(i_s)
    #     ref_traj.append([ix, iy, 0.0])

    # # print(len(ref_traj))
    # while(len(ref_traj) < N+1):
    #     ds = ds * 0.8
    #     ref_traj = []
    #     for i, i_s in enumerate(s):
    #         ix, iy = sp.calc_position(i_s)
    #         ref_traj.append([ix, iy, 0.0])

    # 使用五次多项式拟合
    coeffs_x = np.polyfit(np.linspace(0, 1, len(x)), x, 5)
    coeffs_y = np.polyfit(np.linspace(0, 1, len(y)), y, 5)

    # 生成 N 个点的参考轨迹
    t_values = np.linspace(0, 1, N+1)
    ref_traj = []

    for t in t_values:
        ix = np.polyval(coeffs_x, t)
        iy = np.polyval(coeffs_y, t)
        ref_traj.append([ix, iy, 0.0])

    ref_traj = np.array(ref_traj)

    new_col = np.ones(ref_traj.shape[0]) * v_desired
    new_col = np.expand_dims(new_col, axis=1)
    new_col = new_col.T
    
    ref_traj = np.insert(ref_traj, 2, new_col, axis=1)
    # print(ref_traj.shape)
    ref_traj = ref_traj.T

    # declare solver
    solver = forward(x_start,T, N)
    u_m, x_m = mpc(solver, x_start,ref_traj[:, :N + 1], T, N, v_max,acc_max, omega_max)
    
    # # Visualization
    # plt.scatter(ref_traj[0, :], ref_traj[1, :], s=(0.5) ** 2)
    # plt.plot(x_m[0, :], x_m[1, :], "xb", label="mpc")
    # plt.legend(['reference trajectory', 'mpc predicted states'], loc="lower right")
    # plt.xlabel("x axes frame")
    # plt.ylabel("y axes frame")

    # plt.grid(True)
    # plt.show()
    # plt.close()
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

def nmpc(x_start, global_ref_path,obstacles):
    T = 0.1 # dt
    N = 50 # horizon N
    v_max = 30 # speed max
    v_desired = 5.0 # desired speed
    acc_max = 2.0 # acc max
    omega_max = 1.0 # omega max

    poinst_n = 20

    local_ref_path = find_nearest_points_indices(x_start, global_ref_path, poinst_n)  

    U,X = mpc_api(x_start,local_ref_path, N, T, v_max,v_desired, acc_max, omega_max)
    for i in range(N):
        print("X:",X[0,i],"Y:",X[1,i],"Theta:",X[2,i],"V:",X[3,i])

    return U, X

if __name__ == '__main__':
    

    T = 0.1 # dt
    N = 50 # horizon N
    v_max = 5 # speed max
    v_desired = 2.5 # desired speed
    acc_max = 5 # acc max
    omega_max = 5 # omega max

    x_start = np.array([1, 1, 0, 0]) # initial state x y v theta 

    global_ref_path = np.array([[0*2, 0],
                         [2*2, 1],
                         [3*2, 3],
                         [5*2, 4],
                         [6*2, 6],
                         [8*2, 7]])
    
    local_ref_path = find_nearest_points_indices(x_start, global_ref_path, 5)    
    
    # mpc controller, return [acc, angular_velocity]
    res = mpc_api(x_start,local_ref_path, N, T, v_max,v_desired, acc_max, omega_max)
    print(res)
