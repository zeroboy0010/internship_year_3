#!/usr/bin/env python

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import math
import time
import rospy
from geometry_msgs.msg import Twist, Pose

pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pub_pos = rospy.Publisher('Pose', Pose, queue_size=10)

def publisher_velocity(vx, vy, vth):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = vth
    pub_vel.publish(twist)

def publisher_position(x, y, z, roll, pich, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = yaw
    pub_pos.publish(pose)

## Robot Diff parameter
v_max = 5
v_min = -v_max
omega_max = 2
omega_min = -omega_max
x_max = 4
x_min = -x_max
y_max = 4
y_min = -y_max
theta_max = ca.inf
theta_min = -theta_max
## MPC params
Q_x = 5
Q_y = 5
Q_theta = 1
R1 = 0.5
R2 = 0.5
sim_time = 30.0
step_horizon = 0.1
N = 35

def plot_arrow(x, y, yaw, length=0.05, width=0.25, fc="b", ec="k"):
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
            fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def shift_timestep(step_horizon, t0, x0, x_f, u, f):
    x0 = x0.reshape((-1,1))
    t = t0 + step_horizon
    f_value = f(x0, u[:, 0])
    st = ca.DM.full(x0 + (step_horizon) * f_value)
    u = np.concatenate((u[:, 1:], u[:, -1:]), axis=1)
    x_f = np.concatenate((x_f[:, 1:], x_f[:, -1:]), axis=1)
    return t, st, x_f, u

def discrete_velocity(k: int, x_ref: list, y_ref: list, theta_ref: list, dt=0.1):
    vx = (x_ref[k]-x_ref[k-1])/dt
    vy = (y_ref[k]-y_ref[k-1])/dt
    vth = (theta_ref[k]-theta_ref[k-1])/dt
    return vx, vy, vth

def forward_kinematic(v, omega, theta):
    vx = v*np.cos(theta)
    vy = v*np.sin(theta)
    vth = omega
    return vx, vy, vth

def inverse_kinematic(vx, vy, vth):
    v1  = np.sqrt(vx**2+vy**2)
    omega1 = vth
    return v1, omega1

def desired_command_and_trajectory1(t, T, x0_, N_):
    # initial state / last state
    x_ = x0_.reshape(1, -1).tolist()[0]
    u_ = []
    # states for the next N_ trajectories
    for i in range(N_):
        t_predict = t + T*i
        angle = math.pi/10*t_predict
        x_ref_ = 4*math.cos(angle)
        y_ref_ = 4*math.sin(angle)
        theta_ref_ = angle
        #x_ref_ = 3 + 3 * np.sin(4*np.pi/25*t)
        #y_ref_ = np.sin(4*np.pi*t/25)
        #theta_ref_ = 4*np.pi/25*t
        v_ref_ = 1.0
        omega_ref_ = 5.0
        ##if x_ref_ >= 20.0:
        ##    x_ref_ = 20.0
        ##    v_ref_ = 0.0
        x_.append(x_ref_)
        x_.append(y_ref_)
        x_.append(theta_ref_)
        u_.append(v_ref_)
        u_.append(omega_ref_)
    # return pose and command
    x_ = np.array(x_).reshape(N_+1, -1)
    u_ = np.array(u_).reshape(N, -1)
    return x_, u_

def desired_command_and_trajectory(t, T, x0_, N_):
    # initial state / last state
    x_ = x0_.reshape(1, -1).tolist()[0]
    u_ = []
    # states for the next N_ trajectories
    for i in range(N_):
        t_predict = t + T*i
        x_ref_ = 0.5 * t_predict
        y_ref_ = 1.0
        theta_ref_ = 0.0
        v_ref_ = 0.5
        omega_ref_ = 0.0
        if x_ref_ >= 5.0:
            x_ref_ = 5.0
            v_ref_ = 0.0
        x_.append(x_ref_)
        x_.append(y_ref_)
        x_.append(theta_ref_)
        u_.append(v_ref_)
        u_.append(omega_ref_)
    # return pose and command
    x_ = np.array(x_).reshape(N_+1, -1)
    u_ = np.array(u_).reshape(N, -1)
    return x_, u_

x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(
    x,
    y,
    theta
)
n_states = states.numel()
v = ca.SX.sym('v')
omega = ca.SX.sym('omega')
controls = ca.vertcat(
    v,
    omega
)
n_controls = controls.numel()

X = ca.SX.sym('X', n_states, N+1)
U = ca.SX.sym('U', n_controls, N)
X_ref = ca.SX.sym('X_ref', n_states, N+1)
U_ref = ca.SX.sym('U_ref', n_controls, N)

RHS = ca.vertcat(
    v*ca.cos(theta),
    v*ca.sin(theta),
    omega
)

f = ca.Function('f', [states, controls], [RHS])
cost_fn = 0.0
g = X[:, 0] - X_ref[:, 0]
Q = ca.diagcat(Q_x, Q_y, Q_theta)
R = ca.diagcat(R1, R2)

for k in range(N):
    st_err = X[:, k] - X_ref[:, k]
    con_err = U[:, k] - U_ref[:, k]
    cost_fn = cost_fn + st_err.T @ Q @ st_err + con_err.T @ R @ con_err
    st_next = X[:, k+1]
    f_value = f(X[:, k], U[:, k])
    st_next_euler = X[:, k] + (step_horizon*f_value)
    g = ca.vertcat(g, st_next-st_next_euler)

optimal_var = ca.vertcat(
    ca.reshape(X, -1, 1),
    ca.reshape(U, -1, 1)
)
optimal_par = ca.vertcat(
    ca.reshape(X_ref, -1, 1),
    ca.reshape(U_ref, -1, 1)
)

nlp_prob = {
    'f': cost_fn,
    'x': optimal_var,
    'p': optimal_par,
    'g': g
}
opts = {
        'ipopt.max_iter': 2000,
        'ipopt.print_level': False,
        'ipopt.acceptable_tol': 1e-8,
        'ipopt.acceptable_obj_change_tol': 1e-6,
        'print_time': 0
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx1 = ca.DM.zeros((n_states*(N+1), 1))
ubx1 = ca.DM.zeros((n_states*(N+1), 1))
lbx2 = ca.DM.zeros((n_controls*N, 1))
ubx2 = ca.DM.zeros((n_controls*N, 1))

lbx1[0: n_states*(N+1): n_states] = x_min
lbx1[1: n_states*(N+1): n_states] = y_min
lbx1[2: n_states*(N+1): n_states] = theta_min
ubx1[0: n_states*(N+1): n_states] = x_max
ubx1[1: n_states*(N+1): n_states] = y_max
ubx1[2: n_states*(N+1): n_states] = theta_max

lbx2[0: n_controls*N: n_controls] = v_min
lbx2[1: n_controls*N: n_controls] = omega_min
ubx2[0: n_controls*N: n_controls] = v_max
ubx2[1: n_controls*N: n_controls] = omega_max
lbx = ca.vertcat(
    lbx1,
    lbx2
)

ubx = ca.vertcat(
    ubx1,
    ubx2
)

args = {
    'lbg': ca.DM.zeros((n_states*(N+1),1)),
    'ubg': ca.DM.zeros((n_states*(N+1),1)),
    'lbx': lbx,
    'ubx': ubx,
}

# simulation
t0 = 0
mpciter = 0
init_state = np.array([0.0, 0.0, 0.0])
current_state = init_state.copy()
init_control = np.array([0.0, 0.0])
state = np.tile(init_state.reshape((-1,1)), N+1)
control = np.tile(init_control.reshape((-1,1)), N)
next_trajectories = state.copy()
next_controls = control.copy()
opt_x = []
opt_u = []
t_arr = []
dx, dy, dth = [0], [0], [0]
w = np.linspace(0, 2*np.pi, 100)
#t = np.linspace(0, 25, 100)
#ax = 3 + 3 * np.sin(2*np.pi/25*t)
#ay = np.sin(4*np.pi*t/25)
ax = 4*np.cos(w)
ay = 4*np.sin(w)
rospy.init_node('MPC_DIFF')
rate = rospy.Rate(10)
while ((mpciter  * step_horizon < sim_time) and (not rospy.is_shutdown())):
        current_time = mpciter * step_horizon
        args['p'] = np.concatenate((
            next_trajectories.reshape((-1, 1)),
            next_controls.reshape((-1, 1)))
        )
        args['x0'] = np.concatenate(
            (state.reshape((-1,1)),
            control.reshape((-1,1)))
        )
        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p = args['p']
        )
        sol_x = ca.reshape(sol['x'][:n_states*(N+1)], n_states, N+1)
        sol_u = ca.reshape(sol['x'][n_states*(N+1):], n_controls, N)
        opt_x.append(sol_x.full())
        opt_u.append(sol_u.full())
        dx.append(np.array(opt_x)[:, 0, 1])
        dy.append(np.array(opt_x)[:, 1, 1])
        dth.append(np.array(opt_x)[:, 2, 1])
        t0, current_state, state, control = shift_timestep(step_horizon, t0, current_state, sol_x, sol_u, f)
        next_trajectories, next_controls = desired_command_and_trajectory1(t0, step_horizon, current_state, N)
        ## Forward kinematic
        v_x, v_y , v_th  = forward_kinematic(sol_u[0, 0].full(), sol_u[1, 0].full(), sol_x[2, 0].full())
        publisher_velocity(v_x, v_y, v_th)
        publisher_position(sol_x[0, 0].full(), sol_x[1, 0].full(), 0.0, 0.0, 0.0, sol_x[2, 0].full())
        #next_trajectories, next_control = reference_trajectory(t0, step_horizon, init_state, N, dt=0.1, type="straight")
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
        #plt.scatter(mpciter, v_x, color="red")
        #plt.scatter(next_trajectories[0, :], next_trajectories[1, :], color="red")
        plt.axes(xlim=(-1, 6), ylim=(-1, 6))
        plt.plot(ax, ay, color="green")
        plot_arrow(sol_x[0, 0].full(), sol_x[1, 0].full(), sol_x[2, 0].full())
        #plt.plot(next_trajectories[0, 1:], next_trajectories[1, -1:])
        #plt.scatter(sol_x[0, :].full(), sol_x[1, :].full(), sol_x[2, :].full(), marker="*", lw=6, color="r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0005)
        #print(v_x)
        mpciter = mpciter + 1
        rate.sleep()

#plt.scatter(np.array(sol_x[0, :]), np.array(sol_x[1, :]), color="red")
#plt.show()