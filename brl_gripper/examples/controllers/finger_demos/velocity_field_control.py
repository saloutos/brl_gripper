import time
import numpy as np
import select
import sys

# define controller class
class VelocityFieldControlDemo:

    def __init__(self):
        self.name = "Velocity Field Control Demo"
        self.started = False


    def begin(self, gr_data):
        self.dt = (1.0/300.0) #0.001
        self.time_step = 0

        self.started = True

        # set finger defaults for control
        gr_data.set_q_des_default(gr_data.l_idxs,   np.array([0.0, 0.4, -0.8, -0.8]))
        gr_data.set_q_des_default(gr_data.r_idxs,   np.array([0.0, -0.4, 0.8, 0.8]))

        gr_data.set_kp_default(gr_data.l_idxs,      np.array([8.0, 2.5, 2.5, 2.5]))
        gr_data.set_kp_default(gr_data.r_idxs,      np.array([8.0, 2.5, 2.5, 2.5]))

        gr_data.set_kd_default(gr_data.l_idxs,      np.array([0.05, 0.05, 0.05, 0.05]))
        gr_data.set_kd_default(gr_data.r_idxs,      np.array([0.05, 0.05, 0.05, 0.05]))
        
        # set wrist defaults for control
        gr_data.set_q_des_default(gr_data.w_idxs,   np.zeros((1,)))
        gr_data.set_kp_default(gr_data.w_idxs,      np.array([3.0]))
        gr_data.set_kd_default(gr_data.w_idxs,      np.array([0.1]))

        self.q_0 = np.array([0.0, 0.0, 0.4, -0.8, -0.8, 0.0, -0.4, 0.8, 0.8]) + np.random.randn(9)*0.5

    def velocity_field(self, q_cur):
        qd_des = 10*(self.q_0 - q_cur)
        return qd_des

    def update(self, gr_data, printstr=True):
        q_cur = gr_data.get_q(gr_data.all_idxs)

        if printstr:
            print("Joints:")
            print_joint_str = "q:"
            for i in range(len(q_cur)):
                print_joint_str += f" {q_cur[i]:.2f}"
            print(print_joint_str)

        qd_des = self.velocity_field(q_cur)
        if printstr:
            print("Desired Velocities:")
            print_vel_str = "qd:"
            for i in range(len(qd_des)):
                print_vel_str += f" {qd_des[i]:.2f}"
            print(print_vel_str)


        # set positions, Kp gains will be set in firmware
        gr_data.set_q_des(gr_data.all_idxs,   q_cur)
        gr_data.set_qd_des(gr_data.all_idxs,  qd_des)
        gr_data.set_tau_ff(gr_data.all_idxs,  np.zeros((9,)))
        gr_data.set_kp(gr_data.all_idxs,      gr_data.get_kp_default(gr_data.all_idxs))
        gr_data.set_kd(gr_data.all_idxs,      gr_data.get_kd_default(gr_data.all_idxs))

        # set wrist cartesian position
        gr_data.kinematics['base_des']['p'] = np.array([0.0, 0.0, 0.2])
        gr_data.kinematics['base_des']['R'] = np.eye(3)
