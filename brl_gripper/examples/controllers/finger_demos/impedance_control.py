import time
import numpy as np
import select
import sys

# define controller class
class ImpedanceControlDemo:

    def __init__(self):
        self.name = "Impedance Control Demo"
        self.started = False


    def begin(self, gr_data):
        self.started = True

        # NOTE: joint order for model: mcr, mcp pip, dip

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

    def update(self, gr_data):

        # TODO: check for user inputs?

        # run controller

        # TODO: better print formatting here

        # print joint data
        q_cur = gr_data.get_q(gr_data.all_idxs)
        qd_cur = gr_data.get_qd(gr_data.all_idxs)
        tau_cur = gr_data.get_tau(gr_data.all_idxs)
        # print("\033c", end="")
        # print("Joints:")
        # print("q: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4], q_cur[5], q_cur[6], q_cur[7], q_cur[8]))
        # print("qd: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(qd_cur[0], qd_cur[1], qd_cur[2], qd_cur[3], qd_cur[4], qd_cur[5], qd_cur[6], qd_cur[7], qd_cur[8]))
        # print("tau: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(tau_cur[0], tau_cur[1], tau_cur[2], tau_cur[3], tau_cur[4], tau_cur[5], tau_cur[6], tau_cur[7], tau_cur[8]))
        # # print sensor data
        # print("ToF data:")
        # print(gr_data.sensors['l_dip'].tof_raw)
        # print(gr_data.sensors['l_pip'].tof_raw, gr_data.sensors['l_mcp'].tof_raw)
        # print(gr_data.sensors['palm'].tof_raw)
        # print(gr_data.sensors['r_mcp'].tof_raw, gr_data.sensors['r_pip'].tof_raw)
        # print(gr_data.sensors['r_dip'].tof_raw)
        # print("Contact data:")
        # print(gr_data.sensors['l_dip'].contact_force, gr_data.sensors['l_dip'].contact_angle)
        # print(gr_data.sensors['l_pip'].fsr_raw, gr_data.sensors['l_mcp'].fsr_raw)
        # print(gr_data.sensors['palm'].fsr_raw)
        # print(gr_data.sensors['r_mcp'].fsr_raw, gr_data.sensors['r_pip'].fsr_raw)
        # print(gr_data.sensors['r_dip'].contact_force, gr_data.sensors['r_dip'].contact_angle)
        # print("")

        # set fingers and wrist to default positions
        gr_data.set_q_des(gr_data.all_idxs,   gr_data.get_q_des_default(gr_data.all_idxs))
        gr_data.set_qd_des(gr_data.all_idxs,  np.zeros((9,)))
        gr_data.set_tau_ff(gr_data.all_idxs,  np.zeros((9,)))
        gr_data.set_kp(gr_data.all_idxs,      gr_data.get_kp_default(gr_data.all_idxs))
        gr_data.set_kd(gr_data.all_idxs,      gr_data.get_kd_default(gr_data.all_idxs))

        # TODO: would this be preferable?
        # gr_data.set_wrist_des(q_des=0.0, qd_des=0.0, tau_ff=0.0, kp=self.Kp_WR_default, kd=self.Kd_WR_default)
        # gr_data.set_left_finger_des(q_des=[0.0, 0.0, 0.0, 0.0], 
        #                               qd_des=[0.0, 0.0, 0.0, 0.0], 
        #                               tau_ff=[0.0, 0.0, 0.0, 0.0], 
        #                               kp=self.Kp_WR_default, 
        #                               kd=self.Kd_WR_default)

        # set wrist cartesian position
        gr_data.kinematics['base_des']['p'] = np.array([0.0, 0.0, 0.05])
        gr_data.kinematics['base_des']['R'] = np.eye(3)