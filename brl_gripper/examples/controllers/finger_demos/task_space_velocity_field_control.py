import time
import numpy as np
import select
import sys

from brl_gripper.utils.lie import skew, unskew, expso3

# define controller class
class TaskSpaceVelocityFieldControlDemo:

    def __init__(self):
        self.name = "Task Space (FingerTips) Velocity Field Control Demo"
        self.started = False


    def begin(self, gr_data):
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

    def velocity_field(self, xl_cur, xr_cur):
        # described from the wrold frame
        xc = 0.5 * (xl_cur + xr_cur)
        xd = 0.5 * (xl_cur - xr_cur)
        xcd_des = np.array([0.1, 0.1, 0.1]) - xc
        xdd_des = 10*(np.array([0.05, 0.05, 0.05]) - xd)
        xld_des = xcd_des + xdd_des 
        xrd_des = xcd_des - xdd_des
        return xld_des, xrd_des

    def update(self, gr_data, printstr=True):
        q_cur = gr_data.get_q(gr_data.all_idxs) # joints (wrist + fingers)
        qd_cur = gr_data.get_qd(gr_data.all_idxs) # joint velocities (wrist + fingers)
        xl_cur = gr_data.kinematics['l_dip_tip']['p'] # left fingertip in world frame
        xr_cur = gr_data.kinematics['r_dip_tip']['p'] # right fingertip in world frame
        
        Jxl = gr_data.kinematics['l_dip_tip']['Jacp'] # 3 x 15
        Jxr = gr_data.kinematics['r_dip_tip']['Jacp'] # 3 x 15
        J = np.concatenate((Jxl, Jxr), axis=0) # 6 x 15
        
        w = gr_data.get_w() 
        v = gr_data.get_v()
        V = np.hstack((v, w, qd_cur))
        xd_cur = (J@V.reshape(15, 1)).flatten()

        xld_des, xrd_des = self.velocity_field(xl_cur, xr_cur)
        xd_des = np.hstack((xld_des, xrd_des))

        qrfc_bias = gr_data.qrfc_bias # (v, w, q)
        generalized_force = J.T@(xd_des - xd_cur) + qrfc_bias - np.array([
            5, 5, 5, 1, 1, 1, 0.001, 0.001, 0.001, 0.001, 0.0001, 0.001, 0.001, 0.001, 0.0001])*V

        tau_ff = generalized_force[6:]
        F_ff = generalized_force[:6]
        
        # set positions, Kp gains will be set in firmware
        gr_data.set_q_des(gr_data.all_idxs,   q_cur)
        gr_data.set_qd_des(gr_data.all_idxs,  qd_cur)
        gr_data.set_tau_ff(gr_data.all_idxs,  tau_ff)
        gr_data.set_kp(gr_data.all_idxs,      gr_data.get_kp_default(gr_data.all_idxs))
        gr_data.set_kd(gr_data.all_idxs,      gr_data.get_kd_default(gr_data.all_idxs))
        gr_data.set_F_ff(F_ff)