import time
import numpy as np
import select
import sys

import mujoco as mj
from brl_gripper.utils.lie import skew, unskew, expso3

# define controller class
class GraspingVelocityFieldController:
    def __init__(self, *args, **kwargs):
        self.name = "Velocity Field Control"
        self.started = False

    def begin(self, sim):
        gr_data = sim.gr_data
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

    def update(self, sim, printstr=True):
        gr_data = sim.gr_data

        q_cur = gr_data.get_q(gr_data.all_idxs) # joints (wrist + fingers)
        qd_cur = gr_data.get_qd(gr_data.all_idxs) # joint velocities (wrist + fingers)
        xl_cur = gr_data.kinematics['l_dip_tip']['p'] # left fingertip in world frame
        xr_cur = gr_data.kinematics['r_dip_tip']['p'] # right fingertip in world frame
        p_cur = gr_data.get_p() # floating_2 
        R_cur = gr_data.get_R() # floating_2 
        
        Jxl = gr_data.kinematics['l_dip_tip']['Jacp'] # 3 x 15
        Jxr = gr_data.kinematics['r_dip_tip']['Jacp'] # 3 x 15
        J = np.concatenate((Jxl, Jxr), axis=0) # 6 x 15
        
        w = gr_data.get_w() 
        v = gr_data.get_v()
        V = np.hstack((v, w, qd_cur))
        xlrd_cur = (J@V.reshape(15, 1)).flatten()

        ## velocity Field Construction 
        xc = 0.5 * (xl_cur + xr_cur)
        xd = 0.5 * (xl_cur - xr_cur)

        # object info
        xo = sim.mj_data.body('object').xpos
        Ro = sim.mj_data.body('object').xmat.reshape(3, 3)
        geom_id = mj.mj_name2id(sim.mj_model, mj.mjtObj.mjOBJ_GEOM, "object")
        box_size = sim.mj_model.geom_size[geom_id]

        angles = (np.array([0.0, 0.0, 0.1]).reshape(1, 3)@Ro).reshape(3,)
        max_idx = np.argmax(angles)
        nhat = Ro[:, max_idx] # (3,)

        thats = np.delete(Ro, max_idx, axis=1) # (3, 2)
        thats_pm = np.concatenate([thats, -thats], axis=1) # (3, 4)
        max_idx2 = np.argmax((xd.reshape(1, 3)@thats_pm).reshape(4,))
        
        half_grasp_width = np.delete(box_size, max_idx, axis=0)[np.mod(max_idx2, 2)]

        xd_des = thats_pm[:, max_idx2] * (half_grasp_width + 0.01)
        xc_des = xo

        # finger coordinates vf
        phi_T = 2*np.pi
        err_T = 0.01
        err_thr = 0.03
        V_tan = 3
        Vc = 2
        Vd = 100

        ehat = (xc_des - xc)/np.clip(np.linalg.norm(xc_des - xc), a_min=1.0e-6, a_max=np.inf)
        phi = np.arccos((ehat*nhat).sum())
        weight = np.exp(-phi/phi_T)

        xcd_des = weight * (xc_des - xc) + V_tan * (nhat - ehat*(ehat*nhat).sum()) * (1 - weight)

        error = np.linalg.norm((xc_des - xc))
        weight_err = np.tanh((error - err_thr)/err_T)
        xdd_des = (1 + weight_err)/2 * (2*xd_des - xd) + (1 - weight_err)/2 *(0.8*xd_des - xd)
        
        xld_des = Vc*xcd_des + Vd*xdd_des 
        xrd_des = Vc*xcd_des - Vd*xdd_des

        xlrd_des = np.hstack((xld_des, xrd_des))
        f_finger = J.T@(xlrd_des - xlrd_cur)

        # damping 
        f_damping = - np.array([
            5, 5, 5, 1, 1, 1, 
            1.5, 
            0.01, 0.01, 0.01, 0.001, 
            0.01, 0.01, 0.01, 0.001])*V
        
        # hand orientation 
        Vp_tan = 3
        Jp_base = gr_data.kinematics['base']['Jacp'] # (3, 15)
        ehat = (xc_des - p_cur)/np.clip(np.linalg.norm(xc_des - p_cur), a_min=1.0e-6, a_max=np.inf)
        pd_des = (nhat - ehat*(ehat*nhat).sum()) * Vp_tan
        f_orientation = (Jp_base.T@pd_des.reshape(3, 1)).flatten()

        # gravity + coriolis compansation
        f_qrfc_bias = gr_data.qrfc_bias # (v, w, q)

        # sum
        generalized_force = f_finger + f_damping + f_qrfc_bias + f_orientation
        tau_ff = generalized_force[6:]
        F_ff = generalized_force[:6]
        
        # set positions, Kp gains will be set in firmware
        gr_data.set_q_des(gr_data.all_idxs,   q_cur)
        gr_data.set_qd_des(gr_data.all_idxs,  qd_cur)
        gr_data.set_tau_ff(gr_data.all_idxs,  tau_ff)
        gr_data.set_kp(gr_data.all_idxs,      gr_data.get_kp_default(gr_data.all_idxs))
        gr_data.set_kd(gr_data.all_idxs,      gr_data.get_kd_default(gr_data.all_idxs))
        gr_data.set_F_ff(F_ff)