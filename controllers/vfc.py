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

        # set default parameters
        self.xc_des = np.array([0.0, 0.0, 0.0]) # default center of finger tips
        self.xd_des = np.array([0.0, 0.0, 0.0]) # default difference of finger tips
        self.nhat = np.array([0.0, 0.0, 1.0]) # default reaching normal

        # set initial state
        self.state = 'reaching'

    def proprioception(self, sim):
        gr_data = sim.gr_data

        # proprioception
        q = gr_data.get_q(gr_data.all_idxs) # joints (wrist + fingers)
        x_lft= gr_data.kinematics['l_dip_tip']['p'] # left fingertip in world frame
        x_rft = gr_data.kinematics['r_dip_tip']['p'] # right fingertip in world frame
        R_lft = gr_data.kinematics['l_dip_tip']['R'] # left fingertip rotation matrix in world frame
        R_rft = gr_data.kinematics['r_dip_tip']['R'] # right fingertip rotation matrix in world frame
        R_ldipf = gr_data.kinematics['l_dip_force']['R']
        R_rdipf = gr_data.kinematics['r_dip_force']['R']

        p = gr_data.get_p() # floating_2 
        R = gr_data.get_R() 

        q_dot = gr_data.get_qd(gr_data.all_idxs) # joint velocities (wrist + fingers)
        w = gr_data.get_w() 
        v = gr_data.get_v()
        V = np.hstack((v, w, q_dot))

        J_xlft = gr_data.kinematics['l_dip_tip']['Jacp'] # 3 x 15
        J_xrft = gr_data.kinematics['r_dip_tip']['Jacp'] # 3 x 15
        J_xft = np.concatenate((J_xlft, J_xrft), axis=0) # 6 x 15
        J_Rlft = gr_data.kinematics['l_dip_tip']['JacR'] # (3, 15)
        J_Rrft = gr_data.kinematics['r_dip_tip']['JacR'] # (3, 15)
        J_Rft = np.concatenate((J_Rlft, J_Rrft), axis=0) # 6 x 15
        J_p= gr_data.kinematics['base']['Jacp'] # (3, 15)
        
        
        x_ft_dot = (J_xft@V.reshape(15, 1)).flatten()
        return (
            q,      # (9,) wrist + fingers joints
            x_lft,  # (3,) left finger tip position
            x_rft,  # (3,) right finger tip position
            R_lft,  # (3,3) left finger tip orientation
            R_rft,  # (3,3) right finger tip orientation
            R_ldipf, # (3,3) left dip rotation matrix
            R_rdipf, # (3,3) right dip rotation matrix
            p,      # (3,) base position
            R,      # (3,3) base orientation 
            q_dot,  # (3,) wrist + fingers joint velcity
            w,      # (3,) base angular velocity
            v,      # (3,) base linear velocity
            V,      # (15) generalized velocity
            J_xft,  # (6,15) finger tips position Jacobian
            J_Rft, # (6,15) finger tips rotation Jacobian
            J_p,    # (3,15) base position Jacobian
            x_ft_dot # (6,) finger tips linear velocity
        )

    def tactile_sensors(self, sim):
        gr_data = sim.gr_data
        lf_ca, lf_cf, lf_frame = gr_data.get_contact_data('l_dip')
        rf_ca, rf_cf, rf_frame = gr_data.get_contact_data('r_dip')

        lf_contact_flag = np.linalg.norm(lf_cf) > 0.1
        rf_contact_flag = np.linalg.norm(lf_cf) > 0.1
        return lf_contact_flag, rf_contact_flag, lf_ca, lf_cf, rf_ca, rf_cf, lf_frame, rf_frame

    def grasp_params_sampler(self, sim, xc, xd):
        # object info
        xo = sim.mj_data.body('object').xpos
        Ro = sim.mj_data.body('object').xmat.reshape(3, 3)

        geom_id = mj.mj_name2id(sim.mj_model, mj.mjtObj.mjOBJ_GEOM, "object")
        box_size = sim.mj_model.geom_size[geom_id]

        R_candidates = np.concatenate([Ro, -Ro], axis=1)
        angles = (np.array([0.0, 0.0, 1]).reshape(1, 3)@R_candidates).reshape(6,)
        max_idx = np.argmax(angles)
        if max_idx < 3:
            nhat = Ro[:, max_idx] # (3,)
        else:
            nhat = -Ro[:, max_idx-3]

        max_idx = np.mod(max_idx, 3)
        thats = np.delete(Ro, max_idx-3, axis=1) # (3, 2)
        thats_pm = np.concatenate([thats, -thats], axis=1) # (3, 4)
        max_idx2 = np.argmax((xd.reshape(1, 3)@thats_pm).reshape(4,))
        
        half_grasp_width = np.delete(box_size, max_idx, axis=0)[np.mod(max_idx2, 2)]

        xd_des = thats_pm[:, max_idx2] * (half_grasp_width + 0.01)
        xc_des = xo
        return xc_des, xd_des, nhat

    def vf_reaching_two_fingers(
            self, 
            xc, 
            xd, 
            xc_des, 
            xd_des, 
            nhat, 
            phi_T=2*np.pi, 
            err_T=0.01,
            err_thr=0.03,
            V_tan=1,
            Vc=10,
            Vd=50
            ):
        ehat = (xc_des - xc)/np.clip(np.linalg.norm(xc_des - xc), a_min=1.0e-6, a_max=np.inf)
        phi = np.arccos((ehat*nhat).sum())
        weight = np.exp(-phi/phi_T)

        xc_dot_des = weight * (xc_des - xc) + V_tan * (nhat - ehat*(ehat*nhat).sum()) * (1 - weight)

        error = np.linalg.norm((xc_des - xc))
        weight_err = np.tanh((error - err_thr)/err_T)
        xd_dot_des = (1 + weight_err)/2 * (2*xd_des - xd) + (1 - weight_err)/2 *(0.8*xd_des - xd)
        
        x_lft_dot_des = Vc*xc_dot_des + Vd*xd_dot_des 
        x_rft_dot_des = Vc*xc_dot_des - Vd*xd_dot_des

        x_ft_dot_des = np.hstack((x_lft_dot_des, x_rft_dot_des))
        return x_ft_dot_des
    
    def vf_hand_orientation_matching(self, p, xc_des, nhat, V_tan=3):
        ehat = (xc_des - p)/np.clip(np.linalg.norm(xc_des - p), a_min=1.0e-6, a_max=np.inf)
        p_dot_des = (nhat - ehat*(ehat*nhat).sum()) * V_tan
        return p_dot_des
    
    def vf_finger_pose(self, q, q_thr=30*np.pi/180, weight=2*np.array([0, 1, 0, 1, 1, 1, 0, 1, 1])):
        q_dot_des = np.zeros_like(q)

        def attracting_field(x_star, x):
            e = np.abs(x_star - x)
            return (x_star - x) * np.max(e - 10*np.pi/180, 0)

        q_dot_des[1] = q[5] - q[1]
        q_dot_des[3] = attracting_field(-q_thr, q[3])
        q_dot_des[4] = attracting_field(-q_thr, q[4])
        
        q_dot_des[5] = q[1] - q[5]
        q_dot_des[7] = attracting_field(q_thr, q[7])
        q_dot_des[8] = attracting_field(q_thr, q[8])
        return q_dot_des * weight

    def vf_finger_normal_force(self, lf_cp, rf_cp, V=5):
        return V*np.concatenate([lf_cp, rf_cp])
    
    # def vf_finger_orientation(self, R_ldipf, R_rdipf, lf_frame, rf_frame, x_lft, x_rft, V=5):
    #     lf_cp = R_ldipf @ lf_frame[:3, 2]
    #     rf_cp = R_rdipf @ rf_frame[:3, 2]

    #     e = x_rft - x_lft
    #     ehat = e/np.linalg.norm(e)

    #     inner_prod = (lf_cp*ehat).sum()
    #     lf_w = np.cross(lf_cp, ehat) * np.arccos(inner_prod)
    #     rf_w = np.cross(rf_cp, -ehat) * np.arccos(inner_prod)
    #     return V*np.concatenate([lf_w, rf_w])
        
    def vf_lifting_up_and_rotating(self, xc, Vc=10, vel_thr=10):
        xc_dot_des = Vc*(np.array([0.0, 0.0, 0.3]) - xc)
        x_ft_dot_des = np.hstack((xc_dot_des, xc_dot_des))
        if np.linalg.norm(x_ft_dot_des) > vel_thr:
            x_ft_dot_des = x_ft_dot_des/np.linalg.norm(x_ft_dot_des) * vel_thr
        return x_ft_dot_des
        
    def update(self, sim, printstr=False):
        gr_data = sim.gr_data

        # proprioception
        q, x_lft, x_rft, R_lft, R_rft, R_ldipf, R_rdipf, p, R, q_dot, w, v, V, J_xft, J_Rft, J_p, x_ft_dot = self.proprioception(sim)
        if printstr:
            print(f"q (joint): {q}\n")  
            print(f"x_lft (left finter tip): {x_lft}\n")
            print(f"x_rft (right finter tip): {x_rft}\n")
            print(f"R_lft (left finger tip rotation matrix):\n{R_lft}\n")
            print(f"R_rft (right finger tip rotation matrix):\n{R_rft}\n")
            print(f"R_ldipf (left dip rotation matrix):\n{R_ldipf}\n")
            print(f"R_rdipf (right dip rotation matrix):\n{R_rdipf}\n")
            print(f"p (position): {p}\n")
            print(f"R (rotation matrix):\n{R}\n")
            print(f"q_dot (joint velocities): {q_dot}\n")
            print(f"w (angular velocity): {w}\n")
            print(f"v (linear velocity): {v}\n")
            print(f"V (generalized velocity): {V}\n")
            print(f"J_xft (finger tip Jacobian):\n{J_xft}\n")
            print(f"J_Rft (finger tip rotation Jacobian):\n{J_Rft}\n")
            print(f"J_p (position Jacobian):\n{J_p}\n")
            print(f"x_ft_dot (finger tip velocity): {x_ft_dot}\n")

        xc = 0.5 * (x_lft + x_rft)
        xd = 0.5 * (x_lft - x_rft)
        
        if printstr:
            print(f"xc (center of finger tips): {xc}\n")
            print(f"xd (difference of finger tips): {xd}\n")

        # tactile
        lf_contact_flag, rf_contact_flag, lf_ca, lf_cf, rf_ca, rf_cf, lf_frame, rf_frame = self.tactile_sensors(sim)
        lf_cp = R_ldipf @ lf_frame[:3, 2] # left finger contact normal in world frame
        rf_cp = R_rdipf @ rf_frame[:3, 2] # right finger contact normal in world frame

        # finite state machine
        # check antipodal grasp condition
        stable_grasp_flag = True 
        if lf_contact_flag or rf_contact_flag:
            if stable_grasp_flag:
                # check slip condition
                slip_flag = False
                if slip_flag:
                    self.state = 'slipping'
                else:
                    self.state = 'stable_grasp'
            else:
                self.state = 'regrasping'                
        else:
            self.state = 'reaching'
        print(f"state: {self.state}\n")

        if printstr:
            print(f"lf_ca (left finger contact points): {lf_ca}\n")
            print(f"lf_cf (left finger contact forces): {lf_cf}\n")
            print(f"rf_ca (right finger contact points): {rf_ca}\n")
            print(f"rf_cf (right finger contact forces): {rf_cf}\n")

        
        
        ######################################
        ######################################
        # grasping velocity field controller #
        ######################################
        ######################################

        if self.state == 'reaching':
            xc_des, xd_des, nhat = self.grasp_params_sampler(sim, xc, xd)
            self.xc_des = xc_des
            self.xd_des = xd_des
            self.nhat = nhat
        
        ## velocity field construction
        if self.state == 'reaching':
            # finger tips reaching
            x_ft_dot_des = self.vf_reaching_two_fingers(xc, xd, self.xc_des, self.xd_des, self.nhat)
            f_fingers = J_xft.T@(x_ft_dot_des - x_ft_dot)

            # finger pose
            q_dot_des = self.vf_finger_pose(q)
            f_finger_pose = np.hstack([np.zeros(6,), q_dot_des])

            # hand orientation
            p_dot_des = self.vf_hand_orientation_matching(p, self.xc_des, self.nhat)
            f_orientation = (J_p.T@p_dot_des.reshape(3, 1)).flatten()
            
            f_ctrl = f_fingers  + f_orientation + f_finger_pose 

        elif self.state == 'stable_grasp':
            # normal force
            vf_finger_normal_force = self.vf_finger_normal_force(lf_cp, rf_cp)
            f_contact_following = J_xft.T@vf_finger_normal_force

            # lfiting and moving
            x_ft_dot_des = self.vf_lifting_up_and_rotating(xc)
            f_fingers = J_xft.T@(x_ft_dot_des - x_ft_dot)

            f_ctrl = f_contact_following + f_fingers

        # damping 
        f_damping = - np.array([10, 10, 10, 1, 1, 1, 1.5, 0.1, 0.1, 0.1, 0.01, 0.1, 0.1, 0.1, 0.01])*V
        
        # gravity + coriolis compansation
        f_qrfc_bias = gr_data.qrfc_bias # (v, w, q)

        # sum
        generalized_force = f_ctrl + f_damping + f_qrfc_bias
        tau_ff = generalized_force[6:]
        F_ff = generalized_force[:6]
        
        # set positions, Kp gains will be set in firmware
        gr_data.set_q_des(gr_data.all_idxs,   q)
        gr_data.set_qd_des(gr_data.all_idxs,  q_dot)
        gr_data.set_tau_ff(gr_data.all_idxs,  tau_ff)
        gr_data.set_kp(gr_data.all_idxs,      gr_data.get_kp_default(gr_data.all_idxs))
        gr_data.set_kd(gr_data.all_idxs,      gr_data.get_kd_default(gr_data.all_idxs))
        gr_data.set_F_ff(F_ff)