import time
import numpy as np
import select
import sys

import mujoco as mj
from brl_gripper.utils.lie import skew, unskew, expso3, logSO3

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
        self.q_nominal = np.array([0., 0., 0, -30*np.pi/180, -30*np.pi/180, 0., 0, 30*np.pi/180, 30*np.pi/180]) 
        self.normal_force_des = 0.1
        self.contact_history = [False] * 5
        self.normal_max = 10

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
        J_R = gr_data.kinematics['base']['JacR'] # (3, 15)
        J_pR = np.concatenate((J_p, J_R), axis=0) # 6 x 15
        
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
            J_R,    # (3,15) base rotation Jacobian
            J_pR, # (6,15) base Jacobian
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
        # This is only for box-shaped object
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
            err_thr=0.04,
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
    
    def vf_hand_position_matching(self, p, xc_des, nhat, V=3):
        ehat = (xc_des - p)/np.clip(np.linalg.norm(xc_des - p), a_min=1.0e-6, a_max=np.inf)
        p_dot_des = (nhat - ehat*(ehat*nhat).sum())
        return V * p_dot_des
    
    def vf_finger_pose(self, xc, xc_des, q, error_thr=0.1, V=10):
        error = np.linalg.norm((xc_des - xc))
        q_dot_des = np.zeros_like(q)
        if error > error_thr:
            q_dot_des = V * (self.q_nominal - q)
        else:
            q_dot_des[1] = q[5] - q[1]
            q_dot_des[5] = q[1] - q[5]
        return V * q_dot_des
       
    def vf_finger_normal_force(self, lf_cp, rf_cp, V=5):
        return V*np.concatenate([lf_cp, rf_cp])
        
    def vf_lifting_up(self, xc, V=2):
        xc_dot_des = (np.array([0.0, 0.0, 0.5]) - xc)
        x_ft_dot_des = np.hstack((xc_dot_des, xc_dot_des))
        return V*x_ft_dot_des

    def vf_repulsion_from_table(self, xlf, xrf, x_lft_dot, x_rft_dot, z_equi=0.01, V=1, T=0.05):
        x3_lft_dot_des = np.exp(-(xlf[-1]-z_equi)/T) * (0 - x_lft_dot[-1])
        x3_rft_dot_des = np.exp(-(xrf[-1]-z_equi)/T) * (0 - x_rft_dot[-1])
        x3_ft_dot_delta = np.hstack((x3_lft_dot_des, x3_rft_dot_des))
        return V*x3_ft_dot_delta # (2, 15)

    def slip_flag(self, lf_cf, rf_cf, mu_thr=0.6):
        lf_slip_flag = lf_cf[0]**2 + lf_cf[1]** 2 > mu_thr**2 * lf_cf[-1]**2
        rf_slip_flag = rf_cf[0]**2 + rf_cf[1]** 2 > mu_thr**2 * rf_cf[-1]**2
        return lf_slip_flag or rf_slip_flag

    def contact_classifier(self, x_lft, x_rft, z_equi=0.01, thr = 0.0001):
        if np.abs(x_lft[-1] - z_equi) < thr or np.abs(x_rft[-1] - z_equi) < thr:
            return 'table'
        else:
            return 'object'
    
    def apply_velocity_clipping(self, V, value):
        V = np.clip(V, -value, value)
        return V
    
    def update(self, sim, printstr=False):
        gr_data = sim.gr_data

        # proprioception
        q, x_lft, x_rft, \
            R_lft, R_rft, R_ldipf, R_rdipf, \
                p, R, q_dot, w, v, V, \
                    J_xft, J_Rft, J_p, J_R, J_pR, \
                        x_ft_dot = self.proprioception(sim)

        xc = 0.5 * (x_lft + x_rft)
        xd = 0.5 * (x_lft - x_rft)

        # tactile
        lf_contact_flag, rf_contact_flag, lf_ca, lf_cf, rf_ca, rf_cf, lf_frame, rf_frame = self.tactile_sensors(sim)
        lf_cp = R_ldipf @ lf_frame[:3, 2] # left finger contact normal in world frame
        rf_cp = R_rdipf @ rf_frame[:3, 2] # right finger contact normal in world frame

        # contact flag update
        if lf_contact_flag or rf_contact_flag:
            self.contact_history = self.contact_history[1:] + [True]
        else:
            self.contact_history = self.contact_history[1:] + [False]
        contact_flag = any(self.contact_history)
        print(contact_flag)

        # finite state machine
        if not contact_flag:
            self.state = 'reaching'
        else:
            # check contact with table or object
            contact_type = self.contact_classifier(x_lft, x_rft)
            if contact_type == 'table':
                self.state = 'table_contact_while_reaching'
                print(f'lf_contact_force: {np.linalg.norm(lf_cf)} \n')
                print(f'rf_contact_force: {np.linalg.norm(rf_cf)} \n')
            else:
                # check antipodal grasp condition
                stable_grasp_flag = True 
                if not stable_grasp_flag:
                    self.state = 'regrasping'      
                else:
                    # check slip condition
                    slip_flag = self.slip_flag(lf_cf, rf_cf)
                    if not slip_flag:
                        self.state = 'stable_grasp'
                    else:
                        self.state = 'slipping'
        print(f"state: {self.state}\n")
        
        ######################################
        ######################################
        # grasping velocity field controller #
        ######################################
        ######################################

        ## velocity field construction
        if self.state == 'reaching' or self.state == 'table_contact_while_reaching':
            k_reaching = 1
            k_repulsion = 0.5
            k_hand_position = 0.5
            k_finger_pose = 0.5

            # We assume that we can sample a new grasp pose only while reaching
            # Otherwise, there is likely a self-occulsion, making vision-based grasp sample difficult
            xc_des, xd_des, nhat = self.grasp_params_sampler(sim, xc, xd)
            self.xc_des = xc_des
            self.xd_des = xd_des
            self.nhat = nhat

            # reinitialize normal force desired
            self.normal_force_des = 0.1

            # finger tips reaching + repulsion from table
            x_ft_dot_des = self.vf_reaching_two_fingers(xc, xd, self.xc_des, self.xd_des, self.nhat, Vc=10, Vd=100)
            f_fingers = k_reaching*J_xft.T@(x_ft_dot_des - x_ft_dot)
 
            # Note this is not a velocity, but a force #
            x3_ft_dot_delta = self.vf_repulsion_from_table(x_lft, x_rft, x_ft_dot, x_ft_dot, V=1)
            f_repulsion = k_repulsion*J_xft[[2, 5]].T@x3_ft_dot_delta
            ############################################

            # hand position matching
            p_dot_des = self.vf_hand_position_matching(p, self.xc_des, self.nhat, V=3)
            f_position = k_hand_position*J_p.T@(p_dot_des - v)
            print(f"v: {v} \n")

            # finger pose
            q_dot_des = self.vf_finger_pose(xc, self.xc_des, q, V=5)
            f_finger_pose = k_finger_pose*(np.hstack([np.zeros(6,), q_dot_des]) - np.hstack([np.zeros(6,), q_dot]))

            f_ctrl = f_fingers + f_repulsion + f_position + f_finger_pose

            # if self.state == 'table_contact_while_reaching':
            #     vf_contact_compensation = -np.concatenate([lf_cf[-1]*lf_cp, rf_cf[-1]*rf_cp]) 
            #     f_ctrl += J_xft.T@vf_contact_compensation
        else:
            if self.state == 'stable_grasp':
                k_normal = 1
                k_lifting = 1

                # normal force
                vf_finger_normal_force = self.vf_finger_normal_force(lf_cp, rf_cp, V=self.normal_force_des)
                f_contact_following = k_normal*J_xft.T@(vf_finger_normal_force - x_ft_dot)
                
                # lfiting and moving
                x_ft_dot_des = self.vf_lifting_up(xc)
                f_fingers = k_lifting*J_xft.T@(x_ft_dot_des - x_ft_dot)

                f_ctrl = f_contact_following + f_fingers

            elif self.state == 'slipping':
                k_normal = 1

                if slip_flag:
                    if not self.normal_force_des >= self.normal_max:
                        self.normal_force_des += 0.1
                    else:
                        print("slipping, but max normal force 10N reached")

                # normal force
                vf_finger_normal_force = self.vf_finger_normal_force(lf_cp, rf_cp, V=self.normal_force_des)
                f_contact_following = k_normal*J_xft.T@(vf_finger_normal_force - x_ft_dot)
                f_ctrl = f_contact_following

            elif self.state == 'regrasping':
                pass
        
        # damping 
        f_damping = - np.array([
            10, 10, 10, 
            1, 1, 1, 
            1.5, 
            0.001, 0.001, 0.001, 0.001,
            0.001, 0.001, 0.001, 0.001])*V
        
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