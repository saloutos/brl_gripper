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
        self.observation_error = kwargs.get('observation_error', False)

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

        # init time
        self.init_time = sim.time() 

        ## Addede by YHLee
        # Notation
        # p: base position (base means floating_2 in mujoco)
        # R: base orientation (base means floating_2 in mujoco)
        # q: joints (wrist + fingers, 9-dim)
        # x_lft: left finger tip position in world frame
        # x_rft: right finger tip position in world frame
        # x_ft: finger tips position in world frame (6-dim)
        # xc: center of finger tips (x_lft + x_rft)/2
        # xd: difference of finger tips (x_lft - x_rft)/2
        
        # Sensors
        # lf_ca: not used
        # lf_cf: left finger contact force (described in the contact frame)
        # rf_ca: not used 
        # rf_cf: right finger contact force (described in the contact frame)
        # lf_frame: left finger contact frame (z axis is the contact normal)
        # rf_frame: right finger contact frame (z axis is the contact normal)
        # lf_cp: left finger contact normal in world frame
        # rf_cp: right finger contact normal in world frame

        # Grasp pose
        # xo: grasp position (center of the finger)
        # nhat: grasping direction (reaching direction, finger should direct in -nhat direction)
        # xd: grasp width vector (difference of finger tips)

        # Finite State Machine
        # - reaching
        # - table_contact_while_reaching
        # - grasp_evaluation
        # - stable_grasp
        # - slipping
        # - regrasping

        # set initial state
        self.state = 'reaching'
        self.q_nominal = np.array([
            0., 
            0., 15*np.pi/180, -30*np.pi/180, -30*np.pi/180, 
            0., -15*np.pi/180, 30*np.pi/180, 30*np.pi/180
            ]) # This is encouraged during the reaching state
        self.normal_force_des = 0.1 # initial normal force desired (will increase as slip is detected)
        self.contact_history = [False] * 10 # for robust contact detection
        self.normal_max = 20 # maximum contact normal force for each finger
        self.contact_history_for_regrasp2reach = [False] * 100 # for state transition from regrasping to reaching
        self.stable_grasp_flag = [False] * 10 # if we detect stable grasp five times in a row, we lift it

        self.home_position = np.array([0.0, 0.0, 0.5])
        self.home_orientation = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        
    def proprioception(self, sim):
        gr_data = sim.gr_data

        # proprioception
        q = gr_data.get_q(gr_data.all_idxs) # joints (wrist + fingers)
        x_lft= gr_data.kinematics['l_dip_tip']['p'] # left fingertip in world frame
        x_rft = gr_data.kinematics['r_dip_tip']['p'] # right fingertip in world frame
        R_lft = gr_data.kinematics['l_dip_tip']['R'] # Not used
        R_rft = gr_data.kinematics['r_dip_tip']['R'] # Not used
        R_ldipf = gr_data.kinematics['l_dip_force']['R'] # orientation of left dip (where tactile sensor is attached) from world frame
        R_rdipf = gr_data.kinematics['r_dip_force']['R'] # orientation of right dip (where tactile sensor is attached) from world frame
        
        p = gr_data.get_p() # base position
        R = gr_data.get_R() # base orientation

        q_dot = gr_data.get_qd(gr_data.all_idxs) # joint velocities (wrist + fingers)
        w = gr_data.get_w() # base angular velocity 
        v = gr_data.get_v() # base linear velocity
        V_all = np.hstack((v, w, q_dot)) # generalized velocity

        J_xlft = gr_data.kinematics['l_dip_tip']['Jacp'] # 3 x 15
        J_xrft = gr_data.kinematics['r_dip_tip']['Jacp'] # 3 x 15
        J_xft = np.concatenate((J_xlft, J_xrft), axis=0) # 6 x 15
        J_Rlft = gr_data.kinematics['l_dip_tip']['JacR'] # Not used
        J_Rrft = gr_data.kinematics['r_dip_tip']['JacR'] # Not used
        J_Rft = np.concatenate((J_Rlft, J_Rrft), axis=0) # Not used

        J_p= gr_data.kinematics['base']['Jacp'] # (3, 15)
        J_R = gr_data.kinematics['base']['JacR'] # (3, 15)
        J_pR = np.concatenate((J_p, J_R), axis=0) # 6 x 15
        
        x_ft_dot = (J_xft@V_all.reshape(15, 1)).flatten() # finger tips linear velocity
        return (
            q,       # (9,) wrist + fingers joints
            x_lft,   # (3,) left finger tip position
            x_rft,   # (3,) right finger tip position
            R_lft,   # (3,3) not used
            R_rft,   # (3,3) not used
            R_ldipf, # (3,3) left dip rotation matrix (where tactile sensor is attached)
            R_rdipf, # (3,3) right dip rotation matrix (where tactile sensor is attached)
            p,       # (3,) base position
            R,       # (3,3) base orientation 
            q_dot,   # (3,) wrist + fingers joint velcity
            w,       # (3,) base angular velocity
            v,       # (3,) base linear velocity
            V_all,       # (15) generalized velocity
            J_xft,   # (6,15) finger tips position Jacobian
            J_Rft,   # (6,15) not used
            J_p,     # (3,15) base position Jacobian
            J_R,     # (3,15) base orientation Jacobian
            J_pR,    # (6,15) base Jacobian
            x_ft_dot # (6,) finger tips linear velocity
        )

    def tactile_sensors(self, sim):
        gr_data = sim.gr_data
        lf_ca, lf_cf, lf_frame = gr_data.get_contact_data('l_dip')
        rf_ca, rf_cf, rf_frame = gr_data.get_contact_data('r_dip')

        lf_contact_flag = np.linalg.norm(lf_cf) > 0.1
        rf_contact_flag = np.linalg.norm(lf_cf) > 0.1
        return lf_contact_flag, rf_contact_flag, lf_ca, lf_cf, rf_ca, rf_cf, lf_frame, rf_frame

    def grasp_params_sampler(self, sim, obs_err=False):
        # object info
        xo = sim.mj_data.body('object').xpos
        Ro = sim.mj_data.body('object').xmat.reshape(3, 3)
        geom_id = mj.mj_name2id(sim.mj_model, mj.mjtObj.mjOBJ_GEOM, "object")

        obj_type = sim.mj_model.geom_type[geom_id] # 0: plane, 1: hfield, 2: sphere, 3: capsule, 4: cylinder, 5: mesh, 6: sdf 
        if obj_type == 6: # box
            box_size = sim.mj_model.geom_size[geom_id]
            x_gw = box_size[0] + 0.01 # x_hat_half_grasp_width
            y_gw = box_size[1] + 0.01 # y_hat_half_grasp_width
            z_gw = box_size[2] + 0.01 # z_hat_half_grasp_width

            xo_candidates = np.zeros((24, 3))
            nhat_candidates = np.zeros((24, 3))
            xd_candidates = np.zeros((24, 3))

            ###
            xhat = Ro[:, 0:1].T # (1, 3)
            yhat = Ro[:, 1:2].T # (1, 3)
            zhat = Ro[:, 2:3].T # (1, 3)

            xo_candidates = np.concatenate([xo.reshape(-1, 3)]*24, axis=0)
            nhat_candidates = np.concatenate([
                xhat, xhat, xhat, xhat,
                -xhat, -xhat, -xhat, -xhat, 
                yhat, yhat, yhat, yhat,
                -yhat, -yhat, -yhat, -yhat,
                zhat, zhat, zhat, zhat,
                -zhat, -zhat, -zhat, -zhat
            ], axis=0) # (24, 3)
            xd_candidates = np.concatenate([
                yhat*y_gw, -yhat*y_gw, zhat*z_gw, -zhat*z_gw,
                yhat*y_gw, -yhat*y_gw, zhat*z_gw, -zhat*z_gw,
                xhat*x_gw, -xhat*x_gw, zhat*z_gw, -zhat*z_gw,
                xhat*x_gw, -xhat*x_gw, zhat*z_gw, -zhat*z_gw,
                xhat*x_gw, -xhat*x_gw, yhat*y_gw, -yhat*y_gw,
                xhat*x_gw, -xhat*x_gw, yhat*y_gw, -yhat*y_gw,
            ], axis=0) # (24, 3)
        elif obj_type == 5: # cylinder
            cylinder_size = sim.mj_model.geom_size[geom_id]
            radius = cylinder_size[0]

            r_gw = radius + 0.01 # half_grasp_width

            xhat = Ro[:, 0:1].T # (1, 3)
            yhat = Ro[:, 1:2].T # (1, 3)
            zhat = Ro[:, 2:3].T # (1, 3)

            xo_candidates = np.zeros((36, 3))
            nhat_candidates = np.zeros((36, 3))
            xd_candidates = np.zeros((36, 3))

            xo_candidates = np.concatenate([xo.reshape(-1, 3)]*36, axis=0)
            nhat_candidates = np.concatenate([[zhat[0]]*18, [-zhat[0]]*18], axis=0)
            for i in range(18):
                theta = i * (2 * np.pi / 18)
                cos_theta = np.cos(theta)
                sin_theta = np.sin(theta)
                xd_candidates[i:i+1, :] = r_gw * (cos_theta * xhat + sin_theta * yhat)
                xd_candidates[i+18:i+19, :] = xd_candidates[i:i+1, :]
        else:
            raise NotImplementedError
        
        if obs_err:
            noise_dir = np.cross(xd_candidates/np.linalg.norm(xd_candidates, axis=1).reshape(-1, 1), nhat_candidates) 
            xo_candidates += 0.3*np.linalg.norm(xd_candidates, axis=1).reshape(-1, 1)*noise_dir

        return xo_candidates, nhat_candidates, xd_candidates

    def select_nearest_grasp(self, xc, xd, xo_candidates, nhat_candidates, xd_candidates, desired_grasping_direction=np.array([0, 0, 1])):
        dist_xc = np.linalg.norm(xo_candidates - xc.reshape(1, 3), axis=1) # (n,)
        xd = xd.reshape(1, 3)/np.linalg.norm(xd) # (1, 3)
        xd_hat_candidates = xd_candidates/np.linalg.norm(xd_candidates, axis=1).reshape(-1, 1) # (n, 3)
        dist_xd_angle = - (xd_hat_candidates*xd).sum(axis=1) # (n,)
        dist_nhat_angle = -(nhat_candidates*desired_grasping_direction.reshape(1, -1)).sum(axis=1) # (n,)

        # lower the better
        score = 100*dist_nhat_angle + 10*dist_xd_angle + dist_xc
        min_idx = np.argmin(score)
        return xo_candidates[min_idx], nhat_candidates[min_idx], xd_candidates[min_idx] # (3, ), (3, ), (3, )
    
    def vf_reaching(
            self, 
            xc, # (3,)
            xd, # (3,)
            xc_des, # (3,)
            xd_des, # (3,)
            nhat, # (3,)
            phi_T=0.2*np.pi, 
            err_T=0.02,
            err_thr=0.01,
            Vphi=1,
            Vc=10,
            Vd=100,
            Ad_max=1.5,
            Ad_min=0.5
            ):
        # compute finger tip center reaching error
        e = xc_des - xc
        error = np.linalg.norm((xc_des - xc))
        ehat = e/np.clip(error, a_min=1.0e-6, a_max=np.inf)

        # velocity of the finger tip center (weighted sum of reaching and aligning ehat and nhat) 
        phi = np.arccos(np.clip((ehat*nhat).sum(), a_min=-1, a_max=1)) # -pi ~ pi (pi: aligned, 0: perpendicular)
        weight_phi = np.exp(-(np.pi - phi)/phi_T)
        xc_dot_des = weight_phi * (xc_des - xc) + Vphi * (nhat - ehat*(ehat*nhat).sum()) * (1 - weight_phi)

        # velocity of the finger tip difference (depends on the finger tip center error)
        weight_err = np.tanh((error - err_thr)/err_T)
        xd_dot_des = (1 + weight_err)/2 * (Ad_max*xd_des - xd) + (1 - weight_err)/2 *(Ad_min*xd_des - xd)
        
        # coordinate transformation
        x_lft_dot_des = Vc*xc_dot_des + Vd*xd_dot_des 
        x_rft_dot_des = Vc*xc_dot_des - Vd*xd_dot_des
        return np.hstack((x_lft_dot_des, x_rft_dot_des)) # (6,)
    
    def vf_top_down_grasp(self, p, xc_des, nhat, V=1):
        # align the base position onto the nhat
        e = xc_des - p
        error = np.linalg.norm(e)
        ehat = e/np.clip(error, a_min=1.0e-6, a_max=np.inf)
        p_dot_des = (nhat - ehat*(ehat*nhat).sum())
        return V * p_dot_des
    
    def vf_finger_pose(self, xc, xc_des, q, err_thr=0.1, err_T=0.001, V=1):
        error = np.linalg.norm((xc_des - xc))
        weight = np.exp(-np.clip(error - err_thr, a_min=0, a_max=np.inf)/err_T)
        v1 = (self.q_nominal - q)
        
        v2 = np.zeros_like(q)
        v2[1] = q[5] - q[1]
        v2[5] = q[1] - q[5]
        q_dot_des = (1 - weight) * v1 + weight * v2

        return V * q_dot_des
       
    def vf_finger_normal_force(self, lf_cp, rf_cp, V=5):
        return V*np.concatenate([lf_cp, rf_cp])
        
    def vf_to_home(self, p, R, V=1):
        v_des = (self.home_position - p)
        w_des = 0.1*unskew(logSO3(self.home_orientation@R.T))
        return V*np.concatenate([v_des, w_des])

    def slip_flag(self, lf_cf, rf_cf, mu_thr=0.5):
        # being  convervative by setting mu smaller that actual value
        lf_slip_flag = lf_cf[0]**2 + lf_cf[1]** 2 > mu_thr**2 * lf_cf[-1]**2
        rf_slip_flag = rf_cf[0]**2 + rf_cf[1]** 2 > mu_thr**2 * rf_cf[-1]**2
        return lf_slip_flag or rf_slip_flag

    def contact_classifier(self, x_lft, x_rft, z_equi=0.01, thr = 0.0001):
        # this is needed to distinguish between table and object contact
        # we should not apply normal force to the table
        if np.abs(x_lft[-1] - z_equi) < thr or np.abs(x_rft[-1] - z_equi) < thr:
            return 'table'
        else:
            return 'object'
    
    def project2prevent_impulse_to_table(
            self, 
            x_ft=None, 
            x_ft_dot=None,
            p_dot=None,
            q_dot=None,
            z_equi=0.01, 
            err_T=0.02,
            type='finget_tips',
            *args,
            **kwargs
            ):
        l_err = np.clip(x_ft[2] - z_equi, a_min=0, a_max=np.inf)
        r_err = np.clip(x_ft[5] - z_equi, a_min=0, a_max=np.inf)
        l_weight = np.exp(-l_err/err_T)
        r_weight = np.exp(-r_err/err_T)

        if type == 'finget_tips':
            x_ft_dot[2] = (1 - l_weight)*x_ft_dot[2]
            x_ft_dot[5] = (1 - r_weight)*x_ft_dot[5]
            return x_ft_dot
        elif type == 'base':
            weight = np.maximum(l_weight, r_weight)
            p_dot[2] = (1 - weight)*p_dot[2]
            return p_dot
        elif type == 'joint':
            Jl = kwargs.get('Jac_q2x_lft_z')
            Jr = kwargs.get('Jac_q2x_rft_z')
            q_dot = q_dot - l_weight * (Jl*q_dot).sum()/(Jl*Jl).sum() * Jl
            q_dot = q_dot - r_weight * (Jr*q_dot).sum()/(Jr*Jr).sum() * Jr 
            return q_dot

    def stable_grasp_classifier(self, lf_cp, rf_cp, x_lft, x_rft, angle_thr=5 * np.pi/180):
        vec = (x_rft - x_lft)
        unit_vec = vec/np.linalg.norm(vec)
        unit_lf_cp = lf_cp/np.linalg.norm(lf_cp)
        unit_rf_cp = rf_cp/np.linalg.norm(rf_cp)
        
        l_angle = np.arccos(np.clip((unit_lf_cp*unit_vec).sum(), a_min=-1, a_max=1))
        r_angle = np.arccos(np.clip(-(unit_rf_cp*unit_vec).sum(), a_min=-1, a_max=1))
        
        if l_angle < angle_thr and r_angle < angle_thr:
            return True
        else:
            print(f"left finger angle to contact normal: {l_angle * 180/np.pi}")
            print(f"right finger angle to contact normal: {r_angle * 180/np.pi}")
            return False

    def transition2contact_states(self, lf_cp, rf_cp, x_lft, x_rft, lf_cf, rf_cf, printstr=False):
        stable_grasp_flag = self.stable_grasp_classifier(lf_cp, rf_cp, x_lft, x_rft)
        if not stable_grasp_flag:
            if printstr:
                print(f"{self.state} -> regrasping\n")
            self.state = 'regrasping'
        else:
            slip_flag = self.slip_flag(lf_cf, rf_cf)
            if slip_flag:
                if printstr:
                    print(f"{self.state} -> slipping\n")
                self.state = 'slipping'
            else:
                if printstr:
                    print(f"{self.state} -> stable_grasp\n")
                self.state = 'stable_grasp'

    def finite_state_machine(
            self, contact_flag, x_lft, x_rft, lf_cp, rf_cp, lf_cf, rf_cf, printstr=False):
        if printstr:
            print(f"contact_flag: {contact_flag}")
        if self.state == 'reaching' or self.state == 'table_contact_while_reaching':
            if not contact_flag:
                if printstr:
                    print(f"{self.state} -> reaching\n")
                self.state = 'reaching'
            else:
                contact_type = self.contact_classifier(x_lft, x_rft)
                if contact_type == 'table':
                    if printstr:
                        print(f"{self.state} -> table_contact_while_reaching\n")
                    self.state = 'table_contact_while_reaching'
                    if printstr:
                        print(f'lf_contact_force: {np.linalg.norm(lf_cf)} \n')
                        print(f'rf_contact_force: {np.linalg.norm(rf_cf)} \n')
                else:
                    self.transition2contact_states(lf_cp, rf_cp, x_lft, x_rft, lf_cf, rf_cf, printstr=printstr)
                
        elif self.state == 'regrasping':
            if not any(self.contact_history_for_regrasp2reach):
                if printstr:
                    print(f"{self.state} -> reaching\n")
                self.state = 'reaching'
            else:
                self.transition2contact_states(lf_cp, rf_cp, x_lft, x_rft, lf_cf, rf_cf, printstr=printstr)

        elif self.state == 'stable_grasp' or self.state == 'slipping':
            if not contact_flag:
                if printstr:
                    print(f"{self.state} -> reaching\n")
                self.state = 'reaching'
            else:
                self.transition2contact_states(lf_cp, rf_cp, x_lft, x_rft, lf_cf, rf_cf, printstr=printstr)
        if printstr:
            print(f"state: {self.state}\n")

        if self.state == 'stable_grasp':
            self.stable_grasp_flag = self.stable_grasp_flag[1:] + [True]
        else:
            self.stable_grasp_flag = self.stable_grasp_flag[1:] + [False]

    def clip_velocity(self, v, v_max):
        v_norm = np.linalg.norm(v)
        if v_norm > v_max:
            return v/v_norm * v_max
        else:
            return v
    
    def update(self, sim):
        gr_data = sim.gr_data

        # proprioception
        q, x_lft, x_rft, \
            R_lft, R_rft, R_ldipf, R_rdipf, \
                p, R, q_dot, w, v, V_all, \
                    J_xft, J_Rft, J_p, J_R, J_pR, \
                        x_ft_dot = self.proprioception(sim)

        x_ft = np.hstack((x_lft, x_rft)) # (6,)
        xc = 0.5 * (x_lft + x_rft)
        xd = 0.5 * (x_lft - x_rft)

        # tactile
        lf_contact_flag, rf_contact_flag, lf_ca, lf_cf, rf_ca, rf_cf, lf_frame, rf_frame = self.tactile_sensors(sim)
        lf_cp = R_ldipf @ lf_frame[:3, 2] # left finger contact normal in world frame
        rf_cp = R_rdipf @ rf_frame[:3, 2] # right finger contact normal in world frame

        # contact flag update
        if lf_contact_flag or rf_contact_flag:
            self.contact_history = self.contact_history[1:] + [True] # for robust conatct detection
            self.contact_history_for_regrasp2reach = self.contact_history_for_regrasp2reach[1:] + [True]
        else:
            self.contact_history = self.contact_history[1:] + [False]
            self.contact_history_for_regrasp2reach = self.contact_history_for_regrasp2reach[1:] + [False]
        contact_flag = any(self.contact_history)

        print(f"[New Iteration] Simulation Time: {sim.time() - self.init_time:.4f} sec")
        self.finite_state_machine(contact_flag, x_lft, x_rft, lf_cp, rf_cp, lf_cf, rf_cf, printstr=True)
        
        ######################################
        ######################################
        # grasping velocity field controller #
        ######################################
        ######################################

        ## velocity field construction
        if self.state == 'reaching' or self.state == 'table_contact_while_reaching':
            k_reaching = 1
            k_top_down_grasp = 1
            k_finger_pose = 0.5
            
            # reinitialize normal force desired
            self.normal_force_des = 0.1

            # We assume that we can sample a new grasp pose only while reaching
            # Otherwise, there is likely a self-occulsion, making vision-based grasp sample difficult
            xo_candidates, nhat_candidates, xd_candidates = self.grasp_params_sampler(
                sim, obs_err=self.observation_error)
            xc_des, nhat, xd_des = self.select_nearest_grasp(
                xc, 
                xd, 
                xo_candidates, 
                nhat_candidates, 
                xd_candidates,
                desired_grasping_direction=np.array([0, 0, 1])
                )
            self.xc_des = xc_des
            self.xc_des_updated = np.copy(xc_des)
            self.xd_des = xd_des
            self.xd_des_updated = np.copy(xd_des)
            self.nhat = nhat

            # finger tips reaching
            x_ft_dot_des = self.vf_reaching(
                xc, xd, self.xc_des, self.xd_des, self.nhat, 
                Vc=10, Vd=100, err_T=0.005, err_thr=0.01,
                Ad_max=1.5, Ad_min=0.0
            )
            x_ft_dot_des = self.project2prevent_impulse_to_table(x_ft=x_ft, x_ft_dot=x_ft_dot_des, type='finget_tips')
            x_ft_dot_des = self.clip_velocity(x_ft_dot_des, 5)
            f_fingers = k_reaching*J_xft.T@(x_ft_dot_des - x_ft_dot)
 
            # hand position matching
            p_dot_des = self.vf_top_down_grasp(p, self.xc_des, self.nhat, V=3)
            p_dot_des = self.project2prevent_impulse_to_table(p_dot=p_dot_des, x_ft=x_ft, type='base')
            f_position = k_top_down_grasp*J_p.T@(p_dot_des - v)

            # finger pose
            q_dot_des = self.vf_finger_pose(xc, self.xc_des, q, V=3)
            q_dot_des = self.project2prevent_impulse_to_table(
                q_dot=q_dot_des, x_ft=x_ft, type='joint', Jac_q2x_lft_z=J_xft[2, 6:], Jac_q2x_rft_z=J_xft[5, 6:])
            f_finger_pose = k_finger_pose*(np.hstack([np.zeros(6,), q_dot_des]) - np.hstack([np.zeros(6,), q_dot]))

            f_ctrl = f_fingers + f_position + f_finger_pose

        elif self.state == 'stable_grasp':
            k_normal = 1
            k_lifting_p = 100
            k_lifting_R = 1

            # reset xc_des_updated and xd_des_updated
            # this is important part!!
            self.xc_des_updated = (x_lft + x_rft)/2
            self.xd_des_updated = (x_lft - x_rft)/2

            # normal force
            vf_finger_normal_force = self.vf_finger_normal_force(lf_cp, rf_cp, V=self.normal_force_des)
            f_contact_following = k_normal*J_xft.T@(vf_finger_normal_force - x_ft_dot)
            
            # lfiting and moving
            base_V_dot_des = self.vf_to_home(p, R)
            f_fingers = J_pR.T@(np.array([
                k_lifting_p, k_lifting_p, k_lifting_p, k_lifting_R, k_lifting_R, k_lifting_R
            ])*(base_V_dot_des - V_all[:6]))

            if all(self.stable_grasp_flag):
                f_ctrl = f_contact_following + f_fingers
            else:
                f_ctrl = f_contact_following

        elif self.state == 'slipping':
            k_normal = 1

            if not self.normal_force_des >= self.normal_max:
                self.normal_force_des += 0.1
            else:
                print(f"slipping, but max normal force {self.normal_max}N reached")

            # normal force
            vf_finger_normal_force = self.vf_finger_normal_force(lf_cp, rf_cp, V=self.normal_force_des)
            f_contact_following = k_normal*J_xft.T@(vf_finger_normal_force - x_ft_dot)
            f_ctrl = f_contact_following

        elif self.state == 'regrasping':
            k_regrasping = 1

            ##########################################################
            ##########################################################
            # these are the most important parameters 
            # that should be tuned (for the regrasping state)
            # err_T should be around 0.05 ~ 0.005
            # step_size should be around 0.0001 ~ 0.00001
            # Ad_max should be around 2.5 ~ 1.5
            # Ad_min should be around 0.2 ~ 0.5
            step_size = 0.0001
            err_T = 0.01
            Ad_max = 1.5
            Ad_min = 0.5
            ##########################################################
            ##########################################################

            if lf_contact_flag and rf_contact_flag:
                nl = -lf_cp
                nr = -rf_cp
                vec = x_rft - x_lft
                tl = vec - (vec*nr).sum()*nr
                tl = tl - (tl*nl).sum()*nl
                tl = tl/np.linalg.norm(tl)
                tr = -vec + (vec*nl).sum()*nl
                tr = tr - (tr*nr).sum()*nr
                tr = tr/np.linalg.norm(tr)
                self.xc_des_updated += step_size*(tl + tr) 

            x_ft_dot_des = self.vf_reaching(
                xc, xd, self.xc_des_updated, self.xd_des_updated, 
                self.nhat, Vc=10, Vd=100, err_T=err_T, Ad_max=Ad_max, Ad_min=Ad_min) 

            f_ctrl = k_regrasping*J_xft.T@(x_ft_dot_des - x_ft_dot) 

        # damping 
        f_damping = - np.array([
            10, 10, 10, 
            1, 1, 1, 
            1.5, 
            0.001, 0.001, 0.001, 0.001,
            0.001, 0.001, 0.001, 0.001])*V_all
        
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