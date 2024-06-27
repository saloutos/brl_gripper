import time
import numpy as np

# define controller class
class PotentialFieldsDemo:

    def __init__(self):
        self.name = "Potential Fields Demo"
        self.started = False

        self.glide_thresh = 0.090   # glide threshold
        self.glide_dist =   0.060   # desired contact distance for gliding/contour following
        self.avoid_thresh = 0.070   # avoidance threshold
        # eps fudge factor for gliding distance
        self.glide_eps = 0.020

        # gains for potential fields
        self.Kp_pot_outer =     125.0
        self.Kp_pot_forward =   300.0
        self.Kp_pot_inner =     300.0
        self.Kd_pot =           1.0

        # world link angles
        self.l_tip_W_seen = 0.0
        self.r_tip_W_seen = 0.0

    def begin(self, gr_data):
        self.started = True
        self.count = 0
        # set finger defaults for control
        # gr_data.set_q_des_default(gr_data.l_idxs,   np.array([0.0, 0.8, -0.8, 0.0]))
        # gr_data.set_q_des_default(gr_data.r_idxs,   np.array([0.0, -0.8, 0.8, 0.0]))
        gr_data.set_q_des_default(gr_data.l_idxs,   np.array([0.0, 0.4, -1.1, 1.2]))
        gr_data.set_q_des_default(gr_data.r_idxs,   np.array([0.0, -0.4, 1.1, -1.2]))
        gr_data.set_kp_default(gr_data.l_idxs,      np.array([8.0, 2.5, 2.5, 2.5]))
        gr_data.set_kp_default(gr_data.r_idxs,      np.array([8.0, 2.5, 2.5, 2.5]))
        gr_data.set_kd_default(gr_data.l_idxs,      np.array([0.05, 0.05, 0.05, 0.05]))
        gr_data.set_kd_default(gr_data.r_idxs,      np.array([0.05, 0.05, 0.05, 0.05]))
        # set wrist defaults for control
        gr_data.set_q_des_default(gr_data.w_idxs,   np.zeros((1,)))
        gr_data.set_kp_default(gr_data.w_idxs,      np.array([3.0]))
        gr_data.set_kd_default(gr_data.w_idxs,      np.array([0.1]))

    def update(self, gr_data): # TODO: could pass mj_model, mj_data here if necessary...would need to be careful though!
        # run controller

        # reset controller values
        ql_des = gr_data.get_q_des_default(gr_data.l_idxs)
        taul_des = np.zeros((4,))
        kpl = gr_data.get_kp_default(gr_data.l_idxs)
        kdl = gr_data.get_kd_default(gr_data.l_idxs)
        qr_des = gr_data.get_q_des_default(gr_data.r_idxs)
        taur_des = np.zeros((4,))
        kpr = gr_data.get_kp_default(gr_data.r_idxs)
        kdr = gr_data.get_kd_default(gr_data.r_idxs)

        # get current finger states
        ql_cur = gr_data.get_q(gr_data.l_idxs)
        qdl_cur = gr_data.get_qd(gr_data.l_idxs)
        pl_cur = gr_data.kinematics['l_dip_tip']['p']
        Rl_cur = gr_data.kinematics['l_dip_tip']['R']
        Jl_cur = gr_data.kinematics['l_dip_tip']['Jacp']

        qr_cur = gr_data.get_q(gr_data.r_idxs)
        qdr_cur = gr_data.get_qd(gr_data.r_idxs)
        pr_cur = gr_data.kinematics['r_dip_tip']['p']
        Rr_cur = gr_data.kinematics['r_dip_tip']['R']
        Jr_cur = gr_data.kinematics['r_dip_tip']['Jacp']

        # calculate fingertip velocities
        vl_cur = Jl_cur @ qdl_cur
        vr_cur = Jr_cur @ qdr_cur

        # retrieve sensor data
        # NOTE: dist[0], dist[2] face forward; dist[1], dist[3] face inward; dist[4] faces outward
        l_dist = gr_data.sensors['l_dip'].dist
        r_dist = gr_data.sensors['r_dip'].dist

        # calculate new gliding thresholds
        # NOTE: using defaults for now
        # TODO: fix and re-implement these functions!!! could do this with ray-casting in mujoco now?
        l_glide_thresh = self.glide_thresh #self.calc_left_glide_threshold(pl_cur, Rl_cur, qr_cur)
        l_glide_dist = self.glide_dist #l_glide_thresh*0.5
        r_glide_thresh = self.glide_thresh #self.calc_right_glide_threshold(pr_cur, Rr_cur, ql_cur)
        r_glide_dist = self.glide_dist #r_glide_thresh*0.5

        # if threshold is activated, use potential fields
        if self.check_threshold(l_dist, l_glide_thresh):
            # reduce Kp gains
            kpl[1:3] = 0.05*kpl[1:3]
            # reset force value
            Fl = np.zeros((3,))
            # force from outward sensor, along +Y of tip frame
            if (l_dist[4]<self.avoid_thresh):
                Fl += self.Kp_pot_outer * (l_dist[4]-self.avoid_thresh) * Rl_cur[:,1]
            # force from forward sensors, along +X of tip frame
            if (l_dist[0]<self.avoid_thresh) and (l_dist[2]<self.avoid_thresh):
                Fl += self.Kp_pot_forward * ((0.5*(l_dist[0]+l_dist[2]))-self.avoid_thresh) * Rl_cur[:,0]
            elif (l_dist[0]<self.avoid_thresh):
                Fl += self.Kp_pot_forward * (l_dist[0]-self.avoid_thresh) * Rl_cur[:,0]
            elif (l_dist[2]<self.avoid_thresh):
                Fl += self.Kp_pot_forward * (l_dist[2]-self.avoid_thresh) * Rl_cur[:,0]
            # force from inner sensors, along -Y of tip frame
            if (l_dist[1]<l_glide_thresh) and (l_dist[3]<l_glide_thresh):
                Fl += self.Kp_pot_inner * ((0.5*(l_dist[1]+l_dist[3]))-l_glide_dist) * -Rl_cur[:,1]
            elif (l_dist[1]<l_glide_thresh):
                Fl += self.Kp_pot_inner * (l_dist[1]-l_glide_dist) * -Rl_cur[:,1]
            elif (l_dist[3]<l_glide_thresh):
                Fl += self.Kp_pot_inner * (l_dist[3]-l_glide_dist) * -Rl_cur[:,1]
            # add damping at fingertip
            Fl += -self.Kd_pot * vl_cur
            # convert to feedforward torques for MCP, PIP
            taul_des[1:3] = Jl_cur[:,1:3].T @ Fl
            # calculate desired angle for link 3
            ql_des[3] = self.l_tip_W_seen - ql_cur[1] - ql_cur[2]

        if self.check_threshold(r_dist, r_glide_thresh):
            # reduce Kp gains
            kpr[1:3] = 0.05*kpr[1:3]
            # reset force value
            Fr = np.zeros((3,))
            # force from outward sensor, along -Y of tip frame
            if (r_dist[4]<self.avoid_thresh):
                Fr += self.Kp_pot_outer * (r_dist[4]-self.avoid_thresh) * -Rr_cur[:,1]
            # force from forward sensors, along +X of tip frame
            if (r_dist[0]<self.avoid_thresh) and (r_dist[2]<self.avoid_thresh):
                Fr += self.Kp_pot_forward * ((0.5*(r_dist[0]+r_dist[2]))-self.avoid_thresh) * Rr_cur[:,0]
            elif (r_dist[0]<self.avoid_thresh):
                Fr += self.Kp_pot_forward * (r_dist[0]-self.avoid_thresh) * Rr_cur[:,0]
            elif (r_dist[2]<self.avoid_thresh):
                Fr += self.Kp_pot_forward * (r_dist[2]-self.avoid_thresh) * Rr_cur[:,0]
            # force from inner sensors, along +Y of tip frame
            if (r_dist[1]<r_glide_thresh) and (r_dist[3]<r_glide_thresh):
                Fr += self.Kp_pot_inner * ((0.5*(r_dist[1]+r_dist[3])) - r_glide_dist) * Rr_cur[:,1]
            elif (r_dist[1]<r_glide_thresh):
                Fr += self.Kp_pot_inner * (r_dist[1] - r_glide_dist) * Rr_cur[:,1]
            elif (r_dist[3]<r_glide_thresh):
                Fr += self.Kp_pot_inner * (r_dist[3] - r_glide_dist) * Rr_cur[:,1]
            # add damping at fingertip
            Fr += -self.Kd_pot * vr_cur
            # convert to feedforward torques for MCP, PIP
            taur_des[1:3] = Jr_cur[:,1:3].T @ Fr
            # calculate desired angle for link 3
            qr_des[3] = self.r_tip_W_seen - qr_cur[1] - qr_cur[2]

        # set finger commands
        gr_data.set_q_des(gr_data.l_idxs, ql_des)
        gr_data.set_qd_des(gr_data.l_idxs, np.zeros((4,)))
        gr_data.set_tau_ff(gr_data.l_idxs, taul_des)
        gr_data.set_kp(gr_data.l_idxs, kpl)
        gr_data.set_kd(gr_data.l_idxs, kdl)
        gr_data.set_q_des(gr_data.r_idxs, qr_des)
        gr_data.set_qd_des(gr_data.r_idxs, np.zeros((4,)))
        gr_data.set_tau_ff(gr_data.r_idxs, taur_des)
        gr_data.set_kp(gr_data.r_idxs, kpr)
        gr_data.set_kd(gr_data.r_idxs, kdr)

        # set wrist roll commands
        gr_data.set_q_des(gr_data.w_idxs, gr_data.get_q_des_default(gr_data.w_idxs))
        gr_data.set_qd_des(gr_data.w_idxs, np.zeros((1,)))
        gr_data.set_tau_ff(gr_data.w_idxs, np.zeros((1,)))
        gr_data.set_kp(gr_data.w_idxs, gr_data.get_kp_default(gr_data.w_idxs))
        gr_data.set_kd(gr_data.w_idxs, gr_data.get_kd_default(gr_data.w_idxs))

        # set wrist cartesian position
        new_x_pos = min( -0.3 + self.count*(1/5000), 0.05)
        gr_data.kinematics['base_des']['p']  = np.array([new_x_pos, 0.0, 0.05])
        gr_data.kinematics['base_des']['R']  = np.eye(3)
        self.count += 1

    ### helper functions

    # checking activation thresholds
    def check_threshold(self, d, glide_thresh):
        return ((d[0]<self.avoid_thresh) or (d[2]<self.avoid_thresh) or (d[4]<self.avoid_thresh) or (d[1]<glide_thresh) or (d[3]<glide_thresh))

    # helper functions for checking self-intersection
    def normal_between_points(self, p1,p2):
        # normal vector from p1 to p2
        return (p2-p1)/np.linalg.norm((p2-p1))

    def check_internal_vector(self, n1, n2, ntest):
        # check if normal vector ntest sits between n1 and n2
        n1 = n1.flatten()
        n2 = n2.flatten()
        ntest = ntest.flatten()
        return ((np.cross(n1,n2)*np.cross(n1,ntest)>=0) and (np.cross(n2,n1)*np.cross(n2,ntest)>=0))