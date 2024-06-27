import time
import numpy as np

import time
import numpy as np

# define controller class
class PosTrajDemo:

    def __init__(self):
        self.name = "Position Trajectories Demo"
        self.started = False

    def begin(self, gr_data):

        self.dt = (1.0/300.0) #0.001
        self.time_step = 0
        self.dxl_time = 0
        self.freq1 = 1.0
        self.freq2 = self.freq1/2.0
        self.amp1 = (np.pi/180.0)*20.0
        self.amp2 = (np.pi/180.0)*12.0
        self.idx_freq = 0
        self.max_freq = 22
        self.updown = True

        self.started = True

        # set finger defaults for control
        gr_data.set_kp_default(gr_data.l_idxs,      np.array([8.0, 8.0, 8.0, 8.0]))
        gr_data.set_kp_default(gr_data.r_idxs,      np.array([8.0, 8.0, 8.0, 8.0]))
        gr_data.set_kd_default(gr_data.l_idxs,      np.array([0.05, 0.05, 0.05, 0.05]))
        gr_data.set_kd_default(gr_data.r_idxs,      np.array([0.05, 0.05, 0.05, 0.05]))
        # set wrist defaults for control
        gr_data.set_q_des_default(gr_data.w_idxs,   np.zeros((1,)))
        gr_data.set_kp_default(gr_data.w_idxs,      np.array([3.0]))
        gr_data.set_kd_default(gr_data.w_idxs,      np.array([0.1]))

    def update(self, gr_data):

        # TODO: check for user inputs?

		# calculate trajectories here
        if (self.time_step%1200== 0): #4000
            if (self.freq1 < self.max_freq and self.updown):
                self.freq1 = self.freq1 + 1
            else:
                self.updown = False
                self.freq1 = self.freq1 - 1
                if (self.freq1 < 2):
                    self.updown = True
            self.freq2 = self.freq1 / 2.0

        self.time_step = self.time_step + 1

        ql = np.zeros((4,))
        ql[0] = -(self.amp2)*(np.sin(self.freq2*np.pi*self.dt*self.time_step))
        ql[1] = -0.5*(self.amp1)*(-1.0*np.cos(self.freq1*np.pi*self.dt*self.time_step)+1.0)
        ql[2] = -0.5*(self.amp1)*(-1.0*np.cos(self.freq1*np.pi*self.dt*self.time_step)+1.0)
        ql[3] = -0.5*(self.amp1)*(-1.0*np.cos(self.freq1*np.pi*self.dt*self.time_step)+1.0)

        qr = np.zeros((4,))
        qr[0] = (self.amp2)*(np.sin(self.freq2*np.pi*self.dt*self.time_step))
        qr[1] = 0.5*(self.amp1)*(-1.0*np.cos(self.freq1*np.pi*self.dt*self.time_step)+1.0)
        qr[2] = 0.5*(self.amp1)*(-1.0*np.cos(self.freq1*np.pi*self.dt*self.time_step)+1.0)
        qr[3] = 0.5*(self.amp1)*(-1.0*np.cos(self.freq1*np.pi*self.dt*self.time_step)+1.0)

        # print some data
        print("\033c", end="")
        print("Positions:")
        print(gr_data.get_q(gr_data.l_idxs))
        print(gr_data.get_q(gr_data.r_idxs))
        print("Velocities:")
        print(gr_data.get_qd(gr_data.l_idxs))
        print(gr_data.get_qd(gr_data.r_idxs))
        print("Desired:")
        print(ql)
        print(qr)
        print("")

        # set positions, Kp gains will be set in firmware
        gr_data.set_q_des(gr_data.l_idxs,   ql)
        gr_data.set_q_des(gr_data.r_idxs,   qr)
        #  wrist roll is zeroed
        gr_data.set_q_des(gr_data.w_idxs,   np.array([0.0]))
        # everything else is default
        gr_data.set_qd_des(gr_data.all_idxs,  np.zeros((9,)))
        gr_data.set_tau_ff(gr_data.all_idxs,  np.zeros((9,)))
        gr_data.set_kp(gr_data.all_idxs,      gr_data.get_kp_default(gr_data.all_idxs))
        gr_data.set_kd(gr_data.all_idxs,      gr_data.get_kd_default(gr_data.all_idxs))

        # set wrist cartesian position
        gr_data.kinematics['base_des']['p'] = np.array([0.0, 0.0, 0.2])
        gr_data.kinematics['base_des']['R'] = np.eye(3)
