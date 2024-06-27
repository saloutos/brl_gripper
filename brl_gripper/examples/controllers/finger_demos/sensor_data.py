import time
import numpy as np

# define controller class
class SensorDataDemo:

    def __init__(self):
        self.name = "Sensor Data Demo"
        self.started = False

    def begin(self, gr_data):
        self.started = True

    def update(self, gr_data):

        # TODO: check for user inputs?

        # run controller

        # TODO: better print formatting here
        print("\033c", end="")
        print("ToF data:")
        print(gr_data['l_dip'].tof_raw)
        print(gr_data['l_pip'].tof_raw, gr_data['l_mcp'].tof_raw)
        print(gr_data['palm'].tof_raw)
        print(gr_data['r_mcp'].tof_raw, gr_data['r_pip'].tof_raw)
        print(gr_data['r_dip'].tof_raw)
        print("Contact data:")
        print(gr_data['l_dip'].contact_force, gr_data['l_dip'].contact_angle)
        print(gr_data['l_pip'].fsr_raw, gr_data['l_mcp'].fsr_raw)
        print(gr_data['palm'].fsr_raw)
        print(gr_data['r_mcp'].fsr_raw, gr_data['r_pip'].fsr_raw)
        print(gr_data['r_dip'].contact_force, gr_data['r_dip'].contact_angle)
        print("")

        # set fingers and wrist to default positions
        gr_data.set_q_des(gr_data.all_idxs,   np.zeros((9,)))
        gr_data.set_qd_des(gr_data.all_idxs,  np.zeros((9,)))
        gr_data.set_tau_ff(gr_data.all_idxs,  np.zeros((9,)))
        gr_data.set_kp(gr_data.all_idxs,      np.zeros((9,)))
        gr_data.set_kd(gr_data.all_idxs,      np.zeros((9,)))

        # set wrist cartesian position