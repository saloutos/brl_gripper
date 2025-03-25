# imports
import numpy as np
from brl_gripper.GripperData import GripperData

class RigdBodyData:
    def __init__(self, name=""):
        self.name = name
        self.p = np.zeros((3,))
        self.R = np.eye(3)
        self.v = np.zeros((3,))
        self.w = np.zeros((3,))
        self.F = np.zeros((6,))
        self.p_des = np.zeros((3,))
        self.R_des = np.eye(3)
        self.v_des = np.zeros((3,))
        self.w_des = np.zeros((3,))
        self.F_ff = np.zeros((6,))
        self.kp = np.zeros((6,))
        self.kd = np.zeros((6,))
        self.F_command = np.zeros((6,))
        # default values for contollers to use
        self.p_des_default = np.zeros((3,))
        self.R_des_default = np.eye(3)
        self.kp_default = np.zeros((6,))
        self.kd_default = np.zeros((6,))

    def update_control(self):
        # only feedforward currently
        self.F_command = self.F_ff
        return self.F_command
    
    def log_data(self):
        # TODO
        return []
    
    def log_header(self):
        # TODO
        return []

# gripper data class
class GripperDataV2(GripperData):
    def __init__(self):
        super().__init__()
        self.base = RigdBodyData("0_base")
        self.kinematics['l_dip_tip'] = {'p':np.zeros((3,)), 'R':np.eye(3), 'Jacp':np.zeros((3,15)), 'JacR':np.zeros((3,15))}
        self.kinematics['r_dip_tip'] =  {'p':np.zeros((3,)), 'R':np.eye(3), 'Jacp':np.zeros((3,15)), 'JacR':np.zeros((3,15))}
        
        self.kinematics['base'] = {'p':np.zeros((3,)), 'R':np.eye(3), 'Jacp':np.zeros((3,15)), 'JacR':np.zeros((3,15))}
        self.qrfc_bias = np.zeros((15,))

    def get_contact_data(self, var_name):
        angle = self.sensors[var_name].__dict__['contact_angle']
        force = self.sensors[var_name].__dict__['contact_force']
        frame = self.sensors[var_name].__dict__['T_sensor_contact']
        return angle, force, frame

    def get_rb_data(self, var_name):
        return self.base.__dict__[var_name]
    
    def get_p(self):
        return self.get_rb_data('p')
    
    def get_R(self):
        return self.get_rb_data('R')

    def get_w(self):
        return self.get_rb_data('w')
    
    def get_v(self):
        return self.get_rb_data('v')
    
    def set_p(self, data):
        self.base.__dict__['p'] = data

    def set_R(self, data):
        self.base.__dict__['R'] = data
    
    def set_F_ff(self, data):
        self.base.__dict__['F_ff'] = data

    def get_F_command(self):
        return self.base.F_command