# imports
import time
import can
import sys
import yaml
import numpy as np
import csv
import math
from datetime import datetime as dt
import select
import os
from enum import Enum
import copy

import mujoco as mj
import mujoco.viewer as mjv

import open3d as o3d

from .utils.can_utils import *
from .GripperData import *
from .GripperData_with_controllable_wrist import *
from .utils import UTILS_DIR
from .assets import ASSETS_DIR

from .GripperPlatform import PlatformMode, HardwareEnable, GripperPlatform

# define the Gripper Platform class
class GripperPlatformV2(GripperPlatform):
    def __init__(self, mj_model, viewer_enable=True, hardware_enable=HardwareEnable.NO_HW, setup_rendering=False, log_path=None):
        super().__init__(mj_model, viewer_enable=viewer_enable, hardware_enable=hardware_enable, setup_rendering=setup_rendering, log_path=log_path)
        self.gr_data = GripperDataV2()

        print("Gripper data V2 initialized.")

        # general init for logging
        self.log_enable = (log_path is not None)
        if self.log_enable:
            # TODO: better log name convention here?
            self.log_name = log_path+"log_"+str(dt.now()).replace(" ", "_")+".csv"
            self.log_header = ['t']+self.gr_data.log_header()
            self.log = [ [0]+self.gr_data.log_data() ]
            self.log_file = open(self.log_name, mode='w')
            self.log_writer = csv.writer(self.log_file, delimiter=',')
            self.log_writer.writerows([self.log_header])
            self.log_start = 0.0 # will udpate this in initialize later
            print("Logging enabled.")

    def sync_data(self):
        # check for user input
        self.check_user_input()
        # if hardware is enabled, update mj_data
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.HW_NO_VIS:
            # update mj_data from gr_data
            for key in self.gr_data.joints.keys():
                self.mj_data.joint(key).qpos = self.gr_data.joints[key].q
                self.mj_data.joint(key).qvel = self.gr_data.joints[key].qd
                self.mj_data.joint(key).qfrc_applied = self.gr_data.joints[key].tau
            # call mj_forward to update kinematics
            mj.mj_forward(self.mj_model, self.mj_data)
        # always update some kinematic data
        # TODO: best way to store this data?
        # TODO: do we want any other body kinematics?

        l_dip_p = self.mj_data.body('l_dip').xpos
        l_dip_R = self.mj_data.body('l_dip').xmat.reshape((3,3))
        r_dip_p = self.mj_data.body('r_dip').xpos
        r_dip_R = self.mj_data.body('r_dip').xmat.reshape((3,3))
        self.gr_data.kinematics['l_dip']['p'] = l_dip_p
        self.gr_data.kinematics['l_dip']['R'] = l_dip_R
        self.gr_data.kinematics['r_dip']['p'] = r_dip_p
        self.gr_data.kinematics['r_dip']['R'] = r_dip_R

        l_dip_tip_p = self.mj_data.body('l_dip_tip').xpos
        l_dip_tip_R = self.mj_data.body('l_dip_tip').xmat.reshape((3,3))
        r_dip_tip_p = self.mj_data.body('r_dip_tip').xpos
        r_dip_tip_R = self.mj_data.body('r_dip_tip').xmat.reshape((3,3))
        self.gr_data.kinematics['l_dip_tip']['p'] = l_dip_tip_p
        self.gr_data.kinematics['l_dip_tip']['R'] = l_dip_tip_R
        self.gr_data.kinematics['r_dip_tip']['p'] = r_dip_tip_p
        self.gr_data.kinematics['r_dip_tip']['R'] = r_dip_tip_R

        l_dip_force_p = self.mj_data.site('l_dip_force').xpos
        l_dip_force_R = self.mj_data.site('l_dip_force').xmat.reshape((3,3))
        r_dip_force_p = self.mj_data.site('r_dip_force').xpos
        r_dip_force_R = self.mj_data.site('r_dip_force').xmat.reshape((3,3))
        self.gr_data.kinematics['l_dip_force']['p'] = l_dip_force_p
        self.gr_data.kinematics['l_dip_force']['R'] = l_dip_force_R
        self.gr_data.kinematics['r_dip_force']['p'] = r_dip_force_p
        self.gr_data.kinematics['r_dip_force']['R'] = r_dip_force_R

        # added for base
        self.gr_data.base.p = self.mj_data.body('floating_2').xpos
        self.gr_data.base.R = self.mj_data.body('floating_2').xmat.reshape((3,3))
        # self.gr_data.base.w = self.mj_data.body('floating_2').cvel[:3]
        self.gr_data.base.w = self.mj_data.qvel[3:6]
        self.gr_data.base.v = self.mj_data.body('floating_2').cvel[3:]
        # get fingertip jacobians (w.r.t. world frame)
        Jacp = np.zeros((3, self.mj_model.nv))
        JacR = np.zeros((3, self.mj_model.nv))
        
        mj.mj_jac(self.mj_model, self.mj_data, Jacp, JacR, l_dip_tip_p, self.mj_model.body('l_dip_tip').id)
        self.gr_data.kinematics['l_dip_tip']['Jacp'] = Jacp[:,:15].copy()
        self.gr_data.kinematics['l_dip_tip']['JacR'] = JacR[:,:15].copy()
        
        mj.mj_jac(self.mj_model, self.mj_data, Jacp, JacR, r_dip_tip_p, self.mj_model.body('r_dip_tip').id)
        self.gr_data.kinematics['r_dip_tip']['Jacp'] = Jacp[:,:15].copy()
        self.gr_data.kinematics['r_dip_tip']['JacR'] = JacR[:,:15].copy()

        mj.mj_jac(self.mj_model, self.mj_data, Jacp, JacR, self.gr_data.base.p, self.mj_model.body('floating_2').id)
        self.gr_data.kinematics['base']['Jacp'] = Jacp[:,:15].copy()
        self.gr_data.kinematics['base']['JacR'] = JacR[:,:15].copy()
        

        # get coriolis bias + gravity bias for all joints
        # temp_qvel = self.mj_data.qvel.copy()
        # self.mj_data.qvel = np.zeros(self.mj_model.nv)
        self.gr_data.qrfc_bias = self.mj_data.qfrc_bias[:15].copy()
        # self.mj_data.qvel = temp_qvel


        # update sensor kinematics
        # TODO: better way to do this? initialize with corresponding site name for kinematics?
        sites = ['palm_tof', 'l_mcp_tof', 'l_pip_tof', 'l_dip_force', 'r_mcp_tof', 'r_pip_tof', 'r_dip_force']
        for idx, key in enumerate(self.gr_data.sensors.keys()):
            self.gr_data.sensors[key].update_kinematics(self.mj_data.site(sites[idx]).xpos, \
                                                        self.mj_data.site(sites[idx]).xmat.reshape((3,3)))
        # TODO: capture all sensor site kinematics instead?
        # mj_sensors = [self.mj_model.sensor(i).name for i in range(self.mj_model.nsensor)]
        # for key in self.gr_data.sensors.keys():
        #     site_kinematics = {}
        #     for sensor in mj_sensors:
        #         if key in sensor:
        #             new_key = sensor.replace(key,'').replace('_','')
        #             site_kinematics[new_key] = (self.mj_data.sensor(sensor).xpos, \
        #                                         self.mj_data.sensor(sensor).xmat.reshape((3,3)))
        #     self.gr_data.sensors[key].update_kinematics(site_kinematics) # TODO: would need to change this function

        # if simulation is enabled
        if self.mode==PlatformMode.SIM_WITH_VIS or self.mode==PlatformMode.SIM_NO_VIS:
            # fill gr_data from mj_data
            # start with just joint info (q, qd, tau), access by name
            # TODO: should this iterate through model joints first? then gr_data joint keys?
            mj_joints = [self.mj_model.joint(i).name for i in range(self.mj_model.njnt)]
            for key in self.gr_data.joints.keys():
                self.gr_data.joints[key].q = self.mj_data.joint(key).qpos
                self.gr_data.joints[key].qd = self.mj_data.joint(key).qvel
                self.gr_data.joints[key].tau = self.mj_data.joint(key).qfrc_actuator
            
            # get contact location data for fingertips
            # TODO: should we just do this for phalanges too?
            # TODO: this is now stored above, should just use that?
            l_contact = []
            r_contact = []
            l_contact_R = self.mj_data.site('l_dip_force').xmat.reshape((3,3))
            r_contact_R = self.mj_data.site('r_dip_force').xmat.reshape((3,3))
            nc = len(self.mj_data.contact)
            for c_idx in range(nc):
                # create list of all contact points for each fingertip
                # contact positions are measured from center of fingertip, in world coordinate frame
                # so, apply fingertip R to get position in fingertip frame, then normalize to unit vector
                if self.mj_data.contact[c_idx].geom[0] == self.mj_data.geom('l_dip_tip').id \
                        or self.mj_data.contact[c_idx].geom[1] == self.mj_data.geom('l_dip_tip').id:
                    new_contact_u = l_contact_R.T @ (self.mj_data.contact[c_idx].pos - l_dip_tip_p)
                    new_contact_u = new_contact_u / np.linalg.norm(new_contact_u)
                    l_contact.append(new_contact_u)
                if self.mj_data.contact[c_idx].geom[0] == self.mj_data.geom('r_dip_tip').id \
                        or self.mj_data.contact[c_idx].geom[1] == self.mj_data.geom('l_dip_tip').id:
                    new_contact_u = r_contact_R.T @ (self.mj_data.contact[c_idx].pos - r_dip_tip_p)
                    new_contact_u = new_contact_u / np.linalg.norm(new_contact_u)
                    r_contact.append(new_contact_u)
            # fill in sensor data from mj_data
            mj_sensors = [self.mj_model.sensor(i).name for i in range(self.mj_model.nsensor)]
            for key in self.gr_data.sensors.keys():
                sim_data_dict = {}
                for sensor in mj_sensors:
                    if key in sensor:
                        # store that sensor's data in a dict for this key
                        new_key = sensor.replace(key,'').replace('_','')
                        sim_data_dict[new_key] = self.mj_data.sensor(sensor).data
                # TODO: for contact data, hard-code key here and add to dict? better way to do this?
                if key == 'l_dip':
                    # TODO: take average contact position from list?
                    # TODO: add a contact flag?
                    sim_data_dict['R'] = l_dip_tip_R
                    if len(l_contact)>0:
                        sim_data_dict['contact'] = l_contact[0]
                    else:
                        sim_data_dict['contact'] = np.array([0.0, 0.0, 1.0])
                elif key=='r_dip':
                    sim_data_dict['R'] = r_dip_tip_R
                    if len(r_contact)>0:
                        sim_data_dict['contact'] = r_contact[0]
                    else:
                        sim_data_dict['contact'] = np.array([0.0, 0.0, 1.0])
                self.gr_data.sensors[key].update_raw_data_from_sim(sim_data_dict)
            #  TODO: fill in any other data from sim?
        # update sensor data (i.e. apply filters, etc.)
        self.gr_data.process_all_sensor_data()

    def apply_control(self):
        # update internal value of tau_command for each joint
        self.gr_data.update_all_joint_control()
        self.gr_data.base.update_control()

        # # if hardware mode
        # if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.HW_NO_VIS:
        #     # TODO: could just pass gr_data and idxs to the pack_command function?
        #     left_finger_msg = self.pack_joints(self.gr_data.get_q_des(self.gr_data.l_idxs),
        #                                     self.gr_data.get_qd_des(self.gr_data.l_idxs),
        #                                     self.gr_data.get_kp(self.gr_data.l_idxs),
        #                                     self.gr_data.get_kd(self.gr_data.l_idxs),
        #                                     self.gr_data.get_tau_ff(self.gr_data.l_idxs))
        #     self.CAN_bus_1.send(can.Message(arbitration_id=LEFT_COMMAND, dlc=48, data=left_finger_msg, is_fd=True, is_extended_id=False))
        #     right_finger_msg = self.pack_joints(self.gr_data.get_q_des(self.gr_data.r_idxs),
        #                                     self.gr_data.get_qd_des(self.gr_data.r_idxs),
        #                                     self.gr_data.get_kp(self.gr_data.r_idxs),
        #                                     self.gr_data.get_kd(self.gr_data.r_idxs),
        #                                     self.gr_data.get_tau_ff(self.gr_data.r_idxs))
        #     self.CAN_bus_1.send(can.Message(arbitration_id=RIGHT_COMMAND, dlc=48, data=right_finger_msg, is_fd=True, is_extended_id=False))
        #     if self.wrist_enable:
        #         wrist_msg = self.pack_wrist(self.gr_data.get_q_des(self.gr_data.w_idxs),
        #                                     self.gr_data.get_qd_des(self.gr_data.w_idxs),
        #                                     self.gr_data.get_kp(self.gr_data.w_idxs),
        #                                     self.gr_data.get_kd(self.gr_data.w_idxs),
        #                                     self.gr_data.get_tau_ff(self.gr_data.w_idxs))
        #         self.CAN_bus_2.send(can.Message(arbitration_id=WRIST_ID, data=wrist_msg, is_extended_id=False))

        # if simulation mode
        if self.mode==PlatformMode.SIM_WITH_VIS or self.mode==PlatformMode.SIM_NO_VIS:
            # update actuator commands based on gr_data tau_command, will be applied during next mj_step call
            self.mj_data.ctrl[-6:] = self.gr_data.get_F_command()
            self.mj_data.ctrl[:-6] = self.gr_data.get_tau_command(self.gr_data.all_idxs)
