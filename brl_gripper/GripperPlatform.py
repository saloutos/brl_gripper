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

import mujoco as mj
import mujoco.viewer as mjv

from .utils.can_utils import *
from .GripperData import *
from .utils import UTILS_DIR
from .assets import ASSETS_DIR

# define possible platform modes
class PlatformMode(Enum):
    HW_WITH_VIS = 0
    HW_NO_VIS   = 1
    SIM_ONLY    = 2

# define hardware enable modes
class HardwareEnable(Enum):
    NO_HW          = 0 #[False, False]
    FINGERS_ONLY   = 1 #[True, False]
    WRIST_ONLY     = 2 #[False, True] # just for completeness
    FINGERS_WRIST  = 3 #[True, True]

# TODO: other defines here? other parameters or constants?

# define the Gripper Platform class
class GripperPlatform:
    def __init__(self, mj_model, viewer_enable=True, hardware_enable=HardwareEnable.NO_HW, log_path=None):
        # based on enable flags, set platform mode
        # TODO: might not need to save flags as class variables, should use mode for everything from here onwards?
        self.viewer_enable = viewer_enable
        if hardware_enable==HardwareEnable.NO_HW:
            self.hardware_enable = False
            self.wrist_enable = False
        elif hardware_enable==HardwareEnable.FINGERS_ONLY:
            self.hardware_enable = True
            self.wrist_enable = False
        elif hardware_enable==HardwareEnable.FINGERS_WRIST:
            self.hardware_enable = True
            self.wrist_enable = True
        else:
            print("Invalid hardware enable flags. Defaulting to simulation only.")
            self.hardware_enable = False
            self.wrist_enable = False
        # default to simulation only
        self.mode = PlatformMode.SIM_ONLY
        if self.hardware_enable:
            if self.viewer_enable:
                self.mode = PlatformMode.HW_WITH_VIS
            else:
                self.mode = PlatformMode.HW_NO_VIS

        # load mujoco model and data
        self.mj_model = mj_model
        self.mj_data = mj.MjData(self.mj_model)

        # gripper data init
        # TODO: if GripperData() takes list of joints and sensors as arguments, then pass them here
        self.gr_data = GripperData()

        # general init for platform
        self.paused = False
        self.current_t = 0.0
        self.last_view_t = 0.0
        self.last_control_t = 0.0
        self.sim_dt = 0.001
        self.mj_model.opt.timestep = self.sim_dt # re-assign in initialize for safety
        self.view_dt = 0.0333 # 30fps default
        self.control_dt = 0.002 # 500Hz default
        self.enforce_real_time_sim = True # default to real-time sim
        self.dt_comp = 0.0
        self.sim_steps_per_control = math.floor(self.control_dt/self.sim_dt) # re-calculate in initialize for safety
        self.run_viewer_sync = False
        self.run_control = False
        self.char_in = None
        self.new_char = False

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

        # specific init for hardware
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.HW_NO_VIS:
            # default mode, but this could change before initialize()
            self.hand_control_mode = HandControlMode.CURRENT_CONTROL
            # start CAN bus
            self.CAN_bus_1 = None
            self.CAN_bus_2 = None
            # start CAN busses
            can_config_filepath = os.path.join(UTILS_DIR, 'can_config.yaml')
            with open(can_config_filepath,'r') as file:
                can_config = yaml.safe_load(file)
            try:
                for PCAN_DEVICE in can.detect_available_configs('pcan'):
                    # first CAN bus for finger board
                    if(PCAN_DEVICE['device_id']==eval(can_config['Interface1']['device_id'])):
                        self.CAN_bus_1 = can.interface.Bus(bustype=can_config['Interface1']['bustype'], channel=can_config['Interface1']['channel'],
                                    device_id=eval(can_config['Interface1']['device_id']), state=eval(can_config['Interface1']['state']),
                                    fd=can_config['Interface1']['fd'], f_clock_mhz=can_config['Interface1']['f_clock_mhz'],
                                    bitrate=can_config['Nominal']['bitrate'], nom_brp=can_config['Nominal']['brp'], data_brp=can_config['Data']['brp'],
                                    nom_tseg1=can_config['Nominal']['tseg1'], nom_tseg2=can_config['Nominal']['tseg2'], nom_sjw=can_config['Nominal']['sjw'],
                                    data_tseg1=can_config['Data']['tseg1'], data_tseg2=can_config['Data']['tseg2'], data_sjw=can_config['Data']['sjw'])
                        print(f"CAN_BUS_1: {PCAN_DEVICE['device_id']}")
                    # second CAN bus for wrist roll motor
                    if self.wrist_enable:
                        if(PCAN_DEVICE['device_id']==eval(can_config['Interface2']['device_id'])):
                            self.CAN_bus_2 = can.interface.Bus(bustype=can_config['Interface2']['bustype'], channel=can_config['Interface2']['channel'],
                                        device_id=eval(can_config['Interface2']['device_id']), state=eval(can_config['Interface2']['state']),
                                        fd=can_config['Interface2']['fd'], f_clock_mhz=can_config['Interface2']['f_clock_mhz'],
                                        bitrate=can_config['Nominal']['bitrate'], nom_brp=can_config['Nominal']['brp'], data_brp=can_config['Data']['brp'],
                                        nom_tseg1=can_config['Nominal']['tseg1'], nom_tseg2=can_config['Nominal']['tseg2'], nom_sjw=can_config['Nominal']['sjw'],
                                        data_tseg1=can_config['Nominal']['tseg1'], data_tseg2=can_config['Nominal']['tseg2'], data_sjw=can_config['Nominal']['sjw'])
                            print(f"CAN_BUS_2: {PCAN_DEVICE['device_id']}")
                time.sleep(1.0)
            except Exception as e:
                print(f"CAN init failed: {e}")

    def initialize(self):

        # prepare mujoco model
        mj.mj_forward(self.mj_model, self.mj_data)

        # start viewer
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.SIM_ONLY:
            self.mj_viewer = mjv.launch_passive(self.mj_model, self.mj_data, show_left_ui=False, show_right_ui=False, key_callback=self.key_callback)
            with self.mj_viewer.lock():
                # set viewer options here
                self.mj_viewer.opt.frame = mj.mjtFrame.mjFRAME_WORLD
                # can also tweak visualization elements here
                self.mj_viewer.cam.distance = 1.2
                self.mj_viewer.cam.elevation = -15
                self.mj_viewer.cam.azimuth = 120
                self.mj_viewer.cam.lookat = np.array([-0.1, 0.1, 0.15])
                if self.mode==PlatformMode.SIM_ONLY:
                    self.mj_viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = True
                    self.mj_viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True
                    self.mj_viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTSPLIT] = True
                    self.mj_viewer.opt.flags[mj.mjtVisFlag.mjVIS_SELECT] = True
                    self.mj_viewer.opt.flags[mj.mjtVisFlag.mjVIS_PERTFORCE] = False
                    # enable viewing groups (0,1,2 are enabled by default)
                    self.mj_viewer.opt.geomgroup[3] = True
                if self.mode==PlatformMode.HW_WITH_VIS:
                    self.mj_viewer.opt.flags[mj.mjtVisFlag.mjVIS_RANGEFINDER] = False

                mj.mj_forward(self.mj_model, self.mj_data)
            print("Viewer started.")

        # TODO: step simulation to get initial state? or does mj_forward take care of this?
        # mj.step(self.mj_model, self.mj_data, nstep=1)

        # just in case, re-calculate sim steps per control period and update model timestep
        # this should not change after this point
        self.mj_model.opt.timestep = self.sim_dt
        self.sim_steps_per_control = math.floor(self.control_dt/self.sim_dt)

        # send initialize message to hand
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.HW_NO_VIS:
            print("Waiting for gripper startup.")
            self.CAN_bus_1.send(can.Message(arbitration_id=GRIPPER_ENABLE_ID, dlc=48, data=HAND_MODE_MSGS[self.hand_control_mode], is_fd=True, is_extended_id=False))
            if self.wrist_enable:
                self.CAN_bus_2.send(can.Message(arbitration_id=WRIST_ID, data=U6_EnterMotorMode, is_extended_id=False))
                self.CAN_bus_2.send(can.Message(arbitration_id=WRIST_ID, data=U6_Zero, is_extended_id=False))
            enable_time = self.time()
            # NOTE: timed finger init to take just under 8 seconds
            while (self.time() - enable_time < 8.0):
                time.sleep(0.0005)
                # TODO: necessary to update_data here? this was necessary for SPI, may not be for CAN
                self.step()
                time.sleep(0.0005)
            print("Gripper started.")
            # TODO: any sensor init here? offset forces?

        # save log start time
        if self.log_enable:
            self.log_start = self.time()

    def shutdown(self):
        # send disable messages to gripper
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.HW_NO_VIS:
            self.CAN_bus_1.send(can.Message(arbitration_id=GRIPPER_ENABLE_ID, dlc=48, data=HAND_MODE_MSGS[HandControlMode.DISABLE_CONTROL], is_fd=True, is_extended_id=False))
            if self.wrist_enable:
                self.CAN_bus_2.send(can.Message(arbitration_id=WRIST_ID, data=U6_ExitMotorMode, is_extended_id=False))
            print("CAN bus disabled.")
        # TODO: close CAN busses properly?
        # close viewer
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.SIM_ONLY:
            self.mj_viewer.close()
            print("Viewer closed.")
        # log recorded data
        if self.log_enable:
            print("Logging data.")
            self.log_writer.writerows(self.log)
            self.log_file.close()
            print("Log saved.")
        print("Shutdown.")

    def log_data(self, extra_data=None):
        if self.log_enable:
            t_log = self.time() - self.log_start
            log_line = [t_log] + self.gr_data.log_data()
            # TODO: log some sim data?
            if extra_data is not None:
                log_line = log_line + extra_data
            self.log.append(log_line)

    def check_user_input(self):
        # with no viewer, need to check terminal input
        if self.mode==PlatformMode.HW_NO_VIS:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                self.char_in = sys.stdin.read(1)
            else:
                self.char_in = None
        else: # make sure we only receive mujoco callback input once
            if self.new_char==True:
                self.new_char = False
            else:
                self.char_in = None
    def key_callback(self, keycode):
        # handle keyboard inputs
        self.char_in = chr(keycode)
        self.new_char = True
        if self.char_in==' ' and self.mode==PlatformMode.SIM_ONLY: # can only pause in sim
            self.paused = not self.paused # toggle pause
            print(f"Paused: {self.paused}")

    def time(self):
        if self.mode==PlatformMode.SIM_ONLY and not self.enforce_real_time_sim:
            return self.mj_data.time
        else:
            return time.time()

    def sync_viewer(self):
        # TODO: is this check even necessary? might end up redundant
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.SIM_ONLY:
            self.mj_viewer.user_scn.ngeom = 0
            # TODO: update any other visual elements here
            # sync sensor visualizations
            if self.mode==PlatformMode.HW_WITH_VIS:
                self.gr_data.sync_all_sensor_data_to_viewer(self.mj_viewer.user_scn)

            # sync mujoco viewer
            self.mj_viewer.sync()

    def step(self):
        # get current time
        self.previous_t = self.current_t
        self.current_t = self.time()
        # print(self.current_t-self.previous_t)

        # if mode has viewer
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.SIM_ONLY:
            # check if viewer flag needs to be set
            if self.current_t - self.last_view_t > self.view_dt:
                self.last_view_t = self.current_t
                self.run_viewer_sync = True

        # if hardware mode
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.HW_NO_VIS:
            # check if control flag needs to be set
            if self.current_t - self.last_control_t > self.control_dt:
                self.last_control_t = self.current_t
                self.run_control = True
            # poll for CAN messages
            # TODO: maybe move this to a separate function?
            num_tries = 0
            while (num_tries < CAN_MSG_NUM):
                try:
                    # try to receive CAN message
                    can_message_1 = self.CAN_bus_1.recv(0.0)
                    if self.wrist_enable:
                        can_message_2 = self.CAN_bus_2.recv(0.0)
                    else:
                        can_message_2 = None
                    if (can_message_1 is not None):
                        # unpack message
                        if (can_message_1.data[0] == 0 and can_message_1.data[1] == 0 and can_message_1.data[2] == 0 and can_message_1.data[3] == 0 ):
                            print("Error Cleared!")
                        else:
                            if (can_message_1.arbitration_id == MOTOR_DATA):
                                dxl_pos, dxl_vel, dxl_tau = self.unpack_joints(can_message_1.data)
                                self.gr_data.set_q(self.gr_data.finger_idxs, dxl_pos)
                                self.gr_data.set_qd(self.gr_data.finger_idxs, dxl_vel)
                                self.gr_data.set_tau(self.gr_data.finger_idxs, dxl_tau)
                            elif (can_message_1.arbitration_id == SENSOR_DATA):
                                raw_sensor_data = self.unpack_sensors(can_message_1.data)
                                self.gr_data.update_all_raw_sensor_data_from_hw(raw_sensor_data)
                    if (can_message_2 is not None):
                        if(can_message_2.data[0] == 0 and can_message_2.data[1] == 0 and can_message_2.data[2] == 0 and can_message_2.data[3] == 0):
                            print("Error Cleared!")
                        else:
                            if(can_message_2.arbitration_id == WRIST_ID):
                                pos, vel, tau = self.unpack_wrist(can_message_2.data)
                                self.gr_data.set_q(self.gr_data.w_idxs, pos)
                                self.gr_data.set_qd(self.gr_data.w_idxs, vel)
                                self.gr_data.set_tau(self.gr_data.w_idxs, tau)
                except Exception as e:
                    # error = 1
                    print(f"CAN is dead: {e}")
                num_tries += 1

        # if simulation mode
        if self.mode==PlatformMode.SIM_ONLY:
            # holding controller inputs constant for control dt, so simulate all steps in one go
            # simulate for self.sim_steps_per_control
            mj.mj_step(self.mj_model, self.mj_data, nstep=self.sim_steps_per_control)
            # for real-time sim, wait for control_dt since start of step()
            if self.enforce_real_time_sim:
                while (self.time()-self.current_t < self.control_dt-self.dt_comp): continue
            # update timing and set control flag
            self.last_control_t = self.current_t
            self.run_control = True

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
        wrist_p = self.mj_data.body('palm').xpos
        wrist_R = self.mj_data.body('palm').xmat.reshape((3,3))
        l_dip_tip_p = self.mj_data.body('l_dip_tip').xpos
        l_dip_tip_R = self.mj_data.body('l_dip_tip').xmat.reshape((3,3))
        r_dip_tip_p = self.mj_data.body('r_dip_tip').xpos
        r_dip_tip_R = self.mj_data.body('r_dip_tip').xmat.reshape((3,3))
        self.gr_data.kinematics['base']['p'] = wrist_p
        self.gr_data.kinematics['base']['R'] = wrist_R
        self.gr_data.kinematics['l_dip_tip']['p'] = l_dip_tip_p
        self.gr_data.kinematics['l_dip_tip']['R'] = l_dip_tip_R
        self.gr_data.kinematics['r_dip_tip']['p'] = r_dip_tip_p
        self.gr_data.kinematics['r_dip_tip']['R'] = r_dip_tip_R
        # get fingertip jacobians relative to wrist
        Jacp = np.zeros((3,self.mj_model.nv))
        JacR = np.zeros((3,self.mj_model.nv))
        mj.mj_jac(self.mj_model, self.mj_data, Jacp, JacR, l_dip_tip_p, self.mj_model.body('l_dip_tip').id)
        self.gr_data.kinematics['l_dip_tip']['Jacp'] = Jacp[:,1:5].copy()
        self.gr_data.kinematics['l_dip_tip']['JacR'] = JacR[:,1:5].copy()
        mj.mj_jac(self.mj_model, self.mj_data, Jacp, JacR, r_dip_tip_p, self.mj_model.body('r_dip_tip').id)
        self.gr_data.kinematics['r_dip_tip']['Jacp'] = Jacp[:,5:9].copy()
        self.gr_data.kinematics['r_dip_tip']['JacR'] = JacR[:,5:9].copy()
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
        if self.mode==PlatformMode.SIM_ONLY:
            # fill gr_data from mj_data
            # start with just joint info (q, qd, tau), access by name
            # TODO: should this iterate through model joints first? then gr_data joint keys?
            mj_joints = [self.mj_model.joint(i).name for i in range(self.mj_model.njnt)]
            for key in self.gr_data.joints.keys():
                self.gr_data.joints[key].q = self.mj_data.joint(key).qpos
                self.gr_data.joints[key].qd = self.mj_data.joint(key).qvel
                self.gr_data.joints[key].tau = self.mj_data.joint(key).qfrc_applied
            # get contact location data for fingertips
            # TODO: should we just do this for phalanges too?
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

        # update position and orientation of wrist mocap body
        # TODO: should this be for simulation only? once arm is added, mocap pose will be set from arm forward kinematics?
        self.mj_data.mocap_pos = self.gr_data.kinematics['base_des']['p']
        base_quat_des = np.zeros((4,))
        mj.mju_mat2Quat(base_quat_des, self.gr_data.kinematics['base_des']['R'].flatten())
        self.mj_data.mocap_quat = base_quat_des

        # if hardware mode
        if self.mode==PlatformMode.HW_WITH_VIS or self.mode==PlatformMode.HW_NO_VIS:
            # TODO: could just pass gr_data and idxs to the pack_command function?
            left_finger_msg = self.pack_joints(self.gr_data.get_q_des(self.gr_data.l_idxs),
                                            self.gr_data.get_qd_des(self.gr_data.l_idxs),
                                            self.gr_data.get_kp(self.gr_data.l_idxs),
                                            self.gr_data.get_kd(self.gr_data.l_idxs),
                                            self.gr_data.get_tau_ff(self.gr_data.l_idxs))
            self.CAN_bus_1.send(can.Message(arbitration_id=LEFT_COMMAND, dlc=48, data=left_finger_msg, is_fd=True, is_extended_id=False))
            right_finger_msg = self.pack_joints(self.gr_data.get_q_des(self.gr_data.r_idxs),
                                            self.gr_data.get_qd_des(self.gr_data.r_idxs),
                                            self.gr_data.get_kp(self.gr_data.r_idxs),
                                            self.gr_data.get_kd(self.gr_data.r_idxs),
                                            self.gr_data.get_tau_ff(self.gr_data.r_idxs))
            self.CAN_bus_1.send(can.Message(arbitration_id=RIGHT_COMMAND, dlc=48, data=right_finger_msg, is_fd=True, is_extended_id=False))
            if self.wrist_enable:
                wrist_msg = self.pack_wrist(self.gr_data.get_q_des(self.gr_data.w_idxs),
                                            self.gr_data.get_qd_des(self.gr_data.w_idxs),
                                            self.gr_data.get_kp(self.gr_data.w_idxs),
                                            self.gr_data.get_kd(self.gr_data.w_idxs),
                                            self.gr_data.get_tau_ff(self.gr_data.w_idxs))
                self.CAN_bus_2.send(can.Message(arbitration_id=WRIST_ID, data=wrist_msg, is_extended_id=False))

        # if simulation mode
        if self.mode==PlatformMode.SIM_ONLY:
            # update actuator commands based on gr_data tau_command, will be applied during next mj_step call
            self.mj_data.ctrl = self.gr_data.get_tau_command(self.gr_data.all_idxs)

    # packing CAN commands for fingers
    def pack_joints(self, p_des, v_des, kp, kd, t_ff):
        # p_des, v_des, kp, kd, t_ff will be (4,) numpy arrays
        command_msg = [ int(0) ] * 48
        num_commands = 4
        # joints are out of order in the firmware! so we need to reorder them here
        # here: joint order is 0:MCR, 1:MCP, 2:PIP, 3:DIP
        # FW:   joint order is 0:MCP, 1:PIP, 2:DIP, 3:MCR
        joint_order = [3,0,1,2] # index is order here, value is order in FW
        # NOTE: also need to flip sign of all finger joint command data
        for com in range(num_commands):
            idx = joint_order[com]
            p_int = float_to_uint(-p_des[com], P_MIN, P_MAX, 16)
            v_int = float_to_uint(-v_des[com], V_MIN, V_MAX, 12)
            kp_int = float_to_uint(kp[com]*SCALE, KP_MIN, KP_MAX, 12)
            kd_int = float_to_uint(kd[com]*SCALE, KD_MIN, KD_MAX, 12)
            t_int = float_to_uint(-t_ff[com]*SCALE, T_MIN, T_MAX, 12)
            command_msg[idx*8 + 0] = p_int >> 8
            command_msg[idx*8 + 1] = p_int & 0xFF
            command_msg[idx*8 + 2] = v_int >> 4
            command_msg[idx*8 + 3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
            command_msg[idx*8 + 4] = kp_int & 0xFF
            command_msg[idx*8 + 5] = kd_int >> 4
            command_msg[idx*8 + 6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
            command_msg[idx*8 + 7] = t_int & 0xff
        return command_msg

    # packing CAN commands for wrist roll motor
    def pack_wrist(self, p_des, v_des, kp, kd, t_ff):
        # p_des, v_des, kp, kd, t_ff will be (1,) numpy arrays
        # NOTE: need to flip sign of wrist joint data
        command_msg = [ int(0) ] * 8
        p_int = float_to_uint(-p_des[0], P_MIN, P_MAX, 16)
        v_int = float_to_uint(-v_des[0], V_MIN, V_MAX, 12)
        kp_int = float_to_uint(kp[0], KP_MIN, KP_MAX, 12)
        kd_int = float_to_uint(kd[0], KD_MIN, KD_MAX, 12)
        t_int = float_to_uint(-t_ff[0], T_MIN, T_MAX, 12)
        command_msg[0] = p_int >> 8
        command_msg[1] = p_int & 0xFF
        command_msg[2] = v_int >> 4
        command_msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
        command_msg[4] = kp_int & 0xFF
        command_msg[5] = kd_int >> 4
        command_msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
        command_msg[7] = t_int & 0xff

        return command_msg

    # unpacking received joint data message from gripper
    def unpack_joints(self, msg):
        pos=[]
        vel=[]
        tau=[]
        p_int=[]
        v_int=[]
        i_int=[]

        imsg = [msg[i:i + 5] for i in range(0, 40, 5)] # break to each dxls # range(0, 45, 5) -> range(0, 40, 5) for no wrist

        for jmsg in imsg:
            p_int.append((jmsg[0] << 8) | jmsg[1])
            v_int.append((jmsg[2] << 4) | (jmsg[3] >> 4))
            i_int.append(((jmsg[3] & 0xF) << 8) | jmsg[4])

        for p, v, i in zip(p_int, v_int, i_int):
            pos.append(uint_to_float(p, P_MIN, P_MAX, 16))
            vel.append(uint_to_float(v, V_MIN, V_MAX, 12))
            tau.append(uint_to_float(i, T_MIN, T_MAX, 12)/SCALE)

        # from FW, order is 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
        # here, order should be 0:mcr_l, 1:mcp_l, 2:pip_l, 3:dip_l, 4:mcr_r, 5:mcp_r, 6:pip_r, 7:dip_r
        # NOTE: also need to flip sign of all joint data
        joint_order = [3,0,1,2,7,4,5,6]
        pos = [-pos[i] for i in joint_order]
        vel = [-vel[i] for i in joint_order]
        tau = [-tau[i] for i in joint_order]

        # return (8,) numpy arrays for pos, vel, and tau
        return np.array(pos), np.array(vel), np.array(tau)

    # unpacking received joint data message from wrist roll motor
    def unpack_wrist(self, msg):
        msg_id = msg[0]
        p_int = (msg[1]<<8)|msg[2]
        v_int = (msg[3]<<4)|(msg[4]>>4)
        i_int = ((msg[4]&0xF)<<8)|msg[5]
        tm_int = msg[6]
        error = msg[7]

        pos = uint_to_float(p_int, P_MIN, P_MAX, 16)
        vel = uint_to_float(v_int, V_MIN, V_MAX, 12)
        tau = uint_to_float(i_int, T_MIN, T_MAX, 12)
        temp = uint_to_float(tm_int, 0.0, 125.0, 8)

        # NOTE: need to flip sign of wrist joint data
        # return (1,) numpy arrays for pos, vel, and tau
        return np.array([-pos]), np.array([-vel]), np.array([-tau])

    # unpacking received sensor data message from gripper
    def unpack_sensors(self, msg):

        # fingertip sensors
        fx_int1 = ((msg[0] & 0x0F) << 8) | msg[1]
        fy_int1 = (msg[2] << 4) | (msg[3] >> 4)
        fz_int1 = ((msg[3] & 0x0F) << 8) | msg[4]
        theta_int1 = (msg[5] << 4) | (msg[6] >> 4)
        phi_int1 = ((msg[6] & 0x0F) << 8) | msg[7]

        fx_int2 = ((msg[8] & 0x0F) << 8) | msg[9]
        fy_int2 = (msg[10] << 4) | (msg[11] >> 4)
        fz_int2 = ((msg[11] & 0x0F) << 8) | msg[12]
        theta_int2 = (msg[13] << 4) | (msg[14] >> 4)
        phi_int2 = ((msg[14] & 0x0F) << 8) | msg[15]

        fx_1 = uint_to_float(fx_int1, FT_MIN, FT_MAX, 12)
        fy_1 = uint_to_float(fy_int1, FT_MIN, FT_MAX, 12)
        fz_1 = uint_to_float(fz_int1, FN_MIN, FN_MAX, 12)
        theta_1 = uint_to_float(theta_int1, ANG_MIN, ANG_MAX, 12)
        phi_1 = uint_to_float(phi_int1, ANG_MIN, ANG_MAX, 12)

        fx_2 = uint_to_float(fx_int2, FT_MIN, FT_MAX, 12)
        fy_2 = uint_to_float(fy_int2, FT_MIN, FT_MAX, 12)
        fz_2 = uint_to_float(fz_int2, FN_MIN, FN_MAX, 12)
        theta_2 = uint_to_float(theta_int2, ANG_MIN, ANG_MAX, 12)
        phi_2 = uint_to_float(phi_int2, ANG_MIN, ANG_MAX, 12)

        # raw values for fingertip sensors
        left_dip_force = np.array([fx_1, fy_1, fz_1])
        left_dip_angle = np.array([theta_1, phi_1])
        right_dip_force = np.array([fx_2, fy_2, fz_2])
        right_dip_angle = np.array([theta_2, phi_2])
        left_dip_tof = np.array([msg[16],msg[17],msg[18],msg[19],msg[20]]) # left 0:4
        right_dip_tof = np.array([msg[21],msg[22],msg[23],msg[24],msg[25]]) # right 0:4

        # raw values for palm sensor
        palm_tof = np.array([msg[26]])
        palm_fsr1 = (msg[27] << 4) | (msg[28] >> 4)
        palm_fsr2 = ((msg[28] & 0x0F) << 8) | msg[29]
        palm_fsr = np.array([palm_fsr1, palm_fsr2])

        # raw values for phalange sensors
        left_mcp_tof = np.array([msg[30]]) # left mcp
        left_mcp_fsr1 =  (msg[31] << 4) | (msg[32] >> 4)
        left_mcp_fsr2 = ((msg[32] & 0x0F) << 8) | msg[33]
        left_mcp_fsr = np.array([left_mcp_fsr1, left_mcp_fsr2])

        left_pip_tof = np.array([msg[34]]) # left pip
        left_pip_fsr1 =  (msg[35] << 4) | (msg[36] >> 4)
        left_pip_fsr2 = ((msg[36] & 0x0F) << 8) | msg[37]
        left_pip_fsr = np.array([left_pip_fsr1, left_pip_fsr2])

        right_mcp_tof = np.array([msg[38]]) # right mcp
        right_mcp_fsr1 =  (msg[39] << 4) | (msg[40] >> 4)
        right_mcp_fsr2 = ((msg[40] & 0x0F) << 8) | msg[41]
        right_mcp_fsr = np.array([right_mcp_fsr1, right_mcp_fsr2])

        right_pip_tof = np.array([msg[42]]) # right pip
        right_pip_fsr1 =  (msg[43] << 4) | (msg[44] >> 4)
        right_pip_fsr2 = ((msg[44] & 0x0F) << 8) | msg[45]
        right_pip_fsr = np.array([right_pip_fsr1, right_pip_fsr2])

        # collect lists of arrays of raw data for each sensor
        # output is a dict of these lists
        # NOTE: these keys need to be the same as the names of the sensors in GripperData
        all_data = {"palm":     [palm_fsr, palm_tof],
                    "l_mcp":    [left_mcp_fsr, left_mcp_tof],
                    "l_pip":    [left_pip_fsr, left_pip_tof],
                    "l_dip":    [left_dip_force, left_dip_angle, left_dip_tof],
                    "r_mcp":    [right_mcp_fsr, right_mcp_tof],
                    "r_pip":    [right_pip_fsr, right_pip_tof],
                    "r_dip":    [right_dip_force, right_dip_angle, right_dip_tof]}

        return all_data