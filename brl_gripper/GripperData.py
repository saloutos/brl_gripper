# imports
import numpy as np

# joint data class
class JointData:
    def __init__(self, name=""):
        self.name = name
        # NOTE: all joint data should be stored as 1-D numpy arrays
        self.q = np.array([0.0])
        self.qd = np.array([0.0])
        self.tau = np.array([0.0])
        self.q_des = np.array([0.0])
        self.qd_des = np.array([0.0])
        self.tau_ff = np.array([0.0])
        self.kp = np.array([0.0])
        self.kd = np.array([0.0])
        self.tau_command = np.array([0.0])
        # default values for controllers to use
        self.q_des_default = np.array([0.0])
        self.kp_default = np.array([0.0])
        self.kd_default = np.array([0.0])
    # joint control law
    def update_control(self):
        self.tau_command = self.kp*(self.q_des-self.q) + self.kd*(self.qd_des-self.qd) + self.tau_ff
        return self.tau_command # not sure if the return is necessary
    # logging functions
    def log_data(self):
        # TODO: include gains?
        return [self.q[0], self.qd[0], self.tau[0], self.q_des[0], self.qd_des[0], self.tau_ff[0], self.tau_command[0]]
    def log_header(self):
        # TODO: include gains?
        return [self.name+"_q",self.name+"_qd",self.name+"_tau",
                self.name+"_q_des",self.name+"_qd_des",self.name+"_tau_ff",
                self.name+"_tau_command"]

# sensor data classes
class SensorData:
    def __init__(self, name=""):
        self.name = name
        self.kinematics = (np.zeros((3,)), np.eye(3)) # (pos, R) of sensor frame, in world frame
    # logging functions
    def log_data(self):
        return []
    def log_header(self):
        return []
    # processing sensor data
    def update_kinematics(self, pos, R): # mostly for visualization
        self.kinematics = (pos, R)
    def update_raw_data_from_hw(self):
        pass
    def update_raw_data_from_sim(self):
        pass
    def process_data(self):
        pass
    def sync_data_to_viewer(self, scene, start_idx):
        return start_idx

class FingertipSensorData(SensorData):
    def __init__(self, name=""):
        super().__init__(name)
        # NOTE: all sensor data should be stored as 1-D numpy arrays
        self.contact_force_offset = np.array([0.0, 0.0, 0.0])
        self.contact_force_raw = np.array([0.0, 0.0, 0.0])
        self.contact_angle_raw = np.array([0.0, 0.0])
        self.contact_force = np.array([0.0, 0.0, 0.0]) # fx, fy, fz in N  # fz is normal to surface
        self.contact_angle = np.array([0.0, 0.0])  # theta, phi in deg
        # TODO: set a clearer ToF order here
        self.tof_raw = np.array([0, 0, 0, 0, 0])  # 1,2,3,4,5 in mm
        self.dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # 1,2,3,4,5 in m
        # contact angle ranges and filter coefficients
        self.theta_range = [-45.0, 45.0]
        self.phi_range = [-135.0, 45.0]
        self.contact_force_filter_alpha = 1.0
        self.contact_angle_filter_alpha = 1.0
        self.normal_force_threshold = 1.0
        # variables for visualization
        # NOTE: center of rubber sphere is origin of sensor frame
        self.nominal_contact = np.array([0.0, 0.0, 0.01]) # sphere radius is 10mm
        self.force_scale = 0.02
        self.tof_offsets = np.array([[-0.0025, -0.00782, -0.008], # tof1
                                    [0.0082, -0.00735, 0.0], # tof2
                                    [-0.0025, 0.00782, -0.008], # tof3
                                    [0.0082, 0.00735, 0.0], # tof4
                                    [-0.0015, 0.0, -0.011]]) # tof5
        self.tof_axes = [0, 2, 0, 2, 2]
        self.tof_signs = [-1, 1, -1, 1, -1]

    # logging functions
    def log_data(self):
        return self.contact_force.tolist()+self.contact_angle.tolist()+self.dist.tolist()
    def log_header(self):
        return [self.name+"_fx",self.name+"_fy",self.name+"_fz",
                self.name+"_theta",self.name+"_phi",
                self.name+"_dist1",self.name+"_dist2",self.name+"_dist3",self.name+"_dist4",self.name+"_dist5"]
    # initialization functions
    def set_force_offset(self, offset=None):
        if offset is not None:
            self.contact_force_offset = offset
        else:
            self.contact_force_offset = self.contact_force_raw
    # processing sensor data
    def update_raw_data_from_hw(self, new_data):
        # new data should come in as list of arrays
        self.contact_force_raw = new_data[0]
        self.contact_angle_raw = new_data[1]
        self.tof_raw = new_data[2]
    def update_raw_data_from_sim(self, new_data):
        # new data should come in as dict with mujoco sensor names and data as arrays
        sim_force_data = new_data['force']
        sim_contact_vec = new_data['contact']
        sim_tof_data = np.array([new_data['tof1'],new_data['tof2'],new_data['tof3'],new_data['tof4'],new_data['tof5']]).squeeze()
        # process contact and force data to re-create raw hardware data
        theta_rad = np.arcsin(-sim_contact_vec[1])
        phi_rad = np.arctan2(sim_contact_vec[0], sim_contact_vec[2])
        theta_deg  = np.clip(np.rad2deg(theta_rad), self.theta_range[0], self.theta_range[1])
        phi_deg = np.clip(np.rad2deg(phi_rad), self.phi_range[0], self.phi_range[1])
        self.contact_angle_raw = np.array([theta_deg, phi_deg])
        R_theta = np.array([[1, 0, 0], [0, np.cos(theta_rad), -np.sin(theta_rad)], [0, np.sin(theta_rad), np.cos(theta_rad)]]) # Rx by theta
        R_phi = np.array([[np.cos(phi_rad), 0, np.sin(phi_rad)], [0, 1, 0], [-np.sin(phi_rad), 0, np.cos(phi_rad)]]) # Ry by phi
        R_cont = R_phi @ R_theta
        self.contact_force_raw = -1.0 * (R_cont.T @ sim_force_data.reshape((3,1))).squeeze()
        # process tof data to re-create raw hardware data
        sim_tof_data = np.where(sim_tof_data==-1.0, 0.255, sim_tof_data)
        self.tof_raw = np.round(1000.0*sim_tof_data)
    def process_data(self):
        self.filter_contact_force()
        # TODO: only filter angle if contact force is above threshold
        self.filter_contact_angle()
        self.convert_tof_data()
    def filter_contact_force(self, alpha=None):
        if alpha is not None:
            self.contact_force = (1.0-alpha)*self.contact_force + alpha*(self.contact_force_raw-self.contact_force_offset)
        else:
            self.contact_force = (1.0-self.contact_force_filter_alpha)*self.contact_force + self.contact_force_filter_alpha*(self.contact_force_raw-self.contact_force_offset)
    def filter_contact_angle(self, alpha=None):
        if alpha is not None:
            self.contact_angle = (1.0-alpha)*self.contact_angle + alpha*self.contact_angle_raw
        else:
            self.contact_angle = (1.0-self.contact_angle_filter_alpha)*self.contact_angle + self.contact_angle_filter_alpha*self.contact_angle_raw
    def convert_tof_data(self):
        self.tof_raw = np.where(self.tof_raw==0, 254, self.tof_raw)
        self.dist = self.tof_raw/1000.0  # convert mm to m
    # visualization of hardware data
    def sync_data_to_viewer(self, scene, start_idx):
        idx = start_idx
        # visualize data for 5 ToF sensors
        for t in range(5):
            # line starts at pos, aligns with z-axis of sensor frame
            scene.geoms[idx].type = 103 # line
            scene.geoms[idx].size = np.array([4, 4, self.dist[t]]) # width in px, width in px, length in m
            scene.geoms[idx].pos = self.kinematics[0] + self.kinematics[1].dot(self.tof_offsets[t,:])
            # align z-axis geom frame with tof sensor direction!
            z = self.tof_signs[t]*self.kinematics[1][:,self.tof_axes[t]].reshape((3,1)) # should be unit vec already
            y = np.cross(z.squeeze(), np.array([0, 1, 0]).squeeze()).reshape((3,1))
            x = np.cross(y.squeeze(), z.squeeze()).reshape((3,1))
            scene.geoms[idx].mat = np.hstack((x/np.linalg.norm(x), y/np.linalg.norm(y), z/np.linalg.norm(z))) # full 3x3 matrix, not 9-vector
            scene.geoms[idx].rgba = np.array([1, 0.2, 0, 0.5])
            idx += 1
        # visualize force at contact location
        # arrow starts at pos, aligns with z-axis of sensor frame
        scene.geoms[idx].type = 100 # arrow
        scene.geoms[idx].size = np.array([0.002, 0.002, self.force_scale*np.linalg.norm(self.contact_force)]) # radius, radius, length in m
        # build contact frame from two angles
        theta = np.deg2rad(self.contact_angle[0])
        phi = np.deg2rad(self.contact_angle[1])
        R_theta = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]]) # Rx by theta
        R_phi = np.array([[np.cos(phi), 0, np.sin(phi)], [0, 1, 0], [-np.sin(phi), 0, np.cos(phi)]]) # Ry by phi
        R_cont = R_phi @ R_theta
        p_cont = R_cont.dot(self.nominal_contact)
        scene.geoms[idx].pos = self.kinematics[0] + self.kinematics[1].dot(p_cont)
        # align z-axis geom frame with contact force direction!
        if np.linalg.norm(self.contact_force)>0.01:
            z = self.kinematics[1] @ R_cont @ (self.contact_force.reshape((3,1))/np.linalg.norm(self.contact_force))
            y = np.cross(z.squeeze(), np.array([0, 0, 1]).squeeze()).reshape((3,1))
            x = np.cross(y.squeeze(), z.squeeze()).reshape((3,1))
            scene.geoms[idx].mat = np.hstack((x/np.linalg.norm(x), y/np.linalg.norm(y), z/np.linalg.norm(z))) # full 3x3 matrix, not 9-vector
        else:
            scene.geoms[idx].mat = self.kinematics[1] @ R_cont # full 3x3 matrix, not 9-vector
        scene.geoms[idx].rgba=np.array([1, 0, 1, 0.5])
        idx += 1
        return idx

class PhalangeSensorData(SensorData):
    def __init__(self, name=""):
        super().__init__(name)
        # NOTE: all sensor data should be stored as 1-D numpy arrays
        # TODO: set a clearer fsr order here
        self.fsr_raw = np.array([0, 0]) # 1,2 in counts
        self.contact = np.array([0, 0]) # 1,2 in binary
        self.tof_raw = np.array([0]) # in mm
        self.dist = np.array([0.0]) # in m
        # FSR threshold for contact detection
        self.fsr_threshold = 3750
        # FSR locations in sensor frame
        # NOTE: ToF sensor is at origin of sensor frame
        self.fsr_offsets = np.array([[0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0]])
    # logging functions
    def log_data(self):
        return self.contact.tolist()+self.dist.tolist()
    def log_header(self):
        return [self.name+"_contact1",self.name+"_contact2",self.name+"_dist"]
    # processing sensor data
    def update_raw_data_from_hw(self, new_data):
        # new data should come in as list of arrays
        self.fsr_raw = new_data[0]
        self.tof_raw = new_data[1]
    def update_raw_data_from_sim(self, new_data):
        # new data should come in as dict with mujoco sensor names and data as arrays
        sim_fsr_data = np.array([new_data['fsr1'],new_data['fsr2']]).squeeze()
        sim_tof_data = new_data['tof']
        # TODO: implement correct processing
        # process contact data to re-create raw hardware data?
        self.fsr_raw = np.where(sim_fsr_data>0.0, self.fsr_threshold-100, self.fsr_threshold+100) #4000.0 - sim_fsr_data*3000.0
        # process tof data to re-create raw hardware data?
        sim_tof_data = np.where(sim_tof_data==-1.0, 0.255, sim_tof_data)
        self.tof_raw = np.round(1000.0*sim_tof_data, 0)
    def process_data(self):
        self.filter_contact_data()
        self.convert_tof_data()
    # TODO: fill in these functions
    def filter_contact_data(self):
        self.contact = np.where(self.fsr_raw<self.fsr_threshold, 1, 0)
    def convert_tof_data(self):
        self.tof_raw = np.where(self.tof_raw==0, 254, self.tof_raw)
        self.dist = self.tof_raw/1000.0  # convert mm to m
    # visualization of hardware data
    def sync_data_to_viewer(self, scene, start_idx):
        idx = start_idx
        # visualize ToF data
        # line starts at pos, aligns with z-axis of sensor frame
        scene.geoms[idx].type = 103 # line
        scene.geoms[idx].size = np.array([4, 4, self.dist[0]]) # width in px, width in px, length in m
        scene.geoms[idx].pos = self.kinematics[0]
        scene.geoms[idx].mat = self.kinematics[1] # full 3x3 matrix, not 9-vector
        scene.geoms[idx].rgba = np.array([1, 0.78, 0, 0.5])
        idx += 1
        # visualize forces at contact locations for FSRs
        for f in range(2):
            # arrow starts at pos, aligns with z-axis of sensor frame
            scene.geoms[idx].type = 100 # arrow
            scene.geoms[idx].size = np.array([0.002, 0.002, self.contact[f]*0.05]) # radius, radius, length in m
            scene.geoms[idx].pos = self.kinematics[0] + self.kinematics[1].dot(self.fsr_offsets[f])
            scene.geoms[idx].mat = self.kinematics[1] # full 3x3 matrix, not 9-vector
            scene.geoms[idx].rgba = np.array([0.5, 0.78, 0.2, 0.5])
            idx += 1
        return idx

class PalmSensorData(PhalangeSensorData):
    def __init__(self, name=""):
        super().__init__(name)
        # different FSR locations
        self.fsr_offsets = np.array([[0.0, 0.01465, 0.0],
                                    [0.0, -0.01465, 0.0]])

class McpPhalangeSensorData(PhalangeSensorData):
    def __init__(self, name=""):
        super().__init__(name)
        # different FSR locations
        self.fsr_offsets = np.array([[-0.009325, 0.0, 0.0],
                                    [0.009325, 0.0, 0.0]])

class PipPhalangeSensorData(PhalangeSensorData):
    def __init__(self, name=""):
        super().__init__(name)
        # different FSR locations
        self.fsr_offsets = np.array([[-0.00525, 0.0, 0.0],
                                    [0.00525, 0.0, 0.0]])


# gripper data class
class GripperData:
    # TODO: initialize with lists of joints and sensors? rather than hardcoding?
    # TODO: if init takes joints and sensors, should it take indices too? those could be added in platform?
    def __init__(self): #, joints=[], sensors=[]):
        # list of joints
        joints = [JointData("1_w_roll"),                                                                    # wrist
                    JointData("2_l_mcr"),JointData("3_l_mcp"),JointData("4_l_pip"),JointData("5_l_dip"),    # left finger
                    JointData("6_r_mcr"),JointData("7_r_mcp"),JointData("8_r_pip"),JointData("9_r_dip")]    # right finger
            # list of sensors
        sensors = [PalmSensorData("palm"),                                                                 # palm
                McpPhalangeSensorData("l_mcp"),PipPhalangeSensorData("l_pip"),FingertipSensorData("l_dip"),   # left finger
                McpPhalangeSensorData("r_mcp"),PipPhalangeSensorData("r_pip"),FingertipSensorData("r_dip")]   # right finger
        # NOTE: these should match the names of the joints and sensors in the mujoco model
        # now, store in dicts so that we have named access
        self.joint_names = [joint.name for joint in joints]
        self.joints = dict(zip(self.joint_names, joints))
        self.sensor_names = [sensor.name for sensor in sensors]
        self.sensors = dict(zip(self.sensor_names, sensors))
        # other joint vars
        self.nj = len(joints) # number of joints
        self.ns = len(sensors) # number of sensors
        self.all_idxs = list(range(self.nj)) # all joint idxs
        self.w_idxs = [0] # joint idx for wrist
        self.finger_idxs = [1,2,3,4,5,6,7,8] # joint idxs for fingers
        self.l_idxs = [1,2,3,4] # joint idxs for left finger
        self.r_idxs = [5,6,7,8] # joint idxs for right finger

        # TODO: store kinematics in dict as well?
        # TODO: make "body kinematics data" class that can return T, R, pos, quat, etc?
        # TODO: then, can define list of bodies whose kinematics we want to track, and update all at once in GP
        # TODO: how likely is it we will want to track sensor kinematics for things beyond visualization?
        self.kinematics = {}
        self.kinematics['base'] = {'p': np.zeros((3,)), 'R':np.eye(3)} # pos, R, in world frame
        self.kinematics['l_dip_tip'] = {'p':np.zeros((3,)), 'R':np.eye(3), 'Jacp':np.zeros((3,4)), 'JacR':np.zeros((3,4))} # pos, R, Jt in world frame (for mocap body)
        self.kinematics['r_dip_tip'] =  {'p':np.zeros((3,)), 'R':np.eye(3), 'Jacp':np.zeros((3,4)), 'JacR':np.zeros((3,4))}
        
        # TODO: this doesn't feel like the right way to do this
        self.kinematics['base_des'] = {'p': np.zeros((3,)), 'R':np.eye(3)} # desired base pos, R in world frame (for mocap body)

        # TODO: other useful vars? don't need to populate them here, but could store them here
        # contact points?
        # have a separate "FingerData" class that has all the data for a finger?
        # these could be stored in controllers too
        # these should all be available from mujoco data, so maybe not necessary here

    # logging functions
    def log_data(self):
        line = []
        # TODO: neeed to sort keys of the dicts to ensure consistent order?
        for key in self.joints.keys():
            line += self.joints[key].log_data()
        for key in self.sensors.keys():
            line += self.sensors[key].log_data()
        return line
    def log_header(self):
        header = []
        for key in self.joints.keys():
            header += self.joints[key].log_header()
        for key in self.sensors.keys():
            header += self.sensors[key].log_header()
        return header

    # NOTE: getters and setters need to be in this class to that GD can be passed to controllers
    # NOTE: syncing GD and mjData will happen in the gripper platform class based on mode
    # getting and setting joint data by index (general versions)
    def get_joint_data(self, var_name, joint_idxs):
        # var name is string, joint_idxs is list of ints
        # iterate over joint_idxs, getting key from indexing list of joint names
        data = [self.joints[self.joint_names[idx]].__dict__[var_name][0] for idx in joint_idxs]
        return np.array(data)
    def set_joint_data(self, var_name, joint_idxs, data):
        # var name is string, joint_idxs is list of ints, data is 1-D array
        if len(joint_idxs)!=len(data):
            raise ValueError("Joint indices and data must be the same length.")
        else:
            # iterate over data, getting key from indexing list of joint names by indexing joint indexes
            for i, d in enumerate(data):
                self.joints[self.joint_names[joint_idxs[i]]].__dict__[var_name][0] = d

    # getting and setting specific joint data by index
    def get_q(self, joint_idxs):
        return self.get_joint_data("q", joint_idxs)
    def set_q(self, joint_idxs, data):
        self.set_joint_data("q", joint_idxs, data)
    def get_qd(self, joint_idxs):
        return self.get_joint_data("qd", joint_idxs)
    def set_qd(self, joint_idxs, data):
        self.set_joint_data("qd", joint_idxs, data)
    def get_tau(self, joint_idxs):
        return self.get_joint_data("tau", joint_idxs)
    def set_tau(self, joint_idxs, data):
        self.set_joint_data("tau", joint_idxs, data)
    def get_q_des(self, joint_idxs):
        return self.get_joint_data("q_des", joint_idxs)
    def set_q_des(self, joint_idxs, data):
        self.set_joint_data("q_des", joint_idxs, data)
    def get_qd_des(self, joint_idxs):
        return self.get_joint_data("qd_des", joint_idxs)
    def set_qd_des(self, joint_idxs, data):
        self.set_joint_data("qd_des", joint_idxs, data)
    def get_tau_ff(self, joint_idxs):
        return self.get_joint_data("tau_ff", joint_idxs)
    def set_tau_ff(self, joint_idxs, data):
        self.set_joint_data("tau_ff", joint_idxs, data)
    def get_kp(self, joint_idxs):
        return self.get_joint_data("kp", joint_idxs)
    def set_kp(self, joint_idxs, data):
        self.set_joint_data("kp", joint_idxs, data)
    def get_kd(self, joint_idxs):
        return self.get_joint_data("kd", joint_idxs)
    def set_kd(self, joint_idxs, data):
        self.set_joint_data("kd", joint_idxs, data)
    def get_tau_command(self, joint_idxs):
        return self.get_joint_data("tau_command", joint_idxs)
    def set_tau_command(self, joint_idxs, data):
        self.set_joint_data("tau_command", joint_idxs, data)
    def get_q_des_default(self, joint_idxs):
        return self.get_joint_data("q_des_default", joint_idxs)
    def set_q_des_default(self, joint_idxs, data):
        self.set_joint_data("q_des_default", joint_idxs, data)
    def get_kp_default(self, joint_idxs):
        return self.get_joint_data("kp_default", joint_idxs)
    def set_kp_default(self, joint_idxs, data):
        self.set_joint_data("kp_default", joint_idxs, data)
    def get_kd_default(self, joint_idxs):
        return self.get_joint_data("kd_default", joint_idxs)
    def set_kd_default(self, joint_idxs, data):
        self.set_joint_data("kd_default", joint_idxs, data)

    # TODO: getting and setting joint data for wrist and fingers (i.e. q,qd,tau all at once)?
    # TODO: getting and setting specific sensor data?

    # updating joint control laws
    def update_all_joint_control(self):
        for key in self.joints.keys():
            self.joints[key].update_control()

    # updating sensor data
    def update_all_sensor_kinematics(self, all_data={}):
        for key in self.sensors.keys():
            self.sensors[key].update_kinematics(all_data[key][0], all_data[key][1])
    def update_all_raw_sensor_data_from_hw(self, all_data={}):
        for key in self.sensors.keys():
            self.sensors[key].update_raw_data_from_hw(all_data[key])
    def update_all_raw_sensor_data_from_sim(self, all_data={}):
        for key in self.sensors.keys():
            self.sensors[key].update_raw_data_from_sim(all_data[key])
    def process_all_sensor_data(self):
        for key in self.sensors.keys():
            self.sensors[key].process_data()
    # syncing viewer with hardware sensor data
    def sync_all_sensor_data_to_viewer(self, scene):
        ng = 0
        for key in self.sensors.keys():
            ng += self.sensors[key].sync_data_to_viewer(scene, ng)
        for i in range(ng):
            # set some defaults for all geoms here
            scene.geoms[i].dataid       = -1
            scene.geoms[i].texid        = -1
            scene.geoms[i].specular     = 0.5
            scene.geoms[i].shininess    = 0.5
        scene.ngeom += ng