import torch
import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

import json

from brl_gripper.assets.sensor_training.models import *

from brl_gripper.assets.sensor_training.datasets import *



# ======================= LOAD MODEL FROM DIRECTORY =======================
def load_model(experiment_name):
    # Use cpu
    device = "cpu"

    # Load the model
    script_dir = os.path.dirname(os.path.realpath(__file__))
    experiment_dir = f"{script_dir}/experiments/{experiment_name}"

    with open(f"{experiment_dir}/experiment_info.json", "r") as f:
        experiment_info = json.load(f)

    with open(f"{experiment_dir}/stats.json", "r") as f:
        stats_info = json.load(f)

    mean_X = stats_info["mean_X"]
    std_dev_X = stats_info["std_dev_X"]

    std_dev_X = torch.tensor(std_dev_X).float()
    mean_X = torch.tensor(mean_X).float()


    model = eval(experiment_info["model_name"])(**experiment_info["model_kwargs"]).to(device)
    model.load_state_dict(torch.load(f"{experiment_dir}/best.pt"))


    return model, std_dev_X, mean_X


# ======================= HELPER FUNCTIONS FOR MODELS =======================
def sensor_to_contact_frame(contact_data,theta,phi):

    #for ellipsoid, this angle must be contact normal angle. 
    Fxyz = (contact_data[:,0:3])

    #eventually to convert from sensor Fxyz to contact Fxyz
    R_theta = (R.from_euler("x", theta, degrees = True).as_matrix()).squeeze()
    R_phi = (R.from_euler("y", phi, degrees = True).as_matrix()).squeeze()
    Fxyz_contact = R_phi.T @ R_theta.T @ Fxyz.T
    return (Fxyz_contact.T).squeeze()

def line_ellipsoid_intersection(R_cont,ellipse_params):
        p0 = np.array([0,0,0]) #a point on the line
        a = ellipse_params[0]
        b = ellipse_params[1]
        c = ellipse_params[2]
        # Coefficients of the quadratic equation
        A = (R_cont[0]**2) / a**2 + (R_cont[1]**2) / b**2 + (R_cont[2]**2) / c**2
        B = 2 * ((p0[0] * R_cont[0]) / a**2 + (p0[1] * R_cont[1]) / b**2 + (p0[2] * R_cont[2]) / c**2)
        C = (p0[0]**2) / a**2 + (p0[1]**2) / b**2 + (p0[2]**2) / c**2 - 1
        
        discriminant = B**2 - 4 * A * C
        
        #TODO: Better error handling if there is no intersection
        if discriminant < 0:
            return None  # No intersection
        elif discriminant == 0:
            t = -B / (2 * A)
            p1 = p0 + t * R_cont
            return np.array([p1])  # One intersection (tangent)
        else:
            sqrt_discriminant = np.sqrt(discriminant)
            t1 = (-B + sqrt_discriminant) / (2 * A)
            t2 = (-B - sqrt_discriminant) / (2 * A)
            p1 = p0 + t1 * R_cont
            p2 = p0 + t2 * R_cont

            #return negative z point #TODO: Check this
            if p1[2]>0:
                return p1
            else:
                return p2  # Two intersection points

def sensor_to_contact_frame_ellipsoid(contact_data, theta_rad, phi_rad):
    #for ellipsoid, calculate contact normal angles
    #z is normal to surface
    #should I put this here?
    Fxyz = (contact_data[:,0:3])
    nominal_contact = np.array([0.0, 0.0, 0.01])
    ellipse_params = np.array([0.0105, 0.009, 0.00635])
    R_theta = np.array([[1, 0, 0], [0, np.cos(theta_rad), -np.sin(theta_rad)], [0, np.sin(theta_rad), np.cos(theta_rad)]]) # Rx by theta
    R_phi = np.array([[np.cos(phi_rad), 0, np.sin(phi_rad)], [0, 1, 0], [-np.sin(phi_rad), 0, np.cos(phi_rad)]]) # Ry by phi
    R_cont = (R_phi @ R_theta @ nominal_contact).T
    #calculate radius of vector (intersection between ellipsoid and line in direction of R_cont)
    contact_vec = line_ellipsoid_intersection(R_cont,ellipse_params)
    #calculate normal vector at contact point
    contact_vec_normal = 2*np.array([contact_vec[0]/ellipse_params[0]**2,contact_vec[1]/ellipse_params[1]**2,contact_vec[2]/ellipse_params[2]**2]) #find vector normal to surface at contact location
    contact_vec_unit_normal = contact_vec_normal/np.linalg.norm(contact_vec_normal)
    theta_rad_normal = np.arcsin(-contact_vec_unit_normal[1])
    phi_rad_normal = np.arctan2(contact_vec_unit_normal[0], contact_vec_unit_normal[2])
    R_theta_normal = np.array([[1, 0, 0], [0, np.cos(theta_rad_normal), -np.sin(theta_rad_normal)], [0, np.sin(theta_rad_normal), np.cos(theta_rad_normal)]]) # Rx by theta
    R_phi_normal = np.array([[np.cos(phi_rad_normal), 0, np.sin(phi_rad_normal)], [0, 1, 0], [-np.sin(phi_rad_normal), 0, np.cos(phi_rad_normal)]]) # Ry by phi
    R_cont_normal = R_phi_normal @ R_theta_normal
    Fxyz_cont = R_cont_normal @ Fxyz.T

    return (Fxyz_cont.T).squeeze()

def normalize(data, mean_X, std_dev_X):
    data = (data - mean_X) / std_dev_X
    return data

# ======================= MODEL INITS =======================
def init_run_binned_rnn(model):
    model.eval()

    init_h = torch.randn(model.rnn.num_layers, model.rnn.hidden_size, device="cpu")

    theta_angles = [model.theta_idx_to_angle(i) for i in range(model.n_theta_bins)]
    phi_angles = [model.phi_idx_to_angle(i) for i in range(model.n_phi_bins)]

    theta_angles = torch.tensor(theta_angles).unsqueeze(0).float()
    phi_angles = torch.tensor(phi_angles).unsqueeze(0).float() 

    return init_h, theta_angles, phi_angles

def init_run_mlp(model):
    model.eval()
    return 

# ======================= MODELS =======================
def run_binned_rnn(raw_data, model, std_dev_X, mean_X, theta_angles, phi_angles, h, sensor_type = "sphere"):
  
    raw_data = torch.tensor(raw_data).unsqueeze(0).float()  # torch shape [1x8]
    data = normalize(raw_data, mean_X, std_dev_X) # torch shape [1x8]

    h, _ = model.rnn(data, h) # torch shape [1x48]
    y_pred = model.fc(h).detach() # torch shape [1xN] -> N depends on # of angle bins

    Fxyzn = y_pred[..., :4].numpy() # numpy shape [1x4]

    theta_bins = y_pred[..., 4:4 + model.n_theta_bins] # torch shape [1xthetabins]
    phi_bins = y_pred[..., 4 + model.n_theta_bins:4 + model.n_theta_bins + model.n_phi_bins] # torch shape [1xphibins]
    contact_flag = y_pred[..., -1] # torch shape [1]

    theta_probs = torch.softmax(theta_bins, dim=-1)
    phi_probs = torch.softmax(phi_bins, dim=-1)

    weighted_theta = torch.sum(theta_probs * theta_angles)
    weighted_phi = torch.sum(phi_probs * phi_angles)

    weighted_theta = -weighted_theta.numpy()
    weighted_phi = weighted_phi.numpy()

    contact_prob = torch.sigmoid(contact_flag)

    if sensor_type == "sphere":
        Fxyz_cont = sensor_to_contact_frame(Fxyzn, weighted_theta, weighted_phi)
    elif sensor_type == "ellipsoid":
        Fxyz_cont = sensor_to_contact_frame_ellipsoid(Fxyzn, weighted_theta, weighted_phi)

    weighted_theta = weighted_theta * 180 / np.pi
    weighted_phi = weighted_phi * 180 / np.pi

    predictions = np.concatenate((Fxyz_cont.flatten(), Fxyzn[..., -1].flatten(), weighted_theta.flatten(), weighted_phi.flatten(), contact_prob.numpy().flatten()))

    return predictions, h

def run_mlp(raw_data, model, std_dev_X, mean_X, sensor_type = "sphere"):
    
    raw_data = torch.tensor(raw_data).unsqueeze(0).float()  # torch shape [1x8]
    data = normalize(raw_data, mean_X, std_dev_X) # torch shape [1x8]

    y_pred = model.std_forward(data).detach()

    Fxyzn = y_pred[..., :4].numpy()
    theta = -y_pred[..., 4].numpy()
    phi = y_pred[..., 5] .numpy()
    contact_prob = y_pred[..., -1]

    if sensor_type == "sphere":
        Fxyz_cont = sensor_to_contact_frame(Fxyzn, theta.squeeze(), phi.squeeze())
    elif sensor_type == "ellipsoid":
        Fxyz_cont = sensor_to_contact_frame_ellipsoid(Fxyzn, theta.squeeze(), phi.squeeze())

    theta_deg = theta * 180 / np.pi
    phi_deg = phi * 180 / np.pi

    predictions = np.concatenate((Fxyz_cont.flatten(), Fxyzn[..., -1].flatten(), theta_deg.flatten(), phi_deg.flatten(), contact_prob.numpy().flatten()))

    return predictions


# ======================= CLASS FOR MODELS USED IN GRIPPER PLATFORM =======================
class SensorModel:
    def load(self, model_path):
        raise NotImplementedError

    def initialize(self, model):
        return {}

    def run(self, pressure_vals, model, state, std_dev, mean, sensor_type="sphere", **kwargs):
        raise NotImplementedError

class BinnedRNNModel(SensorModel):
    def load(self, model_path):
        return load_model(model_path)

    def initialize(self, model):
        h, theta_angles, phi_angles = init_run_binned_rnn(model)
        return {
            "h": h,
            "theta_angles": theta_angles,
            "phi_angles": phi_angles
        }

    def run(self, pressure_vals, model, state, std_dev, mean, sensor_type="sphere", **kwargs):
        output, h_new = run_binned_rnn(
            pressure_vals, model, std_dev, mean,
            state["theta_angles"], state["phi_angles"], state["h"], sensor_type
        )
        state["h"] = h_new  # Update internal state
        return output, state

class MLPModel(SensorModel):
    def load(self, model_path):
        return load_model(model_path)

    def initialize(self, model):
        init_run_mlp(model)
        return {}  # No state needed

    def run(self, pressure_vals, model, state, std_dev, mean, sensor_type="sphere", **kwargs):
        output = run_mlp(pressure_vals, model, std_dev, mean, sensor_type)
        return output, state  # No state change
