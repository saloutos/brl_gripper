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


def init_run_binned_rnn(model):
    model.eval()

    init_h = torch.randn(model.rnn.num_layers, model.rnn.hidden_size, device="cpu")

    theta_angles = [model.theta_idx_to_angle(i) for i in range(model.n_theta_bins)]
    phi_angles = [model.phi_idx_to_angle(i) for i in range(model.n_phi_bins)]

    theta_angles = torch.tensor(theta_angles).unsqueeze(0).float()
    phi_angles = torch.tensor(phi_angles).unsqueeze(0).float() 

    return init_h, theta_angles, phi_angles


def run_binned_rnn(raw_data, model, std_dev_X, mean_X, theta_angles, phi_angles, h):
  
    raw_data = torch.tensor(raw_data).unsqueeze(0).float()  # L=1, H_in = 8
    data = (raw_data - mean_X) / std_dev_X

    h, _ = model.rnn(data, h)
    y_pred = model.fc(h)

    y_pred = y_pred.detach()
    h = h.detach()

    Fxyzn = y_pred[..., :4]
    theta_bins = y_pred[..., 4:4 + model.n_theta_bins]
    phi_bins = y_pred[..., 4 + model.n_theta_bins:4 + model.n_theta_bins + model.n_phi_bins]
    contact_flag = y_pred[..., -1:]

    theta_probs = torch.softmax(theta_bins, dim=-1)
    phi_probs = torch.softmax(phi_bins, dim=-1)

    theta_idx = torch.argmax(theta_probs, dim=-1)
    phi_idx = torch.argmax(phi_probs, dim=-1)

    theta = model.theta_idx_to_angle(theta_idx)
    phi = model.phi_idx_to_angle(phi_idx)

    weighted_theta = torch.sum(theta_probs * theta_angles)
    weighted_phi = torch.sum(phi_probs * phi_angles)

    weighted_theta_deg = weighted_theta * 180 / np.pi
    weighted_phi_deg = weighted_phi * 180 / np.pi

    theta_deg = theta * 180 / np.pi
    phi_deg = phi * 180 / np.pi

    contact_prob = torch.sigmoid(contact_flag)

    # print("theta: ", weighted_theta_deg, "phi: ", weighted_phi_deg, "theta: ", theta_deg, "phi: ", phi_deg, "contact", contact_prob)
    # Using Fx = Fy = 0 for now, Fz = Fn
    all_values = np.concatenate(([0, 0], Fxyzn[..., -1].numpy().flatten(), weighted_theta_deg.numpy().flatten(), weighted_phi_deg.numpy().flatten(), contact_prob.numpy().flatten()))

    return all_values, h

def sensor_to_contact_frame(contact_data):

    Fxyz = contact_data[0:3]
    theta = contact_data[3]
    phi = contact_data[4]
    contact_prob = contact_data[5]

    #eventually to convert from sensor Fxyz to contact Fxyz
    R_theta = R.from_euler("y", theta, degrees = True).as_matrix()
    R_phi = R.from_euler("x", phi, degrees = True).as_matrix()
    Fxyz_contact = R_theta.T @ R_phi.T @ -Fxyz

    return Fxyz_contact, theta, phi