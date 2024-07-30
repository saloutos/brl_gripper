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


def run_rnn(data, model, std_dev_X, mean_X):
    #data is shape (8,)

    model.eval()
    h = torch.randn(model.rnn.num_layers, model.rnn.hidden_size, device="cpu") #ERR: For batched 3-D input, hx should also be 3-D but got 2-D tensor 

    data = torch.tensor(data).unsqueeze(0).float() 

    data = (data - mean_X) / std_dev_X

    h, _ = model.rnn(data, h)
    y_pred = model.fc(h).detach()
    
    Fxyzn = y_pred[..., :4]
    theta_rad = y_pred[..., 4]
    phi_rad = y_pred[..., 5]
    contact_flag = y_pred[..., -1]

    theta_deg = theta_rad * 180 / np.pi
    phi_deg = phi_rad * 180 / np.pi
    contact_prob = torch.sigmoid(contact_flag)

    # print("F: ", Fxyzn[..., -4:-1], "theta: ", theta_deg, "phi: ", phi_deg, "contact", contact_prob)

    return Fxyzn[..., -4:-1], theta_deg, phi_deg

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

# def run_mlp(queue, model, std_dev_X, mean_X, done_flag):

#     model.eval()
#     while done_flag.value == 0:
#         if not queue.empty():
#             data = queue.get()
#             data = torch.tensor(data).unsqueeze(0).float()
#             data = (data - mean_X) / std_dev_X

#             y_pred = model.std_forward(data).detach()
#             print(y_pred)

#             Fx = y_pred[..., 0]
#             Fy = y_pred[..., 1]
#             Fz = y_pred[..., 2]
#             Fn = y_pred[..., 3]
#             theta = y_pred[..., 4]
#             phi = y_pred[..., 5]
#             contact_prob = y_pred[..., 6]

#             theta_deg = theta * 180 / np.pi
#             phi_deg = phi * 180 / np.pi

#             print("theta: ", theta_deg.item(), "phi: ", phi_deg.item(), "contact", contact_prob.item())


# def run_mlp_and_rnn(data_queue, mlp_model, rnn_model, std_dev_X, mean_X, done_flag):
#     mlp_model.eval()
#     rnn_model.eval()


#     h = torch.randn(rnn_model.rnn.num_layers, rnn_model.rnn.hidden_size, device="cpu")

#     theta_angles = [rnn_model.theta_idx_to_angle(i) for i in range(rnn_model.n_theta_bins)]
#     phi_angles = [rnn_model.phi_idx_to_angle(i) for i in range(rnn_model.n_phi_bins)]

#     theta_angles = torch.tensor(theta_angles).unsqueeze(0).float()
#     phi_angles = torch.tensor(phi_angles).unsqueeze(0).float()
#     while done_flag.value == 0:
#         if not data_queue.empty():
#             raw_data = data_queue.get()
#             raw_data = torch.tensor(raw_data).unsqueeze(0).float()  # L=1, H_in = 8
#             data = (raw_data - mean_X) / std_dev_X

#             h, _ = rnn_model.rnn(data, h)
#             y_pred_rnn = rnn_model.fc(h)
#             y_pred_mlp = mlp_model.std_forward(data)

#             y_pred_rnn = y_pred_rnn.detach()
#             h = h.detach()

#             Fxyzn = y_pred_rnn[..., :4]
#             theta_bins = y_pred_rnn[..., 4:4 + rnn_model.n_theta_bins]
#             phi_bins = y_pred_rnn[..., 4 + rnn_model.n_theta_bins:4 + rnn_model.n_theta_bins + rnn_model.n_phi_bins]
#             contact_flag = y_pred_rnn[..., -1:]

#             theta_probs = torch.softmax(theta_bins, dim=-1)
#             phi_probs = torch.softmax(phi_bins, dim=-1)

#             theta_idx = torch.argmax(theta_probs, dim=-1)
#             phi_idx = torch.argmax(phi_probs, dim=-1)

#             theta = rnn_model.theta_idx_to_angle(theta_idx)
#             phi = rnn_model.phi_idx_to_angle(phi_idx)

#             weighted_theta = torch.sum(theta_probs * theta_angles)
#             weighted_phi = torch.sum(phi_probs * phi_angles)

#             weighted_theta_deg = weighted_theta * 180 / np.pi
#             weighted_phi_deg = weighted_phi * 180 / np.pi

#             theta_deg = theta * 180 / np.pi
#             phi_deg = phi * 180 / np.pi

#             contact_prob = torch.sigmoid(contact_flag)

#             # print("Theta probs: ", np.round(theta_probs, 3))
#             # print("theta: ", weighted_theta_deg, "phi: ", weighted_phi_deg, "theta: ", theta_deg, "phi: ", phi_deg, "contact", contact_prob)

#             print(raw_data)
#             print("RNN: ", "Fn: ", Fxyzn[..., -1], "theta: ", weighted_theta_deg, "phi: ", weighted_phi_deg, "contact", contact_prob)



#             theta_mlp = y_pred_mlp[..., 4]
#             phi_mlp = y_pred_mlp[..., 5]
#             theta_mlp_deg = theta_mlp * 180 / np.pi
#             phi_mlp_deg = phi_mlp * 180 / np.pi
#             print("MLP: ", "Fn:", y_pred_mlp[..., 3], "theta: ", theta_mlp_deg, "phi: ", phi_mlp_deg, "contact", y_pred_mlp[..., 6])
#             # print(Fxyzn, theta_probs, phi_probs, contact_flag)
    


# def main():
    # sample_time = 1/100
    # target_model_type = "mlp_and_rnn"
    # target_model_fname = "2024-07-08_14-43-33_E9_6_38_and_E9_7_3_BinnedFulloutRNN_cw10_H512_k64"
    # target_model_fname = "2024-07-09_18-04-38_E9_6_38_and_E9_7_3_BinnedFulloutRNN_hd48_H512_k64_bpi32_lr0p0005"
    # # target_model_fname = "2024-07-10_13-03-15_E8_7_8_BinnedFulloutRNN"
    # # target_model_fname = "2024-07-10_13-05-35_E8_7_8_pretrain_E9_BinnedFulloutRNN"
    # # target_model_fname = "2024-07-10_13-08-19_E8_and_E9_pretrain_E8_BinnedFulloutRNN"
    # target_model_fname = "2024-07-10_17-55-48_E9_all_MLP"

    # mlp_model_fname = "2024-07-10_17-55-48_E9_all_MLP"
    # rnn_model_fname = "2024-07-09_18-04-38_E9_6_38_and_E9_7_3_BinnedFulloutRNN_hd48_H512_k64_bpi32_lr0p0005"
    # # rnn_model_fname = "2024-07-10_13-08-19_E8_and_E9_pretrain_E8_BinnedFulloutRNN"

    # # target_model_type = "mlp"
    # # target_model_fname = "2024-07-05_13-08-34_E9_6_28_and_E9_7_3_UnionLoss_12x64x64_h1_p1_dp0p5"

    # # target_model_type = "rnn"
    # # target_model_fname = "2024-07-08_13-14-41_E9_6_38_and_E9_7_3_RNN"

    # done_flag = multiprocessing.Value('i', 0)
    # data_queue = multiprocessing.Queue()
    # reader_process = multiprocessing.Process(target=serial_reader, args=(done_flag, data_queue, sample_time))
    # plotter_process = multiprocessing.Process(target=plot_data, args=(data_queue,))

    # model_runners = {
    #     "mlp": run_mlp,
    #     "rnn": run_rnn,
    #     "binned_rnn": run_binned_rnn
    # }
    # # target_model_runner = model_runners[target_model_type]

    # rnn_model, std_dev_X, mean_X = load_model(rnn_model_fname)
    # mlp_model, std_dev_X, mean_X = load_model(mlp_model_fname)

    # model_process = multiprocessing.Process(target=run_mlp_and_rnn, args=(data_queue, mlp_model, rnn_model, std_dev_X, mean_X, done_flag))

    # # model, std_dev_X, mean_X = load_model(target_model_fname)
    # # model_process = multiprocessing.Process(target=target_model_runner, args=(data_queue, model, std_dev_X, mean_X, done_flag))

    # model_process.start()
    # # plotter_process.start()
    # reader_process.start()

    # end_time = time.time() + 10000
    # while True:
    #     # data = data_queue.get()
    #     # print(data)

    #     if time.time() > end_time:
    #         break

    # done_flag.value = 1

    # model_process.join()
    # plotter_process.join()
    # reader_process.join()
    # print("Serial port closed.")


# if __name__ == "__main__":
#     main()