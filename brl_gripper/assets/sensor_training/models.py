import math
import torch
import torch.nn as nn
from ..sensor_config import *


class BinnedRNNSensorNet(nn.Module):
    if sensor_params.sensor_type == "sphere":
        #spherical range
        theta_range = [-torch.pi / 4, torch.pi / 4]
        phi_range = [-3 * torch.pi / 4, torch.pi / 4]
    elif sensor_params.sensor_type == "ellipsoid":
        #ellipsoid range
        theta_range = [-torch.pi / 4.5, torch.pi / 4.5]
        phi_range = [-2 * torch.pi / 5, 2*torch.pi / 5]
    else:
        print("sensor type is not known")

    def __init__(self, hidden_size=64, num_layers=1, linear_layers = [64,], dropout_p=0.2, bin_width=torch.pi/16, full_out=False):
        # bin_width is 0.15 rad or 8.59437 deg
        # Maybe make it n_bins for theta and n_bins for phi
        # And have bin range.
        super(BinnedRNNSensorNet, self).__init__()
        assert hidden_size == linear_layers[0], "first linear layer must be history size"

        self.bin_width = bin_width
        self.n_theta_bins = int((self.theta_range[1] - self.theta_range[0]) // bin_width)
        self.n_phi_bins = int((self.phi_range[1] - self.phi_range[0]) // bin_width)
        # outputs are Fx, Fy, Fz, Fn, [n_theta_bins], [n_phi_bins], contact_flag

        print(self.n_theta_bins)
        print(self.n_phi_bins)
        output_size = 4 + self.n_theta_bins + self.n_phi_bins + 1
        linear_layers.append(output_size)


        self.rnn = nn.RNN(8, hidden_size, num_layers, batch_first=True, dropout=dropout_p)
        layers = []
        for i in range(len(linear_layers) - 1):
            layers.append(nn.Linear(linear_layers[i], linear_layers[i + 1]))

            if i != (len(linear_layers) - 2):
                layers.append(nn.ReLU())
                layers.append(nn.Dropout(p=dropout_p))
        self.fc = nn.Sequential(*layers)

        self.log_softmax = torch.nn.LogSoftmax(dim=-1)
        self.full_out = full_out


        print(self.rnn)
        print(self.fc)
        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
    def forward(self, x):
        # x should be N x L x 8
        x = x.view(x.size(0), x.size(1), -1)

        h0 = torch.randn(self.rnn.num_layers, x.size(0), self.rnn.hidden_size, device=x.device)
        x, _ = self.rnn(x, h0)

        if self.full_out:
            x = self.fc(x)  # N x L x 8
        else:
            x = self.fc(x[:, -1])  # N x 8
        Fxyzn = x[..., 0:4]
        theta_bins = x[..., 4:4 + self.n_theta_bins]
        phi_bins = x[..., 4 + self.n_theta_bins: 4 + self.n_theta_bins + self.n_phi_bins]
        contact_flag = x[..., -1:]

        log_theta_probs = self.log_softmax(theta_bins)
        log_phi_probs = self.log_softmax(phi_bins)
        contact_prob = torch.sigmoid(contact_flag)
        
        x = torch.cat((Fxyzn, log_theta_probs, log_phi_probs, contact_prob), dim=-1)
        return x

    def theta_idx_to_angle(self, idx):
        # Center in bin
        return self.theta_range[0] + self.bin_width * idx + self.bin_width / 2

    def phi_idx_to_angle(self, idx):
        return self.phi_range[0] + self.bin_width * idx + self.bin_width / 2
    
    def std_forward(self, x):
        x = self.forward(x).detach()
        if self.full_out:
            x = x[:, -1, :]

        Fxyzn = x[:, 0:4]
        theta_probs= x[:, 4:4 + self.n_theta_bins]
        phi_probs = x[:, 4 + self.n_theta_bins: 4 + self.n_theta_bins + self.n_phi_bins]
        contact_flag = x[:, -1:]

        theta_idx = torch.max(theta_probs, dim=1, keepdim=True)[1]
        phi_idx = torch.max(phi_probs, dim=1, keepdim=True)[1]

        theta = self.theta_idx_to_angle(theta_idx)
        phi = self.phi_idx_to_angle(phi_idx)


        x = torch.cat((Fxyzn, theta, phi, contact_flag), dim=1)

        x[:, 0] = torch.clamp(x[:, 0], -20, 20)
        x[:, 1] = torch.clamp(x[:, 1], -20, 20)
        x[:, 2] = torch.clamp(x[:, 2], -20, 20)
        x[:, 3] = torch.clamp(x[:, 3], -40, 0)
        # x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)
        # x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)
        # x[:, 6] = x[:, 6]

        return x

