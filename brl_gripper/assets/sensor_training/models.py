import math
import torch
import torch.nn as nn
from ..sensor_config import *


# ====== OLD - used for FA7 ============

# class BinnedRNNSensorNet(nn.Module):
#     if sensor_params.sensor_type == "sphere":
#         #spherical range
#         theta_range = [-torch.pi / 4, torch.pi / 4]
#         phi_range = [-3 * torch.pi / 4, torch.pi / 4]
#     elif sensor_params.sensor_type == "ellipsoid":
#         #ellipsoid range
#         theta_range = [-torch.pi / 4.5, torch.pi / 4.5]
#         phi_range = [-2 * torch.pi / 5, 2*torch.pi / 5]
#     else:
#         print("sensor type is not known")

#     def __init__(self, hidden_size=64, num_layers=1, linear_layers = [64,], dropout_p=0.2, bin_width=torch.pi/16, full_out=False):
#         # bin_width is 0.15 rad or 8.59437 deg
#         # Maybe make it n_bins for theta and n_bins for phi
#         # And have bin range.
#         super(BinnedRNNSensorNet, self).__init__()
#         assert hidden_size == linear_layers[0], "first linear layer must be history size"

#         self.bin_width = bin_width
#         self.n_theta_bins = int((self.theta_range[1] - self.theta_range[0]) // bin_width)
#         self.n_phi_bins = int((self.phi_range[1] - self.phi_range[0]) // bin_width)
#         # outputs are Fx, Fy, Fz, Fn, [n_theta_bins], [n_phi_bins], contact_flag

#         print(self.n_theta_bins)
#         print(self.n_phi_bins)
#         output_size = 4 + self.n_theta_bins + self.n_phi_bins + 1
#         linear_layers.append(output_size)


#         self.rnn = nn.RNN(8, hidden_size, num_layers, batch_first=True, dropout=dropout_p)
#         layers = []
#         for i in range(len(linear_layers) - 1):
#             layers.append(nn.Linear(linear_layers[i], linear_layers[i + 1]))

#             if i != (len(linear_layers) - 2):
#                 layers.append(nn.ReLU())
#                 layers.append(nn.Dropout(p=dropout_p))
#         self.fc = nn.Sequential(*layers)

#         self.log_softmax = torch.nn.LogSoftmax(dim=-1)
#         self.full_out = full_out


#         print(self.rnn)
#         print(self.fc)
#         print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
#     def forward(self, x):
#         # x should be N x L x 8
#         x = x.view(x.size(0), x.size(1), -1)

#         h0 = torch.randn(self.rnn.num_layers, x.size(0), self.rnn.hidden_size, device=x.device)
#         x, _ = self.rnn(x, h0)

#         if self.full_out:
#             x = self.fc(x)  # N x L x 8
#         else:
#             x = self.fc(x[:, -1])  # N x 8
#         Fxyzn = x[..., 0:4]
#         theta_bins = x[..., 4:4 + self.n_theta_bins]
#         phi_bins = x[..., 4 + self.n_theta_bins: 4 + self.n_theta_bins + self.n_phi_bins]
#         contact_flag = x[..., -1:]

#         log_theta_probs = self.log_softmax(theta_bins)
#         log_phi_probs = self.log_softmax(phi_bins)
#         contact_prob = torch.sigmoid(contact_flag)
        
#         x = torch.cat((Fxyzn, log_theta_probs, log_phi_probs, contact_prob), dim=-1)
#         return x

#     def theta_idx_to_angle(self, idx):
#         # Center in bin
#         return self.theta_range[0] + self.bin_width * idx + self.bin_width / 2

#     def phi_idx_to_angle(self, idx):
#         return self.phi_range[0] + self.bin_width * idx + self.bin_width / 2
    
#     def std_forward(self, x):
#         x = self.forward(x).detach()
#         if self.full_out:
#             x = x[:, -1, :]

#         Fxyzn = x[:, 0:4]
#         theta_probs= x[:, 4:4 + self.n_theta_bins]
#         phi_probs = x[:, 4 + self.n_theta_bins: 4 + self.n_theta_bins + self.n_phi_bins]
#         contact_flag = x[:, -1:]

#         theta_idx = torch.max(theta_probs, dim=1, keepdim=True)[1]
#         phi_idx = torch.max(phi_probs, dim=1, keepdim=True)[1]

#         theta = self.theta_idx_to_angle(theta_idx)
#         phi = self.phi_idx_to_angle(phi_idx)


#         x = torch.cat((Fxyzn, theta, phi, contact_flag), dim=1)

#         x[:, 0] = torch.clamp(x[:, 0], -20, 20)
#         x[:, 1] = torch.clamp(x[:, 1], -20, 20)
#         x[:, 2] = torch.clamp(x[:, 2], -20, 20)
#         x[:, 3] = torch.clamp(x[:, 3], -40, 0)
#         # x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)
#         # x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)
#         # x[:, 6] = x[:, 6]

#         return x


class BinnedRNNSensorNet(nn.Module):
    #spherical range
    theta_range_sphere = [-torch.pi / 4, torch.pi / 4]
    phi_range_sphere = [-3 * torch.pi / 4, torch.pi / 4]
 
    #ellipsoid range
    theta_range_ellipsoid = [-torch.pi / 4.5, torch.pi / 4.5]
    phi_range_ellipsoid = [-2 * torch.pi / 5, 2*torch.pi / 5]

    #ellipsoid range
    # theta_range_ellipsoid = [-torch.pi / 5.3, torch.pi / 5.3]
    # phi_range_ellipsoid = [-3 * torch.pi / 10, 3 * torch.pi / 10]

    # theta_range = [-torch.pi / 4, torch.pi / 4]
    # phi_range = [-3 * torch.pi / 4, torch.pi / 4]

    # theta_range = [-torch.pi / 5.3, torch.pi / 5.3]
    # phi_range = [-3 * torch.pi / 10, 3 * torch.pi / 10]

    # theta_range = [-torch.pi / 4.5, torch.pi / 4.5]
    # phi_range = [-2 * torch.pi / 5, 2*torch.pi / 5]

    def __init__(self, hidden_size=64, num_layers=1, linear_layers = [64,], dropout_p=0.2, bin_width=torch.pi/16, theta_range=theta_range_ellipsoid, phi_range=phi_range_ellipsoid, full_out=False):
        # bin_width is 0.15 rad or 8.59437 deg
        # Maybe make it n_bins for theta and n_bins for phic
        # And have bin range.
        super(BinnedRNNSensorNet, self).__init__()
        assert hidden_size == linear_layers[0], "first linear layer must be history size"

        self.bin_width = bin_width
        self.theta_range = theta_range
        self.phi_range = phi_range
        self.n_theta_bins = int((self.theta_range[1] - self.theta_range[0]) // bin_width)
        self.n_phi_bins = int((self.phi_range[1] - self.phi_range[0]) // bin_width)
        self.theta_angles = torch.asarray([self.theta_idx_to_angle(i) for i in range(self.n_theta_bins)]).cuda()
        self.phi_angles = torch.asarray([self.phi_idx_to_angle(i) for i in range(self.n_phi_bins)]).cuda()
        # outputs are Fx, Fy, Fz, Fn, [n_theta_bins], [n_phi_bins], contact_flag

        print("theta bins: ", self.n_theta_bins)
        print("phi bins: ", self.n_phi_bins)
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

        x, hn = self.rnn(x, h0) 

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
        return x, hn

    def theta_idx_to_angle(self, idx):
        # Center in bin
        return self.theta_range[0] + self.bin_width * idx + self.bin_width / 2

    def phi_idx_to_angle(self, idx):
        return self.phi_range[0] + self.bin_width * idx + self.bin_width / 2
    
    def std_forward(self, x, h0 = None):

        # ***** OG ********
        x, hn = self.forward(x)
        x  = x.detach()
        hn = hn.detach()

        if self.full_out:
            x = x[:, -1, :]

        # Fxyzn = x[:, 0:4]
        # theta_probs= x[:, 4:4 + self.n_theta_bins]
        # phi_probs = x[:, 4 + self.n_theta_bins: 4 + self.n_theta_bins + self.n_phi_bins]
        # contact_flag = x[:, -1:]

        # theta_idx = torch.max(theta_probs, dim=1, keepdim=True)[1]
        # phi_idx = torch.max(phi_probs, dim=1, keepdim=True)[1]

        # theta = self.theta_idx_to_angle(theta_idx)
        # phi = self.phi_idx_to_angle(phi_idx)

        # Validation using weighted thetas for compare experiments
        # print("x shape: " , x.shape)

        Fxyzn = x[:, 0:4]
        theta_bins = x[..., 4:4 + self.n_theta_bins]
        phi_bins = x[..., 4 + self.n_theta_bins:4 + self.n_theta_bins + self.n_phi_bins]
        contact_flag = x[..., -1:]
        # print("og before weighted calc: ",Fxyzn.shape, theta_bins.shape, phi_bins.shape, contact_flag.shape)
        theta_probs = torch.softmax(theta_bins, dim=1)
        phi_probs = torch.softmax(phi_bins, dim=1)

        weighted_theta = torch.sum(theta_probs * self.theta_angles, dim = 1).unsqueeze(1)
        weighted_phi = torch.sum(phi_probs * self.phi_angles, dim = 1).unsqueeze(1)

        x = torch.cat((Fxyzn, weighted_theta, weighted_phi, contact_flag), dim=1)

        print(Fxyzn.shape, weighted_theta.shape, weighted_phi.shape, contact_flag.shape)

        x[:, 0] = torch.clamp(x[:, 0], -20, 20)
        x[:, 1] = torch.clamp(x[:, 1], -20, 20)
        x[:, 2] = torch.clamp(x[:, 2], -20, 20)
        x[:, 3] = torch.clamp(x[:, 3], -40, 0)
        x[:, 4] = torch.clamp(x[:, 4], self.theta_range[0], self.theta_range[1])
        x[:, 5] = torch.clamp(x[:, 5], self.phi_range[0], self.phi_range[1])
        return x, hn
    


class MLPSensorNet(nn.Module):
    #spherical range
    theta_range_sphere = [-torch.pi / 4, torch.pi / 4]
    phi_range_sphere = [-3 * torch.pi / 4, torch.pi / 4]
 
    # ellipsoid range
    theta_range_ellipsoid = [-torch.pi / 4.5, torch.pi / 4.5]
    phi_range_ellipsoid = [-2 * torch.pi / 5, 2*torch.pi / 5]

    # #ellipsoid range
    # theta_range_ellipsoid = [-torch.pi / 5.3, torch.pi / 5.3]
    # phi_range_ellipsoid = [-3 * torch.pi / 10, 3 * torch.pi / 10]

    def __init__(self, hidden_layers=[64, 64], dropout_p=0.2, theta_range = theta_range_ellipsoid, phi_range = phi_range_ellipsoid):
        super(MLPSensorNet, self).__init__()

        self.theta_range = theta_range
        self.phi_range = phi_range
        # Define output size
        # Outputs: Fx, Fy, Fz, Fn, theta, phi, contact_flag
        output_size = 4 + 2 + 1  # Forces (4) + angles (2) + contact flag (1)

        # Define the MLP layers
        layers = []
        input_size = 8  # Number of input features
        for hidden_size in hidden_layers:
            layers.append(nn.Linear(input_size, hidden_size))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(p=dropout_p))
            input_size = hidden_size

        # Output layer
        layers.append(nn.Linear(input_size, output_size))
        self.mlp = nn.Sequential(*layers)

        print(self.mlp)
        print(f"Number of parameters: {sum(p.numel() for p in self.parameters())}")

    def forward(self, x):
        # x should be N x 8 (no temporal dimension for MLP)
        x = self.mlp(x)

        # Separate outputs
        Fxyzn = x[..., 0:4]
        theta = x[..., 4:5]  # Continuous angle for theta
        phi = x[..., 5:6]    # Continuous angle for phi
        contact_flag = x[..., -1:]

        # Apply sigmoid to contact flag to get probability
        contact_prob = torch.sigmoid(contact_flag)

        # Combine outputs
        x = torch.cat((Fxyzn, theta, phi, contact_prob), dim=-1)
        return x

    def std_forward(self, x):
        # Standard forward pass with detaching
        x = self.forward(x).detach()

        Fxyzn = x[:, 0:4]
        theta = x[:, 4:5]
        phi = x[:, 5:6]
        contact_flag = x[:, -1:]

        Fxyzn[:, 0] = torch.clamp(Fxyzn[:, 0], -20, 20)  # Fx
        Fxyzn[:, 1] = torch.clamp(Fxyzn[:, 1], -20, 20)  # Fy
        Fxyzn[:, 2] = torch.clamp(Fxyzn[:, 2], -20, 20)  # Fz
        Fxyzn[:, 3] = torch.clamp(Fxyzn[:, 3], -40, 0)   # Fn
        theta = torch.clamp(theta, self.theta_range[0], self.theta_range[1])
        phi = torch.clamp(phi, self.phi_range[0], self.phi_range[1])

        # Combine results
        x = torch.cat((Fxyzn, theta, phi, contact_flag), dim=1)
        return x
