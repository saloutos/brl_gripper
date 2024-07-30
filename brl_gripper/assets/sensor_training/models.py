import math
import torch
import torch.nn as nn

class OneHotSensorNet(nn.Module):
    def __init__(self):
        super(OneHotSensorNet, self).__init__()

        self.nn = nn.Sequential(
            nn.Linear(8, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 27),  # 9 + 18 = 27 (range of onehot)
            nn.Sigmoid(),
        )

        # First 9 are theta, next 18 are phi

    def forward(self, x):
        return self.nn(x)


class SmallSensorNet(nn.Module):
    def __init__(self):
        super(SmallSensorNet, self).__init__()

        self.nn = nn.Sequential(
            nn.Linear(8, 32), nn.ReLU(), nn.Linear(32, 32), nn.ReLU(), nn.Linear(32, 5)
        )

    def forward(self, x):
        return self.nn(x)


class DefaultSensorNet(nn.Module):
    def __init__(self):
        super(DefaultSensorNet, self).__init__()

        self.nn = nn.Sequential(
            nn.Linear(8, 12),
            nn.ReLU(),
            nn.Linear(12, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 5),
        )

    def forward(self, x):
        return self.nn(x)


class HistorySensorNet(nn.Module):
    def __init__(self, history_length):
        super(HistorySensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        self.nn = nn.Sequential(
            nn.Linear(8 * history_length, 1024),
            nn.ReLU(),
            nn.Linear(1024, 512),
            nn.ReLU(),
            nn.Linear(512, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 5),
        )

        # self.nn = nn.Sequential(
        #     nn.Linear(8 * history_length, 64),
        #     nn.ReLU(),
        #     nn.Linear(64, 64),
        #     nn.ReLU(),
        #     nn.Linear(64, 64),
        #     nn.ReLU(),
        #     nn.Linear(64, 5),
        # )

    def forward(self, x):
        return self.nn(x)

class ContactFlagSensorNet(nn.Module):
    def __init__(self, history_length):
        super(ContactFlagSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        self.nn = nn.Sequential(
            nn.Linear(8 * history_length, 1024),
            nn.ReLU(),
            nn.Linear(1024, 512),
            nn.ReLU(),
            nn.Linear(512, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 6),
        )

    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)
        return x

    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)

        x[0] = torch.clamp(x[0], -10, 10)   # Fx
        x[1] = torch.clamp(x[1], -10, 10)   # Fy
        x[2] = torch.clamp(x[2], 0, 20)     # Fz
        x[3] = torch.clamp(x[3], -torch.pi/4, torch.pi/4)     # theta
        x[4] = torch.clamp(x[4], -3 * torch.pi/4, torch.pi/4)     # phi
        x[5] = x[5] > 0.5

        return x


class MediumContactFlagSensorNet(nn.Module):
    def __init__(self, history_length):
        super(MediumContactFlagSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        self.nn = nn.Sequential(
            nn.Linear(8 * history_length, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 6),
        )


    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)
        return x
    
    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)

        x[0] = torch.clamp(x[0], -10, 10)   # Fx
        x[1] = torch.clamp(x[1], -10, 10)   # Fy
        x[2] = torch.clamp(x[2], 0, 20)     # Fz
        x[3] = torch.clamp(x[3], -torch.pi/4, torch.pi/4)     # theta
        x[4] = torch.clamp(x[4], -3 * torch.pi/4, torch.pi/4)     # phi
        x[5] = x[5] > 0.5

        return x


class SmallContactFlagSensorNet(nn.Module):
    def __init__(self, history_length):
        super(SmallContactFlagSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        self.nn = nn.Sequential(
            nn.Linear(8 * history_length, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 6),
        )


    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)
        return x
    
    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)

        x[0] = torch.clamp(x[0], -10, 10)   # Fx
        x[1] = torch.clamp(x[1], -10, 10)   # Fy
        x[2] = torch.clamp(x[2], 0, 20)     # Fz
        x[3] = torch.clamp(x[3], -torch.pi/4, torch.pi/4)     # theta
        x[4] = torch.clamp(x[4], -3 * torch.pi/4, torch.pi/4)     # phi
        x[5] = x[5] > 0.5

        return x


class DefaultContactFlagSensorNet(nn.Module):
    def __init__(self, history_length):
        super(DefaultContactFlagSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        self.nn = nn.Sequential(
            nn.Linear(8 * history_length, 12),
            nn.ReLU(),
            nn.Linear(12, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 6),
        )


    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)
        return x
    
    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)

        x[0] = torch.clamp(x[0], -10, 10)   # Fx
        x[1] = torch.clamp(x[1], -10, 10)   # Fy
        x[2] = torch.clamp(x[2], 0, 20)     # Fz
        x[3] = torch.clamp(x[3], -torch.pi/4, torch.pi/4)     # theta
        x[4] = torch.clamp(x[4], -3 * torch.pi/4, torch.pi/4)     # phi
        x[5] = x[5] > 0.5

        return x


class DeepContactFlagSensorNet(nn.Module):
    def __init__(self, history_length):
        super(DeepContactFlagSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        self.nn = nn.Sequential(
            nn.Linear(8 * history_length, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Linear(16, 6),
        )


    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)
        return x

    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)

        x[0] = torch.clamp(x[0], -10, 10)   # Fx
        x[1] = torch.clamp(x[1], -10, 10)   # Fy
        x[2] = torch.clamp(x[2], 0, 20)     # Fz
        x[3] = torch.clamp(x[3], -torch.pi/4, torch.pi/4)     # theta
        x[4] = torch.clamp(x[4], -3 * torch.pi/4, torch.pi/4)     # phi
        x[5] = x[5] > 0.5

        return x


# -- New stuff -- #

class UniveresalContactFlagSensorNet(nn.Module):
    def __init__(self, history_length, layer_sizes=[1024, 512, 128, 64]):
        super(UniveresalContactFlagSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        layers = []
        layers.append(nn.Linear(8 * history_length, layer_sizes[0]))

        for i in range(1, len(layer_sizes)):
            layers.append(nn.ReLU())
            layers.append(nn.Linear(layer_sizes[i-1], layer_sizes[i]))
        
        layers.append(nn.Linear(layer_sizes[-1], 6))
        
        self.nn = nn.Sequential(*layers)

    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)
        return x

    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        # Does not store gradient

        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1).detach()  # Will be out of memory otherwise

        x[:, 0] = torch.clamp(x[:, 0], -10, 10)   # Fx
        x[:, 1] = torch.clamp(x[:, 1], -10, 10)   # Fy
        x[:, 2] = torch.clamp(x[:, 2], -20, 0)     # Fz
        x[:, 3] = torch.clamp(x[:, 3], -torch.pi/4, torch.pi/4)     # theta
        x[:, 4] = torch.clamp(x[:, 4], -3 * torch.pi/4, torch.pi/4)     # phi
        x[:, 5] = x[:, 5] > 0.5

        return x


class UniveresalDropoutContactFlagSensorNet(nn.Module):
    def __init__(self, history_length, layer_sizes=[1024, 512, 128, 64, 7], dropout_p=0.2):
        super(UniveresalDropoutContactFlagSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        layers = []
        layers.append(nn.Linear(8 * history_length, layer_sizes[0]))

        for i in range(1, len(layer_sizes)):
            nn.Dropout(p=dropout_p)
            layers.append(nn.ReLU())
            layers.append(nn.Linear(layer_sizes[i-1], layer_sizes[i]))
        
        self.nn = nn.Sequential(*layers)

        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))

    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1)
        return x

    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        # Does not store gradient

        x = self.nn(x)
        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1).detach()

        x[:, 0] = torch.clamp(x[:, 0], -20, 20)   # Fx
        x[:, 1] = torch.clamp(x[:, 1], -20, 20)   # Fy
        x[:, 2] = torch.clamp(x[:, 2], -20, 20)   # Fy
        x[:, 3] = torch.clamp(x[:, 3], -40, 0)     # Fz
        x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)     # theta
        x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)     # phi
        x[:, 6] = x[:, 6]

        return x


class UniversalDerivativeSensorNet(nn.Module):
    def __init__(self, history_period, layer_sizes=[1024, 512, 128, 64, 7], dropout_p=0.2):
        super(UniversalDerivativeSensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        layers = []
        layers.append(nn.Linear(8 * 2, layer_sizes[0]))

        for i in range(1, len(layer_sizes)):
            nn.Dropout(p=dropout_p)
            layers.append(nn.ReLU())
            layers.append(nn.Linear(layer_sizes[i-1], layer_sizes[i]))
        
        self.nn = nn.Sequential(*layers)
        self.history_period = history_period

        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
    def derivative(self, x, history_period):
        # x is B x 16  # Most recent is the latter index

        dx = (x[:, 8:] - x[:, :8]) / history_period
        return dx  # B x 8

    def forward(self, x):
        dx = self.derivative(x, self.history_period)
        x = self.nn(torch.cat((x[:, 8:], dx), dim=1))
        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1)
        return x

    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        # Does not store gradient

        dx = self.derivative(x, self.history_period)
        x = self.nn(torch.cat((x[:, 8:], dx), dim=1))
        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1).detach()

        x[:, 0] = torch.clamp(x[:, 0], -10, 10)   # Fx
        x[:, 1] = torch.clamp(x[:, 1], -10, 10)   # Fy
        x[:, 2] = torch.clamp(x[:, 2], -10, 10)   # Fy
        x[:, 3] = torch.clamp(x[:, 3], -20, 0)     # Fz
        x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)     # theta
        x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)     # phi
        x[:, 6] = x[:, 6]

        return x


class UniversalBinary16SensorNet(nn.Module):
    def __init__(self, history_length, layer_sizes=[1024, 512, 128, 64], dropout_p=0.2, ):
        # Basically Float16 version of UniversalDropoutContactFlagSensorNet
        super(UniversalBinary16SensorNet, self).__init__()

        # x is a vector of length 8*history_length where the last 8 values are the most recent
        layers = []
        layers.append(nn.Linear(8 * history_length, layer_sizes[0]))

        for i in range(1, len(layer_sizes)):
            nn.Dropout(p=0.2)
            layers.append(nn.ReLU())
            layers.append(nn.Linear(layer_sizes[i-1], layer_sizes[i]))
        
        nn.Dropout(p=0.2)
        layers.append(nn.Linear(layer_sizes[-1], 6))
        
        self.nn = nn.Sequential(*layers).type(torch.float16)


    def forward(self, x):
        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1)
        return x

    def std_forward(self, x):
        # Return output in standard format:
        # Fx, Fy, Fz, theta, phi, contact where each is clipped to the correct range

        # Does not store gradient

        x = self.nn(x)
        x = torch.cat((x[:, :5], torch.sigmoid(x[:, 5:])), dim=1).detach()

        x[:, 0] = torch.clamp(x[:, 0], -10, 10)   # Fx
        x[:, 1] = torch.clamp(x[:, 1], -10, 10)   # Fy
        x[:, 2] = torch.clamp(x[:, 2], -20, 0)     # Fz
        x[:, 3] = torch.clamp(x[:, 3], -torch.pi/4, torch.pi/4)     # theta
        x[:, 4] = torch.clamp(x[:, 4], -3 * torch.pi/4, torch.pi/4)     # phi
        x[:, 5] = x[:, 5]

        return x


class ConvSensorNet(nn.Module):
    def __init__(self, history_length, channels=[32, 64, 128], dropout_p = 0.2):
        super(ConvSensorNet, self).__init__()

        assert history_length % 3**3== 0, "History length must be divisible by 3^3"

        L_final =  int(history_length / (3**3))
        print(L_final)

        conv_layers = [
            nn.Conv1d(8, channels[0], 3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(3),

            nn.Conv1d(channels[0], channels[1], 3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(3),

            nn.Conv1d(channels[1], channels[2], 3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(3),
            nn.Flatten(),

            nn.Dropout(p=dropout_p),
            nn.Linear(L_final * channels[2], 64),
            nn.ReLU(),

            nn.Dropout(p=dropout_p),
            nn.Linear(64, 7),
        ]

        self.nn= nn.Sequential(*conv_layers)

        print(self.nn)
        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
    def forward(self, x):
        x = x.view(x.shape[0], 8, -1)  # N x C x L
        x = self.nn(x)
        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1)
        return x
    
    def std_forward(self, x):
        x = x.view(x.shape[0], 8, -1)
        x = self.nn(x)
        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1).detach()

        x[:, 0] = torch.clamp(x[:, 0], -20, 20)   # Fx
        x[:, 1] = torch.clamp(x[:, 1], -20, 20)   # Fy
        x[:, 2] = torch.clamp(x[:, 2], -20, 20)   # Fy
        x[:, 3] = torch.clamp(x[:, 3], -40, 0)     # Fz
        x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)     # theta
        x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)     # phi
        x[:, 6] = x[:, 6]

        return x



class HybridSensorNet(nn.Module):
    def __init__(self, history_length, channels=[32, 64, 128], dropout_p = 0.2):
        super(HybridSensorNet, self).__init__()

        assert history_length % 3**3== 0, "History length must be divisible by 3^3"

        L_final =  int(history_length / (3**3))
        print(L_final)

        conv_layers = [
            nn.Conv1d(8, channels[0], 3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(3),

            nn.Conv1d(channels[0], channels[1], 3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(3),

            nn.Conv1d(channels[1], channels[2], 3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(3),
            nn.Flatten(),
            nn.Dropout(p=dropout_p),
        ]
        linear_layers = [
            nn.Linear(L_final * channels[2] + 8, 64),
            nn.ReLU(),

            nn.Dropout(p=dropout_p),
            nn.Linear(64, 64),
            nn.ReLU(),

            nn.Dropout(p=dropout_p),
            nn.Linear(64, 7),
        ]

        self.conv = nn.Sequential(*conv_layers)
        self.linear = nn.Sequential(*linear_layers)

        print(self.conv)
        print(self.linear)
        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
    def forward(self, x):
        recent_values = x[:, -8:]  # N x 8

        x = x.view(x.shape[0], 8, -1)  # N x C x L
        x = self.conv(x)  # N x L_final * 128

        x_in = torch.zeros(x.shape[0], x.shape[1] + 8, device=x.device)
        x_in[:, :-8] = x
        x_in[:, -8:] = recent_values
        x = self.linear(x_in)

        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1)
        return x
    
    def std_forward(self, x):
        x = self.forward(x).detach()

        x[:, 0] = torch.clamp(x[:, 0], -20, 20)   # Fx
        x[:, 1] = torch.clamp(x[:, 1], -20, 20)   # Fy
        x[:, 2] = torch.clamp(x[:, 2], -20, 20)   # Fy
        x[:, 3] = torch.clamp(x[:, 3], -40, 0)     # Fz
        x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)     # theta
        x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)     # phi
        x[:, 6] = x[:, 6]

        return x


class RNNSensorNet(nn.Module):
    def __init__(self, hidden_size=64, num_layers=2, linear_layers = [64, 7], dropout_p=0.2):

        super(RNNSensorNet, self).__init__()
        assert hidden_size == linear_layers[0], "first linear layer must be history size"

        self.rnn = nn.RNN(8, hidden_size, num_layers, batch_first=True, dropout=dropout_p)
        layers = []
        for i in range(len(linear_layers) - 1):
            layers.append(nn.Linear(linear_layers[i], linear_layers[i + 1]))

            if i != (len(linear_layers) - 2):
                layers.append(nn.ReLU())
                layers.append(nn.Dropout(p=dropout_p))
        self.fc = nn.Sequential(*layers)

        print(self.rnn)
        print(self.fc)
        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
    def forward(self, x):
        # x should be N x L x 8
        x = x.view(x.size(0), x.size(1), -1)

        h0 = torch.randn(self.rnn.num_layers, x.size(0), self.rnn.hidden_size, device=x.device)
        x, _ = self.rnn(x, h0)
        x = self.fc(x[:, -1])
        x = torch.cat((x[:, :-1], torch.sigmoid(x[:, -1:])), dim=1)
        return x
    
    def std_forward(self, x):
        x = self.forward(x).detach()

        x[:, 0] = torch.clamp(x[:, 0], -20, 20)
        x[:, 1] = torch.clamp(x[:, 1], -20, 20)
        x[:, 2] = torch.clamp(x[:, 2], -20, 20)
        x[:, 3] = torch.clamp(x[:, 3], -40, 0)
        x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)
        x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)
        x[:, 6] = x[:, 6]

        return x


class FullOutRNNSensorNet(nn.Module):
    def __init__(self, hidden_size=64, num_layers=2, linear_layers=[64, 7], dropout_p=0.2):
        super(FullOutRNNSensorNet, self).__init__()

        assert hidden_size == linear_layers[0], "first linear layer must be history size"

        self.rnn = nn.RNN(8, hidden_size, num_layers, batch_first=True, dropout=dropout_p)

        layers = []
        for i in range(len(linear_layers) - 1):
            layers.append(nn.Linear(linear_layers[i], linear_layers[i + 1]))

            if i != (len(linear_layers) - 2):
                layers.append(nn.ReLU())
                layers.append(nn.Dropout(p=dropout_p))
        self.fc = nn.Sequential(*layers)

        print(self.rnn)
        print(self.fc)
        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
    def forward(self, x):
        # x should be N x L x 8
        x = x.view(x.size(0), x.size(1), -1)

        h0 = torch.randn(self.rnn.num_layers, x.size(0), self.rnn.hidden_size, device=x.device)
        x, _ = self.rnn(x, h0)
        # fc on all outputs 
        x = self.fc(x)

        # x is shape N x L x 7

        x = torch.cat((x[:, :, :-1], torch.sigmoid(x[:, :, -1:])), dim=2)
        return x
    
    def std_forward(self, x):
        x = self.forward(x).detach()
        x = x[:, -1]  # Only take the last output for display


        x[:, 0] = torch.clamp(x[:, 0], -20, 20)
        x[:, 1] = torch.clamp(x[:, 1], -20, 20)
        x[:, 2] = torch.clamp(x[:, 2], -20, 20)
        x[:, 3] = torch.clamp(x[:, 3], -40, 0)
        x[:, 4] = torch.clamp(x[:, 4], -torch.pi/4, torch.pi/4)
        x[:, 5] = torch.clamp(x[:, 5], -3 * torch.pi/4, torch.pi/4)
        x[:, 6] = x[:, 6]

        return x


class BinnedRNNSensorNet(nn.Module):

    theta_range = [-torch.pi / 4, torch.pi / 4]
    phi_range = [-3 * torch.pi / 4, torch.pi / 4]

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


class MultiHeadSensorNet(nn.Module):
    def __init__(self, base_net_name: nn.Module, head_net_name: nn.Module, num_heads: int, base_net_kwargs={}, head_net_kwargs={}, freeze_base=False):
        super(MultiHeadSensorNet, self).__init__()

        base_net = eval(base_net_name)
        head_net = eval(head_net_name)

        self.base_net = base_net(**base_net_kwargs)
        self.heads = nn.ModuleList([head_net(**head_net_kwargs) for _ in range(num_heads)])

        print(self.base_net)
        print(self.heads)

        if freeze_base:
            for param in self.base_net.parameters():
                param.requires_grad = False
        
        print("Base net parameters: ", sum(p.numel() for p in self.base_net.parameters()))
        print("Head net parameters: ", sum(p.numel() for p in self.heads.parameters()))
        print("Single head parameters: ", sum(p.numel() for p in self.heads.parameters()) / num_heads)
        print("Total number of parameters: ", sum(p.numel() for p in self.parameters()))
        print("Number of trainable parameters: ", sum(p.numel() for p in self.parameters() if p.requires_grad))
    

    def apply_heads(self, x):
         
        # Assume head idx is the last element of the input
        x_out = torch.zeros(*x.shape[:-1], x.shape[-1] - 1, device=x.device) # Don't use empty
        for head_idx in range(len(self.heads)):
            x_mask = (x[..., -1] == head_idx)
            head_output = self.heads[head_idx](x[..., :-1])
            x_out += head_output * x_mask.unsqueeze(-1)


        # print(x[..., :10])
        # print(x_out[..., :10])

        # # Apply different head to different data
        # for head_idx in range(len(self.heads)):
        #     x_mask = x[..., -1] == head_idx
        #     x[x_mask][..., :-1] = self.heads[head_idx](x[x_mask][..., :-1])
        # x = x[..., :-1]
        # print(x.shape)
        return x_out
    
    def forward(self, x):

        # Apply different head to different data
        x = self.apply_heads(x)
        x = self.base_net(x)
        return x


    def std_forward(self, x):
        x = self.apply_heads(x)
        # x = x[..., :-1]
        x = self.base_net.std_forward(x)
        return x


class SimpleHead(nn.Module):
    def __init__(self):
        super(SimpleHead, self).__init__()
        self.fc = nn.Linear(8, 8)
    
    def forward(self, x):
        return self.fc(x)

class NonFCHead(nn.Module):
    def __init__(self):
        super(NonFCHead, self).__init__()

        self.scale = nn.parameter.Parameter(torch.empty((8,)))
        self.bias = nn.parameter.Parameter(torch.empty((8,)))

        self.reset_parameters()

    def forward(self, x):
        return torch.einsum("...a,a->...a", x, self.scale) + self.bias


    def reset_parameters(self) -> None:
        # Setting a=sqrt(5) in kaiming_uniform is the same as initializing with
        # uniform(-1/sqrt(in_features), 1/sqrt(in_features)). For details, see
        # https://github.com/pytorch/pytorch/issues/57109
        nn.init.uniform_(self.scale, a=0.75, b=1.25)  # Scale factor 
        nn.init.uniform_(self.bias, a=-0.5, b=0.5)  # Bias

        print("Reset head param")
        print(self.scale)
        print(self.bias)

class PassThroughHead(nn.Module):
    def __init__(self):
        super(PassThroughHead, self).__init__()

    def forward(self, x):
        return x

class AutoencodingMLP(nn.Module):
    def __init__(self, encoder_channels=[64, 32, 16], decoder_channels=[16, 32, 64], dropout_p=0.2):
        super(AutoencodingMLP, self).__init__()

        assert encoder_channels[0] % 8 == 0, "First encoder channel must be divisible by 8"

        self.history_length = int(encoder_channels[0] / 8)

        layers = []
        for i in range(len(encoder_channels) - 1):
            layers.append(nn.Linear(encoder_channels[i], encoder_channels[i + 1]))

            if i != len(encoder_channels) - 2:
                layers.append(nn.ReLU())

            # if i != 0 and i != len(encoder_channels) - 1:
            #     layers.append(nn.Dropout(p=dropout_p))
        
        self.encoder = nn.Sequential(*layers)

        layers = []
        for i in range(len(decoder_channels) - 1):
            layers.append(nn.Linear(decoder_channels[i], decoder_channels[i + 1]))

            if i != len(decoder_channels) - 2:
                layers.append(nn.ReLU())

            # if i != 0 and i != len(decoder_channels) - 1:
            #     layers.append(nn.Dropout(p=dropout_p))
        
        self.decoder = nn.Sequential(*layers)

        print(self.encoder)
        print(self.decoder)
        print("Number of parameters: ", sum(p.numel() for p in self.parameters()))
    
    def forward(self, x):
        x = self.encoder(x)
        x = self.decoder(x)
        return x
        



