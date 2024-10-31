# define config variables for all states here

# imports
import numpy as np

# general params class to add attributes to for each state
class SensorParams:
    pass

sensor_params = SensorParams()

# sensor type options are ellipsoid or sphere
sensor_params.sensor_type = "ellipsoid" #

# sensor neural nets
sensor_params.rnn_model_fname_lsensor = "2024-08-06_10-59-05_FA7"
# rnn_model_fname_lsensor = "2024-07-09_18-04-38_E9_6_38_and_E9_7_3_BinnedFulloutRNN_hd48_H512_k64_bpi32_lr0p0005"


sensor_params.rnn_model_fname_rsensor = "2024-07-29_17-49-57_FA5_lowforce"
# rnn_model_fname_rsensor = "2024-08-08_21-14-39_E10"

