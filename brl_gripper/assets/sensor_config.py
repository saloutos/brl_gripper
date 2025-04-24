# define config variables for all states here

# imports
import numpy as np

# general params class to add attributes to for each state
class SensorParams:
    pass

sensor_params = SensorParams()

# sensor type options are ellipsoid or sphere
sensor_params.sensor_type = "ellipsoid" # options: "sphere" or "ellipsoid"
sensor_params.model_type = "mlp" # options: "binned rnn" or "mlp"

# sensor neural nets

# =========== ELLIPSOID ===============
sensor_params.model_fname_rsensor = "2025-02-21_08-52-28_MLP_non_binned_FA9"
# sensor_params.model_fname_rsensor = "2025-02-19_17-28-17_RNN_binned_FA9_batchsize_50"
sensor_params.offsets_rsensor = np.array([5912, 1205, 238, 1977, 3033, 1277, 1674, 994]) #FA9
sensor_params.slopes_rsensor = np.array([240.17321041464288, 367.56589391364446, 264.77124152654756, 453.85881131466397, 533.159799006922, 395.189626066646, 396.8763027465099, 409.7251636887756])


sensor_params.model_fname_lsensor = "2025-03-26_17-09-44_MLP_non_binned_FA8_data2" 
# sensor_params.model_fname_lsensor = "2025-03-11_10-36-14_RNN_binned_FA8_data2" # BINNED RNN
sensor_params.offsets_lsensor = np.array([3100,2500,1700,2200,2100,1870,1760,3750])
# sensor_params.offsets_rsensor = np.array([3100,2500,1700,2200,2100,1870,1760,3750]) #FA8
sensor_params.slopes_lsensor = np.array([456.25135706515795, 372.7567734113597, 322.79580840835393, 426.7609308657926, 448.9935435570265, 388.45638967114104, 356.7922175800903, 446.7329413278805])



# =========== SPHERE ===================
# sensor_params.model_fname_rsensor = "2025-02-08_18-49-50_RNN_binned_E10"
# # sensor_params.model_fname_rsensor = "2025-02-08_21-11-06_MLP_non_binned_E10"
# sensor_params.offsets_rsensor = np.array([5376, 5389, 5375, 5835, 3247, 5096, 5802, 4084]) # E10
# sensor_params.slopes_rsensor = np.array([360.7244108673874, 360.7244108673874, 343.0226864295943, 312.5955622939977, 425.90789408735026, 351.03190534887864, 326.9040834972972, 377.0242331854851]) 


# sensor_params.model_fname_lsensor = "2025-02-08_18-49-50_RNN_binned_E10" # BINNED RNN
# # sensor_params.model_fname_lsensor = "2025-02-08_21-11-06_MLP_non_binned_E10"
# sensor_params.offsets_lsensor = np.array([5376, 5389, 5375, 5835, 3247, 5096, 5802, 4084]) # E10
# sensor_params.slopes_lsensor = np.array([360.7244108673874, 360.7244108673874, 343.0226864295943, 312.5955622939977, 425.90789408735026, 351.03190534887864, 326.9040834972972, 377.0242331854851]) 