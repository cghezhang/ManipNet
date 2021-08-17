# Predict next frame by previous frame
KMP_DUPLICATE_LIB_OK = True
import sys
import os
import zipfile

sys.path.append('../Lib_MainNN_Hand')
import numpy as np
import tensorflow as tf
from Main_EncoderRes3Bin import MainNN


load_path = '../Data/Cylinder/train'
test_path = '../Data/Cylinder/test'
save_path = '../Data/ManipNetBIN'

# unzip
if((not os.path.isdir('../Data/Cylinder')) and os.path.exists('../Data/Cylinder.zip')):
    print("unzip files")
    with zipfile.ZipFile("../Data/Cylinder.zip","r") as zip_ref:
        zip_ref.extractall("../Data")

# Main NN
name_model = "ManipNet"
type_normalization = 0
hiddenDim = 0
activations = tf.nn.relu

# # Encoder NN
start_traj = 192
start_dis = 528
start_end = 2350

index_encoder0 = np.arange(0, start_traj)
dim_encoder0 = [512]
activation_encoder0 = [0]

index_encoder1 = np.arange(start_traj, start_dis)
dim_encoder1 = [512]
activation_encoder1 = [0]

index_encoder2 = np.arange(start_dis, start_end)
dim_encoder2 = [512]
activation_encoder2 = [0]


index_encoders = [index_encoder0, index_encoder1, index_encoder2]
dim_encoders = [dim_encoder0, dim_encoder1, dim_encoder2]
activation_encoders = [activation_encoder0, activation_encoder1, activation_encoder2]

# Keep prob
keep_prob_main = 0.7
keep_prob_encoders = [0.7, 0.7, 0.7]



# Tuning para
learning_rate = 0.0001

def main():
    GPU_Occupancy = 0.9
    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=GPU_Occupancy)
    sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))

    mann = MainNN()
    mann.BuildModel(load_path, save_path,
                    type_normalization,
                    hiddenDim=hiddenDim,
                    activations=activations,

                    index_encoders=index_encoders,
                    dim_encoders=dim_encoders,
                    activation_encoders=activation_encoders)
    mann.Train(sess,
               # Model name for Saving
               name_model,
               # Flag/Path of test data
               path_test=test_path,
               flag_save_bin = True,
               step_save=100, max_save=3,
               # HyperParameters
               keep_prob_main=keep_prob_main, batch_size=32, epoch=500, learning_rate_ini=learning_rate,
               keep_prob_encoders=keep_prob_encoders
               )


if __name__ == '__main__':
    main()
