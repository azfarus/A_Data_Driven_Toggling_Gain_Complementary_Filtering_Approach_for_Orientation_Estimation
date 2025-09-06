import os
import time
import smbus
import numpy as np
from imusensor.MPU9250 import MPU9250
import pickle
from utils.utils import norm,skew_symmetric_from_vector,predict_alpha ,quatFromAccMag,count_folders , detect_motion
from scipy.linalg import expm
from pyquaternion import Quaternion
import warnings
warnings.filterwarnings("ignore")


## path of the current script
script_directory = os.path.dirname(os.path.abspath(__file__))
print("Script directory:", script_directory)

## data write
file = open(script_directory+'/data/quats.csv','w+')
file.write('w,x,y,z\n')

## MPU9250 SETUP
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.setSRD(1)
imu.begin()
np.set_printoptions(precision=4, suppress=True)

imu.Magtransform = np.array(
    [   
        [ 0.944 , -0.0444,  0.0793, ],
        [-0.0444,  0.8777, -0.0305, ],
        [ 0.0793, -0.0305,  1.2172  ]
    ]
)

imu.MagBias = np.array([ 0.5709,  3.6827, -2.9334])



## Model loading
model_path_noise = "/home/azfarus/Thesis/Models/RF/RF_noise_9input_selected_data_2025_05_21_23_22.pkl"  # Replace with your actual file path
model_path_movement = "/home/azfarus/Thesis/Models/Tree/DecisionTree_9input_movement_2025_05_21_00_25.pkl"  # Replace with your actual file path


##filter params
sampling_rate =285
alpha = 0.001
dh=120
dl=60
noise_on=True
movement_on=True
axis="X"


if noise_on:
    with open(model_path_noise, "rb") as f:
        model_noise = pickle.load(f)
        
if movement_on:
    with open(model_path_movement, "rb") as f:
        model_movement = pickle.load(f)
    
## filter variables
acc_prev = np.zeros(3) 
mag_prev = np.zeros(3)
gyr_bias = np.zeros(3)
a_noise=np.zeros(3)
m_noise=np.zeros(3)
movement = [False]
delta = 1.0/sampling_rate



import json
# folder opening
i=count_folders("data/")
run_folder = f"run{i}"
f = open(f"data/{run_folder}/togglefilt.csv",mode="a+")
metadata_filepath = f"data/{run_folder}/togglefilt_metadata.json"

metadata = {
    "sampling_rate": sampling_rate,
    "alpha": alpha,
    "dh": dh,
    "dl": dl,
    "noise_on": noise_on,
    "movement_on": movement_on,
    "axis":axis
}
with open(metadata_filepath, "w") as meta_f:
    json.dump(metadata, meta_f, indent=4)

## bias and inizialization
n=2000
for i in range(n):
    
    start_time = time.perf_counter()
    imu.readSensor()
    acc_prev+=(imu.AccelVals/norm(imu.AccelVals))
    mag_prev+=(np.array([imu.MagVals[1],imu.MagVals[0],-imu.MagVals[2]])/norm(imu.MagVals))
    gyr_bias+=imu.GyroVals
     
acc_prev/=n
mag_prev/=n
gyr_bias/=n




print(f"Start of filtering TOGGLE, acc_norm = {np.linalg.norm(imu.AccelVals)}, mag_norm= {np.linalg.norm(imu.MagVals)}")

init_q = quatFromAccMag(acc_prev,mag_prev)



while True:
        
    start_time = time.perf_counter()
    
    imu.readSensor()
    g = imu.GyroVals
    g_nobias = g-gyr_bias
    mag_correct = np.array([imu.MagVals[1],imu.MagVals[0],-imu.MagVals[2]])
    
    g_norm=norm(g)
    g_norm_nobias=norm(g_nobias)
    a_norm=norm(imu.AccelVals)
    m_norm=norm(mag_correct)  
    
    
    g_unit = imu.GyroVals/g_norm
    a_unit = imu.AccelVals/a_norm
    m_unit = mag_correct/m_norm
    g_unit_nobias=g_nobias/g_norm_nobias
    
    
    sample_input = [[*g_unit,*a_unit,*m_unit]]

    
    
    if noise_on:
        noise = model_noise.predict(sample_input)
        a_noise=noise[0][0:3]
        m_noise=noise[0][3:6]
    if movement_on:
        movement = model_movement.predict(sample_input)
    
    
    omega = expm(skew_symmetric_from_vector(-g_nobias)*delta)
    
    acc_gyr = omega @ acc_prev
    mag_gyr = omega @ mag_prev
    
    cur_alpha = predict_alpha(i,movement[0],alpha,dh,dl)
    
    gyr_acc_err_norm = norm(a_unit - acc_gyr)
    gyr_mag_err_norm = norm(m_unit - mag_gyr)

    model_acc_err_norm = norm(a_noise)
    model_mag_err_norm = norm(m_noise)
    
    #Avoid division by zero with small epsilon
    eps = 1e-8
    gyr_acc_alpha = (cur_alpha * gyr_acc_err_norm) / (model_acc_err_norm + gyr_acc_err_norm + eps)
    gyr_mag_alpha = (cur_alpha * gyr_mag_err_norm) / (model_mag_err_norm + gyr_mag_err_norm + eps)

    model_acc_alpha = model_acc_err_norm / (model_acc_err_norm + gyr_acc_err_norm + eps)
    model_mag_alpha = model_mag_err_norm / (model_mag_err_norm + gyr_mag_err_norm + eps)

    acc_vec = acc_gyr + gyr_acc_alpha * (a_unit - model_acc_alpha * a_noise - acc_gyr)
    mag_vec = mag_gyr + gyr_mag_alpha * (m_unit - model_mag_alpha * m_noise - mag_gyr)
    

    acc_prev = acc_vec /norm(acc_vec)
    mag_prev = mag_vec /norm(mag_vec)
    
    
    q = quatFromAccMag(acc_prev,mag_prev)

    
    timestamp_ms = int(time.time() * 1000)

    # Prepare line to write
    elements = (Quaternion(init_q).inverse * Quaternion(q)).elements
    line = f"{timestamp_ms}, {elements[0]}, {elements[1]}, {elements[2]}, {elements[3]}\n"

    # Write to file (append mode)
    f.write(line)
    
        
    
    while (time.perf_counter()-start_time)<delta: pass

    
