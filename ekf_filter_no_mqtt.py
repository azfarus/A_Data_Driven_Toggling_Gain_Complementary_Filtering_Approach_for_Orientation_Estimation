import os
import time
import smbus
import numpy as np
from imusensor.MPU9250 import MPU9250
import pickle
from utils.utils import norm,skew_symmetric_from_vector,predict_alpha ,quatFromAccMag,count_folders
from scipy.linalg import expm
from pyquaternion import Quaternion
from ahrs.filters import EKF

## path of the current script
script_directory = os.path.dirname(os.path.abspath(__file__))
print("Script directory:", script_directory)




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


## quaternion publisher
import paho.mqtt.client as mqtt
import time


##filter params
sampling_rate =285


## filter variables
acc_prev = np.zeros(3) 
mag_prev = np.zeros(3)
gyr_bias = np.zeros(3)
delta = 1.0/sampling_rate



imu.caliberateGyro()


total_sum=0

init_q = Quaternion().elements


noise_g = imu.GyroBias[0]**2 + imu.GyroBias[1]**2 + imu.GyroBias[2]**2
noise_a = imu.AccelBias[0]**2 + imu.AccelBias[1]**2 + imu.AccelBias[2]**2
noise_m = imu.MagBias[0]**2 + imu.MagBias[1]**2 + imu.MagBias[2]**2


ekf_filter = EKF(frequency=sampling_rate)
q=init_q

i=count_folders("data/")
run_folder = f"run{i}"


f = open(f"data/{run_folder}/ekf.csv",mode="a+")

print("Start of filtering EKF")
while True:
    
    
    start_time = time.perf_counter()
    
    imu.readSensor()

    mag_correct = np.array([imu.MagVals[1],imu.MagVals[0],-imu.MagVals[2]])
        
    g = imu.GyroVals - imu.GyroBias
    a = imu.AccelVals
    m = mag_correct*1000000000

    q_norm = np.linalg.norm(q)
    if q_norm != 0:
        q = q / q_norm
    
    # print(g,a,m)
    q = ekf_filter.update(q,g,a,m)    
    
    
    while (time.perf_counter()-start_time)<delta: pass
    end_time = time.perf_counter()

    timestamp_ms = int(time.time() * 1000)

    line = f"{timestamp_ms}, {q[0]}, {q[1]}, {q[2]}, {q[3]}\n"

    # Write to file (append mode)
    f.write(line)
    
    
    total_sum +=(end_time-start_time)*1e6
    
    
