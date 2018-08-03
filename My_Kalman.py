import numpy as np
from time import sleep
from datetime import datetime


#%% =================IMPORTANT NOTE===================
'''
gpiozero library uses Broadcom (BCM) pin numbering
for the GPIO pins, as opposed to physical (BOARD)
numbering. Unlike in the RPi.GPIO library,
this is not configurable.
'''
    
class My_Kalman:
    
    def __init__(self, my_mpu, alpha, n_sensors, n_states):

        self.my_mpu = my_mpu # Note that the MPU return data in m/s**2 for accelerometer and deg/s for gyro
        self.t_prev = datetime.now() # Time during object initialisation
        self.n_sensors = n_sensors # Number of sensors 
        self.n_states = n_states # NUmber of states that the model has
        
        # We need to caliberate the MPU for errors

        calib = []

        print("Starting MPU caliberation..."); sleep(2)

        for i in range(100):

            xx = [self.my_mpu.get_accel_data()['y'], self.my_mpu.get_accel_data()['z'] - 9.8,
                  self.my_mpu.get_gyro_data()['x']]
            
            calib.append(xx)
            
        calib = np.array(calib)
        
        self.error = np.mean(calib, axis = 0)
        
        # Now get the Sensor noise covariance matrix "R"
        
        for i in range(100):
            
            f1 = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1]))
            f2 = self.my_mpu.get_gyro_data()['x'] - self.error[2] 
            
            
            
        self.A = np.zeros((self.n_states, self.n_states)) # System A matrix (n_states * n_states) for describing system dynamics
        self.B = np.zeros((self.n_states,1)) # System B matrix used for giving input
        self.Q = np.zeros((self.n_states, self.n_states)) # System noise covariance matrix contains variance (std**2 of both states)




        






