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
    
    def __init__(self, my_mpu, alpha):

        self.my_mpu = my_mpu # Note that the MPU return data in m/s**2 for accelerometer and deg/s for gyro
        self.t_prev = datetime.now() # Time during object initialisation
        
        # We need to caliberate the MPU for errors

        calib = []

        print("Starting MPU caliberation..."); sleep(1)

        for i in range(100):

            xx = [self.my_mpu.get_accel_data()['y'], self.my_mpu.get_accel_data()['z'] - 9.8,
                  self.my_mpu.get_gyro_data()['x']]
            
            calib.append(xx)
            
        calib = np.array(calib)
        
        self.error = np.mean(calib, axis = 0)
        
        print("MPU caliberated, the mean acc [m/s2] and gyro [deg/s] error is ", self.error[0], self.error[1])
        
        sleep(0.5)
        
        print('Calculating initial conditions for Kalman filter')
        
        # Now get the Sensor noise covariance matrix "R"
        
        R = []
        
        for i in range(100):
            
            f1 = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1]))
            f2 = np.self.my_mpu.get_gyro_data()['x'] - self.error[2] 
            
            R.append([f1,f2])
            
        R = np.array(R)        

        '''
        Now we need to initialise the following matrices for the Kalman filter to work
        1) Initial conditions X_0
        2) Error Covariance matrix P
        3) Process noise covariance matrix Q which tells how noisy our process is
        4) Sensor noise covariance matrix which tells how noisy our sensors are
        '''           
            
        self.A = np.array([[1,-1], [0,1]]) # System A matrix (n_states * n_states) for describing system dynamics
        self.B = np.array([[1], [0]]) # System B matrix used for giving input (n_states * 1)        
        self.C = np.array([[1,0]]) # MEasurement matrix i.e. what measurements are we getting from the system (n_sensors*n_states)       
        
        self.X_0 = np.transpose(np.mean(R, axis = 0)) # These are the initial conditions (n_states * 1)
        self.P = np.diag([0.01, 0.01]) # Error covariance matrix initialised (n_states * n_states)
        self.Q = np.diag([0.01, 0.01]) # Process noise covariance matrix (n_states * n_states) contains variance (std**2 of both states)
        self.R = (np.std(R[:,0]))**2 # Sensor noise covariance matrix for accelerometer

        




        






