# -*- coding: utf-8 -*-
"""
Created on Wed May 16 20:22:19 2018

@author: Nischal
"""

import numpy as np
from time import sleep
from datetime import datetime

class PID_fixed_loop:
    
    def __init__(self, Kp, Kd, Ki, SetPoint, SampleTime, OutMin, OutMax, mode):
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.setpoint = SetPoint
        self.sampletime = SampleTime # Sample time in milliseconds
        self.Kd_scaled = self.Kd / (self.sampletime / 1000) # Scaled Kd
        self.Ki_scaled = self.Ki * (self.sampletime / 1000) # Scaled Ki
        self.Integral_Term = 0 # This term will collect integral term sums
        self.t_start = datetime.now() # This term checks whether the PID loop must be executed
        self.lastInput = 0 # Term used for storing last input used for calculating dInput
        self.OutMin = OutMin # Minimum value of output i.e. output till which the controller doesn't respond (Dead band)
        self.OutMax = OutMax # Maximum value of output
        self.error = 0 # To get the error term while PID Tuning
        self.Output_ratio = (1-self.OutMin / self.OutMax) # Used for Scaling the output from (0 to Outmax) to (Outmin to Outmax) 
        self.mode = mode; # Attribute to turn PID "ON" or "OFF"
    
    def set_tunings(self, kp, kd, ki): # Suppose we want to set parameters (using potentiometer) while the alorithm is running
        
        self.Kp = kp # Set Kp to new kp
        self.Kd = kd # Set Kd to new kd
        self.Ki = ki # Set Ki to new ki
        self.Kd_scaled = self.Kd / (self.sampletime / 1000) # Scaled new Kd
        self.Ki_scaled = self.Ki * (self.sampletime / 1000) # Scaled new Ki
    
    def get_serial_data(self, my_serial, kp_max, kd_max, ki_max):
        
        # The arduino will send data like <,Kp,Kd,Ki,>\r\n
    
        dd = my_serial.readline().decode().replace("\r\n","").split(',')
        
        if ((dd[0] == "<") and (dd[-1] == ">")): # This has the data that Arduino sent
            
            Kp, Kd, Ki = [float(i) for i in dd[1:-1]]
            
            return Kp*(kp_max/1024), Kd*(kd_max/1024), Ki*(ki_max/1024)               
            
        else:
            
            pass     
        
    def Compute_PID_Output(self, Input):
        
        if self.mode == "Manual":
            
            pass
        
        elif self.mode == "Auto":
            
            t_now = datetime.now()
        
            dt = t_now - self.t_start
        
            dt_millisec = dt.total_seconds() * 1000 # time change in milliseconds
                    
            if (dt_millisec >= self.sampletime): # Execute calculations only if elapsed time is greater than sampling time
            
                self.error = self.setpoint - Input # Error term
            
                self.Integral_Term = self.Integral_Term + self.Ki_scaled * self.error # This makes it possible to change Ki on while the algorithm is running
            
                dInput = (Input - self.lastInput) #  Removes “Derivative Kick” effect.
            
                Output = self.Kp * self.error + self.Integral_Term - self.Kd_scaled * dInput
            
                '''
                Clamp Output and Integral term to take care of Reset Windup
                '''
            
                if (Output > self.OutMax):
                    self.Integral_Term = self.Integral_Term - self.Ki_scaled * self.error # Keep Integral term same
                    Output = self.OutMax # Set Output to maximum output limit
                    
                    return abs(Output)
                
                elif (Output < -self.OutMax):
                    self.Integral_Term = self.Integral_Term + self.Ki_scaled * self.error # Keep Integral term same
                    Output = -self.OutMax # Set Output to minimum output limit
                    
                    return abs(Output)
                
                elif (-self.OutMax <= Output <= self.OutMax):
                    
                    # if the Ouput lies between -Out_max to + Out_max, scale it to +Out_min to +Out_max
                    
                    Output_scaled = float(format(self.Output_ratio * abs(Output) + self.OutMin, '.2f')) # Only take 2 significant figures after decimal
                                        
                    return Output_scaled
                
                self.t_start = t_now # Store the previous time variable for calculating "dt" and sampling time
            
                self.lastInput = Input # Store last input to calculate dInput
        
            
            else:  # Do nothing if elapsed time < sampletime
                pass

				
class PID_variable_loop:
    
    def __init__(self, Kp, Kd, Ki, SetPoint, OutMin, OutMax):
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.setpoint = SetPoint
        self.Integral_Term = 0 # This term will collect integral term sums
        self.t_start = datetime.now() # This term checks whether the PID loop must be executed
        self.lastInput = 0 # Term used for storing last input used for calculating dInput
        self.OutMin = OutMin # Minimum value of output i.e. output till which the controller doesn't respond (Dead band)
        self.OutMax = OutMax # Maximum value of output
        self.error = 0 # To get the error term while PID Tuning
        self.Output_ratio = (1-self.OutMin / self.OutMax) # Used for Scaling the output from (0 to Outmax) to (Outmin to Outmax) 
    
    def set_tunings(self, kp, kd, ki): # Suppose we want to set parameters (using potentiometer) while the alorithm is running
        
        self.Kp = kp # Set Kp to new kp
        self.Kd = kd # Set Kd to new kd
        self.Ki = ki # Set Ki to new ki

    
    def get_serial_data(self, my_serial, kp_max, kd_max, ki_max):
        
        # The arduino will send data like <,Kp,Kd,Ki,>\r\n
    
        dd = my_serial.readline().decode().replace("\r\n","").split(',')
        
        if ((dd[0] == "<") and (dd[-1] == ">")): # This has the data that Arduino sent
            
            Kp, Kd, Ki = [float(i) for i in dd[1:-1]]
            
            return Kp*(kp_max/1024), Kd*(kd_max/1024), Ki*(ki_max/1024)               
            
        else:
            
            pass     
        
    def Compute_PID_Output(self, Input):
                   
        t_now = datetime.now()
    
        dt = t_now - self.t_start
    
        dt_sec = dt.total_seconds()  # time change in seconds
        
        self.error = self.setpoint - Input # Error term
    
        self.Integral_Term = self.Integral_Term + self.Ki * self.error * dt_sec # This makes it possible to change Ki on while the algorithm is running
    
        dInput = (Input - self.lastInput) #  Removes “Derivative Kick” effect.
    
        Output = self.Kp * self.error + self.Integral_Term - self.Kd_scaled * dInput / dt_sec
    
        '''
        Clamp Output and Integral term to take care of Reset Windup
        '''
    
        if (Output > self.OutMax):
            self.Integral_Term = self.Integral_Term - self.Ki_scaled * self.error # Keep Integral term same
            Output = self.OutMax # Set Output to maximum output limit
            
            return abs(Output)
        
        elif (Output < -self.OutMax):
            self.Integral_Term = self.Integral_Term + self.Ki_scaled * self.error # Keep Integral term same
            Output = -self.OutMax # Set Output to minimum output limit
            
            return abs(Output)
        
        elif (-self.OutMax <= Output <= self.OutMax):
            
            # if the Ouput lies between -Out_max to + Out_max, scale it to +Out_min to +Out_max
            
            Output_scaled = float(format(self.Output_ratio * abs(Output) + self.OutMin, '.2f')) # Only take 2 significant figures after decimal
                                
            return Output_scaled
        
        self.t_start = t_now # Store the previous time variable for calculating "dt" and sampling time
    
        self.lastInput = Input # Store last input to calculate dInput

		
class My_Kalman:
    
    def __init__(self, my_mpu, caliberate = True):

        self.my_mpu = my_mpu # Note that the MPU return data in [m/s**2] for accelerometer and [deg/s] for gyro
        self.t_prev = datetime.now() # Time when object is initialised
        
        if caliberate == True:# We need to caliberate the MPU for errors        

            calib = []
    
            print("Starting MPU caliberation..."); sleep(1)
    
            for i in range(100):
    
                xx = [self.my_mpu.get_accel_data()['y'], self.my_mpu.get_accel_data()['z'] - 9.8,
                      self.my_mpu.get_gyro_data()['x']]
                
                calib.append(xx)
                
            calib = np.array(calib) # Make is a 100*3 matrix
            
            self.error = np.mean(calib, axis = 0)
            
            print("MPU  caliberated, corrections Y, Z, Omega_x = ", round(self.error[0], 2), round(self.error[1], 2), round(self.error[2], 2))
            
            sleep(0.5)
        
        else:
            
            self.error = np.array([0.4,-0.48,-1.41])
        
        
        print('Calculating initial conditions for Kalman filter')
        
   
        R = [] # For initial values and sensor covariance matrix
        
        for i in range(100):
            
            f1 = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1]))
            f2 = np.deg2rad(self.my_mpu.get_gyro_data()['x'] - self.error[2]) # Gyro bias [rad] 
            
            R.append([f1,f2]) # 100 samples of angle and angular velocity (bias in gyro) in [rad] and [rad/s] respectively
            
        R = np.array(R); # Make it an array of 100*2
       
        init_conditions = np.mean(R, axis = 0).reshape(2,1) # Get initial conditions in 2*1 form
       
        print('Calculated initial states for Kalman filter Acc angle, Gyro bias (constant inaccuracy) ', init_conditions)    
            
        self.B = np.array([[1], [0]]) # System B (input) matrix used for giving input (n_states * 1)        
        self.C = np.array([[1,0]]) # Matrix to map state values onto sensor values (n_sensors*n_states) 
        
        print('B and C matrices initialised, A matrix will be calulated while calculating angle')
        
        '''
        Now we need to initialise the following matrices for the Kalman filter to work
        1) Initial conditions X_0 (n_states*1)
        2) Error Covariance matrix P (n_states*n_states)
        3) Process noise covariance matrix Q which tells how noisy our process is (n_states*n_states)
        4) Sensor noise covariance matrix R which tells how noisy our sensors are (n_sensors*n_sensors)
        '''
        print('Initialising X_0, Q, P and R matrices')
        self.X_0 = init_conditions # These are the initial conditions (n_states * 1)
        self.P = np.random.rand(2,2)*np.eye(2) # Error covariance matrix initialised (n_states * n_states)
        self.Q = np.diag([0.001, 0.003]) # Process noise covariance matrix (n_states * n_states) contains variance (std**2 of both states)
        self.R = (np.std(R[:,0]))**2 # Sensor noise covariance matrix for accelerometer
        
        print('Initialised X_0, Q, P and R matrices, system ready')
        
    def get_angle(self, units = 'rad'):
        
        # Get the time elapsed first
        
        t_now = datetime.now()            
                    
        dt = (t_now - self.t_prev).total_seconds()# Total time difference in seconds
        
        # Step 1: Initialise A matrix and predict state. The system model is X_k = A*X_(k-1) + B*U(k)
        
        self.A = np.array([[1,-dt],[0,1]]) # System A matrix (n_states * n_states) for describing system dynamics
        
        U = np.deg2rad(self.my_mpu.get_gyro_data()['x'] - self.error[2]) # Input angular velocity in [rad/s]
        
        X_now = np.dot(self.A,self.X_0) + self.B*dt*U
      
        # Step 2: Project error covariance matrix, The process noise is added here and multiplied by dt 
        #as it has got added over time to the plant 
        
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q*dt
        
        # Step 3: Calculate Kalman gain and update measurements
        
        self.Kf = np.dot(np.dot(self.P, self.C.T),np.linalg.inv(np.dot(np.dot(self.C, self.P), self.C.T) + self.R)) 
        
        # Step 3a Take angle reading now
        
        Z = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1])) 
        
        # Step 3b Update the predicted values
        
        X_now = X_now + np.dot(self.Kf, Z - np.dot(self.C,X_now))
        
        # Step 3c Update Error covariance matrix
        
        self.P = np.dot((np.eye(2,2)-np.dot(self.Kf, self.C)), self.P)
        
        theta = X_now[0,0]; 

        theta_dot = np.deg2rad(self.my_mpu.get_gyro_data()['x'] - self.error[2]) - X_now[1,0]
       
        # Save variables for next step
        
        self.X_0 = X_now # State prev = state now for next step calculation
                
        self.t_prev = t_now # Time now = time prev for next step
        
        if units == 'deg':
            aa, bb = np.rad2deg(theta), np.rad2deg(theta_dot)
        else:
            aa,bb = theta,theta_dot
        
        return aa,bb


class My_complimentary:
    
    def __init__(self, my_mpu, alpha, caliberate = True):

        self.my_mpu = my_mpu
        self.t_prev = datetime.now() # Time during object initialisation
        self.alpha = alpha # parameter for controlling Gyro and accelerometer contribution    
        
        if caliberate == True: # We need to caliberate the MPU for errors

            calib = []
    
            print("Starting MPU caliberation..."); sleep(2)
    
            for i in range(100):
    
                xx = [self.my_mpu.get_accel_data()['y'], self.my_mpu.get_accel_data()['z']-9.8,
                      self.my_mpu.get_gyro_data()['x']]
                
                calib.append(xx)
    
            self.error = np.mean(np.array(calib), axis = 0)
    
            print("MPU  caliberated, corrections Y, Z, Omega_x = ", round(self.error[0], 2), round(self.error[1], 2), round(self.error[2], 2))

        else:
            
            self.error = np.array([0.4,-0.48,-1.43])
        
        self.Theta_x = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1]))
        self.theta_init = self.Theta_x # Used for calculating theta_dot        
        self.omega_x_prev = self.my_mpu.get_gyro_data()['x'] - self.error[2] # Initial omega_x
        
        sleep(2)
        
        
    def get_angle(self, units = 'rad'): # Get tilt angle  
        
        #   Rotated Angle = previous_velocity * timestep
        
        omega_x_now = self.my_mpu.get_gyro_data()['x'] - self.error[2]  # Gyro data now
        
        t_now = datetime.now() # Find what time is it now
        
        delta_t = t_now - self.t_prev  # Calculate time difference between consecutive readings 
        
        dt = delta_t.total_seconds() # Total time difference in seconds
        
        roll = np.arctan((self.my_mpu.get_accel_data()['y'] - self.error[0]) / (self.my_mpu.get_accel_data()['z'] - self.error[1]))
        
        self.Theta_x = self.alpha*(self.Theta_x + np.deg2rad(self.omega_x_prev * dt)) + (1-self.alpha)*roll # Calculate the total angle using a Complimentary filter
              
        theta_dot = (self.Theta_x - self.theta_init) / dt # Time derivative of theta
        
        self.theta_init = self.Theta_x # Make previous angle equal to new angle for calculating derivative
        
        self.omega_x_prev = omega_x_now # For calculating rotation angle for next time step
        
        self.t_prev = t_now # Store the previous time variable for calculating "dt_gyro" 
        
        if units == 'deg':
            
            aa, bb = np.rad2deg(self.Theta_x), np.rad2deg(theta_dot)
            
        else:
            aa,bb = self.Theta_x, theta_dot
        
        return aa,bb
