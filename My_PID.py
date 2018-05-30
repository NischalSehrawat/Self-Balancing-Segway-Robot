# -*- coding: utf-8 -*-
"""
Created on Wed May 16 20:22:19 2018

@author: Nischal
"""

from datetime import datetime

class PID:
    
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
    
    def get_serial_data(self, my_serial):
        
        # The arduino will send data like <,Kp,Kd,Ki,Theta_x,>\r\n
    
        dd = my_serial.readline().decode().replace("\r\n","").split(',')
        
        if ((dd[0] == "<") and (dd[-1] == ">")): # This has the data that Arduino sent
            
            Kp, Kd, Ki = [float(i) for i in dd[1:-1]]
            
            return Kp, Kd, Ki               
            
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
                
                self.t_start = t_now
            
                self.lastInput = Input
        
            
            else:  # Do nothing if elapsed time < sampletime
                pass
            
    

         
