# -*- coding: utf-8 -*-
"""
Created on Sun May 20 21:27:48 2018

@author: Nischal
"""

import serial 
from datetime import datetime
import time
from numpy import array, zeros, mean, random
from My_PID import PID

cont = PID(Kp = 0, Kd = 0 , Ki = 0, 
           SetPoint = 0, SampleTime = 200, OutMin = 30, OutMax = 255, mode = "Auto") # Initialise controller

arduino = serial.Serial('COM3', baudrate = 115200) # Initialise arduino serial



for i in reversed(range(5)):
    
    print("I am going the start the system in "+(str(i))+ "[sec]" )
    
    time.sleep(1.5)

def is_float(text):
    
    try:
                
        float(text)
        return True
    except Exception: 
        return False

def get_serial_data(self, my_serial):
    
    dd = my_serial.readline().decode()
    
    print(dd)
    
    dd = float(dd.replace("\r\n",""))
    
    return dd

while 1:
    
    data = arduino.readline().decode().replace("\r\n","")
    
    time.sleep(0.03)
        
    if is_float(data):
        
        Theta_x = data
                
        print(" \nTheta_x = ", Theta_x)
        
#        Output_scaled = cont.Compute_PID_Output(Input = Theta_x) # Compute the output of PID Algorithm
#        
#        ff = str(Output_scaled)+','+str(cont.error)
#            
#        arduino.write(ff.encode())
        
#        time.sleep(0.1)
        
        arduino.write("50,2".encode())
#        
#        print("\n Computed data sent TO ARDUINO Output_scaled = ", Output_scaled, "Error = ", cont.error)
                
arduino.close()
