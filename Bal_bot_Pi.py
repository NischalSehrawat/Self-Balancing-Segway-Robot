from numpy import arctan, deg2rad
from time import sleep
from datetime import datetime
from mpu6050 import mpu6050
from gpiozero import Robot, DistanceSensor
from My_PID import PID
from serial import Serial
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD) # Initialize the GPIO to use the BOARD pin numbering schemes

#%%=============== Setup motor control + encoder pins to output / input ==================

l_mot_1 = 11; GPIO.setup(l_mot_1,GPIO.OUT); l_mot_2 = 12; GPIO.setup(l_mot_2,GPIO.OUT)

r_mot_1 = 17; GPIO.setup(r_mot_1,GPIO.OUT); r_mot_2 = 15; GPIO.setup(r_mot_2,GPIO.OUT)

#=========== SETUP ENCODER PINS ===============

enc_L_1 = 22; GPIO.setup(enc_L_1,GPIO.IN); enc_L_2 = 25; GPIO.setup(enc_L_2,GPIO.IN);

enc_R_1 = 29; GPIO.setup(enc_R_1,GPIO.IN); enc_R_2 = 30; GPIO.setup(enc_R_2,GPIO.IN);

#%% ===============================================================

cont = PID(Kp = 0, Kd = 0 , Ki = 0, 
           SetPoint = 0, SampleTime = 200, OutMin = 0.10, OutMax = 1, mode = "Auto") # Initialise PID controller

arduino = Serial('COM3', baudrate = 115200) # Initialise arduino serial for reading Kp, Kd, and Ki analogue input from the potentiometer

my_mpu = mpu6050(0x68) # Initialize MPU to get acceleration and rotation data

robot = Robot(left=(l_mot_1, l_mot_2), right=(r_mot_1, r_mot_2)) # Initialise robot object and its pin numbers

a_y, a_z = my_mpu.get_accel_data()['y'], my_mpu.get_accel_data()['z'] # Acceleration data

Theta_x = arctan(a_y/a_z ); # Rotation about X axis [rad]

t_start = datetime.now() # Timer for gyro unit

alpha = 0.98; # Complimentary filter control parameter

sleep(1)

while 1:
    
    omega_x = my_mpu.get_gyro_data()['x'] # Gyro data
    
    t_now = datetime.now() # Log what time it is now
    
    dt_gyro = (t_now - t_start).total_seconds() # Time difference for gyro angle calculations
    
    a_y, a_z = my_mpu.get_accel_data()['y'], my_mpu.get_accel_data()['z'] # Acceleration data
    
    roll = arctan(a_y/a_z) # Angle calculated by accelerometer readings about X axis in [rad]
  
    Theta_x = alpha*(Theta_x + deg2rad(omega_x * dt_gyro)) + (1-alpha)*roll # Calculate the total angle using a Complimentary filter
    
    
    
    
    
    
    
    
    
    t_start = t_now # Store the previous time variable for calculating "dt_gyro"  
    
    
