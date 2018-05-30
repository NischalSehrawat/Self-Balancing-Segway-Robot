from numpy import arctan
from time import sleep
from mpu6050 import mpu6050
from gpiozero import Robot
from My_PID import PID

cont = PID(Kp = 0, Kd = 0 , Ki = 0, 
           SetPoint = 0, SampleTime = 200, OutMin = 0.30, OutMax = 1, mode = "Auto") # Initialise PID controller

arduino = serial.Serial('COM3', baudrate = 115200) # Initialise arduino serial for reading Kp, Kd, and Ki analogue input from the potentiometer

mympu = mpu6050(0x68)

robot = Robot(left=(4, 14), right=(17, 27)) # Initialise robot object and its pin numbers

A_Y, A_Z = mympu.get_accel_data()['y'], mympu.get_accel_data()['z'] # Acceleration data

omega_x = mympu.get_gyro_data()['x'] # Gyro data

roll = (arctan(A_Y/A_Z ))*57.32; # Rotation about X axis [deg]


