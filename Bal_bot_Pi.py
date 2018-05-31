from numpy import arctan
from time import sleep
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

enc_l_1 = 22; GPIO.setup(enc_l_1,GPIO.IN); enc_l_2 = 25; GPIO.setup(enc_l_2,GPIO.IN);

enc_r_1 = 29; GPIO.setup(enc_r_1,GPIO.IN); enc_r_2 = 30; GPIO.setup(enc_r_2,GPIO.IN);

#%% ===============================================================



cont = PID(Kp = 0, Kd = 0 , Ki = 0, 
           SetPoint = 0, SampleTime = 200, OutMin = 0.30, OutMax = 1, mode = "Auto") # Initialise PID controller

arduino = Serial('COM3', baudrate = 115200) # Initialise arduino serial for reading Kp, Kd, and Ki analogue input from the potentiometer

my_mpu = mpu6050(0x68)

robot = Robot(left=(l_mot_1, l_mot_2), right=(r_mot_1, r_mot_2)) # Initialise robot object and its pin numbers

a_y, a_z = my_mpu.get_accel_data()['y'], my_mpu.get_accel_data()['z'] # Acceleration data

omega_x = my_mpu.get_gyro_data()['x'] # Gyro data

roll = (arctan(a_y/a_z ))*57.32; # Rotation about X axis [deg]


