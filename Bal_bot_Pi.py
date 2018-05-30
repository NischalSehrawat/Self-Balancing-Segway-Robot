from numpy import arctan
import matplotlib.pyplot as plt
from datetime import datetime
from time import sleep
from mpu6050 import mpu6050
from gpiozero import Robot


sensor = mpu6050(0x68)

robot = Robot(left=(4, 14), right=(17, 27)) # Initialise robot object and its pin numbers


A_Y, A_Z = sensor.get_accel_data()['y'], sensor.get_accel_data()['z'] # Acceleration data

omega_x = sensor.get_gyro_data()['x'] # Gyro data

roll = (arctan(A_Y/A_Z ))*57.32; # Rotation about X axis [deg]


