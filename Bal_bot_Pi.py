import numpy as np
import matplotlib.pyplot as plt
from time import sleep
from datetime import datetime
from mpu6050 import mpu6050
import scipy.linalg
from gpiozero import Robot, DistanceSensor
from My_Classes import My_complimentary, My_Kalman
from serial import Serial
import RPi.GPIO as GPIO

#%% =================IMPORTANT NOTE===================
'''
gpiozero library uses Broadcom (BCM) pin numbering
for the GPIO pins, as opposed to physical (BOARD)
numbering. Unlike in the RPi.GPIO library,
this is not configurable.
'''
GPIO.setmode(GPIO.BCM) # Initialize the GPIO to use the BCM pin numbering schemes as
                       # gpiozero uses BCM and is not configurable

#====SETUP MOTOR CONTROL PINS ========

l_mot_1 = 6; GPIO.setup(l_mot_1,GPIO.OUT);
l_mot_2 = 13; GPIO.setup(l_mot_2,GPIO.OUT)

r_mot_1 = 19; GPIO.setup(r_mot_1,GPIO.OUT);
r_mot_2 = 26; GPIO.setup(r_mot_2,GPIO.OUT)

#=========== SETUP ENCODER PINS ===============

enc_L = 4; tick_L = 0;  # Left motor 

enc_R = 17; tick_R = 0; # Right motor

ppr = 990; RPM_limit = 5 # RPM below which we read "0"

num_avg_point = 10 # Number of points to be used for exponential avergaing and smoothing out RPM data 

beta = 1- 1/num_avg_point # parameter used for exponential averaging

alpha = 0.985; # Complimentary filter control parameter

sleep(2)

def my_callback_R(channel):
    global tick_R
    tick_R+=1
  
def my_callback_L(channel):
    global tick_L
    tick_L+=1
    
GPIO.setup(enc_L, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set Pull left encoder pin to up 

GPIO.add_event_detect(enc_L, GPIO.RISING, callback=my_callback_L) # Set the interrupt functions for left motor

GPIO.setup(enc_R, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set Pull right encoder pin to up

GPIO.add_event_detect(enc_R, GPIO.RISING, callback=my_callback_R) # Set the interrupt functions for right motor

class my_motors:
    
    def __init__(self, beta, ppr, position, RPM_limit):

        self.t_prev = datetime.now() # Time during object initialisation

        self.beta = beta; # parameter used for exponential averaging
        
        self.RPM_prev = 0; # Starting RPM value
        
        self.n_prev = 0; # Starting encoder count
        
        self.ppr = ppr # Number of pulses generated per revolution
        
        self.position = position # motor position "L" or "R"
        
        self.RPM_limit = RPM_limit # Below this RPM, send RPM = 0
    
    def get_RPM(self, units = 'rad/s'): # Get motor RPM 
        
        global tick_L, tick_R
        
        if self.position == "L":
            
            n_now = tick_L
        
        elif self.position == "R":
            
            n_now = tick_R            
        
        t_now = datetime.now() # Find what time is it now
        
        delta_t = t_now - self.t_prev  # Calculate time difference between consecutive readings 
        
        dt = delta_t.total_seconds() # Total time difference in seconds

        dn = n_now - self.n_prev # Difference between motor encoder counts
        
        # Calculate RPMs and smooth out the data usinge exponential averaging
        ''' From Andrew Ngs lectrues
        
        V_t = beta * V_(t-1) + (1-beta) * Theta_t 
        
        i.e. more weight is given to previous value and less weight to the present value
        
        '''

        RPM_now = self.beta*self.RPM_prev + (1 - self.beta)*(dn / dt)*(60.0 / self.ppr) # This is Revolutions / min
        
        if RPM_now < self.RPM_limit:
            
            RPM_now = 0 # If RPM is less than the limiting RPM, make it "0"
            
        #RPM_dot = (RPM_now - self.RPM_prev) / dt # Time derivative of omega
        
        self.t_prev = t_now # Make previous time equal to present time
        
        self.n_prev = n_now # Make previous encoder counts equal to present encoder counts 
        
        self.RPM_prev = RPM_now # Make previous RPM equal to present RPM
        
        if units == 'rpm':
            dd = RPM_now # Return RPM in RPM
        else:
            dd = (RPM_now*2*3.14/60) # Return rpm in [rad/s]        
        
        return dd
    
def lqr(A,B,Q,R):
    """Solve the continuous time lqr controller.
     
    dx/dt = A x + B u
     
    cost = integral x.T*Q*x + u.T*R*u
    """
    #ref Bertsekas, p.151
 
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
     
    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
     
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    return np.array(K), np.array(X), np.array(eigVals)


#====================MAIN COMPUTING LOOP ===================

#cont = PID(Kp = 0, Kd = 0 , Ki = 0, 
           #SetPoint = 0, SampleTime = 200, OutMin = 0.10, OutMax = 1, mode = "Auto") # Initialise PID controller

#arduino = Serial('COM3', baudrate = 115200) # Initialise arduino serial for reading Kp, Kd, and Ki analogue input from the potentiometer

my_mpu = mpu6050(0x68) # Initialize MPU to get acceleration and rotation data
    
left_motor = my_motors(beta, ppr, "L", RPM_limit) # Class / object to get RPM data

right_motor = my_motors(beta, ppr, "R", RPM_limit) # Class / object to get RPM data

My_Mpu = My_complimentary(my_mpu, alpha) # Object for getting theta and theta_dot

my_robot = Robot(left=(l_mot_1, l_mot_2), right=(r_mot_1, r_mot_2)) # Initialise robot object and its pin numbers

my_robot.stop()

# Matrices for getting / adjusting control gain matrix

A = np.array([[0,75.03,0,0],[1,0,0,0],[0,-2.15,0,0],[0,0,0,0]])

B = np.array([[-423.114, -423.114],[0,0],[34.43,34.43],[186.6, -168.6]])

Q = np.zeros((4,4));
Q[0,0] = 1; # Penalty For Angular velocity
Q[1,1] = 1; # Penalty For Angle
Q[2,2] = 1; # Penalty For Linear velocity
Q[3,3] = 1; # Penalty For turning velocity

R = 10*np.array([[1,0],[0,1]])

K, _, _ = lqr(A,B,Q,R)

K_t = 0.007 # Motor torque constant [Nm/A]

R_mot = 20 # Motor resistance [Ohm]

print("Starting the system..!!!")

