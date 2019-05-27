"""
This demo calculates multiple things for different scenarios.

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f


"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from imutils.video.pivideostream import PiVideoStream
import imutils
from serial import Serial
from time import sleep
import RPi.GPIO as GPIO
from threading import Thread
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


#--- DEFINE Serial Communication

Robot = Serial('/dev/ttyUSB0', 115200)
sleep(2)

# -- Define a decision dictionary 

decision = {"balance":'0'.encode(),"go fwd":'1'.encode(),
            "go bck":'2'.encode(),"anti_clockwise":'b'.encode(),
            "clockwise":'c'.encode()}

print("Serial Communication established, now alligning servos..!!")

sleep(2)

#--- Allign servos to look forward  i.e. 90 Degrees

# Define Servo functions / parameters 

pan_servo = 3; # Servo going left and right, this pin number is as per BOARD notation
tilt_servo = 5; # Servo going up and down, this pin number is as per BOARD notation
GPIO.setup(pan_servo, GPIO.OUT); GPIO.setup(tilt_servo, GPIO.OUT)
pwm_pan = GPIO.PWM(pan_servo, 50) # Frequency set to 50 Hz
pwm_pan.start(8) # Start the servo from any position, this doesnot matter
pwm_tilt = GPIO.PWM(tilt_servo, 50) # Frequency set to 50 Hz
pwm_tilt.start(8) # Start the servo from any position, this doesnot matter

for angle in range(0,91,10):
	duty_cycle = angle / 18 + 2
	pwm_pan.ChangeDutyCycle(duty_cycle) # Goto updated position	
	pwm_tilt.ChangeDutyCycle(duty_cycle) # Goto updated position
	sleep(0.3)

pwm_pan.stop()
pwm_tilt.stop()

print("Servos in position..!!")


#--- Define Tag
id_to_find  = 72
marker_size  = 18 #- [cm]

#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()

vs = PiVideoStream(resolution = (640,480)).start()
time.sleep(2.0) # Allow camera to warmup

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

global dist_2_target

dist_2_target = None # Distance to target reported by camera [cm]
stopping_dist = 100 # Distance when the robot must stop in [cm]


class CameraDistance:
    def __init__(self, vs, aruco_dict, parameters, camera_matrix, camera_distortion, id_to_find, marker_size, font):
        self._running = True
        self.vs = vs
        self.aruco_dict = aruco_dict
        self.parameters = parameters
        self.camera_matrix = camera_matrix
        self.camera_distortion = camera_distortion
        self.id_to_find = id_to_find
        self.marker_size = marker_size

    def terminate(self):  
        self._running = False  

    def run(self):

        global dist_2_target

        while self._running:
            #-- Read the camera frame
            frame = self.vs.read()
            #frame = cv2.flip(image, 1) # Flip the picture horizontally
            #-- Convert in gray scale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                                                         cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)
            if ids is not None and ids[0] == self.id_to_find:
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)
                #-- Unpack the output, get only the first
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
                #-- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                #-- Now get Position and attitude f the camera respect to the marker
                pos_camera = -R_tc*np.matrix(tvec).T
                str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                dist_2_target = int( pos_camera[2][0, 0]) # Distance to maker in cm

            #--- Display the frame
            cv2.imshow('frame', frame)

            #--- use 'q' to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                break

#Create Class
Camera_Dist = CameraDistance(vs, aruco_dict, parameters,camera_matrix,
                             camera_distortion,id_to_find, marker_size, font)
#Create Thread
Camera_Dist_Thread = Thread(target=Camera_Dist.run) 
#Start Thread 
Camera_Dist_Thread.start()

sleep(5)

mode_now = "go fwd"

Robot.write(decision[mode_now]) # Make the robot go forward
print("Going forward")

while 1:
   
        print("Distance to target [cm]: ", dist_2_target)

        if dist_2_target is not None:
                if mode_now == "go fwd":
                        if dist_2_target<stopping_dist:
                                mode_now = "balance"
                                Robot.write(decision[mode_now]) # Stop the robot
                                mode_now = "go bck"
                                sleep(5)
                                print("Distance reached, stopping the robot @ "+str(dist_2_target)+" cm")
                                print("Now going backward")

                elif mode_now == "go bck":
                        Robot.write(decision[mode_now])
                        if dist_2_target>150:
                                mode_now = "balance"
                                Robot.write(decision[mode_now])
                                print("Distance reached, stopping the robot @ "+str(dist_2_target)+" cm")
                                break







