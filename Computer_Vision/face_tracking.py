# import the necessary packages
import numpy as np
from imutils.video.pivideostream import PiVideoStream
import imutils
import time
import cv2
import RPi.GPIO as GPIO
import os
 


frame_size = 512 # Size of the frame in pixels

# Define Servo functions / parameters 

pin_pan_servo = 3; # Servo going left and right, this pin number is as per BOARD notation
pin_tilt_servo = 5; # Servo going up and down, this pin number is as per BOARD notation
angle_pan = 90; # Set initial angles to 90
angle_tilt = 90; # Set initial angles to 90
# Define areas of no movement
pan_low_lim = frame_size*0.5 - 10; pan_high_lim = frame_size*0.5 + 10;
tilt_low_lim = frame_size*0.5 - 10; tilt_high_lim = frame_size*0.5 + 10;

#position servos 
def ServoPosition (servo_pin, angle):
    os.system("python Control_Servo.py " + str(servo_pin) + " " + str(angle))

# position servos to present object at center of the frame
def mapServoPosition (x, y):
    
    global angle_pan # Define global angles    
    global angle_tilt

    delta_angle = 5 # Angle to increase for servo

    if (x < pan_low_lim):
        angle_pan += delta_angle
        if angle_pan > 140:
            angle_pan = 140
        ServoPosition(pin_pan_servo, angle_pan)
 
    if (x > pan_high_lim):
        angle_pan -= delta_angle
        if angle_pan < 40:
            angle_pan = 40
        ServoPosition(pin_pan_servo, angle_pan)

    if (y > tilt_low_lim):
        angle_tilt += delta_angle
        if angle_tilt > 140:
            angle_tilt = 140
        ServoPosition(pin_tilt_servo, angle_tilt)
 
    if (y < tilt_high_lim):
        angle_tilt -= delta_angle
        if angle_tilt < 40:
            angle_tilt = 40
        ServoPosition(pin_tilt_servo, angle_tilt)

# Initialise servo positions to 90 degrees

ServoPosition (pin_pan_servo, angle_pan) # Move pan servo to initial position
ServoPosition (pin_tilt_servo, angle_tilt) # Move tilt servo to initial position

# Camera parameters 

print("Facial recognition Starting...")
'''
Constructor (self, resolution=(320->Width, 240-> Height), framerate=32)
So the numpy array will be of [height, width]
'''
vs = PiVideoStream().start()
time.sleep(2.0) # Allow camera to warmup

path = "/home/pi/Desktop/my_cv/data/haarcascades/" # Address of all haar cascades 
face_cascade = cv2.CascadeClassifier(path+"haarcascade_frontalface_alt2.xml")


 
# capture frames from the camera
while 1:

	# grab the raw NumPy array representing the image
	image = vs.read()
	#image = imutils.resize(image, width=400)
	image = cv2.flip(image, 1) # Flip the picture horizontally	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # Get a gray scale image

	'''
	This method will return the detected objects (in this case, the faces) 
	as rectangles, so we can easily mark them in the output image.
	'''
	faces = face_cascade.detectMultiScale(gray, scaleFactor = 1.05, minNeighbors = 5) # Get coordinates of the faces detected

	if len(faces) >0: # If the length of faces is not null

		for x,y,w,h in faces:
			image = cv2.rectangle(image, (x,y), (x+w, y+h), (255,255,0), 2)

		centre_X = x + 0.5 * w; centre_Y = y + 0.5 * h; # Get coordinates of the centre of the rectangle around the detected face
		
		#mapServoPosition(centre_X, centre_Y)
	
	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
