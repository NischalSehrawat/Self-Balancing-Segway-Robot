# import the necessary packages
import numpy as np
from imutils.video.pivideostream import PiVideoStream
import imutils
import time
import cv2
import matplotlib.pyplot as plt
 
# initialize the camera and grab a reference to the raw camera capture

print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream().start()
time.sleep(2.0)

frame1 = vs.read() # Get the frame now
time.sleep(1.0)
 
# capture frames from the camera
while 1:
        
	# grab the raw NumPy array representing the image

	frame2 = vs.read()

	diff = cv2.absdiff(frame1,frame2) # get absolute difference between frames

	gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray, (5,5),0)
	_, th = cv2.threshold(blur, 10, 255, cv2.THRESH_BINARY)
	dilated = cv2.dilate(th, np.ones((3,3), np.uint8), iterations = 2)
	eroded = cv2.erode(dilated, np.ones((3,3), np.uint8), iterations = 2)

	_, cont, _ = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
	if len(cont)>0:
                
                for contour in cont:
                        if cv2.contourArea(contour) < 1000: # If contor area is <1000 pixels go back
                                continue
                        (x,y,w,h) = cv2.boundingRect(contour)
                        cv2.rectangle(frame1, (x,y), (x+w,y+h), (0,255,0), 3)

	#cv2.drawContours(frame1, cont, -1, (255,0,0), 2)

	# show the frame
	cv2.imshow("Frame", frame1)
	key = cv2.waitKey(1) & 0xFF

	frame1 = frame2
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
            break
