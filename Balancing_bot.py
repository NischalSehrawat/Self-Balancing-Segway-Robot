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
           SetPoint = 0, SampleTime = 200, OutMin = 30, OutMax = 255) # Initialise controller
