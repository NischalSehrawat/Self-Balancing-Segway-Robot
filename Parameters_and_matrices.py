# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import  scipy.io
from control import *


#%% parameters 

m_wh = 0.058; # MAss of one wheel [kg]

m_mot = 0.2 # Mass of 1 motor stator [Kg]

r = 0.5 * 0.085 # Wheel radius [m]

b = 0.255 # Chassis length (along the axle) [m]

a = 0.1 # Chassis width (perpendicular to the axle) [m]

m_plat = 0.081 # mass of 1 wooden platform [kg]

m_bolt = 0.03 # Mass of 1 bolt [kg]

m_0 = 2*m_mot + 2*m_plat + 4*m_bolt # total Mass of the body [Kg]

d1 = 0.0325 # distance of the axle from the lower woodden platform [m]

L1 = 0.15; # Length of 1 bolt [m]

L_diag = (a**2 + b**2)**0.5 # Digonal distance of the woodden platform [m]

J_wh = 0.025 ; # Moment of inertia of the wheel and motor rotating parts [kg.m2] obtained by experiment

J_mot_y = 2*J_wh # It is assumed that the I_y of motor stator = 2 * (I of wheel + gearbox)

def get_Jy():
    
    '''
    This function is used for calculating the moment of inertia of the
    robot main body about the Y axis or the axle. 
    '''
    J_lp = m_plat*a*a/12 + m_plat*d1**2 # I for lower platform about the axle
    
    J_up = m_plat*a*a/12 + m_plat*(L1+d1)**2 # I for upper platform about the axle
    
    J_bolt = (m_bolt*L1**2)/12 + m_bolt*(L1/2+d1)**2 + m_bolt*(a/2)**2
    
    J_y_total = J_lp + J_up + 4*J_bolt + 2*J_mot_y
    
    return J_y_total

def get_Jz():
    
    '''
    This function is used for calculating the moment of inertia of the
    robot main body about the Z axis . 
    '''
    J_lp = m_plat*(a*a + b*b)/12 # I for 1 platform about Z axis    
  
    J_bolt = m_bolt * (0.5*L_diag)**2
    
    J_mot_z = m_mot * (0.5*b)**2
    
    J_z_total = 2*J_lp + 4*J_bolt + 2*J_mot_z
    
    return J_z_total

J_y = get_Jy() # Moment of inertia of the main body about its principle Y axis

J_z = get_Jz() # Moment of inertia of the main body about its principle Z axis

l = (m_plat*d1+4*m_bolt*(0.5*L1+d1)+m_plat*(L1+d1))/m_0; # Distance of vertical CG of the body from wheel centre [m]

g = 9.8 # Acceleration due to gravity [m/s2]

J5 = J_z + 0.75*m_wh*b**2 + 0.5*m_wh*r*r # Moment of inertia

M = 2*m_0*m_wh*l*l*r*r + 2*J_wh*m_0*l**2 +  J_y*m_0*r**2 + 2*J_y*m_wh*r**2 + 2*J_wh*J_y

#%% A and B matrices

A = np.zeros((4,4)); B = np.zeros((4,2))

A[1,0] = 1 ; A[0, 1] = (m_0*r*r+2*m_wh*r*r+2*J_wh)*m_0*l*g / M

A[2,1] = -m_0*m_0*l*l*r*r*g / M

B[0,0] = -(l*m_0*r+m_0*r*r+2*m_wh*r*r+2*J_wh)/M  # motor torques in terms of sum and difference of motor torques

B[2,0] = r*(l*l*m_0+l*m_0*r+J_y)/M; B[3,1] = b/(2*r*J5)

B_mot_torq = np.dot(B, np.array([[1 ,1],[1,-1]])) # Individual motor torques

Q = np.zeros((4,4)); Q[0,0] = 100; Q[1,1] = 10; Q[2,2] = 1; Q[3,3] = 1; 

R = 10*np.array([[1,0],[0,1]])

a, b, K = lqr(A, B_mot_torq, Q, R)