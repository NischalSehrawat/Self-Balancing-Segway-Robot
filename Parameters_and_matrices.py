# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

#%% parameters 

m_0 = 2 # total Mass of the body [Kg]

m_wh = 1; # MAss of one wheel [kg]

m_mot = 0.2 # Mass of 1 motor stator [Kg]

r = 0.5 * 0.085 # Wheel radius [m]

b = 0.1 # Chassis length (along the axle) [m]

a = 0.08 # Chassis width (perpendicular to the axle) [m]

m_plat = 0.1 # mass of 1 wooden platform [kg]

m_bolt = 0.1 # Mass of 1 bolt [kg]

d1 = 0.0325 # distance of the axle from the lower woodden platform [m]

L1 = 0.1; # Length of 1 bolt [m]

J_wh = 12 ; # Moment of inertia of the wheel and motor rotating parts [kgm2]

J_mot_y = J_wh # It is assumed that the I_y of motor stator = I of wheel + gearbox

def get_Jy():
    
    '''
    This function is used for calculating the moment of inertia of the
    robot main body about the Y axis or the axle. 
    '''
    J_lp = m_plat*a*a/12 + m_plat*d1**2 # I for lower platform about the axle
    
    J_up = m_plat*a*a/12 + m_plat*(L1+d1)**2 # I for upper platform about the axle
    
    J_bolt = (m_bolt*L1**2)/12 + m_bolt*(L1/2+d1)**2 + m_bolt*(a/2)**2
    
    J_y_total = J_lp + J_up + 4*J_bolt + J_mot_y
    
    return J_y_total

J_y = get_Jy() # Moment of inertia of the main body about its principle Y axis

J_z = 52 # Moment of inertia of the main body about its principle Z axis

l = 0.5; # Distance of vertical CG of the body from wheel centre [m]

g = 9.8 # Acceleration due to gravity [m/s2]

J5 = J_z + 0.75*m_wh*b**2 + 0.5*m_wh*r*r # Moment of inertia

M = 2*m_0*m_wh*l*l*r*r + 2*J_wh*m_0*l**2 +  J_y*m_0*r**2 + 2*J_y*m_wh*r**2 + 2*J_wh*J_y

#%% A and B matrices

A = np.zeros((4,4)); B = np.zeros((4,2))

A[1,0] = 1 ; A[0, 1] = (m_0*r*r+2*m_wh*r*r+2*J_wh)*m_0*l*g / M

A[2,1] = -m_0*m_0*l*l*r*r*g / M

B[0,0] = -(l*m_0*r+m_0*r*r+2*m_wh*r*r+2*J_wh)/M

B[2,0] = r*(l*l*m_0+l*m_0*r+J_y)/M; B[3,1] = b/(2*r*J5)

