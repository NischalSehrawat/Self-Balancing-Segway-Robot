# -*- coding: utf-8 -*-
"""
Created on Wed Jul 18 11:45:30 2018

@author: Sehn
"""

import numpy as np
import matplotlib.pyplot as plt

#%% parameters 

m_0 = 2 # Mass of the body [Kg]

m_w = 1; # MAss of one wheel [kg]

m_m = 0.2 # Mass of 1 motor [Kg]

r = 0.5 * 0.085 # Wheel radius [m]

b = 0.1 # Chassis length (along the axle) [m]

a = 0.08 # Chassis width (perpendicular to the axle) [m]

m_plat = 0.1 # mass of 1 wooden platform [kg]

m_bolt = 0.1 # Mass of 1 bolt [kg]

d1 = 0.0325 # distance of the axle from the lower woodden platform [m]

L1 = 0.1; # Length of 1 bolt [m]

def get_Jy():
    
    '''
    This function is used for calculating the moment of inertia of the
    robot main body about the Y axis or the axle. The motor is assumed to be
    a point mass
    '''
    J_lp = m_plat*a*a/12 + m_plat*d1**2 # I for lower platform about the axle
    
    J_up = m_plat*a*a/12 + m_plat*(L1+d1)**2 # I for upper platform about the axle

J_y = 52 # Moment of inertia of the main body about its principle Y axis

J_z = 52 # Moment of inertia of the main body about its principle Z axis

J_w = 12 ; # Moment of inertia of the wheel and motor rotating parts [kgm2]

l = 0.5; # Distance of vertical CG of the body from wheel centre [m]

g = 9.8 # Acceleration due to gravity [m/s2]

J5 = J_z + 0.75*m_w*b**2 + 0.5*m_w*r*r # Moment of inertia

M = 2*m_0*m_w*l*l*r*r + 2*J_w*m_0*l**2 +  J_y*m_0*r**2 + 2*J_y*m_w*r**2 + 2*J_w*J_y

#%% A and B matrices

A = np.zeros((4,4)); B = np.zeros((4,2))

A[1,0] = 1 ; A[0, 1] = (m_0*r*r+2*m_w*r*r+2*J_w)*m_0*l*g / M

A[2,1] = -m_0*m_0*l*l*r*r*g / M

B[0,0] = -(l*m_0*r+m_0*r*r+2*m_w*r*r+2*J_w)/M

B[2,0] = r*(l*l*m_0+l*m_0*r+J_y)/M; B[3,1] = b/(2*r*J5)

