# -*- coding: utf-8 -*-
"""
Created on Tue Apr 30 21:04:04 2019

@author: Nischal
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("Set_2.txt")

df1["DN(Lmot-Rmot)"] = df1["DN_Lmot"] - df1["DN_Rmot"]

#cols2plot = df1.columns

cols2plot = ['Theta_now','V_trans','Output_lmot','DN(Lmot-Rmot)','DN_Lmot','DN_Rmot']
units = ["Deg", "[m/s]", "PWM", "Ticks", "Ticks", "Ticks"]

clr = ['r','g','b','k','b','g', 'r', 'c', 'm']

dt = 0.02 # Sampling time [s]

t = [i*dt for i in range(len(df1)) ]
k1 = 0; k2 = len(df1);

plt.close("all")
plt.figure(figsize = (25,35))
plt.suptitle("Parameters of the robot in rest position subjected to push", fontsize = 16)

for i,col, unit in zip(range(len(cols2plot)), cols2plot, units):
    plt.subplot(len(cols2plot),1,i+1)
    plt.step(t[k1:k2], df1.loc[k1:k2, col], clr[i], label = col)
    plt.legend(loc = "upper right")
    plt.ylabel(unit, fontsize = 14)
    plt.yticks(fontsize = 14)


#a = df1["V_trans"]
#y = np.fft.fft(a) # Getting complex coefficients of the data points in frequency domain
#
#C = np.fft.fftfreq(len(t),dt)# Getting the frequency bins [Hz]
#
#plt.figure()
#plt.subplot(211)
#plt.plot(t,a, label = 'Horizontal direction')
#plt.subplot(212)
#plt.plot(C[C>0],abs(y[C>0])/(0.5*len(y)))
#plt.xlim(0,2)
#plt.xlabel("Frequency [Hz]")
#plt.ylabel("Amplitude [mm/s]")
