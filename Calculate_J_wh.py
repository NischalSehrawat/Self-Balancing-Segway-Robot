from  numpy import array, zeros, argmax
import matplotlib.pyplot as plt
from datetime import datetime

file_name = 'data2.txt'

with open(file_name, 'r') as file:  
            
    ff = file.readlines()
            
dd = []; my_data = zeros((len(ff),2))
        
for i, line in enumerate(ff):
            
    jj = line.split(',')
            
    my_data[i,1] = float(jj[0]); # Angular velocity in RPM
            
    t = jj[1][:-1]# Time stamp in string format
            
    Time = datetime.strptime(t, "%d-%m-%Y-%H:%M:%S.%f") # Convert it to datetime object
            
    if i == 0:
                
        T_init = Time
                
        my_data[i,0] = 0
                
    else:
                
        my_data[i,0] = (Time - T_init).total_seconds()
        
#-================== CALCULATE MOMENT OF INERTIA===================

acc_ind = argmax(my_data[:, 1]) # GEt the index of maximum value of RPM

acc = (my_data[acc_ind, 1]*2*3.14/60) / my_data[acc_ind, 0] # Acc [rad/s2]

dcc = (0-my_data[acc_ind, 1]*2*3.14/60) / (my_data[-1, 0] - my_data[acc_ind, 0]) # Decc [rad/s2]

m_can = 0.37; # Mass of can used [kg]

r = 0.5 * 0.085 # Wheel radius [m]

J = m_can * r *(9.8-acc*r) / (acc - dcc) # Moment of inertia of the wheel + rotor

#==============================PLOT RESULTS===================
         
plt.plot(my_data[:,0],(2*3.14/60)*my_data[:,1] ,'--r', linewidth = 2, label = file_name)
       
plt.xticks(fontsize = 18); plt.yticks(fontsize = 18)
plt.xlabel('T [s]', fontsize = 18);
plt.ylabel(r'$\Omega$ [rad/s]', fontsize = 18)
plt.legend(loc = 'upper right', fontsize = 18)
plt.text(2.5,4, r'Acc = '+format(acc, "0.2f")+ ' $ [rad/s^2]$'+" \nDecc = "+format(dcc,"0.2f")+' $ [rad/s^2]$', fontsize = 18)
plt.text(2.5,2, r'J = '+format(J, "0.2f")+' $ [kg.m^2]$', fontsize = 18)
plt.show()

