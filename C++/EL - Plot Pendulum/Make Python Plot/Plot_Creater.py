# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 10:31:19 2019

@author: Esteban Lopez
"""

import numpy as np
from matplotlib import pyplot as plt  

#Change the data file to your data file's location
data_file=r"C:\Users\17088\Documents\Soft Robotics Research\Wave Lab Documents\C++\EL - Plot Pendulum\build\Release\Pendulum_Position_Data.dat"
        

data=np.loadtxt(data_file)  
time=data[:,0]
x_pos=data[:,1]
y_pos=data[:,2]
z_pos=data[:,3]

plt.title('Position v Time')
plt.xlabel('Time[sec]')
plt.ylabel('Position')
plt.plot(time,x_pos,linewidth='1.0',color='red', label='X-Position') 
plt.plot(time,y_pos,linewidth='1.0',color='black',label='Y-Position')
plt.legend()
plt.show()