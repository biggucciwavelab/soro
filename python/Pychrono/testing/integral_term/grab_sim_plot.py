# -*- coding: utf-8 -*-
"""
Created on Tue May 12 12:50:52 2020

@author: dmulr
"""
import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
from config import *
from grab_sim_plot_objects import *
#path='C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/PyChrono/Phase Diagrams/phase_diagrams2/robot_data'+sim+'/'
#sim="5V"
path='C:/Users/dmulr/OneDrive/Documents/soro_chrono/python/Pychrono/Strings/Grabbing/Grab_sim/robot_data'+sim+'/'


# 0
name='bot_position.csv'
filename=path+name
data0 = np.genfromtxt(filename,delimiter=',')

# 1
name='bot_velocity.csv'
filename=path+name
data1 = np.genfromtxt(filename,delimiter=',')

# 2
name='bot_TotalForces.csv'
filename=path+name
data2 = np.genfromtxt(filename,delimiter=',')

## 3
name='Force_controller.csv'
filename=path+name
data3 = np.genfromtxt(filename,delimiter=',')

## 4
name='Spring_properties.csv'
filename=path+name
data4 = np.genfromtxt(filename,delimiter=',')

## 5 
name= 'variables.csv'
filename=path+name
data100 = open(filename, 'r')
table=[]
for row in csv.reader(data100):
    table.append(row)


sim=str(table[0][1])
control_type=str(table[1][1])

nb=int(table[2][1])
diameter=float(table[4][1])

actbots=np.asarray(table[8]).astype(np.float) 
active=np.asarray(table[9]).astype(np.float) 


if control_type=="shape_form":
    shape=str(table[19][1])
    Rd=float(table[20][1])
    nr=np.asarray(table[21]).astype(np.float)
'''
number of bots
geometry of bot
radius of bot
starting radius
robot mass
number of active bots
active bots
ring numbers
number of interior
particle mass
alpha
beta
k
rl
rlmax
tend
'''

if control_type=='grab_drag' or  control_type=="pot_field_grab":
    name='ballFx.csv'
    filename=path+name
    data5 = np.genfromtxt(filename,delimiter=',')
   
    name='ballFz.csv'
    filename=path+name
    data6 = np.genfromtxt(filename,delimiter=',')

    name='ballx.csv'
    filename=path+name
    data7 = np.genfromtxt(filename,delimiter=',')

    name='ballz.csv'
    filename=path+name
    data8 = np.genfromtxt(filename,delimiter=',')


    name='ballvx.csv'
    filename=path+name
    data11 = np.genfromtxt(filename,delimiter=',')

    name='ballvz.csv'
    filename=path+name
    data12 = np.genfromtxt(filename,delimiter=',')


    name='FBX.csv'
    filename=path+name
    data13 = np.genfromtxt(filename,delimiter=',')

    name='FBZ.csv'
    filename=path+name
    data14 = np.genfromtxt(filename,delimiter=',')
    

    
else:
    data5=None
    data6=None
    data7=None
    data8=None
    data11=None
    data12=None
    data13=None
    data14=None



    
if control_type=="path_following" or control_type=="grab_drag" or control_type=="grab_drag_A" or control_type=="tunneling": 

    name='pathx.csv'
    filename=path+name
    data9 = np.genfromtxt(filename,delimiter=',')


    name='pathz.csv'
    filename=path+name
    data10 = np.genfromtxt(filename,delimiter=',')
    
else:
    data9=None
    data10=None
    

if control_type=="grab_drag":
    name='Pf_controller.csv'
    filename=path+name
    data15 = np.genfromtxt(filename,delimiter=',')

    name='PP_controller.csv'
    filename=path+name
    data16 = np.genfromtxt(filename,delimiter=',')
else:
    data15=None
    data16=None
    
results_dir = os.path.join('plots'+sim) 



if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

    
results=robot_plots(nb,results_dir,sim,active,actbots,data0=data0,data1=data1,data2=data2,data3=data3,data4=data4,data5=data5,data6=data6,data7=data7,data8=data8,data9=data9,data10=data10,data11=data11,data12=data12,data13=data13,data14=data14,data15=data15,data16=data16)   


if control_type=="path_following": 
    results.plot_path()
    results.plot_mag_velocity()
    results.plot_total_forces()
    results.plot_all_spring_length_force()
    results.Plot_conf()
    results.plot_mag_controls()
    results.Plot_fxyzcontroller()
    results.forces_per_bot()

if control_type=="grab_drag":
    results.plot_mag_pf_pp_controls()
    results.plot_abs_ball_force()
    results.plot_mag_velocity()
    results.plot_total_forces()
    results.plot_all_spring_length_force()
    results.plot_ball_force()
    results.Plot_conf()
    results.plot_mag_controls()
    results.Plot_fxyzcontroller()
    results.forces_per_bot()
    results.plot_path()

if control_type=="tunneling":
    results.plot_mag_velocity()
    results.plot_total_forces()
    results.plot_all_spring_length_force() 
    results.plot_mag_controls()
    results.plot_path()
    results.Plot_conf()