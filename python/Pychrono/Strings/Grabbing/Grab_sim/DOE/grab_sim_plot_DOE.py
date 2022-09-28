# -*- coding: utf-8 -*-
"""
Created on Tue May 12 12:50:52 2020

@author: dmulr
"""
import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
from config_DOE import *
import csv
import timeit
from grab_sim_plot_objects_DOE import *
#path='C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/PyChrono/Phase Diagrams/phase_diagrams2/robot_data'+sim+'/'
#sim="5V"
#data_set=[5,6,7,8,9,10]
for i in range(len(data_set)):
    path="F:/Science_robotics_journal/tunneling/DATA/robot_dataT"+str(data_set[i])+"/"
    #path='C:/Users/dmulr/OneDrive/Documents/soro_chrono/python/Pychrono/Strings/Grabbing/Grab_sim/DOE/robot_dataT'+str(data_set[i])+"/"
   # path='C:/Users/dmulr/OneDrive/Documents/soro_chrono/python/Pychrono/Strings/Grabbing/Grab_sim/robot_data'+data_set[i]+'/'

    start=timeit.default_timer()
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
#name='Spring_properties.csv'
#filename=path+name
#data4 = np.genfromtxt(filename,delimiter=',')

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
        #nr=np.asarray(table[21]).astype(np.float)


    if control_type=='grab_drag' or control_type=="pot_field_grab" or control_type=='grab_drag_A' or control_type=='grab_drag_2':
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



    
    if control_type=="path_following" or control_type=="grab_drag" or control_type=="grab_drag_A" or control_type=="tunneling" or control_type=="grab_drag_2" or control_type=="shape_formation": 

        name='pathx.csv'
        filename=path+name
        data9 = np.genfromtxt(filename,delimiter=',')


        name='pathz.csv'
        filename=path+name
        data10 = np.genfromtxt(filename,delimiter=',')
    
    else:
        data9=None
        data10=None
    

    if control_type=="grab_drag" or control_type=="grab_drag_A":
        name='Pf_controller.csv'
        filename=path+name
        data15 = np.genfromtxt(filename,delimiter=',')

        name='PP_controller.csv'
        filename=path+name
        data16 = np.genfromtxt(filename,delimiter=',')
    else:
        data15=None
        data16=None

    # desired shape
    if control_type=="shape_form":
        name="desired_shapex.csv"
        filename=path+name
        data17 = np.genfromtxt(filename,delimiter=',')
        name="desired_shapey.csv"
        filename=path+name
        data18 = np.genfromtxt(filename,delimiter=',')   
 
    else:
        data17=None
        data18=None

# error
    if control_type=="shape_form" or control_type=="shape_formation":
        name="error.csv"
        filename=path+name
        data19 = np.genfromtxt(filename,delimiter=',')
    else:
        data19=None
    results_dir = os.path.join('plots'+sim) 


## Contact force x
#name="x contact force.csv"
#filename=path+name
#data20 = np.genfromtxt(filename,delimiter=',')
#
## Contact force y
#name="y contact force.csv"
#filename=path+name
#data21 = np.genfromtxt(filename,delimiter=',')
#
## Contact force z
#name="z contact force.csv"
#filename=path+name
#data22 = np.genfromtxt(filename,delimiter=',')
#
## contact points x
#name="x contact points.csv"
#filename=path+name
#data23 = np.genfromtxt(filename,delimiter=',')
#
## contact points y
#name="y contact points.csv"
#filename=path+name
#data24 = np.genfromtxt(filename,delimiter=',')
#
## contact points z
#name="z contact points.csv"
#filename=path+name
#data25 = np.genfromtxt(filename,delimiter=',')


# contact points 
#name="nc.csv"
#filename=path+name
#data26 = np.genfromtxt(filename,delimiter=',')

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)

    end=timeit.default_timer()
    print('Time importing data: ',(end-start)/60,'mins')
   
    results=robot_plots(nb,results_dir,sim,active,actbots,data0=data0,data1=data1,data2=data2,data3=data3,data4=None,data5=data5,data6=data6,data7=data7,data8=data8,data9=data9,data10=data10,data11=data11,data12=data12,data13=data13,data14=data14,data15=data15,data16=data16,data17=data17,data18=data18,data19=data19)   


    start=timeit.default_timer()

# if its drag and drag 
    if control_type=="grab_drag" or control_type=="grab_drag_A" :
    
        results.plot_mag_pf_pp_controls()
        results.plot_z_pf_pp_controls()
        results.plot_x_pf_pp_controls()
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
# path following 
    if control_type=="path_following": 
        results.plot_path()
        results.plot_mag_velocity()
        results.plot_total_forces()
        results.plot_all_spring_length_force()
        results.Plot_conf()
        results.plot_mag_controls()
        results.Plot_fxyzcontroller()
        results.forces_per_bot()

# grab and drag 2 
    if control_type=="grab_drag_2":
        results.plot_abs_ball_force()
        results.plot_ball_velocity()
        results.plot_ball_position_time()
        results.plot_ball_force()
        results.plot_mag_velocity()
        results.plot_total_forces()
        #results.plot_all_spring_length_force()
        results.Plot_conf()
        results.plot_mag_controls()
        results.Plot_fxyzcontroller()
        results.forces_per_bot()
        results.plot_path()

    if control_type=="tunneling":
        results.plot_mag_velocity()
        results.plot_total_forces()
        #results.plot_all_spring_length_force() 
        results.plot_mag_controls()
        results.plot_path()
        results.Plot_conf()
        results.forces_per_bot()
    
    if control_type=="shape_formation":
        results.Forcechains()
#    results.plot_error()
#    results.plot_path()
#    results.plot_mag_velocity()
#    results.plot_total_forces()
##    results.plot_all_spring_length_force() 
#    results.plot_mag_controls()   
#    results.Plot_conf()
#    results.forces_per_bot()    
    
    if control_type=="shape_form":
        results.Plot_shape()
        results.plot_mag_velocity()
        results.plot_total_forces()
#    results.plot_all_spring_length_force() 
        results.plot_mag_controls()   
        results.Plot_conf()
        results.forces_per_bot()
    
    end=timeit.default_timer()
    print('Time plotting: ',(end-start)/60,'mins')