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
from phase_sim_plot_objects import *
path='C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/PyChrono/Phase Diagrams/phase_diagrams2/robot_data'+sim+'/'
name='position.csv'
filename=path+name
data1 = np.genfromtxt(filename,delimiter=',')

name='TotalForces.csv'
filename=path+name
data2 = np.genfromtxt(filename,delimiter=',')

name='Force_controller.csv'
filename=path+name
data3 = np.genfromtxt(filename,delimiter=',')


name='Spring_properties.csv'
filename=path+name
data4 = np.genfromtxt(filename,delimiter=',')

# contact points
name='x contact points.csv'
filename=path+name
data5 = np.genfromtxt(filename,delimiter=',')

name='y contact points.csv'
filename=path+name
data6 = np.genfromtxt(filename,delimiter=',')

name='z contact points.csv'
filename=path+name
data7 = np.genfromtxt(filename,delimiter=',')

# contact forces
name='x contact force.csv'
filename=path+name
data8 = np.genfromtxt(filename,delimiter=',')

name='y contact force.csv'
filename=path+name
data9 = np.genfromtxt(filename,delimiter=',')

name='z contact force.csv'
filename=path+name
data10 = np.genfromtxt(filename,delimiter=',')

name='nc.csv'
filename=path+name
data11 = np.genfromtxt(filename,delimiter=',')

results_dir = os.path.join('plots'+sim)     

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)
results=plots(data1,data2,data3,data4,results_dir,nb)

results2=plot_force_chain(data1,data5,data6,data7,data8,data9,data10,data11,results_dir,nb)
results2.Forcechains()
#results.plot_xyz()
#results.Plot_fxyz()
#results.Plot_fxyzcontroller()
#results.Plot_spring_force_length()