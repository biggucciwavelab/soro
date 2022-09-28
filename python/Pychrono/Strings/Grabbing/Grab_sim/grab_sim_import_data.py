# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 08:21:05 2020

@author: dmulr
"""

import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
from config import *
from grab_sim_plot_objects import *
import csv
#path='C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/PyChrono/Phase Diagrams/phase_diagrams2/robot_data'+sim+'/'
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
data5 = open(filename, 'r')
table=[]
for row in csv.reader(data5):
    table.append(row)


sim=str(table[0][1])
nb=int(table[1][1])
diameter=float(table[3][1])

actbots=np.asarray(table[7]).astype(np.float) 
active=np.asarray(table[8]).astype(np.float) 
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
##################################################################
### 5
#name='x contact points.csv'
#filename=path+name
#data5 = np.genfromtxt(filename,delimiter=',')
#
### 6
#name='y contact points.csv'
#filename=path+name
#data6 = np.genfromtxt(filename,delimiter=',')
##
#
### 7
#name='z contact points.csv'
#filename=path+name
#data7 = np.genfromtxt(filename,delimiter=',')
#
##
### 8
#name='x contact force.csv'
#filename=path+name
#data8 = np.genfromtxt(filename,delimiter=',')
#
##
### 9
#name='y contact force.csv'
#filename=path+name
#data9 = np.genfromtxt(filename,delimiter=',')
##
## 10
#name='z contact force.csv'
#filename=path+name
#data10 = np.genfromtxt(filename,delimiter=',')
#
##
## 11
#name='nc.csv'
#filename=path+name
#data11 = np.genfromtxt(filename,delimiter=',')
#
########################################################################
##
##########################################################################
## 12
#name="particle_position.csv"
#filename=path+name
#data12 = np.genfromtxt(filename,delimiter=',')
#
### 13
#name="particle_velocity.csv"
#filename=path+name
#data13 = np.genfromtxt(filename,delimiter=',')