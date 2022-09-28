# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 19:56:00 2020

@author: dmulr
"""
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit


visual="irrlecht"
type_spring="var"
control_type="pot_field"

fixed=False# is it fixed
mode='nmax'  # will there be interiors

# time 
tstep=.005   # time step
tend=100     # time end

# friction
mu_f=.3    # friction
mu_b=.01    # dampning
mu_r=.1     # rolling friction
mu_s=.01    # SPinning fiction

# compliance
Ct=.0001
C=.000000001
Cr=.0001
Cs=.0001

mr=.120       # mass
mp=.06
nb=75     # number of robots
height=.06  # height of cylinder
diameter=.035 # diameter of cylinder and robots
volume=np.pi*.25*height*(diameter)**2   # calculate volume


diameter2=.035 # diameter of cylinder and robots
volume2=np.pi*.25*height*(diameter2)**2   # calculate volume


rowr=mr/volume # calculate density of robot
rowp=mp/volume2

mag=2

rl=0
rlmax=0.01
length=8  # Length of the body floor
tall=1     # height of the body floor
obj=[]
data_path="C:/Users/dmulr/OneDrive/Documents/data/"

   

#shape="circle"
#shape="Square"
#shape="grab"
shape="grab"

R=(diameter*nb/(np.pi*2))+.1 
Rd=4*R

bl=-1.5
br=1.5
p1=0
p2=0

# Ring values based on shape
if shape=="circle":
    nr=[0,1,2,3,4] # circle
    
if shape=="Square":
    nr=[.3,.4,.5,.6] # square
    
if shape=="grab":
    nr=[]

position=True
forces=True
control_force=True
spring=True
contact=True
error=True

save_data=[position,forces,control_force,spring,contact,error]