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
from grab_sim_objects import *


### General parameters ###
visual="irr" # visualization
sim='0p'# sim number 
fixed=False # is it fixed
obj=[]
data_path="C:/Users/dmulr/OneDrive/Documents/data/"

##### control type #####
control_type="path_following"

'''
# Control type:
pot_field: make robot form a shape 
pot_field_grab: make the robot grab an object 
pot_field_drag: have it grab a ball and drag it 
"pot_field_dragA":  drag With analytical function 
"path_following"
"grab_drag"
"none"
'''

##### interior modes #####
mode='empty'
  
'''
# Interior generation modes:
'empty'     Nothing inside
'nonhomo'   diameter alternate
'max'       max possible number of interiors
'nmax'      customize the ring sizes
'nonhnmax'  max number of interiors with different diamters
'homo'      all the same diameterr
'''

##### granular mode #####
granmode="homo"

'''
homo: all the same size
nonhomo: they alternate in diameter
'''

##### time #####
tstep=.002   # time step
tend=3.4 # time end

##### Friction #####
mu_f=.1    # friction
mu_b=.01    # dampning
mu_r=.01     # rolling friction
mu_s=.01    # SPinning fiction

##### compliance #####
Ct=.0001
C=.000001
Cr=.0001
Cs=.0001

##### Robot #####       
mr=.2       # mass
nb=6# number of robots
height=.06  # height of cylinder
diameter=.076/2 # diameter of cylinder and robots
volume=np.pi*.25*height*(diameter)**2   # calculate volume
rowr=mr/volume # calculate density of robot
R=(2*diameter*nb/(np.pi*2))+.1
#R=.063/(np.sin(np.pi/nb))

### Geometry of robot ###
geom="cylinder"

'''
square: Robots will be in a square shape
cylinder: Robots will be a cylinder
shere: robots will be a sphere
'''

### Active bots ###
actbots=np.arange(0,nb,5)
active=np.zeros(nb)
for i  in range(len(active)):
    if any(actbots==i):
        active[i]=1

##### Interior particles #####
mp=.01 # mass 
diameter2=.076/2 # diameter of cylinder and robots
volume2=np.pi*.25*height*(diameter2)**2   # calculate volume
rowp=mp/volume2 # density

##### Spring ##### 
k=700  # spring constant (bots)
rl=0
rlmax=0.005
type_spring='var'

##### Floor #####
length=40 # Length of the body floor
tall=1     # height of the body floor


#####################################################################################
if control_type=="path_following":
    #### Path variables ####
    paths=Path(0,30,'flat_line')
    #paths=Path(ax,ay)    
    pathind=0
    vref=1
    mag=10
    mag_tan=8
    mag_n=20
    poseindt=actbots[0:len(actbots)]
    poseind=np.zeros(nb)
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1

    args=(paths,pathind,vref,poseind)
##########################################################################################################    
if control_type=="pot_field_grab": 
    shape="circle"
    alpha=3
    beta=.01
    balls=0
    p1=0
    p2=.95
    bl=-9
    br=9
    Rd=0.2
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=1
    Rb=.1
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=0
    zball=p2   
    shapes=Points_for_shape(shape,xball,zball,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball


######################################################################################
if control_type=="pot_field_grab_A": 
    tpull=100
    alpha=3
    beta=0
    balls=0
    p1=0
    p2=.95
    bl=-9
    br=9
    Rd=.1
    ##### Ball variables #####
    mb=1
    Rb=.2
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p1
    zball=p2 
    shapes=points_shape_A(p2,p1,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,tpull
    pathind=0
######################################################################################
if control_type=="grab_drag":
    tpull=1
    #(ax,ay)=Path.create_line()
    paths=Path(0,4,'parab')
    pathind=0
    vref=1
    poseindt=actbots[1:len(actbots)]
    poseind=np.zeros(nb)
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1
    
    shape="circle"
    alpha=10
    beta=.1
    balls=0
    p1=1.1
    p2=0
    bl=-12
    br=12
    Rd=.3
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=.1
    Rb=Rd
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p2
    zball=p1  
    shapes=Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,paths,pathind,vref,poseind,tpull

###########################################################################################################################
if control_type=="shape_form":
    shape="Square"
    alpha=300
    beta=.01
    balls=0
    p1=0
    p2=0
    bl=-9
    br=9
    Rd=R
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
        nr=np.asarray(nr)
    ##### Ball variables #####
    shapes=Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,Rd,nr    
#######################################################################################################################
if control_type=="grab_drag_A":
    tpull=1
    #(ax,ay)=Path.create_line()
    paths=Path(0,4,'parab')
    pathind=0
    vref=1
    poseindt=actbots[1:len(actbots)]
    poseind=np.zeros(nb)
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1
    
    shape="square"
    alpha=10
    beta=.1
    balls=0
    p1=1.3
    p2=0
    bl=-9
    br=9
    Rd=.3

    bl=-9
    br=9
    Rd=0.3
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=.1
    Rb=Rd
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p2
    zball=p1  
    shapes=Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,paths,pathind,vref,poseind,tpull

if control_type=="tunneling":
    #### Path variables ####
    paths=Path(0,10,'flat_line')
    #paths=Path(ax,ay)    
    pathind=0
    vref=1
    poseindt=actbots[0:len(actbots)]
    poseind=np.zeros(nb)
    #force 
    mag=10
    mag_tan=5
    mag_n=1
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1
    env_mode="tunnel"
    args=(paths,pathind,vref,poseind)    
###########################################################################################################    
##### Save variables #####
position=True
velocity=True
forces=True
control_force=True
spring=True
contact=True
if mode!='empty':
    part_position=True
    part_vel=True
    part_force=True
else:
    part_position=False
    part_vel=False
    part_force=False

if control_type=='path_following' or control_type=='shape_form' or control_type=="tunneling":
    ball_data=False
else:
    ball_data=True    

if control_type=="grab_drag" or control_type=='path_following' or control_type=="tunneling":
    path_data=True
else:
    path_data=False
    
save_data=[position,velocity,forces,control_force,spring,contact,part_position,part_vel,part_force,ball_data,path_data]