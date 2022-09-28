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



visual="irrlecht" # visualization
sim='10' # sim number 
fixed=False # is it fixed
obj=[]
data_path="C:/Users/dmulr/OneDrive/Documents/data/"

##### control type #####

control_type="pot_field_grab"

'''
# Control type:
nothing: no control 
force_right: make the robots go right 
pot_field: make robot form a shape 
pot_field_grab: make the robot grab an object 
pot_field_drag: have it grab a ball and drag it 
'''

##### interior modes #####
mode='nmax'
  
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
tend=100     # time end


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

mr=.120       # mass
nb=10   # number of robots
height=.06  # height of cylinder
diameter=.035 # diameter of cylinder and robots
volume=np.pi*.25*height*(diameter)**2   # calculate volume
rowr=mr/volume # calculate density of robot
#R=0.51
R=(diameter*nb/(np.pi*2))+.1 


##### Interior particles #####


mp=.06 # mass 
diameter2=.035 # diameter of cylinder and robots
volume2=np.pi*.25*height*(diameter2)**2   # calculate volume
rowp=mp/volume2 # density


###### controller #######

shape="circle"

'''
Square: form a square
grab: grab shape 
circle: form a circle shape 
'''

alpha=500 # alpha gain
beta=1 # beta gain 
mag=5 # constant force

Fballx=0
Fballz=0
# if cirlce or square this is the radius  
Rd=0.1 # circumscribing radius
bl=-2
br=2
p1=.3  # Center point x
p2=0    # Center point z


# Ring values based on shape
if shape=="circle":
    nr=[0,1,2,3,4] # circle
    
if shape=="Square":
    nr=[0,1,2,3] # square
    
if shape=="grab":
    nr=[]



##### Spring ##### 

k=300  # spring constant (bots)
rl=.09
rlmax=rl+.01
type_spring='var'



##### Floor #####

length=8  # Length of the body floor
tall=1     # height of the body floor



##### Ball variables #####
mb=1
Rb=Rd+.1
volume3=np.pi*.25*height*(Rb)**2   # calculate volume
rowb=mb/volume3 # density
xball=0
zball=p1

zbstop=.18
tball=1
xbstop=0


##### Save variables #####
position=True
velocity=True
forces=True
control_force=True
error=True
spring=True
contact=True
part_position=True
part_vel=True
save_data=[position,velocity,forces,control_force,error,spring,contact,part_position,part_vel]