# In[Header]

"""
author: declan mulroy
project: JAMoEBA
email: dmulroy@hawk.iit.edu
date: 10/9/19
"""

# In[import libraries]
import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
from plotters2 import Ballpos2,PackingFraction,tension,Forcechains,NORM_TAN,Total_Force_membrane

# In[Import Data]

#In[Import data]
data=np.load('Experiment 5.npz',allow_pickle=True)

#data2=np.load('compare.npz',allow_pickle=True)

# Positions
qx=data['qx']
qy=data['qy']
qz=data['qz']

# rotations
rot0=data['rot0']
rot1=data['rot1']
rot2=data['rot2']
rot3=data['rot3']

# Spring values
SL=data['SL']

Fmem=data['Fmem']
# Velocity
Xv=data['Xv']
Yv=data['Yv']
Zv=data['Zv']

# Total foces
Fxt=data['Fxt']
Fyt=data['Fyt']
Fzt=data['Fzt']

# Active robots
botcall=data['botcall']

# number of robots
nb=data['nb']
ni=data['ni']
# total number
nt=nb+ni
# diameter
diameter=data['diameter']
# height
height=data['height']
# time
time=data['ttemp']

sim=data['sim']
# for time
count=data['count']
# save file as 
file=".pdf"
# script directory
script_dir = os.path.dirname("plots"+str(sim)+"/")
# ball position
ballp=data['ballp']



# Contact points
xc=data['xc']
yc=data['yc']
zc=data['zc']
nc=data['nc']

# contact forces
Fcx=data['Fcx']
Fcy=data['Fcy']
Fcz=data['Fcz']

# normal direction
VNX=data['VNX']

VNZ=data['VNZ']

VTX=data['VTX']

VTZ=data['VTZ']

# number of frames to remove 
frames=10
# rate video plays at
rate=10

# In[Functions to be plotted]

#Ballpos2(script_dir,time,ballp,file)

#Forcechains(script_dir,nc,xc,yc,zc,Fcx,Fcy,Fcz,time)

#tension(nb,Fmem,time,script_dir,qx,qy,qz)

#Localpackingfraction(script_dir,qx,qy,qz,time)

Total_Force_membrane(nb,time,script_dir,qx,qy,qz,Fxt,Fyt,Fzt)

#NORM_TAN(VNX,VNZ,VTX,VTZ,qx,qy,qz,script_dir,time,nb)

# In[Other functions]






















#position(script_dir,botcall,time,qx,qy,qz,nb,file)

#Rotation(script_dir,time,rot0,rot1,rot2,rot3,nb,botcall,file)

#Velocity(script_dir,botcall,Xv,Yv,Zv,nb,time,file)

#PathTraveled(script_dir,Xv,Yv,Zv,qx,qy,qz,file,nb)

#Springlength(script_dir,time,SL,nb,file)    
            
#SpringForce(script_dir,time,Fmem,nb,file)

#PackingFraction(script_dir,time,nb,ni,diameter,height,qx,qz,file,count)

#TotalForces(script_dir,nb,Fxt,Fyt,Fzt,time,botcall,file)

#Ballpos2(script_dir,time,ballp,file)

#(X,T,Y,columns)=AnimatedForces(nb,Fmem,time,script_dir,frames,rate)

#frameTension(Fmem,time,nb,file,script_dir)

#ContactForce(nb,nt,Fxc,Fyc,Fzc,time,script_dir,frames,rate)

#contactforce_position(nb,nt,qx,qy,qz,Fxc,Fyc,Fzc,time,frames,rate,script_dir)

#Forcechains(script_dir,nc,xc,yc,zc,Fcx,Fcy,Fcz,time)

#tension(nb,Fmem,time,script_dir,qx,qy,qz)

#Localpackingfraction(script_dir,qx,qy,qz,time)