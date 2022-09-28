# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 15:31:26 2020

@author: dmulr
"""
# In[Import libraries]
import numpy as np 

sim='11'
# In[Initializing objects]
obj=[]

# In[Creating the bounding box with left, right, up, and down coordinates]
left,right,up,down=-1,1,1,-1

# In[Robot]
nb=12 # boundary robots
diameter=.07 # diameter cm
R1=(diameter*nb/(np.pi*2))+.1 # Radius of robot cm
#R1=.5
mb=.12 # mass of bot grams
Ib=.5*mb*(diameter/2)**2 # Inertia of bot
phi0b=0 # initial angle
xv0b=0   # x velocity
yv0b=0   # y velocity
phiv0b=0     # rotational velocity
Fx=0
Fy=0
T=0
# In[Internal Particles]
mp=.12 # mass of particle
Ip=.5*mb*(diameter/2)**2 # Inertia of bot
phi0p=0 # initial angle
xv0p=0   # x velocity
yv0p=0   # y velocity
phiv0p=0     # rotational velocity

# In[gravity]
g=0   
# In[Spring Constants]
k=1
l=0.08
# In[Time steps and integration properties]
#Time parameters
t0=0 # start simulation (seconds)
tend=10# end simulation (seconds)
tstep=.01 # time step (seconds)
tbar=1
gamma=.5
inter=int(tend/tstep)
h=tstep
# In[Empty matrices]
time=np.linspace(t0,tend,num=inter) # time matrix
Q=np.zeros((2*nb,time.size)) # empty Q matrix
Et=np.zeros((2*nb,time.size)) # L2 norm of error
E=np.zeros((1,time.size)) # Error

# In[Point to point parameters]
path='up_right'
shape='circle'
Er = 1 # error allowed for each bot
error=20 # L2 error
# In[Shape function parameters]
alpha=600
beta=1
bl=-1
br=1
p1=0
p2=0
R=R1

nr=[0,1,2]
# In[Impact and friction properties]
en=.3
et=0
mu=.8


