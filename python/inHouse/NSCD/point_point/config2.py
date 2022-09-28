# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 15:31:26 2020

@author: dmulr
"""
# In[Import libraries]
import numpy as np 

sim='1'
# In[Initializing objects]
obj=[]

# In[Creating the bounding box with left, right, up, and down coordinates]
left,right,up,down=-1,1,1,-1

# In[Robot]
nb=10 # boundary robots
diameter=.07 # diameter cm
R1=(diameter*nb/(np.pi*2))+.1 # Radius of robot cm
mb=.12 # mass of bot grams
Ib=.5*mb*(diameter/2)**2 # Inertia of bot

phi0b=0 # initial angle
xv0b=0   # x velocity
yv0b=0   # y velocity
phiv0b=0     # rotational velocity

# In[Internal Particles]
mp=1 # mass of particle
Ip=.5*mb*(diameter/2)**2 # Inertia of bot
phi0p=0 # initial angle
xv0p=0   # x velocity
yv0p=0   # y velocity
phiv0p=0     # rotational velocity

# In[Forces]
g=0   # Gravity is on 
Fx=0
Fy=0
T=0
# In[Spring Constants]
k=1
l=0

# In[Time steps and integration properties]
#Time parameters
t0=0 # start simulation (seconds)
tend=1# end simulation (seconds)
tstep=.001 # time step (seconds)
inter=int(tend/tstep)
time=np.linspace(t0,tend,num=inter) # time matrix
Q=np.zeros((2*nb,time.size)) # empty Q matrix
EL2=np.zeros((1,time.size)) # L2 norm of error
COMA=np.zeros((2,time.size)) # empty COM matrix of actual COM 
COM=np.zeros((2,time.size)) # center of mass 
ELAVG=np.zeros((1,time.size)) # average error
E=np.zeros((1,time.size))
#Integration parameters
t_imp=1
gamma=.5
path='up_right'
# In[Impact and friction properties]
en=.3
et=0
mu=.8
Er = 0.1 # error allowed for each bot
error=nb*(Er**2) # L2 error
