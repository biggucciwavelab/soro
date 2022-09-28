# -*- coding: utf-8 -*-
"""
Created on Tue Jan  7 16:30:22 2020

@author: dmulr
"""
import numpy as np



sim="Experiment 6"

record=2

tstep=0.002 #Time Step
tset= 1 #Settling time
tend=20 #Length of simulation
tj=6    # jamm time
tp=6    # pull time 
# In[Define frictional properties]
mu_f=.2     # friction
mu_b=.15   # dampning
mu_r=.1     # rolling friction
mu_s=.01     # SPinning fiction

Ct=.00001
C=.00001
Cr=.0001
Cs=.0001


# In[Define frictional properties for floor material]
mu_f2=.1     # friction
mu_b2=0.1    # dampning
mu_r2=0     # rolling friction
mu_s2=0     # SPinning fiction

# compliance terms 
Ct=.00001
C=.00001
Cr=.0001
Cs=.0001


# In[Create Floor]

length=8    # Length of the body floor
tall=.1     # height of the body floor

# In[cylinder dimmensions/geometries]

nb=10           # number of robots
diameter=.075        # diameter of cylinder and robots
R1=(diameter*nb/(np.pi*2))+.1 
#R1=(diameter*nb/(np.pi*2))+.1 
diameter2=.035      # diameter of cylinder and robots   

#n=np.array([35,32,27,25,15,12,5])-5
#n=np.array([0])
n=np.arange(nb-3,5,-10)
ni=np.sum(n)        # sum them for total number of interior
nt=nb+ni            # total number of bots and particles
nr=nb/ni            # ratio of nb over ni
mr=.15              # mass
mp=.03              # mass of particles

height=0.06        # height of cylinder
hhalf=height/2      # half height of cylinder

k=-10            # spring constant (bots)
spring_b=0         # damping constant
kj=-1      # Jamming Spring constant
rl=.05               # resting length

rlj=0               # desired length jammed
rlmax=.1
rljmax=.1
obj=[]              # empty matrix of bots and particles
Springs=[]          # empty matrix of springs

volume=np.pi*.25*height*(diameter)**2           # calculate volume
volume2=np.pi*.25*height*(diameter2)**2         # calculate volume
rowr=mr/volume # calculate density of robot
rowp=mp/volume2 # calculate density of particles

# In[External forces]

mag =3  #[N]- magnitude of external force applied at each bot

force=[] #empty array to store force objects


# Empty matrix      
botcall=np.zeros((1,nb))

jamcall=np.zeros((1,nb))

normcall=np.zeros((1,nb))


# ID number of robots to be active
#bactive1=np.arange(0,int(nb/2),1)
#bactive2=np.arange(int(nb/2),nb,1)
bactive1=np.array([7,10,13])
bactive=np.hstack((bactive1))

# robots to jam tangently 
jactive1=np.arange(0,int(nb/2),1)
jactive2=np.arange(int(nb/2),nb,1)
jactive=np.hstack((jactive1,jactive2))

# robots to jam normally
nactive1=np.arange(0,int(nb/2),1)
nactive2=np.arange(int(nb/2),nb,1)
nactive=np.hstack((nactive1,nactive2))
# Active robots 
for i in (jactive):
    jamcall[:,i]=1

## For robots that are active fill botcall==1
#for i in (bactive):
#    botcall[:,i]=1

for i in (nactive):
    normcall[:,i]=1
    


