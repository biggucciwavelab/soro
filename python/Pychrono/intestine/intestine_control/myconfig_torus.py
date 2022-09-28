# -*- coding: utf-8 -*-
"""
Created on Tue Jan  7 16:30:22 2020

@author: dmulr
"""
import numpy as np

sim = "Experiment 6"

tstep = 0.002  # Time Step
tend = 20  # Length of simulation

# In[Define frictional properties]
mu_f = .2  # friction
mu_b = .15  # dampning
mu_r = .01  # rolling friction
mu_s = .01  # SPinning fiction
# cohesion
Ct = .00001 # Cohesion tangent
C_ = .00001 # cohesion normal 
Cr = .0001 # Cohesion rolling
Cs = .0001 # cohesion spinning

# In[Create Floor]

length = 8  # Length of the body floor
tall = .1  # height of the body floor

# In[cylinder dimmensions/geometries]

###### We want to plug in coord length, active bots to find radius 
nbactive = 12  # number of robots
nb=2*nbactive # fills in for the non active bots
diameter = .06  # diameter of cylinder and robots
height = 0.06  # height of cylinder
hhalf = height / 2  # half height of cylinder
mr = .15  # mass of robot
mw = .04 # mass of wall 
l = 0.1524 # coord length
R1=l/(2*np.sin(np.pi/nbactive))

# empty botcall matrix
botcall=np.zeros((1,nb))

# fill matrix
for i in range (nbactive):
    botcall[:,2*i]=1


obj = []  # empty matrix of bots and particles
Springs = [] # empty springs matrix
walls = [] # empty wall matrix 
bots=[] # empty 


k = -.25 # spring force
rl =0 #(l-2*diameter)/2 # length between bots
rlmax = 1.25 * rl # max spring length
volume = np.pi * .25 * height * (diameter) ** 2  # calculate volume

rowr = mr / volume  # calculate density of robot
roww = mw / volume
# In[External forces]

mag = .75  # magnitude of force applied to each bot
active = np.ones(nb)  # determines if its active or not
C = np.ones(nb)  # constant
obj = []  # empty matrix of bots and particles
force = []  # empty array to store force objects
res=np.zeros(nbactive)
goal = np.array([.5, 0, .1])  # goal or the target

ta = 0  # angle threshold

error = 0.00  # error for distance between goal and robot
goal_c = np.array([.5, 0, .1])
goal_d = np.array([.5, 0, .1])
Fm=[]