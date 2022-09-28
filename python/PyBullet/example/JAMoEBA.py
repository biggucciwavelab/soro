# -*- coding: utf-8 -*-
"""
Created on Thu Dec 10 22:17:56 2020

@author: elope

A playground used to investgate teh affects of different simulation
paremeters on the simulation itself.
"""

import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
from example.get_user import *

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath(pybullet_data_dict[user])

# Create floor
# plane = p.createCollisionShape(p.GEOM_PLANE)
# p.createMultiBody(0,plane)
planeId = p.loadURDF("plane100.urdf", [0,0,0])

# System Parameters
num_bots=30

# Parameters for interior particles
radius=.025
height=.06
mass=0.01
visualShapeId = -1
orientation=[0,0,0,1]
n=np.arange(num_bots,5,-7)
num_interior=np.sum(n)
particle = p.createCollisionShape(p.GEOM_CYLINDER,
                                  radius=radius,
                                  height=height)
particle_visual = p.createVisualShape(p.GEOM_CYLINDER,
                                      radius=radius,
                                      length=height,
                                      rgbaColor=[1,.412,.706,1])

positions=[]
for i in range(n.size):
    for j in range(n[i]):
        R2=(radius*2)*n[i]/np.pi
        x=R2*np.cos(j*2*np.pi/n[i])
        z=.5*height
        y=R2*np.sin(j*2*np.pi/n[i])
        pos=[x,y,z]
        positions.append(pos)
        
        

masses=[mass]
linlCollisionShapeIndices = [particle]
linkVisualShapeIndices = [-1]
linkPositions = positions
linkOrientations = [[0,0,0,1]]
linkInertialFramePositions = [[0,0,0]]
linkInertialFrameOrientations = [[0,0,0,1]]
indices = [0]

bodyUids = p.createMultiBody(baseMass = mass,
                             baseInertialFramePosition = [0,0,0],
                             baseCollisionShapeIndex = particle,
                             baseVisualShapeIndex = particle_visual,
                             basePosition = [0,0,0],
                             batchPositions = positions) 

#TODO: Iterate through all particles to add a particular color of our choosing
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

while p.isConnected():
    # p.stepSimulation()
    p.setGravity(0,0,-10)