# -*- coding: utf-8 -*-
"""
Created on Fri Dec 11 12:35:26 2020

@author: elopez8

Trying to attach a cloth to two tall cubes
"""

import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
from QuaternionRotation import create_from_axis_angle

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Desktop
# p.setAdditionalSearchPath('C:/Users/elope/anaconda3/Lib/site-packages/bullet3-master/data/')

# Laptop
p.setAdditionalSearchPath("C:/Users/17088/Anaconda3/Lib/site-packages/bullet3-master/data/")

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

# Floor
planeId = p.loadURDF("plane100.urdf", [0,0,0])

# Make Cubes
cube_dim=[.5,.5,1] # Halfdimensions of the cube in (x,y,z))
mass=.5
pos1=[1.5,0,1]
pos2=[-1.5,0,1]
positions=[pos1,pos2]
cube = p.createCollisionShape(p.GEOM_BOX,
                                halfExtents=cube_dim)
cube_visual = p.createVisualShape(p.GEOM_BOX,
                                halfExtents=cube_dim,
                                rgbaColor=[1,0,0,1])

cubeIDs = p.createMultiBody(baseMass = mass,
                            baseInertialFramePosition = [0,0,0],
                            baseCollisionShapeIndex = cube,
                            baseVisualShapeIndex = cube_visual,
                            basePosition = [0,0,0],
                            batchPositions = positions)

# rotation = [-1,0,0,1]
rotation = create_from_axis_angle(-1,0,0,np.pi/2)

# Add Cloth
clothId = p.loadSoftBody("cloth_z_up.obj", 
                         basePosition = [0,0,1],
                         baseOrientation=rotation,
                         scale = 1, 
                         mass = .01,
                         useNeoHookean = 0, 
                         useBendingSprings=1,
                         useMassSpring=1, 
                         springElasticStiffness=10, 
                         springDampingStiffness=.1, 
                         springDampingAllDirections = 1, 
                         useSelfCollision = 0, 
                         frictionCoeff = .5, 
                         useFaceContact=1)

tex=p.loadTexture('Declan_Mulroy.jpg')
p.changeVisualShape(clothId, -1, rgbaColor=[1,1,1,1],textureUniqueId=tex, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

# Get the cloth dimensions!
clothData = p.getMeshData(clothId,-1,flags=p.MESH_DATA_SIMULATION_MESH)

# Add anchors to cloth
p.createSoftBodyAnchor(clothId, 24, cubeIDs[0], -1, [1,0,2])
p.createSoftBodyAnchor(clothId, 15, cubeIDs[0], -1, [1,0,0])
p.createSoftBodyAnchor(clothId, 19, cubeIDs[1], -1, [-1,0,0])
p.createSoftBodyAnchor(clothId, 20, cubeIDs[1], -1, [-1,0,2])


# Setup Simulation
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)


while p.isConnected():
    # p.stepSimulation()
    p.setGravity(0,0,-10)