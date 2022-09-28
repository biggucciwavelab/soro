# -*- coding: utf-8 -*-
"""
Created on Thu Dec 10 15:22:08 2020

@author: elope
"""
import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
from QuaternionRotation import create_from_axis_angle

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath('C:/Users/dmulr/anaconda3/Lib/site-packages/bullet3-master/data')

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.resetDebugVisualizerCamera(3,-420,-30,[0.3,0.9,-2])
p.setGravity(0, 0, -10)

tex = p.loadTexture("uvmap.png")
planeId = p.loadURDF("plane.urdf", [0,0,0])

# Box
# boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

# Wide thin torus
# bunnyId = p.loadSoftBody("torus/torus.obj",mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)

# Thick Fat Torus
rotation = create_from_axis_angle(1,0,0,np.pi/2)

bunnyId = p.loadSoftBody("torus/torus_textured.obj", simFileName="torus.vtk", mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800, basePosition=[0,0,2], baseOrientation=rotation)
p.changeVisualShape(bunnyId, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

while p.isConnected():
  p.stepSimulation()
  p.getCameraImage(320,200)
  p.setGravity(0,0,-10)
