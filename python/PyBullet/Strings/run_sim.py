# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 17:07:11 2021

@author: dmulr
"""

import pybullet as p
from get_user import *
from time import sleep
import pybullet_data
import numpy as np
from objects import *
physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

# Getting relative directories for each machine
data_dir =  pybullet_data_dict[user]
mainDirectory = main_directory_dict[user]
os.chdir(os.path.dirname(__file__))

p.resetDebugVisualizerCamera( cameraDistance=1, cameraYaw=90, cameraPitch=-91, cameraTargetPosition=[0,0,1])
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath(data_dir)

planeId = p.loadURDF("plane100.urdf", [0,0,0])


files = []
for file in os.listdir(mainDirectory):
    files.append(file)
name = files[-1]


mainDirectory = mainDirectory
parameters=np.load(mainDirectory+name+'/Parameters.npy',allow_pickle=True)
parameters=parameters.tolist()
convert_dist=parameters['convert_dist']
convert_mass=parameters['convert_mass']
p.resetDebugVisualizerCamera( cameraDistance=1*convert_dist, cameraYaw=90, cameraPitch=-91, cameraTargetPosition=[0,0,1])

bots=robots(name)
interior=Particles(name)

red_color = [1,0,0,1]
green_color = [0,1,0,1]
black_color = [1,1,1,1]
blue_color = [0,0,1,1]

# for cubic shapes
bot_cube = p.createCollisionShape(p.GEOM_BOX,halfExtents=bots.cube_dim)
bot_cube_visual = p.createVisualShape(p.GEOM_BOX,halfExtents=bots.cube_dim,rgbaColor=red_color)

particle_cube = p.createCollisionShape(p.GEOM_BOX,halfExtents=bots.cube_dim)
particle_cube_visual = p.createVisualShape(p.GEOM_BOX,halfExtents=bots.cube_dim,rgbaColor=green_color)

# for cylinder shapes
bot_cylinder = p.createCollisionShape(p.GEOM_CYLINDER,radius=(bots.bot_width/2),height=(bots.bot_height/2))
bot_cylinder_visual = p.createVisualShape(p.GEOM_CYLINDER,radius=bots.bot_width/2,length=bots.bot_height,rgbaColor=red_color)

particle_cylinder = p.createCollisionShape(p.GEOM_CYLINDER,radius=(bots.bot_width/2),height=(bots.bot_height/2))
particle_cylinder_visual = p.createVisualShape(p.GEOM_CYLINDER,radius=(bots.bot_width/2),length=(bots.bot_height),rgbaColor=green_color)

tex = p.loadTexture("uvmap.png")
for i in range(len(bots.bot_position)):

    if bots.bot_geom=='cube':
        bots.botIDs.append(p.createMultiBody(baseMass = bots.bot_mass,baseInertialFramePosition = [0,0,0],baseCollisionShapeIndex = bot_cube, baseVisualShapeIndex = bot_cube_visual,basePosition = (bots.bot_position[i]),baseOrientation = bots.bot_rotation[i]))
        
    if bots.bot_geom=='cylinder':
        bots.botIDs.append(p.createMultiBody(baseMass = bots.bot_mass,baseInertialFramePosition = [0,0,0],baseCollisionShapeIndex = bot_cylinder, baseVisualShapeIndex = bot_cylinder_visual,basePosition = (bots.bot_position[i]),baseOrientation = bots.bot_rotation[i]))


for i in range(len(interior.particle_position)):

    if interior.particle_geom=='cube':
        interior.particleIDs.append(p.createMultiBody(baseMass = interior.particle_mass,baseInertialFramePosition = [0,0,0],baseCollisionShapeIndex = particle_cube, baseVisualShapeIndex = particle_cube_visual,basePosition = interior.particle_position[i]))
        
    if interior.particle_geom=='cylinder':
        interior.particleIDs.append(p.createMultiBody(baseMass = interior.particle_mass,baseInertialFramePosition = [0,0,0],baseCollisionShapeIndex = particle_cylinder, baseVisualShapeIndex = particle_cylinder_visual,basePosition = interior.particle_position[i]))

for obj in bots.botIDs:
    p.changeDynamics(obj,-1,
                     lateralFriction=bots.lateralFriction,
                     spinningFriction = bots.spinningFriction,
                     rollingFriction = bots.rollingFriction)
        
for obj in interior.particleIDs:
    p.changeDynamics(obj,-1,
                     lateralFriction = interior.lateralFriction,
                     spinningFriction = interior.spinningFriction,
                     rollingFriction = interior.rollingFriction)    
    

for i in range(len(bots.bot_position)):
    #file='F:/Soro_chrono/python/PyBullet/example/membrane.obj'
    
    file=bots.mainDirectory+bots.name+'/membrane.obj'
    clothId = p.loadSoftBody(file, 
                              basePosition = bots.cloth_position[i],
                              baseOrientation = bots.cloth_rotation[i],
                              scale = 1, 
                              mass = .01,
                              useNeoHookean = 0, 
                              useBendingSprings = 0,
                              useMassSpring = 1, 
                              springElasticStiffness = 10, 
                              springDampingStiffness = 0, 
                              springDampingAllDirections = 1,
                              useSelfCollision = 0, 
                              frictionCoeff = bots.lateralFriction, 
                              useFaceContact = 1)       
    clothId = p.loadSoftBody(file, 
                              basePosition = bots.cloth_position[i],
                              baseOrientation = bots.cloth_rotation[i],
                              scale = 1, 
                              mass = .01,     
                              useMassSpring = 1,
                              useBendingSprings = 1,
                             # useNeoHookean = 1, 
                             # NeoHookeanMu = 180, 
                             # NeoHookeanLambda = 600, 
                             # NeoHookeanDamping = 0.01, 
                             # collisionMargin = 0.006,
                              useSelfCollision = 0,
                              useFaceContact = True,
                              frictionCoeff = 0.5, 
                              repulsionStiffness = 800)

    p.changeVisualShape(clothId, -1, rgbaColor=black_color,textureUniqueId=tex, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
    bots.clothIDs.append(clothId)
    

# for ind in range(num_cubes):
info=p.getBasePositionAndOrientation(bots.botIDs[1])

for index, cube in enumerate(bots.botIDs):
    # All cubes EXCEPT the last
    if cube != bots.nb:    
        # Anchor A
        p.createSoftBodyAnchor(bots.clothIDs[index], 19, cube, -1, [bots.AnchorB[index][0], bots.AnchorB[index][1], 0])
        p.createSoftBodyAnchor(bots.clothIDs[index], 16, cube, -1, [bots.AnchorB[index][0], bots.AnchorB[index][1], bots.bot_height/2])
        p.createSoftBodyAnchor(bots.clothIDs[index], 20, cube, -1, [bots.AnchorB[index][0], bots.AnchorB[index][1], bots.bot_height])
        
        # Anchor B (Next Cube)
        p.createSoftBodyAnchor(bots.clothIDs[index], 15, cube+1, -1, [bots.AnchorA[index+1][0], bots.AnchorA[index+1][1], 0])
        p.createSoftBodyAnchor(bots.clothIDs[index], 23, cube+1, -1, [bots.AnchorA[index+1][0], bots.AnchorA[index+1][1], bots.bot_height/2])
        p.createSoftBodyAnchor(bots.clothIDs[index], 24, cube+1, -1, [bots.AnchorA[index+1][0], bots.AnchorA[index+1][1], bots.bot_height])

    # The last cube
    else:
        p.createSoftBodyAnchor(bots.clothIDs[index], 19, cube, -1, [bots.AnchorB[index][0], bots.AnchorB[index][1], 0])
        p.createSoftBodyAnchor(bots.clothIDs[index], 16, cube, -1, [bots.AnchorB[index][0], bots.AnchorB[index][1], bots.bot_height/2])
        p.createSoftBodyAnchor(bots.clothIDs[index], 20, cube, -1, [bots.AnchorB[index][0], bots.AnchorB[index][1], bots.bot_height])
        #print('cubeIDs[0]: ',bots.botIDs[0])
        # Anchor B (Next Cube)
        p.createSoftBodyAnchor(bots.clothIDs[index], 15, bots.botIDs[0], -1, [bots.AnchorA[0][0], bots.AnchorA[0][1], 0])
        p.createSoftBodyAnchor(bots.clothIDs[index], 23, bots.botIDs[0], -1, [bots.AnchorA[0][0], bots.AnchorA[0][1], bots.bot_height/2])
        p.createSoftBodyAnchor(bots.clothIDs[index], 24, bots.botIDs[0], -1, [bots.AnchorA[0][0], bots.AnchorA[0][1], bots.bot_height])

    #### Mate the cubes to the floor
    # p.createConstraint(cube, -1, planeId, -1,
    #                     jointType = p.JOINT_PRISMATIC,
    #                     jointAxis = [1,0,0],
    #                     parentFramePosition = [0,0,0],
    #                     childFramePosition = [0,0,0],
    #                     parentFrameOrientation = [0,0,0,1])
    # p.createConstraint(cube, -1, planeId, -1,
    #                     jointType = p.JOINT_PRISMATIC,
    #                     jointAxis = [0,1,0],
    #                     parentFramePosition = [0,0,0],
    #                     childFramePosition = [0,0,0],
    #                     parentFrameOrientation = [0,0,0,1])
    # p.createConstraint(cube, -1, planeId, -1,
    #                     jointType = p.JOINT_REVOLUTE,
    #                     jointAxis = [0,0,1],
    #                     parentFramePosition = [0,0,0],
    #                     childFramePosition = [0,0,0])
        
    

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

F=30*convert_dist*convert_mass
while p.isConnected():
    p.stepSimulation()
    p.setGravity(0,0,-10*convert_dist)

    for cube in bots.botIDs:
        pos, _ = p.getBasePositionAndOrientation(cube)
        x,y,z = pos
        force_applied = [-x,0,0]
        force_applied /= np.linalg.norm(force_applied)
        p.applyExternalForce(cube,-1,F*force_applied,[0,0,bots.bot_height/6],p.WORLD_FRAME)    

