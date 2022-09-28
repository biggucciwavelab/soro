# -*- coding: utf-8 -*-
"""
Created on Thu Dec 17 12:47:14 2020

@author: elopez8

Expanding on the code in 'CubeClothAttachment.py', the goal here is to get a ton of cubes and cloths connected
"""

import pybullet as p
from time import sleep
from time import time as t
import pybullet_data
import numpy as np
from numpy import sqrt, cos, sin, pi
from QuaternionRotation import create_from_axis_angle, multQ
from math import floor
import os
import cv2
import pdb
import openmesh as om
import pywavefront
physicsClient = p.connect(p.GUI)

from get_user import *

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Esteban Desktop
p.setAdditionalSearchPath(pybullet_data_dict[user])
save_loc = 'example/'; os.makedirs(save_loc, exist_ok=True)

# Esteban Laptop
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
#p.resetDebugVisualizerCamera(30, -346, -16, [0, 0, 2])
p.resetDebugVisualizerCamera( cameraDistance=500, cameraYaw=90, cameraPitch=-91, cameraTargetPosition=[0,0,0])


# Floor
planeId = p.loadURDF("plane100.urdf", [0,0,0],globalScaling=30)

# Parameters for cubes (robots):
mass=500
width = 10  # X-dimension
length = 10 # Y-dimension
height = 10 # Z-dimension
cube_dim=[width/2,length/2,height/2] # Halfdimensions of the cube in (x,y,z)
num_cubes = 30

#Cloth
cloth_width = 40 # Cloth length
cloth_height = 10

#Sphere
sphere_radius = 10
sphere_mass = 10

# System parameters
theta = 2*pi/num_cubes
cord_length = cloth_width + width*cos(theta/2) # Cord length between bot centers
R = sqrt((cord_length**2)/(2*(1-cos(theta))))#-.5 # Radius of system 

# Fuction for sphere locations
in_rings_radius = []
gran_per_ring = []
sphereIDs = []

buffer = .1 # Distance between rings 
d_start = width/2 - sphere_radius # Distance between first ring and outer boxes
if d_start<0: d_start = 0
current_radius = R - (width/2 + sphere_radius + d_start)
current_circumference = 2*pi*current_radius

while current_circumference > (2*sphere_radius)*3: # Not allowing less than 3 spheres in a ring
    in_rings_radius.append(current_radius)
    current_radius -= (2*sphere_radius + buffer)
    current_circumference = 2*pi*current_radius
    
for radius in in_rings_radius:
    current_num_interior = floor((2*pi*radius)/(2*sphere_radius))
    gran_per_ring.append(current_num_interior)
    
for num in in_rings_radius: 
    if num<0: 
        in_rings_radius.remove(num)
for num in gran_per_ring: 
    if num<0: 
        gran_per_ring.remove(num)

num_interiors = sum(gran_per_ring)

cubeIDs = []                                                                                                                                    
clothIDs = []

# Make Cubes
cube = p.createCollisionShape(p.GEOM_BOX,
                                halfExtents=cube_dim)
cube_visual = p.createVisualShape(p.GEOM_BOX,
                                halfExtents=cube_dim,
                                rgbaColor=[1,0,0,1])

cylinder = p.createCollisionShape(p.GEOM_CYLINDER,radius=width,height=height)
cylinder_visual = p.createVisualShape(p.GEOM_CYLINDER,radius=width,length=height,rgbaColor=[1,.412,.706,1])



# cylinder = p.createCollisionShape(p.GEOM_CYLINDER,radius=bots.bot_width/2,height=bots.bot_height)
# cylinder_visual = p.createVisualShape(p.GEOM_CYLINDER,radius=bots.bot_width,length=bots.bot_height,rgbaColor=[1,.412,.706,1])
sphere = p.createCollisionShape(p.GEOM_SPHERE,
                                radius = sphere_radius)
sphere_visual= p.createVisualShape(p.GEOM_SPHERE,
                                   radius = sphere_radius,
                                   rgbaColor = [0,1,0,1])



# Will be used to connect the cloth to the rigid objects
AnchorA = []
AnchorB = []

# Put the cubes in
for i in range(num_cubes):
    x = cos(i*theta)*R
    y = sin(i*theta)*R
    z = height/2
    rot = create_from_axis_angle(0,0,1,i*theta)
    pos = [x,y,z]
    
    T = np.array([[cos(theta), sin(theta)],[-sin(theta),cos(theta)]])
    AnchorA.append(np.matmul(np.linalg.inv(T),np.array([cos(theta)*x + sin(theta)*y, -width/2])))
    AnchorB.append(np.matmul(np.linalg.inv(T),np.array([cos(theta)*x + sin(theta)*y, width/2])))
    
    # cubeIDs.append(p.createMultiBody(baseMass = mass,
    #                             baseInertialFramePosition = [0,0,0], # TODO: It is possible this should be the same as the position of the object. Look into the effects of losing it.s
    #                             baseCollisionShapeIndex = cube,
    #                             baseVisualShapeIndex = cube_visual,
    #                             basePosition = pos,
    #                             baseOrientation = rot))

    cubeIDs.append(p.createMultiBody(baseMass = mass,
                                baseInertialFramePosition = [0,0,0], # TODO: It is possible this should be the same as the position of the object. Look into the effects of losing it.s
                                baseCollisionShapeIndex = cylinder,
                                baseVisualShapeIndex = cylinder_visual,
                                basePosition = pos,
                                baseOrientation = rot))
for index, cube in enumerate(cubeIDs):

    p.createConstraint(cube, -1, planeId, -1,
                        jointType = p.JOINT_PRISMATIC,
                        jointAxis = [1,0,0],
                        parentFramePosition = [0,0,0],
                        childFramePosition = [0,0,0],
                        parentFrameOrientation = [0,0,0,1])
    # p.createConstraint(cube, -1, planeId, -1,
    #                     jointType = p.JOINT_PRISMATIC,
    #                     jointAxis = [0,1,0],
    #                     parentFramePosition = [0,0,0],
    #                     childFramePosition = [0,0,0],
    #                     parentFrameOrientation = [0,0,0,1])    
# # Put spheres in
# for index, in_ring in enumerate(gran_per_ring):
#     radius = in_rings_radius[index]
#     for j in range(in_ring):
#         x = radius * cos(j*2*pi/in_ring)
#         y = radius * sin(j*2*pi/in_ring)
#         z = sphere_radius
#         pos = [x,y,z]
#         sphereIDs.append(p.createMultiBody(baseMass = sphere_mass,
#                                             baseInertialFramePosition = [0,0,0],
#                                             baseCollisionShapeIndex = sphere,
#                                             baseVisualShapeIndex = sphere_visual,
#                                             basePosition = pos))
        
        
#         # sphereIDs.append(p.createMultiBody(baseMass = sphere_mass,
#         #                         baseInertialFramePosition = [0,0,0], # TODO: It is possible this should be the same as the position of the object. Look into the effects of losing it.s
#         #                         baseCollisionShapeIndex = cylinder,
#         #                         baseVisualShapeIndex = cylinder_visual,
#         #                         basePosition = pos))
        
# for sphere in sphereIDs:
#     p.changeDynamics(sphere,-1,
#                       lateralFriction=0.5,
#                       spinningFriction = 0.01,
#                       rollingFriction = 0.001)
        
cloth_rot1 = create_from_axis_angle(-1,0,0,pi/2)
start_theta = theta/2
#tex=p.loadTexture('Declan_Mulroy.jpg')
tex = p.loadTexture("uvmap.png")


    
path= pybullet_data_dict[user] + "cloth_z_up.obj"
# scene = pywavefront.Wavefront(path, collect_faces=True)

# vertices=scene.vertices
# faces=scene.mesh_list[0].faces


# w = cloth_width
# h = cloth_height
# Vert_temp=np.array([[w/2,-h/4,0], # 0
#                     [w/4,-h/2,0], # 1
#                     [w/4,-h/4,0], # 2
#                     [0,-h/4,0], # 3
#                     [-w/4,-h/2,0], # 4
#                     [-w/4,-h/4,0], # 5 
#                     [0,h/4,0], # 6
#                     [-w/4,0,0], # 7
#                     [-w/4,h/4,0], # 8
#                     [w/2,h/4,0], # 9
#                     [w/4,0,0], # 10
#                     [w/4,h/4,0], # 11
#                     [0,0,0], # 12
#                     [w/4,h/2,0], # 13
#                     [0,h/2,0], # 14
#                     [w/2,h/2,0], # 15
#                     [-w/2,0,0], # 16
#                     [-w/2,h/4,0], # 17
#                     [-w/4,h/2,0], # 18 
#                     [-w/2,h/2,0], # 19
#                     [-w/2,-h/2,0], # 20
#                     [-w/2,-h/4,0], # 21
#                     [0,-h/2,0], # 22
#                     [w/2,0,0], # 23
#                     [w/2,-h/2,0]]) # 24

# V = []
# mesh = om.TriMesh()
# for i in range(len(vertices)):
#     temp=Vert_temp[i,:]
#     temp=temp.flatten()
#     V.append(mesh.add_vertex([temp[0],temp[1],temp[2]]))

# vh_list = []  
# for i in range(len(faces)):
#     temp = faces[i]
#     mesh.add_face([V[temp[0]],V[temp[1]],V[temp[2]]])

#     vh_list.append([V[temp[0]],V[temp[1]],V[temp[2]]])

# om.write_mesh('membrane.obj', mesh)

# Add Cloths
for i in range(num_cubes):
    theta_now = i*theta+start_theta
    cloth_rot2 = create_from_axis_angle(0,0,1,theta_now)
    cloth_rotation1 = multQ(cloth_rot1,cloth_rot2)
    cloth_rot3 = create_from_axis_angle(0,0,1,pi/2)
    cloth_rotation = multQ(cloth_rotation1,cloth_rot3)
    cloth_position = [cos(theta_now)*R, sin(theta_now)*R, cloth_height/2]

    file='F:/Soro_chrono/python/PyBullet/example/membrane.obj'

    # Convert path to file if you want to try the new 
    clothId = p.loadSoftBody(path, 
                              basePosition = cloth_position,
                              baseOrientation = cloth_rotation,
                              scale = 1, 
                              mass = .01,
                              useNeoHookean = 0, 
                              useBendingSprings = 1,
                              useMassSpring = 1, 
                              springElasticStiffness = 10, 
                              springDampingStiffness = .1, 
                              springDampingAllDirections = 1, 
                              useSelfCollision = 0, 
                              frictionCoeff = .5, 
                              useFaceContact = 1)    
    
    # clothId = p.loadSoftBody(path, 
    #                           basePosition = cloth_position,
    #                           baseOrientation = cloth_rotation,
    #                           mass = 0.1,
    #                           useMassSpring = 1,
    #                           useBendingSprings = 1,
    #                           # useNeoHookean = 1, 
    #                           # NeoHookeanMu = 180, 
    #                           # NeoHookeanLambda = 600, 
    #                           # NeoHookeanDamping = 0.01, 
    #                           # collisionMargin = 0.006,
    #                           useSelfCollision = 0,
    #                           useFaceContact = True,
    #                           frictionCoeff = 0.5, 
    #                           repulsionStiffness = 800)
    p.changeVisualShape(clothId, -1, rgbaColor=[1,1,1,1],textureUniqueId=tex, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
    clothIDs.append(clothId)

# Add anchors to cloth.
    """
    Cloth Edges are at:
    (20) ----------------- (24)
        |                 |
        |                 |
 Bot i  |                 | Bot i+1
    (16)|                 |(23)
        |                 |
        |                 |
        |                 |
    (19) ----------------- (15)
    """
# for ind in range(num_cubes):
info=p.getBasePositionAndOrientation(cubeIDs[1])
p.createSoftBodyAnchor(clothId, 24, cubeIDs[0], -1, [1,0,2])
p.createSoftBodyAnchor(clothId, 15, cubeIDs[0], -1, [1,0,0])
p.createSoftBodyAnchor(clothId, 19, cubeIDs[1], -1, [-1,0,0])
p.createSoftBodyAnchor(clothId, 20, cubeIDs[1], -1, [-1,0,2])

for index, cube in enumerate(cubeIDs):
    # print('------------')
    # print('cube: ',cube)
    # print('index: ',index)
    # print('------------')
    # All cubes EXCEPT the last
    if cube != num_cubes:    
        # Anchor A
        p.createSoftBodyAnchor(clothIDs[index], 19, cube, -1, [AnchorB[index][0], AnchorB[index][1], 0])
        p.createSoftBodyAnchor(clothIDs[index], 16, cube, -1, [AnchorB[index][0], AnchorB[index][1], height/2])
        p.createSoftBodyAnchor(clothIDs[index], 20, cube, -1, [AnchorB[index][0], AnchorB[index][1], height])
        
        # Anchor B (Next Cube)
        p.createSoftBodyAnchor(clothIDs[index], 15, cube+1, -1, [AnchorA[index+1][0], AnchorA[index+1][1], 0])
        p.createSoftBodyAnchor(clothIDs[index], 23, cube+1, -1, [AnchorA[index+1][0], AnchorA[index+1][1], height/2])
        p.createSoftBodyAnchor(clothIDs[index], 24, cube+1, -1, [AnchorA[index+1][0], AnchorA[index+1][1], height])

    # The last cube
    else:
        p.createSoftBodyAnchor(clothIDs[index], 19, cube, -1, [AnchorB[index][0], AnchorB[index][1], 0])
        p.createSoftBodyAnchor(clothIDs[index], 16, cube, -1, [AnchorB[index][0], AnchorB[index][1], height/2])
        p.createSoftBodyAnchor(clothIDs[index], 20, cube, -1, [AnchorB[index][0], AnchorB[index][1], height])
       #print('cubeIDs[0]: ',cubeIDs[0])
        # Anchor B (Next Cube)
        p.createSoftBodyAnchor(clothIDs[index], 15, cubeIDs[0], -1, [AnchorA[0][0], AnchorA[0][1], 0])
        p.createSoftBodyAnchor(clothIDs[index], 23, cubeIDs[0], -1, [AnchorA[0][0], AnchorA[0][1], height/2])
        p.createSoftBodyAnchor(clothIDs[index], 24, cubeIDs[0], -1, [AnchorA[0][0], AnchorA[0][1], height])
        
    ### Mate the cubes to the floor
    p.createConstraint(cube, -1, planeId, -1,
                        jointType = p.JOINT_PRISMATIC,
                        jointAxis = [1,0,0],
                        parentFramePosition = [0,0,0],
                        childFramePosition = [0,0,0],
                        parentFrameOrientation = [0,0,0,1])
    p.createConstraint(cube, -1, planeId, -1,
                        jointType = p.JOINT_PRISMATIC,
                        jointAxis = [0,1,0],
                        parentFramePosition = [0,0,0],
                        childFramePosition = [0,0,0],
                        parentFrameOrientation = [0,0,0,1])
    # p.createConstraint(cube, -1, planeId, -1,
    #                     jointType = p.JOINT_REVOLUTE,
    #                     jointAxis = [0,0,1],
    #                     parentFramePosition = [0,0,0],
    #                     childFramePosition = [0,0,0])
    
#### Setup Simulation
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

#### Setup Camera
cam_width = 720
cam_height = 720
image_directory = save_loc + 'OuterMembrane_Testing/'
os.makedirs(image_directory,exist_ok=True)

time_force = 1
time=0

start_time = t()
while p.isConnected():
    p.stepSimulation()
    p.setGravity(0,0,-10*100)
    
    # Image Text
    runtime = round(t() - start_time,3)
    text = p.addUserDebugText(str(runtime),[0,0,height+.5],
                              textColorRGB = [1,0,0],
                              textSize = 2)
    
    # Record
    img = p.getCameraImage(cam_width, cam_height)
    cv2image = np.array(img[2])[:,:,:3]
    cv2.imshow('Testing',cv2image)
    cv2.waitKey(1)

    image_title = image_directory+ 'img{}.jpg'.format(str(time))
    cv2.imwrite(image_title, cv2image)
    F=1000*300
    # Applying forces to bot
    #if time<time_force:
    for cube in cubeIDs:
        pos, _ = p.getBasePositionAndOrientation(cube)
        x,y,z = pos
        force_applied = [-x,0,0]
        force_applied /= np.linalg.norm(force_applied)
        p.applyExternalForce(cube,-1,F*force_applied,[0,0,height/6],p.WORLD_FRAME)
    #time+=1 
    
    #physics_info = p.getDynamicsInfo(1,-1)
    #print(physics_info)
    p.removeUserDebugItem(text)
    
    cv2.destroyAllWindows() # Kill all images