# -*- coding: utf-8 -*-
"""
Created on Wed Dec 30 20:23:21 2020

@author: elopez8@hawk.iit.edu

Collision test to see how many objects PyBullet can render collisions for before becoming unstable

Refer to the following to learn how to grab images:
https://github.com/bulletphysics/bullet3/issues/2777
"""
import pybullet as p
from time import sleep
from time import time as t
import pybullet_data
import numpy as np
import os
from numpy import sqrt, cos, sin, pi
from QuaternionRotation import create_from_axis_angle, multQ
from math import floor
import cv2
import pdb

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Esteban Desktop
p.setAdditionalSearchPath('C:/Users/dmulr/anaconda3/Lib/site-packages/bullet3-master/data')
save_loc = './ManyCollisions/'

# Esteban Laptop
# p.setAdditionalSearchPath("C:/Users/17088/Anaconda3/Lib/site-packages/bullet3-master/data/")

# Big Gucci
# p.setAdditionalSearchPath("C:/ProgramData/Anaconda3/Lib/site-packages/bullet3-master/data/")
# save_loc = 'C:/soro_bitbucket/python/Reinforcement_Learning/PyBullet/ManyCollisions/'
os.makedirs(save_loc,exist_ok=True)

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

#### Floor
planeId = p.loadURDF("plane100.urdf", [0,0,0])
p.changeDynamics(planeId,-1,
                     lateralFriction = 0,
                     spinningFriction = 0.01,
                     rollingFriction = 0.001)

#### Parameters for wall:
mass=0
width = 1  # X-dimension
length = 50 # Y-dimension
height = 20 # Z-dimension
wall_dim=[width/2,length/2,height/2] # Halfdimensions of the cube in (x,y,z)
num_cubes = 50

#### Parameters for granulars
sphere_radius = 0.5
sphere_mass = 0.1
mu_lateral = 0.2
contact_damping = 0.0001
contact_stiffness = 0.0001

#### Create Walls
wall = p.createCollisionShape(p.GEOM_BOX,
                                halfExtents=wall_dim)
wall_visual = p.createVisualShape(p.GEOM_BOX,
                                halfExtents=wall_dim,
                                rgbaColor=[1,0,0,0])

rotation = create_from_axis_angle(0,0,1,pi/2)

wall_1 = p.createMultiBody(baseMass = mass,
                           baseInertialFramePosition = [0,0,0],
                           baseCollisionShapeIndex = wall,
                            baseVisualShapeIndex = wall_visual,
                           basePosition = [length/2,0,height/2])
wall_2 = p.createMultiBody(baseMass = mass,
                           baseInertialFramePosition = [0,0,0],
                           baseCollisionShapeIndex = wall,
                            baseVisualShapeIndex = wall_visual,
                           basePosition = [0,length/2,height/2],
                           baseOrientation = rotation)
wall_3 = p.createMultiBody(baseMass = mass,
                           baseInertialFramePosition = [0,0,0],
                           baseCollisionShapeIndex = wall,
                            baseVisualShapeIndex = wall_visual,
                           basePosition = [-length/2,0,height/2])
wall_4 = p.createMultiBody(baseMass = mass,
                           baseInertialFramePosition = [0,0,0],
                           baseCollisionShapeIndex = wall,
                            baseVisualShapeIndex = wall_visual,
                           basePosition = [0,-length/2,height/2],
                           baseOrientation = rotation)

#### Change Wall dynamics
p.changeDynamics(wall_1, -1, lateralFriction=mu_lateral)
p.changeDynamics(wall_2, -1, lateralFriction=mu_lateral)
p.changeDynamics(wall_3, -1, lateralFriction=mu_lateral)
p.changeDynamics(wall_4, -1, lateralFriction=mu_lateral)


#### Create Balls (Passive Particles)
sphere = p.createCollisionShape(p.GEOM_SPHERE,
                                radius = sphere_radius)
sphere_visual= p.createVisualShape(p.GEOM_SPHERE,
                                   radius = sphere_radius,
                                   rgbaColor = [0,0,1,1])
last_sphere = p.createVisualShape(p.GEOM_SPHERE,
                                  radius = sphere_radius,
                                  rgbaColor=[1,0,0,1])

in_rings_radius = []
gran_per_ring = []
sphereIDs = []

buffer = 0.1 # Distance between particles at start of simulation
current_radius = length/2 - (width/2 + sphere_radius + buffer)
current_circumference = 2*pi*current_radius

while current_circumference > (2*sphere_radius)*3:
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

layers = floor(height/(2*sphere_radius+buffer))
num_particles = sum(gran_per_ring*layers)

print('--'*20)
print('Number of colliding objects:', num_particles)
print('--'*20)

for lay in range(layers):

    for index, in_ring in enumerate(gran_per_ring):
        radius = in_rings_radius[index]
        for j in range(in_ring):
            x = radius * cos(j*2*pi/in_ring)
            y = radius * sin(j*2*pi/in_ring)
            z = 2*sphere_radius*(lay+1)
            pos = [x,y,z]
            if lay == layers-1:
                sphereIDs.append(p.createMultiBody(baseMass = sphere_mass,
                                                baseInertialFramePosition = [0,0,0],
                                                baseCollisionShapeIndex = sphere,
                                                baseVisualShapeIndex = last_sphere,
                                                basePosition = pos))
            else:
                sphereIDs.append(p.createMultiBody(baseMass = sphere_mass,
                                                baseInertialFramePosition = [0,0,0],
                                                baseCollisionShapeIndex = sphere,
                                                baseVisualShapeIndex = sphere_visual,
                                                basePosition = pos))
                
#### Update the ball's Dynamics
# Dynamics must include contact stiffness, damping, and friction.
for sphere in sphereIDs:
    p.changeDynamics(sphere,-1,
                     lateralFriction = mu_lateral,
                     spinningFriction = 0.01,
                     rollingFriction = 0.001)

#### Setup Simulation
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

#### Setup Camera
cam_width=720
cam_height=720
cam_pos = [1,1,2]
cam_target = [0,0,0]
cam_up = [0,0,1]
view_matrix = p.computeViewMatrix(cam_pos,cam_target,cam_up)

fov = 60
aspect = width / height
near = 0.02
far = 1
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

image_directory = save_loc+'VideoImages/'
os.makedirs(image_directory,exist_ok=True)

start_time = t()
image_index = 0
while p.isConnected():
    print('--'*20)
    print('Step Number:',image_index)
    print('Current Runtime:',t()-start_time,'seconds')
    p.stepSimulation()
    p.setGravity(0,0,-10)
    # velocities = np.zeros(num_particles)
    # for index, sphere in enumerate(sphereIDs):
    #     velocities[index] = np.linalg.norm(p.getBaseVelocity(sphere)[0])
        
    # Grab images for video
    img = p.getCameraImage(cam_width, cam_height)
    cv2image = np.array(img[2])[:,:,:3]
    cv2.waitKey(1)
    image_title = image_directory + 'img{}.jpg'.format(str(image_index))
    image_index += 1
    cv2.imwrite(image_title,cv2image)
    print('Image Saved!')
    
    # Check if the system has stopped moving
    # velocity_thresh = 0e-7
    # result = np.greater(velocities, velocity_thresh)
    # too_fast = np.any(result)
    # if not too_fast:
    #     p.disconnect()
    #     end_time = t()
        
    
    # file = open(save_loc+'Test_Information.txt', 'w+')
    # file.write('Number of particles in this simulation:',num_particles,'\r\n')
    # file.write("Simulation Runtime:", str(end_time-start_time),'seconds\r\n')
    # file.close()