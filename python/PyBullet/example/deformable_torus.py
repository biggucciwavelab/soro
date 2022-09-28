# -*- coding: utf-8 -*-
"""
Created on Mon Nov 30 16:46:43 2020

@author: elope
"""

import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
from numpy import pi, cos, sin, arccos
from math import floor
import os
import cv2
from QuaternionRotation import create_from_axis_angle, multQ
import pdb


if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Esteban's Laptop
    # p.setAdditionalSearchPath("C:/Users/17088/Anaconda3/Lib/site-packages/bullet3-master/data/")
    
    # Esteban's Desktop
    p.setAdditionalSearchPath('C:/Users/dmulr/anaconda3/Lib/site-packages/bullet3-master/data')
    image_directory = './DeformableTorus/Images/'

    os.makedirs(image_directory, exist_ok=True)
    
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.resetDebugVisualizerCamera(3,-420,-30,[0.3,0.9,-2])
    # p.setGravity(0, 0, -10)
    
    debug = False
    
    tex = p.loadTexture("checker_grid.jpg")
    tex2 = p.loadTexture("checker_grid.jpg")
    # planeId = p.loadURDF("plane.urdf", [0,0,0])
    
    start_height = 0.5
    
    orientation = create_from_axis_angle(1,0,0,pi/2)
    bunnyId = p.loadSoftBody("torus/torus.obj",
                             # simFileName="torus.vtk", 
                             mass = 0.1,
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
                             repulsionStiffness = 800,
                             basePosition = [0,0,start_height],
                             baseOrientation = orientation)
    
    p.changeVisualShape(bunnyId, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    #### Add Spheres in torus
    sphere_radius = .45/2
    sphere_mass = .01
    num_spheres = 4
    R = 0.6496*1.5
    theta = 2*pi/num_spheres
    sphereIds = []
    
    sphere = p.createCollisionShape(p.GEOM_SPHERE,
                                    radius = sphere_radius)
    sphere_visual = p.createVisualShape(p.GEOM_SPHERE,
                                        radius = sphere_radius,
                                        rgbaColor = [0,1,0,1])
    
    for j in range(num_spheres):
        x = R*cos(j*theta)
        y = R*sin(j*theta)
        z = start_height
        pos = [x,y,z]
        sphereIds.append(p.createMultiBody(baseMass = sphere_mass,
                                           baseInertialFramePosition = [0,0,0],
                                           baseCollisionShapeIndex = sphere,
                                           baseVisualShapeIndex = sphere_visual,
                                           basePosition = pos))
    
    #### Adding interior particles to the torus
    particles = []
    fake_theta = arccos((2*R**2 - 4*sphere_radius**2)/2*R**2) + pi/16 # Small buffer at the end to ensure no collision with large spheres at beginning of simulation
    space = R*(theta - fake_theta)
    particle_radius = 0.005
    particles_between_ball = floor(space/(2*particle_radius)) - 3 # The 3 is a buffer.
    theta_particle = (theta - fake_theta)/particles_between_ball
    circles = 4
    layers = 3
    
    num_radii = 2*circles - 2
    D = num_radii*particle_radius
    r1 = R-D/2 - 2*particle_radius
    
    num_height_particles = 2*layers - 2
    h1= start_height - (num_height_particles*particle_radius)/2
    
    #TODO: Fix this. The particles are very small and they needn't be.
    # for lay in range(1,layers+1):
    #     for circle in range(1,circles+1):
            
    #         # Iterate between all globes
    #         for obs in range(num_spheres):
    #             theta_start = theta*obs+fake_theta/2
                
    #             for particle in range(particles_between_ball):
    #                 x = circle*r1*cos(theta_start + theta_particle*particle)
    #                 y = (circle*2*particle_radius+r1)*sin(theta_start + theta_particle*particle)
    #                 z = (lay*2*particle_radius+h1)
                    
    #                 par = p.loadURDF("sphere_1cm.urdf", [x,y,z], useMaximalCoordinates=True)
    #                 particles.append(par)
                
        
    # p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
    p.setRealTimeSimulation(1)
    
    if debug:
        data = p.getMeshData(bunnyId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
        print("--------------")
        print("data=",data)
        print(data[0])
        print(data[1])
        text_uid = []
        for i in range(data[0]):
            pos = data[1][i]
            uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
            text_uid.append(uid)
     
    image_index = 0
    while p.isConnected():
       p.stepSimulation()
       img = p.getCameraImage(720,720)
       # p.setGravity(0,0,-10)
       
       force_applied = np.array([1,0,0])
       force2 = np.array([0,1,0])
       p.applyExternalForce(sphereIds[0], -1, 5*force_applied, [0,0,sphere_radius], p.WORLD_FRAME)
       p.applyExternalForce(sphereIds[2], -1, -5*force_applied, [0,0,sphere_radius], p.WORLD_FRAME)
       p.applyExternalForce(sphereIds[1], -1, 5*force2, [0,0,sphere_radius], p.WORLD_FRAME)
       p.applyExternalForce(sphereIds[3], -1, -5*force2, [0,0,sphere_radius], p.WORLD_FRAME)
       
       cv2image = np.array(img[2])[:,:,:3]
       cv2.waitKey(1)
       image_title = image_directory + 'img{}.jpg'.format(str(image_index))
       cv2.imwrite(image_title,cv2image)
       image_index += 1
       
       # if debug:
       #  data_current = p.getMeshData(bunnyId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
       #  for i in range(data_current[0]):
       #    pos = data_current[1][i]
       #    uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1], replaceItemUniqueId=text_uid[i])