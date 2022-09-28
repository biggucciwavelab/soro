"""
This script will test the importation of a general mesh in PyBullet

author: Esteban Lopez
"""

import pybullet as p
import pybullet_data
from numpy import pi
import numpy as np
import pathlib
import sys
from poisson_sampling import *
from get_user import *
file_path = pathlib.Path(__file__).parent.resolve()

from QuaternionRotation import create_from_axis_angle
import pdb

from tqdm import tqdm

if __name__ =='__main__':
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setAdditionalSearchPath(pybullet_data_dict[user])

    obj_file = 'Blender/Icosphere.obj'

    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.resetDebugVisualizerCamera(3,-420,-30,[0.3,0.9,-2])

    scale = 1 # Changing the size of sim. You CAN edit this!!
    rotation = create_from_axis_angle(1,0,0,0)


    icosphere_radius = 0.25 # Made in Blender (c). DO NOT EDIT!!
    
    # All the parameters in this function need to be played with to understand their affects
    mesh = p.loadSoftBody(obj_file,
                            basePosition = [0,0,0],
                            baseOrientation = rotation,
                            springElasticStiffness = .01,
                            springDampingStiffness = 1,
                            # springBendingStiffness = 10,
                            scale=scale,
                            mass=0.1,
                            useMassSpring=1,
                            useBendingSprings=1,
                            # useNeoHookean = 1,
                            useSelfCollision=1,
                            useFaceContact=True,
                            frictionCoeff=0.5
                            # repulsionStiffness=800
                            )

    # Editing visual for mesh   
    p.changeVisualShape(mesh, -1, rgbaColor=[1,.412,.706,0.5], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    # Get Mesh data
    node_loc = []
    data = p.getMeshData(mesh, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    verts = np.asarray(data[1])

    collect_data = False
    if collect_data: node_loc.append(verts[0])

    # Interior particle creation
    particle_radius = .02
    particle_mass = .01
    cube_width = cube_height = l = icosphere_radius*scale - particle_radius*2
    poisSamples = Poisson_Sampling(particle_radius*2, l,l)
    poisSamples.reset()

    # We are going to go through the various Z-levels
    # And get all particle positions
    z_low = -l/2 + particle_radius
    z_high = l/2 - particle_radius
    levels = np.arange(z_low, z_high, step=particle_radius*2)
    particle_positions = []
    for z in levels:
        coords = poisSamples.get_samples() # Returns a list of x-y coordinates
        for coord in coords:
            this_particle_pos = [coord[0] - l/2, coord[1] - l/2, z]
            particle_positions.append(this_particle_pos)
        poisSamples.reset()
    in_particle = p.createCollisionShape(p.GEOM_SPHERE,
                                        radius = particle_radius)
    particle_IDs = []
    for pos in particle_positions:
        particle_IDs.append(p.createMultiBody(
                            baseMass = particle_mass,
                            baseCollisionShapeIndex = in_particle,
                            basePosition = pos
        ))

    # Define parameters for a control sphere
    control_sphere_radius = .005
    control_sphere_mass = .1
    control_sphere = p.createCollisionShape(p.GEOM_SPHERE,
                                radius = control_sphere_radius)
    control_sphere_visual = p.createVisualShape(p.GEOM_SPHERE,
                                        radius = control_sphere_radius,
                                        rgbaColor = [0,0,0,1])                  

    control_sphere_locs = np.zeros((data[0], 3))
    for i, vert in enumerate(verts):
        dir = vert / np.linalg.norm(vert)
        addition = dir*control_sphere_radius
        control_sphere_locs[i] = vert + addition*1.1

    # Creating the control spheres
    control_sphere_Ids = []
    for index, pos in enumerate(control_sphere_locs):
        control_sphere_Ids.append(p.createMultiBody(
                                    baseMass = control_sphere_mass,
                                    baseInertialFramePosition = [0,0,0],
                                    baseCollisionShapeIndex = control_sphere,
                                    basePosition = pos,
                                    baseVisualShapeIndex = control_sphere_visual
        ))
    
    # Attaching the control sphere to mesh
    for index, cs in enumerate(control_sphere_Ids):
        pass
        p.createSoftBodyAnchor(mesh, index, cs, -1)

    # Simulate!
    p.setRealTimeSimulation(0)
    # timestep = 1/240.
    timestep = .001
    p.setTimeStep(timestep)    

    force_mag = 1 # for control balls. You CAN edit this.

    T = 0 
    end=10_000_000
    for i in tqdm(range(end)):
        if not p.isConnected: 
            break

        T+=1
        if collect_data:
            if T%1000==0:
                pass
                print(T)
        if T>end:
            break
        pass  
        p.stepSimulation()

        # Apply a force to control bodies in direction of origin
        for c_ball in control_sphere_Ids:
            pos = np.asarray(p.getBasePositionAndOrientation(c_ball)[0])
            n_pos = pos / np.linalg.norm(pos)
            force = -force_mag*n_pos
            p.applyExternalForce(c_ball,-1, force, n_pos, p.WORLD_FRAME)
            

        if collect_data:
            data = p.getMeshData(mesh, -1, flags=p.MESH_DATA_SIMULATION_MESH)
            verts = np.asarray(data[1])
            node_loc.append(verts[0])
            # print("--------------")
            # print(data[0])
            # print(data[1])
            # text_uid = []
            # for i in range(data[0]):
            #     pos = data[1][i]
            #     uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
            #     text_uid.append(uid)

    if collect_data:

        import matplotlib.pyplot as plt
        node_loc = np.asarray(node_loc)
        maxes = np.max(np.abs(node_loc), axis=0)
        normed = node_loc.T / maxes[:,None]
        normed = normed.T

        plt.figure()
        plt.plot(normed[:,0],label='X Position')
        plt.plot(normed[:,1],label='Y Position')
        plt.plot(normed[:,2],label='Z Position')
        plt.legend()
        plt.title('Normed Location')
        plt.show()