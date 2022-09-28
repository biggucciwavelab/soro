# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 13:12:52 2021

@author: dmulr
"""

import os
from get_user import *
import csv

import pathlib
from shutil import copyfile

import numpy as np
from numpy.linalg import norm

from datetime import datetime
import pybullet as p
import pybullet_data

import time
from time import sleep
from time import time as t

import openmesh as om
import pywavefront

global mainDirectory
global data_dir
mainDirectory = main_directory_dict[user]
data_dir = pybullet_data_dict[user]

# Defining color schemes
# Defining color schemes
red_color = [1,0,0,1]
green_color = [0,1,0,1]
black_color = [1,1,1,1]
blue_color = [0,0,1,1]

class robots:
    def __init__(self,name):
        self.name=name
        self.mainDirectory = mainDirectory
        self.parameters = np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True).tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.nb=self.parameters['nb']
        self.bot_mass = self.convert_mass*self.parameters['bot_mass']
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.bot_height = self.convert_dist*self.parameters['bot_height']
        self.bot_geom = self.parameters['bot_geom']
        self.R = self.convert_dist*self.parameters['R']
        
        self.cloth_width = self.convert_dist*self.parameters['cloth_width']
        self.cloth_height = self.convert_dist*self.parameters['cloth_height']
        
        self.lateralFriction = self.parameters['lateralFriction'] 
        self.spinningFriction = self.parameters['spinningFriction'] 
        self.rollingFriction = self.parameters['rollingFriction'] 
    
        self.theta = 2*np.pi/self.nb
        self.start_theta = self.theta/2
        
        self.AnchorA = []
        self.AnchorB = []
        
        self.botIDs = []
        self.clothIDs = []
        
        self.bot_position = []
        self.bot_rotation = []        
    
        self.cloth_rotation = []
        self.cloth_position = []
        
        self.xb={} # empty list of x positions
        self.yb={} # empty list of y positions
        self.zb={} # empty list of z positions
        
        self.vxb={} # empty list of x velocities
        self.vyb={} # empty list of y velocities
        self.vzb={} # empty list of z velocities
           
        # For square robots
        self.cube_dim = [self.bot_width,self.bot_width,self.bot_height/2]
        self.create_cloth()
        # fill list with empty array
        for i in range(self.nb):
            
            # positions
            self.xb["botx{0}".format(i)]=[]  #x position
            self.yb["boty{0}".format(i)]=[]  # y position
            self.zb["botz{0}".format(i)]=[]  # z position 
        
            # velocities
            self.vxb["botx{0}".format(i)]=[]  # x velocity
            self.vyb["boty{0}".format(i)]=[]  # y velocity
            self.vzb["botz{0}".format(i)]=[]  # z velocity


        for i in range(self.nb):
            x = np.cos(i*self.theta)*self.R
            y = np.sin(i*self.theta)*self.R
            z = self.bot_height/2
            rot = create_from_axis_angle(0,0,1,i*self.theta)
            pos = [x,y,z]
            T = np.array([[np.cos(self.theta), np.sin(self.theta)],[-np.sin(self.theta),np.cos(self.theta)]])
            self.AnchorA.append(np.matmul(np.linalg.inv(T),np.array([np.cos(self.theta)*x + np.sin(self.theta)*y, -self.bot_width])))
            self.AnchorB.append(np.matmul(np.linalg.inv(T),np.array([np.cos(self.theta)*x + np.sin(self.theta)*y, self.bot_width])))
            self.bot_position.append(pos) 
            self.bot_rotation.append(rot) 
        

        cloth_rot1 = create_from_axis_angle(-1,0,0,np.pi/2)
        for i in range(self.nb):
            theta_now = i*self.theta+self.start_theta
            cloth_rot2 = create_from_axis_angle(0,0,1,theta_now)
            cloth_rotation1 = multQ(cloth_rot1,cloth_rot2)
            cloth_rot3 = create_from_axis_angle(0,0,1,np.pi/2)
            cloth_rotation = multQ(cloth_rotation1,cloth_rot3)
            cloth_position = [np.cos(theta_now)*self.R, np.sin(theta_now)*self.R, self.cloth_height/2]
            
            self.cloth_rotation.append(cloth_rotation)
            self.cloth_position.append(cloth_position)


    def create_cloth(self):
        # Loading the cloth made by PyBullet
        path = data_dir + "cloth_z_up.obj"
        scene = pywavefront.Wavefront(path, collect_faces=True)
        vertices=scene.vertices
        faces=scene.mesh_list[0].faces
    
        # Creating our own cloth
        w = self.cloth_width
        h = self.cloth_height
        Vert_temp=np.array([[w/2,-h/4,0], # 0
                            [w/4,-h/2,0], # 1
                            [w/4,-h/4,0], # 2
                            [0,-h/4,0], # 3
                            [-w/4,-h/2,0], # 4
                            [-w/4,-h/4,0], # 5 
                            [0,h/4,0], # 6
                            [-w/4,0,0], # 7
                            [-w/4,h/4,0], # 8
                            [w/2,h/4,0], # 9
                            [w/4,0,0], # 10
                            [w/4,h/4,0], # 11
                            [0,0,0], # 12
                            [w/4,h/2,0], # 13
                            [0,h/2,0], # 14
                            [w/2,h/2,0], # 15
                            [-w/2,0,0], # 16
                            [-w/2,h/4,0], # 17
                            [-w/4,h/2,0], # 18 
                            [-w/2,h/2,0], # 19
                            [-w/2,-h/2,0], # 20
                            [-w/2,-h/4,0], # 21
                            [0,-h/2,0], # 22
                            [w/2,0,0], # 23
                            [w/2,-h/2,0]]) # 24

        V = []
        mesh = om.TriMesh()
        for i in range(len(vertices)):
            temp=Vert_temp[i,:]
            temp=temp.flatten()
            V.append(mesh.add_vertex([temp[0],temp[1],temp[2]]))
        
        vh_list = []  
        for i in range(len(faces)):
            temp = faces[i]
            mesh.add_face([V[temp[0]],V[temp[1]],V[temp[2]]])
            vh_list.append([V[temp[0]],V[temp[1]],V[temp[2]]])
        
        om.write_mesh(self.mainDirectory+self.name+'/membrane.obj', mesh)

class Particles:
    def __init__(self,name):    

        self.name=name
        self.mainDirectory = mainDirectory
        self.parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True).tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']

        self.nb=self.parameters['nb']
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.particle_geom = self.parameters['particle_geom']

        self.R = self.convert_dist*self.parameters['R']
        self.particle_mode = self.parameters['particle_mode']
        self.particle_mass = self.convert_mass*self.parameters['particle_mass']
        self.particle_width = self.convert_dist*self.parameters['particle_width']
        self.particle_height = self.convert_dist*self.parameters['particle_height']
        
        self.lateralFriction = self.parameters['lateralFriction'] 
        self.spinningFriction = self.parameters['spinningFriction'] 
        self.rollingFriction = self.parameters['rollingFriction'] 
        
        self.buffer = .1
        (self.N,self.Ri) = self.MaxValues()
        self.particleIDs = []
        self.particle_position = []
        self.Rm = []
        print(self.Ri)
        if self.particle_mode=='monodispersion':            
            for i in range(len(self.N)):
                for j in range(int(self.N[i])):
                    
                    x = self.Ri[i]*np.cos(2*j*np.pi/self.N[i])
                    y = self.Ri[i]*np.sin(2*j*np.pi/self.N[i])
                    z = self.particle_height/2     
                    pos = [x,y,z]
                    self.particle_position.append(pos) 
                    self.Rm.append(self.particle_width)          
                    

    def MaxValues(self):
        if self.particle_mode=='monodispersion':
            Rin=self.R-(self.bot_width/2)#-(self.particle_width/2)
            ngrans1=int(Rin/(2*self.particle_width/2))
            ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*self.particle_width/2)
            for i in range(ngrans1):
                remainder=((2*self.particle_width/2))*i
                ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((ri[:,i]*np.pi)/(self.particle_width/2)))
            n=np.asarray(ni,dtype=int)
            n=n[0]
                
        return(n,ri.flatten())    


# Other functions for converting angles to quaternions
def create_from_axis_angle(xx,yy,zz,angle):
    ## Here we calculate the sin(theta/2)
    factor=np.sin(angle/2)
    x = xx*factor
    y = yy*factor
    z = zz*factor
    w = np.cos(angle/2)
    quaternion = np.array([x,y,z,w])
    return quaternion/norm(quaternion)

def multQ(Q1,Q2):
    x0,y0,z0,w0 = Q1   # unpack
    x1,y1,z1,w1 = Q2   # unpack
    return([x1*w0 + y1*z0 - z1*y0 + w1*x0, 
            -x1*z0 + y1*w0 + z1*x0 + w1*y0, 
            x1*y0 - y1*x0 + z1*w0 +  w1*z0,
            -x1*x0 - y1*y0 - z1*z0 + w1*w0])