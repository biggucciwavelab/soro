# -*- coding: utf-8 -*-
"""
Created on Mon Sep 26 10:25:54 2022

@author: dmulr
"""

import warnings
warnings.filterwarnings("ignore")

import numpy as np
from numpy import savetxt
import random
import os
import csv
import glob
import timeit
import cv2
from os.path import exists
from csv import writer

#from IPython.display import HTML
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
import matplotlib.font_manager as fm
import matplotlib.patches as patches
from matplotlib import colors as colors
import matplotlib.cm as cm
from matplotlib.patches import RegularPolygon

from scipy.spatial import ConvexHull
from scipy.interpolate import RegularGridInterpolator
from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.optimize import minimize
from scipy.linalg import qr
from scipy.spatial import Delaunay
from scipy.ndimage import gaussian_filter1d

from sympy import Plane, Point3D
from sympy import *

from tabulate import tabulate
from shutil import copyfile


#fm._rebuild()
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] =8
plt.rcParams['axes.linewidth'] = .1
#plt.rcParams["text.usetex"] = True



class import_data:
    def __init__(self,name,path,wxmin,wxmax,wymin,wymax,savefile):
        self.name=name
        self.path=path
        self.mainDirectory = path   # main directory 
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        
        data=np.load(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',allow_pickle=True) 
        self.Rm=data['Rm'] 
        self.wxmin = wxmin
        self.wxmax = wxmax
        self.wymin = wymin
        self.wymax = wymax        
        self.savefile = savefile        
        self.m = 20
        
        self.err = 5
        self.parameters=parameters.tolist()           
        self.nb=self.parameters['nb'] # number of bots
        self.ni=self.parameters['total_particles']
        self.ns=self.parameters['ns']
        self.nm=self.nb*self.ns
        self.bot_width=self.parameters['bot_width']
        self.particle_width=self.parameters['particle_width']
        self.control_mode=self.parameters['control_mode']
        self.skin_width=self.parameters['skin_width']
        self.mu = self.parameters['lateralFriction']
        

        self.membrane_density = self.parameters['membrane_density']
        self.skin_width = self.parameters['skin_width']
        self.bot_height = self.parameters['bot_height']
        self.bot_mass = self.parameters['bot_mass']
        self.skin_mass = (self.skin_width/2)**2 * np.pi *0.85*self.bot_height
        self.ball_mass = self.parameters['ball_mass']
        self.particle_mass = self.parameters['particle_mass']
        
        self.path=self.mainDirectory+self.name+"/results/"
        os.chdir(self.path)
        self.files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
        if self.control_mode=="grasping":
            self.a2 = self.parameters['a2']
            self.b2 = self.parameters['b2']
            self.xc2 = self.parameters['xc2']     
            self.yc2 = self.parameters['yc2'] 
            
        # Robot Position
        self.bot_position=np.genfromtxt(self.files[self.files.index('bot_position.csv') ] ,delimiter=',')
        (self.m1,self.n1)=np.shape(self.bot_position)
        self.bot_position=self.bot_position[:,1:self.n1]
        self.time=self.bot_position[0,:]
        self.bot_position_x=self.bot_position[1:self.nb+1,:]
        self.bot_position_y=self.bot_position[self.nb+1:2*self.nb+1,:]
        self.bot_position_z=self.bot_position[(2*self.nb)+1:3*self.nb+1,:] 

        # Robot velocity
        self.bot_velocity=np.genfromtxt(self.files[self.files.index('bot_velocity.csv') ] ,delimiter=',')
        (self.m1,self.n1)=np.shape(self.bot_velocity)
        self.bot_velocity=self.bot_velocity[:,1:self.n1]
        self.time=self.bot_position[0,:]
        self.bot_velocity_x=self.bot_velocity[1:self.nb+1,:]
        self.bot_velocity_y=self.bot_velocity[self.nb+1:2*self.nb+1,:]
        self.bot_velocity_z=self.bot_velocity[(2*self.nb)+1:3*self.nb+1,:] 
        
        
        # Robot contact forces
        self.bot_contact_forces=np.genfromtxt(self.files[self.files.index('bot_contact_forces.csv') ] ,delimiter=',')
        (self.m1,self.n1)=np.shape(self.bot_contact_forces)
        self.bot_contact_forces=self.bot_contact_forces[:,1:self.n1]
        self.bot_contact_forces_x=self.bot_contact_forces[1:self.nb+1,:]
        self.bot_contact_forces_y=self.bot_contact_forces[self.nb+1:2*self.nb+1,:]
        self.bot_contact_forces_z=self.bot_contact_forces[(2*self.nb)+1:3*self.nb+1,:]          


        # Robot total forces
        self.bot_total_forces=np.genfromtxt(self.files[self.files.index('bot_total_forces.csv') ] ,delimiter=',')
        (self.m1,self.n1)=np.shape(self.bot_total_forces)
        self.bot_total_forces=self.bot_total_forces[:,1:self.n1]
        self.bot_total_forces_x=self.bot_total_forces[1:self.nb+1,:]
        self.bot_total_forces_y=self.bot_total_forces[self.nb+1:2*self.nb+1,:]
        self.bot_total_forces_z=self.bot_total_forces[(2*self.nb)+1:3*self.nb+1,:]          
        
        
        # Membrane_positions
        self.membrane_position=np.genfromtxt(self.files[self.files.index('membrane_position.csv') ] ,delimiter=',')
        (m,n)=np.shape(self.membrane_position)
        self.membrane_position=self.membrane_position[:,1:n]
        self.membrane_position_x=self.membrane_position[1:self.nm+1,:]
        self.membrane_position_y=self.membrane_position[self.nm+1:2*self.nm+1,:]
        self.membrane_position_z=self.membrane_position[(2*self.nm)+1:3*self.nm+1,:]  

        # membrane total forces
        self.membrane_total_forces=np.genfromtxt(self.files[self.files.index('membrane_total_forces.csv') ] ,delimiter=',')
        (m,n)=np.shape(self.membrane_total_forces)
        self.membrane_total_forces=self.membrane_total_forces[:,1:n]
        self.membrane_total_forces_x=self.membrane_total_forces[1:self.nm+1,:]
        self.membrane_total_forces_y=self.membrane_total_forces[self.nm+1:2*self.nm+1,:]
        self.membrane_total_forces_z=self.membrane_total_forces[(2*self.nm)+1:3*self.nm+1,:]  

        # membrane contact forces 
        self.membrane_contact_forces=np.genfromtxt(self.files[self.files.index('membrane_contact_forces.csv') ] ,delimiter=',')
        (m,n)=np.shape(self.membrane_contact_forces)
        self.membrane_contact_forces=self.membrane_contact_forces[:,1:n]
        self.membrane_contact_forces_x=self.membrane_contact_forces[1:self.nm+1,:]
        self.membrane_contact_forces_y=self.membrane_contact_forces[self.nm+1:2*self.nm+1,:]
        self.mmembrane_contact_forces_z=self.membrane_contact_forces[(2*self.nm)+1:3*self.nm+1,:]  
        
        
        self.control_forces=np.genfromtxt(self.files[self.files.index('control_forces.csv') ] ,delimiter=',')
        (m,n)=np.shape(self.control_forces)
        self.control_forces=self.control_forces[:,1:n]
        self.control_forces_x=self.control_forces[1:self.nb+1,:]
        self.control_forces_z=self.control_forces[self.nb+1:2*self.nb+1,:]

        (self.Fx_sum,self.Fz_sum)=self.sum_control_forces()

        if self.ni==0:
            pass
        else:
            # Particle Position
            self.particle_position=np.genfromtxt(self.files[self.files.index('particle_position.csv') ] ,delimiter=',')
            (self.m4a,self.n4a)=np.shape(self.particle_position)
            self.particle_position=self.particle_position[:,1:self.n4a]
            self.particle_position_x=self.particle_position[1:self.ni+1,:]
            self.particle_position_y=self.particle_position[self.ni+1:2*self.ni+1,:]
            self.particle_position_z=self.particle_position[(2*self.ni)+1:3*self.ni+1,:]


            
            # robot contact forces
            self.particle_contact_forces=np.genfromtxt(self.files[self.files.index('particle_contact_forces.csv') ] ,delimiter=',')
            (self.m1,self.n1)=np.shape(self.particle_contact_forces)
            self.particle_contact_forces=self.particle_contact_forces[:,1:self.n1]
            self.particle_contact_forces_x=self.particle_contact_forces[1:self.ni+1,:]
            self.particle_contact_forces_y=self.particle_contact_forces[self.ni+1:2*self.ni+1,:]
            self.particle_contact_forces_z=self.particle_contact_forces[(2*self.ni)+1:3*self.ni+1,:]          


            # robot total forces
            self.particle_total_forces=np.genfromtxt(self.files[self.files.index('particle_total_forces.csv') ] ,delimiter=',')
            (self.m1,self.n1)=np.shape(self.particle_total_forces)
            self.particle_total_forces=self.particle_total_forces[:,1:self.n1]
            self.particle_total_forces_x=self.particle_total_forces[1:self.ni+1,:]
            self.particle_total_forces_y=self.particle_total_forces[self.ni+1:2*self.ni+1,:]
            self.particle_total_forces_z=self.particle_total_forces[(2*self.ni)+1:3*self.ni+1,:] 
                     
            
        self.geom = self.parameters['ball_geometry'] 
        self.ball_radius = self.parameters['ball_radius']
        self.ball_position=np.genfromtxt(self.files[self.files.index('ball_position.csv') ] ,delimiter=',')            
        (self.m5a,self.n5a)=np.shape(self.ball_position)
        self.ballx_position=self.ball_position[1,:]
        self.ballz_position=self.ball_position[2,:]

        # ball velocity
        self.ball_velocity=np.genfromtxt(self.files[self.files.index('ball_velocity.csv') ] ,delimiter=',')
        (self.m1,self.n1)=np.shape(self.bot_velocity)
        self.ball_velocity_x=self.ball_velocity[1,:]
        self.ball_velocity_z=self.ball_velocity[2,:]         
        
        
        # Ball contact forces
        self.ball_contact_forces=np.genfromtxt(self.files[self.files.index('ball_contact_forces.csv') ] ,delimiter=',')
        (m,n)=np.shape(self.ball_contact_forces)
        self.ball_contact_forces=self.ball_contact_forces[:,1:n]
        self.bFx=self.ball_contact_forces[1,:]
        self.bFy=self.ball_contact_forces[2,:]
        self.bFz=self.ball_contact_forces[3,:]   
        
        
        # Ball total forces
        self.ball_total_forces=np.genfromtxt(self.files[self.files.index('ball_total_forces.csv') ] ,delimiter=',')
        (m,n)=np.shape(self.ball_total_forces)
        self.ball_total_forces=self.ball_total_forces[:,1:n]
        self.bFTx=self.ball_total_forces[1,:]
        self.bFTy=self.ball_total_forces[2,:]
        self.bFTz=self.ball_total_forces[3,:]           
        
        # Pull test information
        self.pull_data=np.genfromtxt(self.files[self.files.index('pull_force.csv') ] ,delimiter=',')            
        (self.m6a,self.n6a)=np.shape(self.pull_data)
        self.pull_data=self.pull_data[:,1:self.n6a]
        self.TIME=self.pull_data[0,:]
        self.PX=self.pull_data[1,:]
        self.PZ=self.pull_data[2,:]
        self.FB=self.pull_data[3,:]  
        
        
        
        # Contact Points and forces 
        self.time_contact = np.genfromtxt(self.files[self.files.index('time_contact.csv') ] ,delimiter=',')
        self.number_contacts = np.genfromtxt(self.files[self.files.index('number_contacts.csv') ] ,delimiter=',')
   
        self.x_contact_force=np.genfromtxt(self.files[self.files.index('x_contact_force.csv') ] ,delimiter=',') 
        self.y_contact_force=np.genfromtxt(self.files[self.files.index('y_contact_force.csv') ] ,delimiter=',') 
        self.z_contact_force=np.genfromtxt(self.files[self.files.index('z_contact_force.csv') ] ,delimiter=',') 
        
        self.x_contact_force2=np.genfromtxt(self.files[self.files.index('x_contact_force2.csv') ] ,delimiter=',') 
        self.y_contact_force2=np.genfromtxt(self.files[self.files.index('y_contact_force2.csv') ] ,delimiter=',') 
        self.z_contact_force2=np.genfromtxt(self.files[self.files.index('z_contact_force2.csv') ] ,delimiter=',') 
        
        self.x_contact_points=np.genfromtxt(self.files[self.files.index('x_contact_points.csv') ] ,delimiter=',') 
        self.y_contact_points=np.genfromtxt(self.files[self.files.index('y_contact_points.csv') ] ,delimiter=',') 
        self.z_contact_points=np.genfromtxt(self.files[self.files.index('z_contact_points.csv') ] ,delimiter=',')          
                 
        self.Dirxx_=np.genfromtxt(self.files[self.files.index('contact_dirxx.csv') ] ,delimiter=',') 
        self.Dirxy_=np.genfromtxt(self.files[self.files.index('contact_dirxy.csv') ] ,delimiter=',') 
        self.Dirxz_=np.genfromtxt(self.files[self.files.index('contact_dirxz.csv') ] ,delimiter=',') 

        self.Diryx_=np.genfromtxt(self.files[self.files.index('contact_diryx.csv') ] ,delimiter=',') 
        self.Diryy_=np.genfromtxt(self.files[self.files.index('contact_diryy.csv') ] ,delimiter=',') 
        self.Diryz_=np.genfromtxt(self.files[self.files.index('contact_diryz.csv') ] ,delimiter=',') 

        self.Dirzx_=np.genfromtxt(self.files[self.files.index('contact_dirzx.csv') ] ,delimiter=',') 
        self.Dirzy_=np.genfromtxt(self.files[self.files.index('contact_dirzy.csv') ] ,delimiter=',') 
        self.Dirzz_=np.genfromtxt(self.files[self.files.index('contact_dirzz.csv') ] ,delimiter=',')       
        
        
        #self.AN = np.genfromtxt(self.files[self.files.index('AN.csv') ] ,delimiter=',') 
        #self.BN = np.genfromtxt(self.files[self.files.index('BN.csv') ] ,delimiter=',') 
        self.AID = np.genfromtxt(self.files[self.files.index('AID.csv') ] ,delimiter=',') 
        self.BID = np.genfromtxt(self.files[self.files.index('BID.csv') ] ,delimiter=',') 
         
        self.AN=[] # empty array of contact ID A
        self.BN=[] # empty array of contact ID B
        infile = open(self.files[self.files.index('AN.csv') ], 'r') # now we fill the array AN
        for row in csv.reader(infile):
            self.AN.append(row[1:])
        infile = open(self.files[self.files.index('BN.csv') ], 'r') # now we fill the array BN
        for row in csv.reader(infile):
            self.BN.append(row[1:])                 
        
    
        self.Forces_x_contact_particles={}
        self.Forces_z_contact_particles={}
        
        self.Forces_x_contact_bots={}
        self.Forces_z_contact_bots={}    
        
        
        self.Force_x_contact_ball={}
        self.Force_z_contact_ball={}
        
        
        self.position_x_contact_ball={}
        self.position_z_contact_ball={}
        
        self.dir_xx_contact_ball={}
        self.dir_xz_contact_ball={}
        
        self.dir_zx_contact_ball={}
        self.dir_zz_contact_ball={}        
        
        self.position_x_contact_bot={}
        self.position_z_contact_bot={}
        
        self.Forces_x_ball_bot={}
        self.Forces_z_ball_bot={}
        
        self.position_x_ball_bot={}
        self.position_z_ball_bot={}
        
        
        
        for i in range(len(self.time_contact)):
            
            # positions
            self.Forces_x_contact_particles["time_contact{0}".format(i)]=[]  #x position
            self.Forces_z_contact_particles["time_contact{0}".format(i)]=[]  #x position
                 
            self.Forces_x_contact_bots["time_contact{0}".format(i)]=[]  #x position
            self.Forces_z_contact_bots["time_contact{0}".format(i)]=[]  #x position

            self.Force_x_contact_ball["time_contact{0}".format(i)]=[]
            self.Force_z_contact_ball["time_contact{0}".format(i)]=[]
            
            self.position_x_contact_ball["time_contact{0}".format(i)]=[]
            self.position_z_contact_ball["time_contact{0}".format(i)]=[]
            
            self.Forces_x_ball_bot["time_contact{0}".format(i)]=[]
            self.Forces_z_ball_bot["time_contact{0}".format(i)]=[]
            
            self.position_x_ball_bot["time_contact{0}".format(i)]=[]
            self.position_z_ball_bot["time_contact{0}".format(i)]=[]
            
            self.position_x_contact_bot["time_contact{0}".format(i)]=[]
            self.position_z_contact_bot["time_contact{0}".format(i)]=[]

            self.dir_xx_contact_ball["time_contact{0}".format(i)]=[]
            self.dir_xz_contact_ball["time_contact{0}".format(i)]=[]
        
            self.dir_zx_contact_ball["time_contact{0}".format(i)]=[]
            self.dir_zz_contact_ball["time_contact{0}".format(i)]=[]            
            
            
        parameters=np.load(self.mainDirectory+self.name+'/graspParams.npy',allow_pickle=True)
        self.parameters=parameters.tolist()


        self.F_control = self.parameters["F_control"] 
        self.Forces_ball_x = self.parameters["Forces_ball_x"]   
        self.Forces_ball_z = self.parameters["Forces_ball_z"] 

        self.contact_points_ball_x = self.parameters["contact_points_ball_x"]  
        self.contact_points_ball_z = self.parameters["contact_points_ball_z"] 

        self.magnitude_forces_on_ball = self.parameters["magnitude_forces_on_ball"] 
        self.torque_ball = self.parameters["torque_ball"] 


        self.Pressure_x_bots = self.parameters["Pressure_x_bots"]
        self.Pressure_z_bots = self.parameters["Pressure_z_bots"]

        self.Pressure_x_particles = self.parameters["Pressure_x_particles"]
        self.Pressure_z_particles = self.parameters["Pressure_z_particles"]

        
        self.MAG_pressure = self.parameters["MAG_pressure"]
        self.MAG_pressure_no_boundary = self.parameters["MAG_pressure_no_boundary"]
        
        self.avg_MAG_pressure = []
        self.avg_MAG_pressure_no_boundary = []
        
        (self.avg_MAG_pressure,self.avg_MAG_pressure_no_boundary)=self.avg_pressure()
        
        self.grasp_id = self.parameters["grasp_id"]

        self.grasp_position_x = self.parameters["grasp_position_x"]
        self.grasp_position_z = self.parameters["grasp_position_z"]

        self.grasp_force_x = self.parameters["grasp_force_x"]
        self.grasp_force_z = self.parameters["grasp_force_z"]
        self.grasp_torque = self.parameters["grasp_torque"]


        #self.WRENCHES = self.parameters["WRENCHES"]
        #self.WRENCH_NORM = self.parameters["WRENCH_NORM"]

        #self.FRAMES = self.parameters["FRAMES"] 

        #self.EPSILON = self.parameters["EPSILON"] 
        #self.HULLWRENCHNORM = self.parameters["HULLWRENCHNORM"] 
        #self.HULLWRENCHMAGS = self.parameters["HULLWRENCHMAGS"] 

        self.framex  = self.parameters["framex"] 
        self.framez  = self.parameters["framez"] 

        self.FT  = self.parameters["FT"] 

        self.Cplus  = self.parameters["Cplus"] 
        self.Cminus  = self.parameters["Cminus"] 

        self.FCplus  = self.parameters["FCplus"]   
        self.FCinus  = self.parameters["FCinus"] 

        self.F_mag  = self.parameters["F_mag"] 
        self.HULL = self.parameters["HULL"] 

        self.WRENCHXY = self.parameters["WRENCHXY"] 
        self.HULLXY = self.parameters["HULLXY"] 

        self.WRENCHXT = self.parameters["WRENCHXT"] 
        self.HULLXT = self.parameters["HULLXT"]

        self.WRENCHYT = self.parameters["WRENCHYT"]
        self.HULLYT = self.parameters["HULLYT"]         
        
        
    def find_contact_forces_2(self):
        '''This function for extacting contact forces without regards for grasping'''
        for i in range(len(self.time_contact)-1):
            #print(i)
            tempx = {}
            tempz = {}
             
            tempx2 = {}
            tempz2 = {} 
            
            tempx21 = {}
            tempz21 = {} 
                        
            tempx3 = {}
            tempz3 = {}
            
            
            tempx4 = {}
            tempz4 = {}
            
            tempx5 = {}
            tempz5 = {}
            
            tempx6 = {}
            tempz6 = {}
                        
            
            tempxx = {}
            tempzz = {}
            
            tempxxx = {}
            tempzzz = {}            
            
            tempx3["ballx"] = []
            tempz3["ballz"] = []
            
            tempx4["ballx"] = []
            tempz4["ballz"] = []

            tempx5["ballx"] = []
            tempz5["ballz"] = []
            
            tempx6["ballx"] = []
            tempz6["ballz"] = []
            

            
            for ii in range(self.ni):
                tempx["ni{0}".format(ii)]=[]  #x position
                tempz["ni{0}".format(ii)]=[]  #x position    
                
            for jj in range(self.nb):    
                tempx2["nb{0}".format(jj)]=[]  #x position
                tempz2["nb{0}".format(jj)]=[]  #x position
                
            for jj in range(self.nb):    
                tempx21["nb{0}".format(jj)]=[]  #x position
                tempz21["nb{0}".format(jj)]=[]  #x position
                
            for jj in range(self.nb):    
                tempxx["nb{0}".format(jj)]=[]  #x position
                tempzz["nb{0}".format(jj)]=[]  #x position  
                
                tempxxx["nb{0}".format(jj)]=[]  #x position
                tempzzz["nb{0}".format(jj)]=[]  #x position      
                
            for j in range(int(self.number_contacts[i])):
                temp1=self.AN[i][j]
                temp2=self.BN[i][j]

                ##### contact forces for passive particles
                if temp1[0:4]=="gran" or temp2[0:4]=="gran":
                    if temp2[0:4]=="gran":
                        les=len(temp2)
                        num=int(temp2[5:les])
                        tempx["ni"+str(num)].append(self.x_contact_force2[j,i])
                        tempz["ni"+str(num)].append(self.z_contact_force2[j,i])
                        
                     
                    if temp1[0:4]=="gran":   
                        les=len(temp1)
                        num=int(temp1[5:les])
                        tempx["ni"+str(num)].append(self.x_contact_force2[j,i])
                        tempz["ni"+str(num)].append(self.z_contact_force2[j,i])    
                         
                            
                ##### contact forces for boundary bots          
                if temp1[0:3]=="bot" or temp2[0:3]=="bot":
                    if temp2[0:3]=="bot":  
                        les=len(temp2)
                        num=int(temp2[3:les])
                        tempx2["nb"+str(num)].append(self.x_contact_force2[j,i])
                        tempz2["nb"+str(num)].append(self.z_contact_force2[j,i])
                        
                        tempx21["nb"+str(num)].append(self.x_contact_points[j,i])
                        tempz21["nb"+str(num)].append(self.z_contact_points[j,i])      
                        
                    if temp1[0:3]=="bot":
                        les=len(temp1)
                        num=int(temp1[3:les])
                        tempx2["nb"+str(num)].append(self.x_contact_force2[j,i])
                        tempz2["nb"+str(num)].append(self.z_contact_force2[j,i]) 
                        
                        tempx21["nb"+str(num)].append(self.x_contact_points[j,i])
                        tempz21["nb"+str(num)].append(self.z_contact_points[j,i])    
                        
                        
                        
                ##### contact forces for ball        
                if temp1[0:4]=="ball" or temp2[0:4]=="ball" :   
                    if temp1[0:4]=="ball":  
                        tempx3["ballx"].append(self.x_contact_force2[j,i])
                        tempz3["ballz"].append(self.z_contact_force2[j,i])
                        
                        tempx4["ballx"].append(self.x_contact_points[j,i])
                        tempz4["ballz"].append(self.z_contact_points[j,i])  
                        
                        tempx5["ballx"].append(self.Dirxx_[j,i])
                        tempz5["ballz"].append(self.Dirxz_[j,i]) 
                        
                        tempx6["ballx"].append(self.Dirzx_[j,i])
                        tempz6["ballz"].append(self.Dirzz_[j,i])
                        
                    if temp2[0:4]=="ball":
                        tempx3["ballx"].append(self.x_contact_force2[j,i])
                        tempz3["ballz"].append(self.z_contact_force2[j,i])
                        
                        tempx4["ballx"].append(self.x_contact_points[j,i])
                        tempz4["ballz"].append(self.z_contact_points[j,i])
                        
                        tempx5["ballx"].append(self.Dirxx_[j,i])
                        tempz5["ballz"].append(self.Dirxz_[j,i]) 
                        
                        tempx6["ballx"].append(self.Dirzx_[j,i])
                        tempz6["ballz"].append(self.Dirzz_[j,i])                        
                    
                    
                        
                ###### contact forces for ball and boundary robots          
                if (temp1[0:4]=="ball" and temp2[0:3]=="bot") or (temp2[0:4]=="ball" or temp1[0:3]=="bot"):
                    if temp1[0:4]=="ball" and temp2[0:3]=="bot" :
                        #les=len(temp2)
                        num=int(temp2[3:les])
                        tempxx["nb"+str(num)].append(self.x_contact_force2[j,i])
                        tempzz["nb"+str(num)].append(self.z_contact_force2[j,i])

                        tempxxx["nb"+str(num)].append(self.x_contact_points[j,i])
                        tempzzz["nb"+str(num)].append(self.z_contact_points[j,i])

                    if temp2[0:4]=="ball" and temp1[0:3]=="bot" :
                        #les=len(temp2)
                        num=int(temp1[3:les])
                        tempxx["nb"+str(num)].append(self.x_contact_force2[j,i])
                        tempzz["nb"+str(num)].append(self.z_contact_force2[j,i])

                        tempxxx["nb"+str(num)].append(self.x_contact_points[j,i])
                        tempzzz["nb"+str(num)].append(self.z_contact_points[j,i])

                
                        
            self.Forces_x_contact_particles["time_contact"+str(i)].append(tempx) # contact forces for passive particles x
            self.Forces_z_contact_particles["time_contact"+str(i)].append(tempz) # contact forces for passive particles z
             
            self.Forces_x_ball_bot["time_contact"+str(i)].append(tempxx) # contact forces for ball and boundary robots x
            self.Forces_z_ball_bot["time_contact"+str(i)].append(tempzz) # contact forces for ball and boundary robots  z 
            
            self.position_x_ball_bot["time_contact"+str(i)].append(tempxxx)  # contact position for ball and boundary robots x
            self.position_z_ball_bot["time_contact"+str(i)].append(tempzzz)  # contact position for ball and boundary robots  z 
            
            self.Forces_x_contact_bots["time_contact"+str(i)].append(tempx2) # contact forces for boundary bots x
            self.Forces_z_contact_bots["time_contact"+str(i)].append(tempz2) # contact forces for boundary bots z         
            
            self.position_x_contact_bot["time_contact"+str(i)].append(tempx21) # contact position for boundary bots x
            self.position_z_contact_bot["time_contact"+str(i)].append(tempz21) # contact position for boundary bots z
        
            self.Force_x_contact_ball["time_contact"+str(i)].append(tempx3) # contact forces for ball x
            self.Force_z_contact_ball["time_contact"+str(i)].append(tempz3) # contact forces for ball z   
            
            self.position_x_contact_ball["time_contact"+str(i)].append(tempx4) # contact position for ball x
            self.position_z_contact_ball["time_contact"+str(i)].append(tempz4) # contact position for ball z               
        

            self.dir_xx_contact_ball["time_contact"+str(i)].append(tempx5) # contact position for ball x
            self.dir_xz_contact_ball["time_contact"+str(i)].append(tempz5) # contact position for ball x
        
            self.dir_zx_contact_ball["time_contact"+str(i)].append(tempx6) # contact position for ball x
            self.dir_zz_contact_ball["time_contact"+str(i)].append(tempz6) # contact position for ball x
                                     
    def sum_control_forces(self):       
        FX=[]
        FY=[]
        for i in range(len(self.time)):
            fxtemp=[]
            fytemp=[]
            for j in range(0,self.nb):
                Fx,Fy=self.control_forces_x[j,i],self.control_forces_z[j,i]  
                fxtemp.append(Fx)
                fytemp.append(Fy)
            FX.append(np.sum(fxtemp))
            FY.append(np.sum(fytemp))

        return(FX,FY)
            
            
    def avg_pressure(self):
        avg_MAG_pressure=[]
        avg_MAG_pressure_no_boundary=[]
        for i in range(len(self.time)-1):
            temp=np.sum(self.MAG_pressure[:,i])
            temp=temp/len(self.MAG_pressure[:,i])
            avg_MAG_pressure.append(temp)
              
            temp2=np.sum(self.MAG_pressure_no_boundary[:,i])
            temp2=temp/len(self.MAG_pressure_no_boundary[:,i])                
            avg_MAG_pressure_no_boundary.append(temp2)
            
        return(avg_MAG_pressure,avg_MAG_pressure_no_boundary)   
    

    
    def create_snap_shot(self,entry,name,d):
        ''' Create snapshots with no forces  '''
        i=entry
        #ratio =(self.wymax-self.wymin)/(self.wxmax-self.wxmin)
        x0,y0=self.ballx_position[i],self.ballz_position[i]
        membrane=True
        wxmax=x0+d
        wxmin=x0-d
        wymax=y0+d
        wymin=y0-d
        const=(wxmax-wxmin)/(wymax-wymin)
        fig = plt.figure(dpi=300)
        fig.set_size_inches(const*2,2)
        ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
        const=self.ball_radius*2
        rx=const
        ry=const
        w=rx/2
        h=ry/2          
        xcenter=self.ballx_position[i]
        ycenter=self.ballz_position[i]
        x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
        y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
        (self.segments)=self.create_segment(x,y) 
        
        
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            if self.PHI(x0,y0,self.segments)<.1:
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:orange')
                ax.add_patch(patch)
            else:
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                ax.add_patch(patch) 

        if membrane==True:
            for j in range(0,self.nm):

                x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                ax.add_patch(patch)
                            
        
        if self.geom=="circle":
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = plt.Circle((x0, y0),self.ball_radius, fc='tab:grey')
            ax.add_patch(patch)
            
        if self.geom=="square":
            const_=self.ball_radius*2
            x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2
            
            patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='tab:grey',edgecolor='tab:grey')     
            ax.add_patch(patch)
            
        if self.geom=="triangle":
            const_=self.ball_radius*2*np.pi/3
            r=const_*np.sqrt(3)/3
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = RegularPolygon((x0,y0),3,r,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
            ax.add_patch(patch)             

                            
        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if np.round(self.Rm[j],4)==0.0508:
                c='tab:blue'
            else:
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
            ax.add_patch(patch)     
        #plt.title(name+'  Time = ' + str(np.round(self.time[i],1))+" s",fontsize=8)
        plt.axis('off')        
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.jpg',dpi=300)
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.svg',dpi=300)
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.pdf',dpi=300)          
        #
        

    def create_snap_shot_forces(self,entry,name,d):
        ''' Create snapshots showing forces  '''
        i=entry
        #ratio =(self.wymax-self.wymin)/(self.wxmax-self.wxmin)
        x0,y0=self.ballx_position[i],self.ballz_position[i]
        membrane=True
        wxmax=x0+d
        wxmin=x0-d
        wymax=y0+d
        wymin=y0-d
        const=(wxmax-wxmin)/(wymax-wymin)
        fig = plt.figure(dpi=300)
        fig.set_size_inches(const*2,2)
        ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
        const=self.ball_radius*2
        rx=const
        ry=const
        w=rx/2
        h=ry/2          
        xcenter=self.ballx_position[i]
        ycenter=self.ballz_position[i]
        x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
        y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]

        (self.segments)=self.create_segment(x,y) 
        
        
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            if self.PHI(x0,y0,self.segments)<.1:
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:orange')
                ax.add_patch(patch)
            else:
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                ax.add_patch(patch) 

        if membrane==True:
            for j in range(0,self.nm):

                x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                ax.add_patch(patch)
                            
        
        if self.geom=="circle":
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = plt.Circle((x0, y0),self.ball_radius, fc='tab:grey')
            ax.add_patch(patch)
            
        if self.geom=="square":
            const_=self.ball_radius*2
            x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2
            
            patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='tab:grey',edgecolor='tab:grey')     
            ax.add_patch(patch)
            
        if self.geom=="triangle":
            const_=self.ball_radius*2*np.pi/3
            r=const_*np.sqrt(3)/3
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = RegularPolygon((x0,y0),3,r,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
            ax.add_patch(patch)             

                            
        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if np.round(self.Rm[j],4)==0.0508:
                c='tab:blue'
            else:
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
            ax.add_patch(patch)     
            
        if self.grasp_id[i]!=[]:

            xo=self.grasp_position_x[i]
            zo=self.grasp_position_z[i]

            Fx=self.grasp_force_x[i]
            Fz=self.grasp_force_z[i]
            #print("Fx:",np.round(Fx,2))
            #print("Fz:",np.round(Fz,2))
            vx=self.framex[i]
            vz=self.framez[i]
            cp=self.Cplus[i]
            cm=self.Cminus[i]   
            plt.quiver(xo,zo, vx[:,0], vx[:,1], color="tab:blue", scale=10,width=0.007,label='controller dir')  
            plt.quiver(xo,zo, vz[:,0], vz[:,1], color="tab:green", scale=10,width=0.007,label='controller dir') 
            plt.quiver(xo,zo, cp[:,0], cp[:,1], color="tab:red", scale=10,width=0.007)   
            plt.quiver(xo,zo, cm[:,0], cm[:,1], color="tab:red", scale=10,width=0.007)              
        plt.axis('off')        
       
  

    def Forcechains(self,entry,name):
        """ create plot force chains"""
        i=entry
        cmap = plt.cm.get_cmap('seismic')
        boundaries=np.arange(1,100,1)
                       
        norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
        count=0
        #for i in range(1,len(self.time)-1):
        Fx=self.x_contact_force[0:int(self.number_contacts[i]),i]
        Fz=self.z_contact_force[0:int(self.number_contacts[i]),i]

        #Fy=Fcy[0:nc[i],i]
        abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)


        x=self.x_contact_points[0:int(self.number_contacts[i]),i]
        y=self.z_contact_points[0:int(self.number_contacts[i]),i]
        x2=[]
        y2=[]
        F2=[]
        for j in range(len(abs_force)):
            x2.append(x[j])
            y2.append(y[j])
            F2.append(abs_force[j])

        plt.figure(figsize=(2,2),dpi=300)
        plt.scatter(x2,y2,s=2*np.power(F2,.65),c=F2,cmap=cmap,norm=norm)
        plt.grid(True)
        plt.colorbar()
        plt.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.jpg',dpi=300)
        plt.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.svg',dpi=300)
        plt.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.pdf',dpi=300)          
        
        
        
    def Forcechains_arrows(self,d,entry,name):
        """ create plot force chains"""
        i=entry
        x0,y0=self.ballx_position[i],self.ballz_position[i]
        membrane=True
        wxmax=x0+d
        wxmin=x0-d
        wymax=y0+d
        wymin=y0-d
        const=(wxmax-wxmin)/(wymax-wymin)
        fig = plt.figure(dpi=300)
        fig.set_size_inches(const*2,2)
        ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
        const=self.ball_radius*2
        rx=const
        ry=const
        w=rx/2
        h=ry/2          
        xcenter=self.ballx_position[i]
        ycenter=self.ballz_position[i]
        x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
        y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
        (self.segments)=self.create_segment(x,y) 
        
        
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            if self.PHI(x0,y0,self.segments)<.1:
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='none',edgecolor='tab:orange')
                ax.add_patch(patch)
            else:
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='none',edgecolor="k")
                ax.add_patch(patch) 

        if membrane==True:
            for j in range(0,self.nm):

                x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.skin_width/2, fc='none',edgecolor='tab:red')
                ax.add_patch(patch)
                            
        
        if self.geom=="circle":
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = plt.Circle((x0, y0),self.ball_radius, fc='tab:grey')
            ax.add_patch(patch)
            
        if self.geom=="square":
            const_=self.ball_radius*2
            x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2
            
            patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='tab:grey')     
            ax.add_patch(patch)
            
        if self.geom=="triangle":
            const_=self.ball_radius*2*np.pi/3
            r=const_*np.sqrt(3)/3
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = RegularPolygon((x0,y0),3,r,orientation=-np.pi/2,fc='none',edgecolor='tab:grey')
            ax.add_patch(patch)             

                            
        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if np.round(self.Rm[j],4)==0.0508:
                c='tab:blue'
            else:
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j], fc="none",edgecolor=c)
            ax.add_patch(patch)          
        
        
        
        
        
        Dirxx=self.Dirxx_[0:int(self.number_contacts[i]),i]
        Dirxz=self.Dirxz_[0:int(self.number_contacts[i]),i]

 
        
        Dirzx=self.Dirzx_[0:int(self.number_contacts[i]),i]
        Dirzz=self.Dirzz_[0:int(self.number_contacts[i]),i]         

        Fx=self.x_contact_force2[0:int(self.number_contacts[i]),i]
        Fz=self.z_contact_force2[0:int(self.number_contacts[i]),i]

        abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)


        x=self.x_contact_points[0:int(self.number_contacts[i]),i]
        y=self.z_contact_points[0:int(self.number_contacts[i]),i]

    
        x0,y0=self.ballx_position[i],self.ballz_position[i]

        for j in range(len(abs_force)):
            mag=np.sqrt(Fx[j]**2 + Fz[j]**2)
            ax.quiver(x[j],y[j],Fx[j]/mag,Fz[j]/mag,color="purple",scale=10,width=0.005,zorder=1)
            mag2=np.sqrt(Dirxx[j]**2+Dirxz[j]**2)
            mag3=np.sqrt(Dirzx[j]**2+Dirzz[j]**2)
            
            ax.quiver(x[j],y[j],Dirxx[j]/mag2,Dirxz[j]/mag2,color="red",scale=20,width=0.005,zorder=2)
            ax.quiver(x[j],y[j],Dirzx[j]/mag3,Dirzz[j]/mag3,color="blue",scale=20,width=0.005,zorder=2)
            
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.jpg',dpi=300)
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.svg',dpi=300)
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.pdf',dpi=300)          
        
        
    def create__frames_robot_forces(self,entry,d,name):
        ''' Create frames for for total forces '''
        i=entry
 
        x0,y0=self.ballx_position[i],self.ballz_position[i]

        wxmax=x0+d
        wxmin=x0-d
        wymax=y0+d
        wymin=y0-d
        const=(wxmax-wxmin)/(wymax-wymin)
        fig = plt.figure(dpi=300)
        fig.set_size_inches(const*2,2)
        ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
        # BOUNDARY ROBOTS
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            patch = plt.Circle((x0, y0),self.bot_width/2, fc='none',edgecolor='k',lw=1)
            ax.add_patch(patch)

        if self.geom=="circle":
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = plt.Circle((x0, y0),self.ball_radius,fc='tab:gray',edgecolor='black',linewidth=1)
            ax.add_patch(patch)

        if self.geom=="square":
            const_=self.ball_radius*2
            x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2

            patch = patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='tab:grey')     
            ax.add_patch(patch)

            patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='blue',linewidth=1)    
            ax.add_patch(patch)
            
        if self.geom=="triangle":
            const_=self.ball_radius*2*np.pi/3
            r=const_*np.sqrt(3)/3
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            patch = RegularPolygon((x0,y0),3,r,orientation=-np.pi/2,fc='tab:gray',edgecolor='tab:grey')
            ax.add_patch(patch)  

        if self.grasp_id[i]!=[]:

            xo=self.grasp_position_x[i]
            zo=self.grasp_position_z[i]

            Fx=self.grasp_force_x[i]
            Fz=self.grasp_force_z[i]

            vx=self.framex[i]
            vz=self.framez[i]
            cp=self.Cplus[i]
            cm=self.Cminus[i]   

            mag1 = np.sqrt(self.bot_contact_forces_x[self.grasp_id[i],i]**2 + self.bot_contact_forces_z[self.grasp_id[i],i]**2)
            mag2 = np.sqrt(self.bot_total_forces_x[self.grasp_id[i],i]**2 + self.bot_total_forces_z[self.grasp_id[i],i]**2)
            mag3=np.sqrt(self.control_forces_x[self.grasp_id[i],i]**2 + self.control_forces_z[self.grasp_id[i],i]**2)
            
            
            ax.quiver(xo,zo, self.bot_contact_forces_x[self.grasp_id[i],i]/mag1,self.bot_contact_forces_z[self.grasp_id[i],i]/mag1, color="tab:orange",label='contact force', scale=10,width=0.007)            
            ax.quiver(xo,zo, self.bot_total_forces_x[self.grasp_id[i],i]/mag2,self.bot_total_forces_z[self.grasp_id[i],i]/mag2, color="k",label='total force', scale=10,width=0.007,zorder=1)  
            ax.quiver(xo,zo, self.control_forces_x[self.grasp_id[i],i]/mag3,self.control_forces_z[self.grasp_id[i],i]/mag3, color="tab:purple",label="control force", scale=10,width=0.005,zorder=1)
            ax.quiver(xo,zo, vx[:,0], vx[:,1], color="tab:blue", scale=10,width=0.007,label=' tangent dir')  
            ax.quiver(xo,zo, vz[:,0], vz[:,1], color="tab:green", scale=10,width=0.007,label='normal dir') 
            ax.quiver(xo,zo, cp[:,0], cp[:,1], color="tab:red", scale=10,width=0.007,label="Friction cone")   
            ax.quiver(xo,zo, cm[:,0], cm[:,1], color="tab:red", scale=10,width=0.007)
            
            fig.legend(bbox_to_anchor=(1.0, 1), loc='upper left', borderaxespad=0,frameon=False)
        for j in range(len(self.grasp_id[i])):
            plt.text(xo[j],zo[j],str(self.grasp_id[i][j]))
        fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
       #plt.savefig(direct+"/frame%04d.jpg" % count)
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.jpg',dpi=300)
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.svg',dpi=300)
        fig.savefig(self.savefile+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+'.pdf',dpi=300)  
        #count=count+1 
            
            #plt.close('all')            
        
        
        
    def extract_forces_frame(self,entry):  
        i=entry

        Fbxc=self.bot_contact_forces_x[:,i]
        Fbzc=self.bot_contact_forces_z[:,i]
        
        Fbxt=self.bot_total_forces_x[:,i]
        Fbzt=self.bot_total_forces_z[:,i]
        
        Fbxco=self.control_forces_x[:,i]
        Fbzco=self.control_forces_z[:,i]
        
        Fpxt=self.particle_total_forces_x[:,i]
        Fpzt=self.particle_total_forces_z[:,i]
        
        Fpxc=self.particle_contact_forces_x[:,i]
        Fpzc=self.particle_contact_forces_z[:,i]
        
        
        return(Fbxc,Fbzc,Fbxt,Fbzt,Fpxc,Fpzc,Fpxt,Fpzt,Fbxco,Fbzco)



        
        
        
        
        
        
    def F(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)

    def delfx(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)

    def delfy(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)

    def T(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc=np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)

    def deltx(self,x,y,x1,x2,y1,y2):
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def delty(self,x,y,x1,x2,y1,y2):
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))


    def phi(self,x,y,x1,x2,y1,y2):
        t=self.T(x,y,x1,x2,y1,y2)
        f=self.F(x,y,x1,x2,y1,y2)
        rho=np.sqrt(t**2 +f**4)
        return(np.sqrt(f**2+((rho-t)/2)**2))

    def delphix(self,x,y,x1,x2,y1,y2):
        ff=self.F(x,y,x1,x2,y1,y2)
        dfx=self.delfx(x,y,x1,x2,y1,y2)
        tf=self.T(x,y,x1,x2,y1,y2)
        dtx=self.deltx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 =  np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def delphiy(self,x,y,x1,x2,y1,y2):
        ff=self.F(x,y,x1,x2,y1,y2)
        dfy=self.delfy(x,y,x1,x2,y1,y2)
        tf=self.T(x,y,x1,x2,y1,y2)
        dty=self.delty(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 =  np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)  

    def PHI(self,x,y,segments):
        #m=4
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) ** self.m
        R = 1/R**(1/self.m)
        return(R)

    def PHIDX(self,x,y,segments):
        #m=4
        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) *self.delphix(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)

    def PHIDY(self,x,y,segments):
        #m=4
        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) * self.delphiy(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)
    
    def create_segment(self,x,y):
        seglen=len(x)
        segments=np.zeros((seglen-1,4))
        for i in range(seglen-1):
            #[x1,y1,x2,y2]
            #[x2,y2,x3,y3]
            segments[i,0]=x[i]
            segments[i,1]=y[i]
            segments[i,2]=x[i+1]
            segments[i,3]=y[i+1]
        return(segments)                    
        
    def shoelace(self,vertices):
        (m,n)=np.shape(vertices)

        sum1=vertices[0,0]*(vertices[1,1]-vertices[1,n-1])
        for i in range(1,n-1):
            sum1=sum1 + vertices[0,i]*(vertices[1,i+1]-vertices[1,i-1])
        i=n-1
        sum1=sum1 + vertices[0,i]*(vertices[1,0]-vertices[1,i-1])

        A=.5*abs(sum1)
        return (A)              
        

def trial_ftn(x,c1):
    return c1*x

def calculate_deriviative(FB,PX):
    dydx=diff(PX)/diff(FB)
    return(dydx)

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth
# In[CODE]

plt.close('all')




experiment_="experiment_12"
#mainDirectory = "C:/Users/dmulr/github_reps/Grasping/Simulation_analysis/"
mainDirectory =os.path.abspath(os.getcwd())
savefile = mainDirectory +'/'+experiment_
os.makedirs(savefile, exist_ok=True)
#os.path.abspath(os.getcwd())
#path="F:/Soro_chrono/python/Pychrono/Strings/String_grasping/Experiments/"
path=os.path.abspath(os.getcwd())
path=path+"/Experiments/"
d=2.5
dxmin=-d
dxmax=3.5
dymin=-d
dymax=d

path = os.path.dirname(__file__)
path=path+"/Experiments/"
# name1= "26_09_2022_14_05_44" # entry=90 # experiment 1
# name1= "26_09_2022_16_18_11" # entry=190 # experiment 2
# name1= "26_09_2022_19_42_46" # entry=190 # experiment 3
# name1= "26_09_2022_20_45_12" # experiment 4
# name1= "26_09_2022_21_15_57" # experiment 5
# name1= "27_09_2022_08_51_33" # experiment 6
# name1= "27_09_2022_10_08_36" # experiment 7
# name1= "27_09_2022_10_32_21" # experiment 8



## IMPORTANT ##
#name1= "27_09_2022_12_36_38" # experiment 9 # LOW COMPLINACe
#name1="28_09_2022_07_48_16" # experiment 10 #MORE COMPLIANCE
name1="28_09_2022_08_34_33" # experiment 11 # HIGH DAMPING
name1="28_09_2022_08_35_52" # experiment 12 # NORMAL dAMMPING more compliance but extended longer 25 seconds 

entry=240
sim_data1=import_data(name1,path,dxmin,dxmax,dymin,dymax,savefile)
sim_data1.find_contact_forces_2()



sim_data1.create__frames_robot_forces(entry,0.5,"robot_forces")

sim_data1.Forcechains(entry,"force chains")

sim_data1.Forcechains_arrows(0.5,entry,"force chains arrows")

sim_data1.create_snap_shot(entry,"snap shot no forces",0.5)


i=entry
ID=sim_data1.grasp_id[entry]  # id of robots in contact for entry    
print(ID)

Fbxc=sim_data1.bot_contact_forces_x[:,i] # contact forces in the x direction of bots for entry
Fbzc=sim_data1.bot_contact_forces_z[:,i] # contact forces in the z direction of bots for entry
FBXC=Fbxc[ID] # contact force x for specific ID
FBZC=Fbzc[ID] # contact force z for specific ID


Fbxt=sim_data1.bot_total_forces_x[:,i] # contact forces in the x direction of bots for entry
Fbzt=sim_data1.bot_total_forces_z[:,i] # contact forces in the z direction of bots for entry
FBXT=Fbxt[ID] # total force x for specific ID
FBZT=Fbzt[ID] # total force z for specific ID


Fbxco=sim_data1.control_forces_x[:,i] # control forces in the x direction of bots for entry
Fbzco=sim_data1.control_forces_z[:,i] # control forces in the z direction of bots for entry

FBXCO=Fbxco[ID]
FBZCO=Fbzco[ID]

Forces_x_ball_bot=sim_data1.Forces_x_ball_bot # forces on the ball and bots
Forces_z_ball_bot=sim_data1.Forces_z_ball_bot # forces on the ball and bots


bot_velocity_x = sim_data1.bot_velocity_x[:,i] # bot velocity x
bot_velocity_y = sim_data1.bot_velocity_y[:,i] # bot velocity y
bot_velocity_z = sim_data1.bot_velocity_z[:,i] # bot velocity z


bot_velocity_x_ = bot_velocity_x[ID] # bot velocity x for the bots in contact
bot_velocity_y_ = bot_velocity_y[ID] # bot velocity y for the bots in contact
bot_velocity_z_ = bot_velocity_z[ID] # bot velocity z for the bots in contact


FXCB = sim_data1.Forces_x_contact_bots # these contacts were manually computed for bots x
FZCB = sim_data1.Forces_z_contact_bots # these contacts were manually computed for bots z

FXCBentry=FXCB["time_contact"+str(entry)] # extract entry for the contact forces manually calculated for entry for x direction
FZCBentry=FZCB["time_contact"+str(entry)] # extract entry for the contact forces manually calculated for entry for z direction

position_x_contact_bot = sim_data1.position_x_contact_bot # these contact points correspond to the ones manually computed 
position_z_contact_bot = sim_data1.position_z_contact_bot # these contact points correspond to the ones manually computed

position_x_contact_bot_ = position_x_contact_bot["time_contact"+str(entry)] # extracting the contact points for specific entry
position_z_contact_bot_ = position_z_contact_bot["time_contact"+str(entry)] # extracting the contact points for specific entry

time = sim_data1.time # extract time
bFTx = sim_data1.bFTx # total forces on ball x these were from chrono
bFTy = sim_data1.bFTy # total forces on ball y these were from chrono
bFTz = sim_data1.bFTz # total forces on ball z these were from chrono

bFcx = sim_data1.bFx # contact forces on ball x these were from chrono
bFcy = sim_data1.bFy # contact forces on ball x these were from chrono
bFcz = sim_data1.bFz # contact forces on ball x these were from chrono
        
ball_velocity_x = sim_data1.ball_velocity_x # ball velocity in the x direction
ball_velocity_z = sim_data1.ball_velocity_z # ball velocity in the z direction  

FXCBentry_=[] # EMPTY array of the bots in contact Force z
FZCBentry_=[] # empty array of the bots in contact Force z

PXCBentry_=[] # EMPTY array of the bots positon in contact x
PZCBentry_=[] # EMPTY array of the bots positon in contact z
for i in range(len(ID)):
    #print(ID[i])
    #print(FXCB91)
    tempx=FXCBentry[0]["nb"+str(ID[i])]
    tempz=FZCBentry[0]["nb"+str(ID[i])]
    
    tempx2=position_x_contact_bot_[0]["nb"+str(ID[i])]
    tempz2=position_z_contact_bot_[0]["nb"+str(ID[i])]
    
    tempxx=[]
    tempzz=[]
    
    tempxx2=[]
    tempzz2=[]    
    for j in range(len(tempx)):
        #if abs(tempx[j])<100:
        tempxx.append(tempx[j])
        tempxx2.append(tempx2[j])
        tempzz2.append(tempz2[j])
        #if abs(tempz[j])<100:                        
        tempzz.append(tempz[j])        

    FXCBentry_.append(tempxx)    
    FZCBentry_.append(tempzz) 
    
    PXCBentry_.append(tempxx2) 
    PZCBentry_.append(tempzz2)   

    
    
    
# These were calculated by me manually. 
Force_x_contact_ball=sim_data1.Force_x_contact_ball # THIS IS THE CONTACT FORCES WHICH WERE IN CONTACT WITH THE BALL X
Force_z_contact_ball=sim_data1.Force_z_contact_ball # THIS IS THE CONTACT FORCES WHICH WERE IN CONTACT WITH THE BALL Z


F_contact_ballx_entry=Force_x_contact_ball["time_contact"+str(entry)] # EXTRACT THE ENTRY
F_contact_ballz_entry=Force_z_contact_ball["time_contact"+str(entry)] # EXTRACT THE ENTRY


F_contact_ballx_entry=F_contact_ballx_entry[0]['ballx'] #  FURTHER EXTRACTION 
F_contact_ballz_entry=F_contact_ballz_entry[0]['ballz'] #  FURTHER EXTRACTION 

sum_Fcontactx=[]
sum_Fcontactz=[]

for i in range(len(time)-2):
    print(i)
    F_contact_tempx=Force_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
    F_contact_tempz=Force_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
    
    F_contact_tempx=F_contact_tempx[0]['ballx'] #  FURTHER EXTRACTION 
    F_contact_tempz=F_contact_tempz[0]['ballz'] #  FURTHER EXTRACTION 
    
    
    sum_Fcontactx.append(np.sum(F_contact_tempx))
    sum_Fcontactz.append(np.sum(F_contact_tempz))  
    
    

position_x_contact_ball=sim_data1.position_x_contact_ball # THIS IS THE CONTACT POSITION WHICH WERE IN CONTACT WITH THE BALL X
position_z_contact_ball=sim_data1.position_z_contact_ball # THIS IS THE CONTACT POSITION WHICH WERE IN CONTACT WITH THE BALL Z


Position_x_contact_entry=position_x_contact_ball["time_contact"+str(entry)] # EXTRACT THE ENTRY
Position_z_contact_entry=position_z_contact_ball["time_contact"+str(entry)] # EXTRACT THE ENTRY


Position_x_contact_entry=Position_x_contact_entry[0]['ballx'] #  FURTHER EXTRACTION 
Position_z_contact_entry=Position_z_contact_entry[0]['ballz'] #  FURTHER EXTRACTION 


dir_xx_contact_ball=sim_data1.dir_xx_contact_ball
dir_xz_contact_ball=sim_data1.dir_xz_contact_ball
dir_zx_contact_ball=sim_data1.dir_zx_contact_ball
dir_zz_contact_ball=sim_data1.dir_zz_contact_ball


dir_xx_contact_ball_entry=dir_xx_contact_ball["time_contact"+str(entry)]
dir_xz_contact_ball_entry=dir_xz_contact_ball["time_contact"+str(entry)]
dir_zx_contact_ball_entry=dir_zx_contact_ball["time_contact"+str(entry)]
dir_zz_contact_ball_entry=dir_zz_contact_ball["time_contact"+str(entry)]


dir_xx_contact_ball_entry=dir_xx_contact_ball_entry[0]['ballx']
dir_xz_contact_ball_entry=dir_xz_contact_ball_entry[0]['ballz']
dir_zx_contact_ball_entry=dir_zx_contact_ball_entry[0]['ballx']
dir_zz_contact_ball_entry=dir_zz_contact_ball_entry[0]['ballz']


# In[Table 1]


table1 = [['nb',"Fxcontact","Fxcontrol","Fxtotal","Fzcontact","Fzcontrol","Fztotal","xvel","zvel","Fmag"]]
for i in range(len(FBXCO)):
    table1.append([str(ID[i]),
    str(np.round(FBXC[i],2)),
    str(np.round(FBXCO[i],2)),
    str(np.round(FBXT[i],2)),   
    str(np.round(FBZC[i],2)),
    str(np.round(FBZCO[i],2)),
    str(np.round(FBZT[i],2)),
    str(np.round(bot_velocity_x_[i],2)),
    str(np.round(bot_velocity_z_[i],2)),
    str(np.round(np.sqrt(FBXCO[i]**2 + FBZCO[i]**2),2))])
   
    
table1.append(["sum",str(np.round(np.sum(FBXC),2)),
                str(np.round(np.sum(FBXCO),2)),
                str(np.round(np.sum(FBXT),2)),
                    str(np.round(np.sum(FBZC),2)),
                    str(np.round(np.sum(FBZCO),2)),
                    str(np.round(np.sum(FBZT),2)),0,0,0])    
    
    
print(tabulate(table1, headers='firstrow'))
np.savetxt(savefile+'/table1.csv', table1, delimiter=",", fmt='%s')
with open(savefile+'/table1.txt', 'w') as f:
    f.write(tabulate(table1))        
        
    
# In[Table 2]

    
table2 = [['nb',"Fxcontact_dir","Fxcontrol_dir","Fxtotal_dir","Fzcontact_dir","Fzcontrol_dir","Fztotal_dir"]]        
for i in range(len(FBXCO)):
    mag1=np.sqrt(FBXC[i]**2 + FBZC[i]**2)
    mag2=np.sqrt(FBXCO[i]**2 + FBZCO[i]**2)  
    mag3=np.sqrt(FBXT[i]**2 + FBZT[i]**2) 
    table2.append([str(ID[i]),             
    str(np.round(FBXC[i]/mag1,2)),
    str(np.round(FBXCO[i]/mag2,2)),
    str(np.round(FBXT[i]/mag3,2)), 

    str(np.round(FBZC[i]/mag1,2)),
    str(np.round(FBZCO[i]/mag2,2)),
    str(np.round(FBZT[i]/mag3,2))])

print(tabulate(table2, headers='firstrow'))

np.savetxt(savefile+'/table2.csv', table2, delimiter=",", fmt='%s')
with open(savefile+'/table2.txt', 'w') as f:
    f.write(tabulate(table2))    

# In[Table 3]
table3= [['nb',"Fxcontrol","Fzcontrol"]]
control_forces_x=sim_data1.control_forces_x[:,entry]
control_forces_z=sim_data1.control_forces_z[:,entry]
for i in range(len(control_forces_x)):
    table3.append([str(i),
                  np.round(control_forces_x[i],2),
                  np.round(control_forces_z[i],2)])


table3.append(["sum=",np.round(np.sum(control_forces_x),2),np.round(np.sum(control_forces_z),2)])


print(tabulate(table3, headers='firstrow'))
np.savetxt(savefile+'/table3.csv', table3, delimiter=",", fmt='%s')
with open(savefile+'/table3.txt', 'w') as f:
    f.write(tabulate(table3))               
               
               
            
# In[sum of control forces]
SUMFX=[]
SUMFZ=[]
control_forces_x_=sim_data1.control_forces_x
control_forces_z_=sim_data1.control_forces_z
for i in range(len(time)):
    SUMFX.append(np.round(np.sum(control_forces_x_[:,i]),2))
    SUMFZ.append(np.round(np.sum(control_forces_z_[:,i]),2))
    
    
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3),dpi=300)

axs.plot(time,SUMFX,color="tab:red",linewidth=1,label='x')
axs.plot(time,SUMFZ,color="tab:blue",linewidth=1,label='z')
axs.set_xlabel('time (s)',fontsize=8,labelpad=1)
axs.set_ylabel('forces(N)',labelpad=1)
axs.set_title("Sum of control forces")
axs.xaxis.set_tick_params(width=.25,length=2)
axs.yaxis.set_tick_params(width=.25,length=2)
axs.grid(True,linewidth=0.25)   
fig.legend( loc='upper right', borderaxespad=0,frameon=False)
#plt.tight_layout()
fig.savefig(savefile+'/'+"sum controll forces"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"sum controll forces"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"sum controll forces"+'_.pdf',dpi=300)



# In[Sum of contact forces on the ball]
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3),dpi=300)
y1 =smooth(sum_Fcontactx, 30)
y2=smooth(sum_Fcontactz, 30)
axs.plot(time[0:-2],sum_Fcontactx,color="tab:red",linewidth=1,label='x')
axs.plot(time[0:-2],y1,color="k",linestyle='-',linewidth=1)
axs.plot(time[0:-2],sum_Fcontactz,color="tab:blue",linewidth=1,label='z')
axs.plot(time[0:-2],y2,color="k",linestyle='--',linewidth=1)
axs.set_xlabel('time (s)',fontsize=8,labelpad=1)
axs.set_ylabel('forces(N)',labelpad=1)
axs.set_title("Sum of contact forces on the ball")
axs.xaxis.set_tick_params(width=.25,length=2)
axs.yaxis.set_tick_params(width=.25,length=2)
axs.grid(True,linewidth=0.25)   
fig.legend( loc='upper right', borderaxespad=0,frameon=False)
#plt.tight_layout()
fig.savefig(savefile+'/'+"sum contact forces"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"sum contact forces"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"sum controll forces"+'_.pdf',dpi=300)


#sum_Fcontactx.append(np.sum(F_contact_tempx))
#sum_Fcontactz.append(np.sum(F_contact_tempz))  
    

# In[Velocities of bots]   
    
# for i in range(len(ID)):
#     y1_velx =smooth(sim_data1.bot_velocity_x[ID[i],:], 20)
#     y2_velz =smooth(sim_data1.bot_velocity_z[ID[i],:], 20)
        
        
#     #y1_velx = gaussian_filter1d(sim_data1.bot_velocity_x[ID[i],:], 30)
#     #y2_velz = gaussian_filter1d(sim_data1.bot_velocity_z[ID[i],:], 30)
#     fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(1.5,1.5),dpi=300)
#     plt.title("Bot"+str(ID[i]),fontsize=8)
#     axs.plot(sim_data1.time,sim_data1.bot_velocity_x[ID[i],:],color="tab:blue",linewidth=1,label='vx')
#     axs.plot(sim_data1.time,sim_data1.bot_velocity_z[ID[i],:],color="tab:red",linewidth=1,label='vz')
#     axs.plot(sim_data1.time,y1_velx,color="k",linewidth=2,label='best fit x')
#     axs.plot(sim_data1.time,y2_velz,color="k",linestyle='--',linewidth=2,label='best fit z')
#     axs.set_xlabel('time (s)',fontsize=8,labelpad=1)
#     axs.set_ylabel('velocity (m/s)',labelpad=1)
#     axs.xaxis.set_tick_params(width=.25,length=2)
#     axs.yaxis.set_tick_params(width=.25,length=2)
#     axs.set_ylim([-0.1,0.1])
#     axs.grid(True,linewidth=0.25)
#     #fig.legend(bbox_to_anchor=(1.0, 1), loc='upper left', borderaxespad=0,frameon=False)
#     fig.legend(loc='upper right', borderaxespad=0,frameon=False)
#     fig.savefig(savefile+'/'+"velocity_bot"+str(ID[i])+'_.jpg',dpi=300)
#     fig.savefig(savefile+'/'+"velocity_bot"+str(ID[i])+'_.svg',dpi=300)
#     fig.savefig(savefile+'/'+"velocity_bot"+str(ID[i])+'_.pdf',dpi=300)
#     #plt.tight_layout()
    
    
    
    
    
print("Total Ball Force x: ",np.round(bFTx[entry],2))
print("Total Ball Force y: ",np.round(bFTy[entry],2))
print("Total Ball Force z: ",np.round(bFTz[entry],2))

print("Contact Ball Force x: ",np.round(bFcx[entry],2))
print("Contact Ball Force y: ",np.round(bFcy[entry],2))
print("Contact Ball Force z: ",np.round(bFcz[entry],2))

print("Ball velocity x: ",np.round(ball_velocity_x[entry],3))
print("Ball velocity z: ",np.round(ball_velocity_z[entry],3))








# In[Table 4]

# table4 = [['entry',
#             "Total Ball Force x",
#             "Total Ball Force z",
#             "Contact Ball Force x",
#             "Contact Ball Force z",
#             "Ball velocity x",
#             "Ball velocity z",
#             "Ball friction cone (N)"]]

# table4.append([str(entry),
# str(np.round(bFTx[entry],2)),
# str(np.round(bFTz[entry],2)),  
               
# str(np.round(bFcx[entry],2)),
# str(np.round(bFcz[entry],2)),
               
# str(np.round(ball_velocity_x[entry],2)),
# str(np.round(ball_velocity_z[entry],2)),
# str(np.round(3*9.81*.2,2))])
# #print(tabulate(table3, headers='firstrow'))
# with open(savefile+'/table4.txt', 'w') as f:
#     f.write(tabulate(table3))

# np.savetxt(savefile+'/table4.csv', table4, delimiter=",", fmt='%s')






# In[Total ball forces vs time]


fig, axs = plt.subplots(nrows=2, ncols=1,figsize=(3,3),dpi=300)
axs[0].plot(time,bFTx,color="tab:red",linewidth=1,label='x')
axs[0].set_xlabel('time (s)',fontsize=8,labelpad=1)
axs[0].set_ylabel('Ball force total x (N)',labelpad=1)
axs[0].xaxis.set_tick_params(width=.25,length=2)
axs[0].yaxis.set_tick_params(width=.25,length=2)
axs[0].grid(True,linewidth=0.25)

axs[1].plot(time,bFTz,color="tab:blue",linewidth=1,label='z')
axs[1].set_xlabel('time (s)',fontsize=8,labelpad=1)
axs[1].set_ylabel('Ball force total z (N)',labelpad=1)
axs[1].xaxis.set_tick_params(width=.25,length=2)
axs[1].yaxis.set_tick_params(width=.25,length=2)
axs[1].grid(True,linewidth=0.25)
fig.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0,frameon=False)
plt.tight_layout()
fig.savefig(savefile+'/'+"Total force ball"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"Total force ball"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"Total force ball"+'_.pdf',dpi=300)


# In[Total contact ball forces vs time]
y1_bFcx = gaussian_filter1d(bFcx, 30)
y1_bFcx = smooth(bFcx, 30)
y2_bFcz = gaussian_filter1d(bFcz, 30)
y2_bFcz = smooth(bFcz, 30)

fig, axs = plt.subplots(nrows=2, ncols=1,figsize=(3,3),dpi=300)
axs[0].plot(time,bFcx,color="tab:red",linewidth=1,label='x')
axs[0].plot(time,y1_bFcx,color="k",linewidth=2,label='best fit x')
axs[0].set_xlabel('time (s)',fontsize=8,labelpad=1)
axs[0].set_ylabel('Ball contact force x (N)',labelpad=1)
axs[0].xaxis.set_tick_params(width=.25,length=2)
axs[0].yaxis.set_tick_params(width=.25,length=2)
axs[0].grid(True,linewidth=0.25)

axs[1].plot(time,bFcz,color="tab:blue",linewidth=1,label='z')
axs[1].plot(time,y2_bFcz,color="k",linestyle='--',linewidth=2,label='best fit z')
axs[1].set_xlabel('time (s)',fontsize=8,labelpad=1)
axs[1].set_ylabel('Ball contact force z (N)',labelpad=1)
axs[1].xaxis.set_tick_params(width=.25,length=2)
axs[1].yaxis.set_tick_params(width=.25,length=2)
axs[1].grid(True,linewidth=0.25)
fig.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0,frameon=False)
#plt.tight_layout()
fig.savefig(savefile+'/'+"Contact force ball"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"Contact force ball"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"Contact force ball"+'_.pdf',dpi=300)



# In[Superimposed forces]
i=entry
#ratio =(self.wymax-self.wymin)/(self.wxmax-self.wxmin)
x0,y0=sim_data1.ballx_position[i],sim_data1.ballz_position[i]
membrane=True
d=0.5
wxmax=x0+d
wxmin=x0-d
wymax=y0+d
wymin=y0-d
const=(wxmax-wxmin)/(wymax-wymin)
fig = plt.figure(dpi=300)
fig.set_size_inches(const*2,2)
ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
const=sim_data1.ball_radius*2
rx=const
ry=const
w=rx/2
h=ry/2
xcenter=sim_data1.ballx_position[i]
ycenter=sim_data1.ballz_position[i]
x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]

(segments)=sim_data1.create_segment(x,y)

PX=[]
PZ=[]
FX=[]
FZ=[]
for j in range(len(FXCBentry_)):
    px=[]
    pz=[]
    fx=[]
    fz=[]
    for k in range(len(FXCBentry_[j])):
        mag=np.sqrt(FXCBentry_[j][k]**2 + FZCBentry_[j][k]**2)
        fx.append(FXCBentry_[j][k]/mag)
        fz.append(FZCBentry_[j][k]/mag)
        px.append(PXCBentry_[j][k])
        pz.append(PZCBentry_[j][k])
    ax.quiver(px,pz,fx,fz,color="k",label="contact forces", scale=7,width=0.01,zorder=2)
#plt.title(name+'  Time = ' + str(np.round(self.time[i],1))+" s",fontsize=8)


for j in range(0,sim_data1.nb):
    x0,y0=sim_data1.bot_position_x[j,i],sim_data1.bot_position_z[j,i]
    if sim_data1.PHI(x0,y0,segments)<.1:
        patch = plt.Circle((x0, y0),sim_data1.bot_width/2, fc='none',edgecolor='tab:orange')
        ax.add_patch(patch)
    else:
        patch = plt.Circle((x0, y0),sim_data1.bot_width/2, fc='none',edgecolor='k')
        ax.add_patch(patch)


for jj in range(len(ID)):
    mag=np.sqrt(sim_data1.control_forces_x[ID[jj],i]**2 + sim_data1.control_forces_z[ID[jj],i]**2)

    ax.quiver(sim_data1.bot_position_x[ID[jj],i],sim_data1.bot_position_z[ID[jj],i], sim_data1.control_forces_x[ID[jj],i]/mag,sim_data1.control_forces_z[ID[jj],i]/mag,
              color="purple",label="control forces", scale=7,width=0.01)

#fig.legend(bbox_to_anchor=(1.0, 1), loc='upper left', borderaxespad=0,frameon=False)

if membrane==True:
    for j in range(0,sim_data1.nm):
        x0,y0=sim_data1.membrane_position_x[j,i],sim_data1.membrane_position_z[j,i]
        patch = plt.Circle((x0, y0),sim_data1.skin_width/2,fc='none',edgecolor='tab:red')
        ax.add_patch(patch)


if sim_data1.geom=="circle":
    x0,y0=sim_data1.ballx_position[i],sim_data1.ballz_position[i]
    patch = plt.Circle((x0, y0),sim_data1.ball_radius, fc='tab:grey')
    ax.add_patch(patch)

if sim_data1.geom=="square":
    const_=sim_data1.ball_radius*2
    x0,y0=sim_data1.ballx_position[i]-const_/2,sim_data1.ballz_position[i] - const_/2

    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='tab:grey',zorder=-1)
    ax.add_patch(patch)
    patch = plt.Circle((sim_data1.xc2, sim_data1.yc2),sim_data1.a2,fc='none',edgecolor='blue',linewidth=1)
    ax.add_patch(patch)

if sim_data1.geom=="triangle":
    const_=sim_data1.ball_radius*2*np.pi/3
    r=const_*np.sqrt(3)/3
    x0,y0=sim_data1.ballx_position[i],sim_data1.ballz_position[i]
    patch = RegularPolygon((x0,y0),3,r,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
    ax.add_patch(patch)


for j in range(sim_data1.ni):
    x0,y0=sim_data1.particle_position_x[j,i],sim_data1.particle_position_z[j,i]
    if np.round(sim_data1.Rm[j],4)==0.0508:
        c='tab:blue'
    else:
        c='tab:green'
    patch = plt.Circle((x0, y0),sim_data1.Rm[j], fc='none',edgecolor=c,zorder=-1)
    ax.add_patch(patch)


for j in range(len(Position_x_contact_entry)):
    x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
    mag1 = np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
    ax.quiver(x0,y0, F_contact_ballx_entry[j]/mag1,F_contact_ballz_entry[j]/mag1,scale=7, color="orange",width=.007,zorder=2)
plt.title('  Time = ' + str(np.round(sim_data1.time[i],1))+" s",fontsize=8)
#plt.axis('off')
fig.savefig(savefile+'/'+"all_plots_superimposed"+'_'+'time'+str(np.round(time[entry],2))+'.jpg',dpi=300)
fig.savefig(savefile+'/'+"all_plots_superimposed"+'_'+'time'+str(np.round(time[entry],2))+'.svg',dpi=300)
fig.savefig(savefile+'/'+"all_plots_superimposed"+'_'+'time'+str(np.round(time[entry],2))+'.pdf',dpi=300)



# In[Ball positions]
xcenter=sim_data1.ballx_position
ycenter=sim_data1.ballz_position
fig, axs = plt.subplots(nrows=2, ncols=1,figsize=(3,2.5),dpi=300)
axs[0].plot(time,xcenter[0:-1],color="tab:red",linewidth=1,label='x')
axs[0].set_ylabel('position x (m)',labelpad=1)
axs[0].yaxis.set_tick_params(width=.25,length=2)
axs[0].grid(True,linewidth=0.25)

axs[1].plot(time,ycenter[0:-1],color="tab:blue",linewidth=1,label='z')
axs[1].set_xlabel('time (s)',fontsize=8,labelpad=1)
axs[1].set_ylabel('position z (cm)',labelpad=1)
axs[1].xaxis.set_tick_params(width=.25,length=2)
axs[1].yaxis.set_tick_params(width=.25,length=2)
axs[1].grid(True,linewidth=0.25)

fig.legend(loc='upper right', borderaxespad=0,frameon=False)
fig.suptitle('ball position') 
#plt.tight_layout()
fig.savefig(savefile+'/'+"position ball"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"position ball"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"position ball"+'_.pdf',dpi=300)






# In[Ball velocities]
y1_velx =smooth(ball_velocity_x[0:-1], 20)
y2_velz =smooth(ball_velocity_z[0:-1], 20)
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,1),dpi=300)

axs.plot(time,ball_velocity_x[0:-1],color="tab:blue",linewidth=1,label='vx')
axs.plot(time,y1_velx,color="k",linewidth=1,linestyle='-')
axs.plot(time,ball_velocity_z[0:-1],color="tab:red",linewidth=1,label='vz')
axs.plot(time,y2_velz,color="k",linewidth=1,linestyle='--')
axs.set_xlabel('time (s)',fontsize=8,labelpad=1)
axs.set_ylabel('velocity (m/s)',labelpad=1)
axs.set_title('Ball velocity')
axs.xaxis.set_tick_params(width=.25,length=2)
axs.yaxis.set_tick_params(width=.25,length=2)
axs.grid(True,linewidth=0.25)
#axs.set_ylim([-0.05,0.05])
fig.legend( loc='upper right', borderaxespad=0,frameon=False)
#plt.tight_layout()
fig.savefig(savefile+'/'+"velocity ball"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"velocity ball"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"velocity ball"+'_.pdf',dpi=300)






def angle(x, y):
    
    rad = np.arctan2(y, x)
    #degrees = np.int(rad*180/np.pi)
    #if rad < 0:
        #rad = 2*np.pi - rad
    return rad






# In[PAll contact forces on cube]

i=entry
d=0.31
x0b,y0b=sim_data1.ballx_position[entry],sim_data1.ballz_position[entry]
const=sim_data1.ball_radius*2-.01
rx=const
ry=const
w=rx/2
h=ry/2                    
#x__=[w+x0b,-w+x0b,-w+x0b,w+x0b,w+x0b]
#y__=[h+y0b,h+y0b,-h+y0b,-h+y0b,h+y0b]
x__=[w,-w,-w,w,w]
y__=[h,h,-h,-h,h]
(segments)=sim_data1.create_segment(x__,y__) 


wxmax=d
wxmin=-d
wymin=-d
wymax=d
const=(wxmax-wxmin)/(wymax-wymin)
fig = plt.figure(dpi=300)
fig.set_size_inches(const*3,3)
ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
const_=sim_data1.ball_radius*2
xb,yb=0-const_/2,0 - const_/2
x0_=xb
y0_=yb
patch = matplotlib.patches.Rectangle((x0_, y0_),const_, const_,fc='none',edgecolor='tab:grey')     
ax.add_patch(patch)

X=[]
Y=[]
theta=[]

temp_position_x = []
temp_position_z = []
temp_force_x = []
temp_force_z = []  
temp_vx = []
temp_vy = []
temp_c1 = []
temp_c2 = []
temp_id = []
temp_theta = []
temp_frames=[]
temp_offset_theta=[]



frames = np.zeros((len(Position_x_contact_entry),3))
Vx=np.zeros((len(Position_x_contact_entry),2))
Vy=np.zeros((len(Position_x_contact_entry),2))
C1=np.zeros((2,len(Position_x_contact_entry))) # positive cone
C2=np.zeros((2,len(Position_x_contact_entry))) # negative cone 
for j in range(len(Position_x_contact_entry)):
    #fig = plt.figure(dpi=300)
    #fig.set_size_inches(const*3,3)
    #ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
    #patch = matplotlib.patches.Rectangle((x0_, y0_),const_, const_,fc='none',edgecolor='tab:grey')     
    #ax.add_patch(patch)
    x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
    ax.text(x0-x0b,y0-y0b,str(j),size=8)
    Fx1=sim_data1.PHIDX(x0-x0b,y0-y0b,segments)
    Fy1=sim_data1.PHIDY(x0-x0b,y0-y0b,segments)
    mag=np.sqrt(Fx1**2 + Fy1**2)
    Fx1=-Fx1/mag
    Fy1=-Fy1/mag
    F_t=np.array([Fx1,Fy1])
    X.append(x0-x0b)
    Y.append(y0-y0b)
    theta1=np.arctan2(.2,1) #+ frames[j,2]
    t=angle(Fx1, Fy1)-np.pi/2
    theta.append(t)
    
    frames[j,0]=x0-x0b
    frames[j,1]=y0-y0b
    frames[j,2]=t
    
    T=np.array([[np.cos(t),-np.sin(t)],[np.sin(t),np.cos(t)]]) # transformation matrix   
    VYpp=T@np.array([[0],[1]]) # transform coordinates X
    VXpp=T@np.array([[1],[0]]) # transform coordinates Y
    VXpp=VXpp.flatten() # flatten the matrix
    VYpp=VYpp.flatten() # flatten the matrix

    Vx[j,:]=VXpp # Save the array X
    Vy[j,:]=VYpp  # save the array Y     
    
    T1=np.array([[np.cos(theta1),-np.sin(theta1)],[np.sin(theta1),np.cos(theta1)]]) # transformation matrix   
    T2=np.array([[np.cos(-theta1),-np.sin(-theta1)],[np.sin(-theta1),np.cos(-theta1)]])    
    tem=Vy[j,:].T
    C1[:,j]=T1@tem
    C2[:,j]=T2@tem


    
    mag1 = np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
    mag2=np.sqrt(dir_xx_contact_ball_entry[j]**2 + dir_xz_contact_ball_entry[j]**2)
    mag3=np.sqrt(dir_zx_contact_ball_entry[j]**2 + dir_zz_contact_ball_entry[j]**2) 
    temp_dirr=[dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2]

    
    ax.quiver(x0-x0b,y0-y0b, dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2,scale=20,color="k",width=.007,zorder=3)    
    ax.quiver(x0-x0b,y0-y0b, F_contact_ballx_entry[j]/mag1,F_contact_ballz_entry[j]/mag1,scale=10, color="tab:orange",width=.007,zorder=5)      
    ax.quiver(x0-x0b,y0-y0b, Fx1,Fy1,scale=10, color="green",width=.007,zorder=1)
    ax.quiver(x0-x0b,y0-y0b,C1[0,j],C1[1,j] ,color="magenta", scale=10,label='Positive cone')     
    ax.quiver(x0-x0b,y0-y0b,C2[0,j],C2[1,j] ,color="magenta", scale=10,label='negative cone')
    ax.quiver(x0-x0b,y0-y0b, VXpp[0],VXpp[1],scale=15,color="b",width=.007,zorder=2)  
    ax.quiver(x0-x0b,y0-y0b, VYpp[0],VYpp[1],scale=15,color="r",width=.007,zorder=2)  

    #print("j",str(j),np.round(t,2),"Fx:",np.round(Fx1,2),"Fy:",np.round(Fy1,2),"Fxf:",np.round(F_contact_ballx_91[j]/mag1,2),"Fyf:",np.round(F_contact_ballz_91[j]/mag1,2))
    
    theta1=np.arctan2(.2,1) #+ frames[j,2]
    temp=np.round(np.nan_to_num(np.arccos(np.dot(VYpp ,temp_dirr))),2)
    fx=F_contact_ballx_entry[j]
    fy=F_contact_ballz_entry[j]
    
    mag_=np.sqrt(fx**2 +fy**2)
    f_=np.array([fx/mag_,fy/mag_])
    temp2=np.round(np.nan_to_num(np.arccos(np.dot(f_ ,Vy[j,:]))),2)
    #print(j,temp2<theta1)
    if temp<=theta1:
        
        if temp2<theta1:
        #print(frames[j,:])
        #print(j,"angle:",np.round(temp,2),"FVYdot",np.round(np.dot(F,VYpp),2),"F",np.round(F,2),"Vy",VYpp)
            temp_offset_theta.append(theta1)
            temp_frames.append(frames[j,:])
            temp_theta.append(frames[j,2])
            temp_id.append(j)
            temp_position_x.append(X[j])
            temp_position_z.append(Y[j])
            temp_force_x.append(fx)
            temp_force_z.append(fy)       
            temp_vx.append(Vx[j,:])
            temp_vy.append(Vy[j,:])
            temp_c1.append(C1[:,j])
            temp_c2.append(C2[:,j])
        #temp_c1.append([Vy[j,0]*np.cos(theta1),Vy[j,0]*np.cos(theta1)]
        #temp_c2.append(C2[:,j])    
    
    
    #ax.scatter(x0,y0,color="tab:blue")
fig.savefig(savefile+'/'+"_contact_ball_all_forces"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"_contact_ball_all_forces"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"_contact_ball_all_forces"+'_.pdf',dpi=300)







# In[Table 5]


table5 = [['entry',"px","pz","Forcex","Forcez"]]
X=[]
Y=[]
i=entry
const_=sim_data1.ball_radius*2
xb,yb=sim_data1.ballx_position[i]-const_/2,sim_data1.ballz_position[i] - const_/2
xb_,yb_=sim_data1.ballx_position[i],sim_data1.ballz_position[i]
for j in range(len(Position_x_contact_entry)):
    x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
    fx,fy=F_contact_ballx_entry[j],F_contact_ballz_entry[j]
    mag1=np.sqrt(fx**2 + fy**2)
    table5.append([str(j),str(np.round(x0-xb_,2)),str(np.round(y0-yb_,2)),str(np.round(fx,3)),str(np.round(fy,3))])
    X.append(x0-xb_)
    Y.append(y0-yb_)
    
table5.append(["sum:",'-','-',np.sum(F_contact_ballx_entry),np.sum(F_contact_ballz_entry)])
print(tabulate(table5, headers='firstrow'))
np.savetxt(savefile+'/table5.csv', table5, delimiter=",", fmt='%s')






# In[Table 6]



table6 = [['entry',"px","pz","Forcex","Forcez","forcemag"]]
i=entry
F_mag=[]
for i in range(len(temp_id)):
    mag_temp=np.sqrt(temp_force_x[i]**2 + temp_force_z[i]**2)
    F_mag.append(mag_temp)
    table6.append([temp_id[i],np.round(temp_position_x[i],2),np.round(temp_position_z[i],2),np.round(temp_force_x[i],2),np.round(temp_force_z[i],2),np.round(mag_temp,2)])

table6.append(["sum",str(0),str(0),str(np.round(np.sum(temp_force_x),2)),str(np.round(np.sum(temp_force_z),2)),str(0)])
print(tabulate(table6, headers='firstrow'))
np.savetxt(savefile+'/table6.csv', table6, delimiter=",", fmt='%s')







# In[contact ball forces sorted]
d=0.3
wxmax=d
wxmin=-d
wymin=-d
wymax=d
const=(wxmax-wxmin)/(wymax-wymin)
fig = plt.figure(dpi=300)
fig.set_size_inches(const*3,3)
ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
const_=sim_data1.ball_radius*2
xb,yb=0-const_/2,0 - const_/2
x0_=xb
y0_=yb
patch = matplotlib.patches.Rectangle((x0_, y0_),const_, const_,fc='none',edgecolor='tab:grey')     
ax.add_patch(patch)

for j in range(len(temp_id)):
    x0,y0=temp_position_x[j],temp_position_z[j]
    vx=temp_vx[j]
    vy=temp_vy[j]
    c1=temp_c1[j]
    c2=temp_c2[j]
    fx=temp_force_x[j]
    fy=temp_force_z[j]    
    ax.text(x0,y0,str(j),size=10)
    mag=np.sqrt(fx**2 + fy**2)
    ax.quiver(x0,y0, fx/mag,fy/mag,scale=10,color="g",width=.007,zorder=1)  
    ax.quiver(x0,y0, vx[0],vx[1],scale=15,color="b",width=.005,zorder=2)  
    ax.quiver(x0,y0, vy[0],vy[1],scale=15,color="r",width=.005,zorder=2) 
    ax.quiver(x0,y0,c1[0],c1[1] ,color="magenta",width=.002, scale=10,label='Positive cone')     
    ax.quiver(x0,y0,c2[0],c2[1] ,color="magenta",width=.002, scale=10,label='negative cone')

fig.savefig(savefile+'/'+"_contact_ball"+'_.jpg',dpi=300)
fig.savefig(savefile+'/'+"_contact_ball"+'_.svg',dpi=300)
fig.savefig(savefile+'/'+"_contact_ball"+'_.pdf',dpi=300)









# In[Table 7]

FE=np.zeros((len(temp_position_x),3))
FEA=np.zeros((len(temp_position_x),3))
FVX_=[]
FVY_=[]
G_=[]
#table6 = [['entry',"px","pz","Forcex","Forcez","forcemag"]]
table7 = [['entry',"px","pz","Forcex","Forcez","forcemag","vy_x","vy_y","fn_x:","fn_y:"]]
for i in range(len(temp_position_x)):
    #print(str(temp_id[i]))
    theta=temp_theta[i]
    temp_offset=temp_offset_theta[i]
    px=temp_position_x[i]
    pz=temp_position_z[i]
    fx_=temp_force_x[i]
    fz_=temp_force_z[i]
    #fn=F_mag[i]*np.cos(temp_offset)
    r=np.array([px,pz,0])
    F=np.array([fx_,fz_,0])
    vy=temp_vy[i]
    fn=np.dot(vy,np.array([fx_,fz_]))
    #print(fn)
    #FVX_.append()
    #FVY_.append(np.round(fn*vy[1],2))
    #print("fn:",np.round(fn,2),"vy:",np.round(vy,2),)
    #fn=np.dot(vy,np.array([fx_,fz_]))
    #print("fn:",np.round(fn,2))
    #print(str(temp_id[i]),"fn:",np.round(fn,2),"vy:",np.round(vy,2))
    table7.append([str(temp_id[i]),np.round(temp_position_x[i],2),np.round(temp_position_z[i],2),np.round(temp_force_x[i],2),np.round(temp_force_z[i],2),str(np.round(fn,2)),np.round(vy[0],2),np.round(vy[1],2),str(np.round(fn*vy[0],2)),str(np.round(fn*vy[1],2))])
    
    Ad=np.array([[np.cos(theta),-np.sin(theta),0],
              [np.sin(theta),np.cos(theta),0],
              [-pz*np.cos(theta) + px*np.sin(theta), pz*np.sin(theta) + px*np.cos(theta),1]])


    B=np.array([[1,0],[0,1],[0,0]])
    G=Ad@B
    G_.append(G)
    f=np.array([[0],[fn]])
    M=np.cross(r,F)
    FE[i,:]=np.round(G@f.flatten(),2)
    FVX_.append(np.round(FE[i,0],2))
    FVY_.append(np.round(FE[i,1],2))    
    FEA[i,:]=np.round([fx_*np.cos(temp_offset),fz_*np.sin(temp_offset),M[2]],2)
    #print("fe:",np.round(G@f.flatten(),2))
    #print("fe_act:",np.round([fx_,fz_,M[2]],2))
    #print("fx:",np.round(fx_,2),"fz:",np.round(fz_,2),"Moment:",np.cross(r,F))
    #print("fx:",np.round(fx_,2))
    #print("fz:",np.round(fz_,2))
jj=entry
table7.append(["sum",
                0,
                0,
                str(np.round(np.sum(temp_force_x),2)),
                str(np.round(np.sum(temp_force_z),2)),
                0,
                0,
                0,
                str(np.round(np.sum(FVX_),2)),
                str(np.round(np.sum(FVY_),2))])

print("Ball contact force x:",np.round(bFcx[entry],2),np.round(y1_bFcx[entry],2))
print("Ball contact force z:",np.round(bFcz[entry],2),np.round(y2_bFcz[entry],2))
#axs.plot(time,bFcx,color="tab:red",linewidth=1,label='x')
#axs.plot(time,bFcz,color="tab:blue",linewidth=1,label='z')

print(tabulate(table7, headers='firstrow'))
np.savetxt(savefile+'/table7.csv', table7, delimiter=",", fmt='%s')









# In[Table 8]
nb = sim_data1.nb
ni = sim_data1.ni
ns = sim_data1.ns



skin_mass = sim_data1.skin_mass
ball_mass = sim_data1.ball_mass 
particle_mass = sim_data1.particle_mass
bot_mass = sim_data1.bot_mass

table8 = [["system_mu_cone","ball_mu_cone"]]

mu=.05
table8.append([nb*bot_mass*mu*9.81 + ni*particle_mass*mu*9.81 + ns*skin_mass*mu,ball_mass*mu*9.81])
print(tabulate(table8, headers='firstrow'))

np.savetxt(savefile+'/table8.csv', table8, delimiter=",", fmt='%s')











