# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 14:14:49 2021

@author: dmulr
"""


import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import warnings
warnings.filterwarnings("ignore")
import numpy as np
from numpy import savetxt
import random
import os
import csv
import glob
from csv import writer
import timeit
import cv2
from os.path import exists
#from IPython.display import HTML
import matplotlib.pyplot as plt
from matplotlib import animation
import matplotlib.font_manager as fm
import matplotlib.patches as patches
import matplotlib
from matplotlib import colors as colors
from matplotlib.pyplot import cm
from matplotlib.patches import RegularPolygon
from shutil import copyfile
from scipy.spatial import ConvexHull
from scipy.interpolate import RegularGridInterpolator
from sympy import Plane, Point3D
from sympy import *
from scipy.spatial import Delaunay
try:
    import pyhull.convex_hull as cvh
except:
    logging.warning('Failed to import pyhull')
try:
    import cvxopt as cvx
except:
    logging.warning('Failed to import cvx')
    
import itertools    
###############################################################################
class robots:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor
        ###### Imported Variables #########
        self.path=path
        self.mainDirectory = self.path
        
        copyfile(__file__,self.mainDirectory+self.name+"/"+'objects.py')
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        
        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        
        self.nb=self.parameters['nb'] # number of bots
        self.bot_mass = self.convert_mass*self.parameters['bot_mass'] 
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.bot_height = self.convert_dist*self.parameters['bot_height']
        self.bot_geom = self.parameters['bot_geom']
        self.R = self.convert_dist*self.parameters['R']
        self.bot_volume=np.pi*self.bot_height*(self.bot_width/2)**2   # calculate volume
        self.membrane_width = self.convert_dist*self.parameters['membrane_width']
        self.membrane_type = self.parameters['membrane_type']
        self.skin_width = self.parameters['skin_width']
        self.ns = self.parameters['ns']
        self.spring_stiffness = self.parameters['spring_stiffness'] 
        self.spring_damping = self.parameters['spring_damping'] 
        
        
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        self.membrane_density = self.parameters['membrane_density']
        
        ####### Calculated variables ######
        self.bot_density=self.bot_mass/self.bot_volume # calculate density of robot 
        self.bot_material = self.Material(self.lateralFriction)
        #self.membrane_density = 2000
        self.countm = 0 
        self.rl = 0

        self.fixed = False
        self.membrane_material = self.Material(self.lateralFriction)
        # Colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        self.col_w = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(.8, .8, .8)) # Purple
        
        
        self.skinM = []
        self.bots = []
        self.force = []
        self.Springs = []
        
        self.bot_xposition = {}
        self.bot_yposition = {}
        self.bot_zposition = {}
        
        self.bot_xvelocity = {}
        self.bot_yvelocity = {}
        self.bot_zvelocity = {}
        
        self.bot_xForcetotal = {}
        self.bot_yForcetotal = {}
        self.bot_zForcetotal = {}
        
        
        self.bot_xForcecontact = {}
        self.bot_yForcecontact = {}
        self.bot_zForcecontact = {}
        
        
        self.skin_xposition = {}
        self.skin_yposition = {}
        self.skin_zposition = {}
        
        
        self.skin_x_contact_forces = {}  #x position
        self.skin_y_contact_forces = {}   # y position
        self.skin_z_contact_forces = {}   # z position
                                
        self.skin_x_total_forces = {}   #x position
        self.skin_y_total_forces = {}   # y position
        self.skin_z_total_forces = {}   # z position
                                      

        
        for i in range(self.nb):
            
            # positions
            self.bot_xposition["bot_xposition{0}".format(i)]=[]  #x position
            self.bot_yposition["bot_yposition{0}".format(i)]=[]  # y position
            self.bot_zposition["bot_zposition{0}".format(i)]=[]  # z position 
            
            
            # velocities
            self.bot_xvelocity["bot_xvelocity{0}".format(i)]=[]  # x velocity
            self.bot_yvelocity["bot_yvelocity{0}".format(i)]=[]  # y velocity
            self.bot_zvelocity["bot_zvelocity{0}".format(i)]=[]  # z velocity
            
            
            # total forces
            self.bot_xForcetotal["bot_xForcetotal{0}".format(i)]=[]  # Force x
            self.bot_yForcetotal["bot_yForcetotal{0}".format(i)]=[]  # Force y
            self.bot_zForcetotal["bot_zForcetotal{0}".format(i)]=[]  # force z
            
            
            # contact forces
            self.bot_xForcecontact["bot_xForcecontact{0}".format(i)]=[]  # Force x
            self.bot_yForcecontact["bot_yForcecontact{0}".format(i)]=[]  # Force y
            self.bot_zForcecontact["bot_zForcecontact{0}".format(i)]=[]  # force z            
        
        
        
        
            # postion 
            theta=i*2*np.pi/self.nb # set angle
            x = self.R*np.cos(theta)+self.xcenter  # set x positon 
            y = self.bot_height/2                     # set y position 
            z = self.R*np.sin(theta)+self.zcenter  # set z position 
            # create body
            #### geometry of the robots to be cylinders
            if self.bot_geom=="cylinder":
                
                bot = chrono.ChBodyEasyCylinder(self.bot_width/2, self.bot_height,self.bot_density,True,True)
                # set position
                bot.SetPos(chrono.ChVectorD(x,y,z)) 
                bot.SetName("bot"+str(i)) # set name (important for contact tracking)
                bot.SetId(i)              # set id   (impoortant for contact tracking )  
                # material 
                bot.SetMaterialSurface(self.bot_material)  # set material 
                # rotate them so we can form the membrane
                rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
                
                
                ##### Add force components to each bot 
                # x forces 
                botforcex = chrono.ChForce()  # create it 
                bot.AddForce(botforcex) # apply it to bot object
                botforcex.SetMode(chrono.ChForce.FORCE) # set the mode 
                botforcex.SetDir(chrono.VECT_X) # set direction 
                self.force.append(botforcex) # add to force list
                
                # y forces    
                botforcey = chrono.ChForce() # create it 
                bot.AddForce(botforcey) # apply it to bot object
                botforcey.SetMode(chrono.ChForce.FORCE) # set the mode
                botforcey.SetDir(chrono.VECT_Y) # set direction 
                self.force.append(botforcey) # add to force list
                
                # z forces            
                botforcez = chrono.ChForce() # create it 
                bot.AddForce(botforcez) # apply it to bot object
                botforcez.SetMode(chrono.ChForce.FORCE) # set the mode
                botforcez.SetDir(chrono.VECT_Z) # set direction 
                self.force.append(botforcez) # add to force list
                
                
                
                # set max speed (Not always needed but it helps)
                bot.SetMaxSpeed(2)
                bot.SetLimitSpeed(False)
                
            
                # make the color blue 
                bot.AddAsset(self.col_b)
                
                # zeroth robot so we know which is which in the videos
                if i==0:   
                    bot.AddAsset(self.col_p)   
                if i>=20 and i<=25:
                    bot.AddAsset(self.col_w)
                if i==22:   
                    bot.AddAsset(self.col_y)  
                
                # set collision
                bot.SetCollide(True)
                
                
                # set fixed
                bot.SetBodyFixed(self.fixed)
                
                # link to floor
                #pt=chrono.ChLinkMatePlane()
                #pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                
                # add link to the system 
                #self.my_system.AddLink(pt)
                
                # add bot to series of array 
                self.my_system.Add(bot) # add bot to system 
                self.bots.append(bot) # add bot to bot array 
                
                
            

            # #t = (2*np.pi/self.nb)/(self.ns+1)
            # for j in range(1,self.ns+1,1):
            #     b_ang=2*np.pi/self.nb                   # angle between centers of bots
            #     o_ang=np.arctan((self.skin_width*1.01)/self.R)   # angle offset for radius of bot
            #     p_ang=np.arctan((self.skin_width*1.01)/self.R)    
            #     theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang 
            #     x=self.R*np.cos(theta)+self.xcenter # x position 
            #     y=self.bot_height/2              # y position 
            #     z=self.R*np.sin(theta)+self.zcenter # z position  
            #     # create them and set position
            #     #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
            #     skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True) # create particle
            #     skinm.SetPos(chrono.ChVectorD(x,y,z)) # set position 
            #     skinm.SetMaterialSurface(self.membrane_material) # add material 
            #     skinm.SetNoGyroTorque(True) # no gyro toruqe 
            #     skinm.SetName('skin'+str(i)) # create name 
            #     skinm.SetId(i) # set id 
            #     skinm.SetBodyFixed(self.fixed)
            #                 # link to floor
            #     #pt=chrono.ChLinkMatePlane()
            #     #pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            #     # add link to the system 
            #     #self.my_system.AddLink(pt)
            #     # rotate them bout y axis
            #     rotation1 = chrono.ChQuaternionD()
            #     rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
            #     skinm.SetRot(rotation1)
            #     self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
            #     self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
            #     self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                
            #     self.skin_x_contact_forces["skin_x_contact_forces{0}".format(self.countm)]=[]  #x position
            #     self.skin_y_contact_forces["skin_y_contact_forces{0}".format(self.countm)]=[]  # y position
            #     self.skin_z_contact_forces["skin_z_contact_forces{0}".format(self.countm)]=[]  # z position
                                        
            #     self.skin_x_total_forces["skin_x_total_forces{0}".format(self.countm)]=[]  #x position
            #     self.skin_y_total_forces["skin_y_total_forces{0}".format(self.countm)]=[]  # y position
            #     self.skin_z_total_forces["skin_z_total_forces{0}".format(self.countm)]=[]  # z position
            #     self.my_system.Add(skinm)
            #     self.skinM.append(skinm)                                                           
    
            
            
            
            
            if self.membrane_type==1:
                b_ang=2*np.pi/self.nb                   # angle between centers of bots
                o_ang=np.arctan((self.bot_width*.66)/self.R)   # angle offset for radius of bot
                p_ang=np.arctan((self.skin_width*1.1)/self.R)           # angle offset for radius of skin particle
                #o_ang=np.arctan((self.skin_width*1.01)/self.R)   # angle offset for radius of bot
                #p_ang=np.arctan((self.skin_width*1.01)/self.R)                 
                # Between this bot and last bot
                if i>=1 and i<self.nb:
                    for j in range(1,self.ns+1,1):
                        # Initial postion of each particle
                        theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang 
                        x=self.R*np.cos(theta)+self.xcenter # x position 
                        y=self.bot_height/2              # y position 
                        z=self.R*np.sin(theta)+self.zcenter # z position  
                        
                        # create them and set position
                        #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
                        skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True) # create particle
                        skinm.SetPos(chrono.ChVectorD(x,y,z)) # set position 
                        skinm.SetMaterialSurface(self.membrane_material) # add material 
                        skinm.SetNoGyroTorque(True) # no gyro toruqe 
                        skinm.SetName('skin'+str(i)) # create name 
                        skinm.SetId(i) # set id 
                                    # link to floor
                        #pt=chrono.ChLinkMatePlane()
                        #pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                        # add link to the system 
                        #self.my_system.AddLink(pt)
                        # rotate them bout y axis
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                        self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                        self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                        
                        self.skin_x_contact_forces["skin_x_contact_forces{0}".format(self.countm)]=[]  #x position
                        self.skin_y_contact_forces["skin_y_contact_forces{0}".format(self.countm)]=[]  # y position
                        self.skin_z_contact_forces["skin_z_contact_forces{0}".format(self.countm)]=[]  # z position
                                                
                        self.skin_x_total_forces["skin_x_total_forces{0}".format(self.countm)]=[]  #x position
                        self.skin_y_total_forces["skin_y_total_forces{0}".format(self.countm)]=[]  # y position
                        self.skin_z_total_forces["skin_z_total_forces{0}".format(self.countm)]=[]  # z position
                                                      
                        print("countm"+str(self.countm))
                        self.countm=self.countm+1
                        # Attach springs if more then one was created    
                        if j>1:
                            ground=chrono.ChLinkSpring() # create spring 1
                            p1=0; p2=self.skin_width/2 # points where each spring is attatched 
                            p3=0; p4=-self.skin_width/2
                            h=self.bot_height/5
                            
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True) # link spring to particles
                            ground.Set_SpringK(self.spring_stiffness) # set spring constant
                            ground.Set_SpringR(self.spring_damping) # set damping constant
                            ground.Set_SpringRestLength(self.rl) # set resting length 
                            ground.AddAsset(self.col_p) # add color 
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground) # add spring to system 
                            self.Springs.append(ground)
                            ground1=chrono.ChLinkSpring() # create spring 2
                            ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True) # link spring to particles 
                            ground1.Set_SpringK(self.spring_stiffness) # set spring constant
                            ground1.Set_SpringR(self.spring_damping) # set damping 
                            ground1.AddAsset(self.col_p) # create color 
                            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground1) # add to the system                                 
                            self.Springs.append(ground1)
                        #Link to cylinder 
                        if j==1:
                            skinm.AddAsset(self.col_p) # add color
                            glue=chrono.ChLinkMateFix() # cretae fix constraint
                            glue.Initialize(skinm,self.bots[i]) # fix object to bot
                            self.my_system.AddLink(glue) # add to system 
                            # Link last particle with this bot
                            if i>=2:
                                glue=chrono.ChLinkMateFix() # create the constraint 
                                glue.Initialize(self.skinM[-1],self.bots[-1]) # add constraint 
                                self.my_system.AddLink(glue) # add to the system 
                        if j==self.ns:
                            skinm.AddAsset(self.col_p)
                        self.my_system.Add(skinm)
                        self.skinM.append(skinm)
                
                # Between this bot and first bot
                if i==self.nb-1:
                    for j in range(1,self.ns+1,1):
                        # Initial postion of each particle
                        theta=(i+1)*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang
                        x=self.R*np.cos(theta)+self.xcenter
                        y=self.bot_height/2
                        z=self.R*np.sin(theta)+self.zcenter
                        
                        self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                        self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                        self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                        
                        self.skin_x_contact_forces["skin_x_contact_forces{0}".format(self.countm)]=[]  #x position
                        self.skin_y_contact_forces["skin_y_contact_forces{0}".format(self.countm)]=[]  # y position
                        self.skin_z_contact_forces["skin_z_contact_forces{0}".format(self.countm)]=[]  # z position
                                                
                        self.skin_x_total_forces["skin_x_total_forces{0}".format(self.countm)]=[]  #x position
                        self.skin_y_total_forces["skin_y_total_forces{0}".format(self.countm)]=[]  # y position
                        self.skin_z_total_forces["skin_z_total_forces{0}".format(self.countm)]=[]  # z position
                                                  
                       
                        
                        
                        self.countm=self.countm+1
                        # Create particles
                        #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
                        skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True)
                        skinm.SetPos(chrono.ChVectorD(x,y,z))
                        skinm.SetMaterialSurface(self.membrane_material)
                        skinm.SetNoGyroTorque(True)
                        skinm.SetName("skin"+str(i))
                        skinm.SetId(i)
                        # rotate them
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        #pt=chrono.ChLinkMatePlane()
                        #pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                        #self.my_system.AddLink(pt)
                        # Attach springs    
                        if j>1:
                            ground=chrono.ChLinkSpring()
                            p1=0; p2=self.skin_width/2
                            p3=0; p4=-self.skin_width/2
                            h=self.bot_height/5
                    
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                            ground.Set_SpringK(self.spring_stiffness)
                            ground.Set_SpringR(self.spring_damping)
                            ground.Set_SpringRestLength(self.rl)
                            ground.AddAsset(self.col_y)
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground)
                            self.Springs.append(ground)
                            ground1=chrono.ChLinkSpring()
                            ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True)
                            ground1.Set_SpringK(self.spring_stiffness)
                            ground1.Set_SpringR(self.spring_damping)
                            ground.Set_SpringRestLength(self.rl) # set resting length 
                            ground1.AddAsset(self.col_y)
                            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground1)  
                            self.Springs.append(ground1)
                        #Link to cylinder
                        if j==1:
                            skinm.AddAsset(self.col_p)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinm,self.bots[0])
                            self.my_system.AddLink(glue)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(self.skinM[-1],self.bots[0])
                            self.my_system.AddLink(glue)
                         
                        if j==self.ns:
                            skinm.AddAsset(self.col_p)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinm,self.bots[1])
                            self.my_system.AddLink(glue)
                            
                        self.my_system.Add(skinm)
                        self.skinM.append(skinm)               
                
               
                
               
                
    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)
    
 
    # return system (Helps with adding to the data extractor, simulaor, and controllers)
    def return_system(self):
        ''' Return system, springs, bots, obj, force '''
        return(self.my_system,self.Springs,self.bots,self.force)

    # save position data
    def save_data_position(self):
        ''' Save position of each bot '''
        for i in range(self.nb):
            self.bot_xposition["bot_xposition"+str(i)].append(self.bots[i].GetPos().x)
            self.bot_yposition["bot_yposition"+str(i)].append(self.bots[i].GetPos().y)
            self.bot_zposition["bot_zposition"+str(i)].append(self.bots[i].GetPos().z)
            
        for i in range(self.countm):
            self.skin_xposition['skin_xposition'+str(i)].append(self.skinM[i].GetPos().x)
            self.skin_yposition['skin_yposition'+str(i)].append(self.skinM[i].GetPos().y)
            self.skin_zposition['skin_zposition'+str(i)].append(self.skinM[i].GetPos().z)
            
                
    def save_data_forces(self):
        ''' Save position of each bot '''
                 
        for i in range(self.countm):
            self.skin_x_contact_forces["skin_x_contact_forces"+str(i)].append(self.skinM[i].GetContactForce().x)
            self.skin_y_contact_forces["skin_y_contact_forces"+str(i)].append(self.skinM[i].GetContactForce().y)
            self.skin_z_contact_forces["skin_z_contact_forces"+str(i)].append(self.skinM[i].GetContactForce().z)
                       
            self.skin_x_total_forces["skin_x_total_forces"+str(i)].append(self.skinM[i].Get_Xforce().x)
            self.skin_y_total_forces["skin_y_total_forces"+str(i)].append(self.skinM[i].Get_Xforce().y)
            self.skin_z_total_forces["skin_z_total_forces"+str(i)].append(self.skinM[i].Get_Xforce().z)
    
    
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.nb):
            self.bot_xvelocity["bot_xvelocity"+str(i)].append(self.bots[i].GetPos_dt().x)
            self.bot_yvelocity["bot_yvelocity"+str(i)].append(self.bots[i].GetPos_dt().y)
            self.bot_zvelocity["bot_zvelocity"+str(i)].append(self.bots[i].GetPos_dt().z)
            

    # save force data            
    def save_data_Forces(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.bot_xForcetotal["bot_xForcetotal"+str(i)].append(self.bots[i].Get_Xforce().x)
            self.bot_yForcetotal["bot_yForcetotal"+str(i)].append(self.bots[i].Get_Xforce().y)
            self.bot_zForcetotal["bot_zForcetotal"+str(i)].append(self.bots[i].Get_Xforce().z)
    
    def save_data_Forces_contact(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.bot_xForcecontact["bot_xForcecontact"+str(i)].append(self.bots[i].GetContactForce().x)
            self.bot_yForcecontact["bot_yForcecontact"+str(i)].append(self.bots[i].GetContactForce().y)
            self.bot_zForcecontact["bot_zForcecontact"+str(i)].append(self.bots[i].GetContactForce().z)
            
            
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.bot_xposition,self.bot_yposition,self.bot_zposition)
    
    # return position membrane data
    def return_position_membrane_data(self):
        ''' return the dictionary of each pmembrane particle '''
        return(self.skin_xposition,self.skin_yposition,self.skin_zposition)  
      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.bot_xvelocity,self.bot_yvelocity,self.bot_zvelocity)
        
    # return force data    
    def return_total_force(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal,)
    
    
    def return_skin_forces(self):
        return(self.skin_x_contact_forces,
               self.skin_y_contact_forces,
               self.skin_z_contact_forces,
               self.skin_x_total_forces,
               self.skin_y_total_forces,
               self.skin_z_total_forces)


    # return force data    
    def return_force_data_contact(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact)
    
class Interiors:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor    
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']

        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        
        self.nb=self.parameters['nb']
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.particle_mass = self.convert_mass*self.parameters['particle_mass']
        self.particle_width = self.convert_dist*self.parameters['particle_width']
        self.particle_height = self.convert_dist*self.parameters['particle_height']
        self.particle_geom = self.parameters['particle_geom']
        #self.R = self.convert_dist*self.parameters['R']
        self.scale_radius = self.parameters['scale_radius']
        self.R =  self.scale_radius*self.convert_dist*self.parameters['R']
        
        self.interior_mode = self.parameters['interior_mode']
        self.offset_radius=self.parameters['offset_radius']
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        
        self.particle_volume=np.pi*self.particle_height*(self.particle_width/2)**2   # calculate volume
        self.particle_density=self.particle_mass/self.particle_volume # calculate density of robot 
        #self.particle_material = self.Material()
        
        self.fixed = False
        (self.n,self.Ri) = self.MaxValues()
        self.total_particles = np.sum(self.n)
        self.particle_material = self.Material(0.01)
        
        
        self.parameters['n'] = self.n
        self.parameters['total_particles'] = self.total_particles
        self.parameters['Ri'] = self.Ri
        
        
        self.number_parameters = self.parameters['number_parameters']
        #print(len(self.parameters),self.number_parameters+4)
        if len(self.parameters)==self.number_parameters+4:
            #print('save')
            np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
            
        else:
             print('not save')
        
        
        

        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))
        
        
        
        
        # with open(self.mainDirectory+name+"/Parameters.csv", 'a') as f_object:
        #     # Pass this file object to csv.writer()
        #     # and get a writer object
        #     writer_object = writer(f_object)
        #     #Pass the list as an argument into
        #     #the writerow()
        #     writer_object.writerow(['ring configuration',self.n])
        #     writer_object.writerow(['total number of particles',self.total_particles])
        #     #Close the file object
        #     f_object.close()        
        
        
        
        self.particle_xposition = {}
        self.particle_yposition = {}
        self.particle_zposition = {}
        
        
        self.particle_xvelocity = {}
        self.particle_yvelocity = {}
        self.particle_zvelocity = {}
        
        
        self.particle_xForcetotal = {}
        self.particle_yForcetotal = {}
        self.particle_zForcetotal = {}
        
        
        self.particle_xForcecontact = {}
        self.particle_yForcecontact = {}
        self.particle_zForcecontact = {}        
        
        
        self.particles = []
        self.Rm = []
        self.Area = 0
        self.N1=0
        self.N2=0
        
        # colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_r = chrono.ChColorAsset(); self.col_r.SetColor(chrono.ChColor(1, 0, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple        
        
        # empty array
        self.particle_xposition = {}
        self.particle_yposition  = {}
        self.particle_zposition  = {}
        
        self.particle_xvelocity = {}
        self.particle_yvelocity  = {}
        self.particle_zvelocity  = {}
        


        #### mono-dispersion #####
        if self.interior_mode=="monodispersion":  
            count=0
            for i in range(self.n.size):
                
                if i%2==0:
                    con='b'
                    const=0
                else:

                    con='a'
                    const=0
                    
                    
                self.radius2=self.particle_width/2 - self.offset_radius
                
                R2=self.radius2*self.n[i]/(np.pi) + const   
                
                for j in range(self.n[i]):
                     
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity
                    

                    count=count+1
                    
                    # position
                    x = R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter # x position 
                    y = .5*self.particle_height                         # y position 
                    z = R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter # z position 
                    
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius2, self.particle_height,self.particle_density,True,True)
                    gran.SetMaterialSurface(self.particle_material) # add material 
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetName("grana"+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
    
                    # add color
                    gran.AddAsset(self.col_r)
                    
                    # mate to floor
                    pt=chrono.ChLinkMatePlane() 
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
            np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)   


        #### bidispersion 
        if self.interior_mode=="bidispersion": 
            count=0
            for i in range(len(self.n)):
                #print("i=",str(i))
                
                if i%2==0:
                    self.radius2 = (self.particle_width/2)*(2**.5)
                    con = 'b'
                    const = 0
                    self.N1 = 1+self.N1
                else:
                    self.radius2 = self.particle_width/2 
                    con = 'a'
                    const = 0
                    self.N2 = 1+self.N2
                    
                    
                R2=self.radius2*self.n[i]/(np.pi) + const
                # empty arrays of variables
                for j in range(self.n[i]):
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 
                    
                    self.particle_xForcetotal["particle_xForcetotal{0}".format(count)]=[]
                    self.particle_yForcetotal["particle_yForcetotal{0}".format(count)]=[]
                    self.particle_zForcetotal["particle_zForcetotal{0}".format(count)]=[]
                    
                    self.particle_xForcecontact["particle_xForcecontact{0}".format(count)]=[]
                    self.particle_yForcecontact["particle_yForcecontact{0}".format(count)]=[]
                    self.particle_zForcecontact["particle_zForcecontact{0}".format(count)]=[]
                    
                    
                    
                    
                    #R2=self.radius2*self.n[i]/(np.pi) + const# raidus of ring 
                    # x,y,z positions
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter
                    y=.5*self.particle_height 
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter
                    #print("j=",str(j),str(np.round(self.radius2,3)),"x,y",str(np.round(x,2)),str(np.round(z,2)))
                    self.Rm.append(self.radius2)
                    self.Area = self.Area + (np.pi)*(self.radius2)**2
                    if i%2==0:
                        con = 'b'
                        const = 0
                        self.N1 = 1+self.N1
                    else: 
                        con = 'a'
                        const = 0
                        self.N2 = 1+self.N2  
                        
                    # create body
                    gran = chrono.ChBody()
                    gran = chrono.ChBodyEasyCylinder(self.radius2 , self.particle_height ,self.particle_density,True,True)
                    
      
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.particle_material)
                    gran.SetName('gran'+str(con)+str(count))
                    gran.SetId(i)
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    count=count+1
                    # alternate colors on rings so one is red the other is green
                    if i%2==0:
                    # add color
                        col_r = chrono.ChColorAsset()
                        col_r.SetColor(chrono.ChColor(1, 0, 0))
                        gran.AddAsset(col_r)
                    else:
                        col_r = chrono.ChColorAsset()
                        col_r.SetColor(chrono.ChColor(0, 1, 0))
                        gran.AddAsset(col_r)  
                    
                    # set speed limit ( Helps but not always needed)
                    # gran.SetMaxSpeed(2)
                    # gran.SetLimitSpeed(True)
                    
                    # link to plane    
                    #pt=chrono.ChLinkMatePlane()
                    #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    #self.my_system.AddLink(pt)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
                    
        np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)     
        self.parameters['Area'] = self.Area
        self.parameters['N1(smaller)'] = self.N1
        self.parameters['N2(larger)'] = self.N2
        
        np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        
        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))             












        #### bi_dispersion_ring #####
        if self.interior_mode=="bi_dispersion_ring":  
            count=0
            for i in range(self.n.size):

                self.radius2 = self.particle_width/2 - self.offset_radius  
                
                R2=self.radius2*self.n[i]/(np.pi) 
                
                for j in range(self.n[i]):
                    radii1=self.radius2*.25
                    radii2=0
                    r_m = np.random.choice([radii1,radii2])
                    if r_m==radii1:
                        color=self.col_r
                        con='a'
                        self.N1=1+self.N1
                    else:
                        color=self.col_g
                        con='b'
                        self.N2=1+self.N2
                        
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity
                    
                    count=count+1
                    # position
                    x = R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter # x position 
                    y = .5*self.particle_height                         # y position 
                    z = R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter # z position 
                    self.Rm.append(self.radius2 - r_m)
                    self.Area = self.Area + (np.pi)*(self.radius2 - r_m)**2
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius2 - r_m, self.particle_height,self.particle_density,True,True)
                    gran.SetMaterialSurface(self.particle_material) # add material 
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetName("gran"+con+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
    
                    # add color
                    gran.AddAsset(color)
                    
                    # mate to floor
                    #pt=chrono.ChLinkMatePlane() 
                    #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    #self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
            np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)     
            self.parameters['Area'] = self.Area
            self.parameters['N1(smaller)'] = self.N1
            self.parameters['N2(larger)'] = self.N2
        
        np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        
        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))
        
        

        
        
        #### bi_dispersion_uniform_ring #####
        if self.interior_mode=="bi_dispersion_uniform_ring":  
            count=0
            for i in range(self.n.size):

                self.radius2 = self.particle_width/2 - self.offset_radius  
                
                R2=self.radius2*self.n[i]/(np.pi) 
                
                for j in range(self.n[i]):
                    radii1=self.radius2
                    radii2=0
                    r_m =(1-radii1) + np.random.uniform(0, radii1)
                    if r_m==radii1:
                        color=self.col_r
                        con='a'
                    else:
                        color=self.col_g
                        con='b'
                        
                        
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity
                    
                    count=count+1
                    # position
                    x = R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter # x position 
                    y = .5*self.particle_height                         # y position 
                    z = R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter # z position 
                    self.Rm.append(self.radius2*r_m)
                    
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius2*r_m, self.particle_height,self.particle_density,True,True)
                    gran.SetMaterialSurface(self.particle_material) # add material 
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetName("gran"+con+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
    
                    # add color
                    gran.AddAsset(color)
                    
                    # mate to floor
                    pt=chrono.ChLinkMatePlane() 
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
            np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)   
        


    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)
        
        
        
        
    def MaxValues(self):
        if self.interior_mode=='empty':
            N=[]
            R=0

        radius=self.bot_width/2
        radius2=self.particle_width/2 
        
        if self.interior_mode=="bidispersion":            
            Rin=self.R-radius
            S=2*radius2+2*np.sqrt(2)*radius2
            P=int(Rin/S)
            Ri=np.zeros(2*P)

            N=[]
            for i in range(P):
                Ri[2*i]=Rin-np.sqrt(2)*radius2-i*S
                Ri[2*i+1]=Rin-(2*np.sqrt(2)*radius2+radius2)-i*S
    

            for i in range(P):
                N.append(int((np.pi*2*Ri[2*i])/(2*np.sqrt(2)*radius2)))
                N.append(int((np.pi*2*Ri[2*i+1])/(2*radius2)))

        if self.interior_mode=="monodispersion":
            Rin=self.R-radius-radius2
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]
            
        if self.interior_mode=="bi_dispersion_ring":
            Rin=self.R-radius
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]           
           
            
        if self.interior_mode=="bi_dispersion_uniform_ring":
            Rin=self.R-radius
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]           
            
            
        return(N,Ri.flatten())
    
    
    
    
    
    # return system (Helps with adding to the data extractor, simulaor, and controllers)
    def return_system(self):
        ''' Return system, springs, bots, obj, force '''
        return(self.my_system,self.particles)

    # save position data
    def save_data_position(self):
        ''' Save position of each bot '''
        for i in range(self.total_particles):
            self.particle_xposition["particle_xposition"+str(i)].append(self.particles[i].GetPos().x)
            self.particle_yposition["particle_yposition"+str(i)].append(self.particles[i].GetPos().y)
            self.particle_zposition["particle_zposition"+str(i)].append(self.particles[i].GetPos().z)
            

            
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.total_particles):
            self.particle_xvelocity["particle_xvelocity"+str(i)].append(self.particles[i].GetPos_dt().x)
            self.particle_yvelocity["particle_yvelocity"+str(i)].append(self.particles[i].GetPos_dt().y)
            self.particle_zvelocity["particle_zvelocity"+str(i)].append(self.particles[i].GetPos_dt().z)
            
    def save_data_Forces(self):
        ''' Save total force on each bot '''
        for i in range(self.total_particles):
            self.particle_xForcetotal["particle_xForcetotal"+str(i)].append(self.particles[i].Get_Xforce().x)
            self.particle_yForcetotal["particle_yForcetotal"+str(i)].append(self.particles[i].Get_Xforce().y)
            self.particle_zForcetotal["particle_zForcetotal"+str(i)].append(self.particles[i].Get_Xforce().z)
    
    def save_data_Forces_contact(self):
        ''' Save total force on each bot '''
        for i in range(self.total_particles):
            self.particle_xForcecontact["particle_xForcecontact"+str(i)].append(self.particles[i].GetContactForce().x)
            self.particle_yForcecontact["particle_yForcecontact"+str(i)].append(self.particles[i].GetContactForce().y)
            self.particle_zForcecontact["particle_zForcecontact"+str(i)].append(self.particles[i].GetContactForce().z)
                     
          
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.particle_xposition,self.particle_yposition,self.particle_zposition)
    

      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity)
        
    # return force data    
    def return_force_data(self):
        ''' return dictionary of each bot forces '''
        return(self.particle_xForcetotal,self.particle_yForcetotal,self.particle_zForcetotal)
  
    # return force data    
    def return_force_data_contact(self):
        ''' return dictionary of each bot forces '''
        return(self.particle_xForcecontact,self.particle_yForcecontact,self.particle_zForcecontact)
      
        
class floor:
    def __init__(self,name,my_system,path):
        self.name=name
        self.my_system = my_system
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.floor_height = self.parameters['floor_height']
        self.floor_length = self.parameters['floor_length']
        self.bot_height = self.convert_dist*self.parameters['bot_height']
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        
        self.material1 = self.Material(.05)
        self.material2 = self.Material(0)
        
        self.body_floor = chrono.ChBody()
        self.body_floor.SetName('floor')
        self.body_floor.SetBodyFixed(True)
        self.body_floor.SetPos(chrono.ChVectorD(0, -self.floor_height, 0 ))
        self.body_floor.SetMaterialSurface(self.material1)
        self.body_floor.GetCollisionModel().ClearModel()
        self.body_floor.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        self.body_floor.GetCollisionModel().BuildModel()       
        self.body_floor.SetCollide(True)
        body_floor_shape = chrono.ChBoxShape()
        body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.floor_length, self.floor_height, self.floor_length)
        self.body_floor.GetAssets().push_back(body_floor_shape)
        col_k = chrono.ChColorAsset()
        col_k.SetColor(chrono.ChColor(0, 0, 0))
        self.body_floor.AddAsset(col_k)
        self.my_system.Add(self.body_floor)
        
 

        self.body_floor2 = chrono.ChBody()
        self.body_floor2.SetName('floor2')
        self.body_floor2.SetBodyFixed(True)
        self.body_floor2.SetPos(chrono.ChVectorD(0, self.floor_height+self.bot_height+.001, 0 ))
        self.body_floor2.SetMaterialSurface(self.material2)
        self.body_floor2.GetCollisionModel().ClearModel()
        self.body_floor2.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        self.body_floor2.GetCollisionModel().BuildModel()       
        self.body_floor2.SetCollide(True)
        self.my_system.Add(self.body_floor2)
        
        
        
        
        
        
        
        
        
        
        
        
        # body_floor_shape2 = chrono.ChBoxShape()
        # body_floor_shape2.GetBoxGeometry().Size = chrono.ChVectorD(self.floor_length, self.floor_height, self.floor_length)
        # self.body_floor2.GetAssets().push_back(body_floor_shape2)
        # col_k = chrono.ChColorAsset()
        # col_k.SetColor(chrono.ChColor(0, 0, 0))
        # self.body_floor2.AddAsset(col_k)
        # self.my_system.Add(self.body_floor2)        
        # self.body_floor = chrono.ChBody()
        #self.body_floor.SetName('floor2')
        # self.body_floor.SetBodyFixed(True)
        # self.body_floor.SetPos(chrono.ChVectorD(0, self.tall+self.height2, 0 ))
        # self.body_floor.SetMaterialSurface(self.material2)
        # self.body_floor.GetCollisionModel().ClearModel()
        # self.body_floor.GetCollisionModel().AddBox(self.length, self.tall, self.length) # hemi sizes
        # self.body_floor_shape = chrono.ChBoxShape()
        
        
        # self.body_floor2 = chrono.ChBody()
        # self.body_floor2.SetName('floor2')
        # self.body_floor2.SetBodyFixed(True)
        # self.body_floor2.SetPos(chrono.ChVectorD(0, self.floor_height+self.bot_height, 0 ))
        # self.body_floor2.SetMaterialSurface(self.material2)
        # self.body_floor2.GetCollisionModel().ClearModel()
        # self.body_floor2.GetCollisionModel().AddBox(self.floor_length, self.floor_height+self.bot_height, self.floor_length) # hemi sizes
        # self.body_floor2.SetCollide(True)
        # self.my_system.Add(self.body_floor2) 
        
        #self.body_floor2_shape = chrono.ChBoxShape()
        # self.body_floor_shape2.GetBoxGeometry().Size = chrono.ChVectorD((self.floor_length, self.floor_tall, self.floor_length))
        # self.body_floor2.GetAssets().push_back(self.body_floor_shape2)
  
        # body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.length, self.tall, self.length)
        # self.body_floor.GetAssets().push_back(body_floor_shape)
        # col_g = chrono.ChColorAsset()
        # col_g.SetColor(chrono.ChColor(0, 0, 0))
        # self.body_floor.AddAsset(col_g)body_floor.GetCollisionModel().BuildModel()       
  
    
    
    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)    
    
    
    def return_enviroment(self):
        return(self.my_system)
    
class Ball:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor
        ###### Imported Variables #########
        self.path=path
        self.mainDirectory = self.path
        
        copyfile(__file__,self.mainDirectory+self.name+"/"+'objects.py')
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.height=self.convert_dist*self.parameters['bot_height']
        self.y = self.convert_dist*self.parameters['bot_height']/2
        self.x =self.parameters['ballx']
        self.z =self.parameters['ballz']
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['Cr'] 
        self.material=self.Material(self.lateralFriction)
        self.geom = self.parameters['ball_geometry'] 
        self.radius = self.parameters['ball_radius']
        self.mb= self.parameters['ball_mass']
        volume3=np.pi*self.height*(self.radius)**2   # calculate volume
        self.rho=self.mb/volume3 # density
        self.balls=[]
        self.obj=[]
        self.forceb=[]
        self.bx={}
        self.bz={}
        self.bvx={}
        self.bvz={}
        
        self.bFx={}
        self.bFy={}
        self.bFz={}
        
        self.bFtx={}
        self.bFty={}
        self.bFtz={}
        
        self.bq0 = {}
        self.bq1 = {}
        self.bq2 = {}
        self.bq3 = {}        
        
        self.Fb={}
        self.PX={}
        self.PY={}
        self.TIME={}
        
        
        
        self.bx["ballx"]=[]
        self.bz["ballz"]=[]        
        self.bvx["ballvx"]=[]        
        self.bvz["ballvz"]=[]
        
        self.bFx["ballvx"]=[]
        self.bFy["ballvy"]=[]
        self.bFz["ballvz"]=[]
        
        
        self.bFtx["ballvx"]=[]
        self.bFty["ballvy"]=[]        
        self.bFtz["ballvz"]=[]
        
        self.bq0["bq0"] = []
        self.bq1["bq1"] = []
        self.bq2["bq2"] = []
        self.bq3["bq3"] = [] 
        
        self.Fb["Fb"]=[]
        self.PX["PX"]=[]
        self.PY["PY"]=[]
        self.TIME["TIME"]=[]
        
        z2x = chrono.ChQuaternionD()
        z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
        z2y = chrono.ChQuaternionD()
        z2y.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))
        
        if self.geom=='circle':
            #Create ball
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rho,True,True) # specify properties and make it a cylinder
            # set position
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetId(10000) 
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(True) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            #ball.SetMaterialSurface(self.material)
            # pt=chrono.ChLinkMatePlane()
            # pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            # self.my_system.AddLink(pt) 
            #pt=chrono.ChLinkMatePlane() 
            #pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            #append the constraint and the ball to array of objects and system
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
          
        if self.geom=="square":
            #const=self.radius*2*np.pi/4
            const=self.radius*2
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyBox(const,self.height,const,self.rho,True,True) # create object # specify properties and make it a cylinder
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetId(10000) 
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(True) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            
        if self.geom=="triangle":     
            
            const=self.radius*2*np.pi/3
            r=const*np.sqrt(3)/3
            
            
            #r = .04/.5236
            x1=0
            y1=r
            x2=r*np.cos(7*np.pi/6)
            y2=r*np.sin(7*np.pi/6)
            x3=r*np.cos(11*np.pi/6)
            y3=r*np.sin(11*np.pi/6)
            pt_vect = chrono.vector_ChVectorD()
            #const=.4627
            # creates bottom
            pt_vect.push_back(chrono.ChVectorD(y1,self.height/2,x1))
            pt_vect.push_back(chrono.ChVectorD(y2,self.height/2,x2))
            pt_vect.push_back(chrono.ChVectorD(y3,self.height/2,x3))
            
            pt_vect.push_back(chrono.ChVectorD(y1,-self.height/2,x1))
            pt_vect.push_back(chrono.ChVectorD(y2,-self.height/2,x2))
            pt_vect.push_back(chrono.ChVectorD(y3,-self.height/2,x3))            

            ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rho,True,True)   
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(True) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            
            
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddConvexHull(pt_vect)
            ball.GetCollisionModel().BuildModel()            
            
        if self.geom=="import":          
  
            ball = chrono.ChBody()
            # # Attach a visualization shape .
            # # First load a .obj from disk into a ChTriangleMeshConnected:
            path="C:/soro/python/Pychrono/Strings/String_grasping/object_file/"
            mesh_for_visualization = chrono.ChTriangleMeshConnected()
            mesh_for_visualization.LoadWavefrontMesh(path+'part3.obj')
            mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
            visualization_shape = chrono.ChTriangleMeshShape()
            visualization_shape.SetMesh(mesh_for_visualization)
            ball.AddAsset(visualization_shape)
            #rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
            #rotation1.Q_from_AngAxis(-np.pi/2, chrono.ChVectorD(0, 1, 0)) 
            #ball.SetRot(rotation1)
            mesh_for_collision = chrono.ChTriangleMeshConnected()
            mesh_for_collision.LoadWavefrontMesh(path+'part3.obj')
            #Optionally: you can scale/shrink/rotate the mesh using this:
            mesh_for_collision.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddTriangleMesh(
                mesh_for_collision, # the mesh 
                False,  # is it static?
                False)  # is it convex?
            ball.GetCollisionModel().BuildModel()        
            ball.SetPos(chrono.ChVectorD(0,0,0))
            ball.SetMass(16)
            ball.SetInertiaXX(chrono.ChVectorD(0.270,0.400,0.427))
            ball.SetInertiaXY(chrono.ChVectorD(0.057,0.037,-0.062))            
            ball.SetBodyFixed(True)
            #rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
           # rotation1.Q_from_AngAxis(-np.pi/2, chrono.ChVectorD(0, 1, 0)) 
            #ball.SetRot(rotation1)
            col_y = chrono.ChColorAsset() # apply color
            col_y.SetColor(chrono.ChColor(1, 1, 0))
            ball.AddAsset(col_y)
            ball.SetCollide(True) # set the collision mode
            self.my_system.Add(ball)      

            # set position
        myforcez = chrono.ChForce() # create it 
        ball.AddForce(myforcez) # apply it to bot object
        myforcez.SetMode(chrono.ChForce.FORCE) # set the mode
        myforcez.SetDir(chrono.VECT_Z) # set direction 
        myforcez.SetVpoint(chrono.ChVectorD(0,0.05,0))
        self.forceb.append(myforcez) # add to force list

        #create constraint to fix it to the floor
        # pt=chrono.ChLinkMatePlane() 
        # pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
        # prismatic_ground_ball = chrono.ChLinkLockPrismatic()
        # prismatic_ground_ball.SetName("prismatic_ground_ball")
        # prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_Z_TO_Y))
        # self.my_system.AddLink(prismatic_ground_ball)
        
        #self.my_system.Add(prismatic_ground_ball) 
                
        
        # material
        ball.SetMaterialSurface(self.material) # apply material
        self.obj.append(ball)
        self.balls.append(ball) 
        self.my_system.Add(ball)            
        
            
            #col_y = chrono.ChColorAsset() # apply color
            #col_y.SetColor(chrono.ChColor(1, 1, 0))
            #ball.AddAsset(col_y)            

    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)            
        
            
        
    def save_data_position(self):
        ''' Function that saves the positon of the ball '''
        self.bx["ballx"].append(self.balls[0].GetPos().x) # x postion 
        self.bz["ballz"].append(self.balls[0].GetPos().z) # z postion 
    

    def save_data_velocity(self):
        ''' save the velocity of the ball '''
        self.bvx["ballvx"].append(self.balls[0].GetPos_dt().x) # x velocity
        self.bvz["ballvz"].append(self.balls[0].GetPos_dt().z)# z velocity 
        
    def save_contact_force(self):
        ''' save contact forces  '''
        self.bFx["ballvx"].append(self.balls[0].GetContactForce().x) # x contact force
        self.bFy["ballvy"].append(self.balls[0].GetContactForce().y) # y contact force
        self.bFz["ballvz"].append(self.balls[0].GetContactForce().z) # z contact force
    
    def save_total_force(self):
        ''' save contact forces  '''
        self.bFtx["ballvx"].append(self.balls[0].Get_Xforce().x) # x contact force
        self.bFty["ballvy"].append(self.balls[0].Get_Xforce().y) # y contact force
        self.bFtz["ballvz"].append(self.balls[0].Get_Xforce().z) # z contact force
    
    def save_angle_data(self):
        ''' save angle in quarterions '''
        temp=self.balls[0].GetRot()  
        q0=temp.e0
        q1=temp.e1
        q2=temp.e2
        q3=temp.e3
        self.bq0["bq0"].append(q0)
        self.bq1["bq1"].append(q1)
        self.bq2["bq2"].append(q2)
        self.bq3["bq3"].append(q3)                  
    
    
    def return_angle_data(self):
        return(self.bq0,self.bq1,self.bq2,self.bq3)
    
    def return_position_data(self):
        ''' return velocity data '''
        return(self.bx,self.bz)   

    def return_velocity_data(self):
        ''' return velocity data '''
        return(self.bvx,self.bvz)    

    # return system    
    def return_system(self):
        ''' return system  '''
        return(self.my_system,self.balls)      
    
    def return_ball_contact_forces(self):
        return(self.bFx,self.bFy,self.bFz)
    
    def return_ball_total_forces(self):
        return(self.bFtx,self.bFty,self.bFtz)
    
class simulate:
    def __init__(self,name,my_system,bots,particles,Ball,controller,my_rep,path):
       
        self.name=name
        self.my_system = my_system
        self.Bots = bots
        self.particles = particles
        self.controller = controller
        self.my_rep = my_rep
        self.ball=Ball
        
        ###### Imported Variables #########
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.control_mode=self.parameters['control_mode']
        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        self.visual = self.parameters['visual']
        self.dt = self.parameters['dt']
        self.time_end = self.parameters['time_end']
        self.save_rate = self.parameters['save_rate']
        
        ###### Predefined variables ######
        self.myapplication=[]
        self.epoch = 0
        self.camx = 0
        self.camy = 5
        self.camz = 0
        self.camy_height = 2
        self.save_video = False
        self.Trip=False
        self.sim_start=timeit.default_timer()
        self.THETA=[]
        self.Rr_=[]
        ###### Empty Arrays ######
        self.time = [] # time empty array
        self.time_contact = [] # contact time  empty array
        self.number_contacts = []
        self.Contact_points_x = []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.Contact_force_x = []
        self.Contact_force_y = []
        self.Contact_force_z = []
        
        self.Contact_force_x2 = []
        self.Contact_force_y2 = []
        self.Contact_force_z2 = []
        
        self.bodiesA = []
        self.bodiesB = []
        self.bodiesA_ID = []
        self.bodiesB_ID = []
        
        self.dir_xx = []
        self.dir_xy = []
        self.dir_xz = []
        
        self.dir_yx = []
        self.dir_yy = []
        self.dir_yz = []
        
        self.dir_zx = []
        self.dir_zy = []
        self.dir_zz = []   
        
        
    # simulate the robot
    def simulate(self):
        #### Irrrlecnt
        #  Create an Irrlicht application to visualize the system
        if self.visual=="irr":
            self.myapplication = chronoirr.ChIrrApp(self.my_system, self.name , chronoirr.dimension2du(800,600))
            self.myapplication.AddTypicalSky()
            self.myapplication.AddTypicalLogo()
            self.myapplication.AddTypicalCamera(chronoirr.vector3df(self.camx,self.camy,self.camz),chronoirr.vector3df(self.camx,0,self.camz))
            self.myapplication.SetSymbolscale(.002)
            self.myapplication.SetShowInfos(True)
            #self.myapplication.SetContactsDrawMode(2)
            self.myapplication.SetPaused(self.Trip)
            self.myapplication.AddTypicalLights()
            self.myapplication.DrawAll               
            self.myapplication.AssetBindAll()
            self.myapplication.AssetUpdateAll()
            self.myapplication.AddShadowAll()
            self.count=0
            self.myapplication.SetTimestep(self.dt)
            self.myapplication.SetTryRealtime(False)
            ##### Run the sim
            while(self.myapplication.GetDevice().run()):
                #self.my_rep.ResetList()
                #self.save_contacts()
                #self.my_rep.ResetList()
                self.myapplication.BeginScene()
                self.myapplication.DrawAll()
                self.myapplication.DoStep()
                self.controller.run_controller()
                self.controller.get_position()
                self.my_rep.ResetList()
                self.save_contacts() 
                #self.my_rep.ResetList()
                
                if self.controller.t>=15:
                    print("time change")
                    self.controller.t=0
                else:
                    self.controller.t=self.controller.t+self.dt
                    print("not time")


                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="shape_morphing":
                    ft=np.round(self.controller.Psi.tanh(np.round(self.my_system.GetChTime(),3)),3)
                
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                else:
                    print('time='+str(time))
                
                self.myapplication.EndScene()
                self.save_parameters()
                
                self.epoch = self.epoch + 1
                self.my_rep.ResetList()
                
                # aaa=len(self.Bots.bots)
                # cam_x=0.33*(self.Bots.bots[0].GetPos().x + self.Bots.bots[int(aaa/3)].GetPos().x + self.Bots.bots[int(2*aaa/3)].GetPos().x)
                # cam_y=0.33*(self.Bots.bots[0].GetPos().y + self.Bots.bots[int(aaa/3)].GetPos().y + self.Bots.bots[int(2*aaa/3)].GetPos().y)
                # cam_z=0.33*(self.Bots.bots[0].GetPos().z + self.Bots.bots[int(aaa/3)].GetPos().z + self.Bots.bots[int(2*aaa/3)].GetPos().z)
                # self.myapplication.GetSceneManager().getActiveCamera().setPosition(chronoirr.vector3df(cam_x,cam_y+self.camy_height,cam_z))
                # self.myapplication.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3df(cam_x,cam_y,cam_z))
                # self.myapplication.SetVideoframeSave(self.save_video)
                # self.myapplication.SetVideoframeSaveInterval(round(1/(self.dt*60)))
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.time_end :
                    self.myapplication.GetDevice().closeDevice()
            self.sim_end=timeit.default_timer()
            
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60
            self.number_parameters = self.parameters['number_parameters']
            
            
        #### pov   
        if self.visual=="pov": 
            while (self.my_system.GetChTime() < self.time_end): 
                self.my_system.DoStepDynamics(self.dt)
                self.controller.get_position()
                self.controller.run_controller()
                self.save_contacts() 
                self.my_rep.ResetList()
                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="shape_morphing":
                    ft=np.round(self.controller.Psi.tanh(np.round(self.my_system.GetChTime(),3)),3)
                
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                else:
                    print('time='+str(time))
                
                self.save_parameters()
                self.epoch = self.epoch + 1
            self.sim_end=timeit.default_timer()     
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60
            self.number_parameters = self.parameters['number_parameters']

        #print(len(self.parameters),self.number_parameters+4)
        if len(self.parameters)==self.number_parameters+1:
        #print('save')
            np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        else:
            print('not save')
        
        
        with open(self.mainDirectory+self.name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))

    def save_contacts(self):
        if self.epoch%self.save_rate==0:
            self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
            crt_list = self.my_rep.GetList()

            if self.my_system.GetContactContainer().GetNcontacts()!=0:
                self.number_contacts.append(self.my_system.GetContactContainer().GetNcontacts())
                self.time_contact.append(self.my_system.GetChTime()) 
                self.Contact_points_x.append(crt_list[0])
                self.Contact_points_y.append(crt_list[1])
                self.Contact_points_z.append(crt_list[2])
                
                self.Contact_force_x.append(crt_list[3])
                self.Contact_force_y.append(crt_list[4])
                self.Contact_force_z.append(crt_list[5])
                
                self.Contact_force_x2.append(crt_list[6])
                self.Contact_force_y2.append(crt_list[7])
                self.Contact_force_z2.append(crt_list[8]) 
                
                self.bodiesA.append(crt_list[9])
                self.bodiesB.append(crt_list[10])
                
                self.bodiesA_ID.append(crt_list[11])
                self.bodiesB_ID.append(crt_list[12]) 

                self.dir_xx.append(crt_list[14])
                self.dir_xy.append(crt_list[15])
                self.dir_xz.append(crt_list[16])
                
                self.dir_yx.append(crt_list[17])
                self.dir_yy.append(crt_list[18])
                self.dir_yz.append(crt_list[19]) 
                
                self.dir_zx.append(crt_list[20])
                self.dir_zy.append(crt_list[21])
                self.dir_zz.append(crt_list[22])       
              
                    
    def save_parameters(self):
        ''' Function that collects data of for the system '''
        if self.epoch%self.save_rate==0:
            self.time.append(np.round(self.my_system.GetChTime(),4))
            self.Bots.save_data_position()
            self.Bots.save_data_Forces()
            self.Bots.save_data_velocity()
            self.Bots.save_data_Forces_contact()
        
            self.Bots.save_data_forces()

            self.particles.save_data_position()
            self.particles.save_data_velocity()
            self.particles.save_data_Forces_contact()
            self.particles.save_data_Forces()  
            
            self.controller.save_field_value()
            self.controller.save_controller_forces()

            if self.control_mode=="grasping":
                self.ball.save_data_position()
                self.ball.save_data_velocity()  
                self.ball.save_contact_force()
                self.ball.save_total_force()
                self.ball.save_angle_data()
                self.ball.Fb["Fb"].append(self.controller.fb)
                self.ball.TIME["TIME"].append(np.round(self.my_system.GetChTime(),4))
                self.ball.PX["PX"].append(self.ball.balls[0].GetPos().x)
                self.ball.PY["PY"].append(self.ball.balls[0].GetPos().z) 
                self.THETA.append(self.controller.theta)
                self.Rr_.append(self.controller.Rr)
class controller():
    """ Class for storung all the controller information"""
    def __init__(self,name,my_system,bots,Psi,Ball,path):
        self.name=name # name of simulation
        self.my_system = my_system # the system object
        self.robots = bots # robot objets
        self.Psi=Psi # potential fields 
        self.Ball=Ball
        self.forceb=self.Ball.forceb
        ##### Extract variables from other imported objects #####
        self.forces=self.robots.force
        self.nb=self.robots.nb 
        self.px=0
        self.pz=0
        self.t=0
        self.Rr=2
        self.theta=0
        self.trig1=1
        self.trig2=1
        self.alpha_=0
        self.a=0.0001
        self.b=0.0001
        ###### Imported Variables #########
        self.mainDirectory = path   # main directory 
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()                 
        self.control_mode=self.parameters['control_mode']  # control mode 
        

        if self.control_mode=="grasping":
            
            self.alpha1 = self.parameters['alpha1']
            self.alpha2 = self.parameters['alpha2']
            self.beta = self.parameters['beta']
            self.tcut1 = self.parameters['tcut1']
            self.tcut2 = self.parameters['tcut2']
            
            self.xc1 = self.parameters["xc1"]
            self.zc1 = self.parameters["yc1"]
            
            self.xc2 = self.parameters["xc2"]
            self.zc2 = self.parameters["yc2"]    
            
            self.ball_radius = self.parameters["ball_radius"]
            
            self.fb = 0
        else:    
            self.alpha1 = self.parameters['alpha1']
            self.alpha2 = self.parameters['alpha2']
            self.beta = self.parameters['beta']
                    
            
        self.bot_position_x=0
        self.bot_position_z=0
        
        self.bot_velocitiy_x=0
        self.bot_velocitiy_z=0
        
        self.Field_value={}
        self.F_controller_x={}
        self.F_controller_z={}
        for i in range(self.nb):
            self.Field_value["bot{0}".format(i)]=[]           
            self.F_controller_x["bot{0}_control_force_x".format(i)]=[]
            self.F_controller_z["bot{0}_control_force_z".format(i)]=[]
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        
        
        
        
        
    def run_controller(self):
        """Function to run the controllers"""
        if self.control_mode =="shape_formation":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.shape_controller() # run shape controller
                self.apply_force2(self.FX,self.FZ)         # apply force
                
        if self.control_mode =="shape_morphing":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.morph_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force            
   
        if self.control_mode =="grasping":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.grasping_controller_explore()
                #(self.FX,self.FZ) = self.grasping_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force    
         


                   
    def save_field_value(self):
        
        if self.control_mode =="shape_formation":  
            for i in range(self.nb):
               self.Field_value["bot"+str(i)].append(self.Psi.F(self.bot_position_x[i],self.bot_position_z[i]))
        if self.control_mode =="shape_morphing":
            t=np.round(self.my_system.GetChTime(),3)
            for i in range(self.nb):
                self.Field_value["bot"+str(i)].append(self.Psi.F_morph(self.bot_position_x[i],self.bot_position_z[i],self.Psi.tanh(t)))    
    
        if self.control_mode=="grasping":
            t=np.round(self.my_system.GetChTime(),3)
            for i in range(self.nb):
                self.Field_value["bot"+str(i)].append(self.Psi.FGRASP(self.bot_position_x[i],self.bot_position_z[i],t))    
                
    def save_controller_forces(self):
        for i in range(self.nb):
            self.F_controller_x["bot"+str(i)+"_control_force_x"].append(self.FX[i])
            self.F_controller_z["bot"+str(i)+"_control_force_z"].append(self.FZ[i])
                        
            
    def shape_controller(self):
        """ Shape controller """
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.Psi.FX(self.bot_position_x[i],self.bot_position_z[i])
            Fz=self.Psi.FY(self.bot_position_x[i],self.bot_position_z[i])
            mag=np.sqrt(Fx**2+Fz**2)
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
            
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))        


    def morph_controller(self):
        """ Morphing Controller for morphing from one shape to another """
        FX=[]
        FZ=[]
        t=np.round(self.my_system.GetChTime(),3)
        ft=self.Psi.tanh(t)
        #print("ft="+str(np.round(ft,2)),end='\r')
        for i in range(self.nb):
            Fx=self.Psi.FX_morph(self.bot_position_x[i],self.bot_position_z[i],ft)
            Fz=self.Psi.FY_morph(self.bot_position_x[i],self.bot_position_z[i],ft)
            
            mag=np.sqrt(Fx**2+Fz**2)
            
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
                
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
        
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))        

    # def grasping_controller(self):
    #     """ Grasping Controller """
    #     FX=[]
    #     FZ=[]
    #     time=np.round(self.my_system.GetChTime(),3)

        
    #     if time>self.Psi.tcut2:
    #         #print("tcut2")
    #         self.Ball.balls[0].SetBodyFixed(False)             
          
    #     if time>self.Psi.tcut3:
    #         #print("tcut3")
    #         self.fb=self.fb+.001
    #         self.forceb[0].SetMforce(self.fb)
    #         self.forceb[0].SetDir(chrono.VECT_X)
    #         self.Ball.balls[0].SetBodyFixed(False)   
            
    #     # self.Ball.Fb["Fb"].append(self.fb)
    #     # self.Ball.TIME["TIME"].append(time)
    #     # self.Ball.PX["PX"].append(self.Ball.balls[0].GetPos().x)
    #     # self.Ball.PY["PY"].append(self.Ball.balls[0].GetPos().z)     
            
            
            
    #     for i in range(self.nb):
    #         Fx1=self.Psi.FXGRASP(self.bot_position_x[i],self.bot_position_z[i],time)
    #         Fz1=self.Psi.FYGRASP(self.bot_position_x[i],self.bot_position_z[i],time)
    #         mag1=np.sqrt(Fx1**2 + Fz1**2)
    #         Fx1=Fx1/mag1
    #         Fz1=Fz1/mag1

            
    #         if mag1==0:
    #             FXX=0
    #             FZZ=0
    #         else:
    #             FXX=Fx1
    #             FZZ=Fz1
 
    #         if self.tcut1>time:
    #             alpha_=self.alpha1
    #         else:
    #             alpha_=self.alpha2
                
    #         fx=-alpha_*FXX-self.beta*self.bot_velocitiy_x[i]
    #         fz=-alpha_*FZZ-self.beta*self.bot_velocitiy_z[i]

    #         FX.append(fx)
    #         FZ.append(fz)
    #     return(np.asarray(FX),np.asarray(FZ)) 



    def grasping_controller(self):
        """ Grasping Controller """
        FX=[]
        FZ=[]
        time=np.round(self.my_system.GetChTime(),3)

        
        # if time>self.Psi.tcut2:
        #     #print("tcut2")
        #     self.Ball.balls[0].SetBodyFixed(False)             
          
        # if time>self.Psi.tcut3:
        #     #print("tcut3")
        #     self.fb=self.fb+.001
        #     self.forceb[0].SetMforce(self.fb)
        #     self.forceb[0].SetDir(chrono.VECT_X)
        #     self.Ball.balls[0].SetBodyFixed(False)   
            
        self.Ball.Fb["Fb"].append(self.fb)
        self.Ball.TIME["TIME"].append(time)
        self.Ball.PX["PX"].append(self.Ball.balls[0].GetPos().x)
        self.Ball.PY["PY"].append(self.Ball.balls[0].GetPos().z)     
  
        for i in range(self.nb):

            fxb = self.Psi.Fx_point(self.bot_position_x[i],self.bot_position_z[i],0,0)
            fyb = self.Psi.Fy_point(self.bot_position_x[i],self.bot_position_z[i],0,0)
            
            #fxb = self.Psi.dphix_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            #fyb = self.Psi.dphiy_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            
            
            
            Fz1 = fxb-1*fyb
            Fx1 = -fyb-1*fxb
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            #if self.tcut1>time:
            alpha_=self.alpha1
            #else:
                #alpha_=self.alpha2
                
            fx=alpha_*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=alpha_*FZZ#-self.beta*self.bot_velocitiy_z[i]

            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 


    def grasping_controller_explore(self):
        """ Grasping Controller """
        FX=[]
        FZ=[]
        time = np.round(self.my_system.GetChTime(),3)

        print("trig1:",self.trig1)
        print("trig2:",self.trig2)
        # approach object
        if self.t<5 and self.trig1!=0: # check on it every 10 times for efficiency
            print("approach")
            self.theta = self.theta
            self.Rr = 0
            self.trig1 = 0
            self.alpha_=self.alpha1
            self.a=.001
            self.b=.001
            
         # back off of object    
        elif self.t>=5 and self.t<=10:
            print("back up")
            self.theta = self.theta
            self.Rr = 2
            self.alpha_=self.alpha1
            self.a=1.31
            self.b=1.31
            
        # move to next position 
        elif self.t>10 and self.t<15.0 and self.trig2!=0:
            print("move")
            self.trig2 = 0
            self.theta = self.theta + np.pi/6
            self.Rr = 2
            self.alpha_=self.alpha2
            self.a=1.31
            self.b=1.31
            
        elif self.t>=15.0:
            self.t = 0
            self.trig1 = 1
            self.trig2 = 1
            print("reset")
        

        self.px = self.Rr*np.cos(self.theta)
        self.pz = self.Rr*np.sin(self.theta)
        
        print("px:",np.round(self.px,2))
        print("pz:",np.round(self.pz,2))
        
        self.Ball.Fb["Fb"].append(self.fb)
        self.Ball.TIME["TIME"].append(time)
        self.Ball.PX["PX"].append(self.Ball.balls[0].GetPos().x)
        self.Ball.PY["PY"].append(self.Ball.balls[0].GetPos().z)     
        
        for i in range(self.nb):
            fxb = self.Psi.dphix_oval(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz,self.a,self.b)
            fyb = self.Psi.dphiy_oval(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz,self.a,self.b)
            #fxb = self.Psi.Fx_point(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz)
            #fyb = self.Psi.Fy_point(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz)
            
            #fxb = self.Psi.dphix_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            #fyb = self.Psi.dphiy_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            
            
            
            Fz1 = fyb
            Fx1 = fxb
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            #if self.tcut1>time:
            #alpha_=self.alpha1
            #else:
                #alpha_=self.alpha2
                
            fx=-self.alpha_*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha_*FZZ#-self.beta*self.bot_velocitiy_z[i]

            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 
    
    
    def get_position(self):
        """ get position of boundary robots """
        xb=[]        
        zb=[]
        for i in range(self.nb):
            xb.append(self.robots.bots[i].GetPos().x)
            zb.append(self.robots.bots[i].GetPos().z)
        return(xb,zb)
      
    def get_velocity(self):
        """ get velocity of boundary robots """
        xbv=[]
        zbv=[]
        for i in range(self.nb):
            xbv.append(self.robots.bots[i].GetPos_dt().x)
            zbv.append(self.robots.bots[i].GetPos_dt().z)
        return(xbv,zbv)

    def apply_force(self,FX,FZ):  
        """ Appy forces to robots """
        for i in range(self.nb):
            self.fxt.append(FX[i])
            self.fyt.append(0)
            self.fzt.append(self.FZ[i])
            self.forces[3*i].SetMforce(float(FX[i]))
            self.forces[3*i].SetDir(chrono.VECT_X)
            self.forces[3*i+2].SetMforce(float(FZ[i]))
            self.forces[3*i+2].SetDir(chrono.VECT_Z)
 
    def apply_force2(self,FX,FZ):  
        """ Appy forces to robots """
        t=np.round(self.my_system.GetChTime(),3)
        
        if t<10:
            
            for i in range(self.nb):
                self.fxt.append(FX[i])
                self.fyt.append(0)
                self.fzt.append(self.FZ[i])
                self.forces[3*i].SetMforce(float(FX[i]))
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(float(FZ[i]))
                self.forces[3*i+2].SetDir(chrono.VECT_Z)
        if t>=10 and t<15:
            for i in range(self.nb):
                self.fxt.append(FX[i])
                self.fyt.append(0)
                self.fzt.append(self.FZ[i])
                rm1=np.random.choice(np.linspace(0,30,10))
                rm2=np.random.choice(np.linspace(0,30,10))
                rm3=np.random.choice(np.linspace(0,30,10))
                rm4=np.random.choice(np.linspace(0,30,10))
                self.forces[3*i].SetMforce(float(FX[i])+3*self.alpha*np.sin(rm1*t)+3*self.alpha*np.sin(rm2*t))
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(float(FZ[i])+3*self.alpha*np.cos(rm3*t)+3*self.alpha*np.cos(rm4*t))
                self.forces[3*i+2].SetDir(chrono.VECT_Z)                
                
                
                # rm1=np.random.choice(np.linspace(-10,10,10))
                # rm2=np.random.choice(np.linspace(-10,10,10))
                
            
                # self.forces[3*i].SetMforce(float(FX[i])+rm1)
                # self.forces[3*i].SetDir(chrono.VECT_X)
                # self.forces[3*i+2].SetMforce(float(FZ[i])+rm2)
                # self.forces[3*i+2].SetDir(chrono.VECT_Z)                
                
                
        if t>=15:
            for i in range(self.nb):
                self.fxt.append(FX[i])
                self.fyt.append(0)
                self.fzt.append(self.FZ[i])
                self.forces[3*i].SetMforce(float(FX[i]))
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(float(FZ[i]))
                self.forces[3*i+2].SetDir(chrono.VECT_Z)      
                
    def clear_temp_forces(self):
        """ Clear Temp forces """
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
  



class export_data():
    def __init__(self,my_system,robots,controls,interior,ball,simulation,Psi,my_rep,path,name):
        self.name = name # name of simulation
        self.mainDirectory = path   # main directory 
        
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.my_system = my_system
        self.robots = robots
        self.controls = controls
        self.interior = interior
        self.simulation = simulation
        self.ball = ball
        self.results_dir=self.mainDirectory+self.name+'/results'
        self.control_mode=self.parameters['control_mode']
        if not os.path.exists(self.results_dir):
            os.mkdir(self.results_dir)
        
        # time
        self.time = {'time':self.simulation.time} 
        
        # robot data
        (self.bot_xposition,self.bot_yposition,self.bot_zposition) = self.robots.return_position_data()
        (self.skin_xposition,self.skin_yposition,self.skin_zposition) = self.robots.return_position_membrane_data()
        (self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal) = self.robots.return_total_force()
        (self.bot_xvelocity,self.bot_yvelocity,self.bot_zvelocity) = self.robots.return_velocity_data()
        (self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact) = self.robots.return_force_data_contact()            
        (self.skin_x_contact_forces,self.skin_y_contact_forces,self.skin_z_contact_forces,self.skin_x_total_forces,self.skin_y_total_forces,self.skin_z_total_forces)=self.robots.return_skin_forces()


        # interior data
        (self.particle_xposition,self.particle_yposition,self.particle_zposition) = self.interior.return_position_data()
        (self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity) = self.interior.return_velocity_data()
        (self.particle_xForcetotal,self.particle_yForcetotal,self.particle_zForcetotal) = self.interior.return_force_data()
        (self.particle_xForcecontact,self.particle_yForcecontact,self.particle_zForcecontact) = self.interior.return_force_data()
        
        if self.control_mode=="grasping":
            (self.ball_xposition,self.ball_zposition)=self.ball.return_position_data()
            (self.ball_xvelocity,self.ball_zvelocity)=self.ball.return_velocity_data()        
            self.Fb=self.ball.Fb
            self.TIME=self.ball.TIME
            self.PX=self.ball.PX
            self.PY=self.ball.PY
            (self.bFx,self.bFy,self.bFz)=self.ball.return_ball_contact_forces()
            (self.bFtx,self.bFty,self.bFtz)=self.ball.return_ball_total_forces()
            (self.bq0,self.bq1,self.bq2,self.bq3)=self.ball.return_angle_data()
            self.THETA=self.simulation.THETA
            self.Rr_=self.simulation.Rr_
            
        self.Field_value=self.controls.Field_value
        
        self.F_controller_x=self.controls.F_controller_x
        self.F_controller_z=self.controls.F_controller_z
        
        self.time_contact = self.simulation.time_contact
        self.number_contacts = self.simulation.number_contacts
        self.Contact_points_x = self.simulation.Contact_points_x
        self.Contact_points_y = self.simulation.Contact_points_y
        self.Contact_points_z = self.simulation.Contact_points_z
        
        self.Contact_force_x = self.simulation.Contact_force_x
        self.Contact_force_y = self.simulation.Contact_force_y
        self.Contact_force_z = self.simulation.Contact_force_z
        
        self.Contact_force_x2 = self.simulation.Contact_force_x2
        self.Contact_force_y2 = self.simulation.Contact_force_y2
        self.Contact_force_z2 = self.simulation.Contact_force_z2
        
        self.bodiesA = self.simulation.bodiesA
        self.bodiesB = self.simulation.bodiesB
        
        self.bodiesA_ID = self.simulation.bodiesA_ID
        self.bodiesB_ID = self.simulation.bodiesB_ID
        

        
        self.Dir_xx = self.simulation.dir_xx
        self.Dir_xy = self.simulation.dir_xy
        self.Dir_xz = self.simulation.dir_xz
        
        self.Dir_yx = self.simulation.dir_yx
        self.Dir_yy = self.simulation.dir_yy
        self.Dir_yz = self.simulation.dir_yz
        
        self.Dir_zx = self.simulation.dir_zx
        self.Dir_zy = self.simulation.dir_zy
        self.Dir_zz = self.simulation.dir_zz

        
        
        
        self.number_contacts=np.asarray(self.number_contacts) # number of contacts
        self.lengthm=np.amax(self.number_contacts) # length of nc
        self.count=len(self.time_contact) 
        
        # #empty array to fill contact locations in each time step
        self.xc=np.zeros((self.lengthm,self.count)) # x points
        self.yc=np.zeros((self.lengthm,self.count)) # y points 
        self.zc=np.zeros((self.lengthm,self.count)) # z points
        
        self.Fcx=np.zeros((self.lengthm,self.count))
        self.Fcy=np.zeros((self.lengthm,self.count))
        self.Fcz=np.zeros((self.lengthm,self.count))
        
        self.Fcx2=np.zeros((self.lengthm,self.count))
        self.Fcy2=np.zeros((self.lengthm,self.count))
        self.Fcz2=np.zeros((self.lengthm,self.count))
        
        self.AID=np.zeros((self.lengthm,self.count))
        self.BID=np.zeros((self.lengthm,self.count))       
        
        self.Dirxx_=np.zeros((self.lengthm,self.count))
        self.Dirxy_=np.zeros((self.lengthm,self.count))
        self.Dirxz_=np.zeros((self.lengthm,self.count))

        self.Diryx_=np.zeros((self.lengthm,self.count))
        self.Diryy_=np.zeros((self.lengthm,self.count))
        self.Diryz_=np.zeros((self.lengthm,self.count))
 
        
        self.Dirzx_=np.zeros((self.lengthm,self.count))
        self.Dirzy_=np.zeros((self.lengthm,self.count))
        self.Dirzz_=np.zeros((self.lengthm,self.count))
        
        self.AN={} # empty array of names of contact bodies
        self.BN={} # empty array of names of contact bodes
        #print(len(self.number_contacts))
        for i in range(len(self.number_contacts)-1):    
             #print(i)
             self.AN["AN{0}".format(i)]=self.bodiesA[i]  # A names
             self.BN["BN{0}".format(i)]=self.bodiesB[i]  # B names
            
        # fill the arrays with contact information 
        for i in range(self.count):
            ind=self.number_contacts[i]
            temp1=self.Contact_points_x[i]
            temp2=self.Contact_points_y[i]
            temp3=self.Contact_points_z[i]
            temp4=self.Contact_force_x[i]
            temp5=self.Contact_force_y[i]
            temp6=self.Contact_force_z[i]
            
            temp7=self.bodiesA_ID[i]
            temp8=self.bodiesB_ID[i]
            
            temp9=self.Contact_force_x2[i]
            temp10=self.Contact_force_y2[i]
            temp11=self.Contact_force_z2[i]   
            
            temp12=self.Dir_xx[i] 
            temp13=self.Dir_xy[i] 
            temp14=self.Dir_xz[i] 
            
            temp15=self.Dir_yx[i] 
            temp16=self.Dir_yy[i] 
            temp17=self.Dir_yz[i] 
        
            temp18=self.Dir_zx[i] 
            temp19=self.Dir_zy[i] 
            temp20=self.Dir_zz[i]             
            
            
            
            
            
            
            

            
            # convert to array
            temp1=np.asarray(temp1)
            temp2=np.asarray(temp2)
            temp3=np.asarray(temp3)
            temp4=np.asarray(temp4)
            temp5=np.asarray(temp5)
            temp6=np.asarray(temp6)
            temp7=np.asarray(temp7)
            temp8=np.asarray(temp8)
            
            
            temp9=np.asarray(temp9)
            temp10=np.asarray(temp10)
            temp11=np.asarray(temp11)
            
            temp12=np.asarray(temp12)
            temp13=np.asarray(temp13)
            temp14=np.asarray(temp14)
            
            temp15=np.asarray(temp15)
            temp16=np.asarray(temp16)
            temp17=np.asarray(temp17)  
            
            temp18=np.asarray(temp18)
            temp19=np.asarray(temp19)
            temp20=np.asarray(temp20)             
            
            #fill array position
            #print(np.shape(temp1))
            #print(np.shape(self.xc[0:ind,i]))
            self.xc[0:ind,i]=np.transpose(temp1)
            self.yc[0:ind,i]=np.transpose(temp2)
            self.zc[0:ind,i]=np.transpose(temp3)
    
            # Fill array forces
            self.Fcx[0:ind,i]=np.transpose(temp4)
            self.Fcy[0:ind,i]=np.transpose(temp5)
            self.Fcz[0:ind,i]=np.transpose(temp6)  
            self.AID[0:ind,i]=np.transpose(temp7)
            self.BID[0:ind,i]=np.transpose(temp8)
            
            self.Fcx2[0:ind,i]=np.transpose(temp9)
            self.Fcy2[0:ind,i]=np.transpose(temp10)
            self.Fcz2[0:ind,i]=np.transpose(temp11) 
            
            
            self.Dirxx_[0:ind,i]=np.transpose(temp12)
            self.Dirxy_[0:ind,i]=np.transpose(temp13)
            self.Dirxz_[0:ind,i]=np.transpose(temp14)
            
            self.Diryx_[0:ind,i]=np.transpose(temp15)
            self.Diryy_[0:ind,i]=np.transpose(temp16)
            self.Diryz_[0:ind,i]=np.transpose(temp17)
            
            self.Dirzx_[0:ind,i]=np.transpose(temp18)
            self.Dirzy_[0:ind,i]=np.transpose(temp19) 
            self.Dirzz_[0:ind,i]=np.transpose(temp20) 

            
    def export_data(self):  
        '''Export Bot positions '''
        
        #### Control forces
        file_name=self.results_dir+'/control_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.F_controller_x.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.F_controller_z.items():
                w.writerow([key, *val]) 
                    
        #### Bot positions
        file_name=self.results_dir+'/bot_position.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xposition.items():
                w.writerow([key, *val])
            
        # write y position to csv file    
            for key, val in self.bot_yposition.items():
                w.writerow([key, *val]) 
            
        # write z position to csv file     
            for key, val in self.bot_zposition.items():
                w.writerow([key, *val])         
        
        
        
        #### Bot total forces
        file_name=self.results_dir+'/bot_total_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xForcetotal.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.bot_yForcetotal.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.bot_zForcetotal.items():
                w.writerow([key, *val])            
        
        #### Bot Contact forces
        file_name=self.results_dir+'/bot_contact_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xForcecontact.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.bot_yForcecontact.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.bot_zForcecontact.items():
                w.writerow([key, *val])   
                



        #### Particle total forces
        file_name=self.results_dir+'/particle_total_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.particle_xForcetotal.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.particle_yForcetotal.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.particle_zForcetotal.items():
                w.writerow([key, *val])            
        
        #### Particle contact forces
        file_name=self.results_dir+'/particle_contact_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.particle_xForcecontact.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.particle_yForcecontact.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.particle_zForcecontact.items():
                w.writerow([key, *val])   





                
        #### Field values
        file_name=self.results_dir+'/field_values.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
                
            # write x position to csv file
            for key, val in self.Field_value.items():
                w.writerow([key, *val])            
                
                
                
        ####Export membrane positions 
        file_name=self.results_dir+'/membrane_position.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.skin_xposition.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.skin_yposition.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.skin_zposition.items():
                w.writerow([key, *val])    





        ####Export membrane positions
        file_name=self.results_dir+'/membrane_total_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.skin_x_total_forces.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.skin_y_total_forces.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.skin_z_total_forces.items():
                w.writerow([key, *val])    



        ####Export membrane positions
        file_name=self.results_dir+'/membrane_contact_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.skin_x_contact_forces.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.skin_y_contact_forces.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.skin_z_contact_forces.items():
                w.writerow([key, *val])    

       
             
        ####Export Bot velocities    
        file_name=self.results_dir+'/bot_velocity.csv'
        # export bot position
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xvelocity.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.bot_yvelocity.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.bot_zvelocity.items():
                w.writerow([key, *val])                   
        
        
        ####Export particle positions    
        file_name=self.results_dir+'/particle_position.csv'

        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.particle_xposition.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.particle_yposition.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.particle_zposition.items():
                w.writerow([key, *val])                 
  
        #### AN        
        file_name=self.results_dir+'/AN.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.AN.items():
                w.writerow([key,*val])
        
        # names of objects in contact        
        file_name=self.results_dir+'/BN.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.BN.items():
                w.writerow([key,*val])            

 
                
        #### contact points x
        file_name=self.results_dir+'/x_contact_points.csv' 
        savetxt(file_name,self.xc, delimiter=',')
    
        #### contact points y
        file_name=self.results_dir+'/y_contact_points.csv' 
        savetxt(file_name,self.yc, delimiter=',')
    
        #### contact points z
        file_name=self.results_dir+'/z_contact_points.csv' 
        savetxt(file_name,self.zc, delimiter=',')
    
        #### contact force x
        file_name=self.results_dir+'/x_contact_force.csv' 
        savetxt(file_name,self.Fcx, delimiter=',')
    
        #### contact force y
        file_name=self.results_dir+'/y_contact_force.csv' 
        savetxt(file_name,self.Fcy, delimiter=',')
    
        #### contact force z
        file_name=self.results_dir+'/z_contact_force.csv' 
        savetxt(file_name,self.Fcz, delimiter=',')
    
    
    
        #### contact force x
        file_name=self.results_dir+'/x_contact_force2.csv' 
        savetxt(file_name,self.Fcx2, delimiter=',')
    
        #### contact force y
        file_name=self.results_dir+'/y_contact_force2.csv' 
        savetxt(file_name,self.Fcy2, delimiter=',')
    
        #### contact force z
        file_name=self.results_dir+'/z_contact_force2.csv' 
        savetxt(file_name,self.Fcz2, delimiter=',')    
    
        #### contact_dirxx
        file_name=self.results_dir+'/contact_dirxx.csv' 
        savetxt(file_name,self.Dirxx_, delimiter=',')     
    
    
        #### contact_dirxy
        file_name=self.results_dir+'/contact_dirxy.csv' 
        savetxt(file_name,self.Dirxy_, delimiter=',')       
    
        #### contact_dirxz
        file_name=self.results_dir+'/contact_dirxz.csv' 
        savetxt(file_name,self.Dirxz_, delimiter=',')  
        
        #### contact_diryx
        file_name=self.results_dir+'/contact_diryx.csv' 
        savetxt(file_name,self.Diryx_, delimiter=',')     
    
        #### contact_diryy
        file_name=self.results_dir+'/contact_diryy.csv' 
        savetxt(file_name,self.Diryy_, delimiter=',')       
    
        #### contact_diryz
        file_name=self.results_dir+'/contact_diryz.csv' 
        savetxt(file_name,self.Diryz_, delimiter=',')         
        
        #### contact_dirzx
        file_name=self.results_dir+'/contact_dirzx.csv' 
        savetxt(file_name,self.Dirzx_, delimiter=',')     
    
        #### contact_dirzy
        file_name=self.results_dir+'/contact_dirzy.csv' 
        savetxt(file_name,self.Dirzy_, delimiter=',')       
    
        #### contact_dirzz
        file_name=self.results_dir+'/contact_dirzz.csv' 
        savetxt(file_name,self.Dirzz_, delimiter=',')  
         
        #### number of contacts
        file_name=self.results_dir+'/number_contacts.csv' 
        savetxt(file_name,self.number_contacts, delimiter=',')
        
        #### contact A id
        file_name=self.results_dir+'/AID.csv' 
        savetxt(file_name,self.AID, delimiter=',')
                    
        #### contact B id
        file_name=self.results_dir+'/BID.csv' 
        savetxt(file_name,self.BID, delimiter=',') 
        
        
        #### Time contact
        file_name=self.results_dir+'/time_contact.csv' 
        savetxt(file_name,self.time_contact, delimiter=',')        
        
        if self.control_mode=="grasping":
            
            #### Export ball positions        
            file_name=self.results_dir+'/ball_position.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.ball_xposition.items():
                    w.writerow([key, *val])
                
                # write z position to csv file     
                for key, val in self.ball_zposition.items():
                    w.writerow([key, *val])              
                    
            #### Export ball velocity        
            file_name=self.results_dir+'/ball_velocity.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.ball_xvelocity.items():
                    w.writerow([key, *val])

                # write z position to csv file     
                for key, val in self.ball_zvelocity.items():
                    w.writerow([key, *val])    
                    
            #### Export ball angles      
            file_name=self.results_dir+'/ball_angle.csv'

            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.bq0.items():
                    w.writerow([key, *val])
                
                # write z position to csv file     
                for key, val in self.bq1.items():
                    w.writerow([key, *val])     
                    
                # write z position to csv file     
                for key, val in self.bq2.items():
                    w.writerow([key, *val])                
           
                # write z position to csv file     
                for key, val in self.bq3.items():
                    w.writerow([key, *val])                      
           
            #### Export ball contact force      
            file_name=self.results_dir+'/ball_contact_forces.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.bFx.items():
                    w.writerow([key, *val])
                
                      
                    
                # write z position to csv file     
                for key, val in self.bFz .items():
                    w.writerow([key, *val])                       
                    
                    
            #### ball total force       
            file_name=self.results_dir+'/ball_total_forces.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.bFtx.items():
                    w.writerow([key, *val])
                
                # write y position to csv file     
                for key, val in self.bFty.items():
                    w.writerow([key, *val])                        
                    
                # write z position to csv file     
                for key, val in self.bFtz.items():
                    w.writerow([key, *val])                       
                    
                    
                    
            #### Pull Force        
            file_name=self.results_dir+'/pull_force.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.TIME.items():
                    w.writerow([key,*val])
                    
                for key, val in self.PX.items():
                    w.writerow([key,*val])  
                    
                for key, val in self.PY.items():
                    w.writerow([key,*val])  
                    
                # write x position to csv file
                for key, val in self.Fb.items():
                    w.writerow([key, *val])

            #### Time contact
            file_name=self.results_dir+'/THETA.csv' 
            savetxt(file_name,self.THETA, delimiter=',') 
            
            #### Time contact
            file_name=self.results_dir+'/Rr_.csv' 
            savetxt(file_name,self.Rr_, delimiter=',')  
            
            
class MyReportContactCallback(chrono.ReportContactCallback):
    """ Class for reporting and storing the the contact forces and postions  """
    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Contact_force_x=[]
        self.Contact_force_y=[]
        self.Contact_force_z=[]
        
        self.Contact_force_x2=[]
        self.Contact_force_y2=[]
        self.Contact_force_z2=[]
        
        self.Contact_points_x= []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]



        self.dir_xx=[]
        self.dir_xy=[]
        self.dir_xz=[]
        
        
        self.dir_yx=[]
        self.dir_yy=[]
        self.dir_yz=[]      
        
        self.dir_zx=[]
        self.dir_zy=[]
        self.dir_zz=[]



        self.CA=[]
        
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        
        IDA = bodyUpA.GetId()
        IDB = bodyUpB.GetId()
        cA=np.asarray(cA.GetMatr())
        self.CA.append(cA)
        self.Contact_points_x.append(vA.x)
        self.Contact_points_y.append(vA.y)
        self.Contact_points_z.append(vA.z)
        

        
        self.Contact_force_x.append(force.x)
        self.Contact_force_y.append(force.y)
        self.Contact_force_z.append(force.z)
        
        
        force=np.array([force.x,force.y,force.z])
        #print("force",force)
        #print("cA",cA)
        force=np.matmul(cA,force)
        #print("force",force)
        self.Contact_force_x2.append(force[0])
        self.Contact_force_y2.append(force[1])
        self.Contact_force_z2.append(force[2])
        
        dirx=np.matmul(cA,np.array([1,0,0]))
        diry=np.matmul(cA,np.array([0,1,0]))        
        dirz=np.matmul(cA,np.array([0,0,1]))
        
        
        #print(dirx,diry,dirz)
        
        #print(dirx.x)
        self.dir_xx.append(dirx[0])
        self.dir_xy.append(dirx[1])
        self.dir_xz.append(dirx[2])
        
        
        self.dir_yx.append(diry[0])
        self.dir_yy.append(diry[1])
        self.dir_yz.append(diry[2])       
        
        self.dir_zx.append(dirz[0])
        self.dir_zy.append(dirz[1])
        self.dir_zz.append(dirz[2])        
        
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        
        self.bodiesA_ID.append(IDA)
        self.bodiesB_ID.append(IDB)
        
        
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.Contact_force_x = []
        self.Contact_force_y = []
        self.Contact_force_z = []
        
        self.Contact_force_x2 = []
        self.Contact_force_y2 = []
        self.Contact_force_z2 = [] 
        
        self.Contact_points_x = []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]
        self.CA=[]
        
        self.dir_xx=[]
        self.dir_xy=[]
        self.dir_xz=[]
        
        
        self.dir_yx=[]
        self.dir_yy=[]
        self.dir_yz=[]      
        
        self.dir_zx=[]
        self.dir_zy=[]
        self.dir_zz=[]
        
    # Get the points
    def GetList(self):
        return (self.Contact_points_x, # 0
                self.Contact_points_y, # 1
                self.Contact_points_z, # 2
                self.Contact_force_x,  # 3
                self.Contact_force_y,  # 4
                self.Contact_force_z,  # 5
                self.Contact_force_x2, # 6
                self.Contact_force_y2, # 7
                self.Contact_force_z2, # 8               
                self.bodiesA,          # 9
                self.bodiesB,          #10     
                self.bodiesA_ID,       #11
                self.bodiesB_ID,       #12
                self.CA,               #13
                self.dir_xx,           #14
                self.dir_xy,           #15
                self.dir_xz,           #16
                self.dir_yx,           #17
                self.dir_yy,           #18
                self.dir_yz,           #19
                self.dir_zx,           #20
                self.dir_zy,           #21
                self.dir_zz)           #22




class R_functions():  
    """ R-function Class """
    def __init__(self,name):
        self.direct = os.path.dirname(__file__)
        self.name = name
        ###### Imported Variables #########
        self.mainDirectory = self.direct+"/Experiments/"
        parameters = np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.control_mode=self.parameters['control_mode']  # control mode
        self.m = 8
        ######### SHAPE FORMATION #########
        if self.control_mode=="shape_formation":
            self.geometry = self.parameters['geometry'] 

            if self.geometry=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['a']
            
            if self.geometry=='pacman':
                self.a = self.parameters['xcenter']
                self.b = self.parameters['zcenter']
                self.R = self.parameters['a']
                self.segments = np.array([[self.a,self.b,self.R*np.cos(np.pi/4),self.R*np.sin(np.pi/4)],[self.a,self.b,self.R*np.cos(-np.pi/4),self.R*np.sin(-np.pi/4)]])
                theta=np.linspace(np.pi/4,7*np.pi/4,100)
                self.xp=[]
                self.yp=[]
                for i in range(len(theta)):
                    
                    self.xp.append(self.R*np.cos(theta[i])+self.a)
                    self.yp.append(self.R*np.sin(theta[i])+self.b)
                
                self.xp.append(self.R*np.cos(-np.pi/4))
                self.xp.append(self.a)
                self.xp.append(self.R*np.cos(np.pi/4))
                
            
                self.yp.append(self.R*np.sin(-np.pi/4))
                self.yp.append(self.a)
                self.yp.append(self.R*np.sin(np.pi/4))                
                
                
                np.savez(self.mainDirectory+'/'+self.name+'/'+'outline'+self.geometry+'.npz',xp=self.xp,yp=self.yp)          
            else:
                data = np.load(self.direct+'/shapes/'+self.geometry+'.npz')
                self.segments = data['segments']
                self.scale = self.parameters['scale']
                self.segments=self.segments


            
        ######### SHAPE MORPHING #########
        if self.control_mode=="shape_morphing":  
            self.p = self.parameters['p']
            self.geometry1 = self.parameters['geometry1'] 
            self.geometry2 = self.parameters['geometry2'] 
            self.scale1 = self.parameters['scale1']
            self.scale2 = self.parameters['scale2']
            
        
            
            if self.geometry1=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']
                
                
            if self.geometry2=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']


            if self.geometry1!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry1+'.npz')
                self.segments = data['segments']
                self.segments = self.scale1*self.segments                   
                
            
            if self.geometry2!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry2+'.npz')
                self.segments = data['segments']
                np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes'+self.geometry2+'.npz',segments=self.segments,xp=data['xp'],yp=data['yp'])          
               
        if self.control_mode=="grasping":

            self.segments = 0
            self.a1 = self.parameters['a1']
            self.b1 = self.parameters['b1']
            self.xc1 = self.parameters['xc1']     
            self.yc1 = self.parameters['yc1']
            
            self.a2 = self.parameters['a2']
            self.b2 = self.parameters['b2']
            self.xc2 = self.parameters['xc2']     
            self.yc2 = self.parameters['yc2']    
            self.tcut1 = self.parameters['tcut1'] 
            self.tcut2 = self.parameters['tcut2'] 
            self.tcut3 = self.parameters['tcut3'] 
##############################################################################            
            

    
    def phi_oval(self,x,y,xc,yc,a,b):
        phi=((x-xc)/a)**2 + ((y-yc)/b)**2 - 1 
        return(abs(phi))     
        
    def dphix_oval(self,x,y,xc,yc,a,b):
        phi=((x-xc)/a)**2 + ((y-yc)/b)**2 - 1 
        return(np.sign(phi) * (2*x - 2*xc) / a**2)

    def dphiy_oval(self,x,y,xc,yc,a,b):
        phi=((x-xc)/a)**2 + ((y-yc)/b)**2 - 1 
        return(np.sign(phi) * (2*y - 2*yc) / b**2)
    
    
    
    
    def phi_circle(self,x,y,a,b,R):
        """ Normalized distance function of a circle """
        phi = (R**2 - (x-a)**2 - (y-b)**2)
        return(abs(phi))
        
    def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def phi_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)

        T=self.phi_line_(x,y,x1,y1,x2,y2)
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        
        phi1=self.Trim(f,T)
        
        return(phi1)
    
    
    def dphix_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt x  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Tx=self.dphix_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fx=self.dphix_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimx(f,T,fx,Tx)
        
        return(phi1)    
    

    def dphiy_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt y  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Ty=self.dphiy_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fy=self.dphiy_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimy(f,T,fy,Ty)
        
        return(phi1)


    
    
    def Trim(self,f,t):
        """ Trim function for two functions  """
        phi=np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((phi-t)/2)**2))

    def Trimx(self,f,t,fx,tx):
        """Derivative Trim function for two functions wrt x  """
        term1 = (2*(f**3)*fx + tx*t)/(np.sqrt(f**4 + t**2)) - tx
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fx
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)
        
    def Trimy(self,f,t,fy,ty):
        """Derivative Trim function for two functions wrt y """
        term1 = (2*(f**3)*fy + ty*t)/(np.sqrt(f**4 + t**2)) - ty
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fy
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)    
    
##############################################################################   

    def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


    def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


    def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


    def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


    def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

    def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   


    def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)

    def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) * self.dphix_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)

    def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) * self.dphiy_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m))**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term3      
        R=(-term1*term2/term3)
        return(R)
    
    
    
    def FGRASP(self,x,y,t):
        if self.tcut1>t:
            phi=self.phi_oval(x,y,self.xc1,self.yc1,self.a1,self.b1)
        if self.tcut1<t:
            phi=self.phi_oval(x,y,self.xc2,self.yc2,self.a2,self.b2)
        return(phi)
    
    def FXGRASP(self,x,y,t):
        if self.tcut1>=t:
            phix=self.dphix_oval(x,y,self.xc1,self.yc1,self.a1,self.b1)
        if self.tcut1<t:
            phix=self.dphix_oval(x,y,self.xc2,self.yc2,self.a2,self.b2)
        return(phix)

    def FYGRASP(self,x,y,t):
        if self.tcut1>=t:
            phiy=self.dphiy_oval(x,y,self.xc1,self.yc1,self.a1,self.b1)
        if self.tcut1<t:
            phiy=self.dphiy_oval(x,y,self.xc2,self.yc2,self.a2,self.b2)
        return(phiy)    
    
    
    def F_object(self,x,y):
        R=3
        theta1=0.13718011
        theta2=2*np.pi-0.13718011
        theta=np.linspace(theta1,theta2,100)
        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)
        
        
    
    def FX(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
            
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1x = self.dphix_quarter_circle(x,y,self.a,self.b,self.R)
            
            f2 = self.phi_segments(x,y,self.segments)
            f2x = self.dphix_segments(x,y,self.segments)
            
            Fx = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2x + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1x - ((self.m*(f2**self.m)*f2x)/f2 + (self.m*(f1**self.m)*f1x)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def FY(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1y = self.dphiy_quarter_circle(x,y,self.a,self.b,self.R)
        
            f2 = self.phi_segments(x,y,self.segments)
            f2y = self.dphiy_segments(x,y,self.segments)
            
            Fy = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2y + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1y - ((self.m*(f2**self.m)*f2y)/f2 + (self.m*(f1**self.m)*f1y)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m +f2**self.m))
            
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    
    
    
    def F(self,x,y):
       """ Single function to call field"""
       if self.geometry=='circle':
           F = self.phi_circle(x,y,self.a,self.b,self.R)
            
       if self.geometry=='pacman':
           f = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
           f2=self.phi_segments(x,y,self.segments)
           F = (f*f2)/((f**self.m +f2**self.m)**(1/self.m))
           
         
       else:
           F = self.phi_segments(x,y,self.segments)
           
       return(F)

       
##################################################################################    
    
#### MORPHING FUNCTIONS
   
    def F_morph(self,x,y,ft):
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        #print(phi1,phi2,ft)     
        F = self.C_morph(phi1,phi2,ft)
        return(F)
    

    def FX_morph(self,x,y,ft):
        """ Control for x component for morphing """
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        
        dphi1x = self.F1X(x,y)
        dphi2x = self.F2X(x,y)
        
        Fx = self.C_morphx(phi1,phi2,dphi1x,dphi2x,ft)
        
        
        return(Fx)

    def FY_morph(self,x,y,ft):
        """ Control for y component for morphing """
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        
        dphi1y = self.F1Y(x,y)
        dphi2y = self.F2Y(x,y)
        
        Fy = self.C_morphy(phi1,phi2,dphi1y,dphi2y,ft)
        
    
        return(Fy)
    
    

    def F1(self,x,y):
        """ Field of the initial starting field """
        if self.geometry1=='circle':
            F1 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F1 = self.dphix_segments(x,y,self.segments1)
    
        return(F1)        
        
    def F2(self,x,y):
        """ Field of the desired  field """
        if self.geometry2=='circle':
            F2 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F2 = self.phi_segments(x,y,self.segments)
    
        return(F2)      


    def F1X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry1=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F1Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry1=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    


    def F2X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry2=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F2Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry2=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)  


    def tanh(self,t):
        """ tanh function """
        tanh=(np.exp(self.p*(t))-1)/(np.exp(self.p*(t))+1)
        #print('tanh=',tanh)
        return(tanh)   
    
    
    def g1(self,phi1,t):
        """ intersection of initial field and -f(t) """
        return(phi1 - t - np.sqrt(phi1**2 + t**2))
        
    def g2(self,phi2,t):
        """ intersection of final field and f(t)-1 """
        return(phi2 + (t-1) - np.sqrt(phi2**2+(t-1)**2))
    
    
    def dgx(self,phi,dphix,s):
        """ derivative of g1 or g2 wrt x """
        # s is either (t) or t-1
        return(dphix-(phi*dphix)/(np.sqrt(s**2 +phi**2)))
    
    
    def dgy(self,phi,dphiy,s):
        """ derivative of g1 or g2 wrt y """
        return(dphiy-(phi*dphiy)/(np.sqrt(s**2 +phi**2)))    
    
    
    def w1(self,g1,g2): 
        """ Weighted function 1 """
        return(g2/(g1+g2))
    
    
    def w2(self,g1,g2):
        """ Weighted function 2 """
        return(g1/(g1+g2))    
    
  
    def dw1x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 1 wrt x """
        return((dg2x) / (g1+g2) - (dg1x+dg2x)*g2 / (g1+g2)**2)
    
    
    def dw2x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 2 wrt x """
        return((dg1x) / (g1+g2) - (dg1x+dg2x)*g1 / (g1+g2)**2)
    
    
    def dw1y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 1 wrt y """
        return((dg2y) / (g1+g2) - (dg1y+dg2y)*g2 / (g1+g2)**2)
    
    
    def dw2y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 2 wrt y """
        return((dg1y) / (g1+g2) - (dg1y+dg2y)*g1 / (g1+g2)**2)  
    
  

    def C_morph(self,phi1,phi2,ft):
        """ Morphing function from phi1 to phi2 """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        return(W1*phi1+W2*phi2)
    
    

    def C_morphx(self,phi1,phi2,dphi1x,dphi2x,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt x """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1x = self.dgx(phi1,dphi1x,ft)
        dg2x = self.dgx(phi2,dphi2x,ft-1)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        DW1X = self.dw1x(dg1x,dg2x,G1,G2)
        DW2X = self.dw2x(dg1x,dg2x,G1,G2)
        
        return(phi1*DW1X + phi2*DW2X + W1*dphi1x + W2*dphi2x)
    
    def C_morphy(self,phi1,phi2,dphi1y,dphi2y,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt y """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1y = self.dgy(phi1,dphi1y,ft)
        dg2y = self.dgy(phi2,dphi2y,ft-1)
    
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        DW1Y = self.dw1y(dg1y,dg2y,G1,G2)
        DW2Y = self.dw2y(dg1y,dg2y,G1,G2)
        
        return(phi1*DW1Y + phi2*DW2Y + W1*dphi1y + W2*dphi2y)


    def dpoint_(self,x,y,xc,yc):
        return((np.sqrt((x-xc)**2 + (y-yc)**2)))

    def dxpoint_(self,x,y,xc,yc):
        return((x-xc)/(np.sqrt((x-xc)**2 + (y-yc)**2)))

    def dypoint_(self,x,y,xc,yc):
        return((y-yc)/(np.sqrt((x-xc)**2 + (y-yc)**2)))

    def F_point(self,x,y,xc,yc):
        d=self.dpoint_(x,y,xc,yc)
        return(0.5*d**2)

    def Fx_point(self,x,y,xc,yc):
        dx=self.dxpoint_(x,y,xc,yc)
        d=self.dpoint_(x,y,xc,yc)
        return(dx*d)

    def Fy_point(self,x,y,xc,yc):
        dy=self.dypoint_(x,y,xc,yc)
        d=self.dpoint_(x,y,xc,yc)
        return(dy*d)

    def create_segment(self,x,y):
        """ Create segment matrix for points of R-function """
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
 
    
    def plot_zero_contours(self,xp1,yp1,xp2,yp2,d):
        xticks = np.linspace(-d, d,5,endpoint=True)
        fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3))
        fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)         
        axs.plot(xp1,yp1,color='k',linewidth=3)
        axs.plot(xp2,yp2,color='tab:red',linewidth=3)
        
        
    def plot_R_function(self,X,Y,R,Rx,Ry,d):
        """ Plot R-function and its derivatives """
        xticks = np.linspace(-d, d,5,endpoint=True)
        fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(10,3))
        fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)
        
        # Plot Phi
        im1=axs[0].contourf(X, Y,R,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        axs[0].contour(X,Y,R,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        #axs[0].plot(xp,yp,color='k',linewidth=3)
        axs[0].set_title('$\phi(x)$')
        axs[0].set_xticks(xticks)
        axs[0].set_yticks(xticks)
        fig.colorbar(im1, ax=axs[0])
        
        # Plot Phix
        im2=axs[1].contourf(X, Y,Rx,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[1].plot(xp,yp,color='k',linewidth=3)
        axs[1].contour(X,Y,Rx,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[1].set_xticks(xticks)
        axs[1].set_yticks(xticks)
        axs[1].set_title(r"$\frac{\partial \phi}{\partial x}$")
        fig.colorbar(im2, ax=axs[1])
        
        # Plot Phiy
        im3=axs[2].contourf(X,Y,Ry,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[2].plot(xp,yp,color='k',linewidth=3)
        axs[2].contour(X,Y,Ry,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[2].set_xticks(xticks)
        axs[2].set_yticks(xticks)
        axs[2].set_title(r"$\frac{\partial \phi}{\partial y}$")
        fig.colorbar(im3, ax=axs[2])
         
        
        
    def plot_R_function_morph(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1y,dphi2y,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks
        for i in range(nr):
            for j in range(nc):
                #print(np.min(C[count]))
                CS1 = axs[i,j].contour(X,Y,CX[count],levels = [0],colors=('tab:red'),linestyles=('-',),linewidths=(3,))
                CS2 = axs[i,j].contour(X,Y,CY[count],levels = [0],colors=('tab:blue'),linestyles=('--',),linewidths=(3,))
                #(xi,yi)=find_intersection(CS1,CS2)
                #axs[i,j].plot(xi,yi,'ko', ms=3)
                #CS2 = axs[i,j].contour(X,Y,CX[count],5,colors='black',zorder=0)
                #axs[i,j].set_xlim([-d,d])
                #axs[i,j].set_ylim([-d,d])
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
        fig.tight_layout()
        
        
        
    def plot_R_function_morph_color(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1x,dphi2x,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks        
        for i in range(nr):
            for j in range(nc):
                print(np.min(CX[count]))
                #axs[i,j].axis('equal')
                #CS1 = axs[i,j].contour(X,Y,C[count],linewidths=(1,))
                im1=axs[i,j].contourf(X, Y,C[count],cmap = 'jet',levels=50,alpha=1,linestyles='solid')  
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
                fig.colorbar(im1, ax=axs[i,j])
        plt.tight_layout()        



def plot_line(xp,yp):  
    fm._rebuild()
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'dejavuserif'
    plt.rcParams['font.size'] = 6
    plt.rcParams['axes.linewidth'] = .1
    #xticks = np.linspace(-d, d,5,endpoint=True)
    fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(10,3))
    fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)
    axs.plot(xp,yp,linewidth=1,color='k')
        
        
def shoelace(vertices):
    """ Find the cross sectional area"""
    (m,n)=np.shape(vertices)

    sum1=vertices[0,0]*(vertices[1,1]-vertices[1,n-1])
    for i in range(1,n-1):
        sum1=sum1 + vertices[0,i]*(vertices[1,i+1]-vertices[1,i-1])
    i=n-1
    sum1=sum1 + vertices[0,i]*(vertices[1,0]-vertices[1,i-1])

    A=.5*abs(sum1)
    return (A)            


def create_segment(x,y):
    """ Create segment matrix for points of R-function """
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





class import_data:
     def __init__(self,name,path,wxmin,wxmax,wymin,wymax,Psi=None):
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
         self.delta = .1
         self.xx = np.arange(self.wxmin,self.wxmax,self.delta)
         self.yy = np.arange(self.wymin,self.wymax,self.delta)
         self.X,self.Y = np.meshgrid(self.xx,self.yy)
         d=1
         self.m=8
         self.wxmin2 = -d
         self.wxmax2 = d
         self.wymin2 = -d
         self.wymax2 = d

         self.Psi=Psi
         self.parameters=parameters.tolist()    # loads saved parameters        
         self.nb=self.parameters['nb'] # number of bots
         self.ni=self.parameters['total_particles']
         self.ns=self.parameters['ns']
         self.nm=self.nb*self.ns # total membrane particles 
         self.bot_width=self.parameters['bot_width']
         self.geom = self.parameters['ball_geometry']
         self.particle_width=self.parameters['particle_width']
         self.radius2=self.particle_width/2 # radius of particles
         self.control_mode=self.parameters['control_mode']
         self.skin_width=self.parameters['skin_width']
         self.height=self.parameters['bot_height']
         
         # if control mode is shape formation
         if self.control_mode=="shape_formation":
             self.geometry = self.parameters['geometry']
             data2=np.load(self.mainDirectory+self.name+'/outline'+self.geometry+'.npz',allow_pickle=True)
             self.xp=data2['xp']
             self.yp=data2['yp']
         
             
         # if control mode is shape morphing
         if self.control_mode=="shape_morphing":
             self.geometry1=self.parameters['geometry1']
             self.geometry2=self.parameters['geometry2']
             data2=np.load(self.mainDirectory+self.name+'/shapes'+self.geometry2+'.npz',allow_pickle=True)
             self.xp=data2['xp']
             self.yp=data2['yp']
         
         # if control mode is grasping  
         if self.control_mode=="grasping":
             self.a2 = self.parameters['a2']
             self.b2 = self.parameters['b2']
             self.xc2 = self.parameters['xc2']     
             self.yc2 = self.parameters['yc2'] 
            
            
         self.path=self.path+self.name+"/results/"
         os.chdir(self.path)
         self.files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
         
         
         #### Robot Position
         self.bot_position=np.genfromtxt(self.files[self.files.index('bot_position.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_position)
         self.bot_position=self.bot_position[:,1:self.n1]
         self.time=self.bot_position[0,:]
         self.bot_position_x=self.bot_position[1:self.nb+1,:]
         self.bot_position_y=self.bot_position[self.nb+1:2*self.nb+1,:]
         self.bot_position_z=self.bot_position[(2*self.nb)+1:3*self.nb+1,:] 

         #### robot contact forces
         self.bot_contact_forces=np.genfromtxt(self.files[self.files.index('bot_contact_forces.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_contact_forces)
         self.bot_contact_forces=self.bot_contact_forces[:,1:self.n1]
         self.bot_contact_forces_x=self.bot_contact_forces[1:self.nb+1,:]
         self.bot_contact_forces_y=self.bot_contact_forces[self.nb+1:2*self.nb+1,:]
         self.bot_contact_forces_z=self.bot_contact_forces[(2*self.nb)+1:3*self.nb+1,:]          


         #### robot total forces
         self.bot_total_forces=np.genfromtxt(self.files[self.files.index('bot_total_forces.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_total_forces)
         self.bot_total_forces=self.bot_total_forces[:,1:self.n1]
         self.bot_total_forces_x=self.bot_total_forces[1:self.nb+1,:]
         self.bot_total_forces_y=self.bot_total_forces[self.nb+1:2*self.nb+1,:]
         self.bot_total_forces_z=self.bot_total_forces[(2*self.nb)+1:3*self.nb+1,:]          


         #### membrane_positions
         self.membrane_position=np.genfromtxt(self.files[self.files.index('membrane_position.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.membrane_position)
         self.membrane_position=self.membrane_position[:,1:n]
         self.membrane_position_x=self.membrane_position[1:self.nm+1,:]
         self.membrane_position_y=self.membrane_position[self.nm+1:2*self.nm+1,:]
         self.membrane_position_z=self.membrane_position[(2*self.nm)+1:3*self.nm+1,:]          
         
         
         
         #### control forces
         self.control_forces=np.genfromtxt(self.files[self.files.index('control_forces.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.control_forces)
         self.control_forces=self.control_forces[:,1:n]
         self.control_forces_x=self.control_forces[1:self.nb+1,:]
         self.control_forces_z=self.control_forces[self.nb+1:2*self.nb+1,:]
       
        
         #### Ball contact forces
         # These were saved directly from chrono.; It calculuaedc the sum of contact forces on the ball
         self.ball_contact_forces=np.genfromtxt(self.files[self.files.index('ball_contact_forces.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.ball_contact_forces)
         self.ball_contact_forces=self.ball_contact_forces[:,1:n]
         self.bFx=self.ball_contact_forces[1,:]
         self.bFy=self.ball_contact_forces[2,:]
         #self.bFz=self.ball_contact_forces[3,:]
         
         
         
         
         #### Potential Field values    
         self.Field_value=np.genfromtxt(self.files[self.files.index('field_values.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.Field_value)
         self.Field_value=self.Field_value[:,1:n]
         self.Field_value_sum=[]
         for i in range(len(self.time)):
             self.Field_value_sum.append(np.sum(abs(self.Field_value[:,i])))
         
         #### Particle Position    
         if self.ni==0:
             pass
         else:
             self.particle_position=np.genfromtxt(self.files[self.files.index('particle_position.csv') ] ,delimiter=',')
             (self.m4a,self.n4a)=np.shape(self.particle_position)
             self.particle_position=self.particle_position[:,1:self.n4a]
             self.particle_position_x=self.particle_position[1:self.ni+1,:]
             self.particle_position_y=self.particle_position[self.ni+1:2*self.ni+1,:]
             self.particle_position_z=self.particle_position[(2*self.ni)+1:3*self.ni+1,:]
         
             #### particle contact forces
             #these were calulculated by chrono
             self.particle_contact_forces=np.genfromtxt(self.files[self.files.index('particle_contact_forces.csv') ] ,delimiter=',')
             (self.m1,self.n1)=np.shape(self.particle_contact_forces)
             self.particle_contact_forces=self.particle_contact_forces[:,1:self.n1]
             self.particle_contact_forces_x=self.particle_contact_forces[1:self.ni+1,:]
             self.particle_contact_forces_y=self.particle_contact_forces[self.ni+1:2*self.ni+1,:]
             self.particle_contact_forces_z=self.particle_contact_forces[(2*self.ni)+1:3*self.ni+1,:]          


             #### particle total forces 
             self.particle_total_forces=np.genfromtxt(self.files[self.files.index('particle_total_forces.csv') ] ,delimiter=',')
             (self.m1,self.n1)=np.shape(self.particle_total_forces)
             self.particle_total_forces=self.particle_total_forces[:,1:self.n1]
             self.particle_total_forces_x=self.particle_total_forces[1:self.ni+1,:]
             self.particle_total_forces_y=self.particle_total_forces[self.ni+1:2*self.ni+1,:]
             self.particle_total_forces_z=self.particle_total_forces[(2*self.ni)+1:3*self.ni+1,:] 
         
         
         
         #### Contact Points and forces 
         # These were collected from chrono.
         self.time_contact = np.genfromtxt(self.files[self.files.index('time_contact.csv') ] ,delimiter=',')
         self.number_contacts = np.genfromtxt(self.files[self.files.index('number_contacts.csv') ] ,delimiter=',')
        
         #### Contact points 
         self.Contact_points_x = np.genfromtxt(self.files[self.files.index('x_contact_points.csv') ] ,delimiter=',')
         self.Contact_points_y = np.genfromtxt(self.files[self.files.index('y_contact_points.csv') ] ,delimiter=',')
         self.Contact_points_z = np.genfromtxt(self.files[self.files.index('z_contact_points.csv') ] ,delimiter=',')
        
         #### Contact forces
         # These were the ones in refenece to the local frame of the contact
         self.Contact_force_x = np.genfromtxt(self.files[self.files.index('x_contact_force.csv') ] ,delimiter=',')
         self.Contact_force_y = np.genfromtxt(self.files[self.files.index('y_contact_force.csv') ] ,delimiter=',')
         self.Contact_force_z = np.genfromtxt(self.files[self.files.index('z_contact_force.csv') ] ,delimiter=',') 
         
         #### Contact forces 2
         # These were the ones in refenece to the global frame 
         self.x_contact_force2=np.genfromtxt(self.files[self.files.index('x_contact_force2.csv') ] ,delimiter=',') 
         self.y_contact_force2=np.genfromtxt(self.files[self.files.index('y_contact_force2.csv') ] ,delimiter=',') 
         self.z_contact_force2=np.genfromtxt(self.files[self.files.index('z_contact_force2.csv') ] ,delimiter=',') 
        
         #### Contact body ID's
         # These were the ID's of the bodys that were in contact 
         self.AID = np.genfromtxt(self.files[self.files.index('AID.csv') ] ,delimiter=',')  # Body A
         self.BID = np.genfromtxt(self.files[self.files.index('BID.csv') ] ,delimiter=',')  # Body B
         
         # We are now sorting them into lists. Its makes it easier to access. 
         self.AN=[] # empty array of contact ID A
         self.BN=[] # empty array of contact ID B
         infile = open(self.files[self.files.index('AN.csv') ], 'r') # now we fill the array AN
         for row in csv.reader(infile):
             self.AN.append(row[1:])
         infile = open(self.files[self.files.index('BN.csv') ], 'r') # now we fill the array BN
         for row in csv.reader(infile):
             self.BN.append(row[1:])         
         
         #### Contact frame dir x
         # These are the global vecotrs of the x direction of the contact frame 
         self.Dirxx_=np.genfromtxt(self.files[self.files.index('contact_dirxx.csv') ] ,delimiter=',') 
         self.Dirxy_=np.genfromtxt(self.files[self.files.index('contact_dirxy.csv') ] ,delimiter=',') 
         self.Dirxz_=np.genfromtxt(self.files[self.files.index('contact_dirxz.csv') ] ,delimiter=',') 
         
         #### Contact frame dir y
         # These are the global vecotrs of the y direction of the contact frame 
         self.Diryx_=np.genfromtxt(self.files[self.files.index('contact_diryx.csv') ] ,delimiter=',') 
         self.Diryy_=np.genfromtxt(self.files[self.files.index('contact_diryy.csv') ] ,delimiter=',') 
         self.Diryz_=np.genfromtxt(self.files[self.files.index('contact_diryz.csv') ] ,delimiter=',') 

         #### Contact frame dir z
         # These are the global vecotrs of the z direction of the contact frame 
         self.Dirzx_=np.genfromtxt(self.files[self.files.index('contact_dirzx.csv') ] ,delimiter=',') 
         self.Dirzy_=np.genfromtxt(self.files[self.files.index('contact_dirzy.csv') ] ,delimiter=',') 
         self.Dirzz_=np.genfromtxt(self.files[self.files.index('contact_dirzz.csv') ] ,delimiter=',')       
        
         self.MAG_pressure=np.zeros((self.nb+self.ni,len(self.time)-1))
         self.MAG_pressure_no_boundary=np.zeros((self.ni,len(self.time)-1))
         
         self.Mag_avg_pressure=[]
         self.Mag_avg_pressure_no_boundary=[]
         # If the control modeis set for grasping. 
         if self.control_mode=="grasping":
             self.geom = self.parameters['ball_geometry'] 
             self.ball_radius = self.parameters['ball_radius']
             self.mu = self.parameters['lateralFriction']
             self.ballx = self.parameters['ballx'] 
             self.ballz = self.parameters['ballz'] 
             self.xc2 = self.parameters['xc2']
             self.yc2 = self.parameters['yc2']
             const=self.ball_radius*2*np.pi/4
             rx=const
             ry=const
             w=rx
             h=ry
             xcenter=self.ballx
             ycenter=self.ballz
             x=[w,-w,-w,w,w]
             y=[h,h,-h,-h,h]
             self.m=8
             x=x + xcenter*np.ones(len(x))
             y=y + ycenter*np.ones(len(x))
             (self.segments)=self.create_segment(x,y)             
  
            
  
             #### Ball position 
             self.ball_position=np.genfromtxt(self.files[self.files.index('ball_position.csv') ] ,delimiter=',')            
             m,n=np.shape(self.ball_position)
             self.ball_position=self.ball_position[:,1:n]
             self.ball_position=self.ball_position[1:m,:]
             self.ballx_position=self.ball_position[0,:]
             self.ballz_position=self.ball_position[1,:]
             
             #### ball velocity
             self.ball_velocity=np.genfromtxt(self.files[self.files.index('ball_velocity.csv') ] ,delimiter=',')
             (self.m1,self.n1)=np.shape(self.ball_velocity)
             self.ball_velocity_x=self.ball_velocity[1,:]
             self.ball_velocity_z=self.ball_velocity[2,:]                      
             
             self.F_control = []
             self.Forces_ball_x = []
             self.Forces_ball_z = []
             
             self.contact_points_ball_x = []
             self.contact_points_ball_z = []
             
             self.magnitude_forces_on_ball = []  
             self.torque_ball = []

             self.THETA=np.genfromtxt(self.files[self.files.index('THETA.csv') ] ,delimiter=',')
             self.Rr_=np.genfromtxt(self.files[self.files.index('Rr_.csv') ] ,delimiter=',')
             self.angle_entries=[]
             self.epsilon_section={}
             self.epsilon_theta_section={}
             
             self.average_epsilon=[]
             self.max_epsilon=[]
             # PRESSURE empty arrays
             self.Pressure_x_bots = np.zeros((self.nb,len(self.time_contact))) # empty array for the pressure of each bot in the x direction 
             self.Pressure_z_bots = np.zeros((self.nb,len(self.time_contact))) # empty array for the pressure of each bot in the z direction
             
             self.Pressure_x_particles = np.zeros((self.ni,len(self.time_contact))) # empty array for the pressure of each particle in the x direction 
             self.Pressure_z_particles = np.zeros((self.ni,len(self.time_contact))) # empty array for the pressure of each particle in the z direction 


             
             self.Forces_x_contact_bots = np.zeros((self.nb,len(self.time_contact)))
             self.Forces_z_contact_bots = np.zeros((self.nb,len(self.time_contact)))


             #### EMPTY ARRAYs FOR CALCULATING ALL THE CONTACT FORCES ####
             
             # empty list for contact forces on particle
             self.Forces_x_contact_particles={}
             self.Forces_z_contact_particles={}
             
             # empty list for contact forces on boundary robots
             self.Forces_x_contact_bots={}
             self.Forces_z_contact_bots={}    
        
             # empty list for contact forces on ball
             self.Force_x_contact_ball={} 
             self.Force_z_contact_ball={}
        
             # empty list for contact positions on ball
             self.position_x_contact_ball={}
             self.position_z_contact_ball={}
        
             # empty list for contact directions on ball x direction
             self.dir_xx_contact_ball={}
             self.dir_xz_contact_ball={}
             
             # empty list for contact directions on ball z direction
             self.dir_zx_contact_ball={}
             self.dir_zz_contact_ball={}        
        
             # empty list for contact positions on bot
             self.position_x_contact_bot={}
             self.position_z_contact_bot={}
        
             # empty list of forces that the robot is applying on the ball 
             self.Forces_x_ball_bot={}
             self.Forces_z_ball_bot={}
        
             # empty list of positions that the robot is applying on the ball 
             self.position_x_ball_bot={}
             self.position_z_ball_bot={}
        
        
             # FILL THE EMPTY ARRAYS
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
            

             #### Version 1
             # THERESE CORRESPOND TO THE FORCES THE ROBOT IS APPLY ONLY
             self.grasp_id = [] #  id of the robots in contact applying forces  
             self.grasp_position_x = [] # positon x
             self.grasp_position_z = [] # position z
             
             self.grasp_force_x = [] # force x in contact
             self.grasp_force_z = [] # forces z in contact   
             self.grasp_torque = []   # toruqes being applied  
             
             self.WRENCHES = 0    
             self.WRENCH_NORM = 0
             
             self.G = []
             
             self.FRAMES=[] # empty array of the contact frames
              
             self.EPSILON=[] # empty array of the epsilon metric
             self.HULLWRENCHNORM=[] # empty array of the wrench norm
             self.HULLWRENCHMAGS=[] # empty array of the magnitude of the wrench magnitutudes
             
             self.framex = []
             self.framez = []

             self.FT = []
             
             self.Cplus = []
             self.Cminus = [] 
             
             self.FCplus = []
             self.FCinus = []
             
             self.F_mag = []
             self.HULL=[]
             
             self.WRENCHXY=[]
             self.HULLXY=[]
                 
             self.WRENCHXT=[]
             self.HULLXT=[]

             self.WRENCHYT=[]
             self.HULLYT=[]            

             self.ANGLE_CHECK=[]


             self.pull_data=np.genfromtxt(self.files[self.files.index('pull_force.csv') ] ,delimiter=',')            
             (self.m6a,self.n6a)=np.shape(self.pull_data)
             self.pull_data=self.pull_data[:,1:self.n6a]
             self.TIME=self.pull_data[0,:]
             self.PX=self.pull_data[1,:]
             self.PZ=self.pull_data[2,:]
             self.FB=self.pull_data[3,:]            

             #### Version 2
             self.temp_offset_theta = []
             self.temp_frames = []
             self.temp_theta = []
             self.temp_id = []
             self.temp_position_x = []
             self.temp_position_z = []
             self.temp_force_x = []
             self.temp_force_z = []  
             self.temp_vx = []
             self.temp_vy = []
             self.temp_c1 = []
             self.temp_c2 = []
             self.temp_force_m = []
             self.temp_wrenches = []
             self.temp_wrenches_norm = []
             self.temp_n = []
             
             self.temp_offset_theta2 = []
             self.temp_frames2 = []
             self.temp_theta2 = []
             self.temp_id2 = []
             self.temp_position_x2 = []
             self.temp_position_z2 = []
             self.temp_force_x2 = []
             self.temp_force_z2 = []  
             self.temp_vx2 = []
             self.temp_vy2 = []
             self.temp_c12 = []
             self.temp_c22 = []
             self.temp_force_m2 = []
             self.temp_wrenches2 = []
             self.temp_wrenches_norm2 = []
             self.temp_n2 = []            
             
         
             self.EPSILON2=[]
             self.HULL2=[]
             self.HULLWRENCHNORM2=[]
             self.HULLWRENCHMAGS2=[]
             self.WRENCHXY2=[]
             self.HULLXY2=[]
             self.WRENCHXT2=[]
             self.HULLXT2=[]
             self.WRENCHYT2=[]
             self.HULLYT2=[]     
                
             
             self.EPSILON3=[]
             self.HULL3=[]
             self.HULLWRENCHNORM3=[]
             self.HULLWRENCHMAGS3=[] 
             self.WRENCHXY3=[]
             self.HULLXY3=[]            
             self.WRENCHXT3=[]
             self.HULLXT3=[]
             self.WRENCHYT3=[]
             self.HULLYT3=[]   
            
            
             self.EPSILON4=[]
             self.HULL4=[]
             self.HULLWRENCHNORM4=[]
             self.HULLWRENCHMAGS4=[]  
             self.WRENCHXY4=[]
             self.HULLXY4=[]
             self.WRENCHXT4=[]
             self.HULLXT4=[]
             self.WRENCHYT4=[]
             self.HULLYT4=[]                
             
             
             
             
             print("extract_control_forces_ball")
             self.extract_control_forces_ball()
             
             print("find_contact_forces_2")
             self.find_contact_forces_2()
             
             print("extract_contact_forces_ball3")
             self.extract_contact_forces_ball_3()

             print("extract_contact_forces_ball4")
             self.extract_contact_forces_ball_4()
             
             
             print("find_pressure")
             self.find_pressure()
             
             print("find_pressure_mag")
             self.find_pressure_mag()
             
             print("average_pressure")
             self.average_pressure()
             
             
             print("set_up_wrenches")
             self.set_up_wrenches()
             
             print("calculate_wrench_components")
             self.calculate_wrench_components()
             
             print("calculate_wrench_components2")
             self.calculate_wrench_components2()
             
             print("calculate_wrench_components3")
             self.calculate_wrench_components3()
             
             
             
             print("grasp analysis")
             self.Grasp_analysis()




     def angle(self,x, y):
    
        rad = np.arctan2(y, x)
        #degrees = np.int(rad*180/np.pi)
        #if rad < 0:
            #rad = 2*np.pi - rad
        return rad



     def Grasp_analysis(self):
         "execute all functions related to grasp analysis"
         if exists(self.mainDirectory+self.name+"/graspParams.npy")==False:
             print("calculating")
             

             # print("set_up_wrenches")
             # self.set_up_wrenches()
             
             # print("calculate_wrench_components")
             # self.calculate_wrench_components()
             
             # print("calculate_wrench_components2")
             # self.calculate_wrench_components2()
             
             # print("calculate_wrench_components3")
             # self.calculate_wrench_components3()
             
             #print("calculate_epsilon")
             #self.calculate_epsilon()
             
             #print("calculate_epsilon2")
             #self.calculate_epsilon2()            
             
             #print("calculate_epsilon3")
             #self.calculate_epsilon3() 
             
             print("calculate_epsilon4")
             self.calculate_epsilon4()
             
            # print("find_pressure")
             #self.find_pressure()
             
             #print("find_pressure_mag")
             #self.find_pressure_mag()
             
             #self.average_pressure()
             print("save_grasp_parameters")
             self.save_grasp_parameters()
             self.graspParams = {}
         else:
             print("calculated")
             self.load_grasp_parameters()
     
            
     
     def load_grasp_parameters(self):
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

         #self.MAG_pressure = self.parameters["MAG_pressure"]
         #self.MAG_pressure_no_boundary = self.parameters["MAG_pressure_no_boundary"]




         self.grasp_id = self.parameters["grasp_id"]
             
         self.grasp_position_x = self.parameters["grasp_position_x"]
         self.grasp_position_z = self.parameters["grasp_position_z"]

         self.grasp_force_x = self.parameters["grasp_force_x"]
         self.grasp_force_z = self.parameters["grasp_force_z"]
         self.grasp_torque = self.parameters["grasp_torque"]
  
             
         #self.WRENCHES = self.parameters["WRENCHES"]
         #self.WRENCH_NORM = self.parameters["WRENCH_NORM"]
             
         self.FRAMES = self.parameters["FRAMES"] 
             
         self.EPSILON = self.parameters["EPSILON"] 
         self.HULLWRENCHNORM = self.parameters["HULLWRENCHNORM"] 
         self.HULLWRENCHMAGS = self.parameters["HULLWRENCHMAGS"] 
             
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
         self.ANGLE_CHECK = self.parameters["ANGLE_CHECK"]   
         
         
         
         self.temp_offset_theta = self.parameters["temp_offset_theta"]
         self.temp_frames = self.parameters["temp_frames"]
         self.temp_theta = self.parameters["temp_theta"]
         self.temp_id = self.parameters["temp_id"]
         self.temp_position_x = self.parameters["temp_position_x"]
         self.temp_position_z = self.parameters["temp_position_z"]
         self.temp_force_x = self.parameters["temp_force_x"]
         self.temp_force_z = self.parameters["temp_force_z"]
         self.temp_wrenches = self.parameters["temp_wrenches"]
         self.temp_wrenches_norm = self.parameters["temp_wrenches_norm"]
         self.temp_vx = self.parameters["temp_vx"]
         self.temp_vy = self.parameters["temp_vy"]
         self.temp_c1 = self.parameters["temp_c1"]
         self.temp_c2 = self.parameters["temp_c2"]
         self.EPSILON2 = self.parameters["EPSILON2"]
         self.HULLWRENCHNORM2 = self.parameters["HULLWRENCHNORM2"]
         self.HULLWRENCHMAGS2 = self.parameters["HULLWRENCHMAGS2"]
         self.HULL2 = self.parameters["HULL2"]           
         
  
         self.temp_offset_theta2 = self.parameters["temp_offset_theta2"]
         self.temp_frames2 = self.parameters["temp_frame2s"]
         self.temp_theta2 = self.parameters["temp_theta2"]
         self.temp_id2 = self.parameters["temp_id2"]
         self.temp_position_x2 = self.parameters["temp_position_x2"]
         self.temp_position_z2 = self.parameters["temp_position_z2"]
         self.temp_force_x2 = self.parameters["temp_force_x2"]
         self.temp_force_z2 = self.parameters["temp_force_z2"]
         self.temp_wrenches2 = self.parameters["temp_wrenches2"]
         self.temp_wrenches_norm2 = self.parameters["temp_wrenches_norm2"]
         self.temp_vx2 = self.parameters["temp_vx2"]
         self.temp_vy2 = self.parameters["temp_vy2"]
         self.temp_c12 = self.parameters["temp_c12"]
         self.temp_c22 = self.parameters["temp_c22"]
         
         self.EPSILON3 = self.parameters["EPSILON3"]
         self.HULLWRENCHNORM3 = self.parameters["HULLWRENCHNORM3"]
         self.HULLWRENCHMAGS3 = self.parameters["HULLWRENCHMAGS3"]
         self.HULL3 = self.parameters["HULL3"]        
         
         self.EPSILON4 = self.parameters["EPSILON4"]
         self.HULLWRENCHNORM4 = self.parameters["HULLWRENCHNORM4"]
         self.HULLWRENCHMAGS4 = self.parameters["HULLWRENCHMAGS4"]
         self.HULL4 = self.parameters["HULL4"]          
         
         
     def calculate_wrench_components(self):
         """ Calculaute the components of the 3D hull of the wrench space
                x-y , x-tau , y-tau sections"""
         for i in range(len(self.WRENCHES)):
             if len(self.grasp_id[i])==0:
                 self.WRENCHXY.append([])
                 self.HULLXY.append([])
                 
                 self.WRENCHXT.append([])
                 self.HULLXT.append([])

                 self.WRENCHYT.append([])
                 self.HULLYT.append([])
                  
             elif len(self.grasp_id[i])==1:
                 self.WRENCHXY.append([])
                 self.HULLXY.append([])
                 
                 self.WRENCHXT.append([])
                 self.HULLXT.append([])

                 self.WRENCHYT.append([])
                 self.HULLYT.append([])
             else:
                 Wrench=self.WRENCH_NORM[i]
                 Wrenchxy=np.zeros((Wrench.shape[0],2))
                 Wrenchxy[:,0]=Wrench[:,0]
                 Wrenchxy[:,1]=Wrench[:,1]
                 hullxy = ConvexHull(Wrenchxy)
        
                 Wrenchxt=np.zeros((Wrench.shape[0],2))
                 Wrenchxt[:,0]=Wrench[:,0]
                 Wrenchxt[:,1]=Wrench[:,2]
                 hullxt = ConvexHull(Wrenchxt)
        
                 Wrenchyt=np.zeros((Wrench.shape[0],2))
                 Wrenchyt[:,0]=Wrench[:,1]
                 Wrenchyt[:,1]=Wrench[:,2]
                 hullyt = ConvexHull(Wrenchyt)                 

                 self.WRENCHXY.append(Wrenchxy)
                 self.HULLXY.append(hullxy)
                 
                 self.WRENCHXT.append(Wrenchxt)
                 self.HULLXT.append(hullxt)

                 self.WRENCHYT.append(Wrenchyt)
                 self.HULLYT.append(hullyt)

     def calculate_wrench_components2(self):
         """ Calculaute the components of the 3D hull of the wrench space
                x-y , x-tau , y-tau sections"""
         for i in range(len(self.temp_wrenches)):
             if len(self.temp_id[i])==0:
                 self.WRENCHXY2.append([])
                 self.HULLXY2.append([])
                 
                 self.WRENCHXT2.append([])
                 self.HULLXT2.append([])

                 self.WRENCHYT2.append([])
                 self.HULLYT2.append([])
                  
             elif len(self.temp_id[i])==1:
                 self.WRENCHXY2.append([])
                 self.HULLXY2.append([])
                 
                 self.WRENCHXT2.append([])
                 self.HULLXT2.append([])

                 self.WRENCHYT2.append([])
                 self.HULLYT2.append([])
             else:
                 Wrench=self.temp_wrenches_norm[i]
                 Wrenchxy=np.zeros((Wrench.shape[0],2))
                 Wrenchxy[:,0]=Wrench[:,0]
                 Wrenchxy[:,1]=Wrench[:,1]
                 hullxy = ConvexHull(Wrenchxy)
        
                 Wrenchxt=np.zeros((Wrench.shape[0],2))
                 Wrenchxt[:,0]=Wrench[:,0]
                 Wrenchxt[:,1]=Wrench[:,2]
                 hullxt = ConvexHull(Wrenchxt)
        
                 Wrenchyt=np.zeros((Wrench.shape[0],2))
                 Wrenchyt[:,0]=Wrench[:,1]
                 Wrenchyt[:,1]=Wrench[:,2]
                 hullyt = ConvexHull(Wrenchyt)                 
                         
                         
                         
                 
                 self.WRENCHXY2.append(Wrenchxy)
                 self.HULLXY2.append(hullxy)
                 
                 self.WRENCHXT2.append(Wrenchxt)
                 self.HULLXT.append(hullxt)

                 self.WRENCHYT2.append(Wrenchyt)
                 self.HULLYT2.append(hullyt)
                 
     def calculate_wrench_components3(self):
         """ Calculaute the components of the 3D hull of the wrench space
                x-y , x-tau , y-tau sections"""
         for i in range(len(self.temp_wrenches2)):
             print(i,"of ", len(self.temp_wrenches2))
             if len(self.temp_id2[i])==0:
                 self.WRENCHXY3.append([])
                 self.HULLXY3.append([])
                 
                 self.WRENCHXT3.append([])
                 self.HULLXT3.append([])

                 self.WRENCHYT3.append([])
                 self.HULLYT3.append([])
                  
             elif len(self.temp_id2[i])==1:
                 self.WRENCHXY3.append([])
                 self.HULLXY3.append([])
                 
                 self.WRENCHXT3.append([])
                 self.HULLXT3.append([])

                 self.WRENCHYT3.append([])
                 self.HULLYT3.append([])
                 
             elif np.all((self.temp_wrenches2[i]==0)):
                 self.WRENCHXY3.append([])
                 self.HULLXY3.append([])
                 
                 self.WRENCHXT3.append([])
                 self.HULLXT3.append([])

                 self.WRENCHYT3.append([])
                 self.HULLYT3.append([])                 
             else:
                 Wrench=self.temp_wrenches_norm2[i]
                 Wrenchxy=np.zeros((Wrench.shape[0],2))
                 Wrenchxy[:,0]=Wrench[:,0]
                 Wrenchxy[:,1]=Wrench[:,1]
                 hullxy = ConvexHull(Wrenchxy)
        
                 Wrenchxt=np.zeros((Wrench.shape[0],2))
                 Wrenchxt[:,0]=Wrench[:,0]
                 Wrenchxt[:,1]=Wrench[:,2]
                 hullxt = ConvexHull(Wrenchxt)
        
                 Wrenchyt=np.zeros((Wrench.shape[0],2))
                 Wrenchyt[:,0]=Wrench[:,1]
                 Wrenchyt[:,1]=Wrench[:,2]
                 hullyt = ConvexHull(Wrenchyt)                 
                         
                         
                         
                 
                 self.WRENCHXY3.append(Wrenchxy)
                 self.HULLXY3.append(hullxy)
                 
                 self.WRENCHXT3.append(Wrenchxt)
                 self.HULLXT3.append(hullxt)

                 self.WRENCHYT3.append(Wrenchyt)
                 self.HULLYT3.append(hullyt)                 
                 
     def square_function1(self,Rx,Ry,x,y):
         P=np.array([x,y])
         R=np.array([Rx,Ry])
         q=np.abs(P)-R
         phi=np.linalg.norm(np.maximum(q,0)) - np.minimum(np.amax(q),0)   
         return(phi)


     def extract_contact_forces_ball_3(self):

        for i in range(len(self.time)-2):
            F_contact_ballx_entry=self.Force_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            F_contact_ballz_entry=self.Force_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(F_contact_ballx_entry)
            F_contact_ballx_entry=F_contact_ballx_entry[0]['ballx'] #  FURTHER EXTRACTION 
            F_contact_ballz_entry=F_contact_ballz_entry[0]['ballz'] #  FURTHER EXTRACTION 

            Position_x_contact_entry=self.position_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            Position_z_contact_entry=self.position_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(Position_x_contact_entry)
            Position_x_contact_entry=Position_x_contact_entry[0]['ballx'] #  FURTHER EXTRACTION 
            Position_z_contact_entry=Position_z_contact_entry[0]['ballz'] #  FURTHER EXTRACTION 

            dir_xx_contact_ball_entry=self.dir_xx_contact_ball["time_contact"+str(i)]
            dir_xz_contact_ball_entry=self.dir_xz_contact_ball["time_contact"+str(i)]
            dir_zx_contact_ball_entry=self.dir_zx_contact_ball["time_contact"+str(i)]
            dir_zz_contact_ball_entry=self.dir_zz_contact_ball["time_contact"+str(i)]


            dir_xx_contact_ball_entry=dir_xx_contact_ball_entry[0]['ballx']
            dir_xz_contact_ball_entry=dir_xz_contact_ball_entry[0]['ballz']
            dir_zx_contact_ball_entry=dir_zx_contact_ball_entry[0]['ballx']
            dir_zz_contact_ball_entry=dir_zz_contact_ball_entry[0]['ballz']
            
            if self.geom=="square":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                const=self.ball_radius*2-.01
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x__=[w,-w,-w,w,w]
                y__=[h,h,-h,-h,h]
                (segments)=self.create_segment(x__,y__) 
                const_=self.ball_radius*2
                xb,yb=0-const_/2,0 - const_/2
                x0_=xb
                y0_=yb


            if self.geom=="triangle":
               x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
               const=self.ball_radius*2*np.pi/3
               r=const*np.sqrt(3)/3 -.01
               x1=r
               y1=0
               x2=r*np.cos(2*np.pi/3)
               y2=r*np.sin(2*np.pi/3)
               
               x3=r*np.cos(4*np.pi/3)
               y3=r*np.sin(4*np.pi/3)
               
               x__ = [x1,x2,x3,x1]
               y__ = [y1,y2,y3,y1]
               (segments)=self.create_segment(x__,y__)


            if self.geom=="import":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                
            if self.geom=="circle":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                
                
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
            temp_wrenches=[]
            temp_wrenches_norm=[]
            temp_n=[]
            frames = np.zeros((len(Position_x_contact_entry),3))
            Vx=np.zeros((len(Position_x_contact_entry),2))
            Vy=np.zeros((len(Position_x_contact_entry),2))
            C1=np.zeros((2,len(Position_x_contact_entry))) # positive cone
            C2=np.zeros((2,len(Position_x_contact_entry))) # negative cone 
            for j in range(len(Position_x_contact_entry)):

                x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
                #ax.text(x0-x0b,y0-y0b,str(j),size=8)
                theta1=np.arctan2(.2,1) #+ frames[j,2]             
                if self.geom=="square":
                    Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                    Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                    mag=1
                    
                if self.geom=="triangle":
                    Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                    Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                    mag=1  
                    
                if self.geom=="import":
                     Fx1,Fy1=self.F_random_objdx(x0-x0b,y0-y0b),self.F_random_objdy(x0-x0b,y0-y0b)
                     mag=np.sqrt(Fx1**2 + Fy1**2)
                     Fx1=-Fx1/mag
                     Fy1=-Fy1/mag
                     F_t=np.array([Fx1,Fy1])
                     X.append(x0-x0b)
                     Y.append(y0-y0b)
                     t=self.angle(Fx1, Fy1)-np.pi/2
                     theta.append(t)
                
                     frames[j,0]=x0-x0b
                     frames[j,1]=y0-y0b
                     frames[j,2]=t
                     mag=1
                     
                if self.geom=="circle":
                    Fx1,Fy1=(self.ballx_position[i]-x0),(self.ballz_position[i]-y0)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=Fx1/mag
                    Fy1=Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
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
                C1[:,j]=mag*T1@tem
                C2[:,j]=mag*T2@tem
            
            
                mag2=np.sqrt(dir_xx_contact_ball_entry[j]**2 + dir_xz_contact_ball_entry[j]**2)
                temp_dirr=[dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2]
            
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
                        mag_=1
                        temp_n.append(mag_*np.cos(temp2))
                        temp_vx.append(Vx[j,:])
                        temp_vy.append(Vy[j,:])
                        #temp_c1.append(mag_*np.cos(temp2)*C1[:,j])
                        #temp_c2.append(mag_*np.cos(temp2)*C2[:,j])  
                        temp_c1.append(C1[:,j])
                        temp_c2.append(C2[:,j])                        
                        if self.geom=="square":
                            k=(self.ball_radius*2)/(2*np.sqrt(3))
             
                        if self.geom=="circle":
                            k = self.ball_radius/np.sqrt(2)   
                            
                        if self.geom=="triangle":
                            const=self.ball_radius*2*np.pi/3
                            #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                            r=const*np.sqrt(3)/3
                            x1=r
                            y1=0
               
                            x2=r*np.cos(2*np.pi/3)
                            y2=r*np.sin(2*np.pi/3)
                            d=np.sqrt((x1-x2)**2 + (y1-y2)**2)
                            k = d*.2041 
                        
                            
                        if self.geom=="import":
                            k=2.18
                        m_t=np.cross(np.array([X[j],Y[j]]),C1[:,j])
                        temp=[C1[0,j],C1[1,j],m_t]
                        wrench1=temp
                        temp_wrenches.append(temp)

                        wrench_norm=np.zeros(3)
                        wrench_norm[0]=wrench1[0]
                        wrench_norm[1]=wrench1[1]
                        wrench_norm[2]=wrench1[2]/k
                        mag=np.sqrt(wrench_norm[0]**2 + wrench_norm[1]**2 + wrench_norm[2]**2)
                        #mag=np.sqrt(wrench_norm[0]**2 + wrench_norm[1]**2)
                        #mag=1
                        wrench_norm[0]=wrench_norm[0]/mag
                        wrench_norm[1]=wrench_norm[1]/mag
                        wrench_norm[2]=wrench_norm[2]/mag
                        temp_wrenches_norm.append(wrench_norm)

                        m_t=np.cross(np.array([X[j],Y[j]]),C2[:,j])
                        temp=[C2[0,j],C2[1,j],m_t]
                        wrench2=temp
                        temp_wrenches.append(temp)
                        
                        wrench_norm=np.zeros(3)
                        wrench_norm[0]=wrench2[0]
                        wrench_norm[1]=wrench2[1]
                        wrench_norm[2]=wrench2[2]/k
                        mag=np.sqrt(wrench_norm[0]**2 + wrench_norm[1]**2 + wrench_norm[2]**2)
                        wrench_norm[0]=wrench_norm[0]/mag
                        wrench_norm[1]=wrench_norm[1]/mag
                        wrench_norm[2]=wrench_norm[2]/mag
                        temp_wrenches_norm.append(wrench_norm) 

                          
                        
            if len(temp_position_x)!=0:
                 self.temp_offset_theta.append(temp_offset_theta)
                 self.temp_frames.append(temp_frames)
                 self.temp_theta.append(temp_theta)
                 self.temp_id.append(temp_id)
                 self.temp_position_x.append(temp_position_x)
                 self.temp_position_z.append(temp_position_z)
                 self.temp_force_x.append(temp_force_x)
                 self.temp_force_z.append(temp_force_z)     
                 self.temp_wrenches.append(np.asarray(temp_wrenches))
                 #self.temp_wrenches_norm.append(np.asarray(temp_wrenches_norm/np.max(temp_n)))
                 self.temp_wrenches_norm.append(np.asarray(temp_wrenches_norm))                
                 self.temp_vx.append(temp_vx)
                 self.temp_vy.append(temp_vy)
                 self.temp_c1.append(temp_c1)
                 self.temp_c2.append(temp_c2) 
                 self.temp_n.append(temp_n)
                 #self.temp_force_m.append(temp_force_m)
                 
            else:
                 self.temp_offset_theta.append([])
                 self.temp_frames.append([])
                 self.temp_theta.append([])
                 self.temp_id.append([])
                 self.temp_position_x.append([])
                 self.temp_position_z.append([])
                 self.temp_force_x.append([])
                 self.temp_force_z.append([])   
                 self.temp_wrenches.append([])
                 self.temp_wrenches_norm.append([])
                 self.temp_vx.append([])
                 self.temp_vy.append([])
                 self.temp_c1.append([])
                 self.temp_c2.append([])
                 self.temp_n.append([])
                 #self.temp_force_m.append([])              
         
            
         
     def extract_contact_forces_ball_4(self):

        for i in range(len(self.time)-2):
            F_contact_ballx_entry=self.Force_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            F_contact_ballz_entry=self.Force_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(F_contact_ballx_entry)
            F_contact_ballx_entry=F_contact_ballx_entry[0]['ballx'] #  FURTHER EXTRACTION 
            F_contact_ballz_entry=F_contact_ballz_entry[0]['ballz'] #  FURTHER EXTRACTION 

            Position_x_contact_entry=self.position_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            Position_z_contact_entry=self.position_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(Position_x_contact_entry)
            Position_x_contact_entry=Position_x_contact_entry[0]['ballx'] #  FURTHER EXTRACTION 
            Position_z_contact_entry=Position_z_contact_entry[0]['ballz'] #  FURTHER EXTRACTION 

            dir_xx_contact_ball_entry=self.dir_xx_contact_ball["time_contact"+str(i)]
            dir_xz_contact_ball_entry=self.dir_xz_contact_ball["time_contact"+str(i)]
            dir_zx_contact_ball_entry=self.dir_zx_contact_ball["time_contact"+str(i)]
            dir_zz_contact_ball_entry=self.dir_zz_contact_ball["time_contact"+str(i)]


            dir_xx_contact_ball_entry=dir_xx_contact_ball_entry[0]['ballx']
            dir_xz_contact_ball_entry=dir_xz_contact_ball_entry[0]['ballz']
            dir_zx_contact_ball_entry=dir_zx_contact_ball_entry[0]['ballx']
            dir_zz_contact_ball_entry=dir_zz_contact_ball_entry[0]['ballz']
            
            
            x0b,y0b=self.ballx_position[i],self.ballz_position[i]
            
            
            if self.geom=="square":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                const=self.ball_radius*2-.01
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x__=[w,-w,-w,w,w]
                y__=[h,h,-h,-h,h]
                (segments)=self.create_segment(x__,y__) 
                const_=self.ball_radius*2
                xb,yb=0-const_/2,0 - const_/2
                x0_=xb
                y0_=yb


            if self.geom=="triangle":
               x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
               const=self.ball_radius*2*np.pi/3
               r=const*np.sqrt(3)/3 -.01
               x1=r
               y1=0
               x2=r*np.cos(2*np.pi/3)
               y2=r*np.sin(2*np.pi/3)
               
               x3=r*np.cos(4*np.pi/3)
               y3=r*np.sin(4*np.pi/3)
               
               x__ = [x1,x2,x3,x1]
               y__ = [y1,y2,y3,y1]
               (segments)=self.create_segment(x__,y__)

            if self.geom=="circle":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                
            if self.geom=="import":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]               
                
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
            temp_wrenches=[]
            temp_wrenches_norm=[]
            temp_n=[]
            frames = np.zeros((len(Position_x_contact_entry),3))
            Vx=np.zeros((len(Position_x_contact_entry),2))
            Vy=np.zeros((len(Position_x_contact_entry),2))
            C1=np.zeros((2,len(Position_x_contact_entry))) # positive cone
            C2=np.zeros((2,len(Position_x_contact_entry))) # negative cone 
            for j in range(len(Position_x_contact_entry)):

                x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
                #ax.text(x0-x0b,y0-y0b,str(j),size=8)
                theta1=np.arctan2(.2,1) #+ frames[j,2]             
                if self.geom=="square":
                    Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                    Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                    mag=1
                    
                elif self.geom=="triangle":
                    Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                    Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                    mag=1  
                    
                    
                elif self.geom=="circle":
                    Fx1,Fy1=(self.ballx_position[i]-x0),(self.ballz_position[i]-y0)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=Fx1/mag
                    Fy1=Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    
                    
                elif self.geom=="import":
                     Fx1,Fy1=self.F_random_objdx(x0-x0b,y0-y0b),self.F_random_objdy(x0-x0b,y0-y0b)
                     mag=np.sqrt(Fx1**2 + Fy1**2)
                     Fx1=-Fx1/mag
                     Fy1=-Fy1/mag
                     F_t=np.array([Fx1,Fy1])
                     X.append(x0-x0b)
                     Y.append(y0-y0b)
                     t=self.angle(Fx1, Fy1)-np.pi/2
                     theta.append(t)
                
                     frames[j,0]=x0-x0b
                     frames[j,1]=y0-y0b
                     frames[j,2]=t
                     #mag=1    
                    
                mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                #mag=1
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
                C1[:,j]=mag*T1@tem
                C2[:,j]=mag*T2@tem
            
            
                mag2=np.sqrt(dir_xx_contact_ball_entry[j]**2 + dir_xz_contact_ball_entry[j]**2)
                temp_dirr=[dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2]
            
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
                        #mag_=1
                        temp_n.append(mag_*np.cos(temp2))
                        temp_vx.append(Vx[j,:])
                        temp_vy.append(Vy[j,:])
                        temp_c1.append(C1[:,j])
                        temp_c2.append(C2[:,j])                        
                        if self.geom=="square":
                            k=(self.ball_radius*2)/(2*np.sqrt(3))
             
                        if self.geom=="circle":
                            k = self.ball_radius/np.sqrt(2)   
                        
                        if self.geom=="import":
                            k=2.18
                        
                        if self.geom=="triangle":   
                            const=self.ball_radius*2*np.pi/3
                            r=const*np.sqrt(3)/3
                            x1=r
                            y1=0
                            x2=r*np.cos(2*np.pi/3)
                            y2=r*np.sin(2*np.pi/3)
                            d=np.sqrt((x1-x2)**2 +(y1-y2)**2)
                            k = d*.2041    
                            
                        m_t=np.cross(np.array([X[j],Y[j]]),C1[:,j])
                        temp=[C1[0,j],C1[1,j],m_t]
                        wrench1=temp
                        temp_wrenches.append(temp)

                        wrench_norm=np.zeros(3)
                        wrench_norm[0]=wrench1[0]
                        wrench_norm[1]=wrench1[1]
                        wrench_norm[2]=wrench1[2]/k
                        mag=1
                        wrench_norm[0]=wrench_norm[0]/mag
                        wrench_norm[1]=wrench_norm[1]/mag
                        wrench_norm[2]=wrench_norm[2]/mag
                        temp_wrenches_norm.append(wrench_norm)

                        m_t=np.cross(np.array([X[j],Y[j]]),C2[:,j])
                        temp=[C2[0,j],C2[1,j],m_t]
                        wrench2=temp
                        temp_wrenches.append(temp)
                        
                        wrench_norm=np.zeros(3)
                        wrench_norm[0]=wrench2[0]
                        wrench_norm[1]=wrench2[1]
                        wrench_norm[2]=wrench2[2]/k
                        mag=1
                        wrench_norm[0]=wrench_norm[0]/mag
                        wrench_norm[1]=wrench_norm[1]/mag
                        wrench_norm[2]=wrench_norm[2]/mag
                        temp_wrenches_norm.append(wrench_norm) 

                          
                        
            if len(temp_position_x)!=0:
                 self.temp_offset_theta2.append(temp_offset_theta)
                 self.temp_frames2.append(temp_frames)
                 self.temp_theta2.append(temp_theta)
                 self.temp_id2.append(temp_id)
                 self.temp_position_x2.append(temp_position_x)
                 self.temp_position_z2.append(temp_position_z)
                 self.temp_force_x2.append(temp_force_x)
                 self.temp_force_z2.append(temp_force_z)     
                 self.temp_wrenches2.append(np.asarray(temp_wrenches))
                 #self.temp_wrenches_norm.append(np.asarray(temp_wrenches_norm/np.max(temp_n)))
                 self.temp_wrenches_norm2.append(np.asarray(temp_wrenches_norm))                
                 self.temp_vx2.append(temp_vx)
                 self.temp_vy2.append(temp_vy)
                 self.temp_c12.append(temp_c1)
                 self.temp_c22.append(temp_c2) 
                 self.temp_n2.append(temp_n)
                 #self.temp_force_m.append(temp_force_m)
                 
            else:
                 self.temp_offset_theta2.append([])
                 self.temp_frames2.append([])
                 self.temp_theta2.append([])
                 self.temp_id2.append([])
                 self.temp_position_x2.append([])
                 self.temp_position_z2.append([])
                 self.temp_force_x2.append([])
                 self.temp_force_z2.append([])   
                 self.temp_wrenches2.append([])
                 self.temp_wrenches_norm2.append([])
                 self.temp_vx2.append([])
                 self.temp_vy2.append([])
                 self.temp_c12.append([])
                 self.temp_c22.append([])
                 self.temp_n2.append([])
                 #self.temp_force_m.append([])              
                     
         
     def extract_contact_forces_ball_(self):

        for i in range(len(self.time)-2):
            F_contact_ballx_entry=self.Force_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            F_contact_ballz_entry=self.Force_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(F_contact_ballx_entry)
            F_contact_ballx_entry=F_contact_ballx_entry[0]['ballx'] #  FURTHER EXTRACTION 
            F_contact_ballz_entry=F_contact_ballz_entry[0]['ballz'] #  FURTHER EXTRACTION 

            Position_x_contact_entry=self.position_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            Position_z_contact_entry=self.position_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(Position_x_contact_entry)
            Position_x_contact_entry=Position_x_contact_entry[0]['ballx'] #  FURTHER EXTRACTION 
            Position_z_contact_entry=Position_z_contact_entry[0]['ballz'] #  FURTHER EXTRACTION 

            dir_xx_contact_ball_entry=self.dir_xx_contact_ball["time_contact"+str(i)]
            dir_xz_contact_ball_entry=self.dir_xz_contact_ball["time_contact"+str(i)]
            dir_zx_contact_ball_entry=self.dir_zx_contact_ball["time_contact"+str(i)]
            dir_zz_contact_ball_entry=self.dir_zz_contact_ball["time_contact"+str(i)]


            dir_xx_contact_ball_entry=dir_xx_contact_ball_entry[0]['ballx']
            dir_xz_contact_ball_entry=dir_xz_contact_ball_entry[0]['ballz']
            dir_zx_contact_ball_entry=dir_zx_contact_ball_entry[0]['ballx']
            dir_zz_contact_ball_entry=dir_zz_contact_ball_entry[0]['ballz']
            
            
            x0b,y0b=self.ballx_position[i],self.ballz_position[i]
            
            
            if self.geom=="square":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                const=self.ball_radius*2-.01
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x__=[w,-w,-w,w,w]
                y__=[h,h,-h,-h,h]
                (segments)=self.create_segment(x__,y__) 
                const_=self.ball_radius*2
                xb,yb=0-const_/2,0 - const_/2
                x0_=xb
                y0_=yb


            if self.geom=="triangle":
               x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
               const=self.ball_radius*2*np.pi/3
               r=const*np.sqrt(3)/3 -.01
               x1=r
               y1=0
               x2=r*np.cos(2*np.pi/3)
               y2=r*np.sin(2*np.pi/3)
               
               x3=r*np.cos(4*np.pi/3)
               y3=r*np.sin(4*np.pi/3)
               
               x__ = [x1,x2,x3,x1]
               y__ = [y1,y2,y3,y1]
               (segments)=self.create_segment(x__,y__)

            if self.geom=="circle":
                x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                
                
                
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
            temp_wrenches=[]
            temp_wrenches_norm=[]
            temp_n=[]
            frames = np.zeros((len(Position_x_contact_entry),3))
            Vx=np.zeros((len(Position_x_contact_entry),2))
            Vy=np.zeros((len(Position_x_contact_entry),2))
            C1=np.zeros((2,len(Position_x_contact_entry))) # positive cone
            C2=np.zeros((2,len(Position_x_contact_entry))) # negative cone 
            for j in range(len(Position_x_contact_entry)):

                x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
                #ax.text(x0-x0b,y0-y0b,str(j),size=8)
                theta1=np.arctan2(.2,1) #+ frames[j,2]             
                if self.geom=="square":
                    Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                    Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                    mag=1
                    
                if self.geom=="triangle":
                    Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                    Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                    mag=1  
                    
                if self.geom=="import":
                     Fx1,Fy1=self.F_random_objdx(x0-x0b,y0-y0b),self.F_random_objdy(x0-x0b,y0-y0b)
                     mag=np.sqrt(Fx1**2 + Fy1**2)
                     Fx1=-Fx1/mag
                     Fy1=-Fy1/mag
                     F_t=np.array([Fx1,Fy1])
                     X.append(x0-x0b)
                     Y.append(y0-y0b)
                     t=self.angle(Fx1, Fy1)-np.pi/2
                     theta.append(t)
                
                     frames[j,0]=x0-x0b
                     frames[j,1]=y0-y0b
                     frames[j,2]=t
                     #mag=1    
                if self.geom=="circle":
                    Fx1,Fy1=(self.ballx_position[i]-x0),(self.ballz_position[i]-y0)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=Fx1/mag
                    Fy1=Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    
                    
                    
                    
                mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                #mag=1
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
                C1[:,j]=mag*T1@tem
                C2[:,j]=mag*T2@tem
            
            
                mag2=np.sqrt(dir_xx_contact_ball_entry[j]**2 + dir_xz_contact_ball_entry[j]**2)
                temp_dirr=[dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2]
            
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
                        #mag_=1
                        temp_n.append(mag_*np.cos(temp2))
                        temp_vx.append(Vx[j,:])
                        temp_vy.append(Vy[j,:])
                        temp_c1.append(C1[:,j])
                        temp_c2.append(C2[:,j])                        
                        if self.geom=="square":
                            k=(self.ball_radius*2)/(2*np.sqrt(3))
             
                        if self.geom=="circle":
                            k = self.ball_radius/np.sqrt(2)   
                        
                        if self.geom=="import":
                            k = 2.18
                        if self.geom=="triangle":   
                            const=self.ball_radius*2*np.pi/3
                            r=const*np.sqrt(3)/3
                            x1=r
                            y1=0
                            x2=r*np.cos(2*np.pi/3)
                            y2=r*np.sin(2*np.pi/3)
                            d=np.sqrt((x1-x2)**2 +(y1-y2)**2)
                            k = d*.2041    
                            
                        m_t=np.cross(np.array([X[j],Y[j]]),C1[:,j])
                        temp=[C1[0,j],C1[1,j],m_t]
                        wrench1=temp
                        temp_wrenches.append(temp)

                        wrench_norm=np.zeros(3)
                        wrench_norm[0]=wrench1[0]
                        wrench_norm[1]=wrench1[1]
                        wrench_norm[2]=wrench1[2]/k
                        mag=1
                        wrench_norm[0]=wrench_norm[0]/mag
                        wrench_norm[1]=wrench_norm[1]/mag
                        wrench_norm[2]=wrench_norm[2]/mag
                        temp_wrenches_norm.append(wrench_norm)

                        m_t=np.cross(np.array([X[j],Y[j]]),C2[:,j])
                        temp=[C2[0,j],C2[1,j],m_t]
                        wrench2=temp
                        temp_wrenches.append(temp)
                        
                        wrench_norm=np.zeros(3)
                        wrench_norm[0]=wrench2[0]
                        wrench_norm[1]=wrench2[1]
                        wrench_norm[2]=wrench2[2]/k
                        mag=1
                        wrench_norm[0]=wrench_norm[0]/mag
                        wrench_norm[1]=wrench_norm[1]/mag
                        wrench_norm[2]=wrench_norm[2]/mag
                        temp_wrenches_norm.append(wrench_norm) 

                          
                        
            if len(temp_position_x)!=0:
                 self.temp_offset_theta2.append(temp_offset_theta)
                 self.temp_frames2.append(temp_frames)
                 self.temp_theta2.append(temp_theta)
                 self.temp_id2.append(temp_id)
                 self.temp_position_x2.append(temp_position_x)
                 self.temp_position_z2.append(temp_position_z)
                 self.temp_force_x2.append(temp_force_x)
                 self.temp_force_z2.append(temp_force_z)     
                 self.temp_wrenches2.append(np.asarray(temp_wrenches))
                 #self.temp_wrenches_norm.append(np.asarray(temp_wrenches_norm/np.max(temp_n)))
                 self.temp_wrenches_norm2.append(np.asarray(temp_wrenches_norm))                
                 self.temp_vx2.append(temp_vx)
                 self.temp_vy2.append(temp_vy)
                 self.temp_c12.append(temp_c1)
                 self.temp_c22.append(temp_c2) 
                 self.temp_n2.append(temp_n)
                 #self.temp_force_m.append(temp_force_m)
                 
            else:
                 self.temp_offset_theta2.append([])
                 self.temp_frames2.append([])
                 self.temp_theta2.append([])
                 self.temp_id2.append([])
                 self.temp_position_x2.append([])
                 self.temp_position_z2.append([])
                 self.temp_force_x2.append([])
                 self.temp_force_z2.append([])   
                 self.temp_wrenches2.append([])
                 self.temp_wrenches_norm2.append([])
                 self.temp_vx2.append([])
                 self.temp_vy2.append([])
                 self.temp_c12.append([])
                 self.temp_c22.append([])
                 self.temp_n2.append([])
                 #self.temp_force_m.append([])                   
         
     def extract_control_forces_ball(self):
         "extract control forces method "
         for i in range(len(self.time)):
             #print(i)
             temp_id=[]
             temp_position_x=[]
             temp_position_z=[]
             temp_force_x=[]
             temp_force_z=[]
             temp_torque=[]
             temp_angle_check=[]
             
             
             if self.geom=="circle":
                 for j in range(self.nb):
                     x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i] 
                     Fx,Fy=self.control_forces_x[j,i],self.control_forces_z[j,i]
                     q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                     qr=np.array([[(x0-self.ballx_position[i]),(y0-self.ballz_position[i]),0]])
                     Fr=np.array([[Fx,Fy,0]])
                     M=np.cross(qr,Fr)
                     F=np.array([[Fx,Fy]])
                         #Ft=self.transform_forces([x0],[y0],self.ballx[i],self.ballz[i],F)
 
                
  
                     if q<=(2 * self.bot_width/2 + self.ball_radius):
                          theta1=np.arctan(self.mu)
                          x0_,y0_=(x0-self.ballx_position[i]),(y0-self.ballz_position[i]) # initia positions
                          theta=np.arctan2(y0_,x0_) # calculate theta
                          T=np.array([[-np.sin(theta),-np.cos(theta)],[np.cos(theta),-np.sin(theta)]]) # transformation matrix 
                          VXpp=T@np.array([[1],[0]]) # transform coordinates X
                          VYpp=T@np.array([[0],[1]]) # transform coordinates Y
                          VXpp=VXpp.flatten() # flatten the matrix
                          VYpp=VYpp.flatten() # flatten the matrix
                          F=np.array([Fx,Fy])
                          temp=np.arccos(np.round(np.dot(F,VYpp)/(np.linalg.norm(F)*np.linalg.norm(VYpp)),2))
                          #print(temp)
                          temp_angle_check.append(temp)
                          if temp<theta1:
                             temp_id.append(j)
                             temp_position_x.append(x0)
                             temp_position_z.append(y0)
                             temp_force_x.append(Fx)
                             temp_force_z.append(Fy)
                             temp_torque.append(M)
                             
             if self.geom=="square":
                 for j in range(self.nb):
                     x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i] 
                     Fx,Fy=self.control_forces_x[j,i],self.control_forces_z[j,i]
                     q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                     qr=np.array([[(x0-self.ballx_position[i]),(y0-self.ballz_position[i]),0]])
                     Fr=np.array([[Fx,Fy,0]])
                     M=np.cross(qr,Fr)
                     #F=np.array([[Fx,Fy]])
                     #Ft=self.transform_forces([x0],[y0],self.ballx[i],self.ballz[i],F)
                     #print(qr)
                     #print(self.square_function1(const,const,qr[0,0],qr[0,1]))
                     xcenter=self.ballx_position[i]
                     ycenter=self.ballz_position[i]
            
                     const=self.ball_radius*2
                     rx=const
                     ry=const
                     w=rx/2
                     h=ry/2                    
                     x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                     y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
      
                     #x = x + xcenter*np.ones(len(x))
                     #y = y + ycenter*np.ones(len(x))
                     (self.segments)=self.create_segment(x,y) 
                
                     if self.PHI(x0,y0,self.segments)<.2:
                            temp_id.append(j)
                            temp_position_x.append(x0)
                            temp_position_z.append(y0)
                            temp_force_x.append(Fx)
                            temp_force_z.append(Fy)
                            temp_torque.append(M)   
                            
                         
             if self.geom=="triangle":
                 for j in range(self.nb):
                     x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i] 
                     Fx,Fy=self.control_forces_x[j,i],self.control_forces_z[j,i]
                     q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                     qr=np.array([[(x0-self.ballx_position[i]),(y0-self.ballz_position[i]),0]])
                     Fr=np.array([[Fx,Fy,0]])
                     M=np.cross(qr,Fr)
                     #F=np.array([[Fx,Fy]])
                     #Ft=self.transform_forces([x0],[y0],self.ballx[i],self.ballz[i],F)
                     #print(qr)
                     #print(self.square_function1(const,const,qr[0,0],qr[0,1]))
                     xcenter=self.ballx_position[i]
                     ycenter=self.ballz_position[i]
            

                     const=(self.ball_radius)*2*np.pi/3
                     r=const*np.sqrt(3)/3
                     x1=r
                     y1=0
               
                     x2=r*np.cos(2*np.pi/3)
                     y2=r*np.sin(2*np.pi/3)
               
                     x3=r*np.cos(4*np.pi/3)
                     y3=r*np.sin(4*np.pi/3)
               
                     x__ = [x1,x2,x3,x1]+xcenter
                     y__ = [y1,y2,y3,y1]+ycenter
                     (self.segments)=self.create_segment(x__,y__)
      
                     if self.PHI(x0,y0,self.segments)<.2:
                            temp_id.append(j)
                            temp_position_x.append(x0)
                            temp_position_z.append(y0)
                            temp_force_x.append(Fx)
                            temp_force_z.append(Fy)
                            temp_torque.append(M)                               
             
             if self.geom=="import":
                 for j in range(self.nb):
                     x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i] 
                     Fx,Fy=self.control_forces_x[j,i],self.control_forces_z[j,i]
                     q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                     qr=np.array([[(x0-self.ballx_position[i]),(y0-self.ballz_position[i]),0]])
                     Fr=np.array([[Fx,Fy,0]])
                     M=np.cross(qr,Fr)
                     #F=np.array([[Fx,Fy]])
                     #Ft=self.transform_forces([x0],[y0],self.ballx[i],self.ballz[i],F)
                     #print(qr)
                     #print(self.square_function1(const,const,qr[0,0],qr[0,1]))
                     xcenter=self.ballx_position[i]
                     ycenter=self.ballz_position[i]
            

               
      
                     if self.F_random_obj(x0,y0)<.2:
                            temp_id.append(j)
                            temp_position_x.append(x0)
                            temp_position_z.append(y0)
                            temp_force_x.append(Fx)
                            temp_force_z.append(Fy)
                            temp_torque.append(M)                               
                                        
             
             #print("Time= "+str(self.time[i])+" id numbers= "+str(temp_id))        
             self.grasp_id.append(temp_id) # append index 
             self.grasp_position_x.append(temp_position_x) # append positon x
             self.grasp_position_z.append(temp_position_z) # append position z
             self.grasp_force_x.append(temp_position_x) # append force x
             self.grasp_force_z.append(temp_position_z) # append force z  
             self.grasp_torque.append(temp_torque)   # append torque  
             
             
             
             if len(temp_position_x)==0:
                 self.framex.append(0)
                 self.framez.append(0)
                 self.F_control.append([])
                 self.FT.append(0)
                 self.FRAMES.append([])
                 self.Cplus.append(0)
                 self.Cminus.append(0) 
                 self.FCplus.append(0)  
                 self.FCinus.append(0)
                 self.ANGLE_CHECK.append([])
                 
             else:
                 F=np.array([[Fx,Fy]])
                 Ft=self.transform_forces([x0],[y0],self.ballx_position[i],self.ballz_position[i],F)
                 self.FT.append(Ft)
                 self.F_control.append(F)

                 (VX,VZ,frames)=self.create_frame(temp_position_x,temp_position_z,self.ballx_position[i],self.ballz_position[i])
                 self.framex.append(VX)
                 self.framez.append(VZ)
                 self.FRAMES.append(frames)

                 (C1,C2)=self.create_friction_cone(VX,VZ,len(temp_position_x))               
                 self.Cplus.append(C1)
                 self.Cminus.append(C2)    
                 self.FCplus.append(C1)  
                 self.FCinus.append(C2)                 
                 self.ANGLE_CHECK.append(temp_angle_check)
      

     def in_hull(self,p, hull):
         """ Create hull """
         from scipy.spatial import Delaunay
         if not isinstance(hull,Delaunay):
             hull = Delaunay(hull)
               
         return (hull.find_simplex(p)>=0)

     def set_up_wrenches(self):
         """
         function to calcuate the wrench matrix and the normalized
         wrench matrux using the radius of gyration
         """
         if self.geom=="square":
             k=(self.ball_radius*2)/(2*np.sqrt(3))
             
         elif self.geom=="circle":
             k = self.ball_radius/np.sqrt(2)
         
         elif self.geom=="triangle":
             const=self.ball_radius*2*np.pi/3
             r=const*np.sqrt(3)/3
             x1=r
             y1=0
             x2=r*np.cos(2*np.pi/3)
             y2=r*np.sin(2*np.pi/3)
             d=np.sqrt((x1-x2)**2 +(y1-y2)**2)
             k = d*.2041
         else:
             k=2.18    
         for i in range(len(self.F_control)):
             if len(self.F_control[i])==0:
                 self.F_mag.append([])
             else:
                 self.F_mag.append(np.sqrt(self.control_forces_x[self.grasp_id[i],i]**2 + self.control_forces_z[self.grasp_id[i],i]**2))
     
         self.WRENCHES = self.calculate_wrenches(self.FRAMES,self.time,self.mu)   
         #self.WRENCH_NORM = self.WRENCHES
         
         self.WRENCH_NORM = self.normalize_wrench(self.WRENCHES,k)




     def calculate_epsilon(self):
         for i in range(len(self.WRENCHES)):
             #print(str(i)+ "of"+ str(len(self.WRENCHES))+"  "+str(len(self.grasp_id[i])))
             if len(self.grasp_id[i])==0:
                 self.EPSILON.append(0)
                 self.HULLWRENCHNORM.append([])
                 self.HULLWRENCHMAGS.append([]) 
                 print(str(i)+ "of"+ str(len(self.WRENCHES))+"  "+str(len(self.grasp_id[i]))+" "+"epsilon=0")
             elif len(self.grasp_id[i])==1:
                 self.EPSILON.append(0)
                 self.HULLWRENCHNORM.append([])
                 self.HULLWRENCHMAGS.append([])
                 print(str(i)+ "of"+ str(len(self.WRENCHES))+"  "+str(len(self.grasp_id[i]))+" "+"epsilon=0")
             elif len(self.grasp_id[i])>1:
                 
                 wrench_norm=self.WRENCH_NORM[i]
                 hull = ConvexHull(wrench_norm)
                 try:
                     
                     self.HULL.append(hull)
                     #(min_dist,hullwrenchmags,hullwrenchnorm,wrenchmags)=self.ferrari_canny_metric(wrench_norm)
                     (hullwrenchnorm,epsilon,hullwrenchmags)=self.calculate_hull(hull,wrench_norm)
                     self.HULLWRENCHNORM.append(hullwrenchnorm)
                     self.HULLWRENCHMAGS.append(hullwrenchmags) 
                     p=[0,0,0]
                     if self.in_hull(p,wrench_norm)==False:
                         self.EPSILON.append(0)
                         print(str(i)+ "of"+ str(len(self.WRENCHES))+"  "+str(len(self.grasp_id[i]))+" "+"epsilon=0")
                     
                     else:
                         print(str(i)+ "of"+ str(len(self.WRENCHES))+"  "+str(len(self.grasp_id[i]))+" "+"epsilon="+str(epsilon))
                         self.EPSILON.append(epsilon)
                
                
                 except:
                     print(str(i)+ "of"+ str(len(self.WRENCHES))+"  "+str(len(self.grasp_id[i]))+" "+"epsilon=0")
                     self.EPSILON.append(0)
                     self.HULLWRENCHNORM.append([])
                     self.HULLWRENCHMAGS.append([])
                     self.HULL.append([])  
                     
     def calculate_epsilon2(self):
         """ This is for calculating the epsilon metrics for contact forces """
         
         for i in range(len(self.temp_wrenches)):
             
             #print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i])))
             if len(self.temp_id[i])==0:
                 print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i]))+" "+"epsilon=0") 
                 self.EPSILON2.append(0)
                 self.HULL2.append([])
                 self.HULLWRENCHNORM2.append([])
                 self.HULLWRENCHMAGS2.append([])                 
             elif len(self.temp_id[i])==1:
                 print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i]))+" "+"epsilon=0")    
                 self.EPSILON2.append(0)
                 self.HULLWRENCHNORM2.append([])
                 self.HULLWRENCHMAGS2.append([])
                 self.HULL2.append([])
                 
             elif np.all((self.temp_wrenches[i]==0)):
                 print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i]))+" "+"epsilon=0")    
                 self.EPSILON2.append(0)
                 self.HULLWRENCHNORM2.append([])
                 self.HULLWRENCHMAGS2.append([])
                 self.HULL2.append([])  
                 
             elif len(self.temp_id[i])>1:
                 wrench_norm=self.temp_wrenches_norm[i]
                 #temp_n=self.temp_n[i]
                 #g=np.max(temp_n)
                 #print(wrench_norm)
                 hull = ConvexHull(wrench_norm)
                 self.HULL2.append(hull)
                 try:
                     (hullwrenchnorm,epsilon,hullwrenchmags)=self.calculate_hull(hull,wrench_norm)
                     self.HULLWRENCHNORM2.append(hullwrenchnorm)
                     self.HULLWRENCHMAGS2.append(hullwrenchmags)
                     
                     p=[0,0,0]
                     if self.in_hull(p,wrench_norm)==False:
                         self.EPSILON2.append(0)
                         print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i]))+" "+"epsilon=0")
                     
                     else:
                        print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i]))+" "+"epsilon="+str(epsilon))
                        self.EPSILON2.append(epsilon)

                 except:
                     print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i]))+" "+"epsilon=0")
                     self.EPSILON2.append(0)
                     self.HULLWRENCHNORM2.append([])
                     self.HULLWRENCHMAGS2.append([])
                     self.HULL2.append([])  
                     
     def calculate_epsilon3(self):
         """ This is for calculating the epsilon metrics for contact forces """
         
         for i in range(len(self.temp_wrenches2)):
             
             #print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i])))
             if len(self.temp_id2[i])==0:
                 self.EPSILON3.append(0)
                 self.HULL3.append([])
                 self.HULLWRENCHNORM3.append([])
                 self.HULLWRENCHMAGS3.append([])    
                 print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0") 
             elif len(self.temp_id2[i])==1:
                 self.EPSILON3.append(0)
                 self.HULLWRENCHNORM3.append([])
                 self.HULLWRENCHMAGS3.append([])
                 self.HULL3.append([])
                 print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")
             
                
             elif np.all((self.temp_wrenches2[i]==0)):
                 print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")    
                 self.EPSILON3.append(0)
                 self.HULLWRENCHNORM3.append([])
                 self.HULLWRENCHMAGS3.append([])
                 self.HULL3.append([])  
                
             elif len(self.temp_id2[i])>1:
                 wrench_norm=self.temp_wrenches_norm2[i]
                 #temp_n=self.temp_n2[i]
                 #g=np.max()
                 #print(wrench_norm)
                 hull = ConvexHull(wrench_norm)
                 self.HULL3.append(hull)
                 try:
                     (hullwrenchnorm,epsilon,hullwrenchmags)=self.calculate_hull(hull,wrench_norm)
                     self.HULLWRENCHNORM3.append(hullwrenchnorm)
                     self.HULLWRENCHMAGS3.append(hullwrenchmags) 
                     p=[0,0,0]
                     if self.in_hull(p,wrench_norm)==False:
                        self.EPSILON3.append(0)
                        print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")
                     
                     else:
                         epsilon=np.round(epsilon/np.max(hullwrenchmags),3)
                         print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon="+str(epsilon))
                         self.EPSILON3.append(epsilon)

                 except:
                     print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")
                     self.EPSILON3.append(0)
                     self.HULLWRENCHNORM3.append([])
                     self.HULLWRENCHMAGS3.append([])
                     self.HULL3.append([]) 

      
     def calculate_epsilon4(self):
         """ This is for calculating the epsilon metrics for contact forces """
         
         for i in range(len(self.temp_wrenches2)):
             
             #print(str(i)+ "of"+ str(len(self.temp_wrenches))+"  "+str(len(self.temp_id[i])))
             if len(self.temp_id2[i])==0:
                 self.EPSILON4.append(0)
                 self.HULL4.append([])
                 self.HULLWRENCHNORM4.append([])
                 self.HULLWRENCHMAGS4.append([])    
                 print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0") 
             elif len(self.temp_id2[i])==1:
                 self.EPSILON4.append(0)
                 self.HULLWRENCHNORM4.append([])
                 self.HULLWRENCHMAGS4.append([])
                 self.HULL4.append([])
                 print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")
             
                
             elif np.all((self.temp_wrenches_norm2[i]==0)):
                 print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")    
                 self.EPSILON4.append(0)
                 self.HULLWRENCHNORM4.append([])
                 self.HULLWRENCHMAGS4.append([])
                 self.HULL4.append([]) 
                
             elif len(self.temp_id2[i])>1:
                 wrench_norm=self.temp_wrenches_norm2[i]
                 #temp_n=self.temp_n2[i]
                 #g=np.max()
                 #print(wrench_norm)
                 
                 #hull = ConvexHull(wrench_norm)
                 #self.HULL4.append(hull)
                 try:
                     hull = ConvexHull(wrench_norm)
                     self.HULL4.append(hull)
                     #(epsilon,hullwrenchnorm,hullwrenchmags)=self.ferrari_canny_metric(wrench_norm)
                     #print("epsilon:",epsilon)
                     (hullwrenchnorm,epsilon,hullwrenchmags)=self.calculate_hull(hull,wrench_norm)
                     #(epsilon,hullwrenchnorm,hullwrenchmags)
                     #self.HULLWRENCHNORM4.append(hullwrenchnorm)
                     #self.HULLWRENCHMAGS4.append(hullwrenchmags)
                     #print("epsilon_",epsilon_)
                           
                     p=[0,0,0]
                     if self.in_hull(p,wrench_norm)==False:
                        self.EPSILON4.append(0)
                        print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")
                     
                     else:
                         epsilon=np.round(epsilon,3)
                         print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon="+str(epsilon))
                         self.EPSILON4.append(epsilon)

                 except:
                     print(str(i)+ "of"+ str(len(self.temp_wrenches2))+"  "+str(len(self.temp_id2[i]))+" "+"epsilon=0")
                     self.EPSILON4.append(0)
                     self.HULLWRENCHNORM4.append([])
                     self.HULLWRENCHMAGS4.append([])
                     self.HULL4.append([]) 


             
     def transform_forces(self,X,Y,XB,YB,F):
         ''' Transform the forces to the local coordinates of contact '''
         Ft=np.zeros((len(X),2))
         frames=np.zeros((len(X),3))
         for i in range(len(X)):
             x0,y0=(X[i]-XB),(Y[i]-YB) # initia positions
             theta=np.arctan2(y0,x0)+np.pi/2 # calculate theta
             frames[i,0],frames[i,1],frames[i,2]=x0,y0,theta
             T=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])      # transformation matrix 
             Ft[i,:]=T.T@F[i,:]        
         return(Ft,frames)

     def min_norm_vector_in_facet(self,facet, wrench_regularizer=1e-8):
          """ Finds the minimum norm point in the convex hull of a given facet (aka simplex) by solving a QP.
          Parameters
          ----------
          facet : 6xN :obj:`numpy.ndarray`
             vectors forming the facet
          wrench_regularizer : float
             small float to make quadratic program positive semidefinite
          Returns
          -------
          float
             minimum norm of any point in the convex hull of the facet
          Nx1 :obj:`numpy.ndarray`
             vector of coefficients that achieves the minimum
          """
          dim = facet.shape[1] # num vertices in facet
    
          # create alpha weights for vertices of facet
          G = facet.T.dot(facet)
          grasp_matrix = G + wrench_regularizer * np.eye(G.shape[0])
    
          # Solve QP to minimize .5 x'Px + q'x subject to Gx <= h, Ax = b
          P = cvx.matrix(2 * grasp_matrix)   # quadratic cost for Euclidean dist
          q = cvx.matrix(np.zeros((dim, 1)))
          G = cvx.matrix(-np.eye(dim))       # greater than zero constraint
          h = cvx.matrix(np.zeros((dim, 1)))
          A = cvx.matrix(np.ones((1, dim)))  # sum constraint to enforce convex
          b = cvx.matrix(np.ones(1))         # combinations of vertices
          
          sol = cvx.solvers.qp(P, q, G, h, A, b,options={'show_progress':False})
          v = np.array(sol['x'])
          
          min_norm = np.sqrt(sol['primal objective'])
          v=facet@v
          return(abs(min_norm),v)
     
    
    
     def ferrari_canny_metric(self,wrenches):
         G=wrenches.T
         hull = cvh.ConvexHull(wrenches)
         min_dist = 100
         wrenchmags=[]
         for i in range(wrenches.shape[0]):
             wrenchmags.append(np.linalg.norm(wrenches[i]))    
         
         
         
         nfacets = np.shape(hull.simplices)[0] #how many facets
         hullwrenchmags = np.zeros(nfacets)
         hullwrenchnorm = np.zeros((nfacets,3))
         i=0
         for v in hull.vertices:
             if np.max(np.array(v)) < G.shape[1]: # because of some occasional odd behavior from pyhull
                 facet = G[:, v]
                 dist, v = self.min_norm_vector_in_facet(facet, wrench_regularizer=1e-10)
             #print(dist)
                 hullwrenchnorm[i,:]=v.T
                 hullwrenchmags[i]=np.linalg.norm(v)
                 if dist < min_dist:
                     min_dist = dist 
                 i=i+1   
             
         return(min_dist,hullwrenchnorm,hullwrenchmags)


     def calculate_hull(self,hull,Wrench):
         """ Create a 3D hull of the wrench space and calculates the epsilon metric """
         origin = Point3D(0,0,0)
         nfacets = np.shape(hull.simplices)[0] #how many facets
         hullwrenchmags = np.zeros(nfacets)
         hullwrenchnorm = np.zeros((nfacets,3))
         i=0
         for s in hull.simplices:
             triangle = Wrench[s]   #convert from simplices to 3D points
             point1 = Point3D(triangle[0])
             point2 = Point3D(triangle[1])
             point3 = Point3D(triangle[2])
             theplane = Plane(point1,point2,point3)
             planedistance =  theplane.distance(origin)
             temp=theplane.normal_vector
             hullwrenchmags[i]=planedistance
             mag=sqrt(N(temp[0])**2 + N(temp[1])**2 + N(temp[2])**2)
             hullwrenchnorm[i,0]=N(temp[0])/mag
             hullwrenchnorm[i,1]=N(temp[1])/mag
             hullwrenchnorm[i,2]=N(temp[2])/mag
             i=i+1
         leastwrench = np.round(np.amin(hullwrenchmags),2)
    
         #print('least wrench (if enclosing), Union hull:',np.round(leastwrench,2))
         epsilon=leastwrench
         
        
         return(hullwrenchnorm,epsilon,hullwrenchmags)   


         
                 
     def normalize_wrench(self,WRENCHES,k):
         WRENCH_NORM=[]
         for i in range(len(WRENCHES)):
            if len(WRENCHES[i])==0:
                WRENCH_NORM.append([])
            else:
                wrench = WRENCHES[i]
                n = wrench.shape[0]
                wrench_norm = np.zeros((n,3))
                for j in range(n):
                    wrench_norm[j,0],wrench_norm[j,1],wrench_norm[j,2]=wrench[j,0],wrench[j,1],wrench[j,2]/k
                    #mag=np.sqrt(wrench_norm[j,0]**2 + wrench_norm[j,1]**2 + wrench_norm[j,2]**2)
                    #mag=np.sqrt(wrench_norm[j,0]**2 + wrench_norm[j,1]**2)
                    #mag=1
                    #wrench_norm[j,0],wrench_norm[j,1],wrench_norm[j,2]=wrench[j,0]/mag,wrench[j,1]/mag,(wrench[j,2]/k)/mag
                WRENCH_NORM.append(wrench_norm)       
         return(WRENCH_NORM)
        
        
     def PTrans(self,x,y,theta):
         ct = np.cos(theta)
         st = np.sin(theta)
         Jbp = np.array([[ct, st, (x*st-y*ct)],[-st, ct, (x*ct+y*st)],[0,0,1]])
         return Jbp
     
     def smooth(self,y, box_pts):
         box = np.ones(box_pts)/box_pts
         y_smooth = np.convolve(y, box, mode='same')
         return y_smooth


        
     def calculate_wrenches(self,FRAMES,time,mu):
         WRENCHES=[]
         for i in range(len(FRAMES)):
             if len(FRAMES[i])==0:
                 WRENCHES.append([])
             else:
                 frames=FRAMES[i]
                 n=frames.shape[0]
                 #f_mag=F_mag[i]
                 phi = np.arctan(mu)
                 wrenches = np.zeros((2*n,3))
                 for j in range(n):
                     fl = np.array([-np.cos(phi), -np.sin(phi), 0])
                     fr = np.array([-np.cos(phi),np.sin(phi), 0])
                     Jb = self.PTrans(frames[j,0],frames[j,1],frames[j,2])  #WrenchUtils.py
                     Jbtrans = Jb.transpose()
                     wrenches[j] = Jbtrans.dot(fl)
                     wrenches[j+n] = Jbtrans.dot(fr)
            
                 WRENCHES.append(wrenches)
                 
         return(WRENCHES)
     
        



     def create_frame(self,X,Y,XB,YB):
         ''' create the local frame of the bots in contact '''
         VX=np.ones(len(X)) # empty vector ones
         VY=np.zeros(len(X)) # empty vector zeros
         VXC=np.zeros((len(X),2)) # empty array of contact x axis
         VYC=np.zeros((len(X),2)) # empty array of contact y axis 
         Frames=np.zeros((len(X),3))
         
         xcenter=XB
         ycenter=YB
         if self.geom=="square":            
             const=self.ball_radius*2
             rx=const
             ry=const
             w=rx/2
             h=ry/2                    
             x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
             y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
             (segments)=self.create_segment(x,y) 
         if self.geom=="triangle":
             const=(self.ball_radius)*2*np.pi/3
             r=const*np.sqrt(3)/3
             x1=r
             y1=0
               
             x2=r*np.cos(2*np.pi/3)
             y2=r*np.sin(2*np.pi/3)
               
             x3=r*np.cos(4*np.pi/3)
             y3=r*np.sin(4*np.pi/3)
               
             x__ = [x1,x2,x3,x1]+xcenter
             y__ = [y1,y2,y3,y1]+ycenter
             (segments)=self.create_segment(x__,y__)

         if self.geom=="square" or self.geom=="triangle":
             for i in range(0,len(X)):
                 #print((Y[i],X[i]))
                 #Fx1=self.Fxn((Y[i],X[i]))
                 #Fy1=self.Fyn((Y[i],X[i]))
                 
                 Fx1=self.PHIDX(X[i],Y[i],segments)
                 Fy1=self.PHIDY(X[i],Y[i],segments)
                 x0=X[i]-(X[i]-Fx1)
                 y0=Y[i]-(Y[i]-Fy1)
          
                 theta=np.arctan2(y0,x0) # calculate theta
                 T=np.array([[-np.sin(theta),-np.cos(theta)],[np.cos(theta),-np.sin(theta)]]) # transformation matrix 
                 VXpp=T@np.array([[1],[0]]) # transform coordinates X
                 VYpp=T@np.array([[0],[1]]) # transform coordinates Y
                 VXpp=VXpp.flatten() # flatten the matrix
                 VYpp=VYpp.flatten() # flatten the matrix
                
                 VXC[i,:]=VXpp # Save the array X
                 VYC[i,:]=VYpp  # save the array Y               
                           
                 x0,y0=(X[i]-XB),(Y[i]-YB) # initia positions           
                 Frames[i,:]=np.array([x0,y0,theta])   
                 
         if self.geom=="import":
             for i in range(0,len(X)):
                 #print((Y[i],X[i]))
                 #Fx1=self.Fxn((Y[i],X[i]))
                 #Fy1=self.Fyn((Y[i],X[i]))
                 
                 Fx1=self.F_random_objdx(X[i],Y[i])
                 Fy1=self.F_random_objdy(X[i],Y[i])
                 x0=X[i]-(X[i]-Fx1)
                 y0=Y[i]-(Y[i]-Fy1)
          
                 theta=np.arctan2(y0,x0) # calculate theta
                 T=np.array([[-np.sin(theta),-np.cos(theta)],[np.cos(theta),-np.sin(theta)]]) # transformation matrix 
                 VXpp=T@np.array([[1],[0]]) # transform coordinates X
                 VYpp=T@np.array([[0],[1]]) # transform coordinates Y
                 VXpp=VXpp.flatten() # flatten the matrix
                 VYpp=VYpp.flatten() # flatten the matrix
                
                 VXC[i,:]=VXpp # Save the array X
                 VYC[i,:]=VYpp  # save the array Y               
                           
                 x0,y0=(X[i]-XB),(Y[i]-YB) # initia positions           
                 Frames[i,:]=np.array([x0,y0,theta])                    
                 
            
         if self.geom=="circle":
             for i in range(0,len(X)):
                 x0,y0=(X[i]-XB),(Y[i]-YB) # initia positions
                 theta=np.arctan2(y0,x0) # calculate theta
                 Frames[i,:]=np.array([x0,y0,theta])
                 T=np.array([[-np.sin(theta),-np.cos(theta)],[np.cos(theta),-np.sin(theta)]]) # transformation matrix 
                 VXpp=T@np.array([[1],[0]]) # transform coordinates X
                 VYpp=T@np.array([[0],[1]]) # transform coordinates Y
                 VXpp=VXpp.flatten() # flatten the matrix
                 VYpp=VYpp.flatten() # flatten the matrix
                
                 VXC[i,:]=VXpp # Save the array X
                 VYC[i,:]=VYpp  # save the array Y      
         return(VXC,VYC,Frames)   

        
     def create_friction_cone(self,VXC,VYC,n):
         ''' Create vectors for friction cone '''
         C1=np.zeros((n,2)) # positive cone
         C2=np.zeros((n,2)) # negative cone 
         theta1=np.arctan(self.mu)
         T1=np.array([[np.cos(theta1),-np.sin(theta1)],[np.sin(theta1),np.cos(theta1)]])
         T2=np.array([[np.cos(-theta1),-np.sin(-theta1)],[np.sin(-theta1),np.cos(-theta1)]])
        
         for i in range((n)):
             C1[i,:]=T1@VYC[i,:]
             C2[i,:]=T2@VYC[i,:]  
         return(C1,C2)  
  


     def find_contact_forces_2(self):
        '''This function for extacting contact forces without regards for grasping'''
        for i in range(len(self.time_contact)-1):
            #print(np.round(self.time_contact[i],2))
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
                        
                        tempx21["nb"+str(num)].append(self.Contact_points_x[j,i])
                        tempz21["nb"+str(num)].append(self.Contact_points_z[j,i])      
                        
                    if temp1[0:3]=="bot":
                        les=len(temp1)
                        num=int(temp1[3:les])
                        tempx2["nb"+str(num)].append(self.x_contact_force2[j,i])
                        tempz2["nb"+str(num)].append(self.z_contact_force2[j,i]) 
                        
                        tempx21["nb"+str(num)].append(self.Contact_points_x[j,i])
                        tempz21["nb"+str(num)].append(self.Contact_points_z[j,i])    
                        
                        
                        
                ##### contact forces for ball        
                if temp1[0:4]=="ball" or temp2[0:4]=="ball" :   
                    if temp1[0:4]=="ball":  
                        tempx3["ballx"].append(self.x_contact_force2[j,i])
                        tempz3["ballz"].append(self.z_contact_force2[j,i])
                        
                        tempx4["ballx"].append(self.Contact_points_x[j,i])
                        tempz4["ballz"].append(self.Contact_points_z[j,i])  
                        
                        tempx5["ballx"].append(self.Dirxx_[j,i])
                        tempz5["ballz"].append(self.Dirxz_[j,i]) 
                        
                        tempx6["ballx"].append(self.Dirzx_[j,i])
                        tempz6["ballz"].append(self.Dirzz_[j,i])
                        
                    if temp2[0:4]=="ball":
                        tempx3["ballx"].append(self.x_contact_force2[j,i])
                        tempz3["ballz"].append(self.z_contact_force2[j,i])
                        
                        tempx4["ballx"].append(self.Contact_points_x[j,i])
                        tempz4["ballz"].append(self.Contact_points_z[j,i])
                        
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

                        tempxxx["nb"+str(num)].append(self.Contact_points_x[j,i])
                        tempzzz["nb"+str(num)].append(self.Contact_points_z[j,i])

                    if temp2[0:4]=="ball" and temp1[0:3]=="bot" :
                        #les=len(temp2)
                        num=int(temp1[3:les])
                        tempxx["nb"+str(num)].append(self.x_contact_force2[j,i])
                        tempzz["nb"+str(num)].append(self.z_contact_force2[j,i])

                        tempxxx["nb"+str(num)].append(self.Contact_points_x[j,i])
                        tempzzz["nb"+str(num)].append(self.Contact_points_z[j,i])

                
                        
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
                                             
                    
             
                  

     def find_pressure(self):
         ''' calculate the internal pressure '''

         for i in range(len(self.time_contact)-1):
             #print(i)
             for j in range(int(self.number_contacts[i])):
                 temp1=self.AN[i][j]
                 temp2=self.BN[i][j]
              
                 # INTERIOR PRESSURE
                 if temp1[0:4]=="gran" or temp2[0:4]=="gran":
                     if temp2[0:4]=="gran":
                         les=len(temp2)
                         num=int(temp2[5:les])
                         if temp2[4]=='b' :
                             r=self.radius2 * np.sqrt(2)
                         else:
                             r=self.radius2 
                         A = np.pi * 2 * r * self.height  
                        
                         les=len(temp2)
                         num=int(temp2[5:les])
                         self.Pressure_x_particles[num,i]=abs((self.Contact_force_x[j,i]/A))+self.Pressure_x_particles[num,i]
                         self.Pressure_z_particles[num,i]=abs((self.Contact_force_z[j,i]/A))+self.Pressure_z_particles[num,i]
                            
  
                     if temp1[0:4]=="gran":
                         if temp1[4]=='b' :
                             r=self.radius2*np.sqrt(2)
                         else:
                             r=self.radius2 
                         A = np.pi * 2 * r * self.height    
                         les=len(temp1)
                         num=int(temp1[5:les])
                         self.Pressure_x_particles[num,i]=abs((self.Contact_force_x[j,i]/A))+self.Pressure_x_particles[num,i]
                         self.Pressure_z_particles[num,i]=abs((self.Contact_force_z[j,i]/A))+self.Pressure_z_particles[num,i]  

                 # BOTS PRESSURE
                 if temp1[0:3]=="bot" or temp2[0:3]=="bot":
                     if temp2[0:3]=="bot":  
                         les=len(temp2)
                         num=int(temp2[3:les])
                         self.Pressure_x_bots[num,i]=abs(self.Contact_force_x[j,i]/(self.bot_width/2 * 2 * np.pi * self.height))+self.Pressure_x_bots[num,i]
                         self.Pressure_z_bots[num,i]=abs(self.Contact_force_z[j,i]/(self.bot_width/2 * 2 * np.pi * self.height))+self.Pressure_z_bots[num,i]
                     else:
                         les=len(temp1)
                         num=int(temp1[3:les])
                         self.Pressure_x_bots[num,i]=abs(self.Contact_force_x[j,i]/(self.bot_width/2 * 2 * np.pi * self.height))+self.Pressure_x_bots[num,i]
                         self.Pressure_z_bots[num,i]=abs(self.Contact_force_z[j,i]/(self.bot_width/2 * 2 * np.pi * self.height))+self.Pressure_z_bots[num,i]
                        

     def find_pressure_mag(self):
        FX=np.vstack([self.Pressure_x_particles])
        FZ=np.vstack([self.Pressure_z_particles])
        X=np.vstack([self.particle_position_x])
        Z=np.vstack([self.particle_position_z])  
        for j in range(len(self.time)-1):
            for i in range(self.ni-1):
                self.MAG_pressure_no_boundary[i,j]=np.sqrt(FZ[i,j]**2+FX[i,j]**2)   
         

        FX=np.vstack([self.Pressure_x_bots,self.Pressure_x_particles])
        FZ=np.vstack([self.Pressure_z_bots,self.Pressure_z_particles])
        X=np.vstack([self.bot_position_x,self.particle_position_x])
        Z=np.vstack([self.bot_position_z,self.particle_position_z])  
        for j in range(len(self.time)-1):
            for i in range(self.nb+self.ni-1):
                self.MAG_pressure[i,j]=np.sqrt(FZ[i,j]**2+FX[i,j]**2)  

     
     def average_pressure(self):
         for j in range(len(self.time)-1):
             self.Mag_avg_pressure.append(np.sum(self.MAG_pressure[:,j])/len(self.MAG_pressure[:,j]))
             self.Mag_avg_pressure_no_boundary.append(np.sum(self.MAG_pressure_no_boundary[:,j])/len(self.MAG_pressure_no_boundary[:,j]))
             
 
    
 
     def save_grasp_parameters(self):
         self.graspParams = {}

     
         self.graspParams["F_control"] = self.F_control 
         self.graspParams["Forces_ball_x"] = self.Forces_ball_x  
         self.graspParams["Forces_ball_z"] = self.Forces_ball_z

         self.graspParams["contact_points_ball_x"] = self.contact_points_ball_x  
         self.graspParams["contact_points_ball_z"] = self.contact_points_ball_z

         self.graspParams["magnitude_forces_on_ball"] = self.magnitude_forces_on_ball
         self.graspParams["torque_ball"] = self.torque_ball

             
         self.graspParams["Pressure_x_bots"] = self.Pressure_x_bots
         self.graspParams["Pressure_z_bots"] = self.Pressure_z_bots
         
         self.graspParams["Pressure_x_particles"] = self.Pressure_x_particles
         self.graspParams["Pressure_z_particles"] = self.Pressure_z_particles
         
         self.graspParams["MAG_pressure"] = self.MAG_pressure
         self.graspParams["MAG_pressure_no_boundary"] = self.MAG_pressure_no_boundary



         
         self.graspParams["bot_contact_forces_x"]=self.bot_contact_forces_x
         self.graspParams["bot_contact_forces_y"]=self.bot_contact_forces_y
         self.graspParams["bot_contact_forces_z"]=self.bot_contact_forces_z        


         self.graspParams["bot_total_forces_x"] = self.bot_total_forces_x
         self.graspParams["bot_total_forces_y"] = self.bot_total_forces_y
         self.graspParams["bot_total_forces_z"] = self.bot_total_forces_z
         
    


         self.graspParams["particle_contact_forces_x"] = self.particle_contact_forces_x
         self.graspParams["particle_contact_forces_y"] = self.particle_contact_forces_y
         self.graspParams["particle_contact_forces_z"] = self.particle_contact_forces_z       


         self.graspParams["particle_total_forces_x"] = self.particle_total_forces_x
         self.graspParams["particle_total_forces_y"] = self.particle_total_forces_y
         self.graspParams["particle_total_forces_z"] = self.particle_total_forces_z
         


         # self.graspParams["Forces_x_contact_particles"] = self.Forces_x_contact_particles
         # self.graspParams["Forces_z_contact_particles"] = self.Forces_z_contact_particles
             
         # self.graspParams["Forces_x_contact_bots"] = self.Forces_x_contact_bots
         # self.graspParams["Forces_z_contact_bots"] = self.Forces_z_contact_bots





         self.graspParams["grasp_id"] = self.grasp_id
             
         self.graspParams["grasp_position_x"] = self.grasp_position_x
         self.graspParams["grasp_position_z"] = self.grasp_position_z

         self.graspParams["grasp_force_x"] = self.grasp_force_x
         self.graspParams["grasp_force_z"] = self.grasp_force_z
         self.graspParams["grasp_torque"] = self.grasp_torque
  
             
         self.graspParams["WRENCHES"] = self.WRENCHES
         self.graspParams["WRENCH_NORM"] = self.WRENCH_NORM
             
         self.graspParams["FRAMES"] = self.FRAMES
             
         self.graspParams["EPSILON"] = self.EPSILON
         self.graspParams["HULLWRENCHNORM"] = self.HULLWRENCHNORM
         self.graspParams["HULLWRENCHMAGS"] = self.HULLWRENCHNORM
             
         self.graspParams["framex"]  = self.framex
         self.graspParams["framez"]  = self.framez

         self.graspParams["FT"]  = self.FT
             
         self.graspParams["Cplus"]  = self.Cplus
         self.graspParams["Cminus"]  = self.Cminus
             
         self.graspParams["FCplus"]  = self.FCplus  
         self.graspParams["FCinus"]  = self.FCinus
             
         self.graspParams["F_mag"]  = self.F_mag
         self.graspParams["HULL"] = self.HULL
             
         self.graspParams["WRENCHXY"] = self.WRENCHXY
         self.graspParams["HULLXY"] = self.HULLXY
                 
         self.graspParams["WRENCHXT"] = self.WRENCHXT
         self.graspParams["HULLXT"] = self.HULLXT

         self.graspParams["WRENCHYT"] = self.WRENCHYT
         self.graspParams["HULLYT"] = self.HULLYT     
         self.graspParams["ANGLE_CHECK"] = self.ANGLE_CHECK
         
         
         
         
         ### THESE ARE FORM COJNTACT POINTS ###
         self.graspParams["temp_offset_theta"]=self.temp_offset_theta
         self.graspParams["temp_frames"]=self.temp_frames
         self.graspParams["temp_theta"]=self.temp_theta
         self.graspParams["temp_id"]=self.temp_id
         self.graspParams["temp_position_x"]=self.temp_position_x
         self.graspParams["temp_position_z"]=self.temp_position_z
         self.graspParams["temp_force_x"]=self.temp_force_x
         self.graspParams["temp_force_z"]=self.temp_force_z
         self.graspParams["temp_wrenches"]=self.temp_wrenches
         self.graspParams["temp_wrenches_norm"]=self.temp_wrenches_norm
         self.graspParams["temp_vx"]=self.temp_vx
         self.graspParams["temp_vy"]=self.temp_vy
         self.graspParams["temp_c1"]=self.temp_c1
         self.graspParams["temp_c2"]=self.temp_c2
         self.graspParams["EPSILON2"] = self.EPSILON2
         self.graspParams["HULLWRENCHNORM2"] = self.HULLWRENCHNORM2
         self.graspParams["HULLWRENCHMAGS2"] = self.HULLWRENCHMAGS2
         self.graspParams["HULL2"] = self.HULL2  
         
         self.graspParams["WRENCHXY2"] = self.WRENCHXY2
         self.graspParams["HULLXY2"] = self.HULLXY2
                 
         self.graspParams["WRENCHXT2"] = self.WRENCHXT2
         self.graspParams["HULLXT2"] = self.HULLXT2

         self.graspParams["WRENCHYT2"] = self.WRENCHYT2
         self.graspParams["HULLYT2"] = self.HULLYT2
         
         
         self.graspParams["temp_offset_theta2"]=self.temp_offset_theta2
         self.graspParams["temp_frame2s"]=self.temp_frames2
         self.graspParams["temp_theta2"]=self.temp_theta2
         self.graspParams["temp_id2"]=self.temp_id2
         self.graspParams["temp_position_x2"]=self.temp_position_x2
         self.graspParams["temp_position_z2"]=self.temp_position_z2
         self.graspParams["temp_force_x2"]=self.temp_force_x2
         self.graspParams["temp_force_z2"]=self.temp_force_z2
         self.graspParams["temp_wrenches2"]=self.temp_wrenches2
         self.graspParams["temp_wrenches_norm2"]=self.temp_wrenches_norm2
         self.graspParams["temp_vx2"]=self.temp_vx2
         self.graspParams["temp_vy2"]=self.temp_vy2
         self.graspParams["temp_c12"]=self.temp_c12
         self.graspParams["temp_c22"]=self.temp_c22
         
         self.graspParams["EPSILON3"] = self.EPSILON3
         self.graspParams["HULLWRENCHNORM3"] = self.HULLWRENCHNORM3
         self.graspParams["HULLWRENCHMAGS3"] = self.HULLWRENCHMAGS3
         self.graspParams["HULL3"] = self.HULL3  
         
         self.graspParams["WRENCHXY3"] = self.WRENCHXY3
         self.graspParams["HULLXY3"] = self.HULLXY3
                 
         self.graspParams["WRENCHXT3"] = self.WRENCHXT3
         self.graspParams["HULLXT3"] = self.HULLXT3

         self.graspParams["WRENCHYT3"] = self.WRENCHYT3
         self.graspParams["HULLYT3"] = self.HULLYT3        
         
         self.graspParams["EPSILON4"] = self.EPSILON4
         self.graspParams["HULLWRENCHNORM4"] = self.HULLWRENCHNORM4
         self.graspParams["HULLWRENCHMAGS4"] = self.HULLWRENCHMAGS4
         self.graspParams["HULL4"] = self.HULL4  
         
         self.graspParams["WRENCHXY4"] = self.WRENCHXY4
         self.graspParams["HULLXY4"] = self.HULLXY4
                 
         self.graspParams["WRENCHXT4"] = self.WRENCHXT4
         self.graspParams["HULLXT4"] = self.HULLXT4

         self.graspParams["WRENCHYT4"] = self.WRENCHYT4
         self.graspParams["HULLYT4"] = self.HULLYT4                
         
       
         
         
         self.graspParams['number_parameters'] = len(self.graspParams)

         np.save(self.mainDirectory+'/'+self.name+'/graspParams.npy',self.graspParams)
         
         
         
     def plot_epsilon(self):
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         epsilon=np.asarray(self.EPSILON)
         axs.plot(self.time,epsilon,color='red',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'$\epsilon$'+" vs time" )
         axs.set_ylabel('$\epsilon$',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_value.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_value.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_value.pdf')         
       
     def plot_epsilon2(self): 
         """ Epsilon regaridng the contact foprces """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         epsilon=np.asarray(self.EPSILON2)
         axs.plot(self.time[0:-2],epsilon,color='blue',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
         y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'$\epsilon_{2}$'+" vs time" )
         axs.set_ylabel('$\epsilon_{2}$',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon2_value.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon2_value.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon2_value.pdf')          
         
   
     def plot_epsilon3(self): 
         """ Epsilon regaridng the contact foprces """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         epsilon=np.asarray(self.EPSILON3)
         axs.plot(self.time[0:-2],epsilon,color='g',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
         y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'$\epsilon_{3}$'+" vs time" )
         axs.set_ylabel('$\epsilon_{3}$',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon3_value.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon3_value.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon3_value.pdf')          
     
     def plot_epsilon4(self): 
         """ Epsilon regaridng the contact foprces """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         epsilon=np.asarray(self.EPSILON4)
         axs.plot(self.time[0:-2],epsilon,color='g',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
         y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'$\epsilon_{4}$'+" vs time" )
         axs.set_ylabel('$\epsilon_{4}$',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.pdf')          
         plt.close('all')
        
     def sort_epsilon_and_theta(self):
        theta=self.THETA[0:-2] 
        epsilon=np.asarray(self.EPSILON4)
        
        for k, g in itertools.groupby(theta):
            self.angle_entries.append(k) 
            self.epsilon_section["theta:"+str(k)]=[]
            
        for i in range(12):
            self.epsilon_theta_section[str(i)]=[]
            
            
        for i in range(len(self.angle_entries)):
            res=np.where(theta==self.angle_entries[i])
            #print(res[0])
            self.epsilon_section["theta:"+str(self.angle_entries[i])].append(epsilon[res[0]])
            epsilon2=epsilon[res[0]]
            res2=np.nonzero(epsilon[res[0]])
            res2=res2[0]
            self.average_epsilon.append(np.mean(epsilon2[res2]))
            self.max_epsilon.append(max(epsilon[res[0]]))
            #print(np.mean(epsilon2[res2]))
        
        count=0
        for i in range(len(self.max_epsilon)):
            
            if count==12:
                count=0
            else:
                count=count
            
            self.epsilon_theta_section[str(count)].append(self.max_epsilon[i])
            count=count+1
        
     def plot_epsilon_vs_theta(self): 
         """ Epsilon vs the angle the system is approaching from """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         epsilon=np.asarray(self.EPSILON4)
         axs.scatter(self.THETA[0:-2],epsilon,color='b',marker='s')
         
         for i in range(len(self.angle_entries)):
             axs.scatter(self.angle_entries[i],self.average_epsilon[i],color='r',marker='s')
         
         x_ticks = np.linspace(self.THETA[0], self.THETA[-2],5,endpoint=True)
         y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'$\epsilon_{4}$'+"vs"+r'$\theta$' )
         axs.set_ylabel('$\epsilon_{4}$',labelpad=1)
         axs.set_xlabel(r'$\theta$' ,labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_v_theta.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_v_theta.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_v_theta.pdf')      
         #plt.close('all')
         
            
         
         color=iter(cm.rainbow(np.linspace(0,1,len(self.angle_entries))))
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         for i in range(len(self.angle_entries)):
             c=next(color)
             epsilon=self.epsilon_section['theta:'+str(self.angle_entries[i])][0]
             axs.plot(epsilon,color=c,label=str(np.round(self.angle_entries[i],2)))    
         
         axs.grid(True)
         axs.legend()
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_section.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_section.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_section.pdf')   
         plt.close('all')
         
         
     def plot_epsilon_vs_theta_section(self):         
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         
         #axs.scatter(self.THETA[0:-2],epsilon,color='b',marker='s')
         
         for i in range(len(self.epsilon_theta_section)):
             entry=int(str(i))
             y=self.epsilon_theta_section[str(i)]
             x=entry*np.ones(len(y))
             axs.scatter(x,y,color='b',marker='s')
             axs.scatter(entry,np.mean(y),color='r',marker='s')
    
         positions=[0,1,2,3,4,5,6,7,8,9,10,11]
         labels=['0','$\pi$/6','$\pi$/3','$\pi$/2','$2\pi$/3','$5\pi$/6','$\pi$','$7\pi$/6','$8\pi$/6','$3\pi$/2','$10\pi$/6','$11\pi$/6']
         axs.set_xticks(positions)
         axs.set_xticklabels(labels,color='k')
         
         axs.set_title(r'$\epsilon$'+" vs "+r'$\theta$' )
         axs.set_ylabel('$\epsilon$',labelpad=1)
         axs.set_xlabel(r'$\theta$' ,labelpad=-2)
         axs.grid(True)
         #axs.legend()
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_theta_section.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_theta_section.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_theta_section.pdf')   
         plt.close('all')
         
     def plot_pressure(self): 
         """ Epsilon regaridng the contact foprces """
 
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         axs.plot(self.time[0:-1],self.Mag_avg_pressure,color='g',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.Mag_avg_pressure), np.max(self.Mag_avg_pressure),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title('Pressue'+" vs time" )
         axs.set_ylabel('Pressure (N/m^2)',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'average_pressure.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'average_pressure.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'average_pressure.pdf')     
     
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         axs.plot(self.time[0:-1],self.Mag_avg_pressure_no_boundary,color='b',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.Mag_avg_pressure_no_boundary), np.max(self.Mag_avg_pressure_no_boundary),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title('Pressue_no boundary'+" vs time" )
         axs.set_ylabel('Pressure (N/m^2)',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'average_pressure_no_boundary.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'average_pressure_no_boundary.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'average_pressure_no_boundary.pdf')          
         plt.close('all')
     
     def plot_contact_number(self):
         """ contact numbers"""
         Num_contact1=[]
         Num_contact2=[]
         for i in range(len(self.temp_id2)):
             Num_contact1.append(len(self.temp_id2[i]))
                      
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         axs.plot(self.time[0:-2],Num_contact1,color='g',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
         y_ticks = np.linspace(0, np.max(Num_contact1),np.max(Num_contact1)+1,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'number_contact'+" vs time" )
         axs.set_ylabel('number of contact',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'number of contact.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'number of contact.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'number of contact.pdf')           
         
     def plot_control_forces(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=2, ncols=1,figsize=(5,4),dpi=300)
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
               
         axs[0].plot(self.time,FX,color='tab:red',linewidth=1,label='x')
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(FX), np.max(FX),5,endpoint=True)
         axs[0].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].set_xticks(np.round(x_ticks,1))
         axs[0].set_yticks(np.round(y_ticks,1))
         axs[0].set_title(r'$\Sigma F_{cx}$'+" vs time" )
         axs[0].set_ylabel('$\Sigma F_{cx}$',labelpad=1)         
         axs[0].grid(True)        
     # axs[0].set_xlim([2.5,10])  
        
         axs[1].plot(self.time,FY,color='tab:blue',linewidth=1,label='y')
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(FY), np.max(FY),5,endpoint=True)
         axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].set_xticks(np.round(x_ticks,1))
         axs[1].set_yticks(np.round(y_ticks,1))
         axs[1].set_title(r'$\Sigma F_{cy}$'+" vs time" )
         axs[1].set_ylabel('$\Sigma F_{cy}$',labelpad=1)
         axs[1].set_xticks(np.round(x_ticks,2))
         axs[1].grid(True)   
     # axs[1].set_xlim([2.5,10])  
        

         
     # axs[2].set_xlim([2.5,10]) 
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(direct+"/control forces.jpg")
         plt.savefig(direct+"/control forces.pdf")
         plt.savefig(direct+"/control forces.svg")
         plt.savefig(direct+"/control forces.eps")
         plt.close('all')          
         
         
     def plot_ball_position(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=2, ncols=1,figsize=(5,4),dpi=300)

               
         axs[0].plot(self.time,self.ballx_position,color='tab:red',linewidth=1,label='x')
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.ballx_position), np.max(self.ballx_position),5,endpoint=True)
         axs[0].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].set_xticks(np.round(x_ticks,2))
         axs[0].set_yticks(np.round(y_ticks,2))
         axs[0].set_title('ball_x_position [x]'+" vs time" )
         axs[0].set_ylabel('ball_x_position [x]',labelpad=1)         
         axs[0].grid(True)        
     # axs[0].set_xlim([2.5,10])  
        
         axs[1].plot(self.time,self.ballz_position,color='tab:blue',linewidth=1,label='y')
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.ballz_position), np.max(self.ballz_position),5,endpoint=True)
         axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].set_xticks(np.round(x_ticks,2))
         #axs[1].set_yticks(np.round(y_ticks,2))
         axs[1].set_title('ball_z_position'+" vs time" )
         axs[1].set_ylabel('ball_z_position [z]',labelpad=1)
         axs[1].set_xticks(np.round(x_ticks,2))
         axs[1].grid(True)   
     # axs[1].set_xlim([2.5,10])  
        

         
     # axs[2].set_xlim([2.5,10]) 
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(direct+"/ball_positions.jpg")
         plt.savefig(direct+"/ball_positions.pdf")
         plt.savefig(direct+"/ball_positions.svg")
         plt.savefig(direct+"/ball_positions.eps")
         plt.close('all') 




     def plot_ball_velocity(self):
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)      
         y1_velx =self.smooth(self.ball_velocity_x[0:-1], 20)
         y2_velz =self.smooth(self.ball_velocity_z[0:-1], 20)
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)

         axs.plot(self.time,self.ball_velocity_x[0:-1],color="tab:blue",linewidth=1,label='vx')
         axs.plot(self.time,y1_velx,color="k",linewidth=1,linestyle='-')
         axs.plot(self.time,self.ball_velocity_z[0:-1],color="tab:red",linewidth=1,label='vz')
         axs.plot(self.time,y2_velz,color="k",linewidth=1,linestyle='--')
         axs.set_xlabel('time (s)',fontsize=8,labelpad=1)
         axs.set_ylabel('velocity (m/s)',labelpad=1)
         axs.set_title('Ball velocity')
         axs.xaxis.set_tick_params(width=.25,length=2)
         axs.yaxis.set_tick_params(width=.25,length=2)
         axs.grid(True,linewidth=0.25)
         #axs.set_ylim([-0.05,0.05])
         fig.legend( loc='upper right', borderaxespad=0,frameon=False)
         #plt.tight_layout()
         #plt.tight_layout()
         plt.savefig(direct+"/ball_velocity.jpg")
         plt.savefig(direct+"/ball_velocity.pdf")
         plt.savefig(direct+"/ball_velocity.svg")
         plt.savefig(direct+"/ball_velocity.eps")
         plt.close('all') 

     def plot_ball_pull_forces_trial(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+self.name,'_pull_trial')    
 
         if not os.path.isdir(direct):
             os.makedirs(direct)  
         count=0    
         for i in range(len(self.time)):
             fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)
  
             #print("FB",np.round(self.FB[i],2))
             axs.plot(self.FB[:i],self.PX[:i]-self.PX[54],color='green',linewidth=5)
             axs.scatter(self.FB[i],self.PX[i]-self.PX[54],color='red',s=20)
             #y_ticks = np.linspace(self.PX[0]-self.PX[0], self.PX[-1]-self.PX[0],5,endpoint=True)
             #x_ticks = np.linspace(np.min(self.FB), np.max(self.FB),5,endpoint=True)
             #axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
             #axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
             #axs.set_xticks(np.round(x_ticks,2))
             #axs.set_yticks(np.round(y_ticks,2))
             axs.set_ylim([0,2])
             #axs.set_title('ball Force ext [x]'+" vs ball position" )
             axs.set_xlabel('Pull Force [N]',labelpad=1)  
             axs.set_ylabel('Ball Position [m]',labelpad=1) 
             axs.grid(True)            
   
             #plt.tight_layout()
             fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
             plt.savefig(direct+"/frame%04d.jpg" % count)
                 
             count=count+1 
            
             plt.close('all')  
         self.create_video('_pull_trial','_pull_trial')    

     def plot_ball_pull_forces(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)
  
         
         axs.plot(self.FB,self.PX-self.PX[0],color='tab:green',linewidth=1,label='x')
         y_ticks = np.linspace(self.PX[0]-self.PX[0], self.PX[-1]-self.PX[0],5,endpoint=True)
         x_ticks = np.linspace(np.min(self.FB), np.max(self.FB),5,endpoint=True)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title('ball Force ext [x]'+" vs ball position" )
         axs.set_xlabel('Pull Force [N]',labelpad=1)  
         axs.set_ylabel('Ball Position [m]',labelpad=1) 
         axs.grid(True)            
   
         plt.tight_layout()
         
         
         plt.savefig(direct+"/ball_pull_force.jpg")
         plt.savefig(direct+"/ball_pull_force.pdf")
         plt.savefig(direct+"/ball_pull_force.svg")
         plt.savefig(direct+"/ball_pull_force.eps")
         plt.close('all') 

     def plot_ball_contact_forces(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=3, ncols=1,figsize=(5,4),dpi=300)
         from scipy.ndimage import gaussian_filter1d
         self.bFx= gaussian_filter1d(self.bFx[80:-1], 50)      
         self.bFz= gaussian_filter1d(self.bFz[80:-1], 50) 
         axs[0].plot(self.time[80:-1],self.bFx,color='tab:red',linewidth=1,label='x')
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.bFx), np.max(self.bFx),5,endpoint=True)
         axs[0].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].yaxis.set_tick_params(width=.25,length=2,pad=1)
         #axs[0].set_xticks(np.round(x_ticks,2))
         #axs[0].set_yticks(np.round(y_ticks,2))
         axs[0].set_title('ball Force [x]'+" vs time" )
         axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
         axs[0].grid(True)        
     # axs[0].set_xlim([2.5,10])  
        
         axs[1].plot(self.time[80:-1],self.bFz,color='tab:blue',linewidth=1,label='x')
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.bFz), np.max(self.bFz),5,endpoint=True)
         axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
         #axs[1].set_xticks(np.round(x_ticks,2))
         #axs[1].set_yticks(np.round(y_ticks,2))
         axs[1].set_title('ball Force [z]'+" vs time" )
         axs[1].set_ylabel('ball Force  [z]',labelpad=1)         
         axs[1].grid(True)   
         
         axs[2].plot(self.TIME,self.FB,color='tab:green',linewidth=1,label='x')
         x_ticks = np.linspace(self.TIME[0], self.TIME[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.FB), np.max(self.FB),5,endpoint=True)
         axs[2].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[2].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[2].set_xticks(np.round(x_ticks,2))
         axs[2].set_yticks(np.round(y_ticks,2))
         axs[2].set_title('ball Force ext [x]'+" vs time" )
         axs[2].set_ylabel('ball Force ext  [x]',labelpad=1)         
         axs[2].grid(True)            
         
     # axs[1].set_xlim([2.5,10])  
        

         
     # axs[2].set_xlim([2.5,10]) 
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(direct+"/ball_contact_force.jpg")
         plt.savefig(direct+"/ball_contact_force.pdf")
         plt.savefig(direct+"/ball_contact_force.svg")
         plt.savefig(direct+"/ball_contact_force.eps")
         plt.close('all')  



         
     def create_frames(self,membrane):
        ''' Create frames for a video '''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        fxb = self.Psi.Fx_point(self.X,self.Y,0,0)
        fyb = self.Psi.Fy_point(self.X,self.Y,0,0)   
        Fz1 = fxb-1*fyb
        Fx1 = -fyb-1*fxb
        for i in range(len(self.time)-2):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(4, 4)
            
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        ax.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        ax.add_patch(patch) 
                        
                elif self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        ax.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        ax.add_patch(patch)                         
                        
                elif self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        ax.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        ax.add_patch(patch)  
                        
                        
                else:
                    if self.F_random_obj(x0-self.ballx_position[i],y0-self.ballz_position[i])<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        ax.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        ax.add_patch(patch)  
                    
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    ax.add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                ax.add_patch(patch)         
     
            if self.control_mode=="grasping":
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    ax.add_patch(patch)
            
                elif self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    ax.add_patch(patch)   
                    
                elif self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    ax.add_patch(patch) 
                    #xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    #yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    #ax.plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
               
                elif self.geom=="import":
                    R=1.25
                    theta1=0.41
                    theta2=2*np.pi-theta1
                    theta=np.linspace(theta1,theta2,100)

                    x1_=-R*np.cos(theta)
                    y1_=R*np.sin(theta)


                    a=.5
                    b=1.65
                    c=2.02

                    w1=.5
                    w2=.63


                    x2_=[x1_[0],x1_[0]-a]
                    y2_=[w1,w1]

                    x3_=[-b,-b]
                    y3_=[w1,w2]

                    x4_=[-b,-c]
                    y4_=[w2,w2]

                    x5_=[-c,-c]
                    y5_=[w2,-w2]

                    x6_=[-c,-b]
                    y6_=[-w2,-w2]

                    x7_=[-b,-b]
                    y7_=[-w1,-w2]

                    x8_=[-b,x1_[-1]]
                    y8_=[-w1,-w1]
                    ax.plot(x1_,y1_,color='k')
                    ax.plot(x2_,y2_,color='k')
                    ax.plot(x3_,y3_,color='k')
                    ax.plot(x4_,y4_,color='k')
                    ax.plot(x5_,y5_,color='k')
                    ax.plot(x6_,y6_,color='k')
                    ax.plot(x7_,y7_,color='k')
                    ax.plot(x8_,y8_,color='k')
      
            #ax.streamplot(self.X,self.Y,Fx1,Fz1,color='b',density = 2,linewidth=0.1,arrowsize=0.25)
            
            plt.title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            plt.gca().set_aspect('equal', adjustable='box')
            
            plt.savefig(direct+"/frame%04d.jpg" % count)
            print(str(i)+ "of"+ str(len(self.time-1)))     
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames','frames')








     def create_frames_zoomed_in(self,membrane,d):
        ''' Create frames for a video '''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_frames_zoomed_in')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        x0b,y0b=self.ballx_position[0],self.ballz_position[0]
        for i in range(len(self.time)-2):   
            
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            const=(wxmax-wxmin)/(wymax-wymin)
            fig = plt.figure(dpi=300)
            fig.set_size_inches(const*4, 4)

            
            ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            const=self.ball_radius*2
            rx=const
            ry=const
            w=rx/2
            h=ry/2                    
            x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
            y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
            #x = x + xcenter*np.ones(len(x))
            #y = y + ycenter*np.ones(len(x))
            (self.segments)=self.create_segment(x,y) 
                
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        ax.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        ax.add_patch(patch)                    
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        ax.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        ax.add_patch(patch)               
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    ax.add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                ax.add_patch(patch)         
     
            if self.control_mode=="grasping":
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    ax.add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='tab:grey')     
                    ax.add_patch(patch)     
        
            #ax.plot(self.xp,self.yp,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            plt.title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            plt.gca().set_aspect('equal', adjustable='box')
            
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')   
        self.create_video('_frames_zoomed_in','_frames_zoomed_in')     
            
        
     def create_frames_contact_forces(self,d):
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct1 = os.path.join(self.mainDirectory+self.name,'_contact_forces_all')    
        if not os.path.isdir(direct1):
            os.makedirs(direct1)
            
            
        direct2 = os.path.join(self.mainDirectory+self.name,'_contact_forces_within_cone')    
        if not os.path.isdir(direct2):
            os.makedirs(direct2)
                       
        count=0

        for i in range(len(self.time)-2):
            F_contact_ballx_entry=self.Force_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            F_contact_ballz_entry=self.Force_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(F_contact_ballx_entry)
            F_contact_ballx_entry=F_contact_ballx_entry[0]['ballx'] #  FURTHER EXTRACTION 
            F_contact_ballz_entry=F_contact_ballz_entry[0]['ballz'] #  FURTHER EXTRACTION 
            
        
            Position_x_contact_entry=self.position_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
            Position_z_contact_entry=self.position_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

            #print(Position_x_contact_entry)
            Position_x_contact_entry=Position_x_contact_entry[0]['ballx'] #  FURTHER EXTRACTION 
            Position_z_contact_entry=Position_z_contact_entry[0]['ballz'] #  FURTHER EXTRACTION 

            dir_xx_contact_ball_entry=self.dir_xx_contact_ball["time_contact"+str(i)]
            dir_xz_contact_ball_entry=self.dir_xz_contact_ball["time_contact"+str(i)]
            dir_zx_contact_ball_entry=self.dir_zx_contact_ball["time_contact"+str(i)]
            dir_zz_contact_ball_entry=self.dir_zz_contact_ball["time_contact"+str(i)]

            dir_xx_contact_ball_entry=dir_xx_contact_ball_entry[0]['ballx']
            dir_xz_contact_ball_entry=dir_xz_contact_ball_entry[0]['ballz']
            dir_zx_contact_ball_entry=dir_zx_contact_ball_entry[0]['ballx']
            dir_zz_contact_ball_entry=dir_zz_contact_ball_entry[0]['ballz']
            
            
            x0b,y0b=self.ballx_position[i],self.ballz_position[i]
            
            
            if self.geom=="square":
                const=self.ball_radius*2-.01
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    

                x__=[w,-w,-w,w,w]
                y__=[h,h,-h,-h,h]
                (segments)=self.create_segment(x__,y__) 

            if self.geom=="triangle":
                const=self.ball_radius*2*np.pi/3
                r=const*np.sqrt(3)/3 -.01
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
                (segments)=self.create_segment(x__,y__)
                
            wxmax=d
            wxmin=-d
            wymin=-d
            wymax=d
            const=(wxmax-wxmin)/(wymax-wymin)


            fig = plt.figure(dpi=300)
            fig.set_size_inches(const*3,3)
            ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
            if self.geom=="square":
                const_=self.ball_radius*2
                xb,yb=0-const_/2,0 - const_/2
                x0_=xb
                y0_=yb
                patch = matplotlib.patches.Rectangle((x0_, y0_),const_, const_,fc='none',edgecolor='tab:grey')     
                ax.add_patch(patch)
            if self.geom=="circle":
                x0_=0
                y0_=0              
                patch = plt.Circle((x0_, y0_),self.ball_radius,fc='none',edgecolor='tab:grey')
                ax.add_patch(patch)
                
            if self.geom=="triangle":               
                const=self.ball_radius*2*np.pi/3
                r=const*np.sqrt(3)/3 +.01
                patch = matplotlib.patches.RegularPolygon((0,0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='tab:grey')
                ax.add_patch(patch) 
                
            if self.geom=="import":
                R=1.25
                theta1=0.41
                theta2=2*np.pi-theta1
                theta=np.linspace(theta1,theta2,100)

                x1_=-R*np.cos(theta)
                y1_=R*np.sin(theta)


                a=.5
                b=1.65
                c=2.02

                w1=.5
                w2=.63
                
                x2_=[x1_[0],x1_[0]-a]
                y2_=[w1,w1]

                x3_=[-b,-b]
                y3_=[w1,w2]

                x4_=[-b,-c]
                y4_=[w2,w2]

                x5_=[-c,-c]
                y5_=[w2,-w2]

                x6_=[-c,-b]
                y6_=[-w2,-w2]

                x7_=[-b,-b]
                y7_=[-w1,-w2]

                x8_=[-b,x1_[-1]]
                y8_=[-w1,-w1]
                ax.plot(x1_,y1_,color='k')
                ax.plot(x2_,y2_,color='k')
                ax.plot(x3_,y3_,color='k')
                ax.plot(x4_,y4_,color='k')
                ax.plot(x5_,y5_,color='k')
                ax.plot(x6_,y6_,color='k')
                ax.plot(x7_,y7_,color='k')
                ax.plot(x8_,y8_,color='k') 
                
            
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
                x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
                ax.text(x0-x0b,y0-y0b,str(j),size=8)
                theta1=np.arctan2(.2,1) #+ frames[j,2]             
                if self.geom=="square" or self.geom=="triangle":
                    Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                    Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                    mag=1
                
                    
                if self.geom=="import":
                    Fx1,Fy1=self.F_random_objdx(x0-x0b,y0-y0b),self.F_random_objdy(x0-x0b,y0-y0b)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=-Fx1/mag
                    Fy1=-Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
                    theta.append(t)
                
                    frames[j,0]=x0-x0b
                    frames[j,1]=y0-y0b
                    frames[j,2]=t
                    
                    
                if self.geom=="circle":
                    Fx1,Fy1=(self.ballx_position[i]-x0),(self.ballz_position[i]-y0)
                    mag=np.sqrt(Fx1**2 + Fy1**2)
                    Fx1=Fx1/mag
                    Fy1=Fy1/mag
                    F_t=np.array([Fx1,Fy1])
                    X.append(x0-x0b)
                    Y.append(y0-y0b)
                    t=self.angle(Fx1, Fy1)-np.pi/2
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
            

                theta1=np.arctan2(.2,1) 
                temp=np.round(np.nan_to_num(np.arccos(np.dot(VYpp ,temp_dirr))),2)
                fx=F_contact_ballx_entry[j]
                fy=F_contact_ballz_entry[j]
                
                mag_=np.sqrt(fx**2 +fy**2)
                f_=np.array([fx/mag_,fy/mag_])
                temp2=np.round(np.nan_to_num(np.arccos(np.dot(f_ ,Vy[j,:]))),2)
 
                if temp<=theta1:
                    
                    if temp2<theta1:
                 
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

            plt.title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            plt.gca().set_aspect('equal', adjustable='box')
            
            plt.savefig(direct1+"/frame%04d.jpg" % count)
            plt.close('all') 


            wxmax=d
            wxmin=-d
            wymin=-d
            wymax=d
            const=(wxmax-wxmin)/(wymax-wymin)
            fig = plt.figure(dpi=300)
            fig.set_size_inches(const*3,3)
            ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
            const_=self.ball_radius*2
            xb,yb=0-const_/2,0 - const_/2
            x0_=xb
            y0_=yb
            if self.geom=="square":
                const_=self.ball_radius*2
                xb,yb=0-const_/2,0 - const_/2
                x0_=xb
                y0_=yb
                patch = matplotlib.patches.Rectangle((x0_, y0_),const_, const_,fc='none',edgecolor='tab:grey')     
                ax.add_patch(patch)
            if self.geom=="circle":
                x0_=0
                y0_=0              
                patch = plt.Circle((x0_, y0_),self.ball_radius,fc='none',edgecolor='tab:grey')
                ax.add_patch(patch)
                
            if self.geom=="triangle":               
                const=self.ball_radius*2*np.pi/3
                r=const*np.sqrt(3)/3
                patch = matplotlib.patches.RegularPolygon((0,0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='tab:grey')
                ax.add_patch(patch) 
             
            if self.geom=="import":
                R=1.25
                theta1=0.41
                theta2=2*np.pi-theta1
                theta=np.linspace(theta1,theta2,100)

                x1_=-R*np.cos(theta)
                y1_=R*np.sin(theta)


                a=.5
                b=1.65
                c=2.02

                w1=.5
                w2=.63
                
                x2_=[x1_[0],x1_[0]-a]
                y2_=[w1,w1]

                x3_=[-b,-b]
                y3_=[w1,w2]

                x4_=[-b,-c]
                y4_=[w2,w2]

                x5_=[-c,-c]
                y5_=[w2,-w2]

                x6_=[-c,-b]
                y6_=[-w2,-w2]

                x7_=[-b,-b]
                y7_=[-w1,-w2]

                x8_=[-b,x1_[-1]]
                y8_=[-w1,-w1]
                ax.plot(x1_,y1_,color='k')
                ax.plot(x2_,y2_,color='k')
                ax.plot(x3_,y3_,color='k')
                ax.plot(x4_,y4_,color='k')
                ax.plot(x5_,y5_,color='k')
                ax.plot(x6_,y6_,color='k')
                ax.plot(x7_,y7_,color='k')
                ax.plot(x8_,y8_,color='k')      
             
                
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

            plt.title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            plt.gca().set_aspect('equal', adjustable='box')
            
            plt.savefig(direct2+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')    
        
            
        
        
        
        
        self.create_video('_contact_forces_within_cone','_contact_forces_within_cone') 
        self.create_video('_contact_forces_all','_contact_forces_all')         
        
     def Forcechains(self):
        """ create plot force chains"""
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_forcechains')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-2):
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            

            fig = plt.figure(dpi=300)
            fig.set_size_inches(4, 4)
            
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
            cmap = plt.cm.get_cmap('seismic')
            boundaries=np.arange(1,100,1)
                           
            norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
            #count=0
            #for i in range(1,len(self.time)-1):
            Fx=self.Contact_force_x[0:int(self.number_contacts[i]),i]
            Fz=self.Contact_force_z[0:int(self.number_contacts[i]),i]

            abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)
    

            x=self.Contact_points_x[0:int(self.number_contacts[i]),i]
            y=self.Contact_points_z[0:int(self.number_contacts[i]),i]
            x2=[]
            y2=[]
            F2=[]
            for j in range(len(abs_force)):
                x2.append(x[j])
                y2.append(y[j])
                F2.append(abs_force[j])
    
            plt.scatter(x2,y2,s=2*np.power(F2,.65),c=F2,cmap=cmap,norm=norm)
            plt.grid(True)
            plt.colorbar()         
            fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')
        
        self.create_video('_forcechains','_forcechains')
        
        
     def Forcechains_arrows(self,d):
        """ create plot force chains"""
        membrane=True
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_contact_arrow_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1): 
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            
            wxmax=x0+d
            wxmin=x0-d
            wymax=y0+d
            wymin=y0-d
            
            wxmin=-2.1
            wxmax=-2.1+1
            wymin=-0.7
            wymax=0.7
            
            const=(wxmax-wxmin)/(wymax-wymin)
            fig = plt.figure(dpi=300)
            fig.set_size_inches(const*4, 4)
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
            x0,y0=self.ballx_position[i],self.ballz_position[i]
        
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
    
             
            if self.geom=="import":
                R=1.25
                theta1=0.41
                theta2=2*np.pi-theta1
                theta=np.linspace(theta1,theta2,100)

                x1_=-R*np.cos(theta)
                y1_=R*np.sin(theta)


                a=.5
                b=1.65
                c=2.02

                w1=.5
                w2=.63
                
                x2_=[x1_[0],x1_[0]-a]
                y2_=[w1,w1]

                x3_=[-b,-b]
                y3_=[w1,w2]

                x4_=[-b,-c]
                y4_=[w2,w2]

                x5_=[-c,-c]
                y5_=[w2,-w2]

                x6_=[-c,-b]
                y6_=[-w2,-w2]

                x7_=[-b,-b]
                y7_=[-w1,-w2]

                x8_=[-b,x1_[-1]]
                y8_=[-w1,-w1]
                ax.plot(x1_,y1_,color='k')
                ax.plot(x2_,y2_,color='k')
                ax.plot(x3_,y3_,color='k')
                ax.plot(x4_,y4_,color='k')
                ax.plot(x5_,y5_,color='k')
                ax.plot(x6_,y6_,color='k')
                ax.plot(x7_,y7_,color='k')
                ax.plot(x8_,y8_,color='k') 
                   
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
    
    
            x=self.Contact_points_x[0:int(self.number_contacts[i]),i]
            y=self.Contact_points_z[0:int(self.number_contacts[i]),i]
            x0,y0=self.ballx_position[i],self.ballz_position[i]
    
            for j in range(len(abs_force)):
                mag=np.sqrt(Fx[j]**2 + Fz[j]**2)
                ax.quiver(x[j],y[j],Fx[j]/mag,Fz[j]/mag,color="purple",scale=10,width=0.005,zorder=1)
                mag2=np.sqrt(Dirxx[j]**2+Dirxz[j]**2)
                mag3=np.sqrt(Dirzx[j]**2+Dirzz[j]**2)
                
                ax.quiver(x[j],y[j],Dirxx[j]/mag2,Dirxz[j]/mag2,color="red",scale=20,width=0.005,zorder=2)
                ax.quiver(x[j],y[j],Dirzx[j]/mag3,Dirzz[j]/mag3,color="blue",scale=20,width=0.005,zorder=2)
    
            fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_contact_arrow_frames','_contact_arrow_frames')        
        
        
     def create_frames_pressure_no_boundary(self):
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        
        direct = os.path.join(self.mainDirectory+self.name,'_pressure_no_boundary_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)

        FX=np.vstack([self.Pressure_x_particles])
        FZ=np.vstack([self.Pressure_z_particles])
        X=np.vstack([self.particle_position_x])
        Z=np.vstack([self.particle_position_z])  
        
        count=0
        
        self.MAG_pressure_no_boundary=np.zeros((self.ni,len(self.time)-1))

        for j in range(len(self.time)-1):
            for i in range(self.ni-1):
                self.MAG_pressure_no_boundary[i,j]=np.sqrt(FZ[i,j]**2+FX[i,j]**2)    


        for i in range(len(self.time)-1):    
            plt.figure(i,figsize=(4,4),dpi=300)
            plt.tricontourf(X[:,i],Z[:,i],self.MAG_pressure_no_boundary[:,i],cmap='jet')
            #plt.plot(X[0:self.nb,i],Z[0:self.nb,i], 'ko ')
            #plt.plot(X[self.nb:self.ni+self.nb,i],Z[self.nb:self.ni+self.nb,i], 'ro ')
            plt.grid(True)
            plt.colorbar()
            plt.xlabel('x position(m)')
            plt.ylabel('z position(m)')
            plt.title('t='+str(round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count) 
            print(str(i)+ "of"+ str(len(self.time)))
            plt.close('all')
            count=count+1   

        self.create_video('_pressure_no_boundary_frames','_pressure_no_boundary_frames')  


     def create_frames_pressure(self):
     
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_pressure_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)

        FX=np.vstack([self.Pressure_x_bots,self.Pressure_x_particles])
        FZ=np.vstack([self.Pressure_z_bots,self.Pressure_z_particles])
        X=np.vstack([self.bot_position_x,self.particle_position_x])
        Z=np.vstack([self.bot_position_z,self.particle_position_z])  
        
        count=0
        self.MAG_pressure=np.zeros((self.ni+self.nb,len(self.time)-1))
        for j in range(len(self.time)-1):
            for i in range(self.nb+self.ni-1):
                self.MAG_pressure[i,j]=np.sqrt(FZ[i,j]**2+FX[i,j]**2)    


        for i in range(len(self.time)-1):    
            plt.figure(i,figsize=(4,4),dpi=300)
            plt.tricontourf(X[:,i],Z[:,i],self.MAG_pressure[:,i],cmap='jet')
            plt.plot(X[0:self.nb,i],Z[0:self.nb,i], 'ko ')
            plt.plot(X[self.nb:self.ni+self.nb,i],Z[self.nb:self.ni+self.nb,i], 'ro ')
            plt.grid(True)
            plt.colorbar()
            plt.xlabel('x position(m)')
            plt.ylabel('z position(m)')
            plt.title('t='+str(round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count) 
            print(str(i)+ "of"+ str(len(self.time)))
            plt.close('all')
            count=count+1   
            
            
        self.create_video('_pressure_frames','_pressure_frames')  
     
        
     def create_frames_control_forces(self,d):
        ''' Create frames for a video '''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_control_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            wxmax=x0+d
            wxmin=x0-d
            wymax=y0+d
            wymin=y0-d
            const=(wxmax-wxmin)/(wymax-wymin)
            fig = plt.figure(dpi=300)
            fig.set_size_inches(const*4, 4)
            ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
            # BOUNDARY ROBOTS
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='black')
                ax.add_patch(patch)
                ax.quiver(x0,y0, self.control_forces_x[j,i],self.control_forces_z[j,i], color="tab:purple", scale=10,width=0.002)
                ax.quiver(x0,y0, self.bot_contact_forces_x[j,i],self.bot_contact_forces_z[j,i], color="tab:blue", scale=1000,width=0.002)            
                ax.quiver(x0,y0, self.bot_total_forces_x[j,i],self.bot_total_forces_z[j,i], color="tab:red", scale=10,width=0.002)            

            if self.geom=="circle":
                x0,y0=self.ballx_position[i],self.ballz_position[i]
                patch = plt.Circle((x0, y0),self.ball_radius,fc='tab:gray',edgecolor='black',linewidth=1)
                ax.add_patch(patch)
            
            if self.geom=="square":
                const_=self.ball_radius*2
                x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2
                patch = patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='tab:grey')     
                ax.add_patch(patch)
                patch = plt.Circle((x0, y0),self.a2,fc='none',edgecolor='tab:blue',linewidth=1)
                ax.add_patch(patch)
            if self.geom=="triangle":
                const_=self.ball_radius*2*np.pi/3
                r=const_*np.sqrt(3)/3
                x0,y0=self.ballx_position[i],self.ballz_position[i]
                patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                ax.add_patch(patch) 
                
            if self.geom=="import":           
                R=1.25
                theta1=0.41
                theta2=2*np.pi-theta1
                theta=np.linspace(theta1,theta2,100)

                x1_=-R*np.cos(theta)
                y1_=R*np.sin(theta)


                a=.5
                b=1.65
                c=2.02

                w1=.5
                w2=.63
                x2_=[x1_[0],x1_[0]-a]
                y2_=[w1,w1]

                x3_=[-b,-b]
                y3_=[w1,w2]

                x4_=[-b,-c]
                y4_=[w2,w2]

                x5_=[-c,-c]
                y5_=[w2,-w2]

                x6_=[-c,-b]
                y6_=[-w2,-w2]

                x7_=[-b,-b]
                y7_=[-w1,-w2]

                x8_=[-b,x1_[-1]]
                y8_=[-w1,-w1]
                ax.plot(x1_,y1_,color='k')
                ax.plot(x2_,y2_,color='k')
                ax.plot(x3_,y3_,color='k')
                ax.plot(x4_,y4_,color='k')
                ax.plot(x5_,y5_,color='k')
                ax.plot(x6_,y6_,color='k')
                ax.plot(x7_,y7_,color='k')
                ax.plot(x8_,y8_,color='k')    
            
            
            
            fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')             
        self.create_video('_control_frames','_control_frames')         
        
        
     def create__frames_robot_forces(self):
        ''' Create frames for for total forces '''
        
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_forces_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            d=1
            wxmax=x0+d
            wxmin=x0-d
            wymax=y0+d
            wymin=y0-d
            const=(wxmax-wxmin)/(wymax-wymin)
            fig = plt.figure(dpi=300)
            fig.set_size_inches(const*4, 4)
            ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
            # BOUNDARY ROBOTS
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='none',edgecolor='k',lw=1)
                ax.add_patch(patch)
                #ax.quiver(x0,y0, self.control_forces_x[j,i],self.control_forces_z[j,i], color="tab:purple", scale=10,width=0.005)
                mag1 = np.sqrt(self.bot_contact_forces_x[j,i]**2 + self.bot_contact_forces_z[j,i]**2)
                mag2 = np.sqrt(self.bot_total_forces_x[j,i]**2 + self.bot_total_forces_z[j,i]**2)
                ax.quiver(x0,y0, self.bot_contact_forces_x[j,i]/mag1,self.bot_contact_forces_z[j,i]/mag1, color="tab:orange", scale=10,width=0.007)            
                ax.quiver(x0,y0, self.bot_total_forces_x[j,i]/mag2,self.bot_total_forces_z[j,i]/mag2, color="tab:purple", scale=10,width=0.007)            

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
                #print("Fx:",np.round(Fx,2))
                #print("Fz:",np.round(Fz,2))
                vx=self.framex[i]
                vz=self.framez[i]
                cp=self.Cplus[i]
                cm=self.Cminus[i]   
                #F=self.F_control[i]
                #F=F.flatten()
                #print(F)
                #Fx=F[0]
                #Fy=F[1]
                #vxn=self.FRAMENX[i]            
            #print('vx',vx,'vxn',vxn)
            #print('sum',vx+vxn)
                #plt.quiver(xo,zo, Fx,Fz, color="tab:purple", scale=10,width=0.002)  
                #ax.quiver(xo,zo, self.control_forces_x[self.grasp_id[i],i],self.control_forces_z[self.grasp_id[i],i], color="tab:purple", scale=10,width=0.005)
                plt.quiver(xo,zo, vx[:,0], vx[:,1], color="tab:blue", scale=10,width=0.007,label='controller dir')  
                plt.quiver(xo,zo, vz[:,0], vz[:,1], color="tab:green", scale=10,width=0.007,label='controller dir') 
                plt.quiver(xo,zo, cp[:,0], cp[:,1], color="tab:red", scale=10,width=0.007)   
                plt.quiver(xo,zo, cm[:,0], cm[:,1], color="tab:red", scale=10,width=0.007)               
            fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
            plt.savefig(direct+"/frame%04d.jpg" % count) 
            count=count+1 
            plt.close('all')  
            
        self.create_video('_forces_frames','_forces_frames') 
         
        
     def create_wrenches_slices_frames(self):
         fm._rebuild()
         plt.rcParams['font.family'] = 'Times New Roman'
         plt.rcParams['mathtext.fontset'] = 'dejavuserif'
         plt.rcParams['font.size'] = 8
         plt.rcParams['axes.linewidth'] = .1
         direct = os.path.join(self.mainDirectory+self.name,'_grasping_frames_wrenches_slices')    
         if not os.path.isdir(direct):
             os.makedirs(direct)
         count=0
         for i in range(len(self.time)-1):    
             fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(5,2),dpi=300)   
             #print(self.grasp_id[i])
             if self.grasp_id[i]!=[] and len(self.grasp_id[i])>1:
                 Wrenchxy=self.WRENCHXY[i]
                 #print(Wrenchxy)
                 hullxy=self.HULLXY[i]
                 
                 Wrenchxt=self.WRENCHXT[i]
                 hullxt=self.HULLXT[i]
                 
                 Wrenchyt=self.WRENCHYT[i]
                 hullyt=self.HULLYT[i]
                 epsilon1=self.EPSILON[i]
        
                
            
                 axs[0].set_aspect('equal', adjustable='box')
                 axs[0].scatter(Wrenchxy[:,0],Wrenchxy[:,1],color='k',s=1)
                 axs[0].fill(Wrenchxy[hullxy.vertices,0], Wrenchxy[hullxy.vertices,1], 'tab:red', alpha=0.3)
                 for j in range((Wrenchxy.shape[0])):
                     axs[0].arrow(0,0,Wrenchxy[j,0],Wrenchxy[j,1],color="green",zorder=3,head_width=0.05,head_length=0.03)
                    
                 #for i in range((minnorm.shape[0])): 
                     #axs[0].arrow(0,0,epsilon1*minnorm[i,0],epsilon1*minnorm[i,1],color="orange",zorder=3,head_width=0.05,head_length=0.03)    
                    
                 patch = plt.Circle((0,0),epsilon1 , fc='tab:blue',alpha=0.5)
                 axs[0].add_patch(patch)
                 axs[0].set_title(r'$x-y$')
                 axs[0].set_xlabel('$x$',labelpad=1)
                 axs[0].set_ylabel('$y$',labelpad=-2)
                 axs[0].xaxis.set_tick_params(width=.25,length=2,pad=1)
                 axs[0].yaxis.set_tick_params(width=.25,length=2,pad=1)
                 xticks=[-1,-0.5,0,0.5,1]
                 yticks=[-1,-0.5,0,0.5,1]
                 #xticks=[-1,-0.5,0,0.5,1]
                 #yticks=[-2,-1,0,1,2]
                 axs[0].set_xticks(np.round(xticks,1))
                 axs[0].set_yticks(np.round(yticks,1))
                 #axs[0].set_xlim([-1,1])
                 #axs[0].set_ylim([-2,2])
                 axs[0].grid(True,linewidth=0.5)  
                
                
                
                
                 axs[1].set_aspect('equal', adjustable='box')
                 axs[1].scatter(Wrenchxt[:,0],Wrenchxt[:,1],color='k',s=1)
                 axs[1].fill(Wrenchxt[hullxt.vertices,0], Wrenchxt[hullxt.vertices,1], 'tab:red', alpha=0.3)
                 patch = plt.Circle((0,0),epsilon1 , fc='tab:blue',alpha=0.5)
                 for j in range((Wrenchxt.shape[0])):
                     axs[1].arrow(0,0,Wrenchxt[j,0],Wrenchxt[j,1],color="green",zorder=3,head_width=0.05,head_length=0.03)
                    
                    
                 #for i in range((minnorm.shape[0])): 
                     #axs[1].arrow(0,0,epsilon1*minnorm[i,0],epsilon1*minnorm[i,2],color="orange",zorder=3,head_width=0.05,head_length=0.03)    
                    
                 axs[1].add_patch(patch)
                 axs[1].set_title(r'$x-\tau$')
                 axs[1].set_xlabel('$x$',labelpad=1)
                 axs[1].set_ylabel(r'$\tau$',labelpad=-2)
                 axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
                 axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
                 xticks=[-1,-0.5,0,0.5,1]
                 yticks=[-1,-0.5,0,0.5,1]
                 #yticks=[-2,-1,0,1,2]
                 axs[1].set_xticks(np.round(xticks,1))
                 axs[1].set_yticks(np.round(yticks,1))
                 #axs[1].set_xlim([-1,1])
                 #axs[1].set_ylim([-2,2])
                 axs[1].grid(True,linewidth=0.5)  
                
                
                 axs[2].set_aspect('equal', adjustable='box')
                 axs[2].scatter(Wrenchyt[:,0],Wrenchyt[:,1],color='k',s=1)
                 axs[2].fill(Wrenchyt[hullyt.vertices,0], Wrenchyt[hullyt.vertices,1], 'tab:red', alpha=0.3)
                 patch = plt.Circle((0,0),epsilon1 , fc='tab:blue',alpha=0.5)
                 for j in range((Wrenchyt.shape[0])):
                     axs[2].arrow(0,0,Wrenchyt[j,0],Wrenchyt[j,1],color="green",zorder=3,head_width=0.05,head_length=0.03)
                    
                 #for i in range((minnorm.shape[0])): 
                     #axs[2].arrow(0,0,epsilon1*minnorm[i,1],epsilon1*minnorm[i,2],color="orange",zorder=3,head_width=0.05,head_length=0.03)    
                    
                 axs[2].add_patch(patch)
                 axs[2].set_title(r'$y-\tau$')
                 axs[2].set_xlabel('$y$',labelpad=1)
                 axs[2].set_ylabel(r'$\tau$',labelpad=0)
                 axs[2].xaxis.set_tick_params(width=.25,length=2,pad=1)
                 axs[2].yaxis.set_tick_params(width=.25,length=2,pad=1)
                 xticks=[-1,-0.5,0,0.5,1]
                 yticks=[-1,-0.5,0,0.5,1]
                 axs[2].set_xticks(np.round(xticks,1))
                 axs[2].set_yticks(np.round(yticks,1))
                 #axs[2].set_xlim([-1,1])
                 #axs[2].set_ylim([-2,2])
                 fig.suptitle('Time= ' + str(np.round(self.time[i],0))+"  $\epsilon$="+str(epsilon1),fontsize=12)
                 plt.tight_layout()
                 axs[2].grid(True,linewidth=0.5) 
                 plt.savefig(direct+"/frame%04d.jpg" % count)  
                 count=count+1 
                 plt.close('all') 

             else:
                 fig.suptitle('Time= ' + str(np.round(self.time[i],0))+"  $\epsilon$="+str(0),fontsize=12)
                 plt.savefig(direct+"/frame%04d.jpg" % count)  
                 count=count+1 
                 plt.close('all')         
        
         self.create_video('_grasping_frames_wrenches_slices','_grasping_frames_wrenches_slices')         



     def create_frames_epsilon(self,membrane,d):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=2, ncols=3,figsize=(12,8),dpi=300)

            # epsilon 1
            epsilon=np.asarray(self.EPSILON4)
            axs[0,1].plot(self.time[:i],epsilon[:i],color='tab:green',linewidth=1)
            axs[0,1].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            axs[0,1].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[0,1].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[0,1].set_xlabel('time [s]',labelpad=-2)
            axs[0,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].grid(True) 

            
            # # pressure
            # axs[1,0].plot(self.time[:i],self.Mag_avg_pressure_no_boundary[:i],color='k',linewidth=1)
            # axs[1,0].scatter(self.time[i-1],self.Mag_avg_pressure_no_boundary[i-1],color='r',s=30)
            # axs[1,0].set_title('Pressue_no boundary: '+str(np.round(self.Mag_avg_pressure_no_boundary[i-1],2)))
            # axs[1,0].set_ylabel('Pressure (N/m^2)',labelpad=1)
            # axs[1,0].set_xlabel('time [s]',labelpad=-2)
            # axs[1,0].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,0].yaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,0].grid(True)
            
            
            # axs[0,2].scatter(np.round(self.THETA[:i],2),epsilon[:i],color='b',marker='s')
            # axs[0,2].scatter(np.round(self.THETA[i-1],2),epsilon[i-1],color='r',s=30)
            # axs[0,2].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            # axs[0,2].set_ylabel('$\epsilon_{1}$',labelpad=1)
            # axs[0,2].set_xlabel(r'$\theta$' ,labelpad=-2)
            # axs[0,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[0,2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[0,2].grid(True)

            
            
            # axs[1,1].plot(self.time[:i],Num_contact1[:i],color='tab:blue',linewidth=1)
            # axs[1,1].scatter(self.time[i-1],Num_contact1[i-1],color='r',s=30)
            # axs[1,1].set_title(r'number_contact: '+str(Num_contact1[i-1]))
            # axs[1,1].set_ylabel('number of contact',labelpad=1)
            # axs[1,1].set_xlabel('time [s]',labelpad=-2)
            # axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,1].grid(True)
    
      
            #### Robot simulation
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[0,0].set_xlim([wxmin,wxmax])
            axs[0,0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0,0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0,0].add_patch(patch)         
     
            if self.control_mode=="grasping":
                patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                axs[0,0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0,0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[0,0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
            axs[0,0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            
          
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined','_frames_combined')  


     def create_frames_pull_epsilon3(self,membrane,d):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined3')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=2,figsize=(6,3),dpi=300)
            
            # epsilon 1
            epsilon=np.asarray(self.EPSILON4)
            axs[1].plot(self.time[:i],epsilon[:i],color='tab:blue',linewidth=1)
            axs[1].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            axs[1].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[1].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[1].set_xlabel('time [s]',labelpad=-2)
            axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1].grid(True) 

            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[0].set_xlim([wxmin,wxmax])
            axs[0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0].add_patch(patch)         
     
            if self.control_mode=="grasping":
                patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                axs[0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
            axs[0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
    
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined3','_frames_combined3')    


     def create_frames_pull_epsilon2(self,membrane,d):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined2')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(9,3),dpi=300)
            
            axs[1].plot(self.FB[:i],self.PX[:i]-self.PX[54],color='tab:green',linewidth=1)
            axs[1].scatter(self.FB[i-1],self.PX[i-1]-self.PX[54],color='tab:red',s=30)
            axs[1].set_ylim([-0.25,1])
            axs[1].set_title('FB='+str(np.round(self.FB[i-1],2))+" Px= "+str(np.round(self.PX[i-1]-self.PX[54],2))  )
            axs[1].set_xlabel('Pull Force [N]',labelpad=1)  
            axs[1].set_ylabel('Ball Position [m]',labelpad=1) 
            axs[1].grid(True) 
            
            
            
            # epsilon 1
            epsilon=np.asarray(self.EPSILON4)
            axs[2].plot(self.time[:i],epsilon[:i],color='tab:blue',linewidth=1)
            axs[2].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            axs[2].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[2].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[2].set_xlabel('time [s]',labelpad=-2)
            axs[2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[2].grid(True) 

            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[0].set_xlim([wxmin,wxmax])
            axs[0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0].add_patch(patch)         
     
            if self.control_mode=="grasping":
                patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                axs[0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
            axs[0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
    
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined2','_frames_combined2')       





     def create_frames_pull_epsilon(self,membrane,d):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(12,4),dpi=300)
            
            axs[0,0].plot(self.FB[:i],self.PX[:i]-self.PX[54],color='tab:blue',linewidth=1)
            axs[0,0].scatter(self.FB[i-1],self.PX[i-1]-self.PX[54],color='tab:red',s=30)
            axs[0,0].set_ylim([-0.25,1])
            axs[0,0].set_title('FB='+str(np.round(self.FB[i-1],2))+" Px= "+str(np.round(self.PX[i-1]-self.PX[54],2))  )
            axs[0,0].set_xlabel('Pull Force [N]',labelpad=1)  
            axs[0,0].set_ylabel('Ball Position [m]',labelpad=1) 
            axs[0,0].grid(True) 
            
            
            
            # epsilon 1
            epsilon=np.asarray(self.EPSILON4)
            axs[0,1].plot(self.time[:i],epsilon[:i],color='tab:green',linewidth=1)
            axs[0,1].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            #axs[0,1].set_xticks(np.round(x_ticks,2))
            #axs[0,1].set_yticks(np.round(y_ticks,2))
            axs[0,1].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[0,1].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[0,1].set_xlabel('time [s]',labelpad=-2)
            axs[0,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].grid(True) 

            # # epsilon 2
            # epsilon=np.asarray(self.EPSILON2)
            # axs[0,2].plot(self.time[:i],epsilon[:i],color='tab:orange',linewidth=1)
            # axs[0,2].scatter(self.time[i-1],epsilon[i-1],color='k',s=30)
            # #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            # #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            # #axs[0,2].set_xticks(np.round(x_ticks,2))
            # #axs[0,2].set_yticks(np.round(y_ticks,2))
            # axs[0,2].set_title(r'$\epsilon_{2}$='+str(np.round(epsilon[i-1],2)))
            # axs[0,2].set_ylabel('$\epsilon_{2}$',labelpad=1)
            # axs[0,2].set_xlabel('time [s]',labelpad=-2)
            # axs[0,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[0,2].yaxis.set_tick_params(width=.25,length=2,pad=1)    
            # axs[0,2].grid(True) 
            
            
            
            # pressure
            axs[0,2].plot(self.time[:i],self.Mag_avg_pressure_no_boundary[:i],color='cyan',linewidth=1)
            axs[0,2].scatter(self.time[i-1],self.Mag_avg_pressure_no_boundary[i-1],color='r',s=30)
            #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            #y_ticks = np.linspace(np.min(self.Mag_avg_pressure_no_boundary), np.max(self.Mag_avg_pressure_no_boundary),10,endpoint=True)
            #axs[1,0].set_xticks(np.round(x_ticks,2))
            #axs1,0].set_yticks(np.round(y_ticks,2))
            axs[0,2].set_title('Pressue_no boundary: '+str(np.round(self.Mag_avg_pressure_no_boundary[i-1],2)))
            axs[0,2].set_ylabel('Pressure (N/m^2)',labelpad=1)
            axs[0,2].set_xlabel('time [s]',labelpad=-2)
            axs[0,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,2].grid(True)
            
            
            
            # # epsilon 3
            # epsilon=np.asarray(self.EPSILON3)
            # axs[1,1].plot(self.time[:i],epsilon[:i],color='m',linewidth=1)
            # axs[1,1].scatter(self.time[i-1],epsilon[i-1],color='k',s=30)
            # #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            # #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            # #axs[1,1].set_xticks(np.round(x_ticks,2))
            # #axs[1,1].set_yticks(np.round(y_ticks,2))
            # axs[1,1].set_title(r'$\epsilon_{3}$='+str(np.round(epsilon[i-1],2)))
            # axs[1,1].set_ylabel('$\epsilon_{3}$',labelpad=1)
            # axs[1,1].set_xlabel('time [s]',labelpad=-2)
            # axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)    
            # axs[1,1].grid(True) 
            
            
            # # epsilon 4
            # epsilon=np.asarray(self.EPSILON4)
            # axs[1,2].plot(self.time[:i],epsilon[:i],color='tab:purple',linewidth=1)
            # axs[1,2].scatter(self.time[i-1],epsilon[i-1],color='k',s=30)
            # #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            # #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            # #axs[1,2].set_xticks(np.round(x_ticks,2))
            # #axs[1,2].set_yticks(np.round(y_ticks,2))
            # axs[1,2].set_title(r'$\epsilon_{4}$='+str(np.round(epsilon[i-1],2)))
            # axs[1,2].set_ylabel('$\epsilon_{4}$',labelpad=1)
            # axs[1,2].set_xlabel('time [s]',labelpad=-2)
            # axs[1,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,2].yaxis.set_tick_params(width=.25,length=2,pad=1)    
            # axs[1,2].grid(True)            
    
    
    
            axs[1,1].plot(self.time[:i],Num_contact1[:i],color='tab:brown',linewidth=1)
            axs[1,1].scatter(self.time[i-1],Num_contact1[i-1],color='r',s=30)
            #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            #y_ticks = np.linspace(0, np.max(Num_contact1),np.max(Num_contact1)+1,endpoint=True)
            #axs[2,1].set_xticks(np.round(x_ticks,2))
            #axs[2,1].set_yticks(np.round(y_ticks,2))
            axs[1,1].set_title(r'number_contact: '+str(Num_contact1[i-1]))
            axs[1,1].set_ylabel('number of contact',labelpad=1)
            axs[1,1].set_xlabel('time [s]',labelpad=-2)
            axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].grid(True)
    
            axs[1,2].plot(self.time[:i],self.ball_velocity_x[:i],color="tab:grey",linewidth=1,label='vx')
            axs[1,2].scatter(self.time[i-1],self.ball_velocity_x[i-1],color='r',s=30)
            axs[1,2].set_title(r'ball velocityx: '+str(np.round(self.ball_velocity_x[i-1],2)))
            axs[1,2].set_ylabel('m/s',labelpad=1)
            axs[1,2].set_xlabel('time [s]',labelpad=-2)
            axs[1,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,2].grid(True)
    
    
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[1,0].set_xlim([wxmin,wxmax])
            axs[1,0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[1,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[1,0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[1,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[1,0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[1,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[1,0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[1,0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[1,0].add_patch(patch)         
     
            if self.control_mode=="grasping":
                patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                axs[1,0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[1,0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[1,0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[1,0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[1,0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
            axs[1,0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            
            
            
            
            
            
            
            
            
            
            
            
            
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined','_frames_combined')        
        
        
        
     def plot_Wrench_space_3D(self,entry):
         """ Plot the 3D wrench space visual for a specified entry """
         i = entry
         epsilon1=self.EPSILON[i]
         hullwrenchmags=self.HULLWRENCHMAGS[i]
         hullwrenchnorm=self.WRENCH_NORM[i]
         Wrench=self.WRENCH_NORM[i]
         hull=self.HULL[i]
         res=np.where(hullwrenchmags==np.amin(hullwrenchmags))
         #print(res)
         
         #minnorm=hullwrenchnorm[res[0],:]
         #minnorm=np.asarray(minnorm[0])
         #minnorm=np.round(minnorm,3)
         #print(minnorm)
         #or i in range(minnorm.shape[0]):
             #print("MINNORM_x : {}, MINNORM_y :{} , MINNORM_z : {}".format(minnorm[i,0], minnorm[i,1],minnorm[i,2]))    
             
         fig3D = plt.figure(figsize=(3,3),dpi=300)
         ax3D = fig3D.add_subplot(111, projection='3d')
         ax3D.set_box_aspect((1, 1, 1)) 
         i=0
         for s in hull.simplices:
        
             s = np.append(s, s[0])  # Cycle back to the first coordinate
             ax3D.plot(Wrench[s, 0], Wrench[s, 1], Wrench[s, 2], "r-",linewidth=0.5)
             ax3D.scatter(Wrench[s, 0], Wrench[s, 1], Wrench[s, 2], marker='o')
             ax3D.scatter([0], [0], [0], marker='x')
            
         #for i in range(minnorm.shape[0]):
         #    ax3D.plot([0,epsilon1*minnorm[i,0]],[0,epsilon1*minnorm[i, 1]], [0,epsilon1*minnorm[i, 2]], "k-")
         r=epsilon1
         u, v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
         x = r*np.cos(u)*np.sin(v)
         y = r*np.sin(u)*np.sin(v)
         z = r*np.cos(v)
         ax3D.plot_wireframe(x, y, z, color="tab:green",linewidth=.25)

         ax3D.set_xlabel('fx')
         ax3D.set_ylabel('fy')
         ax3D.set_zlabel('tau')
         ax3D.set_xlim3d(-1.5,1.5)
         ax3D.set_ylim3d(-1.5,1.5)
         ax3D.set_zlim3d(-1.5,1.5)
         plt.show()       
        
        
     def plot_Wrench_space_3D2(self,entry):
         """ Plot the 3D wrench space visual for a specified entry """
         i = entry
         epsilon1=self.EPSILON[i]
         hullwrenchmags=self.HULLWRENCHMAGS[i]
         hullwrenchnorm=self.WRENCH_NORM[i]
         Wrench=self.WRENCH_NORM[i]
         hull=self.HULL[i]
         res=np.where(hullwrenchmags==np.amin(hullwrenchmags))
         #print(res)
         
         #minnorm=hullwrenchnorm[res[0],:]
         #minnorm=np.asarray(minnorm[0])
         #minnorm=np.round(minnorm,3)
         #print(minnorm)
         #or i in range(minnorm.shape[0]):
             #print("MINNORM_x : {}, MINNORM_y :{} , MINNORM_z : {}".format(minnorm[i,0], minnorm[i,1],minnorm[i,2]))    
             
         fig3D = plt.figure(figsize=(4,4),dpi=300)
         ax3D = fig3D.add_subplot(111, projection='3d')
         ax3D.set_box_aspect((1, 1, 1)) 
         i=0
         for i in range(Wrench.shape[0]):
             ax3D.quiver(0,0,0, Wrench[i, 0], Wrench[i, 1], Wrench[i, 2], "r-",linewidth=0.5)
             
         #for i in range(minnorm.shape[0]):
         #    ax3D.plot([0,epsilon1*minnorm[i,0]],[0,epsilon1*minnorm[i, 1]], [0,epsilon1*minnorm[i, 2]], "k-")
         # r=epsilon1
         # u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
         # x = r*np.cos(u)*np.sin(v)
         # y = r*np.sin(u)*np.sin(v)
         # z = r*np.cos(v)
         # ax3D.plot_wireframe(x, y, z, color="tab:green",linewidth=.25)
         #for i in ["x", "y", "z"]:
         #    eval("ax3D.set_{:s}label('{:s}')".format(i, i))
         ax3D.set_xlabel('fx')
         ax3D.set_ylabel('fy')
         ax3D.set_zlabel('tau')
         ax3D.set_xlim3d(-1.5,1.5)
         ax3D.set_ylim3d(-1.5,1.5)
         ax3D.set_zlim3d(-1.5,1.5)
         plt.show()         
        
        
        
        
     def create_video(self,framename,videoname):
         #import pdb    
         img_array = []
         for index, filename in enumerate(glob.glob(self.mainDirectory+'/'+self.name+'/'+framename+'/'+'/*.jpg')):
             #pdb.set_trace()
             img = cv2.imread(filename)
             height, width, layers = img.shape
             size = (width,height)
             img_array.append(img)
         out = cv2.VideoWriter(self.mainDirectory+'/'+self.name+'/'+videoname+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, (width,height))    

         for i in range(len(img_array)):
            out.write(img_array[i])
         out.release()  
        
        
        
         removing_files = glob.glob(self.mainDirectory+'/'+self.name+'/'+framename+'/'+'/*.jpg')
         for i in removing_files:
             os.remove(i)
        


     def normalize(self,F):
        (fy,fx)=np.gradient(F)
        return(F/np.sqrt(F**2 + fy**2 + fx**2))

     def line_(self,x,y,x1,x2,y1,y2):
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/(np.sqrt((x2-x1)**2 + (y2-y1)**2)))

     def parabola(self,x,y,px,py):
        return((x-px)**2 - py - y)



     def equivalence(self,w1,w2,m):
        return(w1*w2/((w1**m +w2**m)**(1/m)))

     def Union(self,w1,w2):
        return(((w1+w2)/2) + ((np.sqrt((w1-w2)**2))/2))

     def intersection(self,w1,w2):
        return(((w1+w2)/2) - ((np.sqrt((w1-w2)**2))/2))          
        
        
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
    
    
    
    
    
    
    
    
    
     def Trim(self,f,t):
        """ Trim function for two functions  """
        phi=np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((phi-t)/2)**2))

     def Trimx(self,f,t,fx,tx):
        """Derivative Trim function for two functions wrt x  """
        term1 = (2*(f**3)*fx + tx*t)/(np.sqrt(f**4 + t**2)) - tx
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fx
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)
        
     def Trimy(self,f,t,fy,ty):
        """Derivative Trim function for two functions wrt y """
        term1 = (2*(f**3)*fy + ty*t)/(np.sqrt(f**4 + t**2)) - ty
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fy
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)    
      

     def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


     def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


     def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


     def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


     def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

     def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

     def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

     def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

     def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   
    
    
     def circle(self,x,y,R,a,b):
        return(abs((R**2 - (x-a)**2 - (y-b)**2)))

    
    
     def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
     def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))    
    
    
     def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)

     def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) * self.dphix_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)

     def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) * self.dphiy_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m))**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term3      
        R=(-term1*term2/term3)
        return(R)    
    
    

    
                  
    
    
     def F_random_objdx(self,x,y):
        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)


        a=.5
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]


        x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1]]
        y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1]]
        (self.segments)=self.create_segment(x_,y_)
        f1=self.circle(x,y,R,0,0)
        f1x=self.dphix_circle(x,y,0,0,R)
        f2 = self.phi_segments(x,y,self.segments)
        f2x = self.dphix_segments(x,y,self.segments)   
        Fx = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2x + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1x - ((self.m*(f2**self.m)*f2x)/f2 + (self.m*(f1**self.m)*f1x)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        return(Fx)
    
     def F_random_objdy(self,x,y):
        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)


        a=.5
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]

        x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1]]
        y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1]]
        (self.segments)=self.create_segment(x_,y_)
        f1=self.circle(x,y,R,0,0)
        f1y=self.dphiy_circle(x,y,0,0,R)
        f2 = self.phi_segments(x,y,self.segments)
        f2y = self.dphiy_segments(x,y,self.segments)   
        Fy = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2y + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1y - ((self.m*(f2**self.m)*f2y)/f2 + (self.m*(f1**self.m)*f1y)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        return(Fy)
    
    
     def F_random_obj(self,x,y):
        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)


        a=.5
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]

        x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1]]
        y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1]]


        (self.segments)=self.create_segment(x_,y_)

        phi1_=self.phi_segments(x,y,self.segments)


        C=self.circle(x,y,R,0,0)
        phi2_=C
        phi3_=self.equivalence(phi1_,phi2_,self.m)
        return(phi3_)    
    
    
    
    
    
    
    
    
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