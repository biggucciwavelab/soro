# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 18:26:43 2020

@author: dmulr
"""
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import math 
from numpy import savetxt
import matplotlib.pyplot as plt
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
from scipy import signal
import scipy.constants 
from scipy import optimize
import csv

# In[Robot object]
class robot:
    def __init__(self,nb,diameter,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,R,pathind,geom,xc,zc,skind,ratioM):
        # number of interior
        self.nb=nb 
        self.xc=xc
        self.zc=zc
        # diameter
        self.diameter=diameter
        self.diameter2=0
        #height
        self.height=height
        # density
        self.rowr=rowr
        # surface properties
        self.material=material
        # floor
        self.body_floor=body_floor
        # radius of ring
        self.R=R
        # robot position
        self.xb={}; self.yb={}; self.zb={}
        # velocity position
        self.xvb={}; self.yvb={}; self.zvb={}
        self.phib={}
        # total force
        self.Ftxb={}; self.Ftyb={}; self.Ftzb={}
        self.Spring_force={}
        self.spring_length={}
        self.Psi={} # heading angle
        # empty object arrays
        self.my_system=my_system
        self.bots=[]; self.Springs=[]
        self.obj=obj; self.force=[]
        # is object fixed
        self.fixed=fixed
        # Geometry of robots
        self.geom=geom
        self.pathind=pathind
        '''
        # geometry of robots
        square: robot is cube  
        cylinder: robot is cylinder
        sphere
        '''
        self.XL=[]; self.ZL=[]
        
        # spring rleated things
        self.k=k; self.rl=rl
        self.type_spring=type_spring
        self.p1=0; self.p2=self.diameter/2
        self.p3=0; self.p4=-self.diameter/2
        self.h=0; self.mem=3
        
        # Colors
        col_y = chrono.ChColorAsset(); col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        col_b = chrono.ChColorAsset(); col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        col_g = chrono.ChColorAsset(); col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        col_p = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        
        # %% Initialize some parameters for skin particles
        self.skind = skind   # diameter of cylinders for skin particles
        self.ratioM = ratioM   # ratio of membrane skin particles to big bots
        
        self.ratioO = 7      # ratio of outer skin particles to big bots
        self.ratioI = 7      # ratio of inner skin particles to big bots
        self.skinrho = 2000  # density of skin particles [kg/m^3]
        skinO=[]        # empty matrix of outer skin cylinders
        skinM=[]        # empty matrix of membrane skin cylinders
        skinI=[]        # empty matrix of inner skin cylinders
        self.ko=10         # spring constant (skin outside)
        self.bo=0            # damping constant (skin outside)
        self.km=200       # spring constant (skin membrane)
        self.bm=1            # damping constant (skin membrane)
        self.ki=50          # spring constant (skin inside)
        self.bi=0          # damping constant (skin inside)
        
        # %% data arrays of robots
        for i in range(self.nb):
            # positions
            self.xb["botx{0}".format(i)]=[]
            self.yb["boty{0}".format(i)]=[]
            self.zb["botz{0}".format(i)]=[]
            # velocities
            self.xvb["botx{0}".format(i)]=[]
            self.yvb["boty{0}".format(i)]=[]
            self.zvb["botz{0}".format(i)]=[]
            
            # forces 
            self.Ftxb["botx{0}".format(i)]=[]
            self.Ftyb["boty{0}".format(i)]=[]
            self.Ftzb["botz{0}".format(i)]=[]
            
            # spring lengths
            self.spring_length["spring{0}".format(i)]=[]
            self.Spring_force["spring{0}".format(i)]=[]

            # postion 
            theta=i*2*np.pi/self.nb
            x=self.R*np.cos(theta)+self.xc
            y=.5*height
            z=self.R*np.sin(theta)+self.zc

            # create body
            #bot = chrono.ChBody()
            # cylinder
            if self.geom=="cylinder":
                self.rowr2=self.rowr
                self.height2=self.height
                self.diameter2=self.diameter
                
                bot = chrono.ChBodyEasyCylinder(self.diameter2, self.height2,self.rowr2,True,True)
                # set position
                bot.SetPos(chrono.ChVectorD(x,y,z))
                bot.SetName('bot'+str(i))
                bot.SetId(i)
                # material
                bot.SetMaterialSurface(self.material)
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
                myforcex = chrono.ChForce()
                bot.AddForce(myforcex)
                myforcex.SetMode(chrono.ChForce.FORCE)
                myforcex.SetDir(chrono.VECT_X)
                #myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
                self.force.append(myforcex)
            
                # y forces    
                myforcey = chrono.ChForce()
                bot.AddForce(myforcey)
                myforcey.SetMode(chrono.ChForce.FORCE)
                myforcey.SetDir(chrono.VECT_Y)
                #myforcey.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
                self.force.append(myforcey)
            
            # z forces            
                myforcez = chrono.ChForce()
                bot.AddForce(myforcez)
                myforcez.SetMode(chrono.ChForce.FORCE)
                myforcez.SetDir(chrono.VECT_Z)
                self.force.append(myforcez)
                #myforcez.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))

            # square
            if self.geom=="square":
                if self.active[i]==1.0:
                    print('act',self.active[i],i)
                    self.rowr2=self.rowr
                    self.height2=self.height
                    self.diameter2=2*self.diameter
                    bot = chrono.ChBodyEasyBox(self.diameter2,self.height2,self.diameter2,self.rowr2,True,True,self.material)
                else:
                    self.rowr2=self.rowr/2
                    self.height2=.75*self.height
                    self.diameter2=1.5*self.diameter
                    bot = chrono.ChBodyEasyBox(self.diameter2,self.height2,self.diameter2,self.rowr2,True,True,self.material)
                    
                bot.SetPos(chrono.ChVectorD(x,y,z))
                #bot.SetMaterialSurface(self.material)
                bot.SetName('bot'+str(i))
                bot.SetId(i)
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)

            if self.geom=="sphere":
                if self.active[i]==1:
                    
                    self.rowr2=self.rowr
                    self.height2=self.height
                else:
                    mr=.1       # mass
                    volume=np.pi*.25*self.height*(self.diameter)**2   # calculate volume
                    self.rowr2=mr/volume # calculate density of robot
                bot = chrono.ChBodyEasySphere(self.diameter,self.rowr2,True,True,self.material)  # contact material)
                bot.SetPos(chrono.ChVectorD(x,y,z))
                #bot.SetMaterialSurface(self.material)
                bot.SetName('bot')
                bot.SetId(i)
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
             
            

            #bot.SetMaxSpeed(.15)
            #bot.SetLimitSpeed(True)
            bot.AddAsset(col_b)
                
            if self.pathind==i:   
                bot.AddAsset(col_p)               

            # set collision
            bot.SetCollide(True)
            # set fixed
            bot.SetBodyFixed(self.fixed)
            
            # link to floor
            pt=chrono.ChLinkMatePlane()
            pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            self.my_system.AddLink(pt)
                        # add bot to object array 
            self.my_system.Add(bot)
            self.bots.append(bot)
            self.obj.append(bot)    
            if self.type_spring=="None":
                           
            # add bot to object array 
                print('None')
            # %% variable force springs 
            if self.type_spring=="var" and self.mem!=3:
                
                # link springs
                if i>=1:
                    ground=chrono.ChLinkSpring()
                    ground.Initialize(self.bots[i-1], bot,True,chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),True)
                    ground.Set_SpringK(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    ground.AddAsset(col_b)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    
                # Last spring
                if i==self.nb-1:  
                    ground=chrono.ChLinkSpring()
                    ground.Initialize(bot, self.bots[0], True, chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringK(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    ground.AddAsset(col_b)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    
            # %% constant force spring 
            if self.type_spring=="const" and self.mem!=3:
                
                # link springs
                if i>=1:
                    ground=chrono.ChLinkSpring()
                    ground.Initialize(self.bots[i-1], bot,True,chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringF(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    ground.AddAsset(col_b)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    
                # Last spring
                if i==self.nb-1:  
                    ground=chrono.ChLinkSpring()
                    ground.Initialize(bot, self.bots[0], True, chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringF(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    ground.AddAsset(col_b)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground)            
            
        
            # %% membrane particles
            if self.mem==3 and self.type_spring!="None":
                b_ang=2*np.pi/self.nb                   # angle between centers of bots
                o_ang=np.arctan(self.diameter/self.R)   # angle offset for radius of bot
                p_ang=np.arctan(self.skind/self.R)           # angle offset for radius of skin particle
                
                # Between this bot and last bot
                if i>=1 and i<self.nb:
                    for j in range(1,self.ratioM+1,1):
                        # Initial postion of each particle
                        theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ratioM) + p_ang
                        x=self.R*np.cos(theta)+self.xc
                        y=.52*self.height
                        z=self.R*np.sin(theta)+self.zc
                        
                        # Create particles   
                        #skinm = chrono.ChBody()
                        #skinm =chrono.ChBodyEasySphere(skind/2,skinrho,True,True)
                        #skinm = chrono.ChBody()
                        skinm = chrono.ChBodyEasyCylinder(self.skind/2, .5*self.height,self.skinrho,True,True)
                        skinm.SetPos(chrono.ChVectorD(x,y,z))
                        #skinm.SetMaterialSurface(self.material)
                        skinm.SetNoGyroTorque(True)
                        skinm.SetName('skin')
                        skinm.SetId(i)
                        # rotate them
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        
#                        # collision model
#                        skinm.GetCollisionModel().ClearModel()
#                        skinm.GetCollisionModel().AddCylinder(skind/2,skind/2,(.75*self.height/2)) # hemi sizes
#                        skinm.GetCollisionModel().BuildModel()
#                        skinm.SetCollide(True)
                        
                        # Attach springs    
                        if j>1:
                            ground=chrono.ChLinkSpring()
                            p1=0; p2=self.skind/2
                            p3=0; p4=-self.skind/2
                            h=self.height/4
                    
                            ground.Initialize(skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                            ground.Set_SpringK(self.km)
                            ground.Set_SpringR(self.bm)
                            ground.AddAsset(col_p)
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground)
                            
                            ground1=chrono.ChLinkSpring()
                            ground1.Initialize(skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True)
                            ground1.Set_SpringK(self.km)
                            ground1.Set_SpringR(self.bm)
                            ground1.AddAsset(col_p)
                            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground1)                                
                        
                        #Link to cylinder
                        if j==1:
                            skinm.AddAsset(col_p)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinm,self.bots[i])
                            self.my_system.AddLink(glue)
                            # Link last particle with this bot
                            if i>=2:
                                glue=chrono.ChLinkMateFix()
                                glue.Initialize(skinM[-1],self.bots[-1])
                                self.my_system.AddLink(glue)
                            
                        if j==self.ratioM:
                            skinm.AddAsset(col_p)
                            
                        self.my_system.Add(skinm)
                        skinM.append(skinm)
                    
                # Between this bot and first bot
                if i==self.nb-1:
                    for j in range(1,self.ratioM+1,1):
                        # Initial postion of each particle
                        theta=(i+1)*b_ang + j*(b_ang-o_ang-p_ang)/(self.ratioM) + p_ang
                        x=self.R*np.cos(theta)+self.xc
                        y=.52*self.height
                        z=self.R*np.sin(theta)+self.zc
                        # Create particles
                        
#                        skinm =chrono.ChBodyEasySphere(skind/2,skinrho,True,True)
#                        skinm = chrono.ChBody()
                        skinm = chrono.ChBodyEasyCylinder(self.skind/2, .5*self.height,self.skinrho,True,True)
                        skinm.SetPos(chrono.ChVectorD(x,y,z))
                        skinm.SetMaterialSurface(self.material)
                        skinm.SetNoGyroTorque(True)
                        skinm.SetName('skin'+str(i))
                        skinm.SetId(i)
                        # rotate them
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        
                        # collision model
#                        skinm.GetCollisionModel().ClearModel()
#                        skinm.GetCollisionModel().AddCylinder(skind/2,skind/2,(.75*self.height/2)) # hemi sizes
#                        skinm.GetCollisionModel().BuildModel()
#                        skinm.SetCollide(True)
                        
                        # Attach springs    
                        if j>1:
                            ground=chrono.ChLinkSpring()
                            p1=0; p2=self.skind/2
                            p3=0; p4=-self.skind/2
                            h=self.height/4
                    
                            ground.Initialize(skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                            ground.Set_SpringK(self.km)
                            ground.Set_SpringR(self.bm)
                            ground.AddAsset(col_y)
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground)
                            
                            ground1=chrono.ChLinkSpring()
                            ground1.Initialize(skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True)
                            ground1.Set_SpringK(self.km)
                            ground1.Set_SpringR(self.bm)
                            ground1.AddAsset(col_y)
                            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground1)  
                              

                                
                        #Link to cylinder
                        if j==1:
                            skinm.AddAsset(col_p)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinm,self.bots[0])
                            self.my_system.AddLink(glue)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinM[-1],self.bots[0])
                            self.my_system.AddLink(glue)
                         
                        if j==self.ratioM:
                            skinm.AddAsset(col_p)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinm,self.bots[1])
                            self.my_system.AddLink(glue)
                            
                        self.my_system.Add(skinm)
                        skinM.append(skinm)
                        #glue=chrono.ChLinkMateFix()
                        #glue.Initialize(skinM[-1],self.bots[1])
                        #self.my_system.AddLink(glue)
    # %% Initialize some parameters for skin particles
            # if self.mem== 1:
            #     skind = 0.015   # diameter of cylinders for skin particles
            #     ratioO = 7      # ratio of outer skin particles to big bots
            #     ratioI = 7      # ratio of inner skin particles to big bots
            #     skinrho = 1000  # density of skin particles [kg/m^3]
            #     skinO=[]        # empty matrix of outer skin cylinders
            #     skinI=[]        # empty matrix of inner skin cylinders
            #     ko=100           # spring constant (skin outside)
            #     bo=1            # damping constant (skin outside)
            #     ki=100           # spring constant (skin inside)
            #     bi=1            # damping constant (skin inside)
            #     col_y = chrono.ChColorAsset()
            #     col_y.SetColor(chrono.ChColor(0.44, .11, 52))
            
            # # %% Create outer "skin"
            #     so=self.R+(self.diameter)+1.5*(skind/2)
            #     no=ratioO*len(self.bots)
            #     for i in range (no):
                
            #     # Initial postion of each bot
            #     theta=i*2*np.pi/no
            #     x=so*np.cos(theta)
            #     y=.52*self.height
            #     z=so*np.sin(theta)
                
            #     # Create bots    
            #     skino = chrono.ChBody()
            #     skino = chrono.ChBodyEasyCylinder(skind/2, .25*self.height,skinrho,True,True,self.material)
            #     skino.SetPos(chrono.ChVectorD(x,y,z))
            #     #skino.SetMaterialSurface(self.material)
            #     skino.SetNoGyroTorque(True)
                
            #     # rotate them
            #     rotation1 = chrono.ChQuaternionD()
            #     rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
            #     skino.SetRot(rotation1)
                
            #     # collision model
            #     #skino.GetCollisionModel().ClearModel()
            #     #skino.GetCollisionModel().AddCylinder(skind/2,skind/2,(.75*self.height/2)) # hemi sizes
            #     #skino.GetCollisionModel().BuildModel()
            #     #skino.SetCollide(True)
            #     skino.SetBodyFixed(False)
                    
            #     # Attach springs    
            #     if i>=1:
            #         ground=chrono.ChLinkSpring()
            #         p1=0; p2=skind/2
            #         p3=0; p4=-skind/2
            #         h=self.height/4
            
            #         ground.Initialize(skinO[i-1], skino,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
            #         ground.Set_SpringK(ko)
            #         ground.Set_SpringR(bo)
            #         ground.AddAsset(col_y)
            #         ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #         self.my_system.AddLink(ground)
                    
            #         ground1=chrono.ChLinkSpring()
            #         ground1.Initialize(skinO[i-1], skino,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
            #         ground1.Set_SpringK(ko)
            #         ground1.Set_SpringR(bo)
            #         ground1.AddAsset(col_y)
            #         ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #         self.my_system.AddLink(ground1)
                    
            #     # Last spring
            #         if i==no-1:        
            #             ground=chrono.ChLinkSpring()
            #             ground.Initialize(skino, skinO[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
            #             ground.Set_SpringK(ko)
            #             ground.Set_SpringR(bo)
            #             ground.AddAsset(col_y)
            #             ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #             self.my_system.AddLink(ground)
                        
            #             ground1=chrono.ChLinkSpring()
            #             ground1.Initialize(skino, skinO[0],True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
            #             ground1.Set_SpringK(ko)
            #             ground1.Set_SpringR(bo)
            #             ground1.AddAsset(col_y)
            #             ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #             self.my_system.AddLink(ground1)
                        
            #     #Link to cylinder
            #     if i%ratioO==0:
            #         skino.AddAsset(col_y)
            #         glue=chrono.ChLinkMateFix()
            #         glue.Initialize(skino,self.bots[int((i*len(self.bots))/no)])
            #         self.my_system.AddLink(glue)
                    
            #     self.my_system.Add(skino)
            #     skinO.append(skino)
            
            # # %% Create inner "skin"
            # if self.mem==2:
            #     si=self.R-(self.diameter)-1.5*(skind/2)
            #     ni=ratioI*len(self.bots)
            #     for i in range (ni):
                    
            #         # Initial postion of each bot
            #         theta=i*2*np.pi/ni
            #         x=si*np.cos(theta)
            #         y=.52*self.height
            #         z=si*np.sin(theta)
                    
            #         # Create bots    
            #         skini = chrono.ChBody()
            #         skini = chrono.ChBodyEasyCylinder(skind/2, .5*self.height,skinrho,self.material)
            #         skini.SetPos(chrono.ChVectorD(x,y,z))
            #         #skini.SetMaterialSurface(self.material)
            #         skino.SetNoGyroTorque(True)
                    
            #         # rotate them
            #         rotation1 = chrono.ChQuaternionD()
            #         rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
            #         skini.SetRot(rotation1)
                    
            #         # collision model
            #         skini.GetCollisionModel().ClearModel()
            #         skini.GetCollisionModel().AddCylinder(skind/2,skind/2,(.75*self.height/2)) # hemi sizes
            #         skini.GetCollisionModel().BuildModel()
            #         skini.SetCollide(True)
            #         skini.SetBodyFixed(False)
                        
            #         # Attach springs    
            #         if i>=1:
            #             ground=chrono.ChLinkSpring()
            #             p1=0; p2=skind/2
            #             p3=0; p4=-skind/2
            #             h=self.height/4
                
            #             ground.Initialize(skinI[i-1], skini,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
            #             ground.Set_SpringK(ki)
            #             ground.Set_SpringR(bi)
            #             ground.AddAsset(col_y)
            #             ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #             self.my_system.AddLink(ground)
                        
            #             ground1=chrono.ChLinkSpring()
            #             ground1.Initialize(skinI[i-1], skini,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
            #             ground1.Set_SpringK(ko)
            #             ground1.Set_SpringR(bo)
            #             ground1.AddAsset(col_y)
            #             ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #             self.my_system.AddLink(ground1)
                        
            #         # Last spring
            #             if i==ni-1:        
            #                 ground=chrono.ChLinkSpring()
            #                 ground.Initialize(skini, skinI[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
            #                 ground.Set_SpringK(ki)
            #                 ground.Set_SpringR(bi)
            #                 ground.AddAsset(col_y)
            #                 ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #                 self.my_system.AddLink(ground)
                            
            #                 ground1=chrono.ChLinkSpring()
            #                 ground1.Initialize(skini, skinI[0],True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
            #                 ground1.Set_SpringK(ko)
            #                 ground1.Set_SpringR(bo)
            #                 ground1.AddAsset(col_y)
            #                 ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            #                 self.my_system.AddLink(ground1)
                    
            #         #Link to cylinder
            #         if i%ratioO==0:
            #             skini.AddAsset(col_y)
            #             glue=chrono.ChLinkMateFix()
            #             glue.Initialize(skini,self.bots[int((i*len(self.bots))/ni)])
            #             self.my_system.AddLink(glue)
                        
            #         self.my_system.Add(skini)
            #         skinI.append(skini)

                    
    # %% return system
    def return_system(self):
        return(self.my_system,self.Springs,self.bots,self.obj,self.force)

    # save position data
    def save_data_position(self):
        for i in range(self.nb):
            self.xb['botx'+str(i)].append(self.bots[i].GetPos().x)
            self.yb['boty'+str(i)].append(self.bots[i].GetPos().y)
            self.zb['botz'+str(i)].append(self.bots[i].GetPos().z)

    # save velocity data
    def save_data_velocity(self):
        for i in range(self.nb):
            self.xvb['botx'+str(i)].append(self.bots[i].GetPos_dt().x)
            self.yvb['boty'+str(i)].append(self.bots[i].GetPos_dt().y)
            self.zvb['botz'+str(i)].append(self.bots[i].GetPos_dt().z)    
            
    # save force data            
    def save_data_Forces(self):
        for i in range(self.nb):
            self.Ftxb["botx"+str(i)].append(self.bots[i].Get_Xforce().x)
            self.Ftyb["boty"+str(i)].append(self.bots[i].Get_Xforce().y)
            self.Ftzb["botz"+str(i)].append(self.bots[i].Get_Xforce().z)

            
     # save spring data       
    def save_data_spring_force(self):
        if len(self.Springs)!=0:
            for i in range(self.nb):
                self.spring_length["spring"+str(i)].append(self.Springs[i].Get_SpringLength())
                if self.type_spring=="var":
                    self.Spring_force["spring"+str(i)].append(self.Springs[i].Get_SpringLength()*self.Springs[i].Get_SpringK())
    
    # return spring data 
    def return_spring_data(self):
        return(self.Spring_force,self.spring_length)
        
    # return position data
    def return_position_data(self):
        return(self.xb,self.yb,self.zb)
        
    # return velocity data
    def return_velocity_data(self):
        return(self.xvb,self.yvb,self.zvb)
        
    # return force data    
    def return_force_data(self):
        return(self.Ftxb,self.Ftyb,self.Ftzb)
    
    # return last position
    def return_last_position(self):
        for i in range(self.nb):
            self.XL.append(self.xb['botx'+str(i)][-1])
            self.ZL.append(self.zb['botz'+str(i)][-1])
            
        np.savez("points.npz",allow_pickle=True,XL=self.XL,ZL=self.ZL)
        return(self.XL,self.ZL)
        
# In[Interior Particles]
'''
Interior Particles
'''
class Interiors:
    def __init__(self,nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,material,fixed,mode,granmode,R,xc,zc):
        self.xc=xc
        self.zc=zc
        # robots diameter
        self.diameter=diameter
        # particles diameter
        self.diameter2=diameter2
        # number of robots
        self.nb=nb    
        self.R=R
        #self.R=.063/(np.sin(np.pi/nb))
        self.rowp=rowp
        self.height=height
        self.particles=[]
        self.obj=obj
        self.body_floor=body_floor
        self.my_system=my_system
        self.material=material
        self.fixed=fixed
        self.xp={}
        self.yp={}
        self.zp={}
        self.xvp={}
        self.yvp={}
        self.zvp={}
        self.Ftxp={}
        self.Ftyp={}
        self.Ftzp={}
        self.n=[]
        self.ni=0
        self.mode=mode
        self.granmode=granmode
        self.bound_force=[]
        # no interiors
        if self.mode=='empty':
            self.ni=0
            self.n=np.array([0])
        # max interiors
        if self.mode=="max":
            (n)=self.MaxValues()
            self.n=n[0]
            (self.ni)=np.sum(self.n)
        # not max interiors
        if self.mode=="nmax":
            (self.n)=self.Interior()
            (self.ni)=np.sum(self.n)
        # max non homo interios
        if self.mode=="nonhnmax":
            #self.n=2*np.array([29,32,18,18,8])
            
            self.n=np.array([45,55,34,40,22,22,10])
            (self.ni)=np.sum(self.n)
        
        # diamters are the same size
        if self.granmode=="homo":  
            count=0
            for i in range(self.n.size):
                for j in range(self.n[i]):
                    self.xp["gransx{0}".format(count)]=[]
                    self.yp["gransy{0}".format(count)]=[]
                    self.zp["gransz{0}".format(count)]=[]
                    self.xvp["gransvx{0}".format(count)]=[]
                    self.yvp["gransvy{0}".format(count)]=[]
                    self.zvp["gransvz{0}".format(count)]=[]
                    self.Ftxp["gransFx{0}".format(count)]=[]
                    self.Ftyp["gransFy{0}".format(count)]=[]
                    self.Ftzp["gransFz{0}".format(count)]=[]
                    count=count+1

                    R2=self.diameter2*self.n[i]/(np.pi)
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xc
                    y=.5*self.height
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zc
                    # create body
                    #gran = chrono.ChBody()
                    #gran = chrono.ChBodyEasySphere(self.diameter2,self.rowp,True,True)
                    gran = chrono.ChBodyEasyCylinder(self.diameter2, self.height,self.rowp,True,True)
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.material)
                    gran.SetName('gran')
                    gran.SetId(i)
                    # Create collision model
                    # gran.GetCollisionModel().ClearModel()
                    # gran.GetCollisionModel().AddCylinder(self.diameter2/2,self.diameter2/2,self.height/2) # hemi sizes
                    # gran.GetCollisionModel().BuildModel()
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    # add color
                    col_r = chrono.ChColorAsset()
                    col_r.SetColor(chrono.ChColor(1, 0, 0))
                    gran.AddAsset(col_r)
                    # mate to floor
                    pt=chrono.ChLinkMatePlane()
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    self.my_system.AddLink(pt)
                    
                    # Add constraining force object
                    forcec=chrono.ChForce()
                    gran.AddForce(forcec)
                    forcec.SetMode(chrono.ChForce.FORCE)
                    forcec.SetDir(chrono.VECT_Z)
                    self.bound_force.append(forcec)
                    # add to system
                    self.my_system.Add(gran)
                    self.obj.append(gran)
                    self.particles.append(gran)
        # if diamters are not the same size            
        if self.granmode=="nonhomo": 
            count=0
            for i in range(self.n.size):
                print(i)
                if i%2==0:
                    self.diameter3=self.diameter2*(2**.5)
                else:
                    self.diameter3=self.diameter2
                for j in range(self.n[i]):
                    self.xp["gransx{0}".format(count)]=[]
                    self.yp["gransy{0}".format(count)]=[]
                    self.zp["gransz{0}".format(count)]=[]
                    self.xvp["gransvx{0}".format(count)]=[]
                    self.yvp["gransvy{0}".format(count)]=[]
                    self.zvp["gransvz{0}".format(count)]=[]
                    self.Ftxp["gransFx{0}".format(count)]=[]
                    self.Ftyp["gransFy{0}".format(count)]=[]
                    self.Ftzp["gransFz{0}".format(count)]=[]                  
                    count=count+1
                    R2=self.diameter3*self.n[i]/(np.pi)
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xc
                    y=.5*self.height
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zc
                    # create body
                    gran = chrono.ChBody()
                    #gran = chrono.ChBodyEasySphere(self.diameter2,self.rowp,True,True)
                    gran = chrono.ChBodyEasyCylinder(self.diameter3, self.height,self.rowp,True,True)
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.material)
                    gran.SetName('gran'+str(count))
                    gran.SetId(i)
                    # Create collision model
                    # gran.GetCollisionModel().ClearModel()
                    # gran.GetCollisionModel().AddCylinder(self.diameter2/2,self.diameter2/2,self.height/2) # hemi sizes
                    # gran.GetCollisionModel().BuildModel()
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    # add color
                    col_r = chrono.ChColorAsset()
                    col_r.SetColor(chrono.ChColor(1, 0, 0))
                    gran.AddAsset(col_r)
                    # mate to floor
                    #pt=chrono.ChLinkMatePlane()
                    #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    #self.my_system.AddLink(pt)
                    
                    # Add constraining force object
                    forcec=chrono.ChForce()
                    gran.AddForce(forcec)
                    forcec.SetMode(chrono.ChForce.FORCE)
                    forcec.SetDir(chrono.VECT_Z)
                    self.bound_force.append(forcec)
                    # add to system
                    self.my_system.Add(gran)
                    self.obj.append(gran)
                    self.particles.append(gran)
        
# max interior particles
    def MaxValues(self):
        Rin=self.R
        ngrans1=int(self.R/(self.diameter2))
        ri=np.zeros((1,ngrans1))
        ni=np.zeros((1,ngrans1))
        radii=Rin-(self.diameter2)
        for i in range(ngrans1):
            remainder=((self.diameter2))*i
            ri[:,i]=radii-remainder
            print(ri[:,i])
            ni[:,i]=np.floor((ri[:,i]*np.pi)/self.diameter2)
        self.n=np.asarray(ni,dtype=int)
        return(self.n)
        
    # interior     
    def Interior(self):
        #self.n=np.arange(self.nb-5,5,-10)   # array of interior robots
        #self.n=np.arange(self.nb+8,5,-10)

        #self.n=np.array([30,6])
        #self.n=np.array([30,25,10,5])
        #self.n=np.array([30,25,20,15,10,5])
        #self.n=np.array([35,30,25,20,15,10,5])
        #self.n=np.array([12,8])
        self.n=np.arange(self.nb+10,5,-6)   # array of interior robots
        #self.n=np.array([69,63,57,51,45,39,33,27,21,15,9,3])
        return(self.n)
        
    # return system
    def return_system(self):
        return(self.my_system,self.particles,self.obj,self.bound_force)
# save position data
    def save_data_position(self):
        for i in range(self.ni):
            self.xp["gransx"+str(i)].append(self.particles[i].GetPos().x)
            self.yp["gransy"+str(i)].append(self.particles[i].GetPos().y)
            self.zp["gransz"+str(i)].append(self.particles[i].GetPos().z)
        return(self.xp,self.yp,self.zp)
        
    # save velocity data
    def save_data_velocity(self):
        for i in range(self.ni):
            self.xvp['gransvx'+str(i)].append(self.particles[i].GetPos_dt().x)
            self.yvp['gransvy'+str(i)].append(self.particles[i].GetPos_dt().y)
            self.zvp['gransvz'+str(i)].append(self.particles[i].GetPos_dt().z)    
            
    # save force data            
    def save_data_Forces(self):
        for i in range(self.ni):
            self.Ftxp["gransFx"+str(i)].append(self.particles[i].Get_Xforce().x)
            self.Ftyp["gransFy"+str(i)].append(self.particles[i].Get_Xforce().y)
            self.Ftzp["gransFz"+str(i)].append(self.particles[i].Get_Xforce().z)  
    
  # return position data      
    def return_position_data(self):
        return(self.xp,self.yp,self.zp)
        
    # return velocity data
    def return_velocity_data(self):
        return(self.xvp,self.yvp,self.zvp)
        
    # return force data    
    def return_force_data(self):
        return(self.Ftxp,self.Ftyp,self.Ftzp)   
# In[BALL]
'''
Creates a ball for grabbing 
'''  
class Ball:
    def __init__(self,control_type,my_system,body_floor,obj,material,*args):
        self.control_type=control_type
        self.my_system=my_system
        self.body_floor=body_floor
        self.obj=obj
        self.material=material
        self.balls=[]
        self.ballx=[]
        self.ballz=[]
        self.Fballx=0
        self.Fballz=0
        self.nsides=12
        self.bforce=[]
        self.geom="circle"
        self.fixed=True
        self.path='C:/Users/dmulr/OneDrive/Documents/soro_chrono/python/Pychrono/Strings/Grabbing/Grab_sim/shapes/'
        self.bx=[]
        self.by=[]
        self.bz=[]
        self.bvx=[]
        self.bvy=[]
        self.bvz=[]
        self.bFx=[]
        self.bFy=[]
        self.bFz=[]
        if (self.control_type=="pot_field_grab" or self.control_type=="grab_drag" or self.control_type=="pot_field_grab_A" or self.control_type=="grab_drag_A" \
            or self.control_type=="grab_drag_2" or self.control_type=="pot_field_grab_2" or self.control_type=="verify"):
            self.mb=args[0][7]
            self.diameter=args[0][8]
            self.height=args[0][9]
            self.volume=args[0][10]
            self.rowr=args[0][11]
            self.x=args[0][12]
            self.z=args[0][13]
            self.y=self.height/2
        
        if self.control_type=="verify":
            self.mb=args[0][7]
            self.diameter=args[0][8]
            self.height=args[0][9]
            self.volume=args[0][10]
            self.rowr=args[0][11]
            self.x=args[0][12]
            self.z=args[0][13]
            self.y=self.height/2
            
        if self.control_type=="local_field":
            self.diameter=args[0][3]
            self.rowr=args[0][7]
            self.height=args[0][6]
            self.x=args[0][4]
            self.y=self.height/2
            self.z=args[0][5]    
            
        if self.geom=="circle":
        #Create ball
            ball = chrono.ChBody()
            ball = chrono.ChBodyEasyCylinder(self.diameter, self.height,self.rowr,True,True)
            # set position
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName('ball')
            ball.SetCollide(True)
            ball.SetBodyFixed(self.fixed)
            # material
            ball.SetMaterialSurface(self.material)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(0, 1, 0))
            ball.AddAsset(col_g)
            #ball.SetCollide(True)
            #ball.SetBodyFixed(self.fixed)
            pt=chrono.ChLinkMatePlane()
            pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
            self.my_system.AddLink(pt)
            self.obj.append(ball)
            self.balls.append(ball) 
            self.my_system.Add(ball)
            
#             myforcex = chrono.ChForce()
#                ball.AddForce(myforcex)
#                myforcex.SetMode(chrono.ChForce.FORCE)
#                myforcex.SetDir(chrono.VECT_X)
#                self.bforce.append(myforcex)
#                # force y    
#                myforcey = chrono.ChForce()
#                ball.AddForce(myforcey)
#                myforcey.SetMode(chrono.ChForce.FORCE)
#                myforcey.SetDir(chrono.VECT_Y)
#                self.bforce.append(myforcey)
#                # force z            
#                myforcez = chrono.ChForce()
#                ball.AddForce(myforcez)
#                myforcez.SetMode(chrono.ChForce.FORCE)
#                myforcez.SetDir(chrono.VECT_Z)
                #self.bforce.append(myforcez)
                #self.my_system.Add(ball)             
            if self.geom=="square":
                ball = chrono.ChBodyEasyBox(self.diameter,self.height,self.diameter,self.rowr,True,True)
                ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
                ball.SetMaterialSurface(self.material)
            if self.geom=="star":
                ball = chrono.ChBody()
                # Attach a visualization shape .
                # First load a .obj from disk into a ChTriangleMeshConnected:
                mesh_for_visualization = chrono.ChTriangleMeshConnected()
                mesh_for_visualization.LoadWavefrontMesh(self.path+'star.obj')
                #mesh_for_visualization.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
                visualization_shape = chrono.ChTriangleMeshShape()
                visualization_shape.SetMesh(mesh_for_visualization)
                ball.AddAsset(visualization_shape)

                mesh_for_collision = chrono.ChTriangleMeshConnected()
                mesh_for_collision.LoadWavefrontMesh(self.path+'star.obj')
                # Optionally: you can scale/shrink/rotate the mesh using this:
                #mesh_for_collision.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
                ball.GetCollisionModel().ClearModel()
                ball.GetCollisionModel().AddTriangleMesh(
                mesh_for_collision, # the mesh 
                False,  # is it static?
                False)  # is it convex?
                ball.GetCollisionModel().BuildModel()        
                ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
                ball.SetMass(16)
                ball.SetInertiaXX(chrono.ChVectorD(0.270,0.400,0.427))
                ball.SetInertiaXY(chrono.ChVectorD(0.057,0.037,-0.062))
#
#            if self.geom=="polygon":
#                # create points for convex hull
#                pt_vect = chrono.vector_ChVectorD()
#                # creates bottom
#                for i in range(self.nsides):
#                    pt_vect.push_back(chrono.ChVectorD((self.diameter/2)*np.cos(i*2*np.pi/self.nsides),self.height/2,(self.diameter/2)*np.sin(i*2*np.pi/self.nsides)))
#                    #create top 
#                for i in range(self.nsides):
#                    pt_vect.push_back(chrono.ChVectorD((self.diameter/2)*np.cos(i*2*np.pi/self.nsides),-self.height/2,(self.diameter/2)*np.sin(i*2*np.pi/self.nsides)))
#            
#                ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rowr,True,True)   
#                ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
#            
#                ball.GetCollisionModel().ClearModel()
#                ball.GetCollisionModel().AddConvexHull(pt_vect)
#                ball.GetCollisionModel().BuildModel()
# 



            # force x
#            myforcex = chrono.ChForce()
#            ball.AddForce(myforcex)
#            myforcex.SetMode(chrono.ChForce.FORCE)
#            myforcex.SetDir(chrono.VECT_X)
#            self.bforce.append(myforcex)
#            # force y    
#            myforcey = chrono.ChForce()
#            ball.AddForce(myforcey)
#            myforcey.SetMode(chrono.ChForce.FORCE)
#            myforcey.SetDir(chrono.VECT_Y)
#            self.bforce.append(myforcey)
#            # force z            
#            myforcez = chrono.ChForce()
#            ball.AddForce(myforcez)
#            myforcez.SetMode(chrono.ChForce.FORCE)
#            myforcez.SetDir(chrono.VECT_Z)
#            self.bforce.append(myforcez)
#            self.obj.append(ball)


        
    def save_data_position(self):
        self.bx.append(self.balls[0].GetPos().x)
        self.bz.append(self.balls[0].GetPos().z)   
        
    def save_data_velocity(self):
        self.bvx.append(self.balls[0].GetPos_dt().x)
        self.bvz.append(self.balls[0].GetPos_dt().z) 
        
    def save_contact_force(self):
        self.bFx.append(self.balls[0].GetContactForce().x)
        self.bFy.append(self.balls[0].GetContactForce().y)
        self.bFz.append(self.balls[0].GetContactForce().z)
        
    def return_position_data(self):
        return(self.bx,self.bz)
    
    def return_force_data(self):
        return(self.bFx,self.bFz)
    
    def return_velocity_data(self):
        return(self.bvx,self.bvz)    
        
        
    # return system    
    def return_system(self):
        return(self.my_system)
# In[controller]
class Controls:
    # **We should really put variable=None for all the optional arguments so we don't need to init everything every single time**
    def __init__(self,forces,bots,interiors,fbound,Springs,my_system,k,rl,rlmax,type_spring,nb,control_type,balls,tstep,diameter,height,my_rep,*args,**kwargs):
        # objects
        # %% Initialize some parameters for all cases of the controller
        self.forces=forces
        self.bots=bots              # array of all our boundary robots
        self.interiors=interiors    # array of all the interior particles
        self.bound_force=fbound     # array of all the bounding forces for skin collision
        self.Springs=Springs        # array of all the spring objects
        self.my_system=my_system    # the simulation system
        self.balls=balls
        # Springs
        self.k=k
        self.rl=rl
        self.rlmax=rlmax
        self.type_spring=type_spring
        self.tstep=tstep
        self.diameter=diameter
        self.height=height
        # Control variables
        self.control_type=control_type # controller type                  
        self.xb=[]                      
        self.zb=[]
        self.xbv=[]
        self.zbv=[]
        self.nb=nb
        self.E=[]
        # save controller forces
        self.Faxc={}
        self.Fayc={}
        self.Fazc={}
        self.time=0
        self.Faxpp={}
        self.Faypp={}
        self.Fazpp={}
        self.Faxpf={}
        self.Faypf={}
        self.Fazpf={}
        # temporary arrays
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        self.fxtpp=[]
        self.fytpp=[]
        self.fztpp=[]        
        self.fxtpr=[]
        self.fytpr=[]
        self.fztpr=[] 
        self.my_rep=my_rep
        self.jam_mode=False
        self.cond=False
        self.cn=0
        self.count=0
        self.T=0
        self.num=5
        self.pwm=150
        self.n=(self.pwm/255)*.2
        for i in range(self.nb):
            self.Faxc["botx{0}".format(i)]=[]
            self.Fayc["boty{0}".format(i)]=[]
            self.Fazc["botz{0}".format(i)]=[] 
##############################################################################   
        # %% shape form   
        if self.control_type=="shape_form":
            self.shapes=args[0][0]
            (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8] 
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None
############################################################################# 
        # %% shape form2           
        if self.control_type=="shape_form2":
            ''' forming a shape with the region controller '''
            self.shapes=args[0][0]
            self.shapes.create_gradient()
            self.fny=self.shapes.fny
            self.fnx=self.shapes.fnx
            self.f=self.shapes.f
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8] 
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None            
##################################################################################### 
        # %% shape formation           
        if self.control_type=="shape_formation":
            self.shapes=args[0][0]
            self.shapes.create_gradient()
            self.fnz=self.shapes.fnz
            self.fnx=self.shapes.fnx
            #(self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]
            self.vref=args[0][9]
            self.paths=args[0][10]
            self.FX=0
            self.FZ=0
            self.FX2=None
            self.FZ2=None
            self.FX3=None
            self.FZ3=None            
##############################################################################   
        # %% pot_field_grab         
        if self.control_type=='pot_field_grab':
            self.shapes=args[0][0]
            (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]
            self.tpull=args[0][14]
            # array of all the balls being grabbed
            self.ballx=0
            self.ballz=0
            self.bforce=0
            self.Fballx=0
            self.Fballz=0        
            self.ball_pull=False              
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None
            self.count=0
################################################################################################
        # %% pot_field_grab2            
        if self.control_type=='pot_field_grab_2':
            self.shapes=args[0][0]
            (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]
            self.tpull=args[0][14]
            # array of all the balls being grabbed
            self.ballx=0
            self.ballz=0
            self.bforce=0
            self.Fballx=0
            self.Fballz=0        
            self.ball_pull=False              
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None
            self.count=0
#####################################################################################################
        # %% pot_field_grab_A
        if self.control_type=='pot_field_grab_A':
            self.shapes=args[0][0]
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]
            self.tpull=args[0][14]
            # array of all the balls being grabbed
            self.ballx=0
            self.ballz=0
            self.bforce=0
            self.Fballx=0
            self.Fballz=0        
            self.ball_pull=False              
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None    
##############################################################################
        # %% Verify 
        if self.control_type=='verify':
            self.shapes=args[0][0]
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]
            self.Tx=args[0][14]
            self.Tz=args[0][15]
            # array of all the balls being grabbed
            self.ballx=0
            self.ballz=0
            self.bforce=0
            self.Fballx=0
            self.Fballz=0        
            self.ball_pull=False              
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None             
 #################################################################################################################     
        # %% tunneling       
        if self.control_type=="tunneling":
            self.shapes=args[0][0]     
            self.fnz=self.shapes.fnz
            self.fnx=self.shapes.fnx
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.FX=0
            self.FZ=0
            self.FX2=None
            self.FZ2=None
            self.FX3=None
            self.FZ3=None
        # %% path_following
        if self.control_type=='path_following' :
            self.paths=args[0][0]           # len(bots), type(path): array of all path objects
            self.pathind=args[0][1]       # len(bots): indices of bots doing path control
            self.vref=args[0][2]              # reference velocity for path control
            self.poseind=args[0][3]       # indices of bots doing pose control
            self.pose_update = 20   
            self.norm_dir=0
            self.d2v1=0
            self.tan_dir=0
            self.s0=0
            self.s0=0
            self.ref_pts=0
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None
            self.patherr=np.zeros(20)
            self.errweight=1/(np.arange(20)+1)**2
            
            self.mag_t = args[0][4]
            self.kd =args[0][5] # Derivative gain
            self.mag_n=args[0][6]
            self.l=0
 ################################################################################################    
        # %% grab_drag_A        
        if self.control_type=='grab_drag_A':
            self.shapes=args[0][0]
            #(self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]  
            self.paths=args[0][14]           # len(bots), type(path): array of all path objects
            self.pathind=args[0][15]       # len(bots): indices of bots doing path control
            self.vref=args[0][16]              # reference velocity for path control
            self.poseind=args[0][17]       # indices of bots doing pose control
            self.tpull=args[0][18]
            self.mag_t=args[0][19]
            self.kd=args[0][20]
            self.mag_n=args[0][21]
            self.pose_update = 1
            self.norm_dir=0
            self.tan_dir=0
            self.s0=0
            self.l=0
            self.ref_pts=0
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None
            self.Fballx=0
            self.Fballz=0        
            self.ball_pull=False  
            # array of all the balls being grabbed
            self.ballx=0
            self.ballz=0
            self.ballxd=0
            self.ballzd=0
            self.tball=0
            self.bforce=0
            self.tstep=.001
            self.patherr=np.zeros(20)
            self.errweight=1/(np.arange(20)+1)**2
            self.Faxpp={}
            self.Faypp={}
            self.Fazpp={}
            self.Faxpf={}
            self.Faypf={}
            self.Fazpf={}
            # temporary arrays
            self.fxtpp=[]
            self.fytpp=[]
            self.fztpp=[]
            self.fxtpf=[]
            self.fytpf=[]
            self.fztpf=[]
            for i in range(self.nb):
                self.Faxpp["botx{0}".format(i)]=[]
                self.Faypp["boty{0}".format(i)]=[]
                self.Fazpp["botz{0}".format(i)]=[] 
            for i in range(self.nb):
                self.Faxpf["botx{0}".format(i)]=[]
                self.Faypf["boty{0}".format(i)]=[]
                self.Fazpf["botz{0}".format(i)]=[]      
##############################################################################################    
        # %% grab_drag            
        if self.control_type=='grab_drag':
            self.shapes=args[0][0]
            (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]  
            self.paths=args[0][14]           # len(bots), type(path): array of all path objects
            self.pathind=args[0][15]       # len(bots): indices of bots doing path control
            self.vref=args[0][16]              # reference velocity for path control
            self.poseind=args[0][17]       # indices of bots doing pose control
            self.tpull=args[0][18]
            self.mag_t=args[0][19]
            self.kd=args[0][20]
            self.mag_n=args[0][21]
            self.pose_update = 5
            self.norm_dir=0
            self.tan_dir=0
            self.s0=0
            self.l=0
            self.ref_pts=0
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None
            self.Fballx=0
            self.Fballz=0        
            self.ball_pull=False
            # array of all the balls being grabbed
            self.ballx=0
            self.ballz=0
            self.ballxd=0
            self.ballzd=0
            self.tball=0
            self.bforce=0
            self.tstep=.002
            self.patherr=np.zeros(20)
            self.errweight=1/(np.arange(20)+1)**2 
            self.Faxpp={}
            self.Faypp={}
            self.Fazpp={}
            self.Faxpf={}
            self.Faypf={}
            self.Fazpf={}
            
            self.fxtpp=[]
            self.fytpp=[]
            self.fztpp=[]
            self.fxtpf=[]
            self.fytpf=[]
            self.fztpf=[]
            self.theta=self.coord_trans()
####################################################################################################    
        # %% grab_drag_2        
        if self.control_type=='grab_drag_2':
            self.shapes=args[0][0]
            (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.p1=args[0][3]
            self.p2=args[0][4]
            self.bl=args[0][5]
            self.br=args[0][6]
            self.Rd=args[0][7]
            self.nr=args[0][8]  
            self.paths=args[0][14]           # len(bots), type(path): array of all path objects
            self.pathind=args[0][15]       # len(bots): indices of bots doing path control
            self.vref=args[0][16]              # reference velocity for path control
            self.poseind=args[0][17]       # indices of bots doing pose control
            self.tpull=args[0][18]
            self.mag_t=args[0][19]
            self.kd=args[0][20]
            self.mag_n=args[0][21]
            self.pose_update = 5
            self.norm_dir=0
            self.tan_dir=0
            self.s0=0
            self.l=0
            self.ref_pts=0
            self.FX=0
            self.FZ=0
            self.FX2=0
            self.FZ2=0
            self.FX3=None
            self.FZ3=None
            self.Fballx=0
            self.Fballz=0        
            self.ball_pull=False
            # array of all the balls being grabbed
            self.ballx=0
            self.ballz=0
            self.ballxd=0
            self.ballzd=0
            self.tball=0
            self.bforce=0
            self.tstep=.002
            self.patherr=np.zeros(20)
            self.errweight=1/(np.arange(20)+1)**2 
            self.Faxpp={}
            self.Faypp={}
            self.Fazpp={}
            self.Faxpf={}
            self.Faypf={}
            self.Fazpf={}
            
            self.fxtpp=[]
            self.fytpp=[]
            self.fztpp=[]
            self.fxtpf=[]
            self.fytpf=[]
            self.fztpf=[]            
            for i in range(self.nb):
                self.Faxpp["botx{0}".format(i)]=[]
                self.Faypp["boty{0}".format(i)]=[]
                self.Fazpp["botz{0}".format(i)]=[]            
            for i in range(self.nb):
                self.Faxpf["botx{0}".format(i)]=[]
                self.Faypf["boty{0}".format(i)]=[]
                self.Fazpf["botz{0}".format(i)]=[]    
###################################################################################
        # %% local_field
        if self.control_type=="local_field":
            self.shapes=args[0][0]     
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.FX=0
            self.FZ=0
            self.FX2=None
            self.FZ2=None
            self.FX3=None
            self.FZ3=None
            self.xt=1000*np.ones(self.nb)
            self.zt=1000*np.ones(self.nb)

            self.bbb=[]

    # %% run controller
    def run_controller(self):
        # Pot_field_grab: grab a fixed object
        if self.control_type=="pot_field_grab":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            #self.jam_springs()
            #self.Ball_controller()                
            #(self.FX,self.FZ)=self.grab_controller()
            (self.FX,self.FZ)=self.grab_controller()

            self.count=self.count+1
            #T=self.count*.001
            if (np.round(self.my_system.GetChTime(),2))%.001==0:
                print("bongo",np.round(self.my_system.GetChTime(),2),self.count)
                self.apply_force(self.FX,self.FZ,None,None,None,None)
            else:
                self.apply_force(None,None,None,None,None,None)   
        
        if self.control_type=="move_right":
            (self.Springs)=self.setspring()
            #(self.xb,self.zb)=self.get_position()
            #(self.xbv,self.zbv)=self.get_velocity()
            #self.jam_springs()
            #self.Ball_controller()                
            #(self.FX,self.FZ)=self.grab_controller()
            #(self.FX,self.FZ)=self.grab_controller()
            (self.FX,self.FZ)=self.move_right()
            self.count=self.count+1
            self.T=.001+self.T
            
            if self.cond==True:
                tn=.2
            else:
                tn=.037
                
            if self.T>0 and self.T<=tn:
                #print("bongo",np.round(self.my_system.GetChTime(),2),self.count)
                self.apply_force(self.FX,self.FZ,None,None,None,None)
            else:
                self.apply_force(None,None,None,None,None,None) 

            if self.T>.2:
                self.T=0
                print('reset',np.round(self.my_system.GetChTime(),2))
                
        if self.control_type=="pot_field_grab_2":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            #self.jam_springs()
            #self.Ball_controller()                
            #(self.FX,self.FZ)=self.grab_controller()
            (self.FX,self.FZ)=self.grab_controller_2()
            self.count=self.count+1
            self.T=.001+self.T
            if self.T>0 and self.T<=.03:
                
                if self.cond==False:
                    #self.theta=self.coord_trans()
                    #self.FX=self.FX*np.sin(self.theta)
                    self.FZ=self.FZ
                    self.FX=None
                    self.apply_force(self.FX,self.FZ,None,None,None,None)
                else:
                    self.FX=self.FX
                    self.FZ=self.FZ
                    self.apply_force(self.FX,self.FZ,None,None,None,None)
            else:
                self.apply_force(None,None,None,None,None,None)
                
            if self.T>.2:
                self.T=0
                print('reset',np.round(self.my_system.GetChTime(),2))
                #print("bongo",np.round(self.my_system.GetChTime(),2),self.count)

  
        # pot_field_grab_A: grab a fixed ball with a analytical rbf 
        if self.control_type=="pot_field_grab_A":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            self.jam_springs()
            #self.Ball_controller()                
            (self.FX,self.FZ)=self.grab_controller_A()
            #(FXr,FZr)=self.repulsion_controller()
            #print(Fxr,Fzr)
            self.apply_force(self.FX,self.FZ,None,None,None,None,None) 


        # pot_field_grab_A: grab a fixed ball with a analytical rbf 
        if self.control_type=="verify":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            self.jam_springs()
            #self.Ball_controller()
            if self.my_system.GetChTime()<.1:
                self.FX=0
                self.FZ=0
                self.apply_force(None,None,None,None,None,None,None) 
            else:
                (self.FX,self.FZ)=self.grab_controller_verify()
                self.apply_force(self.FX,self.FZ,None,None,None,None,None)
                
        if self.control_type=="shape_formation":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            #self.jam_springs()
            (self.FX,self.FZ)=self.formation_controller()
            self.count=self.count+1
            if (self.count)%self.T==0:
                #print("bongo",np.round(self.my_system.GetChTime(),2),self.count)
                self.apply_force(self.FX,self.FZ,None,None,None,None)
            else:
                self.apply_force(None,None,None,None,None,None)  

             
        # shape_form: form a desired shape    
        if self.control_type=="shape_form":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            self.jam_springs()              
            (self.FX,self.FZ)=self.shape_controller()
            #(FXr,FZr)=self.repulsion_controller()
            self.apply_force(self.FX,self.FZ,None,None,None,None) 
            
        # shape_form2: form a desired shape 2   
        if self.control_type=="shape_form2":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            self.jam_springs()              
            (self.FX,self.FZ)=self.shape_controller2()
            #(FXr,FZr)=self.repulsion_controller()
            self.apply_force(self.FX,self.FZ,None,None,None,None)      
            
            
        # Path_following: have the robot follow a desired trajectory
        # tunneling: have the robot tunnel through a corridor 
        if self.control_type=="tunneling":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            #self.jam_springs()
            (self.FX,self.FZ)=self.tunnel_controller()
            self.count=self.count+1
            self.T=.001+self.T
            
            if self.T>0 and self.T<=self.n:
                #print("bongo",np.round(self.my_system.GetChTime(),2),self.count)
                self.apply_force(self.FX,self.FZ,None,None,None,None)
            else:
                self.apply_force(None,None,None,None,None,None) 

            if self.T>.2:
                self.T=0
                print('reset',np.round(self.my_system.GetChTime(),2))  
                
                
        if self.control_type=="path_following":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            
            (self.FX2,self.FZ2)=self.leader_controller()

            if round(self.my_system.GetChTime(),2)%(self.tstep*self.pose_update) < 1e-3:
                #(self.FX2,self.FZ2)=self.leader_controller()
                (self.FX3,self.FZ3)=self.follower_controller()   
            self.apply_force(None,None,self.FX2,self.FZ2,self.FX3,self.FZ3) 
            
        # grab_drag_A: Grab and drag a object with a analytical rbf 
        if self.control_type=="grab_drag_A":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            self.jam_springs()
            #self.Ball_controller()
            (self.FX,self.FZ)=self.grab_controller_A()
            if self.tpull < self.my_system.GetChTime():  
                (self.FX2,self.FZ2)=self.leader_controller()
                if round(self.my_system.GetChTime(),2)%(self.tstep*self.pose_update) < 1e-3:
                    (self.FX3,self.FZ3)=self.follower_controller()
                self.apply_force(self.FX,self.FZ,self.FX2,self.FZ2,self.FX3,self.FZ3)
            else:
                self.apply_force(self.FX,self.FZ,None,None,None,None)
                
        # grab and drag a non fixed object 
        if self.control_type=="grab_drag":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            #self.jam_springs()
            #(self.FX,self.FZ)=self.grab_controller_move()
            
            (self.FX,self.FZ)=self.grab_controller_move_2()
            print(self.cond)
            #if .5>self.my_system.GetChTime():
            if self.cond==False:
                #self.theta=self.coord_trans()
                #self.FX=self.FX*np.sin(self.theta)
                self.FZ=self.FZ
                self.FX=None
            else:
                self.FX=self.FX
                self.FZ=self.FZ
            #(FXr,FZr)=self.repulsion_controller()
            #print(FXr,FZr)
            self.apply_force(self.FX,self.FZ,None,None,None,None) 

              
        if self.control_type=="grab_drag_2":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            #self.jam_springs()
            #(self.FX,self.FZ)=self.grab_controller_move()
            (self.FX,self.FZ)=self.grab_controller_move_2()
            if .5>self.my_system.GetChTime():
                self.FX=None
            else:
                self.FX=self.FX
            (FXr,FZr)=self.repulsion_controller()
            #print(FXr,FZr)
            self.apply_force(self.FX,self.FZ,None,None,None,None) 
                         
                
        if self.control_type=="local_field":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.local_sensing_controller()
            #(self.FZ,self.FX)=self.sink_pot_fields()
            self.apply_force(self.FX,self.FZ,None,None,None,None)      
# Create transformation matrix
    def coord_trans(self):
        (xc,zc)=self.centroid()
        xb=self.balls.balls[0].GetPos().x
        zb=self.balls.balls[0].GetPos().z  
        
        X=(xb-xc)
        Z=(zb-zc)
        theta=np.arctan(X/Z)
        return(theta)
                
# save the variables        
    def save_data_Forces(self):
       #if len(self.fxt)<self.nb:
        for i in range(self.nb):
             self.fxt.append(0)
             self.fyt.append(0)
             self.fzt.append(0)                
        for i in range(self.nb):
           self.Faxc["botx"+str(i)].append(self.fxt[i])
           self.Fayc["boty"+str(i)].append(self.fyt[i])
           self.Fazc["botz"+str(i)].append(self.fzt[i])

    def save_pppr_Forces(self):
       #if len(self.fxt)<self.nb:
        for i in range(self.nb):
             self.fxtpp.append(0)
             self.fytpp.append(0)
             self.fztpp.append(0)                
        for i in range(self.nb):
           self.Faxpp["botx"+str(i)].append(self.fxtpp[i])
           self.Faypp["boty"+str(i)].append(self.fytpp[i])
           self.Fazpp["botz"+str(i)].append(self.fztpp[i])
        for i in range(self.nb):
             self.fxtpf.append(0)
             self.fytpf.append(0)
             self.fztpf.append(0)                
        for i in range(self.nb):
           self.Faxpf["botx"+str(i)].append(self.fxtpf[i])
           self.Faypf["boty"+str(i)].append(self.fytpf[i])
           self.Fazpf["botz"+str(i)].append(self.fztpf[i])           
                         
    def clear_temp_forces(self):
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        self.fxtpp=[]
        self.fytpp=[]
        self.fztpp=[]        
        self.fxtpr=[]
        self.fytpr=[]
        self.fztpr=[]                
# return force data for controllers     
    def return_force_data(self):
        return(self.Faxc,self.Fayc,self.Fazc) 
            
###########################################################################################################################                
    def grab_controller(self):
        #if .4<self.my_system.GetChTime(): 
            #self.alpha=50
        #self.shapes.shape="circle"
        self.shapes.Rd=self.balls.diameter/2
        self.shapes.p2=self.balls.balls[0].GetPos().x
        self.shapes.p1=self.balls.balls[0].GetPos().z
        (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
                    
        #if self.tpull<self.my_system.GetChTime():
            #self.alpha=50
            #self.ball_pull=True
            #self.balls.balls[0].SetBodyFixed(False)    
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.fny([self.xb[i],self.zb[i]])
            Fz=self.fnx([self.xb[i],self.zb[i]])
            mag=np.sqrt(Fx**2+Fz**2)
            FXX=Fx/mag
            FZZ=Fz/mag
            
            fx=-self.alpha*FXX-self.beta*self.xbv[i]
            fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
            
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))
######################################################################################################
    def grab_controller_2(self):
#        if .6<self.my_system.GetChTime(): 
#            self.alpha=70
        self.shapes.shape="circle"
        self.shapes.Rd=self.balls.diameter
        self.d2v1=np.array([self.balls.balls[0].GetPos_dt().x, self.balls.balls[0].GetPos_dt().z])
        #self.shapes.p2=self.balls.balls[0].GetPos().x
        #self.shapes.p1=self.balls.balls[0].GetPos().z
        (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
        if self.cond==True:
            self.alpha=20
        
        FX=[]
        FZ=[]
        dist=self.balls.diameter*.75

        for i in range(self.nb):
            Mz=(self.zb[i]-self.balls.balls[0].GetPos().z)
            Mx=(self.xb[i]-self.balls.balls[0].GetPos().x)
            #M=np.sqrt(Mz**2+Mx**2)
            if self.cond==True:
                pass
            else:
                if Mz>dist:
                    self.cn+=1
                if self.cn==self.num:
                    self.cond=True

            if abs(Mx)<abs(dist) and abs(Mz)<abs(dist) and self.cond==False:
            
                fx=[0]
                fz=[0]
            else: 
                fyy=self.fny([self.xb[i],self.zb[i]])
                fxx=self.fnx([self.xb[i],self.zb[i]])
                M=np.sqrt(fyy**2+fxx**2)
                VX=fyy/M
                VZ=fxx/M
                
                fx=(-self.alpha*VX-self.beta*self.xbv[i])
                fz=-self.alpha*VZ-self.beta*self.zbv[i]                    
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))     
######################################################################################################    
    def grab_controller_A(self):
 
        self.shapes.py=self.balls.balls[0].GetPos().z
        self.shapes.px=self.balls.balls[0].GetPos().x
        FXt=[]
        FZt=[]
        for i in range(self.nb):

            (FX,FZ)=self.shapes.out_force_2(self.xb[i],self.zb[i])
            fx=-self.alpha*FX-self.beta*self.xbv[i]
            fz=-self.alpha*FZ-self.beta*self.zbv[i]
                #print(-self.alpha)
                #print(np.sqrt(FX**2+FZ**2))
                #print(fx,fz)                 
            FXt.append(fx)
            FZt.append(fz)
        return(np.asarray(FXt),np.asarray(FZt))
###################################################################################################################           
    def grab_controller_verify(self):
        time=self.my_system.GetChTime()
        A=self.Tz(time).flatten()
        B=self.Tx(time).flatten()
        #print(A,B)
        #print(type(A),type(B))
        self.shapes.py=A[0]
        self.shapes.px=B[0]
        #self.shapes=points_shape_A(A[0],B[0],self.Rd)
        self.balls.balls[0].SetPos(chrono.ChVectorD(B[0],self.height/2,A[0]))
        FXt=[]
        FZt=[]
        (self.fny,self.fnx)=self.shapes.create_gradient()
        for i in range(self.nb):

            #(FX,FZ)=self.shapes.out_force_2(self.xb[i],self.zb[i])
            Fx=self.fny((self.xb[i],self.zb[i]))
            Fz=self.fnx((self.xb[i],self.zb[i]))
            mag=np.sqrt(Fx**2+Fz**2)
            FX=Fx/mag
            FZ=Fz/mag
            fx=-self.alpha*FX-self.beta*self.xbv[i]
            fz=-self.alpha*FZ-self.beta*self.zbv[i]        
            FXt.append(fx)
            FZt.append(fz)
        return(np.asarray(FXt),np.asarray(FZt))
###########################################################################################################        
    def grab_controller_move(self):

        if self.tpull/2<self.my_system.GetChTime():
            self.shapes.R=2
        if self.tpull<self.my_system.GetChTime():
            self.alpha=80
            self.ball_pull=True
            self.balls.balls[0].SetBodyFixed(False)
            self.d2v1=np.array([self.balls.balls[0].GetPos_dt().x, self.balls.balls[0].GetPos_dt().z])
            self.shapes.p1=0
            self.shapes.p2=self.balls.balls[0].GetPos().x+.05*(abs(self.vref-self.d2v1[0]))
            (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
        FX=[]
        FZ=[]
        for i in range(self.nb):
            fx=-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i]
            fz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))   
###############################################################################################################       
    def grab_controller_move_2(self):
        self.shapes.shape="circle"
        self.shapes.Rd=self.balls.diameter
        self.d2v1=np.array([self.balls.balls[0].GetPos_dt().x, self.balls.balls[0].GetPos_dt().z])
        self.shapes.p2=self.balls.balls[0].GetPos().x
        if self.tpull<self.my_system.GetChTime():
            self.shapes.p1=self.balls.balls[0].GetPos().z-0.09*(self.vref)
            self.shapes.p2=self.balls.balls[0].GetPos().x*np.sin(self.theta)
        else:
            self.shapes.p1=(self.balls.balls[0].GetPos().z-.25)*np.cos(self.theta)
            
        (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
        FX=[]
        FZ=[]
        dist=self.balls.diameter

        for i in range(self.nb):
            if self.cond==True:
                pass
            else:
                if (np.cos(self.theta)*(self.zb[i]-self.balls.balls[0].GetPos().z))>self.balls.diameter:
                    self.cn+=1
                if self.cn==5:
                    self.cond=True

                
                
            if (np.cos(self.theta)*abs(self.zb[i]-self.balls.balls[0].GetPos().z))<abs(2*dist) and np.sin(self.theta)*abs(self.xb[i]-self.balls.balls[0].GetPos().x)<abs(2*dist) and self.cond==False:
                tx=(-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i])
                tz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]
                if tx[0]>=0:
                    fx=tx-4*tx
                else:
                    fx=tx+4*tx
                if tz[0]>=0:
                    fz=tz-4*tz
                else:
                    fz=tz+4*tz
            else:        
                fx=(-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i])
                fz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]                    
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))        
####################################################################################################################        
    def repulsion_controller(self):
        Q=2*self.diameter+.08
        K=.1
        FX=[]
        FZ=[]
        for i in range(self.nb):
            fx=[]
            fz=[]
            for j in range(self.nb):
                dx=(self.xb[i]-self.xb[j])
                dz=(self.zb[i]-self.zb[j])
                d=np.sqrt(dx**2+dz**2)
                d32=d**(-1.5)
                dq=(1/d)-(1/Q)
                #print("d32= ",1/d32)
                Uz=np.nan_to_num(-K*(dz)*d32*dq)
                Ux=np.nan_to_num(-K*(dx)*d32*dq)
                #print(Uz,Ux)
                #print(d)
                if d<Q:
                    fx.append(Ux)
                    fz.append(Uz)
                else:
                    fx.append(0)
                    fz.append(0)
                         

            FX.append(np.sum(fx))
            FZ.append(np.sum(fz))
        return(np.asarray(FX),np.asarray(FZ))      
###################################################################################################################       
    def move_right(self):
        FX=[]
        FZ=[]

        for i in range(self.nb):
            FX.append(0)
            FZ.append(2)                
        return(np.asarray(FX),np.asarray(FZ))        
##################################################################################################################        
    def shape_controller(self):

        FX=[]
        FZ=[]
        for i in range(self.nb):
            fx=-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i]
            fz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))
###########################################################################################    
    def shape_controller2(self):
        FX=[]
        FZ=[]
        for i in range(self.nb):
            VZ = self.fnx(([self.xb[i],self.zb[i]]))*np.maximum(0,self.f(([self.xb[i],self.zb[i]])))
            VX = self.fny(([self.xb[i],self.zb[i]]))*np.maximum(0,self.f(([self.xb[i],self.zb[i]])))
            A=self.f(([self.xb[i],self.zb[i]]))
            if A<.001:
                #print(str(i),'gucci',A)
                VZ=0
                VX=0
                fx=0
                fz=0
                FX.append(fx)
                FZ.append(fz)
                
            else:
                mag=np.sqrt(VZ**2+VX**2)
                VZ=VZ/mag
                VX=VX/mag

                fx=-self.alpha*VX#-self.beta*self.xbv[i]
                fz=-self.alpha*VZ#-self.beta*self.zbv[i]
                FX.append(fx[0])
                FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))    
 ##############################################################################################       
    def formation_controller(self):
        FX=[]
        FZ=[]
        (xc,zc)=self.centroid()
        #self.shapes.p1=0 # z position
        self.shapes.p1=(self.shapes.p1+(.001)*.5)
        print(self.shapes.p1)
        self.shapes.p2=0
        self.shapes.create_gradient()
        for i in range(self.nb):
            FYY=self.fnz([self.xb[i],self.zb[i]])
            FXX=self.fnx([self.xb[i],self.zb[i]])
            #print(FY,FX)
            mag=np.sqrt(FXX[0]**2+FYY[0]**2)
            if mag==0:
                mag=1
            else:
                mag=mag
            Fy=FYY/mag
            Fx=FXX/mag               
            fx=-self.alpha*Fy-self.beta*self.xbv[i]
            fz=-self.alpha*Fx-self.beta*self.zbv[i]
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))        
################################################################################################################                
    def centroid(self):
        X=[]
        Z=[]
        for i in range(self.nb):
            X.append(self.bots[i].GetPos().x)
            Z.append(self.bots[i].GetPos().z)
            
        xc=sum(X)/len(X)
        zc=sum(Z)/len(Z)
        return(xc,zc)      
#####################################################################################################        
    def leader_controller(self):
        i=self.pathind
        # Heading angle based on velocity
        self.d2v1 = np.array([self.bots[i].GetPos_dt().x, self.bots[i].GetPos_dt().z])
        V=np.linalg.norm(self.d2v1)
        pos = np.array([self.bots[i].GetPos().x, self.bots[i].GetPos().z]) 
        if self.paths.p=='flat_line':
            self.l=self.l+self.d2v1[1]*self.tstep
            #print(self.l)
            #thetaref=np.pi/2
            l, k, thetaref, self.s0,point = self.paths.calc_track_error(pos[0], pos[1], self.s0)
        else:       
            self.l, k, thetaref, self.s0,point = self.paths.calc_track_error(pos[0], pos[1], self.s0)
        

        # Normal and tangent direction unit vectors
        self.tan_dir = np.array([np.cos(thetaref),np.sin(thetaref)])
        norm_dirt = -np.array([pos[0]-point[0],pos[1]-point[1]])
        self.norm_dir = norm_dirt/np.linalg.norm(norm_dirt)
        
        # Update error vector and calculate integral term
        self.patherr[1:20]=self.patherr[0:19]
        self.patherr[0]=self.l
        
        #int_mag=10*np.ma.average(self.patherr, weights=self.errweight)
        # Scale magnitude of force in path normal dirction by distance from path
        mag_n=(self.vref-V)
        mag_norm = self.mag_n*self.l#+int_mag
        #print(abs(self.l**2))
        self.mag_tan=self.mag_t*(self.vref-V)
        #self.mag_tan=self.mag_t
        # Convert mag_tan and mag_norm to Fx and Fz: basically proportional control based on distance and forward speed
        fx = self.mag_tan*self.tan_dir[0] - mag_norm*self.norm_dir[0] 
        fz = self.mag_tan*self.tan_dir[1] - mag_norm*self.norm_dir[1]

        #print('l',self.l)

        fx=np.array([fx])
        fz=np.array([fz])
        FX=[]
        FZ=[]
        if type(fz[0]) is np.float64:
            fza=fz[0]
            fxa=fx[0]
        else:
            fza=fz[0][0]
            fxa=fx[0][0]

        FX.append(fxa)
        FZ.append(fza)
        
        return(np.asarray(FX),np.asarray(FZ))            
##################################################################################################   
    def follower_controller(self):
        FX=[]
        FZ=[]
        
        for i in range(self.nb-1):
            #print(i)
            if self.poseind[i]==1:
                #print(i)
                # Current position and speed
                #pos = np.array([self.bots[i].GetPos().x, self.bots[i].GetPos().z]) 
                #mp= pos/np.linalg.norm(pos)
                vel = np.array([self.bots[i].GetPos_dt().x, self.bots[i].GetPos_dt().z])
                # Current position and speed error
                pos_err = self.tan_dir
                vel_err = vel
                # PD control force based on position and speed
                
                fx=self.mag_tan*pos_err[0]-self.kd*vel_err[0]
                fz=self.mag_tan*pos_err[1]-self.kd*vel_err[1]
                
            else:
                fx=0
                fz=0
                
            self.fxt.append(fx)
            self.fyt.append(0)
            self.fzt.append(fz) 
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))            
###############################################################################################################
    
    def tunnel_controller(self):
        FX=[]
        FZ=[]
        for i in range(self.nb):
            VX=self.fnx([self.xb[i],self.zb[i]])
            VY=self.fnz([self.xb[i],self.zb[i]])
            M=np.sqrt(VX**2+VY**2)
            
            fx=self.alpha*VX/M#-self.beta*self.xbv[i]
            fz=self.alpha*VY/M#-self.beta*self.zbv[i]
            #fz=[0]
            #print(self.fnx([self.xb[i],self.zb[i]]),self.fnz([self.xb[i],self.zb[i]]))
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))        

    
    def sink_pot_fields(self):
        self.shapes.update_F_fx_fz()
        self.fnz=self.shapes.fnz
        self.fnx=self.shapes.fnx
        FX=[]
        FZ=[]    
        for i in range(self.nb):
            if self.active[i]==1:
                fz=-self.alpha*self.fnz([self.zb[i],self.xb[i]])-self.beta*self.zbv[i]
                fx=-self.alpha*self.fnx([self.zb[i],self.xb[i]])-self.beta*self.xbv[i]
            else:
                fx=[0]
                fz=[0]
            FX.append(fx[0])
            FZ.append(fz[0])        
        return(np.asarray(FX),np.asarray(FZ)) 
############################################################################################################             
    

        
    def local_sensing_controller(self):
        self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
        crt_list = self.my_rep.GetList()
        A=crt_list[6] # bodya
        B=crt_list[7] # bodyb 
        C=crt_list[0] # x
        D=crt_list[2] # z
        E=crt_list[8]
        #self.shapes.px=[]
        #self.shapes.pz=[]    
        for i in range(len(D)):
            if (A[i]=='ball' and B[i][0:3]=='bot') or (B[i]=='ball' and A[i][0:3]=='bot'):
                if B[i][0:3]=='bot':
                    leng=len(B[i])
                    num=(B[i][3:leng])
                    num=int(num)
                    if self.xt[num]==1000:
                        self.xt[num]=(C[i])
                        self.zt[num]=(D[i])
                    else:
                        pass
                        
                if A[i][0:3]=='bot':
                    leng=len(A[i])
                    num=(A[i][3:leng])
                    num=int(num)

                    if self.xt[num]==1000:
                        self.xt[num]=(C[i])
                        self.zt[num]=(D[i])
                    else:
                        pass
                    
        FX=[]
        FZ=[]               
        for i in range(self.nb):
            val=0
            if self.xt[i]!=1000:
                val=val+1
                dx=self.xt[i]-self.bots[i].GetPos().x
                dz=self.zt[i]-self.bots[i].GetPos().z
                m=np.sqrt(dx**2+dz**2)
                fx=1*dx/m
                fz=1*dz/m
                FX.append(fx)
                FZ.append(fz)
            if self.xt[i]==1000:
                en10=i-1
                en11=i-2
                en12=i-3
                en20=i+1
                en21=i+2
                en22=i+3
                
                if en20==self.nb:
                    en20=0
                else:
                    en20=i+1
                    
                if en21==self.nb:
                    en21=1
                else:
                    en21=i+2
                    
                if en22==self.nb:
                    en22=2
                else:
                    en22=i+3    
                
                if self.xt[en10]!=1000 or self.xt[en20]!=1000:
                    t=self.my_system.GetChTime()
                    fx=15*signal.square(2 * np.pi *10*t)
                    fz=2
                FX.append(fx)
                FZ.append(fz)
                #if self.xt[en10]!=1000 and self.xt[en11]!=1000 and self.xt[en12]!=1000 :
                    #fx=10
                    #fz=2
                #if self.xt[en20]!=1000 and self.xt[en21]!=1000 and self.xt[en22]!=1000 :
                    #fx=-10
                    #fz=2
                # else:
                #     fx=0
                #     fz=2
                
                
                #     FX.append(fx)
                #     FZ.append(fz)
                
        return(np.asarray(FX),np.asarray(FZ))            
###############################################################################################################                                      
    def apply_force(self,FX=None,FZ=None,FX2=None,FZ2=None,FX3=None,FZ3=None,FXr=None,FZr=None):      
# pot field         
        if FX is not None:
            FX=FX
        else:
            FX=np.zeros((self.nb,1))

        if FZ is not None:
            FZ=FZ
        else:
            FZ=np.zeros((self.nb,1))  
# leader            
        if FX2 is not None:
            FX2=FX2
        else:
            FX2=np.array([0])
            
        if FZ2 is not None:
            FZ2=FZ2
        else:
            FZ2=np.array([0])
# follower            
        if FX3 is not None:
            FX3=FX3
        else:
            FX3=np.zeros((self.nb-1,1))
            
        if FZ3 is not None:
            FZ3=FZ3
        else:
            FZ3=np.zeros((self.nb-1,1))      
        
        if FXr is not None:
            FXr=FXr
        else:
            FXr=np.zeros((self.nb,1))

        if FZr is not None:
            FZr=FZr
        else:
            FZr=np.zeros((self.nb,1))
            
        if self.control_type=="path_following":
            FXX=np.hstack((FX2[0],FX3))
            FZZ=np.hstack((FZ2[0],FZ3))
            FX=FX.flatten()
            FZ=FZ.flatten()
            #FZ=FZ[0]
            for i in range(self.nb):                              
                self.fxt.append(FX[i]+FXX[i])
                self.fyt.append(0)
                self.fzt.append(FZ[i]+FZZ[i])
                self.forces[3*i].SetMforce(FX[i]+FXX[i])
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(FZ[i]+FZZ[i])
                self.forces[3*i+2].SetDir(chrono.VECT_Z)
        else:
            
            FX=FX.flatten()
            FZ=FZ.flatten()
            FXX=np.hstack((FX2.flatten(),FX3.flatten()))
            FZZ=np.hstack((FZ2.flatten(),FZ3.flatten()))
            FXX=FXX.flatten()
            FZZ=FZZ.flatten()
            FXr=FXr.flatten()
            FZr=FZr.flatten()            
#            if self.control_type=="grab_drag" or self.control_type=="grab_drag_A" :
#                for i in range(self.nb):
#                    self.fxtpf.append(FX[i])
#                    self.fytpf.append(0)
#                    self.fztpf.append(FZ[i])
#                    self.fxtpp.append(FXX[i])
#                    self.fytpp.append(0)
#                    self.fztpp.append(FZZ[i])
                #self.save_pppr_Forces()
                
            for i in range(self.nb):                             
                self.fxt.append(FX[i]+FXX[i]+FXr[i])
                self.fyt.append(0)
                self.fzt.append(FZ[i]+FZZ[i]+FZr[i])
                self.forces[3*i].SetMforce(FX[i]+FXX[i]+FXr[i])
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(FZ[i]+FZZ[i]+FZr[i])
                self.forces[3*i+2].SetDir(chrono.VECT_Z)
                
        return(FX,FZ,FX2,FZ2,FX3,FZ3)
################################################################################################################                
# Ball controller                 
    def Ball_controller(self):
        if self.ball_pull==True:
            self.bforce[0].SetMforce(self.Fballz)
            self.bforce[0].SetDir(chrono.VECT_Z)
            self.bforce[1].SetMforce(self.Fballx)
            self.bforce[1].SetDir(chrono.VECT_X)   
     
    def dist(self,x1, y1, x2, y2, x3, y3): # x3,y3 is the point
        dist=np.ones([len(x3),len(x1)])
        for i in range(1,len(x1)):
            px = x2[i]-x1[i]
            py = y2[i]-y1[i]
            norm = px*px + py*py
            u =  ((x3 - x1[i]) * px + (y3 - y1[i]) * py) / float(norm)
            u[u>1]=1
            u[u<0]=0
            x = x1[i] + u * px
            y = y1[i] + u * py
            dx = x - x3
            dy = y - y3
            dist[:,i] = np.sqrt(dx*dx + dy*dy)
        return dist
    
    def constrain_interior(self):
        diam = 0.005
        max_f = 2
        int_pos = np.ones([3,len(self.interiors)])
        pos1 = np.ones([3,self.nb])
        pos2 = np.ones([3,self.nb])
        norms = np.ones([3,self.nb])
        
        for i in range(len(self.interiors)): # Loop over all interior particles
            int_pos[:,i] = [self.interiors[i].GetPos().x,self.interiors[i].GetPos().y,self.interiors[i].GetPos().z]
        for j in range(1,self.nb): # Loop over all bots
            # Current bot
            a = np.array([self.bots[j].GetPos().x,self.bots[j].GetPos().y,self.bots[j].GetPos().z]) 
            # Previous bot
            b = np.array([self.bots[j-1].GetPos().x,self.bots[j-1].GetPos().y,self.bots[j-1].GetPos().z])   
            # should be the inward normal assuming bots were created CCW fashion
            norm = np.cross([0,-1,0],a-b)
            pos1[:,j]=a
            pos2[:,j]=b
            norms[:,j]=norm/np.linalg.norm(norm)
        
        # Distance from each interior particle center to each line segment b/t two bots
        # Disty is [number_interior, number_bots] in size
        disty = self.dist(pos1[0,:],pos1[2,:],pos2[0,:],pos2[2,:],int_pos[0,:],int_pos[2,:])
        disty[disty>diam]=0
        
        # Loop over interior particle
        for k in range(len(disty[:,1])):
            # nonzero distance indices
            nonzero=np.nonzero(disty[k,:])[0]
            if (len(nonzero)==0):
                continue
            # average contact force and normal for each interior particle
            nz_dist=disty[k,nonzero]
            mag=np.sum(1/nz_dist[0])
            if mag>max_f: mag=max_f
            weights = (1/nz_dist[0])/np.linalg.norm(1/nz_dist[0])
            avg_x = np.dot(weights,norms[1,nonzero])[0]
            avg_z = np.dot(weights,norms[2,nonzero])[0]
            
            # Apply force in normal direction proportional to distance to line
            self.bound_force[k].SetDir(chrono.ChVectorD(avg_x,0,avg_z))
            self.bound_force[k].SetMforce(mag)
            
        # Loop over bots
        for k in range(len(disty[0,:])):
            # nonzero distance indices
            nonzero=np.nonzero(disty[:,k])[0]
            if (len(nonzero)==0):
                continue
            # average contact force and normal for each bot pair
            nz_dist=disty[nonzero,k]
            mag=np.sum(1/nz_dist[0])
            if mag>max_f: mag=max_f
            mag_x = mag*norms[0,k]
            mag_z = mag*norms[2,k]
            
            # Add equal and opposite forces to two bots so net force is zero
            fx = self.forces[3*k].GetMforce()
            fz = self.forces[3*k+2].GetMforce()
            #self.forces[3*k].SetMforce(mag_x+fx)
            #self.forces[3*k].SetDir(chrono.VECT_X)
            #self.forces[3*k+2].SetMforce(mag_z+fz)
            #self.forces[3*k+2].SetDir(chrono.VECT_Z)
            
# Keep spring lengths                
    def setspring(self):
        for i in range(len(self.Springs)):
            var1=self.Springs[i].Get_SpringLength()
            # if spring length is less then goes to zero
            if var1<self.rl:
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(0)
                    
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(self.k)
            # if greater then it will double
            if var1>self.rlmax:
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(2*self.k)
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(1*self.k)
                    
        return(self.Springs)
    
# jam springs    
    def jam_springs(self):
        if self.jam_mode==True:
            self.rlmax=self.rlmax/4
            for i in range(len(self.Springs)):
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(3*self.k)
                    
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(2*self.k) 
            
        return(self.Springs)
        
# get current position         
    def get_position(self):
        self.xb=[]        
        self.zb=[]
        for i in range(len(self.bots)):
            self.xb.append(self.bots[i].GetPos().x)
            self.zb.append(self.bots[i].GetPos().z)
        return(self.xb,self.zb)
    
# Get ball Position
    def get_ball_position(self):
        self.ballx=self.balls[0].GetPos().x
        self.ballz=self.balls[0].GetPos().z
        return(self.ballx,self.ballz)
        
 # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.zbv=[]
        for i in range(len(self.bots)):
            self.xbv.append(self.bots[i].GetPos_dt().x)
            self.zbv.append(self.bots[i].GetPos_dt().z)
        return(self.xbv,self.zbv)
        
 #error of controller        
    def Error(self):
        
        if self.control_type=="shape_form":
            et=[]
            for i in range(self.nb):
                val=(self.rbf(self.zb[i],self.xb[i]))**2
                et.append(val)
            self.E.append(.5*sum(et))
            

# In[environment]            
class enviroment:
    def __init__(self,my_system,material,length,tall,env_mode=None,gapw=None):
        self.my_system=my_system
        self.material=material
        self.length=length
        self.tall=tall
        
        if env_mode is not None:
            self.env_mode=env_mode
        else:
            self.env_mode=None
        if gapw is not None:
            self.gapw=gapw
        else:
            self.gapw=None    
        self.body_floor = chrono.ChBody()
        self.body_floor.SetName('floor')
        self.body_floor.SetBodyFixed(True)
        self.body_floor.SetPos(chrono.ChVectorD(0, -self.tall, 0 ))
        self.body_floor.SetMaterialSurface(self.material)
        self.body_floor.GetCollisionModel().ClearModel()
        self.body_floor.GetCollisionModel().AddBox(self.length, self.tall, self.length) # hemi sizes
        self.body_floor.GetCollisionModel().BuildModel()       
        self.body_floor.SetCollide(True)
        body_floor_shape = chrono.ChBoxShape()
        body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.length, self.tall, self.length)
        self.body_floor.GetAssets().push_back(body_floor_shape)
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 0, 0))
        self.body_floor.AddAsset(col_g)
        self.my_system.Add(self.body_floor)
        
        if self.env_mode=='tunnel':            
            rotate = 30*np.pi/180
            w=2
            l=.5
            l2=l/np.cos(rotate)
            px=3
            pz=(self.gapw+2*l)/2
            # left wall
            wall = chrono.ChBody()
            wall.SetName('wall')
            wall.SetBodyFixed(True)
            wall.SetPos(chrono.ChVectorD(px,.25,pz))
            wall.SetMaterialSurface(self.material)
            wall.GetCollisionModel().ClearModel()
            wall.GetCollisionModel().AddBox(w,.25,l)
            wall.SetCollide(True)
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w,.25,l)
            wall.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.1,0.5))
            wall.AddAsset(col_g)
            self.my_system.Add(wall)
            # right wall 
            wall2 = chrono.ChBody()
            wall2.SetName('wall2')
            wall2.SetBodyFixed(True)
            wall2.SetPos(chrono.ChVectorD(px,.25,-pz))
            wall2.SetMaterialSurface(self.material)
            wall2.GetCollisionModel().ClearModel()
            wall2.GetCollisionModel().AddBox(w,.25,l)
            wall2.SetCollide(True)
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w, .25,l)
            wall2.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.1,0.5))
            wall2.AddAsset(col_g)
            self.my_system.Add(wall2)   
            
        
            # Funnel right 
            wall3 = chrono.ChBody()
            wall3.SetName('wall3')
            wall3.SetBodyFixed(True)
            wall3.SetPos(chrono.ChVectorD((px-(w+w*np.cos(rotate))+l2*np.sin(rotate)),.25, -pz-1*np.sin(rotate)*w ))
            wall3.SetRot(chrono.Q_from_AngY(-rotate))
            wall3.SetMaterialSurface(self.material)
            wall3.GetCollisionModel().ClearModel()
            wall3.GetCollisionModel().AddBox(w,.25,l2)
            wall3.SetCollide(True)
            
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w, .25,l2)
            wall3.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.4,0.5))
            wall3.AddAsset(col_g)
            self.my_system.Add(wall3) 
            
            
            # funnel left
            wall4 = chrono.ChBody()
            wall4.SetName('wall4')
            wall4.SetBodyFixed(True)
            wall4.SetPos(chrono.ChVectorD((px-(w+w*np.cos(rotate))+l2*np.sin(rotate)),.25, pz+1*np.sin(rotate)*w))
            wall4.SetRot(chrono.Q_from_AngY(rotate))
            wall4.SetMaterialSurface(self.material)
            wall4.GetCollisionModel().ClearModel()
            wall4.GetCollisionModel().AddBox(w,.25,l2)
            wall4.SetCollide(True)
            
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w, .25,l2)
            wall4.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.4,0.5))
            wall4.AddAsset(col_g)
            self.my_system.Add(wall4) 

    def return_env(self):
        return(self.my_system)
            
# In[Simulate]
class simulate:
    def __init__(self,my_system,bots,particles,balls,controls,Springs,obj,my_rep,sim,tstep,tend,visual,data_path):   
        self.visual=visual
        self.my_system=my_system
        self.sim=sim
        self.tstep=tstep
        self.tend=tend
        self.data_path=data_path
        self.particles=particles
        self.bots=bots
        self.obj=obj
        self.balls= balls
        self.Springs=Springs
        self.controls=controls
        self.time=[]
        self.my_rep=my_rep
        self.nc=[]
        self.cx=[]
        self.cy=[]
        self.cz=[]
        self.Fxct=[]
        self.Fyct=[]
        self.Fzct=[]
        self.bodiesA=[]
        self.bodiesB=[]
        self.Trip=False
        self.myapplication=[]
        self.timec=[]
    # simulate the robot
    def simulate(self):
        
        #  Create an Irrlicht application to visualize the system
        if self.visual=="irr":
            self.myapplication = chronoirr.ChIrrApp(self.my_system, self.sim , chronoirr.dimension2du(1600,900))
            self.myapplication.AddTypicalSky()
            self.myapplication.AddTypicalLogo()
            self.myapplication.AddTypicalCamera(chronoirr.vector3df(0,2,0),chronoirr.vector3df(0,0,0))
            self.myapplication.SetSymbolscale(.002)
            self.myapplication.SetShowInfos(True)
            self.myapplication.SetContactsDrawMode(2)
            self.myapplication.SetPaused(False)
            #self.myapplication.AddLightWithShadow(chronoirr.vector3df(2,5,2),chronoirr.vector3df(2,2,2),10,2,10,120)
            self.myapplication.AddTypicalLights()
            self.myapplication.DrawAll               
            self.myapplication.AssetBindAll()
            self.myapplication.AssetUpdateAll()
            self.myapplication.AddShadowAll()
            self.count=0
            self.myapplication.SetTimestep(self.tstep)
            self.myapplication.SetTryRealtime(False)
            f=round(1/(self.tstep*60))
            while(self.myapplication.GetDevice().run()):
                #self.my_rep.ResetList()
                self.myapplication.BeginScene()
                self.myapplication.DrawAll()
                self.myapplication.DoStep()
                #print ('time=', self.my_system.GetChTime())
                if self.count%1==0:
                    self.time.append(self.my_system.GetChTime())
                    
                if self.controls!=None:
                    self.controls.run_controller()
                    #aaaa=1
                    #if self.count%1==0:
                        
                        #self.controls.save_data_Forces()
                        #self.controls.Error()
                    self.controls.clear_temp_forces()
                    
#                 #save ball position if it exists
#                 if self.balls!=None:
#                     if self.count%1==0:
                        
#                         self.balls.save_data_position()
#                         self.balls.save_contact_force()
#                         self.balls.save_data_velocity()
#                 # save bot parameters
#                 if self.count%1==0:
#                     self.bots.save_data_position()
#                     self.bots.save_data_Forces()
#                     self.bots.save_data_velocity()
#                     self.bots.save_data_spring_force()
                
#                 # save particle parameters if they exist
#                 if self.particles.particles!=[]:
#                     if self.count%1==0:
                        
#                         self.particles.save_data_position()
#                         self.particles.save_data_velocity()
#                         self.particles.save_data_Forces()
                
#                 # contact data
# #
#                 self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
#                 crt_list = self.my_rep.GetList()
#                 self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
#                 if self.my_system.GetContactContainer().GetNcontacts()!=0:
#                     self.timec.append(self.my_system.GetChTime())
#                     self.cx.append(crt_list[0])
#                     self.cy.append(crt_list[1])
#                     self.cz.append(crt_list[2])
#                     self.Fxct.append(crt_list[3])
#                     self.Fyct.append(crt_list[4])
#                     self.Fzct.append(crt_list[5])
#                     self.bodiesA.append(crt_list[6])
#                     self.bodiesB.append(crt_list[7])
                                
                # run step
                #self.myapplication.DoStep()
                self.myapplication.EndScene()
                # move the camera to follow lead bot
                #aaa=len(self.bots.bots)
                #cam_x=0.33*(self.bots.bots[0].GetPos().x + self.bots.bots[int(aaa/3)].GetPos().x + self.bots.bots[int(2*aaa/3)].GetPos().x)
                #cam_y=0.33*(self.bots.bots[0].GetPos().y + self.bots.bots[int(aaa/3)].GetPos().y + self.bots.bots[int(2*aaa/3)].GetPos().y)
                #cam_z=0.33*(self.bots.bots[0].GetPos().z + self.bots.bots[int(aaa/3)].GetPos().z + self.bots.bots[int(2*aaa/3)].GetPos().z)
                #self.myapplication.GetSceneManager().getActiveCamera().setPosition(chronoirr.vector3df(cam_x,cam_y+2,cam_z))
                #self.myapplication.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3df(cam_x,cam_y,cam_z))
                # save data
                self.myapplication.SetVideoframeSave(True)
                self.myapplication.SetVideoframeSaveInterval(round(1/(self.tstep*100)))
                #file="C:/Users/dmulr/OneDrive/Documents/soro_chrono/python/Pychrono/Strings/Grabbing/Grab_sim/trial/"
                #image=self.myapplication.GetVideoDriver().createScreenShot()
                #self.myapplication.GetVideoDriver().writeImageToFile(image,file);
                #image.drop()
                #zc=self.balls.balls[0].GetPos().z
                #vzc=self.balls.balls[0].GetPos_dt().z
                self.count=self.count+1
                #print(zc,vzc)
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.tend :
                    self.myapplication.GetDevice().closeDevice()
        
        # POV-Ray
        if self.visual=="pov":  
            pov_exporter = postprocess.ChPovRay(self.my_system)

            # Sets some file names for in-out processes.
            pov_exporter.SetTemplateFile(chrono.GetChronoDataFile('_template_POV.pov'))
            pov_exporter.SetOutputScriptFile("rendering_frames"+self.sim+".pov")
            
            if not os.path.exists("output"+self.sim):
                os.mkdir("output"+self.sim)
            if not os.path.exists("anim"+self.sim):
                os.mkdir("anim"+self.sim)

            pov_exporter.SetOutputDataFilebase("output"+self.sim+"/my_state")
            pov_exporter.SetPictureFilebase("anim"+self.sim+"/picture")
            
            pov_exporter.SetCamera(chrono.ChVectorD(0,40,0), chrono.ChVectorD(0,0,0,),90)
            #pov_exporter.SetAmbientLight(chrono.ChColor(1,1,0.9))
            #pov_exporter.SetLight(chrono.ChVectorD(-2,2,-1), chrono.ChColor(0.9,0.9,1.1), True)
            pov_exporter.SetLight(chrono.ChVectorD(-2,2,-1), chrono.ChColor(1.1,1.2,1.2), True)
            pov_exporter.SetPictureSize(1600,900)
            pov_exporter.SetAmbientLight(chrono.ChColor(2,2,2))                  
            pov_exporter.AddAll()
            pov_exporter.ExportScript()
            count=0
            while (self.my_system.GetChTime() < self.tend) :                  
                self.my_rep.ResetList()
                self.my_system.DoStepDynamics(self.tstep)
                # run the controllers if they exist
                if self.controls!=None:
                    self.controls.run_controller()
                    self.controls.save_data_Forces()
                    self.controls.clear_temp_forces()
                    #self.controls.Error()
                self.time.append(self.my_system.GetChTime())
    
                # save ball position if it exists
                if self.balls!=None:
                    self.balls.save_data_position()
                    self.balls.save_contact_force()
                    self.balls.save_data_velocity()
                
                # save bot parameters
                self.bots.save_data_position()
                self.bots.save_data_Forces()
                self.bots.save_data_velocity()
                self.bots.save_data_spring_force()
                
                # save particle parameters if they exist
                if self.particles.particles!=[]:
                    self.particles.save_data_position()
                    self.particles.save_data_velocity()
                    self.particles.save_data_Forces()
                
                # contact data
                self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
                crt_list = self.my_rep.GetList()
                self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
                self.cx.append(crt_list[0])
                self.cy.append(crt_list[1])
                self.cz.append(crt_list[2])
                self.Fxct.append(crt_list[3])
                self.Fyct.append(crt_list[4])
                self.Fzct.append(crt_list[5])
                self.bodiesA.append(crt_list[6])
                self.bodiesB.append(crt_list[7])
                print('time=', self.my_system.GetChTime())
                if count%2==0:
                    pov_exporter.ExportData()   
                count=count+1
                
        return(self.bots,self.time,self.controls,self.cx,self.cy,self.cz,self.Fxct,self.Fyct,self.Fzct,self.nc,self.bodiesA,self.bodiesB)     
#[material]          
def Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs):
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(mu_f)
    material.SetDampingF(mu_b)
    material.SetCompliance (C)
    material.SetComplianceT(Ct)
    # material.SetRollingFriction(mu_r)
    # material.SetSpinningFriction(mu_s)
    # material.SetComplianceRolling(Cr)
    # material.SetComplianceSpinning(Cs)
    return material
#[Create Floor]
def Floor(material,length,tall):
    body_floor = chrono.ChBody()
    body_floor.SetName('floor')
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVectorD(0, -tall, 0 ))
    body_floor.SetMaterialSurface(material)
    body_floor.GetCollisionModel().ClearModel()
    body_floor.GetCollisionModel().AddBox(length, tall, length) # hemi sizes
    body_floor.GetCollisionModel().BuildModel()
    body_floor.SetCollide(True)
    body_floor_shape = chrono.ChBoxShape()
    body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(length, tall, length)
    body_floor.GetAssets().push_back(body_floor_shape)
    #body_floor_texture = chrono.ChTexture()
    #body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    #body_floor.GetAssets().push_back(body_floor_texture)
    return(body_floor)
    
# %% Class for creating the points for the RBF
class Points_for_shape:
    def __init__(self,shape,p1,p2,nb,diameter,bl,br,R,nr,Rd):
        self.shape=shape
        self.p1=p1
        self.p2=p2
        self.nb=nb
        self.nr=nr
        self.diameter=diameter
        self.Rd=Rd
        self.R=R
        self.x=0
        self.y=0
        self.z=0
        self.x2=0
        self.y2=0
        self.z2=0
        
        self.rbf=0
        self.rbf2=0
        self.br=br
        self.bl=bl
        self.fny=0
        self.fnx=0
        self.xp=0
        self.yp=0
        self.A=1e-4
    # plot the RBF function
    def Plot_rbf(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (1.25*fsy, fsy))     # Set the figure size to square
        ti = np.linspace(self.bl, self.br, 100)
        xx, yy = np.meshgrid(ti, ti)
        zz = self.rbf(xx, yy)
        plt.pcolor(xx, yy, zz,cmap = 'jet')
        plt.colorbar()
    
        fig = plt.figure(figsize = (fsx, fsy))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(xx, yy, zz, cmap = 'jet')   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        fig.colorbar(surf)                              # Include color bar
        plt.show()

    def f2_(self,x,y):
        d=np.sqrt((1/self.Rd**2)*((x-self.p1)**2+(y-self.p2)**2))
        d1=-2*self.p1+2*x
        d2=-2*self.p2+2*y
        self.fnx=self.A*(2*(2/self.Rd**2)*d1*d*(np.log(d))**2 + (2/self.Rd**2)*d1*d*(np.log(d)))
        self.fny=self.A*(2*(2/self.Rd**2)*d2*d*(np.log(d))**2 + (2/self.Rd**2)*d2*d*(np.log(d)))
        return(self.fnx,self.fny)
        
# create the gradient ^2 function    
    def Create_shape_gradient(self):
        (self.x,self.y,self.z) =self.Points_for_shape()
        (self.rbf)=self.create_RBF()
        ti = np.linspace(self.bl, self.br, 100)
        xx, yy = np.meshgrid(ti, ti)
        zz = self.rbf(xx, yy)
        (zy,zx)=np.gradient(zz**2)
        # gradient of y
        self.fny = RegularGridInterpolator((ti,ti),zy)
        # gradient of x
        self.fnx = RegularGridInterpolator((ti,ti),zx)
        return(self.rbf,self.fnx,self.fny)
# Create the points for the RBF
    def Points_for_shape(self):
        # grab
        if self.shape=='grab':
            (self.x,self.y,self.z)=self.points_grab()
        # grab2   
        if self.shape=='grab2':
            (self.x,self.y,self.z)=self.points_grab2() 
                 
        # Circle           
        if self.shape=='circle':
            (self.x,self.y,self.z)=self.Points_circle()
            
        if self.shape=='oval':
            (self.x,self.y,self.z)=self.Points_oval()    
        # Square                           
        if self.shape=='Square':
            (self.x,self.y,self.z)=self.points_square()   
            
        return (self.x,self.y,self.z)  
    
# create radial basis function                 
    def create_RBF(self):
        self.rbf = Rbf(self.x,self.y,self.z,function='thin_plate')  # radial basis function interpolator instance
        return (self.rbf) 
    
#[ points for Square]    
    def points_square(self):
        xt=np.array([1,1,0,-1,-1,-1,0,1])
        yt=np.array([0,1,1,1,0,-1,-1,-1])
    
        self.x=np.zeros(len(self.nr)*len(xt))
        self.y=np.zeros(len(self.nr)*len(xt))
        self.z=np.zeros(len(self.nr)*len(xt))

    
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                self.x[i+len(xt)*j]=(self.nr[j]+self.Rd)*xt[i]+self.p1
                self.y[i+len(xt)*j]=(self.nr[j]+self.Rd)*yt[i]+self.p2
                if j==0:
                    self.z[i+len(xt)*j]=0
                else:
                    self.z[i+len(xt)*j]=j             
        return (self.x,self.y,self.z)   
#[points for grab]
    def points_grab(self):
        
        #theta1=np.linspace(np.pi/4,7*np.pi/4,int(.67*self.nb))
        self.OR=1.2*self.R
        self.IR=.7*self.R
        self.epsilon=0
        theta1=np.linspace(np.pi/2,3*np.pi/2,int(.67*self.nb))
        x1=self.OR*np.cos(theta1)+self.p1
        y1=self.OR*np.sin(theta1)+self.p2

        theta2=np.linspace(1*np.pi/2,3*np.pi/2,int(.3*self.nb))
        x2=self.IR*np.cos(theta2)+self.epsilon+self.p1
        y2=self.IR*np.sin(theta2)+self.p2 
        
        self.xp=np.concatenate((x1,x2), axis=None)
        self.yp=np.concatenate((y1,y2), axis=None)
        zp=np.zeros(len(self.xp))
        
        theta3=np.linspace(np.pi/2,3*np.pi/2,int(.3*self.nb))
        x3=self.IR*np.cos(theta3)+self.p1+.5
        y3=self.IR*np.sin(theta3)+self.p2

        theta4=np.linspace(np.pi/2,3*np.pi/2,int(.67*self.nb))
        x4=1.5*self.R*np.cos(theta4)+self.p1
        y4=1.5*self.R*np.sin(theta4)+self.p2
        
        xr=np.concatenate((x3, x4), axis=None)
        yr=np.concatenate((y3, y4), axis=None)
        zr=2*np.ones(len(xr))
    
        self.x=np.concatenate((self.xp, xr), axis=None)
        self.y=np.concatenate((self.yp, yr), axis=None)
        self.z=np.concatenate((zp, zr), axis=None)
        
        #plt.plot(x1,y1,'bo',x2,y2,'bo',x3,y3,'go',x4,y4,'go')
        return (self.x,self.y,self.z) 
                 
            
    # plot points used to make rbf
    def plot_points(self):
        plt.scatter(self.xp,self.yp,color='blue')

    # function for points grab 2    
    def points_grab2(self):
        path="C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/python/Pychrono/Grabbing/Grab_sim_pot_field_shape/"
        file="points.npz"
        data=np.load(path+file,allow_pickle=True)
        YL=.7*data['XL']
        XL=.7*data['ZL']
        self.xp2=np.concatenate((XL), axis=None)
        self.yp2=np.concatenate((YL), axis=None)
        XL2=3.5*XL
        YL2=3.5*YL+.2

        XL3=3*XL2
        YL3=3*YL2
        
        zp=np.zeros(len(XL))
        zr=np.ones(len(XL2))
        zr2=2*np.ones(len(XL2))
        self.x=np.concatenate((XL,XL3), axis=None)
        self.y=np.concatenate((YL,YL3), axis=None)
        self.z=np.concatenate((zp, zr2), axis=None)
        return (self.x,self.y,self.z) 

#[ Points for circle]
    def Points_circle(self):
        theta=np.linspace(0,2*np.pi,self.nb)

    # create empty arays
        self.x=np.zeros(len(self.nr)*self.nb)
        self.y=np.zeros(len(self.nr)*self.nb)
        self.z=np.zeros(len(self.nr)*self.nb)
# create rings
        for j in range (len(self.nr)):
            for i in range(self.nb):
                nt=self.nr[j]
#                self.x[i+self.nb*j]=(self.R+nt)*np.cos(theta[i])+self.p1
#                self.y[i+self.nb*j]=(self.R+nt)*np.sin(theta[i])+self.p2
                self.x[i+self.nb*j]=(self.Rd+nt)*np.cos(theta[i])+self.p1
                self.y[i+self.nb*j]=(self.Rd+nt)*np.sin(theta[i])+self.p2
                if j==0:
                    self.z[i+self.nb*j]=0
                else:
                    self.z[i+self.nb*j]=nt
                    
            self.xp=self.x[0:self.nb]
            self.yp=self.y[0:self.nb]           
        return (self.x,self.y,self.z)   
#[ Points for circle]
        
    def Points_oval(self):
        theta=np.linspace(0,2*np.pi,self.nb)

    # create empty arays
        self.x=np.zeros(len(self.nr)*self.nb)
        self.y=np.zeros(len(self.nr)*self.nb)
        self.z=np.zeros(len(self.nr)*self.nb)
# create rings
        for j in range (len(self.nr)):
            for i in range(self.nb):
                nt=self.nr[j]
#                self.x[i+self.nb*j]=(self.R+nt)*np.cos(theta[i])+self.p1
#                self.y[i+self.nb*j]=(self.R+nt)*np.sin(theta[i])+self.p2
                self.x[i+self.nb*j]=((1*self.Rd)+nt)*np.cos(theta[i])+self.p1
                self.y[i+self.nb*j]=((3*self.Rd)+nt)*np.sin(theta[i])+self.p2
                if j==0:
                    self.z[i+self.nb*j]=0
                else:
                    self.z[i+self.nb*j]=nt
                    
            self.xp=self.x[0:self.nb]
            self.yp=self.y[0:self.nb]           
        return (self.x,self.y,self.z)   

# In[Analytical function]
class points_shape_A:
    def __init__(self,px,py,R):
        
        self.px=px # center x
        self.py=py # center y
        
        self.R=R # radius of zero contour
        self.vmax=1
        self.alpha=self.vmax # max velocity
        self.beta=0.1 # dampning term 
        self.limits=True
        
        self.b=6 # how far left or right
        self.res=.1 # resolution 
        self.C=1 # constant
        self.A=1
        # Settting up the grid of the potential field
    
        self.xmin=self.px-self.b 
        self.xmax=self.px+self.b
        
        self.ymin=self.py-self.b
        self.ymax=self.py+self.b  
        
        self.xcount=int(round(abs(self.xmax-self.xmin)/self.res))  # find out how many numbers in the x
        self.ycount=int(round(abs(self.ymax-self.ymin)/self.res))  # find out how many numbers in the y
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # set up range for x 
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # set up range for y
        print(len(self.xp))
        print(len(self.yp))

        
        self.xx,self.yy=np.meshgrid(self.xp, self.yp) # create mesh grid
        self.d=np.sqrt((1/self.R**2)*((self.xx-self.px)**2+(self.yy-self.py)**2)) # create d
        self.f=self.d**2*np.log(self.d) # potential function 
        
        self.maxf=np.max(self.f) # find max value in potential field 
        self.fx=0
        self.fy=0
        # if we want to limit the field 
        if self.limits==True:
            self.f=self.f/self.maxf # scale down potential field 
        
        
        # create gradient 
        self.d1=-2*self.px+2*self.xx
        self.d2=-2*self.py+2*self.yy 
    def create_gradient(self):
        self.xmin=self.px-self.b 
        self.xmax=self.px+self.b
        
        self.ymin=self.py-self.b
        self.ymax=self.py+self.b  
        
        self.xcount=int(round(abs(self.xmax-self.xmin)/self.res))  # find out how many numbers in the x
        self.ycount=int(round(abs(self.ymax-self.ymin)/self.res))  # find out how many numbers in the y
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # set up range for x 
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # set up range for y

        self.xx,self.yy=np.meshgrid(self.xp, self.yp) # create mesh grid
        self.d=np.sqrt((1/self.R**2)*((self.xx-self.px)**2+(self.yy-self.py)**2)) # create d
        self.d1=-2*self.px+2*self.xx
        self.d2=-2*self.py+2*self.yy
        self.fx=np.nan_to_num(self.C*(2*(2/self.R**2)*self.d1*self.d*(np.log((self.d)**2)) + (2/self.R**2)*self.d1*self.d*(np.log(self.d)))) # gradient x
        self.fy=np.nan_to_num(self.C*(2*(2/self.R**2)*self.d2*self.d*(np.log((self.d)**2)) + (2/self.R**2)*self.d2*self.d*(np.log(self.d))))  # gradient y
        

        
        if self.limits==True:
            
            self.fx=self.fx/self.A
            self.fy=self.fy/self.A   
        
        # Create a interpolated version for us to easily access 
        self.fny = RegularGridInterpolator((self.xp,self.yp),self.fy) # gradient of y
        self.fnx = RegularGridInterpolator((self.xp,self.yp),self.fx) # gradient of x 
        return(self.fny,self.fnx)

        
    def out_force(self,x,y):
        f2x=self.fnx((y,x))
        f2z=self.fny((y,x))
     
        return(f2x,f2z)
        
    def out_force_2(self,x,y):
        f2x=self.fnx((y,x))
        f2z=self.fny((y,x))
        mag=np.sqrt(f2x**2+f2z**2)
        FX=f2x/mag
        FZ=f2z/mag
        
        #print(np.sqrt(FX**2+FZ**2))
        return(FX,FZ)
        
# In[local pot_fields]  
class local_field:
    def __init__(self):
        self.px=[]
        self.pz=[]
        self.F=0
        self.fx=0
        self.fz=0
        self.b=10
        self.res=100
        self.xr=np.linspace(-self.b,self.b,self.res)
        self.zr=np.linspace(-self.b,self.b,self.res)
        self.xx,self.zz=np.meshgrid(self.xr,self.zr)
        self.fx=0
        self.fz=0
        self.fnx=0
        self.fnz=0
        self.R=.25
        self.Ft=np.zeros_like(self.xx)
        self.F=np.zeros_like(self.xx)
        
    def add_px(self,x,z):

        self.px.append(x)
        self.pz.append(z)
        
    def update_F_fx_fz(self):  
        self.F=np.zeros_like(self.xx)
        for i in range(len(self.px)):
            self.r=np.sqrt((self.xx+self.px[i])**2+(self.zz+self.pz[i])**2)
            self.Ft=-1/self.r
            self.F=self.F+self.Ft
        
        (self.fz,self.fx)=np.gradient(self.F)
        Ax=np.amax(self.fx)
        Az=np.amax(self.fz)
        self.fx=self.fx/Ax
        self.fx=self.fz/Az        
        self.fnz=RegularGridInterpolator((self.zr,self.xr),self.fz)
        self.fnx=RegularGridInterpolator((self.zr,self.xr),self.fx)            
        
 # In[Point pot field ]       
class point_field:
    def __init__(self,px,pz):
        self.px=px
        self.pz=pz
        self.F=0
        self.fx=0
        self.fz=0
        self.b=20
        self.res=1000
        self.xr=np.linspace(-self.b,self.b,self.res)
        self.zr=np.linspace(-self.b,self.b,self.res)
        self.xx,self.yy=np.meshgrid(self.xr,self.zr)
        self.fx=0
        self.fz=0
        self.fnx=0
        self.fnz=0
        self.r=np.sqrt((self.xx-self.px)**2+(self.yy-self.pz)**2)
        self.F=(self.r)
        self.zz=self.F
        (self.fz,self.fx)=np.gradient(self.F**2)
        Ax=np.amax(self.fx)
        Az=np.amax(self.fz)
        self.fx=self.fx/Ax
        self.fz=self.fz/Az        
        self.fny=RegularGridInterpolator((self.zr,self.xr),self.fz)
        self.fnx=RegularGridInterpolator((self.zr,self.xr),self.fx) 
        
    def plot_f3d(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig = plt.figure(figsize = (fsx, fsy))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(self.xx,self.yy,self.zz,cmap='jet', )   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.title('plot')
        fig.colorbar(surf)                              # Include color bar
        # Annotate text on figure at x, y, z coordinate
        plt.show()  
        
    def plot_fiso(self):        
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig, ax = plt.subplots(figsize = (1.25*fsy, fsy))
        CS = ax.contour(self.xx,self.yy, self.zz,30)
        ax.clabel(CS, inline=1, fontsize=10)
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")       
         
        
    def plot_f2d(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig = plt.figure(figsize = (1.25*fsy, fsy))     # Set the figure size to square
        plt.pcolor(self.xx, self.yy, self.zz, cmap = 'jet')         # Pseudocolor plot of data and choose colormap
        plt.colorbar()
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.show()
# In[Pot fields]               
class moving_pot_fields:
    def __init__(self,px,py,res,mode,r1,r2,k1):
        self.px=px
        self.py=py
        self.res=res
        self.r1=r1
        self.r2=r2
        self.k1=k1
        self.b=10
        self.xmin=self.px-self.b
        self.xmax=self.px+self.b
        self.ymin=self.py-self.b
        self.ymax=self.py+self.b
        self.xcount=int(round((self.xmax-self.xmin)/self.res))
        self.ycount=int(round((self.ymax-self.ymin)/self.res))
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount)
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount)
        self.yy,self.xx=np.meshgrid(self.yp,self.xp)
        self.mode=mode
        if self.mode==1:
            self.f1=(self.xx-self.px)**2+(self.yy-self.py)**2-(self.r1)**2
            self.p1=self.k1*np.maximum(0,self.f1)
            self.p1=self.p1/np.max(self.p1)
            self.f2=((self.r2)**2)-(self.xx-self.px)**2-(self.yy-self.py)**2
            self.p2=(self.k1)*(np.maximum(0,self.f2))**2
            self.p2=self.p2/np.max(self.p2)
            self.zz=self.p1+self.p2
            self.fy,self.fx=np.gradient(self.zz)
            self.f= RegularGridInterpolator((self.yp,self.xp),self.zz)
            self.fnx= RegularGridInterpolator((self.yp,self.xp),self.fx)
            self.fny= RegularGridInterpolator((self.yp,self.xp),self.fy)
            
        if self.mode==2:
            self.shape="Square"
            self.nr=[0,.1,.2,.3]
            self.nr2=np.linspace(0,self.r2/2,3)
            (self.x,self.y,self.z,self.x2,self.y2,self.z2)=self.Points_for_shape()
            self.rbf = Rbf(self.x,self.y,self.z,function='thin_plate')
            self.rbf2 = Rbf(self.x2,self.y2,self.z2,function='thin_plate')
            t=np.linspace(-self.b,self.b,len(self.xp))
            self.xxt,self.yyt=np.meshgrid(t,t)
            self.zz1 = self.rbf(self.xxt, self.yyt)
            self.zz2 = self.rbf2(self.xxt, self.yyt)
            self.zz1=self.zz1/np.max(self.zz1)
            self.zz1=np.maximum(0,self.zz1)
            self.zz2=self.zz2/np.max(self.zz2)
            self.zz2=-self.zz2
            self.zz2=np.maximum(0,self.zz2)
            self.zz=self.zz1+self.zz2
            self.fy,self.fx=np.gradient(self.zz) 
            self.f = RegularGridInterpolator((self.yp,self.xp),self.zz)
            self.fnx= RegularGridInterpolator((self.yp,self.xp),self.fx)
            self.fny= RegularGridInterpolator((self.yp,self.xp),self.fy)
    def create_gradient(self):
        self.xmin=self.px-self.b
        self.xmax=self.px+self.b
        self.ymin=self.py-self.b
        self.ymax=self.py+self.b
        self.xcount=int(round((self.xmax-self.xmin)/self.res))
        self.ycount=int(round((self.ymax-self.ymin)/self.res))
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount)
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount)  
        self.fnx= RegularGridInterpolator((self.yp,self.xp),self.fx)
        self.fny= RegularGridInterpolator((self.yp,self.xp),self.fy)
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz)
            
 

    def Points_for_shape(self):
        # grab
        if self.shape=='grab':
            (self.x,self.y,self.z)=self.points_grab()
        # grab2   
        if self.shape=='grab2':
            (self.x,self.y,self.z)=self.points_grab2() 

        # Circle           
        if self.shape=='circle':
            (self.x,self.y,self.z)=self.Points_circle()

        if self.shape=='oval':
            (self.x,self.y,self.z)=self.Points_oval()    
        # Square                           
        if self.shape=='Square':
            (self.x,self.y,self.z)=self.points_square()
            (self.x2,self.y2,self.z2)=self.points_square2()
        return (self.x,self.y,self.z,self.x2,self.y2,self.z2)              
    
    def square(self,px,xp):
        return(1 * (abs(xp) < abs(px+self.r1)))
    
    #[ points for Square]    
    def points_square(self):
        xt=np.array([1,1,0,-1,-1,-1,0,1])
        yt=np.array([0,1,1,1,0,-1,-1,-1])

        self.x=np.zeros(len(self.nr)*len(xt))
        self.y=np.zeros(len(self.nr)*len(xt))
        self.z=np.zeros(len(self.nr)*len(xt))
        

        for j in range(len(self.nr)):
            for i in range(len(xt)):
                self.x[i+len(xt)*j]=(self.r1+self.nr[j])*xt[i]
                self.y[i+len(xt)*j]=(self.r1+self.nr[j])*yt[i]
                self.z[i+len(xt)*j]=abs(j) 
            
                
        return (self.x,self.y,self.z)   
    
    def points_square2(self):
        xt=np.array([1,1,0,-1,-1,-1,0,1])
        yt=np.array([0,1,1,1,0,-1,-1,-1])

        self.x2=np.zeros(len(self.nr2)*len(xt))
        self.y2=np.zeros(len(self.nr2)*len(xt))
        self.z2=np.zeros(len(self.nr2)*len(xt))


        for j in range(len(self.nr2)):
            for i in range(len(xt)):
                self.x2[i+len(xt)*j]=(self.r2-self.nr2[j])*xt[i]
                self.y2[i+len(xt)*j]=(self.r2-self.nr2[j])*yt[i]
                self.z2[i+len(xt)*j]=-abs(j) 
                
        return (self.x2,self.y2,self.z2) 
    
    def plot_f3d(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig = plt.figure(figsize = (fsx, fsy))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(self.xx,self.yy,self.zz,cmap='jet', )   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.title('plot')
        fig.colorbar(surf)                              # Include color bar
        # Annotate text on figure at x, y, z coordinate
        plt.show()  
        
    def plot_fiso(self):        
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig, ax = plt.subplots(figsize = (1.25*fsy, fsy))
        CS = ax.contour(self.xx,self.yy, self.zz,30)
        ax.clabel(CS, inline=1, fontsize=10)
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")       
         
        
    def plot_f2d(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig = plt.figure(figsize = (1.25*fsy, fsy))     # Set the figure size to square
        plt.pcolor(self.xx, self.yy, self.zz, cmap = 'jet')         # Pseudocolor plot of data and choose colormap
        plt.colorbar()
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.show()  
                        
    def plot_cut(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig = plt.figure(figsize=(16,8)) 
        plt.subplot(1, 2, 1)
        plt.gca().set_title('cross cut of the field')
        plt.plot(self.xp,self.z1,'b')
        plt.plot(self.xp,self.z2,'g')
        plt.plot(self.X,self.Z1,'sr')
        plt.axvline(x=self.r2+self.px,color='violet')
        plt.axvline(x=self.r1+self.px,color='violet')
        plt.axvline(x=self.px-self.r2,color='violet')
        plt.axvline(x=self.px-self.r1,color='violet')
        plt.subplot(1, 2, 2)
        plt.gca().set_title('zoomed in section')
        plt.plot(self.xp,self.z1,'b')
        plt.plot(self.xp,self.z2,'g')
        plt.plot(self.X,self.Z1,'sr')
        plt.axvline(x=self.r2+self.px,color='violet')
        plt.axvline(x=self.r1+self.px,color='violet')
        plt.axvline(x=self.px-self.r2,color='violet')
        plt.axvline(x=self.px-self.r1,color='violet')
        plt.xlim(self.r1+self.px-5,self.r2+self.px+5)
        
        
    def plot_cut2(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden 
        fig = plt.figure(figsize=(16,8)) 
        plt.subplot(1, 2, 1)
        plt.gca().set_title('cross cut of the field')
        plt.plot(self.xp,self.z1,'b')
        plt.plot(self.xp,self.z2,'g')
        plt.plot(self.X,self.Z1,'sr')
        plt.axvline(x=self.r2+self.px,color='violet')
        plt.axvline(x=self.r1+self.px,color='violet')
        plt.axvline(x=self.px-self.r2,color='violet')
        plt.axvline(x=self.px-self.r1,color='violet')
        plt.axvline(x=self.px-6,color='k')
        plt.axvline(x=self.px+6,color='k')
        
        plt.subplot(1, 2, 2)
        plt.gca().set_title('zoomed in section')
        plt.plot(self.xp,self.z1,'b')
        plt.plot(self.xp,self.z2,'g')
        plt.plot(self.X,self.Z1,'sr')
        plt.axvline(x=self.r2+self.px,color='violet')
        plt.axvline(x=self.r1+self.px,color='violet')
        plt.axvline(x=self.px-self.r2,color='violet')
        plt.axvline(x=self.px-self.r1,color='violet')
        plt.axvline(x=self.px-6,color='k')
        plt.axvline(x=self.px+6,color='k')
        plt.xlim(self.r1+self.px-5,self.r2+self.px+5)    

        
    def out_vel2(self, x, y):
            #VY = self.fny((y, x))*np.maximum(0,self.f((y,x)))
            #VX = self.fnx((y, x))*np.maximum(0,self.f((y,x)))
            
            #VX=np.round(VX,3)
            #VY=np.round(VY,3)
            #print(self.f((y,x)))
        print(np.round(self.f((y,x)),3))
        A=np.round(self.f((y,x)),3)
        if A<.003:
            V=0
            Vx=0
            Vy=0
            theta=0
            print('gucci')
        else:
            #print(VX,VY)
            VY = self.fny((y, x))*np.maximum(0,self.f((y,x)))
            VX = self.fnx((y, x))*np.maximum(0,self.f((y,x)))
            #VX=self.fnx((y,x))
            #VY=self.fny((y,x))
            mag=np.sqrt(VY**2+VX**2)
            VY=VY/mag
            VX=VX/mag
            Vx = -self.alpha * VX #- self.beta * vx
            Vy = -self.alpha * VY #- self.beta * vy
            V = np.sqrt(Vx ** 2 + Vy ** 2)
            theta = (180 * np.arctan2(Vy, Vx) / np.pi) % 360
        return (V,Vx,Vy,theta)                       
# In[Path]          
class Path:
    def __init__(self,start,end,p):
        
        self.p=p
        self.start=start
        self.end=end
        self.length=10000
        
        if self.p=='flat_line':
            self.ax=np.linspace(self.start,self.end,self.length)
            self.ay=np.zeros(len(self.ax))
            self.f=scipy.interpolate.interp1d(self.ax,self.ay)
        if self.p=='line':
            self.ax=np.linspace(self.start,self.end,self.length)
            self.ay=self.ax
            self.f=scipy.interpolate.interp1d(self.ax,self.ay)            
        if self.p=='sine':
            self.ax=np.linspace(self.start,self.end,self.length)
            self.w=.1
            self.y0=0
            self.x0=0
            self.ay=np.sin(self.w*self.ax)+self.y0
            
        if self.p=='parab':
            self.ax=np.linspace(self.start,self.end,self.length)
            self.x0=0
            self.y0=0
            self.ay=(self.ax-self.x0)**2+self.y0
            self.f=scipy.interpolate.interp1d(self.ax,self.ay)
        # parameter representation of the curve
        x, y = map(np.asarray, (self.ax, self.ay))
        s = np.append([0],(np.cumsum(np.diff(x)**2) + np.cumsum(np.diff(y)**2))**0.5)
        
        # spline
        self.X = scipy.interpolate.CubicSpline(s, x)
        self.Y = scipy.interpolate.CubicSpline(s, y)
        
        self.dX = self.X.derivative(1)
        self.ddX = self.X.derivative(2)

        self.dY = self.Y.derivative(1)
        self.ddY = self.Y.derivative(2)
        self.length = s[-1]
        
# orientation
    def calc_yaw(self, s):
        dx, dy = self.dX(s), self.dY(s)
        yaw=np.arctan2(dy, dx)
        return yaw
    
# curvature    
    def calc_curvature(self, s):
        dx, dy   = self.dX(s), self.dY(s)
        ddx, ddy   = self.ddX(s), self.ddY(s)
        kp=(ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return kp

# Find distance to nearest point
    def __find_nearest_point(self, s0, x, y):
        def calc_distance(_s, *args):
            _x, _y= self.X(_s), self.Y(_s)
            return np.sqrt((_x - args[0])**2 + (_y - args[1])**2)
        
        def calc_distance_jacobian(_s, *args):
            _x, _y = self.X(_s), self.Y(_s)
            _dx, _dy = self.dX(_s), self.dY(_s)
            return 2*(_dx*(_x - args[0])+_dy*(_y-args[1]))
        minimum = optimize.fmin_cg(calc_distance, s0, calc_distance_jacobian, args=(x, y), full_output=True, disp=False)
        #print(minimum)
        return minimum
    
    def calc_track_error(self, x, y, s0):
        # find nearest point
        ret = self.__find_nearest_point(s0, x, y)
        
        if type(ret[1]) is float:
            e=ret[1]
        else:
            e=ret[1][0]
            
        # s value that gives point P 
        s = ret[0][0]
        
        # curvature
        k   = self.calc_curvature(s)
        # tangent
        yaw_ref = self.calc_yaw(s)
        
        dxl = self.X(s) - x
        dyl = self.Y(s) - y
        # error between the path and robot
        angle = self.pi_2_pi(yaw_ref - math.atan2(dyl, dxl))
        if angle < 0:
            e*= -1
        # Nearest point
        nearpt=np.array([self.X(ret[1]),self.Y(ret[1])])

        return e, k, yaw_ref, s, nearpt    

    # Convert angle       
    def pi_2_pi(self,angle):
        while(angle > math.pi):
            angle = angle - 2.0 * math.pi

        while(angle < -math.pi):
            angle = angle + 2.0 * math.pi

        return angle

    def plot_path(self):
        plt.plot(self.ax,self.ay,'b')
        plt.show()
# In[Functions for Checking if certain things are present] 
def check_shape_generator(shapes,shape,xball,zball):
    if shape=="circleA":
        print('0')
    
    else:
        shapes.p1=zball
        shapes.p2=xball
        shapes.Create_shape_gradient()
    return(shapes)
   
# In[Export Data] 
# Contact callback    
class MyReportContactCallback(chrono.ReportContactCallback):

    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.bodiesA = []
        self.bodiesB = []
        self.CA=[]

    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        #AA=cA*chrono.ChVectorD(vA.x,vA.y,vA.z)
        #BB=cA*chrono.ChVectorD(force.x,force.y,force.z)
        #self.pointx.append(AA.x)
        #self.pointy.append(AA.y)
        #self.pointz.append(AA.z)
#        self.Fxc.append(BB.x)
#        self.Fyc.append(BB.y)
#        self.Fzc.append(BB.z)        
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fxc.append(force.x)
        self.Fyc.append(force.y)
        self.Fzc.append(force.z)
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        self.CA.append(cA)
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = [] 
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.bodiesA=[]
        self.bodiesB=[]
        self.CA=[]
    # Get the points
    def GetList(self):
        return (self.pointx,self.pointy,self.pointz,self.Fxc,self.Fyc,self.Fzc,self.bodiesA,self.bodiesB)
    
class MyReportContactCallback2(chrono.ReportContactCallback):

    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.bodiesA = []
        self.bodiesB = []
        self.CA=[]
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fxc.append(force.x)
        self.Fyc.append(force.y)
        self.Fzc.append(force.z)
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        self.CA.append(cA)
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = [] 
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.bodiesA=[]
        self.bodiesB=[]
        self.CA=[]
        
    # Get the points
    def GetList(self):
        return (self.pointx,self.pointy,self.pointz,self.Fxc,self.Fyc,self.Fzc,self.bodiesA,self.bodiesB,self.CA)
   # export data             
class export_data():
    def __init__(self,bots,nb,sim,time,cx,cy,cz,Fxct,Fyct,Fzct,nc,save_data,mr,mp,mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs,tend,interior,ball,controller,bodiesA,bodiesB,shape):
        # objects 
        self.bots=bots
        if interior is not None:
            self.interior=interior
        else:
            self.interior=None
        if ball is not None:
            
            self.ball=ball
        else:
            self.ball=None
        if controller is not None:
            self.controller=controller
        else:
            self.controller=None
        if shape is not None:
            self.shape=shape
        else:
            self.shape=None
        self.control_type=self.controller.control_type
        
        # additional variables
        self.tend=tend
        self.time={'time': time}
        self.sim=sim
        self.nb=nb
        self.nc=np.asarray(nc)
        self.lengthm=np.amax(self.nc)
        self.cx=cx
        self.cy=cy
        self.cz=cz
        self.bodiesA=bodiesA
        self.bodiesB=bodiesB
        self.FBX=[]
        self.FBZ=[]
        self.Fxct=Fxct
        self.Fyct=Fyct
        self.Fzct=Fzct
        self.count=len(time)
        self.lengthm=np.amax(self.nc)
        self.save_data=save_data
        self.pointx={}
        self.pointz={}
        
        self.Fpointx={}
        self.Fpointz={}
        
        #Create empty contact matrices
        self.xc=np.zeros((self.lengthm,self.count))
        self.yc=np.zeros((self.lengthm,self.count))
        self.zc=np.zeros((self.lengthm,self.count))
# Contact forces
        self.Fcx=np.zeros((self.lengthm,self.count))
        self.Fcy=np.zeros((self.lengthm,self.count))
        self.Fcz=np.zeros((self.lengthm,self.count))
        self.BN=np.zeros((self.lengthm,self.count))
        self.AN=np.zeros((self.lengthm,self.count))
        self.FCX=[]
        self.FCZ=[]
        self.shape=shape
        self.mp=mp
        self.mr=mr
        
        self.mu_f=mu_f
        self.mu_b=mu_b
        self.mu_r=mu_r
        self.mu_s=mu_s
        self.C=C
        self.Ct=Ct
        self.Cr=Cr
        self.Cs=Cs

        #for i in range(self.count):
            # ind=self.nc[i]
            # tryme=self.cx[i]
            # tryme2=self.cy[i]
            # tryme3=self.cz[i]
            # tryme4=self.Fxct[i]
            # tryme5=self.Fyct[i]
            # tryme6=self.Fzct[i]
            # tryme7t=self.bodiesA[i]
            # tryme8t=self.bodiesB[i]
            # tryme7=np.zeros(len(tryme7t))     
            # tryme8=np.zeros(len(tryme7t)) 
            
            # for k in range(len(tryme7)):
            #     #print(tryme7t[k])
            #     if tryme7t[k]=='floor':
            #         tryme7[k]=1
            #     if tryme7t[k]=='bot':
            #         tryme7[k]=2
            #     if tryme7t[k]=='gran':
            #         tryme7[k]=3
            #     if tryme7t[k]=='ball000':
            #         tryme7[k]=4
            #     if tryme7t[k]=='skin':
            #         tryme7[k]=5
            #     if tryme8t[k]=='floor':
            #         tryme8[k]=1
            #     if tryme8t[k]=='bot':
            #         tryme8[k]=2
            #     if tryme8t[k]=='gran':
            #         tryme8[k]=3
            #     if tryme8t[k]=='ball000':
            #         tryme8[k]=4
            #     if tryme8t[k]=='skin':
            #         tryme8[k]=5                   
            # # convert to array
            # tryme=np.asarray(tryme)
            # tryme2=np.asarray(tryme2)
            # tryme3=np.asarray(tryme3)
            # tryme4=np.asarray(tryme4)
            # tryme5=np.asarray(tryme5)
            # tryme6=np.asarray(tryme6)
            # tryme7=np.asarray(tryme7)
            # tryme8=np.asarray(tryme8)        

    # fill array position
#             self.xc[0:ind,i]=np.transpose(tryme)
#             self.yc[0:ind,i]=np.transpose(tryme2)
#             self.zc[0:ind,i]=np.transpose(tryme3)

# # Fill array forces
#             self.Fcx[0:ind,i]=np.transpose(tryme4)
#             self.Fcy[0:ind,i]=np.transpose(tryme5)
#             self.Fcz[0:ind,i]=np.transpose(tryme6)    
            
#             self.AN[0:ind,i]=np.transpose(tryme7)
#             self.BN[0:ind,i]=np.transpose(tryme8)            
        # return data robots
        (self.xb,self.yb,self.zb)=self.bots.return_position_data()
        (self.Faxb,self.Fayb,self.Fazb)=self.bots.return_force_data()
        (self.xvb,self.yvb,self.zvb)=self.bots.return_velocity_data()
        
        # spring data
        (self.Spring_force,self.Spring_length)=self.bots.return_spring_data()    
        
        # control data
        if self.controller is not None:
            (self.Faxc,self.Fayc,self.Fazc)=self.controller.return_force_data()
            if self.controller.control_type=="grab_drag" or  self.controller.control_type=="grab_drag_A" :
                (self.Faxpf,self.Faypf,self.Fazpf,self.Faxpp,self.Faypp,self.Fazpp)=self.controller.return_pppf_force_data()
                
        # Interior
        if self.interior is not None:
        # Return Particle data
            (self.xp,self.yp,self.zp)=self.interior.return_position_data()
            (self.Faxp,self.Fayp,self.Fazp)=self.interior.return_force_data()
            (self.xvp,self.yvp,self.zvp)=self.interior.return_velocity_data()  
            
        if self.ball is not None:
            (self.bx,self.bz)=self.ball.return_position_data()
            (self.bFx,self.bFz)=self.ball.return_force_data()
            (self.bvx,self.bvz)=self.ball.return_velocity_data()
    
            #'F:/Science_robotics_journal/verification/
        self.results_dir = os.path.join('robot_data'+self.sim+'/')     

        if not os.path.isdir(self.results_dir):
            os.makedirs(self.results_dir)
#[position data]            
        if self.save_data[0]==True:        
        
            self.file_name0=self.results_dir+'/bot_position.csv'

            with open(self.file_name0, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xb.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yb.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zb.items():
                    w.writerow([key, *val])     
                    
#[bot velocity]                    
        if self.save_data[1]==True:        
        
            self.file_name1=self.results_dir+'/bot_velocity.csv'

            with open(self.file_name1, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvb.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yvb.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zvb.items():
                    w.writerow([key, *val])      
                    
#In[Forces]
        if self.save_data[2]==True:
         
            self.file_name2=self.results_dir+'/bot_TotalForces.csv'

            with open(self.file_name2, 'w', newline='') as fout:
                w = csv.writer(fout)
                    # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.Faxb.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.Fayb.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.Fazb.items():
                    w.writerow([key, *val])
                
# [Controller force]
        if self.save_data[3]==True:
            
            self.file_name3=self.results_dir+'/Force_controller.csv'

            with open(self.file_name3, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.Faxc.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.Fayc.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.Fazc.items():
                    w.writerow([key, *val])
            if self.controller.control_type=="grab_drag" or self.controller.control_type=="grab_drag_A" :        
                self.file_name4=self.results_dir+'/PP_controller.csv'        
                with open(self.file_name4, 'w', newline='') as fout:
                    w = csv.writer(fout)
                    # write time to csv file
                    for key, val in self.time.items():
                        w.writerow([key,*val])
                
                    # write x position to csv file
                    for key, val in self.Faxpp.items():
                        w.writerow([key, *val])
                
                    #write y position to csv file    
                    for key, val in self.Faypp.items():
                        w.writerow([key, *val]) 
                
                    # write z position to csv file     
                    for key, val in self.Fazpp.items():
                        w.writerow([key, *val])            

                self.file_name5=self.results_dir+'/Pf_controller.csv'        
                with open(self.file_name5, 'w', newline='') as fout:
                    w = csv.writer(fout)
                    # write time to csv file
                    for key, val in self.time.items():
                        w.writerow([key,*val])
                
                    # write x position to csv file
                    for key, val in self.Faxpf.items():
                        w.writerow([key, *val])
                
                    #write y position to csv file    
                    for key, val in self.Faypf.items():
                        w.writerow([key, *val]) 
                
                    # write z position to csv file     
                    for key, val in self.Fazpf.items():
                        w.writerow([key, *val])

            
# [Spring force]
        if self.save_data[4]==True and self.bots.mem!=3:
            
            self.file_name5=self.results_dir+'/Spring_properties.csv' 
       
            with open(self.file_name5, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                    
                for key, val in self.Spring_force.items():
                    w.writerow([key,*val])   
                for key, val in self.Spring_length.items():
                    w.writerow([key,*val])                       
#[Contact points]
        if self.save_data[5]==True:
            # Contact A
            
            # contact points x
            self.file_name6=self.results_dir+'/x contact points.csv' 
            savetxt(self.file_name6,self.xc, delimiter=',')
        
            # contact points y
            self.file_name7=self.results_dir+'/y contact points.csv' 
            savetxt(self.file_name7,self.yc, delimiter=',')
        
            # contact points z
            self.file_name8=self.results_dir+'/z contact points.csv' 
            savetxt(self.file_name8,self.zc, delimiter=',')
        
            # contact force x
            self.file_name9=self.results_dir+'/x contact force.csv' 
     
            savetxt(self.file_name9,self.Fcx, delimiter=',')
        
            # contact force y
            self.file_name10=self.results_dir+'/y contact force.csv' 
            savetxt(self.file_name10,self.Fcy, delimiter=',')
        
            # contact force z
            self.file_name11=self.results_dir+'/z contact force.csv' 
            savetxt(self.file_name11,self.Fcz, delimiter=',')
        
        # number of contacts
            self.file_name12=self.results_dir+'/nc.csv' 
            savetxt(self.file_name12,self.nc, delimiter=',')
            
            self.file_name13=self.results_dir+'/bodyA.csv' 
            savetxt(self.file_name13,self.AN, delimiter=',')
            
            self.file_name14=self.results_dir+'/bodyB.csv' 
            savetxt(self.file_name14,self.BN, delimiter=',')

        if self.save_data[6]==True:
            self.file_name15=self.results_dir+'/particle_position.csv'

            with open(self.file_name15, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xp.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yp.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zp.items():
                    w.writerow([key, *val])  
                    
#[Particle Velocity]                    
        if self.save_data[7]==True:
            self.file_name16=self.results_dir+'/particle_velocity.csv'

            with open(self.file_name16, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvp.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yvp.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zvp.items():
                    w.writerow([key, *val])
                    
        if self.save_data[8]==True:
            self.file_name17=self.results_dir+'/particle_forces.csv'

            with open(self.file_name17, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvp.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yvp.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zvp.items():
                    w.writerow([key, *val])                    
                    

        
        if self.save_data[9]==True:  
            self.file_name18=self.results_dir+'/ballx.csv' 
            savetxt(self.file_name18,self.bx, delimiter=',')
        
            self.file_name19=self.results_dir+'/ballz.csv' 
            savetxt(self.file_name19,self.bz, delimiter=',')
        
            self.file_name20=self.results_dir+'/ballvx.csv' 
            savetxt(self.file_name20,self.bvx, delimiter=',')
            
            self.file_name21=self.results_dir+'/ballvz.csv' 
            savetxt(self.file_name21,self.bvz, delimiter=',')
        
            self.file_name22=self.results_dir+'/ballFx.csv' 
            savetxt(self.file_name22,self.bFx, delimiter=',')

            self.file_name23=self.results_dir+'/ballFz.csv' 
            savetxt(self.file_name23,self.bFz, delimiter=',')
            
            self.file_name24=self.results_dir+'/ballvx.csv' 
            savetxt(self.file_name24,self.bvx, delimiter=',')

            self.file_name25=self.results_dir+'/ballvz.csv' 
            savetxt(self.file_name25,self.bvz, delimiter=',') 
        
            self.file_name26=self.results_dir+'/FBX.csv' 
            savetxt(self.file_name26,self.FBX, delimiter=',')

            self.file_name27=self.results_dir+'/FBZ.csv' 
            savetxt(self.file_name27,self.FBZ, delimiter=',') 
            
                 
        #if self.save_data[10]==True:
            #self.file_name28=self.results_dir+'/pathx.csv'
            #savetxt(self.file_name28,self.controller.paths.ax, delimiter=',')
            
            #self.file_name29=self.results_dir+'/pathz.csv'
            #savetxt(self.file_name29,self.controller.paths.ay, delimiter=',')
            
        if self.save_data[11]==True:
            self.file_name30=self.results_dir+'/desired_shapex.csv'
            savetxt(self.file_name30,self.shape.x, delimiter=',')
            self.file_name31=self.results_dir+'/desired_shapey.csv'
            savetxt(self.file_name31,self.shape.y, delimiter=',')
            
        if self.save_data[12]==True:
            self.file_name32=self.results_dir+'/error.csv'
            savetxt(self.file_name32,self.controller.E, delimiter=',')
            
#[save Variables]                    
    def save_variables(self):
        self.file_name33=self.results_dir+'/variables.csv'
        with open(self.file_name33, 'w', newline='') as fout:
            w = csv.writer(fout)
            # sim 
            w.writerow(['sim',self.sim])
            # control type
            w.writerow(['control_type',self.control_type])
            # number of bots
            w.writerow([' number of bots', self.nb])
            # geometry of bots
            w.writerow(['geometry of bot',self.bots.geom])
            # radius of bots
            w.writerow(['radius of bots',self.bots.diameter])
            # starting radius
            w.writerow(['starting radius', self.bots.R])
            # robot unit mass
            w.writerow(['robot mass', self.mr])           
            # ring rumber
            w.writerow(['ring numbers', self.interior.n])
            # number of interiors
            w.writerow(['number of interior', self.interior.ni])
            # particles mass
            w.writerow(['particle mass', self.mp])
            #         self.skind = 0.02   # diameter of cylinders for skin particles

            if self.bots.mem==3:
                w.writerow(['km',self.bots.km])
                w.writerow(['bm',self.bots.bm])
                w.writerow(['rationM',self.bots.ratioM])
                w.writerow(['skinrho',self.bots.skinrho]) 
                
            #w.writerow(['net_mass',self.mp*self.interior.ni+self.mr*np.sum(self.bots.active)])
            if self.control_type=='grab_drag' or self.control_type=='grab_drag_2' or self.control_type=='grab_drag_A' or self.control_type=='shape_form' or self.control_type=="pot_field_grab"or self.control_type=="pot_field_grab_2" :
                w.writerow(['alpha', self.controller.alpha])
                w.writerow(['beta', self.controller.beta])
            # simulation end
            w.writerow(['tend', self.tend])
            # friction
            w.writerow(['mu_f', self.mu_f])
            # damping 
            w.writerow(['mu_b', self.mu_b])
            # rolling 
            w.writerow(['mu_r', self.mu_r])
            # spinning
            w.writerow(['mu_s', self.mu_s])
            w.writerow(['C', self.C])
            w.writerow(['Ct', self.Ct])
            w.writerow(['Cr', self.Cr])
            w.writerow(['Cs', self.Cs])
            if self.control_type=='shape_form':
                w.writerow(['shape',self.shape.shape])
  
class import_data:
    def __init__(self):
        self.path="C:/Users/dmulr/OneDrive/Documents/soro_chrono/python/Pychrono/Strings/Grabbing/Grab_sim/experiments/2020-08-26 15h50/"
        self.name="data.csv"
        self.nb=12
        self.f=5
        self.filename=self.path+self.name
        self.data0=np.genfromtxt(self.filename,delimiter=',')
        (m,n)=np.shape(self.data0)
        self.data0=self.data0[1:m,:]
        self.data0=self.data0[:,3:n]
        
        self.TX=0.01*self.data0[:,(self.nb)*3]
        self.TY=0.01*self.data0[:,((self.nb)*3)+1]
        self.X=[]
        self.Y=[]
        for i in range(self.nb):
            self.X.append(0.01*self.data0[0,3*i])
            self.Y.append(0.01*self.data0[0,3*i+1])
            
        self.xc=np.sum(self.X)/self.nb
        self.yc=np.sum(self.Y)/self.nb    

        self.t=self.data0[:,-1]

        
        self.time=np.zeros(len(self.t))
        for i in range(len(self.time)):
            self.time[i]=i/self.f
        
    def return_target(self):
        return(self.TX,self.TY,self.time,self.X,self.Y)
        
        