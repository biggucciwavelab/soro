# -*- coding: utf-8 -*-
"""
Created on Fri Oct 16 16:03:14 2020

@author: dmulr
"""
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
from numpy import savetxt
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import csv

# In[Robot object]
class robot:
    def __init__(self,nb,radius,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,R,geom,xc,zc,skind,ratioM):
        # number of interior
        self.nb=nb 
        # starting positions x and z
        self.xc=xc ; self.zc=zc
        # diameter
        self.radius=radius
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
        # total force
        self.Ftxb={}; self.Ftyb={}; self.Ftzb={}
        # empty object arrays
        self.my_system=my_system
        self.bots=[]; self.Springs=[]
        self.obj=obj; self.force=[]
        # is object fixed
        self.fixed=fixed
        # Geometry of robots
        self.geom=geom
        '''
        # geometry of robots
        square: robot is cube  
        cylinder: robot is cylinder
        sphere : make them spheres
        '''


        # spring rleated things
        self.km=k; self.bm=1
        self.type_spring=type_spring
        self.p1=0; self.p2=self.radius/2
        self.p3=0; self.p4=-self.radius/2
        self.h=0; 
        self.mem=3 # membrane mode
        self.skinrho=1000
        self.skind = skind   # diameter of cylinders for skin particles
        self.ratioM = ratioM   # ratio of membrane skin particles to big bots
        skinM=[]        # empty matrix of membrane skin cylinders        
        # Colors
        col_y = chrono.ChColorAsset(); col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        col_b = chrono.ChColorAsset(); col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        col_g = chrono.ChColorAsset(); col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        col_p = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        
        # data arrays of robots
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
        
            # postion 
            theta=i*2*np.pi/self.nb
            x=self.R*np.cos(theta)+self.xc
            y=.5*height
            z=self.R*np.sin(theta)+self.zc

            # create body
            #bot = chrono.ChBody()
            if self.geom=="cylinder":
                bot = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rowr,True,True)
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
                # x forces
                myforcex = chrono.ChForce()
                bot.AddForce(myforcex)
                myforcex.SetMode(chrono.ChForce.FORCE)
                myforcex.SetDir(chrono.VECT_X)
                self.force.append(myforcex)
                # y forces    
                myforcey = chrono.ChForce()
                bot.AddForce(myforcey)
                myforcey.SetMode(chrono.ChForce.FORCE)
                myforcey.SetDir(chrono.VECT_Y)
                self.force.append(myforcey)
                # z forces            
                myforcez = chrono.ChForce()
                bot.AddForce(myforcez)
                myforcez.SetMode(chrono.ChForce.FORCE)
                myforcez.SetDir(chrono.VECT_Z)
                self.force.append(myforcez)
                bot.SetMaxSpeed(2)
                bot.SetLimitSpeed(True)
            # square
            if self.geom=="square":
                bot = chrono.ChBodyEasyBox(2*self.radius,self.height,2*self.radius,self.rowr,True,True)
                bot.SetPos(chrono.ChVectorD(x,y,z))
                bot.SetMaterialSurface(self.material)
                bot.SetName('bot'+str(i))
                bot.SetId(i)
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
            # Sphere
            if self.geom=="sphere":
                bot = chrono.ChBodyEasySphere(self.diameter,self.rowr,True,True)  # contact material)
                bot.SetPos(chrono.ChVectorD(x,y,z))
                bot.SetMaterialSurface(self.material)
                bot.SetName('bot')
                bot.SetId(i)
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
            bot.AddAsset(col_b)
            # zeroth robot
            if i==0:   
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
            
            # if no springs 
            if self.type_spring=="None":         
                print('None')
                
            # variable force springs 
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
                    
            # constant force spring 
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
            
        
            # membrane particles
            if self.mem==3 and self.type_spring!="None":
                b_ang=2*np.pi/self.nb                   # angle between centers of bots
                o_ang=np.arctan(self.radius/self.R)   # angle offset for radius of bot
                p_ang=np.arctan(self.skind/self.R)           # angle offset for radius of skin particle
                
                # Between this bot and last bot
                if i>=1 and i<self.nb:
                    for j in range(1,self.ratioM+1,1):
                        # Initial postion of each particle
                        theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ratioM) + p_ang
                        x=self.R*np.cos(theta)+self.xc
                        y=.52*self.height
                        z=self.R*np.sin(theta)+self.zc

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
                    
    # return system
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
    def __init__(self,nb,radius,rowp,height,my_system,obj,body_floor,material,fixed,mode,R,xc,zc,n):
        self.xc=xc
        self.zc=zc
        # robots diameter
        self.radius=radius
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
        # position
        self.xp={}; self.yp={}; self.zp={}
        # velocity
        self.xvp={}; self.yvp={}; self.zvp={}
        # forces
        self.Ftxp={}; self.Ftyp={}; self.Ftzp={}
        self.mode=mode
        # no interiors
        if self.mode=='empty':
            self.ni=0
            self.n=np.array([0])
        # not max interiors
        if self.mode=="nmax":
            (self.n)=n
            (self.ni)=np.sum(self.n)
        # max non homo interios
        if self.mode=="nonhnmax":
            self.n=np.array([6,3])
            (self.ni)=np.sum(self.n)
        
        # diamters are the same size
        if self.mode=="nmax":  
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
                    # temporary radius
                    R2=self.radius*self.n[i]/(np.pi)
                    # position
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xc
                    y=.5*self.height
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zc
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rowp,True,True)
                    #gran = chrono.ChBodyEasySphere(self.radius,self.rowp,True,True)
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.material)
                    gran.SetName('gran')
                    gran.SetId(i)
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
                    # add to system
                    self.my_system.Add(gran)
                    self.obj.append(gran)
                    self.particles.append(gran)
                    
        # if diamters are not the same size            
        if self.mode=="nonhnmax": 
            count=0
            for i in range(self.n.size):
                print(i)
                if i%2==0:
                    self.radius2=self.radius*(2**.5)
                else:
                    self.radius2=self.radius
                # empty arrays of variables
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
                    R2=self.radius2*self.n[i]/(np.pi) # raidus of ring 
                    # x,y,z positions
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xc
                    y=.5*self.height
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zc
                    # create body
                    gran = chrono.ChBody()
                    gran = chrono.ChBodyEasyCylinder(self.radius2, self.height,self.rowp,True,True)
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.material)
                    gran.SetName('gran'+str(count))
                    gran.SetId(i)
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    # add color
                    col_r = chrono.ChColorAsset()
                    col_r.SetColor(chrono.ChColor(1, 0, 0))
                    gran.AddAsset(col_r)
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
        
    # return system
    def return_system(self):
        return(self.my_system,self.particles,self.obj)
    
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
    def __init__(self,control_type,my_system,body_floor,obj,material,height=None,R=None,rho=None,zball=None,xball=None):
        self.control_type=control_type
        self.my_system=my_system
        self.body_floor=body_floor
        self.obj=obj
        self.material=material
        self.balls=[]
        self.ballx=[]; self.ballz=[]
        self.Fballx=0; self.Fballz=0
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
        self.height=height
        self.radius=R
        self.rho=rho
        self.x=xball
        self.y=0
        self.z=zball
        if self.geom=="circle":
        #Create ball
            ball = chrono.ChBody()
            ball = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rho,True,True)
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
    
# In[controller]
class Controls:
    # **We should really put variable=None for all the optional arguments so we don't need to init everything every single time**
    def __init__(self,forces,bots,interiors,Springs,my_system,nb,control_type,balls,tstep,my_rep,w,tn,tpull,*args,**kwargs):
        # objects
        #### Initialize some parameters for all cases of the controller
        self.forces=forces          # array of forces 
        self.bots=bots              # array of all our boundary robots
        self.interiors=interiors    # array of all the interior particles
        self.Springs=Springs        # array of all the spring objects
        self.my_system=my_system    # the simulation system
        self.balls=balls            # balls if they are there 
        self.tstep=tstep           # time step of simulation
        self.control_type=control_type # controller type                  
        self.nb=nb                  # number of robots
        self.w=w                    # frequency
        self.tn=tn                  # time interval
        self.T=0                    # internal time 
        self.time=0
        self.tpull=tpull
        # save controller forces and error
        self.Faxc={};self.Fayc={};self.Fazc={}; self.E=[]
        
        # temporary arrays
        self.fxt=[]; self.fyt=[]; self.fzt=[]
        # contact class 
        self.my_rep=my_rep
        self.count=0
        self.xc=0
        self.zc=0
        self.xb=0
        self.zb=0
        # create controller forces applied 
        for i in range(self.nb):
            self.Faxc["botx{0}".format(i)]=[]
            self.Fayc["boty{0}".format(i)]=[]
            self.Fazc["botz{0}".format(i)]=[] 
##############################################################################
        if self.control_type=='move_right':
            self.FX=0
            self.FZ=0               
##############################################################################   
        #### pot_field_grab parameters        
        if self.control_type=='pot_field_grab':
            self.phi=args[0][0]
            (self.f,self.fnx,self.fny)=self.phi.update_field_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.b=args[0][3]
            self.Rd=args[0][4]                      
            self.FX=0
            self.FZ=0
            self.count=0
##############################################################################            
        #### grab_and_drag parameters       
        if self.control_type=='grab_drag':
            self.phi=args[0][0]
            (self.f,self.fnx,self.fny)=self.phi.update_field_gradient()
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.b=args[0][3]
            self.Rd=args[0][4]
            self.num=args[0][5]
            self.cond=False
            self.FX=0
            self.FZ=0
            self.count=0
            self.cn=0            
##############################################################################            
        #### shape form parameters
        if self.control_type=='shape_form':
            self.phi=args[0][0]
            self.f=self.phi.f            
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.b=args[0][3]
            self.Rd=args[0][4]                   
            self.FX=0
            self.FZ=0
            self.count=0
            self.E=[]
        #### moving shape form parameters
        if self.control_type=='moving_shape_form':
            self.phi=args[0][0]
            self.f=self.phi.f
            self.fnx=self.phi.fnx
            self.fny=self.phi.fny            
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.b=args[0][3]
            self.Rd=args[0][4]                   
            self.FX=0
            self.FZ=0
            self.count=0
            self.c=.01
        #### tunnelng parameters    
        if self.control_type=="tunnel":
            self.phi=args[0][0]     
            self.fny=self.phi.fny
            self.fnx=self.phi.fnx
            self.alpha=args[0][1]
            self.beta=args[0][2]
            self.b=args[0][3]
            self.FX=0
            self.FZ=0            
##############################################################################
    # run controller
    def run_controller(self):
        # Pot_field_grab: grab a fixed object
        if self.control_type=="pot_field_grab":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.grab_controller()
            self.apply_force(self.FX,self.FZ)
        # grabbing and dragging a non fixed object     
        if self.control_type=="grab_drag":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.grab_drag_controller()
            self.apply_force(self.FX,self.FZ)            
        # run shape formation 
        if self.control_type=="shape_form":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.shape_controller()
            self.apply_force(self.FX,self.FZ)
        # run moving shape formation 
        if self.control_type=="moving_shape_form":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.moving_shape_controller()
            self.apply_force(self.FX,self.FZ) 
        # run tunneling     
        if self.control_type=="tunnel":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.tunnel_controller()
            self.apply_force(self.FX,self.FZ)     
        if self.control_type=='move_right':
            (self.FX,self.FZ)=self.move_right_controller()
            self.apply_force(self.FX,self.FZ)
            
# In[Jumbos stuff]


    



            
            
##############################################################################            
    def grab_controller(self):
        self.phi.py=100*self.balls.balls[0].GetPos().x
        self.phi.px=100*self.balls.balls[0].GetPos().z
        (self.f,self.fnx,self.fny)=self.phi.update_field_gradient()
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.fny([100*self.xb[i],100*self.zb[i]])
            Fz=self.fnx([100*self.xb[i],100*self.zb[i]])
            mag=np.sqrt(Fx**2+Fz**2)
            FXX=Fx/mag
            FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.xbv[i]
            fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
            
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))

##################################################################   
    # def shape_controller(self):
    #     FX=[]
    #     FZ=[]
    #     for i in range(self.nb):
    #         Fx=self.fny([100*self.xb[i],100*self.zb[i]])
    #         Fz=self.fnx([100*self.xb[i],100*self.zb[i]])
    #         mag=np.sqrt(Fx**2+Fz**2)
    #         if mag[0]==0:
    #             FXX=np.array([0])
    #             FZZ=np.array([0])
    #         else:
    #             FXX=Fx/mag
    #             FZZ=Fz/mag
    #         fx=-self.alpha*FXX-self.beta*self.xbv[i]
    #         fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
            
    #         FX.append(fx[0])
    #         FZ.append(fz[0])
    #     return(np.asarray(FX),np.asarray(FZ))
    
    
    def shape_controller(self):
        FX=[]
        FZ=[]
        for i in range(self.nb):

            Fz=self.phi.fy2(self.xb[i],self.zb[i])
            Fx=self.phi.fx2(self.xb[i],self.zb[i]) 
            
            mag=np.sqrt(Fx**2+Fz**2)
            if mag==0:
                FXX=np.array([0])
                FZZ=np.array([0])
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.xbv[i]
            fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
            
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))
                            
        
##############################################################################                
    def apply_force(self,FX,FZ):  
        self.T=self.tstep+self.T
        if self.T>0 and self.T<=self.tn:
            self.FX=FX
            self.FZ=FZ
        else:
            self.FX=np.zeros(len(FX))
            self.FZ=np.zeros(len(FX))
            
        if self.T>(1/self.w):
            self.T=0
            print('reset',np.round(self.my_system.GetChTime(),2))
        for i in range(self.nb):
            self.fxt.append(self.FX[i])
            self.fyt.append(0)
            self.fzt.append(self.FZ[i])
            self.forces[3*i].SetMforce(float(self.FX[i]))
            self.forces[3*i].SetDir(chrono.VECT_X)
            self.forces[3*i+2].SetMforce(float(self.FZ[i]))
            self.forces[3*i+2].SetDir(chrono.VECT_Z)
            
# get current position         
    def get_position(self):
        self.xb=[]        
        self.zb=[]
        for i in range(len(self.bots)):
            self.xb.append(self.bots[i].GetPos().x)
            self.zb.append(self.bots[i].GetPos().z)
        return(self.xb,self.zb)

    def get_centroid(self):
        self.xc=np.sum(self.xb)/self.nb
        self.zc=np.sum(self.zb)/self.nb
        return(self.xc,self.zc)            
# Get ball Position
    def get_ball_position(self):
        self.ballx=self.balls[0].GetPos().x
        self.ballz=self.balls[0].GetPos().z
        return(self.ballx,self.ballz)
    
    def get_error(self):
        if self.control_type=='shae_form':
            e=[]
            for i in range(self.nb):
                e.append(self.f([100*self.xb[i],100*self.zb[i]]))
            self.E.append(np.sum(e))
        else:
            pass

        
            
 # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.zbv=[]
        for i in range(len(self.bots)):
            self.xbv.append(self.bots[i].GetPos_dt().x)
            self.zbv.append(self.bots[i].GetPos_dt().z)
        return(self.xbv,self.zbv)
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

    def clear_temp_forces(self):
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        
# return force data for controllers     
    def return_force_data(self):
        return(self.Faxc,self.Fayc,self.Fazc)         
        
        
# In[Potential Fields]
class potential_fields:
    def __init__(self,field,px,py,b,region_shape=None,R=None,r1=None,r2=None):
        # field type and shape of field if needed
        self.field=field
        self.region_shape=region_shape

    #### super ellipse ####
        self.xo=0
        self.yo=0
        
        self.m=4
        self.n1=12
        self.n2=15
        self.n3=15
        self.C=self.m/4
        self.phi=0
        self.a=15
        self.b=15

        
    


    # arctan2
    def T(self,x,y):
        return(np.arctan2(y,x))
    
    # derivativ in respect to x
    def Tx(self,x,y):
        return(-1* y / (x**2 + y**2))
    
    # derivative in respect to y
    def Ty(self,x,y):
        return( x / (x**2 + y**2))
    
    # Radius function
    def R(self,x,y):
        return((abs(np.cos(self.C*self.T(x,y)))**self.n2 + abs(np.sin(self.C*self.T(x,y)))**self.n3)**(-1/self.n1))
    
    # Terms function to help simplify the derivatives of the radius    
    def Term(self,x,y):
        self.term1 = self.C*self.n3*np.cos(self.C*self.Tx(x,y))*abs(np.sin(self.C*self.T(x,y))/self.b)** self.n3 * np.sign(np.sin(self.C*self.T(x,y))/self.b) / self.b*np.abs(np.sin(self.C*self.T(x,y))/self.b)
        self.term2 = self.C*self.n2*np.sin(self.C*self.Tx(x,y))*abs(np.cos(self.C*self.T(x,y))/self.a)** self.n2 * np.sign(np.cos(self.C*self.T(x,y))/self.a) / self.a*np.abs(np.cos(self.C*self.T(x,y))/self.a)
        self.term3 = (abs(np.cos(self.C*self.T(x,y))/self.a)**self.n2 + abs(np.sin(self.C*self.T(x,y))/self.b)) ** (-1/self.n1)
        self.term4 = self.n1*(abs(np.cos(self.C*self.T(x,y))/self.a) ** self.n2 + abs(np.sin(self.C*self.T(x,y))/self.b) ** self.n3 )
        return(self.term1,self.term2,self.term3,self.term4)
    
    def Term2(self,x,y):
        self.term5=2*((x-self.xo)*np.sin(self.phi)+(y-self.yo)*np.cos(self.phi))**2 / self.R(x,y)**3
        self.term6=2*((x-self.xo)*np.sin(self.phi)+(y-self.yo)*np.cos(self.phi))*np.cos(self.phi) / self.R(x,y)**2
        self.term7=2*((x-self.xo)*np.cos(self.phi)-(y-self.yo)*np.sin(self.phi))**2 /self.R(x,y)**3
        self.term8=2*((x-self.xo)*np.cos(self.phi)+(y-self.yo)*np.sin(self.phi))*np.sin(self.phi) / self.R(x,y)**2
        self.term9= 2*((x-self.xo)*np.sin(self.phi)+(y-self.yo)*np.cos(self.phi))*np.sin(self.phi) / self.R(x,y)**2
        self.term10=2*((x-self.xo)*np.cos(self.phi)+(y-self.yo)*np.sin(self.phi))*np.cos(self.phi) / self.R(x,y)**2
        return(self.term5,self.term6,self.term7,self.term8,self.term9,self.term10)
    

    def Rx(self,x,y):
        (self.term1,self.term2,self.term3,self.term4)=self.Term(x,y)
        return(-((self.term1 * self.Tx(x,y)  - self.term2 * self.Tx(x,y)) * (self.term3)) / self.term4)

    def Ry(self,x,y):
        (self.term1,self.term2,self.term3,self.term4)=self.Term(x,y)
        return(-((self.term1 * self.Ty(x,y)  - self.term2 * self.Ty(x,y)) * (self.term3)) / self.term4)    
    
    # d term
    def d(self,x,y):
        return((((x-self.xo)*np.cos(self.phi)-(y-self.yo)*np.sin(self.phi))**2 / self.R(x,y)**2)+(((x-self.xo)*np.sin(self.phi)+(y-self.yo)*np.cos(self.phi))**2 /self.R(x,y)**2))
    
    # derivative of d in respect to x
    def dx(self,x,y):
        (self.term5,self.term6,self.term7,self.term8,self.term9,self.term10)=self.Term2(x,y)
        return(-1*self.term5*self.Rx(x,y) + self.term9 - self.term7*self.Rx(x,y) + self.term10)

    # derivative of d in respect to y   
    def dy(self,x,y):
        (self.term5,self.term6,self.term7,self.term8,self.term9,self.term10)=self.Term2(x,y)
        return(-1* self.term5*self.Ry(x,y) + self.term6 - self.term7*self.Ry(x,y) - self.term8)
    
    
    # potnetial field function    
    def f(self,x,y):
        return(np.nan_to_num(self.d(x,y)**2 *np.log(self.d(x,y))))
    
    # derivative of field in respect to x
    def fx(self,x,y):
        return(2*self.d(x,y)*np.log(self.d(x,y))*self.dx(x,y) + self.dx(x,y) *self.d(x,y))
    
    # derivative of the field in respect to y
    def fy(self,x,y):
        return(2*self.d(x,y)*np.log(self.d(x,y))*self.dx(x,y) + self.dy(x,y) *self.d(x,y))
    
    # derivative of the field squared in respect to y
    def fy2(self,x,y):
        return(np.nan_to_num(-1*self.fy(x,y)*self.f(x,y)))
    
    # derivative of the field squared in respect to x
    def fx2(self,x,y):
        return(np.nan_to_num(-1*self.fx(x,y)*self.f(x,y)))
    
      

# In[Simulate]
class simulate:
    def __init__(self,my_system,bots,particles,balls,controls,Springs,obj,my_rep,sim,tstep,tend,visual,data_path):   
        self.visual=visual # visualization method
        self.my_system=my_system # my system
        self.sim=sim # sim number 
        self.tstep=tstep # step size
        self.tend=tend # tend of simulation
        self.data_path=data_path # data path
        self.particles=particles # interior particles
        self.bots=bots # bots
        self.obj=obj # object array
        self.balls= balls # ball array 
        self.Springs=Springs # spring array 
        self.controls=controls # controls
        self.time=[] # empty time array 
        self.timec=[] # empty time array for contacts
        self.my_rep=my_rep # contact collectors
        # empty positions forces and number of contact arrays
        self.nc=[]; self.cx=[]; self.cy=[]; self.cz=[]
        self.Fxct=[]; self.Fyct=[]; self.Fzct=[]
        self.bodiesA=[]; self.bodiesB=[]
        self.Trip=False
        self.myapplication=[]
        self.count=0
    # simulate the robot
    def simulate(self):
        #### Irrrlecnt
        #  Create an Irrlicht application to visualize the system
        if self.visual=="irr":
            self.myapplication = chronoirr.ChIrrApp(self.my_system, self.sim , chronoirr.dimension2du(1600,900))
            self.myapplication.AddTypicalSky()
            self.myapplication.AddTypicalLogo()
            self.myapplication.AddTypicalCamera(chronoirr.vector3df(0,30,0),chronoirr.vector3df(0,0,0))
            self.myapplication.SetSymbolscale(.002)
            self.myapplication.SetShowInfos(True)
            self.myapplication.SetContactsDrawMode(2)
            self.myapplication.SetPaused(self.Trip)
            self.myapplication.AddTypicalLights()
            self.myapplication.DrawAll               
            self.myapplication.AssetBindAll()
            self.myapplication.AssetUpdateAll()
            self.myapplication.AddShadowAll()
            self.count=0
            self.myapplication.SetTimestep(self.tstep)
            self.myapplication.SetTryRealtime(False)
            ##### Run the sim
            while(self.myapplication.GetDevice().run()):
                #self.my_rep.ResetList()
                self.myapplication.BeginScene()
                self.myapplication.DrawAll()
                self.myapplication.DoStep()
                self.controls.run_controller()
                self.controls.get_position()
                (xc,zc)=self.controls.get_centroid()
                print(np.round(xc,2),np.round(zc,2))
                #self.controls.clear_temp_forces()
                #print ('time=', self.my_system.GetChTime())
                
                #if self.count%10==0:
                    #self.time.append(self.my_system.GetChTime())
                    #self.controls.save_data_Forces()
                    #self.controls.clear_temp_forces()
                    #self.controls.get_error()
                    #self.bots.save_data_position()
                    #self.bots.save_data_Forces()
                    #self.bots.save_data_velocity()
                    #self.particles.save_data_position()
                    #self.particles.save_data_velocity()
                    #self.particles.save_data_Forces()
                    #self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
                    #crt_list = self.my_rep.GetList()
                    #self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
                #     if self.my_system.GetContactContainer().GetNcontacts()!=0:
                #         self.timec.append(self.my_system.GetChTime())
                #         self.cx.append(crt_list[0])
                #         self.cy.append(crt_list[1])
                #         self.cz.append(crt_list[2])
                #         self.Fxct.append(crt_list[3])
                #         self.Fyct.append(crt_list[4])
                #         self.Fzct.append(crt_list[5])
                #         self.bodiesA.append(crt_list[6])
                #         self.bodiesB.append(crt_list[7])
                # #save ball position if it exists
                # if self.balls!=None:
                #     if self.count%10==0:
                #         self.balls.save_data_position()
                #         self.balls.save_contact_force()
                #         self.balls.save_data_velocity()
                self.controls.clear_temp_forces()
                self.count=self.count+1
                self.myapplication.EndScene()
                # save data
                self.myapplication.SetVideoframeSave(True)
                self.myapplication.SetVideoframeSaveInterval(round(1/(self.tstep*100)))
                #zc=self.balls.balls[0].GetPos().z
                #vzc=self.balls.balls[0].GetPos_dt().z
                #print(zc,vzc)
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.tend :
                    self.myapplication.GetDevice().closeDevice()
        
        #### Pov ray
        if self.visual=="pov": 
            
            
            pov_exporter = postprocess.ChPovRay(self.my_system)

            # Sets some file names for in-out processes.
            pov_exporter.SetTemplateFile("F:/data/"+"_template_POV.pov")
            pov_exporter.SetOutputScriptFile("rendering_frames"+self.sim+".pov")
            if not os.path.exists("output"+self.sim):
                os.mkdir("output"+self.sim)
            if not os.path.exists("anim"+self.sim):
                os.mkdir("anim"+self.sim)
            pov_exporter.SetOutputDataFilebase("output"+self.sim+"/my_state")
            pov_exporter.SetPictureFilebase("anim"+self.sim+"/picture")
            pov_exporter.SetCamera(chrono.ChVectorD(0,30,0), chrono.ChVectorD(0,0,0,),90)
            pov_exporter.SetAmbientLight(chrono.ChColor(1,1,0.9))
            pov_exporter.SetLight(chrono.ChVectorD(0,20,0), chrono.ChColor(1.1,1.1,1.1), True)
            #pov_exporter.SetAmbientLight(chrono.ChColor(1,1,0.9))
            #pov_exporter.SetLight(chrono.ChVectorD(-2,2,-1), chrono.ChColor(0.9,0.9,1.1), True)
            #pov_exporter.SetAmbientLight(chrono.ChColor(2,2,2))                  
            pov_exporter.AddAll()
            pov_exporter.ExportScript()
            count=0
            ##### Run the sim 
            while (self.my_system.GetChTime() < self.tend) :                  
                #self.my_rep.ResetList()
                self.my_system.DoStepDynamics(self.tstep)
                # run the controllers if they exist
                self.controls.run_controller()
                self.controls.clear_temp_forces()
                #print ('time=', self.my_system.GetChTime())
                
                #if self.count%10==0:
                    #self.time.append(self.my_system.GetChTime())
                    #self.controls.save_data_Forces()
                    #self.controls.get_error()
                    #self.controls.clear_temp_forces()
                    #self.bots.save_data_position()
                    #self.bots.save_data_Forces()
                    #self.bots.save_data_velocity()
                    #self.particles.save_data_position()
                    #self.particles.save_data_velocity()
                    #self.particles.save_data_Forces()
                    #self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
                    #crt_list = self.my_rep.GetList()
                    #self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
                    #if self.my_system.GetContactContainer().GetNcontacts()!=0:
                        #self.timec.append(self.my_system.GetChTime())
                        #self.cx.append(crt_list[0])
                        #self.cy.append(crt_list[1])
                        #self.cz.append(crt_list[2])
                        #self.Fxct.append(crt_list[3])
                        #self.Fyct.append(crt_list[4])
                        #self.Fzct.append(crt_list[5])
                        #self.bodiesA.append(crt_list[6])
                        #self.bodiesB.append(crt_list[7])
                #save ball position if it exists
                #if self.balls!=None:
                #    if self.count%10==0:
                 #       self.balls.save_data_position()
                  #      self.balls.save_contact_force()
                  #      self.balls.save_data_velocity()
                #self.count=self.count+1
                #self.controls.clear_temp_forces()
                # contact data
                #self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
                #crt_list = self.my_rep.GetList()
                #self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
                #self.cx.append(crt_list[0])
                #self.cy.append(crt_list[1])
                #self.cz.append(crt_list[2])
                #self.Fxct.append(crt_list[3])
                #self.Fyct.append(crt_list[4])
                #self.Fzct.append(crt_list[5])
                #self.bodiesA.append(crt_list[6])
                #self.bodiesB.append(crt_list[7])
                print('time=', self.my_system.GetChTime())
                if count%12==0:
                    self.controls.get_position()
                    (xc,zc)=self.controls.get_centroid()
                    print(np.round(xc,2),np.round(zc,2))
                    pov_exporter.ExportData()  
                count=count+1
                
        return(self.bots,self.time,self.controls,self.cx,self.cy,self.cz,self.Fxct,self.Fyct,self.Fzct,self.nc,self.bodiesA,self.bodiesB)     




# In[report contact callback]
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


# In[report contact callback2]                
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

# In[Export data]
class export_data():
    def __init__(self,bots,interior,ball,phi,controller,tend,time,sim,nb,mr,mp,mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs,cx,cy,cz,Fxct,Fyct,Fzct,nc,bodiesA,bodiesB,pwm,w,tn,gapw,env_mode,save_data):
        # objects 
        # bots
        self.bots=bots
        # interiors
        if interior is not None:
            self.interior=interior
        else:
            self.interior=None
        # ball
        if ball is not None:
            self.ball=ball
        else:
            self.ball=None
        # controller
        if controller is not None:
            self.controller=controller
        else:
            self.controller=None
            
        # potential field
        if phi is not None:
            self.phi=phi
        else:
            self.phi=None
            
        # control type    
        self.control_type=self.controller.control_type
        # potential field type
        self.field_type=self.phi.field
        # end time
        self.tend=tend
        # time array
        self.time={'time': time}
        self.count=len(time)
        # simulation name
        self.sim=sim
        # number of robots
        self.nb=nb
        # mass of robots
        self.mr=mr
        # mass of particles
        self.mp=mp
        # friction properties 
        self.mu_f=mu_f ; self.mu_b=mu_b ; self.mu_r=mu_r ; self.mu_s=mu_s
        # compliance properties
        self.C=C ; self.Ct=Ct ; self.Cr=Cr ; self.Cs=Cs
        # contact locations
        self.cx=cx ; self.cy=cy ; self.cz=cz ; self.nc=np.asarray(nc)
        self.lengthm=np.amax(self.nc)
        #Create empty contact matrices
        self.xc=np.zeros((self.lengthm,self.count))
        self.yc=np.zeros((self.lengthm,self.count))
        self.zc=np.zeros((self.lengthm,self.count))
        
        # bodies that made contact
        self.bodiesA=bodiesA ; self.bodiesB=bodiesB
        self.BN=np.zeros((self.lengthm,self.count))
        self.AN=np.zeros((self.lengthm,self.count))
        # contact of forces
        self.Fxct=Fxct ; self.Fyct=Fyct ; self.Fzct=Fzct
        self.Fcx=np.zeros((self.lengthm,self.count))
        self.Fcy=np.zeros((self.lengthm,self.count))
        self.Fcz=np.zeros((self.lengthm,self.count))
        # range of time applied 
        self.tn=tn
        # freqency
        self.w=w
        # pwm 
        self.pwm=pwm
        # tunnel mode 
        self.gapw=gapw ; self.env_mode=env_mode
        
        # save data array
        self.save_data=save_data

        ###### Export variables ######
        self.results_dir = os.path.join('robot_data_'+self.sim+'/') 
        if not os.path.isdir(self.results_dir):
            os.makedirs(self.results_dir)
            
        self.file_name0=self.results_dir+'variables.csv'

        with open(self.file_name0, 'w', newline='') as fout:
            w = csv.writer(fout)
            # sim 
            w.writerow(['sim',self.sim])
            # control type
            w.writerow(['control_type',self.control_type])
            # number of bots
            w.writerow(['number of bots', self.nb])
            # geometry of bots
            w.writerow(['geometry of bot',self.bots.geom])
            # radius of bots
            w.writerow(['radius of bots',self.bots.radius])
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
            # granular mode
            w.writerow(['gran mode',self.interior.mode])
            # net mass
            w.writerow(['net_mass',self.mp*self.interior.ni+self.mr*self.nb])
            # frequency 
            w.writerow(['frequency', self.w])
            # pwm 
            w.writerow(['pwm',self.pwm])
            # time active
            w.writerow(['time active', self.tn])
            # simulation end
            w.writerow(['tend', self.tend])            
            if self.bots.mem==3:
                w.writerow(['km',self.bots.km])
                w.writerow(['bm',self.bots.bm])
                w.writerow(['rationM',self.bots.ratioM])
                w.writerow(['skinrho',self.bots.skinrho]) 
            # alpha 
            w.writerow(['alpha', self.controller.alpha])
            # beta
            w.writerow(['beta', self.controller.beta])    
            # field type
            w.writerow(['field type', self.phi.field])
            # field shape
            w.writerow(['region_shape', self.phi.region_shape])
            # inner radius
            w.writerow(['r1', self.phi.r1]) 
            # outer radius
            w.writerow(['r2', self.phi.r2])               
            # friction
            w.writerow(['mu_f', self.mu_f])
            # damping 
            w.writerow(['mu_b', self.mu_b])
            # rolling 
            w.writerow(['mu_r', self.mu_r])
            # spinning
            w.writerow(['mu_s', self.mu_s])
            # Cohesive stuff
            w.writerow(['C', self.C])
            w.writerow(['Ct', self.Ct])
            w.writerow(['Cr', self.Cr])
            w.writerow(['Cs', self.Cs])
            # tunnel mode 
            w.writerow(['env_mode',self.env_mode])
            w.writerow(['gapw',self.gapw])
            
            

            
        for i in range(self.count):
            ind=self.nc[i]
            tryme=self.cx[i]
            tryme2=self.cy[i]
            tryme3=self.cz[i]
            tryme4=self.Fxct[i]
            tryme5=self.Fyct[i]
            tryme6=self.Fzct[i]
            tryme7t=self.bodiesA[i]
            tryme8t=self.bodiesB[i]
            tryme7=np.zeros(len(tryme7t))     
            tryme8=np.zeros(len(tryme7t)) 
            
            for k in range(len(tryme7)):
                #print(tryme7t[k])
                if tryme7t[k]=='floor':
                    tryme7[k]=1
                if tryme7t[k]=='bot':
                    tryme7[k]=2
                if tryme7t[k]=='gran':
                    tryme7[k]=3
                if tryme7t[k]=='ball000':
                    tryme7[k]=4
                if tryme7t[k]=='skin':
                    tryme7[k]=5
                if tryme8t[k]=='floor':
                    tryme8[k]=1
                if tryme8t[k]=='bot':
                    tryme8[k]=2
                if tryme8t[k]=='gran':
                    tryme8[k]=3
                if tryme8t[k]=='ball000':
                    tryme8[k]=4
                if tryme8t[k]=='skin':
                    tryme8[k]=5                   
            # convert to array
            tryme=np.asarray(tryme)
            tryme2=np.asarray(tryme2)
            tryme3=np.asarray(tryme3)
            tryme4=np.asarray(tryme4)
            tryme5=np.asarray(tryme5)
            tryme6=np.asarray(tryme6)
            tryme7=np.asarray(tryme7)
            tryme8=np.asarray(tryme8)        

            #fill array position
            self.xc[0:ind,i]=np.transpose(tryme)
            self.yc[0:ind,i]=np.transpose(tryme2)
            self.zc[0:ind,i]=np.transpose(tryme3)

            # Fill array forces
            self.Fcx[0:ind,i]=np.transpose(tryme4)
            self.Fcy[0:ind,i]=np.transpose(tryme5)
            self.Fcz[0:ind,i]=np.transpose(tryme6)    
            # fill bodies that contacted
            self.AN[0:ind,i]=np.transpose(tryme7)
            self.BN[0:ind,i]=np.transpose(tryme8)         
            
        # return data robots
        (self.xb,self.yb,self.zb)=self.bots.return_position_data()
        (self.Faxb,self.Fayb,self.Fazb)=self.bots.return_force_data()
        (self.xvb,self.yvb,self.zvb)=self.bots.return_velocity_data()
        (self.Faxc,self.Fayc,self.Fazc)=self.controller.return_force_data()


        # Return Particle data
        if self.interior is not None:
            (self.xp,self.yp,self.zp)=self.interior.return_position_data()
            (self.Faxp,self.Fayp,self.Fazp)=self.interior.return_force_data()
            (self.xvp,self.yvp,self.zvp)=self.interior.return_velocity_data()  
        
        # ball properties    
        if self.ball is not None:
            (self.bx,self.bz)=self.ball.return_position_data()
            (self.bFx,self.bFz)=self.ball.return_force_data()
            (self.bvx,self.bvz)=self.ball.return_velocity_data()
    
    
        # position data           
        if self.save_data[0]==True:        
            self.file_name1=self.results_dir+'/bot_position.csv'

            with open(self.file_name1, 'w', newline='') as fout:
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
                    
        # bot velocity                    
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
                    
        # Forces
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
                
        # Controller force
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
            
        #[Contact points]
        if self.save_data[4]==True:
            # Contact A
            
            # contact points x
            self.file_name4=self.results_dir+'/x contact points.csv' 
            savetxt(self.file_name4,self.xc, delimiter=',')
        
            # contact points y
            self.file_name5=self.results_dir+'/y contact points.csv' 
            savetxt(self.file_name5,self.yc, delimiter=',')
        
            # contact points z
            self.file_name6=self.results_dir+'/z contact points.csv' 
            savetxt(self.file_name6,self.zc, delimiter=',')
        
            # contact force x
            self.file_name7=self.results_dir+'/x contact force.csv' 
            savetxt(self.file_name7,self.Fcx, delimiter=',')
        
            # contact force y
            self.file_name8=self.results_dir+'/y contact force.csv' 
            savetxt(self.file_name8,self.Fcy, delimiter=',')
        
            # contact force z
            self.file_name9=self.results_dir+'/z contact force.csv' 
            savetxt(self.file_name9,self.Fcz, delimiter=',')
        
            # number of contacts
            self.file_name10=self.results_dir+'/nc.csv' 
            savetxt(self.file_name10,self.nc, delimiter=',')
            
            self.file_name11=self.results_dir+'/bodyA.csv' 
            savetxt(self.file_name11,self.AN, delimiter=',')
            
            self.file_name12=self.results_dir+'/bodyB.csv' 
            savetxt(self.file_name12,self.BN, delimiter=',')
            
        # particle position
        if self.save_data[5]==True:
            self.file_name13=self.results_dir+'/particle_position.csv'
            with open(self.file_name13, 'w', newline='') as fout:
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
                    
        # Particle Velocity                    
        if self.save_data[6]==True:
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
                    
        if self.save_data[7]==True:
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
                
        # ball variables 
        if self.save_data[8]==True:
            # ball positions x
            self.file_name18=self.results_dir+'/ballx.csv' 
            savetxt(self.file_name18,self.bx, delimiter=',')
            # ball position z
            self.file_name19=self.results_dir+'/ballz.csv' 
            savetxt(self.file_name19,self.bz, delimiter=',')
            # ball velocity x
            self.file_name20=self.results_dir+'/ballvx.csv' 
            savetxt(self.file_name20,self.bvx, delimiter=',')
            # ball velocity z
            self.file_name21=self.results_dir+'/ballvz.csv' 
            savetxt(self.file_name21,self.bvz, delimiter=',')
            # ball force x
            self.file_name22=self.results_dir+'/ballFx.csv' 
            savetxt(self.file_name22,self.bFx, delimiter=',')
            # ball force z
            self.file_name23=self.results_dir+'/ballFz.csv' 
            savetxt(self.file_name23,self.bFz, delimiter=',')
            

            
        # error of robot
        if self.save_data[9]==True:
            self.file_name28=self.results_dir+'/error.csv'
            savetxt(self.file_name28,self.controller.E, delimiter=',')
            
        if self.save_data[10]==True:
            self.file_name24=self.results_dir+'/xd.csv' 
            savetxt(self.file_name24,self.phi.xd, delimiter=',')
            
            self.file_name25=self.results_dir+'/yd.csv' 
            savetxt(self.file_name25,self.phi.yd, delimiter=',')
            
            self.file_name26=self.results_dir+'/xd2.csv' 
            savetxt(self.file_name26,self.phi.xd2, delimiter=',')            
            
            self.file_name27=self.results_dir+'/yd2.csv' 
            savetxt(self.file_name27,self.phi.yd2, delimiter=',')            