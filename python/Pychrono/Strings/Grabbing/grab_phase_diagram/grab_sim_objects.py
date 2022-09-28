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
import timeit
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import scipy.constants 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm 
import matplotlib.pyplot as plt
from matplotlib import animation
import animatplot as amp
from matplotlib import colors as colors
import csv
from numpy import asarray
from numpy import savetxt
#[Robot object]
class robot:
    def __init__(self,nb,diameter,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,mag,R):
        self.diameter=diameter
        self.nb=nb
        self.rowr=rowr
        self.height=height
        self.material=material
        self.body_floor=body_floor
        self.R=R
        # robot position
        self.xb={}
        self.yb={}
        self.zb={}
        # velocity position
        self.xvb={}
        self.yvb={}
        self.zvb={}
        
        
        self.phib={}
        
        # total force
        self.Ftxb={}
        self.Ftyb={}
        self.Ftzb={}
        self.Spring_force={}
        self.spring_length={}
        self.Psi={} # heading angle
        
        self.my_system=my_system
        self.bots=[]
        self.Springs=[]
        
        self.XL=[]
        self.ZL=[]
        self.p1=0
        self.p2=self.diameter/2
        self.p3=0
        self.p4=-self.diameter/2
        self.h=0
        
        self.k=k
        self.rl=rl
        self.fixed=fixed
        
        self.type_spring=type_spring
        self.obj=obj
        self.force=[]
        self.mag=mag
            # Apply forces to active bots


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


            theta=i*2*np.pi/self.nb
            x=self.R*np.cos(theta)
            y=.5*height
            z=self.R*np.sin(theta)

            bot = chrono.ChBody()
            bot = chrono.ChBodyEasyCylinder(self.diameter/2, self.height,self.rowr)
            # set position
            bot.SetPos(chrono.ChVectorD(x,y,z))
            # material
            bot.SetMaterialSurface(self.material)
            # rotate them
            rotation1 = chrono.ChQuaternionD()
            rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
            bot.SetRot(rotation1)
            # forces
            myforcex = chrono.ChForce()
            bot.AddForce(myforcex)
            myforcex.SetMode(chrono.ChForce.FORCE)
            myforcex.SetDir(chrono.VECT_X)
            #myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
            self.force.append(myforcex)
    
            myforcey = chrono.ChForce()
            bot.AddForce(myforcey)
            myforcey.SetMode(chrono.ChForce.FORCE)
            myforcey.SetDir(chrono.VECT_Y)
            self.force.append(myforcey)
            
    
            myforcez = chrono.ChForce()
            bot.AddForce(myforcez)
            myforcez.SetMode(chrono.ChForce.FORCE)
            myforcez.SetDir(chrono.VECT_Z)
            self.force.append(myforcez)
            

            
            # collision models
            bot.GetCollisionModel().ClearModel()
            bot.GetCollisionModel().AddCylinder(self.diameter/2,self.diameter/2,self.height/2) # hemi sizes
            bot.GetCollisionModel().BuildModel()
            bot.SetCollide(True)
            bot.SetBodyFixed(self.fixed)
            pt=chrono.ChLinkMatePlane()
            pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
            self.my_system.AddLink(pt)
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            bot.GetAssets().push_back(body_floor_texture)
            # springs 
            if self.type_spring=="var":
                #self.p3=(self.diameter/2)*np.sin((i-1) * 2 * np.pi / (self.nb))
                #self.p4=(self.diameter/2)*np.cos((i-1) * 2 * np.pi / (self.nb))
                #self.p1=-(self.diameter/2)*np.sin(theta)
                #self.p2=-(self.diameter/2)*np.cos(theta)
         # link springs
                if i>=1:

                    ground=chrono.ChLinkSpring()
                    # Identify points to be attatched to the springs 
                    ground.SetName("ground")
                    # Attatches  springs
                    ground.Initialize(self.bots[i-1], bot,True,chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringK(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    
                    
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    # Last spring
                if i==self.nb-1:  
                    #self.p1=-(self.diameter/2)*np.sin((i) * 2 * np.pi / (self.nb))
                    #self.p2=(self.diameter/2)*np.cos((i) * 2 * np.pi / (self.nb))
                    #self.p3=(self.diameter/2)*np.sin(0)
                    #self.p4=-(self.diameter/2)*np.cos(0)
                    ground=chrono.ChLinkSpring()
                    ground.SetName("ground")
                    ground.Initialize(bot, self.bots[0], True, chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringK(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
            
            if self.type_spring=="const":
         # link springs
                if i>=1:
                    ground=chrono.ChLinkSpring()
                    # Identify points to be attatched to the springs 
                    ground.SetName("ground")
                    # Attatches  springs
                    ground.Initialize(self.bots[i-1], bot,True,chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringF(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    # Last spring
                if i==self.nb-1:  
                    ground=chrono.ChLinkSpring()
                    ground.SetName("ground")
                    ground.Initialize(bot, self.bots[0], True, chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringF(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground)            
            
            self.my_system.Add(bot)
            self.bots.append(bot)
            self.obj.append(bot)
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
            # spring force
     # save spring data       
    def save_data_spring_force(self):
        for i in range(self.nb):
            self.spring_length["spring"+str(i)].append(self.Springs[i].Get_SpringLength())
            if self.type_spring=="var":
                self.Spring_force["spring"+str(i)].append(self.Springs[i].Get_SpringLength()*self.Springs[i].Get_SpringK())
    
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
     
    def return_last_position(self):

        for i in range(self.nb):
            self.XL.append(self.xb['botx'+str(i)][-1])
            self.ZL.append(self.zb['botz'+str(i)][-1])
            
        np.savez("points.npz",allow_pickle=True,XL=self.XL,ZL=self.ZL)
        return(self.XL,self.ZL)
#[Interior Particles]
class Interiors:
    def __init__(self,nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,material,fixed,mode):
        
        self.diameter=diameter
        self.diameter2=diameter2
        self.nb=nb    
        self.R=(self.diameter*self.nb/(np.pi*2))+.1 
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
        self.n=[]
        self.ni=0
        self.mode=mode
        
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
        
        
        for i in range(self.n.size):
            for j in range(self.n[i]):
                self.xp["gransx{0}".format(i)]=[]
                self.yp["gransy{0}".format(i)]=[]
                self.zp["granz{0}".format(i)]=[]
                print(j)
                R2=self.diameter2*self.n[i]/(np.pi*2)
                x=R2*np.cos(j*2*np.pi/self.n[i])
                y=.5*self.height
                z=R2*np.sin(j*2*np.pi/self.n[i])
                gran = chrono.ChBody()
                gran = chrono.ChBodyEasyCylinder(self.diameter2/2, self.height,self.rowp)
                gran.SetPos(chrono.ChVectorD(x,y,z))
                gran.SetMaterialSurface(self.material)
                gran.SetId(i)
                # Create collision model
                gran.GetCollisionModel().ClearModel()
                gran.GetCollisionModel().AddCylinder(self.diameter2/2,self.diameter2/2,self.height/2) # hemi sizes
                gran.GetCollisionModel().BuildModel()
                gran.SetCollide(True)
                gran.SetBodyFixed(self.fixed)
                # add color
                col_r = chrono.ChColorAsset()
                col_r.SetColor(chrono.ChColor(1, 0, 0))
                gran.AddAsset(col_r)
                # mate to floor
                pt=chrono.ChLinkMatePlane()
                pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                my_system.AddLink(pt)
    
                # add to system
                self.my_system.Add(gran)
                self.obj.append(gran)
                self.particles.append(gran)
# max interior particles
    def MaxValues(self):
        Rin=self.R
        ngrans1=int(Rin/(self.diameter2))
        ri=np.zeros((1,ngrans1))
        ni=np.zeros((1,ngrans1))
        radii=Rin-(self.diameter2/2)
        for i in range(ngrans1):
            remainder=((self.diameter2))*i
            ri[:,i]=radii-remainder
            ni[:,i]=np.floor((ri[:,i]*np.pi*2)/self.diameter2)
        self.n=np.asarray(ni,dtype=int)
        return(self.n)
        
    # interior     
    def Interior(self):
        self.n=np.arange(self.nb+6,5,-7)   # array of interior robots
        return(self.n)
        
    # return system
    def return_system(self):
        return(self.my_system,self.particles,self.obj)

    def save_data(self):
        for i in range(self.ni):
            self.xp["gransx"+str(i)].append(self.particles[i].GetPos().x)
            self.yp["gransy"+str(i)].append(self.particles[i].GetPos().y)
            self.zp["gransz"+str(i)].append(self.particles[i].GetPos().z)
        return(self.xp,self.yp,self.zp)
#[Simulate]
class simulate:
    def __init__(self,my_system,bots,particles,Springs,obj,my_rep,sim,tstep,tend,visual,data_path,controls,lc):   
        self.visual=visual
        self.my_system=my_system
        self.sim=sim
        self.tstep=tstep
        self.tend=tend
        self.visual=visual
        self.data_path=data_path
        self.particles=particles
        self.bots=bots
        self.obj=obj
        self.Springs=Springs
        self.controls=controls
        self.time=[]
        self.lc=lc
        self.my_rep=my_rep
        self.nc=[]
        self.cx=[]
        self.cy=[]
        self.cz=[]
        self.Fxct=[]
        self.Fyct=[]
        self.Fzct=[]
        

        # contact points and forces 


    # simulate the robot
    def simulate(self):
        if self.visual=="irrlecht":
    #  Create an Irrlicht application to visualize the system
            myapplication = chronoirr.ChIrrApp(self.my_system,self.sim, chronoirr.dimension2du(1600,1200))
            myapplication.AddTypicalSky()
            myapplication.AddTypicalLogo('logo_pychrono_alpha.png')
            myapplication.AddTypicalCamera(chronoirr.vector3df(.6,.60,-1.00),chronoirr.vector3df(0,0,0))
            myapplication.SetSymbolscale(.002)
            myapplication.SetShowInfos(True)
            myapplication.SetContactsDrawMode(2)
            #myapplication.SetPaused(True)
            myapplication.AddLightWithShadow(chronoirr.vector3df(2,5,2),chronoirr.vector3df(2,2,2),10,2,10,120)

            myapplication.DrawAll               
            myapplication.AssetBindAll();
            myapplication.AssetUpdateAll();
            myapplication.AddShadowAll();
            count=0
            myapplication.SetTimestep(self.tstep)
            myapplication.SetTryRealtime(False)
            while(myapplication.GetDevice().run()):
                self.my_rep.ResetList()
                myapplication.BeginScene()
                myapplication.DrawAll()
                print ('time=', self.my_system.GetChTime())
                t=self.tstep*count  
                self.time.append(t)
                count=count+1
                
                if t>10:
                    self.controls.jam_mode=True
                else: 
                    self.controls.jam_mode=False
                    
                    
                self.controls.run_controller()
                self.controls.save_data_Forces()
                self.controls.clear_temp_forces()
                self.controls.Error()
                self.bots.save_data_position()
                self.bots.save_data_Forces()
                self.bots.save_data_velocity()

                self.bots.save_data_spring_force()
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
                # run step
                myapplication.DoStep()
                myapplication.EndScene()
                # save data
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.tend:
                    myapplication.GetDevice().closeDevice()
            
            
                
                
        return(self.bots,self.time,self.controls,self.cx,self.cy,self.cz,self.Fxct,self.Fyct,self.Fzct,self.nc)
        
#[material]          
def Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs):
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(mu_f)
    material.SetDampingF(mu_b)
    material.SetCompliance (C)
    material.SetComplianceT(Ct)
    material.SetRollingFriction(mu_r)
    material.SetSpinningFriction(mu_s)
    material.SetComplianceRolling(Cr)
    material.SetComplianceSpinning(Cs)
    return material



#[Create Floor]
def Floor(material,length,tall):
    body_floor = chrono.ChBody()
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
    body_floor_texture = chrono.ChTexture()
    body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    body_floor.GetAssets().push_back(body_floor_texture)
    return(body_floor)
    
class Controls:
    def __init__(self,forces,bots,Springs,k,rl,rlmax,type_spring,control_type,mag,fnx,fny,rbf,alpha,beta,nb):
        self.forces=forces
        self.bots=bots
        self.Springs=Springs
        self.k=k
        self.rl=rl
        self.rlmax=rlmax
        self.type_spring=type_spring
        self.control_type=control_type
        self.mag=mag
        self.xb=[]
        self.zb=[]
        self.xbv=[]
        self.zbv=[]
        self.alpha=alpha
        self.beta=beta
        self.fnx=fnx
        self.fny=fny
        self.rbf=rbf
        self.nb=nb
        self.Faxc={}
        self.Fayc={}
        self.Fazc={}
        # temporary arrays
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        self.E=[]
        self.jam_mode=False
        
        for i in range(self.nb):
            self.Faxc["botx{0}".format(i)]=[]
            self.Fayc["boty{0}".format(i)]=[]
            self.Fazc["botz{0}".format(i)]=[]
            
    def save_data_Forces(self):
       for i in range(self.nb):
           self.Faxc["botx"+str(i)].append(self.fxt[i])
           self.Fayc["boty"+str(i)].append(self.fyt[i])
           self.Fazc["botz"+str(i)].append(self.fzt[i])
                
    def clear_temp_forces(self):
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        
    def return_force_data(self):
        return(self.Faxc,self.Fayc,self.Fazc,self.E)        
    # run controller
    
    def run_controller(self):
        # no force applied
        if self.control_type=="nothing":
            
            (self.Springs)=self.setspring()
            
        # robot told to go right    
        if self.control_type=="force_right":
            for i in range(len(self.forces)):
                self.forces[i].SetMforce(self.mag)
                self.forces[i].SetDir(chrono.VECT_X)
       # pot field command         
        if self.control_type=="pot_field":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            
            for i in range(self.nb):
                
                fx=-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i]
                fz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]
                
                self.fxt.append(fx[0])
                self.fyt.append(0)
                self.fzt.append(fz[0])
                
                self.forces[3*i].SetMforce(fx[0])
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(fz[0])
                self.forces[3*i+2].SetDir(chrono.VECT_Z)

            

                  
# Keep spring lengths                
    def setspring(self):
        for i in range(len(self.Springs)):
            var1=self.Springs[i].Get_SpringLength()
            # if spring length is less then goes to zero
            if var1<self.rl:
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(0)
                    
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(0)
            # if greater then it will double
            if var1>self.rlmax:
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(self.k)
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(self.k)
                    
                        
        return(self.Springs)
    
# jam springs    
    def jam_springs(self):
        if self.jam_mode==True:
            self.rlmax=self.rlmax/4
            for i in range(len(self.Springs)):
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(3*self.k)
                    
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(4*self.k) 
            
        return(self.Springs)
        
# get current position         
    def get_position(self):
        self.xb=[]        
        self.zb=[]
        for i in range(len(self.bots)):
            self.xb.append(self.bots[i].GetPos().x)
            self.zb.append(self.bots[i].GetPos().z)
        return(self.xb,self.zb)
        
 # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.zbv=[]
        for i in range(len(self.bots)):
            self.xbv.append(self.bots[i].GetPos_dt().x)
            self.zbv.append(self.bots[i].GetPos_dt().z)
        return(self.xbv,self.zbv)
        
        
# error of controller        
    def Error(self):
        if self.control_type=="pot_field":
            et=[]
            for i in range(self.nb):
                val=(self.rbf(self.zb[i],self.xb[i]))**2
                et.append(val)
            self.E.append(.5*sum(et))
            
# Class for creating the points for the RBF
class Points_for_shape:
    def __init__(self,shape,p1,p2,nb,diameter,bl,br,R,nr,Rd):
        self.shape=shape
        self.p1=p1
        self.p2=p2
        self.nb=nb
        self.nr=nr
        self.diameter=diameter
        self.R=R
        self.Rd=Rd
        self.x=0
        self.y=0
        self.z=0
        self.rbf=0
        self.br=br
        self.bl=bl
        self.fny=0
        self.fnx=0
        
    # pLOT THe RBF function
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
                self.x[i+len(xt)*j]=self.nr[j]*xt[i]
                self.y[i+len(xt)*j]=self.nr[j]*yt[i]
                if j==0:
                    self.z[i+len(xt)*j]=0
                else:
                    self.z[i+len(xt)*j]=j             
        return (self.x,self.y,self.z)   
#[points for grab]
    def points_grab(self):
        
        theta1=np.linspace(np.pi/4,7*np.pi/4,int(.67*self.nb))
        x1=self.R*np.cos(theta1)+self.p1
        y1=self.R*np.sin(theta1)+self.p2

        theta2=np.linspace(np.pi/2,3*np.pi/2,int(.3*self.nb))
        x2=.6*self.R*np.cos(theta2)+.6*self.R+self.p1
        y2=.6*self.R*np.sin(theta2)+self.p2 
        
        xp=np.concatenate((x1, x2), axis=None)
        yp=np.concatenate((y1, y2), axis=None)
        zp=np.zeros(len(xp))
        
        theta3=np.linspace(np.pi/2+.3,3*np.pi/2-.3,int(.3*self.nb))
        x3=.6*self.R*np.cos(theta3)+1.2*self.R+self.p1
        y3=.6*self.R*np.sin(theta3)+self.p2

        theta4=np.linspace(np.pi/4-.2,7*np.pi/4+.2,int(.67*self.nb))
        x4=1.5*self.R*np.cos(theta4)+self.p1
        y4=1.5*self.R*np.sin(theta4)+self.p2
        
        xr=np.concatenate((x3, x4), axis=None)
        yr=np.concatenate((y3, y4), axis=None)
        zr=np.ones(len(xr))
    
        self.x=np.concatenate((xp, xr), axis=None)
        self.y=np.concatenate((yp, yr), axis=None)
        self.z=np.concatenate((zp, zr), axis=None)
        
        #plt.plot(x1,y1,'bo',x2,y2,'bo',x3,y3,'go',x4,y4,'go')
        return (self.x,self.y,self.z) 
    
    def points_grab2(self):
        path="C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/python/Pychrono/Grabbing/"
        file="points.npz"
        data=np.load(path+file,allow_pickle=True)
        YL=data['XL']
        XL=data['ZL']

        XL2=2*XL
        YL2=2*YL
        zp=np.zeros(len(XL))
        zr=np.ones(len(XL2))
        self.x=np.concatenate((XL, XL2), axis=None)
        self.y=np.concatenate((YL, YL2), axis=None)
        self.z=np.concatenate((zp, zr), axis=None)
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
                self.x[i+self.nb*j]=(self.R+nt)*np.cos(theta[i])+self.p1
                self.y[i+self.nb*j]=(self.R+nt)*np.sin(theta[i])+self.p2
                if j==0:
                    self.z[i+self.nb*j]=0
                else:
                    self.z[i+self.nb*j]=nt
        return (self.x,self.y,self.z)     
    
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
        #self.bodies = []
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
#        bodyUpA = chrono.CastContactableToChBody(modA)
#        nameA = bodyUpA.GetId()
#        bodyUpB = chrono.CastContactableToChBody(modB)
#        nameB = bodyUpB.GetId()
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fxc.append(force.x)
        self.Fyc.append(force.y)
        self.Fzc.append(force.z)
        #self.bodies.append([nameA,nameB])
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = [] 
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
    # Get the points
    def GetList(self):
        return (self.pointx,self.pointy,self.pointz,self.Fxc,self.Fyc,self.Fzc)


                
   # export data             
class export_data():
    def __init__(self,bots,interior,cx,cy,cz,Fxct,Fyct,Fzct,nc,controller,nb,sim,time,save_data,path):
        
        # objects 
        self.bots=bots
        self.interior=interior

        self.controller=controller
        # additional variables
        self.time={'time': time}
        self.sim=sim
        self.nb=nb
        self.nc=np.asarray(nc)
        self.lengthm=np.amax(self.nc)
        self.cx=cx
        self.cy=cy
        self.cz=cz
        self.Fxct=Fxct
        self.Fyct=Fyct
        self.Fzct=Fzct
        self.count=len(time)
        self.lengthm=np.amax(self.nc)
        self.save_data=save_data
        self.path=path
        #Create empty contact matrices
        self.xc=np.zeros((self.lengthm,self.count))
        self.yc=np.zeros((self.lengthm,self.count))
        self.zc=np.zeros((self.lengthm,self.count))
# Contact forces
        self.Fcx=np.zeros((self.lengthm,self.count))
        self.Fcy=np.zeros((self.lengthm,self.count))
        self.Fcz=np.zeros((self.lengthm,self.count))
        for i in range(self.count):
            ind=self.nc[i]
            tryme=self.cx[i]
            tryme2=self.cy[i]
            tryme3=self.cz[i]
            tryme4=self.Fxct[i]
            tryme5=self.Fyct[i]
            tryme6=self.Fzct[i]
            # convert to array
            tryme=np.asarray(tryme)
            tryme2=np.asarray(tryme2)
            tryme3=np.asarray(tryme3)
            tryme4=np.asarray(tryme4)
            tryme5=np.asarray(tryme5)
            tryme6=np.asarray(tryme6)
    
    # fill array position
            self.xc[0:ind,i]=np.transpose(tryme)
            self.yc[0:ind,i]=np.transpose(tryme2)
            self.zc[0:ind,i]=np.transpose(tryme3)

# Fill array forces
            self.Fcx[0:ind,i]=np.transpose(tryme4)
            self.Fcy[0:ind,i]=np.transpose(tryme5)
            self.Fcz[0:ind,i]=np.transpose(tryme6)    

    
        # return data robots
        (self.xb,self.yb,self.zb)=self.bots.return_position_data()
        (self.Faxb,self.Fayb,self.Fazb)=self.bots.return_force_data()
        (self.xvb,self.yvb,self.zvb)=self.bots.return_velocity_data()
        (self.Spring_force,self.Spring_length)=self.bots.return_spring_data()    
        (self.Faxc,self.Fayc,self.Fazc,self.E)=self.controller.return_force_data()
      
        
        self.results_dir = os.path.join(self.path)     

        if not os.path.isdir(self.results_dir):
            os.makedirs(self.results_dir)
# In[position data]            
        if self.save_data[0]==True:        
        
            self.file_name1=self.results_dir+'/position.csv'

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
                    
 # In[Forces]
        if self.save_data[1]==True:
         
            self.file_name2=self.results_dir+'/TotalForces.csv'

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
                
# In[Controller force]
        if self.save_data[2]==True:
            
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
             
# In[Error]
        if self.save_data[3]==True:
            self.file_name4=self.results_dir+'/Error.csv'    
            savetxt(self.file_name4,self.E, delimiter=',')                
            

# In[Spring force]
        if self.save_data[4]==True:
            
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
# In[Contact points]
        if self.save_data[5]==True:
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
        
            # contact force z
            self.file_name12=self.results_dir+'/z contact force.csv' 
     
            savetxt(self.file_name12,self.Fcz, delimiter=',')
        
        
            self.file_name13=self.results_dir+'/nc.csv' 
     
            savetxt(self.file_name13,self.nc, delimiter=',')
    def return_contact_forces(self):
        return(self.Fcx,self.Fcy,self.Fcz)
        
