# -*- coding: utf-8 -*- kl;
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
import timeit
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from scipy.optimize import fsolve
from scipy.optimize import minimize
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

# %%
class single_bot:
    def __init__(self,diameter,rowb,height,my_system,obj,body_floor,material):
        self.diameter=diameter
        self.rowb=rowb
        self.height=height
        self.my_system=my_system
        self.obj=obj
        self.body_floor=body_floor
        self.material=material
        self.bots=[]
        self.force=[]
        self.fixed=True
        self.x=0.5
        self.y=self.height
        self.z=0.25
        self.mass=.12
        bot=chrono.ChBody()
        #bot=chrono.ChBodyAuxRef()
        marker =chrono.ChMarker()
        marker.SetName('m')
        bot.AddMarker(marker)

        bot.SetMass(self.mass)
        Iyy=0.5*self.mass*(self.diameter)**2
        Ixx=0.25*self.mass*(self.diameter)**2+0.25*self.mass*(self.y)**2
        bot.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Ixx))
        bot.SetPos(chrono.ChVectorD(self.x, self.y, self.z))
        bot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
        cyl_s = chrono.ChCylinderShape()
        cyl_s.GetCylinderGeometry().p1 = chrono.ChVectorD(0, self.y, 0)
        cyl_s.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -self.y, 0)
        cyl_s.GetCylinderGeometry().rad = self.diameter
        bot.AddAsset(cyl_s)
        bot.GetCollisionModel().ClearModel()
        bot.GetCollisionModel().AddCylinder(self.diameter/4,self.diameter/4,self.height/2) # hemi sizes
        bot.GetCollisionModel().BuildModel()
        bot.SetCollide(True)
        body_floor_texture = chrono.ChTexture()
        body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')     
        bot.GetAssets().push_back(body_floor_texture)
        pt=chrono.ChLinkMatePlane()
        pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
        self.my_system.AddLink(pt)
        # x force
        myforcex = chrono.ChForce()
        bot.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetDir(chrono.VECT_X)
        myforcex.SetVrelpoint(chrono.ChVectorD(self.x,0.3*self.y,self.z))
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
        myforcez.SetVrelpoint(chrono.ChVectorD(self.x,0.3*self.y,self.z))
        self.force.append(myforcez)        
        self.my_system.Add(bot)
        self.bots.append(bot)
        self.obj.append(bot)
    # return system
    def return_system(self):
        return(self.my_system,self.bots,self.obj,self.force)        
        
# %%[Simulate]
class simulate:
    def __init__(self,my_system,bots,obj,sim,controller,tstep,tend,visual,data_path):   
        self.visual=visual
        self.my_system=my_system
        self.controls=controller
        self.sim=sim
        self.tstep=tstep
        self.tend=tend
        self.visual=visual
        self.data_path=data_path
        self.bots=bots
        self.obj=obj
        self.time=[]
        self.x=[]
        self.z=[]
        self.phi=[]
        self.U=[]

    def simulate(self):
        if self.visual=="irrlecht":
    #  Create an Irrlicht application to visualize the system
            myapplication = chronoirr.ChIrrApp(self.my_system,'1', chronoirr.dimension2du(1600,1200))
           
            myapplication.AddTypicalSky()
            myapplication.AddTypicalLogo('logo_pychrono_alpha.png')
            myapplication.AddTypicalCamera(chronoirr.vector3df(0,2,.75),chronoirr.vector3df(0,0,.75))
            myapplication.SetSymbolscale(.02)
            myapplication.SetShowInfos(True)
            myapplication.SetContactsDrawMode(2)
            myapplication.SetPaused(False)
            myapplication.AddLightWithShadow(chronoirr.vector3df(2,5,2),chronoirr.vector3df(2,2,2),10,2,10,120)

            myapplication.DrawAll               
            myapplication.AssetBindAll()
            myapplication.AssetUpdateAll()
            myapplication.AddShadowAll()
            count=0
            myapplication.SetTimestep(self.tstep)
            myapplication.SetTryRealtime(False)
            while(myapplication.GetDevice().run()):
                myapplication.BeginScene()
                myapplication.DrawAll()
                print ('time=', self.my_system.GetChTime())
                
                self.time.append(self.my_system.GetChTime())
                count=count+1
                
                # run the controllers 
                self.controls.run_controller()
                self.x.append(self.bots.bots[0].GetPos().x)
                self.z.append(self.bots.bots[0].GetPos().z)
                self.phi.append(self.controls.current_heading)
                self.U.append(self.controls.u)
                # run step
                myapplication.DoStep()
                myapplication.EndScene()
                # save data
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.tend:
                    myapplication.GetDevice().closeDevice()

class Controls:
    def __init__(self,force,bots,my_system,control_type,mag,path,ref_pt,m_timestep):
        # objects
        self.forces=force
        self.mag=mag
        self.bots=bots
        self.my_system=my_system
        self.control_type=control_type
        self.reference = self.bots.bots[0].SearchMarker('m')
        self.q=self.reference.GetAbsFrame()
        self.pos=0
        self.current_heading=0
        self.theta_tilda=0
        self.path=path
        self.ref_pt=ref_pt
        self.v_ref=1
        self.l, self.k, self.thetaref, self.s0, self.point = self.path.calc_track_error(0, 0, 0.0)
        #self.point = self.path.find_point_nearest(np.asarray((0,0)),1,0.5)
        self.a=180
        self.zeta=10
        self.k2=self.a**2
        self.k3=2*self.zeta*self.a
        self.m_timestep=m_timestep
        self.x=[]
        self.z=[]
        self.Fx=0
        self.Fz=0
        self.u=0
        self.patherr=np.zeros(20)
        self.errweight=1/(np.arange(20)+1)**2
    # run controller
    def run_controller(self):
       # Use the path controller       
        if self.control_type=="path":
            (self.Fx,self.Fz) =self.path_controller()
            self.forces[0].SetMforce(self.Fx)
            self.forces[0].SetDir(chrono.VECT_X)
            self.forces[2].SetMforce(self.Fz)
            self.forces[2].SetDir(chrono.VECT_Z)
                
        # Use the pose controller
        if self.control_type=="pose":
            (self.Fx,self.Fz) = self.pose_controller()
            self.forces[0].SetMforce(self.Fx)
            self.forces[0].SetDir(chrono.VECT_X)
            self.forces[2].SetMforce(self.Fz)
            self.forces[2].SetDir(chrono.VECT_Z)
    
    # Path controller            
    def path_controller(self):        
        # Heading angle based on velocity
        d2v = np.array([self.bots.bots[0].GetPos_dt().x, self.bots.bots[0].GetPos_dt().z])
        
        self.pos = np.array([self.bots.bots[0].GetPos().x, self.bots.bots[0].GetPos().z])
        self.l, self.k, self.thetaref, self.s0, self.point = self.path.calc_track_error(self.pos[0], self.pos[1], self.s0)

        # Normal and tangent direction unit vectors
        tan_dir = np.array([np.cos(self.thetaref),np.sin(self.thetaref)])
        norm_dir = -np.array([self.pos[0]-self.point[0],self.pos[1]-self.point[1]])
        norm_dir = norm_dir/np.linalg.norm(norm_dir)
        
        # Update error vector and calculate integral term
        self.patherr[1:20]=self.patherr[0:19]
        self.patherr[0]=self.l
        int_mag=10*np.ma.average(self.patherr, weights=self.errweight)
        
        # Scale magnitude of force in path tangent direction by current speed
        tan_err = self.v_ref*tan_dir-d2v
        mag_tan = 10
        
        # Scale magnitude of force in path normal dirction by distance from path plus an integral term
        mag_norm = 30*self.mag*abs(self.l**2)-int_mag
        
        # Convert mag_tan and mag_norm to Fx and Fz: basically proportional control based on distance and forward speed
        self.Fx = mag_tan*tan_err[0] + mag_norm*norm_dir[0]
        self.Fz = mag_tan*tan_err[1] + mag_norm*norm_dir[1]
        
        if type(self.Fx) is np.ndarray:
            self.Fx=self.Fx[0]
        if type(self.Fz) is np.ndarray:
            self.Fz=self.Fz[0]
        print('V: ',np.linalg.norm(d2v),'\n')
        return(self.Fx,self.Fz) 
        
    # Pose controller
    def pose_controller(self):
        # Current position and speed
        self.pos = np.array([self.bots.bots[0].GetPos().x, self.bots.bots[0].GetPos().z]) 
        vel = np.array([self.bots.bots[0].GetPos_dt().x, self.bots.bots[0].GetPos_dt().z])
        
        # Current position and speed error
        pos_err = self.ref_pt-self.pos
        vel_err = 0-vel
        
        # PD control force based on position and speed
        kp = 5 # Proportional gain
        kd = 3 # Derivative gain
        self.Fx = kp*pos_err[0]-kd*vel_err[0]
        self.Fz = kp*pos_err[1]-kd*vel_err[1]
        return(self.Fx,self.Fz)
        

    def pi_2_pi(self,angle):
        while(angle > math.pi):
            angle = angle - 2.0 * math.pi

        while(angle < -math.pi):
            angle = angle + 2.0 * math.pi

        return angle            
    
# %%[material]          
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
    
# Path         
class Path:
    def __init__(self,ax,ay):
        self.ax=ax
        self.ay=ay
        
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

        minimum = scipy.optimize.fmin_cg(calc_distance, s0, calc_distance_jacobian, args=(x, y), full_output=True, disp=False)
        
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
        
        # if its negative flip it to positive 
        #if angle < 0:
            #e*= -1
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

# In[Create sine path]
def create_sine_path():
            
    # create a path
    length=100
    st=np.zeros(length)

    for i in range(length):
        st[i]=i/10
    
    ax=np.zeros(length)
    ay=np.zeros(length)

    for i in range(length):
        ax[i]=st[i]
        ay[i]=np.sin(ax[i])

    return(ax,ay)   
         
def create_parabola():
            
    # create a path
    length=1000
    st=np.zeros(length)

    for i in range(length):
        st[i]=i/100
    
    ax=np.zeros(length)
    ay=np.zeros(length)

    for i in range(length):
        ax[i]=st[i]
        ay[i]=ax[i]**2
    
    return(ax,ay)  

def create_line():
            
    # create a path
    length=100
    st=np.zeros(length)

    for i in range(length):
        st[i]=i/100
    
    ax=np.zeros(length)
    ay=np.zeros(length)

    for i in range(length):
        ax[i]=st[i]
        ay[i]=ax[i]
    

    return(ax,ay)  