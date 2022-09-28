# -*- coding: utf-8 -*-
"""
Created on Fri May 22 09:38:25 2020

@author: dmulr
"""
import os
import sys
import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np
from numpy import linalg as LA
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import animatplot as amp
from matplotlib import colors as colors
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from scipy.optimize import fsolve
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm 
import scipy.constants  

# Create Robot
class Create_robot:
    def __init__(self,m_filename,m_visualization,m_datapath,my_system,material):
        self.m_filename=m_filename
        self.m_visualization=m_visualization
        self.m_datapath=m_datapath
        self.my_system=my_system
        self.material=material
        
        self.bot1 = self.my_system.SearchBody('_bot-1')
        self.wheel_left = self.my_system.SearchBody('wheel-1')
        self.wheel_right = self.my_system.SearchBody('wheel-2')
        self.reference = self.bot1.SearchMarker('right_marker')
        
        self.my_ground = self.my_system.SearchBody('ground')

        self.my_ground.SetMaterialSurface(self.material)
        self.r=.016
        self.b=.07324/2

# Create simulation
class simulate:
    def __init__(self,m_timestep,my_system,controller,bot1):
        self.my_system=my_system
        self.controller=controller
        self.bot1=bot1
        self.m_timestep=m_timestep
        self.x=[]
        self.z=[]
        self.desx=[]
        self.desz=[]
        self.des_theta=[]
        self.theta=[]
        self.theta_tilda=0
        self.theta_tilda_dot=0
        self.times=[]
        self.t=0
        self.pos_ref=0
       # extract data 
    def extractdata(self):
        self.x.append(self.bot1.bot1.GetPos().x)
        self.z.append(self.bot1.bot1.GetPos().z)
        self.theta.append(self.current_heading)
        self.times.append(self.t)
        
        self.desx.append(self.pos_ref[0])
        self.desz.append(self.pos_ref[1])
        self.des_theta.append(self.pos_ref[2])    
    # return data    
    def return_data(self):
        return(self.x,self.z,self.theta,self.times,self.desx,self.desz,self.des_theta,self.times)
    # simulate 
    def simulate(self):
                
        myapplication = chronoirr.ChIrrApp(self.my_system, 'Test', chronoirr.dimension2du(1600,1200))
        myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
        myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
        myapplication.AddTypicalCamera(chronoirr.vector3df(0,2,1.5),chronoirr.vector3df(0,0,0))
        myapplication.AddTypicalLights()
        myapplication.SetSymbolscale(0.02)
        myapplication.SetShowInfos(True)
        myapplication.SetContactsDrawMode(3)
        myapplication.SetPaused(True)

#myapplication.AddLightWithShadow(chronoirr.vector3df(10,20,10),chronoirr.vector3df(0,2.6,0), 10 ,10,40, 60, 512);
        myapplication.AssetBindAll()
        myapplication.AssetUpdateAll()
        myapplication.AddShadowAll()

#------------------------------------
# Run the simulation forever until windows is closed
#------------------------------------
        myapplication.SetTimestep(self.m_timestep)



        while(myapplication.GetDevice().run()):
            myapplication.BeginScene()
            myapplication.DrawAll()
            self.t = self.my_system.GetChTime()
            print(self.t)
    
    
            if self.t>.5:

                (self.q_d_dot,self.q_dot,self.pos_ref,self.current_heading)=self.controller.path_controller()
                #print(self.q_d_dot)
                self.bot1.wheel_right.SetWvel_loc(chrono.ChVectorD(0,0,self.q_d_dot[0]))
                self.bot1.wheel_left.SetWvel_loc(chrono.ChVectorD(0,0,-self.q_d_dot[1]))
                self.extractdata()
                
                
            myapplication.DoStep()
            myapplication.EndScene()
            if self.bot1.bot1.GetPos().x > self.controller.path.ax[-1] and self.bot1.bot1.GetPos().z > self.controller.path.ay[-1]:
                myapplication.GetDevice().closeDevice()

### Controller
class controller:
    def __init__(self, bot1,path,my_system,m_timestep):
        self.bot1=bot1
        self.r=self.bot1.r
        self.b=self.bot1.b
        self.path=path
        self.reference=self.bot1.reference
        self.my_system=my_system
        self.m_timestep=m_timestep
        self.a=10
        self.zeta=1
        self.k2=self.a**2
        self.k3=2*self.zeta*self.a
        self.v_ref=0.5
        self.u=0
        self.l, self.k, self.thetaref, self.s0 = self.path.calc_track_error(0, 0, 0.0)
        self.current_heading=0
        self.pos=0
        self.pos_dot=0
        self.v=0
        self.theta_tilda_dot=0
    
    def path_controller(self):
          # Heading angle
        q = self.reference.GetAbsFrame().GetRot()
        d2 = q.Rotate(chrono.ChVectorD(0,-1,0))
        d2v = np.array([d2.x, d2.z])
        d2v = d2v / np.linalg.norm(d2v)
        self.current_heading = np.arctan2(d2v[1],d2v[0])

        self.theta_tilda = self.current_heading - self.thetaref
        self.pos_dot_ref = np.array([self.v_ref*np.cos(self.theta_tilda+self.thetaref), self.v_ref*np.sin(self.theta_tilda+self.thetaref), self.theta_tilda_dot])
        self.pos_ref = self.pos + self.pos_dot_ref*self.m_timestep
        self.pos = np.array([self.bot1.bot1.GetPos().x, self.bot1.bot1.GetPos().z, self.current_heading])
        self.pos_dot = np.array([self.bot1.bot1.GetPos_dt().x, self.bot1.bot1.GetPos_dt().z, -self.bot1.bot1.GetWvel_loc().y])
    
        self.v = math.sqrt(self.bot1.bot1.GetPos_dt().x**2+self.bot1.bot1.GetPos_dt().y**2+self.bot1.bot1.GetPos_dt().z**2)
        self.l, self.k, self.thetaref, self.s0 = self.path.calc_track_error(self.pos[0], self.pos[1], self.s0)
    # Linear feedback law
        print(self.l)
        self.u=(-self.k3*abs(self.v_ref)*self.theta_tilda -self.k2*self.v_ref*self.l )
        #print(self.u)
    # Rates of change

        self.theta_tilda_dot = self.u
# Integrate to get new values
        self.theta_tilda = self.theta_tilda + self.theta_tilda_dot*self.m_timestep
        
        # Desired location, velocity, acceleration (wheels)
        self.q_d_dot = np.matmul(np.array([[1./self.r,self.b/self.r],[1./self.r, -self.b/self.r]]),np.array([-self.v_ref,self.u]))
    
        # Convert current speed to wheel velocities
        self.q_dot = np.matmul(np.array([[1./self.r,self.b/self.r],[1./self.r, -self.b/self.r]]),np.array([[self.v],[self.pos_dot[2]]]))
        

        return(self.q_d_dot,self.q_dot,self.pos_ref,self.current_heading)
        
    def straight_line(self):
        
  # Heading angle
        q = self.reference.GetAbsFrame().GetRot()
        d2 = q.Rotate(chrono.ChVectorD(0,-1,0))
        d2v = np.array([d2.x, d2.z])
        d2v = d2v / np.linalg.norm(d2v)
        self.current_heading = np.arctan2(d2v[1],d2v[0])

        self.theta_tilda = self.current_heading - self.thetaref
        self.pos_dot_ref = np.array([self.v_ref*np.cos(self.theta_tilda+self.thetaref), self.v_ref*np.sin(self.theta_tilda+self.thetaref), self.theta_tilda_dot])
        self.pos_ref = self.pos + self.pos_dot_ref*self.m_timestep
        self.pos = np.array([self.bot1.bot1.GetPos().x, self.bot1.bot1.GetPos().z, self.current_heading])
        self.pos_dot = np.array([self.bot1.bot1.GetPos_dt().x, self.bot1.bot1.GetPos_dt().z, -self.bot1.bot1.GetWvel_loc().y])
    
        self.v = math.sqrt(self.bot1.bot1.GetPos_dt().x**2+self.bot1.bot1.GetPos_dt().y**2+self.bot1.bot1.GetPos_dt().z**2)
    # Linear feedback law
        self.u=(-self.k3*abs(self.v_ref)*self.theta_tilda -self.k2*self.v_ref*self.l )

    # Rates of change
        l_dot = self.v_ref*np.sin(self.theta_tilda)
        self.theta_tilda_dot = self.u
# Integrate to get new values
        self.l = self.l + l_dot*self.m_timestep
        self.theta_tilda = self.theta_tilda + self.theta_tilda_dot*self.m_timestep
        
        # Desired location, velocity, acceleration (wheels)
        self.q_d_dot = np.matmul(np.array([[1./self.r,self.b/self.r],[1./self.r, -self.b/self.r]]),np.array([[-self.v_ref],[self.u]]))
    
        # Convert current speed to wheel velocities
        self.q_dot = np.matmul(np.array([[1./self.r,self.b/self.r],[1./self.r, -self.b/self.r]]),np.array([[self.v],[self.pos_dot[2]]]))
        

        return(self.q_d_dot,self.q_dot,self.pos_ref,self.current_heading)
        
        
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

# Find point nearest 
    def __find_nearest_point(self, s0, x, y):
        def calc_distance(_s, *args):
            _x, _y= self.X(_s), self.Y(_s)
            return (_x - args[0])**2 + (_y - args[1])**2
        
        def calc_distance_jacobian(_s, *args):
            _x, _y = self.X(_s), self.Y(_s)
            _dx, _dy = self.dX(_s), self.dY(_s)
            return 2*_dx*(_x - args[0])+2*_dy*(_y-args[1])

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
        # heading angle 
        yaw_ref = self.calc_yaw(s)
        
        dxl = self.X(s) - x
        dyl = self.Y(s) - y
        # error between the path and robot
        angle = pi_2_pi(yaw_ref - math.atan2(dyl, dxl))
        
        # if its negative flip it to positive 
        if angle < 0:
            e*= -1
        print(e)
        return e, k, yaw_ref, s    

# Convert angle       
def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

class Create_obstacles:
    def __init__(self,ox,oy,my_system,rr):
        self.my_system=my_system
        self.ox=ox
        self.oy=oy
        self.rr=rr
        self.height=.3
        self.rowr=10

        for i in range(len(self.ox)):
            obst = chrono.ChBody()
            obst = chrono.ChBodyEasyCylinder(self.rr/4, self.height,self.rowr)
            # set position
            obst.SetPos(chrono.ChVectorD(self.ox[i],self.height/2,self.oy[i]))
            # collision models
            obst.GetCollisionModel().ClearModel()
            obst.GetCollisionModel().AddCylinder(self.rr/4,self.rr/4,self.height/2) # hemi sizes
            obst.GetCollisionModel().BuildModel()
            obst.SetCollide(True)
            obst.SetBodyFixed(True)
            self.my_system.Add(obst)
            
    def return_system(self):
        return(self.my_system)
# potential fields 
class potential_field_path:
    def __init__(self,gx,gy,sx,sy,ox,oy,rr,AREA_WIDTH,reso,KP,ETA):
        self.gx=gx
        self.gy=gy
        
        self.sx=sx
        self.sy=sy
        
        self.ox=ox
        self.oy=oy
        
        self.xb=0
        self.yb=0
        
        self.qx=[]
        self.qy=[]
        
        self.py=0
        self.px=0
        
        self.fny=0
        self.fnx=0
        
        self.AREA_WIDTH=AREA_WIDTH
        self.KP=10
        self.ETA=1
        self.rr=rr
        self.reso=reso
        self.a=.05
        # find the smallest x
        self.minx = self.sx-self.AREA_WIDTH / 2.0
        # find the smallest y
        self.miny = self.sy-self.AREA_WIDTH / 2.0
        # find the max x
        self.maxx = self.gx+self.AREA_WIDTH / 2.0
        # find the max y
        self.maxy = self.gy+self.AREA_WIDTH / 2.0
        # determine number of grids in x and y direction
        self.xw = int(round((self.maxx - self.minx) / self.reso))
        self.yw = int(round((self.maxy - self.miny) / self.reso))
        self.xp=np.linspace(self.minx,self.maxx,self.xw)
        self.yp=np.linspace(self.miny,self.maxy,self.yw)
        self.xx, self.yy = np.meshgrid(self.xp, self.yp)
        self.pmap=np.zeros((len(self.yp),len(self.xp)))

    def calc_potential_field(self):
        for i in range(self.xw):
            x = self.xp[i]
            for j in range(self.yw):
                y = self.yp[j]
                ug = self.calc_attractive_potential(x, y)
   
                uo = self.calc_repulsive_potential(x, y)
    
                uf = ug+uo
                self.pmap[j,i] = uf

    
    def calc_attractive_potential(self,x, y):
        return (0.5 * self.KP * np.hypot(x - self.gx, y - self.gy))    

    def calc_repulsive_potential(self,x, y):
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(self.ox):
            d = np.hypot(x - self.ox[i], y - self.oy[i])
            if dmin >= d:
                dmin = d
                minid = i

    # calc repulsive potential
        dq = np.hypot(x - self.ox[minid], y - self.oy[minid])

        if dq <= self.rr:
            if dq <= 0.1:
                dq = 0.1

            return (0.5 * self.ETA * (1.0 / dq - 1.0 / self.rr) ** 2)
        else:
            return 0.0
    
    def next_point(self):
        d = np.hypot(self.sx - self.gx, self.sy - self.gy)

        self.qx.append(self.sx)
        self.qy.append(self.sy)
        count=0
        i=0
        while  d>=self.reso:
        #while count<2000:
            count=count+1
            gx_=self.fnx([self.qy[i],self.qx[i]])
            gy_=self.fny([self.qy[i],self.qx[i]])
            qx_i=self.qx[i]-self.a*gx_[0]
            qy_i=self.qy[i]-self.a*gy_[0]
            self.qx.append(qx_i)
            self.qy.append(qy_i)
            d = np.hypot(qx_i - self.gx, qy_i - self.gy)

            i=i+1        
        qxf=np.asarray(self.qx)
        qyf=np.asarray(self.qy)
        w = np.polyfit(qxf[0:len(qxf):3], qyf[0:len(qyf):3], 5)
        model = np.poly1d(w)
        self.xb = np.linspace(qxf.min(),qxf.max(),700)
        self.yb = model(self.xb)
        
    def draw_pot_field(self):

        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (1.25*fsy, fsy))     # Set the figure size to square
        plt.pcolor(self.xx,self.yy,self.pmap,cmap = 'jet')
        plt.colorbar()
        plt.plot(self.sx, self.sy, "*r")
        plt.plot(self.gx, self.gy, "*m")
        fig = plt.figure(figsize = (fsx, fsy))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(self.xx, self.yy,self.pmap, cmap = 'jet')   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        fig.colorbar(surf)                              # Include color bar
        plt.show() 
        
    def gradient_pot_field(self):
        (self.py,self.px)=np.gradient(self.pmap)
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.py)

        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.px)
        
    def plot_gradient_field(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        # Set the figure size to square
        fig, (ax0, ax1) = plt.subplots(nrows=2,figsize=(1.25*fsy, fsy))
         # y gradient
        im=ax0.pcolor(self.xx, self.yy, self.py,cmap = 'jet')
        ax0.set_title('y gradient')
        fig.colorbar(im,ax=ax0)
        # x gradient
        cf=ax1.pcolor(self.xx, self.yy, self.px,cmap = 'jet')
        ax1.set_title('x gradient')
        fig.colorbar(cf,ax=ax1)
    
        fig = plt.figure(figsize = (fsx, fsy))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(self.xx, self.yy, self.px, cmap = 'jet')   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.title(" potential x")
        fig.colorbar(surf)       
        # Include color bar
    
        fig = plt.figure(figsize = (fsx, fsy))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(self.xx, self.yy, self.py, cmap = 'jet')   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.title(" potential y")
        fig.colorbar(surf) 
        plt.show()
    def draw_path_bot(self):
        qxf=np.asarray(self.qx)

        qyf=np.asarray(self.qy)

        w = np.polyfit(qxf[0:len(qxf):3], qyf[0:len(qyf):3], 5)

        model = np.poly1d(w)

        self.xb = np.linspace(qxf.min(),qxf.max(),700)

        self.yb = model(self.xb)
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (1.25*fsy, fsy))     # Set the figure size to square
        plt.pcolor(self.xx,self.yy,self.pmap,cmap = 'jet')
        plt.colorbar()
        plt.plot(self.qx,self.qy,'k')
        plt.plot(self.xb,self.yb,'r')
        plt.plot(self.sx, self.sy, "*r")
        
        