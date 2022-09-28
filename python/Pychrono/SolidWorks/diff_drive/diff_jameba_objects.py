# -*- coding: utf-8 -*-
"""
Created on Sat May 23 10:57:16 2020

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


class Robots:
    def __init__(self,nb,bot_numbers,wheel_num,my_system,tire_mat):
        self.nb=nb
        self.bot_numbers=bot_numbers
        self.wheel_num=wheel_num
        self.my_system=my_system
        self.tire_mat=tire_mat
        self.right_wheels=[]
        self.left_wheels=[]
        self.right_markers = []
        self.left_markers = []
        self.bots = []
        for i in range(self.nb):
            bot1 = my_system.SearchBody('_bot-'+str(self.bot_numbers[i]))
            wheel_left = my_system.SearchBody('wheel-'+str(self.wheel_num[2*i]))
            wheel_right = my_system.SearchBody('wheel-'+str(self.wheel_num[2*i+1]))
            wheel_left.SetMaterialSurface(self.tire_mat)
            wheel_right.SetMaterialSurface(self.tire_mat)
            self.right_wheels.append(wheel_right)
            self.left_wheels.append(wheel_left)
            self.bots.append(bot1)

class interiors:
    def __init__(self,ni,my_system,mass,material):
        self.ni=ni
        self.my_system=my_system
        self.mass=mass
        self.material=material
        
        # Set materials for interior particles
        for i in range(1,self.ni):
            passive_interior = self.my_system.SearchBody('interior-' +str(i))
            passive_interior.SetMaterialSurface(self.material)
            passive_interior.SetMass(self.mass)
            
class simulate:
    def __init__(self,m_timestep,my_system,bot1,interior,control):
        self.my_system=my_system
        self.bot1=bot1
        self.m_timestep=m_timestep
        self.interior=interior
        self.coms_x=[]
        self.coms_z=[]
        self.leader_xs=[]
        self.leader_zs=[]
        self.leader_headings=[]
        self.coms_x=[]
        self.coms_z=[]
        self.control=control
        self.time=[]
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

        myapplication.AddLightWithShadow(chronoirr.vector3df(10,20,10),chronoirr.vector3df(0,2.6,0), 10 ,10,40, 60, 512);
        myapplication.AssetBindAll()
        myapplication.AssetUpdateAll()
        myapplication.AddShadowAll()

# Run the simulation forever until windows is closed
        myapplication.SetTimestep(self.m_timestep)
        while(myapplication.GetDevice().run()):
            myapplication.BeginScene()
            myapplication.DrawAll()
            self.control.t=(self.my_system.GetChTime())
            print(self.my_system.GetChTime())
            self.control.lead_follower_control()
            myapplication.DoStep()
            myapplication.EndScene()
            
            if self.bot1.bots[self.control.leader].GetPos().x > self.control.path.ax[-1]:
                myapplication.GetDevice().closeDevice()
# Class controller
class controller:
    def __init__(self,my_system,bots,path,leader,m_timestep):
        self.r=.016
        self.b=0.07324/2 # 1/2 Wheelbase
        self.leader=leader
        self.path=path
        self.v_ref = 1.5
        self.omega_r=180*np.pi/180
        self.zeta=1
        self.a=10
        self.k2=self.a**2
        self.k3=2*self.zeta*self.a
        self.k1_pose=1
        self.k2_pose=3
        #self.thetaref=0 
        self.ss=1
        self.bots=bots
        #self.l = self.bots.bots[self.leader].GetPos().x-0.2  # Signed distance between P and P
        #self.l_des=self.l
        self.l_dot=0
        self.leader_pos_dot_ref = np.array([0, 0, 0])
        self.followers = [x for i,x in enumerate(self.bots.bots) if i!=self.leader]
        self.followers_l = [x for i,x in enumerate(self.bots.left_wheels) if i!=self.leader]
        self.followers_r = [x for i,x in enumerate(self.bots.right_wheels) if i!=self.leader]
        self.k1_pose=1
        self.k2_pose=3
        self.pose_update = 10
        self.l, self.k, self.thetaref, self.s0 = self.path.calc_track_error(0, 0, 0.0)
        self.reference = self.bots.bots[self.leader].SearchMarker('right_marker')
        self.reference2=0
        self.leader_heading = 0
        self.leader_theta_tilda=0
        self.leader_pos=0
        self.t=0
        self.m_timestep=m_timestep
        self.coms_x=[]
        self.coms_z=[]
        self.leader_xs=[]
        self.leader_zs=[]
        self.leader_headings=[]
        self.coms_x=[]
        self.coms_z=[]
        
    def lead_follower_control(self):
        self.leader_controller()
        if round(self.t,2)%(self.m_timestep*self.pose_update) < 1e-3:
            self.follower_control()
            
# Leader controller             
    def leader_controller(self):
            
        q = self.reference.GetAbsFrame().GetRot()
        d2 = q.Rotate(chrono.ChVectorD(0,-1,0))
        d2v = np.array([d2.x, d2.z])
        d2v = d2v / np.linalg.norm(d2v)
        self.leader_heading = np.arctan2(d2v[1],d2v[0])
    
    # Read current state
        self.leader_theta_tilda = self.leader_heading - self.thetaref
        self.leader_pos = np.array([self.bots.bots[self.leader].GetPos().x, self.bots.bots[self.leader].GetPos().z, self.leader_heading])
        self.leader_pos_dot = np.array([self.bots.bots[self.leader].GetPos_dt().x, self.bots.bots[self.leader].GetPos_dt().z, -self.bots.bots[self.leader].GetWvel_loc().y])
        self.leader_v = math.sqrt(self.bots.bots[self.leader].GetPos_dt().x**2+self.bots.bots[self.leader].GetPos_dt().y**2+self.bots.bots[self.leader].GetPos_dt().z**2)
        self.l, self.k, self.thetaref, self.s0 = self.path.calc_track_error(self.leader_pos[0], self.leader_pos[1], self.s0)
    
    #Linear feedback law
        u=(-self.k3*abs(self.v_ref)*self.leader_theta_tilda -self.k2*self.v_ref*self.l)
        self.l_dot = self.v_ref*np.sin(self.leader_theta_tilda)
        self.leader_theta_tilda_dot = u

        #self.l = self.l + self.l_dot*self.m_timestep
        self.leader_theta_tilda = self.leader_theta_tilda + self.leader_theta_tilda_dot*self.m_timestep
    
    # Update ref velocities and positions and accelerations on path  
        self.leader_prev_pos_dot_ref = self.leader_pos_dot_ref
        self.leader_pos_dot_ref = np.array([self.v_ref*np.cos(self.leader_theta_tilda+self.thetaref), self.v_ref*np.sin(self.leader_theta_tilda+self.thetaref), self.leader_theta_tilda_dot])
        self.leader_pos_ref = self.leader_pos + self.leader_pos_dot_ref*self.m_timestep

    
    # Desired location, velocity, acceleration (wheels)
        self.q_d_dot = np.matmul(np.array([[1./self.r,self.b/self.r],[1./self.r, -self.b/self.r]]),np.array([[-self.v_ref],[u]]))
        self.bots.right_wheels[self.leader].SetWvel_loc(chrono.ChVectorD(0,0,self.q_d_dot[0][0]))
        self.bots.left_wheels[self.leader].SetWvel_loc(chrono.ChVectorD(0,0,-self.q_d_dot[1][0]))

        self.leader_xs.append(self.bots.bots[self.leader].GetPos().x)
        self.leader_zs.append(self.bots.bots[self.leader].GetPos().z)
        self.leader_headings.append(self.leader_heading)
        
        tempx = self.leader_pos[0]
        tempz = self.leader_pos[1]
        
        for robot in self.followers:
            tempx += robot.GetPos().x
            tempz += robot.GetPos().z
            
        com_x = tempx/(len(self.followers)+1)
        com_z = tempz/(len(self.followers)+1)
        self.coms_x.append(com_x)
        self.coms_z.append(com_z)
        
        
    def follower_control(self):
        counter=0
        for i in self.followers:
        # Heading angle
            self.reference2 = i.SearchMarker('right_marker')
            self.q = self.reference2.GetAbsFrame().GetRot()
            d2 = self.q.Rotate(chrono.ChVectorD(0,-1,0))
            d2v = np.array([d2.x,d2.z])
            d2v = d2v / np.linalg.norm(d2v)
            heading = np.arctan2(d2v[1],d2v[0])
            
            #Update current position and references
            z = np.array([[i.GetPos().x], [i.GetPos().z], [heading]])
            z_r = z + np.array([[self.leader_pos_dot_ref[0]],[self.leader_pos_dot_ref[1]],[self.leader_pos_dot_ref[2]]]) * self.m_timestep * 10 * self.pose_update
            z_r[2]=self.leader_heading
            
            # Transform Coordinates
            targ_vect = z_r[0:2:1]-z[0:2:1]
            targ_dist = np.linalg.norm(targ_vect)
            targ_vect = targ_vect/np.linalg.norm(targ_vect)
            targ_ang = np.arctan2(targ_vect[1],targ_vect[0])[0]
            delta = heading - targ_ang
            theta = z_r[2][0] - np.arctan2((z_r[1][0]-z[1][0]),(z_r[0][0]-z[0][0]))
            
            # Control Law
            if targ_dist < 0.05:
                k1_p=0
                v_r=self.v_ref/10
            else:
                k1_p=self.k1_pose
                v_r=self.v_ref
                
            w = (-v_r/targ_dist) * (self.k2_pose*(delta-np.arctan(-k1_p*theta))+(1+(k1_p/(1+(k1_p*theta)**2)))*np.sin(delta))
            u = np.array([[-v_r],[w]])
            
                # Desired wheel velocities
            des_vel_right = u[0][0]/self.r + self.b*u[1][0]/self.r
            des_vel_left = u[0][0]/self.r - self.b*u[1][0]/self.r

            
                # Set the wheel velocities
            self.followers_r[counter].SetWvel_loc(chrono.ChVectorD(0,0,des_vel_right))
            self.followers_l[counter].SetWvel_loc(chrono.ChVectorD(0,0,-des_vel_left))
                        
            counter = counter + 1


# return values
    def return_values(self):
        return(self.coms_x,self.coms_z,self.leader_xs,self.leader_zs,self.leader_headings)
        
        
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

        return e, k, yaw_ref, s    

        
def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle