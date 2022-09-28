# -*- coding: utf-8 -*-
"""
Created on Tue May 12 12:53:23 2020

@author: dmulr
"""

import numpy as np
import math as math
import os
import numpy as np
import math as math
import statistics 
import matplotlib.pyplot as plt
from matplotlib import colors as colors
import matplotlib as mpl
from matplotlib import animation
import matplotlib
import matplotlib.cm as cm
import matplotlib.patches as mpl_patches
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from matplotlib.patches import RegularPolygon
import cv2
import csv
import glob
import pandas as pd
#from scipy.signal import lfilter
from scipy import interpolate
from scipy.ndimage import gaussian_filter1d
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
import matplotlib.patches as patches
import matplotlib
from mpl_toolkits.axes_grid.inset_locator import (inset_axes, InsetPosition,
                                                  mark_inset)
from scipy.interpolate import RegularGridInterpolator
from scipy.interpolate import interp1d
# In
class robot_plots:
    def __init__(self,sim,control_type,path,nb,ni,radius,radius2,height,ball_radius=None,r1=None,r2=None,R=None,mu=None):
        self.sim=sim
        self.radius=radius
        self.radius2=radius2
        self.ball_radius=ball_radius
        self.height=height # height of the objects
        self.control_type=control_type # control type used
        self.path=path 
        self.ni=np.sum(ni) # sum up the interior particles
        self.nb=nb # number of boundary robots
        self.results_dir = os.path.join('plots'+self.sim)
        self.R=R # radius of Robot
        self.r1=r1
        self.r2=r2
        self.mu=mu # friction 
        
        if not os.path.isdir(self.results_dir):
            os.makedirs(self.results_dir)
        
        #### position
        name='bot_position.csv'
        filename=self.path+name
        self.data1 = np.genfromtxt(filename,delimiter=',')
        (self.m1,self.n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:self.n1]
        self.time=self.data1[0,:]
        self.xpos=self.data1[1:self.nb+1,:]
        self.ypos=self.data1[self.nb+1:2*self.nb+1,:]
        self.zpos=self.data1[(2*self.nb)+1:3*self.nb+1,:] 
        
        #### velocity
        name='bot_velocity.csv'
        filename=self.path+name
        self.data2 = np.genfromtxt(filename,delimiter=',')
        (self.m2,self.n2)=np.shape(self.data2)
        self.data2=self.data2[:,1:self.n2]
        #self.time=self.data2[0,:]
        self.xvel=self.data2[1:nb+1,:]
        self.yvel=self.data2[self.nb+1:2*self.nb+1,:]
        self.zvel=self.data2[(2*self.nb)+1:3*self.nb+1,:]
        
        #### Total force
        name='bot_TotalForces.csv'
        filename=self.path+name
        self.data3 = np.genfromtxt(filename,delimiter=',')
        (self.m3,self.n3)=np.shape(self.data3)
        self.data3=self.data3[:,1:self.n3]
        self.Fx=self.data3[1:self.nb+1,:]
        self.Fy=self.data3[nb+1:2*self.nb+1,:]
        self.Fz=self.data3[(2*self.nb)+1:3*self.nb+1,:]

        #### controller force so the force each robot applies 
        name='Force_controller.csv'
        filename=self.path+name
        self.data4 = np.genfromtxt(filename,delimiter=',')
        (self.m4,self.n4)=np.shape(self.data4)
        self.data4=self.data4[:,1:self.n4]
        self.Fxc=self.data4[1:self.nb+1,:]
        self.Fyc=self.data4[nb+1:2*self.nb+1,:]
        self.Fzc=self.data4[(2*self.nb)+1:3*self.nb+1,:] 
        
        
        #### particle position 
        name='particle_position.csv'
        filename=self.path+name
        self.data4a = np.genfromtxt(filename,delimiter=',')
        (self.m4a,self.n4a)=np.shape(self.data4a)
        self.data4a=self.data4a[:,1:self.n4a]
        self.time=self.data4a[0,:]
        self.xposp=self.data4a[1:self.ni+1,:]
        self.yposp=self.data4a[self.ni+1:2*self.ni+1,:]
        self.zposp=self.data4a[(2*self.ni)+1:3*self.ni+1,:]
          
        self.Xb={} # empty list of robot x position
        self.Yb={} # empty list of robot y position 
        
        self.Xp={} # empty list of particle positions
        self.Yp={} # empty list of particle position
        self.Fcx={} # empty list of x controller force
        self.Fcy={} # empty list of x controller force
        self.Fcz={} # empty list of z controller force
        
     

        #### ball variables 
        if self.control_type=='pot_field_grab':
            # ball force c
            name='ballFx.csv'
            filename=path+name
            self.data5 = np.genfromtxt(filename,delimiter=',')
            # ball force z
            name='ballFz.csv'
            filename=path+name
            self.data6 = np.genfromtxt(filename,delimiter=',')
            # ball x position
            name='ballx.csv'
            filename=path+name
            self.ballx = np.genfromtxt(filename,delimiter=',')
            # ball z position
            name='ballz.csv'
            filename=path+name
            self.ballz= np.genfromtxt(filename,delimiter=',')
            # ball velocity x
            name='ballvx.csv'
            filename=path+name
            self.data9 = np.genfromtxt(filename,delimiter=',')
            # ball velocity z
            name='ballvz.csv'
            filename=path+name
            self.data10 = np.genfromtxt(filename,delimiter=',')
            
            # apply force 
            name='ballFb.csv'
            filename=path+name
            self.Fb = np.genfromtxt(filename,delimiter=',')

            # center of field X
            name='PX.csv'
            filename=path+name
            self.PX = np.genfromtxt(filename,delimiter=',')
            
            # center of field Y
            name='PY.csv'
            filename=path+name
            self.PY = np.genfromtxt(filename,delimiter=',')
            
            # mass of ball
            name='MB.csv'
            filename=path+name
            self.MB = np.genfromtxt(filename,delimiter=',')            
                                
            
            # alpha coefficient
            name='ALPHA.csv'
            filename=path+name
            self.ALPHA = np.genfromtxt(filename,delimiter=',')          
        
        
        #### Contact force x
        name="x contact force.csv"
        filename=path+name
        self.data13 = np.genfromtxt(filename,delimiter=',')

        #### Contact force y
        name="y contact force.csv"
        filename=path+name
        self.data14 = np.genfromtxt(filename,delimiter=',')

        #### Contact force z
        name="z contact force.csv"
        filename=path+name
        self.data15 = np.genfromtxt(filename,delimiter=',')

        #### contact points x
        name="x contact points.csv"
        filename=path+name
        self.data16 = np.genfromtxt(filename,delimiter=',')

        #### contact points y
        name="y contact points.csv"
        filename=path+name
        self.data17 = np.genfromtxt(filename,delimiter=',')

        #### contact points z
        name="z contact points.csv"
        filename=path+name
        self.data18 = np.genfromtxt(filename,delimiter=',')

        #### bodyA
        name="AN.csv"
        filename=path+name
        #self.data188 = np.loadtxt(filename, dtype=str)
        #self.data188 = pd.read_csv(filename, sep=',',header=None)
        self.data188=[]
        infile = open(filename, 'r')
        for row in csv.reader(infile):
            self.data188.append(row)
        
            
        #### bodyB
        name="BN.csv"
        filename=path+name        
        self.data189=[]
        infile = open(filename, 'r')
        for row in csv.reader(infile):
            self.data189.append(row)
            

        
        #### contact points 
        name="nc.csv"
        filename=path+name
        self.data19 = np.genfromtxt(filename,delimiter=',')        

        # rename for simplicity
        self.Fcx=self.data13
        self.Fcy=self.data14
        self.Fcz=self.data15        
        self.xc=self.data16
        self.yc=self.data17
        self.zc=self.data18
        self.nc=self.data19        

        self.XC=[]
        self.ZC=[]
    
        self.bodyA=self.data188
        self.bodyB=self.data189
        self.MAG=0

        self.FBX=[]
        self.FBZ=[]
        #(self.FBX,self.FBZ)=self.extract_forces()
        #(self.FXBp,self.FZBp,self.FZPp,self.FXPp)=self.find_pressure() 
        #self.plot_pressure(self.FXBp,self.FZBp,self.FZPp,self.FXPp)
        
        #### Error
        if self.control_type=="shape_form" or self.control_type=='image_warp':
            name="error.csv"
            filename=path+name
            self.data24 = np.genfromtxt(filename,delimiter=',')
            self.E=self.data24  
            
        #### Plot for each case
        if self.control_type=="pot_field_grab":  
            #(self.FBX,self.FBZ,self.XC,self.ZC)=self.extract_force_and_position_contact()
            #self.plot_contact_points()
            self.contact_points_plots()
            # (self.FXBp,self.FZBp,self.FZPp,self.FXPp)=self.find_pressure()
            # self.plot_pressure(self.FXBp,self.FZBp,self.FZPp,self.FXPp)
            # self.pressure_distribution()
            # self.average_pressure_plot()
            # self.plot_xyz()
            # self.plot_velocity_xyz()
            # self.plot_mag_velocity()
            # self.plot_total_forces()
            # self.plot_mag_controls()
            # self.Plot_fxyzcontroller()
            # self.plot_ball_position()
            # #self.plot_ball_force()
            # #self.plot_tug_force()
            # #self.tug_test()
            # self.Forcechains()
            
        # if self.control_type=="tunnel":       
        #     self.plot_xyz()
        #     self.plot_velocity_xyz()
        #     self.plot_mag_velocity()
        #     self.plot_total_forces()
        #     self.plot_mag_controls()
        #     self.Plot_fxyzcontroller()
        #     self.plot_path()
            
        # if self.control_type=="shape_form":
        #     self.plot_xyz()
        #     self.plot_velocity_xyz()
        #     self.plot_mag_velocity()
        #     self.plot_total_forces()
        #     self.plot_mag_controls()
        #     self.Plot_fxyzcontroller() 
        #     #self.Plot_shape()
        #     #self.Plot_shape_start_end()
        #     self.plot_error()
            
        # if self.control_type=="image_warp":
        #     self.plot_xyz()
        #     self.plot_velocity_xyz()
        #     self.plot_mag_velocity()
        #     self.plot_total_forces()
        #     self.plot_mag_controls()
        #     self.Plot_fxyzcontroller() 
        #     #self.Plot_shape()
        #     #self.Plot_shape_start_end()
        #     self.plot_error()
        #     self.Forcechains()


    def sort_data(self):
        # Sort position data of robots and particles #
        (m1,n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:n1]
        self.time=self.data1[0,:]
        Xpos=self.data1[1:self.nb+1,:]
        Ypos=self.data1[self.nb+1:2*self.nb+1,:]
        Zpos=self.data1[(2*self.nb)+1:3*self.nb+1,:] 
        (m4,n4)=np.shape(self.data4)        
        self.data4=self.data4[:,1:n4]        
        Xposp=self.data4[1:self.ni+1,:]
        Yposp=self.data4[self.ni+1:2*self.ni+1,:]
        Zposp=self.data4[(2*self.ni)+1:3*self.ni+1,:]  
        
        for i in range(self.nb):
            self.Xb["Xb{0}".format(i)]=Xpos[i,:]
            self.Yb["Yb{0}".format(i)]=Zpos[i,:]

        
        for i in range(self.ni):
            self.Xp["Xp{0}".format(i)]=Xposp[i,:]
            self.Yp["Yp{0}".format(i)]=Zposp[i,:]

    def sort_data_forces(self):
        # sort control force  #
        (m7,n7)=np.shape(self.data7)
        self.data7=self.data7[:,1:n7]
        Fcx=self.data7[1:self.nb+1,:]
        Fcy=self.data7[self.nb+1:2*self.nb+1,:]
        Fcz=self.data7[(2*self.nb)+1:3*self.nb+1,:] 
        
        for i in range(self.nb):
            self.Fcx["Fcx{0}".format(i)]=Fcx[i,:]
            self.Fcy["Fcy{0}".format(i)]=Fcy[i,:]
            self.Fcz["Fcz{0}".format(i)]=Fcz[i,:]        
        



    #### functions            
    def extract_forces(self):
        for i in range(len(self.time)):
            fbx=[]
            fbz=[]
            for j in range(int(self.nc[i])):
                if self.bodyA[j,i]==1 or self.bodyB[j,i]==1:

                    fbx.append(abs(self.Fcx[j,i]))
                    fbz.append(abs(self.Fcz[j,i]))
            self.FBX.append(np.sum(fbx))
            self.FBZ.append(np.sum(fbz))
    
        return(self.FBX,self.FBZ)             

    def extract_force_and_position_contact(self):
        for i in range(len(self.time)):
            fbx=[]
            fbz=[]
            xc=[]
            zc=[]
            tempA=self.bodyA[i]
            tempB=self.bodyB[i]            
            for j in range(int(self.nc[i])):
                if tempA[j]=='ball' or tempB[j]=='ball':

                    fbx.append(abs(self.Fcx[j,i]))
                    fbz.append(abs(self.Fcz[j,i]))
                    xc.append(self.xc[j,i])
                    zc.append(self.zc[j,i])
            self.FBX.append(fbx)
            self.FBZ.append(fbz)
            self.XC.append(xc)
            self.ZC.append(zc)
        return(self.FBX,self.FBZ,self.XC,self.ZC)        
            
    def plot_xyz(self):
        #---plot the xyz positon of the bots---#
        direct = os.path.join(self.results_dir,'xyz positions')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            fig=plt.figure(i)
            plt.figure(figsize=(15,10))
    
            fig.suptitle("Position  vs Time (s) for Bot " + str(i))
            ax1 = plt.subplot(3,1,1)
            ax1.grid(True)
            plt.gca().set_title('x position (cm) vs time')
            plt.plot(self.time, self.xpos[i,:]*100,'b')
    
            ax2 = plt.subplot(3,1,2)
            plt.gca().set_title('y position(vm) vs time')
            plt.plot(self.time,self.ypos[i,:]*100,'r')
            ax2.grid(True)
            
            ax3 = plt.subplot(3,1,3)
            plt.gca().set_title('z position (cm) vs time')
            plt.plot(self.time, self.zpos[i,:]*100,'g')
            ax3.grid(True)
            plt.subplots_adjust(hspace = 1)
            plt.xlabel('time (seconds)')
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all')
   #[Velocity]
    def plot_velocity_xyz(self):
        #------plot the velocity of the xyz of each bot-----#
        direct = os.path.join(self.results_dir,'xyz velocity')
        
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            fig=plt.figure(i)
            plt.figure(figsize=(15,10))
    
            fig.suptitle("Position  vs Time (s) for Bot " + str(i))
            ax1 = plt.subplot(3,1,1)
            ax1.grid(True)
            plt.gca().set_title('x speed (cm/s) vs time')
            plt.plot(self.time, self.xvel[i,:]*100,'b')
    
            ax2 = plt.subplot(3,1,2)
            plt.gca().set_title('y speed (cm/s) vs time')
            plt.plot(self.time,self.yvel[i,:]*100,'r')
            ax2.grid(True)
            
            ax3 = plt.subplot(3,1,3)
            plt.gca().set_title('z speed (cm/s) vs time')
            plt.plot(self.time, self.zvel[i,:]*100,'g')
            ax3.grid(True)
            plt.subplots_adjust(hspace = 1)
            plt.xlabel('time (seconds)')
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all')
            
    def find_pressure(self):
        #--- calculate the internal pressure ---#
        FXB=np.zeros((self.nb,len(self.time)))
        FZB=np.zeros((self.nb,len(self.time)))
        FXP=np.zeros((self.ni,len(self.time)))
        FZP=np.zeros((self.ni,len(self.time)))
        for i in range(len(self.time)):
            for j in range(int(self.nc[i])):
                temp1=self.bodyA[i][j]
                temp2=self.bodyB[i][j]
              
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
                        FXP[num,i]=abs((self.Fcx[j,i]/A))+FXP[num,i]
                        FZP[num,i]=abs((self.Fcz[j,i]/A))+FZP[num,i]
                            
  
                    if temp1[0:4]=="gran":
                        if temp1[4]=='b' :
                            r=self.radius2*np.sqrt(2)
                        else:
                            r=self.radius2 
                        A = np.pi * 2 * r * self.height    
                        les=len(temp1)
                        num=int(temp1[5:les])
                        FXP[num,i]=abs((self.Fcx[j,i]/A))+FXP[num,i]
                        FZP[num,i]=abs((self.Fcz[j,i]/A))+FZP[num,i]  
                        
              
                # BOTS PRESSURE
                if temp1[0:3]=="bot" or temp2[0:3]=="bot":
                    if temp2[0:3]=="bot":  
                        les=len(temp2)
                        num=int(temp2[3:les])
                        FXB[num,i]=abs(self.Fcx[j,i]/(self.radius * 2 * np.pi * self.height))+FXB[num,i]
                        FZB[num,i]=abs(self.Fcz[j,i]/(self.radius * 2 * np.pi * self.height))+FZB[num,i]
                    else:
                        les=len(temp1)
                        num=int(temp1[3:les])
                        FXB[num,i]=abs(self.Fcx[j,i]/(self.radius * 2 * np.pi * self.height))+FXB[num,i]
                        FZB[num,i]=abs(self.Fcz[j,i]/(self.radius * 2 * np.pi * self.height))+FZB[num,i]
                        
                    
        return(FXB,FZB,FZP,FXP)




    def plot_pressure(self,FXB,FZB,FZP,FXP):
        #--- plot pressure with points---#       
        direct = os.path.join(self.results_dir,'pressure_points')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        
        FX=np.vstack([FXB,FZP])
        FZ=np.vstack([FZB,FZP])
        X=np.vstack([self.xpos,self.xposp])
        Z=np.vstack([self.zpos,self.zposp])        
        count=0
        
        self.MAG=np.zeros((self.nb+self.ni,len(self.time)))
        for j in range(len(self.time)):
            for i in range(self.nb+self.ni-1):
            
                self.MAG[i,j]=np.sqrt(FZ[i,j]**2+FX[i,j]**2)    
            N = 30
  
            # colormap 
            cmap = plt.get_cmap('jet', N)                 
        for i in range(len(self.time)):
            plt.figure(i,figsize=(8,8))
            plt.tricontourf(X[:,i],Z[:,i],self.MAG[:,i],cmap=cmap)
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
        name=self.sim+'_pressure_anim'
        
        self.create_video(name,direct,self.results_dir,'jpg')
        # pressure without points
        direct = os.path.join(self.results_dir,'pressure_wo_points')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range(len(self.time)):    
            plt.figure(i,figsize=(8,8))
            plt.tricontourf(X[:,i],Z[:,i],self.MAG[:,i],cmap=plt.cm.get_cmap('jet', 20))
            plt.plot(X[0:self.nb,i],Z[0:self.nb,i], 'ko ')
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
        name=self.sim+'_pressure_anim_wo_points'
        self.create_video(name,direct,self.results_dir,'jpg')
        # distribution of forces         
        direct = os.path.join(self.results_dir,'distribution')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        
    def plot_contact_points(self):
        # --- plot the pressure distribution how much of each value is there --- # 
        direct = os.path.join(self.results_dir,'contact points')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0    
        
        fig = plt.figure(dpi=300)
        fig.set_size_inches(3, 3)

        widx=8
        widy=8
        ax = plt.axes(xlim=(-widx,widx), ylim=(-widy, widy))
        plt.axis('equal')
        for i in range(len(self.time)):    
            plt.plot(self.XC[i],self.ZC[i],'sb')
            patch = plt.Circle((self.ballz[i], self.ballx[i]),self.ball_radius, fc='tab:red',edgecolor='tab:red',linewidth=1)
            ax.add_patch(patch)
            plt.title('t='+str(round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count) 
            print(str(i)+ "of"+ str(len(self.time)))
            plt.close('all')
            count=count+1   
        plt.ylim(-.2)
        name=self.sim+'contact points'        
        self.create_video(name,direct,self.results_dir,'jpg')  

              
    def pressure_distribution(self):
        # --- plot the pressure distribution how much of each value is there --- # 
        direct = os.path.join(self.results_dir,'distribution')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0    
        for i in range(len(self.time)):    
            plt.figure(i,figsize=(8,8))
            plt.hist(self.MAG[:,i], bins = 20)
            plt.title('t='+str(round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count) 
            print(str(i)+ "of"+ str(len(self.time)))
            plt.close('all')
            count=count+1   
        name=self.sim+'_distribution_forces'        
        self.create_video(name,direct,self.results_dir,'jpg')
        
    def average_pressure_plot(self):
        # --- average pressure --- #
        count=0
        direct = os.path.join(self.results_dir,'avg_pressure')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        MAGV=[]
        for i in range(len(self.time)):  
            MAGV.append(np.mean(self.MAG[:,i]))
        # n = 5 # the larger n is, the smoother curve will be
        # b = [1.0 / n] * n
        # a = 1

        # MAGVe=lfilter(b,a,MAGV)
        y3 = gaussian_filter1d(MAGV, 6)
        plt.figure(1,figsize=(8,8))

        plt.plot(self.time,y3,'tab:red',linewidth=2)
        plt.plot(self.time,MAGV,'tab:blue')
        plt.grid(True)

        plt.xlabel('time (seconds)')
        plt.ylabel('AVG pressure (Pa)')
        plt.title('pressure vs time')
        plt.savefig(direct+'/avg pressure.jpg')  
        plt.savefig(direct+'/avg pressure.eps') 
        plt.close('all')
        count=count+1 
# alpha vs time             
        plt.figure(1,figsize=(8,8))
        plt.plot(self.time,self.ALPHA,'tab:blue')
        plt.grid(True)

        plt.xlabel('time (seconds)')
        plt.ylabel('alpha')
        plt.title('Time vs alpha')
        plt.savefig(direct+'/alpha.jpg')  
        plt.savefig(direct+'/alpha.eps') 
        plt.close('all')        
# MAGV vs alpha
        plt.figure(1,figsize=(8,8))
        plt.plot(self.ALPHA,MAGV,'tab:blue')
        plt.plot(self.ALPHA,y3,'tab:red',linewidth=2)
        plt.grid(True)

        plt.xlabel('alpha')
        plt.ylabel('pressure (pa)')
        plt.title('pressure vs alpha')
        plt.savefig(direct+'/MAGV_ALPHA.jpg')  
        plt.savefig(direct+'/MAGV_ALPHA.eps') 
        plt.close('all')



                      
# #[Magnitude of the velocites]           
    def plot_mag_velocity(self):
        direct = os.path.join(self.results_dir,'magnitude of velocity')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        plt.figure(figsize=(20,10))    
        for i in range (self.nb):
            plt.plot(self.time, np.sqrt(self.xvel[i,:]**2+self.zvel[i,:]**2),label='bot '+str(i))
        plt.grid(True)        
        plt.title("velocity vs Time (s) for Bot "+self.sim )
        plt.xlabel('time (seconds)')
        plt.ylabel('velocity (m/s)')
        plt.savefig(direct+'/'+'magnitude_velocity'+".png") 
        plt.close('all')            
#[Individual robot forces]          
    def Plot_fxyz(self):
        direct = os.path.join(self.results_dir,'xyz Forces')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            fig=plt.figure(i)
            plt.figure(figsize=(15,10))
    
            fig.suptitle("Force vs Time (s) for Bot " + str(i))
            ax1 = plt.subplot(3,1,1)
            ax1.grid(True)
            plt.gca().set_title('X Force (N) vs time')
            plt.plot(self.time, self.Fx[i,:],'b')
    
            ax2 = plt.subplot(3,1,2)
            plt.gca().set_title('Y Force (N) vs time')
            plt.plot(self.time,self.Fy[i,:],'r')
            ax2.grid(True)
            
            ax3 = plt.subplot(3,1,3)
            plt.gca().set_title('Z Force (N) vs time')
            plt.plot(self.time, self.Fz[i,:],'g')
            ax3.grid(True)
            plt.subplots_adjust(hspace = 1)
            plt.xlabel('time (seconds)')
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all')   
            
#[Controller forces]            
    def Plot_fxyzcontroller(self):
        direct = os.path.join(self.results_dir,'xyz Controller Forces')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            fig=plt.figure(i)
            plt.figure(figsize=(20,10))

            fig.suptitle("Force vs Time (s) for Bot " + str(i))
            ax1 = plt.subplot(4,1,1)
            ax1.grid(True)
            plt.gca().set_title('X Force (N) vs time')
            plt.plot(self.time, self.Fxc[i,:],'b')

            ax2 = plt.subplot(4,1,2)
            plt.gca().set_title('Y Force (N) vs time')
            plt.plot(self.time,self.Fyc[i,:],'r')
            ax2.grid(True)
        
            ax3 = plt.subplot(4,1,3)
            plt.gca().set_title('Z Force (N) vs time')
            plt.plot(self.time, self.Fzc[i,:],'g')
            ax3.grid(True)

            ax4 = plt.subplot(4,1,4)
            plt.gca().set_title('Magnitude Force (N) vs time')
            plt.plot(self.time, np.sqrt(self.Fxc[i,:]**2+self.Fzc[i,:]**2),'b')
            ax4.grid(True)
            plt.subplots_adjust(hspace = 1)
            plt.xlabel('time (seconds)')
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all') 
#[magnitude forces from controller]
    def plot_mag_controls(self):
        direct = os.path.join(self.results_dir,'magnitude of Forces')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        plt.figure(figsize=(20,10))
        i=0
        plt.plot(self.time, np.sqrt(self.Fxc[i,:]**2+self.Fzc[i,:]**2),label='bot '+str(i))
        for i in range (1,self.nb):
            plt.plot(self.time, np.sqrt(self.Fxc[i,:]**2+self.Fzc[i,:]**2))
        plt.grid(True)        
        plt.title("Force vs Time (s) for Bot "+self.sim )
        plt.xlabel('time (seconds)')
        plt.legend()
        plt.savefig(direct+'/'+'magnitude_forces'+".png") 
        plt.close('all')
        
     
#[path of bots]  
    def plot_path(self):
        direct = os.path.join(self.results_dir,'path')
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
        Xb=[]
        Zb=[]
        for i in range(self.n1-1):
            Xb.append(sum(self.xpos[:,i])/self.nb)
            Zb.append(sum(self.zpos[:,i])/self.nb)
            
        X=self.xpos[0,:]
        Z=self.zpos[0,:]
        plt.figure(figsize=(20,10))
        plt.plot(X,Z,'b',label="leader_path")
        plt.plot(Xb,Zb,'--r',label="COM path")
        plt.title("path of robot "+self.sim)
        plt.xlabel('x (meters)')
        plt.ylabel('z (meters)')
        plt.grid(True)
        plt.legend()        
        plt.savefig(direct+"/path of robot"+'path_of_robot'+".png") 
        plt.close('all') 

#[Plot error] 
    def plot_error(self):
        direct = os.path.join(self.results_dir,'Shape_error')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        plt.figure(figsize=(15,10))  
        plt.plot(self.time,self.E.T,'b')
        plt.grid(True)
        plt.xlabel('time (seconds)')
        plt.ylabel('Error')
        plt.title('Shape error vs time')
        plt.savefig(direct+"/shape_error.png") 
        plt.close('all') 
        
#[Total force in system ]      
    def plot_total_forces(self):
        direct = os.path.join(self.results_dir,'Total_force')
        if not os.path.isdir(direct):
            os.makedirs(direct)                
        FXS=[]
        FYS=[]
        FZS=[]        
        for i in range(len(self.Fx[0,:])):
            FXS.append(np.sum(self.Fx[:,i]))
            FYS.append(np.sum(self.Fy[:,i]))
            FZS.append(np.sum(self.Fz[:,i]))

        fig=plt.figure(1)
        plt.figure(figsize=(15,10))
    
        fig.suptitle("Total Force vs Time (s) for Bot " )
        ax1 = plt.subplot(3,1,1)
        ax1.grid(True)
        plt.gca().set_title('X Force (N) vs time')
        plt.plot(self.time, FXS,'b')
    
        ax2 = plt.subplot(3,1,2)
        plt.gca().set_title('Y Force (N) vs time')
        plt.plot(self.time,FYS,'r')
        ax2.grid(True)
            
        ax3 = plt.subplot(3,1,3)
        plt.gca().set_title('Z Force (N) vs time')
        plt.plot(self.time, FZS,'tab:green')
        ax3.grid(True)
        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(direct+"/Total Force.png") 
        plt.close('all')
        
#[Ball force]
    def plot_ball_force(self): 
        
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
 
        fig=plt.figure(1)
        plt.figure(figsize=(15,10))
    
        fig.suptitle("Ball Force vs time " )
        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('X  vs time')
        plt.plot(self.time, self.data5,'b')
    
        ax2 = plt.subplot(2,1,2)
        plt.gca().set_title('Z vs time')
        plt.plot(self.time,self.data6,'r')
        ax2.grid(True)

        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(direct+"/ball_force.png") 
        plt.close('all') 

    # abs ball force        
    def plot_abs_ball_force(self): 
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
        
        fig=plt.figure(1)
        plt.figure(figsize=(15,10))
    
        fig.suptitle("Ball abs Force vs time " )
        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('X  vs time')
        plt.plot(self.time, self.FBX,'b')
    
        ax2 = plt.subplot(2,1,2)
        plt.gca().set_title('Z vs time')
        plt.plot(self.time,self.FBZ,'r')
        ax2.grid(True)

        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(direct+"/ball_abs_force.png") 
        plt.close('all')

# ball position        
    def plot_ball_position(self):
        # plots the x and z positon of the ball
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
        plt.figure(figsize=(15,10)) 
        
        plt.plot(self.ballz,self.ballx,'--r')
        plt.title('ball movement')
        plt.xlabel('z (meters)')
        plt.ylabel('x (meters)')
        plt.grid(True)
        plt.savefig(direct+"/ball_position.png") 
        plt.close('all')
        
    def plot_ball_vs_position(self):
        # plots ball position vs time for the z direction
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
        #plt.figure(figsize=(15,10)) 
        fig=plt.figure(1)
        plt.figure(figsize=(15,10))
    
        fig.suptitle("Ball position vs time" )
        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('X  vs time')
        plt.plot(self.time, self.ballz,'b')
        ax1.set_xlabel('time')
        ax1.set_ylabel('ball position')   
           
        ax2 = plt.subplot(2,1,2)
        plt.gca().set_title('Ball force vs ball position')
        plt.plot(self.ballz,self.Fb,'r')
        ax2.grid(True)
        ax2.set_xlabel('ball position')
        ax2.set_ylabel('Ball force')   
        plt.savefig(direct+"/ball_position_vs_force.png") 
        plt.close('all') 
        
        


    def plot_tug_force(self):
        #plots the tug force vs time for the ball
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
        plt.figure(figsize=(15,10)) 
        
        plt.plot(self.time,self.Fb,'--r')
        plt.title('ball movement')
        plt.xlabel('time (seconds)')
        plt.ylabel('Pull force')
        plt.grid(True)
        plt.savefig(direct+"/tug_force.png") 
        plt.close('all')
        
        

    def tug_test(self):
        ''' corresponding to the pull test to see
        the grip strength of the robot'''
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
      
        #plt.figure(figsize=(15,10)) 
        fig=plt.figure(1)
        plt.figure(figsize=(20,15))
    
        ax1 = plt.subplot(4,1,1)
        ax1.grid(True)
        plt.gca().set_title('Ball z position vs time')
        plt.plot(self.time, self.ballz,'tab:blue',label='ball position')

        ax1.set_xlabel('time (seconds)')

           
        ax2 = plt.subplot(4,1,2)
        plt.gca().set_title('tug force vs time')
        plt.plot(self.time,self.Fb,'tab:red')
        ax2.grid(True)   
        ax2.set_xlabel('time (seconds)')
        
        ax3 = plt.subplot(4,1,3)
        plt.gca().set_title('angle vs time ')
        plt.plot(self.time,self.ANGLE,'tab:green')
        ax3.set_xlabel('time (seconds)')
        ax3.grid(True)   
        
        ax4 = plt.subplot(4,1,4)
        plt.gca().set_title('ball position vs tug force')
        plt.plot(self.Fb,self.ballz,'tab:orange')
        ax4.grid(True)   
           
        #plt.tight_layout()
        plt.savefig(direct+"/tugtest.png") 
        plt.savefig(direct+"/tugtest.eps") 
        plt.close('all')        

        
        
        
        
    # def plot_ball_vs_position(self):
    #     direct = os.path.join(self.results_dir)
    #     if not os.path.isdir(direct):
    #         os.makedirs(direct)
            
    #     #plt.figure(figsize=(15,10)) 
    #     fig=plt.figure(1)
    #     plt.figure(figsize=(15,10))
    
    #     fig.suptitle("Ball position vs time" )
    #     ax1 = plt.subplot(2,1,1)
    #     ax1.grid(True)
    #     plt.gca().set_title('X  vs time')
    #     plt.plot(self.time, self.ballz,'b')
    #     ax1.set_xlabel('time')
    #     ax1.set_ylabel('ball position')   
           
    #     ax2 = plt.subplot(2,1,2)
    #     plt.gca().set_title('Ball force vs ball position')
    #     plt.plot(self.ballz,self.Fb,'r')
    #     ax2.grid(True)
    #     ax2.set_xlabel('ball position')
    #     ax2.set_ylabel('Ball force')   
    #     plt.savefig(direct+"/ball_position_vs_force.png") 
    #     plt.close('all')         


    # def plot_pull_test(self):
    #     ''' corresponding to the pull test to see
    #     the grip strength of the robot'''
    #     direct = os.path.join(self.results_dir)
    #     if not os.path.isdir(direct):
    #         os.makedirs(direct)
            
    #     #plt.figure(figsize=(15,10)) 
    #     fig=plt.figure(1)
    #     plt.figure(figsize=(15,10))
    
    #     fig.suptitle("Ball an field  position vs time" )
    #     ax1 = plt.subplot(2,1,1)
    #     ax1.grid(True)
    #     plt.gca().set_title('X  vs time')
    #     plt.plot(self.time, self.ballz,'b',label='ball position')
    #     plt.plot(self.time,self.PX,'r',label='field position')
    #     ax1.set_xlabel('time')
    #     ax1.set_ylabel('position (x)')
    #     ax1.legend()
           
    #     ax2 = plt.subplot(2,1,2)
    #     plt.gca().set_title('Ball mass vs time')
    #     plt.plot(self.time,self.MB,'g')
    #     ax2.grid(True)
    #     ax2.set_xlabel('time')
    #     ax2.set_ylabel('Ball mass')   
    #     plt.savefig(direct+"/pulltest.png") 
    #     plt.close('all')
# ball velocity 
    def plot_ball_velocity(self): 
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
        fig=plt.figure(1)
        plt.figure(figsize=(15,10))
    
        fig.suptitle("Ball velocity vs time " )
        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('X  vs time')
        plt.plot(self.time, self.bvx,'b')
    
        ax2 = plt.subplot(2,1,2)
        plt.gca().set_title('Z vs time')
        plt.plot(self.time,self.bvz,'r')
        ax2.grid(True)

        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(direct+"/ball_velocity.png") 
        plt.close('all')   


        
        

#[force chains] 
    def Forcechains(self):
        direct = os.path.join(self.results_dir,'forcechain')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
   
        cmap = plt.cm.get_cmap('seismic')
        boundaries=np.arange(0,200,30)                
        norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], boundaries[-1]])
        count=0
        for i in range(1,len(self.time)-1):
            Fx=self.Fcx[0:int(self.nc[i]),i]
            Fz=self.Fcz[0:int(self.nc[i]),i]
        #Fy=Fcy[0:nc[i],i]
            abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)
            x=self.xc[0:int(self.nc[i]),i]
            y=self.zc[0:int(self.nc[i]),i]
            x2=[]
            y2=[]
            F2=[]
            for j in range(len(abs_force)):
                if abs_force[j]>=1:
                    x2.append(x[j])
                    y2.append(y[j])
                    F2.append(abs_force[j])
               
  
            if i%1==0:
                count=count+1 
                plt.figure(i,figsize=(8,8),dpi=150)
                plt.scatter(y2,x2,s=np.power(F2,.15),c=F2,cmap=cmap,norm=norm)
                plt.xlim(-2,5)
                plt.ylim(-2,2)
                plt.grid(True)
                plt.colorbar()
                plt.xlabel('x position(m)')
                plt.ylabel('z position(m)')
                plt.title('t='+str(round(self.time[i],3)))
                plt.savefig(direct+"/frame%04d.jpg" % count)  
                print(str(i)+ "of"+ str(len(self.time)))
                plt.close('all')
                
                #plt.savefig(direct+"/frame%04d.jpg" % count)


    def contact_points_plots(self):
        direct = os.path.join(self.results_dir,'forcechain')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
   
        cmap = plt.cm.get_cmap('seismic')
        boundaries=np.arange(0,200,30)                
        norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], boundaries[-1]])
        count=0
        for i in range(1,len(self.time)-1):
            Fx=self.Fcx[0:int(self.nc[i]),i]
            Fz=self.Fcz[0:int(self.nc[i]),i]
        #Fy=Fcy[0:nc[i],i]
            abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)
            x=self.xc[0:int(self.nc[i]),i]
            y=self.zc[0:int(self.nc[i]),i]
            
              
            x2=[]
            y2=[]
            F2=[]
            for j in range(len(abs_force)):
                check=np.sqrt((x[j]-self.ballx[i])**2 + (y[j]-self.ballz[i])**2)
                if check<=self.ball_radius*1.1:
                    x2.append(x[j])
                    y2.append(y[j])
                    F2.append(abs_force[j])
               
  
            if i%1==0:
                count=count+1 
                plt.figure(i,figsize=(8,8),dpi=150)
                plt.scatter(y2,x2,s=np.power(F2,.15),c=F2,cmap=cmap,norm=norm)
                plt.xlim(-2,5)
                plt.ylim(-2,2)
                plt.grid(True)
                plt.colorbar()
                plt.xlabel('x position(m)')
                plt.ylabel('z position(m)')
                plt.title('t='+str(round(self.time[i],3)))
                plt.savefig(direct+"/frame%04d.jpg" % count)  
                print(str(i)+ "of"+ str(len(self.time)))
                plt.close('all')
                
                #plt.savefig(direct+"/frame%04d.jpg" % count)




    def create_video(self,name,directory,export,file):
        for index, filename in enumerate(glob.glob(directory+'/*.'+file)):
            if index>1: break
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            
        out = cv2.VideoWriter(name+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)    
        for filename in glob.glob(directory+'/*.'+file):
            img = cv2.imread(filename)
    
            out.write(img)
        out.release()     
                        

class compare_pressure:
    def __init__(self,nam,path,nb,ni,radius,radius2,height,Rb):
    
        self.path=path
        self.nam=nam
        self.nb=nb
        self.ni=np.sum(ni)
        self.radius=radius
        self.radius2=radius2
        self.Rb=Rb
        
        self.height=height
        
        self.name1='/ALPHA.csv'
        self.name2="/x contact force.csv"
        self.name3="/z contact force.csv"
        self.name4='/bot_position.csv'
        self.name5='/particle_position.csv'
        self.name6="/AN.csv"
        self.name7="/BN.csv"
        self.name8="/nc.csv"
        self.head='robot_data_'
        self.color=['tab:blue','tab:green','tab:red','tab:cyan','tab:orange','tab:purple','tab:brown','tab:olive']
        
        self.TIME=[]
        self.ALPHA=[]
        
        self.XB=[]
        self.ZB=[]
        
        self.XP=[]
        self.ZP=[]
        
        self.FXC=[]
        self.FZC=[]
        
        self.AN=[]
        self.BN=[]
        
        self.NC=[]
        
        self.MAGV=[]
        
        self.filename1=[] 
        self.filename2=[] 
        self.filename3=[]
        self.filename4=[]
        self.filename5=[] 
        self.filename6=[]
        self.filename7=[]
        self.filename8=[]
        
# In[]        
        for i in range(len(self.nam)):
            self.filename1.append(self.head+self.path[i]+self.name1) # alpha
            self.filename2.append(self.head+self.path[i]+self.name2) # x contact forces
            self.filename3.append(self.head+self.path[i]+self.name3) # z contact forces       
            self.filename4.append(self.head+self.path[i]+self.name4) # bot position
            self.filename5.append(self.head+self.path[i]+self.name5) # particle position
            self.filename6.append(self.head+self.path[i]+self.name6) # names of objects A
            self.filename7.append(self.head+self.path[i]+self.name7) # names of objects B
            self.filename8.append(self.head+self.path[i]+self.name8) # number of contacts
        
        for i in range(len(self.nam)):
            self.ALPHA.append(np.genfromtxt(self.filename1[i],delimiter=',')) # alpha
            self.FXC.append(np.genfromtxt(self.filename2[i],delimiter=',')) # x forces
            self.FZC.append(np.genfromtxt(self.filename3[i],delimiter=',')) # z forces
            self.NC.append(np.genfromtxt(self.filename8[i],delimiter=',')) # number of contacts
            
            data = np.genfromtxt(self.filename4[i],delimiter=',') # extract bot postions 
            (m,n)=np.shape(data)
            data=data[:,1:n]
            time=data[0,:] # time for error 1  
            self.TIME.append(time)
            xpos=data[1:self.nb+1,:]
            zpos=data[(2*self.nb)+1:3*self.nb+1,:] 
            self.XB.append(xpos)
            self.ZB.append(zpos)
            
            data = np.genfromtxt(self.filename5[i],delimiter=',') # extract bot postions
            (m,n)=np.shape(data)
            data=data[:,1:n]
            xpos=data[1:self.ni+1,:]
            zpos=data[(2*self.ni)+1:3*self.ni+1,:]
            
            self.XP.append(xpos)
            self.ZP.append(zpos) 
            
            
            an=[]
            infile = open(self.filename6[i], 'r')
            for row in csv.reader(infile):
                an.append(row)
                
            bn=[]
            infile = open(self.filename7[i], 'r')
            for row in csv.reader(infile):
                bn.append(row)    
            
            
            self.AN.append(an)
            self.BN.append(bn)
            
            
         
   
        for i in range(len(self.nam)):
            print(i)
            FXB=np.zeros((self.nb,len(self.TIME[i])))
            FZB=np.zeros((self.nb,len(self.TIME[i])))
            FXP=np.zeros((self.ni,len(self.TIME[i])))
            FZP=np.zeros((self.ni,len(self.TIME[i])))
            
            Fcx=self.FXC[i]
            Fcz=self.FZC[i]
            
            bodyA=self.AN[i]
            bodyB=self.BN[i]
            nc=self.NC[i]
            for j in range(len(self.TIME[i])):
                for l in range(int(nc[j])):
                    temp1=bodyA[j][l]
                    temp2=bodyB[j][l]
                    # INTERIOR PRESSURE
                    if temp1[0:4]=="gran" or temp2[0:4]=="gran":

                        if temp2[0:4]=="gran":
                            les=len(temp2)
                            num=int(temp2[5:les])
                            if temp2[4]=='b' :
                                r=self.radius2*np.sqrt(2)
                            else:
                                r=self.radius2 
                            A = np.pi * 2 * r * self.height                            
                            FXP[num,j]=abs((Fcx[l,j]/A))+FXP[num,j]
                            FZP[num,j]=abs((Fcz[l,j]/A))+FZP[num,j]
                        if temp1[0:4]=="gran":
                            les=len(temp1)
                            num=int(temp1[5:les])
                            if temp1[4]=='b' :
                                r=self.radius2*np.sqrt(2)
                            else:
                                r=self.radius2 
                            A = np.pi * 2 * r * self.height  
                            FXP[num,j]=abs((Fcx[l,j]/A))+FXP[num,j]
                            FZP[num,j]=abs((Fcz[l,j]/A))+FZP[num,j]  
                            
                  
                    # BOTS PRESSURE
                    if temp1[0:3]=="bot" or temp2[0:3]=="bot":
                        if temp2[0:3]=="bot":  
                            les=len(temp2)
                            num=int(temp2[3:les])
                            FXB[num,j]=abs(Fcx[l,j]/(self.radius * 2 * np.pi * self.height))+FXB[num,j]
                            FZB[num,j]=abs(Fcz[l,j]/(self.radius * 2 * np.pi * self.height))+FZB[num,j]
                        else:
                            les=len(temp1)
                            num=int(temp1[3:les])
                            FXB[num,j]=abs(Fcx[l,j]/(self.radius * 2 * np.pi * self.height))+FXB[num,j]
                            FZB[num,j]=abs(Fcz[l,j]/(self.radius * 2 * np.pi * self.height))+FZB[num,j]
                            
                        
            FX=np.vstack([FXB,FZP])
            FZ=np.vstack([FZB,FZP])
            MAG=np.zeros((self.nb+self.ni,len(self.TIME[i])))       
            for j in range(len(self.TIME[i])):
                for k in range(self.nb+self.ni-1):
                    MAG[k,j]=np.sqrt(FZ[k,j]**2+FX[k,j]**2)
                    
            magv=[]
            for m in range(len(self.TIME[i])):  
                magv.append(np.mean(MAG[:,m]))
            
            self.MAGV.append(magv)

    def plot_pressure(self):
        fig=plt.figure(figsize=(8,8))


        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('Pressure vs time')
 
        for i in range(len(self.nam)):
            plt.plot(self.TIME[i],self.MAGV[i],c=self.color[i],label=self.nam[i])
        ax1.set_xlabel('time')
        ax1.set_ylabel('pressure')    
        plt.grid(True)
        plt.legend()
        
        ax2 = plt.subplot(2,1,2)
        ax2.grid(True)
        plt.gca().set_title('Pressure vs alpha')
        for i in range(len(self.nam)):
            plt.plot(self.ALPHA[i],self.MAGV[i],c=self.color[i],label=self.nam[i])
        ax2.set_xlabel('alpha')
        ax2.set_ylabel('pressure')   
        plt.legend()        
        fig.tight_layout()
        plt.savefig('Pressure.jpg')  
        plt.close('all')
        
class compare_error:
    def __init__(self,nam,path):        
        self.path=path
        self.nam=nam
        self.E=[]
        self.TIME=[]
        self.name="/error.csv"
        self.name2='/bot_position.csv'
        self.head='robot_data_'
        self.color=['tab:blue','tab:green','tab:red','tab:cyan','tab:orange','tab:purple','tab:brown','tab:olive']
        self.filename1=[] # error
        self.filename2=[] # time 

        
        for i in range(len(self.nam)):
            self.filename1.append(self.head+self.path[i]+self.name) # error
            self.filename2.append(self.head+self.path[i]+self.name2) # time

        for i in range(len(self.nam)):
            self.E.append(np.genfromtxt(self.filename1[i],delimiter=','))
            self.data3 = np.genfromtxt(self.filename2[i],delimiter=',') # exract time for error 1
            (self.m3,self.n3)=np.shape(self.data3)
            self.data3=self.data3[:,1:self.n3]
            self.time1=self.data3[0,:] # time for error 1  
            self.TIME.append(self.time1)
    def plot_error(self):
        fig=plt.figure(figsize=(12,4))
        ax = fig.add_subplot(1, 3, 1)
        for i in range(len(self.nam)):
            #c=next(self.color)
            plt.plot(self.TIME[i],abs(self.E[i]),c=self.color[i],label=self.nam[i])
        ax.set_xlim([0,9.9])
        ax.set_ylim([0.004,.019])    
        ax.set_title('error plot for change 1', fontname="Arial", fontsize=9)
        ax.set_xlabel("time (seconds)",fontname="Arial", fontsize=9)
        ax.set_ylabel("Error",fontname="Arial", fontsize=9)
        plt.grid(True)
        
        ax = fig.add_subplot(1, 3, 2)
        for i in range(len(self.nam)):
            #c=next(self.color)
            plt.plot(self.TIME[i],abs(self.E[i]),c=self.color[i],label=self.nam[i]) 
        ax.set_xlim([10.001,19.9])
        ax.set_ylim([.4,.9])
        ax.set_title('error plot for change 2', fontname="Arial", fontsize=9)
        ax.set_xlabel("time (seconds)",fontname="Arial", fontsize=9)
        ax.set_ylabel("Error",fontname="Arial", fontsize=9)
        plt.grid(True)
        
        ax = fig.add_subplot(1, 3, 3)
        for i in range(len(self.nam)):
            #c=next(self.color)
            plt.plot(self.TIME[i],abs(self.E[i]),c=self.color[i],label=self.nam[i]) 
        ax.set_xlim([20.001,30])
        ax.set_ylim([0,.15])
        ax.set_title('error plot for change 3', fontname="Arial", fontsize=9)
        ax.set_xlabel("time (seconds)",fontname="Arial", fontsize=9)
        ax.set_ylabel("Error",fontname="Arial", fontsize=9)
        plt.grid(True)
        fig.tight_layout()
        
        plt.legend()
        plt.show()
        #plt.title('comparision of error ')
        #plt.savefig('compare_error.png')
        #plt.savefig('compare_error.eps') 
        #plt.close('all')            
             
#[Compare error]
class compare_grasp:
    def __init__(self,nam,path,ni,nb,radius,radius2,height,Rb):
        self.path=path # path array
        self.nam=nam # name array
        self.ni=np.sum(ni) # number of interiors
        self.nb=nb         # number of bots
        self.radius=radius # radius of bots 
        self.radius2=radius2 # radius of interiors
        self.Rb=Rb # ball radius
        
        self.height=height # height of objects
        
        self.filename1=[] # ball force
        self.filename2=[] # ball position
        self.filename3=[] # position and time
        self.filename4=[] # ALPHA
        self.filename5=[] # x contact force
        self.filename6=[] # z contact force
        self.filename7=[] # particle position
        self.filename8=[] # AN
        self.filename9=[]  # BN
        self.filename10=[] # nc   
        self.filename11=[] # ballx    
        self.filename12=[]
        
        self.FB=[]
        self.BALLZ=[]
        self.BALLX=[]        
        self.TIME=[]
        self.ALPHA=[]
        
        self.XB=[]
        self.ZB=[]
        
        self.XP=[]
        self.ZP=[]
        
        self.FXC=[]
        self.FZC=[]
        
        self.AN=[]
        self.BN=[]
        
        self.NC=[]
        
        self.MAGV=[]        

        self.ANGLE=[]
        
        
        self.XVB=[]
        self.ZVB=[]
        
        
        self.name1='/ballFb.csv'
        self.name2='/ballz.csv'
        self.name3='/bot_position.csv'
        self.name4='/ALPHA.csv'
        self.name5="/x contact force.csv"
        self.name6="/z contact force.csv"
        self.name7='/particle_position.csv'
        self.name8="/AN.csv"
        self.name9="/BN.csv"
        self.name10="/nc.csv"   
        self.name11='/ballx.csv'
        self.name12='/bot_velocity.csv'
        
        self.head='robot_data_'
        
        
        self.color=['tab:blue','tab:green','tab:red','tab:cyan','tab:orange','tab:purple','tab:brown','tab:olive']
        
 
        for i in range(len(self.nam)):
            self.filename1.append(self.head+self.path[i]+self.name1) # ball force
            self.filename2.append(self.head+self.path[i]+self.name2) # ball position
            self.filename3.append(self.head+self.path[i]+self.name3) # position and time
            self.filename4.append(self.head+self.path[i]+self.name4) # ALPHA
            self.filename5.append(self.head+self.path[i]+self.name5) # x contact force
            self.filename6.append(self.head+self.path[i]+self.name6) # z contact force
            self.filename7.append(self.head+self.path[i]+self.name7) # particle position
            self.filename8.append(self.head+self.path[i]+self.name8) # AN
            self.filename9.append(self.head+self.path[i]+self.name9) # BN
            self.filename10.append(self.head+self.path[i]+self.name10) # nc
            self.filename11.append(self.head+self.path[i]+self.name11) # ballx           
            self.filename12.append(self.head+self.path[i]+self.name12) # ballx

        for i in range(len(self.nam)):
            self.FB.append(np.genfromtxt(self.filename1[i],delimiter=','))
            self.BALLZ.append(np.genfromtxt(self.filename2[i],delimiter=','))
            
            self.BALLX.append(np.genfromtxt(self.filename11[i],delimiter=','))              
            self.ALPHA.append(np.genfromtxt(self.filename4[i],delimiter=',')) # alpha
            self.FXC.append(np.genfromtxt(self.filename5[i],delimiter=',')) # x forces
            self.FZC.append(np.genfromtxt(self.filename6[i],delimiter=',')) # z forces
            self.NC.append(np.genfromtxt(self.filename10[i],delimiter=',')) # number of contacts 
            
            
            data = np.genfromtxt(self.filename3[i],delimiter=',') # extract bot postions 
            (m,n)=np.shape(data)
            data=data[:,1:n]
            time=data[0,:] # time for error 1  
            self.TIME.append(time)
            xpos=data[1:self.nb+1,:]
            zpos=data[(2*self.nb)+1:3*self.nb+1,:] 
            self.XB.append(xpos)
            self.ZB.append(zpos)
            
            data = np.genfromtxt(self.filename7[i],delimiter=',') # extract particle postions
            (m,n)=np.shape(data)
            data=data[:,1:n]
            xpos=data[1:self.ni+1,:]
            zpos=data[(2*self.ni)+1:3*self.ni+1,:]
            
            self.XP.append(xpos)
            self.ZP.append(zpos) 
            
            
            
            data = np.genfromtxt(self.filename12[i],delimiter=',') # extract bot postions 
            (m,n)=np.shape(data)
            data=data[:,1:n]
            #time=data[0,:] # time for error 1  
            #self.TIME.append(time)
            xvel=data[1:self.nb+1,:]
            zvel=data[(2*self.nb)+1:3*self.nb+1,:] 
            self.XVB.append(xvel)
            self.ZVB.append(zvel)
            
            
            
            
            an=[]
            infile = open(self.filename8[i], 'r')
            for row in csv.reader(infile):
                an.append(row[1:])
                
            bn=[]
            infile = open(self.filename9[i], 'r')
            for row in csv.reader(infile):
                bn.append(row[1:])
                  
            
            
            self.AN.append(an)
            self.BN.append(bn)
            
            
         
        for i in range(len(self.nam)):
            print(i)
            FXB=np.zeros((self.nb,len(self.TIME[i])))
            FZB=np.zeros((self.nb,len(self.TIME[i])))
            FXP=np.zeros((self.ni,len(self.TIME[i])))
            FZP=np.zeros((self.ni,len(self.TIME[i])))
            
            Fcx=self.FXC[i]
            Fcz=self.FZC[i]
            bodyA=self.AN[i]
            bodyB=self.BN[i]
            nc=self.NC[i]
            xpos=self.XB[i]
            zpos=self.ZB[i]
            ballx=self.BALLX[i]
            ballz=self.BALLZ[i]
            ANGLE=[]
            for j in range(len(self.TIME[i])):
                for l in range(int(nc[j])):
                    temp1=bodyA[j][l]
                    temp2=bodyB[j][l]
                    # INTERIOR PRESSURE
                    if temp1[0:4]=="gran" or temp2[0:4]=="gran":

                        if temp2[0:4]=="gran":
                            les=len(temp2)
                            num=int(temp2[5:les])
                            if temp2[4]=='b' :
                                r=self.radius2*np.sqrt(2)
                            else:
                                r=self.radius2 
                            A = np.pi * 2 * r * self.height                            
                            FXP[num,j]=abs((Fcx[l,j]/A))+FXP[num,j]
                            FZP[num,j]=abs((Fcz[l,j]/A))+FZP[num,j]
                        if temp1[0:4]=="gran":
                            les=len(temp1)
                            num=int(temp1[5:les])
                            if temp1[4]=='b' :
                                r=self.radius2*np.sqrt(2)
                            else:
                                r=self.radius2 
                            A = np.pi * 2 * r * self.height  
                            FXP[num,j]=abs((Fcx[l,j]/A))+FXP[num,j]
                            FZP[num,j]=abs((Fcz[l,j]/A))+FZP[num,j]  
                            
                  
                    # BOTS PRESSURE
                    if temp1[0:3]=="bot" or temp2[0:3]=="bot":
                        if temp2[0:3]=="bot":  
                            les=len(temp2)
                            num=int(temp2[3:les])
                            FXB[num,j]=abs(Fcx[l,j]/(self.radius * 2 * np.pi * self.height))+FXB[num,j]
                            FZB[num,j]=abs(Fcz[l,j]/(self.radius * 2 * np.pi * self.height))+FZB[num,j]
                        else:
                            les=len(temp1)
                            num=int(temp1[3:les])
                            FXB[num,j]=abs(Fcx[l,j]/(self.radius * 2 * np.pi * self.height))+FXB[num,j]
                            FZB[num,j]=abs(Fcz[l,j]/(self.radius * 2 * np.pi * self.height))+FZB[num,j]
            
            
            
            
            for j in range(len(self.TIME[i])):
                            
                temp=[]
                tempx=[]
                tempz=[]
                for k in range(self.nb):
                    q=np.sqrt((xpos[k,j]-ballx[j])**2 + (zpos[k,j]-ballz[j])**2)
                    
                    if q<=(1.1*self.radius + self.Rb):
                        temp.append(j)
                        tempx.append(xpos[k,j])
                        tempz.append(zpos[k,j])
                        
                if len(tempx)==0:
                    ANGLE.append(0)
                else:
                    ang1=np.arctan2((tempx[0]-ballx[j]),(tempz[0]-ballz[j]))
                    ang2=np.arctan2((tempx[-1]-ballx[j]),(tempz[-1]-ballz[j]))
                    ang=((2*np.pi - (abs(ang1)+abs(ang2))))*180 / np.pi
                    
                    ANGLE.append(np.round(ang,3))
            
            
            
            
            self.ANGLE.append(ANGLE)
            
                        
            FX=np.vstack([FXB,FZP])
            FZ=np.vstack([FZB,FZP])
            MAG=np.zeros((self.nb+self.ni,len(self.TIME[i])))       
            for j in range(len(self.TIME[i])):
                for k in range(self.nb+self.ni-1):
                    MAG[k,j]=np.sqrt(FZ[k,j]**2+FX[k,j]**2)
                    
            magv=[]
            for m in range(len(self.TIME[i])):  
                magv.append(np.mean(MAG[:,m]))
            
            self.MAGV.append(magv)
            
            
    def find_tug_force(self,time):
        FORCE=[]
        
        for i in range(len(self.nam)-1): 
            ang=self.ANGLE[i]
            Fb=self.FB[i]
            T=self.TIME[i]
            ang=np.asarray(ang)
            
            res=np.where(ang==0)[0]
            for j in range(len(res)):
                lt=res[j]
                if T[lt]>time:
                    loc=res[j]
                    break
                

            FORCE.append(Fb[loc])
        print(FORCE)
        ypos=np.arange(len(FORCE))
        plt.bar(ypos, FORCE)
        bars=self.nam[0:4]
        plt.xticks(ypos, bars)
        plt.ylabel('Force')
        plt.title('Grasping force for varying alpha')
        plt.grid(True)
        plt.show()   
        plt.savefig("alpha_vs_tug.png") 
        plt.savefig("alpha_vs_tug.eps") 
        plt.close('all')
    
    def smooth(self,y, box_pts):
        ''' smooth the function'''
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth          
    
    
    
    def bar_plot_tug_force(self,time_entry,name):
        entry=[]
        for i in range(len(time_entry)):
            print(time_entry[i])
            temp=self.FB[i]
            entry.append(temp[time_entry[i]])
        np.savez(name+".npz",entry=entry)
        fig = plt.figure()
        ax = fig.add_axes([0.15,0.15,.75,.75])
        langs = [r'$\alpha$=60', r'$\alpha$=80', r'$\alpha$=100']
        ax.bar(langs,entry)   
        #ax.grid(True)
        ax.set_title(r'Grasp Forces vs $\alpha$',fontname="Arial", fontsize=16)
        ax.set_ylabel('Force (N)',fontname="Arial", fontsize=16)
        ax.legend(True)
        
    def bar_plot_tug_force_all_shapes(self):
        data1=np.load("entry1.npz")
        data2=np.load("entry2.npz")
        entry1=data1['entry']
        entry2=data2['entry']
        
        barWidth = 0.25
        fig,ax = plt.subplots(figsize =(12, 8))
        ax.set_title(r"Grasp Force vs $\alpha$",fontname="Arial",fontsize=24)
        ax.set_ylabel("Grasp Force (N)",fontname="Arial",fontsize=16)
        # Set position of bar on X axis
        br1 = np.arange(len(entry1))
        br2 = [x + barWidth for x in br1]
        ax.bar(br1, entry1, color ='tab:red', width = barWidth, label ='circle')
        ax.bar(br2, entry2, color ='tab:blue', width = barWidth, label ='square')
        
        plt.xticks([r + barWidth for r in range(len(entry1))],
        [r'$\alpha$=60', r'$\alpha$=80', r'$\alpha$=100'],fontname="Arial",fontsize=16)       
        ax.legend()
        
    def ball_position_vs_time(self):      
        fig, ax = plt.subplots(figsize=(10,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        # plot 1
        fig.suptitle("Ball Position vs Time" )
        ax.grid(True)
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i], self.BALLZ[i],color=self.color[i],label=self.nam[i])
        ax.set_xlabel('time (seconds)')
        ax.set_ylabel('ball z position')
        #ax.set_ylim([0,3])
        ax.set_ylim([1.6,2.6])
        plt.legend()        
        

    def pull_force_vs_time(self):      
        fig, ax = plt.subplots(figsize=(10,3))
        # plot 1
        fig.suptitle("Pull Force vs Time" )
        ax.grid(True)
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],self.FB[i],color=self.color[i],label=self.nam[i])
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Pull Force (N)')
        #ax.set_ylim([0,3])
        plt.legend()        
        
    def Pull_force_vs_position(self,titl,xticks,yticks): 
        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(1.625, 1.625),dpi=300)
        #fig.subplots_adjust(top=0.85,bottom=0.2,left=0.25,right=0.95,hspace=0,wspace=0)
        fig.subplots_adjust(top=0.88,bottom=0.20,left=0.275,right=0.95,hspace=0,wspace=0)

        # plot 1
        #ax.set_title(titl)
        ax.grid(True)
        # for i in range(len(self.nam)):
        #      ax.plot(self.FB[i],self.BALLZ[i],color=self.color[i],label=self.nam[i])

        #entry = 60
        entry = 25
        
        for i in range(len(self.nam)):
            #ax.plot(self.BALLZ[i],self.FB[i],color=self.color[i],label=self.nam[i])
            #print(i,self.BALLZ[i])
            BZ=self.BALLZ[i]-self.BALLZ[i][entry]
            #print(i,'BZ',BZ,'FB',self.FB[i])
            ax.plot(self.FB[i][entry:-1],BZ[entry:-1],color=self.color[i],label=self.nam[i])
            np.savez(titl+str(i)+'.npz',FB=self.FB[i][entry:-1],BZ=BZ[entry:-1])
        ax.set_xticks(xticks)
        ax.set_yticks(yticks)
        ax.xaxis.set_tick_params(labelsize=8)
        ax.yaxis.set_tick_params(labelsize=8)
        ax.set_xlabel('Pull Force (N)',fontsize=8,labelpad=1)
        ax.set_ylabel('Position (m)',fontsize=8,labelpad=1)
        ax.set_ylim([yticks[0]-.1,yticks[-1]])
        ax.set_xlim([xticks[0]-10,xticks[-1]])
        #ax.legend(prop={'size': 8})             
        #ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.05),ncol=3,prop={'size': 10})
        plt.savefig(titl+"_tug_test.svg") 
        plt.savefig(titl+"_tug_test.pdf") 
        plt.savefig(titl+"_tug_test.png") 
    def internal_pressure_vs_time(self):          
        fig, ax = plt.subplots(figsize=(10,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        # plot 1
        ax.set_title("Pull force vs Position " )
        ax.grid(True)
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],self.smooth(self.MAGV[i],19),color=self.color[i],label=self.nam[i])
        ax.set_ylabel('Pressure (pa)')
        ax.set_xlabel('Time (seconds)')
        #ax.set_ylim([0,3])
        ax.legend()             
        #ax.set_xlim([1.6,3])        
    
        
    def plot_grasp(self):
        fig=plt.figure(1)
        plt.figure(figsize=(10,12))
        # plot 1
        fig.suptitle("Ball z position vs time" )
        ax1 = plt.subplot(5,1,1)
        ax1.grid(True)
        plt.gca().set_title('Z  vs time')
        for i in range(len(self.nam)):
            plt.plot(self.TIME[i], self.BALLZ[i],color=self.color[i],label=self.nam[i])
        ax1.set_xlabel('time (seconds)')
        ax1.set_ylabel('ball z position')
        ax1.set_ylim([0,3])
        plt.legend()
        # plot 2        
        ax2 = plt.subplot(5,1,2)
        plt.gca().set_title('tug force vs time')
        for i in range(len(self.nam)):

            plt.plot(self.TIME[i],self.FB[i],color=self.color[i],label=self.nam[i])
            
        ax2.grid(True)   
        ax2.set_xlabel('time (seconds)')
        ax2.set_ylabel('Tug force')
        plt.legend()
        
        # plot 3       
        ax3 = plt.subplot(5,1,3)
        plt.gca().set_title('ball position vs tug force')
        for i in range(len(self.nam)):
            plt.plot(self.FB[i],self.BALLZ[i],color=self.color[i],label=self.nam[i])
        ax3.grid(True)   
        ax3.set_xlabel('Tug force')
        ax3.set_ylabel('ball z position')
        ax3.set_ylim([0,3])
        plt.legend()  
        
        # plot 4       
        ax4 = plt.subplot(5,1,4)
        plt.gca().set_title('pressure vs time')
        
        for i in range(len(self.nam)):

            y2 = gaussian_filter1d(self.MAGV[i], 2)
            plt.plot(self.TIME[i],self.MAGV[i],color=self.color[i],label=self.nam[i],alpha=0.7)
            plt.plot(self.TIME[i],y2,color=self.color[i],linewidth=2,linestyle='solid')
        ax4.grid(True)   
        ax4.set_xlabel('time')
        ax4.set_ylabel('pressure')
        
        ax4.set_ylim([0,4000])
        plt.legend()          
    
         # plot 5       
        ax5 = plt.subplot(5,1,5)
        plt.gca().set_title('angle vs time')
        for i in range(len(self.nam)):
            plt.plot(self.TIME[i],self.ANGLE[i],color=self.color[i],linewidth=2,linestyle='solid')
            
        ax5.grid(True)   
        ax5.set_xlabel('time')
        ax5.set_ylabel('angle')
        plt.legend()        
    
        plt.tight_layout()
        plt.savefig("tugtest_compare.png") 
        plt.savefig("tugtest_compare.eps") 
        #plt.close('all')      
            





    def plot_ball_vs_position(self):

            
        #plt.figure(figsize=(15,10)) 
        fig=plt.figure(1)
        plt.figure(figsize=(15,10))
    
        fig.suptitle("Ball position vs time" )
        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('X  vs time')
        self.color=iter(cm.rainbow(np.linspace(0,1,(len(self.nam)))))
        
        for i in range(len(self.nam)):
            c=next(self.color)
            plt.plot(self.BALLZ[i],self.TIME[i],c=c,label=self.nam[i]) 
  
        #plt.plot(self.time, self.ballz,'b')
        ax1.set_ylabel('time')
        ax1.set_xlabel('ball position')   
           
        ax2 = plt.subplot(2,1,2)
        plt.gca().set_title('Ball force vs ball position')
        self.color=iter(cm.rainbow(np.linspace(0,1,(len(self.nam)))))
        for i in range(len(self.nam)):
            c=next(self.color)
            plt.plot(self.BALLZ[i],self.FB[i],c=c,label=self.nam[i])      
        #plt.plot(self.ballz,self.Fb,'r')
        ax2.grid(True)
        ax2.set_xlabel('ball position')
        ax2.set_ylabel('Ball force')  
        plt.legend()
        
        plt.savefig('compare_grasp_force_2')  
        #plt.close('all')


    def moving_average(self,a, n=3):
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n    



    def plot_mag_velocity(self,name):
        #direct = os.path.join(self.results_dir,'magnitude of velocity')
        #if not os.path.isdir(direct):
            #os.makedirs(direct)
        fig, ax = plt.subplots(figsize=(5,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        MAGV=[]
        ax.grid(True)

        for i in range(len(self.nam)):
            xvel=self.XVB[i]
            zvel=self.ZVB[i]
            magv=[]
            for j in range(len(self.TIME[i])):
                temp=[]
                for k in range(self.nb):
                    #temp.append(zvel[k,j])
                    temp.append(np.sqrt(xvel[k,j]**2+zvel[k,j]**2))
                magv.append(np.mean(temp))
                
            MAGV.append(magv)
                
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],MAGV[i],color=self.color[i],label=self.nam[i])                
            ax.plot(self.TIME[i][0:len(self.moving_average(MAGV[i]))],self.moving_average(MAGV[i]),color=self.color[i],label=self.nam[i]) 
        ax.set_title(name+"mag")
        #plt.grid(True)        
        #plt.title("velocity vs Time (s) for Bot "+self.sim )
        #plt.xlabel('time (seconds)')
        #plt.ylabel('velocity (m/s)')
        plt.savefig(name+'_avg_magnitude_velocity'+".png") 
    
    
    def plot_compx_velocity(self,name):
        #direct = os.path.join(self.results_dir,'magnitude of velocity')
        #if not os.path.isdir(direct):
            #os.makedirs(direct)
        fig, ax = plt.subplots(figsize=(5,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        MAGV=[]
        ax.grid(True)

        for i in range(len(self.nam)):
            xvel=self.XVB[i]
            zvel=self.ZVB[i]
            magv=[]
            for j in range(len(self.TIME[i])):
                temp=[]
                for k in range(self.nb):
                    #temp.append(zvel[k,j])
                    temp.append(xvel[k,j])
                magv.append(np.sum(temp))
                
            MAGV.append(magv)
                
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],MAGV[i],color=self.color[i],label=self.nam[i])                
            ax.plot(self.TIME[i][0:len(self.moving_average(MAGV[i]))],self.moving_average(MAGV[i]),color=self.color[i],label=self.nam[i],linestyle='dashed') 
        ax.set_title(name+"Xvel")
        #plt.grid(True)        
        #plt.title("velocity vs Time (s) for Bot "+self.sim )
        #plt.xlabel('time (seconds)')
        #plt.ylabel('velocity (m/s)')
        plt.savefig(name+'_sum_x_velocity'+".png") 


    def plot_compz_velocity(self,name):
        #direct = os.path.join(self.results_dir,'magnitude of velocity')
        #if not os.path.isdir(direct):
            #os.makedirs(direct)
        fig, ax = plt.subplots(figsize=(5,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        MAGV=[]
        ax.grid(True)

        for i in range(len(self.nam)):
            xvel=self.XVB[i]
            zvel=self.ZVB[i]
            magv=[]
            for j in range(len(self.TIME[i])):
                temp=[]
                for k in range(self.nb):
                    #temp.append(zvel[k,j])
                    temp.append(zvel[k,j])
                magv.append(np.sum(temp))
                
            MAGV.append(magv)
                
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],MAGV[i],color=self.color[i],label=self.nam[i])                
            ax.plot(self.TIME[i][0:len(self.moving_average(MAGV[i]))],self.moving_average(MAGV[i]),color=self.color[i],label=self.nam[i],linestyle='dashed',linewidth=3) 
        ax.set_title(name+"Zvel")
        #plt.grid(True)        
        #plt.title("velocity vs Time (s) for Bot "+self.sim )
        #plt.xlabel('time (seconds)')
        #plt.ylabel('velocity (m/s)')
        plt.savefig(name+'_sum_z_velocity'+".png") 

    
# [compare path]
class compare_path:
    def __init__(self,path1,path2,nam1,nam2,nb):
        self.path1=path1
        self.path2=path2
        self.nam1=nam1
        self.nam2=nam2
        self.nb = nb
        self.name='/bot_position.csv'
        self.head='robot_data_'
        self.filename1=self.head+self.path1+self.name
        self.filename2=self.head+self.path2+self.name

        self.data1 = np.genfromtxt(self.filename1,delimiter=',') # error 1
        self.data2 = np.genfromtxt(self.filename2,delimiter=',') # error 2

        (self.m1,self.n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:self.n1]
        self.time1=self.data1[0,:]
        self.xpos1=self.data1[1:self.nb+1,:]
        self.zpos1=self.data1[(2*self.nb)+1:3*self.nb+1,:]
        
        (self.m2,self.n2)=np.shape(self.data2)
        self.data2=self.data2[:,1:self.n2]
        self.time2=self.data2[0,:]
        self.xpos2=self.data2[1:self.nb+1,:]
        self.zpos2=self.data2[(2*self.nb)+1:3*self.nb+1,:]
                
        self.Xb1=[]
        self.Zb1=[]
        self.Xb2=[]
        self.Zb2=[]        
        for i in range(self.n1-1):
            self.Xb1.append(sum(self.xpos1[:,i])/self.nb)
            self.Zb1.append(sum(self.zpos1[:,i])/self.nb)
        
        for i in range(self.n2-1):
            self.Xb2.append(sum(self.xpos2[:,i])/self.nb)
            self.Zb2.append(sum(self.zpos2[:,i])/self.nb)

    def plot_position(self):
        plt.figure(figsize=(8,8))
        plt.plot(self.time1,self.Xb1,color='b',label=self.nam1)
        plt.plot(self.time2,self.Xb2,color='r',label=self.nam2)
        plt.grid(True)
        plt.legend()
        plt.title(self.nam1+'vs'+self.nam2+' path')
        plt.savefig(self.path1+' vs '+self.path2+'  '+self.nam1+' vs '+self.nam2+".jpg")  
        plt.close('all')        
        
#[Plotter for comparing robots]       
class compare_cases:
    def __init__(self,cases,variable,nb):
        self.cases=cases
        self.variable=variable
        self.datax={}
        self.dataz={}
        self.cs=['r','g','b']
        self.nb=nb
        self.path='C:/Users/dmulr/OneDrive\Documents/dm-soro_chrono/python/Pychrono/Grabbing/Grab_sim_pot_field_shape/robot_data'
        self.filesx=self.path+str(self.cases[0])+"/X_points_shape.csv"
        self.filesy=self.path+str(self.cases[0])+"/Y_points_shape.csv"

        self.xd = np.genfromtxt(self.filesx,delimiter=',')
        self.yd = np.genfromtxt(self.filesy,delimiter=',')
        
        for i in self.cases:
    
            self.datax["d{0}".format(i)]=[]
            self.dataz["d{0}".format(i)]=[]
    
        for i in self.cases:
            patht=self.path+str(i)+'/'
            name='bot_position.csv'
            filename=patht+name
            data1 = np.genfromtxt(filename,delimiter=',')
            (m1,n1)=np.shape(data1)
            data1=data1[:,1:n1]

            xpos=data1[1:self.nb+1,:]
            ypos=data1[self.nb+1:2*self.nb+1,:]
            zpos=data1[(2*self.nb)+1:3*self.nb+1,:]
    
            zf=zpos[:,-1]
            xf=xpos[:,-1]
            self.datax["d"+str(i)].append(xf)
            self.dataz["d"+str(i)].append(zf)
# compare cases 
    def plot_compare(self):
        count=0
        for i in self.cases:
            print(i)
            plt.scatter(self.dataz["d"+str(i)],self.datax["d"+str(i)],color=self.cs[count],label="alpha="+self.variable[count])
            count=count+1  
        plt.legend(loc='center left')
        plt.grid(True)
    
        
    def plot_compare_shape(self):
        count=0
        for i in self.cases:
            print(i)
            plt.scatter(self.dataz["d"+str(i)],self.datax["d"+str(i)],color=self.cs[count],label="alpha="+self.variable[count])
            count=count+1  
        plt.scatter(self.xd,self.yd,color="black",label="Desired")
        plt.legend(loc='center left')
        plt.grid(True)       
        plt.close('all')
        
#Force chains
class plot_force_chain:
    def __init__(self,data0,data6,data7,data8,data9,data10,data11,data12,results_dir,nb):  
        self.data0=data0
        
        self.time=self.data0[0,:]
        self.xc=data6
        self.yc=data7
        self.zc=data8
        self.Fcx=data9
        self.Fcy=data10
        self.Fcz=data11
        self.nc=data12
        self.nb=nb
        self.results_dir=results_dir
    
    
    def Forcechains(self):
        direct = os.path.join(self.results_dir,'forcechain')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
   
        cmap = plt.cm.get_cmap('seismic')
        boundaries=np.arange(10,30,.1)
                       
        norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
        count=0
        for i in range(1,len(self.time)-1):
            Fx=self.Fcx[0:int(self.nc[i]),i]
            Fz=self.Fcz[0:int(self.nc[i]),i]
        #Fy=Fcy[0:nc[i],i]
            abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)


            x=self.xc[0:int(self.nc[i]),i]
            y=self.zc[0:int(self.nc[i]),i]
            x2=[]
            y2=[]
            F2=[]
            for j in range(len(abs_force)):
                if abs_force[j]>=1:
                    x2.append(x[j])
                    y2.append(y[j])
                    F2.append(abs_force[j])
               
  
            if i%3==0:
                count=count+1 
                plt.figure(i,figsize=(8,8),dpi=150)
                plt.scatter(y2,x2,s=2*np.power(F2,.65),c=F2,cmap=cmap,norm=norm)
                plt.xlim(-.75,1.75)
                plt.ylim(-.75,.75)
                plt.grid(True)
                plt.colorbar()
                plt.xlabel('x position(m)')
                plt.ylabel('z position(m)')
                plt.title('t='+str(round(self.time[i],3)))
                plt.savefig(direct+"/picture"+str(count)+".jpg")  
                print(str(i)+ "of"+ str(len(self.time)))
                plt.close('all')



def create_video(name,directory,export,file):
    img_array = []
    for index, filename in enumerate(glob.glob(directory+'/*.'+file)):
        if index>1: break
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)
 
    out = cv2.VideoWriter(export+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)    
    for filename in glob.glob(directory+'/*.'+file):
        img = cv2.imread(filename)

        out.write(img_array[i])
    out.release()     



class grasping_analysis:
    def __init__(self,filename1,filename2,filename3,filename4,filename5,filename6,filename7,filename8,filename9,filename10,filename11,filename12,filename13,filename14,filename15,filename16,filename17,filename18,filename19,filename20,filename21,filename22,filename23,filename24,result_dir,nb,ni,radius,radius2,Rb,name,width,err,height):    
        self.nb=nb # number of robots
        self.ni=ni # number of interiors
        self.nm=int(self.nb*4)
        self.Rb=Rb
        self.radius=radius
        self.radius2=radius2
        self.radius3=.03/2
        self.height=height
        self.err=0
        ### File direction for specific data ###
        self.filename1=filename1 # name for robot
        self.filename2=filename2 # name for interior
        self.filename3=filename3 # name for radius data
        self.filename4=filename4 # ball x position
        self.filename5=filename5 # ball z position
        self.filename6=filename6  # force controllers   
        self.filename7=filename7 # pull force of ball   
        self.filename8=filename8 # x contact force         
        self.filename9=filename9 # y contact force         
        self.filename10=filename10 # y contact force
        self.filename11=filename11 # AN
        self.filename12=filename12 # BN
        self.filename13=filename13 # x contact point       
        self.filename14=filename14 # y contact point    
        self.filename15=filename15 # z contact point
        self.filename16=filename16 # nc       
        self.filename17=filename17 # phi angle of ball
        self.filename18=filename18 # theta angle of ball
        self.filename19=filename19 # psi angle of ball
        self.filename20=filename20 # contact ID A
        self.filename21=filename21 # contact ID B
        self.filename22=filename22 # bot total force 
        self.filename23=filename23
        self.filename24=filename24
        self.result_dir=result_dir # result directory where to export images
        self.name=name # name of simulation 
        
        self.width=width # width for images
        self.widx=width # width x of images
        self.widy=width # width y of images
        print('0')
        self.data1 = np.genfromtxt(self.filename1,delimiter=',') # extract data for robots
        print('1')
        self.data2 = np.genfromtxt(self.filename2,delimiter=',') # extract data for particles        
        print('2')
        self.data3 = np.load(self.filename3) # radius of each bot interor positon 
        print('3')
        self.data4 = np.genfromtxt(self.filename4,delimiter=',') # extract data ballx
        print('4')
        self.data5 = np.genfromtxt(self.filename5,delimiter=',') # extract data ballz 
        print('5')
        self.data6 = np.genfromtxt(self.filename6,delimiter=',') # force controllers   
        print('6')
        self.data7 = np.genfromtxt(self.filename7,delimiter=',') # extract pull force           
        print('7')
        self.data8 = np.genfromtxt(self.filename8,delimiter=',')  # x contact force    
        print('8')
        self.FCX = self.data8 # rename it this corresponds to control force X
        print('9')
        self.data9 = np.genfromtxt(self.filename9,delimiter=',')  # y contact force
        print('10')
        self.data10 = np.genfromtxt(self.filename10,delimiter=',') # z contact force
        self.FCZ = self.data10 # rename it this corresponds to control force Z
        self.xpos=0
        self.zpos=0
        
        self.xposp=0
        self.zposp=0
        
        self.xposm=0
        self.zposm=0
        
        self.AN=[] # empty array of contact ID A
        print('11')
        infile = open(self.filename11, 'r') # now we fill the array AN
        for row in csv.reader(infile):
            self.AN.append(row[1:])
        print('12')
        self.BN=[] # empty array of contact ID B
        infile = open(self.filename12, 'r') # now we fill the array AN

        for row in csv.reader(infile):
            self.BN.append(row[1:])
        

        print('13')
        self.data13 = np.genfromtxt(self.filename13,delimiter=',') # x contact point
        
        self.xcontact=self.data13  # rename x contact point      
        print('14')
        self.data14 = np.genfromtxt(self.filename14,delimiter=',') # y contact point
        print('15')
        self.data15 = np.genfromtxt(self.filename15,delimiter=',') # z contact point
        self.zcontact=self.data15  # rename z contact point  
        print('16')
        self.data16 = np.genfromtxt(self.filename16,delimiter=',') # nc        
        self.nc=self.data16 # rename nc number of contacts 
        print('17')
        self.data17 = np.genfromtxt(self.filename17,delimiter=',')        
        self.phib=self.data17 # phi angle of ball
        print('18')
        self.data18 = np.genfromtxt(self.filename18,delimiter=',')
        self.thetab=self.data18 # theta angle of ball
        print('19')
        self.data19 = np.genfromtxt(self.filename19,delimiter=',')
        self.psib=self.data19 # psi angle of ball
        
        print('20')
        self.AID = np.genfromtxt(self.filename20,delimiter=',') # ID of contact body  
        print('21')
        self.BID = np.genfromtxt(self.filename21,delimiter=',') # ID of other contact body
        
        self.Rm=self.data3['Rm']  # radiuses of each interior particle needed for bi dispersion
        
        self.ballx=self.data4 # ball x position
        self.ballz=self.data5 # ball z position       
        self.FB=self.data7 # ball pull force
        self.mu=0.4 # friction coefficiet
        print('22')
        self.data22=np.genfromtxt(self.filename22,delimiter=',') # total bot forces
        
        print('23')
        self.data23 = np.genfromtxt(self.filename23,delimiter=',') # extract data for robots
        print('24')
        self.data24 = np.genfromtxt(self.filename24,delimiter=',') # extract data for particle_angles      
        
        self.F2=[]
        self.FGX=[] # empty array of x forces applied on object
        self.FGZ=[] # empty array of z forces applied on object 
        
        self.G=[] # empty array of grasp maps
        
        self.FRAME=[] # empty array of the local frame coordinates
        
        self.GRASP_CON=[] # empty array of indices of robots grasping
        self.GRASP_CONX=[] # empty array of x positions of robots grasping
        self.GRASP_CONZ=[] # empty array of z positions of robots grasping
        
        self.GRASP_FX=[] # empty array of grasping forces x
        self.GRASP_FZ=[] # empty array of grasping forces z
        
        
        self.GRASP_FXT=[]
        self.GRASP_FZT=[]
        
        self.GRASP_FXoT=[]
        self.GRASP_FYoT=[]       
        
        
        self.FRAMEX=[] # empty array of x frame of robots grasping
        self.FRAMEZ=[] # empty array of z frame of robots grasping
        self.FRAMENX=[] # empty array of negative x frame of robots grasping
        
        self.FT=[] # empty array of Forces in contact transformed
        self.FBEFORE=[] # empty arrray of forces in contact before transofmration 
        
        self.FGRASP=[] # grasp force of object without friction
        self.FGRASPMU=[] # grasp force of object with friction
        
        self.CHECK=[] # empty array of checking if G is full rank non friction
        self.CHECKF=[] # empty array of checking if G is full rank for friction
        
        self.Cplus=[] # psotiove friction cone vector
        self.Cminus=[] # negative friction cone vector
        
        self.Equilibrium=[] # empty array showing equilibrium values
        self.S=[] # empty array of the sign for the opposite forces
        
        self.GRASP_FXo=[] # exterior forces x (bots not in contact)
        self.GRASP_FYo=[] # exterior forces y (bots not in contact) 
        
        self.FGRASPTotal=[]
        self.FTot=[]
        
        self.FXo=[] 
        self.FYo=[]
        self.FXoT=[]
        self.FYoT=[]
        self.FBX=[] 
        self.FBZ=[]
        
        self.MAG=[]
        
        self.XC=[] # empty array of contact points of x
        self.ZC=[] # empty array of contact points of z
        
        self.M=[] # empty array of moments
        self.Mo=[]
        
        self.Xb={} # empty list of robot x position
        self.Yb={} # empty list of robot y position 
        
        self.Xp={} # empty list of particle positions
        self.Yp={} # empty list of particle position
        
        self.Xbm={}
        self.Ybm={}
        
        self.Fcx={} # empty list of x controller forc
        self.Fcy={} # empty list of x controller forc
        self.Fcz={} # empty list of z controller force        

        self.Ftotalx={}
        self.Ftotaly={}
        self.Ftotalz={}
  
        self.Pphi={}
        self.Ptheta={}
        self.Ppsi={}
        
    def sort_data(self):
        ''' Sort position data of robots and particles '''
        (m1,n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:n1]
        self.time=self.data1[0,:]
        Xpos=self.data1[1:self.nb+1,:]
        Ypos=self.data1[self.nb+1:2*self.nb+1,:]
        Zpos=self.data1[(2*self.nb)+1:3*self.nb+1,:] 
        self.xpos=Xpos
        self.zpos=Zpos
        
        (m2,n2)=np.shape(self.data2)        
        self.data2=self.data2[:,1:n2]        
        Xposp=self.data2[1:self.ni+1,:]
        Yposp=self.data2[self.ni+1:2*self.ni+1,:]
        Zposp=self.data2[(2*self.ni)+1:3*self.ni+1,:]  
        self.xposp=Xposp
        self.zposp=Zposp
  # In[]      
        (m23,n23)=np.shape(self.data23)
        self.data1=self.data23[:,1:n23]
        Xposm=self.data23[1:self.nm+1,:]
        Yposm=self.data23[self.nm+1:2*self.nm+1,:]
        Zposm=self.data23[(2*self.nm)+1:3*self.nm+1,:] 
        self.xposm=Xposm
        self.zposm=Zposm
        
        (m24,n24)=np.shape(self.data24)
        self.data24=self.data24[:,1:n1]
        self.time=self.data24[0,:]
        Pphi=self.data24[1:self.ni+1,:]
        Ptheta=self.data24[self.ni+1:2*self.ni+1,:]
        Ppsi=self.data24[(2*self.ni)+1:3*self.ni+1,:] 
       

        for i in range(self.ni):
            self.Pphi["Pphi{0}".format(i)]=Pphi[i,:]
            self.Ptheta["Ptheta{0}".format(i)]=Ptheta[i,:]
            self.Ppsi["Ppsi{0}".format(i)]=Ppsi[i,:]
        
        for i in range(self.nb):
            self.Xb["Xb{0}".format(i)]=Xpos[i,:]
            self.Yb["Yb{0}".format(i)]=Zpos[i,:]

        for i in range(self.nm):
            self.Xbm["Xbm{0}".format(i)]=Xposm[i,:]
            self.Ybm["Ybm{0}".format(i)]=Zposm[i,:]
            
        for i in range(self.ni):
            self.Xp["Xp{0}".format(i)]=Xposp[i,:]
            self.Yp["Yp{0}".format(i)]=Zposp[i,:]
            
        
            
    def sort_data_forces(self):
        ''' sort control force  '''
        (m7,n7)=np.shape(self.data6)
        self.data6=self.data6[:,1:n7]
        Fcx=self.data6[1:self.nb+1,:]
        Fcy=self.data6[self.nb+1:2*self.nb+1,:]
        Fcz=self.data6[(2*self.nb)+1:3*self.nb+1,:] 
        
        Ftotalx=self.data22[1:self.nb+1,:]
        Ftotaly=self.data22[self.nb+1:2*self.nb+1,:]
        Ftotalz=self.data22[(2*self.nb)+1:3*self.nb+1,:]
        
        for i in range(self.nb):
            self.Ftotalx["Ftotalx{0}".format(i)]=Ftotalx[i,:]
            self.Ftotaly["Ftotaly{0}".format(i)]=Ftotaly[i,:]
            self.Ftotalz["Ftotalz{0}".format(i)]=Ftotalz[i,:]   
            
            self.Fcx["Fcx{0}".format(i)]=Fcx[i,:]
            self.Fcy["Fcy{0}".format(i)]=Fcy[i,:]
            self.Fcz["Fcz{0}".format(i)]=Fcz[i,:]   
            
    def extract_contact_forces(self):
        ''' extract contact forces on the ball  '''
        for i in range(len(self.time)):
            fbx=[]
            fbz=[]
            cbx=[]
            cbz=[]
            mag=[]
            tempA=self.AID[:,i]
            tempB=self.BID[:,i]            
            for j in range(int(self.nc[i])):
                entryA=tempA[j]
                entryB=tempB[j] 
                if entryB==10000 or entryA==10000:
                    if np.sqrt((self.xcontact[j,i]-self.ballx[i])**2 + (self.zcontact[j,i]-self.ballz[i])**2)> 0.95*self.Rb:
                        mag.append(np.sqrt(self.FCX[j,i]**2 + self.FCZ[j,i]**2))                     
                        fbx.append(self.FCX[j,i])
                        fbz.append(self.FCZ[j,i])
                        cbx.append(self.xcontact[j,i])
                        cbz.append(self.zcontact[j,i])                    

            self.FBX.append(fbx)
            self.FBZ.append(fbz)
            self.XC.append(cbx)
            self.ZC.append(cbz)
            self.MAG.append(mag)    
      
    def find_pressure(self):
        ''' calculate the internal pressure '''
        FXB=np.zeros((self.nb,len(self.time)))
        FZB=np.zeros((self.nb,len(self.time)))
        FXP=np.zeros((self.ni,len(self.time)))
        FZP=np.zeros((self.ni,len(self.time)))
        for i in range(len(self.time)):
            
            for j in range(int(self.nc[i])):
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
                        FXP[num,i]=abs((self.FCX[j,i]/A))+FXP[num,i]
                        FZP[num,i]=abs((self.FCZ[j,i]/A))+FZP[num,i]
                            
  
                    if temp1[0:4]=="gran":
                        if temp1[4]=='b' :
                            r=self.radius2*np.sqrt(2)
                        else:
                            r=self.radius2 
                        A = np.pi * 2 * r * self.height    
                        les=len(temp1)
                        num=int(temp1[5:les])
                        FXP[num,i]=abs((self.FCX[j,i]/A))+FXP[num,i]
                        FZP[num,i]=abs((self.FCZ[j,i]/A))+FZP[num,i]  

                # BOTS PRESSURE
                if temp1[0:3]=="bot" or temp2[0:3]=="bot":
                    if temp2[0:3]=="bot":  
                        les=len(temp2)
                        num=int(temp2[3:les])
                        FXB[num,i]=abs(self.FCX[j,i]/(self.radius * 2 * np.pi * self.height))+FXB[num,i]
                        FZB[num,i]=abs(self.FCZ[j,i]/(self.radius * 2 * np.pi * self.height))+FZB[num,i]
                    else:
                        les=len(temp1)
                        num=int(temp1[3:les])
                        FXB[num,i]=abs(self.FCX[j,i]/(self.radius * 2 * np.pi * self.height))+FXB[num,i]
                        FZB[num,i]=abs(self.FCZ[j,i]/(self.radius * 2 * np.pi * self.height))+FZB[num,i]
                        
                    
        return(FXB,FZB,FZP,FXP)            
    

            
                    
    def create_images_ball(self,geom):
        '''Create the images for the ball'''
        direct = os.path.join(self.result_dir,self.name+'_video_')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0

    
        for i in range(len(self.time)):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(10, 10)
            widx=self.widx
            widy=self.widy
            ax = plt.axes(xlim=(-widx,widx), ylim=(-widy, widy))          
            #ax=plt.axes()
            bots=[]
            Xo=[]
            Yo=[]

            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                Yo.append(y0)
                Xo.append(x0)
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green' 
                patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                    

            #px=self.data5[i] - self.Rb
            #py=self.data4[i] - self.Rb
            #patch = matplotlib.patches.Rectangle((px, py),2*self.Rb, 2*self.Rb,fc='tab:red',edgecolor='tab:red')    
            if geom=="circle":
                patch = plt.Circle((self.data5[i], self.data4[i]),self.Rb, fc='tab:grey',edgecolor='tab:grey',linewidth=1)
                ax.add_patch(patch)            

            if geom=="square":
                const=.5*np.pi*self.Rb
                px=self.data5[i]-const/2 
                py=self.data4[i] -const/2
                patch = matplotlib.patches.Rectangle((px, py),const, const,fc='tab:grey',edgecolor='tab:grey')     
                ax.add_patch(patch)
                
            if geom=="triangle":
                patch = RegularPolygon((self.data5[i], self.data4[i]),3,.4627,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
                ax.add_patch(patch)  
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            print(str(i)+ "of"+ str(len(self.time)))
            plt.close('all')

    def create_images_ball_square(self,geom):
        '''Create the images for the ball'''
        direct = os.path.join(self.result_dir,self.name+'_video_')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0

    
        for i in range(len(self.time)):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(10, 10)
            widx=self.widx
            widy=self.widy
            ax = plt.axes(xlim=(-widx,widx), ylim=(-widy, widy))          
            #ax=plt.axes()
            bots=[]
            Xo=[]
            Yo=[]

            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                theta=self.Ptheta
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                Yo.append(y0)
                Xo.append(x0)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                theta=self.Ptheta['Ptheta'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green' 

                const=self.Rm[j]
                px=x0
                py=y0
                patch = matplotlib.patches.Rectangle((py, px),const, const,fc=c,edgecolor=c,angle=theta*180/np.pi)     
                ax.add_patch(patch)     
                
            #px=self.data5[i] - self.Rb
            #py=self.data4[i] - self.Rb
            #patch = matplotlib.patches.Rectangle((px, py),2*self.Rb, 2*self.Rb,fc='tab:red',edgecolor='tab:red')    
            if geom=="circle":
                patch = plt.Circle((self.data5[i], self.data4[i]),self.Rb, fc='tab:grey',edgecolor='tab:grey',linewidth=1)
                ax.add_patch(patch)            
            

            
            if geom=="square":
                const=.5*np.pi*self.Rb
                px=self.data5[i]-const/2 
                py=self.data4[i] -const/2
                patch = matplotlib.patches.Rectangle((px, py),const, const,fc='tab:grey',edgecolor='tab:grey')     
                ax.add_patch(patch)
                
            if geom=="triangle":
                patch = RegularPolygon((self.data5[i], self.data4[i]),3,.4627,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
                ax.add_patch(patch)  
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            print(str(i)+ "of"+ str(len(self.time)))
            plt.close('all')



    def ball_forces_entry(self,entry,geom):
        ''' shows the forces beign applied '''
        direct = os.path.join(self.result_dir,self.name+'_video_force')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        i=entry
  
        fig = plt.figure(dpi=300)
        fig.set_size_inches(2, 2)

        widx=self.widx
        widy=self.widy
        ax = plt.axes(xlim=(-widx,widx), ylim=(-widy, widy))
        plt.axis('off')
        
        xl=[]
        yl=[]
        
        for j in range(len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]

        #j=0
        #x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]
        #xl.append(x0)
        #yl.append(y0)
        #plt.plot(yl,xl,color="tab:red",zorder=0)
        
        
        #ax=plt.axes()
        bots=[]
        Xo=[]
        Yo=[]
        
        #for j in range(0,len(self.Xbm)):
           # x0,y0=self.Xbm['Xbm'+str(j)][i],self.Ybm['Ybm'+str(j)][i]    
            #patch = plt.Circle((y0, x0),self.radius3-self.err, fc='tab:red')
            #ax.add_patch(patch)   
            #xl.append(x0)
            #yl.append(y0)
        j=0
        x0,y0=self.Xbm['Xbm'+str(j)][i],self.Ybm['Ybm'+str(j)][i]
        xl.append(x0)
        yl.append(y0)
        
        #plt.plot(yl,xl,color="tab:red",zorder=0,linewidth=1)
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((y0, x0),self.radius-self.err, fc='k')
            ax.add_patch(patch)
            
        # for j in range(0,len(self.Xbm)):
        #     x0,y0=self.Xbm['Xbm'+str(j)][i],self.Ybm['Ybm'+str(j)][i]    
        #     patch = plt.Circle((y0, x0),self.radius3-self.err, fc='tab:red')
        #     ax.add_patch(patch)            
            
        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]
            #print(self.Rm[j])
            # if self.Rm[j]==2*.0254:
            #     c='tab:green'
            # else:
            #     c='tab:blue'      
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green' 
    
            patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
            
        if geom=="circle":
            patch = plt.Circle((self.data5[i], self.data4[i]),self.Rb, fc='tab:grey',edgecolor='tab:grey',linewidth=1)
            ax.add_patch(patch)            
        
        if geom=="star":
            
            theta=np.linspace(0,2*np.pi,100)
            R1=.5
            x1=R1*np.cos(theta)
            y1=R1*np.sin(theta)
            R2=.25
            x2=R2*np.cos(theta)
            y2=R2*np.sin(theta)            
            angle_start=np.pi/2
            angle_end=-3*np.pi/2
            theta=np.linspace(angle_start,angle_end,6)
            xp1=R1*np.cos(theta)
            yp1=R1*np.sin(theta)
            const=36*np.pi/180
            angle_start=np.pi/2 
            angle_end=-3*np.pi/2 
            theta=np.linspace(angle_start,angle_end,6)
            xp2=R2*np.cos(theta+const)
            yp2=R2*np.sin(theta+const)
            X=[]
            Y=[]
            
            for ii in range(len(xp2)):
                X.append(xp2[ii]+self.data5[i])
                X.append(xp1[ii]+self.data5[i])
                Y.append(-1*(yp2[ii]+self.data4[i]))
                Y.append(-1*(yp1[ii]+self.data4[i]))           
            
            #ax.fill(X,Y,color='tab:red',alpha=1)        
        
        
        if geom=="square":
            self.Rb=.5*np.pi*self.Rb
            px=self.data5[i] - self.Rb/2
            py=self.data4[i] - self.Rb/2
            patch = matplotlib.patches.Rectangle((px, py),self.Rb,self.Rb,fc='tab:grey',edgecolor='tab:grey')     
            ax.add_patch(patch)
            
            
        if geom=="triangle":
            patch = RegularPolygon((self.data5[i], self.data4[i]),3,.4627,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
            ax.add_patch(patch)
                         
        #plt.xlim([0,2.0])
        plt.axis('equal')
        #patch = plt.Circle((self.data5[i], self.data4[i]),self.Rb*1.1, fc='none',edgecolor='tab:orange',linestyle='--',linewidth=1)        
        #ax.add_patch(patch)
        #print(self.GRASP_CON[i])
        #plt.title('t= ' + str(np.round(self.time[i],3)))


    def plot_pressure(self,FXB,FZB,FZP,FXP):
        #--- plot pressure with points---#       

        FX=np.vstack([FXP])
        FZ=np.vstack([FZP])
        X=np.vstack([self.xposp])
        Z=np.vstack([self.zposp])        
        count=0
        
        self.MAG=np.zeros((self.ni,len(self.time)))
        for j in range(len(self.time)):
            for i in range(self.ni-1):
            
                self.MAG[i,j]=np.sqrt(FZ[i,j]**2+FX[i,j]**2)    

        direct = os.path.join(self.result_dir,self.name+'_pressure_wo_points')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range(len(self.time)):    
            plt.figure(i,figsize=(8,8))
            plt.tricontourf(X[:,i],Z[:,i],self.MAG[:,i],cmap='jet')
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
        #name=self.name+'_pressure_anim_wo_points'
        #self.create_video(name,direct,self.results_dir,'jpg')
        
        
    def smooth(self,y, box_pts):
        ''' smooth the function'''
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth        
        
        
    def ball_angle(self):
        ''' plot the angle  '''
        direct = os.path.join(self.result_dir,self.name+'_force_analysis')    
        if not os.path.isdir(direct):
            os.makedirs(direct)         
        fig, axs = plt.subplots(nrows=3, ncols=1,figsize=(15,10))
        
        # phi angle
        axs[0].plot(self.time,self.phib,color='tab:blue',linewidth=1,label='y')
        axs[0].set_title('phi')
        axs[0].grid(True)   
        # axs[1].set_xlim([2.5,10])  

        # theta angle
        axs[1].plot(self.time,self.thetab,color='tab:green',linewidth=1,label='y')      
        axs[1].set_title('theta')
        axs[1].grid(True)      
         
        # psi angle
        axs[2].plot(self.time,self.psib,color='tab:red',linewidth=1,label='y')
        axs[2].set_title('$\psi$')
        axs[2].grid(True)    
        fig.suptitle('Ball Angle', fontsize=16)     
        plt.savefig(direct+"/ball angles.jpg") 
        plt.close('all')

    def grasp_forces(self):
        ''' plot the grasping forces  '''
        direct = os.path.join(self.result_dir,self.name+'_force_analysis')    
        if not os.path.isdir(direct):
            os.makedirs(direct) 

        for i in range(len(self.time)):
            self.FXo.append(np.sum(self.GRASP_FXo[i]))
            self.FYo.append(np.sum(self.GRASP_FYo[i]))                 
            self.Mo.append(np.sum(self.M[i]))
            
            
        A=np.asarray(self.FGRASP)
        fig, axs = plt.subplots(nrows=3, ncols=1,figsize=(15,10))
        axs[0].plot(self.time,self.smooth(A[:,1],19),color='tab:red',linewidth=1,linestyle='-',label='contact forces')
        axs[0].plot(self.time,self.smooth(self.FYo,19),color='tab:blue',linewidth=1,linestyle='-',label='external forces') 
        axs[0].plot(self.time,self.smooth(self.FYo+A[:,1],19)+self.FB,color='tab:green',linewidth=1,linestyle='-',label='net sum') 
        axs[0].plot(self.time,self.FB,color='tab:grey',linewidth=1,linestyle='-',label='ball force')
        axs[0].set_title('X force')
        axs[0].grid(True)   
        #axs[0].set_ylim([-200,200])
        axs[0].legend()
        
        axs[1].plot(self.time,self.smooth(A[:,0],19),color='tab:red',linewidth=1,linestyle='-',label='contact forces')
        axs[1].plot(self.time,self.smooth(self.FXo,19),color='tab:blue',linewidth=1,linestyle='-',label='external forces')   
        axs[1].plot(self.time,self.smooth(self.FXo+A[:,0],19),color='tab:green',linewidth=1,linestyle='-',label='net sum')          
        axs[1].set_title('Y force')
        axs[1].grid(True)    
        axs[1].legend()        
    
        axs[2].plot(self.time,self.smooth(A[:,2],19),color='tab:red',linewidth=1,linestyle='-',label='contact moment')
        axs[2].plot(self.time,self.smooth(self.Mo,19),color='tab:blue',linewidth=1,linestyle='-',label='external moment')
        axs[2].plot(self.time,self.smooth(self.Mo+A[:,2],19),color='tab:green',linewidth=1,linestyle='-',label='net sum')        
        axs[2].set_title('Moments')
        axs[2].grid(True) 
        axs[2].legend()
        
        fig.suptitle('Grasp Forces', fontsize=16)     
        plt.savefig(direct+"/grasp_forces.jpg")
        plt.close('all')

    def control_forces(self):
        ''' plot the control forces '''
        direct = os.path.join(self.result_dir,self.name+'_force_analysis')    
        if not os.path.isdir(direct):
            os.makedirs(direct)         
        fig, axs = plt.subplots(nrows=3, ncols=1,figsize=(15,10))
        FX=[]
        FY=[]
        for i in range(len(self.time)):
            fxtemp=[]
            fytemp=[]
            for j in range(0,len(self.Xb)):   
                Fx,Fy=self.Fcx['Fcx'+str(j)][i],self.Fcz['Fcz'+str(j)][i]   
                fxtemp.append(Fx)
                fytemp.append(Fy)
            FX.append(np.sum(fxtemp))
            FY.append(np.sum(fytemp)) 
               
        axs[0].plot(self.time,FX,color='tab:red',linewidth=1,label='x')
        axs[0].set_title('X force')
        axs[0].grid(True)        
        # axs[0].set_xlim([2.5,10])  
        
        axs[1].plot(self.time,FY,color='tab:blue',linewidth=1,label='y')
        axs[1].set_title('Y force')
        axs[1].grid(True)   
        # axs[1].set_xlim([2.5,10])  
        
        axs[2].plot(self.time,self.FB,color='tab:green',linewidth=1,label='ball')
        axs[2].set_title('Ball force')
        axs[2].grid(True)  
        # axs[2].set_xlim([2.5,10]) 
        fig.suptitle('Control forces', fontsize=16)         
        plt.savefig(direct+"/control forces.jpg")    
        plt.close('all')
        

class create_videos_irr:
    def __init__(self,filename,filename2,filename3,filename4,filename5,filename6,filename7,filename8,filename9,result_dir,nb,ni,mode,R,radius,radius2,name,width,err,tim,Rb):

        self.nb=nb # number of robots
        self.ni=ni # number of interiors      
        self.nm=int(self.nb*5)
        self.mode=mode        
        self.R=R        
        self.radius=radius
        self.radius2=radius2
        self.radius3=.03/2
        self.ex=5
        self.err=err
        self.widx=width
        self.widy=width
        self.name=name
        self.tim=tim
        self.Rb=Rb
        self.mu=0.4
        self.FGX=[] # empty array of x forces applied on object
        self.FGZ=[] # empty array of z forces applied on object 
        self.G=[]
        self.FRAME=[]    
        self.GRASP_CON=[]
        self.GRASP_CONX=[]
        self.GRASP_CONZ=[]  
        self.GRASP_FX=[] # array of grasping forces
        self.GRASP_FZ=[] 
        self.FRAMEX=[]
        self.FRAMEZ=[]
        self.FT=[]
        self.FBEFORE=[]
        self.FGRASP=[]
        self.CHECK=[]
        self.CHECKF=[]
        self.Cplus=[]
        self.Cminus=[]
        self.Xm=0
        self.Ym=0
        self.Tm=0
        self.xp=0
        self.yp=0
        self.markerx = 0
        self.markery = 0
        self.filename=filename # name for robot
        self.filename2=filename2 # name for interior
        self.filename3=filename3# outline of desried shape
        self.filename4=filename4 # name for radius data
        self.filename5=filename5 # ball x position
        self.filename6=filename6 # ball z position
        self.filename7=filename7  # force controllers   
        self.filename8=filename8 # pull force of ball
        self.filename9=filename9
        self.data1 = np.genfromtxt(self.filename,delimiter=',') # extract data for robots
        self.data2 = np.genfromtxt(self.filename2,delimiter=',') # extract data for particles
        if self.filename9!=None:
            self.data9 = np.genfromtxt(self.filename9,delimiter=',') 
        # if there is no outline of shape this is for grasping 
        if self.filename3!=None:
            # self.data3 = np.load(self.filename3)
            self.data3 = np.load(self.filename3,mmap_mode='r',allow_pickle=True)
            # print(self.data3)
            self.XT=self.data3["XT"]
            self.YT=self.data3["YT"]
            #self.XT[3]=np.hstack([self.XT[3],self.XT[3][0]])
            #self.YT[3]=np.hstack([self.YT[3],self.YT[3][0]])  
            #x=[0,.56,.66,1,1,.75,.75,1,1,.66,.56,0,0]
            #y=[.125,.125,0,0,.125,.125,.375,.375,.5,.5,.375,.375,.125]
            #x=np.dot(-2.0,x)
            #y=np.dot(2.0,y)
            #xp=np.sum(x)/len(x)
            #yp=np.sum(y)/len(y)    
            #self.XT=x-xp+.1
            #self.YT=y-yp
            #self.XT=np.hstack([self.XT,self.XT[0]])
            #self.YT=np.hstack([self.YT,self.YT[0]])
            #self.data3 = np.load(self.filename3,mmap_mode='r',allow_pickle=True)

        #radius of each interior more important for bi dispersion
        self.data4=np.load(self.filename4) 
        self.Rm=self.data4['Rm'] 
        
        # if there is a ball
        if self.filename5!=None:
            self.data5 = np.genfromtxt(self.filename5,delimiter=',') # extract data ballx
            self.data6 = np.genfromtxt(self.filename6,delimiter=',') # extract data ballz  
            self.ballx=self.data5 # ball x position
            self.ballz=self.data6 # ball z position 
            self.data8 = np.genfromtxt(self.filename8,delimiter=',') # extract pull force           
            self.FB=self.data8
        #   If there is controller force
        if self.filename7!=None:
            self.data7 = np.genfromtxt(self.filename7,delimiter=',')      

        self.result_dir=result_dir # result directory where to export images

        self.N=[]
        self.Xb={} # empty list of robot x position
        self.Yb={} # empty list of robot y position 
        
        self.Xbm={} # empty list of robot x position
        self.Ybm={} # empty list of robot y position         
        
        self.Xp={} # empty list of particle positions
        self.Yp={} # empty list of particle position
        
        self.Fcx={} # empty list of x controller forc
        self.Fcy={} # empty list of x controller forc
        self.Fcz={} # empty list of z controller force
        
    def sort_data(self):
        # Sort position data of robots and particles #
        (m1,n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:n1]
        self.time=self.data1[0,:]
        Xpos=self.data1[1:self.nb+1,:]
        Ypos=self.data1[self.nb+1:2*self.nb+1,:]
        Zpos=self.data1[(2*self.nb)+1:3*self.nb+1,:] 
        
        (m2,n2)=np.shape(self.data2)        
        self.data2=self.data2[:,1:n2]        
        Xposp=self.data2[1:self.ni+1,:]
        Yposp=self.data2[self.ni+1:2*self.ni+1,:]
        Zposp=self.data2[(2*self.ni)+1:3*self.ni+1,:]  
        if self.filename9!=None:
            (m9,n9)=np.shape(self.data9)
            self.data9=self.data9[:,1:n9]
            Xposm=self.data9[1:self.nm+1,:]
            Yposm=self.data9[self.nm+1:2*self.nm+1,:]
            Zposm=self.data9[(2*self.nm)+1:3*self.nm+1,:] 
            for i in range(self.nm):
                self.Xbm["Xbm{0}".format(i)]=Xposm[i,:]
                self.Ybm["Ybm{0}".format(i)]=Zposm[i,:]
        for i in range(self.nb):
            self.Xb["Xb{0}".format(i)]=Xpos[i,:]
            self.Yb["Yb{0}".format(i)]=Zpos[i,:]

        
        for i in range(self.ni):
            self.Xp["Xp{0}".format(i)]=Xposp[i,:]
            self.Yp["Yp{0}".format(i)]=Zposp[i,:]

            
    def sort_data_forces(self):
        # sort control force  #
        (m7,n7)=np.shape(self.data7)
        self.data7=self.data7[:,1:n7]
        Fcx=self.data7[1:self.nb+1,:]
        Fcy=self.data7[self.nb+1:2*self.nb+1,:]
        Fcz=self.data7[(2*self.nb)+1:3*self.nb+1,:] 
        
        for i in range(self.nb):
            self.Fcx["Fcx{0}".format(i)]=Fcx[i,:]
            self.Fcy["Fcy{0}".format(i)]=Fcy[i,:]
            self.Fcz["Fcz{0}".format(i)]=Fcz[i,:]
    

    def import_beacon(self):
        data=np.load('Marker.npz')
        self.Xm=data['Xm']
        self.Ym=data['Ym']
        self.Tm=data['time']
        self.markerx =interp1d(self.Tm,self.Xm)
        self.markery =interp1d(self.Tm,self.Ym)

    def change_marker(self,time):
        print(time)
        res=np.where(self.Tm==np.round(time,2))
        print(res)
        if len(res[0])==0:
            #print('na')
            x=self.xp
            y=self.yp
        else:
            x=self.Xm[res[0][0]]
            y=self.Ym[res[0][0]]
            self.xp=x
            self.yp=y
        #print(x,z)
        return(x,y)        
        
    def create_images_verification(self):
        # Create images for the letters #
        direct = os.path.join(self.result_dir,self.name+'_video_verify')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=150)
            fig.set_size_inches(10, 10)
            xp=self.markerx(self.time[i])
            yp=self.markery(self.time[i])
            
            ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
            x0,y0=xp*.0254,yp*.0254    
            patch = plt.Circle((y0, x0),.03, fc='tab:red')
            ax.add_patch(patch)
            #ax=plt.axes()
            bots=[]
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==1.5*.0254:
                    c='tab:blue'
                if self.Rm[j]==2*.0254:
                    c='tab:green'
                patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)  
                
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')                 
    def create_images_nhomomax_snap_shot(self,N):
        # bidispersion of shape with a snap shot #        
        i=N
        fig = plt.figure(dpi=300,figsize=(2,2))
        ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
        bots=[]
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((x0, y0),self.radius-self.err, fc='black')
            ax.add_patch(patch)
            
        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]
            if self.Rm[j]==2*.0254:
                c='tab:blue'
            else:
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
            # if self.time[i]<10:
            #     plt.plot(self.XT[1],self.YT[1],color='tab:red',linestyle='dashed',linewidth=2) 
            # if  self.time[i]>10 and self.time[i]<20:
            #     plt.plot(self.XT[2],self.YT[2],color='tab:red',linestyle='dashed',linewidth=2)
            # if  self.time[i]>20:
            #     plt.plot(self.XT[3],self.YT[3],color='tab:red',linestyle='dashed',linewidth=2)  
        #plt.xlabel('x meters',fontname="Arial", fontsize=8)
        #plt.ylabel('y meters',fontname="Arial", fontsize=8)
        plt.title('Time= ' + str(np.round(self.time[i],0)),fontname="Arial", fontsize=9)
                 

    def create_images_nmax_letter(self):
        # Create images for the letters #
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=150)
            fig.set_size_inches(10, 10)
            
            ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
            #ax=plt.axes()
            bots=[]
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green'
                patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                # J
                if self.time[i]<self.tim[0]:
                    plt.plot(self.XT[1],self.YT[1],color='tab:red',linestyle='dashed',linewidth=1,zorder=0) 
                # A    
                if  self.time[i]>self.tim[0] and self.time[i]<self.tim[1]:
                    plt.plot(self.XT[2],self.YT[2],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)
                # M    
                if  self.time[i]>self.tim[1] and self.time[i]<self.tim[2]:
                    plt.plot(self.XT[3],self.YT[3],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)
                # O   
                if  self.time[i]>self.tim[2] and self.time[i]<self.tim[3]:
                    plt.plot(self.XT[4],self.YT[4],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)                    
                # E   
                if  self.time[i]>self.tim[3] and self.time[i]<self.tim[4]:
                    plt.plot(self.XT[5],self.YT[5],color='tab:red',linestyle='dashed',linewidth=1,zorder=0) 
                # B    
                if  self.time[i]>self.tim[4] and self.time[i]<self.tim[5]:
                    plt.plot(self.XT[6],self.YT[6],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)                      
                # A
                if  self.time[i]>self.tim[5]:
                    plt.plot(self.XT[7],self.YT[7],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)                    
                
                #plt.plot(self.yd,self.xd,color='red',linestyle='dashed',linewidth=2) 
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')           

    def create_images_nmax_letter_snap_shot(self,entry):
        # Create images for specific leter #
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        i=entry
 

        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(2, 2),dpi=300)
        
        #fig.subplots_adjust(top=0.85,bottom=0.2,left=0.25,right=0.90,hspace=0,wspace=0)
        fig.subplots_adjust(top=0.9,bottom=0.049,left=0.001,right=1,hspace=0,wspace=0)
        #plt.axis('off')
        #ax.axis('equal')
        ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
        ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
        plt.axis('off')
        #fig.subplots_adjust(top=0.9,bottom=0.049,left=0.001,right=1,hspace=0.3,wspace=0.2)
        #ax=plt.axes()
        bots=[]
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
            ax.add_patch(patch)
            
        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green'
            patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
        ax.set_aspect('equal', 'box')

        plt.savefig(str(entry)+'.svg')

        #plt.close('all')           


                                  
    def create_images_nmax_1shape(self):
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        nn=1
        tt=self.tim[nn]
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(10, 10)
            
            ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
            #ax=plt.axes()
            bots=[]
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green'
                patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                plt.plot(self.XT,self.YT,color='tab:red',linestyle='dashed',linewidth=2) 

            #plt.plot(self.XT,self.YT,color='tab:red',linestyle='dashed',linewidth=1) 
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            plt.close('all')

                                  
    def create_images_morphing(self):
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        nn=1
        tt=self.tim[nn]
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(10, 10)
            
            ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
            #ax=plt.axes()
            bots=[]
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((x0, y0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                #plt.plot(self.YT,self.XT,color='tab:red',linestyle='dashed',linewidth=2) 
                if self.time[i]<10:
                    plt.plot(self.XT[1],self.YT[1],color='tab:red',linestyle='dashed',linewidth=2) 
                if  self.time[i]>10 and self.time[i]<20:
                    plt.plot(self.XT[2],self.YT[2],color='tab:red',linestyle='dashed',linewidth=2)
                if  self.time[i]>20:
                    plt.plot(self.XT[3],self.YT[3],color='tab:red',linestyle='dashed',linewidth=2)  
                    
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            plt.close('all')

    def create_images_morphing_snap_shot(self,entry,xticks,yticks):
        count=0
        nn=1
        tt=self.tim[nn]
        i=entry
        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(2, 2),dpi=300)
        
        fig.subplots_adjust(top=0.85,bottom=0.2,left=0.25,right=0.90,hspace=0,wspace=0)
        #fig.subplots_adjust(top=0.85,bottom=0.3,left=0.30,right=0.85,hspace=0,wspace=0)
        #fig.subplots_adjust(top=0.9,bottom=0.049,left=0.001,right=1,hspace=0,wspace=0)
        #plt.axis('off')
        #ax.axis('equal')
        ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
        #ax = plt.axes(xlim=(-1,2), ylim=(-1.5, 1.5))
        bots=[]
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((x0, y0),self.radius-self.err, fc='black')
            ax.add_patch(patch)
   
        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
            #plt.plot(self.YT,self.XT,color='tab:red',linestyle='dashed',linewidth=2) 
            if self.time[i]<10:
                plt.plot(self.XT[1],self.YT[1],color='tab:red',linestyle='dashed',linewidth=1,zorder=0) 
            if  self.time[i]>10 and self.time[i]<20:
                plt.plot(self.XT[2],self.YT[2],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)

            if  self.time[i]>20:
                plt.plot(self.XT[3],self.YT[3],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)  
                 

        ax.set_xticks(xticks)
        ax.set_yticks(yticks)
        ax.xaxis.set_tick_params(labelsize=8)
        ax.yaxis.set_tick_params(labelsize=8)
        ax.set_xlabel('$x$ (Meters)',labelpad=1,fontsize=8)
        ax.set_ylabel('$y$ (Meters)',labelpad=1,fontsize=8)
        ax.set_title(str(np.round(self.time[i],1))+" s")
        


    def create_images_nmax_snapshot_1shape(self,entry,titl,xticks,yticks):
        count=0
        nn=1
        tt=self.tim[nn]
        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(2.16, 2.16),dpi=300)
        fig.subplots_adjust(top=0.85,bottom=0.3,left=0.30,right=0.85,hspace=0,wspace=0)
        #plt.axis('off')
        ax.axis('equal')
        i=entry
        #ax=plt.axes()
        bots=[]
        xl=[]
        yl=[]
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
            ax.add_patch(patch)
            xl.append(x0)
            yl.append(y0)
            
            
        # for j in range(0,len(self.Xbm)):
        #     x0,y0=self.Xbm['Xbm'+str(j)][i],self.Ybm['Ybm'+str(j)][i]    
        #     patch = plt.Circle((x0, y0),self.radius3-self.err, fc='tab:red')
        #     ax.add_patch(patch)
  
        #j=0
        #i=entry
        #x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]
        #xl.append(x0)
        #yl.append(y0)
        #plt.plot(xl,yl,color="tab:red",zorder=0)            
        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]
            # if np.round(self.Rm[j],3)==self.radius2*np.sqrt(2):
            #     c='tab:green'
            # if np.round(self.Rm[j],3)==self.radius2:
            #     c='tab:blue'
            #print(self.Rm[j])
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green'                
            patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
            
        plt.savefig(str(entry)+'.svg')
        #y_ticks = np.linspace(-1, 1,3,endpoint=True)
        #x_ticks = np.linspace(-1,1,3,endpoint=True)
        
        
        ax.set_xticks(xticks)
        ax.set_yticks(yticks)
        ax.xaxis.set_tick_params(labelsize=8)
        ax.yaxis.set_tick_params(labelsize=8)
        ax.set_xlabel('$x$ (Meters)',labelpad=1,fontsize=8)
        ax.set_ylabel('$y$ (Meters)',labelpad=1,fontsize=8)  
        ax.plot(self.XT,self.YT,color='tab:red',linestyle='dashed',zorder=0) 
        ax.set_title(titl)
        #plt.savefig()
        # ax.set_xlim(-.45,.45)
        # ax.set_ylim(-.45,.45)
        count=count+1 
        plt.savefig(titl+'.svg')
        #plt.close('all')


    def create_images_nmax_tunnel(self):
        # Create images for the letters #
        direct = os.path.join(self.result_dir,self.name+'_tunnel_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        
        ycenter=-4
        xcenter=8
        
        ranged=10
        
        count=0
        
        xtop=xcenter+ranged
        xbottom=xcenter-ranged
        ytop=ycenter+ranged
        ybottom=ycenter-ranged
        
        # px=14
        # py=-2.5
        # num=100
        # xp=np.linspace(xbottom,xtop,50)
        # yp=np.linspace(ybottom,ytop,50)
        # xx,yy=np.meshgrid(xp,yp)
        # r=np.sqrt((xx-px)**2+(yy-py)**2)
        # zz=r
        # zz=zz/np.max(zz)
        # (fy,fx)=np.gradient(zz)
        # fx=fx/np.max(fx)
        # fy=fy/np.max(fy)

        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=150)
            fig.set_size_inches(10, 10)
            
            ax = plt.axes(xlim=(xbottom,xtop), ylim=(ybottom,ytop))
            #ax=plt.axes()
            bots=[]
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((-x0, y0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if np.round(self.Rm[j],3)==.04:
                    c='tab:blue'
                else:
                    c='tab:green' 
                patch = plt.Circle((-x0, y0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                
            c1=0
            c2=10

        
            # x2=np.linspace(c1,c2,100)
            # x1=np.linspace(c1,c2,100)                
            # line1=np.tanh(x1-5)-3.75
            # line2=np.tanh(x2-5)-2.25 
            # plt.fill_between(x1,line2,color='tab:grey',alpha=1)
            # plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=1)            
            # plt.plot(x1,line1,color='k',linewidth=2)
            # plt.plot(x2,line2,color='k',linewidth=2)
            
            
            x2=np.linspace(c1,c2,1000)
            x1=np.linspace(c1,c2,1000)                
            line1=np.sin(1.745*x1)-3.75
            line2=np.sin(1.745*x1)-2.0
            plt.fill_between(x1,line2,color='tab:grey',alpha=1)
            plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=1)      
            plt.plot(x1,line1,color='k',linewidth=2)
            plt.plot(x2,line2,color='k',linewidth=2)            
            
            # plt.hlines(y=0, xmin=0, xmax=10, linewidth=2, color='k')
            # plt.hlines(y=-6.25, xmin=0, xmax=10, linewidth=2, color='k') 
            # plt.vlines(x=0, ymin=-3.00, ymax=0, linewidth=2, color='k')
            # plt.vlines(x=0, ymin=-6.25, ymax=-5, linewidth=2, color='k')
            # plt.vlines(x=10, ymin=-6.25, ymax=-3.00, linewidth=2, color='k')
            # plt.vlines(x=10, ymin=-1, ymax=0, linewidth=2, color='k')
      
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')      

   
    def create_images_nmax_tunnel_entry(self,entry):
        # Create images for the letters #

        xl=[]
        yl=[]

        ranged=3
        
        count=0
        
        
        px=14
        py=-2.5
        num=100
        i=entry

        const=2.66   
        fx=6
        fy=fx/const
        fig, ax = plt.subplots(figsize=(fx,fy),dpi=300)
        fig.subplots_adjust(top=1,bottom=0.0,left=0.0,right=1,hspace=0,wspace=0)
        #ax = plt.axes(xlim=(-3,15), ylim=(-13,5))
        #ax = plt.axes(xlim=(-3,15.5), ylim=(-6.35,.15))
        plt.axis('off')
        #plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().set_aspect('equal')
        #plt.axis('equal')
        #ax=plt.axes()
        bots=[]

        


        for k in range(len(entry)):
            xt,yt=self.Xb['Xb'+str(0)][entry[k]],self.Yb['Yb'+str(10)][entry[k]]   
            #ax.text(-xt, -.5, 'T= ' + str(np.round(self.time[entry[k]],2))+"(s)", fontsize = 12)
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][entry[k]],self.Yb['Yb'+str(j)][entry[k]]    
                patch = plt.Circle((-x0, y0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
    
                
            # j=0
            # i=entry
            # x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]
            # xl.append(-x0)
            # yl.append(y0)
            # ax.plot(xl,yl,color="tab:red",zorder=0)       
            
            # for j in range(0,len(self.Xbm)):
            #     x0,y0=self.Xbm['Xbm'+str(j)][i],self.Ybm['Ybm'+str(j)][i]    
            #     patch = plt.Circle((-x0, y0),self.radius3-self.err, fc='tab:red')
            #     ax.add_patch(patch)
            #     xl.append(-x0)
            #     yl.append(y0)
                
            # j=0
            # i=entry
            # x0,y0=self.Xbm['Xbm'+str(j)][i],self.Ybm['Ybm'+str(j)][i]
            # xl.append(-x0)
            # yl.append(y0)           
            # ax.plot(xl,yl,color="tab:red",zorder=0)    
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][entry[k]]
                y0=self.Yp['Yp'+str(j)][entry[k]]
                if np.round(self.Rm[j],3)==.04:
                    c='tab:blue'
                else:
                    c='tab:green' 
                    
                patch = plt.Circle((-x0, y0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                
        c1=0
        c2=10
        
        # x2=np.linspace(c1,c2,1000)
        # x1=np.linspace(c1,c2,1000)                
        # line1=np.sin(1.745*x1)-3.75
        # line2=np.sin(1.745*x1)-2.0
        # plt.fill_between(x1,line2,color='tab:grey',alpha=0.5)
        # plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=0.5)
        # plt.plot(x1,line1,color='k')
        # plt.plot(x2,line2,color='k')
        # plt.hlines(y=0, xmin=0, xmax=10, color='k')
        # plt.hlines(y=-6.25, xmin=0, xmax=10,color='k') 
        # plt.vlines(x=0, ymin=-3.2, ymax=0,color='k')
        # plt.vlines(x=0, ymin=-6.25, ymax=-4.75,color='k')
        # plt.vlines(x=10, ymin=-6.25, ymax=-2.75,color='k')
        # plt.vlines(x=10, ymin=-1.25, ymax=0,color='k')         
        
        x2=np.linspace(c1,c2,100)
        x1=np.linspace(c1,c2,100)                
        line1=np.tanh(x1-5)-3.75
        line2=np.tanh(x2-5)-2.25 
        
        plt.fill_between(x1,line2,color='tab:grey',alpha=0.5)
        plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=0.5)            
        plt.plot(x1,line1,color='k')
        plt.plot(x2,line2,color='k')
        
        
        #plt.hlines(y=0, xmin=0, xmax=10, color='k')
        #plt.hlines(y=-6.25, xmin=0, xmax=10,color='k') 
        #plt.vlines(x=0, ymin=-3.2, ymax=0,color='k')
        #plt.vlines(x=0, ymin=-6.25, ymax=-4.75,color='k')
        #plt.vlines(x=10, ymin=-6.25, ymax=-2.75,color='k')
        #plt.vlines(x=10, ymin=-1.25, ymax=0,color='k')   
        #plt.scatter(15,-2.5,marker="*",c="tab:cyan",s=100)
        #plt.title('Time = ' + str(np.round(self.time[i],2)) +" (S)")
        plt.savefig("Tunnel_snap_shots.svg")  
        plt.savefig("Tunnel_snap_shots.png")  
        #plt.savefig("tunnel "+str(entry)+".jpg")  
        #plt.savefig("tunnel "+str(entry)+".eps") 
                 