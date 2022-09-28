# -*- coding: utf-8 -*-
"""
Created on Tue May 12 12:53:23 2020

@author: dmulr
"""
import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
from matplotlib import animation
import animatplot as amp
from matplotlib import colors as colors
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull

class plots:
    def __init__(self,data1,data2,data3,data4,results_dir,nb):
        self.data1=data1
        (self.m1,self.n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:self.n1]
        self.time=self.data1[0,:]
        self.xpos=self.data1[1:nb+1,:]
        self.ypos=self.data1[nb+1:2*nb+1,:]
        self.zpos=self.data1[(2*nb)+1:3*nb+1,:]
    
        self.data2=data2
        (self.m2,self.n2)=np.shape(self.data2)
        self.data2=self.data2[:,1:self.n2]
        self.Fx=self.data2[1:nb+1,:]
        self.Fy=self.data2[nb+1:2*nb+1,:]
        self.Fz=self.data2[(2*nb)+1:3*nb+1,:]
    
        self.data3=data3
        (self.m3,self.n3)=np.shape(self.data3)
        self.data3=self.data3[:,1:self.n3]
        self.Fcx=self.data3[1:nb+1,:]
        self.Fcy=self.data3[nb+1:2*nb+1,:]
        self.Fcz=self.data3[(2*nb)+1:3*nb+1,:]
    
        self.data4=data4
        (self.m4,self.n4)=np.shape(self.data4)
        self.data4=self.data4[:,1:self.n4]
        self.Fsprings=self.data4[1:nb+1,:]
        self.Length_springs=self.data4[nb+1:2*nb+1,:]
        
        self.nb=nb
        self.results_dir=results_dir
        self.mc=1000
        self.lc=100
        self.mcr=1/self.mc
        self.lcr=1/self.lc
        self.fcr=1/1000
# plot each robots x position
    def plot_xyz(self):
        direct = os.path.join(self.results_dir,'xyz positions')
        
    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            print(i)
            fig=plt.figure(i)
            plt.figure(figsize=(15,10))
    
            fig.suptitle("Position  vs Time (s) for Bot " + str(i))
            ax1 = plt.subplot(3,1,1)
            ax1.grid(True)
            plt.gca().set_title('x position (mm) vs time')
            plt.plot(self.time, self.xpos[i,:]*1000,'b')
    
            ax2 = plt.subplot(3,1,2)
            plt.gca().set_title('y position(mm) vs time')
            plt.plot(self.time,self.ypos[i,:]*1000,'r')
            ax2.grid(True)
            
            ax3 = plt.subplot(3,1,3)
            plt.gca().set_title('z position (mm) vs time')
            plt.plot(self.time, self.zpos[i,:]*1000,'g')
            ax3.grid(True)
            plt.subplots_adjust(hspace = 1)
            plt.xlabel('time (seconds)')
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all')
            
            
            
# Plot forces            
    def Plot_fxyz(self):
            
        direct = os.path.join(self.results_dir,'xyz Forces')
        
    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            print(i)
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
            
# plot controller forces            
    def Plot_fxyzcontroller(self):
            
        direct = os.path.join(self.results_dir,'xyz Controller Forces')
        
    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            print(i)
            fig=plt.figure(i)
            plt.figure(figsize=(15,10))
    
            fig.suptitle("Force vs Time (s) for Bot " + str(i))
            ax1 = plt.subplot(3,1,1)
            ax1.grid(True)
            plt.gca().set_title('X Force (N) vs time')
            plt.plot(self.time, self.Fcx[i,:],'b')
    
            ax2 = plt.subplot(3,1,2)
            plt.gca().set_title('Y Force (N) vs time')
            plt.plot(self.time,self.Fcy[i,:],'r')
            ax2.grid(True)
            
            ax3 = plt.subplot(3,1,3)
            plt.gca().set_title('Z Force (N) vs time')
            plt.plot(self.time, self.Fcz[i,:],'g')
            ax3.grid(True)
            plt.subplots_adjust(hspace = 1)
            plt.xlabel('time (seconds)')
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all')                  
# plot spring force and lengths
    def Plot_spring_force_length(self):
                
        direct = os.path.join(self.results_dir,'Spring_force_length')
        
    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            print(i)
            fig=plt.figure(i)
            plt.figure(figsize=(15,10))
            ax1 = plt.subplot(2,1,1)
            ax1.grid(True)
            plt.gca().set_title('length (N) vs time')
            plt.plot(self.time,self.Length_springs[i,:],'b')
    
            ax2 = plt.subplot(2,1,2)
            plt.gca().set_title('Force vs time for bot')
            plt.plot(self.time,self.Fsprings[i,:],'r')
            ax2.grid(True)
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all')


# Force chains
class plot_force_chain:
    def __init__(self,data1,data5,data6,data7,data8,data9,data10,data11,results_dir,nb):  
        self.data1=data1
        self.time=self.data1[0,:]
        
        self.xc=data5
        self.yc=data6
        self.zc=data7
        self.Fcx=data8
        self.Fcy=data9
        self.Fcz=data10
        self.nc=data11
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
                plt.scatter(x2,y2,s=2*np.power(F2,.65),c=F2,cmap=cmap,norm=norm)
                plt.xlim(-1,1)
                plt.ylim(-1,1)
                plt.grid(True)
                plt.colorbar()
                plt.xlabel('x position(m)')
                plt.ylabel('z position(m)')
                plt.title('t='+str(round(self.time[i],3)))
                plt.savefig(direct+"/picture"+str(count)+".jpg")  
                print(str(i)+ "of"+ str(len(self.time)))
                plt.close('all')
            
            
