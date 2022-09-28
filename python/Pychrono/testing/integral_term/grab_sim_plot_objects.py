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
from matplotlib import colors as colors
import matplotlib as mpl
import os
from matplotlib import animation
import animatplot as amp
import matplotlib.cm as cm
from matplotlib import colors as colors
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
import statistics 
import matplotlib.patches as mpl_patches
class robot_plots:
    def __init__(self,nb,results_dir,sim,active,actbots,ax=None,ay=None,data0=None,data1=None,data2=None,\
                 data3=None,data4=None,data5=None,data6=None,data7=None,data8=None,data9=None,\
                 data10=None,data11=None,data12=None,data13=None,data14=None,data15=None,data16=None):

        # position
        if data0 is not None:
            self.data0=data0
            (self.m0,self.n0)=np.shape(self.data0)
            self.data0=self.data0[:,1:self.n0]
            self.time=self.data0[0,:]
            self.xpos=self.data0[1:nb+1,:]
            self.ypos=self.data0[nb+1:2*nb+1,:]
            self.zpos=self.data0[(2*nb)+1:3*nb+1,:]       
        else:
            self.data0=0
            
        # velocity             
        if data1 is not None:
            self.data1=data1
            (self.m1,self.n1)=np.shape(self.data1)
            self.data1=self.data1[:,1:self.n1]
            self.time=self.data1[0,:]
            self.xvel=self.data1[1:nb+1,:]
            self.yvel=self.data1[nb+1:2*nb+1,:]
            self.zvel=self.data1[(2*nb)+1:3*nb+1,:]
        else:
            self.data1=0  
            
        # Bot force            
        if data2 is not None:
            self.data2=data2
            (self.m2,self.n2)=np.shape(self.data2)
            self.data2=self.data2[:,1:self.n2]
            self.Fx=self.data2[1:nb+1,:]
            self.Fy=self.data2[nb+1:2*nb+1,:]
            self.Fz=self.data2[(2*nb)+1:3*nb+1,:]
        else:
            self.data2=0
            
        # Controller force
        if data3 is not None:
            self.data3=data3
            (self.m3,self.n3)=np.shape(self.data3)
            self.data3=self.data3[:,1:self.n3]
            self.Fxc=self.data3[1:nb+1,:]
            self.Fyc=self.data3[nb+1:2*nb+1,:]
            self.Fzc=self.data3[(2*nb)+1:3*nb+1,:]
        else:
            self.data3=0

        # springs
        if data4 is not None:
            self.data4=data4
            (self.m4,self.n4)=np.shape(self.data4)
            self.data4=self.data4[:,1:self.n4]
            self.Fsprings=self.data4[1:nb+1,:]
            self.Length_springs=self.data4[nb+1:2*nb+1,:]  
        else:
            self.data4=0
        
        # ball x force
        if data5 is not None:
            self.ballFx=data5
        else:
            self.ballFx=0

        # ball z force
        if data6 is not None:
            self.ballFz=data6
        else:
            self.ballFz=0
        # ball x position    
        if data7 is not None:
            self.ballx=data7
        else:
            self.ballx=0
        # ball z position
        if data8 is not None:
            self.ballz=data8
        else:
            self.ballz=0
            
        if data9 is not None:
            self.ax1=data9
        else:
            self.ax1=0
        if data10 is not None:
            self.az1=data10
        else:
            self.az1=0
        if data11 is not None:
            self.bvx=data11
        else:
            self.bvx=0
            
        if data12 is not None:
            self.bvz=data12
        else:
            self.bvz=0            
        if data13 is not None:
            self.FBX=data13
        else:
            self.FBX=0
            
        if data14 is not None:
            self.FBZ=data14
        else:
            self.FBZ=0
        
                # Controller force
        if data15 is not None:
            self.data15=data15
            (self.m15,self.n15)=np.shape(self.data15)
            self.data15=self.data15[:,1:self.n15]
            self.Fxcpf=self.data15[1:nb+1,:]
            self.Fycpf=self.data15[nb+1:2*nb+1,:]
            self.Fzcpf=self.data15[(2*nb)+1:3*nb+1,:]
        else:
            self.data15=0
            
        if data16 is not None:
            self.data16=data16
            (self.m16,self.n16)=np.shape(self.data16)
            self.data16=self.data16[:,1:self.n16]
            self.Fxcpp=self.data16[1:nb+1,:]
            self.Fycpp=self.data16[nb+1:2*nb+1,:]
            self.Fzcpp=self.data16[(2*nb)+1:3*nb+1,:]
        else:
            self.data16=0  
            
            
        self.sim=sim    
        self.nb=nb
        self.results_dir=results_dir
        self.active=active
        self.actbots=actbots
# In[Boundary robot positions]
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
# In[Velocity]
    def plot_velocity_xyz(self):
        direct = os.path.join(self.results_dir,'xyz positions')
        
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            #print(i)
            fig=plt.figure(i)
            plt.figure(figsize=(15,10))
    
            fig.suptitle("Position  vs Time (s) for Bot " + str(i))
            ax1 = plt.subplot(3,1,1)
            ax1.grid(True)
            plt.gca().set_title('x position (mm) vs time')
            plt.plot(self.time, self.xvel[i,:]*1000,'b')
    
            ax2 = plt.subplot(3,1,2)
            plt.gca().set_title('y position(mm) vs time')
            plt.plot(self.time,self.yvel[i,:]*1000,'r')
            ax2.grid(True)
            
            ax3 = plt.subplot(3,1,3)
            plt.gca().set_title('z position (mm) vs time')
            plt.plot(self.time, self.zvel[i,:]*1000,'g')
            ax3.grid(True)
            plt.subplots_adjust(hspace = 1)
            plt.xlabel('time (seconds)')
            plt.savefig(direct+'/'+str(i)+".png") 
            plt.close('all')
# In[Magnitude of the velocites]           
    def plot_mag_velocity(self):
        direct = os.path.join(self.results_dir,'magnitude of velocity')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        plt.figure(figsize=(20,10))    
        for i in range (self.nb):
            if self.active[i]==1.0:
                #print(i)
                plt.plot(self.time, np.sqrt(self.xvel[i,:]**2+self.zvel[i,:]**2),label='bot '+str(i))
        plt.grid(True)        
        plt.title("velocity vs Time (s) for Bot "+self.sim )
        plt.xlabel('time (seconds)')
        plt.legend()
        plt.savefig(direct+'/'+'magnitude_velocity'+".png") 
        plt.close('all')            
# In[Individual robot forces]          
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
            
# In[Controller forces]            
    def Plot_fxyzcontroller(self):
        direct = os.path.join(self.results_dir,'xyz Controller Forces')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            if self.active[i]==1.0:
                print(i)
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
# In[magnitude forces from controller]
    def plot_mag_controls(self):
        direct = os.path.join(self.results_dir,'magnitude of Forces')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        plt.figure(figsize=(20,10))    
        for i in range (self.nb):
            if self.active[i]==1.0:
                #print(i)
                plt.plot(self.time, np.sqrt(self.Fxc[i,:]**2+self.Fzc[i,:]**2),label='bot '+str(i))
        plt.grid(True)        
        plt.title("Force vs Time (s) for Bot "+self.sim )
        plt.xlabel('time (seconds)')
        plt.legend()
        plt.savefig(direct+'/'+'magnitude_forces'+".png") 
        plt.close('all')
        
# In[forces from each robot contoller]
    def Plot_conf(self):
        direct = os.path.join(self.results_dir,'Controller Forces')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        for i in range (self.nb):
            if self.active[i]==1.0:
                #print(i)
                fig=plt.figure(i)
                plt.figure(figsize=(20,10))
                fig.suptitle("Force vs Time (s) for Bot " + str(i)+' '+self.sim)
                ax1 = plt.subplot(2,1,1)
                ax1.grid(True)
                plt.gca().set_title('Force (N) vs time')
                plt.plot(self.time, self.Fxc[i,:],'b',label='xforce')
                plt.plot(self.time,self.Fyc[i,:],'r',label='yforce')
                plt.plot(self.time, self.Fzc[i,:],'g',label='zforce')
                        # create a list with two empty handles (or more if needed)

                plt.legend()
                ax2 = plt.subplot(2,1,2)
                plt.gca().set_title('Magnitude Force (N) vs time ' +self.sim)
                plt.plot(self.time, np.sqrt(self.Fxc[i,:]**2+self.Fzc[i,:]**2),'k')
                ax2.grid(True)
                handles = [mpl_patches.Rectangle((0, 0), 1, 1, fc="white", ec="white", lw=0, alpha=0)] * 2
                labels = []
                labels.append("max= {0:.4g}".format(np.max(np.sqrt(self.Fxc[i,:]**2+self.Fzc[i,:]**2))))
                labels.append("min= {0:.4g}".format(np.min(np.sqrt(self.Fxc[i,:]**2+self.Fzc[i,:]**2))))
                plt.legend(handles, labels, loc='best', fontsize='small', fancybox=True, framealpha=0.7, handlelength=0, handletextpad=0)
                #plt.subplots_adjust(hspace = 1)
                plt.xlabel('time (seconds)')
                plt.savefig(direct+'/'+str(i)+".png") 
                plt.close('all')   
                
# In[distrbution of forces from controllers]
    def forces_per_bot(self):
        direct = os.path.join(self.results_dir,'Controller Forces range')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        meanx=[]
        errorx=[]
        meanz=[]
        errorz=[]         
        for i in range (self.nb):
            if self.active[i]==1.0:
                tempx=(np.max(self.Fxc[i,:])-np.min(self.Fxc[i,:]))/2
                meanx.append(tempx+np.min(self.Fxc[i,:]))
                errorx.append(tempx/2)
                
                tempz=(np.max(self.Fzc[i,:])-np.min(self.Fzc[i,:]))/2
                meanz.append(tempz+np.min(self.Fzc[i,:]))
                errorz.append(tempz/2)
                
        fig=plt.figure(1)
        plt.figure(figsize=(20,10))
        ax1 = plt.subplot(2,1,1)
        plt.gca().set_title('x force max and min for each active bot '+self.sim)
        ax1.grid(True)
        plt.errorbar(self.actbots, meanx, yerr=errorx, fmt='o', color='black',ecolor='red', elinewidth=3, capsize=0)
        plt.ylabel('force (N)')
        plt.xlabel('bot')
        plt.xticks(self.actbots)
        plt.grid(True)
        ax2 = plt.subplot(2,1,2)
        plt.gca().set_title('z force max and min for each active bot '+self.sim)
        ax2.grid(True)
        plt.errorbar(self.actbots, meanz, yerr=errorz, fmt='o', color='black',ecolor='red', elinewidth=3, capsize=0)
        plt.ylabel('force (N)')
        plt.xlabel('bot')
        plt.xticks(self.actbots)
        plt.grid(True)
        plt.savefig(direct+'/'+'xzforce'+".png")
        plt.close('all')
    
    
# In[Plot potential field and leared follower controller seperatly ]
    def plot_mag_pf_pp_controls(self):
        direct = os.path.join(self.results_dir,'magnitude of potfield')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        fig=plt.figure(1)
        plt.figure(figsize=(20,10)) 
        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('potential fields  (N) vs time')
                
        for i in range (self.nb):
            if self.active[i]==1.0:
                #print(i)
                plt.plot(self.time, np.sqrt(self.Fxcpf[i,:]**2+self.Fzcpf[i,:]**2),label='bot '+str(i))
        ax1 = plt.subplot(2,1,2)
        ax1.grid(True)
        plt.gca().set_title('path+_pose (N) vs time')
        for i in range (self.nb):
            if self.active[i]==1.0:
                #print(i)
                plt.plot(self.time, np.sqrt(self.Fxcpp[i,:]**2+self.Fzcpp[i,:]**2),label='bot '+str(i))  
                       
        plt.xlabel('time (seconds)')
        #plt.legend()
        plt.savefig(direct+'/'+'magnitude_pP_PF'+".png") 
        plt.close('all')        
        
# In[Seismic controller force x]   
    def Control_force_sesmic_x(self):    
    
        results_dir = os.path.join(self.results_dir, 'Total_Force/')     
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)
        cmap = plt.cm.get_cmap('seismic')
        boundaries=np.arange(np.amin(self.Fcx),np.amax(self.Fcx),.1)
        norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])

        plt.figure(1,figsize=(8,8),dpi=150)
        F=abs(self.Fcx[:,-1])
        plt.scatter(self.zpos[:,-1],self.xpos[:,-1],s=2*np.power(F,.65),c=F,cmap=cmap,norm=norm)
        plt.xlim(-.75,.75)
        plt.ylim(-.75,.75)
        plt.grid(True)
        plt.colorbar()
        plt.xlabel('z position(m)')
        plt.ylabel('x position(m)')
        
# In[Seismic controller force z]      
    def Control_force_sesmic_z(self):    
    
        results_dir = os.path.join(self.results_dir, 'Total_Force/')     
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)
   
        cmap = plt.cm.get_cmap('seismic')
        boundaries=np.arange(np.amin(self.Fcz),np.amax(self.Fcz),.1)
        norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])

        plt.figure(2,figsize=(8,8),dpi=150)
        F=abs(self.Fcx[:,-1])
        plt.scatter(self.zpos[:,-1],self.xpos[:,-1],s=2*np.power(F,.95),c=F,cmap=cmap,norm=norm)
        plt.xlim(-.75,.75)
        plt.ylim(-.75,.75)
        plt.grid(True)
        plt.colorbar()
        plt.xlabel('z position(m)')
        plt.ylabel('x position(m)')
                
# In[Seismic controller force magnitude]  
    def Control_force_sesmic(self):    
        results_dir = os.path.join(self.results_dir, 'Total_Force/')     
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)
            cmap = plt.cm.get_cmap('seismic')
            F=abs(np.sqrt(np.power(self.Fcx[:,-1],2)+np.power(self.Fcz[:,-1],2)))
        
            NX=self.Fcx[:,-1]/F
            NZ=self.Fcz[:,-1]/F
            origin = self.zpos[:,-1],self.xpos[:,-1]
            boundaries=np.arange(np.amin(F),np.amax(F),.1)
            norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
        plt.figure(3,figsize=(8,8),dpi=150)
        plt.quiver(*origin, NZ, NX, color="k", scale=21)
        plt.scatter(self.zpos[:,-1],self.xpos[:,-1],s=2*np.power(F,1),c=F,cmap=cmap,norm=norm)
        plt.xlim(-.75,.75)
        plt.ylim(-.75,.75)
        plt.grid(True)
        plt.colorbar()
        plt.xlabel('z position(m)')
        plt.ylabel('x position(m)')        
    
# In[Seismic controller force animation]   
    def Control_force_sesmic_animation(self):    
        results_dir = os.path.join(self.results_dir, 'Total_Force/')     
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)
        count=0
        for i in range(1,len(self.time)-1):
            plt.figure(i,figsize=(8,8),dpi=150)    
            cmap = plt.cm.get_cmap('seismic')
            F=abs(np.sqrt(np.power(self.Fcx[:,i],2)+np.power(self.Fcz[:,i],2)))
            NX=self.Fcx[:,i]/F
            NZ=self.Fcz[:,i]/F
            origin = self.zpos[:,i],self.xpos[:,i]
            boundaries=np.arange(np.amin(F),np.amax(F),.1)
            norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
            
            if i%3==0:
                count=count+1 
                plt.figure(i,figsize=(8,8),dpi=150)
                plt.quiver(*origin, NZ, NX, color="k", scale=21)
                plt.scatter(self.zpos[:,i],self.xpos[:,i],s=2*np.power(F,1.5),c=F,cmap=cm.hsv,norm=norm)
            
            
                plt.xlim(-.75,1.75)
                plt.ylim(-.75,.75)
                plt.grid(True)
                plt.colorbar()
                plt.title('t='+str(round(self.time[i],3)))
                plt.savefig(results_dir+"/picture"+str(count)+".jpg")  
                print(str(i)+ "of"+ str(len(self.time)))
                plt.close('all')
                plt.xlabel('z position(m)')
                plt.ylabel('x position(m)')         
        
# In[path of bots]  
    def plot_path(self):
        direct = os.path.join(self.results_dir)
        Xb=[]
        Zb=[]
        for i in range(self.n0-1):
            Xb.append(sum(self.xpos[:,i])/self.nb)
            Zb.append(sum(self.zpos[:,i])/self.nb)
            
        X=self.xpos[0,:]
        Z=self.zpos[0,:]
        plt.figure(figsize=(20,10))
        plt.plot(X,Z,'b',label="leader_path")
        plt.plot(self.ax1,self.az1,'g',label="desired_path")
        plt.plot(Xb,Zb,'--r',label="COM path")
        plt.title("path of robot "+self.sim)
        plt.xlabel('x')
        plt.ylabel('z')
        plt.ylim((-.1,.1))
        plt.grid(True)
        plt.legend()        
        plt.savefig(direct+"/path of robot"+self.sim+".png") 
        plt.close('all') 
    
# In[Total force in system ]      
    def plot_total_forces(self):
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
        plt.plot(self.time, FZS,'g')
        ax3.grid(True)
        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(self.results_dir+"/Total Force.png") 
        plt.close('all')              
# In[Plot all spring lengths 1 graph]
    def plot_all_spring_length_force(self):
        direct = os.path.join(self.results_dir,'Spring_force_length')
        if not os.path.isdir(direct):
            os.makedirs(direct)
        fig=plt.figure(1)
        plt.figure(figsize=(15,10))
        ax1 = plt.subplot(2,1,1)
        ax1.grid(True)
        plt.gca().set_title('length (N) vs time')
        for i in range (self.nb):
            plt.plot(self.time,self.Length_springs[i,:],label='spring'+str(i))
        #plt.legend()
        ax2= plt.subplot(2,1,2)
        ax2.grid(True)
        plt.gca().set_title('Force(N) vs time')
        for i in range (self.nb):
            plt.plot(self.time,self.Fsprings[i,:],label='spring'+str(i))    
        #plt.legend()
        plt.savefig(direct+"/spring_force_length.png") 
        plt.close('all')       
# In[Plot all spring lengths individual graphs]
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
            plt.savefig(direct+'/'+str(i)+".pdf") 
            plt.close('all')
            

 
                
# In[Ball force]
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
        plt.plot(self.time, self.ballFx.T,'b')
    
        ax2 = plt.subplot(2,1,2)
        plt.gca().set_title('Z vs time')
        plt.plot(self.time,self.ballFz,'r')
        ax2.grid(True)

        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(direct+"/ball_force.png") 
        plt.close('all') 

# abs force on balls        
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
        plt.plot(self.time, self.FBX.T,'b')
    
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
        direct = os.path.join(self.results_dir)
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
        plt.figure(figsize=(15,10)) 
        
        plt.plot(self.ballx,self.ballz,'--r')
        plt.title('ball movement')
        plt.xlabel('x (meters)')
        plt.ylabel('z (meters)')
        plt.grid(True)
        plt.savefig(direct+"/ball_position.png") 
        plt.close('all')

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
#Plotter for comparing robots        
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
        
        
# %% Force chains
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
            
# %% Packing fraction            
class Packing_fraction:
    def __init__(self,data0,data15,nb,diameter,results_dir):
        self.nb=nb

        self.data0=data0
        (self.m0,self.n0)=np.shape(self.data0)
        self.data0=self.data0[:,1:self.n0]
        self.time=self.data0[0,:]
        self.xposb=self.data0[1:nb+1,:]
        self.yposb=self.data0[nb+1:2*nb+1,:]
        self.zposb=self.data0[(2*nb)+1:3*nb+1,:]
        self.results_dir=results_dir
        
        self.data15=data15
        (self.m15,self.n15)=np.shape(self.data15)
        self.data15=self.data15[:,1:self.n15]
        self.ni=int((self.m15-1)/3)
        self.time=self.data15[0,:]
        self.xposp=self.data15[1:self.ni+1,:]
        self.yposp=self.data15[self.ni+1:2*self.ni+1,:]
        self.zposp=self.data15[(2*self.ni)+1:3*self.ni+1,:]
       
        
        self.X=np.concatenate((self.xposb[:,-1],self.xposp[:,-1]), axis=None)
        self.Z=np.concatenate((self.zposb[:,-1],self.zposp[:,-1]), axis=None)
        self.P=np.stack((self.Z.T, self.X.T), axis=1)
        self.radius=None
        self.new_regions = []
        self.new_vertices=[]
        self.diameter=diameter
        self.A=[]
#        voronoi_plot_2d(self.vor)
#        plt.show()
        
##################################################################      
    def voronoi_finite_polygons_2d(self):
    

        if self.vor.points.shape[1] != 2:
            raise ValueError("Requires 2D input")

            # new vertices
        new_vertices = self.vor.vertices.tolist()


            # calculate center
        center = self.vor.points.mean(axis=0)
        if self.radius is None:
            self.radius = self.vor.points.ptp().max()*2

            # Construct a map containing all ridges for a given point
        all_ridges = {}
    
        for (p1, p2), (v1, v2) in zip(self.vor.ridge_points, self.vor.ridge_vertices):
            all_ridges.setdefault(p1, []).append((p2, v1, v2))
            all_ridges.setdefault(p2, []).append((p1, v1, v2))

            # Reconstruct infinite regions
        for p1, region in enumerate(self.vor.point_region):
            vertices = self.vor.regions[region]

            if all(v >= 0 for v in vertices):
            # finite region
                self.new_regions.append(vertices)
                continue

        # reconstruct a non-finite region
            ridges = all_ridges[p1]
            new_region = [v for v in vertices if v >= 0]

            for p2, v1, v2 in ridges:
                if v2 < 0:
                    v1, v2 = v2, v1
                if v1 >= 0:
                # finite ridge: already in the region
                    continue

            # Compute the missing endpoint of an infinite ridge
                t = self.vor.points[p2] - self.vor.points[p1] # tangent
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])  # normal

                midpoint = self.vor.points[[p1, p2]].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                far_point = self.vor.vertices[v2] + direction * self.radius

                new_region.append(len(new_vertices))
                new_vertices.append(far_point.tolist())

        # sort region counterclockwise
            vs = np.asarray([new_vertices[v] for v in new_region])
            c = vs.mean(axis=0)
            angles = np.arctan2(vs[:,1] - c[1], vs[:,0] - c[0])
            new_region = np.array(new_region)[np.argsort(angles)]

        # finish
            self.new_regions.append(new_region.tolist())

        return self.new_regions, np.asarray(new_vertices)    
###################################################################################    
    def voronoi_Area(self,region):
        Area=ConvexHull(self.vertices[region]).volume
        return(Area)
  #########################################################################  
    def calculate_packing(self):
        self.vor = Voronoi(self.P)
        (self.regions, self.vertices) = self.voronoi_finite_polygons_2d()
        
        Aact=np.pi*(self.diameter/2)**2
        for i in range(len(self.regions)):
            region=self.regions[i]
            self.A.append(Aact/(self.voronoi_Area(region)))
        self.A=np.asarray(self.A)

    def plot_packing(self):
        direct = os.path.join(self.results_dir)     
        if not os.path.isdir(direct):
            os.makedirs(direct)
        minima = min(self.A)
        maxima = max(self.A)

        #normalize chosen colormap
        norm = mpl.colors.Normalize(vmin=minima, vmax=maxima, clip=True)
        mapper = cm.ScalarMappable(norm=norm, cmap=cm.hsv)


        plt.figure(figsize=(13,13))

        for r in range(len(self.vor.point_region)):
            region = self.vor.regions[self.vor.point_region[r]]
            if not -1 in region:
                polygon = [self.vor.vertices[i] for i in region]
                plt.fill(*zip(*polygon), color=mapper.to_rgba(self.A[r]))
        #plt.axis('equal')
        #plt.xlim(self.vor.min_bound[0] - 0.1, self.vor.max_bound[0] + 0.1)
        #plt.ylim(self.vor.min_bound[1] - 0.1, self.vor.max_bound[1] + 0.1)
        Abar = self.A[self.A > .3]
        plt.xlim(0,1.2)
        plt.ylim(-.5,.5)
        plt.xlabel('x position (meters)')
        plt.ylabel('y position (meters)')
        plt.title('Local Packing fraction'+"avg phi="+str(round(statistics.mean(Abar),3)))
        plt.colorbar(mapper)
        plt.savefig(direct+"/Packing_fraction.pdf")
        plt.close('all')

    def stat_packing(self):
        direct = os.path.join(self.results_dir)     
        if not os.path.isdir(direct):
            os.makedirs(direct)
        n, bins, patches = plt.hist(self.A, 20, facecolor='g')


        plt.xlabel('local Packing fraction')
        plt.ylabel('number of occurances')
        Abar = self.A[self.A > .5]
        print(statistics.mean(Abar)) 
        plt.title('Histogram of packing fraction')
        plt.savefig(direct+"/statistics.pdf")
        plt.close('all')
        #plt.text(60, .025, r'$\mu=100,\ \sigma=15$')
        #plt.xlim(.7,.9)
        #plt.ylim(0, 0.03)

#plt.grid(True)
#plt.savefig("distribution 8 robot 15 particles"+name+".png")
#plt.show()     


