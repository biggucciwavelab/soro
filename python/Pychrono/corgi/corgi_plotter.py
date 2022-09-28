# -*- coding: utf-8 -*-
"""
Created on Thu Nov 14 13:42:24 2019

@author: dmulr
"""

import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
from matplotlib import animation
import animatplot as amp
from matplotlib import colors as colors
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull








# In[Force Chains]
    
def Forcechains(script_dir,nc,xc,yc,zc,Fcx,Fcy,Fcz,time):
    results_dir = os.path.join(script_dir, 'force_chain_pos/')     
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
   
    cmap = plt.cm.get_cmap('seismic')
    boundaries=np.arange(10,30,.1)
                       
    norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
    count=0
    for i in range(1,len(time)):
        Fx=Fcx[0:nc[i],i]
        Fz=Fcz[0:nc[i],i]
        #Fy=Fcy[0:nc[i],i]
        abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)


        x=xc[0:nc[i],i]
        y=zc[0:nc[i],i]
        x2=[]
        y2=[]
        F2=[]
        for j in range(len(abs_force)):
           if abs_force[j]>=1:
               x2.append(x[j])
               y2.append(y[j])
               F2.append(abs_force[j])
               
  
        if i%10==0:
            count=count+1 
            plt.figure(i,figsize=(8,8),dpi=150)
            plt.scatter(x2,y2,s=2*np.power(F2,.65),c=F2,cmap=cmap,norm=norm)
            plt.xlim(-2,2)
            plt.ylim(-2,2)
            plt.grid(True)
            plt.colorbar()
            plt.xlabel('x position(m)')
            plt.ylabel('z position(m)')
            plt.title('t='+str(round(time[i],3)))
            plt.savefig(results_dir+"picture"+str(count)+".jpg")  
            print(str(i)+ "of"+ str(len(time)))
            plt.close('all')            
        
    return(abs_force)

#################################################################################################
# In[Packing Fraction]
def PackingFraction(script_dir,time,nb,ni,diameter,height,qx,qz,file,count):
    results_dir = os.path.join(script_dir, 'packing fraction/')
    
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir) 
    
    A=np.zeros(count)
    Atemp=np.zeros(nb) 

    const= .25*ni*(np.pi)*(diameter)**2
 
    for j in range(count):
        for i in range(nb-1):
            Atemp[i]=.5*(qx[i+1,j]+qx[i,j])*(qz[i+1,j]-qz[i,j])
        i=nb-1
        Atemp[i]=.5*(qx[i,j]+qx[0,j])*(qz[0,j]-qz[i,j])
    
        A[j]=const/sum(Atemp)

    plt.figure(1)

    plt.plot(time,A,'b')
    plt.title('Packing fraction vs time')
    plt.xlabel('time')
    plt.ylabel('Packing fraction')
    plt.grid(True)
    plt.savefig(results_dir+" Packing fraction"+file)  
    plt.close('all')    




##################################################################################################
# In[Ball pos2]
def Ballpos2(script_dir,time,ballp,file):
    results_dir = os.path.join(script_dir, 'ball position/')
   
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
    plt.figure(1)
    plt.plot(time,ballp,'b')
    plt.xlim(0,10)
    plt.title('ball position vs time')
    plt.xlabel('time')
    plt.ylabel('ball position')
    plt.grid(True)
    plt.savefig(results_dir+" ball position"+file)  
    plt.close('all')   


#####################################################################################################

# In[Surface Tension]

def tension(nb,Fmem,time,script_dir,qx,qy,qz):    
    
    results_dir = os.path.join(script_dir, 'Tension/')     
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
   
    cmap = plt.cm.get_cmap('seismic')
    boundaries=np.arange(10,30,.1)
    norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
    for i in range(1,len(time)):
        F=abs(Fmem[:,i])
        x=qx[0:nb,i]
        y=qz[0:nb,i]
        name=str(i)
        plt.figure(i,figsize=(8,8),dpi=150)
        plt.scatter(x,y,s=2*np.power(F,.65),c=F,cmap=cmap,norm=norm)
        plt.xlim(-2,2)
        plt.ylim(-2,2)
        plt.grid(True)
        plt.colorbar()
        plt.xlabel('x position(m)')
        plt.ylabel('z position(m)')
        plt.title('t='+str(round(time[i],3)))
        #cbar=plt.colorbar(path)
        #cbar.set_label('Contact force')
        plt.savefig(results_dir+"picture"+name+".jpg")  
        print(str(i)+ "of"+ str(len(time)))
        plt.close('all')            




def NORM_TAN(VNX,VNZ,VTX,VTZ,qx,qy,qz,script_dir,time,nb):
    
    results_dir = os.path.join(script_dir, 'norm_tan/')     
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
    count=0
    for i in range(len(time)):
        plt.figure(i,figsize=(8,8),dpi=150)

        X=qx[0:nb,i]
        Z=qz[0:nb,i]
        NX=10*VNX[:,i]
        NZ=10*VNZ[:,i]
        TX=10*VTX[:,i]
        TZ=10*VTZ[:,i]
        name=str(i)
        #plt.scatter(X,Z,color="b")
        
        if i%10==0:
            count=count+1
        #for j in range(nb):
            origin = X,Z # origin point
            plt.quiver(*origin, NX, NZ, color="g", scale=21)
            plt.quiver(*origin, TX, TZ, color="r", scale=21)
        
            plt.title('t='+str(round(time[i],3)))
            plt.xlabel('x position')
            plt.ylabel('y position')
            plt.grid(True)
            plt.savefig(results_dir+"picture"+str(count)+".jpg")  
            print(str(i)+ "of"+ str(len(time)))

            plt.close('all')  
    
# In[Total Force_membrane]
            
def Total_Force_membrane(nb,time,script_dir,qx,qy,qz,Fxt,Fyt,Fzt):    
    
    results_dir = os.path.join(script_dir, 'Total_Force/')     
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
   
    cmap = plt.cm.get_cmap('seismic')
    boundaries=np.arange(10,30,.1)
    norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
    for i in range(1,len(time)):
        F=abs(np.sqrt(np.power(Fxt[:,i],2)+np.power(Fzt[:,i],2)))
        x=qx[0:nb,i]
        y=qz[0:nb,i]
        name=str(i)
        plt.figure(i,figsize=(8,8),dpi=150)
        plt.scatter(x,y,s=2*np.power(F,.65),c=F,cmap=cmap,norm=norm)
        plt.xlim(-2,2)
        plt.ylim(-2,2)
        plt.grid(True)
        plt.colorbar()
        plt.xlabel('x position(m)')
        plt.ylabel('z position(m)')
        plt.title('t='+str(round(time[i],3)))
        #cbar=plt.colorbar(path)
        #cbar.set_label('Contact force')
        plt.savefig(results_dir+"picture"+name+".jpg")  
        print(str(i)+ "of"+ str(len(time)))
        plt.close('all')              

#def Animation(nb,nt,qx,qy,qz,time,tstep,ballp):
    
    












# In[Local packing Fraction]
def voronoi_area(x,y):
    Atemp=np.zeros_like(x)
    for i in range(len(x)-1):
        var=x[i]*y[i+1]-x[i+1]*y[i]
        Atemp[i]=var
    i=len(x)-1
    var=x[i]*y[0]-x[0]*y[i]
    Atemp[i]=var
    A=.5*sum(Atemp)
    return A
# this isolates the indices
def getem(indices,v):
    v2=np.zeros((len(indices),2))
    j=0
    for i in(indices):
        #print(i)
        v2[j,:]=v[i,:]
        j=j+1
    #print(v2)
    h=ConvexHull(v2)
    x3=np.zeros(len(indices))
    y3=np.zeros(len(indices))
    count=0
    for i in (h.vertices):
        x3[count]=v2[i,0]
        y3[count]=v2[i,1]
        count=count+1
    return(x3,y3)
# this executes it all
def voronoi_regions_area(points):
    # First sort them find the regions 
    v = Voronoi(points)
    #print(v.npoints)
    Area = np.zeros(v.npoints)
    c=0
    for i, reg_num in enumerate(v.point_region):
        indices = v.regions[reg_num]
        if -1 in indices: # some regions can be opened
            Area[i] = 0
        else:
            indi=indices
            vel=v.vertices

            [x3,y3]=getem(indi,vel)
            
            Area[i]=voronoi_area(x3,y3)
            c=c+1
    return (Area,c)
            

def Localpackingfraction(script_dir,qx,qy,qz,time):
    
    for i in range(1):
        x=qx[:,i]
        y=qz[:,i]
        #x=np.transpose(x)
        #y=np.transpose(y)
        
        points=np.array([x,y])
        points=np.transpose(points)
        A=voronoi_regions_area(points)
        
        vor = Voronoi(points)
        voronoi_plot_2d(vor)
    speed = np.random.uniform(low=0.0, high=5.0, size=50)

# generate Voronoi tessellation
    vor = Voronoi(points)

    # find min/max values for normalization
    minima = min(speed)
    maxima = max(speed)

    # normalize chosen colormap
    norm = mpl.colors.Normalize(vmin=minima, vmax=maxima, clip=True)
    mapper = cm.ScalarMappable(norm=norm, cmap=cm.Blues_r)

# plot Voronoi diagram, and fill finite regions with color mapped from speed value
    voronoi_plot_2d(vor, show_points=True, show_vertices=False, s=1)
    for r in range(len(vor.point_region)):
        region = vor.regions[vor.point_region[r]]
    if not -1 in region:
        polygon = [vor.vertices[i] for i in region]
        plt.fill(*zip(*polygon), color=mapper.to_rgba(speed[r]))
    plt.show()

    return A
####################################################################################################





























######################################################################################################
# In[Position Codes]


def position(script_dir,botcall,time,qx,qy,qz,nb,file):
    
    results_dir = os.path.join(script_dir, 'XYZPosition/')

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
    
    for i in range (nb):
        var=botcall[0,i]
        if var==1:
        
            txt=" (Active)"
        else:
            txt=" (passive)"
    
        fig=plt.figure(i)
        plt.figure(figsize=(15,10))
    
        fig.suptitle("Position  vs Time (s) for Bot " + str(i) + txt)
        ax1 = plt.subplot(3,1,1)
        ax1.grid(True)
        plt.gca().set_title('x position (mm) vs time')
        plt.plot(time, qx[i,:]*1000,'b')
    
        ax2 = plt.subplot(3,1,2)
        plt.gca().set_title('y position(mm) vs time')
        plt.plot(time,qy[i,:]*1000,'r')
        ax2.grid(True)
        ax3 = plt.subplot(3,1,3)
        plt.gca().set_title('z position (mm) vs time')
        plt.plot(time, qz[i,:]*1000,'g')
        ax3.grid(True)
        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(results_dir+" bot_pos " + str(i) + txt +file) 
        plt.close('all')

# In[Create plots for rotation]

def Rotation(script_dir,time,rot0,rot1,rot2,rot3,nb,botcall,file):
    results_dir = os.path.join(script_dir, 'rotations/')

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)

    def quaternion_to_euler(x, y, z, w):

            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            X = math.degrees(math.atan2(t0, t1))

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            Y = math.degrees(math.asin(t2))

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            Z = math.degrees(math.atan2(t3, t4))

            return X, Y, Z

    yaw=np.zeros((nb,np.size(time)))
    pitch=np.zeros((nb,np.size(time)))
    roll=np.zeros((nb,np.size(time)))
    for i in range(nb):

        for j in range(np.size(time)):
            Erot=quaternion_to_euler(rot0[i,j], rot1[i,j], rot2[i,j], rot3[i,j])   
            Erot=np.asarray(Erot)
            yaw[i,j]=Erot[0]
            pitch[i,j]=Erot[1]
            roll[i,j]=Erot[2]
        
    for i in range (nb):

        var=botcall[0,i]
        if var==1:
        
            txt=" (Active)"
        else:
            txt=" (passive)"
        fig=plt.figure(i)
        plt.figure(figsize=(13,13))
    
        fig.suptitle("rotation vs Time (s) for Bot " + str(i) + txt)
        ax1 = plt.subplot(3,1,1)
        ax1.grid(True)
        plt.gca().set_title('yaw vs time')
        plt.plot(time, yaw[i,:],'b')
    
        ax2 = plt.subplot(3,1,2)
        plt.gca().set_title('pitch vs time')
        plt.plot(time,pitch[i,:],'r')
        ax2.grid(True)
        ax3 = plt.subplot(3,1,3)
        plt.gca().set_title('roll vs time')
        plt.plot(time, roll[i,:],'g')
        ax3.grid(True)
        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(results_dir+" bot_pos " + str(i) + txt + file) 
        plt.close('all')       

# In[Velocity of the robots]
        
def Velocity(script_dir,botcall,Xv,Yv,Zv,nb,time,file):
    results_dir = os.path.join(script_dir, 'XYZVelocity/')

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
        
        for i in range (nb):

            var=botcall[0,i]
        if var==1:
        
            txt=" (Active)"
        else:
            txt=" (passive)"
        fig=plt.figure(i)
        plt.figure(figsize=(13,13))
    
        fig.suptitle("Velocity  vs Time (s) for Bot " + str(i) + txt)
        ax1 = plt.subplot(3,1,1)
        ax1.grid(True)
        plt.gca().set_title('x Velocity (m/s) vs time')
        plt.plot(time, Xv[i,:],'b')
    
        ax2 = plt.subplot(3,1,2)
        plt.gca().set_title('y Velocity (m/s) vs time')
        plt.plot(time,Yv[i,:],'r')
        ax2.grid(True)
        ax3 = plt.subplot(3,1,3)
        plt.gca().set_title('z Velocity (m/s) vs time')
        plt.plot(time, Zv[i,:],'g')
        ax3.grid(True)
        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(results_dir+" bot_Velocity " + str(i) + txt +file) 
        plt.close('all')      

# In[Path Traveled]
def PathTraveled(script_dir,Xv,Yv,Zv,qx,qy,qz,file,nb):   
    results_dir = os.path.join(script_dir, 'Path traveled/')
    cm = plt.cm.get_cmap('copper')
    abs_vel = np.power(np.add(np.power(Xv,2),np.power(Yv,2),np.power(Zv,2)),.5)

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)  

    plt.figure(num=1,figsize=[6,4],dpi=100)

    for i in range(nb):
    
        path=plt.scatter(qx[i,:]*1000,qz[i,:]*1000,c=abs_vel[i],cmap=cm)

        plt.title('Positon of the bots (unjammed)')
        plt.xlabel('x position (mm)')
        plt.ylim(1000*qz.min(),1000*qz.max())
        plt.ylabel('z position (mm)')
        plt.grid(True)

        cbar=plt.colorbar(path)
        cbar.set_label('Velocity (m/s)')
        plt.savefig(results_dir+" Path of bots" + file) 
        plt.close('all')


    plt.figure(num=2,figsize=[6,4],dpi=100)

    for i in range(nb):
    
        path1=plt.scatter(np.average(qx,axis=0)*1000,np.average(qz,axis=0)*1000,c=np.average(abs_vel,axis=0),cmap=cm)

    plt.title('Average Positon of Robot')
    plt.xlabel('x position (mm)')
    plt.xlim(0,400)
    plt.ylim(-150,100)
    plt.ylabel('z position (mm)')
    plt.grid(True)
    cbar=plt.colorbar(path1)
    cbar.set_label('Velocity (m/s)')
    plt.savefig(results_dir+"Average Path of bots" + ".svg") 
    plt.close('all')

# In[Spring Lengths]
def Springlength(script_dir,time,SL,nb,file):        
    results_dir = os.path.join(script_dir, 'spring lengths/')

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)

    for i in range(nb):

        plt.figure(i)
        plt.title('Spring length (mm) ' + str(i) + ' vs time (seconds)')
        plt.plot(time,SL[i,:]*1000,'b')
        plt.xlabel('time')
        plt.ylabel('Spring length(mm)')
        plt.grid(True)
        plt.savefig(results_dir+" Spring length " + str(i) + file) 
        plt.close('all')

# In[Spring force]  
def SpringForce(script_dir,time,Fmem,nb,file):       
    results_dir = os.path.join(script_dir, 'spring Force/')

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)

    for i in range(nb):

        plt.figure(i)
        plt.title('Spring Force (N) ' + str(i) + ' vs time (seconds)')
        plt.plot(time,Fmem[i,:],'b')
        plt.xlabel('time')
        plt.ylabel('Spring length(mm)')
        plt.grid(True)
        plt.savefig(results_dir+" Spring length " + str(i) + file) 
        plt.close('all') 

  


    
# In[Total Forces]   
def TotalForces(script_dir,nb,Fxt,Fyt,Fzt,time,botcall,file):
    results_dir = os.path.join(script_dir, 'External Forces/')
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
    for i in range (nb):
   
        var=botcall[0,i]
        if var==1:
        
            txt=" (Active)"
        else:
            txt=" (passive)"
        
        fig=plt.figure(i)
        plt.figure(figsize=(13,13))
        fig.suptitle("Force applied vs Time for Bot" + str(i) + txt)
        ax1 = plt.subplot(3,1,1)
        ax1.grid(True)
        plt.gca().set_title('x force (N) vs Time')
        plt.plot(time, Fxt[i,:],'b')
        ax2 = plt.subplot(3,1,2)
        plt.gca().set_title('y force (N) vs Time')
        plt.plot(time, Fyt[i,:],'r')
        ax2.grid(True)
        ax3 = plt.subplot(3,1,3)
        plt.gca().set_title('z force (N) vs Time')
        plt.plot(time, Fzt[i,:],'g')
        ax3.grid(True)
        plt.subplots_adjust(hspace = 1)
        plt.xlabel('time (seconds)')
        plt.savefig(results_dir+"Force applied for" + str(i) + txt + file)
        plt.close('all')


# In[Ball position]        
def Ballpos(script_dir,time,ballp,ballp2,time2,file):
    results_dir = os.path.join(script_dir, 'ball position/')  
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
    plt.figure(1)
    plt.plot(time,ballp,'b')
    plt.xlim(0,10)
    plt.title('ball position vs time')
    plt.xlabel('time')
    plt.ylabel('ball position')
    plt.grid(True)
    plt.savefig(results_dir+" ball position"+file)  
    plt.close('all')    
    
    
    ballpn=np.zeros((len(ballp),1))
    ballpn2=np.zeros((len(ballp2),1))
    for i in range (len(time)):
       
        ballpn[i]=ballp[i]-ballp[1000]
    
    for i in range(len(time2)):
        ballpn2[i]=ballp2[i]-ballp2[1000]
        
    plt.figure(2)
    plt.plot(time,ballpn,'b',label='robot')
    plt.plot(time2,ballpn2,'r',label='no robot')
    plt.xlim(5,10)
    plt.title('ball position vs time')
    plt.xlabel('time')
    plt.ylabel('ball position')
    plt.legend()
    plt.grid(True)
    plt.savefig(results_dir+" ball position after grab"+file)  
    plt.close('all')           
    return (ballpn,ballpn2)   

# In[Total Force_membrane]

        
       
## In[Animated barplots]
#def AnimatedForces(nb,Fmem,time,script_dir,frames,rate):
#    
#    results_dir = os.path.join(script_dir, 'Tension_membrane/')
#
#    if not os.path.isdir(results_dir):
#        os.makedirs(results_dir)
#    Y2=Fmem
#    Y2=np.transpose(Y2)
#    x=np.linspace(1,nb,nb)
#    #time=np.transpose(time)
#    X2,T2 = np.meshgrid(x,time)
#    
#    columns=np.arange(0,len(time),frames)
#    
#    X=np.zeros((len(columns),nb))
#    T=np.zeros((len(columns),nb))
#    Y=np.zeros((len(columns),nb))
# 
#    j=0
#    for i in (columns):
#        X[j,:]=X2[i,:]   
#        T[j,:]=T2[i,:]
#        Y[j,:]=Y2[i,:]
#        j=j+1
#        
#    timeline = amp.Timeline(T,fps=rate)
#    block1 = amp.blocks.Line(X, Y,marker='.', linestyle='-', color='r')
#    block1.ax.set_xlim([1,100])
#    block1.ax.set_ylim([-1,15])
#    block1.ax.grid(True)
#    anim = amp.Animation([block1], timeline)
#
#    anim.controls()
#    anim.save_gif(results_dir+"Tension_membrane") #save animation for the docs
#    plt.show()
#    #plt.close('all')
#
#
#    # plt.show()  
#
#    return(X,T,Y,columns)
#
#
#
## In[Membrane force]
#def ContactForce(nb,nt,Fxc,Fyc,Fzc,time,script_dir,frames,rate):
#    
#    rows=np.arange(nb,nt,1) 
#    Fzc=np.delete(Fzc,rows,axis=0)
#    
#    results_dir = os.path.join(script_dir, 'Contact_forces/')
#
#    if not os.path.isdir(results_dir):
#        os.makedirs(results_dir)
#    F2x=Fzc
#    F2x=np.transpose(F2x)
#    x=np.linspace(1,nb,nb)
#    #time=np.transpose(time)
#    X2,T2 = np.meshgrid(x,time)
#    
#    columns=np.arange(0,len(time),frames)
#    
#    X=np.zeros((len(columns),nb))
#    T=np.zeros((len(columns),nb))
#    Fx=np.zeros((len(columns),nb))  
#    j=0
#    for i in (columns):
#        X[j,:]=X2[i,:]   
#        T[j,:]=T2[i,:]
#        Fx[j,:]=F2x[i,:]
#        j=j+1
#        
#    timeline = amp.Timeline(T,fps=rate)
#    block1 = amp.blocks.Line(X, Fx,marker='.', linestyle='-', color='r')
#    block1.ax.set_xlim([1,100])
#    block1.ax.set_ylim([-25,25])
#    block1.ax.grid(True)
#    anim = amp.Animation([block1], timeline)
#
#    anim.controls()
#    anim.save_gif(results_dir+"contact_forces") #save animation for the docs
#    plt.show()
#    #plt.close('all')
#
#
#    # plt.show()  
#
#    return(X,T,Fx,columns)
#
#
## In[Contact forces and position]
#def contactforce_position(nb,nt,qx,qy,qz,Fxc,Fyc,Fzc,time,frames,rate,script_dir):
#    
#    results_dir = os.path.join(script_dir, 'Contact_forces/')
#
#    if not os.path.isdir(results_dir):
#        os.makedirs(results_dir)
#    
#    # filter out the particles positions
#    rows=np.arange(nb,nt,1) 
#    qx=np.delete(qx,rows,axis=0)
#    qy=np.delete(qy,rows,axis=0)
#    qz=np.delete(qz,rows,axis=0)
#
#    Fxc=np.delete(Fxc,rows,axis=0)
#    Fyc=np.delete(Fyc,rows,axis=0)
#    Fzc=np.delete(Fzc,rows,axis=0)
#    
#    # Take the tranpose
#    qx=np.transpose(qx)
#    qy=np.transpose(qy)
#    qz=np.transpose(qz)
#    
#    Fxc=np.transpose(Fxc)
#    Fyc=np.transpose(Fyc)
#    Fzc=np.transpose(Fzc)
#    
#    
#    # create grid of time
#    T2=np.zeros((len(time),nb))
#    for i in range(nb):
#        T2[:,i]=time
#    columns=np.arange(0,len(time),frames)
#    
#    QX=np.zeros((len(columns),nb))
#    QZ=np.zeros((len(columns),nb))
#    T=np.zeros((len(columns),nb))
#    FXC=np.zeros((len(columns),nb))
#    FZC=np.zeros((len(columns),nb))
#        
#    j=0
#    for i in (columns):
#        QX[j,:]=qx[i,:]
#        QZ[j,:]=qz[i,:]
#        T[j,:]=T2[i,:]
#        FXC[j,:]=Fxc[i,:]
#        FZC[j,:]=Fzc[i,:]
#        j=j+1   
#    F=np.sqrt((FXC**2)+(FZC)**2)    
#    Sizes=10*(np.abs(F))+10
#    Color='b'
#
#    #Color='r'
#    timeline = amp.Timeline(T,fps=rate)
#    block1 = amp.blocks.Scatter(QX,QZ,Sizes,Color)
#    block1.ax.set_xlim([-3,3])
#    block1.ax.set_ylim([-2,2])
#    block1.ax.grid(True)
#    anim = amp.Animation([block1], timeline)
#
#    anim.controls()
#    anim.save_gif(results_dir+"contact_forces") #save animation for the docs
#    plt.show()
#    #plt.close('all')
#    #return(QX,T,FXC,Color)
#    


# In[Membrane force]
#def Tension(nb,Fmem,time,script_dir,qx,qy,qz):
#
#    results_dir = os.path.join(script_dir, 'Tension/')     
#    if not os.path.isdir(results_dir):
#        os.makedirs(results_dir)
#   
#    cmap = plt.cm.get_cmap('seismic')
#    boundaries=np.arange(10,30,.1)
#                       
#    norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
#    for i in range(1,len(time)):
#        F=Fmem[0:nc[i],i]
# 
#        #Fy=Fcy[0:nc[i],i]
#        abs_force=Fmem
#
#        name=str(i)
#
#        x=xc[0:nc[i],i]
#        y=zc[0:nc[i],i]
#        x2=[]
#        y2=[]
#        F2=[]
#        for j in range(len(abs_force)):
#           if abs_force[j]>=1:
#               x2.append(x[j])
#               y2.append(y[j])
#               F2.append(abs_force[j])
#               
#        
#         
#        plt.figure(i,figsize=(8,8),dpi=150)
#        plt.scatter(x2,y2,s=2*np.power(F2,.65),c=F2,cmap=cmap,norm=norm)
#        plt.xlim(-2,2)
#        plt.ylim(-2,2)
#        plt.grid(True)
#        plt.colorbar()
#        plt.xlabel('x position(m)')
#        plt.ylabel('z position(m)')
#        plt.title('t='+str(time[i]))
#        #cbar=plt.colorbar(path)
#        #cbar.set_label('Contact force')
#        plt.savefig(results_dir+"picture"+name+".jpg")  
#        plt.close('all')            
#        
#    return(abs_force)




   
