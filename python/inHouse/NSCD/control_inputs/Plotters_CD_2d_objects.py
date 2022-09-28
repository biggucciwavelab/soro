# -*- coding: utf-8 -*-
"""
Created on Fri Feb 21 12:03:21 2020

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

# In[Plot forces]
def PlotForces(F,nb,nt,time):

    fig, (ax1, ax2) = plt.subplots(2, constrained_layout=True)
    for i in range(nb):
        ax1.plot(time,F[3*i,:],label=str(i))
        ax1.set_title('X Force[N]')
        plt.legend()
        ax2.plot(time,F[3*i+1,:],label=str(i))
        ax2.set_title('Y Force[N]')
        plt.legend()
# In[Plot error]
def PlotError(E,nb,nt,time):
    fig, (ax1, ax2) = plt.subplots(2, constrained_layout=True)
    for i in range(nb):
        ax1.plot(time,E[2*i,:],label=str(i))
        ax1.set_title('X Error')
        plt.legend()
        ax2.plot(time,E[2*i+1,:],label=str(i))
        ax2.set_title('Y Error') 
        plt.legend()
        
# In[Error Plots for the rbf]
def Error_rbf(E,time):
    fig, (ax1) = plt.subplots(1, constrained_layout=True)
    ax1.plot(time,np.transpose(E),color='b')
    ax1.set_title('Error Plots')
    plt.grid(True)
   
# In[Plot Contacts]
def PlotContact(Fcontact,nb,nt,time):
    fig, (ax1, ax2) = plt.subplots(2, constrained_layout=True)
    for i in range(nb):
        ax1.plot(time,Fcontact[3*i,:],label=str(i))
        ax1.set_title('Contact force X')
        ax2.plot(time,Fcontact[3*i+1,:],label=str(i))
        ax2.set_title('Contact force Y')         
# In[Plot boundary Force]
def PlotBoundaryForce(Fbound,nb,nt,time):   
    fig, (ax1, ax2) = plt.subplots(2, constrained_layout=True)
    for i in range(nb):
        ax1.plot(time,Fbound[2*i,:],label=str(i))
        ax1.set_title('Boundary force X')
        ax2.plot(time,Fbound[2*i+1,:],label=str(i))
        ax2.set_title('Boundary force Y')         

# In[COM desired vs COM actual]
        
def COM_comared(COMA,COM,time):
    fig, (ax1,ax2) = plt.subplots(2, constrained_layout=True)
    ax1.plot(COM[0,0:time.size-1],COMA[0,0:time.size-1],color='b')
    ax1.set_title('COM x actual vs desired x')
    ax1.grid(True)
    ax2.scatter(COM[1,:],COMA[1,:],color='r')
    ax2.set_title('COM y actual vs desired y')
    ax2.grid(True)
# In[Average error vs time]    
def Plot_error_avg(ELAVG,time):
    fig, (ax1) = plt.subplots(1, constrained_layout=True)
    ax1.plot(time,np.transpose(ELAVG),color='b')
    ax1.set_title('Average Error vs time')
    ax1.grid(True) 
    plt.ylim((0,1))
# In[Plot Current position]
def plotObj(obj,left,right,up,down,q,time,nt,script_dir,R,pointx,pointy):
    
    results_dir = os.path.join(script_dir,'Animation/')     
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)
    count=0
    for i in range(len(time)):
        if i%2==0:
            count=count+1
            bx=[left,right,right,left,left]
            by=[down,down,up,up,down]
            #Plotting the circles
            cPatches,cLines=[],[]
            fig = plt.figure(i)
            fig.set_dpi(600)
            fig.set_size_inches(12, 12)
            
            ax = plt.axes(xlim=(left-.5, right+.5), ylim=(down-.5, up+.5))
            lineH = plt.Line2D(bx, by, lw=1)
            theta=np.linspace(0,2*np.pi,num=100)
            xb=R*np.cos(theta)+pointy[i]
            yb=R*np.sin(theta)+pointx[i]
#            xb=0.4*np.array([1,1,0,-1,-1,-1,0,1,1])
#            yb=0.4*np.array([0,1,1,1,0,-1,-1,-1,0])
            
#            theta1=np.linspace(np.pi/4,7*np.pi/4,50)
#            x1=.75*np.cos(theta1)
#            y1=.75*np.sin(theta1)
#            theta2=np.linspace(3*np.pi/2,np.pi/2,20)
#            x2=.5*np.cos(theta2)+.6
#            y2=.4*np.sin(theta2)  
    
            #xb=np.concatenate((x2, x1), axis=None)
            #yb=np.concatenate((y2, y1), axis=None)
            Target=plt.Line2D(xb,yb,color='k')
            ax.add_line(Target)
            ax.add_line(lineH)
            ax.grid(True)
            for j in range(0,nt):
                x=q[3*j,i]
                y=q[3*j+1,i]
                phi=q[3*j+2,i]
                r=obj[j].R
                if(obj[j].type=='particle'):
                    patch=plt.Circle((x,y),r,color='r')
                    patch.center=(x, y)
                    lineC1 = plt.Line2D((x+r*np.cos(phi+np.pi), x+r*np.cos(phi)), (y+r*np.sin(phi+np.pi), y+r*np.sin(phi)),color='k')
                    lineC2 = plt.Line2D((x+r*np.cos(phi+np.pi+np.pi/2), x+r*np.cos(phi+np.pi+np.pi/2)), (y+r*np.sin(phi+np.pi+np.pi+np.pi/2), y+r*np.sin(phi+np.pi+np.pi/2)), color='k')
                    cPatches.append(patch)
                    cLines.append(lineC1)
                    cLines.append(lineC2) 
                if(obj[j].type=='robot'):
                    patch=plt.Circle((x,y),r,color='g')
                    patch.center=(x, y)
                    lineC1 = plt.Line2D((x+r*np.cos(phi+np.pi), x+r*np.cos(phi)), (y+r*np.sin(phi+np.pi), y+r*np.sin(phi)),color='k')
                    lineC2 = plt.Line2D((x+r*np.cos(phi+np.pi+np.pi/2), x+r*np.cos(phi+np.pi+np.pi/2)), (y+r*np.sin(phi+np.pi+np.pi+np.pi/2), y+r*np.sin(phi+np.pi+np.pi/2)), color='k')
                    cPatches.append(patch)
                    cLines.append(lineC1)
                    cLines.append(lineC2) 
        
            for ii in range(0,len(cPatches)):
                ax.add_patch(cPatches[ii])
                ax.add_line(cLines[2*ii])
                ax.add_line(cLines[2*ii+1])               

            plt.xlabel('x position(m)')
            plt.ylabel('z position(m)')
            plt.title('t='+str(round(time[i],3)))
            plt.grid(True)
            plt.savefig(results_dir+"picture"+str(count)+".jpg")  
            plt.close('all')            

# In[L2 norm plot]
            
def L2Norm(EL2,nb,nt,time):

    fig, (ax1) = plt.subplots(1, constrained_layout=True)
    ax1.plot(time,np.transpose(EL2))
    ax1.set_title('L2 norm of error')
    plt.legend()