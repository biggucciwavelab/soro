# -*- coding: utf-8 -*-
"""
Created on Sun Apr 11 15:39:45 2021

@author: dmulr
"""
import numpy as np
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt


class Shape_fields:
    def __init__(self,R,px,py,b,res,shape):
        self.px=px # center x
        self.py=py # center y
        self.R=R
        self.b=b # range of field
        self.res=res # resolution of the field so how big are the squares 
        self.shape=shape
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        self.nr=[0,1,2]
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field    
        (self.x,self.y,self.z)=self.points_for_region()
        self.rbf = Rbf(self.x,self.y,self.z,function='thin_plate')  # radial basis function interpolator instance    
        self.zz = self.rbf(self.xx, self.yy) # create zz grid
        self.zz=np.nan_to_num(self.zz/np.max(self.zz)) # normalize the zz grid
        self.XT=self.x[0:int(len(self.x)/len(self.nr))]
        self.YT=self.y[0:int(len(self.x)/len(self.nr))]        
    def points_for_region(self):
        if self.shape=='Square':
            (x,y,z)=self.points_square_region()
            
        if self.shape=='Triangle':
            (x,y,z)=self.points_triangle_region()   
            
        if self.shape=='Circle':
            (x,y,z)=self.points_circle_region()               
        return(x,y,z)
    
    def points_square_region(self):
        xt=np.array([1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25,0,.25,.5,.75,1,1,1,1])
        yt=np.array([0,.25,.5,.75,1,1,1,1,1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25])
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j       
        return (x,y,z) 
    
    def points_triangle_region(self):
        #n=3
        m=3
        n1=4.5
        n2=10
        n3=10
        a=1
        b=1
        theta=np.linspace(0,2*np.pi,100)
        #(xt,yt)=self.super_ellipse(m,n1,n2,n3,a,b,theta)
        n=3
        r=np.cos(np.pi/n)/np.cos((theta%(2*np.pi/n))-(np.pi/n))
        xt=r*np.cos(theta)
        yt=r*np.sin(theta)
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))

        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.nr[j]+self.R)*xt[i]
                y[i+len(xt)*j]=(self.nr[j]+self.R)*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j 
                           
        return (x,y,z)  
    
    def points_circle_region(self):
        theta=np.linspace(0,2*np.pi,30)
        xt=self.R*np.cos(theta)
        yt=self.R*np.sin(theta)
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j       
        return (x,y,z) 
            
    def plot_2D(self,typed):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        plt.pcolor(self.xx,self.yy, self.zz, cmap = 'jet')         # Pseudocolor plot of data and choose colormap
        plt.plot(self.XT,self.YT,color='white')
        plt.colorbar()  
        plt.title('$f(x,y)$ for a '+typed,fontname="Arial", fontsize=14 )
        plt.xlabel("$x$ meters",fontname="Arial", fontsize=14)
        plt.ylabel("$y$ meters",fontname="Arial", fontsize=14)
        plt.show()
        
    def plot_2D_lines(self):
        fig, ax = plt.subplots(figsize = (8, 8))
        CS = ax.contour(self.xx,self.yy,self.zz,10)
        ax.clabel(CS, inline=1, fontsize=10)
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")    
        
    def slice_plot(self,typed):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        y=np.zeros(len(self.xp))
        z=self.rbf(y,self.xp)
        plt.plot(self.xp,z**2,color='tab:blue',label='$f(x,y=0)$')
        plt.plot(self.xp,z,color='tab:red',label='$f(x,y=0)^{2}$')    
        
        plt.title('$f(x,y=0) & f(x,y=0)^{2}$ for a '+typed,fontname="Arial", fontsize=14 )
        plt.xlabel("$x$ meters",fontname="Arial", fontsize=14)
        plt.ylabel("$z$ meters",fontname="Arial", fontsize=14)
        plt.legend()
        plt.grid(True)   
        
R=1
px=0
py=0
b=2
res=0.01
shape='Square'
phi=Shape_fields(R,px,py,b,res,shape)
phi.plot_2D(shape)
phi.plot_2D_lines()
phi.slice_plot(shape)        
        