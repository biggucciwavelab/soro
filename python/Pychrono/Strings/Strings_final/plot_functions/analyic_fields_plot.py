# -*- coding: utf-8 -*-
"""
Created on Sun Apr 11 14:12:51 2021

@author: dmulr
"""
import numpy as np
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt


class analytic_field:
    def __init__(self,a,c,px,py,theta,b,res):
        self.px=px # center x
        self.py=py # center y
        self.a=a # radii 1
        self.b=b # range of field
        self.c=c # radii 2
        self.theta=theta # rotation
        self.res=res # resolution of the field so how big are the squares 
        self.phi=np.linspace(0,2*np.pi,100)
        self.xt=self.a*np.cos(self.phi)
        self.yt=self.c*np.sin(self.phi)
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field 
    
        Y=(self.xx)*np.sin(self.theta)+(self.yy)*np.cos(self.theta) # Y value of distance
        X=(self.xx)*np.cos(self.theta)-(self.yy)*np.sin(self.theta) # X value of distance
        d=np.sqrt((X**2) / (self.a**2) + (Y**2)/(self.c**2)) # d value 
        self.zz = d**2 * np.log(d) # create field gird form 
        self.zz=self.zz/np.max(self.zz)
        (self.fy,self.fx)=np.gradient(self.zz**2) # take gradient 
        self.fx=self.fx/np.max(self.fx) # normaluze gradient x term
        self.fy=self.fy/np.max(self.fy) # normalize gradient y term 
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz) # form potential function 
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx) # form gradient of potnetial function x
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)  # form gradient of potential function y
    
    def plot_2D(self):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        plt.contourf(self.xx,self.yy,self.zz**2,cmap='jet',levels=50,alpha=1,linestyles='solid')
        #plt.pcolormesh(self.yy,self.xx, self.zz,alpha=1, cmap = 'jet',rasterized=False,zorder=-10)         # Pseudocolor plot of data and choose colormap
        plt.colorbar()
        plt.plot(self.xt,self.yt,color='white')
        plt.title('$f(x,y)$ for a Circle',fontname="Arial", fontsize=14 )
        plt.xlabel("$x$ meters",fontname="Arial", fontsize=14)
        plt.ylabel("$y$ meters",fontname="Arial", fontsize=14)
        #plt.show()
        
    def plot_2D_lines(self):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        plt.contourf(self.xx,self.yy,self.zz**2,levels=50)
        plt.plot(self.xt,self.yt,color='white')
        #ax.clabel(CS, inline=1, fontsize=10)
        plt.title('plot')
        plt.xlabel("$x$ meters",fontname="Arial", fontsize=12)
        plt.ylabel("$y$ meters",fontname="Arial", fontsize=12)
        
    def slice_plot(self):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        y=np.zeros(len(self.xp))
        z=self.f((y,self.xp))
        print(np.shape(y),np.shape(z))
        plt.plot(self.xp,z,color='tab:blue',label='$f(x,y)$')
        #plt.plot(self.xp,z**2,color='tab:red',label='$f(x,y)^{2}$')        
        plt.title('$f(x,y)$ for a Circle',fontname="Arial", fontsize=14 )
        plt.xlabel("$x$ meters",fontname="Arial", fontsize=14)
        plt.ylabel("$y$ meters",fontname="Arial", fontsize=14)
        plt.legend()
        plt.grid(True)  
        
a=1
c=1
px=0
py=0
theta=np.pi/2
b=3
res=.01
phi=analytic_field(a,c,px,py,theta,b,res)
phi.plot_2D()
phi.slice_plot()