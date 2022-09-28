# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 09:54:03 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from grab_sim_objects import *

from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt
import os
import matplotlib.pyplot as plt
from matplotlib import animation
import animatplot as amp
from matplotlib import colors as colors
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull

class plots:
    def __init__(self,k,alpha,E,path,avgFxc,avgFyc,avgFzc):
        
        self.k=k
        self.alpha=alpha
        self.E=E
        self.path=path
        self.avgFxc=avgFxc
        self.avgFyc=avgFyc
        self.avgFzc=avgFzc
        self.avgFc=((self.avgFxc)**2+(self.avgFyc)**2+(self.avgFzc)**2)**(1/2)
    def Error_plot(self):
        
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (fsx, fsy))     # Set the figure size to square
        xx, yy = np.meshgrid(self.k, self.alpha)
        plt.pcolormesh(xx, yy, self.E,cmap = 'jet')
        plt.colorbar()
        plt.title("Error plot")
        plt.xlabel("$spring force(N/m)$")
        plt.ylabel("$alpha$")
        plt.savefig(self.path+'/Error.png')
# Xcontact force
    def avg_x_contact(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (fsx, fsy))     # Set the figure size to square
        xx, yy = np.meshgrid(self.k, self.alpha)
        plt.pcolormesh(xx, yy,self.avgFxc,cmap = 'jet')
        plt.colorbar()
        plt.title("average X contact plot")
        plt.xlabel("$spring force(N/m)$")
        plt.ylabel("$alpha$")
        plt.savefig(self.path+'/Xcontact.png')    
# Y contact force        
    def avg_y_contact(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (fsx, fsy))     # Set the figure size to square
        xx, yy = np.meshgrid(self.k, self.alpha)
        plt.pcolormesh(xx, yy,self.avgFyc,cmap = 'jet')
        plt.colorbar()
        plt.title("average Y contact plot")
        plt.xlabel("$spring force(N/m)$")
        plt.ylabel("$alpha$")
        plt.savefig(self.path+'/Ycontact.png')         
        
 # Z contact force        
    def avg_z_contact(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (fsx, fsy))     # Set the figure size to square
        xx, yy = np.meshgrid(self.k, self.alpha)
        plt.pcolormesh(xx, yy,self.avgFzc,cmap = 'jet')
        plt.colorbar()
        plt.title("average Z contact plot")
        plt.xlabel("$spring force(N/m)$")
        plt.ylabel("$alpha$")
        plt.savefig(self.path+'/Zcontact.png') 
        
 # All contact force        
    def avg_contact(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (fsx, fsy))     # Set the figure size to square
        xx, yy = np.meshgrid(self.k, self.alpha)
        plt.pcolormesh(xx, yy,self.avgFc,cmap = 'jet')
        plt.colorbar()
        plt.title("average  contact plot")
        plt.xlabel("$spring force(N/m)$")
        plt.ylabel("$alpha$")
        plt.savefig(self.path+'/Allcontact.png')          

    def avg_contact_3d(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (fsx, fsy))     # Set the figure size to square
        ax = fig.gca(projection='3d')                   # Include axes
        xx, yy = np.meshgrid(self.k, self.alpha)
        surf = ax.plot_surface(xx, yy, self.avgFc, cmap = 'jet')   # Plot the 3-D surface using the "jet" color map
        fig.colorbar(surf)
        plt.title("average  contact plot")
        plt.xlabel("$spring force(N/m)$")
        plt.ylabel("$alpha$")
        plt.savefig(self.path+'/Allcontact3D.png')                                      # Include color bar
   