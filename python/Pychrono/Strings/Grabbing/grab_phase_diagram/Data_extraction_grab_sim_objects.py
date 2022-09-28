# -*- coding: utf-8 -*-
"""
Created on Thu Jun  4 10:32:32 2020

@author: dmulr
"""
import numpy as np
import timeit
from os import listdir
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import colors as colors
import animatplot as amp
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator


class data_extraction:
    def __init__(self,filename,nb,ni):
        self.filename=filename
        self.data = np.genfromtxt(self.filename, delimiter=',')
        self.nb=nb
        self.ni=ni
        (self.m1,self.n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:self.n1]
        self.time=self.data1[0,:]
        self.xpos=self.data1[1:nb+1,:]
        self.ypos=self.data1[nb+1:2*nb+1,:]
        self.zpos=self.data1[(2*nb)+1:3*nb+1,:]
        
            
        
