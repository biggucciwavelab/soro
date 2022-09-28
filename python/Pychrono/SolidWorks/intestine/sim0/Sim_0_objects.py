# -*- coding: utf-8 -*-
"""
Created on Sat Apr  4 16:49:11 2020

@author: dmulr
"""

import os
import math
import time
import sys, getopt
import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib import animation


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z

# In[Class to store Robot data]
class robot_data:
    def __init__(self,number,marker,goal):
        self.qx=[]
        self.qy=[]
        self.qz=[]
        self.qxb=[]
        self.qyb=[]
        self.qzb=[]
        self.theta=[]
        self.id=number
        self.marker=marker
        self.gx=goal[0]
        self.gy=goal[1]
        self.gz=goal[2]
        
    # Extract data
    def Extract_data(self):
        self.qxb.append(self.marker.Point_Ref2World(chrono.ChVectorD(self.gx,self.gy,self.gz)).x)
        self.qyb.append(self.marker.Point_Ref2World(chrono.ChVectorD(self.gx,self.gy,self.gz)).y)
        self.qzb.append(self.marker.Point_Ref2World(chrono.ChVectorD(self.gx,self.gy,self.gz)).z)
        self.qx.append(self.marker.Point_Ref2World(chrono.ChVectorD(0,0,0)).x)
        self.qy.append(self.marker.Point_Ref2World(chrono.ChVectorD(0,0,0)).y)
        self.qz.append(self.marker.Point_Ref2World(chrono.ChVectorD(0,0,0)).z)
        #self.theta.append()
    # Plot data
    def Plot_robot(self):
        plt.plot(self.qx,self.qz,'b',self.gx,self.gz,'gs')


        
