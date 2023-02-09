# -*- coding: utf-8 -*-
"""
Created on Wed Nov 17 12:35:50 2021

@author: dmulr
"""

import numpy as np
import random
import os
import csv
from csv import writer
import matplotlib.pyplot as plt
import objects as sim_obj
direct=os.path.dirname(__file__)
path=direct+"/Experiments"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]
#name='27_12_2021_15_04_00'
plt.close('all')
Psi=sim_obj.R_functions(name)
delta = 0.01
d=1.5
nr=2
nc=2
t=np.linspace(0,1,4)
x = np.arange(-d, d, delta)
y = np.arange(-d, d, delta)
(X,Y) = np.meshgrid(x, y)

phi1=Psi.F1(X,Y)
phi2=Psi.F2(X,Y)

dphi1x=Psi.F1X(X,Y)
dphi2x=Psi.F2X(X,Y)

dphi1y=Psi.F1Y(X,Y)
dphi2y=Psi.F2Y(X,Y)

theta=np.linspace(0,2*np.pi,30)
xp1=Psi.R*np.cos(theta)
yp1=Psi.R*np.sin(theta)



segments=Psi.segments
xp2=np.hstack([segments[:,0],segments[0,0]])
yp2=np.hstack([segments[:,1],segments[0,1]])

Psi.plot_R_function(X,Y,phi1,dphi1x,dphi1y,d)
Psi.plot_R_function(X,Y,phi2,dphi2x,dphi2y,d)
Psi.plot_zero_contours(xp1,yp1,xp2,yp2,d)


Psi.plot_R_function_morph(X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t)


Psi.plot_R_function_morph_color(X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t)
#C_morphy(self,phi1,phi2,dphi1y,dphi2y,t)


# scale=0.5
# segments=scale*Psi.segments
# delta = 0.01
# d=1
# x = np.arange(-d, d, delta)
# y = np.arange(-d, d, delta)
# (X,Y) = np.meshgrid(x, y)
# R=Psi.phi_segments(X,Y,segments)
# Rx=Psi.dphix_segments(X,Y,segments)
# Ry=Psi.dphiy_segments(X,Y,segments)

# xp=np.hstack([segments[:,0],segments[0,0]])
# yp=np.hstack([segments[:,1],segments[0,1]])

#Psi.plot_R_function(X,Y,R,Rx,Ry,xp,yp,d)