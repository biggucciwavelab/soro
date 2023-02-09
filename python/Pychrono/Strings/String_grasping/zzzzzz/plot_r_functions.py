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

Psi=sim_obj.R_functions(name)

delta = 0.01
d=1.2
x = np.arange(-d, d, delta)
y = np.arange(-d, d, delta)
(X,Y) = np.meshgrid(x, y)




# ## Pacman
# F=Psi.F(X,Y)
# Fx=Psi.FY(X,Y)
# Fy=Psi.FX(X,Y)


# Psi.plot_R_function(X,Y,F,Fx,Fy,d)
#### Wrench #####
scale=2
segments=Psi.segments
xp=np.hstack([segments[:,0],segments[0,0]])
yp=np.hstack([segments[:,1],segments[0,1]])

plt.plot(xp,yp)

delta = 0.01
d=1
x = np.arange(-d, d, delta)
y = np.arange(-d, d, delta)
(X,Y) = np.meshgrid(x, y)
R=Psi.phi_segments(X,Y,segments)
Rx=Psi.dphix_segments(X,Y,segments)
Ry=Psi.dphiy_segments(X,Y,segments)



Psi.plot_R_function(X,Y,xp,yp,R,Rx,Ry,d)