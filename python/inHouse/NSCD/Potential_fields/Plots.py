# -*- coding: utf-8 -*-
"""
Created on Fri Feb 21 12:10:08 2020

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
from Plotters_CD_2d_objects import *


data = np.load('7.npz',allow_pickle=True)

q=data['q']
nb=data['nb']
ni=data['ni']
obj=data['obj']
left=data['left']
right=data['right']
up=data['up']
down=data['down']
R=data['R1']
t0=data['t0']
tend=data['tend']
time=data['time']
E=data['E']
R=data['R']
#Fcontact=data['Fcontact']
#Fbound=data['Fbound']
#EL2=data['EL2']
script_dir = os.path.dirname("plots7/")


#Plot_error_avg(ELAVG,time)
#Error_rbf(E,time)

#L2Norm(EL2,nb,nt,time)
#COM_comared(COMA,COM,time)
plotObj(obj,left,right,up,down,q,time,nt,script_dir,R)

#PlotForces(F,nb,nt,time)
#PlotError(E,nb,nt,time)
#PlotContact(Fcontact,nb,nt,time)
#PlotBoundaryForce(Fbound,nb,nt,time)
#Frame_by_Frame(obj,left,right,up,down,q,nb,ni,nt,time)