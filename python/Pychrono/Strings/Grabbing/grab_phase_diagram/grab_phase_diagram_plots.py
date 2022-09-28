# -*- coding: utf-8 -*-
"""
Created on Tue Jun  2 21:10:38 2020

@author: dmulr
"""

import numpy as np
import timeit
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
from grab_phase_diagram_plots_objects import *

file="NPZ3_files/file.npz"
data=np.load(file,allow_pickle=True)

E=data["errors"]
avgFxc=data["avgFxc"]
avgFyc=data["avgFyc"]
avgFzc=data["avgFzc"]
k=data["k"]
alpha=data["alpha"]
#k=[100,150,200,250,300,350,400,450,500,550,600]
#alpha=[100,150,200,250,300,350,400,450,500]


path = os.path.join("plots")     

if not os.path.isdir(path):
    os.makedirs(path)
    
graphs=plots(k,alpha,E,path,avgFxc,avgFyc,avgFzc)

graphs.Error_plot()
graphs.avg_x_contact()
graphs.avg_y_contact()
graphs.avg_z_contact()
graphs.avg_contact()

graphs.avg_contact_3d()