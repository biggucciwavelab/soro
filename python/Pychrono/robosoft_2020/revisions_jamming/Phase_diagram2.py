# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 08:55:59 2020

@author: dmulr
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from matplotlib import colors as colors
import matplotlib as mpl
import matplotlib.cm as cm
from bridson import poisson_disc_samples
import matplotlib.lines as mlines
import statistics 
import os
from Phase_diagram_plotters import voronoi_finite_polygons_2d,voronoi_Area,Make_P_mat,Packing,Stats_packing,Phase_diagram,Voronoi_contour



# In[Load data]
nb=np.load('nb.npy')
k=np.load('k.npy')
PHI2=np.zeros((len(nb),len(k)))

# In[Voronoi Plots]

n=0
m=0
count=0
for i in nb:
    m=0
    for j in k:
        count=count+1        
        print(count)
        data=np.load('F:/Robosoft2020_data/revisiions_jamming/trial11/'+'data_folder/'+str(count)+'.npz',allow_pickle=True)

        # Positions
        qx=data['qx']
        qy=data['qy']
        qz=data['qz']
        nb=data['nb']
        ni=data['ni']
        diameter=data['diameter']

        
        
        (X,Y,P)=Make_P_mat(nb,ni,qx,qz)
        vor = Voronoi(P)
        regions, vertices = voronoi_finite_polygons_2d(vor)
        A=[]
        Aact=np.pi*(diameter/2)**2
        for l in range(len(regions)):
            region=regions[l]
            A.append(Aact/(voronoi_Area(region,vertices)))  
        phi,Abar=Packing(A)
        PHI2[n,m]=phi
        Stats_packing(Abar,i,j)
        Voronoi_contour(vertices,regions,A,vor,j,i)
        m=m+1
    n=n+1

np.save('PHI2',PHI2)

PHI2=np.load('PHI2.npy')
PHI2=np.asarray(PHI2)

nb=np.load('nb.npy')
k=np.load('k.npy')
Phase_diagram(nb,k,PHI2)




   
    
