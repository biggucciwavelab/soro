# -*- coding: utf-8 -*-
"""
Created on Mon May 24 19:40:51 2021

@author: dmulr
"""
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import pywavefront
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from descartes import PolygonPatch
#import alphashape
sys.path.insert(0, os.path.dirname(os.getcwd()))
path="F:/Soro_chrono/python/Pychrono/Strings/Strings_final/shapes/star50.obj"
scene = pywavefront.Wavefront(path)

faces=scene.mesh_list[0].faces
vertices=scene.vertices

X=[]
Y=[]
Z=[]
for i in range(len(vertices)):
    temp=vertices[i]
    X.append(temp[0]) 
    Y.append(temp[1]) 
    Z.append(temp[2])     
    
X=np.asarray(X)
Z=np.asarray(Z)
X=np.matrix(X)
Z=np.matrix(Z)
plt.plot(X,Z,'sb')   
points=np.vstack([X,Z]) 
#alpha_shape = alphashape.alphashape(points, 0.)


fig = plt.figure()
ax = fig.add_subplot(projection='3d')    
ax.scatter(X, Y, Z)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')


print("Faces:", scene.mesh_list[0].faces)
print("Vertices:", scene.vertices)
print("Format:", scene.mesh_list[0].materials[0].vertex_format)
print("Vertices:", scene.mesh_list[0].materials[0].vertices)