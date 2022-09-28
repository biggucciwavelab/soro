# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 13:55:35 2022

@author: dmulr
"""

from numpy import pi
import numpy as np
import pathlib
import sys
import pywavefront
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from icosphere import icosphere

file_path = pathlib.Path(__file__).parent.resolve()
sys.path.insert(0, os.path.dirname(os.getcwd()))


shape = '162_v3.obj'
shape_path = "F:/Soro_chrono/python/PyBullet/3-D_strings/Blender/"
obj_geom = shape_path + shape
scene = pywavefront.Wavefront(obj_geom)

faces1=scene.mesh_list[0].faces
vertices1=scene.vertices
vertices1=np.asarray(vertices1)
(m1,n1)=np.shape(vertices1)
print(m1)


nu = 2  # or any other integer
vertices2, faces2 = icosphere(nu)
vertices2=np.dot(0.25,vertices2)
(m2,n2)=np.shape(vertices2)
print(m2)


pos_vectors=np.zeros((m2,m1))
pos_vectors_list={}
min_value=[]

for i in range(m2):
    pos1=vertices2[i,:]
    pos_vectors_list["bot{0}".format(i)]=[]
    for j in range(m1):
        pos2=vertices1[j,:]
        posL2=np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)
        pos_vectors[i,j]=posL2
        pos_vectors_list["bot"+str(i)].append(posL2)
    index_min = np.argmin(pos_vectors_list["bot"+str(i)])
    min_value.append(index_min)


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_trisurf(vertices1[:,0], vertices1[:,1], vertices1[:,2], triangles = faces1,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
ax.scatter3D(vertices1[min_value,0],vertices1[min_value,1],vertices1[min_value,2], c='k')
plt.show()


print(min_value)

np.savez('42.npz',bot_number=min_value)
#print(m2/m1)   
#fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,1),dpi=300)
#im1=axs.pcolormesh(pos_vectors, cmap='jet')
#axs.invert_yaxis()
#axs.set_title('A lifted')
#fig.colorbar(im1, ax=axs)    


