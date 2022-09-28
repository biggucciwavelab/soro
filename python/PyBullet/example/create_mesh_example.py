# -*- coding: utf-8 -*-
"""
Created on Sun Jul 18 16:22:22 2021

@author: dmulr
"""

import pywavefront
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import openmesh as om
import numpy as np
path="C:/Users/dmulr/anaconda3/Lib/site-packages/bullet3-master/data/cloth_z_up.obj"
scene = pywavefront.Wavefront(path, collect_faces=True)

vertices=scene.vertices
faces=scene.mesh_list[0].faces


w = .5
h = .1
Vert_temp=np.array([[w/2,-h/4,0], # 0
          [w/4,-h/2,0], # 1
          [w/4,-h/4,0], # 2
          [0,-h/4,0], # 3
          [-w/4,-h/2,0], # 4
          [-w/4,-h/4,0], # 5 
          [0,h/4,0], # 6
          [-w/4,0,0], # 7
          [-w/4,h/4,0], # 8
          [w/2,h/4,0], # 9
          [w/4,0,0], # 10
          [w/4,h/4,0], # 11
          [0,0,0], # 12
          [w/4,h/2,0], # 13
          [0,h/2,0], # 14
          [w/2,h/2,0], # 15
          [-w/2,0,0], # 16
          [-w/2,h/4,0], # 17
          [-w/4,h/2,0], # 18 
          [-w/2,h/2,0], # 19
          [-w/2,-h/2,0], # 20
          [-w/2,-h/4,0], # 21
          [0,-h/2,0], # 22
          [w/2,0,0], # 23
          [w/2,-h/2,0]]) # 24

V = []
mesh = om.TriMesh()
for i in range(len(vertices)):
    temp=Vert_temp[i,:]
    temp=temp.flatten()
    V.append(mesh.add_vertex([temp[0],temp[1],temp[2]]))

vh_list = []  
for i in range(len(faces)):
    temp = faces[i]
    mesh.add_face([V[temp[0]],V[temp[1]],V[temp[2]]])

    vh_list.append([V[temp[0]],V[temp[1]],V[temp[2]]])

om.write_mesh('filename.obj', mesh)


X,Y = np.meshgrid(Vert_temp[:,0],Vert_temp[:,1])
plt.scatter(X,Y)

segs1 = np.stack((X,Y), axis=2)
segs2 = segs1.transpose(1,0,2)
plt.gca().add_collection(LineCollection(segs1))
plt.gca().add_collection(LineCollection(segs2))
plt.show()


















# edges=mesh.edges()

# x=[]

# y=[]

# for i in range(len(vertices)):
#     x.append(vertices[i][0])
#     y.append(vertices[i][1])


# V=[]
# mesh = om.TriMesh()
# for i in range(len(vertices)):
#     temp=np.asarray(vertices[i])
#     temp=temp.flatten()
#     V.append(mesh.add_vertex([temp[0],temp[1],temp[2]]))

# vh_list = []  
# for i in range(len(faces)):
#     temp = faces[i]
#     mesh.add_face([V[temp[0]],V[temp[1]],V[temp[2]]])

#     vh_list.append([V[temp[0]],V[temp[1]],V[temp[2]]])

# edges=mesh.edges()

# om.write_mesh('filename.obj', mesh)
