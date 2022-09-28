# -*- coding: utf-8 -*-
"""
Obj file writer

@author: qzhou
"""

import numpy as np
from numpy import genfromtxt
from stl import mesh
import pymesh

faces='elements.csv'
vertices='nodes.csv'
indices='indices.csv'

faces=genfromtxt(faces,delimiter=',',dtype=int)
vertices=genfromtxt(vertices,delimiter=',',dtype=float)
indices=genfromtxt(indices,delimiter=',',dtype=int)

thefile = open('test.obj', 'w')
for i in range(len(vertices)):
  thefile.write("v %0.12f %0.12f %0.12f\n" % (vertices[i,0],vertices[i,1],vertices[i,2]))
  
thefile.write("usemtl Default")

for i in range(len(faces)):
  thefile.write("f %i %i %i %i\n" % (np.where(indices==faces[i,0])[0][0]+1,np.where(indices==faces[i,1])[0][0]+1,\
                                     np.where(indices==faces[i,2])[0][0]+1,np.where(indices==faces[i,3])[0][0]+1))  

thefile.close()

