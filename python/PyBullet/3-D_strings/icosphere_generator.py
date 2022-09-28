# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 10:45:23 2022

@author: dmulr
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from icosphere import icosphere
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
nu = 1  # or any other integer
vertices, faces = icosphere(nu)
(m,n)=np.shape(vertices)
print(m)
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles = faces,edgecolor=[[0,0,0]], linewidth=.25, alpha=0.25,color='tab:red', shade=False)
