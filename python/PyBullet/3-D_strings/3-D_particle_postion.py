# -*- coding: utf-8 -*-
"""
Created on Sat Jan 15 13:09:38 2022

@author: dmulr
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

def frange(start, stop, step):
    i = start
    while i<=stop:
        yield i
        i += step

# r = .35 #radius of shell
# num_x = 12       #Best if it's an even number
# num_y = 12       #Best if it's an even number
# num_z = 12      #Best if it's an even number
# radius = .008
# xs=[]
# ys=[]
# zs=[]
# for ix in frange(-num_x/2,num_x/2,1):
#     for iy in frange(-num_y/2,num_y/2,1):
#         for iz in frange(-num_x/2,num_z/2,1):

#             xs.append(ix*(2.01*radius))
#             ys.append(iy*(2.01*radius)+r)
#             zs.append(iz*(2.01*radius))

# fig = plt.figure(figsize = (6, 6))          # Set the figure size
# ax = fig.gca(projection='3d')                   # Include axes
# ax.scatter(xs, ys, zs, color = "green")



xs=[]
ys=[]
zs=[]
r = .25#radius of shell
bot_width=.01
particle_width=.05


segs=20
xsegs=np.linspace(-r,r,segs)

rad_seg=np.sqrt(r**2-xsegs**2)

ri=[]
ni=[]
for i in range(len(rad_seg)):
    

    Rin=rad_seg[i]-(particle_width/2)#-(self.particle_width/2)
    ngrans1=int(Rin/(particle_width))
    rit=np.zeros((1,ngrans1))
    nit=np.zeros((1,ngrans1))
    radii=Rin-(particle_width)
    for j in range(ngrans1):
        remainder=((particle_width))*j
        rit[:,j]=radii-remainder
        nit[:,j]=np.floor(((rit[:,j]*np.pi)/(particle_width/2)))
    ri.append(rit)
    ni.append(nit)


for k in range(len(rad_seg)):
#     particle_width
    n=np.asarray(ni[k],dtype=int)
    N=n[0]
    r=ri[k]
    Ri=r.flatten()

    for i in range(N.size):
        for j in range(N[i]):
            R2=(particle_width)*N[i]/np.pi
            x=xsegs[k]
            z=R2*np.cos(j*2*np.pi/N[i])
            y=R2*np.sin(j*2*np.pi/N[i])
            
            xs.append(x)
            ys.append(y)
            zs.append(z)

        
        
        
ran=-1
  
fig = plt.figure(figsize = (6, 6))          # Set the figure size
ax = fig.gca(projection='3d')                  # Include axes
ax.scatter(xs[0:ran], ys[0:ran], zs[0:ran], color = "green")

# fig = plt.figure(figsize = (6, 6)) 
# plt.plot(zs[0:100],ys[0:100],marker='s',color='b')
# plt.plot(zs[101:200],ys[101:200],marker='o',color='r')
