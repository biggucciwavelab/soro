# -*- coding: utf-8 -*-
"""
Created on Wed Oct  9 15:18:49 2019

@author: dmulr
"""

import numpy as np
from bridson import poisson_disc_samples
import math as math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib import animation
import sys
import os




nb=8
diameter=.07
R1=(diameter*nb/(np.pi*2))+.05


x=[]
z=[]

patch=[]

p1x=[]
p1z=[]
p2x=[]
p2z=[]


p00=np.array([[R1],[1],[-diameter/3]])

p01=np.array([[R1],[1],[diameter/3]])
for i in range(nb):
    x=R1*np.cos(i*2*np.pi/nb)
    z=R1*np.sin(i*2*np.pi/nb)
    
    c=np.cos(i*2*np.pi/nb)
    s=np.sin(i*2*np.pi/nb)
    
    Q=np.array([[c, 0 ,s],[-s ,0 ,c],[0,1,0]])
    
    ptemp1=np.matmul(Q,p00)
    
    ptemp2=np.matmul(Q,p01)
    
    p1x.append(ptemp1[0])
    p1z.append(ptemp1[1])
    p2x.append(ptemp2[0])
    p2z.append(ptemp2[1])
    
    circle=plt.Circle((x, z),diameter/2, color='r')
    patch.append(circle)

patch=np.asarray(patch)

p1x=np.asarray(p1x)
p1z=np.asarray(p1z)
p2x=np.asarray(p2x)
p2z=np.asarray(p2z)


fig, ax = plt.subplots()

plt.xlim(-.2,.2)
plt.ylim(-.2,.2)

plt.grid(linestyle='--')

ax.set_aspect(1)

plt.scatter(p1x,p1z)
plt.scatter(p2x,p2z)
for i in range(nb):
    ax.add_artist(patch[i])

    
    
plt.title('How to plot a circle with matplotlib ?', fontsize=8)

plt.savefig("plot_circle_matplotlib_02.png", bbox_inches='tight')

plt.show()