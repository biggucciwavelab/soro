# -*- coding: utf-8 -*-
"""
Created on Tue Jul  6 13:06:30 2021

@author: dmulr
"""
import numpy as np
import matplotlib.pyplot as plt

const=2.66
x = np.array([2.212,
     2.951,
     4.270,
     4.737,
     6.678,
     7.455,
     6.716,
     9.357,
     6.172,
     6.871,
     7.493,
     7.493,
     9.279,
     10.094,
     10.483,
     11.377])

x=np.dot(const,x)
xc=np.sum(x)/len(x)
x=x-xc


y = np.array([18.946,
     16.5,
     17.937,
     16.112,
     17.432,
     18.558,
     20.616,
     21.353,
     13.825,
     13.084,
     14.015,
     11.492,
     12.346,
     14.210,
     12.967,
     14.753])


y=np.dot(const,y)
yc=np.sum(y)/len(y)

y=y-yc



R = [1.5,
     2,
     2,
     2,
     2,
     2,
     2,
     1.5,
     1.5,
     1.5,
     1.5,
     1.5,
     2,
     2,
     1.5,
     1.5]

#R = np.dot(R)

fig = plt.figure(dpi=300)
fig.set_size_inches(5, 5)
con=30
a = 0-con
b = 0+con
c = 0-con
d = 0+con
np.savez('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/points1.npz',x=x,y=y,R=R)

ax = plt.axes(xlim=(a,b), ylim=(c, d))          

for i in range(len(x)):
    patch = plt.Circle((y[i], x[i]),R[i], fc='black')
    ax.add_patch(patch)
