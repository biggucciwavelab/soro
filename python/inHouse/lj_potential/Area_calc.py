# -*- coding: utf-8 -*-
"""
Created on Sat Feb  2 13:04:44 2019

@author: dmulr
"""

import numpy as np
import matplotlib.pyplot as plt
#import data 
data = np.load('temp9500_0.npz')

#points imported
x=data['x']
y=data['y']
n=data['n']
nb=data['nb']
ni=data['ni']


#create empty matrix size of boundary particles
A1=np.zeros(nb)

#deterime size
s1=np.size(A1)

s2=np.shape(x)

s2=s2[0]

A=np.zeros(s2-1)

#find the area 
for j in range(s2-1):
    for i in range(s1-1):
        A1[i]=.5*(x[j,i+1]+x[j,i])*(y[j,i+1]-y[j,i])
    i=s1-1
    A1[i]=.5*(x[j,0]+x[j,i])*(y[j,0]-y[j,i])
    A[j]=sum(A1)
    
    

#graph it 
t=np.linspace(0,(10e-3)*s2,s2-1)
plt.plot(t,A,color="green")
plt.grid(True)
plt.title('Area vs time b=.4 ')
plt.xlabel('time')
plt.ylabel('Area')
plt.show()

np.savez('Area9500_0.npz',t=t,A=A)