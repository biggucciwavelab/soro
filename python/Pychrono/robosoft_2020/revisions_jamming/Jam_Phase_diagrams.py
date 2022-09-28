# -*- coding: utf-8 -*-
"""
Created on Mon Feb 10 13:43:26 2020

@author: dmulr
"""

import matplotlib.pyplot as plt
import json
#import large_scale
import timeit
import numpy as np



#simtimes=np.load('simtimes.npy')
nb=np.load('nb.npy')
k=np.load('k.npy')

PHI=np.load('PHI.npy')
PHI=np.asarray(PHI)

nb=np.asarray(nb)
k=-np.asarray(k)

image,ax=plt.subplots(figsize=(9,9))
PHI_reshape=PHI.reshape((len(nb),len(k)))
plt.imshow(PHI_reshape,interpolation='bilinear')
plt.title('Packing Fraction')
plt.xlabel('String Tension [N]')
plt.ylabel('Number of boundary robots')
ax.set_xticks(np.arange(np.shape(PHI_reshape)[0]))
ax.set_yticks(np.arange(np.shape(PHI_reshape)[1]))
ax.set_yticklabels(nb)
ax.set_xticklabels(k)
cbar=plt.colorbar()
cbar.set_label('Packing Fraction')
plt.show()
plt.savefig('Packing_Fraction.png')


#np.save('k2',k)
#np.save('nb2',nb)

plt.figure(num=2,figsize=[9,9])

for i in range(len(nb)):
    
    plt.scatter(k,PHI_reshape[i,:],label='nb='+str(nb[i]))

plt.xlabel('String Tension [N]')
plt.ylabel('Packing Fraction')
plt.title('Packing Fraction vs String Tension')
plt.legend()
plt.grid(True)
plt.savefig('Packing_Fraction_2.png')
