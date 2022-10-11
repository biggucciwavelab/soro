# -*- coding: utf-8 -*-
"""
Created on Mon Oct 10 13:34:50 2022

@author: Big Gucci
"""

import numpy as np 
import matplotlib.pyplot as plt
x=np.linspace(0,10,100)
y=np.sin(x)

plt.figure()

for n in range(len(x)):
    plt.cla()
    plt.plot(x[:n],y[:n],color='k')
    plt.axis([0,10,0,1])
    plt.savefig('Frame%03d.png' %n)