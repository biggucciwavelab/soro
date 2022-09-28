# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 13:11:42 2021

@author: qiyua
"""
import numpy as np
from mp_plot import elliptic_fourier

theta=np.linspace(0,2.*np.pi,100)
x=np.sin(theta)
y=np.cos(theta)

contour = np.vstack((x,y)).T

phi=elliptic_fourier(contour,5,0)
phi.gen_elliptic()

point = phi.interpolate((0,1.5))
print(point)