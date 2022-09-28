# -*- coding: utf-8 -*-
"""
Created on Thu Dec 17 12:17:50 2020

@author: elopez8

Quaterinon rotation as from:
    https://stackoverflow.com/questions/4436764/rotating-a-quaternion-on-1-axis
"""

import numpy as np
from numpy.linalg import norm

def create_from_axis_angle(xx,yy,zz,angle):
    ## Here we calculate the sin(theta/2)
    factor=np.sin(angle/2)
    x = xx*factor
    y = yy*factor
    z = zz*factor
    w = np.cos(angle/2)
    quaternion = np.array([x,y,z,w])
    return quaternion/norm(quaternion)

def multQ(Q1,Q2):
    x0,y0,z0,w0 = Q1   # unpack
    x1,y1,z1,w1 = Q2   # unpack
    return([x1*w0 + y1*z0 - z1*y0 + w1*x0, 
            -x1*z0 + y1*w0 + z1*x0 + w1*y0, 
            x1*y0 - y1*x0 + z1*w0 +  w1*z0,
            -x1*x0 - y1*y0 - z1*z0 + w1*w0])