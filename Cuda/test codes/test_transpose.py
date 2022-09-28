# -*- coding: utf-8 -*-
"""
Created on Thu Aug 15 03:56:45 2019

@author: qiyua
"""
import numpy as np
import cupy as cp

q=np.load('B1.npy')

q1=np.asmatrix(q).T


q2=q.reshape((q.size,1))

b=cp.array(q)

b2=cp.reshape(b,(b.size,1))

print('q: %s' % (q))
print('q1: %s' % (q1))
print('q2: %s' % (q2))
print('b2: %s' % (b2))