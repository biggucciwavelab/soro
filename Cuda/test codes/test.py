# -*- coding: utf-8 -*-
"""
Created on Tue Aug 13 15:35:41 2019

@author: qiyua
"""
import numpy as np

M = np.array([[0.42956806, -0.40076658, -0.02880148, -0.42956806, 0.40076658, 0.02880148],
	       [-0.40076658, 0.47288367, -0.07211709, 0.40076658, -0.47288367, 0.07211709],
	       [-0.02880148, -0.07211709, 0.10091857, 0.02880148, 0.07211709, -0.10091857],
	       [-0.42956806, 0.40076658, 0.02880148, 0.42956806, -0.40076658, -0.02880148],
	       [ 0.40076658, -0.47288367, 0.07211709, -0.40076658, 0.47288367, -0.07211709],
	       [ 0.02880148, 0.07211709, -0.10091857, -0.02880148, -0.07211709, 0.10091857]])

q = np.array([1.09389333, -0.53851907, -0.05537426, -0.79389333, 0.83851907, 0.35537426])

print('M: %s   q: %s' % (M, q))
