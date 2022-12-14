from numpy import *
from openopt import LCP
import numpy as np

#particles=int(2)
#size=int(particles*particles-np.ceil(particles/2))
#M=np.random.rand(size,size)
#q=np.random.rand(size)

M = array([[0.42956806, -0.40076658, -0.02880148, -0.42956806, 0.40076658, 0.02880148],
	       [-0.40076658, 0.47288367, -0.07211709, 0.40076658, -0.47288367, 0.07211709],
	       [-0.02880148, -0.07211709, 0.10091857, 0.02880148, 0.07211709, -0.10091857],
	       [-0.42956806, 0.40076658, 0.02880148, 0.42956806, -0.40076658, -0.02880148],
	       [ 0.40076658, -0.47288367, 0.07211709, -0.40076658, 0.47288367, -0.07211709],
	       [ 0.02880148, 0.07211709, -0.10091857, -0.02880148, -0.07211709, 0.10091857]])

q = array([1.09389333, -0.53851907, -0.05537426, -0.79389333, 0.83851907, 0.35537426])

p = LCP(M, q)
r = p.solve('lcpsolve')
f_opt, x_opt = r.ff, r.xf
w, z = x_opt[x_opt.size/2:], x_opt[:x_opt.size/2]
print('w: %s' % (w))
print('z: %s' % (z))