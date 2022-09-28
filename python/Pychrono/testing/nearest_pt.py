# -*- coding: utf-8 -*-
"""
Created on Wed Jul  1 17:19:51 2020

@author: qiyua
"""

import numpy as np
from scipy import optimize


args = (2, 3)  # Current Position

def f(x, *args):
    u, v = x
    a, b, c, d, e, f = args
    return a*u**2 + b*u*v + c*v**2 + d*u + e*v + f

def gradf(x, *args):
    u, v = x
    a, b, c, d, e, f = args
    gu = 2*a*u + b*v + d     # u-component of the gradient
    gv = b*u + 2*c*v + e     # v-component of the gradient
    return np.asarray((gu, gv))
x0 = np.asarray((0, 0))  # Initial guess.

res1 = optimize.fmin_cg(f, x0, fprime=gradf, args=args)