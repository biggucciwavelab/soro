# -*- coding: utf-8 -*-
"""
Created on Tue Mar 17 10:31:21 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from objects import *
from myconfig_torus import *
from scipy.optimize import LinearConstraint, Bounds, minimize, linprog

start = timeit.default_timer()

# In[Set Path]
#chrono.SetChronoDataPath("C:/Users/Amin/Documents/chrono-data/")
chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")
# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
# my_system.SetTol(1e-6)
# my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
# my_system.SetMaxItersSolverSpeed(300)

material = Material(mu_f, mu_b, mu_r, mu_s, C_, Ct, Cr, Cs)
body_floor = Floor(material, length, tall)
my_system.Add(body_floor)