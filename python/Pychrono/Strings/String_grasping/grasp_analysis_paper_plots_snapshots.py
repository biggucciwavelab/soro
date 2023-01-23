# -*- coding: utf-8 -*-
"""
Created on Tue Jan  3 10:30:46 2023

@author: Big Gucci
"""

import warnings
warnings.filterwarnings("ignore")
import pychrono.core as chrono
import timeit
import numpy as np
start=timeit.default_timer()
import objects4 as sim_obj
import random
import os
import csv
import glob
from IPython.display import HTML
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
import matplotlib.patches as patches
def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)



name1 = "12_01_2023_11_15_12"
d=2

snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name1)  
sim_data1=sim_obj.import_data(name1,path,dxmin,dxmax,dymin,dymax,Psi)



# In[extract data]
# %% In[extract data]
epsilon1=sim_data1.EPSILON_
time1=sim_data1.time
PX1=sim_data1.PX
FB1=sim_data1.FB
nn=10
entry1_=244
epsilon1_clean = moving_average(epsilon1, n=nn)
#epsilon1_clean = gaussian_filter1d(epsilon1, 10)



# %% In[create snap shots]
membrane=True
directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
time_=1
entry1=0
wxmin=-2
wxmax=4
wymin=-2
wymax=2
fxs=1.5
fys=1
name="circle"
sim_data1.create_frames_snapshot(name,True,membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)






