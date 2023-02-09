# -*- coding: utf-8 -*-
"""
Created on Mon Jan  9 09:23:25 2023

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

#### good ones
name3 = "05_01_2023_17_14_08"
name2 = "05_01_2023_17_14_01"
name1 = "05_01_2023_17_13_49"
#### good ones
name2 = "11_01_2023_13_55_11" # square
name3 = "11_01_2023_13_55_19" # triangle
name1 = "11_01_2023_13_54_57" # circle


name = "08_01_2023_12_23_41"
#name = "08_01_2023_12_21_02"
name = "10_01_2023_12_55_42"

d=2
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name1)  
sim_data1=sim_obj.import_data(name1,path,dxmin,dxmax,dymin,dymax,Psi)


membrane=True
directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
entry1=30 
wxmin=0
wxmax=4
wymin=-2
wymax=2
fxs=2
fys=2
sim_data1.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys,"grasp_explain")

wxmin=1.5
wxmax=2.5
wymin=-0.5
wymax=0.5
sim_data1.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys,"grasp_explain_zoom")


sim_data1.create_frames_contact_forces_(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys,"grasp_explain_cone")