# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 13:39:45 2023

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
path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

d=2
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d
name1 = "09_02_2023_11_30_34"
sim_data1=sim_obj.import_data(name1,path,dxmin,dxmax,dymin,dymax,None)

# %%
directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"

entry=25
d=5.5
fxs=1.25
fys=1.25
wxmin=-4
wxmax=1.5
wymin=-2.75
wymax=2.75
name="import"
sim_data1.create_frames_snapshot(name,True,membrane,directory,entry,wxmin,wxmax,wymin,wymax,fxs,fys,False)


entry=115
d=5.5
fxs=1.25
fys=1.25
wxmin=-4
wxmax=1.5
wymin=-2.75
wymax=2.75
name="import"
sim_data1.create_frames_snapshot(name,True,membrane,directory,entry,wxmin,wxmax,wymin,wymax,fxs,fys,False)



entry=340
d=5.5
fxs=1.25
fys=1.25
wxmin=-3
wxmax=2.5
wymin=-3
wymax=2.5
name="import"
sim_data1.create_frames_snapshot(name,True,membrane,directory,entry,wxmin,wxmax,wymin,wymax,fxs,fys,False)



entry=855
d=5.5
fxs=1.25
fys=1.25
wxmin=-3
wxmax=2.5
wymin=-2
wymax=3.5
name="import"
sim_data1.create_frames_snapshot(name,True,membrane,directory,entry,wxmin,wxmax,wymin,wymax,fxs,fys,False)