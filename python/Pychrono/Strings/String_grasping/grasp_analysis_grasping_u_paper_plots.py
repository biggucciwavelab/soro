# -*- coding: utf-8 -*-
"""
Created on Sat Aug 13 15:10:42 2022

@author: dmulr
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

path = os.path.dirname(__file__)
path=path+"/Experiments/grasping_u/good_ones/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

name = "16_05_2023_20_37_20"
name = "17_05_2023_13_12_56"####
name = "17_05_2023_16_45_02" ## circle##

### widh=0.9
name = "17_05_2023_19_05_50" ## circle
name = "17_05_2023_19_05_07" ## square
entry=[0,100,109,130,180]
d=4
snap_shot=False
membrane=False
dxmin=-4
dxmax=2
dymin=-3
dymax=3

wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
time=sim_data.time
TRIG1_=sim_data.TRIG1_
fsx=1.625
fsy=1.625
#entry=[0,5,10,15,20,25,30,35,40]
#entry=[0,125,150,200]
# entry=[0,165,166,180,250] # circle


xticks=[-4,-2,0,2]
yticks=[-3,0,3]
sim_data.create_frames_u3_snapshots(membrane,wxmin,wxmax,wymin,wymax,fsx,fsy,xticks,yticks,entry)



