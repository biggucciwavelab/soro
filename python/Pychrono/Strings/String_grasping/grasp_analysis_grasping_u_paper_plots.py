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
#path=path+"/Experiments/grasping_u/good_ones/"
path=path+"/Experiments/grasping_u/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

name = "16_05_2023_20_37_20"
name = "17_05_2023_13_12_56"####
name = "17_05_2023_16_45_02" ## circle##

### widh=0.9
name = "17_05_2023_19_05_50" ## circle
name = "17_05_2023_19_05_07" ## square


name = "08_07_2023_14_24_56"
#name = "08_07_2023_14_55_06"

# name = "09_07_2023_13_54_20"
# name = "09_07_2023_14_32_07"
# name = "09_07_2023_14_32_07"


name = "09_07_2023_16_20_57"

name = "09_07_2023_17_16_41"
name = "09_07_2023_17_17_11"


name = "09_07_2023_17_23_44"
name = "09_07_2023_17_23_54"

name = "09_07_2023_19_11_21"
#name = "09_07_2023_19_20_42"

name = "09_07_2023_19_31_50"
name = "09_07_2023_19_32_26"

name = "09_07_2023_19_57_28"
name = "09_07_2023_19_57_51"

name = "09_07_2023_20_32_09"
#name = "09_07_2023_20_31_57"

name = "09_07_2023_20_31_57"
#entry=[0,100,109,180]

name = "30_12_2023_13_22_06" # circle

name = "30_12_2023_13_21_58" # square


d=4
snap_shot=False
membrane=False
dxmin=-4
dxmax=3
dymin=-4
dymax=4

wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
time=sim_data.time
TRIG1_=sim_data.TRIG1_
fsx=1.
fsy=1.
#entry=[0,5,10,15,20,25,30,35,40]
#entry=[0,125,150,200]
# entry=[0,165,166,180,250] # circle
sim_data.create_frames_u3_only_robot(membrane,wxmin,wxmax,wymin,wymax)

# xticks=[-4,-2,0,4]
# yticks=[-3,0,3]

# entry=[0,25,54,80,120]
# sim_data.create_frames_u3_snapshots(membrane,wxmin,wxmax,wymin,wymax,fsx,fsy,xticks,yticks,entry)



