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
path=path+"/Experiments/grasping_epsilon/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)


name = files[-1]

d=1.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

#wxmin=-2
#wxmax=3
#wymin=-2.5
#wymax=2.5

name = "08_05_2023_09_51_59"
name = "08_05_2023_10_29_58"
name = "08_05_2023_10_54_27"
name = "08_05_2023_11_17_05"
name = "08_05_2023_11_57_43"
name = "08_05_2023_14_11_19"
name = "08_05_2023_15_50_24"
name = "08_05_2023_16_18_55"
name = "08_05_2023_16_47_10"
name = "08_05_2023_17_20_50"
name = "12_05_2023_14_47_15"
name = "12_05_2023_15_22_51"
name = "15_05_2023_09_26_38"
name = "15_05_2023_10_01_12"
name = "15_05_2023_11_59_21"
name = "15_05_2023_14_19_33"
name = "15_05_2023_16_32_41"
name = "15_05_2023_16_33_10"
name = "16_05_2023_07_34_31"
name = "16_05_2023_07_34_18"

name = "16_05_2023_09_47_22"
name = "16_05_2023_09_47_43"

name = "16_05_2023_11_13_28"
name = "16_05_2023_11_13_42"

name = "17_05_2023_09_55_06"
#name = "17_05_2023_09_55_39"
name = "09_06_2023_12_46_48"
name = "09_06_2023_13_39_37"
name = "09_06_2023_14_08_25"
name = "09_06_2023_14_23_17"
name = "09_06_2023_14_27_00"
wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
#sim_data.save_grasp_parameters()

sim_data.create_frames_pull_epsilon2(membrane,d)

#sim_data.create_frames_pull_epsilon3(membrane,wxmin,wxmax,wymin,wymax)

#print('plot_epsilon4')
sim_data.plot_epsilon4()

print('plot_epsilon5')
sim_data.plot_epsilon5()




