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
path=path+"/Experiments/grasping_u/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)


name = files[-1]


d=4
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
name = "08_05_2023_11_33_16"
name = "08_05_2023_12_01_13"
name = "08_05_2023_14_10_42"
name = "08_05_2023_15_27_17"
name = "08_05_2023_15_51_11"
name = "08_05_2023_16_13_33"
name = "08_05_2023_16_37_31"
name = "08_05_2023_17_12_24"
name = "08_05_2023_21_06_16"
name = "08_05_2023_21_06_34"
wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)


print('create_frames_u3')
sim_data.create_frames_u3(membrane,wxmin,wxmax,wymin,wymax)

print('plot_epsilon4')
sim_data.plot_epsilon4()

sim_data.plot_epsilon5dx()

print('plot_epsilon5')
sim_data.plot_epsilon5()

print('plot_potential_Field_value_sum')
sim_data.plot_potential_Field_value()

print('plot_potential_Field_value')
sim_data.plot_potential_Field_value_individual()

