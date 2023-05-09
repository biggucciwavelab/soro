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
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)


name = files[-1]



name = "20_04_2023_08_15_43"
name = "20_04_2023_08_30_46"
name = "20_04_2023_08_30_46"
name = "20_04_2023_08_52_16"

name = "20_04_2023_09_23_19"
name = "20_04_2023_09_27_02"
name = "20_04_2023_09_34_27"
name = "20_04_2023_09_45_50"
name = "20_04_2023_09_52_47"
name = "20_04_2023_10_23_55"
name = "20_04_2023_10_58_53"
name = "20_04_2023_11_23_52"
name = "20_04_2023_11_42_00"
name = "20_04_2023_11_58_41"

name = "20_04_2023_14_47_37"
name = "20_04_2023_15_14_03"
name = "20_04_2023_15_42_54"

name = "20_04_2023_16_06_30"
name = "20_04_2023_16_10_02"
name = "20_04_2023_16_19_22"

name = "20_04_2023_16_50_36"
name = "20_04_2023_16_50_45"
name = "20_04_2023_16_47_19"
name = "20_04_2023_17_58_01"
name = "20_04_2023_18_13_06"

name = "20_04_2023_18_48_03"
name = "21_04_2023_08_22_43"
name = "21_04_2023_08_25_23"

name = "21_04_2023_12_35_59"
name = "21_04_2023_13_14_24"

name = "21_04_2023_15_31_25"
name = "21_04_2023_16_43_07"
name = "21_04_2023_17_35_56"
name = "22_04_2023_10_32_31"
name = "22_04_2023_14_53_13"
name = "22_04_2023_15_50_41"
name = "23_04_2023_17_19_08"
name = "24_04_2023_08_06_31"
name = "24_04_2023_10_01_31"
name = "24_04_2023_12_34_38"
name = "24_04_2023_15_12_46"
name = "24_04_2023_17_23_29"
name = "24_04_2023_17_23_29"
name = "25_04_2023_11_48_20"
name = "25_04_2023_14_11_01"
name = "25_04_2023_15_23_17"







name = "26_04_2023_10_08_56"


name = "26_04_2023_11_11_25"


name = "26_04_2023_12_57_30"

name = "27_04_2023_18_59_23"

name = "28_04_2023_12_10_45"

name = "28_04_2023_14_13_15"


name = "30_04_2023_15_06_54"

name = "02_05_2023_20_56_30"
name = "02_05_2023_20_55_31"
name = "02_05_2023_20_56_30"
name = "03_05_2023_08_44_09"
name = "03_05_2023_09_22_50"
name = "03_05_2023_10_36_29"
name = "03_05_2023_11_31_50"
d=6
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


wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
#Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.save_grasp_parameters()

print('create_frames_pull_epsilon3')
sim_data.create_frames_pull_epsilon3(True,dxmin,dxmax,dymin,dymax)


print('plot_epsilon4')
sim_data.plot_epsilon4()





