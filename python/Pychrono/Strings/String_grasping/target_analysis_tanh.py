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
path=path+"/Experiments/target_chasing/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)


name = files[-1]


d=3
snap_shot=False
membrane=False
dxmin=-4
dxmax=14
dymin=-6
dymax=0

name = "22_05_2023_00_39_34"
name = "22_05_2023_08_13_36"
name = "22_05_2023_08_43_38"
name = "22_05_2023_08_47_06"
name = "22_05_2023_08_55_44"
name = "22_05_2023_09_17_09"
name = "22_05_2023_08_55_44"
name = "22_05_2023_08_55_44"
name = "22_05_2023_17_40_33"
name = "22_05_2023_19_01_36"
name = "22_05_2023_20_30_54"
name = "22_05_2023_22_00_47"
name = "23_05_2023_16_53_55"
name = "24_05_2023_18_54_08"
name = "24_05_2023_20_23_26"
name = "25_05_2023_14_01_10"
name = "26_05_2023_09_47_37"
name = "26_05_2023_13_41_46"
name = "26_05_2023_13_59_09"
name = "28_05_2023_17_05_45"
name = "28_05_2023_18_02_43"
name = "28_05_2023_18_38_33"
name = "28_05_2023_18_45_27"
name = "28_05_2023_19_44_45"
name = "28_05_2023_22_08_56"
name = "28_05_2023_23_38_50"
name = "29_05_2023_10_22_29"
name = "29_05_2023_14_06_00"
name = "29_05_2023_15_57_40"
name = "29_05_2023_18_37_24"
name = "29_05_2023_20_40_18"
name = "30_05_2023_11_43_32"
name = "30_05_2023_15_01_50"
name = "30_05_2023_15_47_04"
name = "31_05_2023_14_56_54"
name = "31_05_2023_15_10_47"
name = "31_05_2023_17_33_43"
name = "31_05_2023_19_23_31"
name = "05_06_2023_10_38_44"
name = "05_06_2023_12_07_30"
name = "05_06_2023_13_05_07"
name = "05_06_2023_13_02_22"
name = "05_06_2023_14_02_14"
name = "05_06_2023_15_35_03"
name = "05_06_2023_15_38_38"
name = "05_06_2023_17_20_11"
name = "05_06_2023_18_06_20"####
name = "05_06_2023_19_23_04"
name = "05_06_2023_20_19_21"
name = "05_06_2023_20_19_38"
name = "06_06_2023_09_38_01"
name = "06_06_2023_09_53_04"###

name = "06_06_2023_10_56_29"

name = "06_06_2023_12_11_08"
name = "06_06_2023_12_04_28"
name = "06_06_2023_13_53_48"
name = "06_06_2023_14_17_03"
name = "06_06_2023_15_11_53"
name = "06_06_2023_16_13_46"
name = "06_06_2023_16_59_43"
name = "06_06_2023_19_19_38"
name = "06_06_2023_19_41_19"
name = "06_06_2023_20_06_43"
name = "07_06_2023_08_44_12"
name = "07_06_2023_08_56_00"
name = "07_06_2023_09_17_03"
name = "07_06_2023_09_42_12"
name = "07_06_2023_09_39_17"
name = "07_06_2023_09_53_45"
name = "07_06_2023_10_03_10"
name = "07_06_2023_10_24_39"
name = "07_06_2023_10_46_01"
name = "07_06_2023_11_07_52"
name = "07_06_2023_11_50_04"
name = "07_06_2023_12_26_31"
name = "07_06_2023_12_33_22"
name = "07_06_2023_13_21_50"
name = "07_06_2023_14_09_58"
name = "07_06_2023_13_16_25"
name = "07_06_2023_14_36_06"
name = "07_06_2023_14_50_22"
name = "07_06_2023_15_14_38"
name = "07_06_2023_16_27_46"
name = "07_06_2023_16_22_41"
name = "07_06_2023_17_19_55"
name = "07_06_2023_17_38_04"#####
name = "08_06_2023_13_32_33"
wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)

sim_data.create_frames_target(membrane,wxmin,wxmax,wymin,wymax)



