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


d=3
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
name = "09_05_2023_16_16_15"
name = "09_05_2023_16_43_53"
name = "09_05_2023_17_06_18"
name = "09_05_2023_18_56_27"
name = "09_05_2023_19_29_38"
name = "09_05_2023_19_51_14"
name = "09_05_2023_20_13_29"
name = "09_05_2023_20_42_42"
name = "10_05_2023_12_04_20"
name = "10_05_2023_12_42_45"
name = "10_05_2023_12_57_47"
name = "10_05_2023_13_05_04"

name = "10_05_2023_13_09_31"
name = "10_05_2023_13_57_12"
name = "10_05_2023_13_52_17"
name = "10_05_2023_15_09_44"
name = "10_05_2023_15_11_13"

name = "10_05_2023_15_07_50"
name = "10_05_2023_16_05_41"
name = "10_05_2023_16_29_42"

name = "10_05_2023_16_57_53"
name = "10_05_2023_17_33_43"
name = "10_05_2023_19_24_19"
name = "10_05_2023_20_01_56"

name = "11_05_2023_09_11_41"
name = "11_05_2023_09_17_51"
name = "11_05_2023_09_22_30"
name = "11_05_2023_09_39_38"
name = "11_05_2023_09_40_22"
name = "11_05_2023_10_18_50"
name = "11_05_2023_10_26_26"
name = "11_05_2023_11_05_21"

name = "11_05_2023_11_50_53"
name = "11_05_2023_13_50_27"
name = "11_05_2023_14_06_58"
name = "11_05_2023_14_08_24"
name = "11_05_2023_14_27_03"
name = "11_05_2023_14_35_42"
name = "11_05_2023_14_54_23"
name = "11_05_2023_15_20_11"
name = "11_05_2023_16_07_56"
name = "11_05_2023_16_14_13"
name = "11_05_2023_16_53_16"
name = "11_05_2023_17_00_46"
name = "11_05_2023_17_11_32"

name = "11_05_2023_16_46_04"

name = "11_05_2023_16_46_04"
name = "11_05_2023_17_55_35"
name = "11_05_2023_18_01_39"
name = "11_05_2023_18_31_06"
name = "11_05_2023_18_47_01"
name = "11_05_2023_23_22_30"
name = "11_05_2023_23_29_42"
name = "11_05_2023_23_35_52"
name = "12_05_2023_09_39_14"
name = "12_05_2023_09_47_35"
name = "12_05_2023_09_47_52"
name = "12_05_2023_10_10_47"

name = "12_05_2023_10_57_03"
name = "12_05_2023_11_16_37"
name = "12_05_2023_11_30_54"
name = "12_05_2023_12_07_11"
name = "12_05_2023_12_44_49"
name = "12_05_2023_13_17_58"
name = "12_05_2023_13_54_33"
name = "12_05_2023_14_35_11"
name = "12_05_2023_15_13_47"
name = "12_05_2023_16_12_06"
name = "15_05_2023_09_22_37"
name = "15_05_2023_09_41_50"
name = "15_05_2023_10_00_27"
name = "15_05_2023_10_19_48"
name = "15_05_2023_10_34_45"
name = "15_05_2023_10_42_04"
name = "15_05_2023_10_52_11"
name = "15_05_2023_11_21_44"
name = "15_05_2023_12_01_20"
name = "15_05_2023_12_41_38"
name = "15_05_2023_13_31_48"
name = "15_05_2023_14_24_50"
name = "15_05_2023_14_55_50"
name = "15_05_2023_15_26_34"
name = "15_05_2023_15_26_11"
name = "15_05_2023_15_56_17"
name = "15_05_2023_16_11_13"
name = "16_05_2023_07_18_33"
name = "16_05_2023_07_49_09"
name = "16_05_2023_08_29_37"
name = "16_05_2023_08_29_08"
name = "16_05_2023_09_26_27"
name = "16_05_2023_09_29_37"
name = "16_05_2023_09_35_03"
name = "16_05_2023_10_29_18"
name = "16_05_2023_10_29_34"
name = "16_05_2023_11_06_13"
name = "16_05_2023_11_57_13"
name = "16_05_2023_12_50_06"
name = "16_05_2023_12_51_27"
name = "16_05_2023_13_07_25"
name = "16_05_2023_13_30_13"
name = "16_05_2023_13_33_43"

name = "16_05_2023_13_46_26"
name = "16_05_2023_13_47_14"
name = "16_05_2023_13_46_26"
name = "16_05_2023_13_51_48"
name = "16_05_2023_13_57_22"
name = "16_05_2023_14_03_16"
name = "16_05_2023_14_09_21"
name = "16_05_2023_14_16_10"
name = "16_05_2023_14_20_21"

name = "16_05_2023_14_28_23"
name = "16_05_2023_14_28_42"

name = "16_05_2023_14_44_13"
name = "16_05_2023_14_39_21"

name = "16_05_2023_14_33_31"
name = "16_05_2023_14_59_28"
name = "16_05_2023_15_00_43"
name = "16_05_2023_15_00_24"
name = "16_05_2023_15_31_28"
name = "16_05_2023_16_08_45"
name = "16_05_2023_16_52_24"
name = "16_05_2023_17_47_43"
name = "16_05_2023_17_50_25"

name = "16_05_2023_19_11_40"
name = "16_05_2023_19_21_26"
name = "16_05_2023_20_37_20"
# name = "16_05_2023_20_37_02"

# name = "16_05_2023_21_50_18"
# name = "16_05_2023_21_49_51"
# name = "16_05_2023_21_49_35"
#name = "17_05_2023_07_49_17"


name = "17_05_2023_09_45_33"
name = "17_05_2023_09_42_33"
name = "17_05_2023_09_39_16"


name = "17_05_2023_10_00_38"
name = "17_05_2023_09_56_59"
name = "17_05_2023_10_00_38"

name = "17_05_2023_11_12_11"
name = "17_05_2023_11_17_14"
name = "17_05_2023_11_19_47"

name = "17_05_2023_11_58_58"

name = "17_05_2023_12_00_23"

name = "17_05_2023_13_12_56" #### good one
name = "17_05_2023_12_52_18"
name = "17_05_2023_13_59_39"


name = "17_05_2023_14_44_13"

name = "17_05_2023_14_45_12"
name = "17_05_2023_15_12_26"

name = "17_05_2023_16_08_06"
name = "17_05_2023_16_45_02"
name = "17_05_2023_17_45_26"

name = "17_05_2023_18_58_03"
name = "17_05_2023_19_05_07"
name = "17_05_2023_19_05_50"

## high friction
name = "17_05_2023_19_22_56"
name = "17_05_2023_20_27_04"

# width=0.85
name = "18_05_2023_08_42_31"
name = "18_05_2023_08_42_44"

name = "18_05_2023_09_45_27"
name = "18_05_2023_09_45_50"

name = "18_05_2023_09_53_40"

name = "18_05_2023_15_44_05"
name = "18_05_2023_15_44_21"

name = "18_05_2023_15_05_06"

name = "18_05_2023_16_38_31"
name = "18_05_2023_16_38_47"

name = "18_05_2023_17_32_58"
name = "18_05_2023_17_33_30"

name = "18_05_2023_16_48_55"

name = "18_05_2023_17_59_47"
name = "18_05_2023_18_04_31"

name = "18_05_2023_18_30_51"
name = "18_05_2023_18_31_43"

name = "18_05_2023_18_16_30"

name = "18_05_2023_19_22_33"
name = "18_05_2023_19_55_51"

name = "18_05_2023_19_49_25"

name = "22_05_2023_16_22_35"
name = "22_05_2023_16_32_01"
name = "22_05_2023_17_42_11"
name = "22_05_2023_18_03_07"
name = "22_05_2023_18_49_45"
name = "22_05_2023_20_13_50"
name = "22_05_2023_21_20_41"
name = "23_05_2023_15_47_19"
name = "23_05_2023_17_06_47"
name = "24_05_2023_20_14_01"

name = "25_05_2023_13_56_06"
name = "26_05_2023_09_48_15"
name = "26_05_2023_10_29_07"


###c shape
name = "06_06_2023_10_14_38"
name = "06_06_2023_10_54_55"
name = "06_06_2023_11_55_01"
name = "06_06_2023_13_26_57"
name = "06_06_2023_13_58_30"
name = "06_06_2023_14_27_52"
name = "06_06_2023_14_55_05"
name = "06_06_2023_16_20_30"
name = "06_06_2023_17_05_51"
name = "06_06_2023_17_54_40"
name = "06_06_2023_18_21_26"
name = "06_06_2023_19_25_24"
name = "06_06_2023_19_53_20"
name = "07_06_2023_10_14_20"
name = "07_06_2023_10_42_36"
name = "07_06_2023_11_00_14"
name = "07_06_2023_11_18_45"
name = "07_06_2023_11_42_39"
name = "07_06_2023_12_04_09"
name = "07_06_2023_12_22_45"
name = "07_06_2023_12_42_01"
name = "07_06_2023_13_29_02"
name = "07_06_2023_13_50_57"
name = "07_06_2023_14_19_27"
name = "07_06_2023_14_54_25"
name = "07_06_2023_15_25_20"
name = "07_06_2023_16_33_26"
name = "07_06_2023_16_22_41"
name = "08_06_2023_14_44_13"
name = "08_06_2023_17_29_56"
name = "08_06_2023_18_24_55"
name = "08_06_2023_18_50_21"
name = "09_06_2023_11_31_33"
name = "09_06_2023_12_01_14"
name = "09_06_2023_12_01_23"
name = "09_06_2023_12_01_32"
d=3
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d





wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)


print('plot_epsilon4')
sim_data.plot_epsilon4()

print('plot_epsilon5dx')
sim_data.plot_epsilon5dx()

print('plot_epsilon5')
sim_data.plot_epsilon5()

print('plot_potential_Field_value_sum')
sim_data.plot_potential_Field_value()

print('plot_potential_Field_value')
sim_data.plot_potential_Field_value_individual()

print('create_frames_u3')
sim_data.create_frames_u3(membrane,wxmin,wxmax,wymin,wymax)


