# -*- coding: utf-8 -*-
"""
Created on Tue Dec 20 09:51:25 2022

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

path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)


name = files[-1]
#name = "23_12_2022_15_25_27"
#name = "23_12_2022_18_35_55"
#name = "02_01_2023_18_54_57"
# name = "02_01_2023_18_55_08"


# name = "03_01_2023_10_46_20"
# name = "03_01_2023_10_46_12"
# name = "03_01_2023_10_45_55"

# name = "04_01_2023_11_15_09" # circle
# name = "04_01_2023_11_23_36" # square
# name = "04_01_2023_11_23_45" # triangle
# name = "04_01_2023_12_52_26"

name = "10_01_2023_14_54_46" # cylinder
name = "10_01_2023_14_54_56" # square
name = "10_01_2023_14_55_03" # triangle

name = "11_01_2023_13_55_11" # square
name = "11_01_2023_13_55_19" # triangle
name = "11_01_2023_13_54_57" # circle





name = "12_01_2023_09_55_24"
name = "12_01_2023_09_55_32"
name = "12_01_2023_09_55_12"

name = "12_01_2023_11_14_58"
name = "12_01_2023_11_15_12"

name = "13_01_2023_12_54_24"
name = "13_01_2023_12_51_42"
name = "13_01_2023_19_45_09"
name = "13_01_2023_20_01_54"

name = "13_01_2023_22_53_21"
name = "13_01_2023_22_53_41"
name = "13_01_2023_22_53_34"
name = "13_01_2023_22_54_03"
name = "13_01_2023_22_54_11"


# circle 
name = "14_01_2023_12_19_15"
name = "14_01_2023_12_19_21"
name = "14_01_2023_12_19_26"
name = "14_01_2023_12_19_32"
name = "14_01_2023_12_19_38"


# square
name = "14_01_2023_12_20_51"
name = "14_01_2023_12_20_55"
name = "14_01_2023_12_21_01"
name = "14_01_2023_12_21_06"
name = "14_01_2023_12_21_12"



name = "14_01_2023_13_28_53"
name = "14_01_2023_13_29_09"

# Good ones 
name = "14_01_2023_15_32_52"
name = "14_01_2023_15_33_04"

# circle noise ones 
name = "14_01_2023_15_26_35"
name = "14_01_2023_15_26_42"
name = "14_01_2023_15_26_49"
name = "14_01_2023_15_27_04"
name = "14_01_2023_15_27_11"

# square noisy ones 

name = "15_01_2023_14_10_19"
name = "15_01_2023_14_10_29"
name = "15_01_2023_14_10_37"
name = "15_01_2023_14_10_43"
name = "15_01_2023_14_10_51"

# circle noisy
name = "15_01_2023_14_18_01"
name = "15_01_2023_14_16_34"
name = "15_01_2023_14_20_22"
name = "15_01_2023_14_20_31"
name = "15_01_2023_14_20_39"



name= "16_01_2023_19_35_29"




name= "17_01_2023_13_53_15"
name= "17_01_2023_13_55_08"
name= "17_01_2023_13_55_15"
name= "17_01_2023_13_55_21"
name= "17_01_2023_13_55_27"

name= "17_01_2023_14_46_36"
name= "17_01_2023_14_46_45"
name= "17_01_2023_14_47_11"
name= "17_01_2023_14_47_19"
name= "17_01_2023_14_47_35"


name= "17_01_2023_15_42_45"
name= "17_01_2023_15_42_53"
name= "17_01_2023_15_42_58"
name= "17_01_2023_15_43_06"
name= "17_01_2023_15_43_11"

name= "17_01_2023_18_47_53"
name= "17_01_2023_18_48_00"
name= "17_01_2023_18_49_15"
name= "17_01_2023_18_48_11"
name= "17_01_2023_18_48_26"

name = "17_01_2023_19_29_37"
name = "17_01_2023_19_29_43"
name = "17_01_2023_19_29_51"
name = "17_01_2023_19_29_58"
name = "17_01_2023_19_30_08"


name= "18_01_2023_07_43_35"
name= "18_01_2023_07_43_42"
name= "18_01_2023_07_43_55"
name= "18_01_2023_07_44_05"
name= "18_01_2023_07_44_12"
name= "18_01_2023_07_44_22"

name= "18_01_2023_08_33_12"
name= "18_01_2023_08_33_17"
name= "18_01_2023_08_33_21"
name= "18_01_2023_08_33_32"
name= "18_01_2023_08_33_37"
name= "18_01_2023_08_33_43"


name= "18_01_2023_13_47_12"
name= "18_01_2023_13_47_19"
name= "18_01_2023_13_47_24"
name= "18_01_2023_13_47_30"
name= "18_01_2023_13_47_37"
name= "18_01_2023_13_47_48"
name= "18_01_2023_13_47_55"


name= "18_01_2023_15_38_43"
name= "18_01_2023_15_38_54"
name= "18_01_2023_15_39_00"
name= "18_01_2023_15_39_05"
name= "18_01_2023_15_39_11"

name= "18_01_2023_15_39_22"
name= "18_01_2023_15_39_36"
name= "18_01_2023_15_39_50"
name= "18_01_2023_15_40_17"

d=2

snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)

#print('create_frames_contact_forces')
#sim_data.create_frames_contact_forces(wxmax=d,wxmin=-d,wymin=-d,wymax=d)

# print('sort_epsilon_and_theta')
# sim_data.sort_epsilon_and_theta()

# print('plot_epsilon_vs_theta_section')
# sim_data.plot_epsilon_vs_theta_section()

print('plot_epsilon4')
sim_data.plot_epsilon4()

print('plot_Qcm')
sim_data.plot_Qcm()




# print('create_frames')
# sim_data.create_frames(membrane,dxmin,dxmax,dymin,dymax)

sim_data.create_frames_pull_epsilon5(membrane,2)

# print('create_frames_pull_epsilon4')
# sim_data.create_frames_pull_epsilon4(membrane,2)


# print('create_frames_pull_epsilon3')
# sim_data.create_frames_pull_epsilon3(membrane,4.5)

#print('create_frames_pull_epsilon2')
#sim_data.create_frames_pull_epsilon2(membrane,2)

