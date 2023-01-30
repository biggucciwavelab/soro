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

# name = "07_01_2023_13_35_04"
# name = "07_01_2023_18_26_29"
# name = "07_01_2023_18_37_30"

name = "08_01_2023_12_23_41"
#name = "08_01_2023_12_21_02"
name = "10_01_2023_12_55_42"


name = "12_01_2023_11_18_12"

# non noise ones
name = "13_01_2023_19_43_36"
name = "13_01_2023_19_43_52"


# noisy ones
name = "14_01_2023_15_35_36"
name = "14_01_2023_15_35_51"

name = "13_01_2023_19_43_52"
name = "13_01_2023_19_43_36"


name = "16_01_2023_15_33_36"
name = "16_01_2023_15_34_21"
name= "16_01_2023_19_35_29"



name= "17_01_2023_14_01_50"
name= "17_01_2023_14_13_11"


name= "18_01_2023_10_02_41"
name= "18_01_2023_10_03_51" # reverse
name= "18_01_2023_14_57_33"
name= "18_01_2023_14_17_01"

name= "18_01_2023_20_10_57"
name= "18_01_2023_20_26_57"
name= "18_01_2023_20_27_28"


name= "18_01_2023_20_11_27"





# square

name = "21_01_2023_12_06_44"
name= "21_01_2023_14_16_13"
name= "21_01_2023_12_32_51"
name= "21_01_2023_16_09_38"
name = "22_01_2023_16_01_26"
name = "22_01_2023_16_01_45"
name = "22_01_2023_16_01_57"

# slice pi/4
# name= "21_01_2023_16_45_00"
# name = "22_01_2023_13_50_03"
# name = "22_01_2023_13_50_10"
# name = "22_01_2023_13_50_18"
name = "23_01_2023_08_43_42"
name = "23_01_2023_08_46_57"
name = "23_01_2023_08_47_04"
name = "23_01_2023_08_47_25"
name = "23_01_2023_08_49_03"
name = "23_01_2023_08_40_11"
name = "23_01_2023_13_40_41"#
name = "23_01_2023_13_40_48"#
name = "23_01_2023_13_40_55"#
name = "23_01_2023_13_41_03" # needs running#



# circle
name = "23_01_2023_14_46_09"
name = "23_01_2023_14_46_17" # needs running
name ="23_01_2023_17_13_15"
name ="23_01_2023_17_14_05"
name = "24_01_2023_13_12_44"
name = "24_01_2023_13_12_49"
name = "24_01_2023_13_12_57"
name = "24_01_2023_13_14_03"



# Triagnle 
#name = "25_01_2023_09_11_23"
#name = "25_01_2023_09_11_30"
name = "25_01_2023_09_10_55"
name = "25_01_2023_09_11_04"
name = "25_01_2023_09_11_11"

name = "25_01_2023_15_14_15"
name = "25_01_2023_15_14_29"
name = "25_01_2023_15_14_41"
name = "25_01_2023_15_30_53"
name = "25_01_2023_15_44_39"

# square
name = "26_01_2023_11_17_07"
name = "26_01_2023_11_17_19"
name = "26_01_2023_11_17_25"
name = "26_01_2023_11_32_41"

name = "28_01_2023_12_44_16"
name = "28_01_2023_16_02_44"
name = "28_01_2023_16_03_00"
name = "28_01_2023_17_55_50"
name = "28_01_2023_18_03_09"


name = "29_01_2023_10_55_00"
#name = "29_01_2023_10_54_47"
#name = "29_01_2023_10_54_52"

# name = "29_01_2023_22_08_13"
# name = "29_01_2023_22_08_27"
# name = "29_01_2023_22_09_21"

#name = "29_01_2023_22_00_41"
d=4.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)


print('sort_epsilon_and_theta')
sim_data.sort_epsilon_and_theta()

print('plot_epsilon_vs_theta_section')
sim_data.plot_epsilon_vs_theta_section()

#print('plot_epsilon4')
#sim_data.plot_epsilon4()

#print('plot_Qcm')
#sim_data.plot_Qcm()



# print('create_frames')
# sim_data.create_frames(membrane)

print('create_frames_pull_epsilon3')
sim_data.create_frames_pull_epsilon3(membrane,4.5)


