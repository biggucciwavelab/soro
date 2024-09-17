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
name = "30_01_2023_14_31_49"
name = "31_01_2023_08_31_14"

name = "01_02_2023_12_41_14"

name = "01_02_2023_14_09_24"

name = "01_02_2023_16_31_25"

name = "01_02_2023_17_09_25"
name = "01_02_2023_19_41_50"

name = "02_02_2023_07_51_09"


name = "04_02_2023_14_17_44"
name = "05_02_2023_10_45_19"
name = "05_02_2023_13_58_28"
name = "05_02_2023_14_12_10"
name = "05_02_2023_14_38_38"
name = "05_02_2023_14_48_48"
#name = "05_02_2023_14_49_27"
name = "05_02_2023_15_07_28"
name = "05_02_2023_15_09_48"
name = "05_02_2023_15_16_44"
name = "05_02_2023_15_18_48"
name = "05_02_2023_15_29_53"
name = "05_02_2023_15_29_53"
name = "05_02_2023_17_34_30"
name = "05_02_2023_17_45_17"
name = "05_02_2023_17_50_23"
name = "06_02_2023_08_46_25"
name = "06_02_2023_15_59_12"
name = "07_02_2023_08_51_59"
name = "07_02_2023_14_03_02"
name = "07_02_2023_13_49_19"
name = "07_02_2023_16_26_01"
name = "07_02_2023_16_26_17"
name = "07_02_2023_20_45_00"
name = "07_02_2023_20_45_40"
name = "08_02_2023_11_22_07"
name = "08_02_2023_11_33_51"
name = "08_02_2023_11_43_57"
name = "08_02_2023_11_57_45"
name = "08_02_2023_12_09_45"

# circle
#name = 08_02_2023_08_15_08
#name = 08_02_2023_08_15_21
#name = 08_02_2023_08_20_22


name = "08_02_2023_21_43_51"
name = "08_02_2023_21_44_54"
name = "09_02_2023_08_12_41"


# strange object
name = "09_02_2023_11_30_34"
name = "09_02_2023_21_34_40"
name = "10_02_2023_18_53_40"
name = "10_02_2023_19_26_36"
name = "11_02_2023_13_15_36"#
name = "11_02_2023_13_15_43"#
name = "12_02_2023_11_25_50"#
name = "12_02_2023_12_45_48"#
name = "13_02_2023_10_10_12"#
name = "13_02_2023_10_10_21"#
name = "13_02_2023_10_10_04"#
name = "14_02_2023_18_32_26"
name = "14_02_2023_18_32_33"
name = "14_02_2023_18_32_41"




name = "14_02_2023_18_32_26"
name = "14_02_2023_18_32_33"
name = "14_02_2023_18_32_41"
name = "15_02_2023_09_31_03"
name = "15_02_2023_09_31_09"
name = "15_02_2023_09_31_14"
name = "15_02_2023_17_20_28"
name = "15_02_2023_17_20_35"
name = "15_02_2023_17_20_42"
name = "15_02_2023_17_20_50"


name = "27_02_2023_15_05_17"
name = "27_02_2023_15_52_43"
name = "27_02_2023_16_15_53"
name = "27_02_2023_16_30_26"
name = "27_02_2023_17_02_51"
name = "27_02_2023_17_28_15"

name = "29_01_2023_10_54_52"
name = "14_01_2023_15_35_36"
name = "09_02_2023_11_30_34"
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

print('plot_epsilon4')
sim_data.plot_epsilon4()

#print('plot_Qcm')
#sim_data.plot_Qcm()



#print('create_frames')
#sim_data.create_frames(membrane)

print('create_frames_pull_epsilon3')
# wxmin=-2.5
# wxmax=2
# wymin=-2.25
# wymax=2.25

wxmin=-d
wxmax=d
wymin=-d
wymax=d

#sim_data.create_frames_pull_epsilon3(membrane,wxmin,wxmax,wymin,wymax)

sim_data.create_frames_pull_epsilon6(membrane,wxmin,wxmax,wymin,wymax)