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

#name = "14_11_2022_16_36_03"
#name = "14_11_2022_16_36_19"
#name = "14_11_2022_16_35_26"
#name = "11_10_2022_12_12_55" # TRIANGLE
#name = "11_10_2022_17_26_37" # SQUARE
#name = "11_10_2022_12_12_03" # circle
#name ="29_11_2022_18_36_17" # circle 
#name ="29_11_2022_18_36_48" # triangle
#name ="29_11_2022_18_36_33" # square

#name ="30_11_2022_12_51_48"
#name = "30_11_2022_17_39_10"
#name = "30_11_2022_17_58_52"
#name = "01_12_2022_09_43_46"

#name = "06_12_2022_10_40_38" # square
#name = "06_12_2022_10_35_23" #circle
#name = "06_12_2022_10_36_05" # triangle
#name = "08_12_2022_08_33_52"
#name = "08_12_2022_09_35_31"


name = "09_12_2022_12_10_49" # explore  random object
name = "12_12_2022_17_09_09"
#name = "12_12_2022_16_18_10"

name = "13_12_2022_11_23_13"
name = "13_12_2022_11_24_57"


name = "13_12_2022_15_47_16"
name = "13_12_2022_15_49_40"
name = "19_12_2022_08_55_00"
name = "02_02_2023_10_30_32"
name = "02_02_2023_11_09_43"
name = "02_02_2023_11_41_36"
name = "02_02_2023_11_55_09"
name = "02_02_2023_14_01_30"
name = "02_02_2023_15_23_44"
name = "02_02_2023_15_31_32"
name = "02_02_2023_16_20_04"
name = "02_02_2023_16_47_03"
name = "03_02_2023_10_01_27"
name = "03_02_2023_10_12_14"
name = "03_02_2023_14_47_41"
name = "03_02_2023_15_03_25"
name = "03_02_2023_15_31_07"
name = "03_02_2023_15_31_07"
name = "03_02_2023_16_04_41"
name = "03_02_2023_16_04_41"
name = "03_02_2023_16_47_37"
name = "03_02_2023_16_53_47"
name = "04_02_2023_11_39_08"
name = "04_02_2023_12_21_04"
name = "04_02_2023_13_21_29"
name = "04_02_2023_13_41_54"
name = "04_02_2023_14_12_15"

name = "08_02_2023_11_04_46"
name = "09_02_2023_11_10_51"
name = "09_02_2023_11_06_52"
name = "09_02_2023_11_16_11"
name = "09_02_2023_11_22_09"

name = "09_02_2023_11_34_47"
name = "09_02_2023_11_54_35"
name = "09_02_2023_12_12_41"
name = "09_02_2023_12_29_15"
name = "09_02_2023_13_58_57"
name = "09_02_2023_15_35_57"
name = "09_02_2023_16_00_09"
name = "09_02_2023_16_15_18"
name = "09_02_2023_16_28_07"
name = "09_02_2023_16_41_03"
name = "09_02_2023_16_53_40"
name = "09_02_2023_17_37_26"
name = "09_02_2023_18_05_37"
name = "09_02_2023_18_30_50"
name = "09_02_2023_20_03_36"
name = "09_02_2023_20_31_52"
name = "09_02_2023_21_12_42"
name = "10_02_2023_18_53_40"
name = "10_02_2023_19_26_36"

name = "27_03_2023_16_15_32"
name = "27_03_2023_16_30_45"
name = "27_03_2023_16_58_54"
name = "27_03_2023_17_20_17"
name = "28_03_2023_08_51_22"

name = "17_01_2023_19_29_37"
name = "17_01_2023_18_49_15"
name = "04_04_2023_08_52_48"

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


wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
#sim_data.save_grasp_parameters()

print('create_frames_pull_epsilon3')
sim_data.create_frames_pull_epsilon3(True,3)

#print('create_frames_u')
#sim_data.create_frames_u(membrane,wxmin,wxmax,wymin,wymax)

print('plot_epsilon4')
sim_data.plot_epsilon4()

# print('plot_potential_Field_value_sum')
# sim_data.plot_potential_Field_value()

# print('plot_potential_Field_value')
# sim_data.plot_potential_Field_value_individual()

#### this is for pull tests #####
#sim_data.create_frames_pull_epsilon2(True,1.75)

#######################################


# ##### This is for sorting #####
# sim_data.sort_epsilon_and_theta()
# sim_data.plot_epsilon_vs_theta_section()

# sim_data.create_frames_pull_epsilon3(True,6)
# print('plot_epsilon_vs_theta')
# sim_data.plot_epsilon_vs_theta()




# ###### Tjos is for wierd object #####
# sim_data.plot_ball_position_vs_estimate()
# membrane=True
# print('sort_epsilon_and_theta')
# sim_data.sort_epsilon_and_theta()

# print('plot_epsilon_vs_theta_section')
# sim_data.plot_epsilon_vs_theta_section()

# print('create_frames')
# sim_data.create_frames(membrane)

#print('create_frames_pull_epsilon0')
#sim_data.create_frames_pull_epsilon3(membrane,wxmin,wxmax,wymin,wymax)

#print('plot_epsilon4')
#sim_data.plot_epsilon4()

# print('create_wrenches_slices_frames2')
# sim_data.create_wrenches_slices_frames2()

#print('create_frames_contact_forces')
#sim_data.create_frames_contact_forces(2.5)

#print('Forcechains_arrows')
#sim_data.Forcechains_arrows(2.5)

#print('plot_epsilon_vs_theta')
#sim_data.plot_epsilon_vs_theta()


# print('plot_ball_pull_forces_trial')
# sim_data.plot_ball_pull_forces_trial()

# print('plot_ball_pull_forces')
# sim_data.plot_ball_pull_forces()

#print('create_frames_pull_epsilon')
#sim_data.create_frames_pull_epsilon(membrane,1.75)

# print('create_frames_zoomed_in')
# sim_data.create_frames_zoomed_in(membrane,1)



#print('Forcechains_arrows')
#sim_data.Forcechains_arrows(1)

# print('create_frames_pressure_no_boundary')
# sim_data.create_frames_pressure_no_boundary()

# print('create_frames_pressure')
# sim_data.create_frames_pressure()

#print('create_frames_control_forces')
#sim_data.create_frames_control_forces(1)

# print('create__frames_robot_forces')
# sim_data.create__frames_robot_forces()

# print('create_wrenches_slices_frames')
# sim_data.create_wrenches_slices_frames()

# print('plot_epsilon')
# sim_data.plot_epsilon()

# print('plot_epsilon2')
# sim_data.plot_epsilon2()

# print('plot_epsilon3')
# sim_data.plot_epsilon3()

#print('plot_epsilon4')
#sim_data.plot_epsilon4()

# print('plot_pressure')
# sim_data.plot_pressure()


# print('plot_control_forces')
# sim_data.plot_control_forces()

# print('plot_contact_number')
# sim_data.plot_contact_number()

# print('plot_ball_position')
# sim_data.plot_ball_position()

#print('plot_ball_contact_forces')
#sim_data.plot_ball_contact_forces()

#print('plot_ball_velocity')
#sim_data.plot_ball_velocity()

###########################################################


#sim_data.plot_Wrench_space_3D2(50)
#sim_data.plot_Wrench_space_3D(50)



