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

name = "26_04_2023_13_34_06"
name = "26_04_2023_14_20_07"
name = "26_04_2023_15_07_07"
name = "26_04_2023_16_34_49"

name = "26_04_2023_23_16_23"
name = "26_04_2023_23_51_14"
#name = "27_04_2023_00_12_45"
name = "27_04_2023_00_12_45"


# playing with Hz
name = "27_04_2023_10_11_20"
name = "27_04_2023_10_13_20"
name = "27_04_2023_10_21_51"
name = "27_04_2023_10_36_06"
name = "27_04_2023_10_47_40"
name = "27_04_2023_11_02_03"
name = "27_04_2023_12_54_22"
#name = "27_04_2023_10_02_10"
name = "27_04_2023_14_43_56"
name = "27_04_2023_15_17_39"
name = "27_04_2023_15_40_31"
name = "27_04_2023_16_04_11"
name = "27_04_2023_16_28_47"
name = "01_05_2023_10_08_23"
name = "08_05_2023_10_20_29"
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
name1 = "27_04_2023_18_59_23"
name = "28_04_2023_23_08_41"
name = "28_04_2023_23_43_55"
name = "29_04_2023_12_30_46"
name = "29_04_2023_13_05_52"
name = "29_04_2023_13_49_14"
name = "29_04_2023_14_04_50"
name = "29_04_2023_14_18_51"
name = "29_04_2023_14_32_22"
name = "29_04_2023_14_40_38"
name = "29_04_2023_14_50_33"
name = "29_04_2023_16_01_07"
name = "29_04_2023_16_29_04"
name = "29_04_2023_16_55_37"
name = "29_04_2023_17_00_23"
name = "29_04_2023_17_37_18"
name = "29_04_2023_18_49_46"
name = "29_04_2023_19_51_32"
name = "29_04_2023_20_21_40"
name = "29_04_2023_20_56_27"
name = "29_04_2023_21_32_44"
name = "29_04_2023_22_13_07"
name = "29_04_2023_23_07_45"
name = "29_04_2023_23_08_20"
name = "30_04_2023_11_38_47"
name = "30_04_2023_11_38_05"
name = "30_04_2023_12_00_11"
name = "30_04_2023_12_25_22"
name = "30_04_2023_12_52_18"
name = "30_04_2023_13_17_58"
name = "30_04_2023_13_41_40"
name = "30_04_2023_14_35_27"

name = "30_04_2023_14_26_40"
name = "30_04_2023_15_10_13"
name = "30_04_2023_15_34_13"###


#name = "30_04_2023_16_13_02"

name = "30_04_2023_17_24_26"

name = "01_05_2023_09_24_48"
name = "01_05_2023_09_34_05"
name = "01_05_2023_09_49_34"
name = "01_05_2023_10_08_23"
name = "01_05_2023_10_30_22"
name = "01_05_2023_10_58_14"
name = "01_05_2023_11_12_09"
name = "01_05_2023_11_56_10"
name = "01_05_2023_12_17_59"
name = "01_05_2023_12_40_43"
name = "01_05_2023_13_04_29"
name = "01_05_2023_13_29_57"
name = "01_05_2023_13_47_16"
name = "01_05_2023_14_26_12"
name = "01_05_2023_15_27_00"
name = "02_05_2023_12_59_52"
name = "02_05_2023_13_24_40"
name = "02_05_2023_13_23_35"
name = "02_05_2023_13_41_15"
name = "02_05_2023_14_05_06"
name = "02_05_2023_14_16_09"
name = "02_05_2023_14_04_52"
name = "02_05_2023_15_08_36"
name = "02_05_2023_15_12_19"
name = "02_05_2023_15_34_52"
name = "02_05_2023_15_52_32"
name = "02_05_2023_15_48_58"
name = "02_05_2023_16_05_49"
name = "02_05_2023_16_20_21"
name = "02_05_2023_21_00_14"
name = "03_05_2023_08_20_18"
name = "03_05_2023_08_49_36"
name = "03_05_2023_09_20_21"
name = "03_05_2023_09_15_24"
name = "03_05_2023_09_28_10"

name = "03_05_2023_09_57_32"
name = "03_05_2023_09_58_21"
name = "03_05_2023_10_28_36"

name = "03_05_2023_10_54_48"
name = "03_05_2023_11_32_03"
name ="03_05_2023_11_33_10"
name = "03_05_2023_12_45_18"
name = "03_05_2023_12_48_32"
name = "03_05_2023_12_44_06"
name = "03_05_2023_13_08_41"
name = "03_05_2023_13_15_51"
name = "03_05_2023_13_34_21"
name = "03_05_2023_13_39_01"
name = "03_05_2023_14_10_31"
name = "03_05_2023_14_17_09"
name = "03_05_2023_15_13_01"
name = "03_05_2023_15_56_12"
name = "03_05_2023_16_46_04"
name = "03_05_2023_18_43_30"
name = "03_05_2023_19_44_14"

name = "03_05_2023_20_48_43"
name = "04_05_2023_09_51_21"
name = "04_05_2023_11_59_01"
name = "04_05_2023_15_41_11"
name = "04_05_2023_17_35_27"
name = "05_05_2023_10_46_09"

name = "05_05_2023_13_09_58"
name = "05_05_2023_13_08_31"
name = "05_05_2023_13_38_44"
name = "05_05_2023_14_14_07"
name = "05_05_2023_15_39_31"
name = "06_05_2023_12_18_46"
name = "06_05_2023_15_50_47"
name = "07_05_2023_12_26_47"
name = "07_05_2023_14_13_21"
name = "07_05_2023_17_34_09"
name = "07_05_2023_22_05_58"

name = "08_05_2023_09_08_10"
name = "08_05_2023_10_20_29"
#### Grasping epsilon

wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
#sim_data.save_grasp_parameters()

#sim_data.create_frames_rfunction(-2)
#print('create_frames_u2')
#sim_data.create_frames_u2(membrane,wxmin,wxmax,wymin,wymax)


print('create_frames_u3')
sim_data.create_frames_u3(membrane,wxmin,wxmax,wymin,wymax)
# # #print('create_frames_u')
# # #sim_data.create_frames_u(membrane,wxmin,wxmax,wymin,wymax)



print('plot_epsilon4')
sim_data.plot_epsilon4()

sim_data.plot_epsilon5dx()

print('plot_epsilon5')
sim_data.plot_epsilon5()

print('plot_potential_Field_value_sum')
sim_data.plot_potential_Field_value()

print('plot_potential_Field_value')
sim_data.plot_potential_Field_value_individual()

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



