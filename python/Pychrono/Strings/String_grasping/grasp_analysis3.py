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


name = "27_04_2023_00_12_45"
name = "27_04_2023_17_01_36"
name = "27_04_2023_17_36_00"
name = "28_04_2023_13_10_42"
name = "28_04_2023_14_05_03"
name = "28_04_2023_14_57_07"
name = "28_04_2023_15_24_51"
name = "28_04_2023_16_00_38"
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


#print('create_frames_u')
#sim_data.create_frames_u(membrane,wxmin,wxmax,wymin,wymax)



#print('plot_epsilon4')
#sim_data.plot_epsilon4()
sim_data.plot_epsilon5dx()

print('plot_epsilon5')
sim_data.plot_epsilon5()

#sim_data.create_frames_pull_epsilon2(True,d)

#sim_data.create_frames_pull_epsilon3(True,dxmin,dxmax,dymin,dymax)

#print('plot_potential_Field_value_sum')
#sim_data.plot_potential_Field_value()

#print('plot_potential_Field_value')
#sim_data.plot_potential_Field_value_individual()

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



