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
import objects2 as sim_obj
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


#name = files[-1]

#name = "14_11_2022_16_36_03"

#name = "14_11_2022_16_36_19"

#name = "14_11_2022_16_35_26"


#name = "11_10_2022_12_12_55" # TRIANGLE

#name = "11_10_2022_17_26_37" # SQUARE

#name = "11_10_2022_12_12_03" # circle




name ="29_11_2022_18_36_17" # circle 

name ="29_11_2022_18_36_48" # triangle

#name ="29_11_2022_18_36_33" # square



#name ="30_11_2022_12_51_48"
#name = "30_11_2022_17_39_10"
#name = "30_11_2022_17_58_52"

name = "01_12_2022_09_43_46"

d=5.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
#sim_data.save_grasp_parameters()




#### this is for pull tests #####
#sim_data.create_frames_pull_epsilon2(True,1.75)

#######################################


# ##### This is for sorting #####
sim_data.sort_epsilon_and_theta()
sim_data.plot_epsilon_vs_theta_section()

sim_data.create_frames_pull_epsilon3(True,6)
print('plot_epsilon_vs_theta')
sim_data.plot_epsilon_vs_theta()




###### Tjos is for wierd object #####
# membrane=True
# print('create_frames')
# sim_data.create_frames(membrane)

# print('plot_epsilon4')
# sim_data.plot_epsilon4()

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



