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


name = files[-1]

name = "14_11_2022_16_36_03"

#name = "14_11_2022_16_36_19"

#name = "14_11_2022_16_35_26"




#name = "10_11_2022_16_08_08"
# circle
#name="11_10_2022_12_12_03"

# square
#name="11_10_2022_17_26_37"

# triangle
#name="11_10_2022_12_12_55"


## 9.5 second run ##
# circle
#name="12_10_2022_16_40_57"

# square
#name="12_10_2022_16_41_19"

# triangle
#name="12_10_2022_16_41_37"

#name="18_10_2022_15_40_08"

#name="18_10_2022_15_36_41"
d=4.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
#sim_data.save_grasp_parameters()



membrane=True
print('create_frames')
sim_data.create_frames(membrane)

# print('plot_ball_pull_forces_trial')
# sim_data.plot_ball_pull_forces_trial()

# print('plot_ball_pull_forces')
# sim_data.plot_ball_pull_forces()

#print('create_frames_pull_epsilon')
#sim_data.create_frames_pull_epsilon(membrane,1.75)

# print('create_frames_zoomed_in')
# sim_data.create_frames_zoomed_in(membrane,1)

# print('create_frames_contact_forces')
# sim_data.create_frames_contact_forces(.4)

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

print('plot_epsilon4')
sim_data.plot_epsilon4()

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



