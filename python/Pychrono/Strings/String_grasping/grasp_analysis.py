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






# circle
#name="06_10_2022_09_32_06"
#name="10_10_2022_10_09_58"
#name="10_10_2022_11_31_48"
#name="10_10_2022_12_40_01"
#name="10_10_2022_14_15_02"

#name="11_10_2022_12_12_03_circle"

# square
#name="06_10_2022_09_38_13"
#name="10_10_2022_16_06_39"

name="11_10_2022_16_28_04"

name="11_10_2022_17_26_37"

# triangle
#name="06_10_2022_09_38_31"
#name="10_10_2022_16_07_03"
#name="04_10_2022_18_34_49"
#name="11_10_2022_10_08_47"


#name="11_10_2022_12_12_55_triangle"
d=2.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=3.5
dymin=-d
dymax=d


sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax)
sim_data.save_grasp_parameters()



# membrane=True
# print('create_frames')
# sim_data.create_frames(membrane)

# print('plot_ball_pull_forces_trial')
# sim_data.plot_ball_pull_forces_trial()

# print('plot_ball_pull_forces')
# sim_data.plot_ball_pull_forces()

# print('create_frames_pull_epsilon')
# sim_data.create_frames_pull_epsilon(membrane,1.75)

# print('create_frames_zoomed_in')
# sim_data.create_frames_zoomed_in(membrane,1)

#print('create_frames_contact_forces')
#sim_data.create_frames_contact_forces(.4)

# print('Forcechains_arrows')
# sim_data.Forcechains_arrows(1)

# print('create_frames_pressure_no_boundary')
# sim_data.create_frames_pressure_no_boundary()

# print('create_frames_pressure')
# sim_data.create_frames_pressure()

# print('create_frames_control_forces')
# sim_data.create_frames_control_forces(1)

#print('create__frames_robot_forces')
#sim_data.create__frames_robot_forces()

# print('create_wrenches_slices_frames')
# sim_data.create_wrenches_slices_frames()

# print('plot_epsilon')
# sim_data.plot_epsilon()

# print('plot_epsilon2')
# sim_data.plot_epsilon2()

# print('plot_epsilon3')
# sim_data.plot_epsilon3()

# print('plot_epsilon4')
# sim_data.plot_epsilon4()

print('plot_pressure')
sim_data.plot_pressure()


#print('plot_control_forces')
# sim_data.plot_control_forces()


#sim_data.plot_contact_number()

# print('plot_ball_position')
# sim_data.plot_ball_position()

# print('plot_ball_contact_forces')
# sim_data.plot_ball_contact_forces()

# print('plot_ball_velocity')
# sim_data.plot_ball_velocity()
###########################################################

#sim_data.plot_Wrench_space_3D2(200)
#sim_data.plot_Wrench_space_3D(-1)



