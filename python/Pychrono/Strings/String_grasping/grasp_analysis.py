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
import objects as sim_obj
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






#name="23_09_2022_16_51_45"
#name="28_09_2022_08_34_33"
#name="28_09_2022_08_35_52"
d=2.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=3.5
dymin=-d
dymax=d


sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax)
sim_data.save_grasp_parameters()

#sim_data.create_frames_contact_forces(.4)
#sim_data.create_contact_forces_video()
#sim_data.create_frames_control_forces()
#sim_data.create__frames_robot_forces()
#sim_data.create_video_robot_forces()


#sim_data.create_frames_pressure_no_boundary()
#sim_data.create_video_pressure_no_boundary()

# sim_data.create_frames_pressure()
# sim_data.create_video_pressure()

# sim_data.plot_control_forces()
# sim_data.plot_ball_position()
# sim_data.plot_epsilon()
# sim_data.plot_ball_contact_forces()

# sim_data.create_wrenches_slices_frames()
# sim_data.create_wrenches_slices_frames_video()

# sim_data.create_frames_wrench()
# sim_data.create_video_grasping_wrench()

# sim_data.create_frames_grasping()
# sim_data.create_video_grasping()

# membrane=True
# sim_data.create_frames(membrane,d)
# sim_data.create_video()

#sim_data.create_frames_zoomed_in(membrane,1)
#sim_data.create_video_zoomed_in()

# sim_data.Forcechains_arrows(0.4)
# sim_data.create_video_contact_arrow()

# sim_data.Forcechains(0.5)
# sim_data.create_video_forcechain()
# sim_data.create_frames_control_forces()
# sim_data.create_video_control()



#sim_data.plot_Wrench_space_3D(-1)
# #sim_data.extract_contact_forces()
# #sim_data.find_pressure()


