# -*- coding: utf-8 -*-
"""
Created on Tue Dec  7 14:18:08 2021

@author: dmulr
"""

import pychrono.core as chrono
import timeit
start=timeit.default_timer()
import objects as sim_obj
import random
import os
import csv
import glob
from IPython.display import HTML
import matplotlib.pyplot as plt
path = os.path.dirname(__file__)

#name = "11_06_2022_14_13_13"
#name = "11_06_2022_13_09_06"
#path = "F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/Experiments/__good_experiments/shape/Square/"
#path="F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/Experiments/__good_experiments/shape/Oval/"
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]
#name="23_06_2022_16_44_22"# tunnel 2
#name="23_06_2022_16_52_17" # tunnel 1
#name="23_06_2022_18_30_35" # tunnel 0
#name="15_06_2022_17_01_30"
#name = "15_06_2022_13_01_10"
#name="15_06_2022_10_55_14" # oval
#name="11_06_2022_15_44_36" # square
#name="27_06_2022_17_20_28"
#name="26_06_2022_15_22_03"
#name="28_06_2022_10_19_47"
#name="04_07_2022_14_43_26"
#name="04_07_2022_14_36_30"
#name="04_07_2022_14_32_31"

#name="04_07_2022_17_21_48"
#name="04_07_2022_17_25_12"
#name="04_07_2022_17_58_28"

#name="05_07_2022_15_41_54"
#name="05_07_2022_15_46_39"
#name="05_07_2022_15_58_35"

#name="07_07_2022_21_25_05"
#name="07_07_2022_18_37_03"
#name="07_07_2022_09_27_45"
#name="07_07_2022_16_15_07"
#name="07_07_2022_18_37_03"
name="15_07_2022_10_40_45"
#name="15_07_2022_10_40_38"
#name="15_07_2022_10_40_19"

# name="15_07_2022_10_44_51"
# name="15_07_2022_10_44_45"
# name="15_07_2022_10_44_34"

# name="15_07_2022_10_45_44"
# name="15_07_2022_10_45_55"
# name="15_07_2022_10_46_01"
d=175
snap_shot=False
membrane=False
tunnel=False
#dxmin=-155
#dxmax=59
#dymin=-60
#dymax=60
#dymin=-2
#dymax=2
#dxmin=-2
#dxmax=6

dymin=-2
dymax=2
dxmin=0
dxmax=4
const=1
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax)
#sim_data=sim_obj.import_data(name,path,-d,d,-d,d)
#Psi=sim_obj.Potential_fields(name)
#Z=Psi.Z3
#X=Psi.X
#Y=Psi.Y
if snap_shot==False:
    #sim_data.create_frames_morph(membrane)
    sim_data.create_frames(membrane,const,tunnel_=tunnel)
    sim_data.create_video()
    #sim_data.plot_field_values()
    #sim_data.create_snap_shot(0,const,membrane)    
    #sim_data.create_snap_shot_field(-1,membrane,Z,X,Y)
else:
    ## morphing ##
    # entry=0
    # entry=50
    # entry=100
    # entry=150
    # entry=250
    # entry=300
    # entry=350
    # entry=400
    #entry=49
    #entry=99
    #entry=200
    #sim_data.create_snap_shot_morph(entry,const,False) 
    
    ## tunnel2 ##
    #entry=0
    #entry=10
    #entry=20
    #entry=0
    #entry=10
    #entry=20
    #entry=30
    #entry=50
    # entry=30
    # entry=40
    # entry=50
    #entry=120
    # entry=70
    # entry=80
    # entry=90
    # entry=100
    # entry=110
    # entry=120
    #entry=130
    #entry=140
    #entry=160
    #sim_data.create_snap_shot_tunnel([0,100,325,650,1050],const,False,tunnel_=tunnel) 
    #sim_data.create_snap_shot_tunnel([0],const,False,tunnel_=tunnel) 
    sim_data.create_snap_shot(23,const,False,tunnel_=tunnel) 
    ##sim_data.create_snap_shot2(150)
    
#sim_data.create_snap_shot2(None)


