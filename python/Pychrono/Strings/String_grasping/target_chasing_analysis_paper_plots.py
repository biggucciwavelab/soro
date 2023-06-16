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
path=path+"/Experiments/target_chasing/good_ones/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = "06_06_2023_19_19_38"
name = "07_06_2023_17_38_04"#####
d=4
snap_shot=False
membrane=False
dxmin=-4
dxmax=13
dymin=-6
dymax=0

wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
time=sim_data.time
ratio  = (wymax-wymin)/(wxmax-wxmin)
fsx=3.5
fsy=fsx*ratio
entry=[0,113,157,220] # tanh
entry=[0,150,350] # sin
entry=[0,125,350,600]
sim_data.create_frames_target_snapshot(entry,membrane,wxmin,wxmax,wymin,wymax,fsx,fsy)




