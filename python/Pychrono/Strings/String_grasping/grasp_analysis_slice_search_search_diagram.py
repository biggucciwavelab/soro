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
path="D:/dmulroy/Experiments/square_search/"

name = "28_01_2023_12_44_16"

d=3.7
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

#Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,None)

# %%
entry=0
sim_data.create_frames_search(membrane,d,entry,'(i)')

entry=25
sim_data.create_frames_search(membrane,d,entry,'(ii)')

entry=50
sim_data.create_frames_search(membrane,d,entry,'(iii)')

entry=75
sim_data.create_frames_search(membrane,d,entry,'(iv)')
