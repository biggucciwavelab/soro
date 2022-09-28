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


d=175
snap_shot=False
membrane=False
tunnel=False
dymin=-2
dymax=2
dxmin=-2
dxmax=6
const=1
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax)

