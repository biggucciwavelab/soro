# -*- coding: utf-8 -*-
"""
Created on Fri Jul 23 16:22:05 2021

@author: dmulr
"""
import os
import csv
import numpy as np
import pandas as pd
files = []
path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]
print(name)
#name='03_05_2022_13_38_01'
#name='05_01_2022_16_58_58'
mainDirectory = "F:/Soro_chrono/python/Pychrono/Strings/New_strings/Experiments/"
parameters=np.load(mainDirectory+name+'/Parameters.npy',allow_pickle=True)
parameters=parameters.tolist()

print(parameters)

