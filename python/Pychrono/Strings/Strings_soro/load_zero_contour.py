# -*- coding: utf-8 -*-
"""
Created on Mon Dec 27 13:44:27 2021

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
import numpy as np
import matplotlib.pyplot as plt
path = os.path.dirname(__file__)
path=path+"/Experiments/"
name='05_01_2022_16_58_58'
filename="shapeswrench.npz"

data=np.load(path+name+"/"+filename)
x1=data['xp']
y1=data['yp']

os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]
filename="shapeswrench.npz"

data=np.load(path+name+"/"+filename)
x2=data['xp']
y2=data['yp']


#const=2.75
#x=[0,.56,.66,1,1,.75,.75,1,1,.66,.56,0,0]
#y=[.125,.125,0,0,.125,.125,.375,.375,.5,.5,.375,.375,.125]

#x=np.dot(const,x)
#y=np.dot(const,y)
#xp=np.sum(x)/len(x)
#yp=np.sum(y)/len(y)

#x2=x-xp+0.5
#y2=y-yp

plt.plot(x1,y1)
#plt.plot(x2,y2)