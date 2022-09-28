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


path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]

d=175
snap_shot=False
membrane=False
dxmin=-200
dxmax=100
dymin=-60
dymax=60
#dxmin,dxmax,dymin,dymax
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax)
#sim_data=sim_obj.import_data(name,path,-d,d,-d,d)
#Psi=sim_obj.Potential_fields(name)
#Z=Psi.Z3
#X=Psi.X
#Y=Psi.Y
if snap_shot==False:
    sim_data.create_frames_morph(membrane)
    #sim_data.create_frames(membrane)
    sim_data.create_video()
    #sim_data.plot_field_values()
    #sim_data.create_snap_shot(-1,membrane)    
    #sim_data.create_snap_shot_field(-1,membrane,Z,X,Y)
else:

        # entry=0
        #entry=20
        #entry=49
        # entry=55
        #
        # entry=99
    sim_data.create_snap_shot_morph(140,False) 
    
    #sim_data.create_snap_shot(0,False) 
    ##sim_data.create_snap_shot2(150)
    
#sim_data.create_snap_shot2(None)


