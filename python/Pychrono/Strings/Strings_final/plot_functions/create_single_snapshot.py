# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 18:48:28 2021

@author: dmulr
"""


import os
import csv
import timeit
import numpy as np
import math as math
import Strings_objects as sim_obj
import Strings_config as cf
from Strings_sim_plot_objects import *


nb=30
ni=514
err=0.0
radius=0.02908
radius2=0.04
R=0.967094387
mode='nmax'
sim='06_04_2021_16_39_12'


# 04_04_2021_12_30_10
# 04_04_2021_12_30_17


width=2.25
tim=[10,20]
Rb=1
#filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/bot_position.csv'
#filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/particle_position.csv'

filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/bot_position.csv'
filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/particle_position.csv'
filename3='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+sim+'.npz'
filename4='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/radius/R'+sim+'.npz'
result_dir='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'
phi=create_videos_irr(filename,filename2,filename3,filename4,None,None,result_dir,nb,ni,mode,R,radius,radius2,sim,width,err,tim,Rb)
phi.sort_data()
N=199
#N=399
#N=598
#N=0
#phi.create_images_nmax()
#phi.create_images_nhomomax()
#phi.create_images_nhomomax_snap_shot(N)
phi.create_images_nmax1_shape_snapshot(N)
#phi.create_images_nhomomax_1shape()
#phi.create_images_nhomomax_field(cf.phi)

# directory='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'_video'
# export='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'field'
# file='jpg'
# create_video(sim,directory,export,file)