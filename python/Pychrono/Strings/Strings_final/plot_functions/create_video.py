# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 16:45:54 2021

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
ni=401
err=0.0
radius=0.038
radius2=0.04
R=0.967094387
mode='nmax'
sim='02_03_2021_16_51_06'
width=6
#filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/bot_position.csv'
#filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/particle_position.csv'

filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/bot_position.csv'
filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/particle_position.csv'




filename4='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/radius/R'+sim+'.npz' # radius file name 
result_dir='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'
phi=create_videos_irr(filename,filename2,filename4,result_dir,nb,ni,mode,R,radius,radius2,sim,width,err)

phi.sort_data()
phi.create_images_nmax()