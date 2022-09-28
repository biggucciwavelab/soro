# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 13:55:19 2021

@author: dmulr
"""
import os
import csv
import timeit
import numpy as np
import math as math
import Strings_objects as sim_obj
#import Strings_config as cf
from Strings_sim_plot_objects import *



name='18_03_2021_11_25_40'

directory='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/video_capture/'
export='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+name 
file='bmp'
create_video(name,directory,export,file)