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



name='19_05_2021_19_16_40'
directory='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/anim'+name
export='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/pov'+name 
file='png'
create_video(name,directory,export,file)