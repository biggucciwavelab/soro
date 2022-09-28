# -*- coding: utf-8 -*-
"""
Created on Thu Mar 18 17:26:36 2021

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

path1='18_03_2021_11_37_15'
path2='18_03_2021_11_37_21'
path3='18_03_2021_11_37_29'
path4='18_03_2021_11_37_36'
name1='mu=0.1'
name2='mu=0.2'
name3='mu=0.3'
name4='mu=0.4'

name=[name1,name2,name3,name4]
path=[path1,path2,path3,path4]


phi=compare_pressure(name,path,cf.nb,cf.n,cf.radius,cf.radius2,cf.height)
phi.plot_pressure()