# -*- coding: utf-8 -*-
"""
Created on Sun Feb 21 11:35:52 2021

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


path1= '05_04_2021_10_08_55'
path2= '05_04_2021_11_06_22'
path3= '05_04_2021_11_06_52'
path4= '05_04_2021_11_07_00'
path5= '05_04_2021_11_11_32'

path6= '05_04_2021_11_11_32'
path7= '05_04_2021_12_09_49'
path8= '05_04_2021_12_09_55'


name1='p_4'
name2='p_3.5'
name3='p_3'
name4='p_2.5'
name5='p_2.0'

name6='p_1.5'
name7='p_1.0'
name8='p_0.5'


name=[name1,name2,name3,name4,name5,name6,name7,name8]
path=[path1,path2,path3,path4,path5,path6,path7,path8]
phi=compare_error(name,path)
phi.plot_error()
