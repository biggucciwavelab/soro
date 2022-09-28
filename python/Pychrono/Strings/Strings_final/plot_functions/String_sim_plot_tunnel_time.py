# -*- coding: utf-8 -*-
"""
Created on Tue Feb 23 12:34:45 2021

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

path1='23_02_2021_14_41_12'
path2='23_02_2021_14_52_56'
name1='bi_disperse'
name2='mono_disperse'
nb = 20
phi = compare_path(path1,path2,name1,name2,nb)

phi.plot_position()