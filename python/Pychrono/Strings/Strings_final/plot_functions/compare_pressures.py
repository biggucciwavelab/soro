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


path1='23_03_2021_14_58_15'

path2='23_03_2021_14_58_23'

path3='23_03_2021_14_58_31'

path4='23_03_2021_14_58_36'

path5='23_03_2021_14_58_43'


name1='alpha=40'
name2='alpha=50'
name3='alpha=60'
name4='alpha=70'
name5='alpha=80'



# path1='22_03_2021_10_13_09'
# path2='22_03_2021_10_13_19'
# path3='22_03_2021_10_13_27'
# path4='22_03_2021_10_13_32'
# path5='22_03_2021_10_13_37'
# path6='22_03_2021_10_13_49'
# #path7='21_03_2021_15_38_13'

# # name1='mu=0.1'
# # name2='mu=0.2'
# # name3='mu=0.3'
# # name4='mu=0.4'
# # name5='mu=0.5'
# # name6='mu=0.6'

# name1='mu=0.6'
# name2='mu=0.5'
# name3='mu=0.4'
# name4='mu=0.3'
# name5='mu=0.2'
# name6='mu=0.1'

# # name1='40'
# # name2='60'
# # name3='80'
# # name4='100'
# # name5='120'
# # name6='140'
# # name7='160'

name=[name1,name2,name3,name4,name5]
path=[path1,path2,path3,path4,path5]


phi=compare_pressure(name,path,cf.nb,cf.n,cf.radius,cf.radius2,cf.height,cf.Rb)
phi.plot_pressure()