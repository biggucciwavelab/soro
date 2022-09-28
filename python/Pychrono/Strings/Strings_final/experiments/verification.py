# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 13:53:52 2021

@author: dmulr
"""

import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
import csv
import timeit
import scipy.constants 
import pandas as pd
from matplotlib import animation
import matplotlib.font_manager as fm
import matplotlib as mpl
from matplotlib import colors as colors
from IPython.display import HTML
from scipy.signal import lfilter
from objects import *


width=200
#path="2021-06-28 18h45/data.csv"
path="2021-07-05 20h24/data.csv"
(Xb,Yb,time0,Xm,Ym)=import_data(path)
#np.savez('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/experiments/Marker.npz',Xm=Xm,Ym=Ym)



#create_animation(Xb,Yb,Xm,Ym,time0,width)

proc=process_experiments(Xb,Yb,Xm,Ym,time0)


proc.find_velocities()
proc.find_center()
(Vxc,Vyc,Xc,Yc)=proc.return_variables()
entry=1
(d,dr)=proc.find_radius(entry)
(Xm,Ym)=proc.transform_marker(entry,Xm,Ym)
np.savez('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Marker.npz',Xm=Xm,Ym=Ym,time=time0)


proc.plot_center(Vxc,Vyc,Xc,Yc,time0)



proc.plot_marker(Xm,Ym,time0)