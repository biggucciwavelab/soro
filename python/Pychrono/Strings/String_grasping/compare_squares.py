# -*- coding: utf-8 -*-
"""
Created on Sat Jan 28 12:18:59 2023

@author: Big Gucci
"""

import warnings
warnings.filterwarnings("ignore")
import pychrono.core as chrono
import timeit
import numpy as np
start=timeit.default_timer()
import objects4 as sim_obj
import random
import os
import csv
import glob
from IPython.display import HTML
import matplotlib.pyplot as plt
import itertools 
from scipy.stats import norm
import statistics
from matplotlib.ticker import PercentFormatter

parameters=np.load('square_surface.npy',allow_pickle=True)
parameters=parameters.tolist()
mu_s=parameters["mu_"]
time_s=parameters["time"]
epsilonsigmap_s=parameters["epsilonsigmap"]
epsilonsigmam_s=parameters["epsilonsigmammod"]
mean_s=parameters["mean"]
sigma_s=parameters["sigma"]




parameters=np.load('square_corner.npy',allow_pickle=True)
parameters=parameters.tolist()
mu_c=parameters["mu_"]
time_c=parameters["time"]
epsilonsigmap_c=parameters["epsilonsigmap"]
epsilonsigmam_c=parameters["epsilonsigmammod"]
mean_c=parameters["mean"]
sigma_c=parameters["sigma"]


fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)

axs.plot(time_c,mu_c,linewidth=2,color="tab:green")
axs.plot(time_s,mu_s,linewidth=2,color="tab:blue")
axs.set_ylabel('$\epsilon$',labelpad=-1,fontsize=9)
axs.set_xlabel('time (s)',labelpad=-2,fontsize=9)
#axs.set_title(r'$(c)$',fontsize=9)

axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=-1)


