# -*- coding: utf-8 -*-
"""
Created on Tue Jan  3 10:30:46 2023

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
from scipy.ndimage import gaussian_filter1d
import matplotlib.patches as patches
def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

#name2 = "12_01_2023_11_14_58"

#name2 = "13_01_2023_12_51_42"
name2 = "13_01_2023_19_45_09"
d=2
name = "14_01_2023_15_32_52"
name = "14_01_2023_15_33_04"
# circle 


snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d
nn=10


Psi=sim_obj.R_functions(name)
temp=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)

epsilon_=temp.EPSILON_
time_=temp.time
epsilon_clean=moving_average(temp.EPSILON_, n=nn)

# %%
tzero=[]

entry=0
count=0
while entry==0:
    entry=epsilon_[len(epsilon_)-1-count] 
    count=count+1
tzero=len(epsilon_)-count+1




# %%


name="epsilon_circle_shapes_alpha_2p25"
c="tab:red"
name="epsilon_square_shapes_alpha_2p25"
c="tab:blue"
#fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(7.125,2),dpi=300)
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
axs.plot(time_,epsilon_,color=c,linewidth=2,alpha=0.5)
axs.plot(time_[nn-1:],epsilon_clean,color=c,linewidth=2)
axs.scatter(time_[tzero],epsilon_[tzero],marker='o',color='k',edgecolors='k',zorder=3)
entry1_=[0,30,75,tzero]

for i in entry1_:   
    axs.scatter(time_[i],epsilon_[i],marker='o',color=c,edgecolors='k',zorder=3)
    
axs.set_xlim([-1,35])
xticks=[0,5,10,15,20,25,30,35]#,50,55,60]#,65,70,75,80,85,90]
yticks=[0,1,2,3,4]
axs.set_xticks(np.round(xticks,2))
axs.set_yticks(np.round(yticks,2))
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(b)$')
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# plt.close('all')


#%%
# membrane=True
# directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"

# wxmin=-2
# wxmax=4
# wymin=-2
# wymax=2
# fxs=1.5
# fys=1
# name="circle"
# for i in range(len(entry1_)):
#     temp.create_frames_snapshot(name,True,membrane,directory,entry1_[i],wxmin,wxmax,wymin,wymax,fxs,fys)