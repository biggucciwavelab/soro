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


d=2

name= ["18_01_2023_09_21_26",
       "18_01_2023_09_21_33",
       "18_01_2023_09_21_38",
       "18_01_2023_09_21_44",
       "18_01_2023_09_21_50",
       "18_01_2023_09_21_56",
       "18_01_2023_09_22_06",
       "18_01_2023_10_00_12",
       "18_01_2023_10_00_18",
       "18_01_2023_10_00_22",
       "18_01_2023_10_00_30",
       "18_01_2023_10_00_36",
       "18_01_2023_10_00_42",
       "18_01_2023_13_47_12",
       "18_01_2023_13_47_19",
       "18_01_2023_13_47_24",
       "18_01_2023_13_47_30",
       "18_01_2023_13_47_37",
       "18_01_2023_13_47_48",
       "18_01_2023_13_47_55",
       "18_01_2023_15_39_22",
       "18_01_2023_15_39_36",
       "18_01_2023_15_39_50",
       "18_01_2023_15_40_17"]      
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d
nn=10
sim_data=[]
Psi=[]
epsilon_=[]
epsilon_clean=[]
time_=[]
for i in range(len(name)):
    print("set:", str(i))
    Psi.append(sim_obj.R_functions(name[i]))
    temp=sim_obj.import_data(name[i],path,dxmin,dxmax,dymin,dymax,Psi[i])
    sim_data.append(temp)
    epsilon_.append(temp.EPSILON_)
    time_.append(temp.time)
    epsilon_clean.append(moving_average(temp.EPSILON_, n=nn))

# %%
tzero=[]
for i in range(len(epsilon_clean)):
    entry=0
    count=0
    while entry==0:
        entry=epsilon_[i][len(epsilon_[i])-1-count] 
        count=count+1
    tzero.append(len(epsilon_[i])-count+1)
# %%
epsilonmu=[]
epsilonmax=[]
epsilonmin=[]


for i in range(len(time_[0])):
    temp=0
    tempa=[]
    for j in range(len(epsilon_)):
        temp=epsilon_[j][i]+temp
        tempa.append(epsilon_[j][i])

        
    epsilonmax.append(np.max(tempa))
    epsilonmin.append(np.min(tempa))    
    epsilonmu.append(temp/len(epsilon_))




epsilonmu2=[]
epsilonmax2=[]
epsilonmin2=[]
for i in range(len(time_[0][nn-1:])):

    temp2=0
    tempa2=[]
    for j in range(len(epsilon_)):
        temp2=epsilon_clean[j][i]+temp2
        tempa2.append(epsilon_clean[j][i])
        

    epsilonmax2.append(np.max(tempa2))
    epsilonmin2.append(np.min(tempa2))    
    epsilonmu2.append(temp2/len(epsilon_))

# %%
name="epsilon_square_interior_alpha_2p25"
c="tab:blue"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
axs.fill_between(time_[0],epsilonmax,epsilonmin,color=c,alpha=0.5)
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$")
for i in range(len(epsilon_clean)):
    #print(i)
    #for j in tzero:
    if tzero[i]==len(time_[i]):
        pass
    else:
        axs.scatter(time_[i][tzero[i]],epsilon_[i][tzero[i]],marker='o',color='k',s=2,edgecolors='k',zorder=3)

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
# # %% In[epsilon_three_shapes_legend]
# #entry1_=[0,30,75,100,120,125]

# %%
c="tab:blue"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
for i in range(len(epsilon_clean)):
    axs.plot(time_[i][nn-1:],epsilon_clean[i],linewidth=0.5,color=c)
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(b)$')
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # %% In[epsilon_three_shapes_legend]
# #entry1_=[0,30,75,100,120,125]


# %%
c="tab:blue"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
for i in range(len(epsilon_clean)):
    axs.plot(time_[i],epsilon_[i],linewidth=0.5,color=c)
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(b)$')
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # %% In[epsilon_three_shapes_legend]
# #entry1_=[0,30,75,100,120,125]


#%%
#name="epsilon_square_noisy_shapes_alpha_2p25"
c="tab:blue"
nn=10
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
axs.fill_between(time_[0][nn-1:],epsilonmax2,epsilonmin2,color=c,alpha=0.5)
axs.plot(time_[0][nn-1:],epsilonmu2,linewidth=2,color=c,label="$\mu$")

axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(b)$')
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # %% In[epsilon_three_shapes_legend]
# #entry1_=[0,30,75,100,120,125]