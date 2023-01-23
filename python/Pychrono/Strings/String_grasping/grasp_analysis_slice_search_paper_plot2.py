# -*- coding: utf-8 -*-
"""
Created on Tue Dec 20 09:51:25 2022

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

path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

# noisy ones
# name = "14_01_2023_15_35_36"
# name = "14_01_2023_15_35_51"
#name = "13_01_2023_19_43_52"


name = "14_01_2023_15_35_36"
d=4.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name)  
sim_data=sim_obj.select_import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data.sort_epsilon_and_theta()


THETA=sim_data.THETA
EPSILON_=sim_data.EPSILON_
epsilon_section=sim_data.epsilon_section
epsilon_theta_section_max=sim_data.epsilon_theta_section_max
epsilon_theta_section_mean=sim_data.epsilon_theta_section_mean
epsilon_theta_section_median=sim_data.epsilon_theta_section_median

average_epsilon=sim_data.average_epsilon
max_epsilon=sim_data.max_epsilon
median_epsilon=sim_data.median_epsilon


plt.close("all")
# %%
name="searching_circle_bar_plot_max"
c="tab:red"
labels=['0',
        r'$\frac{\pi}{6}$',
        r'$\frac{\pi}{3}$',
        r'$\frac{\pi}{2}$',
        r'$\frac{2\pi}{3}$',
        r'$\frac{5\pi}{6}$',
        r'$\pi$',
        r'$\frac{7\pi}{6}$',
        r'$\frac{8\pi}{6}$',
        r'$\frac{3\pi}{2}$',
        r'$\frac{10\pi}{6}$',
        r'$\frac{11\pi}{6}$']

fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
y=[]
ymean=[]
for i in range(len(epsilon_theta_section_max)):
    entry=int(str(i))
    y_=epsilon_theta_section_max[str(i)]
    res2=np.nonzero(y_)
    res2=res2[0]
    y__=[]
    #print(y_)
    for j in res2:
        if y_[j]<3:
            y__.append(y_[j])
    y.append(y__)
    x=entry*np.ones(len(y__))
    axs2.scatter(x,y__,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
    error=[np.min(y__),np.max(y__)]
    axs2.plot([x[1],x[1]],error,color="tab:green",linewidth=1,zorder=3)
    axs2.scatter(entry,np.mean(y__),color=c,marker='s',s=3,zorder=3)
    ymean.append(np.mean(y__))
positions=[0,1,2,3,4,5,6,7,8,9,10,11]
axs2.set_xticks(positions)
axs2.set_xticklabels(labels,color='k',fontsize=8)
axs2.bar(labels,ymean,color="tab:red",zorder=3)
print(np.max(ymean)-np.min(ymean))
axs2.set_ylabel('$\epsilon$',labelpad=-1)
axs2.set_xlabel('angle',labelpad=-2)
axs2.set_title(r'$(a)$'+"max")
axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")


# %%
# for i in range(len(epsilon_theta_section_max)):
#     entry=int(str(i))
#     y_=epsilon_theta_section_max[str(i)]
#     fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
#     axs2.plot(y_,color="tab:green",linewidth=1,zorder=3)
# # %%
# name = "14_01_2023_15_35_51"

# d=4.5
# snap_shot=False
# membrane=True
# dxmin=-d
# dxmax=d
# dymin=-d
# dymax=d

# Psi=sim_obj.R_functions(name)  
# sim_data=sim_obj.select_import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
# sim_data.sort_epsilon_and_theta()


# THETA=sim_data.THETA
# EPSILON_=sim_data.EPSILON_
# epsilon_section=sim_data.epsilon_section
# epsilon_theta_section_max=sim_data.epsilon_theta_section_max
# epsilon_theta_section_mean=sim_data.epsilon_theta_section_mean
# epsilon_theta_section_median=sim_data.epsilon_theta_section_median

# average_epsilon=sim_data.average_epsilon
# max_epsilon=sim_data.max_epsilon
# median_epsilon=sim_data.median_epsilon


# #plt.close("all")
# # %%
# name="searching_square_bar_plot_max"
# c="tab:red"
# labels=['0',
#         r'$\frac{\pi}{6}$',
#         r'$\frac{\pi}{3}$',
#         r'$\frac{\pi}{2}$',
#         r'$\frac{2\pi}{3}$',
#         r'$\frac{5\pi}{6}$',
#         r'$\pi$',
#         r'$\frac{7\pi}{6}$',
#         r'$\frac{8\pi}{6}$',
#         r'$\frac{3\pi}{2}$',
#         r'$\frac{10\pi}{6}$',
#         r'$\frac{11\pi}{6}$']

# fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
# y=[]
# ymean=[]
# for i in range(len(epsilon_theta_section_max)):
#     entry=int(str(i))
#     y_=epsilon_theta_section_max[str(i)]
#     res2=np.nonzero(y_)
#     res2=res2[0]
#     y__=[]
#     #print(y_)
#     for j in res2:
#         y__.append(y_[j])
#     y.append(y__)
#     x=entry*np.ones(len(y__))
#     axs2.scatter(x,y__,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
#     error=[np.min(y__),np.max(y__)]
#     axs2.plot([x[1],x[1]],error,color="tab:green",linewidth=1,zorder=3)
#     axs2.scatter(entry,np.mean(y__),color=c,marker='s',s=3,zorder=3)
#     ymean.append(np.mean(y__))
# positions=[0,1,2,3,4,5,6,7,8,9,10,11]
# axs2.set_xticks(positions)
# axs2.set_xticklabels(labels,color='k',fontsize=8)
# print(np.max(ymean)-np.min(ymean))
# axs2.bar(labels,ymean,color="tab:blue",zorder=3)
# axs2.set_ylabel('$\epsilon$',labelpad=-1)
# axs2.set_xlabel('angle',labelpad=-2)
# axs2.set_title(r'$(b)$'+"max")
# axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")


# #plt.close("all")