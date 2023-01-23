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
name1 = "12_01_2023_11_15_12"
#name2 = "13_01_2023_12_51_42"
name2 = "13_01_2023_19_45_09"
d=2

# # circle noisy alpha 
# name = ["13_01_2023_22_53_21",
#         "13_01_2023_22_53_41",
#         "13_01_2023_22_53_34",
#         "13_01_2023_22_54_03",
#         "13_01_2023_22_54_11",
#         "14_01_2023_12_19_15",
#         "14_01_2023_12_19_21",
#         "14_01_2023_12_19_26",
#         "14_01_2023_12_19_32",
#         "14_01_2023_12_19_38"]

#circle noisy position
name = ["14_01_2023_15_26_35",
        "14_01_2023_15_26_42",
        "14_01_2023_15_26_49",
        "14_01_2023_15_27_04",
        "14_01_2023_15_27_11"]

# square noise position
name = ["15_01_2023_14_10_19",
        "15_01_2023_14_10_29",
        "15_01_2023_14_10_37",
        "15_01_2023_14_10_43",
        "15_01_2023_14_10_51"]



# name = ["15_01_2023_14_18_01",
#         "15_01_2023_14_16_34",
#         "15_01_2023_14_20_22",
#         "15_01_2023_14_20_31",
#         "15_01_2023_14_20_39"]

# name = ["14_01_2023_12_20_51",
#         "14_01_2023_12_20_55",
#         "14_01_2023_12_21_01",
#         "14_01_2023_12_21_06",
#         "14_01_2023_12_21_12"]



name= ["17_01_2023_15_42_45",
       "17_01_2023_15_42_53",
       "17_01_2023_15_42_58",
       "17_01_2023_15_43_06",
       "17_01_2023_15_43_11"]


name= ["17_01_2023_15_42_45",
       "17_01_2023_15_42_53",
       "17_01_2023_15_42_58",
       "17_01_2023_15_43_06",
       "17_01_2023_15_43_11",
       "17_01_2023_18_47_53",
       "17_01_2023_18_48_00",
       "17_01_2023_18_49_15",
       "17_01_2023_18_48_11",
       "17_01_2023_18_48_26",
       "17_01_2023_19_29_37",
       "17_01_2023_19_29_43",
       "17_01_2023_19_29_51",
       "17_01_2023_19_29_58",
       "17_01_2023_19_30_08",
       "18_01_2023_07_43_35",
       "18_01_2023_07_43_42",
       "18_01_2023_07_43_55",
       "18_01_2023_07_44_05",
       "18_01_2023_07_44_12",
       "18_01_2023_07_44_22",
       "18_01_2023_08_33_12",
       "18_01_2023_08_33_17",
       "18_01_2023_08_33_21",
       "18_01_2023_08_33_32",
       "18_01_2023_08_33_37",
       "18_01_2023_08_33_43",
       "18_01_2023_15_38_43",
       "18_01_2023_15_38_54",
       "18_01_2023_15_39_00",
       "18_01_2023_15_39_05",
       "18_01_2023_15_39_11"]


name= ["17_01_2023_15_42_45",
       "17_01_2023_15_42_53",
       "17_01_2023_15_42_58",
       "17_01_2023_15_43_06",
       "17_01_2023_15_43_11",
       "17_01_2023_18_47_53",
       "17_01_2023_18_48_00",
       "17_01_2023_18_49_15",
       "17_01_2023_18_48_11",
       "17_01_2023_18_48_26",
       "17_01_2023_19_29_37",
       "17_01_2023_19_29_51",
       "17_01_2023_19_29_58",
       "17_01_2023_19_30_08",
       "18_01_2023_07_43_35",
       "18_01_2023_07_43_42",
       "18_01_2023_07_43_55",
       "18_01_2023_07_44_05",
       "18_01_2023_07_44_12",
       "18_01_2023_07_44_22",
       "18_01_2023_08_33_17",
       "18_01_2023_08_33_21",
       "18_01_2023_08_33_32",
       "18_01_2023_08_33_37",
       "18_01_2023_08_33_43",
       "18_01_2023_15_38_43",
       "18_01_2023_15_38_54",
       "18_01_2023_15_39_05"]

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
name="epsilon_circle_change_interior_alpha_2p25"
c="tab:red"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
axs.fill_between(time_[0],epsilonmax,epsilonmin,color=c,alpha=0.5)
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$")
for i in range(len(epsilon_clean)):
    #for j in tzero:   
    axs.scatter(time_[i][tzero[i]],epsilon_[i][tzero[i]],marker='o',color='k',s=1,edgecolors='k',zorder=3)
    #axs.plot(time_[i],epsilon_[i],linewidth=0.15,color="tab:blue",label="$\mu$")    
axs.set_xlim([-1,30])
# xticks=[0,5,10,15,20,25,30,35]#,50,55,60]#,65,70,75,80,85,90]
# yticks=[0,1,2,3]
# axs.set_xticks(np.round(xticks,2))
# axs.set_yticks(np.round(yticks,2))
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(a)$')
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
c="tab:red"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
for i in range(len(epsilon_clean)):
    axs.plot(time_[i][nn-1:],epsilon_clean[i],linewidth=0.5,color=c)
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(a)$')
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
name="epsilon_circle_change_interior_alpha_2p25"
c="tab:red"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
#axs.fill_between(time_[0],epsilonmax,epsilonmin,color=c,alpha=0.5)
#axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$")
for i in range(len(epsilon_clean)):
    #for j in tzero:   
    axs.plot(time_[i],epsilon_[i],color="tab:red",linewidth=0.5,zorder=3)
    #axs.plot(time_[i],epsilon_[i],linewidth=0.15,color="tab:blue",label="$\mu$")    
axs.set_xlim([-1,30])
# xticks=[0,5,10,15,20,25,30,35]#,50,55,60]#,65,70,75,80,85,90]
# yticks=[0,1,2,3]
# axs.set_xticks(np.round(xticks,2))
# axs.set_yticks(np.round(yticks,2))
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(a)$')
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
#name="epsilon_square_noisy_shapes_alpha_2p25"
c="tab:red"
nn=10
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
axs.fill_between(time_[0][nn-1:],epsilonmax2,epsilonmin2,color=c,alpha=0.5)
axs.plot(time_[0][nn-1:],epsilonmu2,linewidth=2,color=c,label="$\mu$")

axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.set_title(r'$(a)$')
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True)

# fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
# for i in range(len(epsilon_clean)):
#     axs.plot(time_[i][nn-1:],epsilon_clean[i],linewidth=2,color=c)

# axs.set_ylabel('$\epsilon$',labelpad=-1)
# axs.set_xlabel('time [s]',labelpad=-2)
# axs.set_title(r'$(b)$')
# axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.grid(True)
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # # %% In[epsilon_three_shapes_legend]
# # #entry1_=[0,30,75,100,120,125]
# #%%
# #name="epsilon_square_noisy_shapes_alpha_2p25"
# c="tab:red"
# nn=10
# fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.5,2),dpi=300)
# axs.fill_between(time_[0][nn-1:],epsilonmax2,epsilonmin2,color=c,alpha=0.5)
# axs.plot(time_[0][nn-1:],epsilonmu2,linewidth=2,color=c,label="$\mu$")
# axs.set_ylabel('$\epsilon$',labelpad=-1)
# axs.set_xlabel('time [s]',labelpad=-2)
# axs.set_title(r'$(a)$')
# axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.grid(True)
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # # %% In[epsilon_three_shapes_legend]
# # #entry1_=[0,30,75,100,120,125]
