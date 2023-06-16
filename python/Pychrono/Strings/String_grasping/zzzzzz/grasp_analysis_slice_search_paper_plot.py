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

name = "13_01_2023_19_43_52"
#name = "13_01_2023_19_43_36"

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
epsilon_theta_section=sim_data.epsilon_theta_section
epsilon_section=sim_data.epsilon_section

max_epsilon=sim_data.max_epsilon

# %%
name="searching_circle_scatter"
name="searching_square_scatter"
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

fig1, axs1 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)


y=[]
ymean=[]
for i in range(len(epsilon_theta_section)):
    entry=int(str(i))
    y_=epsilon_theta_section[str(i)]
    res2=np.nonzero(y_)
    res2=res2[0]
    y__=[]
    print(y_)
    for j in res2:
        y__.append(y_[j])

    y.append(y__)
    x=entry*np.ones(len(y__))
    axs1.scatter(x,y__,color='b',marker='s',s=2,zorder=3)
    axs1.scatter(entry,np.mean(y__),color=c,marker='s',s=3,zorder=3)
    ymean.append(np.mean(y__))
    
# 
positions=[0,1,2,3,4,5,6,7,8,9,10,11]
axs1.set_xticks(positions)
axs1.set_xticklabels(labels,color='k',fontsize=8)
axs1.set_ylabel('$\epsilon$',labelpad=-1)
axs1.set_xlabel('angle',labelpad=-2)
axs1.set_title(r'$(b)$')
axs1.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs1.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs1.grid(True)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")

# %%
fig2, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
bp = axs2.boxplot(y)    
positions2=[1,2,3,4,5,6,7,8,9,10,11,12]
axs2.set_xticks(positions2)
axs2.set_xticklabels(labels,color='k',fontsize=8)
axs2.set_ylabel('$\epsilon$',labelpad=-1)
axs2.set_xlabel('angle',labelpad=-2)
axs2.set_title(r'$(b)$')
axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.grid(True)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".jpeg")

# %%
fig3, axs3 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300) 
axs3.bar(labels,ymean,zorder=3)
#positions2=[1,2,3,4,5,6,7,8,9,10,11,12]
#axs2.set_xticks(positions2)
#axs2.set_xticklabels(labels,color='k',fontsize=8)
axs3.set_ylabel('$\epsilon$',labelpad=-1)
axs3.set_xlabel('angle',labelpad=-2)
axs3.set_title(r'$(b)$')
axs3.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs3.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs3.grid(True)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".jpeg")

# %%
name = "13_01_2023_19_43_36"
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
epsilon_theta_section=sim_data.epsilon_theta_section
epsilon_section=sim_data.epsilon_section

max_epsilon=sim_data.max_epsilon

# %%
name="searching_circle_scatter"
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

fig1, axs1 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
y=[]
ymean=[]
for i in range(len(epsilon_theta_section)):
    entry=int(str(i))
    y_=epsilon_theta_section[str(i)]
    res2=np.nonzero(y_)
    res2=res2[0]
    y__=[]
    print(y_)
    for j in res2:
        y__.append(y_[j])

    y.append(y__)
    x=entry*np.ones(len(y__))
    axs1.scatter(x,y__,color='b',marker='s',s=2,zorder=3)
    axs1.scatter(entry,np.mean(y__),color=c,marker='s',s=3,zorder=3)
    ymean.append(np.mean(y__))
    
# %%
positions=[0,1,2,3,4,5,6,7,8,9,10,11]
axs1.set_xticks(positions)
axs1.set_xticklabels(labels,color='k',fontsize=8)
axs1.set_ylabel('$\epsilon$',labelpad=-1)
axs1.set_xlabel('angle',labelpad=-2)
axs1.set_title(r'$(a)$')
axs1.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs1.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs1.grid(True)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")

# %%
fig2, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
bp = axs2.boxplot(y)    
positions2=[1,2,3,4,5,6,7,8,9,10,11,12]
axs2.set_xticks(positions2)
axs2.set_xticklabels(labels,color='k',fontsize=8)
axs2.set_ylabel('$\epsilon$',labelpad=-1)
axs2.set_xlabel('angle',labelpad=-2)
axs2.set_title(r'$(b)$')
axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.grid(True)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_boxplot_"+".jpeg")

# %%
fig3, axs3 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300) 
axs3.bar(labels,ymean,color="tab:red",zorder=3)
#positions2=[1,2,3,4,5,6,7,8,9,10,11,12]
#axs2.set_xticks(positions2)
#axs2.set_xticklabels(labels,color='k',fontsize=8)
axs3.set_ylabel('$\epsilon$',labelpad=-1)
axs3.set_xlabel('angle',labelpad=-2)
axs3.set_title(r'$(a)$')
axs3.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs3.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs3.grid(True)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+"_barplot_"+".jpeg")
