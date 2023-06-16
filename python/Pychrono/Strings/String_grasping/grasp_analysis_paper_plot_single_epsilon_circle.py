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

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.linewidth'] = .1
path = os.path.dirname(__file__)
path=path+"/Experiments/"
#os.chdir(path)
path="D:/dmulroy/Experiments/circle_pull2/"
def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


d=2

snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d
name1 = "17_01_2023_18_49_15"
name1 = '25_04_2023_18_10_21'
#Psi=sim_obj.R_functions(name1)  
sim_data1=sim_obj.import_data(name1,path,dxmin,dxmax,dymin,dymax,None)
#sim_data1.create_frames_pull_epsilon2(True,d)

# %% In[extract data]
epsilon1=sim_data1.EPSILON_
time1=sim_data1.time
PX1=sim_data1.PX
FB1=sim_data1.FB
nn=10
#entry1_=244
epsilon1_clean = moving_average(epsilon1, n=nn)
#epsilon1_clean = gaussian_filter1d(epsilon1, 10)

tzero=[]
#for i in range(len(epsilon1_clean)):
entry=0
count=0
while entry==0:
    entry=epsilon1[len(epsilon1)-1-count] 
    count=count+1
tzero=len(epsilon1)-count+1
tcut=epsilon1[tzero+1]
#tzero.append(len(epsilon_[i])-count+1)
entry=[0,50,75,100,125,tzero+1,140,145]
entry=[0,50,75,100,125,tzero+1,139,140,141,142,143,144,145]
# %% In[epsilon_three_shapes_legend]
name="epsilon_single_shape"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(7.5,1.5),dpi=300)
axs.plot(time1,epsilon1,color='tab:red',linewidth=2,alpha=0.5)
axs.plot(time1[nn-1:],epsilon1_clean,color='tab:red',linewidth=2,label='Circle')
for i in entry:
    axs.scatter(time1[i],epsilon1[i],marker='o',color='tab:red',edgecolors='k',zorder=3)
axs.axvline(x = 0, color = 'k')
axs.axvline(x = 10, color = 'k')
axs.axvline(x = 15, color = 'k')
p1 = patches.FancyArrowPatch((0, 7), (6, 7), arrowstyle='<->', mutation_scale=10)
axs.add_patch(p1) 
p1 = patches.FancyArrowPatch((6, 7), (15, 7), arrowstyle='<->', mutation_scale=10)
axs.add_patch(p1) 
#xticks=[0,5,10,15,20,25,30]#,50,55,60]#,65,70,75,80,85,90]
#yticks=[0,2,4,6,8]
#axs.set_xticks(np.round(xticks,2))
#axs.set_yticks(np.round(yticks,2))
#axs.set_title('(a)')
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('Time (s)',labelpad=-2)
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=-1)

#plt.tight_layout()
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
#plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
#plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
plt.close('all')


# %%
membrane=True
directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
time_=1
entry=[0,50,75,100,125,tzero+1,139,140,141,142,143,144,145]
wxmin=-2
wxmax=4
wymin=-2
wymax=2
fxs=1.25
fys=.8325
name="circle"
for i in entry:
    sim_data1.create_frames_snapshot(name,True,membrane,directory,i,wxmin,wxmax,wymin,wymax,fxs,fys,True)





