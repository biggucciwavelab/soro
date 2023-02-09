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
import statistics

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.linewidth'] = .1
def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

#D:\dmulroy\Experiments\square_pull
d=2
path="D:/dmulroy/Experiments/square_pull/"
name= ["18_01_2023_09_21_26",
       "18_01_2023_09_21_33",
       "18_01_2023_09_21_38",
       "18_01_2023_09_21_44",
       #"18_01_2023_09_21_50",
       "18_01_2023_09_21_56",
       "18_01_2023_09_22_06",
       "18_01_2023_10_00_12",
       "18_01_2023_10_00_18",
       "18_01_2023_10_00_22",
       "18_01_2023_10_00_30",
       #"18_01_2023_10_00_36",
       "18_01_2023_10_00_42",
       "18_01_2023_13_47_12",
       "18_01_2023_13_47_19",
       "18_01_2023_13_47_24",
       "18_01_2023_13_47_30",
       #"18_01_2023_13_47_37",
       "18_01_2023_13_47_48",
       "18_01_2023_13_47_55",
       "18_01_2023_15_39_22",
       "18_01_2023_15_39_36",
       "18_01_2023_15_39_50",
       "18_01_2023_15_40_17",
       #"25_01_2023_09_14_14",
       "25_01_2023_09_14_20",
       "25_01_2023_09_14_27",
       "25_01_2023_09_14_32",
       "25_01_2023_09_50_06",
       "25_01_2023_09_50_15",
       "25_01_2023_09_50_19",
       "25_01_2023_09_50_25",
       "25_01_2023_09_50_30",
       "30_01_2023_11_38_45",
       "30_01_2023_11_39_12"]

 
# %%
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
    #Psi.append(sim_obj.R_functions(name[i]))
    Psi.append(None)
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
epsilonsigma=[]
epsilonsigmap=[]
epsilonsigmam=[]
epsilonsigmammod=[]
for i in range(len(time_[0])):
    temp=0
    tempa=[]
    for j in range(len(epsilon_)):
        temp=epsilon_[j][i]+temp
        tempa.append(epsilon_[j][i])
        
    epsilonmax.append(np.max(tempa))
    epsilonmin.append(np.min(tempa)) 
    mu=temp/len(epsilon_)
    epsilonmu.append(mu)
    sigma=statistics.stdev(tempa)
    epsilonsigma.append(sigma)
    epsilonsigmap.append(mu+sigma)
    epsilonsigmam.append(mu-sigma)
    if mu-sigma<0:
        epsilonsigmammod.append(0)
    else:
        epsilonsigmammod.append(mu-sigma)




# %%
mu_square=epsilonmu
name="epsilon_square_sigma_2p25"
c="tab:blue"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
#axs.fill_between(time_[0],epsilonsigmap,epsilonsigmam,color=c,alpha=0.5)
axs.fill_between(time_[0],epsilonsigmap,epsilonsigmammod,color=c,alpha=0.5)
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$")
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('Time (s)',labelpad=-2)
axs.set_title('(b)')
axs.set_xticks([0,5,10,15,20,25,30,35])
axs.set_yticks([0,0.5,1,1.5,2])
axs.set_ylim([0,2.1])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=-1)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")




# %%
tcut=[]
fcut=[]
for i in tzero:
    if i==176:
        i=175
    F=((time_[0][i]-15)*1)
    fcut.append(F)
    tcut.append(time_[0][i])
    
    

mean = statistics.mean(tcut)
meanf = statistics.mean(fcut)

sd = statistics.stdev(tcut)
sdf = statistics.stdev(fcut)

print("mu=",np.round(mean,2))
print("sigma=",np.round(sd,2))   

print("muf=",np.round(meanf,2))
print("sigmaf=",np.round(sdf,2))      

params={}
params["mu_"]=epsilonmu
params["time"]=time_[0]
params["epsilonsigmap"]=epsilonsigmap
params["epsilonsigmammod"]=epsilonsigmammod
params["mean"]=np.round(mean,2)
params["sigma"]=np.round(sd,2)
params["meanf"]=np.round(meanf,2)
params["sigmaf"]=np.round(sdf,2)


np.save('C:/soro/python/Pychrono/Strings/String_grasping/square_surface.npy',params)