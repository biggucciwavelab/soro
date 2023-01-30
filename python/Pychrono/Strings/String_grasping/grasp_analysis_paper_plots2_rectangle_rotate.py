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
def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

path="D:/dmulroy/Experiments/square_pull_rotate/"
d=2

name = ["27_01_2023_09_24_23",
        "27_01_2023_09_50_14",
        "27_01_2023_09_50_20",
        "27_01_2023_09_50_25",
        "27_01_2023_09_50_32",
        "27_01_2023_09_50_40",
        "27_01_2023_09_50_54",
        "27_01_2023_13_36_56",
        "27_01_2023_13_37_02",
        "27_01_2023_13_37_09",
        "27_01_2023_13_37_15",
        "27_01_2023_13_37_23",
        "27_01_2023_14_48_20",
        "27_01_2023_14_48_25",
        "27_01_2023_14_48_29",
        "27_01_2023_14_48_36",
        "27_01_2023_14_48_44",
        "27_01_2023_15_15_22",
        "27_01_2023_15_15_28",
        "27_01_2023_15_15_36",
        "27_01_2023_15_15_55",
        "27_01_2023_15_16_07",
        "27_01_2023_15_45_18",
        "27_01_2023_15_45_24",
        "27_01_2023_15_45_29",
        "27_01_2023_15_45_40",
        "27_01_2023_15_45_48",
        "27_01_2023_16_43_16",
        "27_01_2023_16_43_22",
        "27_01_2023_16_43_27",
        "27_01_2023_16_43_31",
        "27_01_2023_16_43_39"]
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
    temp=sim_obj.import_data(name[i],path,dxmin,dxmax,dymin,dymax,None)
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
name="epsilon_square_sigma_2p25"
c="tab:green"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
#axs.fill_between(time_[0],epsilonsigmap,epsilonsigmam,color=c,alpha=0.5)
axs.fill_between(time_[0],epsilonsigmap,epsilonsigmammod,color=c,alpha=0.5)
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$")
axs.set_ylabel('$\epsilon$',labelpad=-1,fontsize=9)
axs.set_xlabel('time (s)',labelpad=-2,fontsize=9)
axs.set_title(r'$(c)$',fontsize=9)
#axs.set_xticks([0,5,10,15,20,25,30,35,40])
#axs.set_yticks([0,0.5,1,1.5,2])
#axs.set_ylim([0,2.1])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=-1)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")




# %%
tcut=[]
fcut=[]
for i in tzero:
    if i==199:
        i=198
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
np.save('C:/soro/python/Pychrono/Strings/String_grasping/square_corner.npy',params)
