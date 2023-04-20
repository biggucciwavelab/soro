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
path="D:/dmulroy/Experiments/circle_pull/"

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
       "25_01_2023_10_41_25",
       "25_01_2023_10_41_31",
       "25_01_2023_10_41_37"]
# %%
snap_shot=False
membrane=True
d=2
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
name="epsilon_circle_sigma_alpha_2p25"
c="tab:red"
mu_circle=epsilonmu
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
#axs.fill_between(time_[0],epsilonsigmap,epsilonsigmam,color=c,alpha=0.5)
axs.fill_between(time_[0],epsilonsigmap,epsilonsigmammod,color=c,alpha=0.5,zorder=3)
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$",zorder=3) 
axs.set_xlim([-1,35])
axs.set_ylim([0,2.1])
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('Time (s)',labelpad=-2)
axs.set_title('(a)')
axs.set_xticks([0,5,10,15,20,25,30,35])
axs.set_yticks([0,0.5,1,1.5,2])
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
params['fcut']=fcut
params['tcut']=tcut
np.save('C:/soro/python/Pychrono/Strings/String_grasping/circle.npy',params)

   
#     tcut.append(time_[0][i])
# fig, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.75),dpi=300)
# positions=np.arange(0,len(tzero),1)
# #axs2.set_xticks(positions)
# #axs2.set_xticklabels(labels,color='k',fontsize=8)
# axs2.bar(positions,tcut,color="tab:green",zorder=3)
# yticks=[0,5,10,15,20,25,30,35]#,50,55,60]#,65,70,75,80,85,90]
# axs2.set_yticks(np.round(yticks,2))


# plt.close('all')

