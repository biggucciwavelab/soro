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
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.linewidth'] = .1
path="D:/dmulroy/Experiments/square_pull_rotate2/"
d=2


name = ["25_04_2023_23_42_52",
        "25_04_2023_23_43_01",
        "25_04_2023_23_43_13",
        "25_04_2023_23_43_21",
        "25_04_2023_23_43_33",
        "25_04_2023_23_43_41",
        "25_04_2023_23_43_50",
        "25_04_2023_23_43_58",
        "25_04_2023_23_44_07",
        "25_04_2023_23_44_17",
        "25_04_2023_23_45_28",
        "25_04_2023_23_45_59",
        "25_04_2023_23_46_08",
        "25_04_2023_23_47_28",
        "25_04_2023_22_57_45",
        "25_04_2023_23_48_04",
        "26_04_2023_01_35_54",
        "26_04_2023_01_36_01",
        "26_04_2023_01_36_09",
        "26_04_2023_01_36_17",
        "26_04_2023_01_36_24",
        "26_04_2023_01_36_31",
        "26_04_2023_01_36_40",
        "26_04_2023_01_36_48",
        "26_04_2023_01_36_58",
        #"26_04_2023_02_04_59",
        "26_04_2023_02_04_12",
        "26_04_2023_02_04_18",
        "26_04_2023_02_04_24",
        "26_04_2023_02_04_31",
        "26_04_2023_02_04_37",
        "26_04_2023_02_04_43",
        "26_04_2023_02_05_08",
        "26_04_2023_02_05_15",
        "26_04_2023_02_05_22",
        "26_04_2023_02_05_30"]


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
    print("set: "+str(i+1)+" of "+str(len(name)))
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

#%%
tcut=[]
fcut=[]
for i in tzero:
    print(i)
    if i==176:
        i=175
    F=((time_[0][i]-15)*1)
    fcut.append(F)
    tcut.append(time_[0][i])
    
print("fcut=",np.round(fcut,2))  

mean = statistics.mean(tcut)
meanf = statistics.mean(fcut)

sd = statistics.stdev(tcut)
sdf = statistics.stdev(fcut)

print("mu=",np.round(mean,2))
print("sigma=",np.round(sd,2))   

print("muf=",np.round(meanf,2))
print("sigmaf=",np.round(sdf,2))    

# %%
name="epsilon_square_sigma_25"
c="tab:green"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
#axs.fill_between(time_[0],epsilonsigmap,epsilonsigmam,color=c,alpha=0.5)
axs.fill_between(time_[0],epsilonsigmap,epsilonsigmammod,color=c,alpha=0.5)
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$")
axs.scatter(tcut,np.zeros(len(tcut)),marker='o',color='k',edgecolors='k',zorder=4)
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time (s)',labelpad=-2)
axs.set_title(r'$(b)$')
#axs.set_xticks([0,5,10,15,20,25,30,35,40])
#axs.set_yticks([0,0.5,1,1.5,2])
#axs.set_ylim([0,2.1])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=-1)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")




# %%
#tcut=[]
#fcut=[]
# for i in tzero:
#     if i==199:
#         i=198
#     F=((time_[0][i]-15)*1)
#     fcut.append(F)
#     tcut.append(time_[0][i])
    
    

# mean = statistics.mean(tcut)
# meanf = statistics.mean(fcut)

# sd = statistics.stdev(tcut)
# sdf = statistics.stdev(fcut)

# print("mu=",np.round(mean,2))
# print("sigma=",np.round(sd,2))   

# print("muf=",np.round(meanf,2))
# print("sigmaf=",np.round(sdf,2))      

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
#np.save('C:/soro/python/Pychrono/Strings/String_grasping/square_corner.npy',params)
