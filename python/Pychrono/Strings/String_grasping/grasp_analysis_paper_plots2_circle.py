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
#import objects2 as sim_obj
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
path="D:/dmulroy/Experiments/circle_pull2/"

plt.close('all')

name=["25_04_2023_16_50_39",
      "25_04_2023_16_50_46",
      "25_04_2023_16_50_57",
      "25_04_2023_16_51_09",
      "25_04_2023_16_51_16",
      "25_04_2023_16_51_26",
      "25_04_2023_16_51_32",
      "25_04_2023_16_51_38",
      "25_04_2023_16_51_45",
      "25_04_2023_18_10_04",
      "25_04_2023_18_10_13",
      "25_04_2023_18_10_21",
      "25_04_2023_18_10_28",
      "25_04_2023_18_10_37",
      "25_04_2023_18_10_45",
      "25_04_2023_18_10_53",
      "25_04_2023_18_11_03",
      "25_04_2023_18_11_03",
      "25_04_2023_18_11_27",
      "25_04_2023_22_28_30",
      "25_04_2023_22_28_45",
      "25_04_2023_22_28_53",
      "25_04_2023_22_29_00",
      "25_04_2023_22_29_22",
      "25_04_2023_22_29_43",
      "25_04_2023_22_29_36",
      "25_04_2023_22_29_29",
      "25_04_2023_22_29_10",
      "25_04_2023_22_28_38",
      "25_04_2023_23_04_38",
      "25_04_2023_23_04_47",
      "25_04_2023_23_04_56",
      "25_04_2023_23_05_03",
      "25_04_2023_23_05_14",
      "25_04_2023_23_05_37",
      "25_04_2023_23_05_45",
      "25_04_2023_23_05_55",
      "25_04_2023_23_06_08",
      "26_04_2023_02_35_04",
      "26_04_2023_02_35_12",
      "26_04_2023_02_35_18",
      "26_04_2023_02_35_29",
      "26_04_2023_02_35_37",
      "26_04_2023_02_35_45",
      "26_04_2023_02_35_51",
      "26_04_2023_02_35_58",
      "26_04_2023_02_36_06",
      "26_04_2023_02_36_18",
      "26_04_2023_02_37_10",
      "26_04_2023_02_37_18",
      "26_04_2023_02_37_36"]

entry=[ 1,  2,  4,  7,  8,  9, 10, 11, 15, 16, 17, 18, 19, 20, 21, 22, 23,
        25, 26, 27, 30, 31, 33, 34, 36, 37, 38, 39, 41, 43, 45, 49]
# entry=[1,2,4,7,8,9,10,11,14,15,16,17,18,19,20,21,22,23,25,26,27,30,31,33,34,36,37,38,39,41,43,45,48,49,50]
name2=[]
for i in entry:
#      #print(i)
    name2.append(name[i])
# print(len(name))
# print(len(name2))

entry2=[ 0,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17,
        18, 19, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31]

name3=[]
for i in entry2:
#      #print(i)
    name3.append(name2[i])
# %%
snap_shot=False
membrane=True
d=3.5
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
for i in range(len(name3)):
    print("set: "+str(i+1)+" of "+str(len(name3)))
    Psi.append(None)
    temp=sim_obj.import_data(name3[i],path,dxmin,dxmax,dymin,dymax,Psi[i])
    sim_data.append(temp)
    epsilon_.append(temp.EPSILON_)
    time_.append(temp.time)
    epsilon_clean.append(moving_average(temp.EPSILON_, n=nn))
    #if i==0:
    #    temp.create_frames_pull_epsilon3(True,dxmin,dxmax,dymin,dymax)
    #else:
    #    print("none")
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
        #print(j)
        #print(len(epsilon_[j]))
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
    if i==175:
        i=174
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


#  pull plots %%
name="epsilon_circle_sigma_alpha_25"
c="tab:red"
mu_circle=epsilonmu
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
#axs.fill_between(time_[0],epsilonsigmap,epsilonsigmam,color=c,alpha=0.5)
#print("issue")
axs.fill_between(time_[0],epsilonsigmap,epsilonsigmammod,color=c,alpha=0.5,zorder=3)
#print("issue")
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$",zorder=3) 
#axs.scatter(tcut,np.zeros(len(tcut)),marker='o',color='k',edgecolors='k',zorder=4)

axs.set_xlim([-1,35])
axs.set_ylim([0,2.1])
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('Time (s)',labelpad=-1)
axs.set_title('(a)')
axs.set_xticks([0,5,10,15,20,25,30,35])
axs.set_yticks([0,0.5,1,1.5,2.0])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=-4)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")

#%%
    

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

