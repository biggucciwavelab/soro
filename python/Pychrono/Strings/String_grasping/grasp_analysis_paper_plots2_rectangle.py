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
d=3.5
#path="D:/dmulroy/Experiments/square_pull/"
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
       #"18_01_2023_10_00_42",
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
       #"30_01_2023_11_38_45",
       "30_01_2023_11_39_12"]


name = ["25_04_2023_16_48_46",
        "25_04_2023_16_48_54",
        "25_04_2023_16_49_00",
        #"25_04_2023_16_49_08",
        "25_04_2023_16_49_15",
        "25_04_2023_16_49_24",
        "25_04_2023_16_49_36",
        "25_04_2023_16_49_42",
        "25_04_2023_17_32_05",
       # "25_04_2023_17_32_11",
       # "25_04_2023_17_32_17",
        "25_04_2023_17_32_23",
        "25_04_2023_17_32_29",
        "25_04_2023_17_32_36",
        "25_04_2023_17_32_41",
        "25_04_2023_17_32_52",
        "25_04_2023_17_33_00",
        "25_04_2023_17_33_06",
        "25_04_2023_17_33_14",
        "25_04_2023_17_33_22",
        "25_04_2023_19_34_30",
        "25_04_2023_19_35_07",
        "25_04_2023_19_34_47",
        "25_04_2023_19_39_06",
        #"25_04_2023_19_39_15",
        "25_04_2023_19_39_23",
        "25_04_2023_19_39_35",
        "25_04_2023_19_39_50",
        "25_04_2023_19_40_27",
        "25_04_2023_19_40_43",
        "25_04_2023_20_18_46",
        "25_04_2023_20_18_53",
        "25_04_2023_20_19_00",
        "25_04_2023_20_19_11",
        "25_04_2023_20_19_19",
        "25_04_2023_20_19_27",
        "25_04_2023_20_19_38",
        #"25_04_2023_20_19_47",
        "25_04_2023_20_19_55",
        "25_04_2023_20_20_17",
        "25_04_2023_20_21_50"]


name = ["25_04_2023_16_48_46",
        "25_04_2023_16_48_54",
        "25_04_2023_16_49_00",
        "25_04_2023_16_49_15",
        "25_04_2023_16_49_24",
        "25_04_2023_16_49_36",
        "25_04_2023_16_49_42",
        "25_04_2023_17_32_05",
        "25_04_2023_17_32_23",
        "25_04_2023_17_32_29",
        "25_04_2023_17_32_36",
        "25_04_2023_17_32_41",
        "25_04_2023_17_33_00",
        "25_04_2023_17_33_06",
        "25_04_2023_17_33_14",
        "25_04_2023_17_33_22",
        "25_04_2023_19_34_30",
        "25_04_2023_19_35_07",
        "25_04_2023_19_34_47",
        "25_04_2023_19_39_06",
        "25_04_2023_19_39_15",
        "25_04_2023_19_39_23",
        "25_04_2023_19_39_35",
        "25_04_2023_20_18_46",
        "25_04_2023_20_19_00",
        "25_04_2023_20_19_19",
        "25_04_2023_20_19_27",
        "25_04_2023_20_19_55",
        "25_04_2023_20_20_17",
        "25_04_2023_20_21_50",
        "25_04_2023_22_57_24",
        "25_04_2023_22_57_31",
        "25_04_2023_22_57_38",
        "25_04_2023_22_57_45",
        "25_04_2023_22_57_54",
        "25_04_2023_22_58_08",
        "25_04_2023_22_58_16",
        "25_04_2023_22_58_23",
        "25_04_2023_22_58_53",
        "25_04_2023_22_59_05",
        "26_04_2023_01_32_56",
        "26_04_2023_01_31_23",
        "26_04_2023_01_31_34",
        "26_04_2023_01_31_56",
        "26_04_2023_01_32_11",
        "26_04_2023_09_03_13",
        "26_04_2023_09_03_22",
        "26_04_2023_09_03_30",
        "26_04_2023_09_03_36",
        "26_04_2023_09_03_44",
        "26_04_2023_09_03_51",
        "26_04_2023_09_04_00",
        "26_04_2023_09_04_07",
        "26_04_2023_09_04_24",
        "26_04_2023_09_04_42",
        "26_04_2023_09_05_08",
        "26_04_2023_09_05_37",
        "26_04_2023_09_05_30"]
        


plt.close('all')
entry=[ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16,
        19, 21, 23, 24, 25, 26, 27, 28, 29, 32, 33, 36, 40, 42, 46, 49, 52,
        53, 55, 57]
# entry=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,31,32,33,35,37,38,39,40,41,42,43,45,46,47,48,49,50,51,52,53,54,56]
name2=[]
for i in entry:
     name2.append(name[i])
     
     
# entry2=[ 0,  2,  4,  5,  6,  8, 10, 11, 12, 14, 15, 16, 17, 18, 20, 21, 23,
#         24, 25, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36]     

# name3=[]
# for i in entry2:
#      print(i)
#      name3.append(name2[i]) 
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
for i in range(len(name2)):
    print("set: "+str(i+1)+" of "+str(len(name2)))
    #Psi.append(sim_obj.R_functions(name[i]))
    Psi.append(None)
    temp=sim_obj.import_data(name2[i],path,dxmin,dxmax,dymin,dymax,Psi[i])
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
tcut=[]
fcut=[]
for i in tzero:
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
mu_square=epsilonmu
name="epsilon_square_sigma_2p25"
c="tab:blue"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
#axs.fill_between(time_[0],epsilonsigmap,epsilonsigmam,color=c,alpha=0.5)
axs.fill_between(time_[0],epsilonsigmap,epsilonsigmammod,color=c,alpha=0.5)
axs.plot(time_[0],epsilonmu,linewidth=2,color=c,label="$\mu$")
#axs.scatter(tcut,np.zeros(len(tcut)),marker='o',color='k',edgecolors='k',zorder=4)
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('Time (s)',labelpad=-2)
axs.set_title('(b)')
axs.set_xticks([0,5,10,15,20,25,30,35])
axs.set_yticks([0,0.5,1,1.5,2])
axs.set_ylim([0,2.1])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=-4)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")




# %%
# tcut=[]
# fcut=[]
# for i in tzero:
#     if i==176:
#         i=175
#     F=((time_[0][i]-15)*1)
#     fcut.append(F)
#     tcut.append(time_[0][i])
    
    
# print("fcut=",np.round(fcut,2))
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

#np.save('C:/soro/python/Pychrono/Strings/String_grasping/square_surface.npy',params)