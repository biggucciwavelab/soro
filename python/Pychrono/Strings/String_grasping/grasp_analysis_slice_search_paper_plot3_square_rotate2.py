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
import itertools 
path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)


labels=['0',
        r'$\frac{\pi}{4}$',
        r'$\frac{\pi}{2}$',
        r'$\frac{3\pi}{4}$',
        r'$\pi$',
        r'$\frac{5\pi}{4}$',
        r'$\frac{3\pi}{2}$',
        r'$\frac{7\pi}{4}$']
# %%
d=4.5
count_range=8
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d
name1 = "21_01_2023_16_45_00"

Psi=sim_obj.R_functions(name1)  
sim_data1=sim_obj.select_import_data(name1,path,dxmin,dxmax,dymin,dymax,Psi)

EPSILON_=sim_data1.EPSILON_
THETA=sim_data1.THETA       
sim_data1.sort_epsilon_and_theta(count_range)
epsilon_section1=sim_data1.epsilon_section
epsilon_theta_section_max1=sim_data1.epsilon_theta_section_max
epsilon_theta_section_mean1=sim_data1.epsilon_theta_section_mean
epsilon_theta_section_median1=sim_data1.epsilon_theta_section_median

average_epsilon1=sim_data1.average_epsilon
max_epsilon1=sim_data1.max_epsilon
median_epsilon1=sim_data1.median_epsilon

# %%
name2 = "22_01_2023_13_50_03"
Psi=sim_obj.R_functions(name2)  
sim_data2=sim_obj.select_import_data(name2,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data2.sort_epsilon_and_theta(count_range)
epsilon_section2=sim_data1.epsilon_section
epsilon_theta_section_max2=sim_data2.epsilon_theta_section_max
epsilon_theta_section_mean2=sim_data2.epsilon_theta_section_mean
epsilon_theta_section_median2=sim_data2.epsilon_theta_section_median

average_epsilon2=sim_data2.average_epsilon
max_epsilon2=sim_data2.max_epsilon
median_epsilon2=sim_data2.median_epsilon


# %%
name3 = "22_01_2023_13_50_10"
Psi=sim_obj.R_functions(name3)  
sim_data3=sim_obj.select_import_data(name3,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data3.sort_epsilon_and_theta(count_range)
epsilon_section3=sim_data3.epsilon_section
epsilon_theta_section_max3=sim_data3.epsilon_theta_section_max
epsilon_theta_section_mean3=sim_data3.epsilon_theta_section_mean
epsilon_theta_section_median3=sim_data3.epsilon_theta_section_median

average_epsilon3=sim_data3.average_epsilon
max_epsilon3=sim_data3.max_epsilon
median_epsilon3=sim_data3.median_epsilon


#%%
name4 = "22_01_2023_13_50_18"
Psi=sim_obj.R_functions(name4)  
sim_data4=sim_obj.select_import_data(name4,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data4.sort_epsilon_and_theta(count_range)
epsilon_section4=sim_data4.epsilon_section
epsilon_theta_section_max4=sim_data4.epsilon_theta_section_max
epsilon_theta_section_mean4=sim_data4.epsilon_theta_section_mean
epsilon_theta_section_median4=sim_data4.epsilon_theta_section_median

average_epsilon4=sim_data4.average_epsilon
max_epsilon4=sim_data4.max_epsilon
median_epsilon4=sim_data4.median_epsilon

name5 = "23_01_2023_08_43_42"
name6 = "23_01_2023_08_46_57"
name7 = "23_01_2023_08_47_04"
name8 = "23_01_2023_08_47_25"
name9 = "23_01_2023_08_49_03"
#%%
name5 = "22_01_2023_13_50_18"
Psi=sim_obj.R_functions(name5)  
sim_data5=sim_obj.select_import_data(name5,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data5.sort_epsilon_and_theta(count_range)
epsilon_section5=sim_data5.epsilon_section
epsilon_theta_section_max5=sim_data5.epsilon_theta_section_max
epsilon_theta_section_mean5=sim_data5.epsilon_theta_section_mean
epsilon_theta_section_median5=sim_data5.epsilon_theta_section_median

average_epsilon5=sim_data5.average_epsilon
max_epsilon5=sim_data5.max_epsilon
median_epsilon5=sim_data5.median_epsilon

#%%
name6 = "23_01_2023_08_46_57"
Psi=sim_obj.R_functions(name6)  
sim_data6=sim_obj.select_import_data(name6,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data6.sort_epsilon_and_theta(count_range)
epsilon_section6=sim_data6.epsilon_section
epsilon_theta_section_max6=sim_data6.epsilon_theta_section_max
epsilon_theta_section_mean6=sim_data6.epsilon_theta_section_mean
epsilon_theta_section_median6=sim_data6.epsilon_theta_section_median

average_epsilon6=sim_data6.average_epsilon
max_epsilon6=sim_data6.max_epsilon
median_epsilon6=sim_data6.median_epsilon


#%%
name7 = "23_01_2023_08_47_04"
Psi=sim_obj.R_functions(name7)  
sim_data7=sim_obj.select_import_data(name7,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data7.sort_epsilon_and_theta(count_range)
epsilon_section7=sim_data7.epsilon_section
epsilon_theta_section_max7=sim_data7.epsilon_theta_section_max
epsilon_theta_section_mean7=sim_data7.epsilon_theta_section_mean
epsilon_theta_section_median7=sim_data7.epsilon_theta_section_median

average_epsilon7=sim_data7.average_epsilon
max_epsilon7=sim_data7.max_epsilon
median_epsilon7=sim_data7.median_epsilon

#%%
name8 = "23_01_2023_08_47_25"
Psi=sim_obj.R_functions(name8)  
sim_data8=sim_obj.select_import_data(name8,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data8.sort_epsilon_and_theta(count_range)
epsilon_section8=sim_data8.epsilon_section
epsilon_theta_section_max8=sim_data8.epsilon_theta_section_max
epsilon_theta_section_mean8=sim_data8.epsilon_theta_section_mean
epsilon_theta_section_median8=sim_data8.epsilon_theta_section_median

average_epsilon8=sim_data8.average_epsilon
max_epsilon8=sim_data8.max_epsilon
median_epsilon8=sim_data8.median_epsilon


#%%
name9 = "23_01_2023_08_49_03"
Psi=sim_obj.R_functions(name9)  
sim_data9=sim_obj.select_import_data(name9,path,dxmin,dxmax,dymin,dymax,Psi)
sim_data9.sort_epsilon_and_theta(count_range)
epsilon_section9=sim_data9.epsilon_section
epsilon_theta_section_max9=sim_data9.epsilon_theta_section_max
epsilon_theta_section_mean9=sim_data9.epsilon_theta_section_mean
epsilon_theta_section_median9=sim_data9.epsilon_theta_section_median

average_epsilon9=sim_data9.average_epsilon
max_epsilon9=sim_data9.max_epsilon
median_epsilon9=sim_data9.median_epsilon

# %%
max_epsilon_={}
for i in range(count_range):
    max_epsilon_[str(i)]=[]
    temp1=np.asarray(epsilon_theta_section_max1[str(i)])
    temp2=np.asarray(epsilon_theta_section_max2[str(i)])
    temp3=np.asarray(epsilon_theta_section_max3[str(i)])
    temp4=np.asarray(epsilon_theta_section_max4[str(i)])
    temp5=np.asarray(epsilon_theta_section_max5[str(i)]) 
    temp6=np.asarray(epsilon_theta_section_max6[str(i)])  
    temp7=np.asarray(epsilon_theta_section_max7[str(i)]) 
    temp8=np.asarray(epsilon_theta_section_max8[str(i)]) 
    temp9=np.asarray(epsilon_theta_section_max9[str(i)]) 
    #temp3=[temp,temp2]
    temp=np.concatenate((temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8,temp9))
    #temp3=temp3.flatten()
    max_epsilon_[str(i)].append(temp)
    #max_epsilon_[str(i)].append(epsilon_theta_section_max1[str(i)])


c="tab:red"
name="searching_circle_bar_plot_max"
fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,2),dpi=300)
y=[]
ymean=[]
for i in range(len(max_epsilon_)):
    entry=int(str(i))
    y_=max_epsilon_[str(i)][0]
    res2=np.nonzero(y_)
    res2=res2[0]
    y__=[]
    #print(y_)
    for j in res2:
        if y_[j]<5:
            y__.append(y_[j])
    y.append(y__)
    x=entry*np.ones(len(y__))
    axs2.scatter(x,y__,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
    error=[np.min(y__),np.max(y__)]
    axs2.plot([x[1],x[1]],error,color="tab:green",linewidth=1,zorder=3)
    axs2.scatter(entry,np.mean(y__),color=c,marker='s',s=3,zorder=3)
    ymean.append(np.mean(y__))
#positions=[0,1,2,3,4,5,6,7,8,9,10,11]
positions=[0,1,2,3,4,5,6,7]
axs2.set_xticks(positions)
#axs2.set_yticks([0,1,2,3])
axs2.set_xticklabels(labels,color='k',fontsize=8)
axs2.bar(labels,ymean,color="tab:red",zorder=3)
print(np.max(ymean)-np.min(ymean))
axs2.set_ylabel('$\epsilon$',labelpad=-1)
axs2.set_xlabel('angle',labelpad=-2)
axs2.set_title(r'$(a)$'+"max")
axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")