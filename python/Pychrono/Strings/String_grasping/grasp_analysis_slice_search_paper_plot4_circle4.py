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
from scipy.stats import norm
import statistics
from matplotlib.ticker import PercentFormatter

#fm._rebuild()
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.linewidth'] = .1
path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
count_range=12
plt.close('all')
path="D:/dmulroy/Experiments/circle_search/"
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

name1= "18_01_2023_10_03_51" # reverse
name2= "18_01_2023_10_02_41"

# %%
d=4.5
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d
name1= "18_01_2023_10_03_51" # reverse
#Psi=sim_obj.R_functions(name1)  
sim_data1=sim_obj.select_import_data(name1,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data1.EPSILON_
THETA=sim_data1.THETA
temp=2.1*np.pi
for i in range(len(THETA)):
    if THETA[i]==0:
        pass
    else:
        temp=abs(2*np.pi-THETA[i])
        while temp>2*np.pi:
            temp=abs(2*np.pi-temp)
        if temp==2*np.pi:
            temp=0
        else:
            temp=temp
           
        THETA[i]=temp

for i in range(len(THETA)):
    if THETA[i]!=0:
        THETA[i]=abs(THETA[i]-2*np.pi)
        
sim_data1.THETA=THETA        
sim_data1.sort_epsilon_and_theta(count_range)

epsilon_section1=sim_data1.epsilon_section
epsilon_theta_section_max1=sim_data1.epsilon_theta_section_max
epsilon_theta_section_mean1=sim_data1.epsilon_theta_section_mean
epsilon_theta_section_median1=sim_data1.epsilon_theta_section_median

average_epsilon1=sim_data1.average_epsilon
max_epsilon1=sim_data1.max_epsilon
median_epsilon1=sim_data1.median_epsilon

# %%
name2= "18_01_2023_10_02_41"
#Psi=sim_obj.R_functions(name2)  
sim_data2=sim_obj.select_import_data(name2,path,dxmin,dxmax,dymin,dymax,None)
sim_data2.sort_epsilon_and_theta(count_range)
epsilon_section2=sim_data1.epsilon_section
epsilon_theta_section_max2=sim_data2.epsilon_theta_section_max
epsilon_theta_section_mean2=sim_data2.epsilon_theta_section_mean
epsilon_theta_section_median2=sim_data2.epsilon_theta_section_median

average_epsilon2=sim_data2.average_epsilon
max_epsilon2=sim_data2.max_epsilon
median_epsilon2=sim_data2.median_epsilon


# %%
name3 = "14_01_2023_15_35_36"
#Psi=sim_obj.R_functions(name3)  
sim_data3=sim_obj.select_import_data(name3,path,dxmin,dxmax,dymin,dymax,None)
sim_data3.sort_epsilon_and_theta(count_range)
epsilon_section3=sim_data3.epsilon_section
epsilon_theta_section_max3=sim_data3.epsilon_theta_section_max
epsilon_theta_section_mean3=sim_data3.epsilon_theta_section_mean
epsilon_theta_section_median3=sim_data3.epsilon_theta_section_median

average_epsilon3=sim_data3.average_epsilon
max_epsilon3=sim_data3.max_epsilon
median_epsilon3=sim_data3.median_epsilon

# %%

name4= "18_01_2023_14_57_33" # reverse
#Psi=sim_obj.R_functions(name4)  
sim_data4=sim_obj.select_import_data(name4,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data4.EPSILON_
THETA=sim_data4.THETA
temp=2.1*np.pi
for i in range(len(THETA)):
    if THETA[i]==0:
        pass
    else:
        temp=abs(2*np.pi-THETA[i])
        while temp>2*np.pi:
            temp=abs(2*np.pi-temp)
        if temp==2*np.pi:
            temp=0
        else:
            temp=temp
           
        THETA[i]=temp

for i in range(len(THETA)):
    if THETA[i]!=0:
        THETA[i]=abs(THETA[i]-2*np.pi)
        
sim_data4.THETA=THETA        
sim_data4.sort_epsilon_and_theta(count_range)

epsilon_section4=sim_data4.epsilon_section
epsilon_theta_section_max4=sim_data4.epsilon_theta_section_max
epsilon_theta_section_mean4=sim_data4.epsilon_theta_section_mean
epsilon_theta_section_median4=sim_data4.epsilon_theta_section_median

average_epsilon4=sim_data4.average_epsilon
max_epsilon4=sim_data4.max_epsilon
median_epsilon4=sim_data4.median_epsilon


#%%
name5= "18_01_2023_14_17_01"
#Psi=sim_obj.R_functions(name5)  
sim_data5=sim_obj.select_import_data(name5,path,dxmin,dxmax,dymin,dymax,None)
sim_data5.sort_epsilon_and_theta(count_range)
epsilon_section5=sim_data5.epsilon_section
epsilon_theta_section_max5=sim_data5.epsilon_theta_section_max
epsilon_theta_section_mean5=sim_data5.epsilon_theta_section_mean
epsilon_theta_section_median5=sim_data5.epsilon_theta_section_median

average_epsilon5=sim_data5.average_epsilon
max_epsilon5=sim_data5.max_epsilon
median_epsilon5=sim_data5.median_epsilon





#%%
name6= "18_01_2023_20_10_57"
#Psi=sim_obj.R_functions(name6)  
sim_data6=sim_obj.select_import_data(name6,path,dxmin,dxmax,dymin,dymax,None)
sim_data6.sort_epsilon_and_theta(count_range)
epsilon_section6=sim_data5.epsilon_section
epsilon_theta_section_max6=sim_data6.epsilon_theta_section_max
epsilon_theta_section_mean6=sim_data6.epsilon_theta_section_mean
epsilon_theta_section_median6=sim_data6.epsilon_theta_section_median

average_epsilon6=sim_data6.average_epsilon
max_epsilon6=sim_data6.max_epsilon
median_epsilon6=sim_data6.median_epsilon





#%%
name7= "18_01_2023_20_26_57"
#Psi=sim_obj.R_functions(name5)  
sim_data7=sim_obj.select_import_data(name7,path,dxmin,dxmax,dymin,dymax,None)
sim_data7.sort_epsilon_and_theta(count_range)
epsilon_section5=sim_data7.epsilon_section
epsilon_theta_section_max7=sim_data7.epsilon_theta_section_max
epsilon_theta_section_mean7=sim_data7.epsilon_theta_section_mean
epsilon_theta_section_median7=sim_data7.epsilon_theta_section_median

average_epsilon7=sim_data7.average_epsilon
max_epsilon7=sim_data7.max_epsilon
median_epsilon7=sim_data7.median_epsilon


#%%
name8 = "18_01_2023_20_27_28"
#Psi=sim_obj.R_functions(name8)  
sim_data8=sim_obj.select_import_data(name8,path,dxmin,dxmax,dymin,dymax,None)
sim_data8.sort_epsilon_and_theta(count_range)
epsilon_section8=sim_data8.epsilon_section
epsilon_theta_section_max8=sim_data8.epsilon_theta_section_max
epsilon_theta_section_mean8=sim_data8.epsilon_theta_section_mean
epsilon_theta_section_median8=sim_data8.epsilon_theta_section_median

average_epsilon8=sim_data8.average_epsilon
max_epsilon8=sim_data8.max_epsilon
median_epsilon8=sim_data8.median_epsilon

#%%
name9 = "18_01_2023_20_11_27"
#Psi=sim_obj.R_functions(name9)  
sim_data9=sim_obj.select_import_data(name5,path,dxmin,dxmax,dymin,dymax,None)
sim_data9.sort_epsilon_and_theta(count_range)
epsilon_section9=sim_data9.epsilon_section
epsilon_theta_section_max9=sim_data9.epsilon_theta_section_max
epsilon_theta_section_mean9=sim_data9.epsilon_theta_section_mean
epsilon_theta_section_median9=sim_data9.epsilon_theta_section_median

average_epsilon9=sim_data5.average_epsilon
max_epsilon9=sim_data5.max_epsilon
median_epsilon9=sim_data5.median_epsilon


#%%
# name10 = "18_01_2023_20_11_27"
# Psi=sim_obj.R_functions(name10)  
# sim_data10=sim_obj.select_import_data(name10,path,dxmin,dxmax,dymin,dymax,Psi)
# sim_data10.sort_epsilon_and_theta(count_range)
# epsilon_section10=sim_data10.epsilon_section
# epsilon_theta_section_max10=sim_data10.epsilon_theta_section_max
# epsilon_theta_section_mean10=sim_data10.epsilon_theta_section_mean
# epsilon_theta_section_median10=sim_data10.epsilon_theta_section_median

# average_epsilon10=sim_data10.average_epsilon
# max_epsilon10=sim_data10.max_epsilon
# median_epsilon10=sim_data10.median_epsilon


name11 = "23_01_2023_14_46_09"
name12 = "23_01_2023_14_46_17" # needs running
name13 ="23_01_2023_17_13_15"
name14 ="23_01_2023_17_14_05"


#%%
name11 = "23_01_2023_14_46_09"
#Psi=sim_obj.R_functions(name11)  
sim_data11=sim_obj.select_import_data(name11,path,dxmin,dxmax,dymin,dymax,None)
sim_data11.sort_epsilon_and_theta(count_range)
epsilon_section11=sim_data11.epsilon_section
epsilon_theta_section_max11=sim_data11.epsilon_theta_section_max
epsilon_theta_section_mean11=sim_data11.epsilon_theta_section_mean
epsilon_theta_section_median11=sim_data11.epsilon_theta_section_median

average_epsilon11=sim_data11.average_epsilon
max_epsilon11=sim_data11.max_epsilon
median_epsilon11=sim_data11.median_epsilon


#%%
name12 = "23_01_2023_14_46_17" # needs running
#Psi=sim_obj.R_functions(name12)  
sim_data12=sim_obj.select_import_data(name12,path,dxmin,dxmax,dymin,dymax,None)
sim_data12.sort_epsilon_and_theta(count_range)
epsilon_section12=sim_data12.epsilon_section
epsilon_theta_section_max12=sim_data12.epsilon_theta_section_max
epsilon_theta_section_mean12=sim_data12.epsilon_theta_section_mean
epsilon_theta_section_median12=sim_data12.epsilon_theta_section_median

average_epsilon12=sim_data12.average_epsilon
max_epsilon12=sim_data12.max_epsilon
median_epsilon12=sim_data12.median_epsilon

#%%
name13 ="23_01_2023_17_13_15"
#Psi=sim_obj.R_functions(name13)  
sim_data13=sim_obj.select_import_data(name13,path,dxmin,dxmax,dymin,dymax,None)
sim_data13.sort_epsilon_and_theta(count_range)
epsilon_section13=sim_data13.epsilon_section
epsilon_theta_section_max13=sim_data13.epsilon_theta_section_max
epsilon_theta_section_mean13=sim_data13.epsilon_theta_section_mean
epsilon_theta_section_median13=sim_data13.epsilon_theta_section_median

average_epsilon13=sim_data13.average_epsilon
max_epsilon13=sim_data13.max_epsilon
median_epsilon13=sim_data13.median_epsilon


#%%
name14 ="23_01_2023_17_14_05"
#Psi=sim_obj.R_functions(name14)  
sim_data14=sim_obj.select_import_data(name14,path,dxmin,dxmax,dymin,dymax,None)
sim_data14.sort_epsilon_and_theta(count_range)
epsilon_section14=sim_data14.epsilon_section
epsilon_theta_section_max14=sim_data14.epsilon_theta_section_max
epsilon_theta_section_mean14=sim_data14.epsilon_theta_section_mean
epsilon_theta_section_median14=sim_data14.epsilon_theta_section_median

average_epsilon14=sim_data14.average_epsilon
max_epsilon14=sim_data14.max_epsilon
median_epsilon14=sim_data14.median_epsilon


name15 = "24_01_2023_13_12_44"
name16 = "24_01_2023_13_12_49"
name17 = "24_01_2023_13_12_57"
name18 = "24_01_2023_13_14_03"

#%%
name15 = "24_01_2023_13_12_44"
#Psi=sim_obj.R_functions(name15)  
sim_data15=sim_obj.select_import_data(name15,path,dxmin,dxmax,dymin,dymax,None)
sim_data15.sort_epsilon_and_theta(count_range)
epsilon_section15=sim_data15.epsilon_section
epsilon_theta_section_max15=sim_data15.epsilon_theta_section_max
epsilon_theta_section_mean15=sim_data15.epsilon_theta_section_mean
epsilon_theta_section_median15=sim_data15.epsilon_theta_section_median

average_epsilon15=sim_data15.average_epsilon
max_epsilon15=sim_data15.max_epsilon
median_epsilon15=sim_data15.median_epsilon

#%%
#Psi=sim_obj.R_functions(name16)  
sim_data16=sim_obj.select_import_data(name16,path,dxmin,dxmax,dymin,dymax,None)
sim_data16.sort_epsilon_and_theta(count_range)
epsilon_section16=sim_data16.epsilon_section
epsilon_theta_section_max16=sim_data16.epsilon_theta_section_max
epsilon_theta_section_mean16=sim_data16.epsilon_theta_section_mean
epsilon_theta_section_median16=sim_data16.epsilon_theta_section_median

average_epsilon16=sim_data16.average_epsilon
max_epsilon16=sim_data16.max_epsilon
median_epsilon16=sim_data16.median_epsilon


#%%
#Psi=sim_obj.R_functions(name17)  
sim_data17=sim_obj.select_import_data(name17,path,dxmin,dxmax,dymin,dymax,None)
sim_data17.sort_epsilon_and_theta(count_range)
epsilon_section17=sim_data17.epsilon_section
epsilon_theta_section_max17=sim_data17.epsilon_theta_section_max
epsilon_theta_section_mean17=sim_data17.epsilon_theta_section_mean
epsilon_theta_section_median17=sim_data17.epsilon_theta_section_median

average_epsilon17=sim_data17.average_epsilon
max_epsilon17=sim_data17.max_epsilon
median_epsilon17=sim_data17.median_epsilon

#%%
#Psi=sim_obj.R_functions(name18)  
sim_data18=sim_obj.select_import_data(name18,path,dxmin,dxmax,dymin,dymax,None)
sim_data18.sort_epsilon_and_theta(count_range)
epsilon_section18=sim_data18.epsilon_section
epsilon_theta_section_max18=sim_data18.epsilon_theta_section_max
epsilon_theta_section_mean18=sim_data18.epsilon_theta_section_mean
epsilon_theta_section_median18=sim_data18.epsilon_theta_section_median

average_epsilon18=sim_data18.average_epsilon
max_epsilon18=sim_data18.max_epsilon
median_epsilon18=sim_data18.median_epsilon

# %%
max_epsilon_={}
for i in range(12):
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
    #temp10=np.asarray(epsilon_theta_section_max10[str(i)])
    temp11=np.asarray(epsilon_theta_section_max11[str(i)])   
    temp12=np.asarray(epsilon_theta_section_max12[str(i)])
    temp13=np.asarray(epsilon_theta_section_max13[str(i)])
    temp14=np.asarray(epsilon_theta_section_max14[str(i)])
    temp15=np.asarray(epsilon_theta_section_max15[str(i)])
    temp16=np.asarray(epsilon_theta_section_max16[str(i)])
    temp17=np.asarray(epsilon_theta_section_max17[str(i)])
    temp18=np.asarray(epsilon_theta_section_max18[str(i)])
    #temp3=[temp,temp2]
    temp=np.concatenate((temp1,
                         temp2,
                         temp3,
                         temp4,
                         temp5,
                         temp6,
                         temp7,
                         temp8,
                         temp9,
                         temp11,
                         temp12,
                         temp13,
                         temp14,
                         temp15,
                         temp16,
                         temp17,
                         temp18))
    #temp3=temp3.flatten()
    max_epsilon_[str(i)].append(temp)
    #max_epsilon_[str(i)].append(epsilon_theta_section_max1[str(i)])



y=[]
ymean=[]
c="tab:red"
name="searching_square_bar_plot_max"
#fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
for i in range(len(max_epsilon_)):
    entry=int(str(i))
    y_=max_epsilon_[str(i)][0]
    res2=np.nonzero(y_)
    res2=res2[0]
    #print(y_)
    #y__=[]
    #for j in res2:
    y__= y_[res2]  
        #y__.append(y_[j])
    for j in range(len(y__)):
        y.append(y__[j])
    x=entry*np.ones(len(y__))     
    mean = np.mean(y__)
    sd = statistics.stdev(y_)
    error=[mean-sd,mean+sd]
    #axs2.plot([x[1],x[1]],error,color="k",linewidth=1,zorder=3)
    #axs2.scatter(entry,np.mean(y__),color=c,marker='s',s=3,zorder=3)
    ymean.append(mean)        
# positions=[0,1,2,3,4,5,6,7,8,9,10,11]
# #positions=[0,1,2,3,4,5,6,7]
# axs2.set_xticks(positions)
# axs2.set_yticks([0,1,2,3,4,5])
# axs2.set_xticklabels(labels,color='k',fontsize=8)
# axs2.bar(labels,ymean,color="tab:blue",zorder=3)
# print(np.max(ymean)-np.min(ymean))
# axs2.set_ylabel('$\epsilon$',labelpad=-1,fontsize=9)
# axs2.set_xlabel('angle (rad)',labelpad=-2,fontsize=9)
# axs2.set_title(r'$(a)$',fontsize=9)
# axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")        

print("number of times encirlcing=",len(y_))   
# %%
ytemp1=y[0] 
c="tab:red"
name="searching_circle_guassian2"
num_bins=25
x_=np.linspace(0,5,200)  
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
#heights, bins = np.histogram(data, bins = len(list(set(data))))
axs.hist(y,num_bins, weights=np.ones(len(y)) / len(y),color="tab:red",zorder=3)
#plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.set_title('(a)')
axs.set_xlabel(r'$\epsilon$')
axs.set_yticks([0,0.05,0.1,0.15,0.2])
axs.set_xticks([0,1,2,3,4,5])
axs.grid(True,linewidth=0.1,zorder=-3)
x_=np.linspace(0,2,200)  
mean = statistics.mean(y)
sd = statistics.stdev(y)
N=norm.pdf(x_, mean, sd)
textstr = '\n'.join((
    r'$\mu=%.2f$' % (mean, ),
    r'$\sigma=%.2f$' % (sd, )))
props = dict(boxstyle='round', facecolor='white', alpha=0.5)
axs.text(0.75, 0.95, textstr, transform=axs.transAxes, fontsize=9,
        verticalalignment='top', bbox=props)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
#plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
#plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")

#num_bins=25
#x_=np.linspace(0,5,200)  

# fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
# mean = statistics.mean(y)
# sd = statistics.stdev(y)
# print(mean)
# print(sd)
# N=norm.pdf(x_, mean, sd)
# print(len(y))
# n, bins, patches = axs.hist(y,num_bins,density=True,color="tab:red",zorder=3)
# axs.plot(x_,N,color='k',zorder=3)
# #axs.axvline(x=mean,linewidth=0.5,color='k',zorder=4)
# #axs.axvline(x=mean-sd,linewidth=0.5,color='k',zorder=4)
# #axs.axvline(x=mean+sd,linewidth=0.5,color='k',zorder=4)
# textstr = '\n'.join((
#     r'$\mu=%.2f$' % (mean, ),
#     r'$\sigma=%.2f$' % (sd, )))
# props = dict(boxstyle='round', facecolor='white', alpha=0.5)
# axs.text(0.75, 0.95, textstr, transform=axs.transAxes, fontsize=9,
#         verticalalignment='top', bbox=props)
# axs.set_xticks([0,1,2,3,4,5])
# #axs.yaxis.set_major_formatter(PercentFormatter(xmax=1))
# axs.set_title('(a)')
# axs.set_xlabel(r'$\epsilon$')
# axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.grid(True,linewidth=0.1,zorder=3)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# #plt.close('all')