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
count_range=8
plt.close('all')
#path="D:/dmulroy/Experiments/circle_search/"
plt.close('all')
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

#%% 1
name1 = "30_04_2023_15_06_54"
#Psi=sim_obj.R_functions(name6)  
sim_data1=sim_obj.select_import_data(name1,path,dxmin,dxmax,dymin,dymax,None)
sim_data1.sort_epsilon_and_theta(count_range)
epsilon_section1=sim_data1.epsilon_section
epsilon_theta_section_max1=sim_data1.epsilon_theta_section_max
epsilon_theta_section_mean1=sim_data1.epsilon_theta_section_mean
epsilon_theta_section_median1=sim_data1.epsilon_theta_section_median

average_epsilon1=sim_data1.average_epsilon
max_epsilon1=sim_data1.max_epsilon
median_epsilon1=sim_data1.median_epsilon


#%% 2
name2 = "30_04_2023_15_07_00"
#Psi=sim_obj.R_functions(name6)  
sim_data2=sim_obj.select_import_data(name2,path,dxmin,dxmax,dymin,dymax,None)
sim_data2.sort_epsilon_and_theta(count_range)
epsilon_section2=sim_data2.epsilon_section
epsilon_theta_section_max2=sim_data2.epsilon_theta_section_max
epsilon_theta_section_mean2=sim_data2.epsilon_theta_section_mean
epsilon_theta_section_median2=sim_data2.epsilon_theta_section_median

average_epsilon2=sim_data2.average_epsilon
max_epsilon2=sim_data2.max_epsilon
median_epsilon2=sim_data2.median_epsilon


#%% 3
name3 = "30_04_2023_15_07_07"
#Psi=sim_obj.R_functions(name6)  
sim_data3=sim_obj.select_import_data(name3,path,dxmin,dxmax,dymin,dymax,None)
sim_data3.sort_epsilon_and_theta(count_range)
epsilon_section3=sim_data3.epsilon_section
epsilon_theta_section_max3=sim_data3.epsilon_theta_section_max
epsilon_theta_section_mean3=sim_data3.epsilon_theta_section_mean
epsilon_theta_section_median3=sim_data3.epsilon_theta_section_median

average_epsilon3=sim_data3.average_epsilon
max_epsilon3=sim_data3.max_epsilon
median_epsilon3=sim_data3.median_epsilon

#%% 4
name4 = "30_04_2023_17_03_41"
#Psi=sim_obj.R_functions(name6)  
sim_data4=sim_obj.select_import_data(name4,path,dxmin,dxmax,dymin,dymax,None)
sim_data4.sort_epsilon_and_theta(count_range)
epsilon_section4=sim_data4.epsilon_section
epsilon_theta_section_max4=sim_data4.epsilon_theta_section_max
epsilon_theta_section_mean4=sim_data4.epsilon_theta_section_mean
epsilon_theta_section_median4=sim_data4.epsilon_theta_section_median

average_epsilon4=sim_data4.average_epsilon
max_epsilon4=sim_data4.max_epsilon
median_epsilon4=sim_data4.median_epsilon

#%% 5
name5 = "30_04_2023_17_19_15"
#Psi=sim_obj.R_functions(name6)  
sim_data5=sim_obj.select_import_data(name5,path,dxmin,dxmax,dymin,dymax,None)
sim_data5.sort_epsilon_and_theta(count_range)
epsilon_section5=sim_data5.epsilon_section
epsilon_theta_section_max5=sim_data5.epsilon_theta_section_max
epsilon_theta_section_mean5=sim_data5.epsilon_theta_section_mean
epsilon_theta_section_median5=sim_data5.epsilon_theta_section_median

average_epsilon5=sim_data5.average_epsilon
max_epsilon5=sim_data5.max_epsilon
median_epsilon5=sim_data5.median_epsilon

#%% 6
name6 = "30_04_2023_17_19_20"
#Psi=sim_obj.R_functions(name6)  
sim_data6=sim_obj.select_import_data(name6,path,dxmin,dxmax,dymin,dymax,None)
sim_data6.sort_epsilon_and_theta(count_range)
epsilon_section6=sim_data6.epsilon_section
epsilon_theta_section_max6=sim_data6.epsilon_theta_section_max
epsilon_theta_section_mean6=sim_data6.epsilon_theta_section_mean
epsilon_theta_section_median6=sim_data6.epsilon_theta_section_median

average_epsilon6=sim_data6.average_epsilon
max_epsilon6=sim_data6.max_epsilon
median_epsilon6=sim_data6.median_epsilon

#%% 7
name7 = "30_04_2023_17_19_27"
#Psi=sim_obj.R_functions(name6)  
sim_data7=sim_obj.select_import_data(name7,path,dxmin,dxmax,dymin,dymax,None)
sim_data7.sort_epsilon_and_theta(count_range)
epsilon_section7=sim_data7.epsilon_section
epsilon_theta_section_max7=sim_data7.epsilon_theta_section_max
epsilon_theta_section_mean7=sim_data7.epsilon_theta_section_mean
epsilon_theta_section_median7=sim_data7.epsilon_theta_section_median

average_epsilon7=sim_data7.average_epsilon
max_epsilon7=sim_data7.max_epsilon
median_epsilon7=sim_data7.median_epsilon

#%% 8
name8 = "01_05_2023_07_51_05"
#Psi=sim_obj.R_functions(name6)  
sim_data8=sim_obj.select_import_data(name8,path,dxmin,dxmax,dymin,dymax,None)
sim_data8.sort_epsilon_and_theta(count_range)
epsilon_section8=sim_data8.epsilon_section
epsilon_theta_section_max8=sim_data8.epsilon_theta_section_max
epsilon_theta_section_mean8=sim_data8.epsilon_theta_section_mean
epsilon_theta_section_median8=sim_data8.epsilon_theta_section_median

average_epsilon8=sim_data8.average_epsilon
max_epsilon8=sim_data8.max_epsilon
median_epsilon8=sim_data8.median_epsilon

#%% 9
name9 = "01_05_2023_07_51_12"
#Psi=sim_obj.R_functions(name6)  
sim_data9=sim_obj.select_import_data(name9,path,dxmin,dxmax,dymin,dymax,None)
sim_data9.sort_epsilon_and_theta(count_range)
epsilon_section9=sim_data9.epsilon_section
epsilon_theta_section_max9=sim_data9.epsilon_theta_section_max
epsilon_theta_section_mean9=sim_data9.epsilon_theta_section_mean
epsilon_theta_section_median9=sim_data9.epsilon_theta_section_median

average_epsilon9=sim_data9.average_epsilon
max_epsilon9=sim_data9.max_epsilon
median_epsilon9=sim_data9.median_epsilon

#%% 10
name10 = "01_05_2023_07_51_18"
#Psi=sim_obj.R_functions(name6)  
sim_data10=sim_obj.select_import_data(name10,path,dxmin,dxmax,dymin,dymax,None)
sim_data10.sort_epsilon_and_theta(count_range)
epsilon_section10=sim_data10.epsilon_section
epsilon_theta_section_max10=sim_data10.epsilon_theta_section_max
epsilon_theta_section_mean10=sim_data10.epsilon_theta_section_mean
epsilon_theta_section_median10=sim_data10.epsilon_theta_section_median

average_epsilon10=sim_data10.average_epsilon
max_epsilon10=sim_data10.max_epsilon
median_epsilon10=sim_data10.median_epsilon


#%% 11
name11 = "01_05_2023_07_51_23"
#Psi=sim_obj.R_functions(name6)  
sim_data11=sim_obj.select_import_data(name11,path,dxmin,dxmax,dymin,dymax,None)
sim_data11.sort_epsilon_and_theta(count_range)
epsilon_section11=sim_data11.epsilon_section
epsilon_theta_section_max11=sim_data11.epsilon_theta_section_max
epsilon_theta_section_mean11=sim_data11.epsilon_theta_section_mean
epsilon_theta_section_median11=sim_data11.epsilon_theta_section_median

average_epsilon11=sim_data11.average_epsilon
max_epsilon11=sim_data11.max_epsilon
median_epsilon11=sim_data11.median_epsilon


#%% 12
name12 = "01_05_2023_07_51_35"
#Psi=sim_obj.R_functions(name6)  
sim_data12=sim_obj.select_import_data(name12,path,dxmin,dxmax,dymin,dymax,None)
sim_data12.sort_epsilon_and_theta(count_range)
epsilon_section12=sim_data12.epsilon_section
epsilon_theta_section_max12=sim_data12.epsilon_theta_section_max
epsilon_theta_section_mean12=sim_data12.epsilon_theta_section_mean
epsilon_theta_section_median12=sim_data12.epsilon_theta_section_median

average_epsilon12=sim_data12.average_epsilon
max_epsilon12=sim_data12.max_epsilon
median_epsilon12=sim_data12.median_epsilon


# %% 13
#Psi=sim_obj.R_functions(name1)  
name13 = "01_05_2023_14_25_01"
sim_data13=sim_obj.select_import_data(name13,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data13.EPSILON_
THETA=sim_data13.THETA
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
        
sim_data13.THETA=THETA        
sim_data13.sort_epsilon_and_theta(count_range)

epsilon_section13=sim_data13.epsilon_section
epsilon_theta_section_max13=sim_data13.epsilon_theta_section_max
epsilon_theta_section_mean13=sim_data13.epsilon_theta_section_mean
epsilon_theta_section_median13=sim_data13.epsilon_theta_section_median

average_epsilon13=sim_data13.average_epsilon
max_epsilon13=sim_data13.max_epsilon
median_epsilon13=sim_data13.median_epsilon

# %% 14
#Psi=sim_obj.R_functions(name1)  
name14 = "01_05_2023_14_25_06"
sim_data14=sim_obj.select_import_data(name14,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data14.EPSILON_
THETA=sim_data14.THETA
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
        
sim_data14.THETA=THETA        
sim_data14.sort_epsilon_and_theta(count_range)

epsilon_section14=sim_data14.epsilon_section
epsilon_theta_section_max14=sim_data14.epsilon_theta_section_max
epsilon_theta_section_mean14=sim_data14.epsilon_theta_section_mean
epsilon_theta_section_median14=sim_data14.epsilon_theta_section_median

average_epsilon14=sim_data14.average_epsilon
max_epsilon14=sim_data14.max_epsilon
median_epsilon14=sim_data14.median_epsilon

# %% 15
#Psi=sim_obj.R_functions(name1)  
name15 = "01_05_2023_14_25_13"
sim_data15=sim_obj.select_import_data(name15,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data15.EPSILON_
THETA=sim_data15.THETA
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
        
sim_data15.THETA=THETA        
sim_data15.sort_epsilon_and_theta(count_range)

epsilon_section15=sim_data15.epsilon_section
epsilon_theta_section_max15=sim_data15.epsilon_theta_section_max
epsilon_theta_section_mean15=sim_data15.epsilon_theta_section_mean
epsilon_theta_section_median145=sim_data15.epsilon_theta_section_median

average_epsilon15=sim_data15.average_epsilon
max_epsilon15=sim_data15.max_epsilon
median_epsilon15=sim_data15.median_epsilon

# %% 16
#Psi=sim_obj.R_functions(name1)  
name16 = "01_05_2023_14_25_19"
sim_data16=sim_obj.select_import_data(name16,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data16.EPSILON_
THETA=sim_data16.THETA
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
        
sim_data16.THETA=THETA        
sim_data16.sort_epsilon_and_theta(count_range)

epsilon_section16=sim_data16.epsilon_section
epsilon_theta_section_max16=sim_data16.epsilon_theta_section_max
epsilon_theta_section_mean16=sim_data16.epsilon_theta_section_mean
epsilon_theta_section_median16=sim_data16.epsilon_theta_section_median

average_epsilon16=sim_data16.average_epsilon
max_epsilon16=sim_data16.max_epsilon
median_epsilon16=sim_data16.median_epsilon


# %% 17
#Psi=sim_obj.R_functions(name1)  
name17 = "01_05_2023_14_25_25"
sim_data17=sim_obj.select_import_data(name17,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data17.EPSILON_
THETA=sim_data17.THETA
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
        
sim_data17.THETA=THETA        
sim_data17.sort_epsilon_and_theta(count_range)

epsilon_section17=sim_data17.epsilon_section
epsilon_theta_section_max17=sim_data17.epsilon_theta_section_max
epsilon_theta_section_mean17=sim_data17.epsilon_theta_section_mean
epsilon_theta_section_median17=sim_data17.epsilon_theta_section_median

average_epsilon17=sim_data17.average_epsilon
max_epsilon17=sim_data17.max_epsilon
median_epsilon17=sim_data17.median_epsilon


# %% 18
#Psi=sim_obj.R_functions(name1)  
name18 = "02_05_2023_10_33_26"
sim_data18=sim_obj.select_import_data(name18,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data18.EPSILON_
THETA=sim_data18.THETA
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
        
sim_data18.THETA=THETA        
sim_data18.sort_epsilon_and_theta(count_range)

epsilon_section18=sim_data18.epsilon_section
epsilon_theta_section_max18=sim_data18.epsilon_theta_section_max
epsilon_theta_section_mean18=sim_data18.epsilon_theta_section_mean
epsilon_theta_section_median18=sim_data18.epsilon_theta_section_median

average_epsilon18=sim_data18.average_epsilon
max_epsilon18=sim_data18.max_epsilon
median_epsilon18=sim_data18.median_epsilon


# %% 19
#Psi=sim_obj.R_functions(name1)  
name19 = "02_05_2023_10_33_34"
sim_data19=sim_obj.select_import_data(name19,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data19.EPSILON_
THETA=sim_data19.THETA
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
        
sim_data19.THETA=THETA        
sim_data19.sort_epsilon_and_theta(count_range)

epsilon_section19=sim_data19.epsilon_section
epsilon_theta_section_max19=sim_data19.epsilon_theta_section_max
epsilon_theta_section_mean19=sim_data19.epsilon_theta_section_mean
epsilon_theta_section_median19=sim_data19.epsilon_theta_section_median

average_epsilon19=sim_data19.average_epsilon
max_epsilon19=sim_data19.max_epsilon
median_epsilon19=sim_data19.median_epsilon


# %% 20
#Psi=sim_obj.R_functions(name1)  
name20 = "02_05_2023_10_33_40"
sim_data20=sim_obj.select_import_data(name20,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data20.EPSILON_
THETA=sim_data20.THETA
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
        
sim_data20.THETA=THETA        
sim_data20.sort_epsilon_and_theta(count_range)

epsilon_section20=sim_data20.epsilon_section
epsilon_theta_section_max20=sim_data20.epsilon_theta_section_max
epsilon_theta_section_mean20=sim_data20.epsilon_theta_section_mean
epsilon_theta_section_median20=sim_data20.epsilon_theta_section_median

average_epsilon20=sim_data20.average_epsilon
max_epsilon20=sim_data20.max_epsilon
median_epsilon20=sim_data20.median_epsilon

# %% 21
#Psi=sim_obj.R_functions(name1)  
name21 = "02_05_2023_10_33_54"
sim_data21=sim_obj.select_import_data(name21,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data21.EPSILON_
THETA=sim_data21.THETA
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
        
sim_data21.THETA=THETA        
sim_data21.sort_epsilon_and_theta(count_range)

epsilon_section21=sim_data21.epsilon_section
epsilon_theta_section_max21=sim_data21.epsilon_theta_section_max
epsilon_theta_section_mean21=sim_data21.epsilon_theta_section_mean
epsilon_theta_section_median21=sim_data21.epsilon_theta_section_median

average_epsilon21=sim_data21.average_epsilon
max_epsilon21=sim_data21.max_epsilon
median_epsilon21=sim_data21.median_epsilon


# %% 22
#Psi=sim_obj.R_functions(name1)  
name22 = "02_05_2023_10_34_01"
sim_data22=sim_obj.select_import_data(name22,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data22.EPSILON_
THETA=sim_data22.THETA
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
        
sim_data22.THETA=THETA        
sim_data22.sort_epsilon_and_theta(count_range)

epsilon_section22=sim_data22.epsilon_section
epsilon_theta_section_max22=sim_data22.epsilon_theta_section_max
epsilon_theta_section_mean22=sim_data22.epsilon_theta_section_mean
epsilon_theta_section_median22=sim_data22.epsilon_theta_section_median

average_epsilon22=sim_data22.average_epsilon
max_epsilon22=sim_data22.max_epsilon
median_epsilon22=sim_data22.median_epsilon


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
    temp10=np.asarray(epsilon_theta_section_max10[str(i)])
    temp11=np.asarray(epsilon_theta_section_max11[str(i)])   
    temp12=np.asarray(epsilon_theta_section_max12[str(i)])
    temp13=np.asarray(epsilon_theta_section_max13[str(i)])
    temp14=np.asarray(epsilon_theta_section_max14[str(i)])
    temp15=np.asarray(epsilon_theta_section_max15[str(i)])
    temp16=np.asarray(epsilon_theta_section_max16[str(i)])
    temp17=np.asarray(epsilon_theta_section_max17[str(i)])
    temp18=np.asarray(epsilon_theta_section_max18[str(i)])
    temp19=np.asarray(epsilon_theta_section_max19[str(i)])
    temp20=np.asarray(epsilon_theta_section_max20[str(i)])
    temp21=np.asarray(epsilon_theta_section_max21[str(i)])
    temp22=np.asarray(epsilon_theta_section_max22[str(i)])
    # temp18=np.asarray(epsilon_theta_section_max18[str(i)])
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
                         temp10,
                         temp11,
                         temp12,
                         temp13,
                         temp14,
                         temp15,
                         temp16,
                         temp17,
                         temp18,
                         temp19,
                         temp20,
                         temp21,
                         temp22))
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
#positions=[0,1,2,3,4,5,6,7,8,9,10,11]
#positions=[0,1,2,3,4,5,6,7]
#axs2.set_xticks(positions)
#axs2.set_yticks([0,1,2,3,4,5])
#axs2.set_xticklabels(labels,color='k',fontsize=8)
#axs2.bar(labels,ymean,color="tab:blue",zorder=3)
#print(np.max(ymean)-np.min(ymean))
#axs2.set_ylabel('$\epsilon$',labelpad=-1,fontsize=9)
#axs2.set_xlabel('angle (rad)',labelpad=-2,fontsize=9)
#axs2.set_title(r'$(a)$',fontsize=9)
#axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
#axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
#axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")        
        
# %%

c="tab:red"
name="searching_circle_guassian2"
num_bins=20
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
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")

#num_bins=25
#x_=np.linspace(0,5,200)  

# fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
mean = statistics.mean(y)
sd = statistics.stdev(y)
print(mean)
print(sd)
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