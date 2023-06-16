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
path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.linewidth'] = .1
#path="D:/dmulroy/Experiments/square_search/"
#path="D:/dmulroy/Experiments/strange_object_search/"
path="D:/dmulroy/Experiments/strange_object_search2/"
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
# %%
d=4.5
count_range=12
snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d
name1 = "03_05_2023_11_31_50"
name2 = "03_05_2023_15_02_15"
name3 = "03_05_2023_15_02_26"
name4 = "03_05_2023_15_02_35"
name5 = "03_05_2023_18_35_06"
name6 = "03_05_2023_18_35_14"
name7 = "03_05_2023_18_35_23"
name8 = "04_05_2023_09_41_58"
name9 = "04_05_2023_09_42_05"
name10 = "04_05_2023_09_42_12"
name11 = "05_05_2023_08_51_00"
name12 = "05_05_2023_08_52_55"
name13 = "05_05_2023_08_53_13"
name14 = "05_05_2023_08_53_35"
name15 = "06_05_2023_12_10_39"
name16 = "06_05_2023_12_10_49"
name17 = "06_05_2023_12_10_57"
name18 = "06_05_2023_12_11_11"
name19 = "07_05_2023_11_54_28"
name20 = "07_05_2023_11_54_36"
name21 = "07_05_2023_11_54_45"
name22 = "07_05_2023_11_54_09"
name23 = "07_05_2023_11_54_16"
name24 = "07_05_2023_21_30_16"
#name25 = "07_05_2023_21_30_25"
#name26 = "08_05_2023_09_30_44"
#name27 = "08_05_2023_09_30_51"
#name28 = "08_05_2023_09_30_57"
#name29= "08_05_2023_09_31_04"
plt.close('all')
# %% Test 1
#Psi=sim_obj.R_functions(name1)  
sim_data1=sim_obj.select_import_data(name1,path,dxmin,dxmax,dymin,dymax,None)
sim_data1.sort_epsilon_and_theta(count_range)
epsilon_section1=sim_data1.epsilon_section
epsilon_theta_section_max1=sim_data1.epsilon_theta_section_max
epsilon_theta_section_mean1=sim_data1.epsilon_theta_section_mean
epsilon_theta_section_median1=sim_data1.epsilon_theta_section_median
average_epsilon1=sim_data1.average_epsilon
max_epsilon1=sim_data1.max_epsilon
median_epsilon1=sim_data1.median_epsilon

# %% Test 2
#Psi=sim_obj.R_functions(name2)  
sim_data2=sim_obj.select_import_data(name2,path,dxmin,dxmax,dymin,dymax,None)
sim_data2.sort_epsilon_and_theta(count_range)
epsilon_section2=sim_data2.epsilon_section
epsilon_theta_section_max2=sim_data2.epsilon_theta_section_max
epsilon_theta_section_mean2=sim_data2.epsilon_theta_section_mean
epsilon_theta_section_median2=sim_data2.epsilon_theta_section_median

average_epsilon2=sim_data2.average_epsilon
max_epsilon2=sim_data2.max_epsilon
median_epsilon2=sim_data2.median_epsilon


# %% Test 3
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


#%% Test 4
#Psi=sim_obj.R_functions(name4)  
sim_data4=sim_obj.select_import_data(name4,path,dxmin,dxmax,dymin,dymax,None)
sim_data4.sort_epsilon_and_theta(count_range)
epsilon_section4=sim_data4.epsilon_section
epsilon_theta_section_max4=sim_data4.epsilon_theta_section_max
epsilon_theta_section_mean4=sim_data4.epsilon_theta_section_mean
epsilon_theta_section_median4=sim_data4.epsilon_theta_section_median

average_epsilon4=sim_data4.average_epsilon
max_epsilon4=sim_data4.max_epsilon
median_epsilon4=sim_data4.median_epsilon


#%% Test 5
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

#%% Test 6
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


#%% Test 7
sim_data7=sim_obj.select_import_data(name7,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data7.EPSILON_
THETA=sim_data7.THETA
temp=2.1*np.pi   
sim_data7.THETA=np.flip(THETA)  
sim_data7.EPSILON_=np.flip(EPSILON_)      
sim_data7.sort_epsilon_and_theta(count_range)

epsilon_section7=sim_data7.epsilon_section
epsilon_theta_section_max7=sim_data7.epsilon_theta_section_max
epsilon_theta_section_mean7=sim_data7.epsilon_theta_section_mean
epsilon_theta_section_median7=sim_data7.epsilon_theta_section_median

average_epsilon7=sim_data7.average_epsilon
max_epsilon7=sim_data7.max_epsilon
median_epsilon7=sim_data7.median_epsilon

#%% Test 8
sim_data8=sim_obj.select_import_data(name8,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data8.EPSILON_
THETA=sim_data8.THETA
temp=2.1*np.pi   
sim_data8.THETA=np.flip(THETA)  
sim_data8.EPSILON_=np.flip(EPSILON_)      
sim_data8.sort_epsilon_and_theta(count_range)

epsilon_section8=sim_data8.epsilon_section
epsilon_theta_section_max8=sim_data8.epsilon_theta_section_max
epsilon_theta_section_mean8=sim_data8.epsilon_theta_section_mean
epsilon_theta_section_median8=sim_data8.epsilon_theta_section_median
average_epsilon8=sim_data8.average_epsilon
max_epsilon8=sim_data8.max_epsilon
median_epsilon8=sim_data8.median_epsilon


#%% Test 9
sim_data9=sim_obj.select_import_data(name9,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data9.EPSILON_
THETA=sim_data9.THETA
temp=2.1*np.pi   
sim_data9.THETA=np.flip(THETA)  
sim_data9.EPSILON_=np.flip(EPSILON_)      
sim_data9.sort_epsilon_and_theta(count_range)

epsilon_section9=sim_data9.epsilon_section
epsilon_theta_section_max9=sim_data9.epsilon_theta_section_max
epsilon_theta_section_mean9=sim_data9.epsilon_theta_section_mean
epsilon_theta_section_median9=sim_data9.epsilon_theta_section_median
average_epsilon9=sim_data9.average_epsilon
max_epsilon9=sim_data9.max_epsilon
median_epsilon9=sim_data9.median_epsilon

#%% Test 10
sim_data10=sim_obj.select_import_data(name10,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data10.EPSILON_
THETA=sim_data10.THETA
temp=2.1*np.pi   
sim_data10.THETA=np.flip(THETA)  
sim_data10.EPSILON_=np.flip(EPSILON_)      
sim_data10.sort_epsilon_and_theta(count_range)

epsilon_section10=sim_data10.epsilon_section
epsilon_theta_section_max10=sim_data10.epsilon_theta_section_max
epsilon_theta_section_mean10=sim_data10.epsilon_theta_section_mean
epsilon_theta_section_median10=sim_data10.epsilon_theta_section_median
average_epsilon10=sim_data10.average_epsilon
max_epsilon10=sim_data10.max_epsilon
median_epsilon10=sim_data10.median_epsilon

#%% Test 11
sim_data11=sim_obj.select_import_data(name11,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data11.EPSILON_
THETA=sim_data11.THETA
temp=2.1*np.pi   
sim_data11.THETA=np.flip(THETA)  
sim_data11.EPSILON_=np.flip(EPSILON_)      
sim_data11.sort_epsilon_and_theta(count_range)

epsilon_section11=sim_data11.epsilon_section
epsilon_theta_section_max11=sim_data11.epsilon_theta_section_max
epsilon_theta_section_mean11=sim_data11.epsilon_theta_section_mean
epsilon_theta_section_median11=sim_data11.epsilon_theta_section_median
average_epsilon11=sim_data11.average_epsilon
max_epsilon11=sim_data11.max_epsilon
median_epsilon11=sim_data11.median_epsilon

#%% Test 12
#Psi=sim_obj.R_functions(name6)  
sim_data12 = sim_obj.select_import_data(name12,path,dxmin,dxmax,dymin,dymax,None)
sim_data12.sort_epsilon_and_theta(count_range)
epsilon_section12 = sim_data12.epsilon_section
epsilon_theta_section_max12 = sim_data12.epsilon_theta_section_max
epsilon_theta_section_mean12 = sim_data12.epsilon_theta_section_mean
epsilon_theta_section_median12 = sim_data12.epsilon_theta_section_median

average_epsilon12 = sim_data12.average_epsilon
max_epsilon12 = sim_data12.max_epsilon
median_epsilon12 = sim_data12.median_epsilon


#%% Test 13
#Psi=sim_obj.R_functions(name6)  
sim_data13 = sim_obj.select_import_data(name13,path,dxmin,dxmax,dymin,dymax,None)
sim_data13.sort_epsilon_and_theta(count_range)
epsilon_section13 = sim_data13.epsilon_section
epsilon_theta_section_max13 = sim_data13.epsilon_theta_section_max
epsilon_theta_section_mean13 = sim_data13.epsilon_theta_section_mean
epsilon_theta_section_median13 = sim_data13.epsilon_theta_section_median

average_epsilon13 = sim_data13.average_epsilon
max_epsilon13 = sim_data13.max_epsilon
median_epsilon13 = sim_data13.median_epsilon

#%% Test 14
sim_data14 = sim_obj.select_import_data(name14,path,dxmin,dxmax,dymin,dymax,None)
sim_data14.sort_epsilon_and_theta(count_range)
epsilon_section14 = sim_data14.epsilon_section
epsilon_theta_section_max14 = sim_data14.epsilon_theta_section_max
epsilon_theta_section_mean14 = sim_data14.epsilon_theta_section_mean
epsilon_theta_section_median14 = sim_data14.epsilon_theta_section_median
average_epsilon14 = sim_data14.average_epsilon
max_epsilon14 = sim_data14.max_epsilon
median_epsilon14 = sim_data14.median_epsilon

#%% Test 15
sim_data15 = sim_obj.select_import_data(name15,path,dxmin,dxmax,dymin,dymax,None)
sim_data15.sort_epsilon_and_theta(count_range)
epsilon_section15 = sim_data15.epsilon_section
epsilon_theta_section_max15 = sim_data15.epsilon_theta_section_max
epsilon_theta_section_mean15 = sim_data15.epsilon_theta_section_mean
epsilon_theta_section_median15 = sim_data15.epsilon_theta_section_median
average_epsilon15 = sim_data15.average_epsilon
max_epsilon15 = sim_data15.max_epsilon
median_epsilon15 = sim_data15.median_epsilon


#%% Test 16
sim_data16 = sim_obj.select_import_data(name16,path,dxmin,dxmax,dymin,dymax,None)
sim_data16.sort_epsilon_and_theta(count_range)
epsilon_section16 = sim_data16.epsilon_section
epsilon_theta_section_max16 = sim_data16.epsilon_theta_section_max
epsilon_theta_section_mean16 = sim_data16.epsilon_theta_section_mean
epsilon_theta_section_median16 = sim_data16.epsilon_theta_section_median
average_epsilon16 = sim_data16.average_epsilon
max_epsilon16 = sim_data16.max_epsilon
median_epsilon16 = sim_data16.median_epsilon

#%% Test 17
sim_data17 = sim_obj.select_import_data(name17,path,dxmin,dxmax,dymin,dymax,None)
sim_data17.sort_epsilon_and_theta(count_range)
epsilon_section17 = sim_data17.epsilon_section
epsilon_theta_section_max17 = sim_data17.epsilon_theta_section_max
epsilon_theta_section_mean17 = sim_data17.epsilon_theta_section_mean
epsilon_theta_section_median17 = sim_data17.epsilon_theta_section_median
average_epsilon17 = sim_data17.average_epsilon
max_epsilon17 = sim_data17.max_epsilon
median_epsilon17 = sim_data17.median_epsilon

#%% Test 18
sim_data18 = sim_obj.select_import_data(name18,path,dxmin,dxmax,dymin,dymax,None)
sim_data18.sort_epsilon_and_theta(count_range)
epsilon_section18 = sim_data18.epsilon_section
epsilon_theta_section_max18 = sim_data18.epsilon_theta_section_max
epsilon_theta_section_mean18 = sim_data18.epsilon_theta_section_mean
epsilon_theta_section_median18 = sim_data18.epsilon_theta_section_median
average_epsilon18 = sim_data18.average_epsilon
max_epsilon18 = sim_data18.max_epsilon
median_epsilon18 = sim_data18.median_epsilon


#%% Test 19
sim_data19 = sim_obj.select_import_data(name19,path,dxmin,dxmax,dymin,dymax,None)
sim_data19.sort_epsilon_and_theta(count_range)
epsilon_section19 = sim_data19.epsilon_section
epsilon_theta_section_max19 = sim_data19.epsilon_theta_section_max
epsilon_theta_section_mean19 = sim_data19.epsilon_theta_section_mean
epsilon_theta_section_median19 = sim_data19.epsilon_theta_section_median
average_epsilon19 = sim_data19.average_epsilon
max_epsilon19 = sim_data19.max_epsilon
median_epsilon19 = sim_data19.median_epsilon

#%% Test 20
sim_data20 = sim_obj.select_import_data(name20,path,dxmin,dxmax,dymin,dymax,None)
sim_data20.sort_epsilon_and_theta(count_range)
epsilon_section20 = sim_data20.epsilon_section
epsilon_theta_section_max20 = sim_data20.epsilon_theta_section_max
epsilon_theta_section_mean20 = sim_data20.epsilon_theta_section_mean
epsilon_theta_section_median20 = sim_data20.epsilon_theta_section_median
average_epsilon20 = sim_data20.average_epsilon
max_epsilon20 = sim_data20.max_epsilon
median_epsilon20 = sim_data20.median_epsilon



#%% Test 21
sim_data21 = sim_obj.select_import_data(name21,path,dxmin,dxmax,dymin,dymax,None)
sim_data21.sort_epsilon_and_theta(count_range)
epsilon_section21 = sim_data21.epsilon_section
epsilon_theta_section_max21 = sim_data21.epsilon_theta_section_max
epsilon_theta_section_mean21 = sim_data21.epsilon_theta_section_mean
epsilon_theta_section_median21 = sim_data21.epsilon_theta_section_median
average_epsilon21 = sim_data21.average_epsilon
max_epsilon21 = sim_data21.max_epsilon
median_epsilon21 = sim_data21.median_epsilon

#%% Test 22
sim_data22 = sim_obj.select_import_data(name22,path,dxmin,dxmax,dymin,dymax,None)
sim_data22.sort_epsilon_and_theta(count_range)
epsilon_section22 = sim_data22.epsilon_section
epsilon_theta_section_max22 = sim_data22.epsilon_theta_section_max
epsilon_theta_section_mean22 = sim_data22.epsilon_theta_section_mean
epsilon_theta_section_median22 = sim_data22.epsilon_theta_section_median
average_epsilon22 = sim_data22.average_epsilon
max_epsilon22 = sim_data22.max_epsilon
median_epsilon22 = sim_data22.median_epsilon


#%% Test 23
sim_data23 = sim_obj.select_import_data(name23,path,dxmin,dxmax,dymin,dymax,None)
sim_data23.sort_epsilon_and_theta(count_range)
epsilon_section23 = sim_data23.epsilon_section
epsilon_theta_section_max23 = sim_data23.epsilon_theta_section_max
epsilon_theta_section_mean23 = sim_data23.epsilon_theta_section_mean
epsilon_theta_section_median23 = sim_data23.epsilon_theta_section_median
average_epsilon23 = sim_data23.average_epsilon
max_epsilon23 = sim_data23.max_epsilon
median_epsilon23 = sim_data23.median_epsilon

#%% Test 24
sim_data24 = sim_obj.select_import_data(name24,path,dxmin,dxmax,dymin,dymax,None)
sim_data24.sort_epsilon_and_theta(count_range)
epsilon_section24 = sim_data24.epsilon_section
epsilon_theta_section_max24 = sim_data24.epsilon_theta_section_max
epsilon_theta_section_mean24 = sim_data24.epsilon_theta_section_mean
epsilon_theta_section_median24 = sim_data24.epsilon_theta_section_median
average_epsilon24 = sim_data24.average_epsilon
max_epsilon24 = sim_data24.max_epsilon
median_epsilon24 = sim_data24.median_epsilon


# #%% Test 25
# sim_data25 = sim_obj.select_import_data(name25,path,dxmin,dxmax,dymin,dymax,None)
# sim_data25.sort_epsilon_and_theta(count_range)
# epsilon_section25 = sim_data25.epsilon_section
# epsilon_theta_section_max25 = sim_data25.epsilon_theta_section_max
# epsilon_theta_section_mean25 = sim_data25.epsilon_theta_section_mean
# epsilon_theta_section_median25 = sim_data25.epsilon_theta_section_median
# average_epsilon25 = sim_data25.average_epsilon
# max_epsilon25 = sim_data25.max_epsilon
# median_epsilon25 = sim_data25.median_epsilon

# #%% Test 26
# sim_data26 = sim_obj.select_import_data(name26,path,dxmin,dxmax,dymin,dymax,None)
# sim_data26.sort_epsilon_and_theta(count_range)
# epsilon_section26 = sim_data26.epsilon_section
# epsilon_theta_section_max26 = sim_data26.epsilon_theta_section_max
# epsilon_theta_section_mean26 = sim_data26.epsilon_theta_section_mean
# epsilon_theta_section_median26 = sim_data26.epsilon_theta_section_median
# average_epsilon26 = sim_data26.average_epsilon
# max_epsilon26 = sim_data26.max_epsilon
# median_epsilon26 = sim_data26.median_epsilon

# #%% Test 27
# sim_data27 = sim_obj.select_import_data(name27,path,dxmin,dxmax,dymin,dymax,None)
# sim_data27.sort_epsilon_and_theta(count_range)
# epsilon_section27 = sim_data27.epsilon_section
# epsilon_theta_section_max27 = sim_data27.epsilon_theta_section_max
# epsilon_theta_section_mean27 = sim_data27.epsilon_theta_section_mean
# epsilon_theta_section_median27 = sim_data27.epsilon_theta_section_median
# average_epsilon27 = sim_data27.average_epsilon
# max_epsilon27 = sim_data27.max_epsilon
# median_epsilon27 = sim_data27.median_epsilon

# #%% Test 28
# sim_data28 = sim_obj.select_import_data(name28,path,dxmin,dxmax,dymin,dymax,None)
# sim_data28.sort_epsilon_and_theta(count_range)
# epsilon_section28 = sim_data28.epsilon_section
# epsilon_theta_section_max28 = sim_data28.epsilon_theta_section_max
# epsilon_theta_section_mean28 = sim_data28.epsilon_theta_section_mean
# epsilon_theta_section_median28 = sim_data28.epsilon_theta_section_median
# average_epsilon28 = sim_data28.average_epsilon
# max_epsilon28 = sim_data28.max_epsilon
# median_epsilon28 = sim_data28.median_epsilon


# #%% Test 29
# sim_data29 = sim_obj.select_import_data(name29,path,dxmin,dxmax,dymin,dymax,None)
# sim_data29.sort_epsilon_and_theta(count_range)
# epsilon_section29 = sim_data29.epsilon_section
# epsilon_theta_section_max29 = sim_data29.epsilon_theta_section_max
# epsilon_theta_section_mean29 = sim_data29.epsilon_theta_section_mean
# epsilon_theta_section_median29 = sim_data29.epsilon_theta_section_median
# average_epsilon29 = sim_data29.average_epsilon
# max_epsilon29 = sim_data29.max_epsilon
# median_epsilon29 = sim_data29.median_epsilon
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
    temp23=np.asarray(epsilon_theta_section_max23[str(i)])  
    temp24=np.asarray(epsilon_theta_section_max24[str(i)]) 
    #temp25=np.asarray(epsilon_theta_section_max25[str(i)])  
    #temp26=np.asarray(epsilon_theta_section_max26[str(i)])  
    # temp27=np.asarray(epsilon_theta_section_max27[str(i)])  
    # temp28=np.asarray(epsilon_theta_section_max28[str(i)])  
    # temp29=np.asarray(epsilon_theta_section_max29[str(i)])  
    #temp3=[temp,temp2]
    
    temp = np.concatenate((temp1,
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
                           temp22,
                           temp23,
                           temp24))
                           #temp25,
                           #temp26))
                           # temp27,
                           # temp28,
                           # temp29))
          
                           #temp25,
                           #temp26,
                           #temp27,
                           #temp28,
                           #temp29))
 
 

    #temp3=temp3.flatten()
    max_epsilon_[str(i)].append(temp)
    #max_epsilon_[str(i)].append(epsilon_theta_section_max1[str(i)])

# %%

c="tab:red"
name="searching_square_bar_plot_max"
#fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
y=[]
ymean=[]
for i in range(len(max_epsilon_)):
    entry=int(str(i))
    y_=np.nan_to_num(max_epsilon_[str(i)][0])
    y__=[]
    if i==0:
        
        res2=np.nonzero(y_)
        res2=res2[0]
        for j in res2:
            y__.append(y_[j])
        y_=y__
    #else:
    #y__=[]
    #print(y_)
    #for j in res2:
    for j in range(len(y_)):
        if y_[j]<2.3:
            y__.append(y_[j])
    y.append(y__)
    x=entry*np.ones(len(y__))
    #axs2.scatter(x,y__,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
    #axs2.scatter(x,y__,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
    mean = np.mean(y_)
    sd = statistics.stdev(y_)
    if np.sign(mean-sd)==-1:
        error=[0,mean+sd]
    else:    
        error=[mean-sd,mean+sd]
    #print(error)
    #print(x)
    #print(len(x),len(error))
    #print(x)
    #print([x[1],x[1]],error)
    #print(i)
    #if len(x)>1:
        #axs2.plot([x[1],x[1]],error,color="k",linewidth=1,zorder=3)
        #axs2.scatter(x,y_,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
        #axs2.scatter(x,y__,color=c,marker='s',s=3,zorder=3)
    ymean.append(mean)
positions=[0,1,2,3,4,5,6,7,8,9,10,11]
#positions=[0,1,2,3,4,5,6,7]
#axs2.set_xticks(positions)
#axs2.set_yticks([0,1,2,3,4,5])
#axs2.set_xticklabels(labels,color='k',fontsize=8)
#axs2.bar(labels,ymean,color="tab:blue",zorder=3)
#print(np.max(ymean)-np.min(ymean))
#axs2.set_ylabel('$\epsilon$',labelpad=-1,fontsize=9)
# axs2.set_xlabel('angle (rad)',labelpad=-2,fontsize=9)
# axs2.set_title(r'$(a)$',fontsize=9)
# axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# plt.close('all')

# %%


ytemp1=y[0] #
ytemp2=y[1]#
ytemp3=y[2]#
ytemp4=y[3]
ytemp5=y[4]
ytemp6=y[5]
ytemp7=y[6]
ytemp8=y[7]
ytemp9=y[8]
ytemp10=y[9]#
ytemp11=y[10]#
ytemp12=y[11]#

print("number of times encirlcing=",len(ytemp2))
#print(len(ytemp12))

ytemp1_=np.concatenate((ytemp2,ytemp12))
print("mu1=",np.mean(ytemp1))
print("mu2=",np.mean(ytemp1_))

#print(len(ytemp1))
#print(len(ytemp1_))

#res=np.where(ytemp1==0)
#print(len(res[0])/len(ytemp1))
ytemp1_=np.round(ytemp1_,1)
res=np.where(ytemp1_==0)
print("failed_cases=",len(res[0])/len(ytemp1_))


#res=np.where(ytemp12==0)
#print(len(res[0])/len(ytemp12))
num_bins=15

# %%
name="face"
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
#heights, bins = np.histogram(data, bins = len(list(set(data))))
n=plt.hist(ytemp1,num_bins, weights=np.ones(len(ytemp1)) / len(ytemp1),color="tab:blue",zorder=3)
axs.hist(ytemp1,num_bins, weights=np.ones(len(ytemp1)) / len(ytemp1),color="tab:blue",zorder=3)
#percent = [i/sum(ytemp1)*100 for i in ytemp1]
#vals = axs.get_yticks()
#axs.set_yticklabels(['%1.2f' %i for i in vals])
axs.set_yticks([0,0.1,0.2,0.3,0.4,0.5])
axs.set_xticks([0,0.5,1,1.5,2])
#plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.set_title('(a)')
axs.set_xlabel(r'$\epsilon$')
axs.grid(True,linewidth=0.1,zorder=-3)
x_=np.linspace(0,2,200)  
mean = statistics.mean(ytemp1)
sd = statistics.stdev(ytemp1)
N=norm.pdf(x_, mean, sd)
textstr = '\n'.join((
    r'$\mu=%.2f$' % (mean, ),
    r'$\sigma=%.2f$' % (sd, )))
props = dict(boxstyle='round', facecolor='white', alpha=0.5)
axs.text(0.75, 0.95, textstr, transform=axs.transAxes, fontsize=9,
        verticalalignment='top', bbox=props)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")


name="sides"
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
#heights, bins = np.histogram(data, bins = len(list(set(data))))
nn=plt.hist(ytemp1_,num_bins, weights=np.ones(len(ytemp1_)) / len(ytemp1_),color="tab:blue",zorder=3)
axs.hist(ytemp1_,num_bins, weights=np.ones(len(ytemp1_)) / len(ytemp1_),color="tab:orange",zorder=3)
axs.set_yticks([0,0.1,0.2,0.3,0.4,0.5])
axs.set_xticks([0,0.5,1,1.5,2])
#plt.gca().yaxis.set_major_formatter(PercentFormatter(1))
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.set_title('(b)')
axs.set_xlabel(r'$\epsilon$')
axs.grid(True,linewidth=0.1,zorder=-3)
x_=np.linspace(0,2,200)  
mean = statistics.mean(ytemp1_)
sd = statistics.stdev(ytemp1_)
N=norm.pdf(x_, mean, sd)
textstr = '\n'.join((
    r'$\mu=%.2f$' % (mean, ),
    r'$\sigma=%.2f$' % (sd, )))
props = dict(boxstyle='round', facecolor='white', alpha=0.5)
axs.text(0.75, 0.95, textstr, transform=axs.transAxes, fontsize=9,
        verticalalignment='top', bbox=props)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")