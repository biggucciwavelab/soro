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
path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.linewidth'] = .1
#path="D:/dmulroy/Experiments/square_search/"
path="D:/dmulroy/Experiments/strange_object_search/"
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
name1 = "09_02_2023_11_30_34"
name2 = "09_02_2023_21_34_40"
name3 = "10_02_2023_18_53_40"
name4 = "10_02_2023_19_26_36"
name5 = "11_02_2023_13_15_36"
name6 = "11_02_2023_13_15_43"
name7 = "12_02_2023_11_25_50"
name8 = "12_02_2023_12_45_48"
name9 = "13_02_2023_10_10_12"
name10 = "13_02_2023_10_10_21"
name11 = "13_02_2023_10_10_04"
name12 = "14_02_2023_18_32_26"
name13 = "14_02_2023_18_32_33"
name14 = "14_02_2023_18_32_41"
name15 = "15_02_2023_09_31_03"
name16 = "15_02_2023_09_31_09"
name17 = "15_02_2023_09_31_14"
name18 = "15_02_2023_17_20_28"
name19 = "15_02_2023_17_20_35"
name20 = "15_02_2023_17_20_42"
name21 = "15_02_2023_17_20_50"
name22 = "16_02_2023_07_14_10"
name23 = "16_02_2023_07_14_16"
name24 = "16_02_2023_07_14_20"
name25 = "16_02_2023_07_14_25"
name26 = "16_02_2023_17_32_08"
name27 = "16_02_2023_17_32_16"
name28 = "16_02_2023_17_32_25"
name29 = "16_02_2023_17_32_32"
name30 = "17_02_2023_12_24_14"
name31 = "17_02_2023_12_24_19"
name32 = "17_02_2023_12_24_22"
name33 = "17_02_2023_12_24_26"
name34 = "17_02_2023_12_24_31"
name35 = "18_02_2023_11_07_57"
name36 = "18_02_2023_11_07_52"
name37 = "18_02_2023_11_08_02"
name38 = "18_02_2023_11_08_06"
name39 = "18_02_2023_11_08_12"
name40 = "18_02_2023_22_10_06"
name41 = "18_02_2023_22_10_17"
name42 = "20_02_2023_09_18_12"
name43 = "20_02_2023_09_18_06"
name44 = "21_02_2023_09_58_33"
name45 = "21_02_2023_09_58_41" 
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


im_data30.epsilon_section
epsilon_theta_section_max30 = sim_data30.epsilon_theta_section_max
epsilon_theta_section_mean30 = sim_data30.epsilon_theta_section_mean
epsilon_theta_section_median30 = sim_data30.epsilon_theta_section_median
average_epsilon30 = sim_data30.average_epsilon
max_epsilon30 = sim_data30.max_epsilon
median_epsilon30 = sim_data30.median_epsilon


#%% Test 31
sim_data31 = sim_obj.select_import_data(name31,path,dxmin,dxmax,dymin,dymax,None)
sim_data31.sort_epsilon_and_theta(count_range)
epsilon_section31 = sim_data31.epsilon_section
epsilon_theta_section_max31 = sim_data31.epsilon_theta_section_max
epsilon_theta_section_mean31 = sim_data31.epsilon_theta_section_mean
epsilon_theta_section_median31 = sim_data31.epsilon_theta_section_median
average_epsilon31 = sim_data31.average_epsilon
max_epsilon31 = sim_data31.max_epsilon
median_epsilon31 = sim_data31.median_epsilon

#%% Test 32
sim_data32 = sim_obj.select_import_data(name32,path,dxmin,dxmax,dymin,dymax,None)
sim_data32.sort_epsilon_and_theta(count_range)
epsilon_section32 = sim_data32.epsilon_section
epsilon_theta_section_max32 = sim_data32.epsilon_theta_section_max
epsilon_theta_section_mean32 = sim_data32.epsilon_theta_section_mean
epsilon_theta_section_median32 = sim_data32.epsilon_theta_section_median
average_epsilon32 = sim_data32.average_epsilon
max_epsilon32 = sim_data32.max_epsilon
median_epsilon32 = sim_data32.median_epsilon

#%% Test 33
sim_data33 = sim_obj.select_import_data(name33,path,dxmin,dxmax,dymin,dymax,None)
sim_data33.sort_epsilon_and_theta(count_range)
epsilon_section33 = sim_data33.epsilon_section
epsilon_theta_section_max33 = sim_data33.epsilon_theta_section_max
epsilon_theta_section_mean33 = sim_data33.epsilon_theta_section_mean
epsilon_theta_section_median33 = sim_data33.epsilon_theta_section_median
average_epsilon33 = sim_data33.average_epsilon
max_epsilon33 = sim_data33.max_epsilon
median_epsilon33 = sim_data33.median_epsilon


#%% Test 34
sim_data34 = sim_obj.select_import_data(name34,path,dxmin,dxmax,dymin,dymax,None)
sim_data34.sort_epsilon_and_theta(count_range)
epsilon_section34 = sim_data34.epsilon_section
epsilon_theta_section_max34 = sim_data34.epsilon_theta_section_max
epsilon_theta_section_mean34 = sim_data34.epsilon_theta_section_mean
epsilon_theta_section_median34 = sim_data34.epsilon_theta_section_median
average_epsilon34 = sim_data34.average_epsilon
max_epsilon34 = sim_data34.max_epsilon
median_epsilon34 = sim_data34.median_epsilon


#%% Test 35
sim_data35 = sim_obj.select_import_data(name35,path,dxmin,dxmax,dymin,dymax,None)
sim_data35.sort_epsilon_and_theta(count_range)
epsilon_section35 = sim_data35.epsilon_section
epsilon_theta_section_max35 = sim_data35.epsilon_theta_section_max
epsilon_theta_section_mean35 = sim_data35.epsilon_theta_section_mean
epsilon_theta_section_median35 = sim_data35.epsilon_theta_section_median
average_epsilon35 = sim_data35.average_epsilon
max_epsilon35 = sim_data35.max_epsilon
median_epsilon35 = sim_data35.median_epsilon

#%% Test 36
sim_data36 = sim_obj.select_import_data(name36,path,dxmin,dxmax,dymin,dymax,None)
sim_data36.sort_epsilon_and_theta(count_range)
epsilon_section36 = sim_data36.epsilon_section
epsilon_theta_section_max36 = sim_data36.epsilon_theta_section_max
epsilon_theta_section_mean36 = sim_data36.epsilon_theta_section_mean
epsilon_theta_section_median36 = sim_data36.epsilon_theta_section_median
average_epsilon36 = sim_data36.average_epsilon
max_epsilon36 = sim_data36.max_epsilon
median_epsilon36 = sim_data36.median_epsilon

#%% Test 37
sim_data37 = sim_obj.select_import_data(name37,path,dxmin,dxmax,dymin,dymax,None)
sim_data37.sort_epsilon_and_theta(count_range)
epsilon_section37 = sim_data37.epsilon_section
epsilon_theta_section_max37 = sim_data37.epsilon_theta_section_max
epsilon_theta_section_mean37 = sim_data37.epsilon_theta_section_mean
epsilon_theta_section_median37 = sim_data37.epsilon_theta_section_median
average_epsilon37 = sim_data37.average_epsilon
max_epsilon37 = sim_data37.max_epsilon
median_epsilon37 = sim_data37.median_epsilon

#%% Test 38
sim_data38 = sim_obj.select_import_data(name38,path,dxmin,dxmax,dymin,dymax,None)
sim_data38.sort_epsilon_and_theta(count_range)
epsilon_section38 = sim_data38.epsilon_section
epsilon_theta_section_max38 = sim_data38.epsilon_theta_section_max
epsilon_theta_section_mean38 = sim_data38.epsilon_theta_section_mean
epsilon_theta_section_median38 = sim_data38.epsilon_theta_section_median
average_epsilon38 = sim_data38.average_epsilon
max_epsilon38 = sim_data38.max_epsilon
median_epsilon38 = sim_data38.median_epsilon

#%% Test 39
sim_data39 = sim_obj.select_import_data(name39,path,dxmin,dxmax,dymin,dymax,None)
sim_data39.sort_epsilon_and_theta(count_range)
epsilon_section39 = sim_data39.epsilon_section
epsilon_theta_section_max39 = sim_data39.epsilon_theta_section_max
epsilon_theta_section_mean39 = sim_data39.epsilon_theta_section_mean
epsilon_theta_section_median39 = sim_data39.epsilon_theta_section_median
average_epsilon39 = sim_data39.average_epsilon
max_epsilon39 = sim_data39.max_epsilon
median_epsilon39 = sim_data39.median_epsilon

#%% Test 40
sim_data40 = sim_obj.select_import_data(name40,path,dxmin,dxmax,dymin,dymax,None)
sim_data40.sort_epsilon_and_theta(count_range)
epsilon_section40 = sim_data40.epsilon_section
epsilon_theta_section_max40 = sim_data40.epsilon_theta_section_max
epsilon_theta_section_mean40 = sim_data40.epsilon_theta_section_mean
epsilon_theta_section_median40 = sim_data40.epsilon_theta_section_median
average_epsilon40 = sim_data40.average_epsilon
max_epsilon40 = sim_data40.max_epsilon
median_epsilon40 = sim_data40.median_epsilon

#%% Test 41
sim_data41 = sim_obj.select_import_data(name41,path,dxmin,dxmax,dymin,dymax,None)
sim_data41.sort_epsilon_and_theta(count_range)
epsilon_section41 = sim_data41.epsilon_section
epsilon_theta_section_max41 = sim_data41.epsilon_theta_section_max
epsilon_theta_section_mean41 = sim_data41.epsilon_theta_section_mean
epsilon_theta_section_median41 = sim_data41.epsilon_theta_section_median
average_epsilon41 = sim_data41.average_epsilon
max_epsilon41 = sim_data41.max_epsilon
median_epsilon41 = sim_data41.median_epsilon


#%% Test 42
sim_data42 = sim_obj.select_import_data(name42,path,dxmin,dxmax,dymin,dymax,None)
sim_data42.sort_epsilon_and_theta(count_range)
epsilon_section42 = sim_data42.epsilon_section
epsilon_theta_section_max42 = sim_data42.epsilon_theta_section_max
epsilon_theta_section_mean42 = sim_data42.epsilon_theta_section_mean
epsilon_theta_section_median42 = sim_data42.epsilon_theta_section_median
average_epsilon42 = sim_data42.average_epsilon
max_epsilon42 = sim_data42.max_epsilon
median_epsilon42 = sim_data42.median_epsilon


#%% Test 43
sim_data43 = sim_obj.select_import_data(name43,path,dxmin,dxmax,dymin,dymax,None)
sim_data43.sort_epsilon_and_theta(count_range)
epsilon_section43 = sim_data43.epsilon_section
epsilon_theta_section_max43 = sim_data43.epsilon_theta_section_max
epsilon_theta_section_mean43 = sim_data43.epsilon_theta_section_mean
epsilon_theta_section_median43 = sim_data43.epsilon_theta_section_median
average_epsilon43 = sim_data43.average_epsilon
max_epsilon43 = sim_data43.max_epsilon
median_epsilon43 = sim_data43.median_epsilon

#%% Test 44
sim_data44 = sim_obj.select_import_data(name44,path,dxmin,dxmax,dymin,dymax,None)
sim_data44.sort_epsilon_and_theta(count_range)
epsilon_section44 = sim_data44.epsilon_section
epsilon_theta_section_max44 = sim_data44.epsilon_theta_section_max
epsilon_theta_section_mean44 = sim_data44.epsilon_theta_section_mean
epsilon_theta_section_median44 = sim_data44.epsilon_theta_section_median
average_epsilon44 = sim_data44.average_epsilon
max_epsilon44 = sim_data44.max_epsilon
median_epsilon44 = sim_data44.median_epsilon

#%% Test 45
sim_data45 = sim_obj.select_import_data(name45,path,dxmin,dxmax,dymin,dymax,None)
sim_data45.sort_epsilon_and_theta(count_range)
epsilon_section45 = sim_data44.epsilon_section
epsilon_theta_section_max45 = sim_data44.epsilon_theta_section_max
epsilon_theta_section_mean45 = sim_data44.epsilon_theta_section_mean
epsilon_theta_section_median45 = sim_data44.epsilon_theta_section_median
average_epsilon45 = sim_data45.average_epsilon
max_epsilon45 = sim_data45.max_epsilon
median_epsilon45 = sim_data45.median_epsilon

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
    #temp7=np.asarray(epsilon_theta_section_max7[str(i)]) ##
    #temp8=np.asarray(epsilon_theta_section_max8[str(i)])
    #temp9=np.asarray(epsilon_theta_section_max9[str(i)])  
    #temp10=np.asarray(epsilon_theta_section_max10[str(i)])   
    #temp11=np.asarray(epsilon_theta_section_max11[str(i)])   
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
    temp25=np.asarray(epsilon_theta_section_max25[str(i)])  
    #temp26=np.asarray(epsilon_theta_section_max26[str(i)])   ##
    #temp27=np.asarray(epsilon_theta_section_max27[str(i)]) ##
    #temp28=np.asarray(epsilon_theta_section_max28[str(i)])  ##
    #temp29=np.asarray(epsilon_theta_section_max29[str(i)])  ##
    temp30=np.asarray(epsilon_theta_section_max30[str(i)])    
    temp31=np.asarray(epsilon_theta_section_max31[str(i)])   
    temp32=np.asarray(epsilon_theta_section_max32[str(i)])   
    temp33=np.asarray(epsilon_theta_section_max33[str(i)])   
    temp34=np.asarray(epsilon_theta_section_max34[str(i)])  
    temp35=np.asarray(epsilon_theta_section_max35[str(i)])  
    temp36=np.asarray(epsilon_theta_section_max36[str(i)])  
    temp37=np.asarray(epsilon_theta_section_max37[str(i)])  
    temp38=np.asarray(epsilon_theta_section_max38[str(i)])  
    temp39=np.asarray(epsilon_theta_section_max39[str(i)])  
    temp40=np.asarray(epsilon_theta_section_max40[str(i)])  
    temp41=np.asarray(epsilon_theta_section_max41[str(i)])  
    temp42=np.asarray(epsilon_theta_section_max42[str(i)])  
    temp43=np.asarray(epsilon_theta_section_max43[str(i)]) 
    temp44=np.asarray(epsilon_theta_section_max44[str(i)]) 
    temp45=np.asarray(epsilon_theta_section_max45[str(i)]) 
    #temp3=[temp,temp2]
    temp=np.concatenate((temp1,
                         temp2,
                         temp3,
                         temp4,
                         temp5,
                         temp6,
                         #temp7,
                         #temp8,
                         #temp9,
                         #temp10,
                         #temp11,
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
                         temp24,
                         temp25,
                         #temp26,
                         #temp27,
                         #temp28,
                         #temp29,
                         temp30,
                         temp31,
                         temp32,
                         temp33,
                         temp34,
                         temp35,
                         temp36,
                         temp37,
                         temp38,
                         temp39,
                         temp40,
                         temp41,
                         temp42,
                         temp43,
                         temp44,
                         temp45))
    #temp3=temp3.flatten()
    max_epsilon_[str(i)].append(temp)
    #max_epsilon_[str(i)].append(epsilon_theta_section_max1[str(i)])

# %%

c="tab:red"
name="searching_square_bar_plot_max"
fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
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
    #for j in range(len(y_)):
        #if y_[j]<10:
        #y__.append(y_[j])
    y.append(y_)
    x=entry*np.ones(len(y_))
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
    print(i)
    if len(x)>1:
        axs2.plot([x[1],x[1]],error,color="k",linewidth=1,zorder=3)
        axs2.scatter(x,y_,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
        #axs2.scatter(x,y__,color=c,marker='s',s=3,zorder=3)
    ymean.append(mean)
positions=[0,1,2,3,4,5,6,7,8,9,10,11]
#positions=[0,1,2,3,4,5,6,7]
axs2.set_xticks(positions)
#axs2.set_yticks([0,1,2,3,4,5])
axs2.set_xticklabels(labels,color='k',fontsize=8)
axs2.bar(labels,ymean,color="tab:blue",zorder=3)
#print(np.max(ymean)-np.min(ymean))
axs2.set_ylabel('$\epsilon$',labelpad=-1,fontsize=9)
axs2.set_xlabel('angle (rad)',labelpad=-2,fontsize=9)
axs2.set_title(r'$(a)$',fontsize=9)
axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# plt.close('all')

# %%
yf=[]
ye1=[]

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


print("mu1=",np.mean(ytemp1))
print("mu2=",np.mean(ytemp2))
print("mu12=",np.mean(ytemp12))
#y#f=np.concatenate((ytemp1,temp3,ytemp5,ytemp7))
#ye=np.concatenate((ytemp2,temp4,ytemp6,ytemp8))


num_bins=30


# %%
name="face"
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
nf, binsf, patchesf = axs.hist(ytemp1,num_bins,density=True,color="tab:grey",zorder=3)
axs.set_xlabel(r'$\epsilon$')
#axs.set_xticks([0,1,2,3,4,5])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=3)
x_=np.linspace(0,2,200)  
mean = statistics.mean(ytemp1)
sd = statistics.stdev(ytemp1)
N=norm.pdf(x_, mean, sd)
#axs.plot(x_,N,color='k',zorder=3)
axs.set_title('(a)')
axs.set_xlabel(r'$\epsilon$')
textstr = '\n'.join((
    r'$\mu=%.2f$' % (mean, ),
    r'$\sigma=%.2f$' % (sd, )))
props = dict(boxstyle='round', facecolor='white', alpha=0.5)
axs.text(0.75, 0.95, textstr, transform=axs.transAxes, fontsize=9,
        verticalalignment='top', bbox=props)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")



name="right_side"
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
nf, binsf, patchesf = axs.hist(ytemp2,num_bins,density=True,color="tab:blue",zorder=3)
axs.set_xlabel(r'$\epsilon$')
#axs.set_xticks([0,1,2,3,4,5])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=3)
mean = statistics.mean(ytemp2)
sd = statistics.stdev(ytemp2)
N=norm.pdf(x_, mean, sd)
#axs.plot(x_,N,color='k',zorder=3)
axs.set_title('(b)')
axs.set_xlabel(r'$\epsilon$')
textstr = '\n'.join((
    r'$\mu=%.2f$' % (mean, ),
    r'$\sigma=%.2f$' % (sd, )))
props = dict(boxstyle='round', facecolor='white', alpha=0.5)
axs.text(0.75, 0.95, textstr, transform=axs.transAxes, fontsize=9,
        verticalalignment='top', bbox=props)

plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")






name="left_side"
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
nf, binsf, patchesf = axs.hist(ytemp12,num_bins,density=True,color="tab:green",zorder=3)
axs.set_xlabel(r'$\epsilon$')
#axs.set_xticks([0,1,2,3,4,5])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=3)
mean = statistics.mean(ytemp12)
sd = statistics.stdev(ytemp12)
N=norm.pdf(x_, mean, sd)
#axs.plot(x_,N,color='k',zorder=3)
axs.set_title('(c)')
axs.set_xlabel(r'$\epsilon$')
textstr = '\n'.join((
    r'$\mu=%.2f$' % (mean, ),
    r'$\sigma=%.2f$' % (sd, )))
props = dict(boxstyle='round', facecolor='white', alpha=0.5)
axs.text(0.75, 0.95, textstr, transform=axs.transAxes, fontsize=9,
        verticalalignment='top', bbox=props)


plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")


#x_=np.linspace(0,5,200)  
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
#plt.close('all')



# fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
# x=np.arange(0,len(ytemp2),1)
# axs.scatter(x,ytemp2,color='tab:blue')
# axs.scatter(x,ytemp12,color='tab:red')

# res1=np.where(ytemp2==0)
# res12=np.where(ytemp12==0)


# print(len(res1[0]))
# print(len(res12[0]))
