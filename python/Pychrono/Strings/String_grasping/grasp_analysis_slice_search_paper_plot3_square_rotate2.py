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

path="D:/dmulroy/Experiments/square_search/"

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
name2 = "22_01_2023_13_50_03"
name3 = "22_01_2023_13_50_10"
name4 = "22_01_2023_13_50_18"
name5 = "23_01_2023_08_43_42"
name6 = "23_01_2023_08_46_57"
name7 = "23_01_2023_08_47_04"
name8 = "23_01_2023_08_47_25"
name9 = "23_01_2023_08_49_03"
name10 = "23_01_2023_08_40_11"
name11 = "23_01_2023_13_40_41"
name12 = "23_01_2023_13_40_48"
name13 = "23_01_2023_13_40_55"
name14 = "23_01_2023_13_41_03"


name15 = "26_01_2023_11_17_07"
name16 = "26_01_2023_11_17_19"
name17 = "26_01_2023_11_17_25"
name18 = "26_01_2023_11_32_41"
name19 = "28_01_2023_12_44_16"
name20 = "28_01_2023_16_02_44"
name21 = "28_01_2023_16_03_00"
name22 = "28_01_2023_17_55_50"
name23 = "28_01_2023_18_03_09"

name24 = "29_01_2023_22_08_13"
name25 = "29_01_2023_22_08_27"
name26 = "29_01_2023_22_09_21"


name27 = "29_01_2023_10_55_00"
name28 = "29_01_2023_10_54_47"
name29 = "29_01_2023_10_54_52"
# %%

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

# %%
name2 = "22_01_2023_13_50_03"
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


# %%
name3 = "22_01_2023_13_50_10"
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


#%%
name4 = "22_01_2023_13_50_18"
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


#%%
name5 = "23_01_2023_08_43_42"
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
name6 = "23_01_2023_08_46_57"
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


#%%
name7 = "23_01_2023_08_47_04"
#Psi=sim_obj.R_functions(name7)  
sim_data7=sim_obj.select_import_data(name7,path,dxmin,dxmax,dymin,dymax,None)
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
name9 = "23_01_2023_08_49_03"
#Psi=sim_obj.R_functions(name9)  
sim_data9=sim_obj.select_import_data(name9,path,dxmin,dxmax,dymin,dymax,None)
sim_data9.sort_epsilon_and_theta(count_range)
epsilon_section9=sim_data9.epsilon_section
epsilon_theta_section_max9=sim_data9.epsilon_theta_section_max
epsilon_theta_section_mean9=sim_data9.epsilon_theta_section_mean
epsilon_theta_section_median9=sim_data9.epsilon_theta_section_median

average_epsilon9=sim_data9.average_epsilon
max_epsilon9=sim_data9.max_epsilon
median_epsilon9=sim_data9.median_epsilon

#%%
name10 = "23_01_2023_08_40_11"
#Psi=sim_obj.R_functions(name10)  
sim_data10=sim_obj.select_import_data(name10,path,dxmin,dxmax,dymin,dymax,None)
sim_data10.sort_epsilon_and_theta(count_range)
epsilon_section10=sim_data10.epsilon_section
epsilon_theta_section_max10=sim_data10.epsilon_theta_section_max
epsilon_theta_section_mean10=sim_data10.epsilon_theta_section_mean
epsilon_theta_section_median10=sim_data10.epsilon_theta_section_median

average_epsilon10=sim_data10.average_epsilon
max_epsilon10=sim_data10.max_epsilon
median_epsilon10=sim_data10.median_epsilon






#%%
name11 = "23_01_2023_13_40_41"#
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
name12 = "23_01_2023_13_40_48"#
#Psi=sim_obj.R_functions(name11)  
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
name13 = "23_01_2023_13_40_55"#
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
name14 = "23_01_2023_13_41_03" # needs running#
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



name15 = "26_01_2023_11_17_07"
name16 = "26_01_2023_11_17_19"
name17 = "26_01_2023_11_17_25"
name18 = "26_01_2023_11_32_41"


#%%
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



#%%

#Psi=sim_obj.R_functions(name19)  
sim_data19=sim_obj.select_import_data(name19,path,dxmin,dxmax,dymin,dymax,None)
sim_data19.sort_epsilon_and_theta(count_range)
epsilon_section19=sim_data19.epsilon_section
epsilon_theta_section_max19=sim_data19.epsilon_theta_section_max
epsilon_theta_section_mean19=sim_data19.epsilon_theta_section_mean
epsilon_theta_section_median19=sim_data19.epsilon_theta_section_median

average_epsilon19=sim_data19.average_epsilon
max_epsilon19=sim_data19.max_epsilon
median_epsilon19=sim_data19.median_epsilon

#%%

#Psi=sim_obj.R_functions(name20)  
sim_data20=sim_obj.select_import_data(name20,path,dxmin,dxmax,dymin,dymax,None)
sim_data20.sort_epsilon_and_theta(count_range)
epsilon_section20=sim_data20.epsilon_section
epsilon_theta_section_max20=sim_data20.epsilon_theta_section_max
epsilon_theta_section_mean20=sim_data20.epsilon_theta_section_mean
epsilon_theta_section_median20=sim_data20.epsilon_theta_section_median

average_epsilon20=sim_data20.average_epsilon
max_epsilon20=sim_data20.max_epsilon
median_epsilon20=sim_data20.median_epsilon

#%%

#Psi=sim_obj.R_functions(name21)  
sim_data21=sim_obj.select_import_data(name21,path,dxmin,dxmax,dymin,dymax,None)
sim_data21.sort_epsilon_and_theta(count_range)
epsilon_section21=sim_data21.epsilon_section
epsilon_theta_section_max21=sim_data21.epsilon_theta_section_max
epsilon_theta_section_mean21=sim_data21.epsilon_theta_section_mean
epsilon_theta_section_median21=sim_data21.epsilon_theta_section_median

average_epsilon21=sim_data21.average_epsilon
max_epsilon21=sim_data21.max_epsilon
median_epsilon21=sim_data21.median_epsilon


#%%
#Psi=sim_obj.R_functions(name22)  
sim_data22=sim_obj.select_import_data(name22,path,dxmin,dxmax,dymin,dymax,None)
sim_data22.sort_epsilon_and_theta(count_range)
epsilon_section22=sim_data22.epsilon_section
epsilon_theta_section_max22=sim_data22.epsilon_theta_section_max
epsilon_theta_section_mean22=sim_data22.epsilon_theta_section_mean
epsilon_theta_section_median22=sim_data22.epsilon_theta_section_median

average_epsilon22=sim_data22.average_epsilon
max_epsilon22=sim_data22.max_epsilon
median_epsilon22=sim_data22.median_epsilon

#%%
#Psi=sim_obj.R_functions(name23)  
sim_data23=sim_obj.select_import_data(name23,path,dxmin,dxmax,dymin,dymax,None)
sim_data23.sort_epsilon_and_theta(count_range)
epsilon_section23=sim_data23.epsilon_section
epsilon_theta_section_max23=sim_data23.epsilon_theta_section_max
epsilon_theta_section_mean23=sim_data23.epsilon_theta_section_mean
epsilon_theta_section_median23=sim_data23.epsilon_theta_section_median

average_epsilon23=sim_data23.average_epsilon
max_epsilon23=sim_data23.max_epsilon
median_epsilon23=sim_data23.median_epsilon

# %%
#Psi=sim_obj.R_functions(name1)  
sim_data24=sim_obj.select_import_data(name24,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data24.EPSILON_
THETA=sim_data24.THETA
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
        
sim_data24.THETA=THETA        
sim_data24.sort_epsilon_and_theta(count_range)

epsilon_section24=sim_data24.epsilon_section
epsilon_theta_section_max24=sim_data24.epsilon_theta_section_max
epsilon_theta_section_mean24=sim_data24.epsilon_theta_section_mean
epsilon_theta_section_median24=sim_data24.epsilon_theta_section_median

average_epsilon24=sim_data24.average_epsilon
max_epsilon24=sim_data24.max_epsilon
median_epsilon24=sim_data24.median_epsilon


# %%
#Psi=sim_obj.R_functions(name1)  
sim_data25=sim_obj.select_import_data(name25,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data25.EPSILON_
THETA=sim_data25.THETA
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
        
sim_data25.THETA=THETA        
sim_data25.sort_epsilon_and_theta(count_range)

epsilon_section25=sim_data25.epsilon_section
epsilon_theta_section_max25=sim_data25.epsilon_theta_section_max
epsilon_theta_section_mean25=sim_data25.epsilon_theta_section_mean
epsilon_theta_section_median25=sim_data25.epsilon_theta_section_median

average_epsilon25=sim_data25.average_epsilon
max_epsilon25=sim_data25.max_epsilon
median_epsilon25=sim_data25.median_epsilon


# %%
#Psi=sim_obj.R_functions(name1)  
sim_data26=sim_obj.select_import_data(name26,path,dxmin,dxmax,dymin,dymax,None)
#sim_data.sort_epsilon_and_theta()
EPSILON_=sim_data26.EPSILON_
THETA=sim_data26.THETA
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
        
sim_data26.THETA=THETA        
sim_data26.sort_epsilon_and_theta(count_range)

epsilon_section26=sim_data26.epsilon_section
epsilon_theta_section_max26=sim_data26.epsilon_theta_section_max
epsilon_theta_section_mean26=sim_data26.epsilon_theta_section_mean
epsilon_theta_section_median26=sim_data26.epsilon_theta_section_median

average_epsilon26=sim_data26.average_epsilon
max_epsilon26=sim_data26.max_epsilon
median_epsilon26=sim_data26.median_epsilon


#%%
#Psi=sim_obj.R_functions(name23)  
sim_data27=sim_obj.select_import_data(name27,path,dxmin,dxmax,dymin,dymax,None)
sim_data27.sort_epsilon_and_theta(count_range)
epsilon_section27=sim_data27.epsilon_section
epsilon_theta_section_max27=sim_data27.epsilon_theta_section_max
epsilon_theta_section_mean27=sim_data27.epsilon_theta_section_mean
epsilon_theta_section_median27=sim_data27.epsilon_theta_section_median

average_epsilon27=sim_data27.average_epsilon
max_epsilon27=sim_data27.max_epsilon
median_epsilon27=sim_data27.median_epsilon



#%%
#Psi=sim_obj.R_functions(name23)  
sim_data28=sim_obj.select_import_data(name28,path,dxmin,dxmax,dymin,dymax,None)
sim_data28.sort_epsilon_and_theta(count_range)
epsilon_section28=sim_data28.epsilon_section
epsilon_theta_section_max28=sim_data28.epsilon_theta_section_max
epsilon_theta_section_mean28=sim_data28.epsilon_theta_section_mean
epsilon_theta_section_median28=sim_data28.epsilon_theta_section_median

average_epsilon28=sim_data28.average_epsilon
max_epsilon28=sim_data28.max_epsilon
median_epsilon28=sim_data28.median_epsilon


#%%
#Psi=sim_obj.R_functions(name23)  
sim_data29=sim_obj.select_import_data(name29,path,dxmin,dxmax,dymin,dymax,None)
sim_data29.sort_epsilon_and_theta(count_range)
epsilon_section29=sim_data29.epsilon_section
epsilon_theta_section_max29=sim_data29.epsilon_theta_section_max
epsilon_theta_section_mean29=sim_data29.epsilon_theta_section_mean
epsilon_theta_section_median29=sim_data29.epsilon_theta_section_median

average_epsilon29=sim_data29.average_epsilon
max_epsilon29=sim_data29.max_epsilon
median_epsilon29=sim_data29.median_epsilon


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
    temp25=np.asarray(epsilon_theta_section_max25[str(i)])
    temp26=np.asarray(epsilon_theta_section_max26[str(i)])
    temp27=np.asarray(epsilon_theta_section_max27[str(i)])
    temp28=np.asarray(epsilon_theta_section_max28[str(i)])
    temp29=np.asarray(epsilon_theta_section_max29[str(i)])    
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
                         temp22,
                         temp23,
                         temp24,
                         temp25,
                         temp26,
                         temp27,
                         temp28,
                         temp29))
    #temp3=temp3.flatten()
    max_epsilon_[str(i)].append(temp)
    #max_epsilon_[str(i)].append(epsilon_theta_section_max1[str(i)])


c="tab:red"
name="searching_square_bar_plot_max"
fig1, axs2 = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
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
        #if y_[j]<5:
        y__.append(y_[j])
    y.append(y__)
    x=entry*np.ones(len(y__))
    #axs2.scatter(x,y__,linewidths = 0.25,edgecolor ="k", color="w",s=5,zorder=5)
    mean = np.mean(y__)
    sd = statistics.stdev(y_)
    error=[mean-sd,mean+sd]
    axs2.plot([x[1],x[1]],error,color="k",linewidth=1,zorder=3)
    #axs2.scatter(entry,np.mean(y__),color=c,marker='s',s=3,zorder=3)
    ymean.append(mean)
#positions=[0,1,2,3,4,5,6,7,8,9,10,11]
positions=[0,1,2,3,4,5,6,7]
axs2.set_xticks(positions)
axs2.set_yticks([0,1,2,3,4,5])
axs2.set_xticklabels(labels,color='k',fontsize=8)
axs2.bar(labels,ymean,color="tab:blue",zorder=3)
print(np.max(ymean)-np.min(ymean))
axs2.set_ylabel('$\epsilon$',labelpad=-1,fontsize=9)
axs2.set_xlabel('angle (rad)',labelpad=-2,fontsize=9)
axs2.set_title(r'$(a)$',fontsize=9)
axs2.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs2.yaxis.grid(True,linewidth=0.1,zorder=3)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# plt.close('all')


yf=[]
ye=[]
ytemp1=y[0]
ytemp2=y[1]
ytemp3=y[2]
ytemp4=y[3]
ytemp5=y[4]
ytemp6=y[5]
ytemp7=y[6]
ytemp8=y[7]


yf=np.concatenate((ytemp1,temp3,ytemp5,ytemp7))
ye=np.concatenate((ytemp2,temp4,ytemp6,ytemp8))


print("mean_faces",np.round(np.mean(yf),2))
print("sigma_faces",np.round(statistics.stdev(yf),2))

print("mean_edges",np.round(np.mean(ye),2))
print("sigma_edges",np.round(statistics.stdev(ye),2))

name="square_face"
num_bins=20
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
n, bins, patches = axs.hist(yf,num_bins,density=True,color="tab:purple",zorder=3)
x_=np.linspace(0,5,200)  
mean = statistics.mean(yf)
sd = statistics.stdev(yf)
N=norm.pdf(x_, mean, sd)
axs.plot(x_,N,color='k',zorder=3)
axs.set_title('(b)',fontsize=9)
axs.set_xlabel(r'$\epsilon$',fontsize=9)
axs.set_xticks([0,1,2,3,4,5])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=3)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
#plt.close('all')

print(len(yf))
print(len(ye))

num_bins=20
name="square_corner"
fig1, axs = plt.subplots(nrows=1, ncols=1,figsize=(3.25,1.5),dpi=300)
n, bins, patches = axs.hist(ye,num_bins,density=True,color="tab:green",zorder=3)
x_=np.linspace(0,5,200)  
mean = statistics.mean(ye)
sd = statistics.stdev(ye)
N=norm.pdf(x_, mean, sd)
axs.plot(x_,N,color='k',zorder=3)
axs.set_title('(c)',fontsize=9)
axs.set_xlabel(r'$\epsilon$',fontsize=9)
axs.set_xticks([0,1,2,3,4,5])
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True,linewidth=0.1,zorder=3)
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
#plt.close('all')