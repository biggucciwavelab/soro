# -*- coding: utf-8 -*-
"""
Created on Mon May 10 00:57:47 2021

@author: qiyua
"""
import numpy as np
import matplotlib.pyplot as plt

bot = 0
nb= 20
dir1 = 'paper/benchmark_100/save_data/'
dir2 = 'paper/benchmark_ol100/save_data/'
#dir2 = '5s/01/save_data/'

time = np.load(dir1+'time.npy')
data1 = np.load(dir1+'pred_err.npy')
data2 = np.load(dir2+'pred_err.npy')
zeros = np.zeros(len(time))
fig, axs = plt.subplots(2,sharex=True,dpi=150)
fig.suptitle('Localization Error, Bot '+str(bot))

titles = ['X Error', 'Y Error']

for i in range(2):
    axs[i].set_title(titles[i])
    axs[i].set_ylabel('Error[cm]')
    axs[i].plot(time,data1[:,i+bot:-1:3*nb],label='Kalman')
    axs[i].plot(time,data2[:,i+bot:-1:3*nb],label='IMU only')
    axs[i].plot(time,zeros,'k:')
    axs[i].legend()
    
com1x = np.mean(data1[:,0:-1:3*nb],axis=1)
com1y = np.mean(data1[:,1:-1:3*nb],axis=1)
com2x = np.mean(data2[:,0:-1:3*nb],axis=1)
com2y = np.mean(data2[:,1:-1:3*nb],axis=1)

fig1, axs1 = plt.subplots(2,sharex=True,dpi=150)
fig1.suptitle('COM error')

titles = ['X Error', 'Y Error']

data1 = [com1x,com1y]
data2 = [com2x,com2y]
for i in range(2):
    axs1[i].set_title(titles[i])
    axs1[i].set_ylabel('Error[cm]')
    axs1[i].plot(time,data1[i],label='Kalman')
    axs1[i].plot(time,data2[i],label='IMU only')
    axs1[i].plot(time,zeros,'k:')
    axs1[i].legend()