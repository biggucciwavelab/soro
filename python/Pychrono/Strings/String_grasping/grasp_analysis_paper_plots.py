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
def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

name1 = "03_01_2023_10_45_55" # circle
name2 = "03_01_2023_10_46_12" # square
name3 = "03_01_2023_10_46_20" # triangle

name1 = "04_01_2023_11_15_09" # circle
name2 = "04_01_2023_11_23_36" # square
#name3 = "04_01_2023_11_23_45" # triangle
name3 = "04_01_2023_12_52_26"

#name3 = "05_01_2023_15_27_34"
name3 = "05_01_2023_17_11_46"
name2 = "05_01_2023_15_27_19"
name1 = "05_01_2023_15_27_06"

#### good ones
name3 = "05_01_2023_17_14_08"
name2 = "05_01_2023_17_14_01"
name1 = "05_01_2023_17_13_49"
#### good ones

# name3 = "05_01_2023_17_14_08"
# name2 = "05_01_2023_17_14_01"
# name1 = "05_01_2023_17_13_49"


# name2 = "11_01_2023_13_55_11" # square
# name3 = "11_01_2023_13_55_19" # triangle
# name1 = "11_01_2023_13_54_57" # circle


# name1 = "11_01_2023_19_23_59"
# name3 = "11_01_2023_19_24_33"
# name2 = "11_01_2023_19_24_20"


name1 = "11_01_2023_19_23_59"
name2 = "11_01_2023_21_13_57"
name3 = "11_01_2023_21_14_10"
d=2

snap_shot=False
membrane=True
dxmin=-d
dxmax=d
dymin=-d
dymax=d

Psi=sim_obj.R_functions(name1)  
sim_data1=sim_obj.import_data(name1,path,dxmin,dxmax,dymin,dymax,Psi)

Psi=sim_obj.R_functions(name2)  
sim_data2=sim_obj.import_data(name2,path,dxmin,dxmax,dymin,dymax,Psi)

Psi=sim_obj.R_functions(name3)  
sim_data3=sim_obj.import_data(name3,path,dxmin,dxmax,dymin,dymax,Psi)

# In[extract data]
# %% In[extract data]
epsilon1=sim_data1.EPSILON_
time1=sim_data1.time
PX1=sim_data1.PX
FB1=sim_data1.FB
nn=10
entry1_=244
epsilon1_clean = moving_average(epsilon1, n=nn)
#epsilon1_clean = gaussian_filter1d(epsilon1, 10)

epsilon2=sim_data2.EPSILON_
time2=sim_data2.time
PX2=sim_data2.PX
FB2=sim_data2.FB
nn=10
entry2_=311
epsilon2_clean = moving_average(epsilon2, n=nn)
#epsilon2_clean = gaussian_filter1d(epsilon2, 10)


epsilon3=sim_data3.EPSILON_
PX3=sim_data3.PX
FB3=sim_data3.FB
time3=sim_data3.time
nn=10
entry3_=370
epsilon3_clean = moving_average(epsilon3, n=nn)
#epsilon3_clean = gaussian_filter1d(epsilon3, 10)





# %% In[epsilon_three_shapes_legend]
name="epsilon_three_shapes_alpha_2p75"
fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(6.5,2),dpi=300)
axs.plot(time1,epsilon1,color='tab:red',linewidth=2,alpha=0.5)
axs.plot(time1[nn-1:],epsilon1_clean,color='tab:red',linewidth=2,label='Circle')
#axs.scatter(time1[entry1_],epsilon1[entry1_],marker='o',color='tab:red',edgecolors='k',zorder=3)

axs.plot(time2,epsilon2,color='tab:green',linewidth=2,alpha=0.5)
axs.plot(time2[nn-1:],epsilon2_clean,color='tab:green',linewidth=2,label='Square')
##axs.scatter(time2[entry2_],epsilon2[entry2_],marker='o',color='tab:green',edgecolors='k',zorder=3)

axs.plot(time3,epsilon3,color='tab:blue',linewidth=2,alpha=0.5)
axs.plot(time3[nn-1:],epsilon3_clean,color='tab:blue',linewidth=2,label='Triangle')
#axs.scatter(time3[entry3_],epsilon3[entry3_],marker='o',color='tab:blue',edgecolors='k',zorder=3)

axs.axvline(x = 0, color = 'k')
axs.axvline(x = 6, color = 'k')
axs.axvline(x = 15, color = 'k')
p1 = patches.FancyArrowPatch((0, 7), (6, 7), arrowstyle='<->', mutation_scale=10)
axs.add_patch(p1) 
p1 = patches.FancyArrowPatch((6, 7), (15, 7), arrowstyle='<->', mutation_scale=10)
axs.add_patch(p1) 
#p2 = patches.FancyArrowPatch((1, 0), (0, 1), arrowstyle='<|-|>', mutation_scale=20)
#ax.text(x0-x0b,y0-y0b,str(j),size=8)
#axs.set_title(r'$\epsilon$'+" vs time" )
xticks=[0,5,10,15,20,25,30,35,40,45]#,50,55,60]#,65,70,75,80,85,90]
yticks=[0,2,4,6,8]
axs.set_xticks(np.round(xticks,2))
axs.set_yticks(np.round(yticks,2))
axs.set_ylabel('$\epsilon$',labelpad=-1)
axs.set_xlabel('time [s]',labelpad=-2)
axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
axs.grid(True)
#axs.legend(loc='upper center', bbox_to_anchor=(0.5, -0.3),fancybox=False, shadow=True,frameon=False, ncol=3,fontsize=8)
plt.tight_layout()
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
plt.close('all')





# %% In[create snap shots]
# membrane=True
# directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
# entry1=30
# wxmin=0
# wxmax=4
# wymin=-2
# wymax=2
# fxs=1
# fys=1
# sim_data1.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)
# sim_data2.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)
# sim_data3.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)




# %% In[create snap shots circle]
# membrane=True
# directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
# entry1_=135
# wxmin=0
# wxmax=4
# wymin=-2
# wymax=2
# fxs=1
# fys=1
# sim_data1.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)


# %% In[create snap shots triangle final]
# membrane=True
# directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
# wxmin=1
# wxmax=5
# wymin=-2
# wymax=2
# fxs=1
# fys=1
# sim_data3.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)













# axs.axvline(x = 0, color = 'k')
# axs.axvline(x = 6, color = 'k')
# axs.axvline(x = 15, color = 'k')
# p1 = patches.FancyArrowPatch((0, 7), (6, 7), arrowstyle='<->', mutation_scale=10)
# axs.add_patch(p1) 
# p1 = patches.FancyArrowPatch((6, 7), (15, 7), arrowstyle='<->', mutation_scale=10)
# axs.add_patch(p1) 
# #p2 = patches.FancyArrowPatch((1, 0), (0, 1), arrowstyle='<|-|>', mutation_scale=20)
# #ax.text(x0-x0b,y0-y0b,str(j),size=8)
# #axs.set_title(r'$\epsilon$'+" vs time" )
# # xticks=[0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90]
# # yticks=[0,2,4,6,8]
# # axs.set_xticks(np.round(xticks,2))
# # axs.set_yticks(np.round(yticks,2))
# axs.set_ylabel('$\epsilon$',labelpad=-1)
# axs.set_xlabel('time [s]',labelpad=-2)
# axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.grid(True)
# #axs.legend()
# plt.tight_layout()
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # plt.close('all')














# # %% In[epsilon_three_shapes_legend]
# name="epsilon_three_shapes_legend"
# fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(6.5,2),dpi=300)
# axs.plot(time1,epsilon1,color='tab:red',linewidth=2,alpha=0.5)
# axs.plot(time1[nn-1:],epsilon1_clean,color='tab:red',linewidth=2,label='Circle')
# #axs.scatter(time1[entry1_],epsilon1[entry1_],marker='o',color='tab:red',edgecolors='k',zorder=3)

# axs.plot(time2,epsilon2,color='tab:green',linewidth=2,alpha=0.5)
# axs.plot(time2[nn-1:],epsilon2_clean,color='tab:green',linewidth=2,label='Square')
# ##axs.scatter(time2[entry2_],epsilon2[entry2_],marker='o',color='tab:green',edgecolors='k',zorder=3)

# axs.plot(time3,epsilon3,color='tab:blue',linewidth=2,alpha=0.5)
# axs.plot(time3[nn-1:],epsilon3_clean,color='tab:blue',linewidth=2,label='Triangle')
# #axs.scatter(time3[entry3_],epsilon3[entry3_],marker='o',color='tab:blue',edgecolors='k',zorder=3)

# axs.axvline(x = 0, color = 'k')
# axs.axvline(x = 6, color = 'k')
# axs.axvline(x = 15, color = 'k')
# p1 = patches.FancyArrowPatch((0, 7), (6, 7), arrowstyle='<->', mutation_scale=10)
# axs.add_patch(p1) 
# p1 = patches.FancyArrowPatch((6, 7), (15, 7), arrowstyle='<->', mutation_scale=10)
# axs.add_patch(p1) 
# #p2 = patches.FancyArrowPatch((1, 0), (0, 1), arrowstyle='<|-|>', mutation_scale=20)
# #ax.text(x0-x0b,y0-y0b,str(j),size=8)
# #axs.set_title(r'$\epsilon$'+" vs time" )
# #xticks=[0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90]
# #yticks=[0,2,4,6,8]
# #axs.set_xticks(np.round(xticks,2))
# #axs.set_yticks(np.round(yticks,2))
# axs.set_ylabel('$\epsilon$',labelpad=-1)
# axs.set_xlabel('time [s]',labelpad=-2)
# axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.grid(True)
# axs.legend(loc='upper center', bbox_to_anchor=(0.5, -0.3),fancybox=False, shadow=True,frameon=False, ncol=3,fontsize=8)
# plt.tight_layout()
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # plt.close('all')

# %% In[create snap shots]
# membrane=True
# directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
# entry1=30
# wxmin=0
# wxmax=4
# wymin=-2
# wymax=2
# fxs=1
# fys=1
# sim_data1.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)
# sim_data2.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)
# sim_data3.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)


# # %% In[create snap shots circle final]
# membrane=True
# directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
# entry1=30
# wxmin=0
# wxmax=4
# wymin=-2
# wymax=2
# fxs=1
# fys=1
# sim_data1.create_frames_snapshot(membrane,directory,entry1_,wxmin,wxmax,wymin,wymax,fxs,fys)

# # %% In[create snap shots triangle final]
# membrane=True
# directory="C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"
# wxmin=1
# wxmax=5
# wymin=-2
# wymax=2
# fxs=1
# fys=1
# #sim_data1.create_frames_snapshot(membrane,directory,entry1_,wxmin,wxmax,wymin,wymax,fxs,fys)
# sim_data3.create_frames_snapshot(membrane,directory,entry3_,wxmin,wxmax,wymin,wymax,fxs,fys)
# #sim_data3.create_frames_snapshot(membrane,directory,entry1,wxmin,wxmax,wymin,wymax,fxs,fys)


# # %% In[force v displacement]
# plt.plot([],[],"ro",label="a")
# plt.plot([],[],"bo",label="b")
# plt.legend(frameon=False,loc="upper left")
# plt.annotate(r"$\}$",fontsize=24,
#             xy=(0.27, 0.77), xycoords='figure fraction'
#             )

# plt.plot() 
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+"squig"+".svg")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+"squig"+".pdf")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+"squig"+".eps")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+"squig"+".jpeg")



# # %% In[greenarrows]
# name="arrows"
# fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(6.5,2),dpi=300)
# axs.plot(time1,epsilon1,color='tab:red',linewidth=2,alpha=0.5)
# axs.plot(time1[nn-1:],epsilon1_clean,color='tab:red',linewidth=2,label='Circle')
# #axs.scatter(time1[entry1_],epsilon1[entry1_],marker='o',color='tab:red',edgecolors='k',zorder=3)

# p1 = patches.FancyArrowPatch((0, 7), (6, 7), arrowstyle='<->',color='tab:blue', mutation_scale=10)
# axs.add_patch(p1) 
# p1 = patches.FancyArrowPatch((6, 7), (15, 7), arrowstyle='<->',color='tab:red', mutation_scale=10)
# axs.add_patch(p1) 
# p1 = patches.FancyArrowPatch((15, 7), (20, 7), arrowstyle='<->',color='tab:green', mutation_scale=10)
# axs.add_patch(p1) 

# #p2 = patches.FancyArrowPatch((1, 0), (0, 1), arrowstyle='<|-|>', mutation_scale=20)
# #ax.text(x0-x0b,y0-y0b,str(j),size=8)
# #axs.set_title(r'$\epsilon$'+" vs time" )
# plt.tight_layout()
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".svg")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".pdf")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".eps")
# # plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+name+".jpeg")
# # #plt.close('all')


# %% In[force v displacement]
# fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(2,2),dpi=300)
# axs.plot(FB1,PX1-PX1[74],color='tab:red',linewidth=1.5,label='circle')
# axs.plot(FB2,PX2-PX2[74],color='tab:green',linewidth=1.5,label='square')
# axs.plot(FB3,PX3-PX3[74],color='tab:blue',linewidth=1.5,label='triangle')
# axs.set_ylim([-0.25,1])
# axs.set_xlabel('Pull Force [N]',labelpad=-2)  
# axs.set_ylabel('Ball Position [m]',labelpad=1)
# axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.yaxis.set_tick_params(width=.25,length=2,pad=1) 
# axs.grid(True) 
# axs.legend()
# plt.tight_layout()
       

# fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,2),dpi=300)
# axs.plot(time1,epsilon1_clean,color='r',linewidth=1,label='circle')
# axs.plot(time2,epsilon2_clean,color='g',linewidth=1,label='square')
# axs.plot(time3,epsilon3_clean,color='b',linewidth=1,label='triangle')
# axs.set_title(r'$\epsilon$'+" vs time" )
# axs.set_ylabel('$\epsilon$',labelpad=1)
# axs.set_xlabel('time [s]',labelpad=-2)
# axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
# axs.grid(True)
# axs.legend()
# plt.tight_layout()
# plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.jpg')
# plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.svg')    
# plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.pdf')          
# plt.close('all')
     