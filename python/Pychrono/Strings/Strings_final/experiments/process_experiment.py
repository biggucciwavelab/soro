
import numpy as np
import math as math
import matplotlib.pyplot as plt
import os
import csv
import timeit
import scipy.constants 
import pandas as pd
from matplotlib import animation
from IPython.display import HTML
from scipy.signal import lfilter
from region_field import *
import matplotlib as mpl
def import_data(path):
    nb=12
    rate=0.2
    filename=path
    data0 = np.genfromtxt(filename,delimiter=',')
    (m,n)=np.shape(data0)
    data0=data0[1:m,:]
    data0=data0[:,3:n]
    time0=data0[:,-1]
    Xe={}
    Ye={}
    theta={}
    times=np.zeros(len(time0))
    for i in range (len(time0)):
        times[i]=rate*i
        
    for i in range(nb):
        Xe["Xe{0}".format(i)]=data0[0:len(time0),3*i]
        Ye["Ye{0}".format(i)]=data0[0:len(time0),3*i+1]
        theta["theta{0}".format(i)]=data0[0:len(time0),3*i+2]
    return(Xe,Ye,times)



def create_animation(Xe,Ye,time0):
    nsteps=len(time0)
    tend=time0[-2]
    FPS = 60
    #framesNum = int(FPS*tend)
    framesNum=len(time0)
    fig = plt.figure()
    fig.set_size_inches(10, 10)

    wid=100
    ax = plt.axes(xlim=(-wid,wid), ylim=(-wid, wid))
    bots=[]
    for i in range(len(Xe)):
        x,y=Xe['Xe'+str(i)][0],Ye['Ye'+str(i)][0]    
        patch = plt.Circle((x, y), 5, fc='g')
        bots.append(patch)

    def init():
        ax.grid(True)
        for i in range (0,len(bots)):
            ax.add_patch(bots[i])

        return []


    def animatePatches(i,bots):
        for j in range (0,len(bots)):
            x,y,r=Xe['Xe'+str(j)][i],Ye['Ye'+str(j)][i],5
            bots[j].center=(x, y)
        return bots,


    def animationManage(i,bots):
        animatePatches(i,bots)
        plt.title('time= ' + str(np.round(time0[i],3)))
        return []


    anim = animation.FuncAnimation(fig, animationManage,init_func=init,frames=nsteps,interval=100,fargs=(bots,))
    return(HTML(anim.to_html5_video()))


def create_snap_shot(Xe,Ye,time0,Z,X,Y,entry,f,titl,xticks,yticks,b):

    import matplotlib.font_manager as fm
    fm._rebuild()
    mpl.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.size'] = 8
    plt.rcParams['axes.linewidth'] = 1      
    fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(1.65*1.287,1.65*1.287),dpi=300)
    #fig.subplots_adjust(left=0.15,bottom=0.12, right=0.9, top=0.85, wspace=0,hspace=0)
    #fig.subplots_adjust(top=.95,bottom=0.15,left=0.2,right=.85,hspace=0,wspace=0)
    #plt.axis('equal')
    axs.axis('scaled')
    im1=axs.contourf(X, Y,np.round(Z,4),cmap = 'jet',levels=100,alpha=1) 
    axs.contour(X, Y,np.round(Z,4),levels = 10,colors=('k',),linestyles=('-',),linewidths=(.1,)) 
    for i in range(len(Xe)):
        x,y=Xe['Xe'+str(i)][entry],Ye['Ye'+str(i)][entry]    
        patch = plt.Circle((x, -y), 5, fc='tab:cyan',alpha=1)
        axs.add_patch(patch)
   # U, S, VT = np.linalg.svd(Z,full_matrices=False)
    #S = np.diag(S)
    #r=5
    #Z = U[:,:r] @ S[0:r,:r] @ VT[:r,:]        # Pseudocolor plot of data and choose colormap
    #axs.contour(X,Y,Z,levels = 60,colors=('k',),linestyles=('-',),linewidths=(.1,))
    #axs.axis('equal')
    #axs.axis('tight')
    axs.set_xticks(xticks)
    axs.set_yticks(yticks)
    axs.set_xlim([-b,b])
    axs.set_ylim([-b,b])
    #axs.set_xlabel('$x$ (cm)',labelpad=1,fontsize=8)
    #axs.set_ylabel('$y$ (cm)',labelpad=1,fontsize=8)
    axs.xaxis.set_tick_params(labelsize=8)
    axs.yaxis.set_tick_params(labelsize=8)
    for c in im1.collections:
        c.set_edgecolor("face")
    #axs.set_title(titl)
    cbar=fig.colorbar(im1, ax=axs,orientation="horizontal")
    cbar.set_ticks([0,.25,.5,.75,1])
    cbar.ax.tick_params(labelsize=8) 
    

# oval:165 R=60
# square:250 R=40
# triangle:360 R=60


R=40
px=0
py=0
b=62
res=.1
shape='Square'
phi=Region_fields(R,px,py,b,res,shape)
#phi.plot_2D(shape)
#phi.plot_2D_lines()
#phi.plot_3D()
#phi.slice_plot(shape,R)
Z=phi.zz
X=phi.xx
Y=phi.yy
f=phi.f

#U, S, VT = np.linalg.svd(Z,full_matrices=False)
#S = np.diag(S)
#r=5
#Z = U[:,:r] @ S[0:r,:r] @ VT[:r,:]  

#path="F:/JAMoEBA/Science_robotics_journal/experimental/good/shapes/ellipse/2021-05-28 14h19/data.csv"

path="F:/JAMoEBA/Science_robotics_journal/experimental/good/shapes/square/2021-05-07 13h22/data.csv"

#path="F:/JAMoEBA/Science_robotics_journal/experimental/good/shapes/triangle/2021-05-26 15h20/data.csv"
(Xe,Ye,time0)=import_data(path)
xticks=[-b,0,b]
yticks=xticks
titl='sQUARE'
create_snap_shot(Xe,Ye,time0,Z,X,Y,250,f,titl,xticks,yticks,b)