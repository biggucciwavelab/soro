# -*- coding: utf-8 -*-




import numpy as np
from bridson import poisson_disc_samples
import math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib import animation
from openopt import LCP
import sys

#np.savez('temp.npz',q,nb=nb,ni=ni,obj,left,right,up,down,R,t0,tend,nsteps)

data = np.load('temp10.npz')


q=data['q']
nb=data['nb']
ni=data['ni']
obj=data['obj']
left=data['left']
right=data['right']
up=data['up']
down=data['down']
R=data['R']
t0=data['t0']
tend=data['tend']
nsteps=data['nsteps']

#nsteps=data['nsteps']
#t0=data['t0']








fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 7)
#xlimits
#Creating border

bx=[left,right,right,left,left]
by=[down,down,up,up,down]
ax = plt.axes(xlim=(left-1., right+1.), ylim=(down-1., up+1.))
lineH = plt.Line2D(bx, by, lw=1)



#Plotting the circles
cPatches,cLines,cInd,Obst=[],[],[],[]
for i in range (0,len(obj)):
    if(obj[i].geom=='circle'):
        ii=obj[i].index
        if i<nb:
            x0,y0,phi0,r=q[3*ii,0],q[3*ii+1,0],q[3*ii+2,0],obj[i].R
            patch = plt.Circle((x0, y0), r, fc='g')

        else:
            x0,y0,phi0,r=q[3*ii,0],q[3*ii+1,0],q[3*ii+2,0],obj[i].R
            patch = plt.Circle((x0, y0), r, fc='r')



        lineC1 = plt.Line2D((x0+r*np.cos(phi0+np.pi), x0+r*np.cos(phi0)), (y0+r*np.sin(phi0+np.pi), y0+r*np.sin(phi0)),color='k')
        lineC2 = plt.Line2D((x0+r*np.cos(phi0+np.pi+np.pi/2), x0+r*np.cos(phi0+np.pi+np.pi/2)), (y0+r*np.sin(phi0+np.pi+np.pi+np.pi/2), y0+r*np.sin(phi0+np.pi+np.pi/2)), color='k')
        cPatches.append(patch)
        cLines.append(lineC1)
        cLines.append(lineC2)
        cInd.append(ii)
    elif(obj[i].geom=='circleObstacle'):

        x0,y0,r=obj[i].x,obj[i].y,obj[i].R
        patch=plt.Circle((x0,y0),r,fc='b')

        Obst.append(patch)



# initialization function: plot the background of each frame
def init():
    ax.add_line(lineH)



    for i in range (0,len(cPatches)):
        ax.add_patch(cPatches[i])
        ax.add_line(cLines[2*i])
        ax.add_line(cLines[2*i+1])
    for i in range(0,len(Obst)):
        ax.add_patch(Obst[i])
    return []

def animationManage(i,cPatches,cLines):
    animatePatches(i,cPatches,cLines)
    animateLine(i,cPatches,cLines)
    return []


def animateLine(i, cPatches, cLines):
    for j in range (0,len(cPatches)):
        x,y,phi,r=q[3*cInd[j],i],q[3*cInd[j]+1,i],q[3*cInd[j]+2,i],obj[cInd[j]].R
        cLines[2*j].set_xdata([x+r*np.cos(phi+np.pi), x+r*np.cos(phi)])
        cLines[2*j].set_ydata([y+r*np.sin(phi+np.pi), y+r*np.sin(phi)])
        cLines[2*j+1].set_xdata([x+r*np.cos(phi+np.pi+np.pi/2), x+r*np.cos(phi+np.pi/2)])
        cLines[2*j+1].set_ydata([y+r*np.sin(phi+np.pi+np.pi/2), y+r*np.sin(phi+np.pi/2)])
    return cLines,


def animatePatches(i,cPatches,cLines):
    for j in range (0,len(cPatches)):
        x,y,phi,r=q[3*cInd[j],i],q[3*cInd[j]+1,i],q[3*cInd[j]+2,i],obj[cInd[j]].R
        cPatches[j].center=(x, y)

    return cPatches,


anim = animation.FuncAnimation(fig, animationManage,
                               init_func=init,
                               frames=nsteps,
                               fargs=(cPatches,cLines,),
                               interval=1,
                               blit=True,
                               repeat=True)

# call the animator.  blit=True means only re-draw the parts that have changed.
#anim = animation.FuncAnimation(fig, animationManage, init_func=init,
#                               frames=nsteps, interval=1, blit=True)


#anim = animation.FuncAnimation(fig, animate, init_func=init,
#                               frames=nsteps, fargs=(q,cPatches, lineH, cLines, cInd), interval=1, blit=True)

plt.show()
