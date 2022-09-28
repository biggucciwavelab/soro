# -*- coding: utf-8 -*-
"""
Created on Fri May 22 10:19:37 2020

@author: dmulr
"""

import os
import sys
import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np
from numpy import linalg as LA
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import animatplot as amp
from matplotlib import colors as colors
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from scipy.optimize import fsolve
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm 
import scipy.constants  
from diff_robot_objects import *

m_filename = "diff.py"
m_timestep = 0.001
m_length = 1.0
m_visualization = "irrlicht"
#m_datapath = "C:/Program Files/ChronoSolidworks/data/"
m_datapath = "C:/Users/dmulr/OneDrive/Documents/data/"
chrono.SetChronoDataPath(m_datapath)
# Remove the trailing .py and add / in case of file without ./
m_absfilename = os.path.abspath(m_filename)
m_modulename = os.path.splitext(m_absfilename)[0]
#print ("Loading C::E scene...");
exported_items = chrono.ImportSolidWorksSystem(m_modulename)


# Add items to the physical system
my_system = chrono.ChSystemNSC()
# Colors
col_y = chrono.ChColorAsset()
col_y.SetColor(chrono.ChColor(0.44, .11, 52))
exported_items[1].AddAsset(col_y)

for my_item in exported_items:
	my_system.Add(my_item)
    
    
# Optionally set some solver parameters.
#timestepper = chrono.ChTimestepperEulerImplicitProjected()
my_system.SetSolverTolerance(1e-10)
my_solver = chrono.ChSolverBB()
my_system.SetSolver(my_solver)
my_solver.SetMaxIterations(200)
my_system.SetTimestepperType(1) # Euler Implicit Projected
my_solver.EnableWarmStart(True);
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81,0))
chrono.ChCollisionModel_SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel_SetDefaultSuggestedMargin(0.001)

# Default Material
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetDampingF(0.)
material.SetCompliance(0.00001)

# Tire Material
tire_mat = chrono.ChMaterialSurfaceNSC()
tire_mat.SetFriction(0.8)
tire_mat.SetDampingF(0.)
tire_mat.SetCompliance(0.00001)

## create a path
#length=150
#st=np.zeros(length)
#
#for i in range(length):
#    st[i]=i/10
#    
#    
#ax=np.zeros(length)
#ay=np.zeros(length)
#
#for i in range(length):
#    ax[i]=st[i]
#    ay[i]=.5*np.sin(ax[i])


sx=0
sy=0
gx=2
gy=2
AREA_WIDTH=1
rr=.2
reso = 0.13  # potential grid size [m]
KP=10
ETA=1
ox=[.5,1.3,1.5,.75]
oy=[1.5,1.3,.35,.75]
ob=Create_obstacles(ox,oy,my_system,rr)
ob.return_system()
pot=potential_field_path(gx,gy,sx,sy,ox,oy,rr,AREA_WIDTH,reso,KP,ETA)
#pot=potential_field_path(gx,gy,sx,sy,ox,oy,rr,AREA_WIDTH,reso)
pot.calc_potential_field()
#pot.draw_pot_field()
pot.gradient_pot_field()
pot.next_point()
#pot.draw_path_bot()

ax1=np.asarray(pot.xb)
ay1=np.asarray(pot.yb)

path = Path(ax1, ay1)
#s = np.arange(0, path.length, 0.1)
#goal=[ax[-1], ay[-1]]




bot=Create_robot(m_filename,m_visualization,m_datapath,my_system,material)
control=controller(bot,path,my_system,m_timestep)
sim=simulate(m_timestep,my_system,control,bot)
sim.simulate()

(x,z,theta,times,desx,desz,des_theta,times)=sim.return_data()

## Plot the path
#fig, ax = plt.subplots()
#ax.plot(x, z)
#q=ax.quiver(x[0:-1:50], z[0:-1:50], np.cos(theta)[0:-1:50], np.sin(theta)[0:-1:50])
#ax.plot(ax, ay)
#ax.set(xlabel='x', ylabel='y', title='Actual Positions')
#ax.grid()
#ax.axis('equal')
#plt.show()

# plot path
plt.figure(7)
plt.plot(x,z,color='b',label='path_traveled')
plt.plot(ax1,ay1,color='g',linestyle='dashed',label='desired_path')
plt.xlabel("x")
plt.ylabel("y")
plt.grid(True)
plt.title('path of robot')
plt.xlim((min(ax1)-.5,max(ax1)+.5))
plt.ylim((min(ay1)-.5,max(ay1)+.5))
plt.legend()  
plt.show()



# plot obstacles
fig, ax = plt.subplots()
plt.xlim((min(ax1)-.5,max(ax1)+.5))
plt.ylim((min(ay1)-.5,max(ay1)+.5))

plt.grid(linestyle='--')

ax.set_aspect(1)
for i in range(len(ox)):
    ax.add_artist(plt.Circle((ox[i], oy[i]), rr/2, color='r'))

line, = ax.plot(ax1,ay1, color='blue', lw=2)
line, = ax.plot(x,z, color='green',linestyle='dashed', lw=2)

plt.title('How to plot a circle with matplotlib ?', fontsize=8)

plt.savefig("plot_circle_matplotlib_02.png", bbox_inches='tight')

plt.show()
np.savez("2"+".npz",allow_pickle=True,
         x=x,
         z=z,
         theta=theta,
         times=times,
         desx=desx,
         desz=desz,
         des_theta=des_theta)
         