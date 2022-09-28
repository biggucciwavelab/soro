# -*- coding: utf-8 -*-
"""
Created on Wed Jun 24 12:22:57 2020

@author: dmulr
"""
import pychrono.core as chrono
import numpy as np
from transverse_sim_objects import *
from config import *

chrono.SetChronoDataPath(data_path)
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 

# add material
material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)
# body floor 
body_floor=Floor(material,length,tall)
# add body floor
my_system.Add(body_floor)


botss=single_bot(diameter,rowr,height,my_system,obj,body_floor,material)
(my_system,bots,obj,force)=botss.return_system()

(ax,ay)=create_sine_path()

#(ax,ay)=create_parabola()
path=Path(ax,ay)
ref_pt = np.array([1,1])
controller=Controls(force,botss,my_system,control_type,mag,path,ref_pt,tstep)
sims=simulate(my_system,botss,obj,sim,controller,tstep,tend,visual,data_path)
sims.simulate()

X=sims.x
Z=sims.z
T=sims.time
phi=sims.phi
U=sims.U
plt.plot(ay,ax,'g',Z,X,'b')
plt.xlim((0,length))
plt.ylim((0,length))

#plt.figure(figsize=(10,10))
#ax1 = plt.subplot(3,1,1)
#ax1.grid(True)
#plt.gca().set_title('x')
#plt.plot(T, X,'b')
#    
#ax2 = plt.subplot(3,1,2)
#plt.gca().set_title('y')
#plt.plot(T, Z,'r')
#ax2.grid(True)
#            
#ax3 = plt.subplot(3,1,3)
#plt.gca().set_title('phi')
#plt.plot(T, phi,'g')
#ax3.grid(True)
#plt.subplots_adjust(hspace = 1)
#plt.xlabel('time (seconds)')
#
#plt.show()
