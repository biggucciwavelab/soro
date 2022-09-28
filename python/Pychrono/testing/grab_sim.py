"""
Created on Mon Apr 27 18:26:11 2020

@author: dmulr
"""
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from grab_sim_objects import *
from config import *
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator

# In[Create system]
chrono.SetChronoDataPath(data_path)
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 
#chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
#chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


# In[Create interiors floor material and robots]
material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)

# create floor
body_floor=Floor(material,length,tall)
my_system.Add(body_floor)

# create robots
boundary=robot(nb,diameter,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,mag,R,active,actbots,pathind)
(my_system,Springs,bots,obj,force)=boundary.return_system()

# create interior
inter=Interiors(nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,material,fixed,mode,granmode,R )
(my_system,particles,obj,fbound)=inter.return_system()

# create ball
balls=Ball(control_type,my_system,body_floor,obj,material,args)
(my_system)=balls.return_system()

# create controller
controller=Controls(force,bots,particles,fbound,Springs,my_system,k,rl,rlmax,type_spring,mag,nb,actbots,active,control_type,balls,args)

# collect contact points
my_rep = MyReportContactCallback()

# Create simulation
simulation=simulate(my_system,boundary,inter,balls,controller,Springs,obj,my_rep,sim,tstep,tend,visual,data_path)
  
# run simulation
(boundary,time,controller,cx,cy,cz,Fxct,Fyct,Fzct,nc,bodiesA,bodiesB)=simulation.simulate()

## export data
data=export_data(boundary,nb,sim,time,cx,cy,cz,Fxct,Fyct,Fzct,nc,save_data,mr,mp,mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs,tend,inter,balls,controller,shapes,bodiesA,bodiesB )
data.save_variables()


