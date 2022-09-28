import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from sphero_object import *
from config import *
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator


# In[Create system]
chrono.SetChronoDataPath(data_path)
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 


# In[Create interiors floor material and robots]
material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)

# create floor
body_floor=Floor(material,length,tall)
my_system.Add(body_floor)

# create robots
boundary=robot(nb,diameter,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,mag,R)
(my_system,Springs,bots,obj,force)=boundary.return_system()

# create interior
inter=Interiors(nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,material,fixed,mode,granmode)
(my_system,particles,obj,fbound)=inter.return_system()

# Balls 
balls=Ball(Rb,height,rowb,xball,zball,my_system,body_floor,obj,material)
(obj,bforce,my_system,ballss)=balls.return_system()

# shapes
shapes=Points_for_shape(shape,zball,xball,nb,diameter,bl,br,R,nr,Rd)
(rbf,fnx,fny)=shapes.Create_shape_gradient()

# create controller
controller=Controls(force,bots,particles,fbound,Springs,balls,my_system,k,rl,rlmax,type_spring,control_type,mag,shapes,alpha,beta,nb)
controller.set_ball_parameters(xbstop,zbstop,tball,bforce,Fballx,Fballz)

# collect contact points
my_rep = MyReportContactCallback()

# Create simulation
simulation=simulate(my_system,boundary,inter,Springs,balls,obj,my_rep,shapes,sim,tstep,tend,visual,data_path,controller)

# run simulation
(boundary,time,controller,cx,cy,cz,Fxct,Fyct,Fzct,nc)=simulation.simulate()

# export data
#data=export_data(boundary,inter,balls,cx,cy,cz,Fxct,Fyct,Fzct,nc,controller,nb,sim,time,shapes,save_data,mr,mp)
#data.save_variables()
#(XL,ZL)=boundary.return_last_position()

