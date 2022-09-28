 # -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 18:26:11 2020

@author: dmulr
"""

import pychrono.core as chrono
from transverse_sim_objects import *
from config import *

# In[Create system]
chrono.SetChronoDataPath(data_path)
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 

shapes=Points_for_shape(shape,p1,p2,nb,diameter,bl,br,Rd,nr)
#shapes.create_RBF()
(rbf,fnx,fny)=shapes.Create_shape_gradient()

# In[Create interiors floor material and robots]
material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)

# create floor
body_floor=Floor(material,length,tall)
my_system.Add(body_floor)

# create robots
boundary=robot(nb,diameter,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,mag,R)
(my_system,Springs,bots,obj,force)=boundary.return_system()

# create interior
inter=Interiors(nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,material,fixed,mode)
(my_system,particles,obj)=inter.return_system()

# create controller
controller=Controls(force,bots,Springs,k,rl,rlmax,type_spring,control_type,mag,fnx,fny,rbf,alpha,beta,nb)

my_rep = MyReportContactCallback()

count=0
#data_contact=Save_contact_data(my_rep,my_system,count)

# Create simulation
simulation=simulate(my_system,boundary,inter,Springs,obj,my_rep,sim,tstep,tend,visual,data_path,controller,lc)
# run simulation
(boundary,time,controller,cx,cy,cz,Fxct,Fyct,Fzct,nc)=simulation.simulate()

data=export_data(boundary,inter,cx,cy,cz,Fxct,Fyct,Fzct,nc,controller,nb,sim,time)



