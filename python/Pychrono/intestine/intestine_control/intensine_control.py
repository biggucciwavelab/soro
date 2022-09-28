# -*- coding: utf-8 -*-
"""
Created on Wed Feb  5 20:00:21 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from torus_objects import *
from myconfig_torus import *
from scipy.optimize import LinearConstraint, Bounds, minimize, linprog

# start timer 
start = timeit.default_timer()

# In[Set Path]
#chrono.SetChronoDataPath("C:/Users/Amin/Documents/chrono-data/")
chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")
# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
# create material
material = Material(mu_f, mu_b, mu_r, mu_s, C_, Ct, Cr, Cs)
# create floor
body_floor = Floor(material, length, tall)
# add floor to system
my_system.Add(body_floor)

# In[Create Robots]
for i in range(nb):
    theta = i * 2 * np.pi / (nb)  # define angle
    x = R1 * np.cos(theta)  # define x position
    y = .5 * height  # define y position
    z = R1 * np.sin(theta)  # defin z position
    # function to create robots 
    (force, my_system, obj, Springs,bots) = BOTS_free(nb, i, x, y, z, theta, diameter, height, rowr, roww,  force, obj, material, body_floor, my_system, Springs, k,rl,botcall,bots)


# Create Goal
GOAL(goal, my_system, diameter)

# stuff for IRRLICHT
myapplication = chronoirr.ChIrrApp(my_system, sim, chronoirr.dimension2du(1600, 1200))
myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.75, 0.75, 1.5))
myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, 2), chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
myapplication.DrawAll
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()
myapplication.AddShadowAll()

count = 0
# Time step

myapplication.SetTimestep(tstep) # set time step
myapplication.SetTryRealtime(False) 

# run sim 
while (myapplication.GetDevice().run()):

    myapplication.BeginScene()
    myapplication.DrawAll()
    print ('time=', my_system.GetChTime())
    # make sure spirng lengths are good
    (Springs,Fm,obj)=fix_spring_lengths(obj,k,rl,rlmax,Springs,t,Fm,nb)
 # controller 4 using the simplex optimization
    #(res, alpha, active, C)=Controller4(my_system, force, obj, mag, goal, nb, active, ta, C, error, nbactive, bots, botcall)
    (res,force)=Controller6(nbactive,bots,goal_d,goal_c,res,force,mag,R1)

#    (res, alpha, active, C) = Controller5(my_system, force, obj, mag, goal, nb, active, ta, C, error)
    t = tstep * count
    # run step
    myapplication.DoStep()
    myapplication.EndScene()
    count = count + 1


    # Close the simulation if time ends
    if t > tend:
        myapplication.GetDevice().closeDevice()

stop = timeit.default_timer()
# In[Print time out]
runtime = stop - start
runtime = runtime * (1 / 60)
print("Total runtime: " + str(runtime) + " minutes")
