# -*- coding: utf-8 -*-
"""
Created on Mon Jan 20 18:15:07 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from corgi_objects import Material,Floor,Square_BOTS_Spinners,Wall
from myconfig6 import *


start = timeit.default_timer()

# In[Set Path]
chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")

# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_SOR_MULTITHREAD)
#my_system.SetTol(1e-6)
#my_system.SetSolverType(chrono.ChSolver.Type_APGD)
my_system.Set_G_acc(chrono.ChVectorD(0,0, 0))
my_system.SetMaxItersSolverSpeed(300)

material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)
body_floor=Floor(material,length,tall)
my_system.Add(body_floor)

x=0
y=height/2
z=0
# Create bots    
bot = chrono.ChBody()
bot = chrono.ChBodyEasyBox(diameter,height,diameter,rowr)
bot.SetPos(chrono.ChVectorD(x,y,z))
bot.SetMaterialSurface(material)

# give ID number

    # collision model
bot.GetCollisionModel().ClearModel()
bot.GetCollisionModel().AddBox(diameter/2,height/2,diameter/2) # hemi sizes
bot.GetCollisionModel().BuildModel()
bot.SetCollide(True)
bot.SetBodyFixed(False)
pt=chrono.ChLinkMatePlane()
pt.Initialize(body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
my_system.AddLink(pt)

myforce=chrono.ChForce()
bot.AddForce(myforce)
myforce.SetMode(chrono.ChForce.TORQUE)
myforce.SetMforce(.1)
myforce.SetDir(chrono.VECT_Y)
my_system.Add(bot)
obj.append(bot)


myapplication = chronoirr.ChIrrApp(my_system,sim, chronoirr.dimension2du(1600,1200))
myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.75,0.75,1.5))
myapplication.AddLightWithShadow(chronoirr.vector3df(2,5,2),chronoirr.vector3df(2,2,2),10,2,10,120)
myapplication.DrawAll               
myapplication.AssetBindAll();
myapplication.AssetUpdateAll();
myapplication.AddShadowAll();
    
count=0
# Time step
myapplication.SetTimestep(tstep)
myapplication.SetTryRealtime(False)
while(myapplication.GetDevice().run()):
 
    myapplication.BeginScene()
    myapplication.DrawAll()
    print ('time=', my_system.GetChTime())
    t=tstep*count  
    # In[Controller Function]
   
        
    # run step
    myapplication.DoStep()
    myapplication.EndScene()
# Close the simulation if time ends
    if t > tend:
        myapplication.GetDevice().closeDevice()


# In[Print time out]
runtime=stop-start
runtime=runtime*(1/60)
print("Total runtime: "+str(runtime)+" minutes")