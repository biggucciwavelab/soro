#------------------------------------------------------------------------------
# Name:        intestine_test
# Purpose:     simulate mesh skin as particles linked with springs
#
# Author:      Qiyuan Zhou
#
# Created:     09/05/2019
# Copyright:   (c)2019 Wavelab IIT, created with Project Chrono using BSD-3 license
#-----------------------------------------------------------------------------------
# Copyright (c) 2016, Project Chrono Development Team
# All rights reserved.

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
#   in the documentation and/or other materials provided with the distribution. 

# - Neither the name of the nor the names of its contributors may be used 
#   to endorse or promote products derived from this software without specific prior written permission. 

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE.
#-------------------------------------------------------------------------------

#  --------------------------------------------
# |          Imports and definitions           |
#  --------------------------------------------

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


def frange(start, stop, step):
    i = start
    while i<stop:
        yield i
        i += step

# Change this path to "data/" asset path
chrono.SetChronoDataPath("C:/Users/Amin/Documents/chrono-data/")

#  --------------------------------------------
# | Create the simulation system and add items |
#  --------------------------------------------
my_system = chrono.ChSystemNSC() #create the system
my_system.SetSolverType(chrono.ChSolver.Type_MUMPS)
#my_system.SetMaxItersSolverSpeed(150)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED) #time stepper
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) #gravity

#  --------------------------------------------
# |         Settings for the membrane         |      
#  --------------------------------------------

#Size of discrete membrane particles
mem_radius = 10   #millimeters
rho_mem = 2000       #kg/m^3
mem_diaginertia = chrono.ChVectorD(1e-2,1e-2,1e-2) #Rotational mass inertia

#Size of torus
c = 0.1  #[m]: radius from center of hole to center of tube
a = 0.05 #[m]: radius of the tube

#logic check
if (a>=c): 
    print("a cannot be larger than c")
    exit()

membrane_material = chrono.ChMaterialSurfaceNSC()
membrane_material.SetFriction(.4)
membrane_material.SetRollingFriction(10000)
membrane_material.SetSpinningFriction(10000)

#calculated properties
mem_radius = mem_radius/1000
mass_mem = rho_mem*(4/3)*3.1415*(pow(mem_radius,3))

mem=[]       # empty matrix to store membrane objects
i = 0        #particle counter

#  --------------------------------------------
# |           Generating the torus             |      
#  --------------------------------------------
for v in frange(0,2*np.pi-np.arctan(mem_radius/a), 2*np.arctan(mem_radius/a)):
    k=0
    for u in frange(0, 2*np.pi-np.arctan(mem_radius/(c+a*np.cos(v))), 2*np.arctan(mem_radius/(c+a*np.cos(v)))):
       
        x = np.cos(u)*(c+a*np.cos(v))
        y = a*np.sin(v)+1.2*a
        z = np.sin(u)*(c+a*np.cos(v))
        
        part = chrono.ChBody()
        part = chrono.ChBodyEasySphere(mem_radius,rho_mem)
        part.SetPos(chrono.ChVectorD(x,y,z))
        
        part.SetInertiaXX(mem_diaginertia)
        part.SetMaterialSurface(membrane_material)
        part.SetMass(mass_mem)
        part.SetId(i)
        part.GetCollisionModel().ClearModel()
        part.GetCollisionModel().AddSphere(mem_radius) # hemi sizes
        part.GetCollisionModel().BuildModel()
        part.SetCollide(True)
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(i/(n_c*n_a*2), 1, k/(n_a)))     #Green
        part.AddAsset(col_g)
        mem.append(part)
        my_system.Add(part)
        
        k=k+1
#  --------------------------------------------
# |        Generating the environment          |      
#  --------------------------------------------

# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -2, 0 ))
body_floor.SetMaterialSurface(membrane_material)

# Collision shape
body_floor.GetCollisionModel().ClearModel()
body_floor.GetCollisionModel().AddBox(2, .1, 2) # hemi sizes
body_floor.GetCollisionModel().BuildModel()
body_floor.SetCollide(False)

# Visualization shape
body_floor_shape = chrono.ChBoxShape()
body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, .1, 2)
body_floor.GetAssets().push_back(body_floor_shape)

body_floor_texture = chrono.ChTexture()
body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'concrete.jpg')
body_floor.GetAssets().push_back(body_floor_texture)

my_system.Add(body_floor)



# Create the table box

size_table_x = 8 #50*num_x*sphere_radius;
size_table_y = .01*4;
size_table_z = 2 #30*num_z*sphere_radius;

body_table = chrono.ChBody()
body_table.SetPos(chrono.ChVectorD(0, -size_table_y/2, 0 ))
body_table.SetMaterialSurface(membrane_material)

# Collision shape
body_table.GetCollisionModel().ClearModel()
body_table.GetCollisionModel().AddBox(size_table_x/2, size_table_y/2, size_table_z/2) # hemi sizes
body_table.GetCollisionModel().BuildModel()
body_table.SetCollide(True)
body_table.SetBodyFixed(True)

# Visualization shape
body_table_shape = chrono.ChBoxShape()
body_table_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_table_x/2, size_table_y/2, size_table_z/2)
body_table_shape.SetColor(chrono.ChColor(0.1,0.1,0.1))
body_table.GetAssets().push_back(body_table_shape)

body_table_texture = chrono.ChTexture()
body_table_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
body_table.GetAssets().push_back(body_table_texture)

my_system.Add(body_table)

# Collision detection settings
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(mem_radius)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(mem_radius)
my_system.SetMaxPenetrationRecoverySpeed(1)
my_system.SetMinBounceSpeed(0.1) 


#solver settings
#my_system.SetSolverType(chrono.ChSolver.Type_SOR_MULTITHREAD)
#my_system.SetMaxItersSolverSpeed(500)
#my_system.SetMaxItersSolverStab(100)

#  --------------------------------------------
# |           Irrlicht Visualization           |      
#  --------------------------------------------

myapplication = chronoirr.ChIrrApp(my_system,                           #target system
                                   'PyChrono example',                  #window title
                                   chronoirr.dimension2du(1200,800),    #window dimensions
                                   False,                               #fullscreen?
                                   False,                               #shadows?
                                   False)                               #anti-aliasing?
                                   #chronoirr.)                          #graphics driver

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.5,.5,1))
myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                 chronoirr.vector3df(0,0,0),    # aimpoint
                                 6,                 # radius (power)
                                 0,9,               # near, far
                                 80)                # angle of FOV
myapplication.AssetBindAll();
myapplication.AssetUpdateAll();
myapplication.AddShadowAll();

#  --------------------------------------------
# |            Run the Simulation              |      
#  --------------------------------------------
myapplication.SetTimestep(0.000003)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    for substep in range(0,5):
        myapplication.DoStep()
    myapplication.EndScene()
