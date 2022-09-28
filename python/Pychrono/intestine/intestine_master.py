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
import pychrono.postprocess as postprocess
import numpy as np
import os
import timeit

start = timeit.default_timer()

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
#my_system.SetSolverType(chrono.ChSolver.Type_SOR_MULTITHREAD)
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
c = 0.15  #[m]: radius from center of hole to center of tube
a = 0.05 #[m]: radius of the tube

#Discretization of torus
n_c = 10 #Half the discretizations in large structure (X-Z plane)
n_a = 4 #Half the discretizations in torus tube (along cirumference of small tube)

#Springs
k1 = 750     #[N/m]: Straight springs
b1 = 3       #[N*s/m]: Straight springs
k2 = 500     #[N/m]: Diagonal springs
b2 = 2       #[N*s/m]: Diagonal springs

#logic check
if (a>=c): 
    print("a cannot be larger than c")
    exit()

#Contact material for membrane
membrane_material = chrono.ChMaterialSurfaceNSC()
membrane_material.SetFriction(.4)
#membrane_material.SetDampingF(0.0001)
#membrane_material.SetCompliance (0.0003)
#membrane_material.SetComplianceT(0.0003)
membrane_material.SetRollingFriction(10000)
membrane_material.SetSpinningFriction(10000)
#membrane_material.SetComplianceRolling(0.0000001)
#membrane_material.SetComplianceSpinning(0.0000001)

#calculated properties
mem_radius = mem_radius/1000
mass_mem = rho_mem*(4/3)*3.1415*(pow(mem_radius,3))

mem=[]       # empty matrix to store membrane objects
yeet = True  #boolean for spring logic
i = 0        #particle counter

#  --------------------------------------------
# |         Settings for the big bots          |      
#  --------------------------------------------
bot_radius = 0.028575 #m
rho_bot = 1083        #kg/m^3
mass_bot = 0.424      #kg
h_bot = 0.09492       #m
n_b = 8               #number of bots

#inertia tensor components (g*mm^2)
bot_ixx = (1e-9)*118636;   bot_ixy = -(1e-9)*1222; bot_ixz = (1e-9)*628
bot_iyy = (1e-9)*125008; bot_iyz = (1e-9)*-490
bot_izz = (1e-9)*52824

#bot_I1 = (1e-3)*301.399; bot_I2 = (1e-3)*116.949; bot_I3 = (1e-3)*244.9

bot_diaginertia = chrono.ChVectorD(bot_ixx, bot_iyy, bot_izz)
bot_offdiag_inertia = chrono.ChVectorD(bot_ixy, bot_ixz, bot_iyz)

#Contact material for big bots
bot_material = chrono.ChMaterialSurfaceNSC()
bot_material.SetFriction(.4)
#bot_material.SetDampingF(0.0001)
#bot_material.SetCompliance (0.0003)
#bot_material.SetComplianceT(0.0003)
bot_material.SetRollingFriction(.05)
bot_material.SetSpinningFriction(.05)
#bot_material.SetComplianceRolling(0.0000001)
#bot_material.SetComplianceSpinning(0.0000001)

big_bot=[]   #empty matrix to store membrane objects
j = 0        #bot counter

#  --------------------------------------------
# |         Settings for the interior          |      
#  --------------------------------------------

#Size of discrete interior particles
int_radius = 10    #millimeters
mass_int = 0.95     #g
int_diaginertia = chrono.ChVectorD(60.27*(1e-8),60.27*(1e-8),60.27*(1e-8)) #Rotational mass inertia

#Contact material for interior
interior_material = chrono.ChMaterialSurfaceNSC()
interior_material.SetFriction(.4)
#interior_material.SetDampingF(0.0001)
#interior_material.SetCompliance (0.0003)
#interior_material.SetComplianceT(0.0003)
interior_material.SetRollingFriction(.05)
interior_material.SetSpinningFriction(.05)
#interior_material.SetComplianceRolling(0.0000001)
#interior_material.SetComplianceSpinning(0.0000001)

#Calculated properties
int_radius = int_radius/1000
mass_int = mass_int/1000 #kg

interior_bot = []   #empty matrix to store interior objects
k= 0                #interior counter

#  --------------------------------------------
# |         Generating the big bots            |      
#  --------------------------------------------
'''
for t in frange (0, 2*np.pi, 2*np.pi/n_b):
    
    x = c*np.cos(t)
    y = h_bot+0.001
    z = c*np.sin(t)

    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyCylinder(bot_radius,h_bot,rho_bot)
    bot.SetPos(chrono.ChVectorD(x,y,z))
    
    bot.SetInertiaXX(bot_diaginertia)
    bot.SetInertiaXY(bot_offdiag_inertia)
    bot.SetMass(mass_bot)
    bot.SetId(j)
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddCylinder(bot_radius,bot_radius, h_bot/2)
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    col_g = chrono.ChColorAsset()
    col_g.SetColor(chrono.ChColor(0, 1, 0))     #Green
    bot.AddAsset(col_g)
    
    my_system.Add(bot)
    big_bot.append(bot)
    
    j=j+1
'''
#  --------------------------------------------
# |           Generating the torus             |      
#  --------------------------------------------
for u in frange(0, 2*np.pi, np.pi/n_c):
    k=0
    for v in frange(0,2*np.pi-np.pi/n_a,np.pi/n_a):
        yeet=True
       
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
        col_g.SetColor(chrono.ChColor(i/80, 1, k/(2*n_a)))     #Green
        part.AddAsset(col_g)
        
        k=k+1
        
        #Set external force
        #constfun = chrono.ChFunction_Const(3)
        '''
        if theta in range(90,130) and phi in range(0,100):
            myforcex = chrono.ChForce()
            part.AddForce(myforcex)
            myforcex.SetMode(chrono.ChForce.FORCE)
            myforcex.SetF_x(constfun)
            myforcex.SetDir(chrono.ChVectorD(1,0,0))
            
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(0.44, .11, 52))     #Purple
            part.AddAsset(col_g)
        '''
        
        #  --------------------------------------------
        # |               Adding Springs               |      
        #  --------------------------------------------
        if (i%(2*n_a))==0:
            yeet=False
        
        if(i==0):
            yeet=True
            
        #small circle direction (k1)
        if i>=1 and yeet==True:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-1], part, False, chrono.ChVectorD(mem[i-1].GetPos().x,mem[i-1].GetPos().y ,mem[i-1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k1)
            ground.Set_SpringR(b1)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,1,0))    #Yellow
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 15))
            '''
            my_system.AddLink(ground)
        
        #small circle direction (k1) (ends)
        if i>=1 and ((i+1)%(2*n_a))==0:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-2*n_a+1], part, False, chrono.ChVectorD(mem[i-2*n_a+1].GetPos().x,mem[i-2*n_a+1].GetPos().y ,mem[i-2*n_a+1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k1)
            ground.Set_SpringR(b1)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0.2,0.2,0))    #Yellow
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius, 80, 15))
            '''
            
            my_system.AddLink(ground)
                        
        #large circle direction (k1)
        if i>=2*n_a:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-2*n_a], part, False, chrono.ChVectorD(mem[i-2*n_a].GetPos().x,mem[i-2*n_a].GetPos().y ,mem[i-2*n_a].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k1)
            ground.Set_SpringR(b1)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,1,0))    #Green
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 15))
            '''
            my_system.AddLink(ground)
            
        #large circle direction (k1) (ends)
        if i>=(4*n_c*(n_a)-2*n_a):
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-(4*n_c*(n_a)-2*n_a)], part, False, chrono.ChVectorD(mem[i-(4*n_c*(n_a)-2*n_a)].GetPos().x,mem[i-(4*n_c*(n_a)-2*n_a)].GetPos().y ,mem[i-(4*n_c*(n_a)-2*n_a)].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k1)
            ground.Set_SpringR(b1)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,0,0))    #Green
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius, 80, 15))
            '''
            my_system.AddLink(ground)
            
            
        #Diagonal Springs (k2)
        if i>=2*n_a and ((i+1)%(2*n_a))!=0:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-2*n_a+1], part, False, chrono.ChVectorD(mem[i-2*n_a+1].GetPos().x,mem[i-2*n_a+1].GetPos().y ,mem[i-2*n_a+1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k2)
            ground.Set_SpringR(b2)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,.2,.2))    #Red
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 30))
            '''
            
            my_system.AddLink(ground)
            
        #Diagonal Springs (k2) (ends, small circle)
        if i>=4*(n_a-1) and ((i+1)%(2*n_a))==0:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-4*n_a+1], part, False, chrono.ChVectorD(mem[i-4*n_a+1].GetPos().x,mem[i-4*n_a+1].GetPos().y ,mem[i-4*n_a+1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k2)
            ground.Set_SpringR(b2)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,.2,.2))    #Red
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 30))
            '''
            
            my_system.AddLink(ground)
            
        #Diagonal Springs (k2) (ends, large circle)
        if i>=(4*n_c*(n_a)-2*n_a) and i<(4*n_c*n_a-1):
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-(4*n_c*(n_a)-2*n_a)+1], part, False, chrono.ChVectorD(mem[i-(4*n_c*(n_a)-2*n_a)+1].GetPos().x,mem[i-(4*n_c*(n_a)-2*n_a)+1].GetPos().y ,mem[i-(4*n_c*(n_a)-2*n_a)+1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k2)
            ground.Set_SpringR(b2)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,0.1,.1))    #Red
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 15))
            '''
            
            my_system.AddLink(ground)
            

        #Diagonal Springs (k2)
        if i>2*n_a and yeet==True:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-2*n_a-1], part, False, chrono.ChVectorD(mem[i-2*n_a-1].GetPos().x,mem[i-2*n_a-1].GetPos().y ,mem[i-2*n_a-1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k2)
            ground.Set_SpringR(b2)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(.1,0.1,.1))    #Red
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 15))
            '''
            
            my_system.AddLink(ground)
            
        #Diagonal Springs (k2) (ends, small circle)
        if i>1 and yeet==False:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-1], part, False, chrono.ChVectorD(mem[i-1].GetPos().x,mem[i-1].GetPos().y ,mem[i-1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k2)
            ground.Set_SpringR(b2)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(.1,0.1,.1))    #Red
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 15))
            '''
            
            my_system.AddLink(ground)
            
        #Diagonal Springs (k2) (ends, large circle)
        if i>(4*n_c*(n_a)-2*n_a):
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(mem[i-(4*n_c*(n_a)-2*n_a)-1], part, False, chrono.ChVectorD(mem[i-(4*n_c*(n_a)-2*n_a)-1].GetPos().x,mem[i-(4*n_c*(n_a)-2*n_a)-1].GetPos().y ,mem[i-(4*n_c*(n_a)-2*n_a)-1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k2)
            ground.Set_SpringR(b2)
            ground.Set_SpringRestLength(mem_radius/2)
            '''
          
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,0.1,.1))    #Red
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(mem_radius/2, 80, 15))
            '''
            
            my_system.AddLink(ground)
            

        my_system.Add(part)
        mem.append(part)
        
        i = i + 1

print("Number of boundary particles: \n", len(mem))
#  --------------------------------------------
# |          Generating the interior           |      
#  --------------------------------------------

# Create the set of the particle clones (many rigid bodies that
# share the same mass and collision shape, so they are memory efficient
# in c
interior = chrono.ChParticlesClones()
interior.SetMass(mass_int);
interior.SetInertiaXX(int_diaginertia);
interior.SetMaterialSurface(interior_material)

# Collision shape (shared by all particle clones) Must be defined BEFORE adding particles
interior.GetCollisionModel().ClearModel()
interior.GetCollisionModel().AddSphere(int_radius)
interior.GetCollisionModel().BuildModel()
interior.SetCollide(True)

int_a = 1
int_c = 0.95*np.pi*c/int_radius

aa=a
a=0
u=0
v=0

for a in frange(0,(aa-2*int_radius)/2,1.5*int_radius):
    int_a=0.95*np.pi*a/int_radius
    if int_a==0:
        int_a= 1
    for u in frange(0, 2*np.pi, 2*np.pi/int_c):
        for v in frange(0,2*np.pi,2*np.pi/int_a):
       
            x = np.cos(u)*(c+a*np.cos(v))
            y = a*np.sin(v)+1.2*aa
            z = np.sin(u)*(c+a*np.cos(v))
        
            interior.AddParticle(chrono.ChCoordsysD(chrono.ChVectorD(x,y,z)))
            interior_bot.append(interior)


# Visualization shape (shared by all particle clones)
interior_shape = chrono.ChSphereShape()
interior_shape.GetSphereGeometry().rad = int_radius
interior.GetAssets().push_back(interior_shape)

my_system.Add(interior)
print("Number of interior particles: \n", len(interior_bot))
#  --------------------------------------------
# |        Generating the environment          |      
#  --------------------------------------------

# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -2, 0 ))
body_floor.SetMaterialSurface(interior_material)

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
body_table.SetMaterialSurface(interior_material)

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
# |               POV-Ray Export               |      
#  --------------------------------------------

pov_exporter = postprocess.ChPovRay(my_system)

# Sets some file names for in-out processes.
pov_exporter.SetTemplateFile        ("D:/WaveLab/Soft Robotics/Chrono/data/_template_POV.pov")
pov_exporter.SetOutputScriptFile    ("rendering_frames.pov")
pov_exporter.SetOutputDataFilebase  ("my_state")
pov_exporter.SetPictureFilebase     ("picture")
 # Save the .dat files and the .bmp files in two subdirectories,
 # to avoid cluttering the current directory...
if not os.path.exists("output"):
    os.mkdir("output")
if not os.path.exists("anim"):
    os.mkdir("anim")
pov_exporter.SetOutputDataFilebase("output/my_state")
pov_exporter.SetPictureFilebase("anim/picture")

 # Tell selectively which physical items you want to render, or use AddAll()
pov_exporter.AddAll()

 # 1) Create the two .pov and .ini files for POV-Ray (this must be done
 #    only once at the beginning of the simulation).
pov_exporter.ExportScript()

#  --------------------------------------------
# |            Run the Simulation              |      
#  --------------------------------------------

f=0
while (my_system.GetChTime() < .25) :

    my_system.DoStepDynamics(0.0002)

    print ('time=', my_system.GetChTime() )

    # 2) Create the incremental nnnn.dat and nnnn.pov files that will be load
    #    by the pov .ini script in POV-Ray (do this at each simulation timestep)
    if f%50==0:
        pov_exporter.ExportData()
    f=f+1

stop = timeit.default_timer()
print('Total Time: ', stop - start) 