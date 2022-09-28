#------------------------------------------------------------------------------
# Name:        balls_in_box_springs
# Purpose:     simulate mesh skin as particles linked with springs
#
# Author:      Qiyuan Zhou
#
# Created:     09/05/2019
# Copyright:   (c) Wavelab IIT
#
#-------------------------------------------------------------------------------


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import numpy as np
import os
import timeit

start = timeit.default_timer()

def frange(start, stop, step):
    i = start
    while i<=stop:
        yield i
        i += step

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")
# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()

my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)

sphere_radius = 0.007 #meters

# Set the default outward/inward shape margins for collision detection,
# this is epecially important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(sphere_radius/5)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(sphere_radius/5)
my_system.SetMaxPenetrationRecoverySpeed(1)
my_system.SetMinBounceSpeed(0.1) 

# Maybe you want to change some settings for the solver. For example you
# might want to use SetMaxItersSolverSpeed to set the number of iterations
# per timestep, etc.

#my_system.SetSolverType(chrono.ChSolver.Type_APGD) # precise, more slow

#my_system.SetMaxItersSolverSpeed(500)
#my_system.SetMaxItersSolverStab(100)

# Create a contact material (surface property)to share between all objects.
# The rolling and spinning parameters are optional - if enabled they double
# the computational time.
brick_material = chrono.ChMaterialSurfaceNSC()
brick_material.SetFriction(.4)
#brick_material.SetDampingF(0.0001)
#brick_material.SetCompliance (0.0003)
#brick_material.SetComplianceT(0.0003)
brick_material.SetRollingFriction(.1)
brick_material.SetSpinningFriction(.1)
#brick_material.SetComplianceRolling(0.0000001)
#brick_material.SetComplianceSpinning(0.0000001)

# Create a contact material (for bots)
bot_material = chrono.ChMaterialSurfaceNSC()
bot_material.SetFriction(.2)
#brick_material.SetDampingF(0.0001)
#brick_material.SetCompliance (0.0003)
#brick_material.SetComplianceT(0.0003)
bot_material.SetRollingFriction(100)
bot_material.SetSpinningFriction(100)
#brick_material.SetComplianceRolling(0.0000001)
#brick_material.SetComplianceSpinning(0.0000001)

my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))

# Create the set of the particle clones (many rigid bodies that
# share the same mass and collision shape, so they are memory efficient
# in c
rho = 4000 #kg/m^3
mass = rho*(4/3)*3.1415*(pow(sphere_radius,3))

#Shell
n_b = 5 #laditude discretization (N-S)
n_c = 5 #Longitude discretization (W-E)
r = .35 #radius of shell

start_b = 10 #Minimum latitude

#Springs
k = 500     #N/m
b = 3       #N*s/m

# constant function for external forces
constfun = chrono.ChFunction_Ramp(.2,.5)

# empty matrix
obj=[]
yeet = True

#particle counter
i = 0

# In[Add Boundary Bots]
for theta in frange(start_b, 180-start_b, n_b): #Theta increment
    
    #Positive half
    for phi in frange(n_c,180,n_c):
        yeet = True
        
        x = r*np.sin(theta*np.pi/180)*np.cos(phi*np.pi/180)
        y = r*np.cos(theta*np.pi/180) + r + .01
        z =  r*np.sin(theta*np.pi/180)*np.sin(phi*np.pi/180)
        
        bot = chrono.ChBody()
        bot = chrono.ChBodyEasySphere(sphere_radius,rho)
        bot.SetPos(chrono.ChVectorD(x,y,z))
        bot.SetMaterialSurface(bot_material)
        bot.SetMass(mass)
        bot.SetId(i)
        bot.GetCollisionModel().ClearModel()
        bot.GetCollisionModel().AddSphere(sphere_radius) # hemi sizes
        bot.GetCollisionModel().BuildModel()
        bot.SetCollide(True)
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))     #Green
        bot.AddAsset(col_g)
        
        
        #Set external force
        #constfun = chrono.ChFunction_Const(1)
        
        if theta in range(50,130) and phi in range(0,100):
            myforcex = chrono.ChForce()
            bot.AddForce(myforcex)
            myforcex.SetMode(chrono.ChForce.FORCE)
            myforcex.SetF_x(constfun)
            myforcex.SetDir(chrono.ChVectorD(1,0,0))
            
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(0.44, .11, 52))     #Purple
            bot.AddAsset(col_g)
        
        
        if (i%(180/n_c))==0:
            yeet=False
        
        if(i==0):
            yeet=True
            
        radial_length = sphere_radius #.25*(np.sqrt(abs(2*(pow(np.sin(n_c*np.pi/180)*r,2)-4*(np.sin(n_c*np.pi/180)*r)*np.cos(n_b*np.pi/180)))))/100
        axial_length = sphere_radius  #.25*(np.sqrt(abs(2*(pow(r,2))-4*r*np.cos(n_c*np.pi/180))))/100
        
        #radial_length = 2*sphere_radius
        #axial_length = radial_length
        
        #radial direction springs (middle)
        if i>=1 and yeet==True:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(obj[i-1], bot, False, chrono.ChVectorD(obj[i-1].GetPos().x,obj[i-1].GetPos().y ,obj[i-1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k)
            ground.Set_SpringR(b)
            ground.Set_SpringRestLength(max(sphere_radius,radial_length))
            '''
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,1,0))    #Yellow
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(sphere_radius/2, 80, 15))
            
                       '''
            my_system.AddLink(ground)
        #Axial direction springs
        if i>=int(360/n_c):
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(obj[i-int(360/n_c)], bot, False, 
                                  chrono.ChVectorD(obj[i-int(360/n_c)].GetPos().x,
                                                       obj[i-int(360/n_c)].GetPos().y ,
                                                       obj[i-int(360/n_c)].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k)
            ground.Set_SpringR(b)
            ground.Set_SpringRestLength(max(sphere_radius,axial_length))
            '''
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,1,0))    #Green
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(sphere_radius/2, 80, 15))
            
'''
            my_system.AddLink(ground)
        my_system.Add(bot)
        obj.append(bot)
        
        i = i + 1
    #Negative half
    for phi in frange(n_c,180,n_c):
        
        x = -r*np.sin(theta*np.pi/180)*np.cos(phi*np.pi/180)
        y = r*np.cos(theta*np.pi/180) + r + .01
        z =  -r*np.sin(theta*np.pi/180)*np.sin(phi*np.pi/180)
        
        bot = chrono.ChBody()
        bot = chrono.ChBodyEasySphere(sphere_radius,rho)
        bot.SetPos(chrono.ChVectorD(x,y,z))
        bot.SetMaterialSurface(bot_material)
        bot.SetMass(mass)
        bot.SetId(i)
        bot.GetCollisionModel().ClearModel()
        bot.GetCollisionModel().AddSphere(sphere_radius) # hemi sizes
        bot.GetCollisionModel().BuildModel()
        bot.SetCollide(True)
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(1, 1, 0))     #Yellow
        bot.AddAsset(col_g)
        
        #Set external force
        #constfun = chrono.ChFunction_Const(1)
        
        if theta in range(50,130) and phi in range(80,180):
            myforcex = chrono.ChForce()
            bot.AddForce(myforcex)
            myforcex.SetMode(chrono.ChForce.FORCE)
            myforcex.SetF_x(constfun)
            myforcex.SetDir(chrono.ChVectorD(1,0,0))
            
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(0.44, .11, 52))     #Purple
            bot.AddAsset(col_g)
        

        #Radial Direction Springs (middle)
        if i>=1:
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(obj[i-1], bot, False, chrono.ChVectorD(obj[i-1].GetPos().x,obj[i-1].GetPos().y ,obj[i-1].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k)
            ground.Set_SpringR(b)
            ground.Set_SpringRestLength(max(sphere_radius,radial_length))
            my_system.AddLink(ground)
            
        #Radial direction springs (ends)
        if (i%(180/n_c))==(180/n_c)-1 and i>=int(2*(180/n_c)-1):
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(obj[i-int(2*(180/n_c)-1)], bot, False, chrono.ChVectorD(
                    obj[i-int(2*(180/n_c)-1)].GetPos().x,obj[i-int(2*(180/n_c)-1)].GetPos().y ,obj[i-int(2*(180/n_c)-1)].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k)
            ground.Set_SpringR(b)
            ground.Set_SpringRestLength(max(sphere_radius,radial_length))
            my_system.AddLink(ground)
        #Axial direction springs
        if i>=int(360/n_c):
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(obj[i-int(360/n_c)], bot, False, 
                                  chrono.ChVectorD(obj[i-int(360/n_c)].GetPos().x,
                                                       obj[i-int(360/n_c)].GetPos().y ,
                                                       obj[i-int(360/n_c)].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k)
            ground.Set_SpringR(b)
            ground.Set_SpringRestLength(max(sphere_radius, axial_length))
            my_system.AddLink(ground)
        #Pole springs
        if i>=int(180/n_c) and (theta==start_b or theta==180-start_b):
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(obj[i-int(180/n_c)], bot, False, 
                                  chrono.ChVectorD(obj[i-int(180/n_c)].GetPos().x,
                                                       obj[i-int(180/n_c)].GetPos().y ,
                                                       obj[i-int(180/n_c)].GetPos().z), chrono.ChVectorD(x,y,z))
            ground.Set_SpringK(k/2)
            ground.Set_SpringR(b)
            ground.Set_SpringRestLength(axial_length)
            my_system.AddLink(ground)
        
        my_system.Add(bot)
        obj.append(bot)
        
        i = i + 1



# In[Create other bodies]

#Particles
# Create the set of the particle clones (many rigid bodies that
# share the same mass and collision shape, so they are memory efficient
# in c
radius = .008
rho2 = 1000 #kg/m^3
mass = rho2*(4/3)*3.1415*(pow(radius,3))
body_bot = chrono.ChParticlesClones()
body_bot.SetMass(mass);
inertia = 2/5*(pow(radius,2))*0.01;
body_bot.SetInertiaXX(chrono.ChVectorD(inertia,inertia,inertia));
body_bot.SetMaterialSurface(brick_material)

# Collision shape (shared by all particle clones) Must be defined BEFORE adding particles
body_bot.GetCollisionModel().ClearModel()
body_bot.GetCollisionModel().AddSphere(radius)
body_bot.GetCollisionModel().BuildModel()
body_bot.SetCollide(True)

#Particles
num_x = 12       #Best if it's an even number
num_y = 12       #Best if it's an even number
num_z = 12      #Best if it's an even number

# add particles
for ix in frange(-num_x/2,num_x/2,1):
    for iy in frange(-num_y/2,num_y/2,1):
        for iz in frange(-num_x/2,num_z/2,1):
            body_bot.AddParticle(chrono.ChCoordsysD(chrono.ChVectorD(ix*(2.01*radius),iy*(2.01*radius)+r, iz*(2.01*radius))))

# Visualization shape (shared by all particle clones)
body_particles_shape = chrono.ChSphereShape()
body_particles_shape.GetSphereGeometry().rad = radius
body_bot.GetAssets().push_back(body_particles_shape)

my_system.Add(body_bot)

#Box
        
x = 1.25
z =  .18
rotate = 9.55

length = 1
height = .25
width = .1

y = height/1.9

body_brick = chrono.ChBody()
body_brick.SetBodyFixed(True)

# set initial position
body_brick.SetPos(chrono.ChVectorD(x,y,z))
body_brick.SetRot(chrono.Q_from_AngY(rotate))

# set mass properties
body_brick.SetMass(.5)
body_brick.SetInertiaXX(chrono.ChVectorD(1,1,1))
# set collision surface properties
body_brick.SetMaterialSurface(brick_material)
# Collision shape
body_brick.GetCollisionModel().ClearModel()
body_brick.GetCollisionModel().AddBox(length/2, height/2, width/2) # must set half sizes
body_brick.GetCollisionModel().BuildModel()
body_brick.SetCollide(True)
# Visualization shape, for rendering animation
body_brick_shape = chrono.ChBoxShape()
body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(length/2, height/2, width/2)
col_brick = chrono.ChColorAsset()
col_brick.SetColor(chrono.ChColor(1,0.1,0.5))     #Pink
body_brick.AddAsset(col_brick)
body_brick.GetAssets().push_back(body_brick_shape)
body_brick.GetAssets().push_back(col_brick)

my_system.Add(body_brick)

#Box2
z =  -z
body_brick1 = chrono.ChBody()
body_brick1.SetBodyFixed(True)
# set initial position
body_brick1.SetPos(chrono.ChVectorD(x,y,z))
body_brick1.SetRot(chrono.Q_from_AngY(-rotate))
# set mass properties
body_brick1.SetMass(.5)
body_brick1.SetInertiaXX(chrono.ChVectorD(1,1,1))
# set collision surface properties
body_brick1.SetMaterialSurface(brick_material)
# Collision shape
body_brick1.GetCollisionModel().ClearModel()
body_brick1.GetCollisionModel().AddBox(length/2, height/2, width/2) # must set half sizes
body_brick1.GetCollisionModel().BuildModel()
body_brick1.SetCollide(True)

body_brick1.GetAssets().push_back(body_brick_shape)
body_brick1.GetAssets().push_back(col_brick)

my_system.Add(body_brick1)

# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -2, 0 ))
body_floor.SetMaterialSurface(brick_material)

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

size_table_x = 10 #50*num_x*sphere_radius;
size_table_y = sphere_radius*4;
size_table_z = 8 #30*num_z*sphere_radius;

body_table = chrono.ChBody()
body_table.SetPos(chrono.ChVectorD(2, -size_table_y/2, 0 ))
body_table.SetMaterialSurface(brick_material)

# Collision shape
body_table.GetCollisionModel().ClearModel()
body_table.GetCollisionModel().AddBox(size_table_x/2, size_table_y/2, size_table_z/2) # hemi sizes
body_table.GetCollisionModel().BuildModel()
body_table.SetCollide(True)

# Visualization shape
body_table_shape = chrono.ChBoxShape()
body_table_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_table_x/2, size_table_y/2, size_table_z/2)
body_table_shape.SetColor(chrono.ChColor(0.1,0.1,0.1))
body_table.GetAssets().push_back(body_table_shape)

body_table_texture = chrono.ChTexture()
body_table_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
body_table.GetAssets().push_back(body_table_texture)

my_system.Add(body_table)

# Create a constraint that blocks free 3 x y z translations and 3 rx ry rz rotations
# of the table respect to the floor, and impose that the relative imposed position
# depends on a specified motion law.

link_shaker = chrono.ChLinkLockLock()
link_shaker.Initialize(body_table, body_floor, chrono.CSYSNORM)
my_system.Add(link_shaker)

# ..create the function for imposed x horizontal motion, etc.
mfunY = chrono.ChFunction_Sine(0,3,0.0)  # phase, frequency, amplitude
link_shaker.SetMotion_Y(mfunY)

# ..create the function for imposed y vertical motion, etc.
mfunZ = chrono.ChFunction_Sine(0,3,0.0)  # phase, frequency, amplitude
link_shaker.SetMotion_Z(mfunZ)

# Note that you could use other types of ChFunction_ objects, or create
# your custom function by class inheritance (see demo_python.py), or also
# set a function for table rotation , etc.

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

 # Perform a short simulation
i=0
while (my_system.GetChTime() < 1.24) :

    my_system.DoStepDynamics(0.0002)

    print ('time=', my_system.GetChTime() )

    # 2) Create the incremental nnnn.dat and nnnn.pov files that will be load
    #    by the pov .ini script in POV-Ray (do this at each simulation timestep)
    if i%50==0:
        pov_exporter.ExportData()
    i=i+1
    
stop = timeit.default_timer()
print('Total Time: ', stop - start) 
