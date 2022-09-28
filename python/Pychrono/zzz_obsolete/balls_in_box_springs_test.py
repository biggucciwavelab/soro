#------------------------------------------------------------------------------
# Name:        balls_in_box_test
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
import numpy as np


def frange(start, stop, step):
    i = start
    while i<=stop:
        yield i
        i += step

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("data/")

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

my_system = chrono.ChSystemNSC()
my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

sphere_radius = 0.007 #meters

# Set the default outward/inward shape margins for collision detection,
# this is epecially important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(.01)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(.01)
my_system.SetMaxPenetrationRecoverySpeed(2)
my_system.SetMinBounceSpeed(0.1) 

# Maybe you want to change some settings for the solver. For example you
# might want to use SetMaxItersSolverSpeed to set the number of iterations
# per timestep, etc.

#my_system.SetSolverType(chrono.ChSolver.Type_APGD) # precise, more slow
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
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
brick_material.SetRollingFriction(100)
brick_material.SetSpinningFriction(100)
#brick_material.SetComplianceRolling(0.0000001)
#brick_material.SetComplianceSpinning(0.0000001)

my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))

# Create the set of the particle clones (many rigid bodies that
# share the same mass and collision shape, so they are memory efficient
# in c
rho = 4000 #kg/m^3
mass = rho*(4/3)*3.1415*(pow(sphere_radius,3))

#Shell
n_b = 15 #laditude discretization (N-S)
n_c = 15 #Longitude discretization (W-E)
r = .35 #radius of shell

start_b = 10 #Minimum latitude

#Springs
k = 750     #N/m
b = 3       #N*s/m

# constant function for external forces
constfun = chrono.ChFunction_Ramp(50,10)

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
        bot.SetMaterialSurface(brick_material)
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
        #constfun = chrono.ChFunction_Const(3)
        
        if theta in range(90,130) and phi in range(0,100):
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
        bot.SetMaterialSurface(brick_material)
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
        #constfun = chrono.ChFunction_Const(5)
        
        if theta in range(90,130) and phi in range(0,100):
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
            '''
            #Visualization element
            col3 = chrono.ChColorAsset()
            col3.SetColor(chrono.ChColor(1,0,0))        #Red
            ground.AddAsset(col3)
            ground.AddAsset(chrono.ChPointPointSpring(sphere_radius/2, 80, 15))
            
            '''
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
            '''
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,1,1))    #Orange
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
            ground.Set_SpringRestLength(max(sphere_radius, axial_length))
            '''
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,1,0))    #Green
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(sphere_radius/2, 80, 15))
                    '''
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
            '''
            #Visualization element
            col1 = chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,1,1))    #Green
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(sphere_radius/2, 80, 15))
            '''
            my_system.AddLink(ground)
        
        my_system.Add(bot)
        obj.append(bot)
        
        i = i + 1

#Box
        
x = 1.25
z =  .22
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
col_brick.SetColor(chrono.ChColor(1,0.2,0.5))     #Pink
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

myforcex.SetDir(chrono.ChVectorD(1,0,1))

#Sphere
rad1 = .25   
x = 0
y = rad1+0.01
z =  0

middle = chrono.ChBody()
middle = chrono.ChBodyEasySphere(rad1,rho)
middle.SetPos(chrono.ChVectorD(x,y,z))
middle.SetMaterialSurface(brick_material)
middle.SetMass(1)

middle.GetCollisionModel().ClearModel()
middle.GetCollisionModel().AddSphere(rad1) # hemi sizes
middle.GetCollisionModel().BuildModel()
middle.SetCollide(True)
middle.AddForce(myforcex)

col_b = chrono.ChColorAsset()
col_b.SetColor(chrono.ChColor(0.05, .05, .05))     #Dark
middle.AddAsset(col_b)

my_system.Add(middle)

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

size_table_x = 8 #50*num_x*sphere_radius;
size_table_y = sphere_radius*4;
size_table_z = 2 #30*num_z*sphere_radius;

body_table = chrono.ChBody()
body_table.SetPos(chrono.ChVectorD(0, -size_table_y/2, 0 ))
body_table.SetMaterialSurface(brick_material)

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

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono example', chronoirr.dimension2du(1200,800))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.5,.5,1))
myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                 chronoirr.vector3df(0,r,0),    # aimpoint
                                 6,                 # radius (power)
                                 0,9,               # near, far
                                 80)                # angle of FOV

            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
                        # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
                        # If you need a finer control on which item really needs a visualization proxy in
                        # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

myapplication.AssetBindAll();

                        # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
                        # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll();

            # If you want to show shadows because you used "AddLightWithShadow()'
            # you must remember this:
myapplication.AddShadowAll();

# ---------------------------------------------------------------------
#
#  Run the simulation
#

myapplication.SetTimestep(0.0003)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    for substep in range(0,5):
        myapplication.DoStep()
    myapplication.EndScene()
