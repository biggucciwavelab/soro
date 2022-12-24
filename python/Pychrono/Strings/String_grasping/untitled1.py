# -*- coding: utf-8 -*-
"""
Created on Wed Dec 21 07:06:15 2022

@author: Big Gucci
"""

#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Alessandro Tasora
#
# Created:     1/01/2019
# Copyright:   (c) ProjectChrono 2019
#
#
# This file shows how to
#   - create a small stack of bricks,
#   - create a support that shakes like an earthquake, with motion function
#   - simulate the bricks that fall
#-------------------------------------------------------------------------------


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

my_system = chrono.ChSystemNSC()
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)
#my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # precise, more slow
#my_system.SetSolverMaxIterations(70)

my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
contact_material = chrono.ChMaterialSurfaceNSC()
# brick_material.SetRollingFriction(rollfrict_param)
# brick_material.SetSpinningFriction(0)
# brick_material.SetComplianceRolling(0.0000001)
# brick_material.SetComplianceSpinning(0.0000001)

#------------------------------------------------------------------------------

# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

# body_floor = chrono.ChBody()
# body_floor.SetBodyFixed(True)
# body_floor.SetPos(chrono.ChVectorD(0, -1, 0 ))
# body_floor.SetMaterialSurface(brick_material)

# # Collision shape
# body_floor.GetCollisionModel().ClearModel()
# body_floor.GetCollisionModel().AddBox(10, .1, 10) # hemi sizes
# body_floor.GetCollisionModel().BuildModel()
# body_floor.SetCollide(True)

# # Visualization shape
# body_floor_shape = chrono.ChBoxShape()
# body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(10, .1, 10)
# body_floor.GetAssets().push_back(body_floor_shape)

# body_floor_texture = chrono.ChTexture()
# body_floor_texture.SetTextureFilename(chrono.GetChronoDataFile('concrete.jpg'))
# body_floor.GetAssets().push_back(body_floor_texture)

# my_system.Add(body_floor)



# Create the shaking table, as a box

# size_table_x = 1;
# size_table_y = 0.2;
# size_table_z = 1;

# body_table = chrono.ChBody()
# body_table.SetPos(chrono.ChVectorD(0, -size_table_y/2, 0 ))
# body_table.SetMaterialSurface(brick_material)

# # Collision shape
# body_table.GetCollisionModel().ClearModel()
# body_table.GetCollisionModel().AddBox(size_table_x/2, size_table_y/2, size_table_z/2) # hemi sizes
# body_table.GetCollisionModel().BuildModel()
# body_table.SetCollide(True)

# # Visualization shape
# body_table_shape = chrono.ChBoxShape()
# body_table_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_table_x/2, size_table_y/2, size_table_z/2)
# body_table_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
# body_table.GetAssets().push_back(body_table_shape)

# body_table_texture = chrono.ChTexture()
# body_table_texture.SetTextureFilename(chrono.GetChronoDataFile('concrete.jpg'))
# body_table.GetAssets().push_back(body_table_texture)

# my_system.Add(body_table)


# Create a constraint that blocks free 3 x y z translations and 3 rx ry rz rotations
# of the table respect to the floor, and impose that the relative imposed position
# depends on a specified motion law.

# link_shaker = chrono.ChLinkLockLock()
# link_shaker.Initialize(body_table, body_floor, chrono.CSYSNORM)
# my_system.Add(link_shaker)

# # ..create the function for imposed x horizontal motion, etc.
# mfunY = chrono.ChFunction_Sine(0,1.5,0.001)  # phase, frequency, amplitude
# link_shaker.SetMotion_Y(mfunY)

# # ..create the function for imposed y vertical motion, etc.
# mfunZ = chrono.ChFunction_Sine(0,1.5,0.12)  # phase, frequency, amplitude
# link_shaker.SetMotion_Z(mfunZ)

# Note that you could use other types of ChFunction_ objects, or create
# your custom function by class inheritance (see demo_python.py), or also
# set a function for table rotation , etc.



path="C:/soro/python/Pychrono/Strings/String_grasping/object_file/"
ball = chrono.ChBody()
ball= chrono.ChBodyEasyMesh(chrono.GetChronoDataFile(path+'part6.obj'),7000,True,True, 0.001,True) 
ball.SetPos(chrono.ChVectorD(0.0,0,0))
col_k = chrono.ChColorAsset()
col_k.SetColor(chrono.ChColor(1, 1, 0))
ball.AddAsset(col_k)
my_system.Add(ball)


# path="C:/soro/python/Pychrono/Strings/String_grasping/object_file/"
# ball = chrono.ChBody()
# ball.SetPos(chrono.ChVectorD(0,0,0))
# ball.SetMass(3000)
# #ball.SetInertiaXX(chrono.ChVectorD(0.0344868, 0.0344869, 0.0683785))
#    # ball.SetInertiaXY(chrono.ChVectorD(0,0,0)) 
# ball.SetBodyFixed(False)
# ball.SetCollide(True) # set the collision mode
# # Attach a visualization shape .
# # First load a .obj from disk into a ChTriangleMeshConnected:
# mesh_for_visualization = chrono.ChTriangleMeshConnected()
# mesh_for_visualization.LoadWavefrontMesh((path+'part6.obj'))
# #mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
# visualization_shape = chrono.ChTriangleMeshShape()
# visualization_shape.SetMesh(mesh_for_visualization)
# ball.AddAsset(visualization_shape)

# mesh_for_collision = chrono.ChTriangleMeshConnected()
# mesh_for_collision.LoadWavefrontMesh((path+'part6.obj'))
# # Optionally: you can scale/shrink/rotate the mesh using this:
# #mesh_for_collision.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
# #ball.GetCollisionModel().ClearModel()
# ball.GetCollisionModel().AddTriangleMesh(
# mesh_for_collision, # the mesh 
# False,  # is it static?
# False)  # is it convex?
# ball.GetCollisionModel().BuildModel()                  

# # rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
# #rotation1.Q_from_AngAxis(np.pi/2, chrono.ChVectorD(0, 1, 0)) 
# #ball.SetRot(rotation1)
# col_y = chrono.ChColorAsset() # apply color
# col_y.SetColor(chrono.ChColor(1, 1, 0))
# ball.AddAsset(col_y)

# my_system.Add(ball)      


  



# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono example', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(0.5,0.5,1.0))
myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                 chronoirr.vector3df(0,0,0),    # aimpoint
                                 9,                 # radius (power)
                                 1,9,               # near, far
                                 30)                # angle of FOV

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

myapplication.SetTimestep(0.001)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    for substep in range(0,5):
        myapplication.DoStep()
    myapplication.EndScene()

