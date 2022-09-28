
# In[ importing the module]

import numpy as np
import math
import pychrono as chrono
from pychrono import irrlicht as chronoirr


# In[proving the formula is wright]
#Lets test for the Aluminium material of
G = 27*10**9
E = 68.3*10**9
L = 0.1
A = 0.05*0.05

def K1(E,G,A,L):
    k1 = (A*(E-G))/(2*L)
    return k1

def K2(G,A,L):
    k2 = (G*A)/(np.sqrt(2)*L)
    return k2

a = K1(E,G,A,L)
b = K2(A,G,L)

# In[ creating the chrono system]
chrono.SetChronoDataPath('C:/codes/Chrono/Chrono_Source/data/')
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0,0,0))


# In[create ground]

ground = chrono.ChBody()
system.AddBody(ground)
ground.SetIdentifier(-1)
ground.SetName("ground")
ground.SetBodyFixed(True)



# In[ creating a shphere]


sphere1 = chrono.ChBody()
system.AddBody(sphere1)
sphere1.SetIdentifier(1)
sphere1.SetName("roller1")
sphere1.SetPos(chrono.ChVectorD(0,0,0))
sphere1.SetMass(1.0)
sphere1.SetInertiaXX(chrono.ChVectorD(0.001,0.001,0.001))
sphere1.SetRot(chrono.ChQuaternionD(1,0,0,0))
        
sph1_b = chrono.ChSphereShape()
sph1_b.GetSphereGeometry().center = chrono.ChVectorD(0, 0, 0)
sph1_b.GetSphereGeometry().rad = 0.2
sphere1.AddAsset(sph1_b)
        
col1_b = chrono.ChColorAsset()
col1_b.SetColor(chrono.ChColor(0.1,0.8,0.5))
sphere1.AddAsset(col1_b)
        



sphere2 = chrono.ChBody()
system.AddBody(sphere2)
sphere2.SetIdentifier(2)
sphere2.SetName("roller2")
sphere2.SetPos(chrono.ChVectorD(0,0,0))
sphere2.SetMass(1.0)
sphere2.SetInertiaXX(chrono.ChVectorD(0.001,0.001,0.001))
sphere2.SetRot(chrono.ChQuaternionD(1,0,0,0))
        
sph2_b = chrono.ChSphereShape()
sph2_b.GetSphereGeometry().center = chrono.ChVectorD(1, 0, 0)
sph2_b.GetSphereGeometry().rad = 0.2
sphere2.AddAsset(sph2_b)
        
col3_b = chrono.ChColorAsset()
col3_b.SetColor(chrono.ChColor(0.5,0.8,0.7))
sphere2.AddAsset(col3_b)





sphere3 = chrono.ChBody()
system.AddBody(sphere3)
sphere3.SetIdentifier(3)
sphere3.SetName("roller3")
sphere3.SetPos(chrono.ChVectorD(0,0,0))
sphere3.SetMass(1.0)
sphere3.SetInertiaXX(chrono.ChVectorD(0.001,0.001,0.001))
sphere3.SetRot(chrono.ChQuaternionD(1,0,0,0))
sphere3.SetBodyFixed(True)
        
sph3_b = chrono.ChSphereShape()
sph3_b.GetSphereGeometry().center = chrono.ChVectorD(0, 0, -1)
sph3_b.GetSphereGeometry().rad = 0.2
sphere3.AddAsset(sph3_b)
        
col3_b = chrono.ChColorAsset()
col3_b.SetColor(chrono.ChColor(0,0.8,0.2))
sphere3.AddAsset(col3_b)
     


sphere4 = chrono.ChBody()
system.AddBody(sphere4)
sphere4.SetIdentifier(4)
sphere4.SetName("roller4")
sphere4.SetPos(chrono.ChVectorD(0,0,0))
sphere4.SetMass(1.0)
sphere4.SetInertiaXX(chrono.ChVectorD(0.001,0.001,0.001))
sphere4.SetRot(chrono.ChQuaternionD(1,0,0,0))
sphere4.SetBodyFixed(True)
        
sph4_b = chrono.ChSphereShape()
sph4_b.GetSphereGeometry().center = chrono.ChVectorD(1, 0, -1)
sph4_b.GetSphereGeometry().rad = 0.2
sphere4.AddAsset(sph4_b)
        
col4_b = chrono.ChColorAsset()
col4_b.SetColor(chrono.ChColor(0.1,0.1,0.5))
sphere4.AddAsset(col4_b)
        





   

tsda_sphere1_sphere2 = chrono.ChLinkSpring()
tsda_sphere1_sphere2.SetName("tsda_sphere1_sphere2")
tsda_sphere1_sphere2.Initialize(sphere1, sphere2, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
tsda_sphere1_sphere2.Set_SpringK(500)
#tsda_sphere1_sphere2.Set_SpringR(5.0)
tsda_sphere1_sphere2.Set_SpringRestLength(1.0)
system.AddLink(tsda_sphere1_sphere2)


tsda_sphere1_sphere3 = chrono.ChLinkSpring()
tsda_sphere1_sphere3.SetName("tsda_sphere1_sphere3")
tsda_sphere1_sphere3.Initialize(sphere1, sphere3, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
tsda_sphere1_sphere3.Set_SpringK(500)
#tsda_sphere1_sphere3.Set_SpringR(5.0)
tsda_sphere1_sphere3.Set_SpringRestLength(1.0)
system.AddLink(tsda_sphere1_sphere3)


tsda_sphere1_sphere4 = chrono.ChLinkSpring()
tsda_sphere1_sphere4.SetName("tsda_sphere2_sphere4")
tsda_sphere1_sphere4.Initialize(sphere1, sphere4, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, -1))
tsda_sphere1_sphere4.Set_SpringK(500)
#tsda_sphere1_sphere4.Set_SpringR(5.0)
tsda_sphere1_sphere4.Set_SpringRestLength(np.sqrt(2))
system.AddLink(tsda_sphere1_sphere4)


tsda_sphere2_sphere3 = chrono.ChLinkSpring()
tsda_sphere2_sphere3.SetName("tsda_sphere2_sphere3")
tsda_sphere2_sphere3.Initialize(sphere2, sphere3, False, chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 0, -1))
tsda_sphere2_sphere3.Set_SpringK(500)
#tsda_sphere2_sphere3.Set_SpringR(5.0)
tsda_sphere2_sphere3.Set_SpringRestLength(np.sqrt(2))
system.AddLink(tsda_sphere2_sphere3)


tsda_sphere2_sphere4 = chrono.ChLinkSpring()
tsda_sphere2_sphere4.SetName("tsda_sphere2_sphere4")
tsda_sphere2_sphere4.Initialize(sphere2, sphere3, False, chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(1, 0, -1))
tsda_sphere2_sphere4.Set_SpringK(500)
#tsda_sphere2_sphere4.Set_SpringR(5.0)
tsda_sphere2_sphere4.Set_SpringRestLength(1.0)
system.AddLink(tsda_sphere2_sphere4)
        

    






# In[adding prismatic joint between ground and puller]

z2x = chrono.ChQuaternionD()
z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 0))

prismatic_ground_sphere1 = chrono.ChLinkLockPrismatic()
prismatic_ground_sphere1.SetName("prismaticGP1")
prismatic_ground_sphere1.Initialize(ground, sphere1, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), z2x))
system.AddLink(prismatic_ground_sphere1)


prismatic_ground_sphere2 = chrono.ChLinkLockPrismatic()
prismatic_ground_sphere2.SetName("prismaticGP2")
prismatic_ground_sphere2.Initialize(ground, sphere2, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), z2x))
system.AddLink(prismatic_ground_sphere2)

constfun1 = chrono.ChFunction_Const(-45)
myforce1 = chrono.ChForce()
sphere1.AddForce(myforce1)
myforce1.SetF_x(constfun1)
myforce1.SetMode(chrono.ChForce.FORCE)
myforce1.SetDir(chrono.ChVectorD(0,0,0))


constfun2 = chrono.ChFunction_Const(45)
myforce2 = chrono.ChForce()
sphere2.AddForce(myforce2)
myforce2.SetF_x(constfun2)
myforce2.SetMode(chrono.ChForce.FORCE)
myforce2.SetDir(chrono.ChVectorD(1,0,0))


# In[simulate in Irrlitch]
system.ShowHierarchy(chrono.GetLog())


  ## 5. Prepare visualization with Irrlicht
  ##    Note that Irrlicht uses left-handed frames with Y up.

  ## Create the Irrlicht application and set-up the camera.
application = chronoirr.ChIrrApp (
                                    system,                               ## pointer to the mechanical system
                                    "bouncing ball",                ## title of the Irrlicht window
                                    chronoirr.dimension2du(800, 600),      ## window dimension (width x height)
                                    False,                                 ## use full screen?
                                    True)                                 ## enable shadows?
application.AddTypicalLogo();
application.AddTypicalSky();
application.AddTypicalLights();
application.AddTypicalCamera(
                            chronoirr.vector3df(-2, 5, -3),             ## camera location
                            chronoirr.vector3df(-2, 0, 0));             ## "look at" location

  ## Let the Irrlicht application convert the visualization assets.
application.AssetBindAll()
application.AssetUpdateAll()


  ## 6. Perform the simulation.

  ## Specify the step-size.
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while (application.GetDevice().run()):

    ## Initialize the graphical scene.
    application.BeginScene()
    
    ## Render all visualization objects.
    application.DrawAll()

    ## Draw an XZ grid at the global origin to add in visualization.
    
    chronoirr.ChIrrTools.drawGrid(
                                  application.GetVideoDriver(), 1, 1, 20, 20,
                                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)),
                                  chronoirr.SColor(255, 80, 100, 100), True)

    ## Advance simulation by one step.
    application.DoStep()

    ## Finalize the graphical scene.
    application.EndScene()