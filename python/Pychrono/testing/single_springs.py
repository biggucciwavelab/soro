"""
Test the stability of a two mass system connected with 
a membrane made up of small spring linked particles

Author: Qiyuan Zhou
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import timeit
    
# In[Set Path]
chrono.SetChronoDataPath("C:/Chrono/Builds/chrono-develop/bin/data/")
# In[Create sysem and other misselanous things]
system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# In[create material properties]
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetDampingF(0.01)
material.SetCompliance (0.0001)
material.SetComplianceT(0.0001)

# %% Colors and shit
col_r = chrono.ChColorAsset(); col_r.SetColor(chrono.ChColor(1, 0, 0))              # Red
col_g = chrono.ChColorAsset(); col_g.SetColor(chrono.ChColor(0, 1, 0))              # Green
col_b = chrono.ChColorAsset(); col_b.SetColor(chrono.ChColor(0, 0, 1))              # Blue
col_y = chrono.ChColorAsset(); col_y.SetColor(chrono.ChColor(1, 1, 0))              # Yellow
col_p = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(0.44, .11, 52))        # Purple
col_gy1=chrono.ChColorAsset(); col_gy1.SetColor(chrono.ChColor(0.25, 0.25, 0.25))   # Dark Grey
col_gy2=chrono.ChColorAsset(); col_gy2.SetColor(chrono.ChColor(0.3, 0.3, 0.3))      # Medium Grey
col_gy3=chrono.ChColorAsset(); col_gy3.SetColor(chrono.ChColor(.35, .35, .35))      # Light Grey

# %% Spring constants
km = 500
bm = 10

# In[Create floor]
if True:
    ankit = chrono.ChBody()
    ankit.SetBodyFixed(True)
    ankit.SetPos(chrono.ChVectorD(0, -.1, 0 ))
    ankit.SetMaterialSurface(material)
    ankit.GetCollisionModel().ClearModel()
    ankit.GetCollisionModel().AddBox(8, .1, 8) # hemi sizes
    ankit.GetCollisionModel().BuildModel()
    ankit.SetCollide(True)
    ankit_shape = chrono.ChBoxShape()
    ankit_shape.GetBoxGeometry().Size = chrono.ChVectorD(8, .1, 8)
    ankit.GetAssets().push_back(ankit_shape)
    ankit_texture = chrono.ChTexture()
    ankit_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    ankit.GetAssets().push_back(ankit_texture)
    system.Add(ankit)

# %% Two big bois
declan = chrono.ChBodyEasyCylinder(0.04725,0.05,2000,True,True)
declan.SetPos(chrono.ChVectorD(0,0.5,0))
declan.SetMaterialSurface(material)
declan.SetBodyFixed(True)

esteban = chrono.ChBodyEasyCylinder(0.04725,0.05,2000,True,True)
esteban.SetPos(chrono.ChVectorD(0.5,0.5,0))
esteban.SetMaterialSurface(material)
esteban.SetBodyFixed(True)

system.Add(declan)
system.Add(esteban)
objects = [declan]

# %% Link them

ratioM = 5

for i in range(ratioM):
    # Link to last object with springs

    # Initial postion of each particle
    x= i * 0.5/(ratioM+1) + 0.012
    y= 0.5
    z= 0
    
    # Create particles:
    skinm = chrono.ChBodyEasyCylinder(0.01, 0.04,2000,True,True)
    skinm.SetPos(chrono.ChVectorD(x,y,z))
    skinm.SetMaterialSurface(material)
    skinm.SetNoGyroTorque(True)
    
    # Make the spring
    ground=chrono.ChLinkTSDA()
    
    p1=0; p2=0.02
    p3=0; p4=-0.02
    h=0.01

    ground.Initialize(objects[i], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False,0.0)
    ground.SetSpringCoefficient(km)
    ground.SetDampingCoefficient(bm)
    ground.AddAsset(col_p)
    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
    system.AddLink(ground)
    
    ground1=chrono.ChLinkTSDA()
    ground1.Initialize(objects[i], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False,0.0)
    ground1.SetSpringCoefficient(km)
    ground1.SetDampingCoefficient(bm)
    ground1.AddAsset(col_p)
    ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
    system.AddLink(ground1)
    
    if i==ratioM-1:
        ground3=chrono.ChLinkTSDA()
        ground3.Initialize(esteban, skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False,0.0)
        ground3.SetSpringCoefficient(km)
        ground3.SetDampingCoefficient(bm)
        ground3.AddAsset(col_p)
        ground3.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        system.AddLink(ground3)
        
        ground4=chrono.ChLinkTSDA()
        ground4.Initialize(esteban, skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False,0.0)
        ground4.SetSpringCoefficient(km)
        ground4.SetDampingCoefficient(bm)
        ground4.AddAsset(col_p)
        ground4.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        system.AddLink(ground4)
    
    objects.append(skinm)
    system.Add(skinm)


# In[IRRLICHT code]
start = timeit.default_timer()
myapplication = chronoirr.ChIrrApp(system, 'Testing', chronoirr.dimension2du(1200,800))
myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.75,0.75,1.5))
myapplication.AddLightWithShadow(chronoirr.vector3df(2,5,2),chronoirr.vector3df(2,2,2),10,2,10,120)
myapplication.SetShowProfiler(False)
myapplication.DrawAll               
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()
myapplication.AddShadowAll()
myapplication.SetPaused(False)

count=0
# Time step
myapplication.SetTimestep(0.001)
myapplication.SetTryRealtime(False)

while(myapplication.GetDevice().run()):
    t=system.GetChTime()
    myapplication.BeginScene()
    myapplication.DrawAll()
    
    print ('time=', round(t,3))
    
    myapplication.DoStep()
    myapplication.EndScene()

stop = timeit.default_timer()

runtime=stop-start
runtime=runtime
print("Total runtime: "+str(round(runtime,3))+" seconds")
