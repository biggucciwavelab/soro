# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 16:07:52 2019

@author: dmulr
"""



# In[import libraries]
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np

chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")
# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()

my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
#my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# In[Define frciton properties]
mu_f=.4
mu_b=.4
mu_r=.4
mu_s=.4

# In[create material]
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(mu_f)
material.SetDampingF(mu_b)
material.SetCompliance (0.00001)
material.SetComplianceT(0.00001)
material.SetRollingFriction(mu_r)
material.SetSpinningFriction(mu_s)
material.SetComplianceRolling(0.00001)
material.SetComplianceSpinning(0.00001)
# In[Create floor]
body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -1, 0 ))
body_floor.SetMaterialSurface(material)

# In[Collision shape]
body_floor.GetCollisionModel().ClearModel()
body_floor.GetCollisionModel().AddBox(3, 1, 3) # hemi sizes
body_floor.GetCollisionModel().BuildModel()
body_floor.SetCollide(True)

# In[Visualization shape]
body_floor_shape = chrono.ChBoxShape()
body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(3, 1, 3)
body_floor.GetAssets().push_back(body_floor_shape)
body_floor_texture = chrono.ChTexture()
body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
body_floor.GetAssets().push_back(body_floor_texture)
my_system.Add(body_floor)

# In[Create wall]

wwidth=.1
wheight=.1
wlength=1.5

meh=np.array([1,-1])
for i in(meh):
    wall = chrono.ChBody()
    wall.SetBodyFixed(True)
    wall.SetPos(chrono.ChVectorD(1.5*i, wheight/2, 0))
    wall.SetMaterialSurface(material)
    # Collision shape
    wall.GetCollisionModel().ClearModel()
    wall.GetCollisionModel().AddBox(wwidth, wheight, wlength) # hemi sizes
    wall.GetCollisionModel().BuildModel()
    wall.SetCollide(True)
    # Visualization shape
    wall_shape = chrono.ChBoxShape()
    wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(wwidth, wheight, wlength)
    wall_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
    wall.GetAssets().push_back(wall_shape)
    wall_texture = chrono.ChTexture()
    wall_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    wall.GetAssets().push_back(wall_texture)
    my_system.Add(wall)

for i in(meh):
    wall = chrono.ChBody()
    wall.SetBodyFixed(True)
    wall.SetPos(chrono.ChVectorD(0, wheight/2, 1.5*i))
    wall.SetMaterialSurface(material)
    # Collision shape
    wall.GetCollisionModel().ClearModel()
    wall.GetCollisionModel().AddBox(wlength, wheight, wwidth) # hemi sizes
    wall.GetCollisionModel().BuildModel()
    wall.SetCollide(True)
    # Visualization shape
    wall_shape = chrono.ChBoxShape()
    wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(wlength,wheight, wwidth)
    wall_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
    wall.GetAssets().push_back(wall_shape)
    wall_texture = chrono.ChTexture()
    wall_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    wall.GetAssets().push_back(wall_texture)
    my_system.Add(wall)

# In[cylinder dimmensions]
# run 1 97, k=100, n=np.array([93,86,77,69,61,52,44,33,22,12,4])

# run 2 94, k=100, n=np.array([93,86,77,69,61,52,44,33,22,12,4])



# number of robots
nb=8
n=np.array([3])
ni=np.sum(n)
# diameter of cylinder and robots
diameter=.07 

R1=(diameter*nb/(np.pi*2))
# mass
mr=.18
# mass of particles
mp=.01
# height of cylinder
height=.12
# calculate volume
volume=np.pi*.25*height*(diameter)**2
# calculate density
rowr=mr/volume

rowp=mp/volume

# spring constant
k=100
# damping
b=.1

# empty matrix
obj=[]
# constant function for external forces



variables=([nb,ni,diameter,height,volume,mr,mp,k,b,mu_f,mu_b,mu_r,mu_s])
texts=["number of boundary(n/a)=","\r\n number of interior(n/a)=","\r\n diameter(m)=","\r\n height of each robot(m)","\r\n volume(m^3)=","\r\n mass of each robot(kg)=","\r\n mass of each particle(kg)","\r\n spring constant(N/m)=","\r\n damping coefficent(Ns/m)=","\r\nsliding friction=","\r\nmaterial dampning=","\r\nRolling friction=","\r\nSpinning friction="]

f= open("simulation_run"+str(1)+"variables.txt","w+")
for i in range(13):
    f.write(texts[i]+str(variables[i]) )

# ..create the function for imposed x horizontal motion, etc.
mfunX = chrono.ChFunction_Sine(0,.002,5)  # phase, frequency, amplitude
#link_shaker.SetMotion_Y(mfunY)
constfun = chrono.ChFunction_Const(.8)

botcall=np.zeros((1,nb))


active=np.array([0,1,2])

for i in (active):
    botcall[:,i]=1

# In[Create Bots]
for i in range (nb):
    x=R1*np.cos(i*2*np.pi/nb)
    y=.5*height
    z=R1*np.sin(i*2*np.pi/nb)
    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyCylinder(diameter/2, height,rowr)
    bot.SetPos(chrono.ChVectorD(x,y,z))
    bot.SetMaterialSurface(material)
    bot.SetId(i)
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddCylinder(diameter/2,diameter/2,height/2) # hemi sizes
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    bot.SetBodyFixed(False)
    if botcall[:,i]==1:
        myforcex = chrono.ChForce()
        bot.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetF_x(constfun)
        myforcex.SetDir(chrono.ChVectorD(1,0,0))
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))
        bot.AddAsset(col_g)
    else:
        print('passive')
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))
        bot.AddAsset(col_g)
    if i>=1:
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        ground.Initialize(obj[i-1], bot, False, chrono.ChVectorD(obj[i-1].GetPos().x,obj[i-1].GetPos().y ,obj[i-1].GetPos().z), chrono.ChVectorD(x, y, z))
        ground.Set_SpringK(k)
        ground.Set_SpringR(b)
        ground.Set_SpringRestLength(diameter)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
    if i==nb-1:
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        ground.Initialize(bot, obj[0], False,chrono.ChVectorD(x, y, z) , chrono.ChVectorD(obj[0].GetPos().x,obj[0].GetPos().y ,obj[0].GetPos().z))
        ground.Set_SpringK(k)
        ground.Set_SpringR(b)
        ground.Set_SpringRestLength(diameter)
        my_system.AddLink(ground)        
    my_system.Add(bot)
    obj.append(bot)        


# In[Create interiors]
for i in range(n.size):
    print(i)
    for j in range(n[i]):
        R2=diameter*n[i]/(np.pi*2)
        x=R2*np.cos(j*2*np.pi/n[i])
        y=.5*height
        z=R2*np.sin(j*2*np.pi/n[i])
        gran = chrono.ChBody()
        gran = chrono.ChBodyEasyCylinder(diameter/2, height/2,rowp)
        gran.SetPos(chrono.ChVectorD(x,y,z))
        gran.SetMaterialSurface(material)
        gran.SetId(i)
        gran.GetCollisionModel().ClearModel()
        gran.GetCollisionModel().AddCylinder(diameter/2,diameter/2,height/4) # hemi sizes
        gran.GetCollisionModel().BuildModel()
        gran.SetCollide(True)
        col_r = chrono.ChColorAsset()
        col_r.SetColor(chrono.ChColor(1, 0, 0))
        gran.AddAsset(col_r)
        my_system.Add(gran)
        obj.append(gran) 


        
# In[Irrlecht Simulation]


Xpos=[]
Ypos=[]
Zpos=[]
Xforce=[]
Yforce=[]
Zforce=[]
Xcontact=[]
Ycontact=[]
Zcontact=[]
ttemp=[]
rott0=[]
rott1=[]
rott2=[]
rott3=[]
count=0

nt=nb+ni

myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono example', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.75,0.75,1.5))
myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                 chronoirr.vector3df(0,0,0),    # aimpoint
                                 9,                 # radius (power)
                                 1,9,               # near, far
                                 50)                # angle of FOV

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
count=0
h=.01
myapplication.SetTimestep(h)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    
    
    for substep in range(0,5):
        t=h*count
        for i in range(nt):
            
            
            temp=obj[i].GetContactForce()
            # total force applied at center of mass
            tempx=obj[i].Get_Xforce()
            Xforce.append(tempx.x)
            Yforce.append(tempx.y)
            Zforce.append(tempx.z)
            Xcontact.append(temp.x)
            Ycontact.append(temp.y)
            Zcontact.append(temp.z)
            Xpos.append(obj[i].GetPos().x)
            Ypos.append(obj[i].GetPos().y)
            Zpos.append(obj[i].GetPos().z)
            rott0.append(obj[i].GetRot().e0)
            rott1.append(obj[i].GetRot().e1)
            rott2.append(obj[i].GetRot().e2)
            rott3.append(obj[i].GetRot().e3)
        ttemp.append(t)
        count=count+1
        myapplication.DoStep()
    myapplication.EndScene()

# In[Convert plist to matrices]
Xpos=np.asarray(Xpos)
Ypos=np.asarray(Ypos)
Zpos=np.asarray(Zpos)

rott0=np.asarray(rott0)
rott1=np.asarray(rott1)
rott2=np.asarray(rott2)
rott3=np.asarray(rott3)

Xforce=np.asarray(Xforce)
Yforce=np.asarray(Yforce)
Zforce=np.asarray(Zforce)

Xcontact=np.asarray(Xcontact)
Ycontact=np.asarray(Ycontact)
Zcontact=np.asarray(Zcontact)

# In[Create empty arrays]
# position
qx=np.zeros((nt,count))
qy=np.zeros((nt,count))
qz=np.zeros((nt,count))
# contact forces
Fxc=np.zeros((nt,count))
Fyc=np.zeros((nt,count))
Fzc=np.zeros((nt,count))
# total forces
Fxt=np.zeros((nt,count))
Fyt=np.zeros((nt,count))
Fzt=np.zeros((nt,count))

rot0=np.zeros((nt,count))
rot1=np.zeros((nt,count))
rot2=np.zeros((nt,count))
rot3=np.zeros((nt,count))


for i in range(count):
    print(i)
    #qxtemp=Xpos[nt*i:nt*i+nt]

    qx[:,i]=Xpos[nt*i:nt*(i)+nt]
    qy[:,i]=Ypos[nt*i:nt*i+nt]  
    qz[:,i]=Zpos[nt*i:nt*i+nt]  
    
    Fxt[:,i]=Xforce[nt*i:nt*i+nt] 
    Fyt[:,i]=Yforce[nt*i:nt*i+nt] 
    Fzt[:,i]=Zforce[nt*i:nt*i+nt] 
    
    Fxc[:,i]=Xcontact[nt*i:nt*i+nt] 
    Fyc[:,i]=Ycontact[nt*i:nt*i+nt] 
    Fzc[:,i]=Zcontact[nt*i:nt*i+nt]
    
    rot0[:,i]=rott0[nt*i:nt*i+nt]
    rot1[:,i]=rott1[nt*i:nt*i+nt]
    rot2[:,i]=rott2[nt*i:nt*i+nt]
    rot3[:,i]=rott3[nt*i:nt*i+nt]
##    
np.savez('resume1str.npz',allow_pickle=True,Fxc=Fxc,Fyc=Fyc,Fzc=Fzc,Fxt=Fxt,Fyt=Fyt,Fzt=Fzt,qx=qx,qy=qy,qz=qz,nb=nb,ni=ni,mr=mr,mp=mp,rowr=rowr,rowp=rowp,height=height,diameter=diameter,volume=volume,ttemp=ttemp,count=count,rot0=rot0,rot1=rot1,rot2=rot2,rot3=rot3,botcall=botcall)
#
##obj = np.asarray(obj, dtype=np.float32)
#np.savez('resume1.npz',obj=obj,ni=ni,nb=nb,mass=mass,height=height,diameter=diameter, allow_pickle=True, fix_imports=True)
