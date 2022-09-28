
# In[Header]

"""
author: declan mulroy
project: JAMoEBA
email: dmulroy@hawk.iit.edu
date: 10/9/19
"""

# In[import libraries]
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit

sim=1
start = timeit.default_timer()
record=0
# In[Set Path]


# In[Create sysem and other misselanous things]
chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")
# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()

my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.01)

my_system.SetMaxPenetrationRecoverySpeed(5)
my_system.SetMinBounceSpeed(.5)

# In[Define frictional properties]
mu_f=.4
mu_b=.01
mu_r=.01
mu_s=.005

# In[create material properties]
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(mu_f)
material.SetDampingF(mu_b)
material.SetCompliance (0.001)
material.SetComplianceT(0.001)
# In[Create floor]
body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -.1, 0 ))
body_floor.SetMaterialSurface(material)

# In[Collision shape]
body_floor.GetCollisionModel().ClearModel()
body_floor.GetCollisionModel().AddBox(3, .1, 3) # hemi sizes
body_floor.GetCollisionModel().BuildModel()
body_floor.SetCollide(True)

# In[Visualization shape]
body_floor_shape = chrono.ChBoxShape()
body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(3, .1, 3)
body_floor.GetAssets().push_back(body_floor_shape)
body_floor_texture = chrono.ChTexture()
body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
body_floor.GetAssets().push_back(body_floor_texture)
my_system.Add(body_floor)

# In[Create wall to enclose the robot]

# Geometry of walls
wwidth=.1
wheight=.1
wlength=1.5

meh=np.array([1,-1])

for i in(meh):
    # Create walls 
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
    # Create walls
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

# In[cylinder dimmensions/geometries]

nb=10                # number of robots
n=np.array([7,0])   # array of interior robots
ni=np.sum(n)        # sum them for total number of interior
nt=nb+ni            # total number of bots and particles
diameter=.07        # diameter of cylinder and robots
mr=.18              # mass
mp=.04              # mass of particles
height=0.12         # height of cylinder
hhalf=.06           # half height of cylinder

skind=0.01         #diameter of cylinders for skin particles
ratioO=8            #ratio of outer skin particles to big bots
ratioI=8            #ratio of inner skin particles to big bots
skinrho=1000         #density of skin particles [kg/m^3]

k=100                # spring constant (bots)
b=k/50               # damping (bots)
ko=100               # spring constant (skin outside)
bo=1                # damping constant (skin outside)
ki=100              # spring constant (skin inside)
bi=1                # damping constant (skin inside)
rl=.01              # resting length
rlmax=.031          # max resting length
obj=[]              # empty matrix of bots and particles
Springs=[]          # empty matrix of springs
skinO=[]             # empty matrix of outer skin cylinders
skinI=[]             # empty matrix of inner skin cylinders
volume=np.pi*.25*height*(diameter)**2      # calculate volume
R1=(diameter*nb/(np.pi*2))+.03               # Radius of postions

rowr=mr/volume # calculate density of robot
rowp=mp/volume # calculate density of particles

# In[External forces]

mag = 1.3 #[N]- magnitude of external force applied at each bot
force=[] #empty array to store force objects

# Empty matrix      
botcall=np.zeros((1,nb))

# ID number of robots to be active
active=np.arange(nb)

# For robots that are active fill botcall==1
for i in (active):
    botcall[:,i]=1

# In[Create Bots]
#material.SetFriction(0)
for i in range (nb):
    
    # Initial postion of each bot
    theta=i*2*np.pi/nb
    x=R1*np.cos(theta)
    y=.55*height
    z=R1*np.sin(theta)
    
    # Create bots    
    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyCylinder(diameter/2, height,rowr)
    bot.SetPos(chrono.ChVectorD(x,y,z))
    bot.SetMaterialSurface(material)
        
    # rotate them
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
    bot.SetRot(rotation1)

    # give ID number
    bot.SetId(i)
    
    # collision model
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddCylinder(diameter/2,diameter/2,hhalf) # hemi sizes
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    bot.SetBodyFixed(False)
    
    # Apply forces to active bots
    if botcall[:,i]==1:
        myforcex = chrono.ChForce()
        bot.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetDir(chrono.VECT_X)
        myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
        myforcex.SetMforce(mag)
        force.append(myforcex)
                
        col_y = chrono.ChColorAsset()
        col_y.SetColor(chrono.ChColor(0.44, .11, 52))
        bot.AddAsset(col_y)
    
    # passive robots
    else:

        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))
        bot.AddAsset(col_g)
        
    # Attach springs    
    if i>=1:
        ground=chrono.ChLinkSpring()
        ground1=chrono.ChLinkSpring()
        ground.SetName("ground")
        p1=0
        p2=diameter/2
        p3=0
        p4=-diameter/2
        h=height/4

        ground.Initialize(obj[i-1], bot,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringK(k)
        ground.Set_SpringR(b)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground)
        
        ground1.Initialize(obj[i-1], bot,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
        ground1.Set_SpringK(k)
        ground1.Set_SpringR(b)
        ground1.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground1.AddAsset(col1)
        ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground1)
        Springs.append(ground1)
    # Last spring
        if i==nb-1:        
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(bot, obj[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
            ground.Set_SpringK(k)
            ground.Set_SpringR(b)
            ground.Set_SpringRestLength(rl)
            col1=chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,0,1))
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            my_system.AddLink(ground)
            Springs.append(ground) 
            
            ground1=chrono.ChLinkSpring()
            ground1.SetName("ground")
            ground1.Initialize(bot, obj[0], True, chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
            ground1.Set_SpringK(k)
            ground1.Set_SpringR(b)
            ground1.Set_SpringRestLength(rl)
            col1=chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(0,0,1))
            ground1.AddAsset(col1)
            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            my_system.AddLink(ground1)
            Springs.append(ground1) 
    my_system.Add(bot)
    obj.append(bot)


# In[Create outer "Skin"]
so=R1+(diameter/2)+1.25*(skind/2)
no=ratioO*nb
for i in range (no):
    
    # Initial postion of each bot
    theta=i*2*np.pi/no
    x=so*np.cos(theta)
    y=.52*height
    z=so*np.sin(theta)
    
    # Create bots    
    skino = chrono.ChBody()
    skino = chrono.ChBodyEasyCylinder(skind/2, .85*height,skinrho)
    skino.SetPos(chrono.ChVectorD(x,y,z))
    skino.SetMaterialSurface(material)
    #skino.SetNoGyroTorque(True)
    
    # rotate them
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
    skino.SetRot(rotation1)
    
    # give ID number
    skino.SetId(i)
    skino.SetNoGyroTorque(True)
    # collision model
    skino.GetCollisionModel().ClearModel()
    skino.GetCollisionModel().AddCylinder(skind/2,skind/2,(.85*height/2)) # hemi sizes
    skino.GetCollisionModel().BuildModel()
    skino.SetCollide(True)
    skino.SetBodyFixed(False)
        
    # Attach springs    
    if i>=1:
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        p1=0
        p2=skind/2
        p3=0
        p4=-skind/2
        h=height/4

        ground.Initialize(skinO[i-1], skino,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringK(ko)
        ground.Set_SpringR(bo)
        ground.Set_SpringRestLength(0)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(1,1,0))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground)
        
        ground1=chrono.ChLinkSpring()
        ground1.Initialize(skinO[i-1], skino,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
        ground1.Set_SpringK(ko)
        ground1.Set_SpringR(bo)
        ground1.Set_SpringRestLength(0)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(1,1,0))
        ground1.AddAsset(col1)
        ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground1)
        Springs.append(ground1)
        
    # Last spring
        if i==no-1:        
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(skino, skinO[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
            ground.Set_SpringK(ko)
            ground.Set_SpringR(bo)
            ground.Set_SpringRestLength(0)
            col1=chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,1,0))
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            my_system.AddLink(ground)
            Springs.append(ground) 
            
            ground1=chrono.ChLinkSpring()
            ground1.Initialize(skino, skinO[0],True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
            ground1.Set_SpringK(ko)
            ground1.Set_SpringR(bo)
            ground1.Set_SpringRestLength(0)
            col1=chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,1,0))
            ground1.AddAsset(col1)
            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            my_system.AddLink(ground1)
            Springs.append(ground1)
            
    
    #Link to cylinder
    if i%ratioO==0:
        skino.AddAsset(col_y)
        glue=chrono.ChLinkMateFix()
        glue.SetName("glue")
        glue.Initialize(skino,obj[int((i*nb)/no)])
        my_system.AddLink(glue)
        
    my_system.Add(skino)
    skinO.append(skino)
# In[Create inner "Skin"]
si=R1-(diameter/2)-1.25*(skind/2)
ni=ratioI*nb
for i in range (ni):
    
    # Initial postion of each bot
    theta=i*2*np.pi/ni
    x=si*np.cos(theta)
    y=.52*height
    z=si*np.sin(theta)
    
    # Create bots    
    skini = chrono.ChBody()
    skini = chrono.ChBodyEasyCylinder(skind/2, .85*height,skinrho)
    skini.SetPos(chrono.ChVectorD(x,y,z))
    skini.SetMaterialSurface(material)
    #skino.SetNoGyroTorque(True)
    
    # rotate them
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
    skini.SetRot(rotation1)
    
    # give ID number
    skini.SetId(i)
    skini.SetNoGyroTorque(True)
    # collision model
    skini.GetCollisionModel().ClearModel()
    skini.GetCollisionModel().AddCylinder(skind/2,skind/2,(.85*height/2)) # hemi sizes
    skini.GetCollisionModel().BuildModel()
    skini.SetCollide(True)
    skini.SetBodyFixed(False)
        
    # Attach springs    
    if i>=1:
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        p1=0
        p2=skind/2
        p3=0
        p4=-skind/2
        h=height/4

        ground.Initialize(skinI[i-1], skini,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringK(ko)
        ground.Set_SpringR(bo)
        ground.Set_SpringRestLength(0)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(1,1,0))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground)
        
        ground1=chrono.ChLinkSpring()
        ground1.Initialize(skinI[i-1], skini,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
        ground1.Set_SpringK(ko)
        ground1.Set_SpringR(bo)
        ground1.Set_SpringRestLength(0)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(1,1,0))
        ground1.AddAsset(col1)
        ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground1)
        Springs.append(ground1)
        
    # Last spring
        if i==ni-1:        
            ground=chrono.ChLinkSpring()
            ground.SetName("ground")
            ground.Initialize(skini, skinI[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
            ground.Set_SpringK(ko)
            ground.Set_SpringR(bo)
            ground.Set_SpringRestLength(0)
            col1=chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,1,0))
            ground.AddAsset(col1)
            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            my_system.AddLink(ground)
            Springs.append(ground) 
            
            ground1=chrono.ChLinkSpring()
            ground1.Initialize(skini, skinI[0],True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),False)
            ground1.Set_SpringK(ko)
            ground1.Set_SpringR(bo)
            ground1.Set_SpringRestLength(0)
            col1=chrono.ChColorAsset()
            col1.SetColor(chrono.ChColor(1,1,0))
            ground1.AddAsset(col1)
            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
            my_system.AddLink(ground1)
            Springs.append(ground1)
            
    
    #Link to cylinder
    if i%ratioO==0:
        skini.AddAsset(col_y)
        glue=chrono.ChLinkMateFix()
        glue.SetName("glue")
        glue.Initialize(skini,obj[int((i*nb)/ni)])
        my_system.AddLink(glue)
        
    my_system.Add(skini)
    skinI.append(skini)

# In[Create interiors]
material.SetFriction(mu_f)
for i in range(n.size):
    for j in range(n[i]):
        R2=diameter*n[i]/(np.pi*2)
        x=R2*np.cos(j*2*np.pi/n[i])
        y=.55*height
        z=R2*np.sin(j*2*np.pi/n[i])
        gran = chrono.ChBody()
        gran = chrono.ChBodyEasyCylinder(diameter/2, height,rowp)
        gran.SetPos(chrono.ChVectorD(x,y,z))
        gran.SetMaterialSurface(material)
        gran.SetId(i)
        gran.GetCollisionModel().ClearModel()
        gran.GetCollisionModel().AddCylinder(diameter/2,diameter/2,height/2) # hemi sizes
        gran.GetCollisionModel().BuildModel()
        gran.SetCollide(True)
        col_r = chrono.ChColorAsset()
        col_r.SetColor(chrono.ChColor(1, 0, 0))
        gran.AddAsset(col_r)
        my_system.Add(gran)
        obj.append(gran)


# In[ Create empty matrices to be filled]
Xpos=[]
Ypos=[]
Zpos=[]

Xforce=[]
Yforce=[]
Zforce=[]

Xcontact=[]
Ycontact=[]
Zcontact=[]

# empty temporary velocity matrices
Xvel=[]
Yvel=[]
Zvel=[]

templ=[]
ttemp=[]
rott0=[]
rott1=[]
rott2=[]
rott3=[]
xunit=[]
yunit=[]
zuni=[]
coord=[]
count=0
h=.01

# In[Pov RAY]
if record==1:
    

    script_dir = os.path.dirname("povvideofiles"+str(sim)+"/")
    pov_exporter = postprocess.ChPovRay(my_system)

    # Sets some file names for in-out processes.
    pov_exporter.SetTemplateFile(chrono.GetChronoDataPath() + "_template_POV.pov")
    pov_exporter.SetOutputScriptFile("rendering"+str(sim)+".pov")
    pov_exporter.SetOutputDataFilebase("my_state")
    pov_exporter.SetPictureFilebase("picture")

    # create folders
    if not os.path.exists("output"+str(sim)):
        os.mkdir("output"+str(sim))
    if not os.path.exists("anim"+str(sim)):
            os.mkdir("anim"+str(sim))
    pov_exporter.SetOutputDataFilebase("output"+str(sim)+"/my_state")
    pov_exporter.SetPictureFilebase("anim"+str(sim)+"/picture")
    #In[Run the simulation]
    count=0
    while (my_system.GetChTime() < 5) :
    


        for i in range(nb):
            var1=Springs[i].Get_SpringLength()
        
            if var1<rl:
                Springs[i].Set_SpringK(0)
        
            if var1>rlmax:
                Springs[i].Set_SpringK(10*k)
            else:
                Springs[i].Set_SpringK(k)
            t=h*count
    
        for i in range(nt):
            
            # Tempory named variables
            templ.append(var1)
            temp=obj[i].GetContactForce()
            tempx=obj[i].Get_Xforce()
            tempxx=obj[i].GetPos_dt()

            # Fill in the temportary  Total force matrices
            Xforce.append(tempx.x)
            Yforce.append((tempx.y))
            Zforce.append(tempx.z)
            
            # Fill in the temporary contact force matrices
            Xcontact.append(temp.x)
            Ycontact.append(temp.y)
            Zcontact.append(temp.z)
            
            # Fill in the temporary position matrices
            Xpos.append(obj[i].GetPos().x)
            Ypos.append((obj[i].GetPos().y))
            Zpos.append(obj[i].GetPos().z)
            
            # fill in the temporary rotational matrices
            rott0.append(obj[i].GetRot().e0)
            rott1.append(obj[i].GetRot().e1)
            rott2.append(obj[i].GetRot().e2)
            rott3.append(obj[i].GetRot().e3)
            
            # fill in the temporary velocity matrices
            Xvel.append(tempxx.x)
            Yvel.append(tempxx.y)
            Zvel.append(tempxx.z)
        

        
        
        ttemp.append(t)
        count=count+1

        my_system.DoStepDynamics(h)
        print ('time=', my_system.GetChTime())
        
        if count%2==0:
            pov_exporter.ExportData()
        
        


# In[IRRLICHT code]
else:

    myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono example', chronoirr.dimension2du(1200,800))

    myapplication.AddTypicalSky()
    myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    myapplication.AddTypicalCamera(chronoirr.vector3df(0.75,0.75,1.5))
    myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),chronoirr.vector3df(0,0,0),9,1,9,50)
    myapplication.DrawAll               

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

# In[ Run the simulation]

    count=0

# Time step
    h=.001

    myapplication.SetTimestep(h)
    myapplication.SetTryRealtime(False)

    while(myapplication.GetDevice().run()):
        myapplication.BeginScene()
        myapplication.DrawAll()
        print ('time=', my_system.GetChTime() )
        
        for i in range(len(force)):
            force[i].SetDir(chrono.VECT_X)
    
        for i in range(nb):
            var1=Springs[i].Get_SpringLength()
        
            if var1<rl:
                Springs[i].Set_SpringK(0)
        
            if var1>rlmax:
                Springs[i].Set_SpringK(10*k)
            else:
                Springs[i].Set_SpringK(k)
     
            t=h*count
        for i in range(nt):
            
            templ.append(var1)
            temp=obj[i].GetContactForce()
            tempx=obj[i].Get_Xforce()
            tempxx=obj[i].GetPos_dt()
                   
                       
            Xforce.append(tempx.x)
            Yforce.append((tempx.y))
            Zforce.append(tempx.z)
            
            Xcontact.append(temp.x)
            Ycontact.append(temp.y)
            Zcontact.append(temp.z)
            
            Xpos.append(obj[i].GetPos().x)
            Ypos.append((obj[i].GetPos().y))
            Zpos.append(obj[i].GetPos().z)
            
            rott0.append(obj[i].GetRot().e0)
            rott1.append(obj[i].GetRot().e1)
            rott2.append(obj[i].GetRot().e2)
            rott3.append(obj[i].GetRot().e3)
            
            # fill in the temporary velocity matrices
            Xvel.append(tempxx.x)
            Yvel.append(tempxx.y)
            Zvel.append(tempxx.z)
                       
           
        ttemp.append(t)
        count=count+1
        myapplication.DoStep()
        myapplication.EndScene()

        if t > 5:
            myapplication.GetDevice().closeDevice()

# In[Convert plist to matrices]
Xpos=np.asarray(Xpos)
Ypos=np.asarray(Ypos)
Zpos=np.asarray(Zpos)

rott0=np.asarray(rott0)
rott1=np.asarray(rott1)
rott2=np.asarray(rott2)
rott3=np.asarray(rott3)

templ=np.asarray(templ)

Xforce=np.asarray(Xforce)
Yforce=np.asarray(Yforce)
Zforce=np.asarray(Zforce)

Xcontact=np.asarray(Xcontact)
Ycontact=np.asarray(Ycontact)
Zcontact=np.asarray(Zcontact)

Xvel=np.asarray(Xvel)
Yvel=np.asarray(Yvel)
Zvel=np.asarray(Zvel)


# In[Create empty arrays]
# position
qx=np.zeros((nt,count))
qy=np.zeros((nt,count))
qz=np.zeros((nt,count))

# empty toational matrices
rot0=np.zeros((nt,count))
rot1=np.zeros((nt,count))
rot2=np.zeros((nt,count))
rot3=np.zeros((nt,count))


# contact forces
Fxc=np.zeros((nt,count))
Fyc=np.zeros((nt,count))
Fzc=np.zeros((nt,count))
# total forces
Fxt=np.zeros((nt,count))
Fyt=np.zeros((nt,count))
Fzt=np.zeros((nt,count))
# Spring length
SL=np.zeros((nt,count))

# Velocity empty matrices
Xv=np.zeros((nt,count))
Yv=np.zeros((nt,count))
Zv=np.zeros((nt,count))

for i in range(count):    
    # fill the position matrices
    qx[:,i]=Xpos[nt*i:nt*i+nt]
    qy[:,i]=Ypos[nt*i:nt*i+nt]  
    qz[:,i]=Zpos[nt*i:nt*i+nt]  
  
    # fill the rotational matrices  
    rot0[:,i]=rott0[nt*i:nt*i+nt]
    rot1[:,i]=rott1[nt*i:nt*i+nt]
    rot2[:,i]=rott2[nt*i:nt*i+nt]
    rot3[:,i]=rott3[nt*i:nt*i+nt]
    
    # fill the total force matrices
    Fxt[:,i]=Xforce[nt*i:nt*i+nt] 
    Fyt[:,i]=Yforce[nt*i:nt*i+nt] 
    Fzt[:,i]=Zforce[nt*i:nt*i+nt] 
    
    # fill the contact force matrices
    Fxc[:,i]=Xcontact[nt*i:nt*i+nt] 
    Fyc[:,i]=Ycontact[nt*i:nt*i+nt] 
    Fzc[:,i]=Zcontact[nt*i:nt*i+nt]
    
    Xv[:,i]=Xvel[nt*i:nt*i+nt]
    Yv[:,i]=Yvel[nt*i:nt*i+nt]
    Zv[:,i]=Zvel[nt*i:nt*i+nt]
    
    SL[:,i]=templ[nt*i:nt*i+nt]
            
variables=([nb,ni,diameter,height,volume,mr,mp,k,b,mu_f,mu_b,mu_r,mu_s,rl])

texts=["number of boundary(n/a)=","\r\n number of interior(n/a)=","\r\n diameter(m)=","\r\n height of each robot(m)=","\r\n volume(m^3)=","\r\n mass of each robot(kg)=","\r\n mass of each particle(kg)=","\r\n spring constant(N/m)=","\r\n damping coefficent(Ns/m)=","\r\nsliding friction=","\r\nmaterial dampning=","\r\nRolling friction=","\r\nSpinning friction=","\r\nresting length="]

f= open("simulation_run"+str(sim)+"variables.txt","w+")

for i in range(np.size(variables)):
    
    f.write(texts[i]+str(variables[i]) )
    
np.savez("resume"+str(sim)+".npz",allow_pickle=True,
         Fxc=Fxc,
         Fyc=Fyc,
         Fzc=Fzc,
         Fxt=Fxt,
         Fyt=Fyt,
         Fzt=Fzt,
         qx=qx,
         qy=qy,
         qz=qz,
         nb=nb,
         ni=ni,
         mr=mr,
         mp=mp,
         k=k,
         rowr=rowr,
         rowp=rowp,
         height=height,
         diameter=diameter,
         volume=volume,
         ttemp=ttemp,
         count=count,
         rot0=rot0,
         rot1=rot1,
         rot2=rot2,
         rot3=rot3,
         botcall=botcall,
         SL=SL,
         Xv=Xv,
         Yv=Yv,
         Zv=Zv,
         sim=sim)
stop = timeit.default_timer()

runtime=stop-start
runtime=runtime
print("Total runtime: "+str(runtime)+" seconds")
