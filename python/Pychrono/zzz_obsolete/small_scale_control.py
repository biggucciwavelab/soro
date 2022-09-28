
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
from pid_tutorial import PidControl as PID

sim="small"
start = timeit.default_timer()
record=2
tstep=0.001 #Time Step
tset= 0.5 #Settling time
tend= 3 #Length of simulation
# In[Set Path]
chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/")

# In[Create sysem and other misselanous things]
my_system = chrono.ChSystemNSC()
#my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverType(chrono.ChSolver.Type_PSOR)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
#chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
#chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0001)
#my_system.SetMaxItersSolverSpeed(500)
my_system.SetMaxPenetrationRecoverySpeed(1)
my_system.SetMinBounceSpeed(.1)

# In[Define frictional properties]
mu_f=.4
mu_b=.01
mu_r=.01
mu_s=.005

# In[create material properties]
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(mu_f)
material.SetDampingF(mu_b)
material.SetCompliance (0.0001)
material.SetComplianceT(0.0001)
material.SetRollingFriction(mu_r)
material.SetSpinningFriction(mu_s)
#material.SetComplianceRolling(0.01)
#material.SetComplianceSpinning(0.01)
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
wlength=1.25

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
nb=8              # number of robots
n=np.array([7,0])   # array of interior robots
ni=np.sum(n)        # sum them for total number of interior
nt=nb+ni            # total number of bots and particles
diameter=.07        # diameter of cylinder and robots
mr=.18              # mass
mp=.04              # mass of particles
height=0.12         # height of cylinder
hhalf=.06           # half height of cylinder

skind=0.005         #diameter of cylinders for skin particles
ratioO=8            #ratio of outer skin particles to big bots
ratioI=8            #ratio of inner skin particles to big bots
skinrho=500         #density of skin particles [kg/m^3]

k=10                # spring constant (bots)
b=k/50               # damping (bots)
ko=500               # spring constant (skin outside)
bo=1                # damping constant (skin outside)
ki=500              # spring constant (skin inside)
bi=1                # damping constant (skin inside)
rl=.01              # resting length
rlmax=.031          # max resting length
obj=[]              # empty matrix of bots and particles
Springs=[]          # empty matrix of springs
skinO=[]             # empty matrix of outer skin cylinders
skinI=[]             # empty matrix of inner skin cylinders
volume=np.pi*.25*height*(diameter)**2      # calculate volume
R1=(diameter*nb/(np.pi*2))+.06               # Radius of postions
bots=[]             #empty matrix to create only bots
rowr=mr/volume # calculate density of robot
rowp=mp/volume # calculate density of particles


# In[External forces]

mag = 1.25 #[N]- magnitude of external force applied at each bot
force=[] #empty array to store force objects
forceDir = []

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
        #myforcex.SetMforce(mag)
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
    bots.append(bot)
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

#dir of body position
XposBody=[]
YposBody=[]
ZposBody=[]

#dir of force direction
XdirForce=[]
YdirForce=[]
ZdirForce=[]

#dir of body position
XposBody1=np.zeros(nb)
YposBody1=np.zeros(nb)
ZposBody1=np.zeros(nb)

#dir of force direction
XdirForce1=np.zeros(nb)
YdirForce1=np.zeros(nb)
ZdirForce1=np.zeros(nb)

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
tar = [0,0,-.5]

# In[Pov RAY]
if record==1:
    script_dir = os.path.dirname("povvideofiles"+sim+"/")
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
    pov_exporter.SetCamera(chrono.ChVectorD(0,3,0), chrono.ChVectorD(0,0,0), 90)# specifiy camera location
    pov_exporter.AddAll()
    pov_exporter.ExportScript()
    #In[Run the simulation]
    count=0
    while (my_system.GetChTime() < tend) :
    
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
            t=tstep*count
        
        if t > tset:
            for i in range(len(force)):
                force[i].SetMforce(mag)
        
        for i in range(len(bots)):
            dirForce = force[i].GetDir()
            posBody  = bots[i].GetPos()
            
            #filling the position of body and force
            XposBody.append(posBody.x)
            YposBody.append(posBody.y)
            ZposBody.append(posBody.z)
            
            XdirForce.append(dirForce.x)
            YdirForce.append(dirForce.y)
            ZdirForce.append(dirForce.z)
    
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
        my_system.DoStepDynamics(tstep)
        print ('time=', my_system.GetChTime())
        
        if count%20==0:
            pov_exporter.ExportData()

# In[IRRLICHT code]
else:
    myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono example', chronoirr.dimension2du(1200,800))
    myapplication.AddTypicalSky()
    myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    myapplication.AddTypicalCamera(chronoirr.vector3df(0.75,0.75,1.5))
    myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),chronoirr.vector3df(0,0,0),9,1,9,50)
    myapplication.DrawAll               
    myapplication.AssetBindAll();
    myapplication.AssetUpdateAll();
    myapplication.AddShadowAll();

    count=0

    myapplication.SetTimestep(tstep)
    myapplication.SetTryRealtime(False)

    while(myapplication.GetDevice().run()):
        myapplication.BeginScene()
        myapplication.DrawAll()
        print ('time=', my_system.GetChTime() )
        
        for i in range(nb):
            var1=Springs[i].Get_SpringLength()
        
            if var1<rl:
                Springs[i].Set_SpringK(0)
        
            if var1>rlmax:
                Springs[i].Set_SpringK(10*k)
            else:
                Springs[i].Set_SpringK(k)
     
            t=tstep*count
            
        if t >= tset:
            for i in range(len(force)):
                force[i].SetDir(chrono.VECT_X)        
                
        
        for i in range(len(bots)):
            dirForce = force[i].GetDir()
            posBody  = bots[i].GetPos()
            
            #filling the position of body and force
            XposBody.append(posBody.x)
            YposBody.append(posBody.y)
            ZposBody.append(posBody.z)
        
            XdirForce.append(dirForce.x)
            YdirForce.append(dirForce.y)
            ZdirForce.append(dirForce.z)

        for i in range(len(bots)):
            XposBody1[i] = XposBody[i]
            YposBody1[i] = YposBody[i]
            ZposBody1[i] = ZposBody[i]
            
            XdirForce1[i] = XdirForce[i]
            YdirForce1[i] = YdirForce[i]
            ZdirForce1[i] = ZdirForce[i]
            
        #control = PID(nb,tar,XdirForce1,YdirForce1,ZdirForce1,XposBody1,YposBody1,ZposBody1,1,0,0)   
        
        if t > tset:
            control = PID(nb,mag,tar,XdirForce1,YdirForce1,ZdirForce1,XposBody1,YposBody1,ZposBody1,1,0,0)
            forcemag = control.intarget
            for i in range(len(force)):
                #force[i].SetDir(chrono.ChVectorD(1,0,1))
                force[i].SetDir(chrono.ChVectorD(np.cos(np.radians(control.cur_angle)),0,np.sin(np.radians(control.cur_angle))))
                force[i].SetMforce(forcemag)
                   
        #if t >= tset:
         #   for i in range(len(force)):
          #      force[i].SetDir(chrono.ChVectorD(1,0,np.arctan(np.radians(control.cur_angle)))) 
            
            
        
                
                            
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
        

        
         #filling the position of body and force
        XposBody.clear()
        YposBody.clear()
        ZposBody.clear()
        
        XdirForce.clear()
        YdirForce.clear()
        ZdirForce.clear()

        if t > tend:
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
    
np.savez("small_scale"+".npz",allow_pickle=True,
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
