"""
author: Qiyuan Zhou
project: JAMoEBA
email: qzhou13@hawk.iit.edu
date: 10/21/19
"""


# In[import libraries]
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.mkl as mkl
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit

def CastNode(nb):

    feaNB = fea.CastToChNodeFEAbase(nb)
    nodeFead = fea.CastToChNodeFEAxyzD(feaNB)
    return nodeFead

sim=1
start = timeit.default_timer()
record=0
# In[Set Path]
chrono.SetChronoDataPath("C:/Chrono/Builds/chrono-develop/bin/data/")

# In[Create sysem and other miscellanous things]
my_system = chrono.ChSystemSMC()

my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

#chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.00001)
#chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.00001)
#my_system.SetMaxItersSolverSpeed(1000)

h=0.0005 # Time step

# In[Define frictional properties]
mu_f=.4
mu_b=.01
mu_r=.4
mu_s=.4

# In[create material properties]
material = chrono.ChMaterialSurfaceSMC()
material.SetFriction(mu_f)
#material.SetDampingF(mu_b)
#material.SetCompliance (0.0001)
#material.SetComplianceT(0.0001)
#material.SetRollingFriction(mu_r)
#material.SetSpinningFriction(mu_s)
#material.SetComplianceRolling(0.001)
#material.SetComplianceSpinning(0.001)
# In[Create floor]
body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -1.1, 0 ))
body_floor.SetMaterialSurface(material)

# Collision shape
body_floor.GetCollisionModel().ClearModel()
body_floor.GetCollisionModel().AddBox(3, 1, 3) # hemi sizes
body_floor.GetCollisionModel().BuildModel()
body_floor.SetCollide(True)

# Visualization shape
body_floor_shape = chrono.ChBoxShape()
body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(3, 1, 3)
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

nb=8                # number of robots
n=np.array([6,1])   # array of interior robots
ni=np.sum(n)        # sum them for total number of interior
nt=nb+ni            # total number of bots and particles
diameter=.07        # diameter of cylinder and robots
mr=.18              # mass
mp=.01              # mass of particles
height=0.12         # height of cylinder
hhalf=.06           # half height of cylinder

k=100               # spring constant
b=.05               # damping
rl=.03              # resting length
rlmax=.031          # max resting length
obj=[]              # empty matrix of bots and particles
Springs=[]          # empty matrix of springs
meshes=[]           #empty matrix of meshes
viz=[]              #empty matrix of mesh visualizations
vizref=[]           #empty matrix of mesh visualizations
vizc=[]             #empty matrix of mesh visualizations
vizd=[]             #empty matrix of mesh visualizations
volume=np.pi*.25*height*(diameter)**2      # calculate volume
R1=(diameter*nb/(np.pi*2))+.03             # Radius of postions

rowr=mr/volume # calculate density of robot
rowp=mp/volume # calculate density of particles

# In[Fea options]

# Geometry of the plate
plate_lenght_x = .5
plate_lenght_y = .2
plate_lenght_z = 0.005
# Specification of the mesh
numDiv_x = 8
numDiv_y = 1
numDiv_z = 1
N_x = numDiv_x + 1
N_y = numDiv_y + 1
N_z = numDiv_z + 1
# Number of elements in the z direction is considered as 1
TotalNumElements = numDiv_x * numDiv_y
TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1)

# For uniform mesh
dx = plate_lenght_x / numDiv_x
dy = plate_lenght_y / numDiv_y
dz = plate_lenght_z / numDiv_z

# Create an orthotropic material.
# All layers for all elements share the same material.
rho = 1000
E = chrono.ChVectorD(2.1e7, 2.1e7, 2.1e7)
nu = chrono.ChVectorD(0.3, 0.3, 0.3)
G = chrono.ChVectorD(8.0769231e6, 8.0769231e6, 8.0769231e6)
mat = fea.ChMaterialShellANCF(rho, E, nu, G)

# In[External forces]
constfun = chrono.ChFunction_Const(1.3)
constfun0 = chrono.ChFunction_Const(0) 
  
# Empty matrix      
botcall=np.zeros((1,nb))

# ID number of robots to be active
active=np.array([])

# For robots that are active fill botcall==1
for i in (active):
    botcall[:,i]=1
    
# In[Create Bots]
#material.SetFriction(0)
for i in range (nb):
    
    # Initial postion of each bot
    theta=i*2*np.pi/nb
    x=R1*np.cos(theta)
    y=.5*height
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
        myforcex.SetF_x(constfun)
        myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
        myforcex.SetDir(chrono.ChVectorD(1,0,0))
        col_y = chrono.ChColorAsset()
        col_y.SetColor(chrono.ChColor(0.44, .11, 52))
        bot.AddAsset(col_y)
    
    # passive robots
    else:
        myforcex = chrono.ChForce()
        bot.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetF_x(constfun0)
        myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
        myforcex.SetDir(chrono.ChVectorD(1,0,0))
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))
        bot.AddAsset(col_g)
        
    #Add the meshes
    meshes.append('mesh'+str(i))
    meshes[i] = fea.ChMesh()

    # Create and add the nodes
    for j in range(TotalNumNodes) :
        # Node location
        # x incremented by dx at each iteration reastart at each 11*n
        loc_x = (j % (numDiv_x + 1)) * dx
        # y incremented by dy every Div_x iterations.
        loc_y = (((j / (numDiv_x + 1))) % (numDiv_y + 1)) * dy
        loc_z = (j) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz

        # Node direction
        dir_x = 0
        dir_y = 0
        dir_z = 1

        # Create the node
        node = fea.ChNodeFEAxyzD(chrono.ChVectorD(loc_x, loc_y, loc_z), chrono.ChVectorD(dir_x, dir_y, dir_z))
        node.SetMass(0)
        meshes[i].AddNode(node)
        '''
    # Create the flywheel and attach it to the center of the beam
	
    mbodyflywheel = chrono.ChBodyEasyCylinder(0.03, 0.05, 7800) # R, h, density
    mbodyflywheel.SetCoord(
		chrono.ChCoordsysD(node.GetPos() + chrono.ChVectorD(0,0,0), # flywheel initial center (plus Y offset)
		chrono.Q_from_AngAxis(3.14/2.0, chrono.VECT_Z)) # flywheel initial alignment (rotate 90° so cylinder axis is on X))
    )    
    my_system.Add(mbodyflywheel)

    myjoint = chrono.ChLinkMateFix()
    myjoint.Initialize(node, mbodyflywheel)
    my_system.Add(myjoint)
    '''
    # Get a handle to the tip node.
    tempnode = meshes[i].GetNode(TotalNumNodes - 1)
    tempfeanode = fea.CastToChNodeFEAbase(tempnode)
    nodetip = fea.CastToChNodeFEAxyzD(tempfeanode)

    # Create the elements
    for k in range(TotalNumElements):
        # Adjacent nodes
        node0 = int( k % numDiv_x)+i/50
        node1 = int(k % numDiv_x + 1)+i/50
        node2 = int(k % numDiv_x + 1 + N_x)+i/50
        node3 = int(k % numDiv_x + N_x)+i/50

    # Create the element and set its nodes.
    
        nodeA=CastNode(meshes[i].GetNode(node0))
        nodeB=CastNode(meshes[i].GetNode(node1))
        nodeC=CastNode(meshes[i].GetNode(node2))
        nodeD=CastNode(meshes[i].GetNode(node3))
    
        element = fea.ChElementShellANCF()
        element.SetNodes(nodeA,
                         nodeB,
                         nodeC,
                         nodeD)

        # Set element dimensions
        element.SetDimensions(dx, dy)

        # Add a single layers with a fiber angle of 0 degrees.
        element.AddLayer(dz, 0 * chrono.CH_C_DEG_TO_RAD, mat)

        # Set other element properties
        element.SetAlphaDamp(0.0)    # Structural damping for this element
        element.SetGravityOn(False)  # turn internal gravitational force calculation off
    
        # Add element to mesh
        meshes[i].AddElement(element)
    '''
    # Add the mesh and contact to the system
    mcontactcloud = fea.ChContactSurfaceNodeCloud()
    meshes[i].AddContactSurface(mcontactcloud)
    mcontactcloud.AddAllNodes(0.01)

    mcontactmesh = fea.ChContactSurfaceMesh()
    meshes[i].AddContactSurface(mcontactmesh)
    mcontactmesh.AddFacesFromBoundary(.01)


    mysurfmaterial = chrono.ChMaterialSurfaceSMC()
    #mysurfmaterial.SetCompliance(0.000001)
    mysurfmaterial.SetFriction(0.3)
    #mysurfmaterial.SetRestitution(0.2)
    #mysurfmaterial.SetCohesion(0)

    mcontactmesh.SetMaterialSurface(mysurfmaterial)
    '''
    my_system.Add(meshes[i])
          
    my_system.Add(bot)
    obj.append(bot)   
    
    #Attach Visualizations to Mesh
    viz.append('visual'+str(i))
    viz[i] = fea.ChVisualizationFEAmesh(meshes[i])
    viz[i] .SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
    viz[i] .SetColorscaleMinMax(0.0, 1.50)
    viz[i] .SetShrinkElements(True, 0.85)
    viz[i] .SetSmoothFaces(True)
    meshes[i].AddAsset(viz[i] )
    
    vizref.append('visualref'+str(i))
    vizref[i] = fea.ChVisualizationFEAmesh(meshes[i])
    vizref[i].SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
    vizref[i].SetWireframe(False)
    vizref[i].SetDrawInUndeformedReference(True)
    meshes[i].AddAsset(vizref[i])
    
    vizc.append('visualc'+str(i))
    vizc[i] = fea.ChVisualizationFEAmesh(meshes[i])
    vizc[i].SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
    vizc[i].SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
    vizc[i].SetSymbolsThickness(0.004)
    meshes[i].AddAsset(vizc[i])
    
    vizd.append('visuald'+str(i))
    vizd[i] = fea.ChVisualizationFEAmesh(meshes[i])
    vizd[i].SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_ELEM_TENS_STRAIN)
    vizd[i].SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
    vizd[i].SetSymbolsScale(1)
    vizd[i].SetColorscaleMinMax(-0.5, .5)
    vizd[i].SetZbufferHide(False)
    meshes[i].AddAsset(vizd[i])
        
# In[Create interiors]
material.SetFriction(mu_f)
for i in range(n.size):
    print(i)
    for j in range(n[i]):
        R2=diameter*n[i]/(np.pi*2)
        x=R2*np.cos(j*2*np.pi/n[i])
        y=.6*height
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

    myapplication.AssetBindAll()
    myapplication.AssetUpdateAll()

            # If you want to show shadows because you used "AddLightWithShadow()'
            # you must remember this:
    myapplication.AddShadowAll();

# In[Run the simulation]

    count=0


    my_system.SetupInitial()
    msolver = mkl.ChSolverMKLcsm()
    my_system.SetSolver(msolver)
    my_system.DoStaticLinear()
    myapplication.SetTimestep(h)
    
    my_system.SetMaxPenetrationRecoverySpeed(.1)
    my_system.SetMinBounceSpeed(.1)
   # myapplication.SetTryRealtime(True)

    while(myapplication.GetDevice().run()):
        myapplication.BeginScene()
        myapplication.DrawAll()
        '''
    
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
        '''
        myapplication.DoStep()
        myapplication.EndScene()
        '''

        if t > 1000:
            myapplication.GetDevice().closeDevice()
            '''
'''
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
    print(i)
    
    
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
            
stop = timeit.default_timer()

runtime=stop-start
runtime=runtime/60


variables=([nb,ni,diameter,height,volume,mr,mp,k,b,mu_f,mu_b,mu_r,mu_s,rl,runtime])

texts=["number of boundary(n/a)=","\r\n number of interior(n/a)=","\r\n diameter(m)=","\r\n height of each robot(m)=","\r\n volume(m^3)=","\r\n mass of each robot(kg)=","\r\n mass of each particle(kg)=","\r\n spring constant(N/m)=","\r\n damping coefficent(Ns/m)=","\r\nsliding friction=","\r\nmaterial dampning=","\r\nRolling friction=","\r\nSpinning friction=","\r\nresting length=","\r\nruntime(minutes)="]

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

'''
