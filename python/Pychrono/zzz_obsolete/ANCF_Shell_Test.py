import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.mkl as mkl



#print(["Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n"])

time_step = 1e-5

chrono.SetChronoDataPath("data/")

my_system = chrono.ChSystemSMC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))

# Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
application = chronoirr.ChIrrApp(my_system, "ANCF Shells", chronoirr.dimension2du(1200, 800))

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(-0.4, -2, 0.0),  # camera location
                             chronoirr.vector3df(0.0, 0.0, -0.1))  # "look at" location

# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

#Brick Material
brick_material = chrono.ChMaterialSurfaceSMC()
brick_material.SetFriction(0.5)
#brick_material.SetDampingF(0.2)
#brick_material.SetCompliance (0.0000001)
#brick_material.SetComplianceT(0.0000001)

body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -2, 0 ))
body_floor.SetMaterialSurface(brick_material)

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
body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'concrete.jpg')
body_floor.GetAssets().push_back(body_floor_texture)

my_system.Add(body_floor)

size_table_x = 1.5;
size_table_y = 0.2;
size_table_z = 1.5;

body_table = chrono.ChBody()
body_table.SetPos(chrono.ChVectorD(0, -size_table_y/1, 0 ))
body_table.SetMaterialSurface(brick_material)

# Collision shape
body_table.GetCollisionModel().ClearModel()
body_table.GetCollisionModel().AddBox(size_table_x/2, size_table_y/2, size_table_z/2) # hemi sizes
body_table.GetCollisionModel().BuildModel()
body_table.SetCollide(True)

# Visualization shape
body_table_shape = chrono.ChBoxShape()
body_table_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_table_x/2, size_table_y/2, size_table_z/2)
body_table_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
body_table.GetAssets().push_back(body_table_shape)

body_table_texture = chrono.ChTexture()
body_table_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
body_table.GetAssets().push_back(body_table_texture)

my_system.Add(body_table)

#Bricks
nbricks_on_x = 10
nbricks_on_y = 15

size_brick_x = 0.03
size_brick_y = 0.015
size_brick_z = 0.015
density_brick = 8000;    # kg/m^3
mass_brick = density_brick * size_brick_x * size_brick_y * size_brick_z;
inertia_brick = 2/5*(pow(size_brick_x,2))*mass_brick; # to do: compute separate xx,yy,zz inertias

for ix in range(0,nbricks_on_x):
    for iy in range(0,nbricks_on_y):
        # create it
        body_brick = chrono.ChBody()
        # set initial position
        body_brick.SetPos(chrono.ChVectorD(ix*(size_brick_x+.01), (iy+0.5)*(size_brick_y+.005), 0.15 ))
        # set mass properties
        body_brick.SetMass(mass_brick)
        body_brick.SetInertiaXX(chrono.ChVectorD(inertia_brick,inertia_brick,inertia_brick))
        # set collision surface properties
        body_brick.SetMaterialSurface(brick_material)

        # Collision shape
        body_brick.GetCollisionModel().ClearModel()
        body_brick.GetCollisionModel().AddBox(size_brick_x/2, size_brick_y/2, size_brick_z/2) # must set half sizes
        body_brick.GetCollisionModel().BuildModel()
        body_brick.SetCollide(True)

        # Visualization shape, for rendering animation
        body_brick_shape = chrono.ChBoxShape()
        body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_brick_x/2, size_brick_y/2, size_brick_z/2)
        if iy%2==0 :
            body_brick_shape.SetColor(chrono.ChColor(0.65, 0.65, 0.6)) # set gray color only for odd bricks
        body_brick.GetAssets().push_back(body_brick_shape)

        my_system.Add(body_brick)

# Create a constraint that blocks free 3 x y z translations and 3 rx ry rz rotations
# of the table respect to the floor, and impose that the relative imposed position
# depends on a specified motion law.

link_shaker = chrono.ChLinkLockLock()
link_shaker.Initialize(body_table, body_floor, chrono.CSYSNORM)
my_system.Add(link_shaker)

# ..create the function for imposed x horizontal motion, etc.
mfunY = chrono.ChFunction_Sine(0,10,0.01)  # phase, frequency, amplitude
link_shaker.SetMotion_Y(mfunY)

# ..create the function for imposed y vertical motion, etc.
mfunZ = chrono.ChFunction_Sine(0,20,.01)  # phase, frequency, amplitude
link_shaker.SetMotion_Z(mfunZ)



print( "-----------------------------------------------------------\n")
print("-----------------------------------------------------------\n")
print("     ANCF Shell Elements demo with implicit integration \n")
print( "-----------------------------------------------------------\n")

# Create a mesh, that is a container for groups of elements and their referenced nodes.
my_mesh = fea.ChMesh()
# Geometry of the plate
plate_lenght_x = .5
plate_lenght_y = .2
plate_lenght_z = 0.005
# Specification of the mesh
numDiv_x = 20
numDiv_y = 1
numDiv_z = 1
#Total number of nodes
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
rho = 2000
E = chrono.ChVectorD(2.1e7, 2.1e7, 2.1e7)
nu = chrono.ChVectorD(0.3, 0.3, 0.3)
G = chrono.ChVectorD(8.0769231e6, 8.0769231e6, 8.0769231e6)
mat = fea.ChMaterialShellANCF(rho, E, nu, G)

# Create and add the nodes
nodes=[]
m=0 #counter
for i in range(N_y):
    for j in range (N_x):
        # Node location
        x = j * dx
        y = i * dy
        z = dz

        # Node direction
        dir_x = 0
        dir_y = 0
        dir_z = 1

        # Create the node
        node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x, y, z),             #location
                                 chrono.ChVectorD(dir_x, dir_y, dir_z)) #surface normal
        node.SetMass(0)
        
        # Add node to mesh and empty matrix
        my_mesh.AddNode(node)
        nodes.append(node)
        
        if i>0:
            # Create the elements
            element = fea.ChElementShellANCF()
            element.SetNodes(nodes[m-1-(N_x*i)],
                             nodes[m-(N_x*i)],
                             nodes[m],
                             nodes[m-1])

            # Set element dimensions
            element.SetDimensions(dx, dy)

            # Add a single layers with a fiber angle of 0 degrees.
            element.AddLayer(dz, 0 * chrono.CH_C_DEG_TO_RAD, mat)

            # Set other element properties
            element.SetAlphaDamp(0.0)    # Structural damping for this element
            element.SetGravityOn(False)  # turn internal gravitational force calculation off
            
            # Add element to mesh
            my_mesh.AddElement(element)
        
        m=m+1

# Add the mesh and contact to the system
mcontactcloud = fea.ChContactSurfaceNodeCloud()
my_mesh.AddContactSurface(mcontactcloud)
mcontactcloud.AddAllNodes(0.01)

mcontactmesh = fea.ChContactSurfaceMesh()
my_mesh.AddContactSurface(mcontactmesh)
mcontactmesh.AddFacesFromBoundary(.01)

 
mysurfmaterial = chrono.ChMaterialSurfaceSMC()
#mysurfmaterial.SetCompliance(0.000001)
mysurfmaterial.SetFriction(0.3)
#mysurfmaterial.SetRestitution(0.2)
#mysurfmaterial.SetCohesion(0)

mcontactmesh.SetMaterialSurface(mysurfmaterial)
my_system.Add(my_mesh)



# -------------------------------------
# Options for visualization in irrlicht
# -------------------------------------

mvisualizemesh = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
mvisualizemesh.SetColorscaleMinMax(0.0, 1.50)
mvisualizemesh.SetShrinkElements(True, 0.85)
mvisualizemesh.SetSmoothFaces(True)
my_mesh.AddAsset(mvisualizemesh)

mvisualizemeshref = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshref.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mvisualizemeshref.SetWireframe(False)
mvisualizemeshref.SetDrawInUndeformedReference(True)
my_mesh.AddAsset(mvisualizemeshref)

mvisualizemeshC = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizemeshC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizemeshC.SetSymbolsThickness(0.004)
my_mesh.AddAsset(mvisualizemeshC)

mvisualizemeshD = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshD.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_ELEM_TENS_STRAIN)
mvisualizemeshD.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizemeshD.SetSymbolsScale(1)
mvisualizemeshD.SetColorscaleMinMax(-0.5, .5)
mvisualizemeshD.SetZbufferHide(False)
my_mesh.AddAsset(mvisualizemeshD)

application.AssetBindAll()
application.AssetUpdateAll()

# ----------------------------------
# Perform a dynamic time integration
# ----------------------------------

#my_system.SetupInitial();

# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverMKL()
my_system.SetSolver(msolver)

application.SetTimestep(0.0005)

my_system.DoStaticLinear()

my_system.SetMaxPenetrationRecoverySpeed(.1)
my_system.SetMinBounceSpeed(.1)

while(application.GetDevice().run()):
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

