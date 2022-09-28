def main():
    pass


if __name__ == '__main__':
    main()

import os
import math
import time
import sys, getopt
import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np
from Sim_0_objects import *

# ---------------------------------------------------------------------
#
# In[Parse command-line parameters]

m_filename = "Sim.py"
m_timestep = 0.01
m_length = 1.0
m_visualization = "irrlicht"
m_datapath = "C:/Program Files/ChronoSolidworks/data/"

try:
    opts, args = getopt.getopt(sys.argv[1:], "f:d:T:v:p:",
                               ["filename=", "timestep=", "Tlength=", "visualization=", "datapath="])
except getopt.GetoptError:
    # print ("run_test.py -f <filename> [-d <timestep> -T <length> -v <pov|irrlicht> -p <chronodatapath>]")
    sys.exit(2)
for opt, arg in opts:
    print("opt:", opt, "  arg", arg)
    if opt in ("-d", "--timestep"):
        m_timestep = float(arg)
    elif opt in ("-T", "--Tlength"):
        m_length = float(arg)
    elif opt in ("-f", "--filename"):
        m_filename = arg
    elif opt in ("-v", "--visualization"):
        m_visualization = arg
    elif opt in ("-p", "--datapath"):
        m_datapath = arg

if m_filename == "":
    print("run_test.py -f <filename> [-d <timestep> -T <length> -v <pov|irrlicht> -p <chronodatapath>]")
    sys.exit(2)

if not os.path.isfile(m_filename):
    print("Error. Filename " + m_filename + " does not exist.")
    sys.exit(2)

chrono.SetChronoDataPath(m_datapath)

print("  file to load is ", m_filename)
print("  timestep is ", m_timestep)
print("  length is ", m_length)
print("  data path for fonts etc.: ", m_datapath)

# ---------------------------------------------------------------------
#
#  load the file generated by the SolidWorks CAD plugin
#  and add it to the ChSystem.
#

# Remove the trailing .py and add / in case of file without ./
m_absfilename = os.path.abspath(m_filename)
m_modulename = os.path.splitext(m_absfilename)[0]

print("Loading C::E scene...")

exported_items = chrono.ImportSolidWorksSystem(m_modulename)

print("...loading done!")

# Print exported items
for my_item in exported_items:
    print(my_item.GetName())

# Add items to the physical system
my_system = chrono.ChSystemNSC()
for my_item in exported_items:
    my_system.Add(my_item)

# Optionally set some solver parameters.

# my_system.SetMaxPenetrationRecoverySpeed(1.00)
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR);
# my_system.SetMaxItersSolverSpeed(600);
# my_system.SetSolverWarmStarting(True);
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


# Material definition

mu_f = .3  # friction
mu_b = 0.05  # dampning
mu_r = .4  # rolling friction
mu_s = .01  # SPinning fiction

Ct = .00001 #Shear compliance
C = .00001  #Bending complince
Cr = .0001  #Rolling complince
Cs = .0001  #Spinning compliance

material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(mu_f)
material.SetDampingF(mu_b)
material.SetCompliance(C)
material.SetComplianceT(Ct)
material.SetRollingFriction(mu_r)
material.SetSpinningFriction(mu_s)
material.SetComplianceRolling(Cr)
material.SetComplianceSpinning(Cs)

material2 = chrono.ChMaterialSurfaceNSC()
material2.SetFriction(0.8)
material2.SetDampingF(0.01)
material2.SetCompliance(C)
material2.SetComplianceT(Ct)
material2.SetRollingFriction(mu_r)
material2.SetSpinningFriction(mu_s)
material2.SetComplianceRolling(Cr)
material2.SetComplianceSpinning(Cs)

ground_marker = my_system.SearchMarker('ground_marker')
my_ground = my_system.SearchBody('graound-1')
if not my_ground:
    sys.exit('Error: cannot find leg from its name in the C::E system!')
my_ground.SetMaterialSurface(material)



my_shaft = my_system.SearchBody('single_bot1-1')
if not my_shaft:
    sys.exit('Error: cannot find shaft  from its name in the C::E system!')

my_marker = my_shaft.SearchMarker('My_marker')
if not my_marker:
    sys.exit('Error: cannot find marker from its name in the C::E system!')
a = my_marker.GetRot()

my_leg = my_system.SearchBody('leg-1')
if not my_leg:
    sys.exit('Error: cannot find leg from its name in the C::E system!')

goal=[-2,1,-1.5]
obj=robot_data(1, my_marker, goal)

#d = (my_marker.GetPos())  # distance from object
#angle = np.arctan2(d.z, d.x)
#q1 = my_marker.GetRot()
#phi = quaternion_to_euler(q1.e0, q1.e1, q1.e2, q1.e3)[0]
#theta = (phi + angle) * 180 / np.pi
##print('Robot#', i)
#print('x:', d.x)
#print('z:', d.z)
#print('phi:', phi)
#print('angle:', angle)
#print('theta:', theta)

revolute_frame = my_marker.GetAbsFrame()
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(my_leg, my_shaft, revolute_frame)
link_motor.SetSpindleConstraint(chrono.ChLinkMotorRotationSpeed.SpindleConstraint_CYLINDRICAL)
link_motor.SetMotorFunction(chrono.ChFunction_Const(-0.3* chrono.CH_C_2PI))  # 1.0 Hz to rad/s
my_system.Add(link_motor)

my_leg.SetMaterialSurface(material2)
X=[]
Y=[]
Z=[]
T=[]

xaxis = chrono.ChBodyEasyCylinder(0.0025,
0.05,
10,
False,
True)
qa = chrono.Q_from_AngAxis(90 * chrono.CH_C_DEG_TO_RAD, revolute_frame.GetRot().Rotate(chrono.ChVectorD(0,0,1)));
xaxis.SetPos(revolute_frame.GetPos())
xaxis.SetRot(qa*revolute_frame.GetRot())
marker_xaxis =chrono.ChMarker()
marker_xaxis.SetName('marker_xaxis')
xaxis.AddMarker(marker_xaxis)
marker_xaxis.Impose_Abs_Coord(chrono.ChCoordsysD(revolute_frame.GetPos(), revolute_frame.GetRot()))
my_system.Add(xaxis)

link_xaxis = chrono.ChLinkMateFix()
link_xaxis.Initialize(xaxis, my_shaft)
# link_xaxis.Initialize(marker_xaxis, my_marker)
my_system.Add(link_xaxis)



if m_visualization == "pov":

    # ---------------------------------------------------------------------
    #
    #  Render a short animation by generating scripts
    #  to be used with POV-Ray
    #

    pov_exporter = postprocess.ChPovRay(my_system)

    # Sets some file names for in-out processes.
    pov_exporter.SetTemplateFile("_template_POV.pov")
    pov_exporter.SetOutputScriptFile("rendering_frames.pov")
    if not os.path.exists("output"):
        os.mkdir("output")
    if not os.path.exists("anim"):
        os.mkdir("anim")
    pov_exporter.SetOutputDataFilebase("output/my_state")
    pov_exporter.SetPictureFilebase("anim/picture")

    # Sets the viewpoint, aimed point, lens angle
    pov_exporter.SetCamera(chrono.ChVectorD(0.4,0.6,0.9), chrono.ChVectorD(0.2,0,0), 90)

    # Sets the default ambient light and default light lamp
    pov_exporter.SetAmbientLight(chrono.ChColor(1, 1, 1))
    pov_exporter.SetLight(chrono.ChVectorD(-2, 2, -1), chrono.ChColor(1.1, 1.2, 1.2), True)

    # Sets other settings
    pov_exporter.SetPictureSize(640, 480)
    pov_exporter.SetAmbientLight(chrono.ChColor(2, 2, 2))

    # Turn on the rendering of xyz axes for the centers of gravity or reference frames:
    # pov_exporter.SetShowCOGs  (1, 0.05)
    # pov_exporter.SetShowFrames(1, 0.02)
    # pov_exporter.SetShowLinks(1, 0.03)
    pov_exporter.SetShowContacts(True,
                                 postprocess.ChPovRay.SYMBOL_VECTOR_SCALELENGTH,
                                 0.2,  # scale
                                 0.0007,  # width
                                 0.1,  # max size
                                 True, 0, 0.5)  # colormap on, blue at 0, red at 0.5

    # Add additional POV objects/lights/materials in the following way, entering
    # an optional text using the POV scene description laguage. This will be
    # appended to the generated .pov file.
    # For multi-line strings, use the python ''' easy string delimiter.
    pov_exporter.SetCustomPOVcommandsScript(
        '''
        light_source{ <1,3,1.5> color rgb<1.1,1.1,1.1> }
        ''')

    # Tell which physical items you want to render
    pov_exporter.AddAll()

    # Create the two .pov and .ini files for POV-Ray (this must be done
    # only once at the beginning of the simulation).
    pov_exporter.ExportScript()

    # Perform a short simulation
    nstep = 0
    while (my_system.GetChTime() < m_length):
        my_system.DoStepDynamics(m_timestep)

        # if math.fmod(nstep,10) ==0 :
        print('time=', my_system.GetChTime())

        # Create the incremental nnnn.dat and nnnn.pov files that will be load
        # by the pov .ini script in POV-Ray (do this at each simulation timestep)
        pov_exporter.ExportData()

        nstep = nstep + 1

    print("\n\nOk, Simulation done!");
    time.sleep(2)

if m_visualization == "irrlicht":

    # ---------------------------------------------------------------------
    #
    #  Create an Irrlicht application to visualize the system
    #

    myapplication = chronoirr.ChIrrApp(my_system, 'Test', chronoirr.dimension2du(1600, 1200))
    myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
    myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    myapplication.AddTypicalCamera(chronoirr.vector3df(1, 1, 1), chronoirr.vector3df(0.0, 0.0, 0.0))
    myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, 2), chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
    # myapplication.AddLightWithShadow(chronoirr.vector3df(10,20,10),chronoirr.vector3df(0,2.6,0), 10 ,10,40, 60, 512);

    # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    # If you need a finer control on which item really needs a visualization proxy in
    # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    myapplication.AssetBindAll();

    # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    myapplication.AssetUpdateAll();
    myapplication.AddShadowAll()

    # ==IMPORTANT!== Use this function for enabling cast soft shadows

    # myapplication.AddShadowAll();

    # ---------------------------------------------------------------------
    #
    #  Run the simulation forever until windows is closed
    #

    myapplication.SetTimestep(m_timestep);

    while (myapplication.GetDevice().run()):
        myapplication.BeginScene()
        myapplication.DrawAll()
#        d = (my_marker.Point_Ref2World(chrono.ChVectorD(0,0,0)))  # distance from object
#        #D=my_marker.GetPos()
        #C=my_marker.GetA()
        #c=my_shaft.Get
        #angle = np.arctan2(d.z, d.x)
#        q1 = my_marker.GetRot()
#        phi = quaternion_to_euler(q1.e0, q1.e1, q1.e2, q1.e3)[0]
#        theta = (phi + angle) * 180 / np.pi
#        #print('Robot#', i)
#        #print('x:', d.x)
#        #print('z:', d.z)
#        #print('y:',d.y)
#        X.append((d.x)*1000)
#        Y.append(d.y*1000)
#        Z.append(d.z*1000)
#        print('phi:', phi)
#        print('angle:', angle)
#        print('theta:', theta)
        #obj.Extract_data()
        #q1 = my_shaft.GetRot()
        #print(quaternion_to_euler(q1.e0, q1.e1, q1.e2, q1.e3)[1])
        #fm=my_marker.GetCoord()
        #LocalToParent
        # b = my_shaft.GetRot()-my_ground.GetRot()
        b = my_marker.GetAbsFrame().GetRot()
        c = quaternion_to_euler(b.e0, b.e1, b.e2, b.e3)
        c = c/np.linalg.norm(c)
        print(c)
        xmarker = my_marker.GetAbsFrame().GetPos()
        xbody = my_shaft.GetPos()
        print('Body:', xbody, 'Marker:', xmarker)
        # print(my_marker.GetPos().Normalize())
        #TransformDirectionParentToLocal
        #print(q1)
        #print('time=', my_system.GetChTime())
        #T.append(my_system.GetChTime())
        myapplication.DoStep()
        myapplication.EndScene()
        

