#-------------------------------------------------------------------------------
# Name:        modulo1
# Purpose:
#
# Author:      tasora
#
# Created:     1/1/2019
# Copyright:   (c) tasora 2019
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python

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


# ---------------------------------------------------------------------
#
# Parse command-line parameters

# m_filename = "Unit.py"
m_filename = "Env.py"
m_timestep = 0.01
m_length = 1.0
m_visualization = "irrlicht"
m_datapath = "C:/workspace/chrono/data/"

try:
	opts, args = getopt.getopt(sys.argv[1:],"f:d:T:v:p:",["filename=","timestep=","Tlength=","visualization=","datapath="])
except getopt.GetoptError:
	#print ("run.py -f <filename> [-d <timestep> -T <length> -v <pov|irrlicht> -p <chronodatapath>]")
	sys.exit(2)
for opt, arg in opts:
	print ("opt:", opt, "  arg", arg)
	if   opt in ("-d", "--timestep"):
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
	print ("run.py -f <filename> [-d <timestep> -T <length> -v <pov|irrlicht> -p <chronodatapath>]")
	sys.exit(2)

if not os.path.isfile(m_filename):
	print ("Error. Filename " + m_filename + " does not exist.")
	sys.exit(2)
    
chrono.SetChronoDataPath(m_datapath)

print ("  file to load is ", m_filename)
print ("  timestep is ", m_timestep)
print ("  length is ", m_length)
print ("  data path for fonts etc.: ", m_datapath)



# ---------------------------------------------------------------------
#
#  load the file generated by the SolidWorks CAD plugin
#  and add it to the ChSystem.
#

# Remove the trailing .py and add / in case of file without ./
m_absfilename = os.path.abspath(m_filename)
m_modulename = os.path.splitext(m_absfilename)[0]

print ("Loading C::E scene...");

exported_items = chrono.ImportSolidWorksSystem(m_modulename)

print ("...loading done!");


# Print exported items
for my_item in exported_items:
	print (my_item.GetName())

# Add items to the physical system
my_system = chrono.ChSystemNSC()
for my_item in exported_items:
	my_system.Add(my_item)
		
		
# Optionally set some solver parameters.

#my_system.SetMaxPenetrationRecoverySpeed(1.00)
# my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
# my_system.SetMaxItersSolverSpeed(600);
# my_system.SetSolverWarmStarting(True);
g = -0.981
my_system.Set_G_acc(chrono.ChVectorD(0,g,0))



	

# -----------------------------------
# Bodies
bodies=[]
mems=[]
floor = my_system.SearchBody('Floor-1')
bodies.append(floor)
# Unit1
mem_out1 = my_system.SearchBody('Unit-1/Mem_Out-1')
mems.append(mem_out1)
bodies.append(mem_out1)
mem_in1 = my_system.SearchBody('Unit-1/Mem_In-1')
mems.append(mem_in1)
bodies.append(mem_in1)
con_out1 = my_system.SearchBody('Unit-1/Test_Cylinder-1')
bodies.append(con_out1)
con_in1 = my_system.SearchBody('Unit-1/Test_Cylinder-2')
bodies.append(con_in1)

# Unit2
mem_out2 = my_system.SearchBody('Unit-2/Mem_Out-1')
mems.append(mem_out2)
bodies.append(mem_out2)
mem_in2 = my_system.SearchBody('Unit-2/Mem_In-1')
mems.append(mem_in2)
bodies.append(mem_in2)
con_out2 = my_system.SearchBody('Unit-2/Test_Cylinder-1')
bodies.append(con_out2)
con_in2 = my_system.SearchBody('Unit-2/Test_Cylinder-2')
bodies.append(con_in2)

# Unit3
mem_out3 = my_system.SearchBody('Unit-3/Mem_Out-1')
mems.append(mem_out3)
bodies.append(mem_out3)
mem_in3 = my_system.SearchBody('Unit-3/Mem_In-1')
mems.append(mem_in3)
bodies.append(mem_in3)
con_out3 = my_system.SearchBody('Unit-3/Test_Cylinder-1')
bodies.append(con_out3)
con_in3 = my_system.SearchBody('Unit-3/Test_Cylinder-2')
bodies.append(con_in3)
# Check created bodies
for check in bodies:
	if not check:
		print('Bodies is not found.')



# -----------------------------------
# Membrane material
mat_mem = chrono.ChMaterialSurfaceNSC()
mat_mem.SetFriction(0.8)
# Floor material
mat_floor = chrono.ChMaterialSurfaceNSC()
mat_floor.SetFriction(0.2)

# -----------------------------------
# Set material surface
# Membrane surface
for item in mems:
	item.SetMaterialSurface(mat_mem)

# Floor mat
floor.SetMaterialSurface(mat_floor)

# -----------------------------------
# Markers
# Unit1 mem
markers=[]
mkr_mem_out1 = mem_out1.SearchMarker('Marker_Joint_Male')
markers.append(mkr_mem_out1)
mkr_mem_in1 = mem_in1.SearchMarker('Marker_Joint_Female')
markers.append(mkr_mem_in1)
# Unit1 spring
mkr_con_out1 = con_out1.SearchMarker('Marker_Connector')
markers.append(mkr_con_out1)
mkr_con_in1 = con_in1.SearchMarker('Marker_Connector')
markers.append(mkr_con_in1)

# Unit2 mem
mkr_mem_out2 = mem_out2.SearchMarker('Marker_Joint_Male')
markers.append(mkr_mem_out2)
mkr_mem_in2 = mem_in2.SearchMarker('Marker_Joint_Female')
markers.append(mkr_mem_in2)
# Unit1 spring
mkr_con_out2 = con_out2.SearchMarker('Marker_Connector')
markers.append(mkr_con_out2)
mkr_con_in2 = con_in2.SearchMarker('Marker_Connector')
markers.append(mkr_con_in2)

# Unit3 mem
mkr_mem_out3 = mem_out3.SearchMarker('Marker_Joint_Male')
markers.append(mkr_mem_out3)
mkr_mem_in3 = mem_in3.SearchMarker('Marker_Joint_Female')
markers.append(mkr_mem_in3)
# Unit1 spring
mkr_con_out3 = con_out3.SearchMarker('Marker_Connector')
markers.append(mkr_con_out3)
mkr_con_in3 = con_in3.SearchMarker('Marker_Connector')
markers.append(mkr_con_in3)
# Check created markers
for check in markers:
	if not check:
		print('Marker is not found.')


# -----------------------------------
# Ball joints
joint1 = chrono.ChLinkLockSpherical()
joint1.Initialize(mkr_mem_out1, mkr_mem_in2)
my_system.Add(joint1)

joint2 = chrono.ChLinkLockSpherical()
joint2.Initialize(mkr_mem_out2, mkr_mem_in3)
my_system.Add(joint2)



# -----------------------------------
# Springs connecting edges of origami
# Spring coefficient
sk = 0.3
# Damping coefficient
cd = 0.1

# Unit1
spring1 = chrono.ChLinkTSDA()
p1_1 = mkr_con_out1.GetPos()
p1_2 = mkr_con_in1.GetPos()
spring1.Initialize(con_out1,con_in1, True, p1_1,p1_2, True)
spring1.SetSpringCoefficient(sk)
spring1.SetDampingCoefficient(cd)
my_system.Add(spring1)

# Unit2
spring2 = chrono.ChLinkTSDA()
p2_1 = mkr_con_out2.GetPos()
p2_2 = mkr_con_in2.GetPos()
spring2.Initialize(con_out2,con_in2, True, p2_1,p2_2, True)
spring2.SetSpringCoefficient(sk)
spring2.SetDampingCoefficient(cd)
my_system.Add(spring2)

# Unit3
spring3 = chrono.ChLinkTSDA()
p3_1 = mkr_con_out3.GetPos()
p3_2 = mkr_con_in3.GetPos()
spring3.Initialize(con_out3,con_in3, True, p3_1,p3_2, True)
spring3.SetSpringCoefficient(sk)
spring3.SetDampingCoefficient(cd)
my_system.Add(spring3)


# -----------------------------------
# Add force
# Force param
f_amp = 0

# Unit1
vacuum1 = chrono.ChForce()
mem_in1.AddForce(vacuum1)
vacuum1.SetMode(chrono.ChForce.FORCE)
vacuum1.SetFrame(chrono.ChForce.BODY)
vacuum1.SetAlign(chrono.ChForce.BODY_DIR)
vacuum1.SetRelDir(chrono.ChVectorD(0,0,1))
# vacuum1.SetMforce(0)

# Unit2
vacuum2 = chrono.ChForce()
mem_in2.AddForce(vacuum2)
vacuum2.SetMode(chrono.ChForce.FORCE)
vacuum2.SetFrame(chrono.ChForce.BODY)
vacuum2.SetAlign(chrono.ChForce.BODY_DIR)
vacuum2.SetRelDir(chrono.ChVectorD(0,0,1))
# vacuum2.SetMforce(0)

# Unit3
vacuum3 = chrono.ChForce()
mem_in3.AddForce(vacuum3)
vacuum3.SetMode(chrono.ChForce.FORCE)
vacuum3.SetFrame(chrono.ChForce.BODY)
vacuum3.SetAlign(chrono.ChForce.BODY_DIR)
vacuum3.SetRelDir(chrono.ChVectorD(0,0,1))
# vacuum3.SetMforce(0)


# -----------------------------------
# Lock vacuumed membrane
lock1 = chrono.ChLinkLockLock()
# lock1.Initialize(mem_in1, mem_out1, mem_in1.GetCoord())
# my_system.Add(lock1)
lock2 = chrono.ChLinkLockLock()
# lock2.Initialize(mem_in2, mem_out2, mem_in2.GetCoord())
# my_system.Add(lock2)
lock3 = chrono.ChLinkLockLock()
# lock3.Initialize(mem_in3, mem_out3, mem_in3.GetCoord())
# my_system.Add(lock3)


# -----------------------------------
# Disable visualization of the floor
floor.GetAssets().clear()



chrono.ChCollisionModel_SetDefaultSuggestedEnvelope(0.001)
	
if m_visualization == "irrlicht":

	# ---------------------------------------------------------------------
	#
	#  Create an Irrlicht application to visualize the system
	#

	myapplication = chronoirr.ChIrrApp(my_system, 'Test', chronoirr.dimension2du(1280,720))
	myapplication.SetShowInfos(True)
	myapplication.SetPaused(True)

	myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
	myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
	myapplication.AddTypicalCamera(chronoirr.vector3df(1,1,1),chronoirr.vector3df(0.0,0.0,0.0))
	myapplication.AddTypicalLights()
	#myapplication.AddLightWithShadow(chronoirr.vector3df(10,20,10),chronoirr.vector3df(0,2.6,0), 10 ,10,40, 60, 512);

				# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
				# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
				# If you need a finer control on which item really needs a visualization proxy in
				# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

	myapplication.AssetBindAll();

				# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
				# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

	myapplication.AssetUpdateAll();

				# ==IMPORTANT!== Use this function for enabling cast soft shadows

	#myapplication.AddShadowAll();

	# ---------------------------------------------------------------------
	#
	#  Run the simulation forever until windows is closed
	#

	myapplication.SetTimestep(m_timestep);
	


# -----------------------------------
# Flag for activate force
force1 = False
force2 = False
force3 = False
# Max force
f_max = 0.0005


# -----------------------------------
# Time interval for control
t_int = 3


# Max shrink length is 40 mm

while(myapplication.GetDevice().run()):
	myapplication.BeginScene()
	myapplication.DrawAll()

	t = my_system.GetChTime()

	# Initial force
	if force1:
		vacuum1.SetMforce(f_amp)
	if force2:
		vacuum2.SetMforce(f_amp)
	if force3:
		vacuum3.SetMforce(f_amp)

	if not force1:
		vacuum1.SetMforce(0)
	if not force2:
		vacuum2.SetMforce(0)
	if not force3:
		vacuum3.SetMforce(0)

	# Force control
	if t > t_int and t <= t_int*2:
		force1 = True
		f_amp = (f_max/t_int)*(t-t_int)
	if t > t_int*2 and t <= t_int*3:
		force1 = False
		# lock1.Initialize(mem_in1, mem_out1, mem_in1.GetCoord())
		# my_system.Add(lock1)
		force2 = True
		f_amp = (f_max/t_int)*(t-t_int*2)
	if t > t_int*3 and t <= t_int*4:
		force2 = False
		# lock2.Initialize(mem_in2, mem_out2, mem_in2.GetCoord())
		# my_system.Add(lock2)
		force3 = True
		f_amp = (f_max/t_int)*(t-t_int*3)
	if t > 25:
		force3 = False
		# lock3.Initialize(mem_in3, mem_out3, mem_in3.GetCoord())
		# my_system.Add(lock3)

	print('vac1=',vacuum1.GetMforce(), 'vac2=',vacuum2.GetMforce(), 'vac3=',vacuum3.GetMforce())

	# Distance calc
	d1_1 = mem_in1.GetPos()
	d1_2 = mem_out1.GetPos()
	dis1 = np.sqrt((d1_1.x - d1_2.x)**2 + (d1_1.z - d1_2.z))



	# Moving frame velocity control


	myapplication.DoStep()
	myapplication.EndScene()

