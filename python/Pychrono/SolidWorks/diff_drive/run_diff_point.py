#-------------------------------------------------------------------------------
# Name:        	run_diff_point.py
# Purpose:	Pose controller for differential drive robot
#
# Author:      	Qiyuan Zhou
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
import matplotlib
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------
# Parse command-line parameters

m_filename = "diff.py"
m_timestep = 0.001
m_length = 1.0
m_visualization = "irrlicht"
m_datapath = "C:/Program Files/ChronoSolidworks/data/"

try:
	opts, args = getopt.getopt(sys.argv[1:],"f:d:T:v:p:",["filename=","timestep=","Tlength=","visualization=","datapath="])
except getopt.GetoptError:
	#print ("run_test.py -f <filename> [-d <timestep> -T <length> -v <pov|irrlicht> -p <chronodatapath>]")
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
	print ("run_test.py -f <filename> [-d <timestep> -T <length> -v <pov|irrlicht> -p <chronodatapath>]")
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
#timestepper = chrono.ChTimestepperEulerImplicitProjected()
my_system.SetSolverTolerance(1e-10)
my_solver = chrono.ChSolverBB()
my_system.SetSolver(my_solver)
my_solver.SetMaxIterations(200)
my_system.SetTimestepperType(1) # Euler Implicit Projected
my_solver.EnableWarmStart(True);
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81,0))
chrono.ChCollisionModel_SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel_SetDefaultSuggestedMargin(0.001)

# Default Material
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetDampingF(0.)
material.SetCompliance(0.00001)

col_y = chrono.ChColorAsset()
col_y.SetColor(chrono.ChColor(0.44, .11, 52))

# Tire Material
tire_mat = chrono.ChMaterialSurfaceNSC()
tire_mat.SetFriction(0.8)
tire_mat.SetDampingF(0.)
tire_mat.SetCompliance(0.00001)

# Empty vectors to store wheel, torque, marker objects
right_wheels = []
left_wheels = []

right_torques = []
left_torques = []

right_markers = []
left_markers = []

bot_numbers = [1,4,5,7,8]   # Thanks stupid naming scheme in Solidworks

# Bot 1
bot1 = my_system.SearchBody('_bot-1')
wheel_left = my_system.SearchBody('wheel-1')
wheel_right = my_system.SearchBody('wheel-2')

reference = bot1.SearchMarker('right_marker')

if not wheel_left or not wheel_right:
    sys.exit('Error: cannot find wheel from its name in the C::E system!')
    
left_torque = chrono.ChForce()
wheel_left.AddForce(left_torque)
left_torque.SetMode(chrono.ChForce.TORQUE)
left_torque.SetRelDir(chrono.ChVectorD(0, 0, 1))
left_torque.SetMforce(0.0005)

right_torque = chrono.ChForce()
wheel_right.AddForce(right_torque)
right_torque.SetMode(chrono.ChForce.TORQUE)
right_torque.SetRelDir(chrono.ChVectorD(0, 0, -1))
right_torque.SetMforce(0.0005)


right_wheels.append(wheel_right)
left_wheels.append(wheel_left)
right_torques.append(right_torque)
left_torques.append(left_torque)

my_ground = my_system.SearchBody('ground')
if not my_ground:
    sys.exit('Error: cannot find leg from its name in the C::E system!')
my_ground.SetMaterialSurface(material)

# Reference positions
z_r = np.array([[-0.5], [0.5], [1*np.pi/2]])

target = chrono.ChBodyEasyCylinder(0.01, 0.05, 1000)
target.SetPos(chrono.ChVectorD(z_r[0][0],0.05,z_r[1][0]))
target.SetBodyFixed(True)
target.AddAsset(col_y)
my_system.Add(target)

#------------------------------------
# Robot Geometric Parameters
#------------------------------------
r = 0.016  # Wheel radius
b = 0.07324/2 # 1/2 Wheelbase
    
#  Create an Irrlicht application to visualize the system
myapplication = chronoirr.ChIrrApp(my_system, 'Test', chronoirr.dimension2du(1600,1200))
myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0,2,1.5),chronoirr.vector3df(0,0,0))
myapplication.AddTypicalLights()
myapplication.SetSymbolscale(0.02)
myapplication.SetShowInfos(True)
myapplication.SetContactsDrawMode(3)
myapplication.SetPaused(True)
myapplication.AddLightWithShadow(chronoirr.vector3df(10,20,10),chronoirr.vector3df(0,2.6,0), 10 ,10,40, 60, 512);
myapplication.AssetBindAll();
myapplication.AssetUpdateAll();
myapplication.AddShadowAll();

#------------------------------------
# Run the simulation forever until windows is closed
#------------------------------------
myapplication.SetTimestep(m_timestep);

measure_torque_left = []
measure_torque_right = []
locations_x = []
locations_z = []
heading_angles=[]
times = []

target_angles = []
deltas = []

#------------------------------------
# Pose controller
#------------------------------------



# Reference velocity
v_r=1

# Gains 
k1 = 1
k2 = 3

#bot1.SetBodyFixed(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    t = my_system.GetChTime()
        
    # Get locations
    xpos = bot1.GetPos().x
    zpos = bot1.GetPos().z
    
    # Heading angle
    q = reference.GetAbsFrame().GetRot()
    d2 = q.Rotate(chrono.ChVectorD(0,-1,0))
    heading = np.arctan2(d2.z,d2.x)
    
    z=np.array([[xpos],[zpos],[heading]])
    
    #------------------------------------
    # Pose Controller
    #------------------------------------
    targ_vect = z_r[0:2:1]-z[0:2:1]
    targ_dist = np.linalg.norm(targ_vect)
    targ_vect = targ_vect/np.linalg.norm(targ_vect)
    targ_ang = np.arctan2(targ_vect[1],targ_vect[0])[0]

    delta = heading - targ_ang
    theta = z_r[2][0] - np.arctan2((z_r[1][0]-z[1][0]),(z_r[0][0]-z[0][0]))
    if targ_dist < 0.1:
        k1=0
        v_r=0.1
    else:
        k1=1
        v_r=1
    w = (-v_r/targ_dist) * (k2*(delta-np.arctan(-k1*theta))+(1+(k1/(1+(k1*theta)**2)))*np.sin(delta))
    u = np.array([[-v_r],[w]])
    
    # Desired wheel velocities
    des_vel_right = u[0][0]/r + b*u[1][0]/r
    des_vel_left = u[0][0]/r - b*u[1][0]/r
    
    # Velocity and acceleration error
    right_wv_err =  wheel_right.GetWvel_loc().z - des_vel_right
    left_wv_err =   wheel_left.GetWvel_loc().z - des_vel_left
    
    right_wa_err =  - right_wv_err*m_timestep #+ wheel_right.GetWacc_loc().z
    left_wa_err =  - left_wv_err*m_timestep # + wheel_left.GetWacc_loc().z
    
    # Torque to apply
    right_apply = (3e-5)*right_wv_err
    left_apply = (3e-5)*left_wv_err
    '''
    # Apply the torque
    right_torque.SetMforce(right_apply)
    left_torque.SetMforce(left_apply)
    '''
    # Set the wheel velocities
    wheel_right.SetWvel_loc(chrono.ChVectorD(0,0,des_vel_right))
    wheel_left.SetWvel_loc(chrono.ChVectorD(0,0,-des_vel_left))
    
    # Store values
    measure_torque_left.append(left_apply)
    measure_torque_right.append(right_apply)
    locations_x.append(bot1.GetPos().x)
    locations_z.append(bot1.GetPos().z)
    heading_angles.append(heading)
    times.append(t)
    
    target_angles.append(targ_ang)
    deltas.append(delta)
    
    myapplication.DoStep()
    myapplication.EndScene()

# Plot the path
fig, ax = plt.subplots()
ax.plot(locations_x, locations_z)
qq=ax.quiver(locations_x[0:-1:100], locations_z[0:-1:100], np.cos(heading_angles)[0:-1:100], np.sin(heading_angles)[0:-1:100])
ax.set(xlabel='x', ylabel='y', title='Position')
ax.grid()
ax.axis('equal')
plt.show()

# Plot the wheel torques
fig, axs = plt.subplots(2)
fig.suptitle('Wheel Torques')
axs[0].plot(times, measure_torque_left)
plt.ylim(-max(measure_torque_left or measure_torque_right),max(measure_torque_left or measure_torque_right))
axs[1].plot(times, measure_torque_right)
plt.ylim(-max(measure_torque_left or measure_torque_right),max(measure_torque_left or measure_torque_right))

# Plot the heading angles
fig, ax = plt.subplots()
fig.suptitle('Heading Angles')
ax.plot(times, 180*np.array(heading_angles)/np.pi)

# Plot target_angles
fig, ax = plt.subplots()
fig.suptitle('Angle to Target')
ax.plot(times, 180*np.array(target_angles)/np.pi)

# Plot delta
fig, ax = plt.subplots()
fig.suptitle('Deltas')
ax.plot(times, 180*np.array(deltas)/np.pi)