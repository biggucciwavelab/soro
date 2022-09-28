""""
config.py: 
    
This file contains the configuration parameters
for the local sensing simulation

Author: Qiyuan Zhou
Date (MM/DD/YY): 09/10/20
Wavelab | Illinois Institute of Technology
"""
import pychrono.core as chrono
import numpy as np

# %% Naming, file paths, visualization controls
data_path='C:/Chrono/Builds/chrono-develop/bin/data/'   # Path for simulation data assets
sim_id='10s/07'                      # Name of simulation
visual='irr'                                            # Visualization method, irr/pov
saveFrames=True                                         # Save frames from irr visualization?
paused=False                                            # Start paused?
showInfo=True                                           # Show sim info?
track_cam=True                                          # None to disable, robot_id to follow robot of choice
cam_h=4                                              # Camera height
saveFPS=60                                              # Frame rate of saved visualization
resx=1280                                               # Video frame x resolution
resy=720                                                # Video frame y resolution 

# %% Simulation time control
tstep=0.001     # Solver time step
tend=10         # End simulation after this time
tout=True      # Print timesteps?

# %% Friction + compliance
mu_f=0.1        # Sliding coefficient
mu_b=0.01        # Damping coefficient
C=0.0001        # Normal compliance
Ct=0.0001       # Tangential compliance

# %% Floor dimensions
fl_h=0.5        # Floor height
fl_l=40         # Floor length and width

# %% Robot parameters
rob_nb=20       # number of robots
rob_m=0.5       # [kg] robot mass, positive value overrides rob_rho
rob_d=0.0945    # [m] robot diameter
rob_h=0.04885    # [m] robot height
rob_rho=2000    # [kg/m^3] robot density
rob_offx=0      # [m] formation center x offset
rob_offz=0      # [m] formation center z offset

# Spacing between center of two robots
rob_spacing=3*rob_d 

# calculate formation radius
t=2*np.pi/rob_nb 
rob_rad=rob_spacing/(2*np.arcsin(0.5*t)) 

# %% Spring and membrane parameters
linked=True     # Robots are linked?
mem_geo='sph'   # 'sph' or 'cyl' for spherical/cylindrical membrane particles
skind = 0.03    # [m] diameter of cylinders for skin particles
ratioM = 6      # ratio of membrane skin particles to big bots
skinrho = 1200  # [kg/m^3] density of skin particles [kg/m^3]
km=1000           # [N/m] spring constant (skin membrane)
bm=0            # [N*s/m]damping constant (skin membrane)

# %% Interior particles
interiors=True     # Create interior particles?
int_ratio=0.75      # None/Ratio of small to large particle size
int_spacing=1.01     # Spacing factor for interior particles
int_d=0.08          # Interior particle diameter
int_m=0.05           # [kg] interior particle mass, positive value overrides int_rho
int_h=rob_h         # [m] interior particle height
int_rho=500        # [kg/m^3] interior particle density
mu_int=0.0         # Friction coefficient for interior particles

# %% Obstacle properties
obstacles=True     # Create obstacles?
obs_shape='box'     # 'cyl' or 'box' type of obstacles
obs_rad=[1.5,3.5]  # [m] upper and lower bounds of obstacle radius

box_dims=[30,10]          # [m] x,z dimensions of bounding box
obs_aspect=[0.4,1]      # [1] lower and upper limit to aspect ratio range
obs_gap=0.22             # [m] minimum gap between objects for poisson sampling

# %% Grasped object properties
grasp=False                      # Create objects to grasp?
g_fix=True                      # Grasped object is fixed?
grasp_n=1                       # Number of things to grab

#grasp_xyz=[2.0,rob_h,0.0]     # [m] xyz coordinates of thing to grasp
grasp_xyz=[1.5,rob_h,0]     # [m] xyz coordinates of thing to grasp
grasp_rad=0.65                   # [m] radius of grasped object

grasp_type='conv'                # Type of object to be grasped: 'cyl', 'conv', 'conc', 'wall'
grasp_vert=6                     # of vertices if grasp_type=='conv' or 'conc'
grasp_irr=0.75                   # Polygon grasp object irregularity if grasp_type=='conv' or 'conc'

# Wall      # X size, Y size, Z size
grasp_wall=[rob_d,    rob_h,  4.*rob_rad]  # In +x direction
#grasp_wall=[4.*rob_rad,    rob_h, rob_d ]  # In +y direction

grasp_h=rob_h                   # [m] height of object being grasped
grasp_rho=1000                  # [kg/m^3] density of object to be grasped
grasp_m = 0                     # [kg] mass of object to be grasped. Overrides grasp_rho if >0

# %% Controller/communication settings
control=True                # Use controller?
con_local=1                 # Localization method
'''
0: Use positions as measured from Chrono
1: Use Kalman filtering and onboard sensors to estimate position
2: Use triangulation based distance method
'''
control_print=False         # Print controller output to console
con_typ=1                   # Type of controller, options listed below:
'''
1: Formation control with reference distances
'''
con_rate=100                # [Hz] frequency of control inputs
com_rate=100                # [Hz] frequency of communications
gain1=100                   # proportional gain 1

con_torq=1                # [N*m] Rolling torque applied by each robot if stuck


# %% Sensing
sens_rate=500            # [Hz] frequency of polling

# IMU/Magnetometer noise parameters
                            # Gaussian  # Random Walk
                            
                        # Gyro quaternion components
noise_params = np.asarray([[1e-2,        1e-2],     
                           [1e-2,        1e-2],     
                           [1e-2,        1e-2],     
                           [1e-2,        1e-2],
                           
                        # Accelerometer x,z
                           [1e-2,        1e-2],
                           [1e-2,        1e-2],
                           
                        # Magnetometer quaternion components
                           [1e-2,        1e-2],
                           [1e-2,        1e-2],
                           [1e-2,        1e-2],
                           [1e-2,        1e-2]])


# %% Kaman Filtering
dt = 1./sens_rate

# A matrix
A=np.eye(8)
A[0,1]=dt
A[2,3]=dt

B=np.asarray([[0.5*dt**2,0.0,       0.0, 0.0, 0.0, 0.0],
              [dt,       0.0,       0.0, 0.0, 0.0, 0.0],
              [0.0,      0.5*dt**2, 0.0, 0.0, 0.0, 0.0],
              [0.0,      dt,        0.0, 0.0, 0.0, 0.0],
              [0.0,      0.0,       dt , 0.0, 0.0, 0.0],
              [0.0,      0.0,       0.0, dt , 0.0, 0.0],
              [0.0,      0.0,       0.0, 0.0, dt , 0.0],
              [0.0,      0.0,       0.0, 0.0, 0.0, dt ]])

# State vector
X=np.zeros((8,1))

# Observation Transformation
H = np.asarray([[1., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 1., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 1., 0., 0., 0.],
                [0., 0., 0., 0., 0., 1., 0., 0.],
                [0., 0., 0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 1.]])

# Observation Errors
Rsig1=5e-3
Rsig2=5e-3
Rsig3=1e-3
Rsig4=1e-3
Rsig5=1e-3
Rsig6=1e-3
R=np.diag([Rsig1,Rsig2,Rsig3,Rsig4,Rsig5,Rsig6])

# Initial Estimation Covariance Matrix
Qsig1=1e-2
Qsig2=1e-2
Qsig3=1e-2
Qsig4=1e-2
Qsig5=1e-2
Qsig6=1e-2
Qsig7=1e-2
Qsig8=1e-2
covar=[Qsig1,Qsig2,Qsig3,Qsig4,Qsig5,Qsig6,Qsig7,Qsig8]
Q=np.zeros((8,8))
P=np.diag(covar)
for ii in range(8):
    for jj in range(8):
        Q[ii,jj]=covar[ii]*covar[jj]

# %% Plotting
live_npts=50                    # number of data points shown at once on live feed
live_interval=250               # [ms] live animation update interval
live_bot=0                      # id of bot to view live data from
live_pos=[False, False]          # Plot/save live position info
live_vel=[False, False]          # Plot/save live velocity info
live_dist=[False, False]         # Plot/save live distance info
live_tens=[False, False]         # Plot/save live spring tension
live_cont=[False, False]         # Plot/save live contact info
live_control=[False, False]      # Plot/save live control inputs
live_area=[False, False]         # Plot/save live area
live_conv=[False, False]         # Plot/save live convexity
live_pres=[False, False]         # Plot/save live internal pressure
live_map =[False, False]         # Plot/save live object mapping

save_format='.png'      # File format for all saved plots
save_sens=True          # Save accelerometer and gyro measurements
save_pred=[True,False]   # Save predicted positions from kalman filtering process (subplots,overlay)
save_pred_d=True      # Save L/R distance estimates and reference
save_pred_err=True      # Save prediction errors
save_pos=False          # Save reference global position info
save_lpos=False         # Save local position info
save_lposa=False        # Save global position reconstructed from local position
save_vel=False          # Save velocity info
save_dist=False         # Save distance info
save_tens=False         # Save spring tensions
save_cont=False         # Save contact info
save_control=False      # Save control inputs
save_area=False         # Save formation area
save_conv=False         # Save formation convexity
save_pres=False         # Save internal pressure
save_lmap=True         # Save object mapping animation (local coordinates)
save_gmap=False         # Save object mapping animation (global coordinates)

# %% Colors
col_r = chrono.ChColorAsset(); col_r.SetColor(chrono.ChColor(1, 0, 0))              # 0 Red
col_g = chrono.ChColorAsset(); col_g.SetColor(chrono.ChColor(0, 1, 0))              # 1 Green
col_b = chrono.ChColorAsset(); col_b.SetColor(chrono.ChColor(0, 0, 1))              # 2 Blue
col_y = chrono.ChColorAsset(); col_y.SetColor(chrono.ChColor(1, 1, 0))              # 3 Yellow
col_p = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(0.44, .11, 52))        # 4 Purple
col_gy1=chrono.ChColorAsset(); col_gy1.SetColor(chrono.ChColor(0.25, 0.25, 0.25))   # 5 Dark Grey
col_gy2=chrono.ChColorAsset(); col_gy2.SetColor(chrono.ChColor(0.3, 0.3, 0.3))      # 6 Medium Grey
col_gy3=chrono.ChColorAsset(); col_gy3.SetColor(chrono.ChColor(.35, .35, .35))      # 7 Light Grey
col_vect = [col_r, col_g, col_b, col_y, col_p]
