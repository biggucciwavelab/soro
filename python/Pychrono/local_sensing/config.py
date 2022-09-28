""""
config.py: 
    
This file contains the configuration parameters
for the local sensing simulation

adi made a change here

Author: Qiyuan Zhou
Date (MM/DD/YY): 09/10/20
Wavelab | Illinois Institute of Technology
"""
import pychrono.core as chrono
import numpy as np


class Configuration:
    def __init__(self, sens_rate=5):

        # Naming, file paths, visualization controls
        self.data_path = 'C:/Chrono/Builds/chrono-develop/bin/data/'  # Path for simulation data assets
        self.sim_id = 'experiments/01'  # Name of simulation
        self.visual = 'exp'  # Visualization method, irr/pov/exp
        self.saveFrames = False  # Save frames from irr visualization?
        self.paused = False  # Start paused?
        self.showInfo = True  # Show sim info?
        self.track_cam = None  # None to disable, robot_id to follow robot of choice
        self.cam_h = 4  # Camera height
        self.saveFPS = 50  # Frame rate of saved visualization
        self.resx = 1280  # Video frame x resolution
        self.resy = 720  # Video frame y resolution

        # Simulation time control
        self.tstep = 0.001  # Solver time step
        self.tend = 5  # End simulation after this time
        self.tout = True  # Print timesteps?

        # Friction + compliance
        self.mu_f = 0.1  # Sliding coefficient
        self.mu_b = 0.01  # Damping coefficient
        self.C = 0.0001  # Normal compliance
        self.Ct = 0.0001  # Tangential compliance

        # Floor dimensions
        self.fl_h = 0.5  # Floor height
        self.fl_l = 40  # Floor length and width

        # %% Robot parameters
        self.rob_nb = 12  # number of robots
        self.rob_m = 0.5  # [kg] robot mass, positive value overrides rob_rho
        self.rob_d = 0.0945  # [m] robot diameter
        self.rob_h = 0.04885  # [m] robot height
        self.rob_rho = 2000  # [kg/m^3] robot density
        self.rob_offx = 0  # [m] formation center x offset
        self.rob_offz = 0  # [m] formation center z offset

        # Spacing between center of two robots
        self.rob_spacing = 3 * self.rob_d

        # calculate formation radius
        self.t = 2 * np.pi / self.rob_nb
        self.rob_rad = self.rob_spacing / (2 * np.arcsin(0.5 * self.t))

        # Spring and membrane parameters
        self.linked = True  # Robots are linked?
        self.mem_geo = 'sph'  # 'sph' or 'cyl' for spherical/cylindrical membrane particles
        self.skind = 0.03  # [m] diameter of cylinders for skin particles
        self.ratioM = 6  # ratio of membrane skin particles to big bots
        self.skinrho = 1200  # [kg/m^3] density of skin particles [kg/m^3]
        self.km = 5000  # [N/m] spring constant (skin membrane)
        self.bm = 0  # [N*s/m]　damping constant (skin membrane)

        # Interior particles
        self.interiors = True  # Create interior particles?
        self.int_ratio = 0.75  # None/Ratio of small to large particle size
        self.int_spacing = 1.01  # Spacing factor for interior particles
        self.int_d = 0.08  # Interior particle diameter
        self.int_m = 0.05  # [kg] interior particle mass, positive value overrides int_rho
        self.int_h = self.rob_h  # [m] interior particle height
        self.int_rho = 500  # [kg/m^3] interior particle density
        self.mu_int = 0.0  # Friction coefficient for interior particles

        # Obstacle properties
        self.obstacles = False  # Create obstacles?
        self.obs_shape = 'box'  # 'cyl' or 'box' type of obstacles
        self.obs_rad = [1.5 * self.rob_rad, 3.5 * self.rob_rad]  # [m] upper and lower bounds of obstacle radius

        self.box_dims = [30, 10]  # [m] x,z dimensions of bounding box
        self.obs_aspect = [0.4, 1]  # [1] lower and upper limit to aspect ratio range
        self.obs_gap = 0.22 * self.rob_rad  # [m] minimum gap between objects for poisson sampling

        # Grasped object properties
        self.grasp = True  # Create objects to grasp?
        self.g_fix = True  # Grasped object is fixed?
        self.grasp_n = 1  # Number of things to grab

        self.grasp_xyz = [2, self.rob_h, 0]  # [m] xyz coordinates of thing to grasp
        self.grasp_rad = 0.65  # [m] radius of grasped object

        self.grasp_type = 'wall'  # Type of object to be grasped: 'cyl', 'conv', 'conc', 'wall'
        self.grasp_vert = 6  # of vertices if grasp_type=='conv' or 'conc'
        self.grasp_irr = 0.75  # Polygon grasp object irregularity if grasp_type=='conv' or 'conc'

        # Wall      # X size, Y size, Z size
        self.grasp_wall = [self.rob_d, self.rob_h, 4. * self.rob_rad]  # In +x direction
        # self.grasp_wall=[4.*rob_rad,    rob_h, rob_d ]  # In +y direction

        self.grasp_h = self.rob_h  # [m] height of object being grasped
        self.grasp_rho = 1000  # [kg/m^3] density of object to be grasped
        self.grasp_m = 0  # [kg] mass of object to be grasped. Overrides grasp_rho if >0

        # Controller/communication settings
        self.control = True  # Use controller?
        self.con_local = 1  # Localization method
        '''
        0: Use positions as measured from Chrono
        1: Use Kalman filtering and onboard sensors to estimate position
        2: Use triangulation based distance method
        '''
        self.control_print = False  # Print controller output to console
        self.con_typ = 1  # Type of controller, options listed below:
        '''
        1: Formation control with reference distances
        '''
        self.con_rate = 100  # [Hz] frequency of control inputs
        self.com_rate = 100  # [Hz] frequency of communications
        self.gain1 = 100  # proportional gain 1

        self.con_torq = 1  # [N*m] Rolling torque applied by each robot if stuck

        # Sensing
        self.sens_rate = sens_rate  # [Hz] frequency of polling
        self.order = 3  # EFD max order
        self.g = 9.80665  # [m/s^2] gravitational constant
        self.T = 298.  # [K] Temperature of operation
        self.V = 3.  # [V] Nominal operation voltage

        self.ug_hz = 1e-6 * self.g / np.sqrt(self.sens_rate)
        self.mg_k = 1e-3 * self.g / self.T
        self.mg_v = 1e-3 * self.g / self.V
        self.d2r = np.pi / 180.

        # IMU/Magnetometer noise parameters
        # Taken from Bosch BMX055: https://neoplc.org/wp-content/uploads/2018/08/BMX055.pdf
        # Gaussian  # Random Walk

        # Gyro quaternion components
        self.noise_params = np.asarray([[1e-1 * self.d2r, 0.1 * self.d2r / self.V],
                                        [1e-1 * self.d2r, 0.1 * self.d2r / self.V],
                                        [1e-1 * self.d2r, 0.1 * self.d2r / self.V],
                                        [1e-1 * self.d2r, 0.1 * self.d2r / self.V],

                                        # Accelerometer x,z
                                        [150 * self.ug_hz, self.mg_k + 0.5 * self.mg_v],
                                        [150 * self.ug_hz, self.mg_k + 0.5 * self.mg_v],

                                        # Magnetometer quaternion components
                                        [3 * self.d2r, 1e-5],
                                        [3 * self.d2r, 1e-5],
                                        [3 * self.d2r, 1e-5],
                                        [3 * self.d2r, 1e-5]])

        # Kaman Filtering
        self.ratio_meas = 0  # Number of inactive bots between units
        self.dt = 1. / self.sens_rate
        dt = self.dt

        # A matrix
        self.A = np.eye(8)
        self.A[0, 1] = self.dt
        self.A[2, 3] = self.dt

        self.B = np.asarray([[0.5 * dt ** 2, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [dt, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.5 * dt ** 2, 0.0, 0.0, 0.0, 0.0],
                             [0.0, dt, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, dt, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, dt, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 0.0, dt, 0.0],
                             [0.0, 0.0, 0.0, 0.0, 0.0, dt]])

        # State vector
        self.X = np.zeros((8, 1))

        # Observation Transformation
        self.H = np.asarray([[1., 0., 0., 0., 0., 0., 0., 0.],
                             [0., 0., 1., 0., 0., 0., 0., 0.],
                             [0., 0., 0., 0., 1., 0., 0., 0.],
                             [0., 0., 0., 0., 0., 1., 0., 0.],
                             [0., 0., 0., 0., 0., 0., 1., 0.],
                             [0., 0., 0., 0., 0., 0., 0., 1.]])

        # Observation Errors
        Rsig1 = 5e-2
        Rsig2 = 5e-2
        Rsig3 = 3 * self.d2r
        Rsig4 = 3 * self.d2r
        Rsig5 = 3 * self.d2r
        Rsig6 = 3 * self.d2r
        self.R = np.diag([Rsig1, Rsig2, Rsig3, Rsig4, Rsig5, Rsig6])

        # Initial Estimation Covariance Matrix
        Qsig1 = 1e-2
        Qsig2 = 1e-2
        Qsig3 = 1e-2
        Qsig4 = 1e-2
        Qsig5 = 1e-2
        Qsig6 = 1e-2
        Qsig7 = 1e-2
        Qsig8 = 1e-2
        covar = [Qsig1, Qsig2, Qsig3, Qsig4, Qsig5, Qsig6, Qsig7, Qsig8]
        self.Q = np.zeros((8, 8))
        self.P = np.diag(covar)
        for ii in range(8):
            for jj in range(8):
                self.Q[ii, jj] = covar[ii] * covar[jj]

        # Plotting
        self.live_npts = 50  # number of data points shown at once on live feed
        self.live_interval = 250  # [ms] live animation update interval
        self.live_bot = 0  # id of bot to view live data from
        self.live_pos = [False, False]  # Plot/save live position info
        self.live_vel = [False, False]  # Plot/save live velocity info
        self.live_dist = [False, False]  # Plot/save live distance info
        self.live_tens = [False, False]  # Plot/save live spring tension
        self.live_cont = [False, False]  # Plot/save live contact info
        self.live_control = [False, False]  # Plot/save live control inputs
        self.live_area = [False, False]  # Plot/save live area
        self.live_conv = [False, False]  # Plot/save live convexity
        self.live_pres = [False, False]  # Plot/save live internal pressure
        self.live_map = [False, False]  # Plot/save live object mapping

        self.save_format = '.png'  # File format for all saved plots
        self.save_sens = True  # Save accelerometer and gyro measurements
        self.save_pred = [True, False]  # Save predicted positions from kalman filtering process (subplots,overlay)
        self.save_pred_d = False  # Save L/R distance estimates and reference
        self.save_pred_err = True  # Save prediction errors
        self.save_pos = False  # Save reference global position info
        self.save_lpos = False  # Save local position info
        self.save_lposa = False  # Save global position reconstructed from local position
        self.save_vel = False  # Save velocity info
        self.save_dist = False  # Save distance info
        self.save_tens = False  # Save spring tensions
        self.save_cont = False  # Save contact info
        self.save_control = False  # Save control inputs
        self.save_area = False  # Save formation area
        self.save_conv = False  # Save formation convexity
        self.save_pres = False  # Save internal pressure
        self.save_lmap = False  # Save object mapping animation (local coordinates)
        self.save_gmap = False  # Save object mapping animation (global coordinates)

        # Colors
        self.col_r = chrono.ChColorAsset()
        self.col_r.SetColor(chrono.ChColor(1, 0, 0))  # 0 Red
        self.col_g = chrono.ChColorAsset()
        self.col_g.SetColor(chrono.ChColor(0, 1, 0))  # 1 Green
        self.col_b = chrono.ChColorAsset()
        self.col_b.SetColor(chrono.ChColor(0, 0, 1))  # 2 Blue
        self.col_y = chrono.ChColorAsset()
        self.col_y.SetColor(chrono.ChColor(1, 1, 0))  # 3 Yellow
        self.col_p = chrono.ChColorAsset()
        self.col_p.SetColor(chrono.ChColor(0.44, .11, 52))  # 4 Purple
        self.col_gy1 = chrono.ChColorAsset()
        self.col_gy1.SetColor(chrono.ChColor(0.25, 0.25, 0.25))  # 5 Dark Grey
        self.col_gy2 = chrono.ChColorAsset()
        self.col_gy2.SetColor(chrono.ChColor(0.3, 0.3, 0.3))  # 6 Medium Grey
        self.col_gy3 = chrono.ChColorAsset()
        self.col_gy3.SetColor(chrono.ChColor(.35, .35, .35))  # 7 Light Grey
        self.col_vect = [self.col_r, self.col_g, self.col_b, self.col_y, self.col_p]
