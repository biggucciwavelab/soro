""""
environment.py: 
    
This file contains the classes and defs which
create the local sensing simulation environment

Author: Qiyuan Zhou
Date (MM/DD/YY): 09/10/20
Wavelab | Illinois Institute of Technology
"""
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
from scipy.spatial import ConvexHull
# from concurrent.futures.thread import ThreadPoolExecutor, wait
from mp_plot import elliptic_fourier
import random
import math as ma
import numpy as np
import robots as rob
import plotters as plot
import shutil
import os


class SimEnv:
    def __init__(self, config):
        # Config
        self.cf = config

        # Create result folders and copy config file
        if not os.path.exists(self.cf.sim_id):
            os.makedirs(self.cf.sim_id)
        shutil.copyfile('config.py', self.cf.sim_id + '/config.py')

        # Active and inactive IDs
        self.active = []
        self.inactive = []

        for i in range(self.cf.rob_nb):
            if i % (self.cf.ratio_meas + 1) != 0:
                self.inactive.append(i)
            if i % (self.cf.ratio_meas + 1) == 0:
                self.active.append(i)

        # Array of robot objects
        self.bots = []

        # Chrono simulation specific inits
        if config.visual != 'exp':
            # Multiprocessing cores
            self.cores = 12

            # Create system
            self.sys = self.make_sys()
            chrono.SetChronoDataPath(self.cf.data_path)
            chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
            chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0005)

            # Define our contact material
            self.material = self.make_material(self.cf.mu_f)
            self.int_mat = self.make_material(self.cf.mu_int)

            # Create floor
            self.floor = self.make_floor()
            self.sys.Add(self.floor)

            # IDs of tower robots for triangulation
            self.towers = [0, round(self.cf.rob_nb / 3), round(2 * self.cf.rob_nb / 3)]

            # Create Robots
            self.make_robots()

            # Link Robots
            if self.cf.linked:
                self.skinM = []  # empty list to which membrane skin cylinders are to be appended
                self.link_robots()

            # Shared data between robots
            self.area = -1.0  # Current area of robot
            self.conv_hull = -1.0  # Area of convex hull
            self.convexity = -1.0  # Measure of convexity
            self.pressure = -1.0  # Internal pressure
            self.coord_rot = 0

            # Create interiors
            if self.cf.interiors:
                self.gran = []  # empty list to which interior particles are to be appended
                self.make_interior()

            # Create (cylindrical) obstacles and grasping objects
            self.obstacles = []
            self.grasps = []
            self.make_objects()

            # Populate arrays for distance and contact sensors
            self.objects = []
            for item in self.obstacles:
                self.objects.append(item)
            for item in self.grasps:
                self.objects.append(item)

            self.rad = self.cf.rob_nb * [0.5 * self.cf.rob_d]

            # Initiate the contact list
            self.contact_list = np.asarray([[0], [0]])

            # Make some reference values for constantly spaced on a circle
            if False:
                ref_vals = [] * len(self.bots)
                for angle in np.arange(2 * np.pi / self.cf.rob_nb, 2 * np.pi, 2 * np.pi / self.cf.rob_nb):
                    ref_vals.append(3 * self.cf.rob_rad * ma.sin(angle / 2))

                self.ref_vals = np.zeros((len(ref_vals), len(self.bots)))
                self.ref_vals[:, 0] = np.asarray(ref_vals)

                for i in range(1, len(self.bots)):
                    a = self.ref_vals[:, i - 1]
                    a = np.roll(a, 1)
                    self.ref_vals[:, i] = a

            # Init the contact reporting class
            self.my_rep = MyReportContactCallback()

        # Initialize things for running algorithm with simulation data    
        elif config.visual == 'exp':

            # Set the experiment number
            self.exp = '01'

            # Read in reference data            
            self.sens_dat = np.genfromtxt('_koki_data/' + self.exp + '/data_sensor.csv', delimiter=',')[1:, :]
            self.sens_dat[:, 0] -= self.sens_dat[0, 0]
            self.sens_dat[:, 0] = self.sens_dat[:, 0] / (1e9)
            self.times = self.sens_dat[:, 0]

            # Convert cm to m
            self.sens_dat = self.sens_dat[:, 9:-8] / 100.

            # Convert angles to degrees
            for i in range(self.cf.rob_nb):
                self.sens_dat[:, i * 8 + 2] = 10. * np.pi * self.sens_dat[:, i * 8 + 2] / 9.

                # Rotate accelerations around x axis so z component is pointing straight down
                xs = self.sens_dat[:, i * 8 + 3]
                ys = self.sens_dat[:, i * 8 + 4]
                zs = self.sens_dat[:, i * 8 + 5]
                for j in range(len(zs)):
                    # Vector used to normalize gravity in xz plane
                    vec1 = np.asarray([xs[j], zs[j]])
                    vec1 = 9.81 * vec1 / np.linalg.norm(vec1)

                    vec2 = np.asarray([ys[j], zs[j]])
                    vec2 = 9.81 * vec2 / np.linalg.norm(vec2)

                    # Rotation angle about y axis
                    t_y = np.arccos(vec1[1] / 9.81)

                    # Rotation angle about x axis
                    t_x = np.arccos(vec2[1] / 9.81)

                    alpha = 0.0  # Rotation about z
                    beta = 0.0  # -t_y    # Rotation about y
                    gamma = -t_x  # Rotation about x
                    ca = np.cos(alpha)
                    sa = np.sin(alpha)
                    cb = np.cos(beta)
                    sb = np.sin(beta)
                    cg = np.cos(gamma)
                    sg = np.sin(gamma)

                    accs = np.asarray([xs[j], ys[j], zs[j]])
                    rot = np.asarray([[ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg],
                                      [sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg],
                                      [-sb, cb * sg, cb * cg]])
                    accs = np.matmul(rot, accs)
                    self.sens_dat[j, i * 8 + 3] = accs[0]
                    self.sens_dat[j, i * 8 + 4] = accs[1]
                    self.sens_dat[j, i * 8 + 5] = accs[2]

            '''
            Columns of sens_dat:
                x_n, y_n, theta_n, accx_n, accy_n, accz_n, posx_n, posxy_n
            '''

            # Init 'robot' objects
            self.bots = []
            self.make_exp_bots()

            # Set up data collection?

            a = 1

        # Init efd class: Common to both
        contour = []
        for robot in self.bots:
            if robot.active:
                contour.append(robot.local_pos)
        contour = np.asarray(contour)
        npoints = self.cf.rob_nb
        order = self.cf.order
        self.elliptic = elliptic_fourier(contour, order, npoints)
        self.elliptic.gen_elliptic()

    def make_sys(self):  # Create Chrono system
        sys = chrono.ChSystemNSC()
        sys.SetSolverType(chrono.ChSolver.Type_PSSOR)
        sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
        return sys

    def make_material(self, mu):  # Create Chrono surface material
        material = chrono.ChMaterialSurfaceNSC()
        material.SetFriction(mu)
        material.SetDampingF(self.cf.mu_b)
        material.SetCompliance(self.cf.C)
        material.SetComplianceT(self.cf.Ct)
        return material

    def make_floor(self):  # Create simulation floor
        floor = chrono.ChBody()
        floor.SetName('floor')
        floor.SetBodyFixed(True)
        floor.SetPos(chrono.ChVectorD(0, -self.cf.fl_h, 0))
        floor.SetMaterialSurface(self.material)
        floor.GetCollisionModel().ClearModel()
        floor.GetCollisionModel().AddBox(self.cf.fl_l, self.cf.fl_h, self.cf.fl_l)  # hemi sizes
        floor.GetCollisionModel().BuildModel()
        floor.SetCollide(True)
        floor.SetId(5000)
        body_floor_shape = chrono.ChBoxShape()
        body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.cf.fl_l, self.cf.fl_h, self.cf.fl_l)
        floor.GetAssets().push_back(body_floor_shape)
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
        floor.AddAsset(col_g)
        return floor

    def make_robots(self):  # Create the robots of type robot class
        for i in range(self.cf.rob_nb):
            # Calculate inital position and rotation
            theta = float(i * 2.0 * np.pi) / float(self.cf.rob_nb)
            if theta < 0: theta = 2 * np.pi + theta
            x = self.cf.rob_rad * np.cos(theta) + self.cf.rob_offx
            y = 0.5 * self.cf.rob_h
            z = self.cf.rob_rad * np.sin(theta) + self.cf.rob_offz
            xyz = [x, y, z, theta]

            # Initialize robot and append to object array
            robot = rob.robot(self.sys, xyz, self.material, True, True, self.cf)
            robot.ID = i
            robot.body.SetId(i)
            if i in self.active: robot.active = True
            if i in self.inactive: robot.active = False
            # robot.active=False

            rot = robot.body.GetRot()
            initial = np.asarray([[x], [0.0], [z], [0.0], [rot.e0], [rot.e1], [rot.e2], [rot.e3]])
            if i == self.towers[0]:
                robot.body.AddAsset(self.cf.col_r)
            if i == self.towers[1]:
                robot.body.AddAsset(self.cf.col_g)
            if i == self.towers[2]:
                robot.body.AddAsset(self.cf.col_b)
            robot.init_sens_kalman(self.cf.noise_params, initial)

            # Planar mate between robot and floor
            pt = chrono.ChLinkMatePlane()
            pt.Initialize(robot.body, self.floor, True,
                          chrono.ChVectorD(0, -self.cf.rob_h / 2, 0),
                          chrono.ChVectorD(0, self.cf.fl_h, 0),
                          chrono.ChVectorD(0, -1, 0),
                          chrono.ChVectorD(0, 1, 0))
            self.bots.append(robot)
            self.sys.AddLink(pt)

    def make_exp_bots(self):  # Create the robot objects for experimental data

        for i in range(self.cf.rob_nb):
            # Calculate inital position and rotation
            theta = self.sens_dat[0, 9 * i + 3]
            if theta < 0: theta = 2 * np.pi + theta
            x = self.sens_dat[0, 9 * i + 1]
            y = self.sens_dat[0, 9 * i + 2]
            z = 0.0
            xyz = [x, y, z, theta]

            # Initialize robot and append to object array
            robot = rob.robot(xyz=xyz, actu=True, sens=True, config=self.cf)
            robot.ID = i
            if i in self.active: robot.active = True
            if i in self.inactive: robot.active = False

            # Init Kalman filtering stuff for this bot
            initial = np.asarray([[x], [0.0], [y], [0.0], [0.0], [0.0], [0.0], [0.0]])
            robot.init_sens_kalman(self.cf.noise_params, initial)

            # Add bot to bots array
            self.bots.append(robot)

    def link_robots(self):  # Link the robots and initialize tension sensors
        b_ang = 2 * np.pi / self.cf.rob_nb  # angle between centers of bots
        o_ang = np.arctan(self.cf.rob_d / (2 * self.cf.rob_rad))  # angle offset for radius of bot
        p_ang = np.arctan(self.cf.skind / (2 * self.cf.rob_rad))  # angle offset for radius of skin particle

        for i in range(self.cf.rob_nb):

            # Between this bot and last bot
            if 1 <= i < self.cf.rob_nb:
                for j in range(1, self.cf.ratioM + 1, 1):
                    # Initial position of each particle
                    theta = i * b_ang + j * (b_ang - o_ang - 2 * p_ang) / (self.cf.ratioM) + p_ang
                    x = self.cf.rob_rad * np.cos(theta) + self.cf.rob_offx
                    y = .52 * self.cf.rob_h
                    z = self.cf.rob_rad * np.sin(theta) + self.cf.rob_offz

                    # Create particles   
                    if self.cf.mem_geo == 'sph' or self.cf.mem_geo == 'Sph':
                        skinm = chrono.ChBodyEasySphere(self.cf.skind / 2, self.cf.skinrho, True, True)
                    if self.cf.mem_geo == 'cyl' or self.cf.mem_geo == 'Cyl':
                        skinm = chrono.ChBodyEasyCylinder(self.cf.skind / 2, .5 * self.cf.rob_h, self.cf.skinrho, True,
                                                          True)
                    skinm.SetPos(chrono.ChVectorD(x, y, z))
                    skinm.SetMaterialSurface(self.material)
                    skinm.SetNoGyroTorque(True)
                    # rotate them
                    rotation1 = chrono.ChQuaternionD()
                    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0))
                    skinm.SetRot(rotation1)

                    # Attach springs and add to robot list of controllable springs    
                    if j > 1:
                        ground = chrono.ChLinkTSDA()
                        if self.cf.mem_geo == 'sph' or self.cf.mem_geo == 'Sph':
                            ground.Initialize(self.skinM[-1], skinm, True, chrono.ChVectorD(0, 0, self.cf.skind / 2),
                                              chrono.ChVectorD(0, 0, -self.cf.skind / 2), False, 0.0)
                            ground.SetSpringCoefficient(self.cf.km)
                            ground.SetDampingCoefficient(self.cf.bm)
                            ground.AddAsset(self.cf.col_p)
                            ground.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
                            self.sys.AddLink(ground)
                            self.bots[i].springs.append(ground)

                        if self.cf.mem_geo == 'cyl' or self.cf.mem_geo == 'Cyl':
                            p1 = 0
                            p2 = self.cf.skind / 2
                            p3 = 0
                            p4 = -self.cf.skind / 2
                            h = self.cf.rob_h / 4

                            ground.Initialize(self.skinM[-1], skinm, True, chrono.ChVectorD(p1, h, p2),
                                              chrono.ChVectorD(p3, h, p4), False, 0.0)
                            ground.SetSpringCoefficient(self.cf.km)
                            ground.SetDampingCoefficient(self.cf.bm)
                            ground.AddAsset(self.cf.col_p)
                            ground.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
                            self.sys.AddLink(ground)
                            self.bots[i].springs.append(ground)

                            ground1 = chrono.ChLinkTSDA()
                            ground1.Initialize(self.skinM[-1], skinm, True, chrono.ChVectorD(p1, -h, p2),
                                               chrono.ChVectorD(p3, -h, p4), False, 0.0)
                            ground1.SetSpringCoefficient(self.cf.km)
                            ground1.SetDampingCoefficient(self.cf.bm)
                            ground1.AddAsset(self.cf.col_p)
                            ground1.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
                            self.sys.AddLink(ground1)
                            self.bots[i].springs.append(ground1)

                            # Link to cylinder (left)
                    if j == 1:
                        skinm.AddAsset(self.cf.col_p)
                        glue = chrono.ChLinkMateFix()
                        glue.Initialize(skinm, self.bots[i].body)
                        self.sys.AddLink(glue)
                        skinm.SetCollide(False)
                        self.bots[i].linkL = glue
                        self.bots[i].mem[0][0] = skinm

                        # if i<self.cf.rob_nb-1: self.bots[i+1].mem[0][1]=skinm
                        # if i<self.cf.rob_nb-1: self.bots[i+1].mem[1][1]=skinm
                        # if i==self.cf.rob_nb-2: self.bots[-1].mem[1][1]=skinm

                        # Link last particle with this bot (right)
                        if i >= 2:
                            glue = chrono.ChLinkMateFix()
                            glue.Initialize(self.skinM[-1], self.bots[i].body)
                            self.sys.AddLink(glue)
                            skinm.SetCollide(False)
                            self.bots[i].linkR = glue
                            self.bots[i].mem[1][0] = self.skinM[-1]
                            # self.bots[i-1].mem[0][1]=self.skinM[-1]

                    if j == 2:
                        self.bots[i].mem[0][1] = skinm

                    if j == self.cf.ratioM - 1 and i < self.cf.rob_nb - 1:
                        self.bots[i + 1].mem[1][1] = skinm

                    if j == self.cf.ratioM:
                        skinm.AddAsset(self.cf.col_p)

                    skinm.SetId(10000)
                    self.sys.Add(skinm)
                    self.skinM.append(skinm)

            # Between this bot and first bot
            if i == self.cf.rob_nb - 1:
                self.bots[-1].mem[1][1] = self.skinM[-self.cf.ratioM - 2]
                self.bots[0].mem[1][1] = self.skinM[-2]

                for j in range(1, self.cf.ratioM + 1, 1):
                    # Initial postion of each particle
                    theta = (i + 1) * b_ang + j * (b_ang - o_ang - 2 * p_ang) / self.cf.ratioM + p_ang
                    x = self.cf.rob_rad * np.cos(theta) + self.cf.rob_offx
                    y = .52 * self.cf.rob_h
                    z = self.cf.rob_rad * np.sin(theta) + self.cf.rob_offz
                    # Create particles
                    if self.cf.mem_geo == 'sph' or self.cf.mem_geo == 'Sph':
                        skinm = chrono.ChBodyEasySphere(self.cf.skind / 2, self.cf.skinrho, True, True)
                    if self.cf.mem_geo == 'cyl' or self.cf.mem_geo == 'Cyl':
                        skinm = chrono.ChBodyEasyCylinder(self.cf.skind / 2, .5 * self.cf.rob_h, self.cf.skinrho, True,
                                                          True)
                    skinm.SetPos(chrono.ChVectorD(x, y, z))
                    skinm.SetMaterialSurface(self.material)
                    skinm.SetNoGyroTorque(True)
                    # rotate them
                    rotation1 = chrono.ChQuaternionD()
                    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));
                    skinm.SetRot(rotation1)

                    if j == 0:
                        self.bots[-1].mem[1][1] = self.skinM[-1]
                        self.bots[0].mem[1][1] = self.skinM[-self.cf.ratioM - 2]
                        self.bots[1].mem[1][1] = skinm

                    # Attach springs and add to list of controllable springs    
                    if j > 1:
                        ground = chrono.ChLinkTSDA()
                        if self.cf.mem_geo == 'sph' or self.cf.mem_geo == 'Sph':
                            ground.Initialize(self.skinM[-1], skinm, True, chrono.ChVectorD(0, 0, self.cf.skind / 2),
                                              chrono.ChVectorD(0, 0, -self.cf.skind / 2), False, 0.0)
                            ground.SetSpringCoefficient(self.cf.km)
                            ground.SetDampingCoefficient(self.cf.bm)
                            ground.AddAsset(self.cf.col_y)
                            ground.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
                            self.sys.AddLink(ground)
                            self.bots[0].springs.append(ground)

                        if self.cf.mem_geo == 'cyl' or self.cf.mem_geo == 'Cyl':
                            p1 = 0
                            p2 = self.cf.skind / 2
                            p3 = 0
                            p4 = -self.cf.skind / 2
                            h = self.cf.rob_h / 4

                            ground.Initialize(self.skinM[-1], skinm, True, chrono.ChVectorD(p1, h, p2),
                                              chrono.ChVectorD(p3, h, p4), False, 0.0)
                            ground.SetSpringCoefficient(self.cf.km)
                            ground.SetDampingCoefficient(self.cf.bm)
                            ground.AddAsset(self.cf.col_y)
                            ground.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
                            self.sys.AddLink(ground)
                            self.bots[0].springs.append(ground)

                            ground1 = chrono.ChLinkTSDA()
                            ground1.Initialize(self.skinM[-1], skinm, True, chrono.ChVectorD(p1, -h, p2),
                                               chrono.ChVectorD(p3, -h, p4), False, 0.0)
                            ground1.SetSpringCoefficient(self.cf.km)
                            ground1.SetDampingCoefficient(self.cf.bm)
                            ground1.AddAsset(self.cf.col_y)
                            ground1.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
                            self.sys.AddLink(ground1)
                            self.bots[0].springs.append(ground1)

                    # Link to cylinder
                    if j == 1:
                        skinm.AddAsset(self.cf.col_p)
                        glue = chrono.ChLinkMateFix()
                        glue.Initialize(skinm, self.bots[0].body)
                        self.sys.AddLink(glue)
                        self.bots[0].linkL = glue
                        self.bots[0].mem[0][0] = skinm
                        # self.bots[1].mem[1][1]=skinm

                        self.bots[0].mem[0][1] = skinm

                        glue = chrono.ChLinkMateFix()
                        glue.Initialize(self.skinM[-1], self.bots[0].body)
                        self.sys.AddLink(glue)
                        skinm.SetCollide(False)
                        self.bots[0].linkR = glue
                        self.bots[0].mem[1][0] = self.skinM[-1]
                        # self.bots[0].mem[1][1]=self.skinM[-self.cf.ratioM]
                        # self.bots[-1].mem[0][1]=self.skinM[-1]

                    if j == 2:
                        self.bots[0].mem[0][1] = skinm

                    if j == self.cf.ratioM - 1:
                        self.bots[1].mem[1][1] = skinm

                    if j == self.cf.ratioM:
                        skinm.AddAsset(self.cf.col_p)
                        glue = chrono.ChLinkMateFix()
                        glue.Initialize(skinm, self.bots[1].body)
                        self.sys.AddLink(glue)
                        skinm.SetCollide(False)
                        self.bots[1].linkR = glue

                        self.bots[1].mem[1][0] = skinm
                        # self.bots[0].mem[0][1]=skinm

                    skinm.SetId(10000)
                    self.sys.Add(skinm)
                    self.skinM.append(skinm)

    def make_interior(self):  # Create the interior particles
        rad = self.cf.rob_rad - self.cf.int_spacing * (0.5 * self.cf.rob_d + 0.5 * self.cf.int_d)
        nn = 3000
        while rad > self.cf.int_d:
            # spacing in theta for each ring
            theta = 2 * np.sin(0.5 * self.cf.int_d / rad)
            color = [self.cf.col_gy1, self.cf.col_gy2, self.cf.col_gy3]
            count = 0
            for t in np.arange(0, 2 * np.pi - 0.75 * theta, theta):
                x = rad * np.cos(t) + self.cf.rob_offx
                y = 0.5 * self.cf.int_h
                z = rad * np.sin(t) + self.cf.rob_offz

                # Create the ChBody
                if bool(random.getrandbits(1)) and self.cf.int_ratio != None:
                    intd = self.cf.int_ratio * self.cf.int_d
                else:
                    intd = self.cf.int_d
                interior = chrono.ChBodyEasyCylinder(intd / 2, self.cf.int_h, self.cf.int_rho, True, True)
                if self.cf.int_m > 0:
                    interior.SetMass(self.cf.int_m)
                interior.SetPos(chrono.ChVectorD(x, y, z))
                interior.SetMaterialSurface(self.int_mat)
                interior.SetId(nn)
                # Add visual assets
                interior.AddAsset(color[np.mod(count, 3)])

                # Planar mate between robot and floor
                pt = chrono.ChLinkMatePlane()
                pt.Initialize(interior, self.floor, True,
                              chrono.ChVectorD(0, -self.cf.int_h / 2, 0),
                              chrono.ChVectorD(0, self.cf.fl_h, 0),
                              chrono.ChVectorD(0, -1, 0),
                              chrono.ChVectorD(0, 1, 0))

                # Add to system and object list
                self.sys.Add(interior)
                self.sys.AddLink(pt)
                self.gran.append(interior)
                count = count + 1
            rad = rad - self.cf.int_spacing * self.cf.int_d

    def make_objects(self):  # Create obstacles and things to be grabbed
        if self.cf.obstacles:
            samples = Poisson_Sampling(self.cf.obs_gap + self.cf.obs_rad[1], self.cf.box_dims[0], self.cf.box_dims[1])
            coordinates = np.asarray(samples.get_samples()) + np.asarray(
                [2 * self.cf.rob_rad, -0.5 * self.cf.box_dims[1]])

            # create obstacle field
            ii = 0
            for coord in coordinates:
                xyz = chrono.ChVectorD(coord[0], 0.5 * self.cf.rob_h, coord[1])
                obstacle = self.make_obstacle(xyz, ii + 1001)
                self.sys.Add(obstacle)
                obs_class = body_bag(obstacle)
                self.obstacles.append(obs_class)
                ii += 1

        if self.cf.grasp:
            grasp = grasp_class(self.material, self.cf)
            grasp.create_grasp()
            self.sys.Add(grasp.body)
            self.grasps.append(grasp)

    def make_obstacle(self, xyz, i):  # Create the actual obstacle and return the ChBody
        col_ind = 4 * round(np.random.rand())
        if self.cf.obs_shape == 'cyl':
            radius = np.random.default_rng().uniform(self.cf.obs_rad[0], self.cf.obs_rad[1], 1)[0]
            body = chrono.ChBodyEasyCylinder(radius, self.cf.rob_h, 1000, True, True)
        if self.cf.obs_shape == 'box':
            xsize = np.random.default_rng().uniform(self.cf.obs_rad[0], self.cf.obs_rad[1], 1)[0]
            ysize = xsize * np.random.default_rng().uniform(self.cf.obs_aspect[0], self.cf.obs_aspect[1], 1)[0]
            theta = np.random.random() * np.pi
            rotation = chrono.ChQuaternionD()
            rotation.Q_from_AngAxis(theta, chrono.ChVectorD(0, 1, 0))
            body = (chrono.ChBodyEasyBox(xsize, self.cf.rob_h, ysize, 1000, True, True))
            body.SetRot(rotation)

        body.SetPos(xyz)
        body.SetMaterialSurface(self.material)
        body.SetBodyFixed(True)
        body.SetUseSleeping(True)
        body.AddAsset(self.cf.col_vect[col_ind])
        body.SetId(i)
        return body

    def make_grasp(self, i):  # Create the actual thing to be grasped and return the ChBody
        xyz = chrono.ChVectorD(self.cf.grasp_xyz[0], self.cf.grasp_xyz[1], self.cf.grasp_xyz[2])
        body = chrono.ChBodyEasyCylinder(self.cf.grasp_rad, self.cf.grasp_h, 1000, True, True)
        body.SetPos(xyz)
        body.SetMaterialSurface(self.material)
        body.SetBodyFixed(True)
        body.AddAsset(self.cf.col_vect[4])
        body.SetId(i)
        return body

    def sens_tens(self):  # Update tension sensors on each robot
        for robot in self.bots:
            robot.get_tension()

    def sens_dist(self):  # Update distance and contact sensors on each robot
        for robot in self.bots:
            robot.get_dist(self.bots + self.objects, self.rad)

    def triangulate(self):  # Update local positions, relative positions, surface normals of each robot

        # IDs of 'tower' robots, this can be switched to more advance logic later
        tower_IDs = self.towers

        # First tower is always origin
        self.bots[tower_IDs[0]].local_pos = np.zeros(2)

        # Second tower is always +x axis
        self.bots[tower_IDs[1]].local_pos = np.array([self.bots[tower_IDs[1]].distance[tower_IDs[0]], 0])

        # Law of cosines to find third tower coordinates
        d1 = self.bots[tower_IDs[1]].distance[tower_IDs[0]]
        d2 = self.bots[tower_IDs[2]].distance[tower_IDs[1]]
        d3 = self.bots[tower_IDs[2]].distance[tower_IDs[0]]
        theta = np.arccos((d2 ** 2 - d1 ** 2 - d3 ** 2) / (-2 * d1 * d3))
        self.bots[tower_IDs[2]].local_pos = np.array([d3 * np.cos(theta), d3 * np.sin(theta)])

        # Array to store local coordinates to calculate differences later
        local_coords = [None] * self.cf.rob_nb
        for i in range(3):
            local_coords[tower_IDs[i]] = self.bots[tower_IDs[i]].local_pos

        # Perform the triangulation for each of the other bots
        # Modified from https://www.101computing.net/cell-phone-trilateration-algorithm/
        x1 = self.bots[tower_IDs[0]].local_pos[0]
        y1 = self.bots[tower_IDs[0]].local_pos[1]
        x2 = self.bots[tower_IDs[1]].local_pos[0]
        y2 = self.bots[tower_IDs[1]].local_pos[1]
        x3 = self.bots[tower_IDs[2]].local_pos[0]
        y3 = self.bots[tower_IDs[2]].local_pos[1]
        for i in [x for x in range(len(self.bots)) if x not in tower_IDs]:
            r1 = self.bots[i].distance[tower_IDs[0]]
            r2 = self.bots[i].distance[tower_IDs[1]]
            r3 = self.bots[i].distance[tower_IDs[2]]
            A = 2 * x2 - 2 * x1
            B = 2 * y2 - 2 * y1
            C = r1 ** 2 - r2 ** 2 - x1 ** 2 + x2 ** 2 - y1 ** 2 + y2 ** 2
            D = 2 * x3 - 2 * x2
            E = 2 * y3 - 2 * y2
            F = r2 ** 2 - r3 ** 2 - x2 ** 2 + x3 ** 2 - y2 ** 2 + y3 ** 2
            x = (C * E - F * B) / (E * A - B * D)
            y = (C * D - A * F) / (B * D - A * E)
            self.bots[i].local_pos = np.copy(np.array([x, y]))
            local_coords[i] = np.array(np.array([x, y]))

        # Get the global x, y directions and the CCW rotation of local coordinate frame
        # (cheating I know, but we need this to properly set force components in Chrono)
        xdir = self.bots[tower_IDs[1]].body.GetPos() - self.bots[tower_IDs[0]].body.GetPos()
        ydir = np.cross([xdir.x, xdir.y, xdir.z], [0, 1, 0])
        coord_rot = self.angle([1, 0], [xdir.x, xdir.z] / np.linalg.norm([xdir.x, xdir.z]))
        self.coord_rot = coord_rot

        # Update relative coordinate differences and forcex forcey directions
        for i in range(self.cf.rob_nb):
            coord_array = np.asarray(local_coords[:i] + local_coords[i + 1:])
            self.bots[i].coord_rot = coord_rot
            self.bots[i].coord_diff = self.bots[i].local_pos - coord_array
            self.bots[i].forcex.SetDir(xdir)
            self.bots[i].forcey.SetDir(chrono.ChVectorD(ydir[0], ydir[1], ydir[2]))
            self.bots[i].calc_normal(local_coords)

    def exp_distance(self):  # Calculate relative distances from experiment data
        a = 1

    def coordinates(self):  # Use global coordinates (mostly for debugging)
        global_coords = [None] * self.cf.rob_nb
        for i in range(self.cf.rob_nb):
            self.bots[i].local_pos = np.asarray([self.bots[i].body.GetPos().x, self.bots[i].body.GetPos().z])
            self.bots[i].coord_rot = 0
            self.bots[i].forcex.SetDir(chrono.ChVectorD(1, 0, 0))
            self.bots[i].forcey.SetDir(chrono.ChVectorD(0, 0, 1))
            global_coords[i] = self.bots[i].local_pos

        for i in range(self.cf.rob_nb):
            coord_array = np.asarray(global_coords[:i] + global_coords[i + 1:])
            self.bots[i].coord_diff = self.bots[i].local_pos - coord_array
            self.bots[i].calc_normal(coord_array)

    def kalman(self):  # Perform the Kalman filtering logic for simulation
        coords = [None] * self.cf.rob_nb

        # Get locations of active bots
        contour = []
        for robot in self.bots:
            if robot.active:
                contour.append(robot.local_pos)
        contour = np.asarray(contour)
        npoints = self.cf.rob_nb
        order = self.cf.order

        # Init the EFD class
        self.elliptic = elliptic_fourier(contour, order, npoints)
        self.elliptic.gen_elliptic()

        for i in range(self.cf.rob_nb):

            # Update angles for each spring
            self.bots[i].get_angle()

            # ChBody objects for this bot
            kal = self.bots[i].kalman
            bot = self.bots[i].body
            botR = self.bots[i - 1]
            if i < self.cf.rob_nb - 1:
                botL = self.bots[i + 1]
            else:
                botL = self.bots[0]

            ### Measurement

            # Update x, z, theta
            self.bots[i].get_loc(botL, botR)

            # Skip if passive
            if not self.bots[i].active:
                self.bots[i].local_pos = self.bots[i].pos_est
                self.bots[i].kalman.X[0][0] = self.bots[i].pos_est[0]
                self.bots[i].kalman.X[2][0] = self.bots[i].pos_est[1]
                self.bots[i].local_vel = np.asarray([0., 0.])
                coords[i] = self.bots[i].pos_est

                # interpolate locations of inactive bots
                # loc = self.elliptic.interpolate(self.bots[i].local_pos)

                # # Store data
                # self.bots[i].local_pos = loc
                # self.bots[i].kalman.X[0][0]=loc[0]
                # self.bots[i].kalman.X[2][0]=loc[1]
                # self.bots[i].local_vel=np.asarray([0.,0.])
                # coords[i]=self.bots[i].pos_est
                continue

            ### Filtering
            rot = self.bots[i].theta
            rot_dt = bot.GetRot_dt()
            xacc = bot.GetPos_dtdt().x
            zacc = bot.GetPos_dtdt().z

            state = [rot_dt, xacc, zacc, rot]
            out = self.bots[i].IMU.get_measurement(state)
            U = np.asarray([[out[4]], [out[5]], [out[0]], [out[1]], [out[2]], [out[3]]])

            ### Make the estimate
            kal.predict(U)

            # Make the measurement
            xx = np.zeros((8, 1))
            xx[0, 0] = self.bots[i].pos_est[0]
            xx[2, 0] = self.bots[i].pos_est[1]
            xx[4, 0] = out[6]
            xx[5, 0] = out[7]
            xx[6, 0] = out[8]
            xx[7, 0] = out[9]
            Y = np.matmul(self.cf.H, xx)

            ### Update
            kal.kf_update(Y)
            self.bots[i].local_pos = np.asarray([kal.X[0][0], kal.X[2][0]])
            self.bots[i].local_vel = np.asarray([kal.X[1][0], kal.X[3][0]])
            coords[i] = np.asarray([kal.X[0][0], kal.X[2][0]])

        for i in range(self.cf.rob_nb):
            coord_array = np.asarray(coords[:i] + coords[i + 1:])
            self.bots[i].coord_diff = self.bots[i].local_pos - coord_array
            self.bots[i].calc_normal(coord_array)

    def kalman_exp(self, step):  # Perform the Kalman filtering logic for experimental data
        coords = [None] * self.cf.rob_nb

        # Get locations of active bots
        contour = []
        for robot in self.bots:
            if robot.active:
                contour.append(robot.local_pos)
        contour = np.asarray(contour)
        npoints = self.cf.rob_nb
        order = self.cf.order

        # Init the EFD class
        self.elliptic = elliptic_fourier(contour, order, npoints)
        self.elliptic.gen_elliptic()

        # Load positions of other bots
        xys = np.zeros((2, self.cf.rob_nb))
        for i in range(self.cf_rob_nb):
            xys[0, i] = self.bots[i].local_pos[0]
            xys[1, i] = self.bots[i].local_pos[1]

        for i in range(self.cf.rob_nb):

            # Update distance to neighbor robot from AprilTag data
            self.bots[i].get_dist_exp(xys)

            # Kalman filtering object for this bot
            kal = self.bots[i].kalman

            ### Measurement

            # Skip if passive
            if not self.bots[i].active:
                self.bots[i].local_pos = self.bots[i].pos_est
                self.bots[i].kalman.X[0][0] = self.bots[i].pos_est[0]
                self.bots[i].kalman.X[2][0] = self.bots[i].pos_est[1]
                self.bots[i].local_vel = np.asarray([0., 0.])
                coords[i] = self.bots[i].pos_est
                continue

            ### Filtering
            rot = self.bots[i].theta
            rot_dt = 0.0
            xacc = self.sens_dat[step, i * 8 + 3]
            yacc = self.sens_dat[step, i * 8 + 3]

            state = [rot_dt, xacc, yacc, rot]
            out = self.bots[i].IMU.get_measurement(state)
            U = np.asarray([[out[4]], [out[5]], [out[0]], [out[1]], [out[2]], [out[3]]])

            ### Make the estimate
            kal.predict(U)

            # Make the measurement
            xx = np.zeros((8, 1))
            xx[0, 0] = self.bots[i].pos_est[0]
            xx[2, 0] = self.bots[i].pos_est[1]
            xx[4, 0] = out[6]
            xx[5, 0] = out[7]
            xx[6, 0] = out[8]
            xx[7, 0] = out[9]
            Y = np.matmul(self.cf.H, xx)

            ### Update
            kal.kf_update(Y)
            self.bots[i].local_pos = np.asarray([kal.X[0][0], kal.X[2][0]])
            self.bots[i].local_vel = np.asarray([kal.X[1][0], kal.X[3][0]])
            coords[i] = np.asarray([kal.X[0][0], kal.X[2][0]])

        for i in range(self.cf.rob_nb):
            coord_array = np.asarray(coords[:i] + coords[i + 1:])
            self.bots[i].coord_diff = self.bots[i].local_pos - coord_array
            self.bots[i].calc_normal(coord_array)

    def update_angles(self):
        for robot in self.bots:
            robot.theta = robot.body.GetRot()

    def angle(self, x, y):  # Calculate the angle of rotation between two vectors in 2D
        dot = x[0] * y[0] + x[1] * y[1]  # dot product
        det = x[0] * y[1] - y[0] * x[1]  # determinant
        return ma.atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)

    def calc_areas(self):  # Compute actual/convex hull areas as well as convexity
        # Modified shoelace formula from: https://www.101computing.net/the-shoelace-algorithm/
        sum1 = 0;
        sum2 = 0
        for i in range(0, len(self.bots) - 1):
            sum1 += self.bots[i].local_pos[0] * self.bots[i + 1].local_pos[1]
            sum2 += self.bots[i].local_pos[1] * self.bots[i + 1].local_pos[0]
        sum1 += self.bots[-1].local_pos[0] * self.bots[0].local_pos[1]  # Add xn.y1 
        sum2 += self.bots[0].local_pos[0] * self.bots[-1].local_pos[1]  # Add x1.yn
        self.area = abs(sum1 - sum2) / 2

        # Convex hull area
        points = np.zeros(2)
        for i in range(1, len(self.bots)):
            points = np.vstack((points, self.bots[i].local_pos))
        hull = ConvexHull(points)
        self.conv_hull = hull.volume

        # Convexity
        self.convexity = self.area / self.conv_hull

    def calc_pressure(self):  # Calculate approximate internal pressure
        tensions = [];
        lengths = []
        n = self.cf.rob_nb
        for robot in self.bots:
            tensions.append(np.mean(robot.tension))
            lengths.append(robot.distance[robot.ID - 1])
        length = np.mean(lengths)
        tension = np.mean(tensions)
        theta = 0.5 * np.pi * (n - 2) / (2 * n)  # [rad] half the interior angle of regular polygon
        diam = length / (np.sin(np.pi / n))  # [m]   diameter of circumcircle from calculated length
        area = np.pi * self.cf.rob_h * diam  # [m]   cylinder side area
        self.pressure = (n * tension * np.cos(theta)) / (2 * area)

    def rel_effort(self):  # Update each robot's relative effort
        efforts = []
        for robot in self.bots:
            efforts.append(np.mean([robot.forcex.GetMforce(), robot.forcey.GetMforce()]))
        effort = np.mean(efforts)
        for i in range(len(self.bots)):
            self.bots[i].rel_effort = efforts[i] / effort

    def store_contacts(self):  # Update contact masking array with grasped object (and possibly other contact info)
        # Get the contact information from Chrono's built in contact detection routines
        self.my_rep.ResetList()
        self.sys.GetContactContainer().ReportAllContacts(self.my_rep)
        self.contact_list = np.asarray(self.my_rep.GetList()[6])

        # Remove contacts with interior particles (3000), floor (5000), 
        # linking particles (10000), between two bots (<1000) and duplicate entries
        self.contact_list = self.contact_list[np.all(self.contact_list != 3000, axis=1), :]
        self.contact_list = self.contact_list[np.all(self.contact_list != 5000, axis=1), :]
        self.contact_list = self.contact_list[np.all(self.contact_list != 10000, axis=1), :]
        self.contact_list = np.sort(self.contact_list, axis=1)
        self.contact_list = self.unique_rows(self.contact_list)
        index = 0
        if len(self.contact_list > 0):
            if len(np.nonzero(self.contact_list[:, 1] > self.cf.rob_nb)[0]) > 0:
                index = np.nonzero(self.contact_list[:, 1] > self.cf.rob_nb)[0][0]
        if index > 0 and len(self.contact_list > 0): self.contact_list = self.contact_list[index:, :]

        for i in range(len(self.contact_list)):
            ind = self.contact_list[i, 0]
            self.bots[ind].contact.append(self.contact_list[i, 1])

    def map_grasp(self, contIDs, theta, offset):  # Return locations of contacts in local and global coordinates
        trans = np.asarray([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        positions_l = []
        positions_g = []
        for i in range(len(contIDs)):
            loc_pos = np.copy(self.bots[contIDs[i]].local_pos)
            glob_pos = trans.dot(loc_pos) + offset
            positions_l.append(loc_pos)
            positions_g.append(glob_pos)

        return positions_l, positions_g

    def avg_speed(self):  # Calculate average velocity of bots
        # For now, use average speed directly from Chrono simulation
        speeds = np.zeros(len(self.bots))
        for i in range(len(self.bots)):
            bot = self.bots[i].body
            speeds[i] = (bot.GetPos_dt().x ** 2 + bot.GetPos_dt().y ** 2 + bot.GetPos_dt().z ** 2) ** 0.5

        return np.mean(speeds)

    def avg_com(self):  # Calculate geometric center of robot
        pos = np.zeros((len(self.bots), 3))

        for i in range(len(self.bots)):
            bot = self.bots[i].body
            pos[i, :] = [bot.GetPos().x, bot.GetPos().y, bot.GetPos().z]

        return np.mean(pos, axis=0)

    def roll_bots(self, direction, com):  # Roll bots about center of geometry
        tangents = np.zeros((len(self.bots), 3))

        for i in range(len(self.bots)):
            tangents[i, :] = np.cross([0, 0, 1], [self.bots[i].normal[0], 0, self.bots[i].normal[1]])

        if direction == 1:  # Roll CCW
            for i in range(len(self.bots)):
                robot = self.bots[i]
                arm = np.asarray([robot.xyz[0] - com[0],
                                  robot.xyz[1] - com[1],
                                  robot.xyz[2] - com[2]])
                robot.forcex.SetDir(chrono.ChVectorD(tangents[i, 0], tangents[i, 1], tangents[i, 2]))
                length = np.linalg.norm(arm)
                robot.forcex.SetMforce(self.cf.con_torq / length)

        elif direction == 2:  # Roll CW
            for i in range(len(self.bots)):
                robot = self.bots[i]
                arm = np.asarray([robot.xyz[0] - com[0],
                                  robot.xyz[1] - com[1],
                                  robot.xyz[2] - com[2]])
                robot.forcex.SetDir(chrono.ChVectorD(tangents[i, 0], tangents[i, 1], tangents[i, 2]))
                length = np.linalg.norm(arm)
                robot.forcex.SetMforce(-self.cf.con_torq / length)

    # Print a progress bar
    def printProgressBar(self, length, iteration, total, fill='=', prefix='', suffix='Complete', printEnd='\r'):
        percent = ("{0:." + str(1) + "f}").format(100 * (iteration / float(total)))
        filledLength = int(length * iteration // total)
        bar = fill * filledLength + '-' * (length - filledLength)
        print(f'\r{prefix} |{bar}| {percent}% {suffix}', end=printEnd)

    def simulate(self):  # Run the simulation

        # Initialize the plotting object
        self.plot_obj = plot.plotter(self.bots, self.cf)

        # Start live plots
        self.plot_obj.make_live_plots()

        if self.cf.visual == 'irr':
            self.sim_win = chronoirr.ChIrrApp(self.sys, self.cf.sim_id,
                                              chronoirr.dimension2du(self.cf.resx, self.cf.resy))
            self.sim_win.AddTypicalSky();
            self.sim_win.AddTypicalLogo(self.cf.data_path + 'logo_pychrono_alpha.png')
            self.sim_win.AddTypicalCamera(chronoirr.vector3df(0, self.cf.cam_h, 0), chronoirr.vector3df(0, 0, 0))
            self.sim_win.SetSymbolscale(.002);
            self.sim_win.SetShowInfos(self.cf.showInfo)
            self.sim_win.SetContactsDrawMode(2);
            self.sim_win.SetPaused(self.cf.paused)
            self.sim_win.AddTypicalLights();
            self.sim_win.DrawAll
            self.sim_win.AssetBindAll();
            self.sim_win.AssetUpdateAll()
            self.sim_win.AddShadowAll();
            self.sim_win.SetTimestep(self.cf.tstep)
            self.sim_win.SetTryRealtime(False)
            self.sim_win.SetVideoframeSaveInterval(round(1 / (self.cf.tstep * self.cf.saveFPS)))
            self.sim_win.SetVideoframeSave(self.cf.saveFrames)
            step = 0;
            step1 = 0;
            sens_catch = 0.0;
            nsteps = int((1 / self.cf.sens_rate) / self.cf.tstep);
            tstuck = 0.0
            self.sink_loc = np.asarray([np.nan, np.nan])
            while (self.sim_win.GetDevice().run()):
                # Begin the scene and get current simulation time
                self.sim_win.BeginScene()
                time = self.sys.GetChTime()

                # initialize values

                if (not self.sim_win.GetPaused()):

                    # move the camera to follow bot
                    if (self.cf.track_cam != None):
                        cam_x = self.bots[self.cf.track_cam].body.GetPos().x
                        cam_y = self.bots[self.cf.track_cam].body.GetPos().y
                        cam_z = self.bots[self.cf.track_cam].body.GetPos().z
                        self.sim_win.GetSceneManager().getActiveCamera().setPosition(
                            chronoirr.vector3df(cam_x, cam_y + self.cf.cam_h, cam_z))
                        self.sim_win.GetSceneManager().getActiveCamera().setTarget(
                            chronoirr.vector3df(cam_x, cam_y, cam_z))

                        # Update reference angle measurement
                    self.update_angles()

                    # Draw the scene and print current time
                    self.sim_win.DrawAll()
                    if (self.cf.tout):
                        self.printProgressBar(40, time, self.cf.tend, prefix=(str(round(time, 3)) + ' s'),
                                              suffix='of ' + str(self.cf.tend) + ' s')
                    map_catch = False

                    # Update sensors and calculated data quantities
                    if (step % nsteps == 0 or sens_catch > self.cf.tstep * self.cf.sens_rate) and time > 0.0:
                        sens_catch = 0.0
                        map_catch = True
                        contact_flag = False  # Reset the contact flag

                        if (self.cf.linked):
                            # self.sens_dist()       # Update distance/contact sensors
                            self.sens_tens()  # Update tension sensors
                            # self.calc_pressure()    # Calculate internal pressure

                        # Localization method
                        if (self.cf.con_local == 0):
                            self.coordinates()  # Update using global bot locations
                        elif (self.cf.con_local == 1):
                            self.kalman()  # Estimate position using acclerometers/gyro and kalman filtering

                        elif (self.cf.con_local == 2):
                            self.triangulate()  # Update using local bot locations

                        self.store_contacts()  # Update locations of contacts (local frame)
                        # self.calc_areas()           # Update areas and convexity information
                        # self.rel_effort()           # Update relative effort of each robot

                        # Insert controls functions here
                        for robot in self.bots:
                            # move in set direction until contact is detected
                            robot.user_move([1, 0], 3)

                            # Update the contact flag if there are contacts
                        if len(self.contact_list) > 0: contact_flag = True

                        # One subrobot in contact with something
                        if contact_flag:
                            ncontact = len(self.contact_list)

                            # Environment exploration mode
                            t = self.coord_rot
                            x, y, z = self.bots[0].save_pos()  # Local coordinates
                            self.lpos, self.gpos = self.map_grasp(self.contact_list[:, 0], t, [x, z])

                            # Update normal direction based on robots that are currently in contact
                            locations = [];
                            normals = []
                            for index in self.contact_list[:, 0]:
                                x = self.bots[index].local_pos[0]
                                z = self.bots[index].local_pos[1]
                                locations.append([x, z])

                            for i in range(0, len(locations)):
                                tnormal = np.copy(self.bots[self.contact_list[i, 0]].normal)
                                normals.append(tnormal)

                            locations = np.asarray(locations);
                            position = np.mean(locations, axis=0)  # Local frame
                            normals = np.asarray(normals);
                            normal = np.mean(normals, axis=0)  # Local frame
                            normal = normal / np.linalg.norm(normal)

                            # Local frame, project sink location along contact normal
                            self.sink_loc = position + (2 - 2 * ncontact / self.cf.rob_nb) * self.cf.grasp_rad * normal
                            # print(self.sink_loc)
                            trans = np.asarray([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
                            normal = trans.dot(normal)  # Global frame normal direction

                            # Calculate average speed
                            avg_spd = self.avg_speed()

                            # If not stuck
                            if avg_spd > 0.9:
                                tstuck = time
                                # Move towards target or in target direction
                                for robot in self.bots:
                                    robot.user_move([1, 0], 3)

                            # If stuck
                            elif avg_spd < 0.9 and time - tstuck < 1:
                                # Calculate COM
                                com = self.avg_com()

                                # Roll about COM such that each robot has equal torque
                                self.roll_bots(1, com)

                            # Transition between stuck and moving again
                            else:
                                # Calculate COM
                                com = self.avg_com()

                                # Roll about COM such that each robot has equal torque
                                self.roll_bots(1, com)

                                # Scale user move force component by time stuck
                                for robot in self.bots:
                                    robot.user_move([1, 0], 0.5 * (time - tstuck))

                        # Advance inner stepper
                        step1 += 1

                    # Data collection
                    self.plot_obj.collect(round(time, 3), self.area, self.convexity, self.pressure, self.contact_list,
                                          self.elliptic)

                    if (step % nsteps == 0 or map_catch) and time > 0.0:
                        # Save plot animation frames
                        self.plot_obj.save_plot_frames(step1)

                    # Advance simulation and steppers
                    self.sim_win.DoStep();
                    self.sim_win.EndScene();
                    step += 1;
                    sens_catch += self.cf.tstep

                    # Close the simulation if time ends
                    if time >= self.cf.tend or self.bots[0].body.GetPos().x > 22.:
                        self.sim_win.GetDevice().closeDevice()

        # POV-Ray
        if self.cf.visual == 'pov':

            # Data Paths
            pic_path = self.cf.sim_id + '/pov/picture/'
            out_path = self.cf.sim_id + '/pov/output/'
            anim_path = self.cf.sim_id + '/pov/anim/'
            stat_path = self.cf.sim_id + '/pov/my_state/'

            if not os.path.exists(self.cf.sim_id + '/pov'):
                os.makedirs(self.cf.sim_id + '/pov')
            if not os.path.exists(pic_path):
                os.makedirs(pic_path)
            if not os.path.exists(out_path):
                os.makedirs(out_path)
            if not os.path.exists(anim_path):
                os.makedirs(anim_path)
            if not os.path.exists(stat_path):
                os.makedirs(stat_path)

            # Sets some file names for in-out processes
            pov_exporter = postprocess.ChPovRay(self.sys)
            pov_exporter.SetTemplateFile(chrono.GetChronoDataFile('_template_POV.pov'))
            pov_exporter.SetOutputScriptFile('render.pov')

            pov_exporter.SetOutputDataFilebase(stat_path)
            pov_exporter.SetPictureFilebase(pic_path)
            pov_exporter.SetAntialiasing(True, 9, 0.3)
            # pov_exporter.SetCustomPOVcommandsData("-GW +WL0")
            pov_exporter.SetCamera(chrono.ChVectorD(4, 20, 0), chrono.ChVectorD(4, 0, 0), 45)
            pov_exporter.SetLight(chrono.ChVectorD(-2, 2, -1), chrono.ChColor(1.1, 1.2, 1.2), True)
            pov_exporter.SetPictureSize(self.cf.resx, self.cf.resy)
            pov_exporter.SetAmbientLight(chrono.ChColor(3, 3, 3))
            pov_exporter.AddAll()
            pov_exporter.ExportScript()

            # Initialize Values
            step = 0;
            step1 = 0;
            sens_catch = 0.0;
            nsteps = int((1 / self.cf.sens_rate) / self.cf.tstep)
            tstuck = 0.0;
            frame_int = round(1 / (self.cf.tstep * self.cf.saveFPS))

            # Simulation Loop
            while (self.sys.GetChTime() < self.cf.tend and self.bots[0].body.GetPos().x < 22.):
                time = self.sys.GetChTime()

                # move the camera to follow bot
                if (self.cf.track_cam != None):
                    cam_x = self.bots[self.cf.track_cam].body.GetPos().x
                    cam_y = self.bots[self.cf.track_cam].body.GetPos().y
                    cam_z = self.bots[self.cf.track_cam].body.GetPos().z
                    pov_exporter.SetCamera(chrono.ChVectorD(cam_x, cam_y + self.cf.cam_h, cam_z),
                                           chrono.ChVectorD(cam_x, 0, cam_z), 45)

                    # Update reference angle measurement
                self.update_angles()

                # Print time
                if (self.cf.tout):
                    self.printProgressBar(40, time, self.cf.tend, prefix=(str(round(time, 3)) + ' s'),
                                          suffix='of ' + str(self.cf.tend) + ' s')
                map_catch = False

                # Insert controls and data save functions here

                # Update sensors and calculated data quantities
                if (step % nsteps == 0 or sens_catch > self.cf.tstep * self.cf.sens_rate) and time > 0.0:
                    sens_catch = 0.0
                    map_catch = True
                    contact_flag = False  # Reset the contact flag

                    if (self.cf.linked):
                        # self.sens_dist()       # Update distance/contact sensors
                        self.sens_tens()  # Update tension sensors
                        # self.calc_pressure()    # Calculate internal pressure

                    # Localization method
                    if (self.cf.con_local == 0):
                        self.coordinates()  # Update using global bot locations
                    elif (self.cf.con_local == 1):
                        self.kalman()  # Estimate position using acclerometers/gyro and kalman filtering

                    elif (self.cf.con_local == 2):
                        self.triangulate()  # Update using local bot locations

                    self.store_contacts()  # Update locations of contacts (local frame)
                    # self.calc_areas()           # Update areas and convexity information
                    # self.rel_effort()           # Update relative effort of each robot

                    # Insert controls functions here
                    for robot in self.bots:
                        # move in set direction until contact is detected
                        robot.user_move([1, 0], 3)

                        # Update the contact flag if there are contacts
                    if len(self.contact_list) > 0: contact_flag = True

                    # One subrobot in contact with something
                    if contact_flag:
                        ncontact = len(self.contact_list)

                        # Environment exploration mode
                        t = self.coord_rot
                        x, y, z = self.bots[0].save_pos()  # Local coordinates
                        self.lpos, self.gpos = self.map_grasp(self.contact_list[:, 0], t, [x, z])

                        # Update normal direction based on robots that are currently in contact
                        locations = [];
                        normals = []
                        for index in self.contact_list[:, 0]:
                            x = self.bots[index].local_pos[0]
                            z = self.bots[index].local_pos[1]
                            locations.append([x, z])

                        for i in range(0, len(locations)):
                            tnormal = np.copy(self.bots[self.contact_list[i, 0]].normal)
                            normals.append(tnormal)

                        locations = np.asarray(locations);
                        position = np.mean(locations, axis=0)  # Local frame
                        normals = np.asarray(normals);
                        normal = np.mean(normals, axis=0)  # Local frame
                        normal = normal / np.linalg.norm(normal)

                        # Calculate average speed
                        avg_spd = self.avg_speed()

                        # If not stuck
                        if avg_spd > 0.9:
                            tstuck = time
                            # Move towards target or in target direction
                            for robot in self.bots:
                                robot.user_move([1, 0], 3)

                        # If stuck
                        elif avg_spd < 0.9 and time - tstuck < 1:
                            # Calculate COM
                            com = self.avg_com()

                            # Roll about COM such that each robot has equal torque
                            self.roll_bots(1, com)

                        # Transition between stuck and moving again
                        else:
                            # Calculate COM
                            com = self.avg_com()

                            # Roll about COM such that each robot has equal torque
                            self.roll_bots(1, com)

                            # Scale user move force component by time stuck
                            for robot in self.bots:
                                robot.user_move([1, 0], 0.5 * (time - tstuck))

                    # Advance inner stepper
                    step1 += 1

                # Data collection
                self.plot_obj.collect(round(time, 3), self.area, self.convexity, self.pressure, self.contact_list,
                                      self.elliptic)

                if (step % nsteps == 0 or map_catch) and time > 0.0:
                    # Save plot animation frames
                    self.plot_obj.save_plot_frames(step1)

                if step % frame_int == 0 and time > 0:
                    pov_exporter.ExportData()

                # Advance simulation and steppers
                self.sys.DoStepDynamics(self.cf.tstep)
                step += 1;
                sens_catch += self.cf.tstep

        if self.cf.visual == 'exp':
            a = 1

            # Loop over stored experiment times

            # Read data for each bot's 'sensors' to feed into Kalman filter

            # Perform Kalman filtering

            # Advance timestep

    def unique_rows(self, a):  # Return numpy array of just unique rows
        a = np.ascontiguousarray(a)
        unique_a = np.unique(a.view([('', a.dtype)] * a.shape[1]))
        return unique_a.view(a.dtype).reshape((unique_a.shape[0], a.shape[1]))


# Class to synchronize mapped obstacles between robots
class mapped_obs:
    def __init__(self):
        a = 1


# Class to contain ChBodies that are just obstacles
class body_bag:
    def __init__(self, ChBody):
        self.body = ChBody


# Class to generate objects to be grasped
class grasp_class:
    def __init__(self, material, config):
        self.cf = config
        self.body = chrono.ChBody()
        self.xyz = chrono.ChVectorD(self.cf.grasp_xyz[0], self.cf.grasp_xyz[1], self.cf.grasp_xyz[2])
        self.material = material

    def create_grasp(self):
        if self.cf.grasp_type == 'cyl':
            body = chrono.ChBodyEasyCylinder(self.cf.grasp_rad, self.cf.rob_h, 1000, True, True)

        if self.cf.grasp_type == 'conv':
            # verts = self.generatePolygon(0, 0, self.cf.grasp_rad, self.cf.grasp_irr, 0.0, self.cf.grasp_vert)            
            # body_verts=chrono.vector_ChVectorD()
            # for vert in verts:
            #     body_verts.push_back(chrono.ChVectorD(vert[0],0,vert[1]))

            # body = chrono.ChBodyEasyConvexHull(body_verts,1000.,True,True)

            # # create points for convex hull
            pt_vect = chrono.vector_ChVectorD()
            # creates bottom
            for i in range(self.cf.grasp_vert):
                pt_vect.push_back(
                    chrono.ChVectorD((self.cf.grasp_rad) * np.cos(i * 2 * np.pi / self.cf.grasp_vert), self.cf.rob_h,
                                     (self.cf.grasp_rad) * np.sin(i * 2 * np.pi / self.cf.grasp_vert)))
                # create top
            for i in range(self.cf.grasp_vert):
                pt_vect.push_back(chrono.ChVectorD((self.cf.grasp_rad) * np.cos(i * 2 * np.pi / self.cf.grasp_vert), 0,
                                                   (self.cf.grasp_rad) * np.sin(i * 2 * np.pi / self.cf.grasp_vert)))

            body = chrono.ChBodyEasyConvexHull(pt_vect, 1000, True, True)
        if self.cf.grasp_type == 'conc':
            a = 1

        if self.cf.grasp_type == 'wall':
            param = self.cf.grasp_wall
            body = chrono.ChBodyEasyBox(param[0], param[1], param[2], self.cf.rob_rho, True, True)

        body.SetPos(self.xyz)
        body.SetMaterialSurface(self.material)
        body.SetBodyFixed(True)
        body.AddAsset(self.cf.col_vect[4])
        body.SetId(1001)
        self.body = body

    def generatePolygon(self, ctrX, ctrY, aveRadius, irregularity, spikeyness, numVerts):
        # Generate a random polygon
        '''
        Modified from: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon
        Start with the centre of the polygon at ctrX, ctrY, 
        then creates the polygon by sampling points on a circle around the centre. 
        Randon noise is added by varying the angular spacing between sequential points,
        and by varying the radial distance of each point from the centre.
        
        Params:
        ctrX, ctrY - coordinates of the "centre" of the polygon
        aveRadius - in px, the average radius of this polygon, this roughly controls how large the polygon is, really only useful for order of magnitude.
        irregularity - [0,1] indicating how much variance there is in the angular spacing of vertices. [0,1] will map to [0, 2pi/numberOfVerts]
        spikeyness - [0,1] indicating how much variance there is in each vertex from the circle of radius aveRadius. [0,1] will map to [0, aveRadius]
        numVerts - self-explanatory
        
        Returns a list of vertices, in CCW order.
           '''
        irregularity = self.clip(irregularity, 0, 1) * 2. * np.pi / numVerts
        spikeyness = self.clip(spikeyness, 0, 1) * aveRadius

        # generate n angle steps
        angleSteps = []
        lower = (2. * np.pi / numVerts) - irregularity
        upper = (2. * np.pi / numVerts) + irregularity
        sum = 0
        for i in range(numVerts):
            tmp = np.random.uniform(lower, upper)
            angleSteps.append(tmp)
            sum = sum + tmp

        # normalize the steps so that point 0 and point n+1 are the same
        k = sum / (2. * np.pi)
        for i in range(numVerts):
            angleSteps[i] = angleSteps[i] / k

        # now generate the points
        points = []
        pointsb = []
        angle = np.random.uniform(0, 2. * np.pi)
        for i in range(numVerts):
            r_i = self.clip(np.random.normal(aveRadius, spikeyness), 0, 2. * aveRadius)
            x = ctrX + r_i * np.cos(angle)
            y = ctrY + r_i * np.sin(angle)
            angle = angle + angleSteps[i]
            points.append([x, 0, y])
            pointsb.append([x, self.cf.rob_h, y])

        points = points + pointsb
        return points

    def clip(self, x, min, max):
        if (min > max):
            return x
        elif (x < min):
            return min
        elif (x > max):
            return max
        else:
            return x


# Contact reporting class
class MyReportContactCallback(chrono.ReportContactCallback):
    def __init__(self):
        chrono.ReportContactCallback.__init__(self)
        self.Fcx = []
        self.Fcy = []
        self.Fcz = []
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.bodies = []

    def OnReportContact(self, vA, vB, cA, dist, rad, force, torque, modA, modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetId()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetId()
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fcx.append(force.x)
        self.Fcy.append(force.y)
        self.Fcz.append(force.z)
        self.bodies.append([nameA, nameB])
        return True  # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.Fcx = []
        self.Fcy = []
        self.Fcz = []
        self.bodies = []

    # Get the points
    def GetList(self):
        return (self.pointx, self.pointy, self.pointz, self.Fcx, self.Fcy, self.Fcz, self.bodies)


# Poisson Disc sampling class from https://scipython.com/blog/power-spectra-for-blue-and-uniform-noise/
class Poisson_Sampling:
    def __init__(self, min_distance, width, height):
        """
        Inputs:
            min_distance := The minimum distance between obstacles
            width := x_pos in chrono environment
            height := z_pos in chrono environment
        """

        self.k = 30

        # Minimum distance between samples
        self.r = min_distance

        self.width, self.height = width, height

        # Cell side length
        self.a = self.r / np.sqrt(2)
        # Number of cells in the x- and y-directions of the grid
        self.nx, self.ny = int(width / self.a) + 1, int(height / self.a) + 1

        # A list of coordinates in the grid of cells
        self.coords_list = [(ix, iy) for ix in range(self.nx) for iy in range(self.ny)]
        # Initilalize the dictionary of cells: each key is a cell's coordinates, the
        # corresponding value is the index of that cell's point's coordinates in the
        # samples list (or None if the cell is empty).
        self.cells = {coords: None for coords in self.coords_list}

    def get_cell_coords(self, pt):
        """Get the coordinates of the cell that pt = (x,y) falls in."""

        return int(pt[0] // self.a), int(pt[1] // self.a)

    def get_neighbours(self, coords):
        """Return the indexes of points in cells neighbouring cell at coords.

        For the cell at coords = (x,y), return the indexes of points in the cells
        with neighbouring coordinates illustrated below: ie those cells that could 
        contain points closer than r.

                                     ooo
                                    ooooo
                                    ooXoo
                                    ooooo
                                     ooo

        """

        dxdy = [(-1, -2), (0, -2), (1, -2), (-2, -1), (-1, -1), (0, -1), (1, -1), (2, -1),
                (-2, 0), (-1, 0), (1, 0), (2, 0), (-2, 1), (-1, 1), (0, 1), (1, 1), (2, 1),
                (-1, 2), (0, 2), (1, 2), (0, 0)]
        neighbours = []
        for dx, dy in dxdy:
            neighbour_coords = coords[0] + dx, coords[1] + dy
            if not (0 <= neighbour_coords[0] < self.nx and
                    0 <= neighbour_coords[1] < self.ny):
                # We're off the grid: no neighbours here.
                continue
            neighbour_cell = self.cells[neighbour_coords]
            if neighbour_cell is not None:
                # This cell is occupied: store this index of the contained point.
                neighbours.append(neighbour_cell)
        return neighbours

    def point_valid(self, pt):
        """Is pt a valid point to emit as a sample?

        It must be no closer than r from any other point: check the cells in its
        immediate neighbourhood.

        """

        cell_coords = self.get_cell_coords(pt)
        for idx in self.get_neighbours(cell_coords):
            nearby_pt = self.samples[idx]
            # Squared distance between or candidate point, pt, and this nearby_pt.
            distance2 = (nearby_pt[0] - pt[0]) ** 2 + (nearby_pt[1] - pt[1]) ** 2
            if distance2 < self.r ** 2:
                # The points are too close, so pt is not a candidate.
                return False
        # All points tested: if we're here, pt is valid
        return True

    def get_point(self, k, refpt):
        """Try to find a candidate point relative to refpt to emit in the sample.

        We draw up to k points from the annulus of inner radius r, outer radius 2r
        around the reference point, refpt. If none of them are suitable (because
        they're too close to existing points in the sample), return False.
        Otherwise, return the pt.

        """
        i = 0
        while i < k:
            rho, theta = np.random.uniform(self.r, 2 * self.r), np.random.uniform(0, 2 * np.pi)
            pt = refpt[0] + rho * np.cos(theta), refpt[1] + rho * np.sin(theta)
            if not (0 <= pt[0] < self.width and 0 <= pt[1] < self.height):
                # This point falls outside the domain, so try again.
                continue
            if self.point_valid(pt):
                return pt
            i += 1
        # We failed to find a suitable point in the vicinity of refpt.
        return False

    def get_samples(self):
        # Pick a random point to start with.
        pt = (np.random.uniform(0, self.width), np.random.uniform(0, self.height))
        self.samples = [pt]
        # Our first sample is indexed at 0 in the samples list...
        self.cells[self.get_cell_coords(pt)] = 0
        # ... and it is active, in the sense that we're going to look for more points
        # in its neighbourhood.
        active = [0]

        nsamples = 1
        # As long as there are points in the active list, keep trying to find samples.
        while active:
            # choose a random "reference" point from the active list.
            idx = np.random.choice(active)
            refpt = self.samples[idx]
            # Try to pick a new point relative to the reference point.
            pt = self.get_point(self.k, refpt)
            if pt:
                # Point pt is valid: add it to the samples list and mark it as active
                self.samples.append(pt)
                nsamples += 1
                active.append(len(self.samples) - 1)
                self.cells[self.get_cell_coords(pt)] = len(self.samples) - 1
            else:
                # We had to give up looking for valid points near refpt, so remove it
                # from the list of "active" points.
                active.remove(idx)
        return self.samples
