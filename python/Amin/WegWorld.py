# -*- coding: utf-8 -*-
"""
Created on Wed Feb  3 09:52:38 2021

@author: elope

Use this in the future to add collision detection in the observation space:
    https://stackoverflow.com/questions/50815789/non-colliding-objects-which-has-colliding-pairs-pymunk-pygame
    
This example shows how to draw collisions:
    https://github.com/viblo/pymunk/blob/master/examples/contact_and_no_flipy.py
"""
import pymunk
import pygame
from pygame.color import THECOLORS
from pymunk.pygame_util import *
from pygame.locals import *
from pymunk.vec2d import Vec2d
import sys
import numpy as np
from numpy import sin, cos, sqrt, log
from math import floor
import math
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.interpolate import RegularGridInterpolator
from shutil import rmtree
import glob # For creating videos
import cv2 # For creating videos
from warnings import warn
import os
import pdb

class robot:
    pulling = False
    selected_shapes = []

    def __init__(self, dt, ppm, numStepsPerStep, screenHeight, screenWidth, maxNumSteps, 
                 R, numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, floorFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, minSeparation,
                 headings = None, # Use this if you want to define the heading angles of the robots yourself.
                 dataCollect=False, experimentName="NOT NAMED", 
                 saveVideo = False, systemStart = None):
        """
        All units in this function should be in standard physical units:
            Distance: Meters
            Force: N
            Velocity: m/s
            Mass: kg
        """
        # Basic simulation parameters
        self.dt = dt                                                 # Simulation timestep
        self.ppm = ppm                                               # Pixels per Meter
        self.numStepsPerStep = numStepsPerStep                       # Number of simulation time steps for each single call to 'step()'
        self.convert = Convert(ppm)                                  # Conversion to be used for all parameters given
        self.height= self.convert.Pixels2Meters(screenHeight)        # Height of the screen, in pixels
        self.width = self.convert.Pixels2Meters(screenWidth)         # Width of the screen, in pixels
        self.screenHeight = screenHeight                             # Defining the screen height in pixels
        self.screenWidth = screenWidth                               # Defining the screen width in pixels
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video

        # Distance sensors
        self.background = THECOLORS['grey']                          # Background color of simulation
        self.show_sensors = False                                  # Whether to draw the sensors as well
        self.sensor_color = THECOLORS['blue']                        # If you are drawing sensors, what color should they be?
        self.refined_sensor_color = THECOLORS['pink']               # Colors of sensors that offer refined readings
        self.show_constraints = True                                # Whether to draw the spring and slider constraints
        self.show_membrane = False                                  # Whether to show the membrane. Still under-development
        self.show_heading = False                                    # Whether to draw the headings for bots. This should be removed if you have a large number of bots

        # Parameters for distance sensor read points
        self.distance_reading_max = botRadius*10 # How far the distance sensors can read to
        self.num_points = 30 # Number of sensor points to get a measurement from
        self.firstSensor = botRadius + botRadius/4 # Distance for first sensor from bot center. Must be AT LEAST botRadius, but should make it slightly bigger to offer a little wiggle room
        assert self.firstSensor>=botRadius
        self.refined_distance = botRadius/4 # If we get a reading, we take measurements that we this far spaced apart from the read point

        # Constraint parameters
        self.double_springs = False # If True, will use two springs instead of one at the connection points, where each new spring is added above and below where the original spring was
        self.revolute_joint = True # If True, will add a revolite joint away from the two bots to prevent them from getting too close to eachother.

        # System membrane parameters
        self.R = R
        self.numBots = numBots
        self.action_size = numBots
        self.state_size = 7*numBots # Bot pos (x,y), Vel (x,y), Forces Applied (x,y), External Forces Seen (x,y)
        self.botMass = botMass
        self.botRadius = botRadius     # Radius of
        self.botFriction = botFriction # Friction of bots and skins
        self.skinRadius = skinRadius
        self.skinMass = skinMass
        self.skinRatio = skinRatio

        # Spring parameters
        self.springK = springK
        self.springB = springB
        self.springRL = springRL
        self.maxSeparation = maxSeparation
        self.minSeparation = minSeparation

        # Interior parameters
        self.inRadius = inRadius
        self.inMass = inMass
        self.inFriction = inFriction
        self.percentInteriorRemove = percentInteriorRemove

        # Paramaters for wall and space
        self.slidingFriction = floorFriction      # Coefficient of friction of all bots with the floor.
        self.wallThickness = wallThickness
        self.maxNumSteps = maxNumSteps
        if systemStart==None:
            self.systemStart = self.width/2, self.height/2
        else:
            self.systemStart = systemStart

        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius, self.percentInteriorRemove)
        self.numInterior = np.sum(granPerRing)

        if headings is not None: assert len(headings)==numBots, 'List of heading angles must be the same as the bots!'
        self.botHeadings = headings

        #### Data Collection
        self.environment_parameters = [['PixelsPerMeter (ppm)',str(self.ppm)],
                                       ['SystemRadius',str(self.R)],
                                       ['ScreenWidth',str(self.width)],
                                       ['ScreenHeight',str(self.height)],
                                       ['Num_Bots:', str(self.numBots)],
                                       ['Num_Interior:', str(self.numInterior)],
                                       ['Bot_Radius:', str(self.botRadius)],
                                       ['JAMoEBA_Radius:', str(self.R)],
                                       ['SpringK:', str(self.springK)],
                                       ['SpringD:', str(self.springB)],
                                       ['SpringRL:', str(self.springRL)],
                                       ['inRadius',str(self.inRadius)],
                                       ['inFriction',str(self.inFriction)],
                                       ['skinRatio',str(self.skinRatio)],
                                       ['skinMass',str(self.skinMass)],
                                       ['skinRadius',str(self.skinRadius)],
                                       ['botMass',str(self.botMass)],
                                       ['botFriction',str(self.botFriction)],
                                       ['floorFriction',str(self.slidingFriction)],
                                       ['maxSeparation',str(self.maxSeparation)],
                                       ['maxNumSteps',str(self.maxNumSteps)],
                                       ['percentInteriorRemoved',str(self.percentInteriorRemove)],
                                       ['systemStart',str(systemStart)]]


        self.dataCollect = dataCollect
        if self.dataCollect:
            now = str(datetime.now())
            now = now.replace(":","")
            now = now[:-7]

            self.saveFolder = experimentName+ " Data and Plots "+now+"/"
            os.makedirs(self.saveFolder,exist_ok=True)

            # This +1 is for the extra column needed to record time.
            self.X_data = np.zeros(self.numBots + 1)
            self.X_vel_data = np.zeros(self.numBots + 1)
            self.Y_data = np.zeros(self.numBots + 1)
            self.Y_vel_data = np.zeros(self.numBots + 1)
            self.ac = np.zeros(self.action_size + 1)
            self.reward_data = np.zeros(2)
            self.obs_data = np.zeros(self.state_size +1)

        if self.saveVideo:
            self.videoFolder = experimentName + '_VideoImages/'
            os.makedirs(self.videoFolder,exist_ok=True)

        return None

    def do_event(self, event):
        if event.type == QUIT:
            self.running = False
            print('Pressed escape')

        elif event.type == KEYDOWN:
            print('Pressed key')
            if event.key in (K_q, K_ESCAPE):
                self.running = False



        elif event.type == MOUSEBUTTONDOWN:

            p = from_pygame(event.pos, self.screen)

            self.active_shape = None

            for s in self.space.shapes:
                data = s.point_query(p)
                dist = data.distance
                if dist < 0:
                    self.active_shape = s
                    self.pulling = True
                    s.body.position = p



        elif event.type == MOUSEMOTION:
            self.p = event.pos
            if self.pulling:
                #self.active_shape.body.position = self.p
                self.active_shape.body.position = self.p


        elif event.type == MOUSEBUTTONUP:
            if self.pulling:
                self.pulling = False
                #b.apply_impulse_at_local_point(impulse)


    def reset(self):
        """
        In order the use the variables as they have been defined in the __init__ function, it will be necessary to conver them into
        units of pixels. Thus you will note a portion of this function that converts all variables into one based on units of pixels
        """
        self.render_setup = False # Changes to true once the rendering tools have been setup. This will only happen externally if rendering has been requested
        self.space = pymunk.Space()
        self.space.gravity = 0,0
        self.timestep = 0 # Initializing timestep
        self.time = 0     # Initializing time

        # Information for contact, will be changed later
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)

        # Converting units to pixel coordinates before feeding into space for creation
        R = self.convert.Meters2Pixels(self.R)
        systemStart = self.convert.Meters2Pixels(self.systemStart[0]), self.convert.Meters2Pixels(self.systemStart[1])
        botRadius = self.convert.Meters2Pixels(self.botRadius)
        skinRadius = self.convert.Meters2Pixels(self.skinRadius)
        inRadius = self.convert.Meters2Pixels(self.inRadius)
        springK = self.convert.SpringK2Pixels(self.springK)
        springRL = self.convert.Meters2Pixels(self.springRL)
        maxSeparation = self.convert.Meters2Pixels(self.maxSeparation)
        minSeparation = self.convert.Meters2Pixels(self.minSeparation)

        height = self.convert.Meters2Pixels(self.height)
        width = self.convert.Meters2Pixels(self.width)
        wallThickness = self.convert.Meters2Pixels(self.wallThickness)

        #### Create the system
        # All items that require conversion has been converted above
        kwargs={'space':self.space,
            'systemCenterLocation':systemStart,
            'systemRadius':R,
            'numBots':self.numBots,
            'botMass':self.botMass,
            'botRadius':botRadius,
            'botHeadings':self.botHeadings,
            'skinMass':self.skinMass,
            'skinRadius':skinRadius,
            'skinRatio':self.skinRatio,
            'botFriction':self.botFriction,
            'inRadius':inRadius,
            'inMass':self.inMass,
            'inFriction':self.inFriction,
            'revolute_joint':self.revolute_joint,
            'springK':springK,
            'springB':self.springB,
            'springRL':springRL,
            'twoSprings':self.double_springs,
            'maxSeparation':maxSeparation,
            'minSeparation':minSeparation,
            'percentInteriorRemove':self.percentInteriorRemove,
            'botCollisionIntStart': 2} # All obstacles will be of collision_type=1
        self.bots, interiorParticles, self.membrane, self.constraints, self.distances = createJamoeba(**kwargs)

        self.jamoeba = [*self.membrane, *interiorParticles] # Contains all bodies which make up the JAMoEBA system

        # Add simulated friction if we must
        if self.slidingFriction > 0:
            add_sliding_friction(self.space, self.jamoeba, self.slidingFriction)

        for y, dis in enumerate(self.distances): # Distances are given in pixels, so this must be converted back to meters
            self.distances[y] = self.convert.Pixels2Meters(dis)
        """
        A Note on the above objects:
            self.bots: A list containing all the bots in the system
            interiorParticles: A list containing all the interior particles of the system
            self.membrane: A list containing all bots and skin particles. I.e. anything that makes up the membrane
            self.constraints: Specifically, all the springs in the system
            self.distances: the initial distances of the springs.
        """


        self.jamoeba = [*interiorParticles, *self.membrane]

        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness}
        createWalls(**wallKwargs) # Not creating the walls right now.

        obstacleKwargs = dict(
            space=self.space,
            convert=self.convert
        )
        createObstacles(**obstacleKwargs)

        ## Adding a single ball to show how distance center will work
        # objectKwargs = dict(
        #     space=self.space,
        #     position=(systemStart[0] + 2.5*R,systemStart[1]),
        #     radius=R,
        #     friction=self.botFriction
        # )
        # Obstacle(**objectKwargs)

        #### Collision Handler
        # Reports collisions with walls, objects, and obstacles
        for bot in self.bots:
            cHandler = self.space.add_collision_handler(1,bot.shape.collision_type)
            cHandler.post_solve = self.colPost

        self.render()
        observation, _ = self.getOb()
        return observation





    def step(self, ac):
        """
        Input ac should be a vector describing the force that each bot will apply in that direction.
        Note that forces are only applied in the heading direction of the bot.
        """

        for event in pygame.event.get():
            self.do_event(event)


        # Contact information for storing
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)

        # Taking a step in the environment
        for i in range(self.numStepsPerStep):
            for index, bot in enumerate(self.bots):
                force = ac[index]
                botPos = bot.body.position
                botAngle = bot.body.angle
                xDir = cos(botAngle)
                yDir = sin(botAngle)
                bot.body.apply_force_at_world_point((force*xDir, force*yDir), (botPos.x, botPos.y))

            self.space.step(self.dt)
            self.time += self.dt

        self.timestep+=1
        self.render()

        # Gather information
        obs, systemCenter = self.getOb()
        isDone = self.isDone(systemCenter)

        if self.dataCollect: self.dataCollection(ac,obs)

        return obs, isDone

    def getOb(self):
        """
        Getting a reading from the robot at the current time step.
        The observation is a [n,7] matrix, where n is the number of bots.

        The 7 components of the observation are:
            - Position (x,y)
            - Velocity (x,y)
            - Heading vector (x,y)
            - Distance Reading

        All units are in meters
        """
        # Getting Position and velocity of each bot
        botPos = np.zeros((self.numBots,2))
        botVel = np.zeros((self.numBots,2))
        botAng = np.zeros((self.numBots,2))
        distanceReading = np.zeros((self.numBots,1))

        for index, bot in enumerate(self.bots):
            pos = bot.body.position
            currentBotPos = self.convert.Pixels2Meters(pos.x), self.convert.Pixels2Meters(pos.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            currentBotAng = bot.body.angle

            # Get the sonar readings for this bot
            distanceReading[index] = self.get_sonar_reading(pos, currentBotAng, index) - self.botRadius

            botPos[index, :] = np.array(currentBotPos)
            botVel[index, :] = currentBotVel
            botAng[index, :] = np.hstack([np.cos(currentBotAng), np.sin(currentBotAng)])

        systemCenter = np.mean(botPos, axis=0)

        # Getting external forces of each bot
        extForces = np.abs(np.concatenate((self.extForcesX, self.extForcesY)))

        # The observation is made separately,
        # should you ever want to add or remove items from it.
        observation = np.hstack((botPos, botVel, botAng, distanceReading))

        return observation, systemCenter





    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        self.extForcesX[botIndex-2] = impulse[0] / self.dt
        self.extForcesY[botIndex-2] = impulse[1] / self.dt





    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True




    def get_sonar_reading(self, pos, angle, index):
        """
        Instead of using a grid of boolean(ish) sensors, sonar readings
        simply return N "distance" readings, one for each sonar
        we're simulating. The distance is a count of the first non-zero
        reading starting at the object. For instance, if the fifth sensor
        in a sonar "arm" is non-zero, then that arm returns a distance of 5.

        Refer to :
        https://github.com/harvitronix/reinforcement-learning-car/blob/6d1007410485ee7caaf0cace8d889c768a311a41/flat_game/carmunk.py#L267
        """
        # Make our arm
        x, y = pos.x, pos.y
        pos = np.asarray(pos)
        arm_middle = self.make_sonar_arm(x, y)

        # Rotate them and get readings.
        if self.botHeadings is not None:
            angle -= self.botHeadings[index]
        point = self.get_arm_distance(arm_middle, x, y, angle, 0)
        point = np.asarray(point)

        # Calculate the distance
        distance = np.linalg.norm(point - pos)
        distance = self.convert.Pixels2Meters(distance)

        if self.show_sensors:
            pygame.display.update()

        return distance



    def make_sonar_arm(self, x, y):
        """
        Make an arm of points that we measure the color to `detect` collision

        Refer to:
        https://github.com/harvitronix/reinforcement-learning-car/blob/6d1007410485ee7caaf0cace8d889c768a311a41/flat_game/carmunk.py#L267
        """
        # Get global information
        distance = self.convert.Meters2Pixels(self.firstSensor)
        distance_reading_max = self.convert.Meters2Pixels(self.distance_reading_max)
        num_points = self.num_points

        # Make an arm. We build it flat, but can rotate about
        # center later.
        x_poses = np.linspace(distance, distance_reading_max, num=num_points)
        x_poses += x
        y_poses = np.ones_like(x_poses)*y

        arm_points = np.vstack((x_poses,y_poses)).T
        return arm_points


    def make_refined_arm(self, x, y):
        """
        This function is to be used if we have gather a sonar reading
        and want a more accurate measurement
        """
        refine_dis = self.convert.Meters2Pixels(self.refined_distance)
        num_points = self.num_points

        start = x - refine_dis
        stop = x + refine_dis

        # Make an arm. We build it flat, but can rotate about
        # center later.
        x_poses = np.linspace(start, stop, num=num_points)
        y_poses = np.ones_like(x_poses)*y

        arm_points = np.vstack((x_poses,y_poses)).T
        return arm_points


    def get_arm_distance(self, arm, x, y, angle, offset, refined=False):

        screenHeight = self.screenHeight
        screenWidth = self.screenWidth

        # Look at each point and see if we've hit something
        for point in arm:

            # Move the point to the right spot.
            rotated_p = self.get_rotated_point(
                x, y, point[0], point[1], angle + offset
            )

            # Check if we've hit something. Return the current i (distance)
            # if we did.
            if rotated_p[0] <= 0 or rotated_p[1] <= 0 \
                    or rotated_p[0] >= screenWidth or rotated_p[1] >= screenHeight:
                return point  # Sensor is off the screen.
            else:
                obs = self.screen.get_at(rotated_p)
                if self.get_track_or_not(obs) != 0:
                    # An object has been found.
                    # We must get a refined reading around this area!!
                    # First we make a new arm
                    if not refined:
                        new_arm = self.make_refined_arm(point[0], point[1])
                        point = self.get_arm_distance(new_arm, x, y, angle,0, refined=True)
                        return point
                    else:
                    # for point in new_arm:
                    #     rotated_p = self.get_rotated_point(
                    #         x, y, point[0], point[1], angle + offset
                    #     )
                        return point

            if self.show_sensors:
                if not refined:
                    pygame.draw.circle(self.screen, self.sensor_color, (rotated_p), 2)
                if refined:
                    pygame.draw.circle(self.screen, self.refined_sensor_color, (rotated_p), 2)

        # Return the final point of the arm
        return point



    def get_rotated_point(self, x_1, y_1, x_2, y_2, radians):
        # Rotate x_2, y_2 around x_1, y_1 by angle.
        x_change = (x_2 - x_1) * math.cos(radians) + \
            (y_2 - y_1) * math.sin(radians)
        y_change = (y_1 - y_2) * math.cos(radians) - \
            (x_1 - x_2) * math.sin(radians)
        new_x = x_change + x_1
        new_y = self.screenHeight - (y_change + y_1)
        return int(new_x), int(new_y)




    def get_track_or_not(self, reading):
        if reading == self.background:
            return 0
        else:
            return 1



    def isDone(self, systemCenter):
        """
        A function that defines returns True when the simulation should be terminated.

        It is still the user's responsibility to kill the simulation.
        """
        done=False
        if self.timestep>self.maxNumSteps:
            done=True

        return done


    def draw_membrane(self):
        """
        Will draw lines to represent the membrane.

        Drawing constraints directly does not work well for very large systems.
        """
        N = len(self.membrane)
        joint_pairs = []
        for c, obj in enumerate(self.membrane):
            if c==N-1: # For the last joint
                forward_joint = self.constraints[-1]

                # Get the coordiante refernece current obj
                # We are going to rotate the relative connection point
                # into our current (actual) refernece
                angle = -obj.body.angle
                objA_pos = np.asarray(obj.body.position)
                jointA_pos = np.asarray(forward_joint.anchor_a) # This is the point connected to the current body
                rot_matrix = np.array([
                    [cos(angle), sin(angle)],
                    [-sin(angle), cos(angle)]
                ])
                jointA_pos = np.matmul(rot_matrix, jointA_pos) # This is the relative joint position
                anchor_a = objA_pos + jointA_pos

                # Do the same for anchor B
                objB = self.membrane[0]
                angle = objB.body.angle
                objB_pos = np.asarray(objB.body.position)
                jointB_pos = np.asarray(forward_joint.anchor_b)
                rot_matrix = np.array([
                    [cos(angle), -sin(angle)],
                    [sin(angle), cos(angle)]
                ])
                jointB_pos = np.matmul(rot_matrix, jointB_pos)
                anchor_b = objB_pos + jointB_pos
                joint_pairs.append([anchor_a, anchor_b])
            else:
                forward_joint = self.constraints[c]

                # Get the coordiante refernece current obj
                # We are going to rotate the relative connection point
                # into our current (actual) refernece
                angle = -obj.body.angle
                objA_pos = np.asarray(obj.body.position)
                jointA_pos = np.asarray(forward_joint.anchor_a) # This is the point connected to the current body
                rot_matrix = np.array([
                    [cos(angle), sin(angle)],
                    [-sin(angle), cos(angle)]
                ])
                jointA_pos = np.matmul(rot_matrix, jointA_pos) # This is the relative joint position
                anchor_a = objA_pos + jointA_pos

                # Do the same for anchor B
                objB = self.membrane[c+1]
                angle = objB.body.angle
                objB_pos = np.asarray(objB.body.position)
                jointB_pos = np.asarray(forward_joint.anchor_b)
                rot_matrix = np.array([
                    [cos(angle), -sin(angle)],
                    [sin(angle), cos(angle)]
                ])
                jointB_pos = np.matmul(rot_matrix, jointB_pos)
                anchor_b = objB_pos + jointB_pos
                joint_pairs.append([anchor_a, anchor_b])

        for coord in joint_pairs:
            coord[0] = pymunk.pygame_util.to_pygame(coord[0], self.screen)
            coord[1] = pymunk.pygame_util.to_pygame(coord[1], self.screen)
            pygame.draw.line(
                self.screen,
                THECOLORS['magenta'],
                coord[0], coord[1], 2)







    def render(self):
        if not self.render_setup:
            from pymunk.pygame_util import DrawOptions

            screenHeight = floor(self.convert.Meters2Pixels(self.height))
            screenWidth = floor(self.convert.Meters2Pixels(self.width))

            pygame.init()
            self.screen = pygame.display.set_mode((screenWidth, screenHeight))
            self.clock = pygame.time.Clock()

            self.drawOptions = DrawOptions(self.screen)
            self.drawOptions.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES
            if self.show_heading:
                self.drawOptions.shape_outline_color = (0,0,0,255)
            else:
                # self.drawOptions.shape_outline_color = self.drawOptions.shape_dynamic_color
                self.drawOptions.shape_outline_color = self.background
            if self.show_constraints:
                self.drawOptions.flags |= pymunk.SpaceDebugDrawOptions.DRAW_CONSTRAINTS

            pymunk.pygame_util.positive_y_is_up=True

            self.render_setup=True

        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                pygame.display.quit()
                self.close()
                exit()
            if event.type==pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.display.quit()
                self.close()
                exit()


        self.screen.fill(self.background)
        self.space.debug_draw(self.drawOptions)
        if self.show_membrane:
            self.draw_membrane()
        pygame.display.update()
        if self.saveVideo and self.timestep%1==0: # Can edit this so you are not saving images with as high a frequency
            pygame.image.save(self.screen, self.videoFolder+'image%06d.jpg' % self.timestep)
        self.clock.tick()



    def close(self):
        if self.render_setup:
            pygame.display.quit()
        del self.space





    def dataCollection(self,ac,obs):

        # Create new and empty vecotrs
        X_Pos_temp = [self.time]
        Y_Pos_temp = [self.time]

        X_vel_temp = [self.time]
        Y_vel_temp = [self.time]

        action_temp = [self.time]
        obs_temp = [self.time]

        for bot in self.bots:
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)

            X_Pos_temp.append(currentBotPos[0])
            Y_Pos_temp.append(currentBotPos[1])

            X_vel_temp.append(currentBotVel[0])
            Y_vel_temp.append(currentBotVel[1])


        for action in ac:
            action_temp.append(action)

        for observation in obs.flatten():
            obs_temp.append(observation)


        # Convert to Numpy Arrays
        X_Pos_temp = np.asarray(X_Pos_temp)
        Y_Pos_temp = np.asarray(Y_Pos_temp)

        X_vel_temp = np.asarray(X_vel_temp)
        Y_vel_temp = np.asarray(Y_vel_temp)

        action_temp = np.asarray(action_temp)
        obs_temp = np.asarray(obs_temp)

        # Now append to the master list
        self.X_data = np.vstack([self.X_data, X_Pos_temp])
        self.X_vel_data = np.vstack([self.X_vel_data, X_vel_temp])
        self.Y_data = np.vstack([self.Y_data, Y_Pos_temp])
        self.Y_vel_data = np.vstack([self.Y_vel_data, Y_vel_temp])
        self.ac = np.vstack([self.ac, action_temp])
        self.obs_data = np.vstack([self.obs_data, obs_temp])

    def parameterExport(self, saveLoc=None):
        if saveLoc==None:
            parameter_file=  self.experimentName + '_Environment_parameters.txt'
            print('\n','--'*20)
            warn('No Save location for environment parameters has been specificed')
            print('--'*20,'\n')
        else:
            os.makedirs(saveLoc,exist_ok=True)
            parameter_file = saveLoc+'Environment_parameters.txt'

        with open(parameter_file, 'w') as f:
            for line in self.environment_parameters:
                f.write("%s\n" % line)
            f.write('\n')
            # TODO: Update this to be included only when training is occuring
            # f.write("Number of Training Episodes:{}".format(str(self.episode)))





    def dataExport(self):

        print('Exporting data...',end='')

        # Delete the temporarily made first row of zeroes
        self.X_data = np.delete(self.X_data, 0, 0)
        self.X_vel_data = np.delete(self.X_vel_data, 0, 0)
        self.Y_data = np.delete(self.Y_data, 0, 0)
        self.Y_vel_data = np.delete(self.Y_vel_data, 0, 0)
        self.ac = np.delete(self.ac, 0, 0)
        self.obs_data = np.delete(self.obs_data, 0, 0)


        # Save the data on .csv files
        np.savetxt(self.saveFolder + 'X_data.csv', self.X_data, delimiter=',')
        np.savetxt(self.saveFolder + 'X_vel_data.csv', self.X_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_data.csv', self.Y_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_vel_data.csv', self.Y_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'actions.csv', self.ac, delimiter=',')
        np.savetxt(self.saveFolder + 'observations.csv', self.obs_data, delimiter=',')

        self.plot_data()
        self.parameterExport(self.saveFolder)

        print('Data Export Complete')




    def plot_data(self):
        # Common Among Position and Velocity Data
        last_col = len(self.X_data[0])-1
        time = self.X_data[:,0]
        xlabel = 'Time [sec]'

        # Plot X-Position
        X_COM = []
        for row in self.X_data:
            pos = np.mean(row[1:last_col])
            X_COM.append(pos)
        plt.figure('X-Pos')
        plt.plot(time,X_COM)
        plt.xlabel(xlabel)
        plt.ylabel('X-Position (rel to target) [m]')
        plt.title('X-Center Position')
        plt.savefig(self.saveFolder + 'X-Center Position.jpg')

        # Plot Y-Position
        Y_COM = []
        for row in self.Y_data:
            pos = np.mean(row[1:last_col])
            Y_COM.append(pos)
        plt.figure('Y-Pos')
        plt.plot(time,Y_COM)
        plt.xlabel(xlabel)
        plt.ylabel('Y-Position (rel to target) [m]')
        plt.title('Y-Center Position')
        plt.savefig(self.saveFolder + 'Y-Center Position.jpg')

        # Plot X-velocity
        plt.figure('X-Vel')
        for i in range(self.numBots):
            plt.plot(time, self.X_vel_data[:,i+1], label = 'Bot' + str(i+1))
        plt.xlabel(xlabel)
        plt.ylabel('X Velocity [m/s]')
        plt.title('X-Velocity')
        plt.legend(loc='lower right')
        plt.savefig(self.saveFolder + 'X-Velocity.jpg')

        # Plot Y-Velocity
        plt.figure('Y-Vel')
        for i in range(self.numBots):
            plt.plot(time, self.Y_vel_data[:,i+1], label = 'Bot' + str(i+1))
        plt.xlabel(xlabel)
        plt.ylabel('Y Velocity [m/s]')
        plt.title('Y-Velocity')
        plt.legend(loc='lower right')
        plt.savefig(self.saveFolder + 'Y-Velocity.jpg')

        # Plot Actions
        last_col_2 = len(self.ac[0])-1
        bot=1
        for i in range(last_col_2):
            plt.figure('Applied Force Bot ' + str(bot))
            plt.plot(time, self.ac[:,i])
            plt.xlabel(xlabel)
            plt.ylabel('Force [N]')
            plt.title('Applied Force on Bot ' + str(bot))
            plt.legend(loc='lower right')
            plt.savefig(self.saveFolder + 'Bot ' + str(bot) + ' Applied Forces.jpg')
            bot+=1





class Convert:
    def __init__(self, conversion_ratio):
        """
        Parameters
        ----------
        conversion_ratio : float
            The conversion ratio between pixels and meters in the form (pixels/meter).
        """
        self.ratio = conversion_ratio

    def Pixels2Meters(self, num_pixels):
        return (num_pixels*(1/self.ratio))

    def Meters2Pixels(self, meters):
        return (meters*self.ratio)

    def SpringK2Pixels(self, springK):
        return (springK*(1/self.ratio))

    def Pixels2SpringK(self, springKPixels):
        return (springKPixels*self.ratio)


class Ball:
    def __init__(self, space, position, radius, mass, friction,
                 collisionType = 0, color = (0,255,0,255), theta=0):
        self.body = pymunk.Body()
        self.radius = radius
        self.body.position = position
        self.body.angle = theta
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.mass = mass
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = collisionType
        space.add(self.body, self.shape)


class Bar:
    def __init__(self, space, body, position, radius, mass, friction,
                 collisionType = 0, color = (0,255,0,255), theta=0):
        self.body = body#pymunk.Body()
        self.radius = radius
        #self.body.position = position
        #self.body.angle = theta
        a = tuple(position - np.array([radius, 0]))
        b = tuple(position + np.array([radius, 0]))
        a = (-radius * np.cos(theta), -radius * np.sin(theta))
        b =  (radius * np.cos(theta), radius * np.sin(theta))
        self.shape = pymunk.Segment(self.body, a, b, radius/10)
        self.shape.mass = mass
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = collisionType
        #space.add(self.body, self.shape)
        space.add(self.shape)




class Obstacle:
    def __init__(self, space, position, radius, friction, color = (0,0,0,255)):
        self.body = pymunk.Body(body_type = pymunk.Body.STATIC)
        self.radius = radius
        self.body.position = position
        self.shape = pymunk.Circle(self.body, self.radius)
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = 1
        space.add(self.body, self.shape)





def connectBalls(space, b1, b2, rest_length, spring_stiffness,
                 spring_damping, maxSeparation, minSeparation, isBot=[False, False],
                 thetaPlus=[0,0], twoSprings=False, revolute_joint = False, revolute_joint_bar=False):
    """
    If any(isBot==True), then the True index is a bot and we must account for
    its rotation when attaching a spring. This extra rotation is accounted for
    in thetaPlus

    twoSprings (Boolean): If true, will add two springs instead of one.
    revolute_joint (Boolean): If true, will NOT use springs, and instead connect bots using a pin joint. Will default to this if both this and 'revolute_joint_bar' are true.
    revolute_joint_bar (Boolean): If true, will NOT use springs, and will instead connect bots with a pin and physical bar.
    """

    if not revolute_joint:
        """
        Using springs instead of a revolution joint.
        Note that the actual joint is a pin joint, not a revolute.
        """
        # The original anchors are left here for the slide joint.
        anchor_a = b1.radius*sin(thetaPlus[0]), b1.radius*cos(thetaPlus[0])
        anchor_b = -b2.radius*sin(thetaPlus[1]),-b2.radius*cos(thetaPlus[1])

        if not twoSprings: # Standard spring creation
            springConstraint = pymunk.DampedSpring(b1.body, b2.body,
                                                anchor_a,anchor_b,
                                                rest_length, spring_stiffness,
                                                spring_damping)
            space.add(springConstraint)




        else: # Adding two springs instead of one
            xrot, yrot = cos(np.pi/4), sin(np.pi/4)

            len_m = 1.

            # First Body
            anchor_a1 = np.array([b1.radius*xrot, b1.radius*yrot])
            anchor_a2 = np.array([-b1.radius*xrot, b1.radius*yrot])
            rot = rot_matrix(-thetaPlus[0])
            anchor_a1 = tuple(np.matmul(rot,anchor_a1))
            anchor_a2 = tuple(np.matmul(rot,anchor_a2))


            # Second body
            anchor_b1 = np.array([b2.radius*xrot, -b2.radius*yrot])
            anchor_b2 = np.array([-b2.radius*xrot, -b2.radius*yrot])
            rot = rot_matrix(-thetaPlus[1])
            anchor_b1 = tuple(np.matmul(rot,anchor_b1))
            anchor_b2 = tuple(np.matmul(rot,anchor_b2))

            r1 = rot_matrix(b1.body.angle)
            r2 = rot_matrix(b2.body.angle)
            tmp = np.subtract(np.add(r1 @ anchor_a1, b1.body.position), np.add(r2 @ anchor_b1, b2.body.position))
            rest_length = np.linalg.norm(tmp) * len_m
            # Now add the springs!
            springConstraint = pymunk.DampedSpring(b1.body, b2.body, # The outer spring is treated as the 'main' one
                                                    anchor_a1,anchor_b1,
                                                    rest_length, spring_stiffness,
                                                    spring_damping)

            tmp = np.subtract(np.add(r1 @ anchor_a2, b1.body.position), np.add(r2 @ anchor_b2, b2.body.position))
            rest_length = np.linalg.norm(tmp) * len_m
            springConstraint2 = pymunk.DampedSpring(b1.body, b2.body,
                                                    anchor_a2,anchor_b2,
                                                    rest_length, spring_stiffness,
                                                    spring_damping)

            tmp = np.subtract(np.add(r1 @ anchor_a1, b1.body.position), np.add(r2 @ anchor_b2, b2.body.position))
            rest_length = np.linalg.norm(tmp) * len_m
            # Now add the springs!
            springConstraint3 = pymunk.DampedSpring(b1.body, b2.body,  # The outer spring is treated as the 'main' one
                                                anchor_a1, anchor_b2,
                                                rest_length, spring_stiffness,
                                                spring_damping)

            tmp = np.subtract(np.add(r1 @ anchor_a2, b1.body.position), np.add(r2 @ anchor_b1, b2.body.position))
            rest_length = np.linalg.norm(tmp) * len_m
            springConstraint4 = pymunk.DampedSpring(b1.body, b2.body,
                                                    anchor_a2, anchor_b1,
                                                    rest_length, spring_stiffness,
                                                    spring_damping)

            #  springConstraint2,
            space.add(springConstraint, springConstraint2, springConstraint3, springConstraint4)

        if minSeparation > 0 and maxSeparation < np.inf: # Distance constraints are added all the same
            slideJoint = pymunk.SlideJoint(b1.body, b2.body,
                                        anchor_a,anchor_b,
                                        minSeparation, maxSeparation)
            space.add(slideJoint)


    elif revolute_joint:
        """
        Connect the bodies using a pin joint about their center. 
        A virutal body will be created between the two bodies' centers.
        Also, the joint will be with respect to the center of both bodies.
        """
        
        # First get the center position between both of these bodies
        anchor_a = p1 = np.asarray(b1.body.position)
        anchor_b = p2 = np.asarray(b2.body.position)
        c = np.vstack((p1,p2))
        c = np.mean(c,axis=0)

        # Get distances between each side. Also create a buffer
        d1 = np.linalg.norm(c-p1)
        d2 = np.linalg.norm(c-p2)
        buffer = b1.radius/5

        # Add ball
        b = Bar(space, b1.body, (c[0], c[1]), 22, 0.01, 0.1, 0, theta=thetaPlus[1])

        rest_length = np.linalg.norm(np.subtract(p1, p2))
        springConstraint4 = pymunk.DampedSpring(b1.body, b2.body,
                                                (0, 5), (0, 5),
                                                rest_length, spring_stiffness,
                                                spring_damping)

        # Constrain both of these bodies w.r.t. c_body
        pivot = pymunk.PivotJoint(
            b1.body, b2.body,
            tuple(c)
        )
        space.add(pivot, springConstraint4)

        # Add a virtual sprint, just so we can show the membrane
        springConstraint = pymunk.DampedSpring(
            b1.body, b2.body,
            tuple(p1),tuple(p2),
            0,0,0 # The spring has no properties, so should have no effect on the system
        )
        springConstraint = None


    # Get distance between anchors
    a = np.asarray(anchor_a)
    b = np.asarray(anchor_b)
    dis = np.linalg.norm(a-b)

    return springConstraint, dis




def createJamoeba(space, systemCenterLocation, systemRadius, numBots, botMass,
                  botRadius, botHeadings, skinMass, skinRadius, skinRatio,
                  botFriction, springK, springB, springRL, maxSeparation,
                  minSeparation, inRadius, inMass, inFriction,
                  twoSprings=False, revolute_joint=False,
                  percentInteriorRemove = 0, botCollisionIntStart = 2):
    """
    twoSprings: Whether to use two springs when connecting the bots instead of 1
    revolute_joint: Whether to pivot 
    
    """
    xCenter = systemCenterLocation[0]
    yCenter = systemCenterLocation[1]

    collisionType = botCollisionIntStart

    bots = []
    interiorParticles = []
    membrane = []
    isBotList = []
    constraints = []
    distances = []

    # Get interior particles
    gran_per_ring, in_rings_radius = interiorPattern(systemRadius, inRadius,
                                                     botRadius, percentInteriorRemove)

    #Parameter for skins
    t = (2*np.pi/numBots)/(skinRatio+1)

    # Did the user pre-define the heading angles for the bots?
    useHeadings = False
    if botHeadings is not None: useHeadings = True

    for i in range(numBots):
        theta = i*2*np.pi/numBots
        x = xCenter + systemRadius*np.cos(theta)
        y = yCenter + systemRadius*np.sin(theta)

        thetaAdd = 0
        if useHeadings:
            thetaAdd = botHeadings[i]
        thetaPlus = theta + thetaAdd
        if i == 0:
            color = (255,255,255,255) # Color the first bot white
        else:
            color = (255,0,0,255)
        bot = Ball(space, (x,y), botRadius, botMass, botFriction, collisionType, color=color, theta=thetaPlus)
        collisionType += 1
        bots.append(bot)
        membrane.append(bot)
        isBotList.append(True)

        # Skin particles
        for j in range(1,skinRatio+1):
            thetaSkin = theta + j*t
            x = xCenter + systemRadius*np.cos(thetaSkin)
            y = yCenter + systemRadius*np.sin(thetaSkin)
            skin = Ball(space, (x,y), skinRadius, skinMass, botFriction, color=(0,0,255,255), theta = thetaSkin)
            membrane.append(skin)
            isBotList.append(False) #Needed to decipher the heading angles of each bot

    numBodies = len(membrane)
    if useHeadings:
        whereBots = np.where(isBotList)[0] # This will help us track which bot we are currently observing.
        for index, body in enumerate(membrane):
            if index < (numBodies-1):
                """
                We must check if the ball we are connecting is a bot.
                If it is a bot, then we would like to add the headingAngle that 
                corresponds with that bot. Otherwise, proceed as normal.
                """

                isBot = [isBotList[index], isBotList[index+1]] # We must know if the current or next object we are attaching is a bot that has been rotated!
                thetaPlus = [0,0]
                for k, w in enumerate(isBot):
                    if w: # This is True if it is a bot
                        # We track which heading index we are on using 'whereBots'
                        whichHeading = np.where(whereBots==index+k)[0][0]
                        thetaPlus[k] = botHeadings[whichHeading]

                c, dis = connectBalls(space, body,
                             membrane[index+1], springRL, springK, springB,
                             maxSeparation, minSeparation, isBot, thetaPlus, twoSprings=twoSprings, revolute_joint=revolute_joint)
                constraints.append(c)
                distances.append(dis)


            else:
                isBot = [isBotList[-1], isBotList[0]]
                thetaPlus = [0,0]
                for k, w in enumerate(isBot):
                    if w: # This is True if it is a bot
                        # We track which heading index we are on using 'whereBots'
                        if k: # k=1
                            whichHeading = 0 # We assume that the last thing to connect is ALWAYS a bot
                        else: # k=0
                            whichHeading = np.where(whereBots==index)[0][0]
                        thetaPlus[k] = botHeadings[whichHeading]
                c, dis = connectBalls(space, body, membrane[0], springRL,
                             springK, springB, maxSeparation, minSeparation, isBot, thetaPlus, twoSprings=twoSprings, revolute_joint=revolute_joint)
                constraints.append(c)
                distances.append(dis)


    # Connect the balls normally
    else:
        for index, body in enumerate(membrane):
            if index < (numBodies-1):
                c, dis = connectBalls(space, body, membrane[index+1], springRL, springK, springB, maxSeparation, minSeparation, twoSprings=twoSprings, revolute_joint=revolute_joint)
                constraints.append(c)
                distances.append(dis)
            else:
                # Connect last ball to first
                c, dis = connectBalls(space, body, membrane[0], springRL, springK, springB, maxSeparation, minSeparation, twoSprings=twoSprings, revolute_joint=revolute_joint)
                constraints.append(c)
                distances.append(dis)


    # Create Interiors
    for index, in_ring in enumerate(gran_per_ring):
        radius = in_rings_radius[index]
        for j in range(in_ring):
            in_theta = j*2*np.pi/in_ring
            x = xCenter + radius*np.cos(in_theta)
            y = yCenter + radius*np.sin(in_theta)

            interiorParticle = Ball(space, (x,y), inRadius, inMass, inFriction)
            interiorParticles.append(interiorParticle)

    return bots, interiorParticles, membrane, constraints, distances



def add_sliding_friction(space, system, mu):
    """
    Will iterate through system and add a simulated sliding friction
    """
    static_body = space.static_body
    g = 9.81 # Getting the acceleration due to gravity

    for obj in system:
        body = obj.body
        mass = body.mass
        friction_force = g*mass*mu

        # Create constraint for friction force
        pivot = pymunk.PivotJoint(static_body, body, (0,0), (0,0))
        space.add(pivot)
        pivot.max_bias = 0
        pivot.max_force = friction_force

        # Create a constraint for sliding friction
        gear = pymunk.GearJoint(static_body, body, 0.0, 1.0)
        space.add(gear)
        gear.max_bias = 0 # Disable joint correctioon
        gear.max_force = friction_force




def interiorPattern(systemRadius, inRadius, botRadius, percentInteriorRemove=0):
    R = systemRadius
    in_rings_radius = [] #**
    gran_per_ring = []

    buffer = .001 # Distance between rings
    d_start = botRadius - inRadius # Distance between first ring and outer boxes
    if d_start<0: d_start = 0
    current_radius = R - (2*botRadius/2 + inRadius + d_start)
    current_circumference = 2*np.pi*current_radius

    while current_circumference > (2*inRadius)*3: # Not allowing less than 3 spheres in a ring
        in_rings_radius.append(current_radius)
        current_radius -= (2*inRadius + buffer)
        current_circumference = 2*np.pi*current_radius

    for radius in in_rings_radius:
        current_num_interior = floor((2*np.pi*radius)/(2*inRadius))
        gran_per_ring.append(current_num_interior)

    for num in in_rings_radius:
        if num<0:
            in_rings_radius.remove(num)
    for num in gran_per_ring:
        if num<0:
            gran_per_ring.remove(num)

    # Removing a percentage of the interior if too crowded
    numInterior = np.sum(gran_per_ring)
    numRemove = floor(numInterior*percentInteriorRemove)

    numRings= len(gran_per_ring)
    if numRemove==0: removePerRing=0
    else:
        left_to_remove = numRemove
        order = np.arange(numRings)
        np.random.shuffle(order)
        for index in order:
            gran_this_ring = gran_per_ring[index]
            if left_to_remove > gran_this_ring:
                gran_per_ring[index] = 0
                left_to_remove -= gran_this_ring

            elif left_to_remove>0 and left_to_remove<=gran_this_ring:
                gran_per_ring[index] -= left_to_remove
                left_to_remove -= left_to_remove

            if left_to_remove <= 0:
                break


    return (gran_per_ring, in_rings_radius)





def createWalls(space, screenHeight, screenWidth, wallThickness):
    # All values for this function are assumed to already be in Pixels

    # Bottom Wall
    body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1 = pymunk.Poly.create_box(body1, (screenWidth,wallThickness))
    shape1.body.position = (screenWidth/2, 0)
    shape1.collision_type = 1

    # Top Wall
    body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2 = pymunk.Poly.create_box(body2, (screenWidth,wallThickness))
    shape2.body.position = (screenWidth/2, screenHeight)
    shape2.collision_type=1

    # Back wall
    body3 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape3 = pymunk.Poly.create_box(body3,(wallThickness, screenHeight))
    shape3.body.position = (0, screenHeight/2)
    shape3.collision_type=1

    # Front wall
    body4 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    shape4 = pymunk.Poly.create_box(body4, (wallThickness, screenHeight))
    shape4.body.position = (screenWidth,screenHeight/2)
    shape4.collision_type = 1

    space.add(shape1,body1,shape2,body2,shape3,body3,shape4,body4)
    return None

def createObstacles(space, convert):
    # Adding the obstacles per Amin's specification
    x_dim, y_dim = .135, .65
    dim = np.array([x_dim,y_dim])
    dim = convert.Meters2Pixels(dim)

    # Creating left wall
    x = .65+.135
    y = .135/2+.65/2
    pos1 = np.array([x,y])
    pos1 = convert.Meters2Pixels(pos1)
    body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1 = pymunk.Poly.create_box(body1, dim)
    shape1.body.position = tuple(pos1)
    shape1.collision_type = 1

    # Creating the right wall
    x = .65*2+.135
    y = .135/2+1.3 - .65/2
    pos2 = np.array([x,y])
    pos2 = convert.Meters2Pixels(pos2)
    body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2 = pymunk.Poly.create_box(body2, dim)
    shape2.body.position = tuple(pos2)
    shape2.collision_type = 1

    space.add(shape1,body1,shape2,body2)



def createVideo(saveLoc, imgLoc, videoName, imgShape):
    print('\nCreating video...',end='')
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 80, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        out.write(img)
    out.release

    rmtree(imgLoc)
    print('Video Creation Complete')

def rot_matrix(theta):
    """
    Creates and returns a rotation matrix
    """
    rot = np.array([
        [cos(theta), -sin(theta)],
        [sin(theta),cos(theta)]
    ])
    return rot