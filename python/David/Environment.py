
"""
Use this in the future to add collision detection in the observation space:
    https://stackoverflow.com/questions/50815789/non-colliding-objects-which-has-colliding-pairs-pymunk-pygame
    
This example shows how to draw collisions:
    https://github.com/viblo/pymunk/blob/master/examples/contact_and_no_flipy.py
"""

from matplotlib.cbook import maxdict
import pymunk
import pygame
from pygame.color import THECOLORS
import numpy as np
from numpy.linalg import norm
from math import floor
from gym import Env
import matplotlib.pyplot as plt
from datetime import datetime
from shutil import rmtree
import glob # For creating videos
import cv2 # For creating videos
from tqdm import tqdm
from warnings import warn
import os
from copy import copy

class robot(Env):
    
    info = {'There is no info to be aware of.':'Machinelearn away.'}
    
    def __init__(self, dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, inRadiusRandomness, inRadiusRandomnessPer, botFriction, inMass, inFriction, percentInteriorRemove,
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect=False, experimentName="NOT NAMED", 
                 saveVideo = False,
                 numStepsPerStep=1,
                 floorFriction=0):
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
        self.convert = Convert(ppm)                                  # Conversion to be used for all parameters given
        self.height= self.convert.Pixels2Meters(screenHeight)        # Height of the screen, in pixels
        self.width = self.convert.Pixels2Meters(screenWidth)         # Width of the screen, in pixels
        self.screenHeight = screenHeight                             # Defining the screen height in pixels
        self.screenWidth = screenWidth                               # Defining the screen width in pixels
        self.maxNumSteps = maxNumSteps                               # Number of steps until simulation terminated
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        self.numStepsPerStep = numStepsPerStep                       # The number of simulation timesteps to run for each call to 'step' function
        self.slidingFriction = floorFriction                         # Coefficient of friction for objects sliding on the ground. I.e. a top-down view and translation friction.

        # Some visualization parameters
        self.draw_forces = False
        self.force_color = THECOLORS['yellow']

        # Variables for sticky joints
        self.use_sticky_connections = True
        if self.use_sticky_connections: # If you use a sticky connection, then you must define how long it is allowed to be
            max_sticky_connection = botRadius/2
            self.breaking_force = np.inf # Force in Newtons at which the joint should be broken
            self.max_sticky_distance = self.convert.Meters2Pixels(max_sticky_connection) # This value should be in Pixels, as it is transferred over to PyMunk
            self.max_sticky_force = []

        # System membrane parameters
        self.R = R
        self.numBots = numBots
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
        
        # Interior parameters
        self.inRadius = inRadius
        self.inRadiusRandomness = inRadiusRandomness
        self.inRadiusRandomnessPer = inRadiusRandomnessPer
        self.inMass = inMass
        self.inFriction = inFriction
        self.percentInteriorRemove = percentInteriorRemove
        
        # Paramaters for wall and space
        self.wallThickness = wallThickness
        self.systemStart = R+botRadius+wallThickness*1.2, self.convert.Pixels2Meters(screenHeight/2)
        
        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius, self.percentInteriorRemove)
        self.numInterior = np.sum(granPerRing)
        
        #### Data Collection
        self.environment_parameters = [['dt:',str(self.dt)],
                                       ['NumSetpsPerStep:',str(self.numStepsPerStep)],
                                       ['Num_Bots', str(self.numBots)],
                                       ['botMass',str(self.botMass)],
                                       ['botRadius',str(self.botRadius)],
                                       ['skinRadius',str(self.skinRadius)],
                                       ['skinMass',str(self.skinMass)],
                                       ['skinRatio',str(self.skinRatio)],
                                       ['inRadius',str(self.inRadius)],
                                       ['inRadiusRandomness', self.inRadiusRandomness],
                                       ['inRadiusRandomnessPer', str(self.inRadiusRandomnessPer)],
                                       ['botFriction',str(self.botFriction)],
                                       ['inMass',str(self.inMass)],
                                       ['inFriction',str(self.inFriction)],
                                       ['percentInteriorRemoved',str(self.percentInteriorRemove)],
                                       ['SpringK:', str(self.springK)], 
                                       ['SpringB:', str(self.springB)], 
                                       ['SpringRL:', str(self.springRL)],
                                       ['maxSeparation',str(self.maxSeparation)],
                                       ['floorFriction', str(self.slidingFriction)],
                                       ['PixelsPerMeter (ppm):',str(self.ppm)],
                                       ['SystemRadius',str(self.R)],
                                       ['ScreenWidth',str(self.width)],
                                       ['ScreenHeight',str(self.height)],
                                       ['Num_Interior:', str(self.numInterior)],
                                       ['Bot_Radius:', str(self.botRadius)],
                                       ['JAMoEBA_Radius:', str(self.R)], 
                                       ['maxNumSteps',str(self.maxNumSteps)]
                                       ]
        
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
            self.ac = np.zeros(self.numBots*2 + 1)
            self.reward_data = np.zeros(2)
            self.obs_data = np.zeros(self.numBots*7 +1) # Wil need to be changed if you later change the observations size.

            # TODO:
            # The above arrays do not need to be numpy arrays. In fact,
            # that is probably what is slowing down the computation during 
            # data collection. 
            # They should instead be empty lists that we just append to.
            
        if self.saveVideo:
            assert self.dataCollect, "You must also collect data when saving a video!"
            self.videoFolder = experimentName + '_VideoImages/'
            os.makedirs(self.videoFolder,exist_ok=True)
        
        return None
        
        
        
    def setBackground(self, img):
        self.background = pygame.image.fromstring(img.tobytes(), img.size, img.mode)
        
    def reset(self, startLoc=None):
        
        self.render_setup = False # Changes to true once the rendering tools have been setup. This will only happen externally if rendering has been requested
        self.space = pymunk.Space()
        self.space.gravity = 0,0
        self.timestep = 0 # Initializing timestep
        self.time = 0     # Initializing time
        
        # Information for contact, will be changed later
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)

        # Sticky constraints
        self.constrained_bots = [] # Bots that are attached to an object will go here.
        self.sticky_connections = [] # Where we will store the joints that we make (which are sticky)
        
        # Converting units to pixel coordinates before feeding into space for creation
        R = self.convert.Meters2Pixels(self.R)
        botRadius = self.convert.Meters2Pixels(self.botRadius)
        skinRadius = self.convert.Meters2Pixels(self.skinRadius)
        inRadius = self.convert.Meters2Pixels(self.inRadius)
        springK = self.convert.SpringK2Pixels(self.springK)
        springRL = self.convert.Meters2Pixels(self.springRL)
        maxSeparation = self.convert.Meters2Pixels(self.maxSeparation)

        # Defining the system start
        if startLoc is None:
            systemStart = self.convert.Meters2Pixels(self.systemStart[0]), self.convert.Meters2Pixels(self.systemStart[1])
        else:
            systemStart = self.convert.Meters2Pixels(startLoc)
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
            'skinMass':self.skinMass,
            'skinRadius':skinRadius,
            'skinRatio':self.skinRatio,
            'botFriction':self.botFriction,
            'inRadius':inRadius,
            'inRadiusRandomness': self.inRadiusRandomness,
            'inRadiusRandomnessPer': self.inRadiusRandomnessPer,
            'inMass':self.inMass,
            'inFriction':self.inFriction,
            'springK':springK,
            'springB':self.springB,
            'springRL':springRL,
            'maxSeparation':maxSeparation,
            'percentInteriorRemove':self.percentInteriorRemove,
            'botCollisionIntStart': 2} # All obstacles will be of collision_type=1
        self.bots, interiorParticles, self.extraObjects, membrane, self.springs = createJamoeba(**kwargs)

        # Note: Membrane consists of bots and skin particles
        self.jamoeba = [*membrane, *interiorParticles] # This is all bodies within the space! 
        self.passive_membrane = []
        for membrane_object in membrane:
            if not membrane_object.isBot:
                self.passive_membrane.append(membrane_object)



        # Add simulated friction if we must
        if self.slidingFriction > 0:
            add_sliding_friction(self.space, self.jamoeba, self.slidingFriction)

        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness,
                    'env':self}
        createWalls(**wallKwargs) # Not creating the walls right now.
    
        # Adding object to grab
        # total_mass = 0
        # for obj in self.jamoeba:
        #     total_mass += obj.body.mass
        # square_mass = total_mass/2

        # squareKwargs = dict(
        #     space = self.space,
        #     position = (systemStart[0] + R*1.5, systemStart[1]),
        #     mass = square_mass,
        #     length = R/2,
        #     friction = self.botFriction
        # )
        # self.square = squareObject(**squareKwargs)

        #### Collision Handler
        # Reports collisions with walls, objects, and obstacles
        for bot in self.bots:
            cHandler = self.space.add_collision_handler(1, bot.shape.collision_type)
            cHandler.post_solve = self.colPost
        
        #### Initiate Observation
        if self.draw_forces: self.force_vectors = []
        ac = np.array([0]*2*self.numBots) # Take no action
        observation, _ = self.getOb(ac)
        return observation
    
    
    
    
        
    def step(self, ac, passive_ac=None):
        
        assert ac.shape == (self.numBots, 2), "Action must be of shape (numBots, 2)"

        if self.draw_forces:
            self.force_vectors = []
        
        # Contact information for storing
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
        
        for i in range(self.numStepsPerStep):

            forcesX = ac[:, 0] * self.ppm
            forcesY = ac[:, 1] * self.ppm
                
            for index, bot in enumerate(self.bots):
                xForce = forcesX[index]
                yForce = forcesY[index]

                # To apply force, we find the body's location and apply the force its center
                botPos = bot.body.position
                bot.body.apply_force_at_world_point((xForce, yForce), botPos)

            if passive_ac is not None:
                try:
                    for index, passive_particle in enumerate(self.passive_membrane):
                        heading_angle = passive_particle.body.angle
                        heading_vector = np.asarray((np.cos(heading_angle), np.sin(heading_angle)))

                        passive_force = passive_ac[index]   
                        force_vec = tuple(passive_force*heading_vector)
                        passivePos = passive_particle.body.position
                        passive_particle.body.apply_force_at_world_point(force_vec, passivePos)
                except:
                    print('Failure to apply forces to passive particles!')

                if self.draw_forces:
                    # Collect the vector here
                    force_vec = np.array([xForce, yForce])
                    #if np.linalg.norm(force_vec) > .001:
                    #    force_vec /= np.linalg.norm(force_vec)
                    #else:
                    #    force_vec = np.array([1., 0.])
                    anchor_a = np.array([botPos.x, botPos.y])
                    anchor_b = anchor_a + force_vec*10 # The scalar we multiply with defines how many pixels long the line is
                    
                    # Convert to pygame coordinates
                    anchor_a = pymunk.pygame_util.to_pygame(anchor_a, self.screen)
                    anchor_b = pymunk.pygame_util.to_pygame(anchor_b, self.screen)

                    self.force_vectors.append(
                        [anchor_a, anchor_b]
                    )

        # Taking a step in the environment
            self.space.step(self.dt)
            self.time += self.dt

        # Check if we need to break any sticky joints
        if self.use_sticky_connections:
            self.check_sticky_connections()
        
        # Note the important difference between timestep and time!
        # The timestep is number of times we call self.step()
        self.timestep+=1

        # Gather an observation and whether the simulation is done.
        obs, _ = self.getOb(ac)
        isDone = self.isDone()

        if self.dataCollect: 
            self.dataCollection(ac, obs)
 
        return obs, isDone
    
    
    
    def getOb(self, ac):

        botPos = np.zeros((self.numBots, 2))
        botVel = np.zeros((self.numBots, 2))
        botAng = np.zeros((self.numBots, 2))
        botAngVel = np.zeros((self.numBots, 1))
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            currentBotAng = bot.body.angle
            currentBotAngVel = bot.body.angular_velocity
            
            # Define position as relative to the target
            botPos[index, :] = np.array(currentBotPos)
            botVel[index, :] = currentBotVel
            botAng[index, :] = np.hstack([np.cos(currentBotAng), np.sin(currentBotAng)])
            botAngVel[index, :] = currentBotAngVel

        objectData = np.array([[self.convert.Pixels2Meters(self.extraObjects[0].body.position.x),
                                self.convert.Pixels2Meters(self.extraObjects[0].body.position.y)]])
        objectData = np.concatenate((objectData, np.zeros((self.numBots - 1, 2))))

        systemCenter = np.mean(botPos, axis=0)

        # All external forces the bots are experiences
        extForces = np.abs(np.vstack((self.extForcesX, self.extForcesY))).T
        
        # The observation is made separately, should you ever want to add or remove items from it.
        observation = np.hstack((botPos, botVel, botAng, botAngVel, objectData))
        
        return observation, systemCenter


    def check_sticky_connections(self):
        """
        Iterating through all sticky joints and 
        checking if they must be broken
        """
        max_force = 0
        for index, joint in enumerate(self.sticky_connections):
            joint_force = joint.impulse/self.dt

            if joint_force>max_force: max_force=joint_force

            if joint_force >= self.breaking_force:
                self.space.remove(joint)
                del self.constrained_bots[index] # The bot is not longer constrained
                del self.sticky_connections[index] # We must delete the reference to this sticky connection
        
        self.max_sticky_force.append([self.time, max_force])



    def create_stick(self, arbiter, collisionPair):
        
        # First check if the bot is already constrained
        botIndex = max(collisionPair)
        if botIndex in self.constrained_bots:
            return None
        
        # Get contact points
        points = arbiter.contact_point_set.points[0]
        pointA, pointB = points.point_a, points.point_b

        # Get the shapes in contact
        shapes = arbiter.shapes
        shapeA, shapeB = shapes[0], shapes[1]

        # Get positions of bodies
        posA = shapeA.body.position
        posB = shapeB.body.position

        # Get relative contact points
        relA = pointA - posA
        relB = pointB - posB

        joint = pymunk.SlideJoint(shapeA.body, shapeB.body,

                                    relA, relB,
                                    min=0, max=self.max_sticky_distance)
        self.space.add(joint)
        self.sticky_connections.append(joint)
        self.constrained_bots.append(botIndex)
        return None


    def reportContact(self, arbiter, collisionPair):
        impulse = arbiter.total_impulse
        botIndex = max(collisionPair)
        
        # Impulse is the amount of force * timestep it was applied. 
        # Thus to get force, we divide the impulse by the timestep
        self.extForcesX[botIndex-2] = impulse[0] / self.dt
        self.extForcesY[botIndex-2] = impulse[1] / self.dt

        return None
    


    
    def colPost(self, arbiter, space, data):

        # Get which objects are in collision
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]

        # Report contacts in proper vectors
        self.reportContact(arbiter, collisionPair)

        if self.use_sticky_connections:
            # Create constraints for touching objects
            self.create_stick(arbiter, collisionPair)

        return True
    
    
    
    
    def isDone(self):
        """
        Use this function to define when the simulation should be completed.

        Return done=True if your conditions are met.
        """
        done=False
        if self.timestep>self.maxNumSteps:
            done=True
            
        return done
    
    
    

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
            # self.drawOptions.flags |= pymunk.SpaceDebugDrawOptions.DRAW_COLLISION_POINTS
            # self.drawOptions.flags |= pymunk.SpaceDebugDrawOptions.DRAW_CONSTRAINTS
            self.drawOptions.shape_outline_color = (0,0,0,255)
            
            # self.background = pygame.image.load("<path/to/image.jpg>")

            pymunk.pygame_util.positive_y_is_up=True
            
            self.render_setup=True
            
        for event in pygame.event.get():
            if event.type==pygame.QUIT: 
                pygame.display.quit()
                self.close()
            if event.type==pygame.KEYDOWN and event.key == pygame.K_ESCAPE: 
                pygame.display.quit()
                self.close()
                

        self.screen.fill((192,192,192))

        # Add background image
        self.screen.blit(self.background, (0, 0))

        self.space.debug_draw(self.drawOptions)

        # Drawing force vectors
        if self.draw_forces:
            for coord in self.force_vectors:
                pygame.draw.line(
                    self.screen,
                    self.force_color,
                    coord[0], coord[1], width=1
                )

        pygame.display.update()
        if self.saveVideo:# and self.timestep%10==0:
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
        rew_temp = [self.time]
        obs_temp = [self.time]
        
        for bot in self.bots:
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
            X_Pos_temp.append(currentBotPos[0])
            Y_Pos_temp.append(currentBotPos[1])
            
            X_vel_temp.append(currentBotVel[0])
            Y_vel_temp.append(currentBotVel[1])
            
            
        for action in ac.flatten():
            action_temp.append(action)
        
            
        for observation in obs.flatten():
            obs_temp.append(observation)
            
        # Convert to Numpy Arrays
        X_Pos_temp = np.asarray(X_Pos_temp)
        Y_Pos_temp = np.asarray(Y_Pos_temp)
        
        X_vel_temp = np.asarray(X_vel_temp)
        Y_vel_temp = np.asarray(Y_vel_temp)
        
        action_temp = np.asarray(action_temp)
        rew_temp = np.asarray(rew_temp)
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
            
            
    def dataExport(self, custom_data=None, custom_name=None):
        
        print('\nExporting and plotting data...',end='')

        # Delete the temporarily made first row of zeroes
        self.X_data = np.delete(self.X_data, 0, 0)
        self.X_vel_data = np.delete(self.X_vel_data, 0, 0)
        self.Y_data = np.delete(self.Y_data, 0, 0)
        self.Y_vel_data = np.delete(self.Y_vel_data, 0, 0)
        self.ac = np.delete(self.ac, 0, 0)
        self.reward_data = np.delete(self.reward_data, 0, 0)
        self.obs_data = np.delete(self.obs_data, 0, 0)
        

        # Save the data on .csv files
        #np.savetxt(self.saveFolder + 'X_data.csv', self.X_data, delimiter=',')
        #np.savetxt(self.saveFolder + 'X_vel_data.csv', self.X_vel_data, delimiter=',')
        #np.savetxt(self.saveFolder + 'Y_data.csv', self.Y_data, delimiter=',')
        #np.savetxt(self.saveFolder + 'Y_vel_data.csv', self.Y_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'actions.csv', self.ac, delimiter=',')
        np.savetxt(self.saveFolder + 'observations.csv', self.obs_data, delimiter=',')

        if not isinstance(custom_data, type(None)) and not isinstance(custom_name, type(None)):
            np.savetxt(self.saveFolder + custom_name + '.csv', custom_data, delimiter=',')
        
        self.plot_data(custom_data=custom_data, custom_name=custom_name)
        self.parameterExport(self.saveFolder)
        
        print('Data Export and Plot Complete')


    def plot_data(self, custom_data=None, custom_name=None):
        # Common Among Position and Velocity Data
        last_col = len(self.X_data[0])-1
        time = self.X_data[:,0]
        xlabel = 'Time [sec]'
        
        # # Plot X-Position
        # X_COM = []
        # for row in self.X_data:
        #     if self.numBots > 1:
        #         pos = np.mean(row[1:last_col])
        #     else:
        #         pos = row[1:]
        #     X_COM.append(pos)
        # plt.figure('X-Pos')
        # plt.plot(time,X_COM)
        # plt.xlabel(xlabel)
        # plt.ylabel('X-Position [m]')
        # plt.title('X-Center Position')
        # plt.savefig(self.saveFolder + 'X-Center Position.jpg')
        #
        # # Plot Y-Position
        # Y_COM = []
        # for row in self.Y_data:
        #     if self.numBots > 1:
        #         pos = np.mean(row[1:last_col])
        #     else:
        #         pos = row[1:]
        #     Y_COM.append(pos)
        # plt.figure('Y-Pos')
        # plt.plot(time,Y_COM)
        # plt.xlabel(xlabel)
        # plt.ylabel('Y-Position [m]')
        # plt.title('Y-Center Position')
        # plt.savefig(self.saveFolder + 'Y-Center Position.jpg')
        #
        # # Plot X-velocity
        # plt.figure('X-Vel')
        # for i in range(self.numBots):
        #     plt.plot(time, self.X_vel_data[:,i+1], label = 'Bot' + str(i+1))
        # plt.xlabel(xlabel)
        # plt.ylabel('X Velocity [m/s]')
        # plt.title('X-Velocity')
        # plt.legend(loc='lower right')
        # plt.savefig(self.saveFolder + 'X-Velocity.jpg')
        #
        # # Plot Y-Velocity
        # plt.figure('Y-Vel')
        # for i in range(self.numBots):
        #     plt.plot(time, self.Y_vel_data[:,i+1], label = 'Bot' + str(i+1))
        # plt.xlabel(xlabel)
        # plt.ylabel('Y Velocity [m/s]')
        # plt.title('Y-Velocity')
        # plt.legend(loc='lower right')
        # plt.savefig(self.saveFolder + 'Y-Velocity.jpg')
            
        # Plot Actions
        # last_col_2 = len(self.ac[0])-1
        # bot=1
        # for i in range(last_col_2):
        #     if i%2!=0:
        #         plt.figure('Applied Forces Bot ' + str(bot))
        #         plt.plot(time, self.ac[:,i], label='X-Force')
        #         plt.plot(time, self.ac[:,i+1], label='Y-Force')
        #         plt.xlabel(xlabel)
        #         plt.ylabel('Force [N]')
        #         plt.title('Applied Forces on Bot ' + str(bot))
        #         plt.legend(loc='lower right')
        #         plt.savefig(self.saveFolder + 'Bot ' + str(bot) + ' Applied Forces.jpg')
        #         bot+=1

        # Plot Custom
        if not isinstance(custom_data, type(None)) and not isinstance(custom_name, type(None)):
            for i in range(custom_data.shape[1] - 1):
                plt.figure('Cost')
                plt.plot(custom_data[:, 0], custom_data[:, i + 1], label='X-Force')
                plt.xlabel('Time [s]')
                plt.ylabel('Cost')
                plt.title('Cost evolution')
                #plt.legend(loc='lower right')
                plt.savefig(self.saveFolder + custom_name + '.jpg')
            
        
    
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
                 collisionType = 0, color = (0,255,0,255), theta=0, isBot=False, bodyType=0):
        self.body = pymunk.Body(body_type=bodyType)
        self.radius = radius
        self.body.position = position
        self.body.angle = theta
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.mass = mass
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = collisionType
        space.add(self.body, self.shape)
        self.isBot = isBot
        



def connectBalls(space, theta1, theta2, b1, b2, rest_length, spring_stiffness, 
                 spring_damping, maxSeparation, isBot=[False, False],
                 thetaPlus=[0,0]):
    """
    If any(isBot==True), then the True index is a bot and we must account for 
    its rotation when attaching a spring. This extra rotation is accounted for 
    in thetaPlus
    """
    locsTrue = np.where(isBot)[0]
    nBots = locsTrue.size
    if nBots>0:
        """
        At least one of the balls to attach is a bot
        """
        springConstraint = pymunk.DampedSpring(b1.body, b2.body,
                                               (b1.radius*np.sin(thetaPlus[0]), b1.radius*np.cos(thetaPlus[0])),(-b2.radius*np.sin(thetaPlus[1]),-b2.radius*np.cos(thetaPlus[1])),
                                               rest_length, spring_stiffness,
                                               spring_damping)
        slideJoint = pymunk.SlideJoint(b1.body, b2.body,
                                      (b1.radius*np.sin(thetaPlus[0]), b1.radius*np.cos(thetaPlus[0])),(-b2.radius*np.sin(thetaPlus[1]),-b2.radius*np.cos(thetaPlus[1])),
                                      0, maxSeparation)
            
    else: # There are no bots in the sequence
        springConstraint = pymunk.DampedSpring(b1.body, b2.body, 
                                               (0, b1.radius), (0, -b2.radius), 
                                                rest_length, spring_stiffness, 
                                                spring_damping)
        
        slideJoint = pymunk.SlideJoint(b1.body, b2.body, 
                                       (0, b1.radius), (0, -b2.radius), 
                                       0, maxSeparation)
    space.add(springConstraint, slideJoint)
    return springConstraint




def createJamoeba(space, systemCenterLocation, systemRadius, numBots, botMass, 
                  botRadius, skinMass, skinRadius, skinRatio, 
                  botFriction, springK, springB, springRL, maxSeparation, 
                  inRadius, inRadiusRandomness, inRadiusRandomnessPer, inMass, inFriction,
                  percentInteriorRemove = 0, botCollisionIntStart = 2):
    xCenter = systemCenterLocation[0]
    yCenter = systemCenterLocation[1]
    
    collisionType = botCollisionIntStart
    
    bots = []
    interiorParticles = []
    membrane = []
    springs = []
    
    # Get interior particles
    gran_per_ring, in_rings_radius = interiorPattern(systemRadius, inRadius, 
                                                     botRadius, percentInteriorRemove)
    
    #Parameter for skins
    t = (2*np.pi/numBots)/(skinRatio+1)
    
    for i in range(numBots):
        theta = i*2*np.pi/numBots
        x = xCenter + systemRadius*np.cos(theta)
        y = yCenter + systemRadius*np.sin(theta)
        
        thetaAdd = 0
        thetaPlus = theta + thetaAdd
        if i == 0:
            color = (255,255,255,255) # Color the first bot white
        else:
            color = (255,0,0,255)
        bot = Ball(space, (x,y), botRadius, botMass, botFriction, collisionType, color=color, theta=thetaPlus, isBot=True)
        collisionType += 1
        bots.append(bot)
        membrane.append(bot)
        
        # Skin particles
        for j in range(1,skinRatio+1):
            thetaSkin = theta + j*t
            x = xCenter + systemRadius*np.cos(thetaSkin)
            y = yCenter + systemRadius*np.sin(thetaSkin)
            skin = Ball(space, (x,y), skinRadius, skinMass, botFriction, color=(0,0,255,255), theta = thetaSkin)
            membrane.append(skin)

    numBodies = len(membrane)

    # Connect the balls    
    for index, body in enumerate(membrane):
        if index < (numBodies-1):
            spring = connectBalls(space, t*index, t*(index+1), body, membrane[index+1], springRL, springK, springB, maxSeparation)
            springs.append(spring)
        else: 
            # Connect last ball to first
            spring = connectBalls(space, t*index, 0, body, membrane[0], springRL, springK, springB, maxSeparation)
            springs.append(spring)
    
    # Create Interiors
    for index, in_ring in enumerate(gran_per_ring):
        radius = in_rings_radius[index]
        for j in range(in_ring):
            in_theta = j*2*np.pi/in_ring
            x = xCenter + radius*np.cos(in_theta)
            y = yCenter + radius*np.sin(in_theta)

            if inRadiusRandomness == 'uniform':
                r_m = (1-inRadiusRandomnessPer) + np.random.uniform(0, inRadiusRandomnessPer)
            elif inRadiusRandomness == 'binary':
                r_m = np.random.choice([(1-inRadiusRandomnessPer), 1.])
            else:
                r_m = 1.
            c_m = int(100 * ((r_m - (1 - inRadiusRandomnessPer))/(inRadiusRandomnessPer))**2 + (255-100))
            interiorParticle = Ball(space, (x, y), inRadius * r_m, inMass, inFriction, color=(0, c_m, 0, 255))
            interiorParticles.append(interiorParticle)


    extraObjects = [Ball(space, (xCenter + 1.5 * 300, yCenter +0. * 300), 80, 4, .1, color=(0, c_m, 0, 255), bodyType=0)]

    return bots, interiorParticles, extraObjects, membrane, springs



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
    numRemove = int(np.round(numInterior*percentInteriorRemove, 0))
    
    numRings= len(gran_per_ring)
    if numRemove==0: removePerRing=0
    else: removePerRing = int(np.round(numRemove/numRings, 0))
    removed = 0
    for ind, ring in enumerate(gran_per_ring):
        removedNow = int(np.round(ring*percentInteriorRemove, 0))# - removePerRing
        if removedNow <= 0:
            removed += gran_per_ring[ind]
            gran_per_ring[ind]=0
        else:
            gran_per_ring[ind] -= removedNow #removePerRing
            removed += removedNow #removePerRing
        if removed>=numRemove: break
    
    return (gran_per_ring, in_rings_radius)




def createWalls(space, screenHeight, screenWidth, wallThickness, tunnel=False, env= None):
    # All values for this function are assumed to already be in Pixels

    wall_collision_type = 0

    # Bottom Wall
    body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1 = pymunk.Poly.create_box(body1, (screenWidth,wallThickness))
    shape1.body.position = (screenWidth//2,wallThickness//2)
    shape1.collision_type = wall_collision_type
    
    # Top Wall
    body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2 = pymunk.Poly.create_box(body2, (screenWidth,wallThickness))
    shape2.body.position = (screenWidth//2, screenHeight-wallThickness//2)
    shape2.collision_type=wall_collision_type
    
    # Back wall
    body3 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape3 = pymunk.Poly.create_box(body3,(wallThickness, screenHeight))
    shape3.body.position = (wallThickness//2, screenHeight//2)
    shape3.collision_type=wall_collision_type

    # End wall
    body4 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape4 = pymunk.Poly.create_box(body4,(wallThickness, screenHeight))
    shape4.body.position = (screenWidth + wallThickness//2, screenHeight//2)
    shape4.collision_type=wall_collision_type
    
    space.add(shape1,body1,shape2,body2,shape3,body3,shape4,body4)
    
    return None

class squareObject:
    def __init__(self, space, position, mass, length, friction, rotation=0, color = (0,0,0,255)):
        self.length = length
        self.body = pymunk.Body() # Kinematic body
        # self.body = pymunk.Body(body_type=pymunk.Body.STATIC) # Static body
        self.shape = pymunk.Poly.create_box(self.body, (length,length))
        self.body.position=position
        self.body.angle = rotation
        self.shape.friction = friction
        self.shape.mass = mass # Kinematic Body
         # Static Body MASS (i.e. do not add mass to the system)
        self.shape.collision_type = 1

        # Add to space
        space.add(self.body, self.shape)


def createVideo(saveLoc, imgLoc, videoName, imgShape, fps):
    out = cv2.VideoWriter(saveLoc+videoName+'.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
    rmtree(imgLoc)
        

# Function to limit the velocity
class limiter:
    def __init__(self, max_velocity):
        """
        Max Velocity should be in PyMunk units (i.e. pixels)
        """
        self.max_velocity = max_velocity

    def limit_velocity(self, body, gravity, damping, dt):
        pymunk.Body.update_velocity(body, gravity, damping, dt)
        l = body.velocity.length
        if l > self.max_velocity:
            scale = self.max_velocity / l
            body.velocity = body.velocity*scale


def calc_JAMoEBA_Radius(skinRadius, skinRatio, botRadius, numBots):
    """
    Inputs:
        - skinRadius (float): The radius of skin particles on system
        - skinRatio (int): Ratio of number of skin particles per bot
        - botRadius (float): The radius of bot particles on system
        - numBots (int): Number of bots in the system

    Returns:
        - R (float): Radius of the system given parameters
    """
    startDistance = skinRadius # The start distance between bots
    arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
    theta = 2*np.pi/numBots
    R = arcLength/theta #**
    return R

class Convert:
    def __init__(self, conversion_ratio=100):
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

# tmp = "Test_0004"
# createVideo2(tmp + '/', tmp + '_VideoImages/', tmp)
def createVideo2(saveLoc, imgLoc, videoName):
    flnm = glob.glob(imgLoc + '*.jpg')[0]
    img_tmp = cv2.imread(flnm)
    height, width, layers = img_tmp.shape
    size = (width, height)

    print('\nCreating video...')
    #out = cv2.VideoWriter(saveLoc + videoName + '.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, size)
    out = cv2.VideoWriter(saveLoc + videoName + '.mp4', cv2.VideoWriter_fourcc('m','p','4','v'), 66, size)
    for filename in glob.glob(imgLoc + '*.jpg'):
        img = cv2.imread(filename)
        out.write(img)
    out.release()

    #rmtree(imgLoc)
    print('Video Creation Complete')

def createVideo(saveLoc, imgLoc, videoName, imgShape):
    print('\nCreating video...')
    import glob # For creating videos
    import cv2 # For creating videos
    from shutil import rmtree

    #out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    out = cv2.VideoWriter(saveLoc + videoName + '.mp4', cv2.VideoWriter_fourcc('M','P','4','V'), 66, imgShape)
    for file in tqdm(glob.glob(imgLoc+'*.jpg')):
        img = cv2.imread(file)
        out.write(img)
    out.release()
    
    rmtree(imgLoc)
    print('Video Creation Complete')


def save_runtime(saveloc,  file_name, runtime):
    from datetime import datetime, timedelta
    """
    Inputs:
        - saveloc (str): Path (NOT file name!) where the runtime will be stored
        - file_name (str): Name to save the information under
        - runtime (float): Runtime (in seconds) of some program
    Given a runtime (in seconds), 
    will save the amount of time it took to run some program at location 'saveloc'
    """
    sec = timedelta(seconds=runtime)
    d = datetime(1,1,1) + sec

    with open(saveloc + file_name +'.txt','w') as f:
        f.write('DAYS:HOURS:MIN:SEC\n')
        f.write("%d:%d:%d:%d" % (d.day-1, d.hour, d.minute, d.second))
    