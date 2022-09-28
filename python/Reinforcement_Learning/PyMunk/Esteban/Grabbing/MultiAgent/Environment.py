
"""
Use this in the future to add collision detection in the observation space:
    https://stackoverflow.com/questions/50815789/non-colliding-objects-which-has-colliding-pairs-pymunk-pygame
    
This example shows how to draw collisions:
    https://github.com/viblo/pymunk/blob/master/examples/contact_and_no_flipy.py
"""

import pymunk
import pygame
import sys
import numpy as np
from numpy import sin, cos, sqrt, log
from numpy.linalg import norm
from tqdm import tqdm
from math import floor
from gym import spaces
import matplotlib.pyplot as plt
from datetime import datetime
from pettingzoo.utils import wrappers, ParallelEnv, agent_selector, from_parallel
from shutil import rmtree
import glob # For creating videos
import cv2 # For creating videos
from warnings import warn
import os
import pdb

def reg_env(dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect=False, experimentName="NOT NAMED", 
                 saveVideo = False, 
                 numStepsPerStep=1,
                 slidingFriction=0):
    """
    For infromation on how the environment should be created:
    https://www.pettingzoo.ml/environment_creation#example-custom-environment
    """
    env = raw_env(dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect, experimentName, 
                 saveVideo, 
                 numStepsPerStep,
                 slidingFriction)
    env = wrappers.OrderEnforcingWrapper(env)

    return env

def raw_env(dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect, experimentName, 
                 saveVideo,
                 numStepsPerStep,
                 slidingFriction):

    env = parallel_env(dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect, experimentName, 
                 saveVideo,
                 numStepsPerStep,
                 slidingFriction)
    env = from_parallel(env)
    return env

class parallel_env(ParallelEnv):
    
    metadata = {'render.modes':['human'],
                'name':'Stars'}
    
    def __init__(self, dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect=False, experimentName="NOT NAMED", 
                 saveVideo = False,
                 numStepsPerStep=1,
                 slidingFriction=0):
        """
        All units in this function should be in standard physical units:
            Distance: Meters
            Force: N
            Velocity: m/s
            Mass: kg

        Per the requirements on https://www.pettingzoo.ml/environment_creation#example-custom-environment,
        the following attributes must be defined here:
            - possible_agents
            - action_spaces
            - observation_spaces
        These attributes should not be changed after initialization
        """

        # Basic simulation parameters
        self.dt = dt                                                 # Simulation timestep
        self.ppm = ppm                                               # Pixels per Meter
        self.convert = Convert(ppm)                                  # Conversion to be used for all parameters given
        self.height= self.convert.Pixels2Meters(screenHeight)        # Height of the screen, in pixels
        self.width = self.convert.Pixels2Meters(screenWidth)         # Width of the screen, in pixels
        self.maxNumSteps = maxNumSteps                               # Number of steps until simulation terminated
        self.maxVelocity = 10                                        # Arbitrarily set, may need changing later.
        self.forceGain = 2                                           # Gain for force recommendations from neural network
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        self.numStepsPerStep = numStepsPerStep                       # The number of simulation timesteps to run for each call to 'step' function
        self.slidingFriction = slidingFriction                       # Coefficient of friction for objects sliding on the ground

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
        self.inMass = inMass
        self.inFriction = inFriction
        self.percentInteriorRemove = percentInteriorRemove
        
        # Paramaters for wall and space
        self.wallThickness = wallThickness
        
        # Square size
        self.squareLength = self.R * .75

        # Properties for object
        # Object Starting Position
        objectX = self.width - (self.squareLength/2 + wallThickness/2)        # Start on right side of environment
        # objectX = self.width - (self.squareLength/2 + wallThickness/2 + buffer)          # Start on right side of environment with some wiggle room 
        # objectX = wallThickness*3 + 2.1*(R + botRadius*2) # Start on left side of environment
        objectY = self.height/2
        self.objectPosition = np.array([objectX, objectY])

        # Start location of JAMoEBA system
        self.start_options = [
            # [objectX, objectY + R + self.squareLength + botRadius],    # Above the box
            # [objectX, objectY - (R + self.squareLength + botRadius)],  # Below the box
            [objectX - (R + self.squareLength + botRadius), objectY]#,  # Left side of box
            # [objectX + (R + self.squareLength + botRadius), objectY],     # Right side of box
            # [objectX - (R + 2*botRadius), self.height/2] # ???
            # [wallThickness*3 + R + botRadius*2, objectY] # Left side of screen
        ]
        self.num_starts = len(self.start_options)
        whichStart = np.random.randint(0,self.num_starts)
        self.systemStart = self.start_options[whichStart]

        # Position of target, relative to system start
        self.targetLoc = np.array([self.width/2, self.height/2])
        
        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius, self.percentInteriorRemove)
        self.numInterior = np.sum(granPerRing)
        
        # Setting up environment API
        num_obs = 6 # botVel (x,y), botExternalContact(x,y)
        # self.state_size = self.numBots*6 # botRePosition (x,y), botVel (x,y), botExternalContact(x,y)
        self.state_size = self.numBots*num_obs # botVel (x,y), botExternalContact(x,y)
        self.action_size = self.numBots*2
        force_low, force_high = -1.0, 1.0  # This should be between [-1,1] since we plan on multiplying with a force gain, anyways

        self.possible_agents = ['Bot_'+str(r) for r in range(numBots)]
        
        self.action_spaces = {agent: spaces.Box(
            low = force_low,
            high = force_high,
            shape=(2,),
            dtype=np.float32
            ) for agent in self.possible_agents}

        low  = np.full(num_obs, -10)
        high = np.full(num_obs, 10)
        self.observation_spaces = {agent: spaces.Box(low,high,dtype=np.float32) for agent in self.possible_agents}

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
                                       ['botFriction',str(self.botFriction)],
                                       ['inMass',str(self.inMass)],
                                       ['inFriction',str(self.inFriction)],
                                       ['percentInteriorRemoved',str(self.percentInteriorRemove)],
                                       ['SpringK:', str(self.springK)], 
                                       ['SpringB:', str(self.springB)], 
                                       ['SpringRL:', str(self.springRL)],
                                       ['maxSeparation',str(self.maxSeparation)],
                                       ['slidingFriction', str(self.slidingFriction)],
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
            self.ac = np.zeros(self.action_size + 1)
            self.reward_data = np.zeros(2)
            self.obs_data = np.zeros(self.state_size +1)
            
        if self.saveVideo:
            self.videoFolder = experimentName + '_VideoImages/'
            os.makedirs(self.videoFolder,exist_ok=True)
        
        return None
        
        
        
        
        
    def reset(self):
        """
        Reset needs to initailize the following attributes:
            - agents
            - rewards
            - _culmulative_rewards
            - dones
            - infos
            - agent_selection

        Look here for more information: https://www.pettingzoo.ml/environment_creation#example-custom-environment
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

        # Square properties
        length = self.convert.Meters2Pixels(self.squareLength)
        objectPosition = self.convert.Meters2Pixels(self.objectPosition)

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
            'inMass':self.inMass,
            'inFriction':self.inFriction,
            'springK':springK,
            'springB':self.springB,
            'springRL':springRL,
            'maxSeparation':maxSeparation,
            'percentInteriorRemove':self.percentInteriorRemove,
            'botCollisionIntStart': 2} # All obstacles will be of collision_type=1
        self.bots, membrane, interiorParticles = createJamoeba(**kwargs)
        
        # Note: Membrane consists of bots and skin particles
        self.jamoeba = [*membrane, *interiorParticles] # This is all bodies within the space! 
        
        #### Add object to grab
        objectMass = 0.5*self.botMass*self.numBots + self.skinMass*(self.skinRatio*self.numBots) + self.inMass*self.numInterior
        objectKwargs = dict(
            space=self.space,
            position=tuple(objectPosition),
            length=length,
            friction=self.botFriction,
            mass=objectMass,
            rotation=0,
            static=True
        )
        self.square = squareObject(**objectKwargs)    

        # Add simulated friction if we must
        if not self.square.static:
            self.all_objects = [self.square, *self.jamoeba]
        else:
            self.all_objects = [*self.jamoeba]

        if self.slidingFriction > 0:
            add_sliding_friction(self.space, self.all_objects, self.slidingFriction)

        # Creating the pot field
        pot_field = create_pot_field(self)
        self.control = controller(pot_field)

        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness,
                    'env':self}
        createWalls(**wallKwargs) # Not creating the walls right now.

        #### Collision Handler
        # Reports collisions with walls, objects, and obstacles
        for bot in self.bots:
            cHandler = self.space.add_collision_handler(1,bot.shape.collision_type)
            cHandler.post_solve = self.colPost
        
        #### Add target visual
        target = pymunk.Body(body_type = pymunk.Body.STATIC)
        targetRad = self.convert.Meters2Pixels(self.R*0.1)
        target.position = self.convert.Meters2Pixels(self.targetLoc[0]), self.convert.Meters2Pixels(self.targetLoc[1])
        targetShape = pymunk.Circle(target, targetRad)
        targetShape.color = (255, 0, 0, 255)
        targetShape.filter = pymunk.ShapeFilter(group=0)
        self.space.add(target, targetShape)
        
        # Creating required attributes as defined by PettingZoo
        self.agents = self.possible_agents
        observations = {agent: self.observation(agent) for agent in self.agents}

        # Getting initial system info for reward processing
        _, self.previousDistance = self.systemInfo()

        return observations
    
    
        
    def step(self, actions):
        """
        step(action) takes in an action for each agent and should return
            - observations
            - rewards
            - dones
            - infos
        Where each of the above is a dict that looks like {agent_1: item_1, agent_2: item_2}
        """
        # If a user passes in actions with  no agents, then just return empty observations, etc.
        if not actions:
            self.agents = []
            return {}, {}, {}, {}

        # Resetting contact information for storing
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
        
        # Gathering actions to be taken
        forcesX = np.zeros(self.numBots)
        forcesY = np.zeros(self.numBots)
        for agent in actions.keys():
            start = agent.find('_')+1
            whichBot = int(agent[start:])
            forcesX[whichBot] = actions[agent][0] * self.forceGain
            forcesY[whichBot] = actions[agent][1] * self.forceGain

        # Get the pot field forces as well
        potFx, potFy = self.control.get_action()
        forcesX += potFx*self.forceGain
        forcesY += potFy*self.forceGain

        # Applying the actions in the environment
        for i in range(self.numStepsPerStep):
            for index, bot in enumerate(self.bots):
                xForce = forcesX[index]
                yForce = forcesY[index]
                
                botPos = bot.body.position # We must query the bot position at each timestep to apply the forec at the proper location
                bot.body.apply_force_at_world_point((xForce, yForce), (botPos.x, botPos.y))

        # Taking a step in the environment
            self.space.step(self.dt)
            self.time += self.dt
        
        # Note the important difference between timestep and time!
        self.timestep+=1
        
        # Gather observations of the system
        # Remember, this should be a dictionary!    
        observations = {agent: self.observation(agent) for agent in self.agents}

        # We can still get system level observation
        systemCenter, distanceToTarget = self.systemInfo()
    
        # Get the reward
        rew = self.calcRew()
        
        # Get whether we are done
        isDone, rew = self.isDone(rew, systemCenter, distanceToTarget)
        self.previousDistance = distanceToTarget

        # Put the information in their dictionaries
        rewards = {agent: rew for agent in self.agents}
        dones = {agent: isDone for agent in self.agents}
        infos = {agent: {} for agent in self.agents}

        if self.dataCollect:
            obs = []
            ac = []
            for agent in observations.keys():
                obs.append(observations[agent])
                ac.append(actions[agent])
            obs = np.asarray(obs)
            obs = obs.flatten()
            ac = np.asarray(ac)
            ac = ac.flatten()
            self.dataCollection(ac,rew,obs)
 
        return observations, rewards, dones, infos
    
    
    def observation(self, agent):
        """
        THIS IS THE SHALLOW OBSERVATION SPACE

        Takes the name of an agent and returns the observation for that agent
        """
        
        # First we need to get the specific bot we are interested in
        start = agent.find('_')+1
        whichBot = int(agent[start:])
        bot = self.bots[whichBot]

        # Get system information
        systemCenter, _ = self.systemInfo(rel_center=False)
        systemCenterTarget, _ = self.systemInfo()

        # Get position
        pos_pixel = np.asarray(bot.body.position)
        pos = self.convert.Pixels2Meters(pos_pixel)
        pos[0] -= systemCenter[0]
        pos[1] -= systemCenter[1] 

        # Get velocity of bot
        vel_pixels = np.asarray([bot.body.velocity])
        vel = self.convert.Pixels2Meters(vel_pixels)

        # Get whether the bot is in contact
        X_force = self.extForcesX[whichBot] / self.forceGain
        Y_force = self.extForcesY[whichBot] / self.forceGain
        extForce = np.abs(np.array([X_force, Y_force]))

        obs = np.concatenate((systemCenter.flatten(), vel.flatten(), extForce.flatten()))

        return obs


    # def observation(self, agent):
    #     """
    #     THIS IS THE DENSE OBSERVATION SPACE

    #     Takes the name of an agent and returns the observation for that agent
    #     """
        
    #     # First we need to get the specific bot we are interested in
    #     start = agent.find('_')+1
    #     whichBot = int(agent[start:])
    #     bots = np.roll(self.bots, -whichBot)

    #     bot_pos = np.zeros((self.numBots,2))
    #     bot_vel = np.zeros((self.numBots,2))
    #     for index, bot in enumerate(bots):
    #         # Position
    #         currentBotPos = np.asarray(bot.body.position)
    #         currentBotPos = self.convert.Pixels2Meters(currentBotPos)

    #         # Velocity
    #         currentBotVel = np.asarray(bot.body.velocity)
    #         currentBotVel = self.convert.Pixels2Meters(currentBotVel)

    #         # Adding to list
    #         bot_pos[index,:] = currentBotPos
    #         bot_vel[index,:] = currentBotVel


    #     # Get position
    #     pos_pixel = np.asarray(bot.body.position)
    #     pos = self.convert.Pixels2Meters(pos_pixel)
    #     pos[0] /= self.width
    #     pos[1] /= self.height

    #     # Get velocity of bot
    #     vel_pixels = np.asarray([bot.body.velocity])
    #     vel = self.convert.Pixels2Meters(vel_pixels)

    #     # Get whether the bot is in contact
    #     X_force = self.extForcesX[whichBot] / self.forceGain
    #     Y_force = self.extForcesY[whichBot] / self.forceGain
    #     extForce = np.abs(np.array([X_force, Y_force]))

    #     # obs = np.concatenate((pos.flatten(), vel.flatten(), extForce.flatten()))
    #     obs = np.concatenate((vel.flatten(), extForce.flatten()))

    #     return obs



    def systemInfo(self, rel_center = True):
        
        runTime = [self.timestep/self.maxNumSteps]
        
        botPos = np.zeros((self.numBots,2))
        botVel = np.zeros((self.numBots,2))
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
            # Define position as relative to the target
            if rel_center:
                botPos[index,:] = np.array(self.targetLoc) - np.array(currentBotPos)
            else:
                botPos[index,:] = np.array(currentBotPos)
            botVel[index,:] = currentBotVel

        systemCenter = np.mean(botPos, axis=0)
        
        distanceToTarget = np.linalg.norm(systemCenter) # This value is in metersd
                
        return systemCenter, distanceToTarget

    

    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        # Doing a += so that as multiple contacts occur over the many timesteps, we add them
        forces = impulse / self.dt
        self.extForcesX[botIndex-2] = forces[0]
        self.extForcesY[botIndex-2] = forces[1]
    
    
    
    
    
    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True
    
    
    
    
    
    def calcRew(self):

        rew = 0

        return rew
    
    
    def isDone(self, rew, systemCenter, distanceToTarget):
        """
        Can return later on an add penalties for taking too long or other actions we do not want
        """
        done=False
        if self.timestep>self.maxNumSteps:
            done=True
            
        if systemCenter[0] < self.R*2:
            done = True
            
        if distanceToTarget<1e-1:
            done=True
            rew+=10
          
        return done, rew
    
    
    
    def render(self, arg=None):
        if not self.render_setup:
            from pymunk.pygame_util import DrawOptions
            
            screenHeight = floor(self.convert.Meters2Pixels(self.height))
            screenWidth = floor(self.convert.Meters2Pixels(self.width))
            
            pygame.init()
            self.screen = pygame.display.set_mode((screenWidth, screenHeight))
            self.clock = pygame.time.Clock()
            
            self.drawOptions = DrawOptions(self.screen)
            self.drawOptions.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES
            self.drawOptions.shape_outline_color = (0,0,0,255)
            
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
        self.space.debug_draw(self.drawOptions)
        pygame.display.update()
        if self.saveVideo:# and self.timestep%10==0:
            pygame.image.save(self.screen, self.videoFolder+'image%06d.jpg' % self.timestep)
        self.clock.tick()
    
    def close(self):
        if self.render_setup:
            pygame.display.quit()
        del self.space
            
            
    def dataCollection(self,ac,rew,obs):
        
        # Create new and empty vecotrs
        X_Pos_temp = [self.time]
        Y_Pos_temp = [self.time]
        
        X_vel_temp = [self.time]
        Y_vel_temp = [self.time]
        
        action_temp = [self.time]
        rew_temp = [self.time]
        obs_temp = [self.time]
        
        for bot in self.bots:
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x)-self.targetLoc[0], self.convert.Pixels2Meters(bot.body.position.y)-self.targetLoc[1]
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
            X_Pos_temp.append(currentBotPos[0])
            Y_Pos_temp.append(currentBotPos[1])
            
            X_vel_temp.append(currentBotVel[0])
            Y_vel_temp.append(currentBotVel[1])
            
            
        for action in ac:
            action_temp.append(action)
        
        rew_temp.append(rew)
            
        for observation in obs:
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
        self.reward_data = np.vstack([self.reward_data, rew_temp])
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
        np.savetxt(self.saveFolder + 'X_data.csv', self.X_data, delimiter=',')
        np.savetxt(self.saveFolder + 'X_vel_data.csv', self.X_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_data.csv', self.Y_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_vel_data.csv', self.Y_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'actions.csv', self.ac, delimiter=',')
        np.savetxt(self.saveFolder + 'reward.csv', self.reward_data, delimiter=',')
        np.savetxt(self.saveFolder + 'observations.csv', self.obs_data, delimiter=',')
        
        self.plot_data()
        self.parameterExport(self.saveFolder)
        
        print('Data Export and Plot Complete')


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
            if i%2!=0:
                plt.figure('Applied Forces Bot ' + str(bot))
                plt.plot(time, self.ac[:,i], label='X-Force')
                plt.plot(time, self.ac[:,i+1], label='Y-Force')
                plt.xlabel(xlabel)
                plt.ylabel('Force [N]')
                plt.title('Applied Forces on Bot ' + str(bot))
                plt.legend(loc='lower right')
                plt.savefig(self.saveFolder + 'Bot ' + str(bot) + ' Applied Forces.jpg')
                bot+=1
                
        # Plot reward
        time = self.reward_data[:,0]
        rewards = self.reward_data[:,1]
        plt.figure('Rewards')
        plt.plot(time, rewards)
        plt.xlabel(xlabel)
        plt.ylabel('Reward')
        plt.title('Reward for JAMoEBA')
        plt.savefig(self.saveFolder + 'Reward.jpg')
        
            
        
    
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
    def __init__(self, space, position, radius, mass, friction, collisionType = 0, color = (0,255,0,255)):
        self.body = pymunk.Body()
        self.radius = radius
        self.body.position = position
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.mass = mass
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = collisionType
        space.add(self.body, self.shape)
        
                
class squareObject:
    def __init__(self, space, position, mass, length, friction, rotation=0, color = (0,0,0,255), static=False):
        self.static = static
        if static:
            self.body = pymunk.Body(0,0, body_type=pymunk.Body.STATIC) # Static Body
        else:
            self.body = pymunk.Body() # Kinematic body

        self.length = length
        self.shape = pymunk.Poly.create_box(self.body, (length,length))
        self.shape.body.position = position
        self.shape.body.angle = rotation
        self.shape.friction = friction
        if not static:
            self.shape.mass = mass # Only moving bodies can have a mass
        self.shape.collision_type = 1

        # Add to space
        space.add(self.body, self.shape)


def connectBalls(space, theta1, theta2, b1, b2, rest_length, spring_stiffness, spring_damping, maxSeparation):
    springConstraint = pymunk.DampedSpring(b1.body, b2.body, 
                                           (-b1.radius*np.sin(theta1), b1.radius*np.cos(theta1)), (b2.radius*np.sin(theta2), -b2.radius*np.cos(theta2)), 
                                            rest_length, spring_stiffness, spring_damping)
    slideJoint = pymunk.SlideJoint(b1.body, b2.body, 
                                   (-b1.radius*np.sin(theta1), b1.radius*np.cos(theta1)), (b2.radius*np.sin(theta2), -b2.radius*np.cos(theta2)), 
                                   0, maxSeparation)
    space.add(springConstraint, slideJoint)
    return None

def createJamoeba(space, systemCenterLocation, systemRadius, numBots, botMass, botRadius, skinMass, skinRadius, skinRatio, botFriction, springK, springB, springRL, maxSeparation, inRadius, inMass, inFriction,  percentInteriorRemove = 0, botCollisionIntStart = 2):
    xCenter = systemCenterLocation[0]
    yCenter = systemCenterLocation[1]
    
    collisionType = botCollisionIntStart
    
    bots = []
    interiorParticles = []
    membrane = []
    
    # Get interior particles
    gran_per_ring, in_rings_radius = interiorPattern(systemRadius, inRadius, botRadius, percentInteriorRemove)
    
    #Parameter for skins
    t = (2*np.pi/numBots)/(skinRatio+1)
    
    for i in range(numBots):
        theta = i*2*np.pi/numBots
        x = xCenter + systemRadius*np.cos(theta)
        y = yCenter + systemRadius*np.sin(theta)
        
        bot = Ball(space, (x,y), botRadius, botMass, botFriction, collisionType, color=(255,0,0,255))
        collisionType += 1
        bots.append(bot)
        membrane.append(bot)
        
        # Skin particles
        for j in range(1,skinRatio+1):
            x = xCenter + systemRadius*np.cos(theta + j*t)
            y = yCenter + systemRadius*np.sin(theta + j*t)
            skin = Ball(space, (x,y), skinRadius, skinMass, botFriction, color=(0,0,255,255))
            membrane.append(skin)
            
    numBodies = len(membrane)
    for index, body in enumerate(membrane):
        if index < (numBodies-1):
            connectBalls(space, t*index, t*(index+1), body, membrane[index+1], springRL, springK, springB, maxSeparation)
        else:
            connectBalls(space, t*index, 0, body, membrane[0], springRL, springK, springB, maxSeparation)
    
    # Create Interiors
    for index, in_ring in enumerate(gran_per_ring):
        radius = in_rings_radius[index]
        for j in range(in_ring):
            in_theta = j*2*np.pi/in_ring
            x = xCenter + radius*np.cos(in_theta)
            y = yCenter + radius*np.sin(in_theta)
            
            interiorParticle = Ball(space, (x,y), inRadius, inMass, inFriction)
            interiorParticles.append(interiorParticle)
            
    return bots, membrane, interiorParticles


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
    else: removePerRing = numRemove//numRings
    removed = 0
    for ind, ring in enumerate(gran_per_ring):
        removedNow = ring - removePerRing
        if removedNow<0:
            removed += gran_per_ring[ind]
            gran_per_ring[ind]=0
        else:
            gran_per_ring[ind] -= removePerRing
            removed += removePerRing
        if removed>=numRemove: break
    
    return (gran_per_ring, in_rings_radius)


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

        gear = pymunk.GearJoint(static_body, body, 0.0, 1.0)
        space.add(gear)
        gear.max_bias = 0 # Disable joint correctioon
        gear.max_force = friction_force


def createWalls(space, screenHeight, screenWidth, wallThickness, tunnel=False, env= None):
    # All values for this function are assumed to already be in Pixels
    
    # Bottom Wall
    body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1 = pymunk.Poly.create_box(body1, (screenWidth,wallThickness))
    shape1.body.position = (screenWidth//2,wallThickness//2)
    shape1.collision_type = 1
    
    # Top Wall
    body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2 = pymunk.Poly.create_box(body2, (screenWidth,wallThickness))
    shape2.body.position = (screenWidth//2, screenHeight-wallThickness//2)
    shape2.collision_type=1
    
    # Back wall
    body3 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape3 = pymunk.Poly.create_box(body3,(wallThickness, screenHeight))
    shape3.body.position = (wallThickness//2, screenHeight//2)
    shape3.collision_type=1
    
    # Front wall
    body4 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape4 = pymunk.Poly.create_box(body4,(wallThickness,screenHeight))
    shape4.body.position = (screenWidth + wallThickness//2, screenHeight//2)
    shape4.collision_type = 1

    space.add(shape1,body1,shape2,body2,shape3,body3,body4,shape4)
    
    return None


def createVideo(saveLoc, imgLoc, videoName, imgShape):
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
    rmtree(imgLoc)
        
    

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



"""
Potential field functions
"""
class controller:
    def __init__(self, pot_field_controller, noise=False):
        """
        Pot field controller
        """
        self.noise = noise
        if self.noise:
            self.gauss = np.random.normal # For adding noise to the actuation
            self.scale = 2 # Standard deviation from Gaussian distribution to sample noise from
        self.pot_field_controller = pot_field_controller

    def get_action(self, _obs=None, return_action=False):
        """
        Given the observation from the environment, return the recommended action to take per the pot field.
        We don't need the observation, thus it is set as _obs.

        return_action=True: Will return the forces on each bot as a single vector (i.e. [Fx1, Fy1, Fx2, Fy2,...,Fyn])
        return_action=False: Will return separate vectors for the forces in the X and Y direction (i.e. Fx, Fy)
        """
        Fx, Fy = self.pot_field_controller.run_controller()
        action = [] 
        for i in range(Fx.size):
            action.append(Fx[i])
            action.append(Fy[i])
        # action = self.correct_force(Fx, Fy) # Correct the action to fit into the proper space if required.
        # Adding noise to the action
        if self.noise:
            action = self.add_noise(action)
        
        if return_action:
            return action
        else:
            return Fx, Fy

    def add_noise(self, action):
        """
        Given the action vector, noise will be added
        """

        action = np.asarray(action) # Verify that action is an array
        action += self.gauss(size=action.size, scale=self.scale)
        action = np.clip(action,-1,1)
        action = action.tolist()

        return action

    def correct_force(self, Fx, Fy):
        """
        The pot field contoller returns two vectors defining the force in x and y direction for each bot.
        This must be adjusted to b in the discrete format that the environment is currently setup in.
        This can be removed if the agent is allowed to apply continuous actions on the space.

        Inputs:
            - Fx (ndarray): The forces in X-direction for each bot, as calculated by the pot field
            - Fy (ndarray): The force in Y-direction for each bot, as calculated by the pot field

        Returns:
            - action (ndarray): The recommended action to take, passed onto env.step() function
        """

        n = len(Fx) # Get the number of bots

        """
        Discrete Action Mapping:
            0: -1N
            1:  0N
            2:  1N
        """
        action = []
        thresh = 0.3 # Threshold for when a force should be applied versus not

        # Iterate through the forces for each bot
        for i in range(n):
            
            # Force in X-Direction
            force_x = Fx[i]
            
            if np.abs(force_x) < thresh: # Too small of a force, don't apply any forces
                action.append(1)
            elif force_x > 0:          # Move forward
                action.append(2)
            elif force_x < 0:          # Move backward
                action.append(0)
            
            # Force in Y-Direction
            force_y = Fy[i]

            if np.abs(force_y) < thresh:
                action.append(1)
            elif force_y > 0:
                action.append(2)
            elif force_y < 0:
                action.append(0)

        return np.asarray(action)


def create_pot_field(env):
    """
    Takes an instance of PyMunk environment and returns a pot field controller
    
    This function is used to get the pot controller given environment paramerters
    It is setup as a function because we need the environment to be created before we can create the pot field,
    and that happens in the calling file.
    """
    fieldLen = np.max([env.width, env.height])
    potFieldParams={
        'a':env.convert.Meters2Pixels(env.squareLength*.25),
        'b':env.convert.Meters2Pixels(env.squareLength*.25),
        'px':env.convert.Meters2Pixels(env.objectPosition[0]),
        'py':env.convert.Meters2Pixels(env.objectPosition[1]),
        'theta':0,
        'fieldLen':env.convert.Meters2Pixels(fieldLen),
        'res':env.convert.Meters2Pixels(0.1)}

    env.phi = analyticField(**potFieldParams)

    PWM = 255 # must be a number 0 < PWM < 255
    w = 1     # Frequency in hertz of PWM
    tn = (PWM/255)/w
    potControlParams={'environment':env,
                        'w':w,
                        'tn':tn,
                        'phi':env.phi,  # The field itself
                        'alpha':1,      # Gain of potfield. THIS IS THE FORCE IT WILL APPLY TO EACH BOT
                        'beta':0}       # Damping term of potfield

    pot_field = PotControl(**potControlParams)

    return pot_field

class PotControl:
    def __init__(self, environment, w, tn, phi, alpha, beta):

        #### Initialize some parameters for all cases of the controller
        self.env = environment      # The environment above (PyMunk)
        self.w=w                    # frequency for PWM
        self.tn=tn                  # time interval for PWM
        self.T=0                    # internal time for PWM
        
        #### Parameters needed from environment
        self.numBots = self.env.numBots  # Number of Bots
        self.tstep = self.env.dt         # Environment Timestep
                   
        ##### POT_FIELD_GRAB PARAMETERS        
        self.phi=phi          # potential field object
        self.fnx=self.phi.df2x # gradient in x direction  
        self.fny=self.phi.df2y # gradient in the y direction      
        self.alpha=alpha      # proportional gain 
        self.beta=beta        # damping term               
        

    # run controller
    def run_controller(self):
        self.get_position()            # get the position of the bots
        # self.get_velocity()            # get the velocity of the bots
        self.xbv = self.ybv = np.zeros(self.numBots)
        FX, FZ=self.grab_controller() # run the grab controller
        FX, FZ = self.apply_force(FX,FZ)        # Gather whether we will apply force to bots, based on PWM
        return FX, FZ
        

    #  Grabbing a ball controller 
    def grab_controller(self):
        ''' This controller is used for grabbing a fixed object'''
        FX=[]
        FY=[]
        px = self.env.square.body.position.x
        py = self.env.square.body.position.y
        for i in range(self.numBots):
            Fy=self.fny(self.xb[i], self.yb[i], px, py, 0 )
            Fx=self.fnx(self.xb[i], self.yb[i], px, py, 0 )
            mag=np.linalg.norm([Fx,Fy])
            FXX=Fx/mag
            FYY=Fy/mag
            fy=-self.alpha*FYY-self.beta*self.ybv[i]
            fx=-self.alpha*FXX-self.beta*self.xbv[i]
            FX.append(fx)
            FY.append(fy)
        return(np.asarray(FX),np.asarray(FY))
                   

    def apply_force(self, FX, FZ):  
        self.T=self.tstep+self.T
        self.T = round(self.T,4)
        if self.T>0 and self.T<=self.tn:
            FX=FX
            FZ=FZ
        else:
            pass # Currently passing to avoid setting force to 0
            # FX=np.zeros(len(FX))
            # FZ=np.zeros(len(FX))
            
        if self.T>(1/self.w):
            self.T=0

        return FX, FZ
        
    # get current position         
    def get_position(self):
        self.xb=[]        
        self.yb=[]
        for i in range(self.numBots):
            self.xb.append(self.env.bots[i].body.position.x)
            self.yb.append(self.env.bots[i].body.position.y)
        

    # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.ybv=[]
        for i in range(self.numBots):
            self.xbv.append(self.env.bots[i].body.velocity.x)
            self.ybv.append(self.env.bots[i].body.velocity.y)



class analyticField:
    def __init__(self, a, b, px, py, theta = 0, res = 0, fieldLen=0):
        """
        a, b: Radii in two directions
        px, py: Center location of pot field
        theta: Rotation of field (about x-axis)
        res: Resolution of field
        fieldLen: The length of the field. i.e. the amount in each Cartesian direction that the field will be calculated for.
        """
        self.a = a
        self.b = b
        self.fieldLen = fieldLen
        self.px = px
        self.py = py
        self.theta = theta
        self.res = res
        
    def d_xy(self, x, y, px, py, a, b, phi=0):
        first  = (((-px+x)*sin(phi)+(-py+y)*cos(phi))**2)/(b**2)
        second = (((-px+x)*cos(phi)-(-py+y)*sin(phi))**2)/(a**2)
        tot = sqrt(first+second)
        return tot

    def d_dx(self, x, y, px, py, a, b, phi=0):
        denom1 = (((-px+x)*sin(phi)+(-py+y)*cos(phi))**2)/(b**2)
        denom2 =  (((-px+x)*cos(phi)-(-py+y)*sin(phi))**2)/(a**2)
        denom = sqrt(denom1 + denom2)
        
        num1 = ((-px+x)*sin(phi)+(-py+y)*cos(phi))*sin(phi)/(b**2)
        num2 = ((-px+x)*cos(phi)-(-py+y)*sin(phi))*cos(phi)/(a**2)
        num = num1+num2
        return (num/denom)
    
    def d_dy(self, x, y, px, py, a, b, phi=0):
        denom1 = (((-px+x)*sin(phi)+(-py+y)*cos(phi))**2)/(b**2)
        denom2 =  ((-px+x)*cos(phi)-(-py+y)*sin(phi))**2/(a**2)
        denom = sqrt(denom1 + denom2)
        
        num1 = ((-px+x)*sin(phi)+(-py+y)*cos(phi))*cos(phi)/(b**2)
        num2 = ((-px+x)*cos(phi)-(-py+y)*sin(phi))*sin(phi)/(a**2)
        num = num1-num2
        return (num/denom)
    
    def df2x(self, x, y, px, py, phi=0):
        kwargs = {'x':x, 'y':y, 'px':px, 'py':py, 'a':self.a, 'b':self.b, 'phi':phi}
        result = 4*self.d_xy(**kwargs)**3*(log(self.d_xy(**kwargs)))**2*self.d_dx(**kwargs) + 2*self.d_xy(**kwargs)**3*log(self.d_xy(**kwargs))*self.d_dx(**kwargs)
        return result
    
    def df2y(self, x, y, px, py, phi=0):
        kwargs = {'x':x, 'y':y, 'px':px, 'py':py, 'a':self.a, 'b':self.b, 'phi':phi}
        result = 4*self.d_xy(**kwargs)**3*(log(self.d_xy(**kwargs)))**2*self.d_dy(**kwargs) + 2*self.d_xy(**kwargs)**3*log(self.d_xy(**kwargs))*self.d_dy(**kwargs)
        return result
    
    def f_xy(self, x, y, px, py, a, b, phi=0):
        kwargs = {'x':x, 'y':y, 'px':px, 'py':py, 'a':a, 'b':b, 'phi':phi}
        result = self.d_xy(**kwargs)**2 * log(self.d_xy(**kwargs))
        return result
    
    def getField(self):
        # Calculating some parameters:
        self.xmin = self.px - self.fieldLen
        self.xmax = self.px + self.fieldLen
        self.ymin = self.py - self.fieldLen
        self.ymax = self.py + self.fieldLen
        xcount = int(round((self.xmax-self.xmin)/self.res))
        ycount = int(round((self.ymax-self.ymin)/self.res))
        xp = np.linspace(self.xmin,self.xmax,xcount)
        yp = np.linspace(self.ymin,self.ymax,ycount)
        
        xx, yy = np.meshgrid( xp, yp )
        kwargs={'x':xx, 'y':yy, 'px':self.px, 'py':self.py, 'a':self.a, 'b':self.b, 'theta':self.theta}
        zz = self.f_xy(**kwargs)
        fy, fx = np.gradient(zz**2)
        fx /= np.max(fx)
        fy /= np.max(fy)
        self.fnx = self.df2x
        self.fny = self.df2y
        return None



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




def createVideo(saveLoc, imgLoc, videoName, imgShape):
    print('\nCreating video...')
    import glob # For creating videos
    import cv2 # For creating videos
    from shutil import rmtree

    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    for file in tqdm(glob.glob(imgLoc+'*.jpg')):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
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
    