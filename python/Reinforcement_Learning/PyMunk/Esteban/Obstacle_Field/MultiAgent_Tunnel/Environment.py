
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
from math import floor
from gym import spaces
from tqdm import tqdm
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
                 energy=False, kineticEnergy = False, 
                 velocityPenalty=False, distanceReward=False, numStepsPerStep=1,
                 slidingFriction=0, velocity_limit=0):
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
                 energy, kineticEnergy, 
                 velocityPenalty, distanceReward, numStepsPerStep,
                 slidingFriction, velocity_limit)
    env = wrappers.OrderEnforcingWrapper(env)

    return env

def raw_env(dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect, experimentName, 
                 saveVideo,
                 energy, kineticEnergy, 
                 velocityPenalty, distanceReward, numStepsPerStep,
                 slidingFriction, velocity_limit):

    env = parallel_env(dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect, experimentName, 
                 saveVideo,
                 energy, kineticEnergy, 
                 velocityPenalty, distanceReward, numStepsPerStep,
                 slidingFriction, velocity_limit)
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
                 energy=False, kineticEnergy = False, 
                 velocityPenalty=False, distanceReward=False, numStepsPerStep=1,
                 slidingFriction=0, velocity_limit=0):
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
        self.forceGain = 3
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        self.energy=energy                                           # If True, then we do care about calculating how much energy our system is expending to complete its mission
        self.numStepsPerStep = numStepsPerStep                       # The number of simulation timesteps to run for each call to 'step' function
        self.slidingFriction = slidingFriction                       # Coefficient of friction for objects sliding on the ground
        
        # Storing previous distances to ensure movement
        self.distance_horizon = 500 # How may time steps we must move by, or episode is terminated!
        self.distance_storage = np.zeros(self.distance_horizon)

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
        self.systemStart = R+botRadius+wallThickness*1.2, self.convert.Pixels2Meters(screenHeight/4)
        self.startDistance = np.linalg.norm(self.systemStart)            
            
        self.kineticEnergy = kineticEnergy # If True, then we are calculating the Kinetic energy of the system
        if self.kineticEnergy:
            self.KE = np.zeros(30) # We will store 30 timesteps worth of information
            
        self.velocityPenalty = velocityPenalty
        if self.velocityPenalty:
            self.velRecent = np.zeros(15) # We will store 30 timesteps worth of information
        
        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius, self.percentInteriorRemove)
        self.numInterior = np.sum(granPerRing)
        
        # Setting up environment API
        num_obs = 6 # Number of data points collected per bot
        self.state_size = self.numBots*num_obs # botRePosition (x,y), botVel (x,y), botExternalContact(x,y)
        self.action_size = self.numBots*2
        force_low, force_high = -1.0, 1.0

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
                                       ['energy',str(energy)],
                                       ['kinetic energy',str(kineticEnergy)],
                                       ['velocity penalty',str(velocityPenalty)],
                                       ['distance reward',str(distanceReward)],
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

        # Add simulated friction if we must
        if self.slidingFriction > 0:
            add_sliding_friction(self.space, self.jamoeba, self.slidingFriction, self.convert)
        
        #### Obstacle field
        obsKwargs = {
                    'systemStart':systemStart,
                    'width':width,
                    'height':height,
                    'totalR':R+botRadius,
                    'space':self.space,
                    'friction':self.botFriction,
                    'wallThickness':wallThickness
                    }
        targetLoc = createObstacleField(**obsKwargs)
        _, firstTurn, secondTurn = createObstacleField(**obsKwargs, info=True)
        self.firstTurn, self.secondTurn = self.convert.Pixels2Meters(np.asarray(firstTurn)), self.convert.Pixels2Meters(np.asarray(secondTurn))

        self.targetLoc = self.convert.Pixels2Meters(np.asarray(targetLoc)) 

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
        systemCenter, distanceToTarget, Vx, Vy = self.systemInfo(velocities=True)
    
        # Get the reward
        rew = self.calcRew(distanceToTarget, Vx, Vy)
        
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
        Takes the name of an agent and returns the observation for that agent
        """
        
        # First we need to get the specific bot we are interested in
        start = agent.find('_')+1
        whichBot = int(agent[start:])
        bot = self.bots[whichBot]

        # Get system information
        systemCenter, _ = self.systemInfo()

        # Get position
        pos_pixel = np.asarray(bot.body.position)
        pos = self.convert.Pixels2Meters(pos_pixel)
        pos[0] -= systemCenter[0]
        pos[1] -= systemCenter[1]

        # Get velocity of bot
        vel_pixels = np.asarray([bot.body.velocity])
        vel = self.convert.Pixels2Meters(vel_pixels)
        vel /= self.maxVelocity

        # Get whether the bot is in contact
        X_force = self.extForcesX[whichBot] / self.forceGain
        Y_force = self.extForcesY[whichBot] / self.forceGain
        extForce = np.abs(np.array([X_force, Y_force]))

        obs = np.concatenate((pos.flatten(), vel.flatten(), extForce.flatten()))
        # obs = pos.flatten()
        return obs




    def systemInfo(self, velocities=False):
        
        runTime = [self.timestep/self.maxNumSteps]
        
        botPos = np.zeros((self.numBots,2))
        botVel = np.zeros((self.numBots,2))
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
            # Define position as relative to the target
            botPos[index,:] = np.array(currentBotPos)
            botVel[index,:] = currentBotVel

        systemCenter = np.mean(botPos, axis=0)
        if velocities:
            Vx, Vy = np.mean(botVel, axis=0)
        
        distanceToTarget = np.linalg.norm(systemCenter) # This value is in metersd
        
        #### Calculating Kinetic Energy of system
        if self.kineticEnergy:
            self.KE = np.roll(self.KE,1)
            KE_now = 0
            for obj in self.jamoeba:
                mass = obj.shape.mass
                speed = self.convert.Pixels2Meters(obj.body.velocity.length)
                KE_now += 0.5*mass*speed**2
            self.KE[0] = KE_now
            
        #### Calculating penalty for not moving (based on velocity)
        if self.velocityPenalty:
            self.velRecent = np.roll(self.velRecent,1)
            velX = np.mean(botVel[:,0])
            velY = np.mean(botVel[:,1])
            self.velRecent[0] = np.linalg.norm([velX,velY])

        if velocities:
            return systemCenter, distanceToTarget, Vx, Vy
        else: 
            return systemCenter, distanceToTarget



    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        # Doing a += so that as multiple contacts occur over the many timesteps, we add them
        self.extForcesX[botIndex-2] = impulse[0] / self.dt
        self.extForcesY[botIndex-2] = impulse[1] / self.dt
    
    
    
    
    
    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True
    
    
    
    
    
    def calcRew(self, distanceToTarget, Vx, Vy):

        rew = 0 # Initiate reward

        # Reward for progress
        # progress = self.previousDistance - distanceToTarget # Relative to the velocity or speed for arriving at target                    
        # if progress > 0:
        #     rew += 1

        # Reward for moving forward and up
        rew = Vx + Vy
        rew *= 10

        # Penalty for taking time
        # rew -= (self.timestep/self.maxNumSteps)

        #### Kinetic Energy Reward
        if self.kineticEnergy and np.mean(self.KE)<1e-2:
            rew-=5
            
        #### Velocity Penalty
            if self.velocityPenalty and np.mean(self.velRecent)<0.2:
                rew -= 5
            
        return rew
    
    


    def isDone(self, rew, systemCenter, distanceToTarget):
        """
        Can return later on an add penalties for taking too long or other actions we do not want
        """
        done=False
        if self.timestep>self.maxNumSteps:
            done=True

        if distanceToTarget < self.R/3:
            done=True
            rew += 10

        # # Let's check if the system is moving. This is a 'stuck' termination
        if self.timestep > self.distance_horizon: 
            change = np.abs(self.distance_storage[0] - self.distance_storage[-1])
            if change < self.R/2: # Needs to move at least this much every 'distance_horizon' time steps
                done=True
                rew -= 10  
            
        return done, rew
    
    

    
    def render(self, arg=None):
        targetLoc = self.convert.Meters2Pixels(self.targetLoc)
        
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
            # Adding a visual for the target
            targetLoc = pymunk.pygame_util.to_pygame(targetLoc, self.screen) # Converting to pygame coordinates
            pygame.draw.circle(self.screen, (255,0,0), targetLoc, radius = 5)

        for event in pygame.event.get():
            if event.type==pygame.QUIT: 
                pygame.display.quit()
                self.close()
            if event.type==pygame.KEYDOWN and event.key == pygame.K_ESCAPE: 
                pygame.display.quit()
                self.close()
        
        
        self.screen.fill((192,192,192))
        self.space.debug_draw(self.drawOptions)
        # Adding a visual for the target
        targetLoc = pymunk.pygame_util.to_pygame(targetLoc, self.screen) # Converting to pygame coordinates
        pygame.draw.circle(self.screen, (255,0,0), targetLoc, radius = 5)

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
    


def createObstacleField(systemStart, width, height, totalR, space, friction, wallThickness=0.5, info=False):
    """
    Inputs
        systemStart: To know where the walls should begin
        width: Width of environment (in pixels)
        height: Height of environment (in pixels)
        totalR: totalRadius of JAMoEBA system. I.e. distance from systemStart to outermost point
        space: pymunk.space the environment is created in
        friction: Friction coefficient for the walls
        wallThickness: How thick the walls should be
        info: Whether to return ONLY information on the turns of the wall. Used for testing.
    """

    ct = 1 # Collision type for all 
    w2 = wallThickness/2
    gapSize = totalR*2
    X,Y = systemStart

    bodies = []
    shapes = []
    
    if info:
        firstTurn, secondTurn = None, None # We will be storing the center locations of turns

    # Wall1-UP -- (First wall)
    x1a, y1a = X-totalR, Y + totalR + w2 # The w2 is needed as a buffer to ensure the system is not started inside the wall.
    x1b, y1b = x1a+width/6, y1a
    body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1 = pymunk.Segment(body1, (x1a,y1a), (x1b,y1b), wallThickness)
    bodies.append(body1)
    shapes.append(shape1)

    #Wall2-UP |
    x2a,y2a = x1b,y1b
    x2b,y2b = x2a, y2a + height/2
    body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2 = pymunk.Segment(body2,(x2a,y2a), (x2b,y2b), wallThickness)
    bodies.append(body2)
    shapes.append(shape2)

    # Wall3-UP -- (We are high, and moving right)
    x3a,y3a = x2b,y2b
    x3b,y3b = x3a + width/6 + gapSize, y3a
    body3 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape3 = pymunk.Segment(body3, (x3a,y3a),(x3b,y3b),wallThickness)
    bodies.append(body3)
    shapes.append(shape3)

    # Wall6-Down -- (First bottom wall)
    x6a,y6a = x1a, Y-totalR-w2
    x6b,y6b = x1b + gapSize, y6a
    body6 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape6 = pymunk.Segment(body6,(x6a,y6a),(x6b,y6b),wallThickness)
    bodies.append(body6)
    shapes.append(shape6)

    #Wall7-Down | (Going up)
    x7a,y7a = x6b, y6b
    x7b,y7b = x7a, y2b-gapSize
    body7 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    shape7 = pymunk.Segment(body7, (x7a,y7a), (x7b,y7b), wallThickness)
    bodies.append(body7)
    shapes.append(shape7)

    # Wall8-Down -- (We are high, and moving right)
    x8a,y8a = x7b,y7b
    # x8b,y8b = x8a + width/6-gapSize, y8a # Use this if ending high
    x8b, y8b = x8a + width/6, y8a
    body8 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape8 = pymunk.Segment(body8, (x8a,y8a), (x8b,y8b), wallThickness)
    bodies.append(body8)
    shapes.append(shape8)

    # Adding friction and collision type to each shape
    for s in shapes:
        s.friction = friction
        s.collision_type = ct

    if not info:
        for b in bodies:
            space.add(b)
        for s in shapes:
            space.add(s)

    center_end = x3b, y3b - totalR

    if info:
        firstTurn  = [x1b, y1b]
        secondTurn = [x7b, y7b]
        return center_end, firstTurn, secondTurn
    else:
        return center_end



def createObstacleField2(systemStart, width, height, totalR, space, friction, wallThickness=0.5):
    """
    Inputs
        systemStart: To know where the walls should begin
        width: Width of environment (in pixels)
        height: Height of environment (in pixels)
        totalR: totalRadius of JAMoEBA system. I.e. distance from systemStart to outermost point (should be the edge of the bots)
        space: pymunk.space the environment is created in
        friction: Friction coefficient for the walls
        wallThickness: How thick the walls should be

    This function creates a tunnel that requires squeezing
    
    """

    ct=1 # Collision type for all 
    w2=wallThickness/2
    D = totalR*2 + w2*2
    gapSize = totalR
    X,Y = systemStart

    bodies = []
    shapes = []

    # Wall0-UP -- (First wall)
    xu0a, yu0a = X-totalR, Y + D/2 # The w2 is needed as a buffer to ensure the system is not started inside the wall.
    xu0b, yu0b = xu0a+totalR*2, yu0a
    body0u = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape0u = pymunk.Segment(body0u, (xu0a,yu0a), (xu0b,yu0b), wallThickness)
    bodies.append(body0u)
    shapes.append(shape0u)

    # Wall 1-UP-- Creating Tunnel
    xu1a, yu1a = xu0b, yu0b
    xu1b, yu1b = xu1a, yu1a - (D-gapSize)*0.5
    body1u = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1u = pymunk.Segment(body1u, (xu1a,yu1a), (xu1b,yu1b), wallThickness)
    bodies.append(body1u)
    shapes.append(shape1u)
    
    # Wall2-UP -- Moving right
    xu2a,yu2a = xu1b,yu1b
    xu2b,yu2b = xu2a+width/6, yu2a
    body2u = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2u = pymunk.Segment(body2u,(xu2a,yu2a), (xu2b,yu2b), wallThickness)
    bodies.append(body2u)
    shapes.append(shape2u)

    #Wall3-UP Going Up
    xu3a,yu3a = xu2b,yu2b
    xu3b,yu3b = xu3a, yu3a + height/2
    body3u = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape3u = pymunk.Segment(body3u,(xu3a,yu3a), (xu3b,yu3b), wallThickness)
    bodies.append(body3u)
    shapes.append(shape3u)

    # Wall4-UP -- (We are high, and moving right)
    xu4a,yu4a = xu3b,yu3b
    xu4b,yu4b = xu4a + width/6 + gapSize, yu4a
    body4u = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape4u = pymunk.Segment(body4u, (xu4a,yu4a),(xu4b,yu4b),wallThickness)
    bodies.append(body4u)
    shapes.append(shape4u)
    
    # Wall0-Down -- (First bottom wall)
    xd0a,yd0a = xu0a, Y-D/2
    xd0b,yd0b = xu0b, yd0a
    body0d = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape0d = pymunk.Segment(body0d,(xd0a,yd0a),(xd0b,yd0b),wallThickness)
    bodies.append(body0d)
    shapes.append(shape0d)

    # Wall1-DOWN Creating tunnel
    xd1a,yd1a = xd0b, yd0b
    xd1b,yd1b = xd1a, yd1a + (D-gapSize)*0.5
    body1d = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1d = pymunk.Segment(body1d,(xd1a,yd1a),(xd1b,yd1b),wallThickness)
    bodies.append(body1d)
    shapes.append(shape1d)

    # Wall2-DOWN Going to the right
    xd2a,yd2a = xd1b, yd1b
    xd2b,yd2b = xu2b + gapSize, yd2a
    body2d = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2d = pymunk.Segment(body2d,(xd2a,yd2a),(xd2b,yd2b),wallThickness)
    bodies.append(body2d)
    shapes.append(shape2d)

    #Wall3-Down | (Going up)
    xd3a,yd3a = xd2b, yd2b
    xd3b,yd3b = xd3a, yu3b-gapSize
    body3d = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    shape3d = pymunk.Segment(body3d, (xd3a,yd3a), (xd3b,yd3b), wallThickness)
    bodies.append(body3d)
    shapes.append(shape3d)

    # Wall4-Down -- (We are high, and moving right)
    xd4a, yd4a = xd3b, yd3b
    xd4b, yd4b = xd4a + width/6, yd4a
    body4d = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape4d = pymunk.Segment(body4d, (xd4a,yd4a), (xd4b,yd4b), wallThickness)
    bodies.append(body4d)
    shapes.append(shape4d)

    # Adding friction for each shape
    for s in shapes:
        s.friction = friction
        s.collision_type = 1

    for b in bodies:
        space.add(b)
    for s in shapes:
        space.add(s)

    center_end = xu4b, yu4b - gapSize/2
    
    return center_end
        
        

                

def connectBalls(space, theta1, theta2, b1, b2, rest_length, spring_stiffness, spring_damping, maxSeparation):
    springConstraint = pymunk.DampedSpring(b1.body, b2.body, 
                                           (-b1.radius*np.sin(theta1), b1.radius*np.cos(theta1)), (b2.radius*np.sin(theta2), -b2.radius*np.cos(theta2)), 
                                            rest_length, spring_stiffness, spring_damping)
    slideJoint = pymunk.SlideJoint(b1.body, b2.body, 
                                   (-b1.radius*np.sin(theta1), b1.radius*np.cos(theta1)), (b2.radius*np.sin(theta2), -b2.radius*np.cos(theta2)), 
                                   0, maxSeparation)
    space.add(springConstraint, slideJoint)
    return None


def add_sliding_friction(space, system, mu, convert):
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
    shape4 = pymunk.Poly.create_box(body4,(wallThickness, screenHeight))
    shape4.body.position = (screenWidth + wallThickness//2, screenHeight//2)
    shape4.collision_type=1

    space.add(shape1,body1,shape2,body2,shape3,body3,shape4,body4)
    
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