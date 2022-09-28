
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
                 energy=False, kineticEnergy = False, 
                 velocityPenalty=False, distanceReward=False, numStepsPerStep=1,
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
                 energy, kineticEnergy, 
                 velocityPenalty, distanceReward, numStepsPerStep,
                 slidingFriction)
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
                 slidingFriction):

    env = parallel_env(dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, 
                 inRadius, botFriction, inMass, inFriction, percentInteriorRemove, 
                 springK, springB, springRL, wallThickness, maxSeparation, 
                 dataCollect, experimentName, 
                 saveVideo,
                 energy, kineticEnergy, 
                 velocityPenalty, distanceReward, numStepsPerStep,
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
                 energy=False, kineticEnergy = False, 
                 velocityPenalty=False, distanceReward=False, numStepsPerStep=1,
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

        self.report_all_data = False # SHOULD BE FALSE DURING TRAINING

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
        self.energy=energy                                           # If True, then we do care about calculating how much energy our system is expending to complete its mission
        self.numStepsPerStep = numStepsPerStep                       # The number of simulation timesteps to run for each call to 'step' function
        self.slidingFriction = slidingFriction                       # Coefficient of friction for objects sliding on the ground
        
        # Storing previous distances to ensure movement
        self.stuck_termination = False
        if self.stuck_termination:
            self.distance_horizon = 150 # How may time steps we must move by, or episode is terminated!
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
        
        # Parameters for action smoothing
        self.smooth_method = 'linear' # This should be one of the following: [None, 'linear', 'exponential']
        omega = np.sqrt(springK/botMass)
        # self.decay_rate = 1/(omega*dt*numStepsPerStep) # This should be a float value greater than or equal to 0. 
        self.decay_rate = 1/(numBots/2)
        if self.smooth_method is not None:
            self.weights = self.calc_weights()
            self.loc_1 = np.where(self.weights==1)[0][0] # Get the 'center' position of the weights. This will become necessary when rolling the weights

        # Paramaters for wall and space
        self.wallThickness = wallThickness
        self.systemStart = R+botRadius+wallThickness*1.2, self.convert.Pixels2Meters(screenHeight/2)
        self.startDistance = np.linalg.norm(self.systemStart)
        
        # Position of target, relative to system start
        self.targetLoc = self.systemStart[0]+R*6, self.systemStart[1] # Located directly down the x-axis
        self.targetLoc = np.asarray(self.targetLoc)

        self.kineticEnergy = kineticEnergy # If True, then we are calculating the Kinetic energy of the system
        if self.kineticEnergy:
            self.KE = np.zeros(30) # We will store 30 timesteps worth of information
            
        self.velocityPenalty = velocityPenalty
        if self.velocityPenalty:
            self.velRecent = np.zeros(30) # We will store 30 timesteps worth of information
        
        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius, self.percentInteriorRemove)
        self.numInterior = np.sum(granPerRing)
        
        # Setting up environment API
        num_obs = 7 # botVel (x,y), botExternalContact(x,y)
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
        self.last_rew = 0
        self.smooth_weight = 0.9
        
        # Information for contact, will be changed later
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
        self.botContacts = np.zeros(self.numBots)


        # Further data collection
        if self.report_all_data:
            self.botPositions = []
            self.skinPositions = []
            self.interiorPositions = []

        
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
        targetLoc = self.convert.Meters2Pixels(self.targetLoc)
        
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
        self.bots, self.skinParticles, self.interiorParticles = createJamoeba(**kwargs)
        
        # Note: Membrane consists of bots and skin particles
        self.jamoeba = [*self.bots, *self.skinParticles, *self.interiorParticles] # This is all bodies within the space! 
        
        # Add simulated friction if we must
        if self.slidingFriction > 0:
            add_sliding_friction(self.space, self.jamoeba, self.slidingFriction)

        #### Obstacle field
        gapWidth = R*.75
        gapCenterLoc = targetLoc[0]-gapWidth/2, targetLoc[1]
        obsKwargs = dict(
            space=self.space,
            systemStart = systemStart,
            R=R,
            thickness=wallThickness,
            gapCenterLoc=gapCenterLoc,
            gapHeight=gapWidth,
            gapWidth=R,
            height=height,
            report_points=self.report_all_data
        )

        if self.report_all_data:
            obs_points = createObstacleField(**obsKwargs)
            self.obs_points = self.convert.Pixels2Meters(obs_points)
        else:
            createObstacleField(**obsKwargs)



        # Create walls to surround the environment
        wallKwargs = dict(
            space=self.space,
            screenHeight=height,
            screenWidth=width,
            wallThickness=wallThickness
        )
        createWalls(**wallKwargs)

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
        if self.stuck_termination:
            self.distance_storage[0] = self.previousDistance

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
        self.botContacts = np.zeros(self.numBots)
        
        # Gathering actions to be taken
        forcesX = np.zeros(self.numBots)
        forcesY = np.zeros(self.numBots)

        # Smooth the actions
        actions = self.smooth_actions(actions)

        # This is done just in case the backend doesn't maintain the order of the robots
        for agent in actions.keys():
            start = agent.find('_')+1
            whichBot = int(agent[start:])
            forcesX[whichBot] = self.convert.Meters2Pixels(actions[agent][0] * self.forceGain)
            forcesY[whichBot] = self.convert.Meters2Pixels(actions[agent][1] * self.forceGain)

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
        if self.stuck_termination:
            self.distance_storage = np.roll(self.distance_storage, 1)
            self.distance_storage[0] = distanceToTarget
    
        # Get the reward
        rew = self.calcRew(distanceToTarget)
        
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

        if self.report_all_data:
            self.collectAllData()
 
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
        # X_force = self.extForcesX[whichBot] / self.forceGain
        # Y_force = self.extForcesY[whichBot] / self.forceGain
        # extForce = np.abs(np.array([X_force, Y_force]))
        botContact = self.botContacts[whichBot]

        obs = np.concatenate((systemCenterTarget.flatten(), pos.flatten(), vel.flatten(), [botContact]))

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
                
        return systemCenter, distanceToTarget



    def calc_weights(self):
        """
        Calculate the weights for the rolling average 
        """
        # No smoothing of actios
        if self.smooth_method is None:
            return None

        # If we are here, then we are going to calculate weights
        # Get length of vector
        if self.numBots%2==0: # Even number of robots
            n = int(self.numBots/2)
        else: # Odd number of robots
            n = int((self.numBots+1)/2)
        
        weights = np.zeros(n)

        allowed_methods = ['linear','exponential']
        assert(self.smooth_method in allowed_methods), "The smoothing method specified is not available!"

        # Linear smoothing
        if self.smooth_method=='linear':
            # Iterate through weights and apply linear decay
            for i, _ in enumerate(weights): 
                weights[i] = -self.decay_rate*i+1 # Linear decay at decay rate

        # Exponential smoothing
        elif self.smooth_method=='exponential':
            for i, _ in enumerate(weights):
                weights[i] = (1-self.decay_rate)**i

        # Ensure there are no negative weights
        weights = weights.clip(min=0)
        
        # Create the entire weight vector
        Weights = np.zeros(self.numBots)
        Weights[:n] = np.flip(weights)
        if self.numBots%2==0: # Even number of bots
            Weights[n:-1] = weights[1:]
            Weights[-1] = weights[-1]
        else: # Odd number of robots
            Weights[n:] = weights[1:]
        return Weights



    def smooth_actions(self, actions):
        """
        Will take all the actions as a dictionary and will smooth them appropriately.

        action (dict):
            Dictionary of actions taken for each robot

        smooth_method (str or None): 
            If None, returns the actions passed through. 
            If 'linear' uses a linear decay for the weighted average.
            If 'exponential' uses an exponential decay for the weighted average.

        decay (float): A decay rate fo the decay method on smoothing
        """

        # No smoothing of the actions
        if self.smooth_method is None:
            return actions
        
        # Use the previously calculated weights to smooth actions
        else:
            # First get the order of the robots and the forces applied
            order = np.zeros(self.numBots, dtype=int)
            Fx = np.zeros(self.numBots)
            Fy = np.zeros(self.numBots)
            for i, agent in enumerate(actions.keys()):
                # Order
                start = agent.find('_')+1
                whichBot = int(agent[start:])
                order[i] = whichBot
                
                # Forces
                forces = actions[agent]
                Fx[i], Fy[i] = forces[0], forces[1]

            # Reorder, if necessray.
            o = np.argsort(order)
            order = order[o]
            Fx = Fx[o]
            Fy = Fy[o]

            # Create smoothed action vectors
            Fx_smooth = np.zeros(self.numBots)
            Fy_smooth = np.zeros(self.numBots)

            # Iterate through each action and smooth
            for bot_index in order:
                W = np.roll(self.weights, self.loc_1+bot_index+1)
                Fx_bot = np.average(Fx, weights=W)
                Fy_bot = np.average(Fy, weights=W)

                Fx_smooth[bot_index] = Fx_bot
                Fy_smooth[bot_index] = Fy_bot

            # Now let's create the final dictionary again
            smooth_actions = dict()
            for bot_index in order:

                agent = f'Bot_{bot_index}'
                smooth_actions[agent] = [Fx_smooth[bot_index], Fy_smooth[bot_index]]
            return smooth_actions
            

    def smooth_rew(self, rew):
        """
        Given the current reward, a smoothing factor, and the previous reward, 
        returns a reward which is smoothed using exponential moving average. 
        """
        smooth_rew = self.last_rew*self.smooth_weight + (1-self.smooth_weight)*rew
        return smooth_rew
    
    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        # Doing a += so that as multiple contacts occur over the many timesteps, we add them
        self.extForcesX[botIndex-2] = self.convert.Pixels2Meters(impulse[0] / self.dt)
        self.extForcesY[botIndex-2] = self.convert.Pixels2Meters(impulse[1] / self.dt)
        self.botContacts[botIndex-2] = 1
    
        
    
    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True
    
    
    
    
    
    def calcRew(self, distanceToTarget):

        progress = self.previousDistance - distanceToTarget # Relative to the velocity or speed for arriving at target
                    
        rew = progress*50
        smooth_rew = self.smooth_rew(rew)
        self.last_rew = rew
        rew = smooth_rew
        
        """
        Note that above we are only rewarding the system for moving forward! 
        This is to NOT penalize movements backward, which may result in system getting 
        stuck and not back-tracking to remove itself from stuck position
        """
        
        #### Kinetic Energy Reward
        if self.kineticEnergy and np.mean(self.KE)<1e-4:
            rew-=5
            
        #### Velocity Penalty
            if self.velocityPenalty and np.mean(self.velRecent)<0.2:
                rew -= 5
            
        # Reward for decreasing the distance to the target
        # relDistance = 1 - (distanceToTarget/self.startDistance)
        # rew += relDistance*10
            
        return rew
    
    
    def isDone(self, rew, systemCenter, distanceToTarget):
        """
        Can return later on an add penalties for taking too long or other actions we do not want
        """
        done=False
        if self.timestep>self.maxNumSteps:
            rew -= 10
            done=True
            
        # if systemCenter[0] < self.R*2:
        #     done = True
            
        if distanceToTarget<self.R/4:
            done=True
            rew+=10

        # Let's check if the system is moving. This is a 'stuck' termination
        if self.stuck_termination:
            if self.timestep > self.distance_horizon: 
                change = np.abs(self.distance_storage[0] - self.distance_storage[-1])
                if change < self.R/2:
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
            
            # Adding a visual for the target
            targetLoc = pymunk.pygame_util.to_pygame(targetLoc, self.screen) # Converting to pygame coordinates
            pygame.draw.circle(self.screen, (255,0,0), targetLoc, radius = 5)

            self.render_setup=True
            
        for event in pygame.event.get():
            if event.type==pygame.QUIT: 
                pygame.display.quit()
                self.close()
                sys.exit()
            if event.type==pygame.KEYDOWN and event.key == pygame.K_ESCAPE: 
                pygame.display.quit()
                self.close()
                sys.exit()
        
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
            

    def collectAllData(self):
        """
        Will collect all data and store in the global list made in __init__ method
        """
        tempBotPositions = [self.time]
        tempSkinPositions = [self.time]
        tempInteriorPositions = [self.time]

        # Get all positions
        for bot in self.bots:
            botPos = np.asarray(bot.body.position)
            botPos = self.convert.Pixels2Meters(botPos)
            tempBotPositions.append(botPos)
        
        for skin in self.skinParticles:
            skinPos = np.asarray(skin.body.position)
            skinPos = self.convert.Pixels2Meters(skinPos)
            tempSkinPositions.append(skinPos)

        for interior in self.interiorParticles:
            inPos = np.asarray(interior.body.position)
            inPos = self.convert.Pixels2Meters(inPos)
            tempInteriorPositions.append(inPos)

        # Unwrap all lists
        tempBotPositions = list(flatten(tempBotPositions))
        tempSkinPositions = list(flatten(tempSkinPositions))
        tempInteriorPositions = list(flatten((tempInteriorPositions)))

        # Append to the master lists
        self.botPositions.append(tempBotPositions)
        self.skinPositions.append(tempSkinPositions)
        self.interiorPositions.append(tempInteriorPositions)
        
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
        
            
    def exportAllData(self):
        """
        Will export all data that was collected 
        as a result of self.report_all_data
        """
        np.save(self.saveFolder + 'obs_coords', self.obs_points)
        np.save(self.saveFolder + 'bot_coords', np.asarray(self.botPositions))
        np.save(self.saveFolder + 'skin_coords', np.asarray(self.skinPositions))
        np.save(self.saveFolder + 'interior_coords', np.asarray(self.interiorPositions))
        np.save(self.saveFolder + 'target_loc', np.asarray(self.targetLoc))    
    
    
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
    skinParticles = []
    
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
            skinParticles.append(skin)
            
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
            
    return bots, skinParticles, interiorParticles


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


def createWalls(space, screenHeight, screenWidth, wallThickness):
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



def createObstacleField(
    space,
    systemStart, R, thickness, 
    gapCenterLoc, gapHeight, gapWidth,
    height,
    report_points
    ):
    """
    Given the space, a gap center location, and the dimensions of the gap, a gap will be created.
    We also need the dimensions of the field we are in.
    Assume all units passed into this function are in pixels.

    Also, we add walls above and below the system to limit wrongful exploration.
    """
    
    # Get dimensions and positions of the boxes we are about to create
    h = (height-gapHeight)/2
    w = gapWidth
    x = gapCenterLoc[0]
    y_up = gapCenterLoc[1] + (gapHeight/2 + h/2)
    y_down = gapCenterLoc[1] - (gapHeight/2 + h/2)

    # Create the top wall.
    top_b = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    top_s = pymunk.Poly.create_box(top_b, (w,h))
    top_s.body.position = x,y_up
    top_s.collision_type = 1
    space.add(top_b,top_s)

    # Create bottom wall
    bot_b = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    bot_s = pymunk.Poly.create_box(bot_b, (w,h))
    bot_s.body.position = x,y_down
    bot_s.collision_type = 1
    space.add(bot_b,bot_s)
    
    # Add wall above
    X, Y = systemStart[0] , systemStart[1] + R*1.1
    len = 2*(x-X)
    top_wall_b = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    top_wall_s = pymunk.Poly.create_box(top_wall_b, (len,thickness))
    top_wall_s.body.position = X,Y
    top_wall_s.collision_type = 1
    space.add(top_wall_b, top_wall_s)

    # Add wall below
    X, Y = systemStart[0] , systemStart[1] - R*1.1
    len = 2*(x-X)
    bot_wall_b = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    bot_wall_s = pymunk.Poly.create_box(bot_wall_b, (len,thickness))
    bot_wall_s.body.position = X,Y
    bot_wall_s.collision_type = 1
    space.add(bot_wall_b, bot_wall_s)


    if report_points:
        top_points = [
            [x - gapWidth/2, y_up + h/2],
            [x - gapWidth/2, y_up - h/2],
            [x + gapWidth/2, y_up - h/2],
            [x + gapWidth/2, y_up + h/2],
            [x - gapWidth/2, y_up + h/2]
        ]
        bot_points = [
            [x - gapWidth/2, y_down + h/2],
            [x - gapWidth/2, y_down - h/2],
            [x + gapWidth/2, y_down - h/2],
            [x + gapWidth/2, y_down + h/2],
            [x - gapWidth/2, y_down + h/2]
        ]
        points = [top_points, bot_points]
        return np.asarray(points)




def createVideo(saveLoc, imgLoc, videoName, imgShape):
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
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

def flatten(l):
    """
    Given a list that may contain arrays and scalars, will return an unwrapped list
    """
    for item in l:
        try:
            yield from flatten(item)
        except TypeError:
            yield item

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
    