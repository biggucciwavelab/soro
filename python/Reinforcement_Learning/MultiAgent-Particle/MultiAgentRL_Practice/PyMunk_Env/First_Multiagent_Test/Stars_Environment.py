
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
from gym import spaces, Env
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.interpolate import RegularGridInterpolator
from shutil import rmtree
import glob # For creating videos
import cv2 # For creating videos
from warnings import warn
import os
import pdb

class pymunkEnv(Env):
    
    info = {'There is no info to be aware of.':'Machinelearn away.'}
    
    n=2 # Two agents in this environment

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
        """
        # Basic simulation parameters
        self.dt = dt                                                 # Simulation timestep
        self.ppm = ppm                                               # Pixels per Meter
        self.convert = Convert(ppm)                                  # Conversion to be used for all parameters given
        self.height= self.convert.Pixels2Meters(screenHeight)        # Height of the screen, in pixels
        self.width = self.convert.Pixels2Meters(screenWidth)         # Width of the screen, in pixels
        self.maxNumSteps = maxNumSteps                               # Number of steps until simulation terminated
        self.maxVelocity = 20                                        # Arbitrarily set, may need changing later.
        self.maxAppliedForce = 1                                     # The maximum force a bot can apply in any given direction
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        self.energy=energy                                           # If True, then we do care about calculating how much energy our system is expending to complete its mission
        self.numStepsPerStep = numStepsPerStep                       # The number of simulation timesteps to run for each call to 'step' function
        self.slidingFriction = slidingFriction                       # Coefficient of friction for objects sliding on the ground
        
        # The maximum veloctiy that any object is able to move at.
        self.velocity_limit = velocity_limit # Set to 0 if no limit desired
        
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
        self.systemStart = R+botRadius+wallThickness*1.2, self.convert.Pixels2Meters(screenHeight/2)
        self.startDistance = np.linalg.norm(self.systemStart)
        
        # Position of target, relative to system start
        self.targetDistance = targetDistance = R*72
        self.targetLoc = self.systemStart[0]+targetDistance, self.systemStart[1] # Located directly down the x-axis
            
            
        self.kineticEnergy = kineticEnergy # If True, then we are calculating the Kinetic energy of the system
        if self.kineticEnergy:
            self.KE = np.zeros(30) # We will store 30 timesteps worth of information
            
        self.velocityPenalty = velocityPenalty
        if self.velocityPenalty:
            self.velRecent = np.zeros(15) # We will store 30 timesteps worth of information

        self.state_size = self.numBots*8 # botRePosition (x,y), botVel (x,y), botCurrentAction(x,y), botExternalForce(x,y)
        self.action_size = self.numBots*2
        force_low, force_high = -self.maxAppliedForce, self.maxAppliedForce
        
        low = np.full(self.state_size, -np.inf) # This is the state size of only the bots (agent 1)
        high = np.full(self.state_size, np.inf) # This is the state size of only the bots (agent 1)
        self.observation_space = [
            spaces.Box(low,high,dtype=np.float32), # First agent sees all information regarding 
            spaces.Box(low = 0, high=1,shape=(8,),dtype=np.float32) # In the future, the observation should be more tightly bound
        ]
        
        self.action_space = [
            spaces.Box(low = force_low, high = force_high, shape=(self.action_size,), dtype=np.float32), # Agent 1 actuates the system
            spaces.Box(low=0, high=1, shape=(2,), dtype=np.float32)
        ]
        
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
                                       ['botFriction',str(self.botFriction)],
                                       ['inMass',str(self.inMass)],
                                       ['inFriction',str(self.inFriction)],
                                       ['percentInteriorRemoved',str(self.percentInteriorRemove)],
                                       ['SpringK:', str(self.springK)], 
                                       ['SpringB:', str(self.springB)], 
                                       ['SpringRL:', str(self.springRL)],
                                       ['maxSeparation',str(self.maxSeparation)],
                                       ['slidingFriction', str(self.slidingFriction)],
                                       ['velocity_limit', str(self.velocity_limit)],
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
            self.ac = np.zeros(self.action_size  +1)
            self.reward_data = np.zeros(3) # Storing rewards for agent 1 and 2
            self.obs_data = np.zeros(self.state_size + 8 + 1)
            self.agent2_locs = np.zeros(3) # Storing the decision made by agent 2
            
        if self.saveVideo:
            self.videoFolder = experimentName + '_VideoImages/'
            os.makedirs(self.videoFolder,exist_ok=True)
        
        return None
        
        
        
        
        
    def reset(self):
        
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
        velocity_limit = self.convert.Meters2Pixels(self.velocity_limit) # The maxmimum velocity any object in the system can move at.

        
        height = self.convert.Meters2Pixels(self.height)
        width = self.convert.Meters2Pixels(self.width)
        wallThickness = self.convert.Meters2Pixels(self.wallThickness)
        targetDistance = self.convert.Meters2Pixels(self.targetDistance)
        
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
        
        #### Obstacle field
        obsRadius = 2*R
        fieldWidth = targetDistance - 4*obsRadius
        fieldHeight = height-2*obsRadius
        minObsDistance = 4.5*R # Stars and Squares
        # minObsDistance = 6*R # Circles
        poisSamples = Poisson_Sampling(minObsDistance,fieldWidth,fieldHeight)
        obsCoordinates = poisSamples.get_samples()
        obsKwargs = {'space':self.space,
                    'obstacleCoordinates': obsCoordinates,
                        'obsRadius':obsRadius,
                        'friction':self.botFriction,
                        'square':False,
                        'star':True}
        # obstacles = createObstacleField(**obsKwargs)

        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness,
                    'env':self}
        createWalls(**wallKwargs) # Not creating the walls right now.
        
        if self.velocity_limit > 0 :
            vel_limiter = limiter(velocity_limit)
            for obj in self.jamoeba:
                obj.body.velocity_func = vel_limiter.limit_velocity

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

        # Create a previous agent 2 selected position
        self.agent2_prev_loc = self.systemStart / np.array([self.width, self.height])
        self.prev_center = self.systemStart / np.array([self.width, self.height])
        
        #### Initiate Observation
        ac = [np.array([0]*2*self.numBots),  self.agent2_prev_loc]
        ac = np.asarray(ac)
        observation, _, self.previousDistance2, _, self.previousDistance1 = self.getOb(ac)
        return observation
    
    
    
    
        
    def step(self, action):
        
        ac = action[0]
        agent2_loc = action[1]

        # Contact information for storing
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
        
        for i in range(self.numStepsPerStep):
            # Take action
            forcesX = []
            forcesY = []
            for index in range(self.numBots):
                
                xForce = ac[2*index]
                yForce = ac[2*index+1]
                
                forcesX.append(xForce)
                forcesY.append(yForce)
                
            forcesX = np.asarray(forcesX, dtype=np.float)
            forcesY = np.asarray(forcesY, dtype=np.float)
                
            for index, bot in enumerate(self.bots):
                xForce = forcesX[index]
                yForce = forcesY[index]
                
                botPos = bot.body.position
                bot.body.apply_force_at_world_point((xForce, yForce), (botPos.x, botPos.y))
            
            # Apply frictional force to moving objects
            if self.slidingFriction > 0:
                self.apply_friction()

        # Taking a step in the environment
            self.space.step(self.dt)
            self.time += self.dt
        
        # Note the important difference between timestep and time!
        self.timestep+=1
        
        # Gather information
        obs, systemCenter_agent2, distanceToTarget, sysNormForce, distanceToAgent2 = self.getOb(action)
        """
        obs: List with 2 elements. Contains an observation for each agent
        systemCenter_agent2: The system's center w.r.t. the target location
        distanceToTarget: The distance from system center to target
        sysNormForce: The L2 norm of forces applied to system 
        distanceToAgent2: Distance from system center to location specified by system 2
        """
        
        rew = self.calcRew(distanceToTarget, sysNormForce, distanceToAgent2)
        isDone, rew = self.isDone(rew, systemCenter_agent2, distanceToTarget)

        # Update previous informations
        self.previousDistance2 = distanceToTarget
        self.previousDistance1 = distanceToAgent2
        self.prev_center = systemCenter_agent2
        self.agent2_prev_loc = agent2_loc

        if self.dataCollect: 
            self.dataCollection(action,rew,obs)
 
        return obs, rew, isDone, self.info
    
    
    
    
    
    def getOb(self, ac):
        
        runTime = [self.timestep/self.maxNumSteps] # This can be used in the future, if needed
        
        bot_actions = ac[0]
        agent2_pos = ac[1]
        
        field_norm = np.array([self.width, self.height]) # Field normalizer

        # Gathering base information
        botPos = np.zeros((self.numBots,2))         # Position of bots relative to agent 2's selected location
        bot_posCenter = np.zeros((self.numBots,2))  # Position of bots relative to target location
        botVel = np.zeros((self.numBots,2))
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
            # Position is relative to different measures
            botPos[index,:] = np.array(agent2_pos) - (np.array(currentBotPos)/field_norm) # The position is relative to the selected location of agent 2
            bot_posCenter[index,:] = np.array(self.targetLoc) - np.array(currentBotPos)   # The position is relative to the target location
            botVel[index,:] = currentBotVel

        # Info for agent 1
        systemCenter_agent1 = np.mean(botPos, axis=0)
        distanceToAgent2 = norm(systemCenter_agent1)
        systemCenter_agent1 = systemCenter_agent1 / field_norm # Can be used to calculate how quickly we approach where system 2 is.

        botForces = np.zeros(self.numBots*2)
        for index, action in enumerate(bot_actions):
            botForces[index] = action

        # Info for agent 2
        systemCenter_agent2 = np.mean(bot_posCenter, axis=0)
        distanceToTarget = np.linalg.norm(systemCenter_agent2) # This value is in meters
        systemCenter_agent2 = systemCenter_agent2 / field_norm # Normalized system center location
        

        #### For Energy Consumption
        sysNormForce=0
        if self.energy:
            sysNormForce = np.linalg.norm(botForces*self.numStepsPerStep)
        
        #### Calculating Kinetic Energy of system
        if self.kineticEnergy:
            self.calc_KE()
            
        #### Calculating penalty for not moving (based on velocity)
        if self.velocityPenalty:
            self.velRecent = np.roll(self.velRecent,1)
            velX = np.mean(botVel[:,0])
            velY = np.mean(botVel[:,1])
            self.velRecent[0] = np.linalg.norm([velX,velY])
            
        # Normalizing observation
        botPos[:,0]=botPos[:,0]/(self.targetDistance+2*self.R) #Normalizing X-Coordinate
        botPos[:,1]=botPos[:,1]/(self.height)                  #Normalizing Y-Coordinate
        botVel /= self.maxVelocity                             #Normalizing Velocity
        
        extForces = np.abs(np.concatenate((self.extForcesX, self.extForcesY)))
        extForces /= self.maxAppliedForce
        
        # Defingin observations
        obs_agent1 = np.concatenate((botPos.flatten(), botVel.flatten(), botForces, extForces))
        obs_agent2 = np.concatenate((systemCenter_agent2, self.prev_center, agent2_pos, self.agent2_prev_loc))
        observation = [obs_agent1, obs_agent2]
        
        return observation, systemCenter_agent2, distanceToTarget, sysNormForce, distanceToAgent2
    


    
    def calc_KE(self):
        self.KE = np.roll(self.KE,1)
        KE_now = 0
        for obj in self.jamoeba:
            mass = obj.shape.mass
            speed = self.convert.Pixels2Meters(obj.body.velocity.length)
            KE_now += 0.5*mass*speed**2
        self.KE[0] = KE_now




    def apply_friction(self):
        """
        Will apply a frictional force to all bodies that are moving
        """
        for obj in self.jamoeba:
            body = obj.body

            # Check the velocity of the body.
            vel = np.asarray(body.velocity)
            vel_norm = norm(vel)
            if vel_norm>1e-3: # WE HAVE MOTION
                mass = body.mass
                pos = body.position
                dir = vel/vel_norm
                dir *= (-1)
                F_friction  = mass*self.slidingFriction
                F_vector = dir*F_friction

                body.apply_force_at_world_point((F_vector[0],F_vector[1]), (pos.x,pos.y))

    



    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        # Doing a += so that as multiple contacts occur over the many timesteps, we add them
        self.extForcesX[botIndex-2] = impulse[0]
        self.extForcesY[botIndex-2] = impulse[1]
    
    
    
    
    
    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True
    
    
    
    
    
    def calcRew(self, distanceToTarget, sysNormForce, distanceToAgent2):
        
        # Calc reward for agent 1
        if self.timestep <= 1:
            rew1 = 0 
        else:
            progress1 = self.previousDistance1 - distanceToAgent2
            rew1 = progress1*4_000

        # Calc reward for agent 2
        """
        Note that we are only rewarding the system for moving forward! 
        This is to NOT penalize movements backward, which may result in system getting 
        stuck and not back-tracking to remove itself from stuck position
        """
        progress2 = self.previousDistance2 - distanceToTarget # Relative to the velocity or speed for arriving at target
        rew2 = progress2*2_000
        if rew2<0: rew = 0 

        # Reward for decreasing the distance to the target
        relDistance = 1 - (distanceToTarget/self.startDistance)
        # rew2 += relDistance*10
        
        #### Kinetic Energy Penalty
        if self.kineticEnergy and np.mean(self.KE)<1e-4:
            rew-=5
            
        #### Velocity Penalty
        if self.velocityPenalty and np.mean(self.velRecent)<0.2:
            rew -= 5
        
        rew = [rew1, rew2]
        return rew
    



    
    def isDone(self, rew, systemCenter, distanceToTarget):
        """
        Can return later on an add penalties for taking too long or other actions we do not want
        """
        done=[False,False]
        if self.timestep>self.maxNumSteps:
            done=[True,True]
            
        if distanceToTarget<1e-1:
            done=[True,True]
            # rew[0]+=10
            rew[1]+=10 # Reward agent 2 for getting the system there.
            
        return done, rew
    
    
    
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
        agent2_loc_temp = [self.time]
        
        for bot in self.bots:
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x)-self.targetLoc[0], self.convert.Pixels2Meters(bot.body.position.y)-self.targetLoc[1]
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
            X_Pos_temp.append(currentBotPos[0])
            Y_Pos_temp.append(currentBotPos[1])
            
            X_vel_temp.append(currentBotVel[0])
            Y_vel_temp.append(currentBotVel[1])
            
            
        for action in ac[0]:
            action_temp.append(action)
        
        for r in rew:
            rew_temp.append(r)
            
        for observation in obs:
            for num in observation.flatten():
                obs_temp.append(num)

        for val in ac[1]:
            agent2_loc_temp.append(val)
            
        # Convert to Numpy Arrays
        X_Pos_temp = np.asarray(X_Pos_temp)
        Y_Pos_temp = np.asarray(Y_Pos_temp)
        
        X_vel_temp = np.asarray(X_vel_temp)
        Y_vel_temp = np.asarray(Y_vel_temp)
        
        action_temp = np.asarray(action_temp)
        rew_temp = np.asarray(rew_temp)
        obs_temp = np.asarray(obs_temp)

        agent2_loc_temp = np.asarray(agent2_loc_temp)
        
        # Now append to the master list
        self.X_data = np.vstack([self.X_data, X_Pos_temp])
        self.X_vel_data = np.vstack([self.X_vel_data, X_vel_temp])
        self.Y_data = np.vstack([self.Y_data, Y_Pos_temp])
        self.Y_vel_data = np.vstack([self.Y_vel_data, Y_vel_temp])
        self.ac = np.vstack([self.ac, action_temp])
        self.reward_data = np.vstack([self.reward_data, rew_temp])
        self.obs_data = np.vstack([self.obs_data, obs_temp])
        self.agent2_locs = np.vstack([self.agent2_locs, agent2_loc_temp])
            
        
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
        self.agent2_locs = np.delete(self.agent2_locs, 0, 0)
        

        # Save the data on .csv files
        np.savetxt(self.saveFolder + 'X_data.csv', self.X_data, delimiter=',')
        np.savetxt(self.saveFolder + 'X_vel_data.csv', self.X_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_data.csv', self.Y_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_vel_data.csv', self.Y_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'actions.csv', self.ac, delimiter=',')
        np.savetxt(self.saveFolder + 'reward.csv', self.reward_data, delimiter=',')
        np.savetxt(self.saveFolder + 'observations.csv', self.obs_data, delimiter=',')
        np.savetxt(self.saveFolder + 'agent2_locs.csv', self.agent2_locs, delimiter=',')
        
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
                plt.plot(time, self.ac[:,i+1], label='Z-Force')
                plt.xlabel(xlabel)
                plt.ylabel('Force [N]')
                plt.title('Applied Forces on Bot ' + str(bot))
                plt.legend(loc='lower right')
                plt.savefig(self.saveFolder + 'Bot ' + str(bot) + ' Applied Forces.jpg')
                bot+=1
                
        # Plot reward
        time = self.reward_data[:,0]
        rewards_agent1 = self.reward_data[:,1]
        rewards_agent2 = self.reward_data[:,2]
        plt.figure('Rewards')
        plt.plot(time, rewards_agent1, label='Agent 1 Reward')
        plt.plot(time, rewards_agent2, label ='Agent 2 Reward')
        plt.xlabel(xlabel)
        plt.ylabel('Reward')
        plt.title('Reward for JAMoEBA')
        plt.legend()
        plt.savefig(self.saveFolder + 'Reward.jpg')
        
        # Plotting decisions made by agent 2
        plt.figure('Agent2Locations')
        plt.plot(self.agent2_locs[:,1], self.agent2_locs[:,2])
        plt.xlabel('X-Axis')
        plt.ylabel('Y-Axis')
        plt.title('Agent 2 Selected Locations')
        plt.savefig(self.saveFolder + 'Agent2_Locs.jpg')
            
        
    
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
        
class starObstacle:
    def __init__(self, space, position, width, friction, rotation=0, color=(0,0,0,255)):
        self.body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.body.position = position
        self.body.angle=rotation
        self.width = width
        self.position = position
        self.friction = friction
        
        space.add(self.body)
        
        # Gather points of star based
        xo = [] # Exterior x-points
        yo = [] # Exterior y-points
        xi = [] # Interior x-points
        yi = [] # Interioer y-points
        polyPoints= [] # Gathering all the points in the order that we want to connect them in
        for k in range(5):
            xOutside = width*cos(2*np.pi*k/5)
            yOutside = width*sin(2*np.pi*k/5)
            xInside = (width/2)*cos(2*np.pi*k/5 + np.pi/5)
            yInside = (width/2)*sin(2*np.pi*k/5 + np.pi/5)
            xo.append(xOutside)
            yo.append(yOutside)
            xi.append(xInside)
            yi.append(xInside)
            polyPoints.append((xOutside,yOutside))
            polyPoints.append((xInside,yInside))
        
        """
        Creating shape via segments
        """
        segments = []
        numPoints = len(polyPoints)
        for point in range(numPoints-1):
            seg = pymunk.Segment(self.body,polyPoints[point],polyPoints[point+1],5)
            seg.friction = friction
            seg.color = color
            seg.collision_type = 1
            segments.append(seg)
            space.add(seg)
            
            # Connecting the last point to the first 
            if point == numPoints-2:
                seg = pymunk.Segment(self.body, polyPoints[-1],polyPoints[0],5)
                seg.friction = friction
                seg.color = color
                seg.collision_type = 1
                space.add(seg)
                

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
    
    space.add(shape1,body1,shape2,body2,shape3,body3)
    
    return None


def createVideo(saveLoc, imgLoc, videoName, imgShape):
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
    rmtree(imgLoc)
    
def createObstacleField(space, obstacleCoordinates, obsRadius, friction, square=False, star=False):

        obstacles = []
        for coord in obstacleCoordinates:
            posX = coord[0]+obsRadius*2
            posY = coord[1]+obsRadius
            
            obstacle = starObstacle(space,(posX,posY),obsRadius,friction,(np.pi)*np.random.rand())
            obstacles.append(obstacle)
            
        return obstacles
        
    
def printProgressBar(length, iteration, total,fill='=', prefix='', suffix='Complete', printEnd='\r'):
    percent = ("{0:." + str(1) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    

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
        self.a = self.r/np.sqrt(2)
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

        dxdy = [(-1,-2),(0,-2),(1,-2),(-2,-1),(-1,-1),(0,-1),(1,-1),(2,-1),
            (-2,0),(-1,0),(1,0),(2,0),(-2,1),(-1,1),(0,1),(1,1),(2,1),
            (-1,2),(0,2),(1,2),(0,0)]
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
            distance2 = (nearby_pt[0]-pt[0])**2 + (nearby_pt[1]-pt[1])**2
            if distance2 < self.r**2:
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
            rho, theta = np.random.uniform(self.r, 2*self.r), np.random.uniform(0, 2*np.pi)
            pt = refpt[0] + rho*np.cos(theta), refpt[1] + rho*np.sin(theta)
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
                active.append(len(self.samples)-1)
                self.cells[self.get_cell_coords(pt)] = len(self.samples) - 1
            else:
                # We had to give up looking for valid points near refpt, so remove it
                # from the list of "active" points.
                active.remove(idx)
        return self.samples

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