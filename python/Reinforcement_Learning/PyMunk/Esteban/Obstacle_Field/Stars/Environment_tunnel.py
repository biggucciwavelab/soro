
"""
This is NOT the environment that was used for training! This is on which we are using to test the system's ability to navigate through a tunnel
"""

import pymunk
import pygame
import sys
import numpy as np
from tqdm import tqdm
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
        """

        self.report_all_data = True # If True, will report data points for ALL items in simulation, including obstacles and walls.

        # Basic simulation parameters
        self.dt = dt                                                 # Simulation timestep
        self.ppm = ppm                                               # Pixels per Meter
        self.convert = Convert(ppm)                                  # Conversion to be used for all parameters given
        self.height= self.convert.Pixels2Meters(screenHeight)        # Height of the screen, in pixels
        self.width = self.convert.Pixels2Meters(screenWidth)         # Width of the screen, in pixels
        self.maxNumSteps = maxNumSteps                               # Number of steps until simulation terminated
        self.maxVelocity = 20                                        # Arbitrarily set, may need changing later.
        self.forceGain = 2                                     # The maximum force a bot can apply in any given direction
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        self.energy=energy                                           # If True, then we do care about calculating how much energy our system is expending to complete its mission
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
        self.systemStart = R+botRadius+wallThickness*1.2, self.convert.Pixels2Meters(screenHeight/2)
        self.startDistance = np.linalg.norm(self.systemStart)
        
        # Position of target, relative to system start
        self.targetDistance = targetDistance = R*72
        self.targetLoc = np.asarray([self.systemStart[0]+targetDistance, self.systemStart[1]]) # Located directly down the x-axis
            
            
        self.kineticEnergy = kineticEnergy # If True, then we are calculating the Kinetic energy of the system
        if self.kineticEnergy:
            self.KE = np.zeros(30) # We will store 30 timesteps worth of information
            
        self.velocityPenalty = velocityPenalty
        if self.velocityPenalty:
            self.velRecent = np.zeros(30) # We will store 30 timesteps worth of information

        self.state_size = self.numBots*5 # botRePosition (x,y), botContacts
        self.action_size = self.numBots*2
        force_low, force_high = -1.0, 1.0
        
        low = np.full(self.state_size, -10)
        high = np.full(self.state_size, 10)
        self.observation_space = spaces.Box(low,high,dtype=np.float32)
        
        self.action_space = spaces.Box(
            low = force_low,
            high = force_high,
            shape=(self.action_size,),
            dtype=np.float32
            ) 
        
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
                                       ['energy',str(energy)],
                                       ['kinetic energy',str(kineticEnergy)],
                                       ['velocity penalty',str(velocityPenalty)],
                                       ['distance reward',str(distanceReward)],
                                       ['PixelsPerMeter (ppm):',str(self.ppm)],
                                       ['SystemRadius',str(self.R)],
                                       ['ScreenWidth',str(self.width)],
                                       ['ScreenHeight',str(self.height)],
                                       ['Num_Interior:', str(self.numInterior)],
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
        
        self.render_setup = False # Changes to true once the rendering tools have been setup. This will only happen externally if rendering has been requested
        self.space = pymunk.Space()
        self.space.gravity = 0,0
        self.timestep = 0 # Initializing timestep
        self.time = 0     # Initializing time
        
        # Information for contact, will be changed later
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
        self.botContacts = np.zeros(self.numBots)
        
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
        self.bots, self.skinParticles, self.interiorParticles = createJamoeba(**kwargs)
        
        # Note: Membrane consists of bots and skin particles
        self.jamoeba = [*self.bots, *self.skinParticles, *self.interiorParticles] # This is all bodies within the space! 
        
        # Add sliding friction as needed
        if self.slidingFriction > 0:
            add_sliding_friction(self.space, self.jamoeba, self.slidingFriction)

        #### Obstacle field
        # tunnelKwargs = dict(
        #     space=self.space,
        #     tunnel_thickness=R/1.5,
        #     tunnel_width = 2*R,
        #     system_R=R,
        #     systemStart=systemStart,
        #     height=height
        # )
        # create_tunnel(**tunnelKwargs)

        tunnel2Kwargs = dict(
            space=self.space,
            tunnel_open = R,
            tunnel_close = R/1.5,
            tunnel_width = 2*R,
            system_R=R,
            systemStart=systemStart,
            height=height,
            return_points = self.report_all_data
        )
        if self.report_all_data:
            self.tunnel_points = create_tunnel2(**tunnel2Kwargs)
            self.convertTunnelPoints()
        else:
            create_tunnel2(**tunnel2Kwargs)

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
        
        # Storing previous distances to ensure movement
        self.distance_horizon = 100 # How may time steps we must move by, or episode is terminated!
        self.distance_storage = np.zeros(self.distance_horizon)
        
        # Creating folder for all data
        if self.report_all_data:
            self.botPositions = []
            self.skinPositions = []
            self.interiorPositions = []

        #### Initiate Observation
        ac = np.array([0]*2*self.numBots) # Take no action
        observation, _, self.previousDistance, _ = self.getOb(ac)
        self.distance_storage[0] = self.previousDistance
        return observation
    
    
    
    
        
    def step(self, ac):
        
        # Contact information for storing
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
        self.botContacts = np.zeros(self.numBots)
        
        for i in range(self.numStepsPerStep):
            # Take action
            forcesX = []
            forcesY = []
            for index in range(self.numBots):
                
                xForce = ac[2*index]*self.forceGain
                yForce = ac[2*index+1]*self.forceGain
                
                forcesX.append(xForce)
                forcesY.append(yForce)
                
            forcesX = np.asarray(forcesX, dtype=np.float)
            forcesY = np.asarray(forcesY, dtype=np.float)
                
            for index, bot in enumerate(self.bots):
                xForce = forcesX[index]
                yForce = forcesY[index]
                
                botPos = bot.body.position
                bot.body.apply_force_at_world_point((xForce, yForce), (botPos.x, botPos.y))

        # Taking a step in the environment
            self.space.step(self.dt)
            self.time += self.dt
        
        # Note the important difference between timestep and time!
        self.timestep+=1
        
        # Gather information
        obs, systemCenter, distanceToTarget, sysNormForce = self.getOb(ac)
        self.distance_storage = np.roll(self.distance_storage, 1)
        self.distance_storage[0] = distanceToTarget
        
        rew = self.calcRew(distanceToTarget, sysNormForce)
        isDone, rew = self.isDone(rew, systemCenter, distanceToTarget)
        self.previousDistance = distanceToTarget

        if self.dataCollect: 
            self.dataCollection(ac,rew,obs)

        if self.report_all_data:
            self.collectAllData() # If we are reporting all data, then we must report all data positions
 
        return obs, rew, isDone, self.info
    
    
    
    
    
    def getOb(self, ac):
        
        runTime = [self.timestep/self.maxNumSteps]
        
        botPos = np.zeros((self.numBots,2))
        botVel = np.zeros((self.numBots,2))
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
            # Define position as relative to the target
            botPos[index,:] = np.array(self.targetLoc) - np.array(currentBotPos)
            botVel[index,:] = currentBotVel

        systemCenter = np.mean(botPos, axis=0)
        
        distanceToTarget = np.linalg.norm(systemCenter) # This value is in meters
        
        botForces = np.zeros(self.numBots*2)
        for index, action in enumerate(ac):
            botForces[index] = action
        
        #### For Energy Consumption
        sysNormForce=0
        if self.energy:
            sysNormForce = np.linalg.norm(botForces*self.numStepsPerStep)
        
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
            
        # Normalizing observation
        botPos[:,0]=botPos[:,0]/(self.width) #Normalizing X-Coordinate
        botPos[:,1]=botPos[:,1]/(self.height)                  #Normalizing Y-Coordinate
        # botVel /= self.maxVelocity                             #Normalizing Velocity
        
        extForces = np.abs(np.concatenate((self.extForcesX, self.extForcesY)))
        extForces /= self.forceGain
        
        # observation = np.concatenate((botPos.flatten(), botVel.flatten(), botForces, extForces, runTime))
        observation = np.concatenate((botPos.flatten(), botVel.flatten(), self.botContacts))
        
        return observation, systemCenter, distanceToTarget, sysNormForce


    
    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        # Doing a += so that as multiple contacts occur over the many timesteps, we add them
        self.extForcesX[botIndex-2] = impulse[0] / self.dt
        self.extForcesY[botIndex-2] = impulse[1] / self.dt
        self.botContacts[botIndex-2] = 1
    

    
    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True
    

    
    def calcRew(self, distanceToTarget, sysNormForce):

        progress = self.previousDistance - distanceToTarget # Relative to the velocity or speed for arriving at target
                    
        rew = progress*200 - sysNormForce*.5
        if rew<0: rew = 0 
        """
        Note that above we are only rewarding the system for moving forward! 
        This is to NOT penalize movements backward, which may result in system getting 
        stuck and not back-tracking to remove itself from stuck position
        """
        
        #### Kinetic Energy Reward
        if self.kineticEnergy and np.mean(self.KE)<1e-4:
            rew-=5
            
        #### Velocity Penalty
        
        if self.velocityPenalty:
            if self.numBots ==10:
                thresh = 0.075
            elif self.numBots == 30:
                thresh = .1
            if np.mean(self.velRecent)<thresh:
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
            done=True
            
        if systemCenter[0] < self.R*2:
            done = True
            
        if distanceToTarget<1e-1:
            done=True
            rew+=10
        
        # if self.timestep > self.distance_horizon+100:
        #     change = np.abs(self.distance_storage[0] - self.distance_storage[-1])
        #     if change < self.R/2:
        #         done=True
        #         rew -= 10  
            
        return done, rew
    
    
    
    def render(self):
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
            
            
    def convertTunnelPoints(self):
        tunnel_points = np.asarray(self.tunnel_points)
        self.tunnel_points = self.convert.Pixels2Meters(tunnel_points)


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
        np.save(self.saveFolder + 'tunnel_coords', self.tunnel_points)
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
        pivot.max_bias = 0 # Disable joint correction
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
    


def create_tunnel(space, tunnel_thickness, tunnel_width, 
                    systemStart, system_R,
                    height):
    """
    Thus tunnel will take into account the system's start position, 
    and the height of the environment, and will position itself in front of the system.
    """
    # Get tunnel position
    X, Y = systemStart
    x = X + tunnel_width/2 + system_R*4 # The x-position for top and bottom tunnel should be the exact same
    box_height = (height - tunnel_thickness)/2
    y_bottom = box_height/2
    y_top = height - box_height/2

    # Make the top tunnel
    top = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    top_s = pymunk.Poly.create_box(top, (tunnel_width, box_height))
    top_s.body.position = x,y_top
    top_s.collision_type = 1

    # Make bottom tunnel
    bottom = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    bottom_s = pymunk.Poly.create_box(bottom, (tunnel_width, box_height))
    bottom_s.body.position = x,y_bottom
    bottom_s.collision_type = 1

    # Add to space
    space.add(top,bottom,top_s,bottom_s)


def create_tunnel2(space, tunnel_open, tunnel_close, tunnel_width, 
                    systemStart, system_R,
                    height, return_points = False):
    """
    This function will create a tunnel which is closing to a tight hole, thus constraining the system's ability to get through.
    """
    # Getting points for tunnel
    X,Y = systemStart
    x1 = x4 = X + system_R*5
    x2 = x3 = x1 + tunnel_width

    h1 = (height-tunnel_open)/2
    h2 = (height-tunnel_close)/2
    yb1 = h1
    yb2 = h2
    yb3 = yb4 = 0
    yt1 = height-h1
    yt2 = height-h2
    yt3 = yt4 = height

    # Lists with vertices
    top_points = [
        (x1,yt1), (x2,yt2), (x3,yt3), (x4,yt3)
    ]
    bottom_points = [
        (x1,yb1), (x2,yb2), (x3,yb3), (x4,yb4)
    ]

    # Create top wall
    top_b = pymunk.Body(body_type=pymunk.Body.STATIC)
    top_s = pymunk.Poly(top_b, top_points)
    top_s.collision_type = 1

    # Create bottom wall
    bottom_b = pymunk.Body(body_type=pymunk.Body.STATIC)
    bottom_s = pymunk.Poly(bottom_b, bottom_points)
    bottom_s.collision_type = 1

    space.add(top_b, top_s, bottom_b, bottom_s)

    if return_points:
        all_points = [top_points, bottom_points]
        return all_points

  


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