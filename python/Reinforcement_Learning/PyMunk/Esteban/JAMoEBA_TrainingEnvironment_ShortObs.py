
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
import sys
import numpy as np
from numpy import sin, cos, sqrt, log
from math import floor
from gym import spaces, Env
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.interpolate import RegularGridInterpolator
from shutil import rmtree
import glob # For creating videos
import cv2 # For creating videos
from warnings import warn
import easygui
import os
import pdb

class pymunkEnv(Env):
    
    info = {'There is no info to be aware of.':'Machinelearn away.'}
    
    def __init__(self, dt, ppm, screenHeight, screenWidth, maxNumSteps, R, numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, inRadius, botFriction, inMass, inFriction, percentInteriorRemove, springK, springB, springRL, wallThickness, maxSeparation, targetDistance, dataCollect=False, experimentName="NOT NAMED", saveVideo = False, obstaclesPresent = False, grabObjectPresent=False, potFieldPresent=False, tunnel=False, targetLoc = None, systemStart = None):
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
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        self.obstaclesPresent = obstaclesPresent                     # Are obstacles present in this environment?
        self.grabObjectPresent = grabObjectPresent                   # Does this environment have an object that we are grabbing?
        self.potFieldPresent = potFieldPresent                       # Whether to use a PotField with the object
        if self.potFieldPresent: assert grabObjectPresent, 'ERROR: Must have an object to grab if a potField is present!'
        self.tunnel = tunnel
        
        self.numSkip = 30 # A parameter to decide how many bots to gather information from.
        
        assert self.numSkip <= numBots, "You cannot skip more bots than you have! Change this variable."
        
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
        
        # Position of target, relative to system start
        self.targetDistance = targetDistance
        if targetLoc == None:
            self.targetLoc = self.systemStart[0]+targetDistance, self.systemStart[1] # Located directly down the x-axis
            self.moveObject = False
        else:
            assert systemStart!=None, 'Specifying the targetlocation also requires specifying the system start location'
            self.targetLoc = targetLoc
            self.systemStart = systemStart
            self.moveObject = True
            
            
        
        #### Gym API
        if self.grabObjectPresent:
            self.state_size = self.numBots*8 + 4 + 1 # botRelPosition (x,y), botVel (x,y), botCurrentAction(x,y), objectPos(x,y), botVel(x,y), timeLeft
        else:
            self.state_size = (self.numBots//self.numSkip)*8 + 1 # botRePosition (x,y), botVel (x,y), botCurrentAction(x,y), timeLeft
        print(self.state_size)
        self.action_size = self.numBots*2
        
        low = np.full(self.state_size, -10)
        high = np.full(self.state_size, 10)
        self.observation_space = spaces.Box(low,high,dtype=np.float32)
        
        self.action_space = spaces.MultiDiscrete([3]*2*self.numBots) # Will return a 0, 1, or 2at every timestep. This will correspond to positive, negative, or zero force applied.
        
        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius, self.percentInteriorRemove)
        self.numInterior = np.sum(granPerRing)
        
        #### Data Collection
        self.environment_parameters = [['Target Distance:',str(targetDistance)],
                                       ['PixelsPerMeter (ppm)',str(self.ppm)],
                                       ['SystemRadius',str(self.R)],
                                       ['ScreenWidth',str(self.width)],
                                       ['ScreenHeight',str(self.height)],
                                       ['Num_Bots:', str(self.numBots)],
                                       ['Num_Interior:', str(self.numInterior)],
                                       ['Bot_Radius:', str(self.botRadius)],
                                       ['JAMoEBA_Radius:', str(self.R)], 
                                       ['SpringK:', str(self.springK)], 
                                       ['SpringN:', str(self.springB)], 
                                       ['SpringRL:', str(self.springRL)],
                                       ['inRadius',str(self.inRadius)],
                                       ['inFriction',str(self.inFriction)],
                                       ['skinRatio',str(self.skinRatio)],
                                       ['skinMass',str(self.skinMass)],
                                       ['skinRadius',str(self.skinRadius)],
                                       ['botMass',str(self.botMass)],
                                       ['botFriction',str(self.botFriction)],
                                       ['maxSeparation',str(self.maxSeparation)],
                                       ['maxNumSteps',str(self.maxNumSteps)],
                                       ['percentInteriorRemoved',str(self.percentInteriorRemove)],
                                       ['obstaclesPresent',str(obstaclesPresent)],
                                       ['grabObjectPresent',str(grabObjectPresent)],
                                       ['tunnel',str(tunnel)],
                                       ['potFieldPresent',str(potFieldPresent)],
                                       ['targetLoc',str(targetLoc)],
                                       ['systemStart',str(systemStart)]]
        
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
            if self.grabObjectPresent:
                self.objPos = np.zeros(3)
                self.objVel = np.zeros(3)
        
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
        self.bots, interiorParticles = createJamoeba(**kwargs)
        
        #### Obstacle field
        if self.obstaclesPresent:
            obsRadius = 2*R
            fieldWidth = targetDistance - 4*obsRadius
            fieldHeight = height - 2*obsRadius
            minObsDistance = 6*R
            poisSamples = Poisson_Sampling(minObsDistance,fieldWidth,fieldHeight)
            obsCoordinates = poisSamples.get_samples()
            obsKwargs = {'space':self.space,
                        'obstacleCoordinates': obsCoordinates,
                         'obsRadius':obsRadius,
                         'friction':self.botFriction,
                         'square':False}
            obstacles = createObstacleField(**obsKwargs)
        
        #### Object to grasp
        if self.grabObjectPresent:
            systemMass = np.sum([self.numBots*self.botMass,
                                 self.skinRatio*self.numBots*self.skinMass,
                                 self.numInterior*self.inMass])
            if self.moveObject:
                objPosition = self.convert.Meters2Pixels(self.R*6), self.convert.Meters2Pixels(self.height/2)
            else:
                objPosition = self.convert.Meters2Pixels(self.systemStart[0] + 1.5*self.R + self.botRadius), self.convert.Meters2Pixels(self.systemStart[1]) # Starting right next to the system
            objRadius = self.R*0.25
            self.prevBotDisToObj = np.linalg.norm([(1.5*self.R + self.botRadius), 0]) 
            objKwargs = {'space':self.space,
                          'position':objPosition,
                          'radius':self.convert.Meters2Pixels(objRadius),
                          'mass':systemMass/2,
                          'friction': self.botFriction,
                          'color': (255,20,147,255),
                          'collisionType':1
                          }
            self.grabIt = Ball(**objKwargs) # When this is returned to 'Ball' for the dynamic object, then the above lines will need to be uncommented.
            
        ### Pot Field Setup
        if self.potFieldPresent:
            potFieldParams={
                            'a':self.convert.Meters2Pixels(objRadius*.8),
                            'b':self.convert.Meters2Pixels(objRadius*.8),
                            'px':objPosition[1],
                            'py':objPosition[0],
                            'theta':0,
                            'fieldLen':self.convert.Meters2Pixels(self.R*72),
                            'res':self.convert.Meters2Pixels(0.1)}
            self.phi = analyticField2(**potFieldParams)
            
            PWM = 255 # must be a number 0 < PWM < 255
            w = 5     # Frequency in hertz of PWM
            tn = (PWM/255)/w
            potControlParams={'environment':self,
                              'w':w,
                              'tn':tn,
                              'phi':self.phi, # The field itself
                              'alpha':1,      # Gain of potfield. THIS IS THE FORCE IT WILL APPLY TO EACH BOT
                              'beta':0}       # Damping term of potfield
            
            self.potControl = PotControl2(**potControlParams)
        
        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness,
                    'tunnel':self.tunnel,
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
        
        
        ac = np.array([1]*2*self.numBots) # Take no action
        observation, _, self.previousDistance = self.getOb(ac)
        return observation
    
    
    
    
        
    def step(self, ac):
        # Take action
        forcesX = []
        forcesY = []
        for index in range(self.numBots):
            
            actionX = ac[2*index]
            actionY = ac[2*index+1]
            
            # Since the action_ will be [0,1,2], simply subtract 1. This 0 indicates move left, 1 no move, 2 move right
            xForce = actionX - 1 
            yForce = actionY - 1
            
            forcesX.append(xForce)
            forcesY.append(yForce)
            
        forcesX = np.asarray(forcesX, dtype=np.float)
        forcesY = np.asarray(forcesY, dtype=np.float)
            
        if self.potFieldPresent:
            potForceX, potForceY = self.potControl.run_controller()
            forcesX += potForceX
            forcesY += potForceY
            
        for index, bot in enumerate(self.bots):
            xForce = forcesX[index]
            yForce = forcesY[index]
            
            botPos = bot.body.position
            bot.body.apply_force_at_world_point((xForce, yForce), (botPos.x, botPos.y))
            
        # Contact information for storing
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
            
        # Taking a step in the environment
        self.space.step(self.dt)
        self.timestep+=1
        self.time += self.dt
            
        # Gather information
        obs, systemCenter, distanceToTarget = self.getOb(ac)
        botDisToObj = 0
        if self.grabObjectPresent:
            botInfo = obs[:self.numBots*2]
            botPosToObj = np.zeros((self.numBots,2))
            for bot in range(self.numBots):
                botPosToObj[bot,:] = botInfo[2*bot]*(self.targetDistance+2*self.R), botInfo[2*bot+1]*self.height # Must reverse the normalization factor
            botPosFromObj = np.mean(botPosToObj,axis=0)
            botDisToObj = np.linalg.norm([botPosFromObj[0],botPosFromObj[1]])
            
        rew = self.calcRew(distanceToTarget, botDisToObj)
        isDone, rew = self.isDone(rew, systemCenter, distanceToTarget)
        self.previousDistance = distanceToTarget
        if self.grabObjectPresent:
            self.prevBotDisToObj = botDisToObj
        
        if self.dataCollect: self.dataCollection(ac,rew,obs)
 
        return obs, rew, isDone, self.info
    
    
    
    
    
    def getOb(self, ac):
        
        runTime = [self.timestep/self.maxNumSteps]
        
        botPos = np.zeros((self.numBots//self.numSkip,2))
        botVel = np.zeros((self.numBots//self.numSkip,2))
        i=0
        for index, bot in enumerate(self.bots):
            if index%self.numSkip==0:
                currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
                currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            
                if self.grabObjectPresent:
                    grabItPos = self.convert.Pixels2Meters(self.grabIt.body.position.x), self.convert.Pixels2Meters(self.grabIt.body.position.y)
                    grabItVel = self.convert.Pixels2Meters(self.grabIt.body.velocity.x), self.convert.Pixels2Meters(self.grabIt.body.velocity.y)
                    botPos[index,:] = np.array(grabItPos) - np.array(currentBotPos)
    
                else:
                    botPos[i,:] = np.array(self.targetLoc) - np.array(currentBotPos)
                    
                    
                botVel[i,:] = currentBotVel
                i+=1
        
        if self.grabObjectPresent:
            systemCenter = np.array(self.targetLoc) - np.array(grabItPos)
        else:
            systemCenter = np.mean(botPos, axis=0)
        
        distanceToTarget = np.linalg.norm(systemCenter) # This value is in meters
        
        botForces = np.zeros((self.numBots//self.numSkip)*2)
        i=0
        for index, action in enumerate(ac):
            if index%self.numSkip==0:
                botForces[i] = action-1
                i+=1
        
        # Normalizing observation
        botPos[:,0]=botPos[:,0]/(self.targetDistance+2*self.R) #Normalizing X-Coordinate
        botPos[:,1]=botPos[:,1]/(self.height)                  #Normalizing Y-Coordinate
        botVel /= self.maxVelocity                             #Normalizing Velocity
        
        externalX = []
        externalY = []
        for index in range(self.numBots):
            if index%self.numSkip==0:
                externalX.append(self.extForcesX[index])
                externalY.append(self.extForcesY[index])
        
        extForces = np.abs(np.concatenate((externalX, externalY)))
        norm = np.linalg.norm(extForces)
        if norm != 0: extForces/= norm
        
        observation = np.concatenate((botPos.flatten(), botVel.flatten(), botForces, extForces, runTime))
        if self.grabObjectPresent:
            observation = np.concatenate((observation, systemCenter, grabItVel))
        return observation, systemCenter, distanceToTarget
    
    
    
    
    
    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        self.extForcesX[botIndex-2] = impulse[0]
        self.extForcesY[botIndex-2] = impulse[1]
    
    
    
    
    
    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True
    
    
    
    
    
    def calcRew(self, distanceToTarget, botDisToObj):
        progress = self.previousDistance - distanceToTarget # Relative to the velocity or speed for arriving at target
        
        # closer = ((self.targetDistance/self.targetDistance) - (distanceToTarget/self.targetDistance))*10
        
        botsToBallProgress = 0
        if self.grabObjectPresent:
            # botsToBallProgress = self.prevBotDisToObj - botDisToObj
            pass

        rew = progress*2000 + botsToBallProgress*500
        # Removing penalties, as they are not helping.
        # if botDisToObj>self.R*2:
        #     rew -= .1
            
        return rew
    
    
    def isDone(self, rew, systemCenter, distanceToTarget):
        """
        Can return later on an add penalties for taking too long or surpassing the target
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
        if self.saveVideo and self.timestep%10==0:
            pygame.image.save(self.screen, self.videoFolder+'image%06d.jpg' % self.timestep)
        self.clock.tick()
        printProgressBar(40,self.timestep,self.maxNumSteps)
    
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
        
        if self.grabObjectPresent:
            objPosTemp = [self.time]
            objVelTemp = [self.time]
            
            for index, pos in enumerate(self.grabIt.body.position):
                objPosTemp.append(self.convert.Pixels2Meters(pos)-self.targetLoc[index])
            for vel in self.grabIt.body.velocity:
                objVelTemp.append(self.convert.Pixels2Meters(vel))
            
            self.objPos = np.vstack([self.objPos,np.asarray(objPosTemp)])
            self.objVel= np.vstack([self.objVel, np.asarray(objVelTemp)])
            
        
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
        
        if self.grabObjectPresent:
            self.objPos = np.delete(self.objPos,0, 0)
            self.objVel = np.delete(self.objVel, 0, 0)
            
            np.savetxt(self.saveFolder + 'objectPosition.csv',self.objPos, delimiter=',')
            np.savetxt(self.saveFolder + 'objectVelocity.csv',self.objVel,delimiter=',')
        
        self.plot_data()
        self.parameterExport(self.saveFolder)
        
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
        
        # Plot Object information, if present
        if self.grabObjectPresent:
            plt.figure('ObjectPosition')
            plt.plot(time,self.objPos[:,1],label='X-Pos')
            plt.plot(time,self.objPos[:,2],label='Y-Pos')
            plt.xlabel(xlabel)
            plt.ylabel('Position (rel to target) [m]')
            plt.title('Object Position')
            plt.legend(loc='best')
            plt.savefig(self.saveFolder+'objectPosition.jpg')
            
            plt.figure('ObjectVelocity')
            plt.plot(time,self.objVel[:,1],label='X-Vel')
            plt.plot(time,self.objVel[:,2],label='Y-Vel')
            plt.xlabel(xlabel)
            plt.ylabel('Velocity [m/s]')
            plt.title('Object Velocity')
            plt.legend(loc='best')
            plt.savefig(self.saveFolder+'objectVelocity.jpg')
            
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
        
class squareObstacle:
    def __init__(self, space, position, length, friction, rotation=0, color = (0,0,0,255)):
        self.body = pymunk.Body(body_type = pymunk.Body.STATIC)
        self.body.position=position
        self.body.angle = rotation
        self.length = length
        self.position = position
        self.friction = friction
        
        # Creating the sides
        left = pymunk.Segment(self.body, (-length/2,-length/2), (-length/2,length/2), 5)
        right = pymunk.Segment(self.body, (length/2,-length/2), (length/2,length/2), 5)
        top = pymunk.Segment(self.body, (-length/2,length/2), (length/2,length/2), 5)
        bottom = pymunk.Segment(self.body, (-length/2,-length/2), (length/2,-length/2), 5)
        self.segments = [left,right,top,bottom]
        
        # Changing properties of body
        for seg in self.segments:
            seg.friction = friction
            seg.color = color
            seg.collision_type=1
        # Add to space 
        space.add(self.body,left,right,top,bottom)
        

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
            
    return bots, interiorParticles


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
    
    if tunnel:
        funnelStart = screenWidth//8
        gap = 20 # In pixels. This is still a somewhat arbitrary number.
        #Obstacle  1
        obst1 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
        vs1 = [(funnelStart,screenHeight),((screenWidth/6),screenHeight),((funnelStart),(screenHeight//2)+(wallThickness*gap)),((screenWidth/6),(screenHeight//2)+(wallThickness*gap))]
        obst_shape1 = pymunk.Poly(obst1,vs1)
        obst_shape1.collision_type=1
        
        #Obstacle  2
        obst2 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
        vs2 = [(funnelStart,0),((screenWidth/6),0),((funnelStart),(screenHeight//2)-(wallThickness*gap)),((screenWidth/6),(screenHeight//2)-(wallThickness*gap))]
        obst_shape2 = pymunk.Poly(obst2,vs2)
        obst_shape2.collision_type=1
    
        space.add(obst1, obst_shape1, obst2, obst_shape2)
    
    if env.potFieldPresent:
        wallCenter = env.convert.Meters2Pixels(env.systemStart[0] + 4*env.R), env.convert.Meters2Pixels(env.systemStart[1])
        R_pixels = env.convert.Meters2Pixels(env.R)
        theta = 15*(np.pi/180)
        
        inWay = pymunk.Body(0,0,body_type= pymunk.Body.STATIC)
        inWayShape = pymunk.Poly.create_box(inWay, (wallThickness*4, R_pixels*4))
        inWayShape.body.position = wallCenter[0] , wallCenter[1]
        inWayShape.collision_type = 1
        
        # topBarrier = pymunk.Segment(inWay,                                                                                 # Body it is attached to
        #                             (wallCenter[0], wallCenter[1]+R_pixels*2),                                             # Point a of segment
        #                             (wallCenter[0]-3*R_pixels*cos(theta), wallCenter[1]+R_pixels*2+3*R_pixels*sin(theta)), # Point b of segment
        #                             wallThickness*4)    
        topBarrier = pymunk.Segment(inWay,                                                                                 # Body it is attached to
                                    (0, R_pixels*2),                                             # Point a of segment
                                    (-3*R_pixels*cos(theta), R_pixels*2+3*R_pixels*sin(theta)), # Point b of segment
                                    wallThickness*4)                                                                       # Wall Thickness
        topBarrier.collision_type = 1
        
        # bottomBarrier = pymunk.Segment(inWay, 
        #                                (wallCenter[0], wallCenter[1]-R_pixels*2), 
        #                                (wallCenter[0]-3*R_pixels*cos(theta), wallCenter[1]-R_pixels*2-3*R_pixels*sin(theta)), 
        #                                wallThickness*4)
        bottomBarrier = pymunk.Segment(inWay, 
                                    (0, -R_pixels*2), 
                                    (-3*R_pixels*cos(theta), -R_pixels*2-3*R_pixels*sin(theta)), 
                                    wallThickness*4)
        bottomBarrier.collision_type = 1
        
        # space.add(inWay, inWayShape, topBarrier, bottomBarrier)
        
        
    
    return None


def createVideo(saveLoc, imgLoc, videoName, imgShape):
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 80, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
    rmtree(imgLoc)
    
def createObstacleField(space, obstacleCoordinates, obsRadius, friction, square=False):

        obstacles = []
        for coord in obstacleCoordinates:
            posX = coord[0]+obsRadius*2
            posY = coord[1]+obsRadius
            
            if square: obstacle = squareObstacle(space,(posX,posY), obsRadius, friction, (np.pi/2)*np.random.rand())
            else: obstacle = Obstacle(space,(posX,posY),obsRadius,friction)
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
    
    
    
    
    
class PotControl:
    # TODO: Put variable=None for all the optional arguments
    def __init__(self, environment, w, tn, phi, alpha, beta):

        #### Initialize some parameters for all cases of the controller
        self.env = environment      # The environment above (PyMunk)
        self.w=w                    # frequency for PWM
        self.tn=tn                  # time interval for PWM
        self.T=0                    # internal time  for PWM
        
        #### Parameters needed from environment
        self.numBots = self.env.numBots  # Number of Bots
        self.tstep = self.env.dt         # Environment Timestep
        self.bots = self.env.bots
                   
        ##### POT_FIELD_GRAB PARAMETERS        
        self.phi=phi          # potential field object
        self.fnx=self.phi.fnx # gradient in x direction  
        self.fny=self.phi.fny # gradient in the y direction      
        self.alpha=alpha      # proportional gain 
        self.beta=beta        # damping term               
        
    # run controller
    def run_controller(self):
        self.get_position()            # get the position of the bots
        # self.get_velocity()            # get the velocity of the bots
        self.xbv = self.ybv = np.zeros(self.numBots)
        (FX,FZ)=self.grab_controller() # run the grab controller
        self.apply_force(FX,FZ)        # Gather whether we will apply force to bots, based on PWM
        
        return self.FX, self.FZ
        
    #  Grabbing a ball controller 
    def grab_controller(self):
        ''' This controller is used for grabbing a fixed object'''
        FX=[]
        FY=[]
        self.phi.py = self.env.grabIt.body.position.x
        self.phi.px = self.env.grabIt.body.position.y
        (_, self.fny, self.fnx) = self.phi.update_field_gradient()         
        for i in range(self.numBots):
            Fx=self.fny([self.xb[i],self.yb[i]])
            Fy=self.fnx([self.xb[i],self.yb[i]])
            mag=np.linalg.norm([Fx,Fy])
            FXX=Fx/mag
            FYY=Fy/mag
            fy=-self.alpha*FYY-self.beta*self.ybv[i] 
            fx=-self.alpha*FXX-self.beta*self.xbv[i]        
            FX.append(fx[0])
            FY.append(fy[0])
        return(np.asarray(FX),np.asarray(FY))
                   
    def apply_force(self, FX, FZ):  
        self.T=self.tstep+self.T
        if self.T>0 and self.T<=self.tn:
            self.FX=FX
            self.FZ=FZ
        else:
            self.FX=np.zeros(len(FX))
            self.FZ=np.zeros(len(FX))
            
        if self.T>(1/self.w):
            self.T=0
            
    # get current position         
    def get_position(self):
        self.xb=[]        
        self.yb=[]
        for i in range(self.numBots):
            self.xb.append(self.bots[i].body.position.x)
            self.yb.append(self.bots[i].body.position.y)
        
    # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.ybv=[]
        for i in range(self.numBots):
            self.xbv.append(self.bots[i].body.velocity.x)
            self.ybv.append(self.bots[i].body.velocity.y)
    
    
    
    
   
class analytic_field:
    def __init__(self,a,c,px,py,theta=0,b=0,res=0):
        self.type='Shape_Fields_analytic'
        self.px=px # center x
        self.py=py # center y
        self.a=a # radii 1
        self.b=b # range of field
        self.c=c # radii 2
        self.theta=theta # rotation
        self.res=res # resolution of the field so how big are the squares 

        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xt=np.linspace(-self.b,self.b,self.xcount)
        self.yt=np.linspace(-self.b,self.b,self.ycount)
        self.xxt,self.yyt=np.meshgrid(self.xt,self.yt) # create grid for the field 
        self.xx=0
        self.yy=0
        Y=(self.yyt)#*np.sin(self.theta)+(self.yyt)*np.cos(self.theta) # Y value of distance
        X=(self.xxt)#*np.cos(self.theta)-(self.yyt)*np.sin(self.theta) # X value of distance
        d=np.sqrt((X**2) / (self.a**2) + (Y**2)/(self.c**2)) # d value 
        self.zz = d**2 * np.log(d) # create field grid form 
        self.zz=self.zz/np.max(self.zz)
        (self.fy,self.fx)=np.gradient(self.zz**2) # take gradient 
        self.fx=self.fx/np.max(self.fx) # normaluze gradient x term
        self.fy=self.fy/np.max(self.fy) # normalize gradient y term 
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz) # form potential function 
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx) # form gradient of potential function x
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)  # form gradient of potential function y

    def update_field_gradient(self):
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field 
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz) # form potential function 
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx) # form gradient of potnetial function x
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)  # form gradient of potential function y
        return(self.f,self.fny,self.fnx)
    
    
class analyticField2:
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
        first = (((-px+x)*sin(phi)+(-py+y)*cos(phi))**2)/(b**2)
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
        self.fny =self.df2y
        return None
        

class PotControl2:
    # TODO: Put variable=None for all the optional arguments
    def __init__(self, environment, w, tn, phi, alpha, beta):

        #### Initialize some parameters for all cases of the controller
        self.env = environment      # The environment above (PyMunk)
        self.w=w                    # frequency for PWM
        self.tn=tn                  # time interval for PWM
        self.T=0                    # internal time  for PWM
        
        #### Parameters needed from environment
        self.numBots = self.env.numBots  # Number of Bots
        self.tstep = self.env.dt         # Environment Timestep
        self.bots = self.env.bots
                   
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
        (FX,FZ)=self.grab_controller() # run the grab controller
        self.apply_force(FX,FZ)        # Gather whether we will apply force to bots, based on PWM
        return self.FX, self.FZ
        
    #  Grabbing a ball controller 
    def grab_controller(self):
        ''' This controller is used for grabbing a fixed object'''
        FX=[]
        FY=[]
        px = self.env.grabIt.body.position.x
        py = self.env.grabIt.body.position.y     
        for i in range(self.numBots):
            Fy=self.fny(self.xb[i], self.yb[i], px, py, 0 )
            Fx=self.fnx(self.xb[i],self.yb[i], px, py, 0 )
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
        if self.T>0 and self.T<=self.tn:
            self.FX=FX
            self.FZ=FZ
        else:
            self.FX=np.zeros(len(FX))
            self.FZ=np.zeros(len(FX))
            
        if self.T>(1/self.w):
            self.T=0
            
    # get current position         
    def get_position(self):
        self.xb=[]        
        self.yb=[]
        for i in range(self.numBots):
            self.xb.append(self.bots[i].body.position.x)
            self.yb.append(self.bots[i].body.position.y)
        
    # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.ybv=[]
        for i in range(self.numBots):
            self.xbv.append(self.bots[i].body.velocity.x)
            self.ybv.append(self.bots[i].body.velocity.y)