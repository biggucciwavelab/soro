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
from math import floor
from gym import spaces, Env
import matplotlib.pyplot as plt
from datetime import datetime
from shutil import rmtree
import glob # For creating videos
import cv2 # For creating videos
from warnings import warn
import os
import pdb

# from JAMoEBA_utils import createObst

class pymunkEnv(Env):
    
    info = {'There is no info to be aware of.':'Machinelearn away.'}
    
    def __init__(self, dt, ppm, screenHeight, screenWidth, maxNumSteps, R, numBots, botMass, botRadius, skinRadius, skinMass, skinRatio, inRadius, botFriction, inMass, inFriction, percentInteriorRemove, springK, springB, springRL, wallThickness, maxSeparation, targetDistance, dataCollect=False, experimentName="NOT NAMED", saveVideo = False):
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
        self.maxVelocity = 5000                                      # Arbitrarily set, may need changing later.
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName
        
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
        self.targetLoc = self.systemStart[0]+targetDistance, self.systemStart[1]+targetDistance/4 # Located directly down the x-axis
        self.targetLoc_x = self.systemStart[0]+targetDistance
        self.targetLoc_y = self.systemStart[1]+targetDistance/4
        
        
        #### Gym API
        self.state_size = self.numBots*6+1 # botRePosition (x,y), botVel (x,y), botCurrentAction(x,y)
        self.action_size = self.numBots*2
        
        low = np.full(self.state_size, -10)
        high = np.full(self.state_size, 10)
        self.observation_space = spaces.Box(low,high,dtype=np.float32)
        
        self.action_space = spaces.MultiDiscrete([3]*2*self.numBots) # Will return a 0, 1, or 2at every timestep. This will correspond to positive, negative, or zero force applied.
        
        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius)
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
                                       ['percentInteriorRemoved',str(self.percentInteriorRemove)]]
        
        self.contactPoints = [] # For storing contact points throughout time
        
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
            'botCollisionIntStart' : 2} # All obstacles will be of collision_type=1
        self.bots, interiorParticles = createJamoeba(**kwargs)
        
        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness}
        self.wall1, self.wall2 = createWalls(**wallKwargs) 
        
        ac = np.array([1]*2*self.numBots) # Take no action
        observation, _, self.previousDistance,self.previousForce = self.getOb(ac)
        return observation
        
        #### Create  the  Obstacle
        # obstKwargs={'space':self.space,
        #          'screenHeight':height,
        #          'screenWidth':width}
        # createObst(**obstKwargs)  #Creating  the  obstacle
    
        #### Collision handler
        for bot in self.bots:
            cHandler = self.space.add_collision_handler(1,bot.shape.collision_type)
            cHandler.post_solve = self.colPost
    
    
    def step(self, ac):
        # Take action
        for index, bot in enumerate(self.bots):
            
            actionX = ac[2*index]
            actionY = ac[2*index+1]
            
            # Since the action_ will be [0,1,2], simply subtract 1. This 0 indicates move left, 1 no move, 2 move right
            xForce = actionX - 1 
            yForce = actionY - 1
            
            botPos = bot.body.position
            
            bot.body.apply_force_at_world_point((xForce, yForce), (botPos.x, botPos.y))
            
        self.space.step(self.dt)
        self.timestep+=1
        self.time += self.dt
            
        # Gather information
        obs, systemCenter, distanceToTarget, sysNormForce = self.getOb(ac)
        rew = self.calcRew(distanceToTarget, sysNormForce)
        isDone, rew = self.isDone(rew, systemCenter, distanceToTarget)
        self.previousDistance = distanceToTarget
        self.previousForce = sysNormForce
        # self.previousDistance_x = systemCenter_x
        # self.previousDistance_y = systemCenter_y
        
        
        if self.dataCollect: self.dataCollection(ac,rew,obs)
 
        return obs, rew, isDone, self.info
    
    
    
    
    
    def getOb(self, ac):
        
        runTime = [self.timestep/self.maxNumSteps]
        
        botPos = np.zeros((self.numBots,2))
        # botPos_x = np.zeros((self.numBots,1))
        # botPos_y = np.zeros((self.numBots,1))
        botVel = np.zeros((self.numBots,2))
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            # currentBotPos_x = self.convert.Pixels2Meters(bot.body.position.x)
            # currentBotPos_y = self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            botPos[index,:] = np.array(currentBotPos) - np.array(self.targetLoc)
            # botPos_x = np.array(currentBotPos_x) - np.array(self.targetLoc_x) #Finding  the  x  pos  of  each  bot
            # botPos_y = np.array(currentBotPos_y) - np.array(self.targetLoc_y) #Finding  the  y  pos  of  each  bot
            botVel[index,:]        = currentBotVel
        
        systemCenter = np.mean(botPos, axis=0)
        # systemCenter_x = np.mean(botPos_x, axis=0) #Mean  x  pos  of  the  bots
        # systemCenter_y = np.mean(botPos_y, axis=0) #Mean  y  pos  of  the  bots
        distanceToTarget = np.linalg.norm(systemCenter) # This value is in meters
        
        botForces = np.zeros(self.numBots*2)
        for index, action in enumerate(ac):
            botForces[index] = action-1
        
        sysForce = np.mean(botForces, axis = 0) # Calculating  the  mean  force  on  all  bots
        sysNormForce = np.linalg.norm(sysForce) # Value  is  in  Newtons
        
        # Normalizing observation
        botPos[:,0]=botPos[:,0]/(self.targetDistance+2*self.R) #Normalizing X-Coordinate
        botPos[:,1]=botPos[:,1]/(self.height)                  #Normalizing Y-Coordinate
        botVel /= self.maxVelocity                             #Normalizing Velocity
        

        
        observation = np.concatenate((botPos.flatten(), botVel.flatten(), botForces, runTime))
        return observation, systemCenter, distanceToTarget, sysNormForce            
            
            
    def colPost(self, arbiter, space, data):
        print('Contact Detected')
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        
        # Reporting contact
        if arbiter.is_first_contact:
            for contact in arbiter.contact_point_set.points:
                contactLoc = self.convert.Pixels2Meters(contact.point_a)
                self.contactPoints.append(contactLoc)
        
        return True
        
    
    
    # def power_consumption(self, distanceToTarget, sysForce):            
    #     """
    #     Power Consumption - How much energy was used in this step?
    #     Multiples F*dx
    #     """
        
    #     # past_x = np.mean(self.X_Positions[:self.midway][:],axis=0)
    #     # current_x = np.mean(self.X_Positions[self.midway:][:],axis=0)
        
    #     # past_z = np.mean(self.Z_Positions[:self.midway][:],axis=0)
    #     # current_z = np.mean(self.Z_Positions[self.midway:][:],axis=0)
        
    #     # past_X_actions = np.mean(self.X_Actions[self.midway:][:],axis=0)
    #     # current_X_actions = np.mean(self.X_Actions[:self.midway][:],axis=0)
        
    #     # past_Z_actions = np.mean(self.Z_Actions[self.midway:][:],axis=0)
    #     # current_Z_actions = np.mean(self.Z_Actions[:self.midway][:],axis=0)
    
    #     # dx = current_x - past_x
    #     # dz = current_z - past_z
        
    #     # delta_force_x = current_X_actions - past_X_actions
    #     # delta_force_z = current_Z_actions - past_Z_actions
        
    #     # Px = delta_force_x*dx
    #     # Pz = delta_force_z*dz
        
    #     dx = self.previousDistance - distanceToTarget
    #     df = self.previousForce - sysForce
    #     power_used = df*dx
        
    #     # power_used = sum(abs(Px)) + sum(abs(Pz))
    #     # self.tot_power += power_used
        
    #     return power_used
    
    def calcRew(self, distanceToTarget, sysNormForce):
        progress = self.previousDistance - distanceToTarget # Relative to the velocity or speed for arriving at target
        # progress_x = self.previousDistance_x - systemCenter_x
        # progress_y = self.previousDistance_y - systemCenter_y
        
        df = self.previousForce - sysNormForce
        power_used = df*progress
        # closer = ((self.targetDistance/self.targetDistance) - (distanceToTarget/self.targetDistance))*10

        rew = progress*500 - power_used*200
        return rew
    
    
    def isDone(self, rew, systemCenter, distanceToTarget):
        """
        Can return later on an add penalties for taking too long or surpassing the target
        """
        done=False
        if self.timestep>self.maxNumSteps:
            done=True
            
        if systemCenter[0] > self.targetLoc[0]+self.R*2:
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
                sys.exit()
            if event.type==pygame.KEYDOWN and event.key == pygame.K_ESCAPE: 
                pygame.display.quit()
                sys.exit()
        
        
        self.screen.fill((192,192,192))
        self.space.debug_draw(self.drawOptions)
        pygame.display.update()
        if self.saveVideo:
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
        
    def parameterExport(self, saveLoc=None):
        if saveLoc==None:
            parameter_file=  self.experimentName + '_Environment_parameters.txt'
            print('\n','--'*20)
            warn('No Save location for environment parameters has been specificed')
            print('--'*20,'\n')
        else:
            os.makedirs(saveLoc,exist_ok=True)
            parameter_file = saveLoc+ self.experimentName+'_Environment_parameters.txt'
        
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
        
        # Converting contact points to numpy array
        self.contactPoints = np.array(self.contactPoints)
        
        # Save the data on .csv files
        np.savetxt(self.saveFolder + 'X_data.csv', self.X_data, delimiter=',')
        np.savetxt(self.saveFolder + 'X_vel_data.csv', self.X_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_data.csv', self.Y_data, delimiter=',')
        np.savetxt(self.saveFolder + 'Y_vel_data.csv', self.Y_vel_data, delimiter=',')
        np.savetxt(self.saveFolder + 'actions.csv', self.ac, delimiter=',')
        np.savetxt(self.saveFolder + 'reward.csv', self.reward_data, delimiter=',')
        np.savetxt(self.saveFolder + 'observations.csv', self.obs_data, delimiter=',')
        np.savetxt(self.saveFolder + 'contactPoints.csv', self.contactPoints, delimiter=',')
        
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
        plt.ylabel('X-Position [m]')
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
        plt.ylabel('Y-Position [m]')
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
    def __init__(self, space, position, radius, mass, friction, collisionType=0, color = (0,255,0,255)):
        self.body = pymunk.Body()
        self.radius = radius
        self.body.position = position
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.mass = mass
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = collisionType
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
    thetaMaster = (2*np.pi/numBots)/(skinRatio+1)
    
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
            t = thetaMaster
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
    else: removePerRing = numRings//numRemove
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


def createWalls(space, screenHeight, screenWidth, wallThickness):
    # All values for this function are assumed to already be in Pixels
    sw = screenWidth
    sh = screenHeight
    wt = wallThickness
    # Bottom Wall
    # body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    # shape1 = pymunk.Poly.create_box(body1, (screenWidth,wallThickness))
    # shape1.body.position = (screenWidth//2,wallThickness//2)
    
    # Top Wall
    # body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    # shape2 = pymunk.Poly.create_box(body2, (screenWidth,wallThickness))
    # shape2.body.position = (screenWidth//2, screenHeight-wallThickness//2)
    
    # Back wall
    # body3 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    # shape3 = pymunk.Poly.create_box(body3,(wallThickness, screenHeight))
    # shape3.body.position = (wallThickness//2, screenHeight//2)
    
    # Obstacle  3
    # obst3 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    # vs3 = [(sw//4,sh),(sw//2,sh),(sw//4,(sh//2)+wt*25),(sw//2,(sh//2)+wt*25)]
    # obst_shape3 = pymunk.Poly(obst3,vs3)
    
    #Obstacle  4
    obst4 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    vs4 = [(sw//4,0),(sw,0),(sw//2,(sh//2)-wt*7),(sw,(sh//2)-wt*7)]
    obst_shape4 = pymunk.Poly(obst4,vs4)
    obst_shape4.collision_type = 1
    
    #Obstacle  1
    obst1 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    vs1 = [(sw//4,sh),(sw,sh),(sw//2,(sh//2)+wt*7),(sw,(sh//2)+wt*7)]
    obst_shape1 = pymunk.Poly(obst1,vs1)
    obst_shape1.collision_type = 1
    # obst_shape1.body.position = (screenWidth//2,screenHeight//2 + 220)
    
    # #Obstacle  2
    # obst2 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    # vs2 = [(sw//2,sh),(sw,sh),(sw//2,(sh//2)+wt*10),(sw,(sh//2)+wt*10)]
    # obst_shape2 = pymunk.Poly(obst2,vs2)
    # obst_shape2.body.position = (screenWidth//2,screenHeight//2 - 220)

    # Adding  the  obstacle  to  the  observation  space
    
    space.add(obst_shape1,obst1,obst_shape4,obst4)
    
    return obst_shape1, obst_shape4

    
def createVideo(saveLoc, imgLoc, videoName, imgShape):
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 20, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        height, width, layers = img.shape
        out.write(img)
    out.release
    
    return out


def printProgressBar(length, iteration, total,fill='=', prefix='', suffix='Complete', printEnd='\r'):
    percent = ("{0:." + str(1) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)