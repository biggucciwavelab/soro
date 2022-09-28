# -*- coding: utf-8 -*-
"""
Created on Sun Apr 25 18:13:31 2021

@author: elope
"""



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
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.interpolate import RegularGridInterpolator
from shutil import rmtree
import glob # For creating videos
import cv2 # For creating videos
from warnings import warn
import os
import pdb

class singleRobot:
    
    SpenkMe = False
    
    def __init__(self, dt, ppm, numStepsPerStep, screenHeight, screenWidth, maxNumSteps, botMass, botRadius,botFriction, wallThickness, dataCollect=False, experimentName="NOT NAMED", saveVideo = False, systemStart = None):
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
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        
        # System membrane parameters
        self.botMass = botMass
        self.botRadius = botRadius     # Radius of 
        self.botFriction = botFriction # Friction of bots and skins
        self.state_size = 8 + 1        # X-Pos, Y-Pos, X-Vel, Y-Vel, Fx, Fy, External_Fx, External_Fy
        
        # Paramaters for wall and space
        self.wallThickness = wallThickness
        self.maxNumSteps = maxNumSteps
        
        # NOTE: SystemStart is now (x,y,theta)!!
        if systemStart==None:
            self.systemStart = self.width/2, self.height/2, 0
        else:
            self.systemStart = systemStart
            assert len(systemStart)==3, 'The System Start should now be 3 variables: (x,y,theta)'
        
        
        #### Data Collection
        self.environment_parameters = [['PixelsPerMeter (ppm)',str(self.ppm)],
                                       ['Screen_Width',str(self.width)],
                                       ['Screen_Height',str(self.height)],
                                       ['Bot_Radius:', str(self.botRadius)],
                                       ['Bot_Mass',str(self.botMass)],
                                       ['Bot_Friction',str(self.botFriction)],
                                       ['System_Start',str(systemStart)]]
        
        self.dataCollect = dataCollect
        if self.dataCollect:
            now = str(datetime.now())
            now = now.replace(":","")
            now = now[:-7]
            
            self.saveFolder = experimentName+ " Data and Plots "+now+"/"
            os.makedirs(self.saveFolder,exist_ok=True)
            
            
            # This +1 is for the extra column needed to record time.
            self.X_data = np.zeros(2)     # X-Pos, Time
            self.X_vel_data = np.zeros(2) # X-Vel, Time
            self.Y_data = np.zeros(2)     # Y-Pos, Time
            self.Y_vel_data = np.zeros(2) # Y-Vel, Time
            self.ac = np.zeros(3)         # Fx, Fy, time
            self.obs_data = np.zeros(self.state_size + 1)
        
        if self.saveVideo:
            self.videoFolder = experimentName + '_VideoImages/'
            os.makedirs(self.videoFolder,exist_ok=True)
        
        return None
        
        
        
        
        
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
        self.extForcesX = np.zeros(1)
        self.extForcesY = np.zeros(1)
        
        # Converting units to pixel coordinates before feeding into space for creation
        systemStart = self.convert.Meters2Pixels(self.systemStart[0]), self.convert.Meters2Pixels(self.systemStart[1]), self.systemStart[2]
        botRadius = self.convert.Meters2Pixels(self.botRadius)
        height = self.convert.Meters2Pixels(self.height)
        width = self.convert.Meters2Pixels(self.width)
        wallThickness = self.convert.Meters2Pixels(self.wallThickness)
        
        #### Creating single robot
        botKwargs = {'space': self.space,
                     'position':systemStart,
                     'radius':botRadius,
                     'mass':self.botMass,
                     'friction':self.botFriction,
                     'collisionType':2} # The collision type of all other objects should be 1!!
        self.robot = Ball(**botKwargs)
            
        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness,
                    'env':self}
        createWalls(**wallKwargs) # Not creating the walls right now.
        
        
        #### Collision Handler
        # Reports collisions with walls, objects, and obstacles
        cHandler = self.space.add_collision_handler(1,self.robot.shape.collision_type)
        cHandler.post_solve = self.colPost
        
        
        ac = np.zeros(2) # Take no action
        observation = self.getOb(ac)
        return observation
    
    
    
    
        
    def step(self, ac):
        """
        Input 'ac' should be a vector describing the force that each bot will apply in the x and y direction, respectively.
        Ordered as follows: Fx = 
        """   
        # Contact information for storing
        self.extForcesX = np.zeros(1)
        self.extForcesY = np.zeros(1)
        
        for i in range(self.numStepsPerStep):         
            vel = ac[0]
            omega = ac[1]
            
            # Convert the velocity to PyMunk coordinates
            vel = self.convert.Meters2Pixels(vel)
            
            #Convert the velocity to be in the direction of the bot's heading angle
            botAngle = self.robot.body.angle
            velX = vel*cos(botAngle)
            velY = vel*sin(botAngle)
            
            self.robot.body.velocity = velX, velY
            self.robot.body.angular_velocity = omega
                          
            # Taking a step in the environment
            self.space.step(self.dt)
            self.time += self.dt
        self.timestep+=1
        
        # Gather information
        obs = self.getOb(ac)            
        isDone = self.isDone()
        
        if self.dataCollect: self.dataCollection(ac,obs)
 
        return obs, isDone
    
    
    
    
    
    def getOb(self, ac):
        
        runTime = [self.timestep/self.maxNumSteps]
        
        # Gathering bot location and velocity
        currentBotPos = self.convert.Pixels2Meters(self.robot.body.position.x), self.convert.Pixels2Meters(self.robot.body.position.y)
        currentBotVel = self.convert.Pixels2Meters(self.robot.body.velocity.x), self.convert.Pixels2Meters(self.robot.body.velocity.y)
        
        # Putting information into a numpy array for concatenation
        botPos = np.array(currentBotPos)
        botVel = np.array(currentBotVel)
        botForces = np.array(ac)
        extForces = np.abs(np.array([self.extForcesX, self.extForcesY]))
        norm = np.linalg.norm(extForces)
        if norm != 0: extForces/= norm
        
        observation = np.concatenate((botPos.flatten(), botVel.flatten(), botForces, extForces.flatten(), runTime))

        return observation
    
    
    
    
    
    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        self.extForcesX = np.array(impulse[0])
        self.extForcesY = np.array(impulse[1])
    
    
    
    
    
    def colPost(self, arbiter, space, data):
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)
        return True
    
    
    
    
    
    def isDone(self):
        """
        Add information to denote if the simulation should terminate early
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
            self.drawOptions.shape_outline_color = (0,0,0,255)
            
            pymunk.pygame_util.positive_y_is_up=True
            
            if self.SpenkMe:
                self.img = pygame.image.load('Spenks.jpg')
                self.img.convert()
                self.rect = self.img.get_rect()
                self.rect.center = screenWidth//2, screenHeight//2
            self.render_setup=True
            
        for event in pygame.event.get():
            if event.type==pygame.QUIT: 
                pygame.display.quit()
                self.close()
            if event.type==pygame.KEYDOWN and event.key == pygame.K_ESCAPE: 
                pygame.display.quit()
                self.close()
        
        
        self.screen.fill((192,192,192))
        if self.SpenkMe:
            self.screen.blit(self.img, self.rect)
            pygame.draw.rect(self.screen, (251,251,251), self.rect, 1)
        self.space.debug_draw(self.drawOptions)
        pygame.display.update()
        if self.saveVideo and self.timestep%5==0:
            pygame.image.save(self.screen, self.videoFolder+'image%06d.jpg' % self.timestep)
        self.clock.tick()
        printProgressBar(40,self.timestep,self.maxNumSteps)
    
    
    
    
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
        
        currentBotPos = obs[:2]
        currentBotVel = obs[2:4]
            
        X_Pos_temp.append(currentBotPos[0])
        Y_Pos_temp.append(currentBotPos[1])
        
        X_vel_temp.append(currentBotVel[0])
        Y_vel_temp.append(currentBotVel[1])
            
        for action in ac:
            action_temp.append(action)
            
        for observation in obs:
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
        
    def plot_data(self):
        # Common Among Position and Velocity Data
        time = self.X_data[:,0]
        xlabel = 'Time [sec]'
        
        # Plot X-Position
        plt.figure('X-Pos')
        plt.plot(time,self.X_data[:,1])
        plt.xlabel(xlabel)
        plt.ylabel('X-Position [m]')
        plt.title('X-Position')
        plt.savefig(self.saveFolder + 'X-Center Position.jpg')       
        
        # Plot Y-Position
        plt.figure('Y-Pos')
        plt.plot(time,self.Y_data[:,1])
        plt.xlabel(xlabel)
        plt.ylabel('Y-Position [m]')
        plt.title('Y-Position')
        plt.savefig(self.saveFolder + 'Y-Center Position.jpg')
        
        # Plot X-velocity
        plt.figure('X-Vel')
        plt.plot(time, self.X_vel_data[:,1])
        plt.xlabel(xlabel)
        plt.ylabel('X Velocity [m/s]')
        plt.title('X-Velocity')
        plt.savefig(self.saveFolder + 'X-Velocity.jpg')
        
        # Plot Y-Velocity
        plt.figure('Y-Vel')
        plt.plot(time, self.Y_vel_data[:,1])
        plt.xlabel(xlabel)
        plt.ylabel('Y Velocity [m/s]')
        plt.title('Y-Velocity')
        plt.savefig(self.saveFolder + 'Y-Velocity.jpg')        
            
        # Plot Actions
        last_col_2 = len(self.ac[0])-1
        bot=1
        for i in range(last_col_2):
            if i%2!=0:
                plt.figure('Applied Acctuation Bot ' + str(bot))
                plt.plot(time, self.ac[:,i], label='Velocity')
                plt.plot(time, self.ac[:,i+1], label='Angular Velocity')
                plt.xlabel(xlabel)
                plt.ylabel('[m/s] and [rad/s]')
                plt.title('Applied Acctuation on Bot ' + str(bot))
                plt.legend(loc='lower right')
                plt.savefig(self.saveFolder + 'Bot ' + str(bot) + ' Applied Actuation.jpg')
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
    def __init__(self, space, position, radius, mass, friction, collisionType = 0, color = (0,255,0,255)):
        self.body = pymunk.Body()
        self.radius = radius
        self.body.position = position[:2]
        self.body.angle = position[2]
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.mass = mass
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = collisionType
        space.add(self.body, self.shape)





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
    body4 = pymunk.Body(0,0,body_type = pymunk.Body.STATIC)
    shape4 = pymunk.Poly.create_box(body4, (wallThickness, screenHeight))
    shape4.body.position = (screenWidth,screenHeight//2)
    shape4.collision_type = 1
    
    space.add(shape1,body1,shape2,body2,shape3,body3,shape4,body4)
    return None





def createVideo(saveLoc, imgLoc, videoName, imgShape):
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 80, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
    rmtree(imgLoc)
        
    
    
    
    
def printProgressBar(length, iteration, total,fill='=', prefix='', suffix='Complete', printEnd='\r'):
    percent = ("{0:." + str(1) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)