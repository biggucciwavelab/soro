"""
Use this in the future to add collision detection in the observation space:
    https://stackoverflow.com/questions/50815789/non-colliding-objects-which-has-colliding-pairs-pymunk-pygame
    
This example shows how to draw collisions:
    https://github.com/viblo/pymunk/blob/master/examples/contact_and_no_flipy.py
"""
"""
This environment does not have the slider joints which constrict the system from moving too much.
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
from warnings import warn
import os
import pdb

class pymunkEnv(Env):
    
    info = {'There is no info to be aware of.':'Machinelearn away.'}
    
    def __init__(self, dt, ppm, screenHeight, screenWidth, 
                maxNumSteps, R, numBots, botMass, botRadius, 
                skinRadius, skinMass, skinRatio, inRadius, 
                botFriction, inMass, inFriction, percentInteriorRemove, 
                springK, springB, springRL, wallThickness, maxSeparation, 
                dataCollect=False, experimentName="NOT NAMED", saveVideo = False, 
                energy=False, kineticEnergy=False, numStepsPerStep=1):
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
        self.gain=2
        self.dataCollect = dataCollect                               # Are we collecting data rn?
        self.saveVideo = saveVideo
        self.experimentName = experimentName                         # Experiment name. Is assigned to plots folder and video
        self.energy=energy                                           # If True, then we do care about calculating how much energy our system is expending to complete its mission
        self.numStepsPerStep = numStepsPerStep                       # The number of simulation timesteps to run for each call to 'step' function
        # self.maxVelSeen = 0
        
        
        # JAMoEBA system membrane parameters
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
        
        # Thickness of wall
        self.wallThickness = wallThickness

        # Start location of JAMoEBA system
        self.systemStart = self.width/2, self.height/2 # Start at center
        # self.systemStart = self.width + self.R + botRadius, self.height/2 # Uncomment if you want the JAMoEBA system to be out of the picture.
       
        # Define a reference point for all positions. In this case, the center.
        self.targetLoc = self.width/2, self.height/2
            
        #### Gym API
        self.state_size = self.numBots*5  # botRePosition (x,y), botVel (x,y), botCurrentAction(x,y)
        self.action_size = 2*self.numBots
        force_low, force_high = -1.0, 1.0 # Defining how large (and in each direction) a force may be applied by the agent

        low = np.full(self.state_size, -10)
        high = np.full(self.state_size, 10)
        self.observation_space = spaces.Box(low,high,dtype=np.float32)
        self.action_space = spaces.Box(low=force_low, 
                                        high=force_high, 
                                        shape=(self.action_size,),
                                        dtype=np.float32)
                
        #Gather information on number of interior
        granPerRing, _ = interiorPattern(self.R, self.inRadius, self.botRadius, self.percentInteriorRemove)
        self.numInterior = np.sum(granPerRing)
        
        #### Data Collection
        self.environment_parameters = [
                                        ['dt:',str(self.dt)],
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
                                       ['energy',str(energy)],
                                       ['kinetic energy',str(kineticEnergy)],
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
            self.X_data = np.zeros(self.numBots + 1)     # X-Position of each bot
            self.X_vel_data = np.zeros(self.numBots + 1) # X-Velocity of each bot
            self.Y_data = np.zeros(self.numBots + 1)     # Y-Position of each bot
            self.Y_vel_data = np.zeros(self.numBots + 1) # Y-Velocity of each bot
            self.ac = np.zeros(self.action_size + 1)     # Action taken on each bot
            self.reward_data = np.zeros(2)               # Reward signal sent to RL
            self.obs_data = np.zeros(self.state_size +1) # Observation at each time step
        
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
        self.contacts = np.zeros(self.numBots)

        # Getting forces on square
        self.extForceSquare = np.zeros(2)
        
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
        self.bots, self.membrane, interiorParticles, self.bindingSprings, self.bindingList \
            = createJamoeba(**kwargs)
    
        # Note: Membrane consists of bots and skin particles
        self.jamoeba = [*self.membrane, *interiorParticles]
        
        #### Create Walls
        wallKwargs={'space':self.space,
                    'screenHeight':height,
                    'screenWidth':width,
                    'wallThickness':wallThickness,
                    'env':self}
        _, _, _, wall4 = createWalls(**wallKwargs) 
        _, wall4_body = wall4
        self.wall_loc = self.convert.Pixels2Meters(np.asarray(wall4_body.position))
    
        #### Collision Handler
        # Reports collisions with walls, objects, and obstacles
        for bot in self.bots:
            cHandler = self.space.add_collision_handler(1,bot.shape.collision_type)
            cHandler.post_solve = self.colPost
        
        #### Initiate Observation
        ac = np.array([0]*2*self.numBots) # Take no action
        observation, systemCenter = self.getOb(ac)
        return observation
    
    
    
    
        
    def step(self, ac):
        
        # Contact information for storing
        self.extForcesX[:] = 0
        self.extForcesY[:] = 0
        self.contacts[:] = 0
        
        # Define forces for this step
        forcesX = []
        forcesY = []

        for index in range(self.numBots):
            
            xForce = ac[2*index]
            yForce = ac[2*index+1]
            
            forcesX.append(xForce)
            forcesY.append(yForce)
        
        forcesX = np.asarray(forcesX, dtype=np.float)
        forcesY = np.asarray(forcesY, dtype=np.float)

        # Apply those forces for the number of steps-per-step
        for i in range(self.numStepsPerStep):
            for index, bot in enumerate(self.bots):
                xForce = forcesX[index]*self.gain
                yForce = forcesY[index]*self.gain
                
                botPos = bot.body.position
                bot.body.apply_force_at_world_point((xForce, yForce), (botPos.x, botPos.y))

                # Gathering and printing max velocity
                # vel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
                # if np.abs(vel[0])>self.maxVelSeen:
                #     self.maxVelSeen = np.abs(vel[0])

        # Taking a step in the environment
            self.space.step(self.dt)
            self.time += self.dt

        # Note that we are considering timestep and time as different!
        self.timestep+=1
        
        # Gather information
        obs, systemCenter= self.getOb(ac)
            
        rew = self.calcRew(forcesX, forcesY)
        isDone, rew = self.isDone(rew, systemCenter)
        
        if self.dataCollect: 
            self.dataCollection(ac,rew,obs)

        return obs, rew, isDone, self.info
    
    
    def getOb(self, ac):
        
        runTime = [self.timestep/self.maxNumSteps]
        
        botPos = np.zeros((self.numBots,2))
        botVel = np.zeros((self.numBots,2))
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)

            botPos[index,:] = np.array(currentBotPos) - np.array(self.targetLoc)
                
            botVel[index,:] = currentBotVel
        
        systemCenter = np.mean(botPos, axis=0)

        # This value is in meters. Distance of square to target location
        distanceToTarget = np.linalg.norm(systemCenter) 
        
        botForces = np.zeros(self.numBots*2)
        for index, action in enumerate(ac):
            botForces[index] = action-1
        
        #### For Energy Consumption
        sysNormForce=0
        if self.energy:
            sysNormForce = np.linalg.norm(botForces*self.numStepsPerStep)

        # Normalizing observation
        botPos[:,0]=botPos[:,0]/(self.width) #Normalizing X-Coordinate
        botPos[:,1]=botPos[:,1]/(self.height)                  #Normalizing Y-Coordinate
        botVel /= self.maxVelocity                             #Normalizing Velocity

        extForces = np.abs(np.concatenate((self.extForcesX, self.extForcesY)))
        norm = np.linalg.norm(extForces)
        if norm != 0: 
            extForces/= norm
        
        observation = np.concatenate((botPos.flatten(), botVel.flatten(), self.contacts.flatten()))
        
        return observation, distanceToTarget
    
    
    def reportContact(self, contactPair, impulse):
        botIndex = max(contactPair)
        # Doing a += so that as multiple contacts occur over the many timesteps, we add them
        forces = impulse/self.dt
        self.extForcesX[botIndex-2] = forces[0]
        self.extForcesY[botIndex-2] = forces[1]
        self.contacts[botIndex-2] = 1


    def colPost(self, arbiter, space, data):
        """
        Post collision solver, as required by PyMunk
        """
        impulse = arbiter.total_impulse
        collisionShapes = arbiter.shapes
        collisionPair = [collisionShapes[0].collision_type, collisionShapes[1].collision_type]
        self.reportContact(collisionPair, impulse)

        # Printing information to verify callback working
        # print('__'*10)
        # print(impulse/self.dt,end='')
        # currentBotPos = self.convert.Pixels2Meters(self.bots[0].body.position.x), self.convert.Pixels2Meters(self.bots[0].body.position.y)
        # print(' ', currentBotPos,' ', end='')
        # print(arbiter.contact_point_set)

        return True
    
    
    def calcRew(self, forcesX, forcesY):
        """
        System should be rewarded for applying a maximum L2 norm force, 
        but discouraged for getting an net force greater than 0 in any direction.

        Thus the reward is SOLELY based on the action....

        Technically speaking, there should be no need for a physical system, since all we care about is the action taken.
    
        But this is still a good test to verify that the observation is meaningful enough to achieve a desired behaviour
        """
        
        F = np.vstack((forcesX, forcesY))
        
        # Get forces acting on each bot as a result of constraints (springs and slider joints)
        springForcesX, springForcesY = self.getConstraintForces()

        # Get sum of L2 force APPLIED
        l2_F = np.linalg.norm(F,axis=0)
        l2_sum = np.sum(l2_F)
        
        # Get net force on object
        # This is a result of forces APPLIED to each bot and the springs
        F[0] += springForcesX
        F[1] += springForcesY
        sum_F = np.sum(F, axis=1)
        net_F = np.linalg.norm(sum_F)

        rew = l2_sum - net_F

        return rew
    
    
    def isDone(self, rew, systemCenter):
        """
        Can return later on an add penalties for taking too long or surpassing the target
        """
        done=False
        # Takes too long to complete
        if self.timestep>self.maxNumSteps:
            done=True
            rew = 0
        
        if np.linalg.norm(systemCenter) > self.width/4:
            """
            End the simulation if there is too much drift.
            """
            done = True
            rew = 0

        return done, rew
            
    
    def render(self):
        if not self.render_setup:
            from pymunk.pygame_util import DrawOptions
            
            screenHeight = floor(self.convert.Meters2Pixels(self.height))
            screenWidth = floor(self.convert.Meters2Pixels(self.width))
            
            pygame.init()
            self.screen = pygame.display.set_mode((screenWidth, screenHeight))
            self.clock = pygame.time.Clock()
            
            """
            Review this link for information on drawing in PyGame
            http://www.pymunk.org/en/latest/pymunk.html#pymunk.SpaceDebugDrawOptions
            """
            self.drawOptions = DrawOptions(self.screen)
            self.drawOptions.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES 
            self.drawOptions.flags |= pymunk.SpaceDebugDrawOptions.DRAW_CONSTRAINTS # Uncomment to allow constraints to be drawn
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
            
    def getConstraintForces(self) -> tuple:
        """
        This function will calculate the X and Y forces acting on each bot as a result of the springs connecting them.
        """
        springForceX = np.zeros(self.numBots)
        springForceY = np.zeros(self.numBots)
        for i, bot in enumerate(self.bots):
            springs = self.bindingSprings[i]
            connections = self.bindingList[i]
            
            # Get the forces
            forceBack = np.abs(springs[0].impulse/self.dt)
            forceFront = np.abs(springs[1].impulse/self.dt)

            # Get the connecting indecies in self.membrane
            backConnect = connections[0]
            frontConnect = connections[1]

            # Get the directions for each force
            backDir = self.membrane[backConnect[0]].body.position - bot.body.position
            frontDir = self.membrane[frontConnect[1]].body.position - bot.body.position

            # Get normalized directions
            backDir = np.asarray(backDir)
            frontDir = np.asarray(frontDir)
            normBack = np.linalg.norm(backDir)
            normFront = np.linalg.norm(frontDir)
            backDir /= normBack
            frontDir /= normFront

            # Now get the directional forces in their respective directions.
            forceBack = forceBack*backDir    # This gives us the directional force values w.r.t. the location of bot 
            forceFront = forceFront*frontDir # This gives us the directional force values w.r.t. the location of the bot.
            
            # Append results to spring force arrays
            springForceX[i] = forceBack[0] + forceFront[0]
            springForceY[i] = forceBack[1] + forceFront[1]

        return springForceX, springForceY


            
    def dataCollection(self,forcesApplied,rew,obs):
        
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
            
            
        for action in forcesApplied:
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
        # Refer to __init__ method to learn what each of these variables are
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
        
        print('\n\nExporting and plotting data...',end='')

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



def connectBalls(space, theta1, theta2, b1, b2, rest_length, spring_stiffness, spring_damping, maxSeparation):
    springConstraint = pymunk.DampedSpring(b1.body, b2.body, 
                                           (-b1.radius*np.sin(theta1), b1.radius*np.cos(theta1)), (b2.radius*np.sin(theta2), -b2.radius*np.cos(theta2)), 
                                            rest_length, spring_stiffness, spring_damping)
    space.add(springConstraint)
    return springConstraint



def createJamoeba(space, systemCenterLocation, systemRadius, numBots, botMass, botRadius, skinMass, skinRadius, skinRatio, botFriction, springK, springB, springRL, maxSeparation, inRadius, inMass, inFriction,  percentInteriorRemove = 0, botCollisionIntStart = 2):
    xCenter = systemCenterLocation[0]
    yCenter = systemCenterLocation[1]
    
    collisionType = botCollisionIntStart      
    
    bots = []
    interiorParticles = []
    membrane = []
    isBot = []             # Appending 1 if an element in membrane is a bot, 0 otherwise.
    bindingSprings = []    # Where we will store any spring attached to a bot. Ignoring those attached to skin particles.
    bindingList = []       # how to locate the bodies in membrane each spring is connected to.

    # Set the following variable to true if you do not want to create membrane or interior particles.
    bots_only = False

    # Get interior particles
    ## Comment out if you only want bots
    if not bots_only:
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
        isBot.append(1)
        
        # Skin particles
        # Comment out if you only want bots
        if not bots_only:
            for j in range(1,skinRatio+1):
                x = xCenter + systemRadius*np.cos(theta + j*t)
                y = yCenter + systemRadius*np.sin(theta + j*t)
                skin = Ball(space, (x,y), skinRadius, skinMass, botFriction, color=(0,0,255,255))
                membrane.append(skin)
                isBot.append(0)

    # Connecting the membrane using springs
    if not bots_only:  
        numBodies = len(membrane)
        springs = []
        sliders = []
        for index, body in enumerate(membrane):
            if index < (numBodies-1):
                springJoint = connectBalls(space, t*index, t*(index+1), body, membrane[index+1], springRL, springK, springB, maxSeparation)
                
            else:
                springJoint= connectBalls(space, t*index, 0, body, membrane[0], springRL, springK, springB, maxSeparation) # GOOD ONE
            springs.append(springJoint)

        # We are now going to only keep those springs and slider joints that are connected to a bot.
        isBot = np.asarray(isBot)
        keep1 = np.where(isBot)[0]
        keep2 = keep1 + skinRatio
        keep = np.concatenate((keep1,keep2))
        keep = np.sort(keep)
        keep = np.roll(keep,1)

        springs = np.asarray(springs)[keep]

        bindingIndex = []
        last= keep[0]
        for i in keep:
            if i != last:
                connection = [i,i+1]
            else:
                connection = [i,0]
            bindingIndex.append(connection)
            
        for i in range(numBots):
            botConnections = [bindingIndex[2*i], bindingIndex[2*i+1]]
            bot_springs = [springs[2*i], springs[2*i+1]]
  

            bindingList.append(botConnections)
            bindingSprings.append(bot_springs)

        bindingList = np.asarray(bindingList)
        bindingSprings = np.asarray(bindingSprings)

        """
        The following code is left to later prove (if required) that the joints are properly indexed. 
        To be specific, the bindingSprings matrix is ordered such that each row are the constraints for that index bot.
        To prove this, uncomment the below code, choose a bot index and witness that the constraints around that bot are removed.
        """
        # for spring in bindingSprings[1].flatten():
        #     space.remove(spring)
        # for slider in bindingSliders[1].flatten():
        #     space.remove(slider)

    # Create Interiors
    # # Comment out if you only want bots
    if not bots_only:
        for index, in_ring in enumerate(gran_per_ring):
            radius = in_rings_radius[index]
            for j in range(in_ring):
                in_theta = j*2*np.pi/in_ring
                x = xCenter + radius*np.cos(in_theta)
                y = yCenter + radius*np.sin(in_theta)
                
                interiorParticle = Ball(space, (x,y), inRadius, inMass, inFriction)
                interiorParticles.append(interiorParticle)
            
    return bots, membrane, interiorParticles, bindingSprings, bindingList


def interiorPattern(systemRadius, inRadius, botRadius, percentInteriorRemove=0):
    R = systemRadius
    in_rings_radius = [] #**
    gran_per_ring = []

    if percentInteriorRemove==1:
        return gran_per_ring, in_rings_radius
    
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
    wall_collision_type = 1
    
    # Bottom Wall
    body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1 = pymunk.Poly.create_box(body1, (screenWidth,wallThickness))
    shape1.body.position = (screenWidth//2,wallThickness//2)
    shape1.collision_type = wall_collision_type
    
    # Top Wall
    body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2 = pymunk.Poly.create_box(body2, (screenWidth,wallThickness))
    shape2.body.position = (screenWidth//2, screenHeight-wallThickness//2)
    shape2.collision_type = wall_collision_type
    
    # Back wall
    body3 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape3 = pymunk.Poly.create_box(body3,(wallThickness, screenHeight))
    shape3.body.position = (wallThickness//2, screenHeight//2)
    shape3.collision_type = wall_collision_type
    

    # Front Wall
    body4 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape4 = pymunk.Poly.create_box(body4,(wallThickness, screenHeight))
    shape4.body.position = (screenWidth, screenHeight//2)
    shape4.collision_type = wall_collision_type

    space.add(shape1,body1,shape2,body2,shape3,body3,shape4,body4)

    return [[shape1, body1], [shape2, body2], [shape3, body3], [shape4, body4]]


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
        