"""
Use this in the future to add collision detection in the observation space:
    https://stackoverflow.com/questions/50815789/non-colliding-objects-which-has-colliding-pairs-pymunk-pygame
    
This example shows how to draw collisions:
    https://github.com/viblo/pymunk/blob/master/examples/contact_and_no_flipy.py
"""
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

class robot(Env):
    
    info = {'There is no info to be aware of.':'Machinelearn away.'}
    
    def __init__(self, dt, ppm, screenHeight, screenWidth, maxNumSteps, R, 
                 botMass, botRadius, botFriction, force,
                 springK, springB, springRL, minSeparation, maxSeparation, 
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
        self.draw_forces = True
        self.force_color = THECOLORS['yellow']

        # System membrane parameters
        self.R = R
        self.numBots = 4
        self.subTheta = 2*np.pi/3
        self.botMass = botMass
        self.botRadius = botRadius     # Radius of 
        self.botFriction = botFriction # Friction of bots and skins
        self.force = force # The amount of force applied during each call of step
        
        # Spring parameters
        self.springK = springK
        self.springB = springB
        self.springRL = springRL
        self.minSeparation = minSeparation
        self.maxSeparation = maxSeparation
        
        self.systemStart = R+botRadius, self.convert.Pixels2Meters(screenHeight/2)
        
        #### Data Collection
        self.env_params = [['dt:',str(self.dt)],
                                       ['NumSetpsPerStep:',str(self.numStepsPerStep)],
                                       ['Num_Bots', str(self.numBots)],
                                       ['botMass',str(self.botMass)],
                                       ['botRadius',str(self.botRadius)],
                                       ['botFriction',str(self.botFriction)],
                                       ['SpringK:', str(self.springK)], 
                                       ['SpringB:', str(self.springB)], 
                                       ['SpringRL:', str(self.springRL)],
                                       ['maxSeparation',str(self.maxSeparation)],
                                       ['floorFriction', str(self.slidingFriction)],
                                       ['PixelsPerMeter (ppm):',str(self.ppm)],
                                       ['SystemRadius',str(self.R)],
                                       ['ScreenWidth',str(self.width)],
                                       ['ScreenHeight',str(self.height)],
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
            self.obs_data = np.zeros(self.numBots*6 +1) # Wil need to be changed if you later change the observations size.

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
        springK = self.convert.SpringK2Pixels(self.springK)
        springRL = self.convert.Meters2Pixels(self.springRL)
        maxSeparation = self.convert.Meters2Pixels(self.maxSeparation)
        minSeparation = self.convert.Meters2Pixels(self.minSeparation)

        # Defining the system start
        if startLoc is None:
            systemStart = self.convert.Meters2Pixels(self.systemStart[0]), self.convert.Meters2Pixels(self.systemStart[1])
        else:
            systemStart = self.convert.Meters2Pixels(startLoc)
        height = self.convert.Meters2Pixels(self.height)
        width = self.convert.Meters2Pixels(self.width)
        
        #### Create the system
        # All items that require conversion has been converted above
        kwargs={'space':self.space,
            'systemCenterLocation':systemStart,
            'systemRadius':R,
            'botMass':self.botMass,
            'botRadius':botRadius,
            'botFriction':self.botFriction,
            'springK':springK,
            'springB':self.springB,
            'springRL':springRL,
            'minSeparation':minSeparation,
            'maxSeparation':maxSeparation,
            'botCollisionIntStart': 2} # All obstacles will be of collision_type=1
        self.bots = createJamoeba(**kwargs)

        # Add simulated friction if we must
        if self.slidingFriction > 0:
            add_sliding_friction(self.space, self.bots, self.slidingFriction)

        #### Collision Handler
        # Reports collisions with walls, objects, and obstacles
        # for bot in self.bots:
        #     cHandler = self.space.add_collision_handler(1,bot.shape.collision_type)
        #     cHandler.post_solve = self.colPost
        
        #### Initiate Observation
        if self.draw_forces: self.force_vectors = []
        ac = np.array([0]*self.numBots) # Take no action
        observation, _ = self.getOb(ac)
        return observation
    
    
    
    
        
    def step(self, ac):
        ac = np.asarray(ac)
        assert ac.size == self.numBots, "Action must be of shape (numBots, 1)"

        if self.draw_forces:
            self.force_vectors = []
        
        # Contact information for storing
        self.extForcesX = np.zeros(self.numBots)
        self.extForcesY = np.zeros(self.numBots)
        
        for i in range(self.numStepsPerStep):
                
            for index, bot in enumerate(self.bots):
                dir = ac[index] # Direction we are applying a force in this time step
                if dir == -1:
                    continue
                thetaPlus = dir*self.subTheta # Add this to the heading of the bot, and that is the direction to apply the force
                botAngle = bot.body.angle
                vec = np.asarray([np.cos(botAngle + thetaPlus), np.sin(botAngle + thetaPlus)])
                forceVec = vec*self.force

                # To apply force, we find the body's location and apply the force its center
                botPos = bot.body.position
                bot.body.apply_force_at_world_point(tuple(forceVec), botPos)

                if self.draw_forces:
                    # Collect the vector here
                    anchor_a = np.array([botPos.x, botPos.y])
                    anchor_b = anchor_a + vec.flatten() * 10 # The scalar we multiply with defines how many pixels long the line is
                    
                    # Convert to pygame coordinates
                    anchor_a = pymunk.pygame_util.to_pygame(anchor_a, self.screen)
                    anchor_b = pymunk.pygame_util.to_pygame(anchor_b, self.screen)

                    self.force_vectors.append(
                        [anchor_a, anchor_b]
                    )

        # Taking a step in the environment
            self.space.step(self.dt)
            self.time += self.dt
        
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
        for index, bot in enumerate(self.bots):
            currentBotPos = self.convert.Pixels2Meters(bot.body.position.x), self.convert.Pixels2Meters(bot.body.position.y)
            currentBotVel = self.convert.Pixels2Meters(bot.body.velocity.x), self.convert.Pixels2Meters(bot.body.velocity.y)
            currentBotAng = bot.body.angle
            
            # Define position as relative to the target
            botPos[index, :] = np.array(currentBotPos)
            botVel[index, :] = currentBotVel
            botAng[index, :] = np.hstack([np.cos(currentBotAng), np.sin(currentBotAng)])

        systemCenter = np.mean(botPos, axis=0)

        # All external forces the bots are experiences
        extForces = np.abs(np.vstack((self.extForcesX, self.extForcesY))).T
        
        # The observation is made separately, should you ever want to add or remove items from it.
        observation = np.hstack((botPos, botVel, botAng))
        
        return observation, systemCenter


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
            self.drawOptions.flags |= pymunk.SpaceDebugDrawOptions.DRAW_CONSTRAINTS
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

        # Drawing force vectors
        if self.draw_forces:
            for coord in self.force_vectors:
                pygame.draw.line(
                    self.screen,
                    self.force_color,
                    coord[0], coord[1], width=1
                )

        pygame.display.update()
        if self.saveVideo:
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
            for line in self.env_params:
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
            if self.numBots > 1:
                pos = np.mean(row[1:last_col])
            else:
                pos = row[1:]
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
            if self.numBots > 1:
                pos = np.mean(row[1:last_col])
            else:
                pos = row[1:]
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
                plt.plot(time, self.ac[:,i+1], label='Y-Force')
                plt.xlabel(xlabel)
                plt.ylabel('Force [N]')
                plt.title('Applied Forces on Bot ' + str(bot))
                plt.legend(loc='lower right')
                plt.savefig(self.saveFolder + 'Bot ' + str(bot) + ' Applied Forces.jpg')
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
    def __init__(self, space, position, radius, mass, friction, 
                 collisionType = 0, color = (0,0,255,200), theta=0):
        self.body = pymunk.Body()
        self.radius = radius
        self.body.position = position
        self.body.angle = theta
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.mass = mass
        self.shape.color = color
        self.shape.friction = friction
        self.shape.collision_type = collisionType
        space.add(self.body, self.shape)
        



def connectBalls(space, b1, b2, rest_length, spring_stiffness, 
                 spring_damping, minSeparation, maxSeparation):

    springConstraint = pymunk.DampedSpring(b1.body, b2.body,
                                            (0, 0),(0,0),
                                            rest_length, spring_stiffness,
                                            spring_damping)
    slideJoint = pymunk.SlideJoint(b1.body, b2.body,
                                    (0,0),(0,0),
                                    minSeparation, maxSeparation)
            
    space.add(springConstraint, slideJoint)
    return springConstraint




def createJamoeba(space, systemCenterLocation, systemRadius,  
                  botMass, botRadius, botFriction,
                  springK, springB, springRL, minSeparation, maxSeparation, 
                  botCollisionIntStart = 2):
    """
    Creates the system of 3 bots surrounding a center bot, 
    all connected by their centers
    """

    # Will store the bots in this list
    bots = []

    # Defining the system center and radius
    X, Y = systemCenterLocation
    R = systemRadius
    
    # Needed to gather contact information later (if needed)
    collisionType = botCollisionIntStart
    
    # Create the bots
    ## First the center bot
    centerBot = Ball(space, (X,Y), botRadius, botMass, botFriction, collisionType)
    bots.append(centerBot)
    collisionType += 1

    ## Then the surrounding bots
    subTheta = 2*np.pi/3
    for i in range(3):
        theta = subTheta*i
        x, y = X + R*np.cos(theta), Y + R*np.sin(theta)
        bot = Ball(space, (x,y), botRadius, botMass, botFriction, collisionType, theta=theta)
        bots.append(bot)
        collisionType += 1

    # Connect the bots
    for index in range(1,4):
        bot = bots[index]
        spring = connectBalls(space, centerBot, bot, springRL, springK, springB, minSeparation, maxSeparation)

    return bots



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


def createVideo(saveLoc, imgLoc, videoName, imgShape):
    out = cv2.VideoWriter(saveLoc+videoName+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), 40, imgShape)
    for file in glob.glob(imgLoc+'*.jpg'):
        img = cv2.imread(file)
        out.write(img)
    out.release
    
    rmtree(imgLoc)

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
    # out = cv2.VideoWriter(saveLoc + videoName + '.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 40, imgShape)
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
    