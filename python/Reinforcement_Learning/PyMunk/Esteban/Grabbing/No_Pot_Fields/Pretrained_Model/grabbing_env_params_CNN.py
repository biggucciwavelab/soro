"""
This file contains parameters for the execution of grabbing
"""

# Basic libraries needed
import pdb
import sys
import matplotlib.pyplot as plt
import os
from shutil import copyfile
import numpy as np
from numpy import sin, cos, sqrt, log
from warnings import warn
from win32api import GetSystemMetrics
from tqdm import tqdm
sys.path.append('../..')

# Importing custom libraries and functions
from utils.utils import calc_JAMoEBA_Radius, Convert, createVideo
from Grabbing_Environment_CNN_Playground import pymunkEnv
import Grabbing_Environment_CNN_Playground as master_env

# Number of bots in system. 
# This key parameter affects others, so it is kept separate from the rest.
numBots = 10

# Dictionary of how many time steps an episode should last,
# based on how many bots the system is made of
botTimestepDict = {3:40_000,
                    10:500, # Expert Trajectory creation
                    # 10:1000,    # Testing
                    15:6000,
                    20:3000,
                    25:4000,
                    30:500}

# Dictionary of pixels-per-meter,
# based on how many bots the system is made of.
botPPMdict = {3:500,
              10:500,
              15:100,
              20:75,
              25:75,
              30:250}

# Environment Parameters
dt = 1/200.0 # Simulation timestep
numStepsPerStep = 50
botMass = .2
botRadius = .025  #**
skinRadius = .015 #**
skinMass = botMass
skinRatio=2
inRadius = botRadius #**
botFriction = .9
inMass = .01   
inFriction = .1
percentInteriorRemove = .5
springK = 1 #**
springB = 5 
springRL = 0 #**
wallThickness = botRadius/2 #**
maxSeparation = inRadius*1.75 #**
energy=False
kineticEnergy = False
slidingFriction = 0
velocity_limit = 0 # in m/s

# Stiffness of binding springs (N/m)
binding_spring_K = .5

# Defining system radius
R = calc_JAMoEBA_Radius(skinRadius,skinRatio,botRadius,numBots)

#Screen parameters (Taken from my big screen (; )
# I.e. use this if operating on any other system
width = 3096
height = 1296

# Esteban's desktop:
# width = floor(GetSystemMetrics(0)*.9)
# height = floor(GetSystemMetrics(1)*.9)
maxNumSteps = botTimestepDict[numBots]
ppm = botPPMdict[numBots] # Pixels Per Meter

# Parameters for specifiying the system and target locations at start
# If you do not want to specify these, simply set them as 'None'
convert = Convert(botPPMdict[numBots])
render = False
saveVideo = False
dataCollect = False
experimentName = "Expert_Data_run_{}" # Make sure the name has {} to allow for additional formatting!

"""
_____________________________________________________________________
Items below this comment should not have to be edited by user
_____________________________________________________________________
"""

# Put all environment changeable parameters into a dictionary. 
envParams = {'dt':dt,
            'numStepsPerStep': numStepsPerStep,
            'ppm':ppm,
            'screenHeight':height,
            'screenWidth':width,
            'maxNumSteps':maxNumSteps,
            'R':R,
            'numBots':numBots,
            'botMass':botMass,
            'botRadius':botRadius,
            'skinRadius':skinRadius,
            'skinMass':skinMass,
            'skinRatio':skinRatio,
            'inRadius':inRadius,
            'botFriction':botFriction,
            'inMass':inMass,
            'inFriction':inFriction,
            'percentInteriorRemove':percentInteriorRemove,
            'springK':springK,
            'springB':springB,
            'springRL':springRL,
            'slidingFriction':slidingFriction,
            'wallThickness':wallThickness,
            'maxSeparation':maxSeparation,
            'binding_spring_K':binding_spring_K,
            'dataCollect':dataCollect,
            'experimentName':experimentName,
            'energy':energy,
            'kineticEnergy':kineticEnergy,
            'saveVideo':saveVideo
            }

class controller:
    """
    Controller which will call the pot field for actions to be passed to the discrete actions
    """
    def __init__(self, pot_field_controller, noise=False):
        self.noise = noise
        if self.noise:
            self.gauss = np.random.normal # For adding noise to the actuation
            self.scale = 2 # Standard deviation from Gaussian distribution to sample noise from
        self.pot_field_controller = pot_field_controller

    def get_action(self, _obs):
        """
        Given the observation from the environment, return the recommended action to take per the pot field.
        We don't need the observation, thus it is set as _obs.
        """
        Fx, Fy = self.pot_field_controller.run_controller()
        action = [] 
        for i in range(Fx.size):
            action.append(Fx[i])
            action.append(Fy[i])
        # action = self.correct_force(Fx, Fy)
        # Adding noise to the action
        if self.noise:
            action = self.add_noise(action)
        return action

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
        This must be adjusted to b ein the discrete format that the environment is currently setup in.
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
        # pdb.set_trace()
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