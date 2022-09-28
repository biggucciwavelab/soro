"""
This file contains parameters for the execution of grabbing
"""

# Basic libraries needed
import sys
import matplotlib.pyplot as plt
import os
import numpy as np
from math import floor
from win32api import GetSystemMetrics
from tqdm import tqdm

# Importing custom libraries and functions
from utils import calc_JAMoEBA_Radius, Convert, createVideo

# Modified Action and Observation Environment
from Environment import robot

# Number of bots in system. 
# This key parameter affects others, so it is kept separate from the rest.
numBots = 75

"""
This dictionary is only a recommendation of
number_of_bots : PixelsPerMeter

This parameter is called below
"""
botPPMdict = {3:500,
              10:150,
              15:100,
              20:75,
              25:75,
              numBots:300}

# Environment Parameters    
dt = 1/200                    # [seconds] Simulation timestep
numStepsPerStep = 5            # How many simulation timesteps to run with each call of step(). Note: If you have a
                                # timestep of .005 and set this parameter to 50, then each call to step() pushes the sim forward .25 seconds
botMass = .3                    # [kg] Mass of bots
botRadius = .025                # [m] Radius of bots
skinRadius = .0125               # [kg] Radius of passive membrane particles
skinMass = botMass*.01              # [kg] Mass of passive membrane particles
skinRatio = 1                   # How many passive skin particles between each bot.
inRadius = botRadius            # [m] Radius of passive interior particles
inRadiusRandomness = 'none'     # 'none', 'uniform', binary
inRadiusRandomnessPer = 0.25     # Variation will be down to percent value with respect to inRadius
botFriction = .8                # Coefficient of (contact) friction for bots, membrane, and other walls in the environment
inMass = botMass * .001                     # [kg] Mass of pass interior particles
inFriction = .8                 # Coefficient of (contact) friction for interior passive particles. This is separate, as interior particle friction changes jamming characteristics
percentInteriorRemove = .85       # [%] Percent between [0,1] of interior particles to remove. Currently does not work properly with percentInteriorRemove = 1
springK = 10                   # [N/m] Spring stiffness of binding springs on the membrane
springB = 0.1                    # Spring damping.
springRL = inRadius/2                    # Resting length of binding spring on membrane.
wallThickness = botRadius/2     # Thickness of wall binding environment in-place.
maxSeparation = inRadius*.2   # [m] Maximum separation of bodies on the membrane
floorFriction = 0.           # Sliding friction coefficient of 'floor'. Note: Setting this greater than 0 slows the simulation.
width = floor(GetSystemMetrics(0)*1.) # [pixels] Width of simulation window. This will work natively on your machine.
height = floor(GetSystemMetrics(1)*1.)# [pixels] Height of simulation window. This will work natively on your machine.
maxNumSteps = 1000#np.inf            # Maximum number of calls to step() for simulation to run for. This parameter can be used to kill the sim, preventing it from running for ever.
ppm = botPPMdict[numBots]       # Pixels Per Meter. This parameter is VERY important! Defines how large everything is on your screen.

# Defining system radius
R = calc_JAMoEBA_Radius(skinRadius,skinRatio,botRadius,numBots) # You can also set this parameter yourself, this is merely a helper function.

# Parameters for running the simulation
t = 0
convert = Convert(botPPMdict[numBots])
dataCollect = t == 1  # Whether to collect and export data from the simulation. Naturally, this slows the simulation down.
saveVideo = t == 1     # Whether to save a video of the simulation you are currently running. Note: If this is True, make sure you have implemented a maxNumSteps>0!!!
experimentName = "mpc" # Title for savefolders (if you choose to use dataCollect or saveVideo)

"""
Items below this comment should not have to be edited by user
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
            'inRadiusRandomness': inRadiusRandomness,
            'inRadiusRandomnessPer': inRadiusRandomnessPer,
            'botFriction':botFriction,
            'inMass':inMass,
            'inFriction':inFriction,
            'percentInteriorRemove':percentInteriorRemove,
            'springK':springK,
            'springB':springB,
            'springRL':springRL,
            'floorFriction':floorFriction,
            'wallThickness':wallThickness,
            'maxSeparation':maxSeparation,
            'dataCollect':dataCollect,
            'experimentName':experimentName,
            'saveVideo':saveVideo
            }