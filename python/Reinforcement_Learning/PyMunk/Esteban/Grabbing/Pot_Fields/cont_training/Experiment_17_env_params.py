"""
This file contains parameters for the execution of grabbing
"""

# Basic libraries needed
from io import SEEK_CUR
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
from cont_training.Environment_Experiment_17 import pymunkEnv
import cont_training.Environment_Experiment_17 as master_env

# Number of bots in system. 
# This key parameter affects others, so it is kept separate from the rest.
numBots = 10

# Dictionary of how many time steps an episode should last,
# based on how many bots the system is made of
botTimestepDict = {3:40_000,
                    10:1_000,   
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
botRadius = .025  
skinRadius = .015 
skinMass = botMass
skinRatio=2
inRadius = botRadius 
botFriction = .9
inMass = .01   
inFriction = .1
percentInteriorRemove = .5
springK = 1
springB = 5 
springRL = 0 
wallThickness = botRadius/2 
maxSeparation = inRadius*1.75 
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
render = True
saveVideo = False
dataCollect = False
experimentName = "Test_Run" # Make sure the name has {} to allow for additional formatting!

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
            'velocity_limit':velocity_limit,
            'wallThickness':wallThickness,
            'maxSeparation':maxSeparation,
            'binding_spring_K':binding_spring_K,
            'dataCollect':dataCollect,
            'experimentName':experimentName,
            'energy':energy,
            'kineticEnergy':kineticEnergy,
            'saveVideo':saveVideo
            }