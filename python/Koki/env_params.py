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

# Modified Action and Observation Environment
from Environment import robot, Convert, createVideo

# Simulation parameters
ppm = 500 # Pixels per Meter
dt = 1/200              # [seconds] Simulation timestep
numStepsPerStep = 50    # How many simulation timesteps to run with each call of step(). Note: If you have a
                        # timestep of .005 and set this parameter to 50, then each call to step() pushes the sim forward .25 seconds
maxNumSteps = 1000      # Maximum number of calls to step() for simulation to run for. This parameter can be used to kill the sim, preventing it from running for ever.
width = floor(GetSystemMetrics(0)*.9) # [pixels] Width of simulation window. This will work natively on your machine.
height = floor(GetSystemMetrics(1)*.9)# [pixels] Height of simulation window. This will work natively on your machine.

# System physical parameters
botMass = 1            # [kg] Mass of bots 
botRadius = .025        # [m] Radius of bots
bigR = botRadius * 5    # Radius of bots surrounding center bot
botFriction = .3        # Coefficient of (contact) friction for bots, membrane, and other walls in the environment
floorFriction = .1       # Sliding friction coefficient of 'floor'. Note: Setting this greater than 0 slows the simulation.
force = 1               # The amount of force applied when calling Step

# Connection parameters
springK = 0                    # [N/m] Spring stiffness of binding springs on the membrane
springB = 0                    # Spring damping.
springRL = bigR                    # Resting length of binding spring on membrane.
minSeparation = 0
maxSeparation = bigR   # [m] Maximum separation of bodies on the membrane 

# Parameters for running the simulation. Whether to save data or not.
t = 0
dataCollect = t   # Whether to collect and export data from the simulation. Naturally, this slows the simulation down.
saveVideo = t     # Whether to save a video of the simulation you are currently running. Note: If this is True, make sure you have implemented a maxNumSteps>0!!!
experimentName = "NO NAME" # Title for savefolders (if you choose to use dataCollect or saveVideo)

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
            'R':bigR,
            'force':force,
            'botMass':botMass,
            'botRadius':botRadius,
            'botFriction':botFriction,
            'springK':springK,
            'springB':springB,
            'springRL':springRL,
            'floorFriction':floorFriction,
            'minSeparation':minSeparation,
            'maxSeparation':maxSeparation,
            'dataCollect':dataCollect,
            'experimentName':experimentName,
            'saveVideo':saveVideo
            }