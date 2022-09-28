# -*- coding: utf-8 -*-
"""
Created on Sun Apr 25 18:41:15 2021

@author: elope
"""


# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 10:13:04 2021

@author: elope
"""


# from WegWorld import robot, createVideo
from SingleBotSystem import singleRobot, createVideo, Convert
import numpy as np
from numpy import pi
from win32api import GetSystemMetrics
from math import floor

"""
Input units:
    Distance: Meters
    Force: N
    Mass: kg
"""

dt = 1/20.0 # Simulation timestep
ppm = 500 #Pixels Per Meter - Defines the conversion ratio betweend units of meters to pixels. CRITICAL 
numStepsPerStep = 1 # Number of simulation timesteps to integrate on every call of 'step'
botMass = .02
botRadius = .025 #**
botFriction = .1
inMass = .002
inFriction = .1
wallThickness = botRadius/2 #** # Thickness of the bounding walls in the environment
maxNumSteps = 2_000 # Simulation will self-terminate after this many simulation timesteps have passed. (in the event you forgot to close out)

# How large the window will be on your screen
width = floor(GetSystemMetrics(0)*.9)
height = floor(GetSystemMetrics(1)*.9)

convert=Convert(ppm)
widthMeters = convert.Pixels2Meters(width)
heightMeters = convert.Pixels2Meters(height)

# Feel free to ignore these parameters for now
dataCollect = False
experimentName = 'OneBot'
saveVideo = False
systemStart = widthMeters/2, heightMeters/2, pi/4

envKwargs = {'dt':dt,
             'ppm':ppm,
             'numStepsPerStep':numStepsPerStep,
             'screenHeight':height,
             'screenWidth':width,
             'maxNumSteps':maxNumSteps,
             'botMass':botMass,
             'botRadius':botRadius,
             'botFriction':botFriction,
             'wallThickness':wallThickness,
             'dataCollect':dataCollect,
             'experimentName':experimentName,
             'saveVideo':saveVideo,
             'systemStart':systemStart}

amin = singleRobot(**envKwargs)
amin.reset()

#Define what action we will take at every timestep. As of now, we are applying 0 force.
action = [.1,.1]

# Run the simulation
for i in range(maxNumSteps):
    amin.step(action)
    amin.render()

if dataCollect: amin.dataExport()
if saveVideo:
    createVideo('./', amin.videoFolder, experimentName, (width, height))
amin.close()

import matplotlib.pyplot as plt
plt.close('all')