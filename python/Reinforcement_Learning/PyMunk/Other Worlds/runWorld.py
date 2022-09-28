# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 10:13:04 2021

@author: elope
"""


from WegWorld import robot, createVideo
# from SpheroWorld import robot, createVideo
import numpy as np
from win32api import GetSystemMetrics
from math import floor

"""
Input units:
    Distance: Meters
    Force: N
    Mass: kg
"""


dt = 1/200.0 # Simulation timestep
ppm = 500 #Pixels Per Meter - Defines the conversion ratio between units of meters to pixels. CRITICAL 
numStepsPerStep =10 # Number of simulation timesteps to integrate on every call of 'step'
numBots = 30
botMass = .02
botRadius = .025 #**
skinRadius = .015 #**
skinMass = botMass
skinRatio=1
inRadius = botRadius #**
botFriction = .5
inMass = .002
inFriction = .5
floorFriction = .1 # Coefficient of friction of floor with all objects
percentInteriorRemove = .5 # Percentage of interior particles to remove. Should be a value between 0 and 1.
springK = 500 #** # Spring Stiffness
springB = .1
springRL = 0 #**
wallThickness = botRadius/2 #** # Thickness of the bounding walls in the environment
maxSeparation = inRadius*1.75 #** # The maximum distance bots are allowed to be from each other
minSeparation =0 # The minimum distance bots are allowed to be from each other
maxNumSteps = 2_000 # Simulation will self-terminate after this many simulation timesteps have passed. (in the event you forgot to close out)

#Defining system Radius
startDistance = skinRadius # The start distance between bots
arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
theta = 2*np.pi/numBots
R = arcLength/theta #**
R*=1.5

# How large the window will be on your screen
width = floor(GetSystemMetrics(0)*.9)
height = floor(GetSystemMetrics(1)*.9)

# Feel free to ignore these parameters for now
dataCollect = False
experimentName = 'BigData2'
saveVideo = False
systemStart = None

envKwargs = {'dt':dt,
             'ppm':ppm,
             'numStepsPerStep':numStepsPerStep,
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
             'floorFriction':floorFriction,
             'percentInteriorRemove':percentInteriorRemove,
             'springK':springK,
             'springB':springB,
             'springRL':springRL,
             'wallThickness':wallThickness,
             'maxSeparation':maxSeparation,
             'minSeparation':minSeparation,
             'dataCollect':dataCollect,
             'experimentName':experimentName,
             'saveVideo':saveVideo,
             'systemStart':systemStart}

amin = robot(**envKwargs)
amin.reset()

#Define what action we will take at every timestep. As of now, we are applying 0 force.
"""
Sphero World:
"""
# action = np.zeros(2*numBots)
# for i in range(numBots):
#     if i%2==0:
#         action[2*i]=1
        

"""
WegWorld:
"""
# Run the simulation
for i in range(maxNumSteps):
    # num = np.sin(i*0.025)
    # print(num)
    # if num>0: action = np.zeros(numBots)
    # else:action = np.ones(numBots)
    action = np.zeros(numBots)
    amin.step(action)
    amin.render()

if dataCollect: amin.dataExport()
if saveVideo:
    createVideo('./', amin.videoFolder, experimentName, (width, height))
amin.close()