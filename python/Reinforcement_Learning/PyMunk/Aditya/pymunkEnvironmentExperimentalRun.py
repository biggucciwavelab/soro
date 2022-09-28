# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 17:31:58 2021

@author: elope
"""

import JAMoEBA_TrainingEnvironment as Strings
import matplotlib.pyplot as plt
import numpy as np
from math import floor
import pdb

maxTime = 500
    
# Environment Parameters
ppm = 100 # Pixels Per Meter
dt = 1/50.0 # Simulation timestep
numBots = 10
botMass = .02
botRadius = .038/2 #**
skinRadius = .015 #**
skinMass = botMass
skinRatio=2
inRadius = botRadius #**
botFriction = .01
inMass = .002
inFriction = .05
percentInteriorRemove = 0
springK = 50 #**
springB = .1
springRL = 0 #**
wallThickness = botRadius/2 #**
maxSeparation = inRadius*1.75 #**

#Defining system Radius
startDistance = skinRadius # The start distance between bots
arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
theta = 2*np.pi/numBots
R = arcLength/theta #**

# Target distance from X-start location
targetDistance = 10 # Unit: m

#Screen parameters (Taken from my big screen (; )
width = 3096
height = 1296
maxNumSteps = 2000

dataCollect = False
experimentName = "Test"

envParams = {'dt':dt,
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
         'wallThickness':wallThickness,
         'maxSeparation':maxSeparation,
         'targetDistance':targetDistance,
         'dataCollect':dataCollect,
         'experimentName':experimentName}

env = Strings.pymunkEnv(**envParams)

action=[]
for num in range(numBots*2):
    if num%2==0: action.append(2)
    else: action.append(1)

for _ in range(2):
    
    env.reset()

    for _ in range(maxTime):
        env.render()
        env.step(action)
        
    env.dataExport()
    env.close()