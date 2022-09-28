# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 17:31:58 2021

@author: elopez8
"""

import JAMoEBA_TestEnv as Strings
from JAMoEBA_TestEnv import createVideo
from win32api import GetSystemMetrics
from conversion import Convert
import matplotlib.pyplot as plt
import numpy as np
from math import floor
import os
import pathlib
import pdb
    
botTimestepDict = {3:40_000,
                    10:60_000,
                    15:6000,
                    20:3000,
                    25:4000,
                    30:50_000}

botPPMdict = {3:500,
              10:150,
              15:100,
              20:75,
              25:75,
              30:55}

# Environment Parameters
dt = 1/200.0 # Simulation timestep
numStepsPerStep = 50
numBots = 30
botMass = .02
botRadius = .025 #**
skinRadius = .015 #**
skinMass = botMass
skinRatio=2
inRadius = botRadius #**
botFriction = .1
inMass = .002
inFriction = .1
percentInteriorRemove = 0
springK = 50 #**
springB = .1
springRL = 0 #**
wallThickness = botRadius/2 #**
maxSeparation = inRadius*1.75 #**
obstaclesPresent = True
grabObjectPresent = False
energy=False
kineticEnergy = True
potFieldPresent = False
tunnel = False

#Defining system Radius
startDistance = skinRadius # The start distance between bots
arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
theta = 2*np.pi/numBots
R = arcLength/theta #**

# Target distance from X-start location
targetDistance = R*72 # Unit: m

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
targetLoc = None
systemStart = None
# systemStart = convert.Pixels2Meters(width/4), convert.Pixels2Meters(height/2)
# targetLoc = systemStart[0]+R*36, systemStart[1]
render = True
saveVideo = False
dataCollect = False
experimentName = "CountingStars"

if saveVideo:
    mainDirectory = str(pathlib.Path(__file__).parent.absolute())
    savefile = mainDirectory + '\\{}\\'.format(experimentName)
    os.makedirs(savefile,exist_ok=True)
    
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
         'wallThickness':wallThickness,
         'maxSeparation':maxSeparation,
         'targetDistance':targetDistance,
         'dataCollect':dataCollect,
         'experimentName':experimentName,
         'obstaclesPresent':obstaclesPresent,
         'grabObjectPresent':grabObjectPresent,
         'potFieldPresent':potFieldPresent,
         'tunnel': tunnel,
         'energy':energy,
         'kineticEnergy':kineticEnergy,
         'targetLoc':targetLoc,
         'systemStart':systemStart,
         'saveVideo':saveVideo
         }

env = Strings.pymunkEnv(**envParams)

action=[]
for num in range(numBots*2):
    if num%2==0: action.append(2)
    else: action.append(1)

for _ in range(1):
    plt.close('all')
    env.reset()

    for _ in range(maxNumSteps):
        if render:
            env.render()
        _, _, done, _ = env.step(action)
        if done: 
            break
            print('Done')
    
    if dataCollect: env.dataExport()
    if saveVideo: createVideo(savefile, env.videoFolder, experimentName, (width, height))
    env.close()
    plt.close('all')