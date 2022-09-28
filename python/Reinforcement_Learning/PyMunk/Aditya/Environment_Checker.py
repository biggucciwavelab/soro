# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 09:56:07 2020

@author: elopez8

Checks to see if the environment can be used by Stable-Baselines

Link:
    https://stable-baselines.readthedocs.io/en/master/guide/custom_env.html
"""

import gym
from stable_baselines.common.env_checker import check_env
import JAMoEBA_TrainingEnvironment as Strings
import numpy as np


#env = gym.make('CartPole-v1')                  # This one is correct! It is already a gym environment.
#env = ChronoPendulum.Env(render=False)         # Trying to get this one to work
#env = ChronoGymPendulum.ChronoPendulum()       # This is the chrono-gym one! Does not have a render call. That can be gotten from the other if needed.
#env = GoLeftEnv_Sample_Custom_Env.GoLeftEnv()   # Custom environment following Gym API
#env = ChronoGymAnt.ChronoAnt()

ppm = 50 # Pixels Per Meter
dt = 1/50.0 # Simulation timestep
# dt = .001
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
dataCollect = False

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
             'dataCollect':dataCollect}

env = Strings.pymunkEnv(**envParams)

check_env(env)
# If the environment is good, nothing will show.
# Update: You may get a few Tensorflow and colorize errors, but ignore them.