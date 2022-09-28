# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 09:39:29 2020

@author: 17088
"""

from stable_baselines import PPO2
from Strings_Environment_Experiment_253 import pymunkEnv, createVideo
experiment_num = 253
bestModel = False
# from JAMoEBA_TrainingEnvironment import pymunkEnv, createVideo
import matplotlib.pyplot as plt
import numpy as np
from warnings import warn
import easygui


print('\nA warning box should have popped up. Look for it!!\n')
easygui.msgbox("""Please be aware that you MUST check the parameters and verify they are the same as when training, otherwise the results you are about to obtain may not make sense!""",
               title='Warning',
               ok_button='Accept Warning')

botTimestepDict = {3:30_000,
                    # 10:50_000,
                    # 15:2500,
                    # 20:3000,
                    # 25:4000,
                    # 30:50_000
                    }

botPPMdict = {3:500,
              # 10:150,
#               15:100,
#               20:75,
#               25:50,
              # 30:55
              }


# Environment Parameters
dt = 1/200.0 # Simulation timestep
numStepsPerStep = 100
numBots = 3
botMass = .02
botRadius = .019 #**
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
maxSeparation = inRadius*1.5 #**
obstaclesPresent = True
grabObjectPresent = False
potFieldPresent = False
tunnel = False
energy = False

#Defining system Radius
startDistance = skinRadius # The start distance between bots
arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
theta = 2*np.pi/numBots
R = arcLength/theta

# Target distance from X-start location
targetDistance = R*72 # Unit: m

# Screen parameters (Taken from my big screen (; )
width = 3096
height = 1296
maxNumSteps = botTimestepDict[numBots]
ppm = botPPMdict[numBots] # Pixels Per Meter

# General Parameters
dataCollect = True
saveVideo = True
render = True

tests=3

if bestModel:
    modelName = 'best_model.zip'
else:
    modelName = 'ppo2_Experiment_{}.zip'.format(experiment_num)
model = PPO2.load(modelName)



# Training loop
for i in range(tests):
    experimentName = 'Experiment_{}_v{}_LongerRun'.format(str(experiment_num), str(i+1))
    if bestModel: experimentName += '_BestModel'

# All together now!
    envParams = {'dt':dt,
             'numStepsPerStep':numStepsPerStep,
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
              'obstaclesPresent':obstaclesPresent,
              'grabObjectPresent':grabObjectPresent,
              'potFieldPresent':potFieldPresent,
              'tunnel':tunnel,
              'energy':energy
             }
    
    envParams['dataCollect']=dataCollect
    envParams['saveVideo']=saveVideo
    envParams['experimentName']= experimentName
    
    
    env = pymunkEnv(**envParams)
    obs=env.reset()
    
    for i in range(botTimestepDict[numBots]):
        action, _states = model.predict(obs)
        obs, reward, done, info = env.step(action)
        
        if render:
            env.render()
        
        if done:
            break
    
    if dataCollect:
        env.dataExport() # Export the data from the simulation
    
    
    plt.close('all')
    if saveVideo:
        createVideo('./',env.videoFolder,experimentName, (width,height))
    env.close()