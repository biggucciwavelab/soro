# -*- coding: utf-8 -*-
"""
Created on Wed Jan 27 11:05:16 2021

@author: elopez8

Overview of PyMunk:
    http://www.pymunk.org/en/latest/overview.html#object-shape

First attempts at JAMoEBA in PyMunik

The standard units will be as follows:
    Force = N (Newtons)
    Mass = kg (kilograms)
    Distance = m (meters)
"""
import pymunk
import pygame
import sys
import numpy as np
from math import floor
from win32api import GetSystemMetrics
from conversion import Convert
from JAMoEBA_utils import createJamoeba, createWalls, createObst
import matplotlib.pyplot as plt
import time
import os
import pdb

# Change working directory
path = os.getcwd()
print(path)
print()

#Desktop
# Using half the dimensions of the screen
width = floor(GetSystemMetrics(0) *0.9)
height = floor(GetSystemMetrics(1) *0.9)
print (width, height)
# Big Gucci
# Width = 3096
# height = 1296

# Visualization Boolean
render = True
maxSteps=1000 # Only applies when the system is not being rendered
if render:
    from pymunk.pygame_util import DrawOptions
    
center = (width//2, height//2) 

# Simulation conversion
"""
NOTE THAT THIS PARAMETER IS CRITICAL. Not only does it affect visualizaion, but the perceived location of your system as well.
"""
ppm = 75 # Pixels Per Meter
convert = Convert(ppm)

# Parameters for JAMoEBA
#** Denotes a parameter that requires conversion
dt = 1/50.0 # Simulation timestep
# dt = .001
numBots = 20
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
wallThickness = 1
maxSeparation = inRadius*1.5 #**


#Defining system Radius
startDistance = skinRadius # The start distance between bots
arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
theta = 2*np.pi/numBots
R = arcLength/theta #**

# Defining start location of system and target location
systemStart = R+botRadius+wallThickness*1.2, convert.Pixels2Meters(height/2) #**
# systemStart = convert.Pixels2Meters(center[0]),convert.Pixels2Meters(center[1]) #**
targetLoc = systemStart[0]+R*72, systemStart[1] #**

#### Convert Parameters that requireconversion
R = convert.Meters2Pixels(R)
botRadius = convert.Meters2Pixels(botRadius)
inRadius = convert.Meters2Pixels(inRadius)
springK = convert.SpringK2Pixels(springK)
springRL = convert.Meters2Pixels(springRL)
skinRadius = convert.Meters2Pixels(skinRadius)
maxSeparation = convert.Meters2Pixels(maxSeparation)
wallThickness = convert.Meters2Pixels(wallThickness)
systemStart = convert.Meters2Pixels(systemStart[0]), convert.Meters2Pixels(systemStart[1])
targetLoc = convert.Meters2Pixels(targetLoc[0]), convert.Meters2Pixels(targetLoc[1])

print('System Radius (in pixels):',R)
print('Screen height:',height)

        

def main():
    #### Initialize PyMunk andPyGame
    
    # #Create  the  obstacle
    # def create_obst(space):
    #     body = pymunk.Body(1, 100, body_type = pymunk.Body.STATIC)
    #     body.position = (0,0)
    #     shape = pymunk.Circle(body, 50)
    #     space.add(body,shape)
    #     return shape

    # def draw_obst(obstacle):
    #     for obst in obstacle:
    #         pos_x = int(obst.body.position.x)
    #         pos_y = int(obst.body.position.y)
    #         pygame.draw.circle(screen, (255,255,255), (pos_x,pos_y), 5000)
    
    
    if render:
        pygame.init()
        screen = pygame.display.set_mode((width,height))
        space = pymunk.Space()
        clock = pygame.time.Clock()
        
        # obst =[]
        # obst.append(create_obst(space))
    
        draw_options = DrawOptions(screen)
        draw_options.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES
        # draw_options.flags |= pymunk.SpaceDebugDrawOptions.DRAW_COLLISION_POINTS
        # draw_options.collision_point_color = (0,0,0,255)
        draw_options.shape_outline_color = (0,0,0,255)
    
    #### Create PyMunk Space
    space = pymunk.Space()
    space.gravity = 0, 0
    
    #### Create JAMEoEBA System
    kwargs={'space':space,
            'systemCenterLocation':systemStart,
            'systemRadius':R,
            'numBots':numBots,
            'botMass':botMass,
            'botRadius':botRadius,
            'skinMass':skinMass,
            'skinRadius':skinRadius,
            'skinRatio':skinRatio,
            'botFriction':botFriction,
            'inRadius':inRadius,
            'inMass':inMass,
            'inFriction':inFriction,
            'springK':springK,
            'springB':springB,
            'springRL':springRL,
            'maxSeparation':maxSeparation,
            'conversionRatio':ppm,
            'percentInteriorRemove':percentInteriorRemove}
    bots, interiorParticles = createJamoeba(**kwargs)
    
    print('--'*20)
    print('Number of External Bots:',len(bots))
    print('Number of interiorParticles:',len(interiorParticles))
    print('--'*20)
    
    #### Create Bounding Walls
    wallKwargs={'space':space,
                 'screenHeight':height,
                 'screenWidth':width,
                 'wallThickness':wallThickness}
    createWalls(**wallKwargs) #Creating the walls right now.
    
    # draw_obst(obst) #Drawing  the  wall
    
    #####Create  the  obstacle
    obstKwargs={'space':space,
                 'screenHeight':height,
                 'screenWidth':width}
    createObst(**obstKwargs)  #Creating  the  obstacle
    
    # draw_obst(obst)

    
    #### Simulation Loop
    if render:
        pymunk.pygame_util.positive_y_is_up=True
        
    running = True
    DisToTarget = []
    Time = []
    t= 0
    startTime = time.time()
    while running:
        if render:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.display.quit() # Terminate the pygame windowas well
                    plt.plot(Time,DisToTarget)
                    endTime = time.time()
                    print('--'*20)
                    print("Simulation runtime:",round(endTime-startTime,2),'seconds')
                    print('Num steps to end:', t)
                    print('With timestep of',dt,',this means the physical runtime in simulation was:',dt*t,'seconds')
                    print('--'*20)
                    print('Number of External Bots:',len(bots))
                    print('Number of interiorParticles:',len(interiorParticles))
                    print('--'*20)
                    del space
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    plt.plot(Time,DisToTarget)
                    pygame.display.quit()
                    endTime = time.time()
                    print('--'*20)
                    print("Simulation runtime:",round(endTime-startTime,2),'seconds')
                    print('Num steps to end:', t,'steps')
                    print('With timestep of',dt,',this means the physical runtime in simulation was:',dt*t,'seconds')
                    print('--'*20)
                    print('Number of External Bots:',len(bots))
                    print('Number of interiorParticles:',len(interiorParticles))
                    print('--'*20)
                    del space
                    sys.exit(0)
            
            screen.fill((192,192,192)) # Background color for the screen
            space.debug_draw(draw_options)
            
            pygame.display.update()
            clock.tick()
            
        
        # Apply forces on each bot
        relPositions = []
        positions = []
        # time to cross
        for index, bot in enumerate(bots):  
            # pdb.set_trace()
            botPos = bot.body.position
            relPositions.append(botPos-targetLoc)
            positions.append(botPos)
            x = botPos.x
            y = botPos.y
            bot.body.apply_force_at_world_point((1,0), (x,y))
        
        systemCenter = np.mean(positions,axis=0)
        posRelTarget = systemCenter - targetLoc
        distance = np.linalg.norm(posRelTarget)
        distance = convert.Pixels2Meters(distance)

        # The below code applies a force towards the center of the system or away, depending on the sgn of the forceApply
        # for index, bot in enumerate(bots):
        #     relPosition = positions[index]-systemCenter
        #     normalizer = np.linalg.norm(relPosition)
        #     forceApply=relPosition/normalizer
        #     if np.sin(t*.01)>0: bot.body.apply_force_at_world_point((forceApply[0],forceApply[1]), (positions[index][0],positions[index][1]))
        #     else: bot.body.apply_force_at_world_point((-forceApply[0],-forceApply[1]), (positions[index][0],positions[index][1]))
            
            
        # Find average X position
        Time.append(t*dt)
        DisToTarget.append(distance)
        
        print('Current Distance:',distance)
        
        if distance<1e-1 and render:
            if render: pygame.display.quit()
            plt.plot(Time,DisToTarget)
            endTime = time.time()
            print('--'*20)
            print("Simulation runtime:",round(endTime-startTime,2),'seconds')
            print('Num steps to end:', t,'steps')
            print('With timestep of',dt,',this means the physical runtime in simulation was:',dt*t,'seconds')
            print('--'*20)
            print('Number of External Bots:',len(bots))
            print('Number of interiorParticles:',len(interiorParticles))
            print('--'*20)
            del space
            sys.exit(0)
        
        if posRelTarget[0]>1: # Passed target
            if render: pygame.display.quit()
            plt.plot(Time,DisToTarget)
            endTime = time.time()
            print('--'*20)
            print("Simulation runtime:",round(endTime-startTime,2),'seconds')
            print('Num steps to end:', t,'steps')
            print('With timestep of',dt,',this means the physical runtime in simulation was:',dt*t,'seconds')
            print('--'*20)
            print('Number of External Bots:',len(bots))
            print('Number of interiorParticles:',len(interiorParticles))
            print('--'*20)
            del space
            sys.exit(0)
            
        if t>maxSteps and not render:
            plt.plot(Time,DisToTarget)
            endTime = time.time()
            print('--'*20)
            print("Simulation runtime:",round(endTime-startTime,2),'seconds')
            print('Num steps to end:', t,'steps')
            print('With timestep of',dt,',this means the physical runtime in simulation was:',dt*t,'seconds')
            print('--'*20)
            print('Number of External Bots:',len(bots))
            print('Number of interiorParticles:',len(interiorParticles))
            print('--'*20)
            del space
            sys.exit(0)
            
        # Take simulation step
        space.step(dt)
        if t%10==0:
            print()
            print('Executed step number',t)
            print()
        t+=1
        
        
if __name__ == "__main__":
    sys.exit(main())