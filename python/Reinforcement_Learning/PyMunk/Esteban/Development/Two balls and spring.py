# %% Main cell
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 27 15:52:21 2021

@author: elopez8

Example of two balls connected with a spring
"""


import pymunk
import pygame
import numpy as np
from math import floor
import sys
from pymunk.pygame_util import DrawOptions
from win32api import GetSystemMetrics
from conversion import Convert

width = GetSystemMetrics(0)//2
height = floor(GetSystemMetrics(1)*.99)
center = width//2, height//2

# Simulation parameters
ppm = 500 # Pixels Per Meter
convert = Convert(ppm)
ballMass = 1
ballRadius = .050 # 5cm
friction = 1
distance_between_balls = ballRadius*4

# Sring Parameters
k = 400 # Spring stiffness
b = 0 # Spring damping
rl = distance_between_balls/2.5

# Convert simulation parameters to pixels
ballRadius = convert.Meters2Pixels(ballRadius)
distance_between_balls = convert.Meters2Pixels(distance_between_balls)
rl = convert.Meters2Pixels(rl)
k = convert.SpringK2Pixels(k)

# Calculating Ball Locations:
pos1 = center[0] - 0.5*distance_between_balls, center[1]
pos2 = center[0] + 0.5*distance_between_balls, center[1]

class Ball:
    def __init__(self, space, position, radius, mass, friction):
        self.body = pymunk.Body()
        self.radius = radius
        self.body.position = position
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.mass = mass
        self.shape.friction = friction
        space.add(self.body, self.shape)
        
        
def connectBalls(space, b1, b2, rest_length, spring_stiffness, spring_damping, maxSeparation):
    springConstraint = pymunk.DampedSpring(b1.body, b2.body, (b1.radius, 0), (-b2.radius, 0), rest_length, spring_stiffness, spring_damping)
    slideJoint = pymunk.SlideJoint(b1.body, b2.body, (b1.radius, 0), (-b2.radius, 0), 0, maxSeparation)
    space.add(springConstraint, slideJoint)
    return None
    
def main():
    time = []
    position = []
    dt = 1/50.0
    # dt=.005
    pygame.init()
    screen = pygame.display.set_mode((width,height))
    clock = pygame.time.Clock()
    
    draw_options = DrawOptions(screen)
    
    space = pymunk.Space()
    space.gravity = 0, 0
    
    #Add items to space
    ball1 = Ball(space, pos1, ballRadius, ballMass, friction)
    ball2 = Ball(space, pos2, ballRadius, ballMass, friction)
    connectBalls(space, ball1, ball2, rl, k, b, np.inf)
    
    pymunk.pygame_util.positive_y_is_up=True
    
    # Simulation Loop
    t=0
    while True:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.display.quit() # Terminate the pygame windowas well
                sys.exit(0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.display.quit()
                sys.exit(0)                             
            
        screen.fill((255,255,255)) # Background color for the screen
        space.debug_draw(draw_options)
        space.step(dt)
        pygame.display.update()
        clock.tick() # By removing this, the video will keep up with the simulation runtime. Hence, REMOVE THIS VALUE FOR FAST TRAINING

if __name__ == "__main__":
    sys.exit(main())
# %%
