# -*- coding: utf-8 -*-
"""
Created on Mon Feb 15 10:08:51 2021

@author: adity
"""

import pygame,sys,pymunk

def create_ball(space):
    body = pymunk.Body(1, 100, body_type = pymunk.Body.DYNAMIC)
    body.position(400,0)
    shape = pymunk.Circle(body,80)#define  the  radius  and  then  the  body  type  from  above
    space.add(body,shape)
    return shape

def draw_ball(balls):
    for ball in balls:
        pos_x = int(ball.body.position.x)
        pos_y = int(ball.body.position.y)
        pygame.draw.circle(screen,(0,0,0),(pos_x,pos_y),80)
        #rec_obj = rec_obj_surface.get_rect(center = (pos_x,pos_y))
        #screen.blit(rec_obj_surface,rec_object)
        
def static_ball(space):
    body1 = pymunk.Body(body_type = pymunk.Body.STATIC)
    body1.position(400,400)
    shape1 = pymunk.Circle(body1,50)
    space.add(body1,shape1)
    return shape1

def draw_static_ball(static_balls):
    for static_ball in static_balls:
        pos_x = int(static_ball.body.position.x)
        pos_y = int(static_ball.body.position.y)
        pygame.draw.circle(screen,(0,0,0),(pos_x,pos_y),50)
        
    

pygame.init()
screen = pygame.display.set_mode((900,800))
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = (0,250) #gravit!!!
#Create  the  falling  object  list
balls = []
balls.append(create_ball(space))
#Create  the  obstacle  list
static_balls = []
static_balls.append(static_ball(space))

#Running  the  Simulation
while True:
    for episode in pygame.event.get():
        if episode.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
            
    screen.fill((217,217,217))
    draw_ball(balls) # Need  to  draw  the  falling  balls
    draw_static_balls(static_balls) # Need  to  draw  the  obstacle  balls
    #Some  random  standard  shit
    space.step(1/50)
    pygame.display.update()
    clock.tick(120)