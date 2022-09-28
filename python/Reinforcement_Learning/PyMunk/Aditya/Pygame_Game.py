# -*- coding: utf-8 -*-
"""
Created on Mon Feb 15 11:24:57 2021

@author: adity
"""

import pymunk,sys,pygame, random

def draw_floor():
    screen.blit(floor_surface,(floor_xpos,900))
    screen.blit(floor_surface,(floor_xpos+576,900))
    
def create_pipe():
    pipe_h = random.choice(pipe_height)
    new_pipe = pipe_surface.get_rect(midetop = (700,pipe_h))
    top_pipe = pipe_surface.get_rect(midbottom = (700,pipe_h - 300))
    return new_pipe, top_pipe

def move_pipes(pipes):
    for pipe in pipes:
        pipe.centerx -= 5
    return pipes

def draw_pipes(pipes):
    for pipe in pipes:
        screen.blit(pipe_surface, pipe)
    
screen = pygame.display.set_mode((576,1024))
clock = pygame.time.Clock()
gravity =   1
bird_move = 0

bg_surface = pygame.image.load('assets/background-day.png')
bg_surface = pygame.transform.scale2x(bg_surface)

floor_surface = pygame.image.load('assets/base.png')
floor_surface = pygame.transform.scale2x(floor_surface)

bird = pygame.image.load('assets/bluebird-midflap.png')
bird_rect = bird.get_rect(center = (100,576))

pipe_surface = pygame.image.load('assets/pipe-green.png')

pipe_list = []
SPAWNPIPE = pygame.USEREVENT
pygame.time.set_timer(SPAWNPIPE, 1200)
pipe_height = [400,600,800]



floor_xpos = 0

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                bird_move = 0
                bird_move -= 12
        if event.type == SPAWNPIPE:
            pipe_list.extend(create_pipe())
    
    screen.blit(bg_surface(0,0))
    # Bird
    bird_move += gravity
    bird_rect.centery += bird_move
    screen.blit(bird,bird_rect)
    
    #Pipes
    pipe_list = move_pipes(pipe_list)
    draw_pipes(pipe_list)
    
    floor_xpos-=1
    draw_floor()
    if floor_xpos <= -576:
        floor_xpos = 0
        
        
    
    
    
    pygame.display.update() #Draws  the  objects  on  the  screen
    clock.tick(120)
    