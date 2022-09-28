# %% Main cell
"""
Will simulate balls bouncing in a box
"""

import pymunk
import pygame
import numpy as np
import sys
import os
from pymunk.pygame_util import DrawOptions
from conversion import Convert
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = "serif"
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

# Whether to shift the ball a little
shift = True

# Whether to save images for a video
saveVideo = True
if saveVideo:
    videoFolder = "BallsInBoxPics/"
    os.makedirs(videoFolder, exist_ok=True)


# Box Parameters, in pixels
width = 1440
height = 1080
wallThickness = 5
center = width//2, height//2

# Simulation parameters
ppm = 250 # Pixels Per Meter
convert = Convert(ppm)
ballMass = 1
ballRadius = .050 # 5cm
elastic = 0.95
friction = .3
distance_between_balls = ballRadius*4 # With buffer

# Convert simulation parameters to pixels
g = convert.Meters2Pixels(9.81)
ballRadius = convert.Meters2Pixels(ballRadius)
distance_between_balls = convert.Meters2Pixels(distance_between_balls)

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
        self.shape.elasticity = elastic
        self.shape.friction = friction
        space.add(self.body, self.shape)


def createBox(space, screenHeight, screenWidth, wallThickness):
    """
    Creates walls for simulation
    """
    # All values for this function are assumed to already be in Pixels
    
    # Bottom Wall
    body1 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape1 = pymunk.Poly.create_box(body1, (screenWidth,wallThickness))
    shape1.body.position = (screenWidth//2,wallThickness//2)
    shape1.elasticity = elastic
    
    # Top Wall
    body2 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape2 = pymunk.Poly.create_box(body2, (screenWidth,wallThickness))
    shape2.body.position = (screenWidth//2, screenHeight-wallThickness//2)
    shape2.elasticity = elastic

    # Left wall
    body3 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape3 = pymunk.Poly.create_box(body3,(wallThickness, screenHeight))
    shape3.body.position = (wallThickness//2, screenHeight//2)
    shape3.elasticity = elastic

    # Right wall
    body4 = pymunk.Body(0,0,body_type=pymunk.Body.STATIC)
    shape4 = pymunk.Poly.create_box(body4,(wallThickness, screenHeight))
    shape4.body.position = (screenWidth, screenHeight//2)
    shape4.elasticity = elastic

    space.add(shape1,body1,shape2,body2,shape3,body3,shape4,body4)
    
    return None


def plot_positions(positions):
    """
    Given an array (in list form) of position, will plot the positions!
    """
    positions = np.asarray(positions)
    fig, ax = plt.subplots()
    ax.plot(positions[:,0],positions[:,1])
    ax.scatter(positions[0,0],positions[0,1],c='red')
    ax.set_xlabel('X [meters]')
    ax.set_ylabel('Y [meters]')
    plt.show()

def main(shift=False):
    time = []
    position = []
    dt = 1/50.0
    dt = 1/200.0
    pygame.init()
    screen = pygame.display.set_mode((width,height))
    clock = pygame.time.Clock()
    
    draw_options = DrawOptions(screen)
    
    space = pymunk.Space()
    space.gravity = 0, -g

    # Specify Ball Locations
    posX_top = np.arange(wallThickness, width-wallThickness, distance_between_balls)
    posX_bottom = np.arange(wallThickness+ballRadius, width-wallThickness, distance_between_balls)
    posY_top = height-(wallThickness + ballRadius)
    posY_bottom  = posY_top - (3*ballRadius)
    top_pos = []
    bot_pos = []
    for X in posX_top:
        top_pos.append([X,posY_top])
    for X in posX_bottom:
        bot_pos.append([X,posY_bottom])

    # Create the balls
    balls = []
    for pos in top_pos: # Top balls
        ball = Ball(space, pos, ballRadius, ballMass, friction)
        balls.append(ball)
    for pos in bot_pos: # Bottom balls
        ball = Ball(space, pos, ballRadius, ballMass, friction)
        balls.append(ball)
    
    # Color the first ball a different color
    mid = len(balls)//4
    track = balls[mid]
    move = balls[mid+1]
    track.shape.color = (0,255,0,255)
    if shift:
        pos_now = move.body.position
        pos_now = pos_now[0]+ballRadius*1.5, pos_now[1]
        move.body.position = pos_now
    

    # Create the walls
    createBox(space,height,width,wallThickness)

    pymunk.pygame_util.positive_y_is_up=True
    
    # Simulation Loop
    t=0
    step = 0
    time = []
    track_positions = []
    while True:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.display.quit() # Terminate the pygame windows well
                # plot_positions(track_positions)
                title = 'Shift {}'.format(shift)
                # np.save(title,np.asarray(track_positions))
                sys.exit(0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.display.quit()
                # plot_positions(track_positions)
                title = 'Shift {}'.format(shift)
                # np.save(title,np.asarray(track_positions))
                sys.exit(0)
        
        if step>= 1210:
            pygame.display.quit()
            title = 'Next Shift {}'.format(shift)
            np.save(title,np.asarray(track_positions))
            sys.exit(0)

            
        screen.fill((255,255,255)) # Background color for the screen
        space.debug_draw(draw_options)
        space.step(dt)
        t+=dt
        step+=1
        pygame.display.update()
        clock.tick() # By removing this, the video will keep up with the simulation runtime. Hence, REMOVE THIS VALUE FOR FAST TRAINING

        # Save video, if needed
        if saveVideo:
            pygame.image.save(screen, videoFolder+'image%06d.jpg' % step)


        # Gather data on tracker
        time.append(t)
        pos = np.asarray(track.body.position)
        pos = convert.Pixels2Meters(pos)
        track_positions.append(pos)


if __name__ == "__main__":
    sys.exit(main(shift))