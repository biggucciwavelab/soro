"""
This is a custom environment to test how we would implement sliding friction
similar to how it is done in the tank environment
"""

import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = "serif"
# plt.rcParams['mathtext.fontset'] = 'dejavuserif'
from win32api import GetSystemMetrics
from math import floor
from tqdm import tqdm
import pymunk
import pymunk.pygame_util
import pygame

class Convert:
    def __init__(self, conversion_ratio=100):
        """
        Parameters
        ----------
        conversion_ratio : float
            The conversion ratio between pixels and meters in the form (pixels/meter).
        """
        self.ratio = conversion_ratio
        
    def Pixels2Meters(self, num_pixels):
        return (num_pixels*(1/self.ratio))
    
    def Meters2Pixels(self, meters):
        return (meters*self.ratio)
    
    def SpringK2Pixels(self, springK):
        return (springK*(1/self.ratio))
    
    def Pixels2SpringK(self, springKPixels):
        return (springKPixels*self.ratio)

def add_main_box(space, box_length, mass):
    global height
    body = pymunk.Body()
    body.position = 5 + box_length/2, height/2
    shape = pymunk.Poly.create_box(body, (box_length, box_length), 0.0)
    shape.mass = mass
    shape.friction = 0.0
    shape.color = (0, 0, 150, 255)
    space.add(body, shape)
    return body



def add_friction(space, b2, friction):
    static_body = space.static_body
    pivot = pymunk.PivotJoint(static_body, b2, (0,0), (0,0))
    space.add(pivot)
    pivot.max_bias = 0 # Disable joint correction
    pivot.max_force = friction


def create_space(mass, box_length, fake_friction = np.inf):
    space = pymunk.Space()
    space.gravity = 0,0

    main_box = add_main_box(space, box_length, mass)
    add_friction(space, main_box, friction)
    return space, main_box


def render_setup(width, height):
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    # draw_options.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES

    pymunk.pygame_util.positive_y_is_up = True

    return screen, draw_options, clock    


def collect_data(main_box, data_storage, index):
    global convert
    main_vel = convert.Pixels2Meters(main_box.velocity.length)
    data_storage[index] = np.array([index, main_vel])
    return data_storage


def plot_data(data_storage):
    plt.figure()
    plt.plot(data_storage[:,0], data_storage[:,1])
    plt.legend()
    plt.ylabel('Speed (m/s)')
    plt.xlabel('Time step')
    plt.title('Box Velocity')
    plt.show()

def run_sim(space, width, height, num_steps, dt,
            main_box, initial_force):
    
    # Apply an impulse
    main_box.apply_impulse_at_local_point((initial_force, 0), (0,0))

    # Setup simulation
    screen, draw_options, clock = render_setup(width, height)
    
    # Create empty array to store data
    data_storage = np.zeros((num_steps, 2))

    for i in tqdm(range(num_steps)):
        space.step(dt)

        # Kill the sim if we exit
        for event in pygame.event.get():
            if (
                event.type == pygame.QUIT
                or event.type == pygame.KEYDOWN
                and (event.key in [pygame.K_ESCAPE, pygame.K_q])
            ):
                exit()

        screen.fill(pygame.Color("gray"))
        space.debug_draw(draw_options)
        pygame.display.update()
        clock.tick()
        data_storage = collect_data(main_box, data_storage, i)


    pygame.display.quit()
    plot_data(data_storage)

if __name__=="__main__":
    #---------------------
    # Define Parameters
    # --------------------

    friction = 1    # The friction UNDER INVESTIGATION
    mass = 1             # kg
    initial_force = 10   # N
    box_length = 1      # m
    ppm = 100            # Pixels per meter
    dt = 1/60            # Timestep
    numSteps = 1_000      # Number of simulation steps


    # Get screen parameters
    global width
    global height
    width = floor(GetSystemMetrics(0)*.9)
    height = floor(GetSystemMetrics(1)*.9)

    # Get conversion of units
    global convert
    convert = Convert(ppm)
    box_length = convert.Meters2Pixels(box_length)

    # Create simulation
    space, main_box = create_space(mass, box_length, friction)

    # Run simulation
    run_sim(space, width, height, numSteps, dt, main_box, initial_force)