import numpy as np
from win32api import GetSystemMetrics
from tqdm import tqdm
from math import floor
from WegWorld_v2 import robot, createVideo, Convert


"""
Input units:
    Distance: Meters
    Force: N
    Mass: kg
    Time: Seconds
"""

# Any parameters below this can be varied.
numBots = 120

# The following parameters are considered CONSTANT, per Amin's specification
arc_between_bots = .15

C = numBots * arc_between_bots
R = C / (2 * np.pi)

R = arc_between_bots / (2 * np.sin(np.pi / 12))


R = .29 / 1.05
arc_between_bots = R * 2 * np.sin(np.pi / numBots)

dt = 1/200.0            # Simulation timestep
botRadius = .025
ppm = int(400 * .15 / arc_between_bots)#1100               # Pixels Per Meter - Defines the conversion ratio between units of meters to pixels. CRITICAL



dt = 1/200.0            # Simulation timestep
dt_cont = .1
botRadius0 = .058/2
botRadius = botRadius0 * 12 / numBots
numStepsPerStep = 5     # Number of simulation timesteps to integrate on every call of 'step'.
botMass = .416/2 * 12 / numBots           # kg (4.993 / 12), individual unit .358
skinRadius = .015 # DO NOT TOUCH
skinMass = .1 #botMass# DO NOT TOUCH
skinRatio = 0     # DO NOT TOUCH
inRadius = botRadius 
botFriction = .17
inMass = .02
inFriction = .17
floorFriction = 0.        # Coefficient of friction of floor with all objects
percentInteriorRemove = 1 # Percentage of interior particles to remove. Should be a value between 0 and 1.
springK = 300_000 / 12**2 * numBots**2#100_000 // 600_000 // 1_800_000 // 18_000_000             # Spring Stiffness
springB = 300
springRL = botRadius*2 - arc_between_bots
startDistance = springRL # The start distance between bots
maxSeparation = np.inf # The maximum distance bots are allowed to be from each other
minSeparation = 0             # The minimum distance bots are allowed to be from each other
maxNumSteps = 10_000           # Simulation will self-terminate after this many simulation timesteps have passed. (in the event you forgot to close out)
automatic_window_size = True
large_map = False


# Changing headings to be something we specifically call for
headings = None
headings = np.zeros(numBots)
for index, _ in enumerate(headings):
    if index % 2==0:
        headings[index] = 0*np.pi/2


# How large the window will be on your screen
# Using 'GetSystemMetrics', you can get the largest possible field for your screen

if automatic_window_size:
    cube_L_ratio = 2.001996 *1.05# Ratio of the longest dimension of the cube to the system radius, R.
    cube_S_ratio = 0.415799 *1.05# Ratio of the shortest dimension of the cube to the system radius, R.

    # We define the system start
    # We set it so that the top-right corner of the screen is the top-right corner of the obstacle field
    cube_L = cube_L_ratio*(R + botRadius0) # Longest dimension of cube
    cube_S = cube_S_ratio*(R + botRadius0) # Shortest dimension of cube

    # Set width and height
    if large_map:
        width = (2*cube_S + 5*cube_L) * ppm + 1
        height = (5*cube_L - 2*cube_S) * ppm + 1
    else:
        width = (2 * cube_S + 3 * cube_L) * ppm + 1
        height = (2 * cube_L - cube_S) * ppm + 1
else:
    width = 2200
    height = 1200

# Running parameters
experimentName = 'FSIM_120_bots_norm' # Name of experiment.
dataCollect = True# Whether to collect and store data in this simulation
saveVideo = True# Whetehr to save the video

"""
Users should not have to edit parameters below this line
"""

envKwargs = {'dt':dt,
             'ppm':ppm,
             'numStepsPerStep':numStepsPerStep,
             'screenHeight':height,
             'screenWidth':width,
             'maxNumSteps':maxNumSteps,
             'R': R,
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
             'floorFriction':floorFriction,
             'percentInteriorRemove':percentInteriorRemove,
             'springK':springK,
             'springB':springB,
             'springRL':springRL,
             'maxSeparation':maxSeparation,
             'minSeparation':minSeparation,
             'headings':headings,
             'dataCollect':dataCollect,
             'experimentName':experimentName,
             'saveVideo':saveVideo,
             'large_map': large_map}