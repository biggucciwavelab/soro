import numpy as np
from win32api import GetSystemMetrics
from tqdm import tqdm
from math import floor
from WegWorld import robot, createVideo, Convert


"""
Input units:
    Distance: Meters
    Force: N
    Mass: kg
    Time: Seconds
"""
r_over_arc = .0286/.1
R = .30023 # System radius. Kept constant!
C = 2*np.pi*R

numBots = 18

botRadius = C/(numBots*(2+(1/r_over_arc))) # Calculated by recognizing the ratio of bot radius and distance between bots (calculated above using the 12 bot schematic) must remain constant, but the total circumference cannot change.

dt = 1/200.0            # Simulation timestep
ppm = 583.784               # Pixels Per Meter - Defines the conversion ratio between units of meters to pixels. CRITICAL 
numStepsPerStep = 20     # Number of simulation timesteps to integrate on every call of 'step'.
botMass = .2
skinRadius = .015 # DO NOT TOUCH
skinMass = botMass# DO NOT TOUCH
skinRatio = 0     # DO NOT TOUCH
inRadius = botRadius 
botFriction = .1
inMass = .02
inFriction = .1
floorFriction = 0.1        # Coefficient of friction of floor with all objects
percentInteriorRemove = 1 # Percentage of interior particles to remove. Should be a value between 0 and 1.
springK = 3000             # Spring Stiffness
springB = 5
# revolve_springK = 1
# revolve_springB = .5
springRL = botRadius*(1/r_over_arc)
startDistance = springRL # The start distance between bots
wallThickness = .135   # Thickness of the bounding walls in the environment
maxSeparation = np.inf # The maximum distance bots are allowed to be from each other
minSeparation = 0             # The minimum distance bots are allowed to be from each other
maxNumSteps = 100_000_000           # Simulation will self-terminate after this many simulation timesteps have passed. (in the event you forgot to close out)

# Changing headings to be something we specifically call for
headings = None
headings = np.zeros(numBots)
for index, _ in enumerate(headings):
    if index%2==0:
        headings[index] = np.pi/2

# Defining system Radius
# arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
# theta = 2*np.pi/numBots
# R = arcLength/theta #**


# How large the window will be on your screen
# Using 'GetSystemMetrics', you can get the largest possible field for your screen
ratio = 1.641   # Ratio gotten from Amin's drawing. 
width = 1296    # Floor(GetSystemMetrics(0)*.9) # Value in PIXELS
height = width/ratio#floor(GetSystemMetrics(1)*.9) # Value in PIXELS

# Defining the start location for the system
# Can be left as None
convert = Convert(ppm)
width_m = convert.Pixels2Meters(width)
height_m = convert.Pixels2Meters(height)
x = width_m - (R+botRadius+wallThickness/2)
y = height_m - (R+botRadius+wallThickness/2)
systemStart = x,y

# Running parameters
experimentName = 'Parallel' # Name of experiment.
dataCollect = False # Whether to collect and store data in this simulation
saveVideo = False           # Whetehr to save the video

"""
Users should not have to edit parameters below this line
"""

envKwargs = {'dt':dt,
             'ppm':ppm,
             'numStepsPerStep':numStepsPerStep,
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
             'floorFriction':floorFriction,
             'percentInteriorRemove':percentInteriorRemove,
             'springK':springK,
             'springB':springB,
            #  'revolve_springK':revolve_springK,
            #  'revolve_springB':revolve_springB,
             'springRL':springRL,
             'wallThickness':wallThickness,
             'maxSeparation':maxSeparation,
             'minSeparation':minSeparation,
             'headings':headings,
             'dataCollect':dataCollect,
             'experimentName':experimentName,
             'saveVideo':saveVideo,
             'systemStart':systemStart}