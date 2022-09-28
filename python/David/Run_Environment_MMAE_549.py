# %%
# Main cell

# Step 0: Importing Parameters and all needed libraries
from env_params import *
import numpy as np
from PotentialFields import *
import pygame
from MPC_Controllers import *
from PIL import ImageDraw, Image
from time import time
from time import sleep

# Step 1: Ensure working directory is set proper
from env_params import convert

filepath = os.path.dirname(os.path.realpath(__file__))

# Step 2: Create the environment
env = robot(**envParams)

# Step 3: If you are saving a video, create folder to store temporary photos
if saveVideo:
    savefile = filepath + '\\{}\\'.format(experimentName)
    os.makedirs(savefile, exist_ok=True)

# Step 4: Always call env.reset() before running the sim!
# This creates the PyMunk space
startLoc = np.array([env.width / 2, env.height / 2])  # This starts the system at the center of the environment
startLoc += np.array([0, 0])
obs = env.reset(startLoc)  # You can also choose to not enter an argument here, and the system will spawn on the left-side


# Step 5: Define some action.
# Action should a numpy array of shape (numBots, 2)
# The first column are the forces in the x-direction for each bot, the second column are forces in the y-direction
action = np.zeros((numBots, 2))
passive_ac = None
observation = np.zeros((numBots, 6))

slowdown_factor = 5
controller_horizon = 7
ref = ImageFourierComplex([0, 0], 0, np.array([1, 1]) * 1.6, [env.width, env.height], env.ppm)
ref.generateRef('images\GeoC.png', 155, 10)
controller = FourierControlComplexMPC(numBots, env.botMass, env.dt * env.numStepsPerStep * slowdown_factor, ref, 500 / 1.09)    # max N
N = 5
controller.calculateStaticMatrices(N)


# Step 6: define Background
img = controller.ref.getImage()
env.setBackground(img)

print('Timestep time: ' + str(env.dt * env.numStepsPerStep * slowdown_factor))

env.render()
observation, _ = env.step(action, passive_ac)
t0 = time()
output = np.zeros((env.numBots, 2))
# Start simulation
for i in tqdm(range(2_500)):

    if i > 600:
        controller.mapto = .1

    # Step the environment forward a step.
    # Returns an observation now that a step was taken, and whether the environment is done.
    positions = observation[:, :2].T
    velocities = observation[:, 2:4].T
    normals = observation[:, 4:6].T
    ang_vel = observation[:, 6].T
    objData = observation[0, 7:9]

    if i % slowdown_factor == 0:

        if controller.calculating:
            controller.waitForThread()

        output = controller.output
        controller.obj_data = objData
        controller.update(positions, velocities, normals, ang_vel)


    # Constrain forces to heading angle
    action = output # np.multiply(normals, output).T
    passive_ac = None # np.zeros(numBots)

    env.render()
    observation, _ = env.step(action, passive_ac)
    plt.pause(.001)



print('End of episode')

if dataCollect:
    env.dataExport()
if saveVideo:
    createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height), 1 / (dt * numStepsPerStep))

# Make sure to call env.close() when you are done! This exists the physics engine cleanly.
# Not a requirement, just good practice.
env.close()

# Closing any plots that may have been opened.
# This is only actually needed when you dataCollect=True
plt.close('all')