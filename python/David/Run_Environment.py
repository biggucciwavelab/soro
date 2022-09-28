# %% 
# Main cell

# Step 0: Importing Parameters and all needed libraries
from env_params import *
import numpy as np
from PotentialFields import *
import pygame

# Step 1: Ensure working directory is set proper
from env_params import convert

filepath = os.path.dirname(os.path.realpath(__file__))

# Step 2: Create the environment
env = robot(**envParams)

# Step 4: Always call env.reset() before running the sim!
# This creates the PyMunk space
startLoc = np.array([env.width/2, env.height/2]) # This starts the system at the center of the environment
obs = env.reset(startLoc)                        # You can also choose to not enter an argument here, and the system will spawn on the left-side

# Step 5: define controller 
# Ellipse semimajor and semiminor axes:
sem_major = 1    # in m
sem_minor = 1    # in m
# Inclination (angle from x axis to sem_major:
alf = 30.            # in °
# Scale factor
k = .05
center = np.array([0, 0])   # in m and relative to center

# Calculate rotation matrix and define parameter
M_mat = np.diag([1/sem_major, 1/sem_minor])**2
c, s = np.cos(alf*np.pi/180), np.sin(alf*np.pi/180)
R = np.array(((c, -s), (s, c)))
M_mat = R.T @ M_mat @ R
center = center + np.array([env.width/2, env.height/2])

pf1 = RadialOperator(WideSinkFunction(center, M_mat, 800, .01))
pf2 = RadialOperator(EllipseFunction(center, M_mat, 220))
pot_field = pf1 + pf2

# Step 6: Define some action.
# Action should a numpy array of shape (numBots, 2)
# The first column are the forces in the x-direction for each bot, the second column are forces in the y-direction
action = np.zeros((numBots, 2))
passive_action = np.ones((numBots*skinRatio))

observation = np.zeros((numBots, 6))

img1 = pf1.getImage([env.width, env.height], [convert.Meters2Pixels(env.width), convert.Meters2Pixels(env.height)])
img2 = pot_field.getImage([env.width, env.height], [convert.Meters2Pixels(env.width), convert.Meters2Pixels(env.height)])
env.background = img2

k = 2
for i in tqdm(range(10_000)):
    # You can choose to NOT render the simulation if all you want to do is collect data
    # This drastically speeds up the wall-clock time, as now there is no need to render all objects
    # and only the physics engine is running
    env.render()

    # Step the environment forward a step.
    # Returns an observation now that a step was taken, and whether the environment is done.
    positions = observation[:, :2].T
    velocities = observation[:, 2:4].T
    normals = observation[:, 4:].T


    if i > 5:
        sem_major = 1.5 * max((1-i/300/k), 0.6)  # in m
        sem_minor = 0.8 * max((1-i/300/k), 0.6) # in m

        off = np.reshape(np.array([np.sin(i/60/k)*1.5, .5*np.sin(i/80/k)*1]) + np.array([env.width/2, env.height/2]), (2, 1))
        # Inclination (angle from x axis to sem_major:
        alf = 30. + i/1.8/k  # in °

        # Calculate rotation matrix and define parameter
        M_mat = np.diag([1 / sem_major, 1 / sem_minor]) ** 2
        c, s = np.cos(alf * np.pi / 180), np.sin(alf * np.pi / 180)
        R = np.array(((c, -s), (s, c)))
        M_mat = R.T @ M_mat @ R

        pot_field.operators[0].transform = M_mat
        #pot_field.operators[0].offset = off
        pot_field.operators[1].transform = M_mat
        #pot_field.operators[1].offset = off

        #pf2 = RadialOperator(pot_field.operators[-1])

        #if i == 500:
        #    pot_field.operators[0].multiplier = 300

        env.background = pot_field.getImage([env.width, env.height],
                             [convert.Meters2Pixels(env.width), convert.Meters2Pixels(env.height)])

        # Change spring constant...
        #for s in env.springs:
        #    s.stiffness = s.stiffness * 1.05

    _, grads = pot_field.get_field_and_grad(positions)

    action = -grads*.01 - velocities * 10 #0.0035

    # Constrain forces to heading angle
    action = np.sum(action * normals, axis=0) * normals

    passive_ac = np.max(action)*passive_action # LOOK HERE. This is the change in normal forces to passive membrane

    observation, _ = env.step(action.T, passive_ac)

print('End of episode')

if dataCollect:
    env.dataExport()
if saveVideo: 
    createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height))

# Make sure to call env.close() when you are done! This exists the physics engine cleanly. 
# Not a requirement, just good practice.
env.close()

# Closing any plots that may have been opened. 
# This is only actually needed when you dataCollect=True
plt.close('all') 