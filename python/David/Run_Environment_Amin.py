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

# Step 3: If you are saving a video, create folder to store temporary photos
if saveVideo:   
    savefile = filepath + '\\{}\\'.format(experimentName)
    os.makedirs(savefile, exist_ok=True)

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
off = center

pf1 = RadialOperator(WideSinkFunction(center, M_mat, 800, .01))
pf2 = RadialOperator(EllipseFunction(center, M_mat, 220))
pot_field = pf1 + pf2

# Step 6: Define some action.
# Action should a numpy array of shape (numBots, 2)
# The first column are the forces in the x-direction for each bot, the second column are forces in the y-direction
action = np.zeros((numBots, 2))

observation = np.zeros((numBots, 6))

img1 = pf1.getImage([env.width, env.height], [convert.Meters2Pixels(env.width), convert.Meters2Pixels(env.height)])
img2 = pot_field.getImage([env.width, env.height], [convert.Meters2Pixels(env.width), convert.Meters2Pixels(env.height)])
env.background = img2
off_v = np.reshape(np.array([0, 0]), (2, 1))

k = 2
w = np.pi/180/1.8/k/2
pressure = 0
pressure_targ = -5
leakage = 0.02
press_list = []
timestamps = []
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot([], [], 'r-')
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
        sem_major = 1.5 * max((1-i/300/k), 0.5)  # in m
        sem_minor = 0.8 * max((1-i/300/k), 0.5) # in m

        tmp_k = 10000
        v1 = 120 * tmp_k
        v2 = 160 * tmp_k

        off = np.reshape(np.array([np.sin(i/v1/k)*1.5, .5*np.sin(i/v2/k)*1]) + np.array([env.width/2, env.height/2]), (2, 1))

        off_v = np.reshape(np.array([np.cos(i/v1/k)*1.5 /v1/k / env.dt, .5*np.sin(i/v2/k)/v2/k / env.dt]), (2, 1))
        # Inclination (angle from x axis to sem_major:
        alf = 30./180*np.pi + i*w  # in °

        # Calculate rotation matrix and define parameter
        M_mat = np.diag([1 / sem_major, 1 / sem_minor]) ** 2
        c, s = np.cos(alf), np.sin(alf)
        R = np.array(((c, -s), (s, c)))
        M_mat = R.T @ M_mat @ R

        pot_field.operators[0].transform = M_mat
        pot_field.operators[0].offset = off
        pot_field.operators[1].transform = M_mat
        pot_field.operators[1].offset = off

        #pf2 = RadialOperator(pot_field.operators[-1])

        #if i == 500:
        #    pot_field.operators[0].multiplier = 300

        env.background = pot_field.getImage([env.width, env.height],
                             [convert.Meters2Pixels(env.width), convert.Meters2Pixels(env.height)])

        # Change spring constant...
        #for s in env.springs:
        #    s.stiffness = s.stiffness * 1.05

    M_mat2 = M_mat/1.5#np.array([[.75, 0], [0, 1]])
    off2 = off
    off2 = off2.reshape((2,))



    beta = .99
    r = 0.8
    posits_des = []
    for posit in positions.T:
        p_tmp = posit - off2
        tmp = M_mat2 @ p_tmp
        d = np.linalg.norm(tmp)
        p_out = (r * (1 - beta) + d * beta) * p_tmp / d + off2
        posits_des.append(p_out)

    # List of desired positions p_k bar
    posits_des = np.array(posits_des).T

    # List of desired velocities v_k bar
    #velocities_des = np.zeros(posits_des.shape)
    tmp = positions.T - off.T#positions.T - off.T
    velocities_des = w/env.dt * np.array([-tmp[:, 1], tmp[:, 0]]) + off_v
    #velocities_des = velocities_des*(1 - beta) + velocities * beta

    # Target mass flow:
    Q_dot = (pressure_targ - pressure) * 2


    # Constants
    dt = env.dt
    alfa = 0 * dt**2
    mu = 1.
    phi = .001
    N = 20

    pressure *= (1 - leakage)
    pressure += Q_dot * dt

    timestamps.append(i)
    press_list.append(pressure)

    lim_x = [np.min(timestamps), np.max(timestamps)]
    lim_y = [np.min(press_list), np.max(press_list)]

    line1.set_ydata(press_list)
    line1.set_xdata(timestamps)

    ax.set_ylim(lim_y)
    ax.set_xlim(lim_x)

    fig.canvas.draw()
    plt.pause(.001)


    tmp = posits_des + velocities_des * 2 * alfa / dt - positions - velocities * (dt + 2 * alfa / dt)
    u_hat = np.sum(tmp * normals, axis=0) / (mu * 2 * alfa + dt ** 2 / 2)
    if phi == 0:
        lam = 0
    else:
        lam = (phi * np.sum(u_hat) - Q_dot) / (N * phi**2 / (mu**2 * dt **2 * (2*alfa + dt**2 / 2)))
    scale = phi / (mu**2 * dt **2 * (2*alfa + dt**2 / 2))
    action = 0.001 * (u_hat - lam * scale)


    # Constrain forces to heading angle
    action = action * normals #- velocities*.2

    passive_ac = np.ones((numBots*skinRatio)) * pressure / 5
    print(str(np.round(pressure, 2)) + ' ' + str(np.round(Q_dot, 2)))
    
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