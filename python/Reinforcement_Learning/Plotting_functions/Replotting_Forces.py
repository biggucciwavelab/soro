"""
This script will read and replot the forces in actions.csv. 

This is needed to get Times New Roman font in the title and side.
"""

import numpy as np
import matplotlib.pyplot as plt
import os

plt.rcParams['font.family'] = "serif"
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

# Create a save folder
saveLoc = 'Actions Replotted/'
os.makedirs(saveLoc, exist_ok=True)

# Load data
actions = np.genfromtxt('actions.csv',delimiter=',')
# Separate the time from the actual actions
time = actions[:,0]
actions = actions[:,1:]

num_steps, num_bots = actions.shape
num_bots /= 2 # We have an X and Y force for each bot, so divide by 2
num_bots = int(num_bots)

# Iterate through the actions and separate X and Y actions
x_force = np.zeros((num_steps,num_bots))
y_force = np.zeros((num_steps, num_bots))
for time in range(num_steps):
    for bot in range(num_bots):
        x_force[time, bot] = actions[time, 2*bot-2]
        y_force[time,bot] = actions[time, 2*bot-1]

# Now plot!
for bot in range(num_bots):

    title = f"Bot {bot+1} Forces"

    fig, ax = plt.subplots()
    ax.set_title(title)
    ax.plot(x_force[:,bot], label='X-Force')
    ax.plot(y_force[:,bot], label='Y-Force')
    ax.legend()
    plt.savefig(saveLoc + title + '.png')
    plt.close(fig)