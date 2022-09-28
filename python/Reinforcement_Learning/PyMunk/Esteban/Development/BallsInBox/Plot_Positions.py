"""
An accompanying script to plot the positions of the bouncing balls simulation
"""
# %%

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = "serif"
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

non_shift = np.load('Shift False.npy')
shift = np.load('Shift True.npy')
next_shift = np.load('Next Shift True.npy')

def plot_positions(non_shift, shift):
    """
    Given two arrays of positions, shifted and non_shifted.
    Plotting both
    """
    fig, ax = plt.subplots()

    # Plot the non shifted positions
    ax.plot(non_shift[:,0],non_shift[:,1],label='Original')
    ax.scatter(non_shift[0,0],non_shift[0,1])

    # Plot the shifted positions
    # ax.plot(shift[:,0],shift[:,1],label='Shifted')
    # ax.scatter(shift[0,0],shift[0,1])

    # Plot Next Shifted
    ax.plot(next_shift[:,0],next_shift[:,1],label='Next Shifted')

    ax.set_xlabel('X [meters]')
    ax.set_ylabel('Y [meters]')
    ax.legend()
    plt.show()

plot_positions(non_shift, shift)
