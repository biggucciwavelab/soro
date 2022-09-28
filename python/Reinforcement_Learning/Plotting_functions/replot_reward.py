"""
Replotting the reward for this test. Mostly to get the title fixed.
"""
# %%

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter

plt.rcParams['font.family'] = "serif"
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size']= 22

def millions_formatter(x, pos):
    return f'{int(x / 1000000)}'

data = np.genfromtxt('episode_reward.csv', delimiter=',')
smooth_data = np.genfromtxt('episode_reward_009_Smoothed.csv', delimiter=',')

# Normalize the rewards
data[:,3] /=  np.nanmax(data[:,3])
smooth_data[:,3] /= np.nanmax(smooth_data[:,3])

title = 'Reward During Training'

fig, ax = plt.subplots()
ax.plot(data[:,1],data[:,3],color='darkorange',alpha=.5)
ax.plot(smooth_data[:,1],smooth_data[:,3],color='darkorange',alpha=1)
ax.set_xlabel('Time step (millions)')
ax.set_ylabel('Value')
ax.xaxis.set_major_formatter(FuncFormatter(millions_formatter))
ax.set_title(title)
plt.tight_layout()
plt.savefig(title +'.png', dpi=300)
plt.close(fig)
