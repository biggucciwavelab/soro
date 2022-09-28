# -*- coding: utf-8 -*-
"""
Live plot test
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from itertools import count

# def ama(i,index,ax,xs,ys):
#     # Counter
#     x=next(index)
#     print(x)
    
#     # Data
#     data = np.random.rand(2)
    
#     # Read data
#     xs.append(x); ys.append(data[1])
    
#     # Trim arrays
#     if x>40:
#         xs.pop(0)
#         ys.pop(0)
#         ax1.clear()
    
#     ax.set_title('aaaa')
#     ax.plot(xs, ys)

# style.use('fivethirtyeight')

# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)
# index=count()
# # Plot arrays
# x = []; y = []

# ani = animation.FuncAnimation(fig, ama, fargs=(index,ax1,x,y))

# #ani.save('ayo.mp4',writer='ffmpeg',fps=30)


def ama(i,index,ax,xs,ys):
    # Counter
    x=next(index)/10
    
    # Read data
    xs.append(x); ys.append(np.sin(x))
    
    # Trim arrays
    if x>2*np.pi:
        xs.pop(0)
        ys.pop(0)
        ax1.clear()
    
    ax.set_title('aaaa')
    ax.plot(xs, ys, 'b--')

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
index=count()
# Plot arrays
x = []; y = []

ani = animation.FuncAnimation(fig, ama, interval=100, fargs=(index,ax1,x,y), save_count=200)

ani.save('ayo.mp4',writer='ffmpeg',fps=30)