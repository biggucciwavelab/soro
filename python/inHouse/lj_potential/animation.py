# -*- coding: utf-8 -*-
"""
Created on Wed Nov 28 15:38:03 2018

@author: dmulr
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation



data = np.load('temp.npz')


x=data['x']
y=data['y']
n=data['n']
nb=data['nb']
ni=data['ni']
nsteps=np.size(x[:,0])

fig = plt.figure()
ax = plt.axes(xlim=(-25, 25), ylim=(-25, 25))
plt.title('Animation')
line, = ax.plot([], [], lw=1, marker='o',color='black', markersize=5)
line1, = ax.plot([], [], lw=0, marker='o',color='red',markersize=10)

# initialization function: plot the background of each frame
def init():
    line.set_data([],[])
    line1.set_data([],[])
    return line,line1,

# animation function.  This is called sequentially
def animate(i,x,y):
    line.set_data(x[i,0:nb], y[i,0:nb])
    line1.set_data(x[i,nb:n], y[i,nb:n])
    return line,line1,

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=nsteps, fargs=(x,y), interval=1, blit=True)

# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
#anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

plt.show()
