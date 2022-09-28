import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt('Full_120_simulation/observations.csv', skiprows=1, delimiter=',')

time = data[:, 0]
px = data[:, 1::8].flatten()
py = data[:, 2::8].flatten()

nx = data[:, 5::8].flatten()
ny = data[:, 6::8].flatten()

d = data[:, 7::8].flatten()
t = data[:, 8::8].flatten()

x = (px + d * nx)[t == 1]
y = (py + d * ny)[t == 1]

plt.plot(x, y, '*')
plt.show()