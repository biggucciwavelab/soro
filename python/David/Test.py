import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, cos, sin, real, imag
from elliptic2 import *
import cv2
from scipy.signal import convolve, square
from MPC_Reference_Generators import *
from math import floor
from win32api import GetSystemMetrics

width = floor(GetSystemMetrics(0)*1.2) # [pixels] Width of simulation window. This will work natively on your machine.
height = floor(GetSystemMetrics(1)*1.2)# [pixels] Height of simulation window. This will work natively on your machine.
ref = ImageFourierReal([0, 0], 0, np.array([1, 1]) * 1.6, [width, height], 300)
ref.generateRef('Test7.png', 155, 20)

p = ref.points

pc = p[:, 0] + 1j * p[:, 1]

N = 10
c1 = nurfft(p[:, 0], N)
c2 = nurfft(p[:, 1], N)

k1 = 1
k2 = 1
rot = 0 / 180 * np.pi

i = np.abs(np.arange(10, 10, 1))


c1 = nufft(pc, N)#nufft(pc, N)

l = .75
mapping = lambda z: l**2 * (z - 1.5)**l + 1.5#np.sqrt(np.conj(z) * z) * np.exp(-1j * np.arctan(np.imag(z) / np.real(z))**2 / np.pi)

c2 = mapping(c1)
c3 = nufft(mapping(pc), N)


n = 100
#ff = np.vstack((irfft_n(c1, n), irfft_n(c2, n))).T
ff = ifft_n(c1, n)
ff = np.vstack((np.real(ff), np.imag(ff))).T
print(len(ff))

cc = ifft_n(c2, n)
cc = np.vstack((np.real(cc), np.imag(cc))).T
print(len(cc))

dd = ifft_n(c3, n)
dd = np.vstack((np.real(dd), np.imag(dd))).T
print(len(cc))


# Plot result
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(ff[:, 0], ff[:, 1])
ax.plot(cc[:, 0], cc[:, 1])
ax.plot(dd[:, 0], dd[:, 1])
ax.set_aspect('equal')

plt.show()