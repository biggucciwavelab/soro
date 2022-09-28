from elliptic2 import *
import cv2
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, cos, sin, real, imag
from scipy.linalg import block_diag
from numpy import block


# Get sequence from image:
# Open file
fileName = 'images/Test.png'
im = cv2.imread(fileName)
im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

# Generate points
p, hierarchy = cv2.findContours(im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
p = np.reshape(p[1], (p[1].shape[0], 2))
p = np.subtract(p, np.average(p, axis=0))

#t = np.sort(np.random.random(60)) * 2 * np.pi#np.linspace(-np.pi, np.pi, 50)
#p = np.vstack((np.cos(t), np.sin(t))).T


# Order
N = 5

# Generate initial coeficients:
coefs = elliptic_fourier_descriptors(p, order=N)
dc = calculate_dc_coefficients(p)

# Set coefficients as flat
coefs_flat = coefs.flatten()

# Length of points selected
P = np.max(p.shape)

# Generate matrices
tr = np.array([[0, -1], [1, 0]])
tr = block_diag(tr, tr)
di = np.diag(1 / (np.pi * np.arange(1, N+1)))

int_mat = np.kron(di, tr)

del tr, di
tr = np.array([[0, 1], [-1, 0]])
tr = block_diag(tr, tr)
di = np.diag((np.pi * np.arange(1, N+1)))

der_mat = np.kron(di, tr)

new_coefs = int_mat @ coefs_flat
new_coefs = np.reshape(new_coefs, (N, 4))


p_0 = reconstruct_contour(coefs, num_points=300)
p_1 = reconstruct_contour(new_coefs, num_points=300)

# plt.plot(p_0[:, 0])
# plt.plot(p_1[:, 0])
# plt.plot(np.cumsum(p_0[:, 0])/len(p_0[:, 0])*2)
#
# plt.legend(['Original', 'Mat integration', 'Numeric integration'])

# distances = np.concatenate(([0], np.cumsum(np.sqrt(np.sum(np.diff(p, axis=0)**2, axis=1)))))
# distances /= distances[-1]
#
# ki = np.linspace(0, 1, len(p_0[:, 1]))
#
# plt.plot(distances, p[:, 1], '*-')
#
# plt.plot(ki, p_0[:, 1] + dc[1])
#
# plt.legend(['Original geo', 'Descriptor geo'])
# #plt.axis('equal')
# plt.xlabel('t')
# plt.ylabel('Y - value')

plt.plot(p[:, 0], p[:, 1], '*-')
plt.plot(p_0[:, 0]+ dc[0], p_0[:, 1] + dc[1])
plt.legend(['Original geo', 'Descriptor geo'])
plt.axis('equal')
plt.xlabel('X axis')
plt.ylabel('Y axis')

print(coefs)
plt.show()