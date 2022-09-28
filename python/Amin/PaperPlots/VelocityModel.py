import numpy as np
from numpy import loadtxt
import matplotlib.pyplot as plt
import itertools
from scipy.signal import butter, lfilter, freqz, bessel, lfiltic


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, a, b):
    y = lfilter(b, a, np.array(data).T)
    return y.T

b, a = butter_lowpass(.05, 10, order=3)

folder = 'Data/'
filename = 'Vel_Model_Data_exp.csv'
#filename = 'Vel_Model_Data_24_sim.csv'

# Load data
data = loadtxt(folder + filename, skiprows=1, delimiter=',')

N = int((data.shape[1] - 2) / 7)
t = data.shape[0]
# Split data into position, velocity, heading and controller
p_l = data[:, :N*2].reshape((t, N, 2))
v_l = data[:, N*2:N*4].reshape((t, N, 2))
n_l = np.array(list(itertools.chain(*zip(data[:, N*4:N*5].T, data[:, N*5:N*6].T)))).T.reshape((t, N, 2))
c_l = data[:, N*6:N*7]
d_l = data[:, N*7:]

vel_tru_list = []
pos_trut_list = []
m1_l = []
m2_l = []
m3_l = []
for p, v, n, c, d in zip(p_l, v_l, n_l, c_l, d_l):
    d = np.array([d[0], d[1]])

    n = (n[:, 0] + 1j * n[:, 1]) * np.exp(1j * np.pi / 2 * (np.arange(N) % 2))
    n = np.vstack((np.real(n), np.imag(n))).T

    # True velocity
    vel_t = np.average(v, axis=0)

    vel_e = (n.T * c).T

    # Method 1 average velocities only some
    m1 = np.average(vel_e, axis=0)

    # Method 2 average velocities
    m2 = np.average(vel_e[np.arange(N) % 2 == 0, :], axis=0)

    # Method 3 command direction
    factor = (3.25 - np.average(np.dot(n, d) * c) * 3.81) / 500
    m3 = d * factor #* 0.007

    # plt.clf()
    # plt.quiver(p[:, 0], p[:, 1], n[:, 0] * c, n[:, 1] * c)
    # plt.quiver([np.average(p[:, 0])], [np.average(p[:, 1])], [d[0]], [d[1]])
    # plt.pause(.01)

    vel_tru_list.append(vel_t)
    pos_trut_list.append(np.average(p, axis=0))
    m1_l.append(m1)
    m2_l.append(m2)
    m3_l.append(m3)

pos_tru_list = np.cumsum(vel_tru_list, axis=0) * .1
pos_trut_list = np.array(pos_trut_list) - pos_trut_list[0]

m1_l = butter_lowpass_filter(m1_l, a, b)

m1_l = np.cumsum(m1_l, axis=0) * .1
m2_l = np.cumsum(m2_l, axis=0) * .1
m3_l = np.cumsum(m3_l, axis=0) * .1

v_av = np.average(np.linalg.norm(pos_tru_list, axis=0) / np.linalg.norm(m1_l, axis=0))
print('StatValue = ' + str(v_av))
m1_l *= 0.045 / 2#v_av
v_av = np.average(np.linalg.norm(pos_tru_list, axis=0) / np.linalg.norm(m2_l, axis=0))
m2_l *= v_av

plt.plot(pos_trut_list[:, 0], pos_trut_list[:, 1])
#plt.plot(pos_tru_list[:, 0], pos_tru_list[:, 1])# / np.cumsum(vel_est_list))
plt.plot(m1_l[:, 0], m1_l[:, 1])
#plt.plot(m2_l[:, 0], m2_l[:, 1])
plt.plot(m3_l[:, 0], m3_l[:, 1])
plt.show()