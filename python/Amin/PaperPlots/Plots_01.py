from result_utils import *
import numpy as np
import matplotlib.pyplot as plt

folder = 'ResultsPaper'
fn1 = 'Exp_180.csv'
fn2 = 'Exp_360.csv'
fn3 = 'Exp_720.csv'

# First lets plot the localization error plots for a given experiment:
data1 = np.loadtxt(folder + fn1, skiprows=1, delimiter=',')
# Get position of base link
p_est1, p_tru1, r_est, r_tru = getPoseLocalization(data1)
# Convert data to proper units
p_est1 = (p_est1 - p_est1[0, :]) #/ 2 / 1.1
p_tru1 = (p_tru1 - p_tru1[0, :]) #/ 2 / 1.1
#p_tru1[:, 1] = - p_tru1[:, 1]

# First lets plot the localization error plots for a given experiment:
data2 = np.loadtxt(folder + fn2, skiprows=1, delimiter=',')
# Get position of base link
p_est2, p_tru2, r_est, r_tru = getPoseLocalization(data2)
# Convert data to proper units
p_est2 = (p_est2 - p_est2[0, :]) #/ 2 / 1.1
p_tru2 = (p_tru2 - p_tru2[0, :]) #/ 2 / 1.1
#p_tru2[:, 1] = - p_tru2[:, 1]

# First lets plot the localization error plots for a given experiment:
data3 = np.loadtxt(folder + fn3, skiprows=1, delimiter=',')
# Get position of base link
p_est3, p_tru3, r_est, r_tru = getPoseLocalization(data3)
# Convert data to proper units
p_est3 = (p_est3 - p_est3[0, :]) #/ 2 / 1.1
p_tru3 = (p_tru3 - p_tru3[0, :]) #/ 2 / 1.1
#p_tru3[:, 1] = - p_tru3[:, 1]

# plot path
k = 1800
n = 5
plt.plot(p_est1[:k, 0][::n], p_est1[:k, 1][::n])
plt.plot(p_est2[:k, 0][::n], p_est2[:k, 1][::n])
plt.plot(p_est3[:k, 0][::n], p_est3[:k, 1][::n])
plt.plot(p_tru1[:k, 0][::n], p_tru1[:k, 1][::n])
plt.legend(['180', '360', '720', 'true'])

# Plot error
plt.figure()
err = np.sqrt(np.sum((p_est1 - p_tru1)**2, axis=1))[::n]
plt.plot(err)

err = np.sqrt(np.sum((p_est2 - p_tru2)**2, axis=1))[::n]
plt.plot(err)
err = np.sqrt(np.sum((p_est3 - p_tru3)**2, axis=1))[::n]
plt.plot(err)

plt.legend(['180', '360', '720', 'true'])

plt.show()