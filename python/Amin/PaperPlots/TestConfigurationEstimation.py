import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import ttest_rel
from scipy.signal import butter, lfilter, freqz, bessel
from scipy.optimize import minimize

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def bessel_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = bessel(order, normal_cutoff, btype='lowpass', analog=False)
    return b, a

def bessel_lowpass_filter(data, cutoff, fs, order=5):
    b, a = bessel_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# define functions to use:
# Convert experimental data to same format as simulated data
def convertData(raw, dt=.05):
    col = np.zeros((raw.shape[0], 1))
    N = int(data.shape[1] / 3) - 1
    tmp = np.arange(raw.shape[0]).reshape((raw.shape[0], 1)) * dt
    for i in range(N):
        k = i#N - i - 1
        dir = np.vstack((-np.cos((raw[:, 3 * k + 2] - raw[:, -1]) / 180 * np.pi + np.pi / 2 * (k%2)),
                        -np.sin((raw[:, 3 * k + 2] - raw[:, -1]) / 180 * np.pi + np.pi / 2 * (k%2)))).T

        tmp = np.hstack((tmp, (raw[:, (3 * k):(3 * k + 2)] - raw[:, -3:-1]) / 100.,
                         col, col,
                         dir, col))
    tmp[1, 0] = .05
    return tmp

# Estimate positions based on imu data
def getEstimatedPositions(data_slice, correction=False, method='linear', adjust_position=False):
    lmax = 150 / 2000

    data_slice = data_slice[1:].reshape((-1, 7))
    N = data_slice.shape[0]
    p = data_slice[:, :2]
    v = data_slice[:, 2:4]
    n = data_slice[:, 4:6]
    d = data_slice[:, 6]

    # Calculate normal angles
    X = (n[:, 0] + 1j*n[:, 1]) * np.exp(-1j * np.pi / 2)

    ### Precalculate variables that will later be nidded
    # Calculate difference in angle, constrain between - pi to pi
    diff = np.angle(np.roll(X, -1) / X)

    # Calculate direction to next bot:
    theta = - (np.roll(X, -1) + X)
    theta /= np.abs(theta)

    # estimated distances
    if method == 'linear':
        L = (155.93 - 0.8547 * 180 / np.pi * np.abs(diff)) / 1000.
    elif method == 'linear2':
        L = (158.842 - 0.937 * 180 / np.pi * np.abs(diff)) / 1000.
    elif method == 'rigid_joint':
        L = 2 * lmax * np.cos(.5 * (diff - 2 * np.pi / N))
    else:
        L = np.ones(diff.shape) * .12


    if correction:
        #Adjust result:
        C = np.real(theta).reshape((-1, N))
        S = np.imag(theta).reshape((-1, N))

        # Calculate the inverse of the covariances
        COVi = np.eye(N)

        # Values to be used in operation
        CCT = (C @ COVi @ C.T)[0, 0]
        SST = (S @ COVi @ S.T)[0, 0]
        SCT = (S @ COVi @ C.T)[0, 0]
        CST = (C @ COVi @ S.T)[0, 0]

        P = (SCT / CCT * C - S) / (SCT * CST / CCT - SST)
        Q = (C - CST * P) / CCT

        # Apply adjustment
        x = - COVi @ (C.T @ Q @ L + S.T @ P @ L)
        L += x

    # Matrix for directions:
    B = np.vstack((np.real(theta), np.imag(theta)))

    # Calculate relative positions
    B = (L * B).T

    rel_positions = np.vstack((np.zeros((1, 2)), np.cumsum(B, axis=0)[:-1, :]))
    true_positions = p - p[0, :]

    # Find best fit rotation and displacement
    error = 0.
    if adjust_position:
        M1 = np.hstack((rel_positions,
                        np.ones((rel_positions.shape[0], 1)),
                        np.zeros((rel_positions.shape[0], 1))))
        M2 = np.vstack((rel_positions[:, 1],
                        -rel_positions[:, 0],
                        np.zeros(rel_positions.shape[0]),
                        np.ones(rel_positions.shape[0]))).T
        M = np.vstack((M1, M2))
        b = np.hstack((true_positions[:, 0], true_positions[:, 1]))

        sol = np.linalg.inv(M.T @ M) @ M.T @ b

        def fun(x):
            J = M @ np.array([np.cos(x[0]), np.sin(x[0]), x[1], x[2]]) - b
            return J.T @ J

        out = minimize(fun, [0, 0, 0])

        sol = out.x

        tmp = np.vstack((rel_positions[:, 0] * np.cos(sol[0]) + rel_positions[:, 1] * np.sin(sol[0]) + sol[1],
                         -rel_positions[:, 0] * np.sin(sol[0]) + rel_positions[:, 1] * np.cos(sol[0]) + sol[2]))

        rel_positions = tmp.T
    else:
        rel_positions -= np.average(rel_positions, axis=0)
        true_positions -= np.average(true_positions, axis=0)
    error = np.max(np.sqrt(np.sum((true_positions - rel_positions)**2, axis=1))) * 1000

    return rel_positions, true_positions, error, n

def getEstimatedPositions2(data_slice, correction=False, method='linear', adjust_position=False):
    lmax = 150 / 2000

    data_slice = data_slice[1:].reshape((-1, 7))
    N = data_slice.shape[0]
    p = data_slice[:, :2]
    v = data_slice[:, 2:4]
    n = data_slice[:, 4:6]
    d = data_slice[:, 6]

    # Calculate normal angles
    X = (n[:, 0] + 1j*n[:, 1]) * np.exp(-1j * np.pi / 2)

    ### Precalculate variables that will later be nidded
    # Calculate difference in angle, constrain between - pi to pi
    diff = np.angle(np.roll(X, -1) / X)

    # Calculate direction to next bot:
    theta = - (np.roll(X, -1) + X)
    theta /= np.abs(theta)

    # estimated distances
    if method == 'linear':
        L = (155.93 - 0.8547 * 180 / np.pi * np.abs(diff)) / 1000.
    elif method == 'linear2':
        #L = (158.842 - 0.937 * 180 / np.pi * np.abs(diff)) / 1000.
        L = (161.587 - 0.813 * 180 / np.pi * np.abs(diff)) / 1000.
    elif method == 'rigid_joint':
        L = 2 * lmax * np.cos(.5 * (diff - 2 * np.pi / N))
    else:
        L = np.ones(diff.shape) * .12

    x1 = np.zeros(len(L))
    x2 = np.zeros(len(L))
    if correction:
        #Adjust result:
        C = np.real(theta).reshape((N, -1))
        S = np.imag(theta).reshape((N, -1))


        # Calculate the inverse of the covariances
        COVi1 = np.eye(N) * (9.2794 / 1000)**2
        COVi1 = np.eye(N) * (10.86363 / 1000)**2
        #COVi2 = np.eye(N) * (11.9145 * np.pi / 180)**2
        #COVi2 = np.diag((diff * 180/np.pi * .1268037 + 6.398563) * np.pi / 180)**2
        COVi2 = np.diag((diff * 180 / np.pi * 0.10540195 + 4.95587) * np.pi / 180)**2

        L_m = np.diag(L)

        COVi2 = L_m**2 @ COVi2 @ L_m**2

        # Values to be used in operation
        A_m = (C.T @ COVi1 @ C + S.T @ COVi2 @ S)[0, 0]
        B_m = (C.T @ COVi1 @ S - S.T @ COVi2 @ C)[0, 0]
        C_m = (S.T @ COVi1 @ C - C.T @ COVi2 @ S)[0, 0]
        D_m = (S.T @ COVi1 @ S + C.T @ COVi2 @ C)[0, 0]

        b_l = np.array([-C.T @ L, -S.T @ L])

        lam_sol = (np.linalg.inv([[A_m, B_m], [C_m, D_m]]) @ b_l).flatten()

        x1 = COVi1 @ (lam_sol[0] * C + lam_sol[1] * S)
        x2 = COVi2 @ (lam_sol[0] * L_m @ S - lam_sol[1] * L_m @ C)

    # Matrix for directions:
    B1 = np.vstack((np.real(theta), np.imag(theta)))
    B2 = np.vstack((np.imag(theta), -np.real(theta)))

    # Calculate relative positions
    B = ((L + x1.reshape(-1)) * B1).T + (x2.reshape(-1) * B2).T

    rel_positions = np.vstack((np.zeros((1, 2)), np.cumsum(B, axis=0)[:-1, :]))
    true_positions = p - p[0, :]

    # Find best fit rotation and displacement
    error = 0.
    if adjust_position:
        M1 = np.hstack((rel_positions,
                        np.ones((rel_positions.shape[0], 1)),
                        np.zeros((rel_positions.shape[0], 1))))
        M2 = np.vstack((rel_positions[:, 1],
                        -rel_positions[:, 0],
                        np.zeros(rel_positions.shape[0]),
                        np.ones(rel_positions.shape[0]))).T
        M = np.vstack((M1, M2))
        b = np.hstack((true_positions[:, 0], true_positions[:, 1]))

        sol = np.linalg.inv(M.T @ M) @ M.T @ b

        def fun(x):
            J = M @ np.array([np.cos(x[0]), np.sin(x[0]), x[1], x[2]]) - b
            return J.T @ J

        out = minimize(fun, [0, 0, 0])

        sol = out.x

        tmp = np.vstack((rel_positions[:, 0] * np.cos(sol[0]) + rel_positions[:, 1] * np.sin(sol[0]) + sol[1],
                         -rel_positions[:, 0] * np.sin(sol[0]) + rel_positions[:, 1] * np.cos(sol[0]) + sol[2]))

        rel_positions = tmp.T
    else:
        rel_positions -= np.average(rel_positions, axis=0)
        true_positions -= np.average(true_positions, axis=0)
    error = np.max(np.sqrt(np.sum((true_positions - rel_positions)**2, axis=1))) * 1000

    return rel_positions, true_positions, error, n


def plot_slice_headings(slice):
    data_slice = slice[1:].reshape((-1, 7))
    N = data_slice.shape[0]
    p = data_slice[:, :2]
    v = data_slice[:, 2:4]
    n = data_slice[:, 4:6]
    d = data_slice[:, 6]

    x = np.zeros(n.shape[0])
    for i, ni in enumerate(n):
        plt.quiver(0, 0, ni[0], ni[1], color='C' + str(i%10), scale=5)
    plt.show()

# Load data file
# data = np.loadtxt('Experiment_12 Data and Plots 2022-08-17 134631/observations.csv', delimiter=',')

data = np.loadtxt('LearningData/2021-05-16 17h09/data.csv', skiprows=100, delimiter=',')
data = convertData(data, dt=.1)


################
#data = np.loadtxt('Sim_results/OLD/Example_exp_data_imu.csv', skiprows=1, delimiter=',')
#data_ang = np.exp(-1j*data[:, 3::4])#np.cos(data[:, 3::4]) + 1j * np.sin(data[:, 3::4]) #* np.exp(1j * np.pi / 2)
#data_ang_tru = np.exp(-1j * data[:, 4::4])

# Apply low pass filter
#data_ang = butter_lowpass_filter(data_ang.T, .14, 10, 5).T
#data_ang /= np.abs(data_ang)

# Apply offset
# k = 100
# phase = np.average(data_ang_tru[:k, :] / data_ang[:k, :], axis=0)
# print('Correction phases: ' + str(phase))
#
# err = data_ang_tru / data_ang / phase
#
# plt.plot(np.angle(err[:, 0]))
#
# tmp = butter_lowpass_filter(err.T, .15, 10, 5).T
# err /= tmp
# err /= np.abs(err)
#
# plt.plot(np.angle(err[:, 0]))
# plt.show()
#
# mu = np.average(err[100:160, :], axis=0)
# std = np.real(np.conj(mu) * mu)
# std = np.sqrt(- 2 * np.log(std))
# mu = np.angle(mu)
#
# print('Average MU: ' + str(np.average(mu) * 180/np.pi))
# print('Average std: ' + str(np.average(std) * 180/np.pi))
#
# data_ang = np.angle(data_ang * phase)
# data_ang_tru = np.angle(data_ang_tru)
#
# # Convert data type:
# tmp = np.linspace(0, data.shape[0] * .1, data.shape[0]).reshape((data.shape[0], 1))
# zer1 = np.ones((data.shape[0], 1))
# zer2 = np.ones((data.shape[0], 2))
# for i in range(12):
#     xy = data[:, i * 4 + 1: i * 4 + 3] / 100.
#     xy[:, 1] = -xy[:, 1]
#
#     angle = data_ang[:, i].reshape((data_ang.shape[0], 1)) - np.pi / 2 * (i%2)
#     data_ang_tru[:, i] = data_ang_tru[:, i] - np.pi / 2 * (i % 2)
#
#     #tmp = np.hstack((tmp, xy, zer2, np.cos(angle), np.sin(angle), zer1))
#     tmp = np.hstack((tmp, xy, zer2, np.cos(data_ang_tru[:, i].reshape((data_ang.shape[0], 1))),
#                      np.sin(data_ang_tru[:, i].reshape((data_ang.shape[0], 1))), zer1))
# data = tmp
#
#
#
# plt.plot(np.arctan2(data[:, 6], data[:, 5]))
# plt.plot(data_ang[:, 0])
# plt.plot(data_ang_tru[:, 0])
# plt.show()




#plot_slice_headings(data[0, :])

# Get time interval
dt = data[1, 0] - data[0, 0]


# Generate data to plot:
rel_stat, true, er1, n1 = getEstimatedPositions(data[0, :], correction=False, method='linear2', adjust_position=True)
rel_lin, true, er2, n2 = getEstimatedPositions(data[0, :], correction=True, method='linear2', adjust_position=True)
rel_rig, true, er3, n3 = getEstimatedPositions2(data[0, :], correction=True, method='linear2', adjust_position=True)

fig, axs = plt.subplots(1, 2)
fig.set_size_inches(18.5, 10.5)
l1, = axs[0].plot(true[:, 0], true[:, 1], '*--')
#q1 = axs[0].quiver(true[:, 0], true[:, 1], np.cos(data_ang_tru[0, :]), np.sin(data_ang_tru[0, :]), color='C0')
q2 = axs[0].quiver(true[:, 0], true[:, 1], n1[:, 0], n1[:, 1], color='C1')
l2, = axs[0].plot(rel_stat[:, 0], rel_stat[:, 1], 'x:')
l3, = axs[0].plot(rel_lin[:, 0], rel_lin[:, 1], 'x:')
l4, = axs[0].plot(rel_rig[:, 0], rel_rig[:, 1], 'x:')
axs[0].set_xlabel('X axis [m]')
axs[0].set_ylabel('Y axis [m]')
axs[0].legend(['True', 'Constant', 'Regressor', 'Rigid link'])

er1_l = [er1]
er2_l = [er2]
er3_l = [er3]

axs[1].boxplot([er1_l, er2_l])
#for slice, ang in zip(data, data_ang_tru):
for slice in data:
    rel_stat, true, er1, n1 = getEstimatedPositions(slice, correction=False, method='linear2', adjust_position=False)
    rel_lin, true, er2, n2 = getEstimatedPositions(slice, correction=True, method='linear2', adjust_position=False)
    rel_rig, true, er3, n3 = getEstimatedPositions2(slice, correction=True, method='linear2', adjust_position=False)

    er1_l.append(er1)
    er2_l.append(er2)
    er3_l.append(er3)

    l1.set_xdata(true[:, 0])
    l1.set_ydata(true[:, 1])

    #U = true[:, 0] + np.cos(ang)
    #V = true[:, 1] + np.sin(ang)
    #q1.set_UVC(U, V)
    #q1.set_offsets(true)

    U = true[:, 0] - n1[:, 0]
    V = true[:, 1] - n1[:, 1]
    q2.set_UVC(U, V)
    q2.set_offsets(true)

    l2.set_xdata(rel_stat[:, 0])
    l2.set_ydata(rel_stat[:, 1])

    l3.set_xdata(rel_lin[:, 0])
    l3.set_ydata(rel_lin[:, 1])

    l4.set_xdata(rel_rig[:, 0])
    l4.set_ydata(rel_rig[:, 1])


    axs[1].cla()
    axs[1].boxplot([er1_l, er2_l, er3_l], labels=['Constant', 'Regressor', 'Rigid link'])
    axs[1].grid()
    t, p = ttest_rel(er2_l, er3_l)
    axs[1].set_title('T-test for Regressor and Rigid link: P_value = ' + str(np.round(p, 4)))


    limx = np.array([np.min((true[:, 0], rel_lin[:, 0])) - .2, np.max((true[:, 0], rel_lin[:, 0])) + .2])
    limy = np.array([np.min((true[:, 1], rel_lin[:, 1])) - .2, np.max((true[:, 1], rel_lin[:, 1])) + .2])

    axs[0].set_xlim(tuple(limx))
    axs[0].set_ylim(tuple(limy))
    axs[0].set_aspect('equal', 'box')

    plt.draw()
    plt.pause(.001)
    #plt.pause(dt)

plt.figure()
plt.plot(er1_l)
plt.plot(er2_l)
plt.plot(er3_l)
plt.legend(['Constant', 'Regressor', 'Rigid link'])
plt.show()