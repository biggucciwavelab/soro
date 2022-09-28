import numpy as np

# Convert experimental data to same format as simulated data
def convertData(raw, dt=.05):
    col = np.zeros((raw.shape[0], 1))
    N = int(raw.shape[1] / 3) - 1
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

def getPoseLocalization(raw):
    p_est = raw[:, 2:4]
    r_est = raw[:, 4]
    p_tru = raw[:, 5:7]
    r_tru = raw[:, 7]

    return p_est, p_tru, r_est, r_tru

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
    ang = np.arctan2(n[:, 1], n[:, 0])
    ang += (ang > np.pi) * (- 2 * np.pi) + (ang < - np.pi) * (2 * np.pi)

    # Calculate angle difference
    diff = np.roll(ang, -1) - ang
    diff += (diff > np.pi) * (- 2 * np.pi) + (diff < - np.pi) * (2 * np.pi)

    # Calculate the direction that connects two bots
    theta = np.exp(1j * np.roll(ang, -1)) + np.exp(1j * ang)
    theta = np.angle(theta) + np.pi / 2
    theta += (theta > np.pi) * (- 2 * np.pi) + (theta < - np.pi) * (2 * np.pi)

    # estimated distances
    if method == 'linear':
        L = (155.93 - 0.8547 * 180 / np.pi * np.abs(diff)) / 1000.
    elif method == 'rigid_joint':
        L = 2 * lmax * np.cos(.5 * (diff - 2 * np.pi / N))
    else:
        L = np.ones(diff.shape) * .12


    if correction:
        #Adjust result:
        C = np.cos(theta).reshape((-1, N))
        S = np.sin(theta).reshape((-1, N))

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
    B = np.vstack((np.cos(theta), np.sin(theta)))

    # Calculate relative positions
    B = (L * B).T

    rel_positions = np.vstack((np.zeros((1, 2)), np.cumsum(B, axis=0)[:-1, :]))
    true_positions = p - p[0, :]

    # Find best fit rotation and displacement
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

        tmp = np.vstack((rel_positions[:, 0] * sol[0] + rel_positions[:, 1] * sol[1] + sol[2],
                         -rel_positions[:, 0] * sol[1] + rel_positions[:, 1] * sol[0] + sol[3]))

        rel_positions = tmp.T

        error = np.average(np.sqrt(np.sum((true_positions - rel_positions)**2, axis=1))) * 1000

    return rel_positions, true_positions, error
