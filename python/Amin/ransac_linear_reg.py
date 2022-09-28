import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from sklearn import linear_model, datasets
import seaborn as sns

def standardize_angle(angle):
    angle = angle*180/np.pi
    # reduce the angle
    angle = angle % 360
    # force it to be the positive remainder, so that 0 <= angle < 360
    angle = (angle + 360) % 360
    # force into the minimum absolute value residue class, so that -180 < angle <= 180
    if (angle > 180).any():
        angle[angle>180] -= 360
    return angle


def load_dataset():
    X = []
    Y = []
    path_to_parent = "PaperPlots/LearningData/2021-05-16 17h09/"
    #for folder in os.listdir(path_to_parent):
    csv_files = glob.glob(path_to_parent + '/*.csv')
    for csv_file in csv_files:
        df = pd.read_csv(csv_file)
        df.drop(df.index[range(0, 100)], inplace=True)
        # print(df[df.columns[2:37:3]])
        # print(df[df.columns[2:37:3]])
        if len(X) < 1:
            X = df[df.columns[2:36:3]]
            Y = np.hstack((df[df.columns[0:36:3]], df[df.columns[1:36:3]]))
        else:
            X = np.vstack((X, df[df.columns[2:36:3]]))
            Y = np.vstack((Y, np.hstack((df[df.columns[0:36:3]], df[df.columns[1:36:3]]))))

    return X, Y


# load dataset
X, y = load_dataset()
A = np.array([])
A_r = np.array([])
Theta = np.array([])
Theta_r = np.array([])

for n in range(12):
    offset0 = 90 * (n % 2 != 0)
    offset1 = 90 * ((n + 1) % 2 != 0)

    if n < 11:
        theta = ((X[:, n + 0] + offset0) - (X[:, n + 1] + offset1)) * np.pi / 180
        a = ((y[:, 0 + n] - y[:, 1 + n]) ** 2 + (y[:, 12 + n] - y[:, 13 + n]) ** 2) ** 0.5
    else:
        theta = ((X[:, 11] + offset0) - (X[:, 0] + offset1)) * np.pi / 180
        a = ((y[:, 11] - y[:, 0]) ** 2 + (y[:, 23] - y[:, 12]) ** 2) ** 0.5
    if not A_r.size > 0:
        A_r = a.reshape([-1, 1])
    else:
        A_r = np.vstack((A_r, a.reshape([-1, 1])))

    if not Theta.size > 0:
        Theta = theta.reshape([-1, 1])
    else:
        Theta = np.vstack((Theta, theta.reshape([-1, 1])))


x = abs(standardize_angle(Theta[::100]))
y = A_r[::100]
# Robustly fit linear model with RANSAC algorithm
ransac = linear_model.RANSACRegressor(random_state=0, residual_threshold=1)
ransac.fit(x, y)
inlier_mask = ransac.inlier_mask_
outlier_mask = np.logical_not(inlier_mask)

# Predict data of estimated models
tmp = ransac.predict([0, 90])
slope = tmp[1]/tmp[0]
#offset =

sns.set(style="ticks", palette="deep", color_codes=True, font_scale=0.6)
f, axes = plt.subplots(1, 1, figsize=(3.5, 3.5 * 3.5 / 4.5))

# plt.plot(standardize_angle(Theta)[standardize_angle(Theta)<0], A_r[standardize_angle(Theta)<0], ".")
plt.plot(x[inlier_mask], y[inlier_mask], ".", c="yellowgreen", alpha=0.5, label="Outliers")
plt.plot(x[outlier_mask], y[outlier_mask], ".", c="gold", alpha=0.5, label="Inliers")
# plt.plot(x, 16*np.sin(0.5*(180-x)*np.pi/180), "ro")
# plt.plot(x, -8*x/90+16, "go")
plt.plot(line_X, line_y_ransac, c='navy', linewidth=2, label="RANSAC regressor")

# plt.axis
plt.ylim([7.5, 18])
plt.xlabel("Heading angle difference [deg]")
plt.ylabel("Subunit distance [cm]")
plt.legend()
plt.tight_layout()
# plt.show()
plt.savefig("2021-05-16 21h49/ranscac.pdf")

#%%
X, y = load_dataset()
X = X[::10, :]
y = y[::10, :]
A = []
A_r = []
Theta = []
Theta_r = []

for n in range(12):
    if n < 11:
        a = ((y[:, 1 + n] - y[:, 0 + n]) ** 2 + (y[:, 13 + n] - y[:, 12 + n]) ** 2) ** 0.5
    else:
        a = ((y[:, 0] - y[:, 11]) ** 2 + (y[:, 12] - y[:, 23]) ** 2) ** 0.5
    if A_r == []:
        A_r = a
    else:
        A_r = np.vstack((A_r, a))
A_r = A_r.T

Lmax = 20
Lmean = 12.5
for n in range(12):
    offset0 = 90 * (n % 2 != 0)
    offset1 = 90 * ((n + 1) % 2 != 0)
    if n < 11:
        a = Lmax * np.abs(np.sin((X[:, 1 + n] - X[:, 0 + n] + 180) * np.pi / 360))
        theta = ((X[:, n + 0] + offset0) - (X[:, n + 1] + offset1)) * np.pi / 180
    else:
        theta = ((X[:, 11] + offset0) - (X[:, 0] + offset1)) * np.pi / 180
    a = ransac.predict(abs(standardize_angle(theta[:, np.newaxis])))
    a = np.array(a).reshape([1, -1])
    if A == []:
        A = a
    else:
        A = np.vstack((A, a))
A = A.T

for n in range(12):
    offset0 = 0 * (n % 2 == 0) + 90 * (n % 2 > 0)
    offset1 = 0 * ((n + 1) % 2 == 0) + 90 * ((n + 1) % 2 > 0)
    if n < 11:
        # theta = 0.5 * (X[:, n + 0] + X[:, n + 1] + 90) * np.pi / 180
        theta = np.arctan2(
            np.sin((X[:, n + 0] + 90 + offset0) * np.pi / 180) + np.sin((X[:, n + 1] + 90 + offset1) * np.pi / 180),
            np.cos((X[:, n + 0] + 90 + offset0) * np.pi / 180) + np.cos((X[:, n + 1] + 90 + offset1) * np.pi / 180))
    else:
        # theta = 0.5 * (X[:, 11] + X[:, 0] + 90) * np.pi / 180
        theta = np.arctan2(np.sin((X[:, 11] + 180) * np.pi / 180) + np.sin((X[:, 0] + 90) * np.pi / 180),
                           np.cos((X[:, 11] + 180) * np.pi / 180) + np.cos((X[:, 0] + 90) * np.pi / 180))

    if Theta == []:
        Theta = theta
    else:
        Theta = np.vstack((Theta, theta))

for n in range(12):
    if n < 11:
        theta = np.arctan2((y[:, 13 + n] - y[:, 12 + n]), (y[:, 1 + n] - y[:, 0 + n]))
    else:
        theta = np.arctan2((y[:, 12] - y[:, 23]), (y[:, 0] - y[:, 11]))
    if Theta_r == []:
        Theta_r = theta
    else:
        Theta_r = np.vstack((Theta_r, theta))

B = np.vstack((-np.cos(Theta), -np.sin(Theta))).T

B_r = np.hstack((np.cos(Theta_r.T), np.sin(Theta_r.T)))

res = []
for i in np.arange(0, X.shape[0], 1):
    positions_r = np.zeros([12, 2])
    for j in range(0, 11):
        positions_r[j+1, :] = positions_r[j, :] + A_r[i, j] * B_r[i, [j, j + 12]]
    # plt.plot(np.append(positions_r[:, 0], positions_r[0, 0]), np.append(positions_r[:, 1], positions_r[0, 1]))

    positions = np.zeros([12, 2])
    l = 12
    for j in range(0, 11):
        positions[j+1, :] = positions[j, :] + A[i, j] * B[i, [j, j + 12]]
    # if np.abs(np.linalg.norm(positions[0, :] - positions[-1, :]) - l) > 3:
    #     positions[-1, :] = 0.5 * (positions[-2, :] + positions[0, :])
    res = np.append(res, np.linalg.norm(positions-positions_r)/12)
    # plt.plot(np.append(positions[:, 0], positions[0, 0]), np.append(positions[:, 1], positions[0, 1]))
    # plt.axis('equal')
    # plt.show()

print("mean error:", np.mean(res), "std error", np.std(res))
