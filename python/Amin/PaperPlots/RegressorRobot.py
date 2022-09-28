import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import ttest_rel
from glob import glob
from sklearn import linear_model
from nmmn.plots import parulacmap, wolframcmap
from matplotlib import cm

# define functions to use:
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

folder = 'LearningData/'

dist_l = np.empty(1)
ang_l = np.empty(1)
ang_diff = np.empty(1)

folders = glob("LearningData/*/", recursive=True)
folders = ['LearningData/2021-11-24_15h15/']
for f in folders:
    data = np.loadtxt(f + 'data.csv', skiprows=100, delimiter=',')
    data = convertData(data, dt=.1)

    # get data
    px = data[:, 1::7]
    py = data[:, 2::7]
    nx = data[:, 5::7]
    ny = data[:, 6::7]

    # Calculate distances
    distances = np.sqrt((np.roll(px, -1) - px)**2 + (np.roll(py, -1) - py)**2)
    angles = nx + 1j * ny
    angles = np.angle(np.roll(angles, -1) / angles)

    # Calculate angles
    angles = angles.flatten()
    distances = distances.flatten()

    dist_l = np.concatenate((dist_l, distances))
    ang_l = np.concatenate((ang_l, angles))

    # Calculate direction angles:
    # Ideal
    id = - (np.roll(ny, -1) + ny) + 1j * (np.roll(nx, -1) + nx)
    id /= np.abs(id)

    # True
    it = (np.roll(px, -1) - px) + 1j * (np.roll(py, -1) - py)
    it /= np.abs(it)

    tmp = np.angle(id / it).flatten()
    ang_diff = np.concatenate((ang_diff, tmp))


x = np.abs(ang_l) * 180 / np.pi
y = dist_l*1000


# Ransac regressor:
ransac = linear_model.RANSACRegressor(random_state=0, max_trials=1000)
ransac.fit(x.reshape(-1, 1), y.reshape(-1, 1))

inlier_mask = ransac.inlier_mask_
outlier_mask = np.logical_not(inlier_mask)

tmp = ransac.predict(np.array([0, 90]).reshape(-1, 1))
A = ((tmp[1] - tmp[0]) / 90.)[0]
b = tmp[0][0]


################################################################################################
################################################################################################
binsX = int(90/3)
binsY = 40

hist, xedges, yedges = np.histogram2d(x, y, bins=(binsX, binsY), range=[[0, 90], [80, 180]])
hist = hist.T
with np.errstate(divide='ignore', invalid='ignore'):  # suppress division by zero warnings
    hist *= 1 / hist.sum(axis=0, keepdims=False)

axis_font = {'fontname': 'calibri', 'math_fontfamily': 'stix', 'size': '8'}
plt.figure(1, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)
plt.rcParams.update({'font.size': 8,
                     'font.family': 'calibri'})

# 'dejavusans', 'dejavuserif', 'cm', 'stix', 'stixsans', 'custom'

tmp = plt.pcolormesh(xedges, yedges, hist * 100, cmap='magma_r', vmin=0, vmax=20)
cbar = plt.colorbar(tmp)
cbar.set_label('Probability [%]', **axis_font)
plt.ylabel(r'$d_i$ [mm]', **axis_font)
plt.xlabel(r'$\Delta \theta_i = |\theta_{i+1} - \theta_i|$ [°]', **axis_font)
plt.title(r'$P(d_i | \Delta \theta_i)$', **axis_font)

x_line = np.linspace(0, 90, 10)
y_line = A * x_line + b
plt.plot(x_line, y_line, 'b', linewidth=1.5)

means = np.sum(hist.T * yedges[:-1], axis=1).T
stds = np.sum(hist * (yedges[:-1].reshape([-1, 1]) * np.ones(binsX) - means) ** 2, axis=0)
stds = np.sqrt(stds)
print('Average STDS Distance: ' + str(np.average(stds)))

plt.plot(xedges[:-1] + .5, means, '--k')
plt.plot(xedges[:-1] + .5, means + stds, ':k')
plt.plot(xedges[:-1] + .5, means - stds, ':k')

plt.legend(['Linear model', 'Mean', '$\pm\sigma$'])
plt.xlim([0, 90])
plt.ylim([80, 180])
plt.tight_layout()

print('Model: y=' + str(round(A, 3)) + '*x + ' + str(round(b, 3)))
print('Total points: ' + str(len(x)))
plt.savefig('line_plot.pdf')
#plt.plot(x, y, '*')


################################################################################################
################################################################################################
binsX = int(90/3)
binsY = 40
y2 = ang_diff * 180 / np.pi
hist, xedges, yedges = np.histogram2d(x, y2, bins=(binsX, binsY), range=[[0, 90], [-45, 45]])
hist = hist.T
with np.errstate(divide='ignore', invalid='ignore'):  # suppress division by zero warnings
    hist *= 1 / hist.sum(axis=0, keepdims=False)

plt.figure(2, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)
plt.rcParams.update({'font.size': 8,
                     'font.family': 'calibri'})
tmp = plt.pcolormesh(xedges, yedges, hist * 100, cmap='magma_r', vmin=0, vmax=20)
cbar = plt.colorbar(tmp)
cbar.set_label('Probability [%]', **axis_font)
plt.ylabel(r'$\Delta\gamma_i$ [°]', **axis_font)
plt.xlabel(r'$\Delta \theta_i = |\theta_{i+1} - \theta_i|$ [°]', **axis_font)
plt.title(r'$P(\Delta\gamma_i | \Delta \theta_i)$', **axis_font)



means = np.sum(hist.T * yedges[:-1], axis=1).T# * (yedges[1] - yedges[0])
stds = np.sum(hist * (yedges[:-1].reshape([-1, 1]) * np.ones(binsX) - means) ** 2, axis=0)# * (yedges[1] - yedges[0])
stds = np.sqrt(stds)

print('Average STDS Angle: ' + str(np.average(stds)))

plt.plot(xedges, np.zeros(len(xedges)), 'b', linewidth=1.5)
plt.plot(xedges[:-1] + .5, means, '--k')
plt.plot(xedges[:-1] + .5, means + stds, ':k')
plt.plot(xedges[:-1] + .5, means - stds, ':k')

plt.legend(['Linear model', 'Mean', '$\pm\sigma$'])

plt.xlim([0, 90])
plt.ylim([-45, 45])
plt.tight_layout()


tmpx = xedges[:-1].reshape((-1, 1))
tmpy = stds.reshape((-1, 1))

mat = np.hstack((tmpx, np.ones((len(tmpx), 1))))
sol = (np.linalg.inv(mat.T @ mat) @ mat.T @ tmpy).flatten()


plt.figure(5)
plt.plot(tmpx, tmpy, '*')
plt.plot(tmpx, tmpx * sol[0] + sol[1])

print('std model: sigma = ' + str(sol[0]) + '*theta + ' + str(sol[1]))



################################################################################################
################################################################################################
binsX = int(90/3)
binsY = 40
x2 = dist_l * 1000
y2 = ang_diff * 180 / np.pi
hist, xedges, yedges = np.histogram2d(x2, y2, bins=(binsX, binsY), range=[[80, 180], [-45, 45]])
hist = hist.T
with np.errstate(divide='ignore', invalid='ignore'):  # suppress division by zero warnings
    hist *= 1 / hist.sum(keepdims=False)

plt.figure(3, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)
plt.rcParams.update({'font.size': 8,
                     'font.family': 'calibri'})
tmp = plt.pcolormesh(xedges, yedges, hist * 100, cmap='magma_r', vmin=0, vmax=1)
cbar = plt.colorbar(tmp)
cbar.set_label('Probability [%]', **axis_font)
plt.ylabel(r'$\Delta\phi_i$ [°]', **axis_font)
plt.xlabel(r'$d_i$ [mm]', **axis_font)
plt.title(r'$P(\Delta\phi_i , d_i)$', **axis_font)

plt.xlim([80, 180])
plt.ylim([-45, 45])
plt.tight_layout()





plt.show()