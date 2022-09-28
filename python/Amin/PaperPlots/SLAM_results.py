import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import re
import yaml

# Data format:
# xy_t, nxy_t, control, control_dir, xy_est, nxy_est

folder = 'SLAM_results/'
filename1 = 'True_12_bots.csv' # 'Test.csv' # 'True_12_bots.csv'
filename2 = 'Sim_12_bots.csv' # 'Test.csv' # 'True_12_bots.csv'
filename3 = 'Sim_120_bots.csv' # 'Test.csv' # 'True_12_bots.csv'

plt.rcParams.update({'font.size': 8,
                     'font.family': 'calibri'})
fig = plt.figure(1, figsize=(3.5 , 3.5 * 3.5 / 4.5 * 2), dpi=200)     # For small map


axs = fig.subplots(3, 1, sharex=True)

experimental = True


# For 12 bot system or 120:
scale = 1#1/1.05
offset = .1#-.1

L = .6415
S = .13324
points = [[0, 0],
          [L, 0],
          [L, L],
          [L+S, L],
          [L+S, 0],
          [3*L, 0],
          [3*L, 2*L - 2*S],
          [2*L, 2*L - 2*S],
          [2*L, L - 2*S],
          [2*L - S, L - 2*S],
          [2*L - S, 2*L - 2*S],
          [0, 2*L - 2*S],
          [0, 0]]
points = np.array(points).T


pm = np.average(points, axis=1).reshape([-1, 1])
points = (points - pm) * .9 + pm
points += np.array([[.07 + offset, 0.05]]).T
axs[0].set_xlim([S + (-S), S + (3 * L + S)])
axs[0].set_ylim([S / 2 + (-S), S / 2 + (2 * L - S)])

# Bottom line
axs[0].fill_between(points[0, :6], points[1, :6] - 10, points[1, :6], **{'color': 'gray'})
axs[0].fill_between(points[0, 6:12], points[1, 6:12], points[1, 6:12] + 10, **{'color': 'gray'})
axs[0].fill_betweenx([points[1, 5] - 10, points[1, 6] + 10], points[0, 5:6], points[1, 5:6] + 10, **{'color': 'gray'})
axs[0].fill_betweenx([points[1, 11] - 10, points[1, 12] + 10], [points[0, 11] - 10, points[0, 12] - 10], [points[0, 11], points[0, 12]], **{'color': 'gray'})

data = np.loadtxt(folder + filename1, delimiter=',')
data = data[::5, :]

N = int((data.shape[1] - 2) / 11)
t = data.shape[0]
fs = 10

t_max = 2000
if t > t_max:
    t = t_max
    data = data[:t_max, :]



time = np.linspace(0, t / fs, t)
xy_t = data[:, :2 * N].reshape((t, N, 2)) * scale
vxy_t = data[:, 2 * N:4 * N].reshape((t, N, 2)) * scale
nxy_t = data[:, 4 * N:6 * N].reshape((t, N, 2))
cont = data[:, 6 * N:7 * N]
cont_dir = data[:, 7 * N:7 * N + 2]
xy_e = data[:, 7 * N + 2:9 * N + 2].reshape((t, N, 2)) * scale
nxy_e = data[:, 9 * N + 2:].reshape((t, N, 2))



#####################################################
# Plot true and estimated positions                 #
#####################################################
av_xy_t = np.average(xy_t, axis=1)
av_xy_e = np.average(xy_e, axis=1)

#plt.figure(1, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)  # For large map
#plt.figure(1, figsize=(3.5 * 2 / 3, 3.5 * 3.5 / 4.5 * 2 / 3), dpi=200)     # For small map
#plt.rcParams.update({'font.size': 8,
#                     'font.family': 'calibri'})
#plt.title('Center of mass positioning error for ' + str(N) + ' sub-units')
axs[0].plot(av_xy_t[:, 0] + offset, av_xy_t[:, 1])
axs[0].plot(av_xy_e[:, 0] + offset, av_xy_e[:, 1])
#plt.xlabel('X axis [m]')
#plt.ylabel('Y axis [m]')
axs[0].grid()

#####################################################
# Plot Odometry path                                #
#####################################################

exp = nxy_e[:, :, 0] + 1j * nxy_e[:, :, 1]
exp[:, np.arange(N) % 2 == 1] = exp[:, np.arange(N) % 2 == 1] * (-1j)

velx = np.real(exp) * cont #* .011 / 3
vely = np.imag(exp) * cont #* .011 / 3

p_x_t = av_xy_t[:, 0]
p_y_t = av_xy_t[:, 1]
p_x_o = np.cumsum(np.average(velx, axis=1)) / fs
p_y_o = np.cumsum(np.average(vely, axis=1)) / fs

avx = np.average(p_x_t)
avy = np.average(p_y_t)
avx2 = np.average(p_x_o)
avy2 = np.average(p_y_o)

def err(k):
    global p_x_t
    global p_y_t
    global p_x_o
    global p_y_o
    global avx
    global avy
    global avx2
    global avy2

    error = (p_x_t - avx - (p_x_o - avx2) * k)**2 + (p_y_t - avy - (p_y_o - avy2) * k)**2

    return np.average(error)

out = minimize(err, [.011], bounds=[(0.0001, 1)])

print(out)

#p_x_o = p_x_o * out.x + av_xy_t[0, 0]
#p_y_o = p_y_o * out.x + av_xy_t[0, 1]

# Simulation
#p_x_o = p_x_o * .0172 + av_xy_t[0, 0]
#p_y_o = p_y_o * .0172 + av_xy_t[0, 1]
# Esperimental

if N < 20:
    p_x_o = p_x_o * .01686 + av_xy_t[0, 0]
    p_y_o = p_y_o * .01686 + av_xy_t[0, 1]
else:
    p_x_o = p_x_o * .02179 + av_xy_t[0, 0]
    p_y_o = p_y_o * .02179 + av_xy_t[0, 1]

#plt.plot(p_x_o + offset, p_y_o, '--')
#plt.legend(['Ground truth', 'SLAM estimate', 'Odometry'])
#axs[0].tight_layout()

#####################################################
# Plot map                                          #
#####################################################
def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename + '.pgm', 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)

    with open(filename + '.yaml', "r") as stream:
        try:
            data_head = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    data_arr = np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

    x = np.linspace(0, data_arr.shape[1] * data_head['resolution'], data_arr.shape[1]) + data_head['origin'][0]
    y = -np.linspace(0, data_arr.shape[0] * data_head['resolution'], data_arr.shape[0]) - data_head['origin'][1]

    return data_arr, data_head, x, y, data_head['occupied_thresh'] * 255, data_head['free_thresh']*255

image, head, x, y, t1, t2 = read_pgm("SLAM_results/" + filename1.split('.')[0] + '/map', byteorder='<')
x, y = np.meshgrid(x, y)
mask = image > t1
alpha = np.ones(image.shape)
alpha[mask] = 0
tmp = np.array(image, dtype=np.float)
tmp[mask] = np.nan
#plt.figure(5)
axs[0].pcolor(x, y, tmp)
#plt.xlim([-.15, 3.55])
#plt.ylim([-.15, 3.15])
axs[0].set_aspect('equal')
#.tight_layout()

########################################################################################################################

experimental = False

L = .6415
S = .13324
points = [[0, 0],
          [L, 0],
          [L, L],
          [L+S, L],
          [L+S, 0],
          [3*L, 0],
          [3*L, 2*L - 2*S],
          [2*L, 2*L - 2*S],
          [2*L, L - 2*S],
          [2*L - S, L - 2*S],
          [2*L - S, 2*L - 2*S],
          [0, 2*L - 2*S],
          [0, 0]]
points = np.array(points).T

if experimental:
    pm = np.average(points, axis=1).reshape([-1, 1])
    points = (points - pm) * .9 + pm
    points += np.array([[.07, 0]]).T
    axs[1].set_xlim([(-S), (3 * L + S)])
    axs[1].set_ylim([(-S), (2 * L - S)])
else:
    points += np.array([[S, S/2]]).T
    axs[1].set_xlim([S + (-S), S + (3 * L + S)])
    axs[1].set_ylim([S / 2 + (-S), S / 2 + (2 * L - S)])

# Bottom line
axs[1].fill_between(points[0, :6], points[1, :6] - 10, points[1, :6], **{'color': 'gray'})
axs[1].fill_between(points[0, 6:12], points[1, 6:12], points[1, 6:12] + 10, **{'color': 'gray'})
axs[1].fill_betweenx([points[1, 5] - 10, points[1, 6] + 10], points[0, 5:6], points[1, 5:6] + 10, **{'color': 'gray'})
axs[1].fill_betweenx([points[1, 11] - 10, points[1, 12] + 10], [points[0, 11] - 10, points[0, 12] - 10], [points[0, 11], points[0, 12]], **{'color': 'gray'})

data = np.loadtxt(folder + filename2, delimiter=',')
data = data[::5, :]

N = int((data.shape[1] - 2) / 11)
t = data.shape[0]
fs = 10

t_max = 12000
if t > t_max:
    t = t_max
    data = data[:t_max, :]

# For 12 bot system or 120:
scale = 1/1.05
offset = -.1

time = np.linspace(0, t / fs, t)
xy_t = data[:, :2 * N].reshape((t, N, 2)) * scale
vxy_t = data[:, 2 * N:4 * N].reshape((t, N, 2)) * scale
nxy_t = data[:, 4 * N:6 * N].reshape((t, N, 2))
cont = data[:, 6 * N:7 * N]
cont_dir = data[:, 7 * N:7 * N + 2]
xy_e = data[:, 7 * N + 2:9 * N + 2].reshape((t, N, 2)) * scale
nxy_e = data[:, 9 * N + 2:].reshape((t, N, 2))



#####################################################
# Plot true and estimated positions                 #
#####################################################
av_xy_t = np.average(xy_t, axis=1)
av_xy_e = np.average(xy_e, axis=1)

#plt.figure(1, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)  # For large map
#plt.figure(1, figsize=(3.5 * 2 / 3, 3.5 * 3.5 / 4.5 * 2 / 3), dpi=200)     # For small map
#plt.rcParams.update({'font.size': 8,
#                     'font.family': 'calibri'})
#plt.title('Center of mass positioning error for ' + str(N) + ' sub-units')
axs[1].plot(av_xy_t[:, 0] + offset, av_xy_t[:, 1])
axs[1].plot(av_xy_e[:, 0] + offset, av_xy_e[:, 1])
#plt.xlabel('X axis [m]')
#plt.ylabel('Y axis [m]')
axs[1].grid()

#####################################################
# Plot Odometry path                                #
#####################################################

exp = nxy_e[:, :, 0] + 1j * nxy_e[:, :, 1]
exp[:, np.arange(N) % 2 == 1] = exp[:, np.arange(N) % 2 == 1] * (-1j)

velx = np.real(exp) * cont #* .011 / 3
vely = np.imag(exp) * cont #* .011 / 3

p_x_t = av_xy_t[:, 0]
p_y_t = av_xy_t[:, 1]
p_x_o = np.cumsum(np.average(velx, axis=1)) / fs
p_y_o = np.cumsum(np.average(vely, axis=1)) / fs

avx = np.average(p_x_t)
avy = np.average(p_y_t)
avx2 = np.average(p_x_o)
avy2 = np.average(p_y_o)

def err(k):
    global p_x_t
    global p_y_t
    global p_x_o
    global p_y_o
    global avx
    global avy
    global avx2
    global avy2

    error = (p_x_t - avx - (p_x_o - avx2) * k)**2 + (p_y_t - avy - (p_y_o - avy2) * k)**2

    return np.average(error)

out = minimize(err, [.011], bounds=[(0.0001, 1)])

print(out)

#p_x_o = p_x_o * out.x + av_xy_t[0, 0]
#p_y_o = p_y_o * out.x + av_xy_t[0, 1]

# Simulation
#p_x_o = p_x_o * .0172 + av_xy_t[0, 0]
#p_y_o = p_y_o * .0172 + av_xy_t[0, 1]
# Esperimental

if N < 20:
    p_x_o = p_x_o * .01686 + av_xy_t[0, 0]
    p_y_o = p_y_o * .01686 + av_xy_t[0, 1]
else:
    p_x_o = p_x_o * .02179 + av_xy_t[0, 0]
    p_y_o = p_y_o * .02179 + av_xy_t[0, 1]

#plt.plot(p_x_o + offset, p_y_o, '--')
#plt.legend(['Ground truth', 'SLAM estimate', 'Odometry'])
#plt.tight_layout()

#####################################################
# Plot map                                          #
#####################################################
def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename + '.pgm', 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)

    with open(filename + '.yaml', "r") as stream:
        try:
            data_head = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    data_arr = np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

    x = np.linspace(0, data_arr.shape[1] * data_head['resolution'], data_arr.shape[1]) + data_head['origin'][0]
    y = -np.linspace(0, data_arr.shape[0] * data_head['resolution'], data_arr.shape[0]) - data_head['origin'][1]

    return data_arr, data_head, x, y, data_head['occupied_thresh'] * 255, data_head['free_thresh']*255

image, head, x, y, t1, t2 = read_pgm("SLAM_results/" + filename2.split('.')[0] + '/map', byteorder='<')
x, y = np.meshgrid(x, y)
mask = image > t1
alpha = np.ones(image.shape)
alpha[mask] = 0
tmp = np.array(image, dtype=np.float)
tmp[mask] = np.nan
#plt.figure(5)
axs[1].pcolor(x, y, tmp)
#plt.xlim([-.15, 3.55])
#plt.ylim([-.15, 3.15])
axs[1].set_aspect('equal')

axs[1].set_ylabel('Y axis [m]', font='calibri')
#axs[1].set_xlabel(' ')
#plt.tight_layout()
########################################################################################################################

experimental = False

L = .6415
S = .13324
points = [[0, 0],
          [L, 0],
          [L, L],
          [L+S, L],
          [L+S, 0],
          [3*L, 0],
          [3*L, 2*L - 2*S],
          [2*L, 2*L - 2*S],
          [2*L, L - 2*S],
          [2*L - S, L - 2*S],
          [2*L - S, 2*L - 2*S],
          [0, 2*L - 2*S],
          [0, 0]]
points = np.array(points).T

if experimental:
    pm = np.average(points, axis=1).reshape([-1, 1])
    points = (points - pm) * .9 + pm
    points += np.array([[.07, 0]]).T
    axs[2].set_xlim([(-S), (3 * L + S)])
    axs[2].set_ylim([(-S), (2 * L - S)])
else:
    points += np.array([[S, S/2]]).T
    axs[2].set_xlim([S + (-S), S + (3 * L + S)])
    axs[2].set_ylim([S / 2 + (-S), S / 2 + (2 * L - S)])

# Bottom line
axs[2].fill_between(points[0, :6], points[1, :6] - 10, points[1, :6], **{'color': 'gray'})
axs[2].fill_between(points[0, 6:12], points[1, 6:12], points[1, 6:12] + 10, **{'color': 'gray'})
axs[2].fill_betweenx([points[1, 5] - 10, points[1, 6] + 10], points[0, 5:6], points[1, 5:6] + 10, **{'color': 'gray'})
axs[2].fill_betweenx([points[1, 11] - 10, points[1, 12] + 10], [points[0, 11] - 10, points[0, 12] - 10], [points[0, 11], points[0, 12]], **{'color': 'gray'})

data = np.loadtxt(folder + filename3, delimiter=',')
data = data[::5, :]

N = int((data.shape[1] - 2) / 11)
t = data.shape[0]
fs = 10

t_max = 12000
if t > t_max:
    t = t_max
    data = data[:t_max, :]

# For 12 bot system or 120:
scale = 1/1.05
offset = -.1

time = np.linspace(0, t / fs, t)
xy_t = data[:, :2 * N].reshape((t, N, 2)) * scale
vxy_t = data[:, 2 * N:4 * N].reshape((t, N, 2)) * scale
nxy_t = data[:, 4 * N:6 * N].reshape((t, N, 2))
cont = data[:, 6 * N:7 * N]
cont_dir = data[:, 7 * N:7 * N + 2]
xy_e = data[:, 7 * N + 2:9 * N + 2].reshape((t, N, 2)) * scale
nxy_e = data[:, 9 * N + 2:].reshape((t, N, 2))



#####################################################
# Plot true and estimated positions                 #
#####################################################
av_xy_t = np.average(xy_t, axis=1)
av_xy_e = np.average(xy_e, axis=1)

#plt.figure(1, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)  # For large map
#plt.figure(1, figsize=(3.5 * 2 / 3, 3.5 * 3.5 / 4.5 * 2 / 3), dpi=200)     # For small map
#plt.rcParams.update({'font.size': 8,
#                     'font.family': 'calibri'})
#plt.title('Center of mass positioning error for ' + str(N) + ' sub-units')
axs[2].plot(av_xy_t[:, 0] + offset, av_xy_t[:, 1])
axs[2].plot(av_xy_e[:, 0] + offset, av_xy_e[:, 1])
#plt.xlabel('X axis [m]')
#plt.ylabel('Y axis [m]')
axs[2].grid()

#####################################################
# Plot Odometry path                                #
#####################################################

exp = nxy_e[:, :, 0] + 1j * nxy_e[:, :, 1]
exp[:, np.arange(N) % 2 == 1] = exp[:, np.arange(N) % 2 == 1] * (-1j)

velx = np.real(exp) * cont #* .011 / 3
vely = np.imag(exp) * cont #* .011 / 3

p_x_t = av_xy_t[:, 0]
p_y_t = av_xy_t[:, 1]
p_x_o = np.cumsum(np.average(velx, axis=1)) / fs
p_y_o = np.cumsum(np.average(vely, axis=1)) / fs

avx = np.average(p_x_t)
avy = np.average(p_y_t)
avx2 = np.average(p_x_o)
avy2 = np.average(p_y_o)

def err(k):
    global p_x_t
    global p_y_t
    global p_x_o
    global p_y_o
    global avx
    global avy
    global avx2
    global avy2

    error = (p_x_t - avx - (p_x_o - avx2) * k)**2 + (p_y_t - avy - (p_y_o - avy2) * k)**2

    return np.average(error)

out = minimize(err, [.011], bounds=[(0.0001, 1)])

print(out)

#p_x_o = p_x_o * out.x + av_xy_t[0, 0]
#p_y_o = p_y_o * out.x + av_xy_t[0, 1]

# Simulation
#p_x_o = p_x_o * .0172 + av_xy_t[0, 0]
#p_y_o = p_y_o * .0172 + av_xy_t[0, 1]
# Esperimental

if N < 20:
    p_x_o = p_x_o * .01686 + av_xy_t[0, 0]
    p_y_o = p_y_o * .01686 + av_xy_t[0, 1]
else:
    p_x_o = p_x_o * .02179 + av_xy_t[0, 0]
    p_y_o = p_y_o * .02179 + av_xy_t[0, 1]

#plt.plot(p_x_o + offset, p_y_o, '--')
#plt.legend(['Ground truth', 'SLAM estimate', 'Odometry'])
#plt.tight_layout()

#####################################################
# Plot map                                          #
#####################################################
def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename + '.pgm', 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)

    with open(filename + '.yaml', "r") as stream:
        try:
            data_head = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    data_arr = np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

    x = np.linspace(0, data_arr.shape[1] * data_head['resolution'], data_arr.shape[1]) + data_head['origin'][0]
    y = -np.linspace(0, data_arr.shape[0] * data_head['resolution'], data_arr.shape[0]) - data_head['origin'][1]

    return data_arr, data_head, x, y, data_head['occupied_thresh'] * 255, data_head['free_thresh']*255

image, head, x, y, t1, t2 = read_pgm("SLAM_results/" + filename3.split('.')[0] + '/map', byteorder='<')
x, y = np.meshgrid(x, y)
mask = image > t1
alpha = np.ones(image.shape)
alpha[mask] = 0
tmp = np.array(image, dtype=np.float)
tmp[mask] = np.nan
#plt.figure(5)
axs[2].pcolor(x, y, tmp)
#plt.xlim([-.15, 3.55])
#plt.ylim([-.15, 3.15])
axs[2].set_aspect('equal')

#axs[2].plot(points[0, :], points[1, :], '-k')
#axs[2].set_ylabel(' ')
axs[2].set_xlabel('X axis [m]', font='calibri')
#plt.tight_layout()


# For large map only
#plt.xlim([-.1, 3.75])
#plt.ylim([0, 3.45])

#####################################################
# Plot average abs position error                   #
#####################################################
# plt.figure(2, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)
# plt.rcParams.update({'font.size': 8,
#                      'font.family': 'calibri'})
# errx = xy_t[:, :, 0] - xy_e[:, :, 0]#(xy_t[:, :, 0].T - av_xy_t[:, 0]).T - (xy_e[:, :, 0].T - av_xy_e[:, 0]).T
# erry = xy_t[:, :, 1] - xy_e[:, :, 1]#(xy_t[:, :, 1].T - av_xy_t[:, 1]).T - (xy_e[:, :, 1].T - av_xy_e[:, 1]).T
# err = errx**2 + erry**2
# err = np.sqrt(err)
#
# err_mu = np.mean(err, axis=1) * 1000
# err_sig = np.std(err, axis=1) * 1000
#
# plt.plot(time, err_mu)
# plt.fill_between(time, err_mu - err_sig * 3, err_mu + err_sig * 3, facecolor='C0', alpha=0.5)
# plt.xlabel('Time [s]')
# plt.ylabel('Error [mm]')
# plt.title('Average positioning error of ' + str(N) + ' sub-units [$\pm 3\cdot\sigma$]')
# plt.grid()
#
# #####################################################
# # Plot relative abs position error                  #
# #####################################################
# plt.figure(3, figsize=(3.5, 3.5 * 3.5 / 4.5), dpi=200)
# plt.rcParams.update({'font.size': 8,
#                      'font.family': 'calibri'})
# errx = (xy_t[:, :, 0].T - av_xy_t[:, 0]).T - (xy_e[:, :, 0].T - av_xy_e[:, 0]).T
# erry = (xy_t[:, :, 1].T - av_xy_t[:, 1]).T - (xy_e[:, :, 1].T - av_xy_e[:, 1]).T
# err = errx**2 + erry**2
# err = np.sqrt(err)
#
# err_mu = np.mean(err, axis=1) * 1000
# err_sig = np.std(err, axis=1) * 1000
#
# plt.plot(time, err_mu)
# plt.fill_between(time, err_mu - err_sig, err_mu + err_sig, facecolor='C0', alpha=0.5)
# plt.xlabel('Time [s]')
# plt.ylabel('Error [mm]')
# plt.title('Average shape formation error of ' + str(N) + ' sub-units [$\pm \sigma$]')
# plt.grid()

# plt.figure()
# for n in exp:
#     plt.clf()
#     plt.quiver(xy_e[0, :, 0], xy_e[0, :, 1], np.real(n), np.imag(n))
#     plt.pause(.01)

fig.set_tight_layout(True)
plt.show()