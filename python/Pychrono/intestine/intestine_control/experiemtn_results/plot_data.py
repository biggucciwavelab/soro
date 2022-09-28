# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 14:57:31 2020

@author: dmulr
"""

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
from os import listdir
####################################################


####################################################
class DataPlotter:
    """ A class to plot the data of a previous test """
    def __init__(self, filename, freq=5., standard_notation=True):
        """ Initialisation of the class """
        self.freq = freq
        self.dt = 1. / self.freq

        # Getting the data from the file
        self.data = np.genfromtxt(filename, delimiter=',')
        self.num_robot = int(len(self.data[0, :]) / 3)
        self.data_length = len(self.data[:, 0])
        self.data = self.data[3:self.data_length, :]
        self.data_length = len(self.data[:, 0])
        self.t = np.arange(self.data_length) * self.dt
        self.speed = 0
        self.meanx = np.zeros(self.data_length)
        self.meany = np.zeros(self.data_length)
        if standard_notation:
            self.save_dir = filename.replace("data.csv", "anim.mp4")
        else:
            self.save_dir = filename.replace(".csv", ".mp4")

    def animate(self, style="vib18", speed=1., save=False, show=True):
        """ Method to animate the data """
        if style == "vib18":
            self._animate_vib18(speed=speed, save=save, show=show)
        elif style == "string":
            self._animate_string(speed=speed, save=save, show=show)

    def _animate_vib18(self, speed=1., save=False, show=True):
        """ Method to animate the data for the 18 vibration robot"""
        self.compute_mean(style="vib18")
        self.speed = speed
        fig = plt.figure()
        ax = plt.axes(xlim=(-100, 100), ylim=(-100, 100))
        ax.set_aspect("equal")
        ax.set_xlabel("x position (cm)")
        ax.set_ylabel("y position (cm)")
        self.line, = ax.plot([], [], c="pink", lw=8)
        self.point, = ax.plot([], [], c="red", lw=0, marker="+")
        self.center, = ax.plot([], [], c="blue", lw=2)
        ax.legend((self.center, self.point), ('Center of Robot', 'Target'), loc='upper right', shadow=True)
        self.time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        anim = FuncAnimation(fig, self._frame_vib18, init_func=self._init_vib18, frames=len(self.t), interval=self.dt*1000./self.speed, blit=True)
        if save:
            anim.save(self.save_dir, writer='ffmpeg')
        if show:
            plt.show()

    def _init_vib18(self):
        self.line.set_data([], [])
        self.point.set_data([], [])
        self.center.set_data([], [])
        self.time_text.set_text('')
        return self.line, self.point, self.center, self.time_text

    def _frame_vib18(self, i):
        x = np.zeros(self.num_robot)
        y = np.zeros(self.num_robot)
        for j in range(self.num_robot-1):
            x[j] = self.data[i, 3*j]
            y[j] = -self.data[i, 3*j + 1]
        x[-1] = x[0]
        y[-1] = y[0]
        self.line.set_data(x, y)
        x = self.meanx[0:i]
        y = -self.meany[0:i]
        self.center.set_data(x, y)
        x = self.data[i, 3*(self.num_robot - 1)]
        y = -self.data[i, 3*(self.num_robot - 1) + 1]
        self.point.set_data(x, y)
        self.time_text.set_text('Time = %.1fs (%.0fX)' % (self.t[i], self.speed))
        return self.line, self.point, self.center, self.time_text

    def _animate_string(self, speed=1., save=False, show=True):
        """ Method to animate the data for the 18 vibration robot"""
        self.compute_mean(style="string")
        self.speed = speed
        fig = plt.figure()
        ax = plt.axes(xlim=(-100, 100), ylim=(-100, 100))
        ax.set_aspect("equal")
        ax.set_xlabel("x position (cm)")
        ax.set_ylabel("y position (cm)")
        self.line, = ax.plot([], [], c="red", lw=0, marker='o')
        self.center, = ax.plot([], [], c="blue", lw=2)
        ax.legend((self.line, self.center), ('Robot node', 'Center of Robot'), loc='upper right', shadow=True)
        self.time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        anim = FuncAnimation(fig, self._frame_string, init_func=self._init_string, frames=len(self.t), interval=self.dt*1000./self.speed, blit=True)
        if save:
            anim.save(self.save_dir, writer='ffmpeg')
        if show:
            plt.show()

    def _init_string(self):
        self.line.set_data([], [])
        self.center.set_data([], [])
        self.time_text.set_text('')
        return self.line, self.center, self.time_text

    def _frame_string(self, i):
        x = np.zeros(self.num_robot)
        y = np.zeros(self.num_robot)
        for j in range(self.num_robot):
            x[j] = self.data[i, 3*j]
            y[j] = -self.data[i, 3*j + 1]
        self.line.set_data(x, y)
        x = self.meanx[0:i]
        y = -self.meany[0:i]
        self.center.set_data(x, y)
        self.time_text.set_text('Time = %.1fs (%.0fX)' % (self.t[i], self.speed))
        return self.line, self.center, self.time_text

    def plot(self):
        """ Method to plot the data """
        for i in range(self.num_robot):
            plt.figure()
            plt.title("Robot #" + str(i))
            plt.plot(self.t, self.data[:, 3 * i], label='X')
            plt.plot(self.t, self.data[:, 3 * i + 1], label='Y')
            plt.plot(self.t, self.data[:, 3 * i + 2], label='Angle')

            plt.legend()

        plt.figure()
        plt.title("Robots in 2D")
        for i in range(self.num_robot):
            plt.plot(self.data[:, 3 * i], self.data[:, 3 * i + 1], label="Robot #" + str(i))

            plt.legend()

        plt.show()

    def compute_mean(self, style="vib18"):
        """ Method to compute the mean of the robots over time """
        if style == "vib18":
            num = self.num_robot - 1
        elif style == "string":
            num = self.num_robot
        else:
            num = self.num_robot
        for i in range(num):
            self.meanx += self.data[:, 3 * i] / self.num_robot
            self.meany += self.data[:, 3 * i + 1] / self.num_robot
####################################################


# Set the file path
filepath = "C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/PyChrono/intestine controls/Good_torus_codes/experiemtn_results/collapsing_v1/"

# Extract every files from the path
files = listdir(filepath)

# Create the Vibration robot
for file in files:
    # Get the filename
    filename = filepath + file

    # Start the data plotter
    dp = DataPlotter(filename=filename, standard_notation=False)

    # Animate the data
    dp.animate(speed=20., save=False, show=True, style="string")

    # Delete the data plotter
    del dp

# Plot the data
# dp.plot()