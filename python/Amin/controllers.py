import numpy as np
import matplotlib.pyplot as plt

from weg_world_params_v2 import *

class Controller:
    state = 0
    current_spin = -1
    current_quad = 2
    quads = np.array([[0, -1], [1, 0], [0, 1], [-1, 0]])
    distance_threshold = 0.05
    def_vel = 4e-2
    def_max_f = 20

    def __init__(self, numBots, filter_k=0.1, def_vel=1e-2):
        self.num_bots = numBots
        self.posit_bots = np.zeros((numBots, 2))
        self.vel_bots = np.zeros((numBots, 2))
        self.norm_bots = np.zeros((numBots, 2))
        self.sensor_bots = np.zeros((numBots, 1))
        self.mean_dir = np.array([0., -1.])
        self.filtered_mean_dir = self.mean_dir
        self.filter_k = filter_k
        self.def_vel = def_vel

    def updateInput(self, observation):
        self.posit_bots = observation[:, :2]
        self.vel_bots = observation[:, 2:4]
        norm_tmp = observation[:, 4:6]
        self.norm_bots = []
        for i in range(self.num_bots):
            if i % 2 == 0:
                tmp = [norm_tmp[i, 1], -norm_tmp[i, 0]]
            else:
                tmp = [norm_tmp[i, 0], norm_tmp[i, 1]]
            tmp = np.array(tmp) / np.linalg.norm(tmp)
            self.norm_bots.append(tmp)
        self.norm_bots = np.array(self.norm_bots)
        self.true_norm_bots = observation[:, 4:6]
        self.sensor_bots = observation[:, 6]
        self.mean_dir = np.sum(np.multiply(self.norm_bots.T, self.sensor_bots), 1) / self.num_bots
        self.filtered_mean_dir = self.filtered_mean_dir * (1 - self.filter_k) + self.mean_dir * self.filter_k

    # Returns the quadrant with largest measuring distance
    def findMaxQuad(self):
        quad_idx = np.argmax(np.sum(np.multiply(self.quads, self.filtered_mean_dir), 1))
        self.current_quad = quad_idx

    # Returns the quadrant to the right or left with the largest distance and the spin direction
    def findNewQuad(self):
        quad_left = (self.current_quad + 1) % 4
        quad_right = (self.current_quad + 3) % 4

        new_quads = np.array([self.quads[quad_left, :], self.quads[quad_right, :]])

        quad_idx = np.argmax(np.sum(np.multiply(new_quads, self.filtered_mean_dir), 1))

        # Left
        if quad_idx == 0:
            self.current_quad = quad_left
            self.current_spin = 1
        else:
            self.current_quad = quad_right
            self.current_spin = -1

    def getCurrentDirDistance(self):
        tmp = np.sum(np.multiply(self.norm_bots, self.quads[self.current_quad, :]), 1)
        dist = np.average(self.sensor_bots[tmp > 0.5])

        return dist

    # Updates the heading quadrant
    def getActions(self, observation):
        self.updateInput(observation)

        if self.state == 0:
            self.findMaxQuad()
            self.state = 1

        elif self.state == 1 and self.getCurrentDirDistance() < self.distance_threshold:
            self.findNewQuad()

        ref = self.quads[self.current_quad, :]
        actions = np.sum(np.multiply(self.norm_bots, ref), 1)

        actions[actions > 0.7] = 1
        actions[actions < -0.7] = -1
        actions[np.abs(actions) < 0.8] = 0

        for i in range(len(actions)):
            if i % 2 == 0:
                actions[i] = self.current_spin

        # Apply speed controller:
        # self.vel_bots
        desired_vel = self.def_vel * actions
        current_vel = np.sum(np.multiply(self.true_norm_bots, self.vel_bots), 1)

        new_actions = 2000 * (desired_vel - current_vel)

        new_actions[np.abs(actions) < 0.8] = new_actions[np.abs(actions) < 0.8] * .5

        new_actions[new_actions > self.def_max_f] = self.def_max_f
        new_actions[new_actions < -self.def_max_f] = -self.def_max_f

        # tmp = np.multiply(self.def_vel * actions)

        return new_actions

# Mimics amin's control strategy
class AminController(Controller):
    current_spin = 1
    current_quad = 9
    quads = np.array(
        [np.cos(np.linspace(0, np.pi * 2 - np.pi / 6, 12)), np.sin(np.linspace(0, np.pi * 2 - np.pi / 6, 12))]).T

    def calculateAD(self):
        n_des = self.quads[self.current_quad, :]

        cos_norm = np.dot(self.norm_bots, n_des)

        ad = cos_norm * self.sensor_bots

        ad = ad[cos_norm > np.cos(np.pi / 6)]
        ad = np.average(ad)

        return ad

    def updateSpins(self):
        # Calculate average distance
        AD = self.calculateAD()

        if AD < self.distance_threshold:
            self.current_quad = (self.current_quad - 1) % 12

        # Get reference quad
        ref = self.quads[self.current_quad, :]

        # get desired velocities
        actions = np.sign(np.sum(np.multiply(self.norm_bots, ref), 1))
        actions[np.arange(0, self.num_bots, 2)] = self.current_spin

        self.actions = actions

    def getActions(self, observation):
        self.updateInput(observation)

        actions = self.actions

        # Calculate forces
        desired_vel = self.def_vel * actions
        current_vel = np.sum(np.multiply(self.true_norm_bots, self.vel_bots), 1)
        new_actions = 1000 * (desired_vel - current_vel)

        new_actions[np.abs(actions) < 0.8] = new_actions[np.abs(actions) < 0.8] * .5

        new_actions[new_actions > self.def_max_f] = self.def_max_f
        new_actions[new_actions < -self.def_max_f] = -self.def_max_f

        # tmp = np.multiply(self.def_vel * actions)

        return new_actions


class WegController:

    # Random Generator:
    rng = np.random.default_rng(463182)

    # Number of bots
    num_bots = -1

    # Threshold before detecting wall
    distance_threshold = 50

    # Current spin direction
    current_spin = 1

    # Current direction
    current_quad = 9

    # possible movement directions
    control_dirs = np.arange(12) / 12 * 2 * np.pi

    # Possible target direction
    quads = np.array([np.cos(np.linspace(0, np.pi * 2 - np.pi / 6, 12)),
                      np.sin(np.linspace(0, np.pi * 2 - np.pi / 6, 12))]).T

    dt_sim = dt  # 0.0001     # [s] -     Simulation timestep
    Dt_cont = dt_cont  # 0.1       # [s] -     Controller timestep
    Nt_cont = int(Dt_cont / dt_sim)  # steps per simulation
    F_slip = 5.5  # [N] -     Slip force of wheel when in contact with ground
    F_drag = .25 # [N] - Drag force when weg is not in contact with the ground
    D_wheel = 20  # [mm] -    Wheel diameter
    V_wheel = 44.8        # [mm/s] -  Wheel speed
    W_wheel = V_wheel * 2 / D_wheel  # [rad/s] - Wheel spin velocity
    slip_per = 0.85  # [-] -     Fraction of time wheel is in contact with ground
    slip_ppr = 4  # [-] -     Slip cycles per revolution
    slip_drift = []

    current_time = 0
    update_controller = False

    control_actions = []

    counter = 0

    # Initialize variables
    def __init__(self, numBots, dt_sim, Dt_cont, F_slip, slip_per=0.5, slip_ppr=4):
        self.num_bots = numBots
        self.control_actions = np.zeros(self.num_bots)
        self.control_forces = np.zeros((self.num_bots, 2))
        self.slip_drift = self.rng.random((numBots, 1)) * 2 * np.pi
        self.posit_bots = np.zeros((numBots, 2))
        self.vel_bots = np.zeros((numBots, 2))
        self.norm_bots = np.zeros((numBots, 2))
        self.sensor_bots = np.zeros((numBots, 1))
        self.sensor_bots_type = np.zeros((numBots, 1))

        self.F_drag  = self.F_drag * 12 / self.num_bots

        self.dt_sim = dt_sim
        self.Dt_cont = Dt_cont
        self.Nt_cont = int(Dt_cont / dt_sim)
        self.F_slip = F_slip
        self.slip_per = slip_per
        self.slip_ppr = slip_ppr

        self.quads = np.vstack((np.cos(self.control_dirs), np.sin(self.control_dirs))).T


    def updateState(self, observation, j):
        self.posit_bots = observation[:, :2]
        self.vel_bots = observation[:, 2:4]
        self.norm_bots = observation[:, 4:6]
        self.wheel_norm_bots = []


        # Temporary code to calculate some statistics
        v_average = np.average(self.vel_bots, axis=0) * 1000
        v_des = self.V_wheel * self.quads[self.current_quad]

        tmp = (self.norm_bots @ v_des) @ self.control_actions / self.num_bots
        factor = np.linalg.norm(v_average) / tmp

        #print('Average velocity: ' + str(np.round(np.linalg.norm(v_average), 1)) + ' mm/s \t\t Lambda Factor: '
        #                                 + str(np.round(factor, 3)))

        # Code continues here
        for i in range(self.num_bots):
            if i % 2 == 0:
                tmp = [self.norm_bots[i, 1], -self.norm_bots[i, 0]]
            else:
                tmp = [self.norm_bots[i, 0], self.norm_bots[i, 1]]
            tmp = np.array(tmp) / np.linalg.norm(tmp)
            self.wheel_norm_bots.append(tmp)

        self.wheel_norm_bots = np.array(self.wheel_norm_bots)
        self.sensor_bots = observation[:, 6]
        self.sensor_bots_type = observation[:, 7]
        self.current_time = i * self.dt_sim

        # Set flag to update controller
        if j % self.Nt_cont == 0:
            self.update_controller = True
        else:
            self.update_controller = False

        return self.update()

    def updateWheelForces(self):
        # Update wheel position
        tmp = np.reshape(self.control_actions + (self.rng.random(self.num_bots) - .5) * .1, (self.num_bots, 1))
        self.slip_drift += tmp * self.W_wheel * self.dt_sim
        self.slip_drift[self.control_actions == 0] = 0

        # Calculate wheels which are in contact with the ground
        wheel_state =(self.slip_drift % (2 * np.pi / self.slip_ppr)) < 2 * np.pi * self.slip_per/ self.slip_ppr
        #wheel_state = np.sin((self.slip_drift) * self.slip_ppr + np.pi / 2) > 0

        v_diff = -(self.vel_bots - np.multiply(self.wheel_norm_bots.T,
                                               self.V_wheel * self.control_actions).T / 1000) * 1000

        v_diff = v_diff * 1000.
        v_diff = np.clip(v_diff, -self.F_slip, self.F_slip)

        v_drag = - self.vel_bots * 1000.
        v_drag = np.clip(v_drag, -self.F_drag, self.F_drag)

        v_diff[wheel_state[:, 0], :] = v_drag[wheel_state[:, 0], :]

        return v_diff

    def calculateADQuad(self, quad):
        n_des = self.quads[quad, :]

        cos_norm = np.dot(self.norm_bots, n_des)

        ad = cos_norm * self.sensor_bots

        ad = ad[cos_norm > np.cos(np.pi / 4)]
        ad = np.average(ad)

        return ad

    def calculateAD(self):

        return self.calculateADQuad(self.current_quad)

    # Calculates the average projected distance in each quadrant
    def calculateDistancesInDirections(self):
        cosines = np.dot(self.norm_bots[self.sensor_bots_type == 1, :], self.quads.T)
        proj_distances = np.multiply(self.sensor_bots[self.sensor_bots_type == 1], cosines.T)
        average_distances = []

        for c, d in zip(cosines.T, proj_distances):
            av_dist = np.average(d[c > np.cos(np.pi / 4)])
            average_distances.append(av_dist)

        return [average_distances[(self.current_quad - 2) % 12],
                average_distances[(self.current_quad - 1) % 12],
                average_distances[self.current_quad],
                average_distances[(self.current_quad + 1) % 12],
                average_distances[(self.current_quad + 2) % 12]]



    def updateActions(self):

        # For each direction calculate average dir
        av_distances = self.calculateDistancesInDirections()

        # To filter the response, wait for several measurements below this threshold, 1 is the same as not having this
        self.counter += 1

        if self.counter > 2:
            self.counter = 0
            # CHeck for direction change
            if av_distances[2] < self.distance_threshold / 1000:
                diff = np.argmax(av_distances) - 2
                self.current_quad += int(np.sign(diff))
                self.current_quad = self.current_quad % 12

                if diff * self.current_spin > 0:
                    self.current_spin *= -1

            else:
                self.counter = 0



        # Get reference quad
        ref = self.quads[self.current_quad, :]

        # get desired velocities
        tmp = np.dot(self.norm_bots, ref)
        actions = np.sign(tmp) * 1
        #actions[np.abs(tmp) < np.cos(40 / 180 * np.pi)] = 0
        #actions[np.abs(tmp) < np.cos(10 / 180 * np.pi)] = actions[np.abs(tmp) < np.cos(10 / 180 * np.pi)] + .5
        actions[self.sensor_bots < self.distance_threshold / 1000.] = -1
        actions[np.arange(0, self.num_bots, 2)] = self.current_spin * 1

        self.control_actions = actions

    def update(self):

        if self.update_controller:
            self.updateActions()
            self.update_controller = False


        forces = self.updateWheelForces()

        return forces



class Statistics(WegController):
    all_distances = []
    all_angles = []

    def startPlot(self):
        self.fig = plt.figure()
        self.ax = self.fig.subplots(1, 1)
        # self.line, = self.ax.plot([], [], '.')
        # self.line, = self.ax.scatter([], [])
        self.line = self.ax.imshow(np.zeros((100, 100)))
        # self.ax.set_xlim([0, 180])
        # self.ax.set_ylim([0.05, 0.2])

    def updateData(self, observation, j):
        self.updateState(observation, j)

        new_distances = []
        new_angles = []

        for i in range(self.num_bots):
            dist = np.linalg.norm(self.posit_bots[i, :] - self.posit_bots[(i + 1) % self.num_bots, :])
            ang = np.arccos(np.dot(self.norm_bots[i, :], self.norm_bots[(i + 1) % self.num_bots, :]) /
                            (np.linalg.norm(self.norm_bots[i, :]) * np.linalg.norm(
                                self.norm_bots[(i + 1) % self.num_bots, :])))
            tmp1 = np.arctan2(self.norm_bots[i, 1], self.norm_bots[i, 0])
            tmp2 = np.arctan2(self.norm_bots[(i + 1) % self.num_bots, 1], self.norm_bots[(i + 1) % self.num_bots, 0])

            ang = tmp1 - tmp2

            if ang > np.pi:
                ang -= np.pi * 2
            elif ang < - np.pi:
                ang += np.pi * 2

            # ang = np.arcsin(np.cross(self.norm_bots[(i + 1) % self.num_bots, :], self.norm_bots[i, :]))

            new_distances.append(dist)
            new_angles.append(ang)

        self.all_distances += new_distances
        self.all_angles += new_angles

    def getPxy(self):

        data_x = np.array(self.all_angles) * 180 / np.pi
        data_y = np.array(self.all_distances) * 1000

        data, x, y = np.histogram2d(data_x, data_y, bins=100)

        return data, x, y

    def getPyforx(self):
        k = 10000

        data_x = self.all_angles * 180 / np.pi
        data_y = self.all_distances * 1000

        data, x, y = np.histogram2d(data_x, data_y, bins=100)

        return data, x, y

    def getPyforx2(self):
        data_x = np.array(self.all_angles) * 180 / np.pi
        data_y = np.array(self.all_distances) * 1000

        data, x, y = np.histogram2d(data_x, data_y, bins=100)

        data2, x2 = np.histogram(data_x, bins=100)

        data = np.divide(data.T, np.sum(data, axis=1)) * 100

        data[data2 < 1e-10, :] = 0

        data = np.flip(data, axis=0)

        return data, x, y

    def updatePlot(self):

        data, x, y = self.getPyforx2()

        #plt.imshow(data, extent=[min(x), max(x), min(y), max(y)], aspect='auto')
        plt.imshow(data, extent=[0, 100, 80, 180], aspect='auto')
        plt.xlabel('Angle difference [°]')
        plt.ylabel('Distance [mm]')

        self.line.set_data(data)

        # self.line.set_xdata(x_data)
        # self.line.set_ydata(y_data)

        self.fig.canvas.draw()

        self.fig.canvas.flush_events()
        plt.pause(.001)