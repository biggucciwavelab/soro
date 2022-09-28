def main():
    pass


if __name__ == '__main__':
    main()

import os
import math
import time
import sys, getopt
import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
import matplotlib.lines as mlines
from matplotlib import animation


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


# In[Class to store Robot data]
class InitRobot:

    def __init__(self, num_of_robots, marker, goal, method):
        self.qx = []
        self.qz = []
        self.marker = marker
        self.num_of_robots = num_of_robots
        self.goal = goal
        self.cosTheta = np.zeros(12)
        self.control_method = method
        self.t_a = 45
        self.dist = 0

        self.angle_mode = "relative"
        self.stopped_robot = []
        self.data_dict = {}
        self.state = np.zeros(self.num_of_robots)
        self.at_least_one_stopped = False
        self.stop_dist_default = 0.1
        self.stop_dist = np.ones(self.num_of_robots) * self.stop_dist_default
        self.xo = self.goal.GetAbsFrame().GetPos().x
        self.yo = self.goal.GetAbsFrame().GetPos().z
        self.first_row = np.zeros(6)
        self.back_row = np.zeros(6)
        self.force = 0
        self.K = 0.8
        self.circle_initialization = False
        self.dx={}
        self.dy={}
        self.dz={}
        self.dphi={}
        for i in range(self.num_of_robots):
            self.dx["botx{0}".format(i)]=[]
            self.dy["boty{0}".format(i)]=[]
            self.dz["botz{0}".format(i)]=[]
    # Extract data
    def extract_data(self):
        self.qx = []
        self.qz = []
        for i in range(0, self.num_of_robots):
            self.qx.append(self.marker[str(i)].GetAbsFrame().GetPos().x)
            self.qz.append(self.marker[str(i)].GetAbsFrame().GetPos().z)
            d1v = np.array([self.marker[str(i)].GetAbsFrame().GetPos().x - self.goal.GetAbsFrame().GetPos().x,
                            self.marker[str(i)].GetAbsFrame().GetPos().z - self.goal.GetAbsFrame().GetPos().z])
            d1v = d1v / np.linalg.norm(d1v)
            q = self.marker[str(i)].GetAbsFrame().GetRot()
            d2 = q.Rotate(chrono.ChVectorD(1, 0, 0))
            d2v = np.array([d2.x, d2.z])
            d2v = d2v / np.linalg.norm(d2v)
            self.cosTheta[i] = np.dot(d1v, d2v)
            self.data_dict[str(i)] = np.array([self.qx[-1], self.qz[-1], np.arccos(d2v[0])])
        self.force = abs(self.cosTheta[(np.cos(self.t_a * np.pi / 180) <= self.cosTheta) * (self.cosTheta <= 1)]).sum() + \
                     abs(self.cosTheta[(-1 <= self.cosTheta) * (self.cosTheta <= -np.cos(self.t_a * np.pi / 180))]).sum()

    def save_data(self):
        for i in range(self.num_of_robots):
            self.dx['botx'+str(i)].append(self.marker[str(i)].GetAbsFrame().GetPos().x)
            self.dy['boty'+str(i)].append(self.marker[str(i)].GetAbsFrame().GetPos().y)
            self.dz['botz'+str(i)].append(self.marker[str(i)].GetAbsFrame().GetPos().z)
        return(self.dx,self.dy,self.dz)
    def controller(self):
        """ Method to send the desired signals to the photons """
        alpha = self.get_mean_polar_angle(self.data_dict)
        angle = self.get_polar_angle(self.data_dict)
        self.dist = self.get_dist(self.data_dict)
        closest_id = self.get_min_dist_id(dist=self.dist)
        theta = np.zeros(self.num_of_robots)  # Relative heading angles: depends on the relative angle computation mode
        self.stopped_robot = sum(1 * (self.stop_dist == self.stop_dist_default + 0.05))
        self.at_least_one_stopped = self.stopped_robot >= 1  # At least one stopped
        if self.at_least_one_stopped:  # If one robot is stopped
            self.angle_mode = "abs_closest"
        else:
            self.angle_mode = "relative"
        # Get the angles values of the legged robot and put them in a array for the optimization
        for i in range(0, self.num_of_robots):
            if self.angle_mode == "relative":
                theta[i] = angle[i] - self.data_dict[str(i)][2]  # Each robot individually follows the target
            elif self.angle_mode == "closest":
                theta[i] = angle[closest_id] - self.data_dict[str(i)][
                    2]  # Engulfing mode: use the closest robot to the object as center of rotation
            elif self.angle_mode == "mean":
                theta[i] = alpha - self.data_dict[str(i)][2]  # Center of robot follows the target
            elif self.angle_mode == "abs_closest":
                theta[i] = self.data_dict[str(self.get_stopped_closest(robot_id=i, data_dict=self.data_dict))][2] - \
                           self.data_dict[str(i)][
                               2]  # Engulfing mode: use the closest stopped robot to robot i as center of rotation
        result = self.get_opt_result(method=self.control_method, theta=theta, angle=angle, dist=self.dist, alpha=alpha,
                                     leg_closest=closest_id,
                                     data_dict=self.data_dict)  # Get the active and inactive robot selection
        for i in range(0, self.num_of_robots):
            if result[i] == 1:
                self.state[i] = 1  # Move forward
            elif result[i] == -1:
                self.state[i] = -1  # Move backward
            else:
                self.state[i] = 0  # Stop
            # for i in range(self.num_of_robots):
            if self.dist[i] < self.stop_dist[i]:  # Stop if you get close to the target
                self.state[i] = 0  # Stop
                self.stop_dist[i] = self.stop_dist_default + 0.05  # Ignore small oscillations of the robot
            else:
                self.stop_dist[i] = self.stop_dist_default  # Reset stop distance (if changed before) to default value

        return self.state

    def get_mean_polar_angle(self, data_dict=None):
        """ Method to get the alpha angle of the whole swarm """
        xm = 0.0
        ym = 0.0
        for i in range(self.num_of_robots):
            xm += data_dict[str(i)][0]
            ym += data_dict[str(i)][1]

        dx = self.goal.GetAbsFrame().GetPos().x - xm / self.num_of_robots
        dy = self.goal.GetAbsFrame().GetPos().z - ym / self.num_of_robots

        return 180. * np.arctan2(dy, dx) / np.pi

    def get_polar_angle(self, data_dict=None):
        """ Method to get the alpha angle of the whole swarm """
        angle = np.zeros(self.num_of_robots)
        for i in range(self.num_of_robots):
            dx = self.goal.GetAbsFrame().GetPos().x - data_dict[str(i)][0]
            dy = self.goal.GetAbsFrame().GetPos().z - data_dict[str(i)][1]
            angle[i] = 180. * np.arctan2(dy, dx) / np.pi

        return angle

    def get_dist(self, data_dict=None):
        """ Method to get the alpha angle of the whole swarm """
        dist = np.zeros(self.num_of_robots)
        for i in range(self.num_of_robots):
            dx = self.goal.GetAbsFrame().GetPos().x - data_dict[str(i)][0]
            dy = self.goal.GetAbsFrame().GetPos().z - data_dict[str(i)][1]
            dist[i] = np.sqrt(dx ** 2 + dy ** 2)

        return dist

    def get_min_dist_id(self, dist=np.ones(12)):
        """ Method to get the closest legged and vacuum robot """

        return np.argmin(dist)

    def get_stopped_closest(self, robot_id=0, data_dict=None):
        """ Method to get the id of the closest stopped robot"""
        closest = 10000000
        closest_id = 0
        for (ids, isStopped) in enumerate(self.stop_dist == self.stop_dist_default + 0.05):
            if isStopped:
                dx = data_dict[str(ids)][0] - data_dict[str(robot_id)][0]
                dy = data_dict[str(ids)][1] - data_dict[str(robot_id)][1]
                dist = np.sqrt(dx ** 2 + dy ** 2)
                if dist < closest:
                    closest = dist
                    closest_id = ids

        return closest_id

    def get_first_row(self, leg_closest, data_dict):
        i1 = leg_closest - 5
        i2 = leg_closest + 6
        a = np.multiply(1 * np.multiply((np.arange(i1, i2) >= 0), (np.arange(i1, i2) <= 11)),
                        np.arange(i1, i2)) + (
                    12 * (np.arange(i1, i2) < 0) + np.multiply((np.arange(i1, i2) < 0),
                                                               np.arange(i1, i2))) + (
                    np.multiply(1 * (np.arange(i1, i2) > 11), np.arange(i1, i2)) - 12 * (
                    np.arange(i1, i2) > 11))
        dev = np.zeros(5)
        dev_sum = np.zeros(6)
        for i in range(0, 6):
            row = a[range(0 + i, 6 + i)]
            for j in range(0, 5):
                dev[j] = (data_dict[str(row[j])][2] - data_dict[str(row[j] + 1 * (row[j] != 11) - 11 * (row[j] == 11))][
                    2]) ** 2
            dev_sum[i] = sum(dev) ** 0.5
        idd = np.nonzero(dev_sum == min(dev_sum))[0].astype(int).item()
        first_row = a[np.arange(0 + idd, 6 + idd)]

        return first_row

    def form_circle(self, circle_init):

        dx = np.array(self.qx) - np.mean(self.qx)
        dy = np.array(self.qz) - np.mean(self.qz)
        d = np.sqrt(dx ** 2 + dy ** 2)
        if np.sqrt((self.xo - self.goal.GetAbsFrame().GetPos().x) ** 2 + (
                self.yo - self.goal.GetAbsFrame().GetPos().z) ** 2) > 0.1 and circle_init and (d < 0.25).any():
            # If the object is moved form a circle shape to initialize the alignment
            result = True
        else:  # Reset the object position
            self.xo = self.goal.GetAbsFrame().GetPos().x
            self.yo = self.goal.GetAbsFrame().GetPos().z
            result = False

        return result

    def get_opt_result(self, method, theta, angle, dist, alpha, leg_closest, data_dict):

        result = np.zeros(self.num_of_robots)
        if method == 'comparison':
            for i in range(0, self.num_of_robots):
                if np.cos(self.t_a * np.pi / 180) <= self.cosTheta[i] <= 1:
                    result[i] = 1
                elif -1 <= self.cosTheta[i] <= -np.cos(self.t_a * np.pi / 180):
                    result[i] = -1
                else:
                    result[i] = 0

        elif method == 'simplex':
            res = opt.linprog(1 - 2 * self.cosTheta ** 2, method='simplex', bounds=[(0, 1)] * self.num_of_robots)
            for i in range(0, self.num_of_robots):
                if self.at_least_one_stopped:
                    if np.cos(theta[i]) < 0:
                        result[i] = -1
                    elif np.cos(theta[i]) > 0:
                        result[i] = 1
                    else:
                        result[i] = 0
                else:
                    result[i] = -res.x[i] * (self.cosTheta[i] < 0) + res.x[i] * (self.cosTheta[i] > 0)

        elif method == 'local_alignment':

            if self.form_circle(circle_init=self.circle_initialization):
                result = 1 * np.ones(self.num_of_robots)
            else:
                self.first_row = self.get_first_row(leg_closest, data_dict)
                leg_closest_opposite = (leg_closest + 6) * (leg_closest < 6) + (
                        leg_closest - 6) * (leg_closest > 5)
                b = np.arange(0, 12)
                self.back_row = b[np.isin(np.arange(0, 12), self.first_row, invert=True)]
                self.first_row = self.first_row.astype(int)
                self.back_row = self.back_row.astype(int)
                cond1 = np.zeros(self.num_of_robots)
                cond2 = np.zeros(self.num_of_robots)
                m = -1 / np.tan(np.pi * angle[leg_closest] / 180)
                i3 = np.arange(0, len(self.first_row))[(self.first_row == leg_closest)].item()
                for i in range(0, i3):
                    yd = (data_dict[str(int(self.first_row[i]))][1] - data_dict[str(int(leg_closest))][1])
                    xd = (data_dict[str(int(self.first_row[i]))][0] - data_dict[str(int(leg_closest))][0])
                    cond1[self.first_row[i]] = (abs(yd - m * xd) > 0.05)
                    cond2[self.first_row[i]] = dist[self.first_row[i]] > dist[self.first_row[i + 1]]
                    if cond1[self.first_row[i]] * cond2[self.first_row[i]]:
                        result[self.first_row[i]] = 1
                    elif cond1[self.first_row[i]] * (not cond2[self.first_row[i]]):
                        result[self.first_row[i]] = 0
                for i in range(0, len(self.first_row) - i3):
                    yd = (data_dict[str(self.first_row[5 - i])][1] - data_dict[str(leg_closest)][1])
                    xd = (data_dict[str(self.first_row[5 - i])][0] - data_dict[str(leg_closest)][0])
                    cond1[self.first_row[5 - i]] = (abs(yd - m * xd) > 0.05)
                    cond2[self.first_row[5 - i]] = dist[self.first_row[5 - i]] > dist[self.first_row[5 - (i + 1)]]
                    if cond1[self.first_row[5 - i]] and cond2[self.first_row[5 - i]]:
                        result[self.first_row[5 - i]] = 1
                    elif cond1[self.first_row[5 - i]] and (not cond2[self.first_row[5 - i]]):
                        result[self.first_row[5 - (i + 1)]] = 0
                for i in self.back_row:
                    d = np.zeros(len(self.back_row))
                    for j in range(0, len(self.first_row)):
                        dx = data_dict[str(i)][0] - data_dict[str(self.first_row[j])][0]
                        dy = data_dict[str(i)][1] - data_dict[str(self.first_row[j])][1]
                        d[j] = np.sqrt(dx ** 2 + dy ** 2)
                    if (d < 0.1).any():
                        result[i] = 0
                    else:
                        result[i] = -1

        elif method == 'engulfing':
            # self.first_row = self.get_first_row(leg_closest, data_dict)
            # b = np.arange(0, 12)
            # self.back_row = b[np.isin(np.arange(0, 12), self.first_row, invert=True)]
            # self.first_row = self.first_row.astype(int)
            # self.back_row = self.back_row.astype(int)
            for i in self.first_row:
                result[i] = 1
            for i in self.back_row:
                result[i] = -1
            # for i in range(0, self.num_of_robots):
            #     # print('engulfingMode:', self.at_least_one_stopped)
            #     if np.cos(theta[i]) < 0:
            #         result[i] = 1
            #     elif np.cos(theta[i]) > 0:
            #         result[i] = -1
            #     else:
            #         result[i] = 0

        elif method == "gradient_based":
            if self.form_circle(circle_init=self.circle_initialization):
                result = 1 * np.ones(self.num_of_robots)
            else:
                n_data_shape = 10000
                dx = self.goal.GetAbsFrame().GetPos().x - data_dict[str(leg_closest)][0]
                dy = self.goal.GetAbsFrame().GetPos().z - data_dict[str(leg_closest)][1]
                calpha = dy / (dx ** 2 + dy ** 2) ** 0.5
                salpha = -dx / (dx ** 2 + dy ** 2) ** 0.5
                shape_wid = 20
                epsilon = 0.05
                stop_dist = 0.05

                # Relative position toward the center of mass
                rel_pos = np.empty((n_data_shape, 2))

                # for line
                rel_pos[:, 0] = np.linspace(-shape_wid / 2, shape_wid / 2, n_data_shape) * calpha
                rel_pos[:, 1] = np.linspace(-shape_wid / 2, shape_wid / 2, n_data_shape) * salpha

                # calculate CM
                pos = np.empty((self.num_of_robots, 2))
                thetaL = np.empty(self.num_of_robots)
                for i in range(self.num_of_robots):
                    pos[i, 0] = data_dict[str(i)][0]
                    pos[i, 1] = data_dict[str(i)][1]
                    thetaL[i] = data_dict[str(i)][2]
                cm = np.array([data_dict[str(leg_closest)][0], data_dict[str(leg_closest)][1]]) + 0.2 / (
                        dx ** 2 + dy ** 2) * np.array([dx, dy])
                # cm = np.array([self.goal.GetAbsFrame().GetPos().x, self.goal.GetAbsFrame().GetPos().z])
                # moving vector
                move_vec = np.empty((self.num_of_robots, 2))
                move_vec[:, 0] = np.cos(thetaL) * epsilon
                move_vec[:, 1] = np.sin(thetaL) * epsilon

                # Decide whether to move backward or forward
                target = rel_pos + cm
                # moving forward and backward
                pos_forward = pos + move_vec
                pos_backward = pos - move_vec
                # Calculate shortest distance toward target
                dist = np.min(np.linalg.norm(pos[:, np.newaxis, :] - target, axis=2), axis=1)
                dist_forward = np.min(np.linalg.norm(pos_forward[:, np.newaxis, :] - target, axis=2), axis=1)
                dist_backward = np.min(np.linalg.norm(pos_backward[:, np.newaxis, :] - target, axis=2), axis=1)
                dist_diff = dist_forward - dist_backward

                for i in range(self.num_of_robots):
                    if dist[i] < stop_dist:
                        if abs(dist_diff[i]) < 2 * epsilon * calpha:
                            result[i] = -1
                        else:
                            result[i] = 0
                    # move forward
                    elif dist_diff[i] < 0:
                        result[i] = -1
                    # move backward
                    else:
                        result[i] = 1

        elif method == "stear":

            if self.form_circle(circle_init=self.circle_initialization):
                result = 1 * np.ones(self.num_of_robots)
            else:
                self.first_row = self.get_first_row(leg_closest, data_dict).astype(int)
                self.back_row = np.arange(0, 12)[np.isin(np.arange(0, 12), self.first_row, invert=True)].astype(int)
                cond1 = np.zeros(self.num_of_robots)
                cond2 = np.zeros(self.num_of_robots)
                m = -1 / np.tan(np.pi * angle[leg_closest] / 180)
                for i in range(0, 2):
                    yd = (data_dict[str(int(self.first_row[i]))][1] - data_dict[str(int(self.first_row[2]))][1])
                    xd = (data_dict[str(int(self.first_row[i]))][0] - data_dict[str(int(self.first_row[2]))][0])
                    cond1[self.first_row[i]] = (abs(yd - m * xd) > 0)
                    cond2[self.first_row[i]] = dist[self.first_row[i]] > dist[self.first_row[2]]
                    if cond1[self.first_row[i]] * cond2[self.first_row[i]]:
                        result[self.first_row[i]] = 1
                    elif cond1[self.first_row[i]] * (not cond2[self.first_row[i]]):
                        result[self.first_row[i]] = -1
                for i in range(0, 3):
                    yd = (data_dict[str(self.first_row[5 - i])][1] - data_dict[str(self.first_row[2])][1])
                    xd = (data_dict[str(self.first_row[5 - i])][0] - data_dict[str(self.first_row[2])][0])
                    cond1[self.first_row[5 - i]] = (abs(yd - m * xd) > 0)
                    cond2[self.first_row[5 - i]] = dist[self.first_row[5 - i]] > dist[self.first_row[2]]
                    if cond1[self.first_row[5 - i]] and cond2[self.first_row[5 - i]]:
                        result[self.first_row[5 - i]] = 1
                    elif cond1[self.first_row[5 - i]] and (not cond2[self.first_row[5 - i]]):
                        result[self.first_row[5 - (i + 1)]] = -1
                for i in self.back_row:
                    d = np.zeros(len(self.back_row))
                    for j in range(0, len(self.first_row)):
                        dx = data_dict[str(i)][0] - data_dict[str(self.first_row[j])][0]
                        dy = data_dict[str(i)][1] - data_dict[str(self.first_row[j])][1]
                        d[j] = np.sqrt(dx ** 2 + dy ** 2)
                    result[i] = -result[self.first_row[d == np.min(d)]]

        return result

    def get_data(self):

        return self.data_dict

    # Plot data
    def plot_robot(self):

        plt.plot(self.qx, self.qz, 'b', self.goal.x, self.goal.z, 'gs')
