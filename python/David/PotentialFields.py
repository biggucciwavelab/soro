import numpy as np
from numpy import ones, zeros
import matplotlib.pyplot as plt
from PIL import Image, ImageOps
import pygame
import cv2

class PotField:

    field_operators = []
    field_grad_operators = []

    def __init__(self, field_operators, field_grad_operators):
        if type(field_operators) == list:
            self.field_operators = field_operators
        else:
            self.field_operators = [field_operators]

        if type(field_grad_operators) == list:
            self.field_grad_operators = field_grad_operators
        else:
            self.field_grad_operators = [field_grad_operators]

    def __add__(self, other):
        field_operators = self.field_operators + other.field_operators
        field_grad_operators = self.field_grad_operators + other.field_grad_operators
        return PotField(field_operators, field_grad_operators)

    def __sub__(self, other):
        field_operators = self.field_operators + [lambda x: -op(x) for op in other.field_operators]
        field_grad_operators = self.field_grad_operators + [lambda x: -op(x) for op in other.field_grad_operators]
        return PotField(field_operators, field_grad_operators)

    def field(self, x):
        output = np.zeros((1, x.shape[0]), dtype=float)
        if len(self.field_operators) != 0:
            for op in self.field_operators:
                output = output + op(x)
        return output

    def field_grad(self, x):
        output = np.zeros(x.shape)
        if len(self.field_grad_operators) != 0:
            for op in self.field_grad_operators:
                output = output + op(x)
        return output

    def set_field_parameters(self, parameters):
        pass

    def plot_field(self):
        res = 100

        center = np.array([4., 3.])
        exteremes = 2*center

        x = np.linspace(0, exteremes[0], res)
        y = np.linspace(0, exteremes[1], res)
        X, Y = np.meshgrid(x, y)
        data = np.vstack([X.flatten(), Y.flatten()]).T
        Z = self.field(data).reshape((res, res))

        fig = plt.figure('Field!')
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X, Y, Z, cmap='RdGy')

    def getImage(self, size, res):
        r = 100
        res_scale = [int(res[0]/res[1]*r), int(r)]

        x = np.linspace(0, size[0], res_scale[0])
        y = np.linspace(0, size[1], res_scale[1])
        X, Y = np.meshgrid(x, y)
        data = np.vstack([X.flatten(), Y.flatten()]).T
        Z = self.field(data).reshape([res_scale[1], res_scale[0]])
        Z = Z * 2

        cm = plt.get_cmap('Blues_r')
        img = cm(Z)

        img = Image.fromarray((img[:, :, :3] * 255).astype(np.uint8))
        img = img.resize(np.array(res, dtype=int))
        img = ImageOps.flip(img)

        #return img

        return pygame.image.fromstring(img.tobytes(), img.size, img.mode)

#class

class RadialOperator:

    def __init__(self, func=None):
        if func == None:
            self.operators = [RadialFunction()]
        elif type(func) == list:
            self.operators = func
        else:
            self.operators = [func]
        self.offsets = self.get_offsets()
        self.transforms = self.get_transforms()
        self.multipliers = self.get_multipliers()

    def get_offsets(self):
        return [op.offset for op in self.operators]

    def get_transforms(self):
        return [op.transform for op in self.operators]

    def get_multipliers(self):
        return [op.multiplier for op in self.operators]

    def get_types(self):
        return [op.type for op in self.operators]

    def get_field_and_grad(self, x):
        output_field = np.zeros(x.shape[1])
        output_grad = np.zeros(x.shape)

        for op in self.operators:
            tmpf, tmpg = op.get_field_and_grad(x)
            output_field += tmpf
            output_grad += tmpg

        return output_field, output_grad

    def get_field(self, x):
        output_field = np.zeros(x.shape[1])

        for op in self.operators:
            tmpf = op.get_field(x)
            output_field += tmpf

        return output_field

    def __add__(self, other):
        if len(self.operators) == 1 and self.operators[0].type == 'constant':
            self.operators = other.operators
        else:
            self.operators += other.operators
        self.offsets = self.get_offsets()
        self.transforms = self.get_transforms()
        self.multipliers = self.get_multipliers()

        return self

    def __sub__(self, other):
        tmp = other.operators
        for i in range(len(tmp)):
            tmp[i].multiplier *= -1

        ro = RadialOperator(tmp)

        return self.__add__(ro)

    def __mul__(self, other):
        pass

    def getImage(self, size, res):
        r = 60
        res_scale = [int(res[0] / res[1] * r), int(r)]

        x = np.linspace(0, size[0], res_scale[0])
        y = np.linspace(0, size[1], res_scale[1])
        X, Y = np.meshgrid(x, y)
        data = np.vstack([X.flatten(), Y.flatten()])
        Z = self.get_field(data).reshape([res_scale[1], res_scale[0]])
        Z = Z * 2

        Z[np.isnan(Z)]=0

        #Z = (Z - np.min(Z)) / (np.max(Z) - np.min(Z))*50
        Z *=.001

        cm = plt.get_cmap('Blues_r')
        img = cm(Z)

        img = Image.fromarray((img[:, :, :3] * 255).astype(np.uint8))
        img = img.resize(np.array(res, dtype=int), cv2.INTER_CUBIC)
        img = ImageOps.flip(img)


        # ratio = res[1] / size[1]
        #
        # center = self.operators[-1].offset * ratio
        # axes = (100, 200)
        # angle = np.arctan2(self.operators[-1].transform[0, 0], self.operators[-1].transform[0, 1])*360/np.pi
        #
        # img = (np.ones((int(res[1]), int(res[0]), 3))*230).astype(np.uint8)
        #
        # llipse_float = (center, axes, angle)
        # img = Image.fromarray(cv2.ellipse(img, llipse_float, (200, 200, 255), -1))

        return pygame.image.fromstring(img.tobytes(), img.size, img.mode)


# By default is a constant function
class RadialFunction:

    # Uses radial functions with radius defined as: func[(x - x0).T @ M @ (x - x0)]
    # Gradient defined as: func'[r] * 2 * M @ (x - x0)

    def __init__(self, offset=np.zeros((2, 1)), transform=np.identity(2), multiplier=1.):
        self.offset = np.reshape(offset, (2, 1))
        self.transform = transform
        self.multiplier = multiplier
        self.type = 'constant'

    # This changes with each definintion
    def radialf_and_grad(self, r):
        r_func = np.ones(r.shape[0])
        r_grad = np.zeros(r.shape)
        return r_func * self.multiplier, r_grad * self.multiplier

    # This doesn't change with each definition
    def radius_and_grad(self, x):
        diff = x - self.offset
        grad = self.transform @ diff
        radius = np.sum(diff * grad, axis=0)
        return radius, 2 * grad

    def get_field(self, x):
        r, _ = self.radius_and_grad(x)
        field, _ = self.radialf_and_grad(r)
        return field

    def get_field_and_grad(self, x):
        r, rd = self.radius_and_grad(x)
        f, fd = self.radialf_and_grad(r)

        return f, fd * rd

class EllipseFunction(RadialFunction):

    def __init__(self, offset=np.zeros((2, 1)), transform=np.identity(2), multiplier=1.):
        super().__init__(offset, transform, multiplier)
        self.type = 'ellipse'

    def radialf_and_grad(self, r):
        r_func = r
        r_grad = np.ones(r.shape)
        return r_func * self.multiplier, r_grad * self.multiplier

class SinkFunction(RadialFunction):

    def __init__(self, offset=np.zeros((2, 1)), transform=np.identity(2), multiplier=1.):
        super().__init__(offset, transform, multiplier)
        self.type = 'sink'

    def radialf_and_grad(self, r):
        r_func = - np.log(r) / (2 * np.pi)
        r_grad = - 1 / (2 * np.pi * r)
        return r_func * self.multiplier, r_grad * self.multiplier

class WideSinkFunction(RadialFunction):
    def __init__(self, offset=np.zeros((2, 1)), transform=np.identity(2), multiplier=1., width=0.):
        super().__init__(offset, transform, multiplier)
        self.width = width
        self.type = 'wsink'

    def radialf_and_grad(self, r):
        r_func = - np.log(r) / (2 * np.pi)
        #r_grad = - 1 / (2 * np.pi * r)
        r_func = - np.log(r) / (2 * np.pi)*(r > self.width) - np.log(self.width) / (2 * np.pi)*(r <= self.width)
        r_grad = - 1 / (2 * np.pi * r)*(r > self.width) - 1 / (2 * np.pi * self.width)*(r <= self.width)
        tmp = 1
        return r_func * self.multiplier * tmp, r_grad * self.multiplier * tmp

class WideEllipseFunction(RadialFunction):

    def __init__(self, offset=np.zeros((2, 1)), transform=np.identity(2), multiplier=1., width=0.):
        super().__init__(offset, transform, multiplier)
        self.width = width
        self.type = 'wellipse'

    def radialf_and_grad(self, r):
        r_func = r - self.width
        r_grad = np.ones(r.shape)
        return r_func * self.multiplier, r_grad * self.multiplier

class LinearFunction(RadialFunction):

    def __init__(self, offset=np.zeros((2, 1)), transform=np.identity(2), multiplier=1.):
        super().__init__(offset, transform, multiplier)
        self.type = 'linear'

    def radialf_and_grad(self, r):
        r_func = np.sqrt(r)
        r_grad = - 0.5 / r_func
        return r_func * self.multiplier, r_grad * self.multiplier

def get_ellipse_field_opers(parameters):
    necessary_params = ['axes', 'rotation', 'scale', 'center']
    if all(elem in list(parameters.keys()) for elem in necessary_params):
        M = parameters['axes']
        Q = parameters['rotation']
        k = parameters['scale']

        S_mat = (Q @ M @ Q.T) * k
        offset = np.reshape(parameters['center'], (2, 1))

        def field(x):
            return np.sum((x.T - offset) * (S_mat @ (x.T - offset)), axis=0)

        #field = lambda x: np.diagonal((x.T - offset).T @ S_mat @ (x.T - offset))
        grad_field = lambda x: 0.5 * S_mat @ (x - offset)

        return field, grad_field

    else:
        raise Exception("Not all the needed parameters are found for an Elliptic Pot Field")

def get_sink_field_opers(parameters):
    necessary_params = ['axes', 'rotation', 'scale', 'center']
    if all(elem in list(parameters.keys()) for elem in necessary_params):
        m = parameters['scale']
        offset = np.reshape(parameters['center'], (2, 1))

        M = parameters['axes']
        Q = parameters['rotation']
        k = parameters['scale']

        S_mat = (Q @ M @ Q.T) * k

        field = lambda x: m / np.pi * np.log(np.sum((S_mat @ (x.T - offset))**2, axis=0))
        grad_field = lambda x: 2 * m / np.pi * (S_mat @ (x - offset)) / np.sum((S_mat @ (x - offset))**2, axis=0)

        return field, grad_field

    else:
        raise Exception("Not all the needed parameters are found for an Elliptic Pot Field")