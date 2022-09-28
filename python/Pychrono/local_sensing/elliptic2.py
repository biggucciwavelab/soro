#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A Python implementation of the method described in [#a]_ and [#b]_ for
calculating Fourier coefficients for characterizing
closed contours.
References
----------
.. [#a] F. P. Kuhl and C. R. Giardina, “Elliptic Fourier Features of a
   Closed Contour," Computer Vision, Graphics and Image Processing,
   Vol. 18, pp. 236-258, 1982.
.. [#b] Oivind Due Trier, Anil K. Jain and Torfinn Taxt, “Feature Extraction
   Methods for Character Recognition - A Survey”, Pattern Recognition
   Vol. 29, No.4, pp. 641-662, 1996
Created by hbldh <henrik.blidh@nedomkull.com> on 2016-01-30.
"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import


import numpy as np

try:
    _range = xrange
except NameError:
    _range = range

def clip( x, mini, maxi) :
    if( mini > maxi ) :  return x    
    elif( x < mini ) :  return mini
    elif( x > maxi ) :  return maxi
    else :             return x
    
def generatePolygon( ctrX, ctrY, aveRadius, irregularity, spikeyness, numVerts) :
    # Generate a random polygon
    '''
    Modified from: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon
    Start with the centre of the polygon at ctrX, ctrY, 
    then creates the polygon by sampling points on a circle around the centre. 
    Randon noise is added by varying the angular spacing between sequential points,
    and by varying the radial distance of each point from the centre.
    
    Params:
    ctrX, ctrY - coordinates of the "centre" of the polygon
    aveRadius - in px, the average radius of this polygon, this roughly controls how large the polygon is, really only useful for order of magnitude.
    irregularity - [0,1] indicating how much variance there is in the angular spacing of vertices. [0,1] will map to [0, 2pi/numberOfVerts]
    spikeyness - [0,1] indicating how much variance there is in each vertex from the circle of radius aveRadius. [0,1] will map to [0, aveRadius]
    numVerts - self-explanatory
    
    Returns a list of vertices, in CCW order.
       '''
    irregularity = clip( irregularity, 0,1 ) * 2.*np.pi / numVerts
    spikeyness = clip( spikeyness, 0,1 ) * aveRadius
    
    # generate n angle steps
    angleSteps = []
    lower = (2.*np.pi / numVerts) - irregularity
    upper = (2.*np.pi / numVerts) + irregularity
    sum = 0
    for i in range(numVerts) :
        tmp = np.random.uniform(lower, upper)
        angleSteps.append( tmp )
        sum = sum + tmp
       
    # normalize the steps so that point 0 and point n+1 are the same
    k = sum / (2.*np.pi)
    for i in range(numVerts) :
        angleSteps[i] = angleSteps[i] / k
    
    # now generate the points
    xs=[]
    ys=[]
    
    angle = np.random.uniform(0, 2.*np.pi)
    for i in range(numVerts) :
        r_i = clip( np.random.normal(aveRadius, spikeyness), 0, 2.*aveRadius )
        x = ctrX + r_i*np.cos(angle)
        y = ctrY + r_i*np.sin(angle)
        angle = angle + angleSteps[i]
        xs.append(x)
        ys.append(y)
        
    return np.asarray(xs),np.asarray(ys)

def elliptic_fourier_descriptors(contour, order=10, normalize=False, return_transformation=False):
    """Calculate elliptical Fourier descriptors for a contour.
    :param numpy.ndarray contour: A contour array of size ``[M x 2]``.
    :param int order: The order of Fourier coefficients to calculate.
    :param bool normalize: If the coefficients should be normalized;
        see references for details.
    :param bool return_transformation: If the normalization parametres should be returned.
        Default is ``False``.
    :return: A ``[order x 4]`` array of Fourier coefficients and optionally the
        transformation parametres ``scale``, ``psi_1`` (rotation) and ``theta_1`` (phase)
    :rtype: ::py:class:`numpy.ndarray` or (:py:class:`numpy.ndarray`, (float, float, float))
    """
    dxy = np.diff(contour, axis=0)
    dt = np.sqrt((dxy ** 2).sum(axis=1))
    t = np.concatenate([([0.0]), np.cumsum(dt)])
    T = t[-1] #

    phi = (2. * np.pi * t) / T

    orders = np.arange(1, order + 1)
    consts = T / (2. * orders * orders * np.pi * np.pi)
    phi = phi * orders.reshape((order, -1))
    d_cos_phi = np.cos(phi[:, 1:]) - np.cos(phi[:, :-1])
    d_sin_phi = np.sin(phi[:, 1:]) - np.sin(phi[:, :-1])
    cos_phi = (dxy[:, 0] / dt) * d_cos_phi
    a = consts * np.sum(cos_phi, axis=1)
    b = consts * np.sum((dxy[:, 0] / dt) * d_sin_phi, axis=1)
    c = consts * np.sum((dxy[:, 1] / dt) * d_cos_phi, axis=1)
    d = consts * np.sum((dxy[:, 1] / dt) * d_sin_phi, axis=1)

    coeffs = np.concatenate(
        [a.reshape((order, 1)),
         b.reshape((order, 1)),
         c.reshape((order, 1)),
         d.reshape((order, 1)),], axis=1)

    if normalize:
        coeffs = normalize_efd(coeffs, return_transformation=return_transformation)

    return coeffs


def normalize_efd(coeffs, size_invariant=True, return_transformation=False):
    """Normalizes an array of Fourier coefficients.
    See [#a]_ and [#b]_ for details.
    :param numpy.ndarray coeffs: A ``[n x 4]`` Fourier coefficient array.
    :param bool size_invariant: If size invariance normalizing should be done as well.
        Default is ``True``.
    :param bool return_transformation: If the normalization parametres should be returned.
        Default is ``False``.
    :return: The normalized ``[n x 4]`` Fourier coefficient array and optionally the
        transformation parametres ``scale``, :math:`psi_1` (rotation) and :math:`theta_1` (phase)
    :rtype: :py:class:`numpy.ndarray` or (:py:class:`numpy.ndarray`, (float, float, float))
    """
    # Make the coefficients have a zero phase shift from
    # the first major axis. Theta_1 is that shift angle.
    theta_1 = 0.5 * np.arctan2(
        2 * ((coeffs[0, 0] * coeffs[0, 1]) + (coeffs[0, 2] * coeffs[0, 3])),
        (
            (coeffs[0, 0] ** 2)
            - (coeffs[0, 1] ** 2)
            + (coeffs[0, 2] ** 2)
            - (coeffs[0, 3] ** 2)
        ),
    )
    # Rotate all coefficients by theta_1.
    for n in _range(1, coeffs.shape[0] + 1):
        coeffs[n - 1, :] = np.dot(
            np.array([[coeffs[n - 1, 0], coeffs[n - 1, 1]],
                      [coeffs[n - 1, 2], coeffs[n - 1, 3]],]),
            np.array([[np.cos(n * theta_1), -np.sin(n * theta_1)],
                    [np.sin(n * theta_1), np.cos(n * theta_1)],]),).flatten()

    # Make the coefficients rotation invariant by rotating so that
    # the semi-major axis is parallel to the x-axis.
    psi_1 = np.arctan2(coeffs[0, 2], coeffs[0, 0])
    psi_rotation_matrix = np.array(
        [[np.cos(psi_1), np.sin(psi_1)], [-np.sin(psi_1), np.cos(psi_1)]]
    )
    # Rotate all coefficients by -psi_1.
    for n in _range(1, coeffs.shape[0] + 1):
        coeffs[n - 1, :] = psi_rotation_matrix.dot(
            np.array(
                [
                    [coeffs[n - 1, 0], coeffs[n - 1, 1]],
                    [coeffs[n - 1, 2], coeffs[n - 1, 3]],
                ]
            )
        ).flatten()

    size = coeffs[0, 0]
    if size_invariant:
        # Obtain size-invariance by normalizing.
        coeffs /= np.abs(size)

    if return_transformation:
        return coeffs, (size, psi_1, theta_1)
    else:
        return coeffs


def calculate_dc_coefficients(contour):
    """Calculate the :math:`A_0` and :math:`C_0` coefficients of the elliptic Fourier series.
    :param numpy.ndarray contour: A contour array of size ``[M x 2]``.
    :return: The :math:`A_0` and :math:`C_0` coefficients.
    :rtype: tuple
    """
    dxy = np.diff(contour, axis=0)
    dt = np.sqrt((dxy ** 2).sum(axis=1))
    t = np.concatenate([([0.0]), np.cumsum(dt)])
    T = t[-1]

    xi = np.cumsum(dxy[:, 0]) - (dxy[:, 0] / dt) * t[1:]
    A0 = (1 / T) * np.sum(((dxy[:, 0] / (2 * dt)) * np.diff(t ** 2)) + xi * dt)
    delta = np.cumsum(dxy[:, 1]) - (dxy[:, 1] / dt) * t[1:]
    C0 = (1 / T) * np.sum(((dxy[:, 1] / (2 * dt)) * np.diff(t ** 2)) + delta * dt)

    # A0 and CO relate to the first point of the contour array as origin.
    # Adding those values to the coefficients to make them relate to true origin.
    return contour[0, 0] + A0, contour[0, 1] + C0


def reconstruct_contour(coeffs, locus=(0, 0), num_points=360):
    """Returns the contour specified by the coefficients.
    :param coeffs: A ``[n x 4]`` Fourier coefficient array.
    :type coeffs: numpy.ndarray
    :param locus: The :math:`A_0` and :math:`C_0` elliptic locus in [#a]_ and [#b]_.
    :type locus: list, tuple or numpy.ndarray
    :param num_points: The number of sample points used for reconstructing the contour from the EFD.
    :type num_points: int
    :return: A list of x,y coordinates for the reconstructed contour.
    :rtype: numpy.ndarray
    """
    t = np.linspace(0, 1.0, num_points)
    # Append extra dimension to enable element-wise broadcasted multiplication
    coeffs = coeffs.reshape(coeffs.shape[0], coeffs.shape[1], 1)

    orders = coeffs.shape[0]
    orders = np.arange(1, orders + 1).reshape(-1, 1)
    order_phases = 2 * orders * np.pi * t.reshape(1, -1)

    xt_all = coeffs[:, 0] * np.cos(order_phases) + coeffs[:, 1] * np.sin(order_phases)
    yt_all = coeffs[:, 2] * np.cos(order_phases) + coeffs[:, 3] * np.sin(order_phases)

    xt_all = xt_all.sum(axis=0)
    yt_all = yt_all.sum(axis=0)
    xt_all = xt_all + np.ones((num_points,)) * locus[0]
    yt_all = yt_all + np.ones((num_points,)) * locus[1]

    reconstruction = np.stack([xt_all, yt_all], axis=1)
    return reconstruction


def plot_efd(coeffs, orig, locus=(0.0, 0.0), image=None, contour=None, n=360):
    """Plot a ``[2 x (N / 2)]`` grid of successive truncations of the series.
    .. note::
        Requires `matplotlib <http://matplotlib.org/>`_!
    :param numpy.ndarray coeffs: ``[N x 4]`` Fourier coefficient array.
    :param list, tuple or numpy.ndarray locus:
        The :math:`A_0` and :math:`C_0` elliptic locus in [#a]_ and [#b]_.
    :param int n: Number of points to use for plotting of Fourier series.
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Cannot plot: matplotlib was not installed.")
        return

    N = coeffs.shape[0]
    N_half = int(np.ceil(N / 2))
    n_rows = 2

    t = np.linspace(0, 1.0, n)
    xt = np.ones((n,)) * locus[0]
    yt = np.ones((n,)) * locus[1]
    
    orig = np.vstack((orig,orig[0,:]))
    
    for n in _range(coeffs.shape[0]):
        xt += (coeffs[n, 0] * np.cos(2 * (n + 1) * np.pi * t)) + (
            coeffs[n, 1] * np.sin(2 * (n + 1) * np.pi * t)
        )
        yt += (coeffs[n, 2] * np.cos(2 * (n + 1) * np.pi * t)) + (
            coeffs[n, 3] * np.sin(2 * (n + 1) * np.pi * t)
        )
        ax = plt.subplot2grid((n_rows, N_half), (n // N_half, n % N_half))
        ax.set_title(str(n + 1))
        if contour is not None:
            ax.plot(contour[:, 0], contour[:, 1], "c--", linewidth=2)
        ax.plot(orig[:,0], orig[:,1])
        ax.plot(xt, yt, "r", linewidth=2)
        if image is not None:
            ax.imshow(image, plt.cm.gray)

    plt.show()

#x,y = generatePolygon(0.0, 0.0, 1, 0.0, 0.0, 50)

x=np.asarray([1,-1,-1,1,1])+1
y=[1,1,-1,-1,1]

contour=np.zeros((len(x),2))
contour[:,0]=x
contour[:,1]=y

order=25

coeffs = elliptic_fourier_descriptors(contour,order)

x0,y0=calculate_dc_coefficients(contour)

reconstruction = reconstruct_contour(coeffs,locus=(x0,y0))

plot_efd(coeffs,contour,locus=(x0,y0))