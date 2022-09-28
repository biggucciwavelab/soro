# -*- coding: utf-8 -*-
"""
Created on Sat Feb 15 20:36:12 2020

@author: dmulr
"""
# In[Import Libraries]
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from matplotlib import colors as colors
import matplotlib as mpl
import matplotlib.cm as cm
from bridson import poisson_disc_samples
import matplotlib.lines as mlines
import statistics 
# In[Voroni Function]
def voronoi_finite_polygons_2d(vor, radius=None):
    """
    Reconstruct infinite voronoi regions in a 2D diagram to finite
    regions.

    Parameters
    ----------
    vor : Voronoi
        Input diagram
        
    radius : float, optional
        Distance to 'points at infinity'.

    Returns
    -------
    regions : list of tuples
        Indices of vertices in each revised Voronoi regions.
        
    vertices : list of tuples
        Coordinates for revised Voronoi vertices. Same as coordinates
        of input vertices, with 'points at infinity' appended to the
        end.

    """

    if vor.points.shape[1] != 2:
        raise ValueError("Requires 2D input")
# new region 
    new_regions = []
# new vertices
    new_vertices = vor.vertices.tolist()
# area 
    new_area=[]
    # calculate center
    center = vor.points.mean(axis=0)
    if radius is None:
        radius = vor.points.ptp().max()*2

    # Construct a map containing all ridges for a given point
    all_ridges = {}
    
    for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
        all_ridges.setdefault(p1, []).append((p2, v1, v2))
        all_ridges.setdefault(p2, []).append((p1, v1, v2))

    # Reconstruct infinite regions
    for p1, region in enumerate(vor.point_region):
        vertices = vor.regions[region]

        if all(v >= 0 for v in vertices):
            # finite region
            new_regions.append(vertices)
            continue

        # reconstruct a non-finite region
        ridges = all_ridges[p1]
        new_region = [v for v in vertices if v >= 0]

        for p2, v1, v2 in ridges:
            if v2 < 0:
                v1, v2 = v2, v1
            if v1 >= 0:
                # finite ridge: already in the region
                continue

            # Compute the missing endpoint of an infinite ridge
            t = vor.points[p2] - vor.points[p1] # tangent
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])  # normal

            midpoint = vor.points[[p1, p2]].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, n)) * n
            far_point = vor.vertices[v2] + direction * radius

            new_region.append(len(new_vertices))
            new_vertices.append(far_point.tolist())

        # sort region counterclockwise
        vs = np.asarray([new_vertices[v] for v in new_region])
        c = vs.mean(axis=0)
        angles = np.arctan2(vs[:,1] - c[1], vs[:,0] - c[0])
        new_region = np.array(new_region)[np.argsort(angles)]

        # finish
        new_regions.append(new_region.tolist())

    return new_regions, np.asarray(new_vertices)


# In[Voronoi Area]
def voronoi_Area(region,vertices):
    Area=ConvexHull(vertices[region]).volume
    return(Area)



data=np.load('100.npz',allow_pickle=True)

# Positions
qx=data['qx']
qy=data['qy']
qz=data['qz']
nb=data['nb']
ni=data['ni']
diameter=data['diameter']

P=[]
for i in range(nb+ni):
    P.append(np.array([X[i],Y[i]]))
P=np.asarray(P)

X=[]
Y=[]
for i in range(nb+ni):
    X.append(qx[i,-1])
    Y.append(qz[i,-1])
X=np.asarray(X)
Y=np.asarray(Y)