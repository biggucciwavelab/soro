# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 08:52:53 2020

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
import os
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
# In[Make array for points]   
def Make_P_mat(nb,ni,qx,qz):
    X=[]
    Y=[]
    P=[]
    for i in range(nb+ni):
        X.append(qx[i,-1])
        Y.append(qz[i,-1])
    X=np.asarray(X)
    Y=np.asarray(Y)
    for i in range(nb+ni):
        P.append(np.array([X[i],Y[i]]))
    P=np.asarray(P)
    
    return (X,Y,P)

# In[Voronoi contour plot]
def Voronoi_contour(vertices,regions,A,vor,k,nb):
    
    results_dir = os.path.join('F:/Robosoft2020_data/revisiions_jamming/trial10/Contour/')
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)  
    minima = min(A)
    maxima = max(A)


    #normalize chosen colormap
    norm = mpl.colors.Normalize(vmin=minima, vmax=maxima, clip=True)
    mapper = cm.ScalarMappable(norm=norm, cmap=cm.seismic)
#plt.figure(figsize=(13,13))
#plot Voronoi diagram, and fill finite regions with color mapped from speed value
#voronoi_plot_2d(vor, show_points=True, show_vertices=False, s=1)

    plt.figure(figsize=(13,13))
#plotObj(obj)
    for r in range(len(vor.point_region)):
        region = vor.regions[vor.point_region[r]]
        if not -1 in region:
            polygon = [vor.vertices[i] for i in region]
            plt.fill(*zip(*polygon), color=mapper.to_rgba(A[r]))
    plt.axis('equal')
    plt.xlim(vor.min_bound[0] - 0.1, vor.max_bound[0] + 0.1)
    plt.ylim(vor.min_bound[1] - 0.1, vor.max_bound[1] + 0.1)

    plt.xlabel('x position (meters)')
    plt.ylabel('y position (meters)')
    plt.title(' nb= '+str(nb)+' k= '+str(k))
    plt.colorbar(mapper)
    plt.savefig(results_dir+"Packing_fraction"+str(k)+"_"+str(nb)+".png")
    plt.close('all')     
  # In[Stats]  
def Stats_packing(Abar,nb,k):
    results_dir = os.path.join('F:/Robosoft2020_data/revisiions_jamming/trial10/Distrib/')
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)  
    n, bins, patches = plt.hist(Abar, 100, facecolor='g')
    plt.xlabel('local Packing fraction')
    plt.ylabel('number of occurances')
    plt.title(' k= '+str(k)+' nb= '+str(nb))
    #plt.text(60, .025, r'$\mu=100,\ \sigma=15$')
    #plt.xlim(.7,.9)
    #plt.ylim(0, 0.03)
    plt.grid(True)
    plt.savefig(results_dir+"distribution"+str(k)+"_"+str(nb)+".png")
    plt.close('all')     
# In[Real Packing Fraction]
def Packing(A):
    A=np.asarray(A)
    Abar=A[A>.3]
    phi=statistics.mean(Abar) 
    return phi,Abar

# In[Phase diagram of phi vs boundary]
def Phase_diagram(nb,k,PHI2):
    image,ax=plt.subplots(figsize=(9,9))
    PHI_reshape=PHI2.reshape((len(nb),len(k)))
    plt.imshow(PHI_reshape,interpolation='bilinear')
    plt.title('Packing Fraction')
    plt.xlabel('String Tension [N]')
    plt.ylabel('Number of boundary robots')
    ax.set_xticks(np.arange(np.shape(PHI_reshape)[0]))
    ax.set_yticks(np.arange(np.shape(PHI_reshape)[1]))
    ax.set_yticklabels(nb)
    ax.set_xticklabels(k)
    cbar=plt.colorbar()
    cbar.set_label('Packing Fraction')
    plt.show()
    plt.savefig('Packing_Fraction.png')
     