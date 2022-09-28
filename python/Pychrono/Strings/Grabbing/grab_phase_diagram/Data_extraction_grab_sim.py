# -*- coding: utf-8 -*-
"""
Created on Thu Jun  4 10:32:32 2020

@author: dmulr
"""
import numpy as np
import timeit
from os import listdir
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import colors as colors
import animatplot as amp
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator


# Set the file path
filepath = "F:/Grabbing3"

# Extract every files from the path
files = listdir(filepath)

for file in files:
    # Get the filename
    filename = filepath + file
