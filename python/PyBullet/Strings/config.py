# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 11:12:20 2021

@author: dmulr
"""

import os
os.chdir(os.path.dirname(__file__)+"\\")
from get_user import *
import pathlib
from shutil import copyfile
import csv
import time
import numpy as np
from datetime import datetime
from objects import *
from QuaternionRotation import create_from_axis_angle, multQ

#### SIMULATION MODES ####
dimension = '2D' #2D: 2D sim   3D: 3D sim
dt = 1/240 # time step 
visual = True

if dimension =='2D':
    #### ROBOT PROPERTIES #### 
    '''
    cm: convert_dist = 100
    meters: convert_dist = 1
    
    kg: convert mass = 1
    grams: convert mass = 1000
    
    '''
    ##### if cm convert_dist = 100 if meter convert_dist=1 ####
    convert_dist = 100 # if its meters or cm
    convert_mass = 1000 # if its grams or kg
    nb = 30 # number of bots
    bot_mass = .300  # mass of bot kg 
    bot_geom = 'cylinder'

    bot_width = .07  #  bot width  [m]
    bot_height = .07/2  # bot height    [m]
    cloth_height=bot_height # cloth height [m]
    cloth_width=.1 # cloth width [m]
    theta = 2*np.pi/nb
    # System parameters
    cord_length = cloth_width + bot_width*np.cos(theta/2) # Cord length between bot centers
    R = np.sqrt((cord_length**2)/(2*(1-np.cos(theta)))) # radius [m]
    
    #### INTERIOR PROPERTIES ####
    particle_mass = .1 # kg 
    particle_width = .07 # meters
    particle_height = .07/2 # meters
    
    particle_geom = 'cylinder'
    particle_mode = 'monodispersion' # interior particle mode
    
    
    #### ENVIROMENT PARAMETERS ####
    lateralFriction=0.3
    spinningFriction = 0.01
    rollingFriction = 0.001
    
    
    #### SAVE SIMULATION ####
    now = datetime.now()
    dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
    mainDirectory = main_directory_dict[user]
    #savefile = mainDirectory +'Experiments/'+ 'sim number '+'___('+str(file_count)+')___'+dt_string
    savefile = mainDirectory + dt_string
    os.makedirs(savefile, exist_ok=True)
    csvFile = savefile+'/Parameters.csv'
    
    envParams = dict(
        dimension = dimension,
        dt = dt,
        convert_dist = convert_dist,
        convert_mass = convert_mass,
        visual = visual,

        # Robot parameters
        nb = nb,
        bot_mass = bot_mass,
        bot_geom = bot_geom,
        bot_width = bot_width,
        bot_height = bot_height,
        cloth_width = cloth_width,
        cloth_height = cloth_height,
        cord_length = cord_length,
        R = R,

        # Particle Parameters
        particle_mode = particle_mode,
        particle_mass = particle_mass,
        particle_width = particle_width,
        particle_height = particle_height,
        particle_geom = particle_geom,

        lateralFriction = lateralFriction,
        spinningFriction = spinningFriction,
        rollingFriction = rollingFriction
    )
    
    # Saving the dictionary as a CSV and .npy file
    with open(csvFile, 'w') as fout:
        for key in envParams.keys():
            fout.write("%s,%s\n"%(key,envParams[key]))
    
    np.save(savefile+'/Parameters.npy',envParams)



# Can load parameters using the following method
#data=np.load(savefile+'/Parameters.npy',allow_pickle=True)
#data=data.tolist()