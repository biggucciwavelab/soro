# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 14:14:49 2021

@author: dmulr
"""

import os
import pathlib
from shutil import copyfile
import csv
import time
import numpy as np
from datetime import datetime
from shutil import copyfile

#### SIMULATION MODES ####
dimension = '2D' #2D: 2D sim   3D: 3D sim
dt = 0.001 # time step 
time_end = 180
save_rate = 300 #save every n number of steps
visual = 'irr'

xcenter = 2
zcenter = 0

#### Control Modes ####
'''
Control modes:
shape formation: Control mode specific to shape formation
shape morphing linear: linear shape morphing 
shape_morphing: transfinte morphing 
'''
 
control_mode = "grasping"

#control_mode = "Verify"
#### GEOMETRIES ####
"""
circle:
square:
wrench
"""

#### CONVERSION VARIABLES ####
'''
cm: convert_dist = 100
meters: convert_dist = 1

kg: convert mass = 1
grams: convert mass = 1000

'''
convert_dist = 1 # if its meters or cm
convert_mass = 1 # if its grams or kg


#### ROBOT VARIABLES #####
nb = 30 # number of bots
bot_mass = .200  # mass of bot kg 
bot_geom = 'cylinder'
bot_width = 0.09525  #  bot width  [m]
bot_height = 0.09525/2 # bot height    [m]
membrane_type = 1
skin_width = 0.03# diameter of membrane particles
ns =6 # number of skin particles 
membrane_width = ns*skin_width # cloth width [m]
spring_stiffness = 100 # spring stiffness
spring_damping = 0 # spring damping 
theta = 2*np.pi/nb 
cord_length = membrane_width + bot_width*np.cos(theta/2) # Cord length between bot centers
R = np.sqrt((cord_length**2)/(2*(1-np.cos(theta)))) # radius [m]
membrane_density = 2000

ns=6



#### INTERIOR PROPERTIES ####
'''

"bi_dispersion_ring"
"bidispersion"
"monodispersion"
"bi_dispersion_uniform_ring"
'''



particle_mass = 0.01 # kg 
#particle_width = 0.0762 # meters
particle_width = 0.1016  # meters
particle_width2 = 0.1016 # meters
particle_height = bot_height # meters
particle_geom = 'cylinder'
interior_mode = "bidispersion" # interior particle mode
#interior_mode = "Verify"
scale_radius = 1
offset_radius = 0

#### FLOOR PARAMETERS ####
floor_length=100 # Length of the body floor
floor_height=particle_height     # height of the body floor


#### ENVIROMENT PARAMETERS ####
lateralFriction = 0.2
spinningFriction = 0.1
rollingFriction = 0.1
dampingterm = 0.0001
Ct = 0.0001 # tangent compliane
C = 0.00001 # compliance
Cr = 0.00001 # rolling compliance
Cs = 0.00001 # sliding compliance


#### CONTROL MODE -- SHAPE FORMATION ####
if control_mode=="shape_formation":
    geometry = 'pacman'
    if geometry == "circle":
        a = 1
        b = 1
        
    if geometry == 'pacman':
        a = 0.82
        b = 0.82
        
        
    else:
        a=0.82
        b=0.82      
    scale = 1
    alpha = 0.75
    beta = 0

#### CONTROL MODE -- MORPHING ####
if control_mode=="shape_morphing":
    geometry1 = 'circle'
    geometry2 = 'wrench'
    if geometry1 == "circle":
        a = 0
        b = 0
    
    if geometry2 == "circle":
        a = 0
        b = 0    
    
    scale1 = R
    scale2 = 1
    p = 0.5
    alpha = 1.0
    beta = 0
    
#### CONTROL MODE -- GRASPING ####
if control_mode=="grasping":
    
     
    ball_geometry = "circle" 
    
    # circle
    if ball_geometry=="circle":
        ball_radius=0.6570655533082147/2
        #ball_radius=.3
    # square     
    if ball_geometry=="square":	
        ball_radius=0.5160580788/2	
        
        #ball_radius = .25 

    # triangle
    if ball_geometry=="triangle":   
        ball_radius=0.6570655533082147/2
    
    if ball_geometry=="import":
        ball_radius=3
    #ball_radius = R*0.3

    ballx = 0
    ballz = 0     
    ball_mass = 5
    a1 = .01*ball_radius
    b1 = 5*ball_radius
    
    
    const=.01
    a2 = const
    b2 = const
    
    
    
    xc1 = ballz
    yc1 = ballx
    
    xc2 = ballx
    yc2 = ballz
    
    tcut1 = 2
    tcut2 = 60
    tcut3 = 100
    alpha1 = 2
    alpha2 = 3.5
    beta = 0
#### SAVE SIMULATION ####
now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
#mainDirectory = "F:/Soro_chrono/python/Pychrono/Strings/String_grasping/"
mainDirectory =os.path.abspath(os.getcwd())
savefile = mainDirectory +'/Experiments/'+ dt_string
os.makedirs(savefile, exist_ok=True)
txtFile = savefile+'/Parameters.csv'
    




##### export as npy file
envParams = {}
envParams['dt_string'] = dt_string
envParams['dt'] = dt
envParams['time_end'] = time_end
envParams['save_rate'] = save_rate
envParams['convert_dist'] = convert_dist
envParams['convert_mass'] = convert_mass
envParams['visual'] = visual
envParams['xcenter'] = xcenter
envParams['zcenter'] = zcenter

# control mode
envParams['control_mode'] = control_mode
envParams['alpha1'] = alpha1
envParams['alpha2'] = alpha2
envParams['beta'] = beta

if control_mode=='shape_formation':
    envParams['geometry'] = geometry
    envParams['scale'] = scale
    envParams['a'] = a
    envParams['b'] = b
    
    
if control_mode=='shape_morphing':
    envParams['geometry1'] = geometry1
    envParams['geometry2'] = geometry2
    envParams['scale1'] = scale1   
    envParams['scale2'] = scale2
    envParams['p'] = p
    if geometry1=='circle' or geometry2=='circle':
        envParams['a'] = a
        envParams['b'] = b
        
if control_mode=="grasping":
    envParams['a1'] = a1
    envParams['b1'] = b1    

    envParams['a2'] = a2
    envParams['b2'] = b2

    envParams['xc1'] = xc1
    envParams['yc1'] = yc1

    envParams['xc2'] = xc2
    envParams['yc2'] = yc2    

    envParams['tcut1'] = tcut1
    envParams['tcut2'] = tcut2
    envParams['tcut3'] = tcut3   
    
    envParams['ballx'] = ballx
    envParams['ballz'] = ballz 
    
    envParams['ball_geometry'] = ball_geometry
    envParams['ball_radius'] = ball_radius
    envParams['ball_mass'] = ball_mass
# Robot Parameters
envParams['nb'] = nb 
envParams['bot_mass'] = bot_mass
envParams['bot_geom'] = bot_geom
envParams['bot_width'] = bot_width
envParams['bot_height'] = bot_height
envParams['membrane_width'] = membrane_width
envParams['skin_width'] = skin_width
envParams['membrane_type'] = membrane_type
envParams['cord_length'] = cord_length
envParams['spring_stiffness'] = spring_stiffness
envParams['spring_damping'] = spring_damping
envParams['ns'] = ns
envParams['R'] = R
envParams['membrane_density']=membrane_density
# Particle Parameters  
envParams['interior_mode'] = interior_mode
envParams['particle_mass'] = particle_mass
envParams['particle_width'] = particle_width
envParams['particle_width2'] = particle_width2
envParams['particle_height'] = particle_height 
envParams['particle_geom'] = particle_geom
envParams['offset_radius'] = offset_radius
envParams['scale_radius'] = scale_radius
# floor parameters
envParams['floor_length'] = floor_length
envParams['floor_height'] = floor_height



# Physical Paramters
envParams['lateralFriction'] = lateralFriction
envParams['spinningFriction'] = spinningFriction
envParams['rollingFriction'] = rollingFriction
envParams['dampingterm'] = dampingterm
envParams['Ct'] = Ct
envParams['C'] = C
envParams['Cr'] = Cr
envParams['Cs'] = Cs
envParams['number_parameters'] = len(envParams)

np.save(savefile+'/Parameters.npy',envParams)


copyfile(__file__,savefile+"/"+'config.py')


#data=np.load(savefile+'/Parameters.npy',allow_pickle=True)
#data=data.tolist()