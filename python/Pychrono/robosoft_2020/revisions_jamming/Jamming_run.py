# -*- coding: utf-8 -*-
"""
Created on Sun Feb  9 11:18:33 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
import json

def run():
    
    with open('parameters.json') as json_data_file:
        parameters = json.load(json_data_file)
        
    sim=str(parameters["output_dir"])
    
    # create folders
    path="results/"+sim
    if not os.path.exists(path):
        #os.mkdir("results/")
        os.mkdir(path)
    
    start = timeit.default_timer()
    record=parameters["run_mode"]
    tstep=parameters['t_step'] #Time Step
    tset= 0.1 #Settling time
    
    # In[Set Path]
    #chrono.SetChronoDataPath("C:/Chrono/Builds/chrono-develop/bin/data/")
    chrono.SetChronoDataPath("C:/Users/dmulr/OneDrive/Documents/data/"
    # In[Create sysem and other misselanous things]
    my_system = chrono.ChSystemNSC()
    #my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    my_system.SetSolverType(chrono.ChSolver.Type_SOR_MULTITHREAD)
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    #chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0001)
    #chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.01)
    my_system.SetMaxItersSolverSpeed(150)
    #my_system.SetMaxPenetrationRecoverySpeed(5)
    #my_system.SetMinBounceSpeed(.5)
    
    # In[Define frictional properties]
    mu_f=parameters["sliding_friction"]
    mu_b=.01
    mu_r=parameters["rolling_friction"]
    mu_s=parameters["spinning_friction"]
    
    material=Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs)
    
    Floor(material,length,tall)
    