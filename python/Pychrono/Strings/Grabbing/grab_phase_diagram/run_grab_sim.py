 # -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 18:26:11 2020

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from grab_sim_objects import *
from config import *
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import grab_sim
import json

start = timeit.default_timer()

with open('parameters.json') as json_data_file:
    parameters = json.load(json_data_file)
    
parameters['output_dir']=0



k=[50,75,100,125,150,175,200,225,250,275,300,325,350,375,400,425,450,475,500,525,550,575,600]
alpha=[100,125,150,175,200,225,250,275,300,325,350,375,400,425,450,475,500]

errors=np.zeros((len(alpha),len(k)))
avgFxc=np.zeros((len(alpha),len(k)))
avgFyc=np.zeros((len(alpha),len(k)))
avgFzc=np.zeros((len(alpha),len(k)))
path="C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/python/Pychrono/Grabbing/grab_phase_diagram/"
n=0
for i in k:
    m=0
    for j in alpha:
        
        parameters['k']=i
        parameters['alpha']=j

        parameters['output_dir']+=1
        simn=parameters['output_dir']
        with open('parameters.json', 'w') as outfile: 
            json.dump(parameters, outfile,indent=2)
            
        # Load the new parameters
        with open('parameters.json') as json_data_file:
            parameters = json.load(json_data_file)
        
        (E,Fxct,Fyct,Fzct)=grab_sim.run()
        errors[m,n]=E
        avgFxc[m,n]=np.average(Fxct[:,-1])
        avgFyc[m,n]=np.average(Fyct[:,-1])
        avgFzc[m,n]=np.average(Fzct[:,-1])
        m=m+1
    n=n+1
        
results_dir = os.path.join(path+"NPZ3_files")     

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)
    
np.savez(results_dir+"/file.npz",allow_pickle=True,k=k,alpha=alpha,errors=errors,avgFxc=avgFxc,avgFyc=avgFyc,avgFzc=avgFzc,avgFc=avgFc)
    
stop = timeit.default_timer()
runtime=start-stop
runtime=runtime/3600
print("Total runtime: "+str(runtime)+" hours")
