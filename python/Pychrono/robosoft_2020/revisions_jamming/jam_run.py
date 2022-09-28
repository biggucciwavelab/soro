# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 11:12:06 2020

@author: qiyua
"""
import matplotlib.pyplot as plt
import json
import sim
import timeit
import numpy as np
from objects import PackingFraction
start = timeit.default_timer()

# Import the .json file

with open('parameters.json') as json_data_file:
    parameters = json.load(json_data_file)
    
parameters['output_dir']=0
#PHI=[]
simtimes=[]

#k=[-10,-11,-12,-13,-14,-15,-16,-17,-18,-19,-20,-21,-22,-23,-24,-25,-26,-27,-28]
<<<<<<< HEAD
nb=[50]
k=[50,100,200,300,400,500]
=======
nb=[10,20,30,40,50,60,70,80,90,100]
k=[-1,-2,-3,-4,-5,-6,-7,-8,-9,-10]
>>>>>>> 19a287d6c9470158a21d88103158ff13bcc58a6e


PHI=np.zeros((len(nb),len(k)))
#nb=[10,20,30,40,50,60,70,80,90,100]
m=0
n=0
np.save('nb',nb)
np.save('k',k)
for i in nb:
    m=0
    for j in k:
        parameters['nb']=i
        parameters['k']=j
        parameters['output_dir']+=1
        simn=parameters['output_dir']
        with open('parameters.json', 'w') as outfile: 
            json.dump(parameters, outfile,indent=2)
            
        # Load the new parameters
        with open('parameters.json') as json_data_file:
            parameters = json.load(json_data_file)
        (qx,qz,simtime,time,nb,ni,diameter,height,count)=sim.run()
        simtimes.append(simtime)
        path="F:/Robosoft2020_data/revisiions_jamming/trial11/"+str(simn)+"/"
       
        file='.png'
        m=m+1
    n=n+1
#        PHI.append(phi)

                     
np.save('simtimes',simtimes)
stop = timeit.default_timer()
runtime=stop-start
runtime=runtime/3600
print("Total runtime: "+str(runtime)+" hours")