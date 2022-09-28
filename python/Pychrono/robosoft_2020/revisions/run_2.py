# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 11:12:06 2020

@author: qiyua
"""
import matplotlib.pyplot as plt
import json
import large_scale
import timeit
import numpy as np

start = timeit.default_timer()

# Import the .json file

with open('parameters.json') as json_data_file:
    parameters = json.load(json_data_file)
    
parameters['output_dir']=0
simtimes=[]
    
# Loop over all variables
number_boundary= [10,20,30,40,50,60,70,80,90,100,110,120]
#number_boundary= [120,130,140,150]
#sliding_friction= [0.07,0.14,0.21,0.35,0.42,0.49,0.56,0.63,0.7,0.77,0.84,0.91,0.98]
interiors=[-6,-5,-4,-3,-2]
for i in number_boundary:
    for j in interiors:
        # Write the new parameters
        parameters['boundary_bots']=i
        parameters['interiors']=j
        parameters['output_dir']+=1
        with open('parameters.json', 'w') as outfile:
            json.dump(parameters, outfile,indent=2)
            
        # Load the new parameters
        with open('parameters.json') as json_data_file:
            parameters = json.load(json_data_file)
        
        # Run simulation
        simtime=large_scale.run()
        simtimes.append(simtime)
    
stop = timeit.default_timer()
runtime=stop-start
runtime=runtime
print("Total runtime: "+str(runtime)+" seconds")

np.save('sim_times',simtimes)

# Plot

times=np.asarray(simtimes)

times_reshape=times.reshape((np.size(number_boundary),np.size(interiors)))

image,ax=plt.subplots(figsize=(9,9))
plt.imshow(times_reshape)
plt.title('Time to traverse gap')
plt.xlabel('Sliding Friction Coefficient')
plt.ylabel('Number of Boundary Robots')
ax.set_xticks(np.arange(np.shape(times_reshape)[0]))
ax.set_yticks(np.arange(np.shape(times_reshape)[1]))
ax.set_xticklabels(interiors)
ax.set_yticklabels(number_boundary)
cbar=plt.colorbar()
cbar.set_label('Time [s]')
plt.show()
plt.savefig('tunnel_times.png')

image,ax=plt.subplots(figsize=(9,9))
plt.imshow(times_reshape,interpolation='bilinear')
plt.title('Time to traverse gap')
plt.xlabel('Sliding Friction Coefficient')
plt.ylabel('Number of Boundary Robots')
ax.set_xticks(np.arange(np.shape(times_reshape)[0]))
ax.set_yticks(np.arange(np.shape(times_reshape)[1]))
ax.set_xticklabels(interiors)
ax.set_yticklabels(number_boundary)
cbar=plt.colorbar()
cbar.set_label('Time [s]')
plt.show()
plt.savefig('tunnel_times_bilinear.png')

image,ax=plt.subplots(figsize=(9,9))
plt.imshow(times_reshape,interpolation='bicubic')
plt.title('Time to traverse gap')
plt.xlabel('Sliding Friction Coefficient')
plt.ylabel('Number of Boundary Robots')
ax.set_xticks(np.arange(np.shape(times_reshape)[0]))
ax.set_yticks(np.arange(np.shape(times_reshape)[1]))
ax.set_xticklabels(interiors)
ax.set_yticklabels(number_boundary)
cbar=plt.colorbar()
cbar.set_label('Time [s]')
plt.show()
plt.savefig('tunnel_times_bicubic.png')