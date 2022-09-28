# -*- coding: utf-8 -*-
"""
Created on Tue Feb 23 16:10:25 2021

@author: qiyua
"""
import environment as env
import config as conf
import numpy as np
import shutil
import os
import time
import subprocess
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
'''
** Make sure to set up config file for running into wall test for this one**
Things to check for:
    visual='pov'
    tend=5
    obstacles=False
    grasp=True
    C=1e-4
    Ct=1e-4
    sens_rate=500
    
    self.noise_params = np.asarray([[1e-1*self.d2r,        0.1*self.d2r/self.V],     
                                   [1e-1*self.d2r,        0.1*self.d2r/self.V],     
                                   [1e-1*self.d2r,        0.1*self.d2r/self.V],     
                                   [1e-1*self.d2r,        0.1*self.d2r/self.V],
                                   
                                # Accelerometer x,z
                                   [150*self.ug_hz,        self.mg_k+0.5*self.mg_v],
                                   [150*self.ug_hz,        self.mg_k+0.5*self.mg_v],
                                   
                                # Magnetometer quaternion components
                                   [3*self.d2r,        1e-5],
                                   [3*self.d2r,        1e-5],
                                   [3*self.d2r,        1e-5],
                                   [3*self.d2r,        1e-5]])
    and the only active plot is
    self.save_pred_err = True
'''

def run_sim(cf):
    # Start simulation timer
    start = time.time()
    
    # Delete previous data
    if True:
        if os.path.exists(cf.sim_id):
            shutil.rmtree(cf.sim_id)
    
    # Initialize the environment
    simulation=env.sim_env(cf)    
    
    # Run the simulation loop
    simulation.simulate()   
    
        # Create post-processing plots if we hit the end of the simulation or close the window
    if cf.visual=='irr':
        if simulation.sys.GetChTime() >= cf.tend or not simulation.sim_win.GetDevice().run():
            
            # Simulation Timers
            end = time.time()
            mins = int(np.floor((end - start)/60.0))
            secs = round((end-start) - mins*60,3)
            print('')
            print('Elapsed simulation time: ', mins, ' min ', secs, ' s')
            
            # Plot timer
            start = time.time()
            
            # Plots
            simulation.plot_obj.save_to_disk()
            if os.path.exists(cf.sim_id+'/video_capture/'):
                shutil.rmtree(cf.sim_id+'/video_capture/')
                shutil.move('video_capture/',cf.sim_id+'/') 
            simulation.plot_obj.make_post_plots()
            
            # Plot timers
            end = time.time()
            mins = int(np.floor((end - start)/60.0))
            secs = round((end-start) - mins*60,3)
            print('Elapsed plotting time:   ', mins, ' min ', secs, ' s')
                
    elif cf.visual=='pov':
        # Simulation Timers
        end = time.time()
        mins = int(np.floor((end - start)/60.0))
        secs = round((end-start) - mins*60,3)
        print('')
        print('Elapsed simulation time: ', mins, ' min ', secs, ' s')
        
        # Plot timer
        start = time.time()
        
        # Plots
        simulation.plot_obj.save_to_disk()
        simulation.plot_obj.make_post_plots()
        
        # Plot timers
        end = time.time()
        mins = int(np.floor((end - start)/60.0))
        secs = round((end-start) - mins*60,3)
        print('Elapsed plotting time:   ', mins, ' min ', secs, ' s')
        
        # Copy Pov-Ray files and render
        # POV-Ray timer
        start = time.time()
        
        # Copy render files
        shutil.copyfile('render.pov', cf.sim_id+'/render.pov')
        shutil.copyfile('render.pov.assets', cf.sim_id+'/render.pov.assets')
        shutil.copyfile('render.pov.ini', cf.sim_id+'/render.pov.ini')
        
        # Command line launch
        if False:
            args = "render.pov.ini"
            path = "C:/Chrono/Dependencies/povray-autobuild-alpha_v380/windows/vs2015/bin64/pvengine64-avx.exe"
            subprocess.run([path,args])
        
        # Render timers
        end = time.time()
        mins = int(np.floor((end - start)/60.0))
        secs = round((end-start) - mins*60,3)
        print('Elapsed render time: ', mins, ' min ', secs, ' s')
   
# %% Simulate update rate and contact stiffness vs tracking accuracy
if True:
    start = time.time()
    # Array of parameters to vary
    
    # Update frequency
    #params1=np.arange(100,600,100)
    #params1=[50.,100.,200.,500.,1000.]
    
    # Spring stiffness
    params1=np.linspace(100,5000,5)
    
    # Contact stiffness
    params2=np.logspace(np.log10(1e-8), np.log10(1e-3), 6)
    
    # Sensor noise
    #params2=np.logspace(np.log10(1e-5), np.log10(1e-1), 5)
    
    # Number of inactive bots between each active bot
    #params2 = np.linspace(0,5,6)
    
    # Iterate over param1
    for ii in range(len(params1)):
        
        # Change param1 in config file
        cf = conf.configuration(sens_rate=100)
    
        cf.visual='pov'
        cf.tend=5
        cf.obstacles=False
        cf.grasp=True
        cf.save_lmap=False
        cf.km=params1[ii]
        
        # Iterate over param2
        for jj in range(len(params2)):
    
            # cf.ratio_meas = params2[jj]
            # Sensor noise params
            # cf.noise_params = np.zeros((10,2))
            # cf.noise_params[4,0] = params2[jj]
            # cf.noise_params[5,0] = params2[jj]
            # cf.noise_params[4,1] = params2[jj]
            # cf.noise_params[5,1] = params2[jj]
            
            # Change param2 in config file
            cf.C = params2[jj]
            cf.Ct = params2[jj]
            #cf.sim_id = 'phasesim6/sens_rate'+str(params1[ii])+'C'+'{:0.3e}'.format(cf.C)
            #cf.sim_id = 'phasesim8/sens_rate'+str(params1[ii])+'-accel_noise'+'{:0.3e}'.format(params2[jj])
            #cf.sim_id = 'phasesim5/sense_rate'+str(params1[ii])+'-nactive'+str(params2[jj])
            cf.sim_id = 'phasesim9/spr_rate'+str(params1[ii])+'-cont_rate'+'{:0.3e}'.format(params2[jj])
            
            # Console print
            #print('\nsens_rate: ', cf.sens_rate, '[Hz]                  (', ii+1,'of', len(params1),')')
            print('spring :    ','{:0.3e}'.format(cf.km),'[N/m]           (', ii+1,'of', len(params1), ')\n')
            print('C, Ct :    ','{:0.3e}'.format(cf.C),'[m/N]           (', jj+1,'of', len(params2), ')\n')
            #print('Accel noise: ','{:0.3e}'.format(params2[jj]),'[m/s^2]       (', jj+1,'of', len(params2), ')\n')
            
            # Run the simulation
            run_sim(cf)
            
    end = time.time()
    mins = int(np.floor((end - start)/60.0))
    secs = round((end-start) - mins*60,3)
    
    print('\n*----------------------------------------------*')
    print('Total elapsed time:   ', mins, ' min ', secs, ' s')
    print('*----------------------------------------------*')

# %% Plot results for update rate and contact stiffness vs tracking accuracy
start = time.time()
# Update frequency
#params1=np.arange(100,600,100)

# Sensor noise
#params2=np.logspace(np.log10(1e-5), np.log10(1e-1), 5)
# Contact stiffness
#params2=np.logspace(np.log10(1e-8), np.log10(1e-3), 10)
#params1=np.arange(100,1000,100)

#params2=np.logspace(np.log10(1e-6), np.log10(1e-3), 10)
#params2=np.logspace(np.log10(1e-8), np.log10(1e-5), 10)
#params2=np.logspace(np.log10(1e-10), np.log10(1e-3), 30)
#params2 = np.linspace(0,5,6)

x_mean = np.zeros((len(params1),len(params2)))
z_mean = np.zeros((len(params1),len(params2)))
t_mean = np.zeros((len(params1),len(params2)))

x_max = np.zeros((len(params1),len(params2)))
z_max = np.zeros((len(params1),len(params2)))
t_max = np.zeros((len(params1),len(params2)))

out_path = 'phasesim9/_figures/'
if not os.path.exists(out_path):
    os.makedirs(out_path)

# Iterate over param1
for ii in range(len(params1)):
    
    # Get param1
    param1 = params1[ii]
    
    # Iterate over param2
    for jj in range(len(params2)):
        
        # Get param2
        param2 = params2[jj]
        # sim_path = 'phasesim6/sens_rate'+str(param1)+'C'+'{:0.3e}'.format(param2)
        # sim_path = 'phasesim3/sens_rate'+str(param1)+'C'+'{:0.3e}'.format(param2)
        #sim_path = 'phasesim8/sens_rate'+str(param1)+'-accel_noise'+'{:0.3e}'.format(param2)
        #sim_path = 'phasesim5/sense_rate'+str(params1[ii])+'-nactive'+str(param2)
        sim_path = 'phasesim9/spr_rate'+str(params1[ii])+'-cont_rate'+'{:0.3e}'.format(params2[jj])
        dat_path = sim_path + '/save_data/pred_err.npy'
        
        # Errors for x,z,theta
        # x=np.load(dat_path)[0:-1:int(1000/param1),0:-1:3]
        # z=np.load(dat_path)[0:-1:int(1000/param1),1:-1:3]
        # t=np.load(dat_path)[0:-1:int(1000/param1),2:-1:3]
        
        # active vs inactive bots
        # x=np.load(dat_path)[0:-1:int(1000/param1),0:-1:int(3*(param2+1))]
        # z=np.load(dat_path)[0:-1:int(1000/param1),1:-1:int(3*(param2+1))]
        # t=np.load(dat_path)[0:-1:int(1000/param1),2:-1:int(3*(param2+1))]
        
        # Single bot
        # bot = int(2 * param2 + 1)
        # x=np.load(dat_path)[0:-1:int(1000/param1),bot*3]
        # z=np.load(dat_path)[0:-1:int(1000/param1),bot*3+1]
        # t=np.load(dat_path)[0:-1:int(1000/param1),bot*3+2]
        
        x=np.load(dat_path)[:,0::3]
        z=np.load(dat_path)[:,1::3]
        t=np.load(dat_path)[:,2::3]
        
        # Mean errors
        x_mean[ii,jj]=np.mean(np.abs(x))/9.45#(1000./param1)
        z_mean[ii,jj]=np.mean(np.abs(z))/9.45#(1000./param1)
        t_mean[ii,jj]=np.mean(np.abs(t))/9.45#(1000./param1)
        
        # Max errors
        x_max[ii,jj]=np.max(np.abs(x))/9.45
        z_max[ii,jj]=np.max(np.abs(z))/9.45
        t_max[ii,jj]=np.max(np.abs(t))/9.45
        
# Put all the means and max arrays into lists
means = [x_mean,z_mean,t_mean,'X','Y','Theta']
maxes = [x_max, z_max, t_max]

# Make the max plots
for i in range(3):
    fig = plt.figure
    cmap = plt.get_cmap('summer')
    plt.pcolormesh(params1, params2, maxes[i].T,cmap=cmap,norm=LogNorm())
    plt.colorbar()
    plt.xlabel('Update Rate [Hz]')
    # plt.ylabel('Contact Stiffness [m/N]')
    plt.ylabel('Accelerometer Noise [m/s^2]')
    # plt.ylabel('Number of inactive bots between each active bot')
    plt.yscale('log')
    plt.title('Max Tracking Error ' + means[i+3])
    plt.savefig(out_path + means[i+3] + 'max.png')
    #plt.savefig(out_path + str(2) + '/' + means[i+3] + 'max.png')
    plt.close()

# Make the mean plots
for i in range(3):
    fig = plt.figure()
    cmap = plt.get_cmap('summer')
    plt.pcolormesh(params1, params2, means[i].T,cmap=cmap)#,norm=LogNorm())
    plt.colorbar()
    plt.xlabel('Update Rate [Hz]')
    #plt.ylabel('Contact Stiffness [m/N]')
    #plt.ylabel('Number of inactive bots between each active bot')
    plt.ylabel('Accelerometer Noise [m/s^2]')
    plt.yscale('log')
    plt.title('Mean Tracking Error ' + means[i+3])
    plt.savefig(out_path + means[i+3] + 'mean.png')
    #plt.savefig(out_path + str(2) + '/' + means[i+3] + 'mean.png')
    plt.close()

end = time.time()
mins = int(np.floor((end - start)/60.0))
secs = round((end-start) - mins*60,3)
print('\n*----------------------------------------------*')
print('Total plotting time:   ', mins, ' min ', secs, ' s')
print('*----------------------------------------------*')

# %% Simulate changing accelerometer noise vs accuracy

