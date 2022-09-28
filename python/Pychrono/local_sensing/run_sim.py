""""
run_sim.py: 
    
This is the main run file for running simulations
of local sensing controls strategies

Author: Qiyuan Zhou
Date (MM/DD/YY): 09/10/20
Wavelab | Illinois Institute of Technology
"""
import environment as env
import config as cf
import numpy as np
import shutil
import os
import time
import subprocess

# Start simulation timer
start = time.time()

# Initialize config object
config = cf.Configuration()

# Delete previous data
if True:
    if os.path.exists(config.sim_id):
        shutil.rmtree(config.sim_id)

# Initialize the environment
simulation = env.SimEnv(config)

# Run the simulation loop
simulation.simulate()   

# Create post-processing plots if we hit the end of the simulation or close the window

# Chrono simulation with Irrlicht visuals
# %%
if config.visual=='irr':
    if simulation.sys.GetChTime() >= config.tend or not simulation.sim_win.GetDevice().run():
        
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
        if os.path.exists(config.sim_id+'/video_capture/'):
            shutil.rmtree(config.sim_id+'/video_capture/')
            shutil.move('video_capture/',config.sim_id+'/') 
        simulation.plot_obj.make_post_plots()
        
        # Plot timers
        end = time.time()
        mins = int(np.floor((end - start)/60.0))
        secs = round((end-start) - mins*60,3)
        print('Elapsed plotting time:   ', mins, ' min ', secs, ' s')
        
# Chrono simulation to generate files for rendering in POV-Ray
# %%
elif config.visual=='pov':
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
    shutil.copyfile('render.pov', config.sim_id+'/render.pov')
    shutil.copyfile('render.pov.assets', config.sim_id+'/render.pov.assets')
    shutil.copyfile('render.pov.ini', config.sim_id+'/render.pov.ini')
    
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
    
# Run simulation with matplotlib animation with experiemental results
# %%
elif config.visual=='exp':
    
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