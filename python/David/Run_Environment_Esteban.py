# %% 
# Main cell
from env_params import *

def code():
    # Step 0: Importing Parameters and all needed libraries  
    import numpy as np

    # Step 1: Ensure working directory is set proper
    pass

    filepath = os.path.dirname(os.path.realpath(__file__))

    # Step 2: Create the environment
    env = robot(**envParams)

    # Step 4: Always call env.reset() before running the sim!
    # This creates the PyMunk space
    startLoc = np.array([env.width/2, env.height/2]) # This starts the system at the center of the environment
    obs = env.reset(startLoc)                        # You can also choose to not enter an argument here, and the system will spawn on the left-side

    # Step 6: Define some action.
    # Action should a numpy array of shape (numBots, 2)
    # The first column are the forces in the x-direction for each bot, the second column are forces in the y-direction
    action = np.zeros((numBots, 2))
    action[:,0] = 1

    for i in tqdm(range(300)):
        # You can choose to NOT render the simulation if all you want to do is collect data
        # This drastically speeds up the wall-clock time, as now there is no need to render all objects
        # and only the physics engine is running
        env.render()

        # Step the environment forward a step.
        # Returns an observation now that a step was taken, and whether the environment is done.
        observation, _ = env.step(action)
        if i==200:
            action[:,0] = -1

    print('End of episode')

    if dataCollect:
        env.dataExport()
    if saveVideo: 
        createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height))

    # Make sure to call env.close() when you are done! This exists the physics engine cleanly. 
    # Not a requirement, just good practice.
    env.close()

    # Closing any plots that may have been opened. 
    # This is only actually needed when you dataCollect=True
    plt.close('all') 
    
    return env


if __name__=="__main__":
    import cProfile
    from pstats import Stats

    profile = False
    if profile:
        pr = cProfile.Profile()
        pr.enable()
        code()
        pr.disable()

        with open('profiling_stats.txt', 'w') as stream:
            stats = Stats(pr, stream=stream)
            stats.strip_dirs()
            stats.sort_stats('time')
            stats.dump_stats('.prof_stats')
            stats.print_stats()
    else:
        env = code()