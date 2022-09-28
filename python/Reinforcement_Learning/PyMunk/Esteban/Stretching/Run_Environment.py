# %% Main cell
"""
This script can be used to run an environment WITHOUT using any learned agent.

Of course, the user can also simply import a model here and call that agent
for actions to be taken.
"""

# Importing Parameters and all needed libraries
from stretching_env_params import *
import time

# Ensure working directory is set proper
filepath = os.path.dirname(os.path.realpath(__file__))
sys.path.append('../..') # Commands Python to search for modules in this folder as well.

# Create the environment
env = pymunkEnv(**envParams)

action=[]
for num in range(numBots*2):
    if num%2==0: 
        action.append(1) # Fx

    else: 
        action.append(0) # Fy
action = np.asarray(action)
action[10:] *= (-1)

# Run Simulation
for _ in range(1):
    plt.close('all')
    env.reset()

    for i in tqdm(range(maxNumSteps)):
        if render:
            env.render()

        _, rew, done, _ = env.step(action)

        if done: 
            print('Done')
            break

    print('End of episode')
    
    if dataCollect:
        env.dataExport()
    if saveVideo: 
        createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height))
    env.close()
    plt.close('all')