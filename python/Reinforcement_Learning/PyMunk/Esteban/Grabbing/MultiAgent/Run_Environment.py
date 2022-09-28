# %% Main cell
"""
This script can be used to run an environment WITHOUT using any learned agent.

Of course, the user can also simply import a model here and call that agent
for actions to be taken.
"""

# Importing Parameters and all needed libraries
from env_params import *

# Ensure working directory is set proper
filepath = os.path.dirname(os.path.realpath(__file__))
sys.path.append('../..') # Commands Python to search for modules in this folder as well.

# Create the environment
env = parallel_env(**envParams)
_  = env.reset()

# Defining actions for simple L-Turn
actions=dict()
for agent in env.agents:
    f = 0
    actions[agent] = [0,0]

for _ in range(1):
    plt.close('all')
    obs = env.reset()

    for i in tqdm(range(maxNumSteps)):
        if render:
            env.render(None)
        obs, _, done, _ = env.step(actions)
        
        # if any(list(done.values())): 
        #     print('Done')
        #     break

    print('End of episode')
    
    if dataCollect:
        env.dataExport()
    if saveVideo: 
        createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height))
    env.close()
    plt.close('all')