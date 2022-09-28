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

# Defining actions
actions=dict()
# f = np.linspace(-1,1,env.numBots)
# for i, agent in enumerate(env.agents):
#     actions[agent] = [f[i],f[i]]

for i, agent in enumerate(env.agents):
    f = .5
    actions[agent]=[f,0]




# Slowing down sim so it is easier to see
from time import sleep
slow = False # Set to True if you need the video to slow down.
             # Do not use if you are recording the video!

for _ in range(1):
    plt.close('all')
    if dataCollect:
        env.report_all_data = True
    obs = env.reset()

    for i in tqdm(range(maxNumSteps)):
        if slow: sleep(.01)
        if render:
            env.render(None)
        obs, _, done, _ = env.step(actions)
        if done['Bot_0']:
            print('Terminated Sim')
            break
        
    print('End of episode')
    
    if dataCollect:
        env.dataExport()
    if saveVideo: 
        createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height))
    if env.report_all_data:
        env.exportAllData()
    env.close()
    plt.close('all')