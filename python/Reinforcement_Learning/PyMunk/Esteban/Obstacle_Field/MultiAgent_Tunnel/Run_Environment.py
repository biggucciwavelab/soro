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
    # f = 15
    f = 5
    actions[agent] = [f,f]

# Slowing down sim so it is easier to see
from time import sleep
slow = False # Set to True if you need the video to slow down.
             # Do not use if you are recording the video!

for _ in range(1):
    plt.close('all')
    obs = env.reset()

    # Gathering information on the location of turns for testing
    firstTurn, secondTurn = env.firstTurn, env.secondTurn
    x1, y1 = firstTurn
    x2, y2 = secondTurn

    for i in tqdm(range(maxNumSteps)):
        if slow: sleep(.01)
        if render:
            env.render()
        
        # actions = dict()
        # for agent in env.agents:
        #     pos = env.observation(agent)
        #     x,y = pos
        #     x *= env.width
        #     y *= env.height
        #     if x < x1:
        #         actions[agent] = [f,0]
        #     elif x > x1 and y < y2:
        #         actions[agent] = [0,f]
        #     else:
        #         actions[agent] = [f,0]


        obs, _, done, _ = env.step(actions)
        
        if any(list(done.values())): 
            print('Done')
            break

    print('End of episode')
    
    if dataCollect:
        env.dataExport()
    if saveVideo: 
        createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height))
    env.close()
    plt.close('all')