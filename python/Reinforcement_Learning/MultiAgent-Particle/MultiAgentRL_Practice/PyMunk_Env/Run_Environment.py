# %% Main cell
"""
This script can be used to run an environment WITHOUT using any learned agent.

Of course, the user can also simply import a model here and call that agent
for actions to be taken.
"""

# Importing Parameters and all needed libraries
from star_env_params import *
# from Experiment_1_env_params import *

# Ensure working directory is set proper
filepath = os.path.dirname(os.path.realpath(__file__))

# Create the environment
env = pymunkEnv(**envParams)

if saveVideo:   
    savefile = filepath + '\\{}\\'.format(experimentName)
    os.makedirs(savefile,exist_ok=True)


bot_action=[]
for num in range(numBots*2):
    if num%2==0: bot_action.append(1) # Fx
    else: bot_action.append(0)        # Fy

agent2 = [env.targetDistance/env.width, 0.5]

action = [bot_action, agent2]

for _ in range(1):
    plt.close('all')
    obs = env.reset()

    for i in tqdm(range(maxNumSteps)):

        if render:
            env.render()
        obs, _, done, _ = env.step(action)

        if np.any(done): 
            print('Done')
            break

    print('End of episode')
    
    if dataCollect:
        env.dataExport()
    if saveVideo: 
        createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height))
    env.close()
    plt.close('all')