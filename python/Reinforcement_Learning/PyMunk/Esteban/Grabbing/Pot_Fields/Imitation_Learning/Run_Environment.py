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

# Create the environment
env = pymunkEnv(**envParams)

# Defining action
action=[]
for num in range(numBots*2):
    if num%2==0: action.append(0) # Fx
    else: action.append(0)        # Fy

# Creating pot field
pot_field_controller = create_pot_field(env)

# Define the controller, based on the pot field
control = controller(pot_field_controller)

for _ in range(1):
    plt.close('all') 
    obs = env.reset()
    obs1 = obs

    for i in tqdm(range(maxNumSteps)):
        if render:
            env.render()

        # Pot field control actions
        action = control.get_action(obs)
        
        obs, rew, done, _ = env.step(action)

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