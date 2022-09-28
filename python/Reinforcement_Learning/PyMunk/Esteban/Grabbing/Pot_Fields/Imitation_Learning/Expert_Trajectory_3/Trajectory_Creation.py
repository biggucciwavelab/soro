# %% 
# Import Dependencies

from stable_baselines.gail import generate_expert_traj

# Importing environment parameters
from env_params import *

# Prepare Environment

env = pymunkEnv(**envParams)

# Define the pot field
pot_field_controller = create_pot_field(env)

# Define the controller
control = controller(pot_field_controller)

# Create Save paths and save all information for this controller
name_of_experiment = 'Expert_Trajectory_3'

save_folder = name_of_experiment + '/'
os.makedirs(save_folder, exist_ok=True)

# Saving the parameters and environment for this data
import env_params, Environment_PF
copyfile(Environment_PF.__file__,save_folder+'Environment.py')
copyfile(env_params.__file__, save_folder+'env_params.py')
copyfile(__file__, save_folder + 'Trajectory_Creation.py')


# Create Expert Trajectory

generate_expert_traj(control.get_action,             # Expert Model
                    save_folder + name_of_experiment,   # Save Path
                    env,                                # Environment
                    3_000,                             # Number of timesteps
                    1)                                  # Number of episodes
                    
if dataCollect:                    
    env.dataExport()
