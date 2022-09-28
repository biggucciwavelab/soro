"""
This file contains all parameters needed for running reinforcememt learning training on the grabbing environmnet
"""

# Import standard libraries
import time
from datetime import date
import os
import sys
import pathlib
from shutil import copyfile
import matplotlib.pyplot as plt
import numpy as np
sys.path.append('../..')

from stable_baselines.common.policies import register_policy, FeedForwardPolicy
from stable_baselines import PPO2
from stable_baselines.common import make_vec_env
from stable_baselines.bench import Monitor
from stable_baselines.common.vec_env import SubprocVecEnv

# Loading info from environment needed in this list
from env_params import maxNumSteps, numBots

# Defining Experiment Parameters
experimentNum = '4'
        
# Depending on how many bots there are in the system, 
# the number of training time steps will vary
botTrainingSteps = {#3:1_000_000,
                    # 10:10_000,
                    10:50_000_000,
                    # 15:20_000_000,
                    30:10_000_000
                    }


# Training Parameters
neural_network =[128,128]
policyName = 'CustomPolicy_'+str(experimentNum)# Name of policy. This can be ignored.
gamma = 0.99                                      # Discount factor
n_steps = 2000                                 # Number of steps to run in each environment per update. Batchsize = n_steps*n_env
ent_coef = 0.01                                # Entropy coefficient
learning_rate = 0.001                          # Learning Rate, can be a funcion
vf_coef = 0.0                                  # Value Function Coefficient in Loss Function
max_grad_norm = 0.5                            # Clipping factor for gradients. Should prevent exploding gradients
lam = 0.95                                     # Factor for bias vs variance for GAE
nminibatches = 8                               # Number of minibatches at each update. Thus the minibatchsize = batchsize//minibatches
noptepochs = 10                                # Number of epochs each update
cliprange = 0.2                                # Cliprange for PPO
seed = 12345                                   # Seed for neural network initialization
nEnvs = 1                                      # Number of parallel environments
training_timesteps = botTrainingSteps[numBots] # number of timesteps to train the environment agent for


# Parameters for callback
num_ep_save = 2 # Calculate the mean reward for this number of episodes and save that model
check_freq = 50000 # After how many timesteps do we check the frequency


# Post training parameters
test=True                       # Whether testing should occur post training
num_tests=3                     # Number of tests with thispolicy to run
render=True                     # Whether to visualize the training
time_per_test = maxNumSteps     # Number of timesteps to run each results episode for.


"""
Users should not have to change anything on below this comment
"""

# Ensuring we are in the proper directory
experimentName = 'Experiment_{}'.format(experimentNum)

policy_kwargs = dict(
    net_arch = [dict(
        pi=neural_network,
        vf=neural_network
    )]
)


from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.common.callbacks import BaseCallback


"""
First we define a class that takes the object 'BaseCallback' in its argument.

This specific callback will be called every X timesteps, where X is the variable in 'check_freq' and evaluate the model.
If the past 100 calls to the model have been the best ones yet, then we save that model!
"""
class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """
    def __init__(self, check_freq: int, log_dir: str, num_ep_save: int, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.num_ep_save = num_ep_save
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

          # Retrieve training reward
          x, y = ts2xy(load_results(self.log_dir), 'timesteps')
          if len(x) > 0:
              # Mean training reward over the last 100 episodes
              mean_reward = np.mean(y[-self.num_ep_save:])
              if self.verbose > 0:
                print("Num timesteps: {}".format(self.num_timesteps))
                print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(self.best_mean_reward, mean_reward))

              # New best model, you could save the agent here
              if mean_reward > self.best_mean_reward:
                  self.best_mean_reward = mean_reward
                  # Example for saving best model
                  if self.verbose > 0:
                    print("Saving new best model to {}".format(self.save_path))
                  self.model.save(self.save_path)

        return True