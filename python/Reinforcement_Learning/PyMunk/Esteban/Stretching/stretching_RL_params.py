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
from win32api import NameCanonicalEx
sys.path.append('..')

# Import custom functions
from utils.utils import save_runtime

# Importing Stable Baselines
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback

# Loading info from environment needed in this list
from stretching_env_params import maxNumSteps, numBots

# Defining Experiment Parameters
experimentNum = 4
        
# Depending on how many bots there are in the system, 
# the number of training time steps will vary
botTrainingSteps = {#3:1_000_000,
                    # 10: 1_000, # For debugging
                    10:5_000_000, # For actual training
                    # 15:20_000_000,
                    # 30:30_000_000}
                    # 30:50_000_000}
                    30:5_000_000
                    }

# Training Parameters
neural_network =[1000,1000]
gamma = 0.99                                   # Discount factor
n_steps = 2000                                  # Number of steps to run in each environment per update. Batchsize = n_steps*n_env
ent_coef = 0.00                                # Entropy coefficient
learning_rate = 0.001                        # Learning Rate, can be a funcion
vf_coef = .5                                   # Value Function Coefficient in Loss Function
max_grad_norm = 0.5                            # Clipping factor for gradients. Should prevent exploding gradients
lam = 0.95                                     # Factor for bias vs variance for GAE
nminibatches = 100                             # Minibatch size at each update.
noptepochs = 4                                 # Number of epochs each update
cliprange = 0.2                                # Cliprange for PPO
seed = 12345                                   # Seed for neural network initialization
nEnvs = 4                                      # Number of parallel environments
training_timesteps = botTrainingSteps[numBots] # number of timesteps to train the environment agent for

# Parameters for callback
check_freq = 10_000 # How many iterations do before we save the most recent model.


# Post training parameters
test=True                       # Whether testing should occur post training
num_tests=3                     # Number of tests with thispolicy to run
render=True                     # Whether to visualize the training
time_per_test = maxNumSteps     # Number of timesteps to run each results episode for.


"""
____________________________________________________________
Users should not have to change anything on below this comment
____________________________________________________________
"""
policy_kwargs = dict(
    net_arch = [dict(
        pi=neural_network,
        vf=neural_network
    )]
)