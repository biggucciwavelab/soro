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

# Import custom functions
from utils.utils import save_runtime

# Import custom functions
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback

# Loading info from environment needed in this list
from env_params import maxNumSteps, numBots

# Defining Experiment Parameters
experimentNum = '1c'
        
# Depending on how many bots there are in the system, 
# the number of training time steps will vary

training_timesteps = 100_000_000 # number of timesteps to train the environment agent for

# Training Parameters
neural_network =[500,500]
policyName = 'CustomPolicy_'+str(experimentNum)# Name of policy. This can be ignored.
gamma = 1.00                                   # Discount factor
n_steps = 1_000                                # Number of steps to run in each environment per update. Batchsize = n_steps*n_env
ent_coef = 0.01                                # Entropy coefficient
learning_rate = 0.00025                        # Learning Rate, can be a funcion
vf_coef = .5                                   # Value Function Coefficient in Loss Function
max_grad_norm = 0.5                            # Clipping factor for gradients. Should prevent exploding gradients
lam = 0.95                                     # Factor for bias vs variance for GAE
batch_size = 2_000                             # Number of minibatches at each update.
noptepochs = 7                                 # Number of epochs each update
cliprange = 0.2                                # Cliprange for PPO
seed = 12345                                   # Seed for neural network initialization
nEnvs = 4                                      # Number of parallel environments


# Parameters for callback
num_ep_save = 2 # Calculate the mean reward for this number of episodes and save that model
check_freq = 50000 # After how many timesteps do we check the frequency


# Post training parameters
test=True                       # Whether testing should occur post training
num_tests=3                     # Number of tests with thispolicy to run
render=True                     # Whether to visualize the training
time_per_test = maxNumSteps    # Number of timesteps to run each results episode for.


"""
____________________________________________________________
Users should not have to change anything on below this comment
____________________________________________________________
"""

# Ensuring we are in the proper directory
experimentName = 'Experiment_{}'.format(experimentNum)

policy_kwargs = dict(
    net_arch = [dict(
        pi=neural_network,
        vf=neural_network
    )]
)