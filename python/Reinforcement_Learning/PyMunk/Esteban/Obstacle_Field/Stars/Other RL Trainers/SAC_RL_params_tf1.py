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
from stable_baselines import SAC

# Loading info from environment needed in this list
from env_params import maxNumSteps, numBots

# Defining Experiment Parameters
experimentNum = '12'
        
# Depending on how many bots there are in the system, 
# the number of training time steps will vary
botTrainingSteps = {#3:1_000_000,
                    # 10:1_000,
                    10:30_000_000,
                    # 15:20_000_000,
                    # 30:30_000_000}d
                    # 30:50_000_000}
                    30:10_000_000
                    }


# Training Parameters
neural_network =[128,128]
gamma = 0.99
learning_rate = .0003
buffer_size = 50_000
batch_size = 500
tau = .005
ent_coef = 'auto'
train_freq = 4_000
learning_starts = 1_000
gradient_steps = 5
seed = 12345
training_timesteps = botTrainingSteps[numBots] # number of timesteps to train the environment agent for

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

# Defining the policy kwargs to be passed
# This makes the neural network that we desire
policy_kwargs = dict(
    layers=neural_network
)