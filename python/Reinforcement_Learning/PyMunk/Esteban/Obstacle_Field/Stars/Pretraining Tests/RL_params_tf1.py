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
from stable_baselines.common.callbacks import BaseCallback
from stable_baselines.common.policies import register_policy, FeedForwardPolicy
from stable_baselines import PPO2
from stable_baselines.common import make_vec_env
from stable_baselines.bench import Monitor
from stable_baselines.common.vec_env import SubprocVecEnv

# Loading info from environment needed in this list
from env_params import maxNumSteps, numBots

# Defining Experiment Parameters
experimentNum = 'Testing'

# Training Parameters
neural_network =[1000,1000]
policyName = 'CustomPolicy_'+str(experimentNum)# Name of policy. This can be ignored.
gamma = .99                                      # Discount factor
n_steps = 2000                                 # Number of steps to run in each environment per update. Batchsize = n_steps*n_env
ent_coef = 0.00                                # Entropy coefficient
learning_rate = 0.001                          # Learning Rate, can be a funcion
vf_coef = 0.0                                  # Value Function Coefficient in Loss Function
max_grad_norm = 0.5                            # Clipping factor for gradients. Should prevent exploding gradients
lam = 0.95                                     # Factor for bias vs variance for GAE
nminibatches = 8                               # Number of minibatches at each update. Thus the minibatchsize = batchsize//minibatches
noptepochs = 10                                # Number of epochs each update
cliprange = 0.2                                # Cliprange for PPO
seed = 12345                                   # Seed for neural network initialization

# Pretrainin parameter
pretrain_batchsize = 1000                      # Batch size for pretraining the model
n_epochs = 20

# Post training parameters
test=True                       # Whether testing should occur post training
num_tests=3                     # Number of tests with thispolicy to run
render=True                     # Whether to visualize the training
time_per_test = maxNumSteps     # Number of timesteps to run each results episode for.

# Defining the policy kwargs to be passed
# This makes the neural network that we desire
policy_kwargs = dict(
    net_arch = [dict(
        pi = neural_network,
        vf = neural_network
    )]
)