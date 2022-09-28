# -*- coding: utf-8 -*-
"""
Created on Sun Mar 28 22:26:53 2021

@author: elope
Taken from:
    https://stable-baselines.readthedocs.io/en/master/guide/examples.html#recurrent-policies
"""

# %%This code works!
from stable_baselines import PPO2
import gym
import pathlib
import os
from stable_baselines.common.vec_env import DummyVecEnv

# For recurrent policies, with PPO2, the number of environments run in parallel
# should be a multiple of nminibatches.

def make_env():
    def maker():
        env = gym.make('CartPole-v1')
        return env
    return maker

n_training_envs = 1
envs = DummyVecEnv([make_env() for _ in range(n_training_envs)])


# model = PPO2('MlpLstmPolicy', 'CartPole-v1', nminibatches=1, verbose=1) # This one works
model = PPO2('MlpLstmPolicy', envs, nminibatches=1, verbose=1) # We are trying to see if this one works.
model.learn(50000)
mainDirectory = str(pathlib.Path(__file__).parent.absolute())
model.save(mainDirectory+'\\testAgent')

# # Retrieve the env
"""
Neither of these are needed since only one environment is vectorized
"""
# env = model.get_env() # This one works
# env = make_env()() # We are trying to see if this one works

obs = envs.reset()
# Passing state=None to the predict function means
# it is the initial state
state = None
# When using VecEnv, done is a vector
# done = [False for _ in range(env.num_envs)] # This one works
done = [False] # We are trying to see if this one works
for _ in range(1000):
    # We need to pass the previous state and a mask for recurrent policies
    # to reset lstm state when a new episode begin
    action, state = model.predict(obs, state=state, mask=done)
    obs, reward , done, _ = envs.step(action)
    # Note: with VecEnv, env.reset() is automatically called

    # Show the env
    envs.render()

envs.close()


# %%Testing if this code will work.
"""
This is the code that contains a custom policy
"""
from stable_baselines import PPO2
import gym
import pathlib
import os
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.policies import LstmPolicy, register_policy
from stable_baselines.common import make_vec_env
from saveBestTrainingCallback import SaveOnBestTrainingRewardCallback


def make_env():
    def maker():
        env = gym.make('CartPole-v1')
        return env
    return maker

neural_network = [100,100]
class CustomLSTMPolicy(LstmPolicy):
    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm=50, reuse=False, **_kwargs):
        super().__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm, reuse,
                          net_arch=[0, 'lstm', dict(vf=neural_network, pi=neural_network)],
                          layer_norm=False, feature_extraction="mlp", **_kwargs)

register_policy('CustomPolicy', CustomLSTMPolicy)

# %%Run Custom Policy

# Inserting callback
mainDirectory = str(pathlib.Path(__file__).parent.absolute())
check_freq = 5000
savefile = mainDirectory
num_ep_save = 5
callback = SaveOnBestTrainingRewardCallback(check_freq = check_freq, log_dir = savefile, num_ep_save = num_ep_save) 


n_training_envs = 1
# envs = DummyVecEnv([make_env() for _ in range(n_training_envs)]) # Works
envs = make_vec_env(make_env(), n_envs=n_training_envs, monitor_dir = savefile)# Trying to see if this works

model = PPO2('CustomPolicy', envs, nminibatches=1, verbose=1) # We are trying to see if this one works.
model.learn(100_000, callback=callback)
model.save(mainDirectory+'\\customPolicyAgent')

obs = envs.reset()
# Passing state=None to the predict function means
# it is the initial state
state = None
# When using VecEnv, done is a vector
done = [False] # We are trying to see if this one works
for _ in range(1000):
    # We need to pass the previous state and a mask for recurrent policies
    # to reset lstm state when a new episode begin
    action, state = model.predict(obs, state=state, mask=done)
    obs, reward , done, _ = envs.step(action)
    # Note: with VecEnv, env.reset() is automatically called

    # Show the env
    envs.render()

envs.close()