# -*- coding: utf-8 -*-
"""
Created on Fri Nov 27 16:38:32 2020

@author: elope

Refer to the following for information:
    https://stable-baselines.readthedocs.io/en/master/guide/custom_policy.html
    
    
Also taking a suggestion from:
    https://github.com/hill-a/stable-baselines/issues/1063
    https://github.com/hill-a/stable-baselines/issues/1015
"""

import gym
import numpy as np
import pdb

from stable_baselines.common.policies import FeedForwardPolicy, register_policy, LstmPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines.bench import Monitor
from stable_baselines.common import make_vec_env
# from Strings_Environment import Strings
from stable_baselines import PPO2

if __name__=='__main__':

    # Custom Environment
    # kwargs = {'mem':2, 'gain':2}
    # env = make_vec_env(Strings, n_envs= 4, env_kwargs=kwargs)
    # env = SubprocVecEnv([lambda : env_id for i in range(num_cpu)])
    
    # Gym environment for testing
    n_envs = 4
    envSingle = gym.make('CartPole-v1')
    # env = make_vec_env('CartPole-v1', n_envs = n_envs, vec_env_cls=SubprocVecEnv)
    
    # Algorithm parameters
    neural_network= [100,100,100]
    
    # Custom LSTM policy
    class CustomLSTMPolicy(LstmPolicy):
        def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm=64, reuse=False, **_kwargs):
            super().__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm, reuse,
                              net_arch=[0, 'lstm', dict(vf=neural_network, pi=neural_network)],
                              layer_norm=False, feature_extraction="mlp", **_kwargs)
    
    # class CustomLSTMPolicy(LstmPolicy):
    #     def __init__(self, n_lstm=64, reuse=False, *args, **kwargs):
    #         super().__init__(net_arch=[8, 'lstm', dict(vf=neural_network, pi=neural_network)],
    #                          layer_norm=True, feature_extraction="mlp", *args, **kwargs)
    
    # Register the policy, it will check that the name is not already taken
    register_policy('CustomPolicy2', CustomLSTMPolicy)
    
    # Because the policy is now registered, you can pass
    # a string to the agent constructor instead of passing a class
    model = PPO2(policy='CustomPolicy2', env=envSingle, verbose=1, noptepochs=20, nminibatches=1, n_steps=400)
    model.learn(total_timesteps=100_000)
    
    obs = envSingle.reset()
    state=None
    # zeroPaddedObs = np.zeros((n_envs,) + envSingle.observation_space.shape)
    # zeroPaddedObs[0,:] = obs
    for i in range(200):
        # action, state = model.predict(zeroPaddedObs, state=state)
        action, state = model.predict(obs, state=state)
        print(state)
        obs, rewards, dones, info = envSingle.step(action)
        print(action[0])
        envSingle.render()
        # zeroPaddedObs[0,:] = obs
        if i%20==0: print(i)
    
    envSingle.close()