# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 13:33:58 2021

Taken from: https://github.com/hill-a/stable-baselines/issues/166

In order to train an LSTM policy with multiple policies then train on a single env, the obs.space must be padded with zeros
"""


import gym
import numpy as np

from stable_baselines import PPO2
from stable_baselines.common.policies import LstmPolicy, register_policy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines.common import make_vec_env

#### Make the custom policy

if __name__=='__main__':
    
    neural_network = [100,100,100]
    class CustomLSTMPolicy(LstmPolicy):
        def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm=64, reuse=False, **_kwargs):
            super().__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm, reuse,
                              net_arch=[0, 'lstm', dict(vf=neural_network, pi=neural_network)],
                              layer_norm=False, feature_extraction="mlp", **_kwargs)
    
    register_policy('CustomPolicy', CustomLSTMPolicy)
    
    def make_env():
       def maker():
           env = gym.make("CartPole-v1")
           return env
       return maker
    
    # Train with 2 envs
    n_training_envs = 2
    envs = DummyVecEnv([make_env() for _ in range(n_training_envs)])
    # envs = make_vec_env(make_env(), n_envs=n_training_envs, vec_env_cls=SubprocVecEnv) # Currently commenting out the making of SubProvVecEnv
    model = PPO2("CustomPolicy", envs, nminibatches=2)
    model.learn(total_timesteps=10_000)
    
    # Create one env for testing
    test_env = DummyVecEnv([make_env() for _ in range(1)])
    test_obs = test_env.reset()
    
    # model.predict(test_obs) would through an error
    # because the number of test env is different from the number of training env
    # so we need to complete the observation with zeroes
    zero_completed_obs = np.zeros((n_training_envs,) + envs.observation_space.shape)
    zero_completed_obs[0, :] = test_obs
    
    # IMPORTANT: with recurrent policies, don't forget the state
    state = None
    
    for _ in range(1000):
        action, state = model.predict(zero_completed_obs, state=state)
        # The test env is expecting only one action
        new_obs, reward, done, info = test_env.step([action[0]])
        # Update the obs
        zero_completed_obs[0, :] = new_obs