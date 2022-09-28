# MADDPG Libraries
from maddpg.trainer.maddpg import MADDPGAgentTrainer
import maddpg.common.tf_util as U

# Tensorflow requirements
import tensorflow.contrib.layers as layers
import tensorflow as tf

# Standard libraries
import time
import numpy as np
import argparse
import pickle
import os


def mlp_model(input, num_outputs, scope, reuse=False, num_units=64, rnn_cell=None):
    # This model takes as input an observation and returns values of all actions
    with tf.variable_scope(scope, reuse=reuse):
        out = input
        out = layers.fully_connected(out, num_outputs=num_units, activation_fn=tf.nn.relu)
        out = layers.fully_connected(out, num_outputs=num_units, activation_fn=tf.nn.relu)
        out = layers.fully_connected(out, num_outputs=num_outputs, activation_fn=None)
        return out




def get_trainers(env, num_adversaries, obs_shape_n, arglist):
    trainers = []
    model = mlp_model
    trainer = MADDPGAgentTrainer
    for i in range(num_adversaries):
        trainers.append(trainer(
            "agent_%d" % i, model, obs_shape_n, env.action_space, i, arglist,
            local_q_func=(arglist.adv_policy=='ddpg')))
    for i in range(num_adversaries, env.n):
        trainers.append(trainer(
            "agent_%d" % i, model, obs_shape_n, env.action_space, i, arglist,
            local_q_func=(arglist.good_policy=='ddpg')))
    return trainers




def train(arglist):
    with U.single_threaded_session(): # Runs the tensorflow graph on a single CPU
        # Create environment
        env = arglist.env
        # Create agent trainers
        obs_shape_n = [env.observation_space[i].shape for i in range(env.n)]
        num_adversaries = 0
        trainers = get_trainers(env, num_adversaries, obs_shape_n, arglist) # Returns a list of MADDPG agents for training
        print('Using good policy {} and adv policy {}'.format(arglist.good_policy, arglist.adv_policy))

        # Initialize
        U.initialize()

        # Load previous results, if necessary
        if arglist.load_dir == "":
            arglist.load_dir = arglist.save_dir
        if arglist.display or arglist.restore or arglist.benchmark:
            print('Loading previous state...')
            try:
                U.load_state(arglist.load_dir)
            except:
                print('Failed to load previously trained parametrers. Is this your first time training?')
                print('If so, you cannot display, restore, or benchmark the environment! ')

        episode_rewards = [0.0]  # sum of rewards for all agents
        agent_rewards = [[0.0] for _ in range(env.n)]  # individual agent reward
        final_ep_rewards = []  # sum of rewards for training curve
        final_ep_ag_rewards = []  # agent rewards for training curve
        agent_info = [[[]]]  # placeholder for benchmarking info
        saver = tf.train.Saver()
        obs_n = env.reset()
        episode_step = 0
        train_step = 0
        t_start = time.time()

        print('Starting iterations...')
        while True:
            # get action
            action_n = [agent.action(obs) for agent, obs in zip(trainers,obs_n)]
            # environment step
            new_obs_n, rew_n, done_n, info_n = env.step(action_n)
            episode_step += 1
            done = all(done_n)
            terminal = (episode_step >= arglist.max_episode_len)
            # collect experience
            for i, agent in enumerate(trainers):
                agent.experience(obs_n[i], action_n[i], rew_n[i], new_obs_n[i], done_n[i], terminal)
            obs_n = new_obs_n

            for i, rew in enumerate(rew_n):
                episode_rewards[-1] += rew
                agent_rewards[i][-1] += rew

            if done or terminal:
                obs_n = env.reset()
                episode_step = 0
                episode_rewards.append(0)
                for a in agent_rewards:
                    a.append(0)
                agent_info.append([[]])

            # increment global step counter
            train_step += 1

            # for benchmarking learned policies
            if arglist.benchmark:
                for i, info in enumerate(info_n):
                    agent_info[-1][i].append(info_n['n'])
                if train_step > arglist.benchmark_iters and (done or terminal):
                    file_name = arglist.benchmark_dir + arglist.exp_name + '.pkl'
                    print('Finished benchmarking, now saving...')
                    with open(file_name, 'wb') as fp:
                        pickle.dump(agent_info[:-1], fp)
                    break
                continue

            # for displaying learned policies
            if arglist.display:
                time.sleep(0.01)
                env.render()
                continue

            # update all trainers, if not in display or benchmark mode
            loss = None
            for agent in trainers:
                agent.preupdate()
            for agent in trainers:
                loss = agent.update(trainers, train_step)

            # Save model, display training output
            if terminal and (len(episode_rewards) % arglist.save_rate == 0):
                U.save_state(arglist.save_dir, saver=saver)
                # print statement depends on whether or not there are adversaries
                if num_adversaries == 0:
                    print("Steps: {}, Episodes: {}, Mean episode reward: {}, Time: {}".format(
                        train_step, len(episode_rewards), np.mean(episode_rewards[-arglist.save_rate:]), round(time.time()-t_start, 3)))
                else:
                    print("Steps: {}, Episodes: {}, Mean episode reward: {}, Agent episode reward: {}, Time: {}".format(
                        train_step, len(episode_rewards), np.mean(episode_rewards[-arglist.save_rate:]),
                        [np.mean(rew[-arglist.save_rate:]) for rew in agent_rewards], round(time.time()-t_start, 3)))
                t_start = time.time()
                # Keep track of final episode reward
                final_ep_rewards.append(np.mean(episode_rewards[-arglist.save_rate:]))
                for rew in agent_rewards:
                    final_ep_ag_rewards.append(np.mean(rew[-arglist.save_rate:]))

            # Saves all training incremenets rewards for plotting reward curve afterwards.
            if len(episode_rewards) > arglist.num_episodes:
                # Saving system rewards
                rew_file_name = arglist.plots_dir + arglist.exp_name + '_rewards'
                os.makedirs(os.path.dirname(rew_file_name), exist_ok=True)
                np.save(rew_file_name, final_ep_rewards)
                
                agrew_file_name = arglist.plots_dir + arglist.exp_name + '_agrewards'
                os.makedirs(os.path.dirname(agrew_file_name), exist_ok=True)
                np.save(agrew_file_name, final_ep_ag_rewards)
                print('...Finished total of {} episodes.'.format(len(episode_rewards)))
                break
                





def get_args(
    # Environment arguments
    env,                    # The environment itself. This MUST be passed       

    max_episode_len = 25,   # Maximum episode length
    num_episodes = 60_000,  # Number of episodes to train over
    good_policy = 'maddpg', # Policy for good agents
    adv_policy = 'maddpg',  # Policy of adversaries

    # Training parameters
    lr = 1e-2,              # Learning rate for Adam optimizer
    gamma = 0.95,           # Discount Factor
    batch_size = 1024,      # Number of episodes to optimize at the same time
    num_units = 64,         # Number of units in the mlp

    # Checkpointing
    exp_name = None,            # Name of the experiment
    save_dir = '/tmp/policy/',  # Directory in which training state and model should be saved
    save_rate = 1_000,          # Save model once every time this many episodes are completed
    load_dir = "",              # Directoy in which trainined and state model are loaded

    #Evaluation
    restore = False,
    display=False,
    benchmark = False,
    benchmark_iters = 100_000,              # Number of iterations run for benchmarking
    benchmark_dir = './benchmark_files/',   # Directory where benchmark data is saved
    plots_dir = './learning_curves/'        # Directory where plot data is saved
):
    """
    This function is replacing the argparser as originally implemented in 'train.py'

    It returns a dictionary where elemnts can be called as methods (i.e. 'dict.key = value')
    """
    print('Reinforcement Learning experiments for multiagent environments')

    argsdict = {
    'env' : env,

    'max_episode_len' : max_episode_len,  # Maximum episode length
    'num_episodes' : num_episodes,        # Number of episodes to train over
    'good_policy' : good_policy,          # Policy for good agents
    'adv_policy' : adv_policy,            # Policy of adversaries

    # Training parameters
    'lr' : lr,                  # Learning rate for Adam optimizer
    'gamma' : gamma,            # Discount Factor
    'batch_size' : batch_size,  # Number of episodes to optimize at the same time
    'num_units' : num_units,    # Number of units in the mlp

    # Checkpointing
    'exp_name' : exp_name,      # Name of the experiment
    'save_dir' : save_dir,      # Directory in which training state and model should be saved
    'save_rate' : save_rate,    # Save model once every time this many episodes are completed
    'load_dir' : load_dir,      # Directoy in which trainined and state model are loaded

    #Evaluation
    'restore' : restore,
    'display' : display,
    'benchmark' : benchmark,
    'benchmark_iters' : benchmark_iters, # Number of iterations run for benchmarking
    'benchmark_dir' : benchmark_dir,     # Directory where benchmark data is saved
    'plots_dir' : plots_dir              # Directory where plot data is saved
    }

    class AttrDict(dict):
        def __init__(self, *args, **kwargs):
            super(AttrDict, self).__init__(*args, **kwargs)
            self.__dict__ = self

    return AttrDict(argsdict)
