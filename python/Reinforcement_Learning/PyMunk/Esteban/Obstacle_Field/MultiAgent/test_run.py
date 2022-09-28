"""
Code taken directly from the following location:
https://github.com/Farama-Foundation/PettingZoo/blob/master/tutorials/13_lines.py
"""
# %%

# Importing environment
from star_env_params import *

# Import RL 
from star_RL_params import *

if __name__=='__main__':
    freeze_support()

    # Ensure no recording of video or data occurs right now
    envParams['dataCollect'] = False
    envParams['saveVideo'] = False

    # Creating environment
    env = parallel_env(**envParams)
    env = ss.frame_stack_v1(env, frame_stack)
    env = ss.pettingzoo_env_to_vec_env_v1(env)
    env = ss.concat_vec_envs_v1(env, 1, num_cpus=1, base_class='stable_baselines3')
    print('\nCurrently only works with concat_vec_envs.num_cpus=1. Must look into if we can still use PPO multiprocessing.\n')
    
    # Locating center string
    mainDirectory = str(pathlib.Path(__file__).parent.absolute()) # Get the path of this file
    savefile = mainDirectory + '\\Experiment {} {}\\'.format(str(experimentNum), date.today())

    # Setting up checekpoint callback
    checkpoint_callback = CheckpointCallback(save_freq=check_freq, save_path = savefile, name_prefix = 'model_checkpoint') 

    # Creating model and training
    model = PPO('MlpPolicy', env, verbose=1, policy_kwargs=policy_kwargs,
                    gamma=gamma, 
                    n_steps = n_steps, 
                    ent_coef = ent_coef,
                    learning_rate = learning_rate,
                    vf_coef = vf_coef,
                    batch_size = batch_size,
                    n_epochs = noptepochs,
                    clip_range = cliprange,
                    tensorboard_log = savefile,
                    seed = seed)

    # model.learn(total_timesteps=10_000, callback=checkpoint_callback)
    savename = "{}policy".format(savefile)
    model.save(savename)

    del model
    del env

    # Now we practice loading and executing the policy
    model = PPO.load(savename)
    env = parallel_env(**envParams)
    env = ss.frame_stack_v1(env, 3)

    observations = env.reset()
    for i in range(1000):
        # Get the actions for each agent
        actions = {}
        for agent in env.agents:
            obs = observations[agent]
            act = model.predict(obs, deterministic=True)[0]
            actions[agent] = act

        observations, rewards, dones, infos = env.step(actions)
        env.render()
    env.close()