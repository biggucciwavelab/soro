# %%
# Train the system

from training_funcs import *
from star_env_params import *
from Stars_Environment import pymunkEnv


if __name__ == '__main__':

    env = pymunkEnv(**envParams)

    training_kwargs=dict(
        env = env,
        exp_name = experimentName,
        display = False,
        num_episodes = 10_000,
        max_episode_len = 1_000,
        save_rate=1_000, # The rate at which information on policy will be saved. 
        num_units = 500,
        save_dir = f'./{experimentName}/' 
    )

    arglist = get_args(**training_kwargs)
    train(arglist)