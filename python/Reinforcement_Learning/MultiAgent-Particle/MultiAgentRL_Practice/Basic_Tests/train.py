# %% Main Cell

from training_funcs import *

if __name__ == '__main__':

    exp_name = 'Test_3'

    training_kwargs=dict(
        scenario = 'simple_push',
        num_adversaries = 1, # Must match what is true for the environment
        exp_name = exp_name,
        display = False,
        num_episodes = 10_000,
        save_dir = f'./{exp_name}/' 
    )

    arglist = get_args(**training_kwargs)
    rew = train(arglist)