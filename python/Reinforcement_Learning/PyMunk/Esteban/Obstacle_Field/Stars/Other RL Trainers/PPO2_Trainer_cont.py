# %% Main cell
"""
Main execution script for training the grabbing environment.

Use this script when loading a model that you wish to CONTINUE training on, as opposed to starting a training from scratch.
"""
import pathlib
from shutil import copyfile
from datetime import date
import time
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3 import PPO
import sys
sys.path.append('../..')
from utils.utils import save_runtime


if __name__ == '__main__':
    """
    Change these variables
    """
    import cont_training
    from cont_training.Experiment_6_env_params import *
    from cont_training import Experiment_6_env_params, Environment_Experiment_6
    star_env_params, Stars_Environment = Experiment_6_env_params, Environment_Experiment_6
    old_model = 'cont_training/Experiment_6_agent.zip'
    experimentNum = "7" # The new experiment number
    training_timesteps = 10_000_000
    nEnvs = 4
    check_freq = 50_000

    # Creating directory to save all results in
    mainDirectory = str(pathlib.Path(__file__).parent.absolute()) # Get the path of this file
    savefile = mainDirectory + '\\Experiment {} {}\\'.format(str(experimentNum), date.today())
    os.makedirs(savefile, exist_ok=True)
    
    # Importing and saving these files
    copyfile(Stars_Environment.__file__,savefile+'Environment_Experiment_{}.py'.format(str(experimentNum)))
    copyfile(star_env_params.__file__,savefile+'Experiment_{}_env_params.py'.format(str(experimentNum)))

    # Setting up Callback
    checkpoint_callback = CheckpointCallback(save_freq=check_freq, save_path = savefile, name_prefix = 'model_checkpoint') 

    # Assert that we are not gathering simulation information during training
    envParams['dataCollect']=False
    envParams['saveVideo']=False

    # Creating multiple environments for multiple workers
    env = make_vec_env(pymunkEnv, n_envs=nEnvs, env_kwargs=envParams, vec_env_cls=SubprocVecEnv, monitor_dir = savefile)

    # Loading the old model
    model = PPO.load(
        old_model,
        env=env,
        tensorboard_log = savefile
    )

    # Train the model
    learn_start = time.time()
    model.learn(total_timesteps=training_timesteps, callback=checkpoint_callback)
    learn_end = time.time()
    
    # Save information on how long training took
    runtime = learn_end - learn_start
    save_runtime(savefile, 'Training_Time', runtime)
    
    # Save the model
    model.save(savefile + experimentName + '_agent')

    # Close the environment
    env.close()

    # Delete the model and environment
    # This is done to verify that, if there are multiple trainings being done on a computer, 
    # that the correct agent and environment are paired.
    del model
    del env
    
    # Model Evaluation
    # We will now test the agent!
    os.chdir(savefile)

    model = PPO.load(experimentName+'_agent.zip')
    envParams['dataCollect'] = True
    envParams['saveVideo'] = True
    
    for i in tqdm(range(3)):
        name = experimentName+'_v{}'.format(str(i+1))
        envParams['experimentName']=savefile + name
        Env=pymunkEnv(**envParams)
        obs=Env.reset()
        
        for j in range(maxNumSteps):
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = Env.step(action)
        
            if render: 
                Env.render()
                
            if done: 
                break
        
        if Env.dataCollect:
            Env.dataExport() # Export the data from the simulation
            plt.close('all')
        if Env.saveVideo:
            createVideo(savefile,Env.videoFolder,name, (width, height))
        Env.close()