# -*- coding: utf-8 -*-
"""
@author: elopez8
"""

if __name__ == '__main__':
    # %%Import Dependenciesq    
    import os
    import site
    from datetime import date 
    from shutil import copyfile
    import matplotlib.pyplot as plt
    import pdb
    
    """
    To avoid having to copy and paste the desired environment into the site packages everytime there is a modeifcation made, the below process ensures that the environment of interest is copied directly into the site-packages directory before import.
    """
    import JAMoEBA_TrainingEnvironment as Strings1
    import saveBestTrainingCallback as SavingCallback
    os.makedirs(site.getsitepackages()[1]+'\\ELRL\\', exist_ok=True)
    copyfile(Strings1.__file__,site.getsitepackages()[1]+'\\ELRL\\Strings_Environment.py')
    copyfile(SavingCallback.__file__, site.getsitepackages()[1]+'\\ELRL\\saveBestTrainingCallback.py')
    
    #Strings Environment
    import ELRL.Strings_Environment as Strings
    
    # Callback to save the best learned model in the background
    from stable_baselines.bench import Monitor
    from stable_baselines.common.vec_env import SubprocVecEnv
    from ELRL.saveBestTrainingCallback import SaveOnBestTrainingRewardCallback
    
    #Stable Baselines Dependences
    from stable_baselines.common.policies import FeedForwardPolicy, register_policy
    from stable_baselines import PPO1
    from stable_baselines.gail import ExpertDataset
    
    # %%Prepare Experiment Parameters
    
    experiment_num=163
    
    #Pre-Training batchsize
    pretrain_batchsize=200
    pretrain_epochs = 1000 # Number of epochs for pretraining
    
    # Training Parameters
    # nns=[[500,250,100,60]]
    nns=[[2000,2000,1000,500,250,60]]
    policy_names=['Policy_1'] # There must be a name for each NN
    training_timesteps = 2000000
    timesteps_per_actorbatch = 20000
    gamma = 1 # Discount Factor
    batchsize = 500
    epochs = 4
    entcoeff = 0
    schedule='linear'
    seed=12345
    
    # Environment Parameters
    gain=2
    membrane=2
    experiment_name = "Experiment_{}".format(str(experiment_num))
    experiments = [experiment_name+'']
    
    # Parameters for Calback
    num_ep_save = 2 # Calculate the mean reward for this number of episodes and save that model
    check_freq = 50000 # After how many timesteps do we check the frequency
    
    pretraining = False
    pretrain2 = 'None' # Even if pretraining is FALSE, there still must be a string here
    if pretraining:
        # Pretrain location
        #Esteban's Desktop
        # pretrain = 'C:/Users/elope/Documents/Repositories/Soro-Chrono/python/Reinforcement_Learning/RL JAMoEBA - Strings/Expert Trajectory/'
        
        # Big Gucci
        pretrain = 'C:/soro_bitbucket/python/Reinforcement_Learning/RL JAMoEBA - Strings/Expert Trajectory/'
        pretrain += pretrain2
        dataset = ExpertDataset(pretrain, traj_limitation=1, batch_size=pretrain_batchsize)
    
    #Save location
    import pathlib
    current = str(pathlib.Path(__file__).parent.absolute())
    savefile=current+'\\Experiment {} {}\\'.format(str(experiment_num), date.today())
    os.makedirs(savefile, exist_ok=True)
    
    #Test after training
    test=True
    num_tests=3
    data_collect=True
    plot=True
    render=True
    POV_Ray=True
    time_per_test=20000
    
    # %%Create Tensorboard Directories, Neural Network and Train
    
    # If you have more than 1 experiment being conducted now, this will run a for loop through them all
    for i in range(len(experiments)):     
        tensorboard_log = savefile + experiments[i] + " Tensorboard Log/"
        os.makedirs(tensorboard_log,exist_ok=True)
            
        # Custom MLP policy of three layers of size 'n' each
        class CustomPolicy(FeedForwardPolicy):
            def __init__(self, *args, **kwargs):
                super(CustomPolicy, self).__init__(*args, **kwargs,
                                                    net_arch=[dict(pi=nns[i],
                                                                  vf=nns[i])],
                                                    feature_extraction="mlp")
        
        # Register the policy, it will check that the name is not already taken
        register_policy(policy_names[i], CustomPolicy)
        
        # For callback
        env = Monitor(Strings.Strings(experiment_name=savefile + experiments[i], mem=membrane, gain=gain), savefile)
        callback = SaveOnBestTrainingRewardCallback(check_freq = check_freq, log_dir = savefile, num_ep_save = num_ep_save)
        
        new_env='Strings_Environment_Experiment_{}'.format(str(experiment_num))
        copyfile(Strings.__file__,savefile+new_env+'.py')
        # Create and pretrain the model with expert trajectories
        model = PPO1(policy_names[i], env, verbose=1, optim_batchsize=batchsize, tensorboard_log=tensorboard_log, gamma=gamma, timesteps_per_actorbatch=timesteps_per_actorbatch, seed=seed, optim_epochs=epochs, entcoeff=entcoeff, schedule=schedule)
        
        if pretraining:
            model.pretrain(dataset, n_epochs=pretrain_epochs)
        """
        # Use this when trying to continue training a previously trained model
        model_loc='C:/Users/elope/Documents/Repositories/Soro-Chrono/python/Reinforcement_Learning/RL JAMoEBA - Strings/Experiment 130 2020-11-30/ppo1_Strings_CustomPolicy_Experiment_130.zip'
        model = PPO1.load(model_loc, env)
        """
        
        model.learn(total_timesteps=training_timesteps, callback=callback)
        model.save(savefile+'ppo1_Strings_CustomPolicy_' + experiments[i])
        
        env.parameter_export()
        env.close()
        
        RL_Training_Parameters=[['Experiment Name:',experiments[i]],
                                ['Neural Network:',str(nns[i])],
                                ['Training Timesteps:',str(training_timesteps)],
                                ['Timesteps Per Actor Update:', str(timesteps_per_actorbatch)],
                                ['Batchsize Per Update:',str(batchsize)],
                                ['Epochs Per Update:',str(epochs)],
                                ['Gamma:',str(gamma)],
                                ['Pretrain Batchsize:',str(pretrain_batchsize)],
                                ['Pretrain Epochs:',str(pretrain_epochs)],
                                ['Entropy Coefficient:',str(entcoeff)],
                                ['Learning Rate Scheduler:', schedule],
                                ['Seed:',str(seed)],
                                ['Pretraining Present?:',str(pretraining)],
                                ['Pretraning npz:',pretrain2]]
        
        txt_file=savefile+'RL Training Paremeters {}.txt'.format(str(i))
        with open(txt_file, 'w') as f:
            for line in RL_Training_Parameters:
                f.write("%s\n" % line)
        
        del model
        del env
        
    # %%Model Evaluation - Test the Agent
    """
    Currently only works when testing with one made model. Not ready for multiple ANN tests.
    """
    if test:
        os.chdir(savefile)
        Env_import=__import__(new_env)
        model = PPO1.load(savefile+'ppo1_Strings_CustomPolicy_' + experiment_name+'.zip')
        
        for i in range(num_tests):
            name=experiment_name+'_v{}'.format(str(i+1))
            Env=Env_import.Strings(data_collect=data_collect, experiment_name=name, plot=plot, POV_Ray = POV_Ray, mem=membrane, gain=gain)
            obs=Env.reset()
            
            for j in range(time_per_test):
                action, _states = model.predict(obs)
                obs, reward, done, info = Env.step(action)
            
                if render: Env.render()
                
                if i%10==0: print('Time Step:',j)
                    
                if done: break
        
            if data_collect: Env.data_export() # Export the data from the simulation
            plt.close('all')
            Env.close()