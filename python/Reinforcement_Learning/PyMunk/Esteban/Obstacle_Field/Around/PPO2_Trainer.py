# %% Main cell
"""
Main execution script for training the grabbing environment
"""

if __name__ == '__main__':
    # Importing Environment and environment dependencies
    from star_env_params import *

    # Importing RL parameters and dependencies
    from star_RL_params import *

    # Creating directory to save all results in
    mainDirectory = str(pathlib.Path(__file__).parent.absolute()) # Get the path of this file
    savefile = mainDirectory + '\\Experiment {} {}\\'.format(str(experimentNum), date.today())
    os.makedirs(savefile, exist_ok=True)
    
    # Importing and saving these files
    import star_env_params, star_RL_params
    copyfile(master_env.__file__,savefile+'Environment_Experiment_{}.py'.format(str(experimentNum)))
    copyfile(star_RL_params.__file__,savefile+'Experiment_{}_RL_params.py'.format(str(experimentNum)))
    copyfile(star_env_params.__file__,savefile+'Experiment_{}_env_params.py'.format(str(experimentNum)))
   
    # Setting up Callback
    checkpoint_callback = CheckpointCallback(save_freq=check_freq, save_path = savefile, name_prefix = 'model_checkpoint') 

    # Assert that we are not gathering simulation information during training
    envParams['dataCollect']=False
    envParams['saveVideo']=False

    # Creating multiple environments for multiple workers
    env = make_vec_env(pymunkEnv, n_envs= nEnvs, env_kwargs=envParams, vec_env_cls=SubprocVecEnv, monitor_dir = savefile)

    # Create the model
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

    # Load model here if needed
    # model = PPO2.load('INSERT MODEL NAME.zip', env, tensorboard_log=savefile, verbose=1)
    
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
    if test:
        os.chdir(savefile)
    
        model = PPO.load(experimentName+'_agent.zip')
        envParams['dataCollect']=True
        envParams['saveVideo']=True
        
        for i in tqdm(range(num_tests)):
            name = experimentName+'_v{}'.format(str(i+1))
            envParams['experimentName']=savefile + name
            Env=pymunkEnv(**envParams)
            obs=Env.reset()
            
            for j in range(time_per_test):

                action, _states = model.predict(obs, deterministic=True)
                obs, reward, done, info = Env.step(action)

                if render: 
                    Env.render()
                    
                if done:
                    break
        
            Env.dataExport() # Export the data from the simulation
            plt.close('all')
            createVideo(savefile,Env.videoFolder,name, (width, height))
            Env.close()