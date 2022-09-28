# %% 
# Main cell
"""
Main execution script for training the grabbing environment
"""

if __name__ == '__main__':
    # Importing Environment and environment dependencies
    from env_params import *

    # Importing RL parameters and dependencies
    from RL_params import *

    # Creating directory to save all results in
    mainDirectory = str(pathlib.Path(__file__).parent.absolute()) # Get the path of this file
    savefile = mainDirectory + '\\Experiment {} {}\\'.format(str(experimentNum), date.today())
    os.makedirs(savefile, exist_ok=True)
    
    # Importing and saving these files
    import env_params, RL_params
    copyfile(master_env.__file__,savefile+'Environment_.py')
    copyfile(RL_params.__file__,savefile+'RL_params.py')
    copyfile(env_params.__file__,savefile+'env_params.py')
    copyfile(__file__, savefile+'Trainer.py')
   
    # Setting up Callback
    checkpoint_callback = CheckpointCallback(save_freq=check_freq, save_path = savefile, name_prefix = 'model_checkpoint') 

    # Assert that we are not gathering simulation information during training
    envParams['dataCollect']=False
    envParams['saveVideo']=False

    # Create the environment
    env = parallel_env(**envParams)
    env = ss.frame_stack_v1(env, frame_stack)
    env = ss.pettingzoo_env_to_vec_env_v1(env)
    env = ss.concat_vec_envs_v1(env, 1, num_cpus=1, base_class='stable_baselines3')

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
    env.reset()
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
        
        for i in range(num_tests):
            name = experimentName+'_v{}'.format(str(i+1))
            envParams['experimentName']=savefile + name

            # Environment must be prepared as it previously was
            Env=parallel_env(**envParams)
            Env = ss.frame_stack_v1(Env, 3)
            observation = Env.reset()
            
            for i in tqdm(range(time_per_test)):
                actions = {}
                for agent in Env.agents:
                    obs = observation[agent]
                    action, _states = model.predict(obs, deterministic=True)
                    actions[agent] = action
                observation, _, dones, _= Env.step(actions)

                if render: 
                    Env.render()
                    
                if dones[agent]:
                    break
            
            # We have to unwrap the environment to access data
            Env = Env.unwrapped

            Env.dataExport() # Export the data from the simulation
            plt.close('all')
            createVideo(savefile,Env.videoFolder,name, (width, height))
            Env.close()