"""
This script will be used to execute a pretraining in a variety of scenarios.

Parameters regarding the pretraining should be prepared in the scripts:
    - env_params.py
    - RL_params_tf1.py
"""

if __name__ == '__main__':
    # Importing Environment and environment dependencies
    from env_params import *

    # Importing RL parameters and dependencies
    from RL_params_tf1 import *

    # Importing the control law
    from Control import Control as control

    # Creating directory to save all results in
    mainDirectory = str(pathlib.Path(__file__).parent.absolute()) # Get the path of this file
    savefile = mainDirectory + '\\Pretraining Experiment {} {}\\'.format(str(experimentNum), date.today())
    os.makedirs(savefile, exist_ok=True)
    
    # Importing and saving these files
    import env_params, RL_params_tf1, Control
    copyfile(master_env.__file__,savefile+'Environment.py')
    copyfile(RL_params_tf1.__file__,savefile+'RL_params_tf1.py')
    copyfile(env_params.__file__,savefile+'env_params.py')
    copyfile(Control.__file__, savefile+'Controller.py')
    copyfile(__file__, savefile+'Trainer.py')

    # Create the environment
    # Must verify that we are not rendering during the creation of expert trajectories
    envParams['saveVideo'] = False
    envParams['dataCollect'] = False
    env = pymunkEnv(**envParams)

    # Create the controller needed for the system
    controller = control(R)
    controller = controller.control_10

    #----------------
    # Create an environment just to show how testing works
    print('Testing controller before training...\n')
    envParams_testing = envParams
    envParams_testing['saveVideo'] = True
    envParams_testing['dataCollect'] = True
    
    for i in range(3):
        plt.close('all')
        name = 'Controller Sample v{}'.format(i+1)
        envParams_testing['experimentName'] = savefile + name
        env_testing = pymunkEnv(**envParams_testing)
        env_testing.report_all_data = True

        obs = env_testing.reset()
        for t in tqdm(range(time_per_test)):
            action = controller(obs)
            obs, reward, done, info = env_testing.step(action)
            env_testing.render()

            if done:
                break

        env_testing.dataExport()
        env_testing.exportAllData()
        createVideo(env_testing.saveFolder, env_testing.videoFolder, name, (width,height))
        env_testing.close()

    #--------------
    # Create the expert trajectories
    print('Creating expert trajectories...\n')
    from stable_baselines.gail import generate_expert_traj, ExpertDataset
    expert_traj_file = savefile + 'Expert Trajectory'
    generate_expert_traj(controller, expert_traj_file, env, 10_000, 10)
    print('Expert trajectories created.\n')


    #---------------
    # Now we create the model and pretrain it

    print('Creating the model...\n')
    model = PPO2('MlpPolicy', env, verbose=1,
                gamma=gamma, 
                n_steps = n_steps, 
                ent_coef = ent_coef,
                learning_rate = learning_rate,
                vf_coef = vf_coef,
                nminibatches = nminibatches,
                noptepochs = noptepochs,
                cliprange = cliprange,
                tensorboard_log = savefile,
                seed = seed,
                policy_kwargs=policy_kwargs)

    print('Pretraining the model...')
    dataset = ExpertDataset(expert_traj_file+'.npz', batch_size = pretrain_batchsize)
    model.pretrain(dataset, n_epochs = n_epochs)
    model.save(savefile + 'Pretrained Model')
    print('Pretraining Complete.\n')


    #---------
    # Test the pretraing model
    print('Testing pretrained model...')
    for i in range(3):
        plt.close('all')
        name = 'Pretrained Model v{}'.format(i+1)
        envParams_testing['experimentName'] = savefile + name
        env_testing = pymunkEnv(**envParams_testing)
        env_testing.report_all_data = True

        obs = env_testing.reset()
        for t in tqdm(range(time_per_test)):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env_testing.step(action)
            env_testing.render()

            if done:
                break

        env_testing.dataExport()
        env_testing.exportAllData()
        createVideo(env_testing.saveFolder, env_testing.videoFolder, name, (width,height))
        env_testing.close()

    print('Completed.')

    print('Retesting with new systemStart location...')
    # Generating the new starts...
    systemStart = env_testing.systemStart
    starts = []
    for index in range(1,4):
        thisStart = systemStart[0], systemStart[1] + R*index
        starts.append(thisStart)
    
    # Adding one more where the system is started forward a little bit
    thisStart = systemStart[0] + R*4, systemStart[1]
    starts.append(thisStart)

    for i in range(4):
        plt.close('all')
        name = 'Pretrained Model v{}'.format(i+1)
        envParams_testing['experimentName'] = savefile + name
        env_testing = pymunkEnv(**envParams_testing)
        env_testing.report_all_data = True
        env_testing.systemStart = starts[i]

        obs = env_testing.reset()
        for t in tqdm(range(time_per_test)):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env_testing.step(action)
            env_testing.render()

            if done:
                break

        env_testing.dataExport()
        env_testing.exportAllData()
        createVideo(env_testing.saveFolder, env_testing.videoFolder, name, (width,height))
        env_testing.close()