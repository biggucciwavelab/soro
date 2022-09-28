# -*- coding: utf-8 -*-
"""
Created on Thu Mar  5 22:20:55 2020

@author: elopez8
"""
# In[Import Dependencies]
import os
import pathlib
from shutil import copyfile
import matplotlib.pyplot as plt
import numpy as np

#Strings Environment
import JAMoEBA_TrainingEnvironment as Strings
from utils.conversion import Convert
from JAMoEBA_TrainingEnvironment import createVideo
from saveBestTrainingCallback import SaveOnBestTrainingRewardCallback

#Stable Baselines Dependences
from stable_baselines.common.policies import register_policy, LstmPolicy
from stable_baselines import PPO2
from stable_baselines.common import make_vec_env
from stable_baselines.bench import Monitor
from datetime import date
import time
import pdb


if __name__ == '__main__':
    # This dictionary correlates how large the system to the maximum number of timesteps it will be given to reach its target
    botTimestepDict = {#3:15000,
                        # 10:2000,
                        # 15:15000,
                        # 20:3000,
                        # 25:4000,
                         30:50_000
                        }
    
    botPPMdict = {3:500,
                  # 10:150,
                  # 15:100,
                  # 20:75,
                  # 25:75,
                  30:55}

    
    botTrainingSteps = {3:20_000_000,
                      # 15:20_000_000,
                      # 30:30_000_000}
                      30:30_000_000}


    mainDirectory = str(pathlib.Path(__file__).parent.absolute())
    experimentNum=229
    for numBots in botTimestepDict:
    # In[Prepare Experiment Parameters]
        
        # Ensuring we are in the proper directory
        os.chdir(mainDirectory)
        experimentName = 'Experiment_{}'.format(experimentNum)
        
        # Training Parameters
        neural_network =[1000,1000]
        numLSTMnodes = 64
        policyName = 'CustomPolicy_'+str(experimentNum)
        gamma = 0.99            # Discount factor
        n_steps = 500          # Number of steps to run in each environment per update. Batchsize = n_steps*n_env
        ent_coef = 0.01         # Entropy coefficient
        learning_rate = 0.00025 # Learning Rate, can be a funcion
        vf_coef = .5             # Value Function Coefficient in Loss Function
        max_grad_norm = 0.5     # Clipping factor for gradients. Should prevent exploding gradients
        lam = 0.95              # Factor for bias vs variance for GAE
        nminibatches = 1       # Number of minibatches at each update. Thus the minibatchsize = batchsize//minibatches
        noptepochs = 4          # Number of epochs each updaet
        cliprange = 0.2         # Cliprange for PPO
        seed = 12345            # Seed for neural network initialization
        nEnvs = 1               # Number of parallel environments
        
        # Environment Parameters
        dt = 1/200.0 # Simulation timestep
        numStepsPerStep = 100
        numBots = numBots
        botMass = .02
        botRadius = .019 #**
        skinRadius = .015 #**
        skinMass = botMass
        skinRatio=2
        inRadius = botRadius #**
        botFriction = .5
        inMass = .002
        inFriction = .5
        percentInteriorRemove = 0
        springK = 50 #**
        springB = .1
        springRL = 0 #**
        wallThickness = botRadius/2 #**
        maxSeparation = inRadius*1.5 #**
        obstaclesPresent = True
        grabObjectPresent = False
        potFieldPresent = False
        tunnel = False
        energy = False
        
        #Defining system Radius
        startDistance = skinRadius # The start distance between bots
        arcLength = 2*botRadius+skinRatio*(2*skinRadius)+(skinRatio+1)*startDistance
        theta = 2*np.pi/numBots
        R = arcLength/theta
    
        # Target distance from X-start location
        # targetDistance = R*72 # Unit: m
        targetDistance = R*72
    
        # Screen parameters (Taken from my big screen (; )
        width = 3096
        height = 1296
        maxNumSteps = botTimestepDict[numBots]
        ppm = botPPMdict[numBots] # Pixels Per Meter
        
        # Move bot to another location
        convert = Convert(botPPMdict[numBots])
        # systemStart = convert.Pixels2Meters(width/4), convert.Pixels2Meters(height/2)
        # targetLoc = systemStart[0]+R*36, systemStart[1]
        systemStart = targetLoc = None
        
        # All together now!
        envParams = {'dt':dt,
                 'numStepsPerStep':numStepsPerStep,
                 'ppm':ppm,
                 'screenHeight':height,
                 'screenWidth':width,
                 'maxNumSteps':maxNumSteps,
                 'R':R,
                 'numBots':numBots,
                 'botMass':botMass,
                 'botRadius':botRadius,
                 'skinRadius':skinRadius,
                 'skinMass':skinMass,
                 'skinRatio':skinRatio,
                 'inRadius':inRadius,
                 'botFriction':botFriction,
                 'inMass':inMass,
                 'inFriction':inFriction,
                 'percentInteriorRemove':percentInteriorRemove,
                 'springK':springK,
                 'springB':springB,
                 'springRL':springRL,
                 'wallThickness':wallThickness,
                 'maxSeparation':maxSeparation,
                 'targetDistance':targetDistance,
                 'obstaclesPresent':obstaclesPresent,
                 'grabObjectPresent':grabObjectPresent,
                 'potFieldPresent':potFieldPresent,
                 'tunnel':tunnel,
                 'energy':energy,
                 'targetLoc':targetLoc,
                 'systemStart':systemStart
                 }
        
        # Parameters for Calback
        num_ep_save = 2 # Calculate the mean reward for this number of episodes and save that model
        check_freq = 50000 # After how many timesteps do we check the frequency
        
        training_timesteps = botTrainingSteps[numBots]
        
        #Test after training
        # TODO: Integrate post training testing
        test=True
        num_tests=3
        render=True
        time_per_test=maxNumSteps
            
        #Save location
        savefile = mainDirectory + '\\Experiment {} {}\\'.format(str(experimentNum), date.today())
        os.makedirs(savefile, exist_ok=True)
        
        # In[Create Tensorboard Directories, Neural Network and Train]

        # Custom LSTM policy
        class CustomLSTMPolicy(LstmPolicy):
            def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm=numLSTMnodes, reuse=False, **_kwargs):
                super().__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, n_lstm, reuse,
                                  net_arch=[0, 'lstm', dict(vf=neural_network, pi=neural_network)],
                                  layer_norm=False, feature_extraction="mlp", **_kwargs)
        
        register_policy(policyName, CustomLSTMPolicy)
     
        # Setting up Callback
        callback = SaveOnBestTrainingRewardCallback(check_freq = check_freq, log_dir = savefile, num_ep_save = num_ep_save) 
        
         # Creating multiple environments for multiple workers
        env = make_vec_env(Strings.pymunkEnv, n_envs= nEnvs, env_kwargs=envParams, monitor_dir = savefile)
        copyfile(Strings.__file__,savefile+'Strings_Environment_Experiment_{}.py'.format(str(experimentNum)))
    
        # Create and pretrain the model with expert trajectories
        model = PPO2(policyName, env, verbose=1, 
                      gamma=gamma, 
                      n_steps = n_steps, 
                      ent_coef = ent_coef,
                      learning_rate = learning_rate,
                      vf_coef = vf_coef,
                      nminibatches = nminibatches,
                      noptepochs = noptepochs,
                      cliprange = cliprange,
                      tensorboard_log = savefile,
                      seed = seed)
        # model = PPO2.load('ppo2_Experiment_193.zip',env,tensorboard_log=savefile,verbose=1)
        
        # Saving model and training parameters
        RL_Training_Parameters=[['Experiment Name:',experimentName],
                        ['Neural Network:',str(neural_network)],
                        ['Training Timesteps:',str(training_timesteps)],
                        ['Timesteps Per Environmant Per Update:', str(n_steps)],
                        ['Num of Minibatches:',str(nminibatches)],
                        ['Epochs Per Update:',str(noptepochs)],
                        ['Gamma:',str(gamma)],
                        ['Entropy Coefficient:',str(ent_coef)],
                        ['Learning Rate:', str(learning_rate)],
                        ['Value Function Coeff:',str(vf_coef)],
                        # ['Time for Learning:',str(round(learn_end - learn_start,2))],
                        ['Seed:',str(seed)],
                        ['Network Type:','LSTM']]
                        
        txt_file = savefile+'RL Training Paremeters.txt'
        with open(txt_file, 'w') as f:
            for line in RL_Training_Parameters:
                f.write("%s\n" % line)
        
        txtFile2 = savefile+'Environment Parameters for Experiment {}.txt'.format(experimentNum)
        envParamsSave = []
        for key in envParams:
            envParamsSave.append([key+':',envParams[key]])
        with open(txtFile2,'w') as file:
            for line in envParamsSave:
                file.write("%s\n" % line)
        
        learn_start = time.time()
        model.learn(total_timesteps=training_timesteps, callback=callback)
        learn_end = time.time()
        
        model.save(savefile+'ppo2_LSTM_' + experimentName)
        env.close()
        
        del model
        del env
        
        # %%Model Evaluation - Test the Agent
        """
        Currently only works when testing with one made model. Not ready for multiple ANN tests.
        """
        if test:
            os.chdir(savefile)
        
            model = PPO2.load('ppo2_LSTM_' + experimentName+'.zip')
            envParams['dataCollect']=True
            envParams['saveVideo']=True
            
            for i in range(num_tests):
                name=experimentName+'_v{}'.format(str(i+1))
                envParams['experimentName']=savefile + name
                # env = make_vec_env(Strings.pymunkEnv, n_envs= nEnvs, env_kwargs=envParams, monitor_dir=savefile)
                env = Strings.pymunkEnv(**envParams)
                obs=env.reset()
                l = obs.shape[0]
                obs = obs.reshape((1,l))
                
                done=[False]
                state = None
                for j in range(time_per_test):
                    action, state = model.predict(obs, state=state, mask=done)
                    obs, reward, done, info = env.step(action[0])                
                    if render: env.render()
                    if done: break
                    obs = obs.reshape((1,l))
                    done = [done]
            
                env.dataExport() # Export the data from the simulation
                plt.close('all')
                createVideo(savefile, env.videoFolder,name, (width, height))
                env.close()
            
            experimentNum += 1