"""
This script will do the following:
    1) Create expert datasets from the CNN environment 
    2) Store the expert dataset
    3) Create functions for training the model
    4) Create a model using a custom CNN
    5) Train the created model
"""

# %%
#### 1) Creating expert Dataset
#______________________________________________

from grabbing_env_params_CNN import *
import time

# Create the environment
env = pymunkEnv(**envParams)
basename = 'Experiment_Data_Run_{}'

# Number of interactions to have with the environment
num_interactions = 50_000
expert_observations = np.empty((num_interactions,) + env.observation_space.shape)
expert_actions = np.empty((num_interactions,) + env.action_space.shape)

from gym import spaces
totalR = R + botRadius
wallEnd = totalR + env.wallThickness
# Use this code IF the box is started on the right end of the environment and we wish to learn how to grab it there.
#_________________________________________________
# Start Comment

# Creating a box in which the start location of the system can be placed
startLocs = [spaces.Box( # This is good IF the box is started near the wall
    low = np.array([env.wallThickness + totalR, env.wallThickness + totalR]),
    high = np.array([env.width - totalR - env.squareLength, env.height - totalR])
)]

# Starting the system above or below the box to learn those areas as well
startLocsNear = [
    [env.width - totalR - env.wallThickness, env.height/4],  # Below the box
    [env.width - totalR - env.wallThickness, 3*env.height/4] # Above the box
]

start_near = .5 # What percentage of iterations should start above and below the system for grabbing.
start_near_iter = num_interactions * (1-start_near)
num_boxes=1
# Stop Comment here
#________________________________________________
"""
# Use this code if the environment has the object starting out on the right end of the environment
#_________________________________________________
# Start Comment
startLocs = [
    spaces.Box(low = np.array([wallEnd, wallEnd]), high = np.array([env.width/2 - (env.squareLength + totalR), env.height - wallEnd])), # Box 1
    spaces.Box(low = np.array([(env.width/2) + env.squareLength + totalR, wallEnd]), high = np.array([env.width - wallEnd, env.height-wallEnd])), # Box 2
    spaces.Box(low = np.array([wallEnd, (env.height/2 + env.squareLength + totalR)]), high = np.array([env.width - wallEnd, env.height - wallEnd])), # Box 3
    spaces.Box(low = np.array([wallEnd, wallEnd]), high = np.array([env.width - wallEnd, (env.height/2) - (env.squareLength + totalR)]))
]
start_near_iter = np.inf
num_boxes = 4
# Stop Comment here
#________________________________________________
"""

# Creating first folder and env for collecting data
del env
envParams['dataCollect'] = False
envParams['experimentName'] = basename.format(iter)
env = pymunkEnv(**envParams)
obs = env.reset()

# Create the controller
# Must be done for each new environment initiation
pot_field = create_pot_field(env)
noise=False
control = controller(pot_field, noise=noise) # Noise dictates whether noise will be added to the potential field
start_time = time.time()
for i in tqdm(range(num_interactions)):

    if render:
        env.render()
    action = control.get_action(obs)

    # Store the observation-action pair
    expert_observations[i] = obs
    expert_actions[i] = action

    # Step the environment
    obs, rew, done, info = env.step(action)

    if done:
        if env.dataCollect:
            env.dataExport()
            plt.close('all')
        del env
        envParams['experimentName'] = basename.format(iter)
        env = pymunkEnv(**envParams)

        pot_field = create_pot_field(env)
        control = controller(pot_field, noise = noise)

        if i >= start_near_iter:
            which = np.random.randint(0,2)
            newStartingLoc = startLocsNear[which]
        else:
            whichBox = np.random.randint(0,num_boxes)
            newStartingLoc = startLocs[whichBox].sample()
        obs = env.reset(newStartingLoc)

end_time = time.time()

if env.dataCollect:
    env.dataExport()
    plt.close('all')
env.close()

#### 2) Store the expert dataset
#______________________________________________

from torch.utils.data.dataset import Dataset, random_split
class ExpertDataSet(Dataset):
    def __init__(self, expert_observations, expert_actions):
        self.observations = expert_observations
        self.actions = expert_actions

    def __getitem__(self, index):
        return (self.observations[index], self.actions[index])

    def __len__(self):
        return len(self.observations)

expert_dataset = ExpertDataSet(expert_observations, expert_actions)
train_size = int(0.8 * len(expert_dataset))
test_size = len(expert_dataset) - train_size

train_expert_dataset, test_expert_dataset = random_split(
                                                        expert_dataset,
                                                        [train_size, test_size]
)


#### 3) Creating functions for pretraining the network
#______________________________________________
import torch as th
import torch.nn as nn
import gym
from stable_baselines3 import PPO, A2C
import torch.optim as optim
from torch.optim.lr_scheduler import StepLR

def pretrain_agent(
    student,
    batch_size = 64,
    epochs = 1_000,
    scheduler_gamma = 0.7,
    learning_rate = 1.0,
    log_interval=100,
    no_cuda=True,
    seed=1,
    test_batch_size=64
):
    use_cuda= not no_cuda and th.cuda.is_available() # If Cuda is available, will train via cuda
    th.manual_seed(seed)
    device = th.device("cuda" if use_cuda else "cpu")
    kwargs = {
        "num_workers":1,
        "pin_memory":True} if use_cuda else {}
        
    if isinstance(env.action_space, gym.spaces.Box):
        criterion = nn.MSELoss()
    else:
        criterion = nn.CrossEntropyLoss()

    # Extract initial policy
    model = student.policy.to(device)


    def train(model, device, train_loader, optimizer):
        model.train()

        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.to(device), target.to(device)
            optimizer.zero_grad()

            if isinstance(env.action_space, gym.spaces.Box):
                # A2C/PPO policy outputs actions, values, log_prob
                if isinstance(student, (A2C, PPO)):
                    action, _, _ = model(data) # Note that data = obs
                else:
                    # SAC3/TD3
                    action = model(data)
                action_prediction = action.double()
            else:
                # Retrieve the logits for A2C/PPO when using discrete actions
                latent_pi, _, _ = model._get_latent(data)
                logits = model.action_net(latent_pi)
                action_prediction = logits
                target = target.long()

            loss = criterion(action_prediction, target)
            loss.backward()
            optimizer.step()
            if batch_idx % log_interval == 0:
                print(
                    "Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}".format(
                    epoch,
                    batch_idx * len(data),
                    len(train_loader.dataset),
                    100.0 * batch_idx / len(train_loader),
                    loss.item(),
                    )
                )


    def test(model, device, test_loader):
        model.eval()
        test_loss = 0
        with th.no_grad():
            for data, target in test_loader:
                data, target = data.to(device), target.to(device)
                    
                if isinstance(env.action_space, gym.spaces.Box):
                    # A2C/PPO policy outputs actions, values, log_prob
                    # SAC/TD3 policy outputs actions only
                    if isinstance(student, (A2C, PPO)):
                        action, _, _ = model(data)
                    else:
                        # SAC/TD3:
                        action = model(data)
                    
                    action_prediction = action.double()
                
                else:
                    # Retrieve the logits for A2C/PPO when using discrete actions
                    latent_pi, _, _ = model._get_latent(data)
                    logits = model.action_net(latent_pi)
                    action_prediction = logits
                    target = target.long()

                test_loss = criterion(action_prediction, target)
        test_loss /= len(test_loader.dataset)
        print(f"test set: Average Loss: {test_loss:.4f}")


    # using PyTorch's 'DataLoader' to load previously created 'ExpertDataset' for training
    train_loader = th.utils.data.DataLoader(
        dataset=train_expert_dataset, batch_size=batch_size, shuffle=True, **kwargs
    )
    test_loader = th.utils.data.DataLoader(
        dataset=test_expert_dataset, batch_size=test_batch_size, shuffle=True, **kwargs
    )

    # Define an optimizer and a learning rate schedule
    optimizer = optim.Adadelta(model.parameters(), lr=learning_rate)
    scheduler = StepLR(optimizer, step_size=1, gamma=scheduler_gamma)

    for epoch in range(1, epochs+1):
        train(model, device, train_loader, optimizer)
        test(model, device, test_loader)
        scheduler.step()

    try:
        student.policy = model
    except:
        print('\n\nThe student must be a stable-baselines3 model!')


# %%
#### 7) Create MLP policy
#______________________________________________
"""
Create and train an MLP Model using the expert data set
"""

mlp_policy_kwargs = dict(
    net_arch = [dict(pi=[500,400,300,200,100], vf=[500,400,300,200,100])]
)

student_mlp =  PPO(
    'MlpPolicy',
    env,
    policy_kwargs = mlp_policy_kwargs,
    verbose=1,
    gamma=0.99,
    n_steps=1000,
    ent_coef=0.0,
    learning_rate=0.00025,
    vf_coef=0.0,
    batch_size=100,
    n_epochs=4,
    clip_range=0.2,
    seed=12345
)

pretrain_agent(
    student_mlp,
    epochs=7,
    scheduler_gamma=0.7,
    learning_rate=1.0,
    log_interval=100,
    no_cuda=True,
    seed=1,
    batch_size=64,
    test_batch_size=100,
)
#%%
# Run Without pretraining.
# This code be done if the model has already been created and pretrained.
whichBox = np.random.randint(0,num_boxes)
newStartingLoc = startLocs[whichBox].sample()
obs = env.reset(newStartingLoc)

for i in tqdm(range(3000)):
    env.render()

    action, _states = student_mlp.predict(obs, deterministic=True)
    obs, _, done, _ = env.step(action)
    
    if done:
        whichBox = np.random.randint(0,num_boxes)
        newStartingLoc = startLocs[whichBox].sample()
        obs = env.reset(newStartingLoc)


env.close()