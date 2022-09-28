# %% Main Cell

from weg_world_params import *
from controllers import *
import matplotlib.pyplot as plt
import utils as ut

# Create simulation
env = robot(**envKwargs)

# Initialize simulation
observation = env.reset()

#Define what action we will take at every timestep. As of now, we are applying 0 force.
action = np.ones(numBots)*1

dir_max_av = np.array([0, -1.])

controller = AminController(numBots, 0.1)

#plotter = Statistics(numBots)
#plotter.startPlot()

# Run simulation
for i in tqdm(range(maxNumSteps)):

    if i%10 == 0:
        controller.updateSpins()

    action = controller.getActions(observation)



    #plotter.updateData(observation)

    #if i%500 == 0:
    #    plotter.updatePlot()


    observation, _ = env.step(action)
    # xy eac



if dataCollect:
    env.dataExport()
if saveVideo:
    #createVideo('./', env.videoFolder, experimentName, (width, height))
    ut.createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height), 1 / (dt * numStepsPerStep))

env.close()

