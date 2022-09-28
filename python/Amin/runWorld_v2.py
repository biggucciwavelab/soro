# %% Main Cell

from weg_world_params_v2 import *
import matplotlib.pyplot as plt
from controllers import *
import utils as ut

# Create simulation
env = robot(**envKwargs)

# Initialize simulation
observation = env.reset()

# Define what action we will take at every timestep. As of now, we are applying 0 force.
action = np.ones((numBots, 2)) * 0

dir_max_av = np.array([0, -1.])

dt_cont = env.dt * 2
controller = WegController(numBots, env.dt, dt_cont, 25 * numBots / numBots , slip_per=0.25)

#plotter = Statistics(numBots, env.dt, 0.01, 200)
#plotter.startPlot()

#plotter = Statistics(numBots)
#plotter.startPlot()

# Run simulation
for i in tqdm(range(maxNumSteps)):

    action = controller.updateState(observation, i)

    #plotter.updateData(observation, i)

    if i%5 == 0:
        #plotter.updatePlot()
        pass

    observation, _ = env.step(action)
    # xy eac

if dataCollect:
    env.dataExport()
#if saveVideo:
#    ut.createVideo(env.saveFolder, env.videoFolder, experimentName, (width, height), 1 / (dt * numStepsPerStep))
    #createVideo('./', env.videoFolder, experimentName, (width, height))

env.close()

