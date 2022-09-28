"""
Original idea:
https://stackoverflow.com/questions/42355122/can-i-export-a-tensorflow-summary-to-csv#:~:text=Just%20check%20the%20%22Data%20download,appear%20under%20your%20scalar%20summary.
"""

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import os
import numpy as np
import pandas as pd
from tqdm import tqdm
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = "serif"
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

def smooth(scalars: list, weight: float) -> list:  # Weight between 0 and 1
    #https://stackoverflow.com/questions/42281844/what-is-the-mathematics-behind-the-smoothing-parameter-in-tensorboards-scalar
    last = scalars[0]  # First value in the plot (first timestep)
    smoothed = list()
    for point in scalars:
        smoothed_val = last * weight + (1 - weight) * point  # Calculate smoothed value
        smoothed.append(smoothed_val)                        # Save it
        last = smoothed_val                                  # Anchor the last smoothed value

    return smoothed


def plot_data(timestep, reg_data, smoothed_data, save_loc, title):
    """
    Plots the data and saves
    """
    fig, ax = plt.subplots()
    ax.plot(timestep, reg_data, color='darkorange', alpha = .5)
    ax.plot(timestep, smoothed_data, color='darkorange', alpha=1)
    ax.set_xlabel('Timestep')
    ax.set_ylabel('Value')
    ax.set_title(title)
    ax.ticklabel_format(useOffset=False)
    plt.savefig(save_loc + title)
    plt.close(fig)

def tabulate_events(dpath):

    # Smoothing factor, as we will be saving both regular and smoothed data
    smoothing_factor = .9
    txt_smooth_factor = str(smoothing_factor)
    txt_smooth_factor = txt_smooth_factor.replace('.','0')

    for dname in os.listdir(dpath):
        print(f"Converting run {dname}",end="\n\n")
        print('Loading the tensorboard data...',end='')
        file = os.path.join(dpath, dname)
        ea = EventAccumulator(file).Reload()
        print('Complete.')
        tags = ea.Tags()['scalars']

        print('Iterating through data and saving...')
        for tag in tqdm(tags):
            tag_values=[]
            wall_time=[]
            steps=[]

            for event in tqdm(ea.Scalars(tag), leave=False, desc='Scalars for Tag {}'.format(tag)):
                tag_values.append(event.value)
                wall_time.append(event.wall_time)
                steps.append(event.step)
            
            # Create the dataframe and export it
            data = dict(
                step = steps,
                wall_time = wall_time,
                value = tag_values
            )
            df = pd.DataFrame(data)
            title = os.path.join(dpath,'{}.csv'.format(tag))
            dirname = os.path.dirname(title)
            os.makedirs(dirname, exist_ok=True)
            df.to_csv(title)

            # We will smooth the data as well, and save that as well.
            smoothed_values = smooth(tag_values, smoothing_factor)
            smoothed_data = dict(
                step = steps,
                wall_time = wall_time,
                smoothed_value = smoothed_values
            )
            smoothed_df = pd.DataFrame(smoothed_data)
            title = os.path.join(dpath,'{}_{}_Smoothed.csv'.format(tag, txt_smooth_factor))
            dirname = os.path.dirname(title)
            os.makedirs(dirname, exist_ok=True)
            smoothed_df.to_csv(title)


            # Plot the resultant curves
            plot_path = dpath + 'Plots/'; os.makedirs(plot_path, exist_ok=True)
            plot_title = os.path.basename(os.path.basename(tag))
            plot_data(steps, tag_values, smoothed_values, plot_path, plot_title)



if __name__=="__main__":
    dpath = "PPO2_1/"
    tabulate_events(dpath)