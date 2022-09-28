"""
Script which executes animation
"""
#%%
# Execute

# Data Locations:
# data_loc = "Obstacle_Field/Stars/AllDataRuns_v1 Data and Plots 2022-02-10 150040/" # Obstacle Field
# data_loc = "Obstacle_Field/Stars/AllDataTunnelRuns_v1 Data and Plots 2022-02-08 201220/" # Obstacle Field Tunneling
# data_loc = "Obstacle_Field/Tunnel/AllDataRuns_v4 Data and Plots 2022-02-09 160340/" # Tunneling - Maze Navigation
data_loc = "Grabbing/AllDataRuns_v1 Data and Plots 2022-02-21 145931/" # Grabbing

# Creating a master location to save everything
save_loc = "Plots/"

from animation_objects import *

#video_name = "Rew Test Check Animation"

# In[Grasping]
time = 50
phi = GrabbingDataManager(data_loc)
# phi.create_animation(video_name="grasping")
name=save_loc + f"Grasping Transform/grasping {time}"
entry=[time]
phi.create_motion_snaps(entry, name, axis_off=True)


# In[Obstacle Field]
phi = ObstacleFieldDataManager(
    data_loc,
    # For zoom up. Otherwise, only set wxmin=0 
    wxmin=1.6,
    wxmax=2.4,
    wymin=4.3,
    wymax=5.1
)
name=save_loc + "Obstacle Movement Springs Back/Plot"
#phi.create_animation(video_name=video_name)
# entry=[0,1000,2000,2200,4000]
entry = [1000] # For zoom up
phi.create_motion_snaps_SYSTEMONLY(entry, name, axis_off=True)

# In[Tunnel]
# name = save_loc + "Tunnel 2/Tunnel"
# phi=TunnelDataManager(data_loc)
# entry=[0,60,120,140]
# phi.create_motion_snaps(entry, name, axis_off=True)


# In[Narrow Cooridor]
# name = save_loc + "Cooridor 3/Cooridor"
# phi=CooridorDataManager(data_loc, wxmax=3.5)
# entry=[0,300,900]
# phi.create_motion_snaps(entry, name, axis_off=True)
