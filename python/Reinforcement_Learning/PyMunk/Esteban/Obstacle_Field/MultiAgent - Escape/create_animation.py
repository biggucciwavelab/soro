"""
Script which executes animation
"""
#%%
# Execute

# Data Location:
data_loc = "Data Collection Test Data and Plots 2022-04-26 113558/" # Grabbing

# Creating a master location to save everything
save_loc = "Plots/"

from animation_objects import *

wxmin = 0
wxmax = 5.5
wymin = 1
wymax = 5
phi = TunnelDataManager(
    data_loc,
    # For zoom up. Otherwise, only set wxmin=0 
    wxmin=wxmin,
    wxmax=wxmax,
    wymin=wymin,
    wymax=wymax
)
entry = [0]
name=save_loc + f"Snapshot {entry} Min [{wxmin},{wymin}] Max [{wxmax},{wymax}]"
phi.create_motion_snaps(entry, name, axis_off=False)