"""
This script exists to get the username
of person executing this script. 

If you are attempting to use your program in a new
machine or on a new virtual environment, you will have to edit the parameters defined here.
"""
# %%
# Execute
import sys
try:
    import getpass
except:
    sys.exit('Unable to locate module getpass')

user = getpass.getuser()

all_users = [
    'elope',
    'dmulr']

assert user in all_users, "The username of this machine has not been included."

main_directory_dict = dict(
    elope = "Experiments/",
    dmulr = "F:/Soro_chrono/python/PyBullet/Strings/Experiments/"
)

pybullet_data_dict = dict(
    elope = "C:/Users/elope/anaconda3/Lib/site-packages/bullet3-master/data/",
    dmulr = "C:/Users/dmulr/anaconda3/Lib/site-packages/bullet3-master/data/"
)