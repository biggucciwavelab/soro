# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 19:56:00 2020

@author: dmulr
"""
import numpy as np
import Strings_objects_jumbo as sim_obj
from scipy.interpolate import interp1d
from datetime import datetime
now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
# In[Variables for all bots]
### General parameters ###
visual="pov" # visualization
sim= dt_string# sim number 
fixed=False # is it fixed
obj=[] # empty file to ave all objects
data_path="F:/data/" # data file for other visuals

##### control type #####
control_type="shape_form"

# Control type:
'''
'pot_field_grab': grab an object
"tunneling" tunneling 
'grab_drag': grab and drag an object 
"shape_form": Form a shape
"shape_formation": maintain a formation 
"local_field": local pot fields
"verify":
"move right": 
'''
##### interior modes #####
mode='nmax'

# Interior generation modes:
'''
'empty':Nothing inside
'nmax' :customize the ring sizes
'nonhnmax'  max number of interiors with different diamters
'''
##### time #####
tstep=.001   # time step
tend=5 # time end

##### Friction #####
mu_f=.1  # friction
mu_b=.01    # dampning
mu_r=.01     # rolling friction
mu_s=.01    # SPinning fiction

##### compliance #####
Ct=.0001 # tangent compliane
C=.0001 # compliance
Cr=.0001 # rolling compliance
Cs=.0001 # sliding compliance

##### Robot #####       
mr=.2       # mass
nb=1000 #number of robots
height=.06  # height of cylinder
radius=.038 # diameter of cylinder and robots
volume=np.pi*height*(radius)**2   # calculate volume
rowr=mr/volume # calculate density of robot
skind=.02
rationM=3
#R=(2*diameter*nb/(np.pi*2))+.26
R=(skind*rationM)/(np.sin(np.pi/nb))
### Geometry of robot ###
geom="cylinder"

'''
square: Robots will be in a square shape
cylinder: Robots will be a cylinder
shere: robots will be a sphere
'''

##### Interior particles #####
mp=.03 # mass 
#radius2=.25 # diameter of cylinder and robots
radius2=.25
volume2=np.pi*height*(radius2)**2   # calculate volume
rowp=mp/volume2 # density

##### Spring ##### 
k=200  # spring constant (bots)
rl=0 # resting length
type_spring='var'

##### Floor #####
length=40 # Length of the body floor
tall=1     # height of the body floor


##### pwm #####
pwm=255# 0-255
w=5 # freqency
tn=(pwm/255)/w  # pulse length so out of .2 seconds it runs and applies for for tn seconds 

# In[move right]
if control_type=="move_right":
    phi=None        # shape set to none since were not forming a desired shape 
    gapw=0
    env_mode=None
    xc=0            # center of robot x
    zc=0            # center of robot z
    n=np.arange(nb-710,30,-10) # interior arrangement
    tpull=0
    args=[]
# In[Pot field grab]    
if control_type=="grab_drag": 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    field="Analytic" # field type
    alpha=1          # controller gain
    beta=0           # damping term
    b=9              # range of field 
    Rd=.05           # radius of zero contour
    px=0             # ball location x
    py=1            # ball location y
    phi=sim_obj.potential_fields(field,px,py,b,region_shape=None,R=Rd,r1=None,r2=None)
    
    ##### Ball variables #####
    mb=3 # mass of ball
    Rb=2*Rd # radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=px # center of ball x
    zball=py # center of ball y
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    n=np.arange(nb,5,-6) # interior arrangement
    num=3
    #### arguements #####
    args=(phi,alpha,beta,b,Rd,num)
    
# In[pot field grab]    
if control_type=="pot_field_grab": 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    field="Analytic" # field type
    alpha=3          # controller gain
    beta=0           # damping term
    b=9              # range of field 
    Rd=.1            # radius of zero contour
    px=0             # ball location x
    py=1            # ball location y
    phi=sim_obj.potential_fields(field,px,py,b,region_shape=None,R=Rd,r1=None,r2=None)
    
    ##### Ball variables #####
    mb=3 # mass of ball
    Rb=2*Rd # radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=px # center of ball x
    zball=py # center of ball y
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    n=np.arange(nb+5,5,-6) # interior arrangement
    #### arguements #####
    args=(phi,alpha,beta,b,Rd)
# In[Shape formation]
if control_type=="shape_form": 
    tpull=20        # time to pull omn the object
    xc=0        # center of robot x
    zc=0         # center of robot z
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    field='analytic_square' # field type
    alpha=25   # controller gain
    beta=2           # damping term
    b=R+1             # range of field 
    Rd=.1            # radius of zero contour
    px=0             # ball location x
    py=0           # ball location y
    r2=(3*R)            # outer radius
    r1=r2-.05            # inner radius
    # potential field
    phi=sim_obj.potential_fields(field,px,py,b,region_shape="Square",R=None,r1=r1,r2=r2)    
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    #n=np.arange(nb-770,5,-9) # interior arrangement
    n=np.arange(nb-770,5,-12)
    #n=np.arange(nb-400,5,-20)
    #n=np.arange(nb+40,5,-6)
    #### arguements #####
    args=(phi,alpha,beta,b,Rd)    

# In[Shape formation]
if control_type=="moving_shape_form": 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    field="region" # field type
    alpha=1     # controller gain
    beta=0           # damping term
    b=1              # range of field 
    Rd=.1            # radius of zero contour
    px=0             # ball location x
    py=0           # ball location y
    r2=1.5*(R-.1)            # outer radius
    r1=r2-.05            # inner radius
    # potential field
    phi=sim_obj.potential_fields(field,px,py,b,region_shape="Circle",R=None,r1=r1,r2=r2)    
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    n=np.arange(nb+5,6,-5)  # interior rings
    #### arguements #####
    args=(phi,alpha,beta,b,Rd)



# In[Tunnel]
if control_type=="tunnel": 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    field="point_field" # field type
    alpha=1          # controller gain
    beta=0           # damping term
    b=10              # range of field 
    Rd=.1            # radius of zero contour
    px=0             # goal location x
    py=5            # goal location y
    # potential field
    phi=sim_obj.potential_fields(field,px,py,b,region_shape=None,R=None,r1=None,r2=None)    
    gapw=.5 # size of gap if tunneling
    env_mode='tunnel' # create tunnel 
    n=np.arange(nb+5,5,-6)  # interior rings
    #### arguements #####
    args=(phi,alpha,beta,b,Rd)      
    
# In[save variables] 
position=False # save bot position
velocity=False# save bot velocity
forces=False   # save bot forces
control_force=False # save all contact forces
contact_positions=False

# interior particle postions, velocity and forces
if mode!='empty':
    particle_position=False
    particle_vel=False
    particle_force=False
else:
    particle_position=False
    particle_vel=False
    particle_force=False

# ball information
if control_type=='pot_field_grab':
    ball_data=False
else:
    ball_data=False

    


if control_type=='shape_form':
    shape=True
    error=True
else:
    shape=False
    error=False
save_data=[position,velocity,forces,control_force,contact_positions,particle_position,particle_vel,particle_force,ball_data,error,shape]        