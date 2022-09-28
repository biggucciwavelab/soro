# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 19:56:00 2020

@author: dmulr
"""
# import libraries
import numpy as np
import Strings_objects as sim_obj
from scipy.interpolate import interp1d
from datetime import datetime
import os
import sys


# timer (used to time how long the config file takes)
now = datetime.now()
# create name of sim based on  (day, month, year hour minute second)
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")

# In[Variables for all bots]
### General parameters ###
visual="irr" # visualization
sim= dt_string# sim number 
fixed=False # is it fixed
obj=[] # empty file to ave all objects
data_path="F:/data/" # data file for other visuals
#data_path="F:/data"

# if not os.path.exists(sim):
#     os.mkdir(sim)



##### control type #####
control_type="target_verify"
env_mode = None # create tunnel 

# Control type:
'''
"move_right" 
"shape_form"
"shape_form_analytic"
"pot_field_grab"
"image_warp"
"tunnel"
"import_field"
"Complex_image_warp"
Obstacle
import_field_letters
R-fields
"GRASP"
Verify
tunnel_verify
grasp_verify
target_verify
'''



##### interior modes #####
mode="target_verify"

# Interior generation modes:
'''
'empty':Nothing inside
'nmax' :customize the ring sizes
'nonhnmax'  max number of interiors with different diamters
anger
verify
target_verify
tunnel_verify
grasp_verify
'''


##### time #####
tstep=.001  # time step
tend=151 #time end

##### Friction #####
mu_f=.4# friction
mu_b=.01    # dampning
mu_r=.1     # rolling friction
mu_s=.1  # SPinning fiction

##### compliance #####
Ct=.00001 # tangent compliane
C=.000001 # compliance
Cr=.00001 # rolling compliance
Cs=.00001 # sliding compliance

##### Robot #####       
mr=.3       # mass
nb=12 #number of robots
height=.1  # height of cylinder
radius=.05 # diameter of cylinder and robots
volume=np.pi*height*(radius)**2   # calculate volume
rowr=mr/volume # calculate density of robot
skind=.03 # diamter of memebrane particles
rationM=4# how many membrane particles between each bot
#R=(2*diameter*nb/(np.pi*2))+.26
R=(skind*(rationM))/(np.sin(np.pi/nb))
#=(.18)/(np.sin(np.pi/nb))

#R=.51

rationM=4
### Geometry of robot ###
geom="cylinder"

'''
square: Robots will be in a square shape
cylinder: Robots will be a cylinder
shere: robots will be a sphere
'''

##### Interior particles #####

mp=.02 # mass 
#radius2=.75 # diameter of cylinder and robots
radius2=.04
volume2=np.pi*height*(radius2)**2   # calculate volume
rowp=mp/volume2 # density



#(n,Area)=sim_obj.MaxValues(R-17.5*radius2,radius,radius2,nb,mode)
#(n,Area)=sim_obj.MaxValues(R-3.5*radius2,radius,radius2,nb,mode)
(n,Area)=sim_obj.MaxValues(R,radius,radius2,nb,mode)
R = R
##### Spring ##### 
k=100# spring constant (bots)
rl=0 # resting length
type_spring='var'

##### Floor #####
length=1000 # Length of the body floor
tall=1     # height of the body floor


##### pwm #####
pwm=255# 0-255
w=5 # freqency
tn=(pwm/255)/w  # pulse length so out of .2 seconds it runs and applies for for tn seconds 

# In[move right]
''' This is the variables needed for making the robot move to the right with constant force'''
if control_type=="move_right":
    phi=None        # shape set to none since were not forming a desired shape 
    gapw=0          # theres no tunnel so no need of gap width
    env_mode=None   # no tunnel so no cooridor 
    xc=0            # center of robot x
    zc=0            # center of robot z
    n=np.arange(nb-710,30,-10) # interior arrangement
    tpull=0         # set to zero meanign there isnt any grabbing and draging
    
 
    
# In[Shape form]

''' This sets up the variables neede for forming a shape '''
if control_type=="shape_form": 
    tpull=0        # time to pull omn the object  not needed 
    xc=0        # center of robot x
    zc=0         # center of robot z
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    'importstar' this imports a star shape potential field 
    """
    shape="Circle"
    alpha=1# controller gain
    beta=0           # damping term
    b=5          # range of field 
    res=0.05
    px=0             # ball location x
    py=0           # ball location y
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # potential field
    # triangle R=1.3R
    # Square R=.85R
    phi=sim_obj.Shape_fields(1.2*R,px,py,b,res,shape,sim)
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # arguemnts
    args=(phi,alpha,beta,b) # create arguement list 
    X=[]
    Z=[]     
 
    
 
# In[Analytic Fields]
''' This sets up the variables neede for forming a shape '''
if control_type=="shape_form_analytic": 
    tpull=0        # time to pull omn the object  not needed 
    xc=0        # center of robot x
    zc=0         # center of robot z
    
    a=.58*R # radii 1
    c=1*R # radii 2
    theta=0 # rotation 
    res=0.1 # resolution of the field 
    
    alpha=20   # controller gain
    beta=0           # damping term
    b=5          # range of field 
    res=0.1
    px=0             # ball location x
    py=0           # ball location y
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # potential field    
    phi=sim_obj.analytic_field(a,c,px,py,theta,b,res)
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # arguements #
    args=(phi,alpha,beta,b) # create arguement list 
     

# In[Grab]
if control_type=="pot_field_grab": 
    #const=0.6010980273469619/2 #square
    const= .2313 #triangle 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    # Ball variables #####
    mb=1# mass of ball
    Rb=R/3# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    px=1.5*R          # ball location x
    py=0        # ball location y    
    xball=py # center of ball x
    zball=px # center of ball y

    #a=1.1*const# radii 1
    #c=1.1*const # radii 2
    a=0.25*Rb
    c=0.25*Rb
    theta=0 # rotation 
    res=0.1 # resolution of the field
    alpha=60 # controller gain
    beta=0          # damping term
    b=20          # range of field 

    Rd=R
    X=[]
    Z=[]
    
    phi=sim_obj.analytic_field(a,c,px,py,theta,b,res)
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    # arguements 
    args=(phi,alpha,beta,b)
    

# In[Grasp]
if control_type=="GRASP": 
    const=0.6010980273469619/2 #square
    #const= .2313 #triangle 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    # Ball variables #####
    mb=1# mass of ball
    Rb=R/3# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    px=4*R          # ball location x
    py=0        # ball location y    
    xball=py # center of ball x
    zball=1.5*R # center of ball y
    
    
    #a=1.1*const# radii 1
    #c=1.1*const # radii 2



    a=.25*Rb
    c=4.0*Rb
    alpha=3# controller gain
    alpha2=.4
    beta=0          # damping term
    b=20          # range of field 
    
    Rd=R
    X=[]
    Z=[]
    #phi=sim_obj.GRASPING_FIELD(a,c,px,py,Rb,zball,xball)
    phi=sim_obj.GRASPING_FIELD(a,c,px,py,Rb,zball,xball,alpha2)
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    # arguements 
    args=(phi,alpha,beta,b)


   

# In[complex image warp]
if control_type=="Complex_image_warp": 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    
    px=0         # ball location x
    py=0        # ball location y    

    
    alpha=10        # controller gain
    beta=0         # damping term
    name='C'
    Rd=R
    # potential field
    phi=sim_obj.image_warping_import(name,Rd,px,py)
    env_mode=None # create tunnel 
    gapw=3
    # arguements 
    args=(phi,alpha,beta)    
    X=[]
    Z=[]
# In[Image warping ]
''' This sets up the variables neede for forming a shape '''
if control_type=="image_warp": 
    tpull=0        # time to pull omn the object  not needed 
    xc=0        # center of robot x
    zc=0         # center of robot z
    alpha=100  # controller gain
    beta=0           # damping term
    b=100        # range of field 
    res=1
    px=0             # ball location x
    py=0           # ball location y
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    Rd=R
    shape=['Circle','EllipseL','Triangle','Square']
    # potential field
    phi=sim_obj.image_warping(px,py,b,res,shape,sim,a=.75*Rd,c=1.2*Rd,theta=0,R=0.85*Rd,R2=1.5*R)
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    #arguements 
    args=(phi,alpha,beta,b) # create arguement list     
    X=[]
    Z=[]
# In[Tunnel]
if control_type=="tunnel": 
    tpull=20        # time to pull omn the object
    xc=1.5         # center of robot x
    zc=-4            # center of robot z
    a=.58*R # radii 1
    c=1.2*R # radii 2

    theta = 0
    alpha=30      # controller gain
    beta=0          # damping term
    b=20             # range of field 
    Rd=.1            # radius of zero contour
    #px=-2.5            # goal location x
    px = -2.5
    py=-15  
    res=1          # goal location y
    # potential field
    phi=sim_obj.point_field(px,py,res,b) 
    
    #phi2=sim_obj.analytic_field(a,c,xc,zc,theta,b,res)
    #phi=[phi2,phi1]
    env_mode='tunnel' # create tunnel 
    gapw=1
    X=[]
    Z=[]
    #arguements 
    args=(phi,alpha,beta,b)      
    
# In[Tunnel verify]   
if control_type=="tunnel_verify": 
    tpull=20        # time to pull omn the object
    xc=1.5         # center of robot x
    zc=-4            # center of robot z
    a=.58*R # radii 1
    c=1.2*R # radii 2

    theta = 0
    alpha=3      # controller gain
    beta=0          # damping term
    b=20             # range of field 
    Rd=.1            # radius of zero contour
    #px=-2.5            # goal location x
    px = -2.5
    py=-15  
    res=1          # goal location y
    # potential field
    phi=sim_obj.point_field(px,py,res,b) 
    
    #phi2=sim_obj.analytic_field(a,c,xc,zc,theta,b,res)
    #phi=[phi2,phi1]
    env_mode='verify_tunnel' # create tunnel 
    gapw=1
    X=[]
    Z=[]
    #arguements 
    args=(phi,alpha,beta,b)   
    
# In[Import field]
if control_type=="import_field": 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    alpha=40          # controller gain
    beta=.5           # damping term
    field='A'
    # potential field
    phi=sim_obj.imported_fields(field)
    env_mode=None # create tunnel 
    gapw=3
    #arguements
    args=(phi,alpha,beta)     
    
# # In[complex image warp]
# if control_type=="Complex_image_warp": 
#     tpull=20        # time to pull omn the object
#     xc=0           # center of robot x
#     zc=0            # center of robot z
    
#     ##### pot field ######
#     """types of fields
#     "Analytic fields:
#     numeric fields: fields created numerically
#     region: region based fields 
#     'point_field' : point field 
#     """
#     alpha=10        # controller gain
#     beta=0         # damping term
#     name='C'
#     Rd=R
#     # potential field
#     phi=sim_obj.image_warping_import(name,Rd)
#     env_mode=None # create tunnel 
#     gapw=3
#     #### arguements #####
#     args=(phi,alpha,beta)

    
# In[Import field letters]
if control_type=="import_field_letters": 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    alpha=150          # controller gain
    beta=.1         # damping term
    field='JAM4'
    # potential field
    phi=sim_obj.imported_fields_letters(field,sim)
    env_mode=None # create tunnel 
    gapw=3
    px=0
    py=0
    #arguements
    args=(phi,alpha,beta)
    X=[]
    Z=[]
    gapw=1
    
    
# In[Verify]
if control_type=="target_verify": 
    tpull = 20        # time to pull omn the object
    
    # Ball variables #####
    mb=1# mass of ball
    Rb=.07# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=0 # center of ball x
    zball=0 # center of ball y
    
    xc = 0         # center of robot x
    zc = 0           # center of robot z

    alpha = 50     # controller gain
    beta = 5          # damping term
    b = 20             # range of field 
    px = 0            # goal location x
    py = 0  
    # potential field
    phi = sim_obj.source_field(px,py)
    env_mode = None # create tunnel 
    gapw = 1
    X = []
    Z = []
    #arguements 
    args = (phi,alpha,beta,b)     
    
    
# In[tunnel verify]
if control_type=="tunnel_verify": 
    tpull = 20        # time to pull omn the object
    
    # Ball variables #####
    mb=1# mass of ball
    Rb=.05# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=0 # center of ball x
    zball=0 # center of ball y
    
    xc = .5         # center of robot x
    zc = 0           # center of robot z

    alpha = 10     # controller gain
    beta = 3          # damping term
    b = 20             # range of field 
    px = 0            # goal location x
    py = 0  
    # potential field
    phi = sim_obj.source_field(px,py)
    env_mode = None # create tunnel 
    gapw = 1
    X = []
    Z = []
    #arguements 
    args = (phi,alpha,beta,b)   
    
    
    
# In[Grasp Verify]
if control_type=="grasp_verify": 
    const=0.6010980273469619/2 #square
    #const= .2313 #triangle 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    # Ball variables #####
    mb=.7# mass of ball
    Rb=.07# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density

    xball=-1
    zball=-0.04    
    py=(xball-.5)
    px=-0.04       # ball location y    
    
    
    
    #a=1.1*const# radii 1
    #c=1.1*const # radii 2



    c=.02
    a=3*Rb
    alpha=.4# controller gain
    alpha2=.4
    beta=3         # damping term
    b=20          # range of field 
    
    Rd=R
    X=[]
    Z=[]
    #phi=sim_obj.GRASPING_FIELD(a,c,px,py,Rb,zball,xball)
    phi1 = sim_obj.source_field(px,py)
    phi2=sim_obj.GRASPING_FIELD(a,c,px,py,Rb/2,zball,xball,alpha2)
    phi=[phi1,phi2]
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    # arguements 
    args=(phi,alpha,beta,b)
    
# In[door rip]
if control_type=="door_rip": 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    a=.5*R # radii 1
    c=1.6*R # radii 2
    px = 0 
    py = 0
    theta = 0
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    alpha=10       # controller gain
    beta=0          # damping term
    b=20             # range of field 
    Rd=2            # radius of zero contour
    px=0             # goal location x
    py=8  
    res=.1          # goal location y
    shape=['EllipseS','Circle']
    
    # potential field
    phi1=sim_obj.analytic_field(a,c,xc,zc,theta,b,res)
    phi2=sim_obj.point_field(px,py,res,b) 
    phi3=sim_obj.image_warping(4,0,b,res,shape,a=a,c=c,theta=0,R=Rd,R2=None)
    phi=[phi1,phi2,phi3]
    env_mode='tunnel' # create tunnel 
    gapw=1
    #arguements 
    args=(phi,alpha,beta,b)      

# In[Obstacle]
if control_type=="Obstacle": 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    a=.58*R # radii 1
    c=1.2*R # radii 2
    px = 0 
    py = 0
    theta = 0
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    """
    alpha=10       # controller gain
    beta=0          # damping term
    b=100             # range of field 
    Rd=.1            # radius of zero contour
    px=0             # goal location x
    py=12  
    res=1          # goal location y
    # potential field
    phi=sim_obj.point_field(px,py,res,b) 
    
    
    env_mode='Obstacles' # create tunnel 
    gapw=1
    min_distance = 1.25
    widths = 8
    heights = 8 
    phis=sim_obj.Poisson_Sampling(min_distance, widths, heights)
    obs_coord= phis.get_samples()
    Z=[]
    X=[]
    for i in range(len(obs_coord)):
        Z.append(obs_coord[i][1]-2)
        X.append(obs_coord[i][0]+2)
    #Z=[-1,1,0]
    #X=[2,5,8]
    #arguements 
    args=(phi,alpha,beta,b)      

# In[R-functions]
if control_type=="R-fields":
    tpull=0        # time to pull omn the object  not needed 
    xc=0        # center of robot x
    zc=0         # center of robot z
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    'importstar' this imports a star shape potential field 
    """
    alpha=120# controller gain
    beta=0           # damping term
    b=5          # range of field 
    res=0.05
    px=0             # ball location x
    py=0           # ball location y
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    Psi1=sim_obj.Circle_R_function()
    Psi1.a=0
    Psi1.b=0
    Psi1.R=R
    
    Psi2=sim_obj.series_of_lines(sim)

    x=[0,.56,.66,1,1,.75,.75,1,1,.66,.56,0,0]
    y=[.125,.125,0,0,.125,.125,.375,.375,.5,.5,.375,.375,.125]
    x=np.dot(-2.0,x)
    y=np.dot(2.0,y)
    xp=np.sum(x)/len(x)
    yp=np.sum(y)/len(y)
    (segments)=Psi2.create_segment(x-xp+.1,y-yp)
    Psi2.XT=x-xp+.1
    Psi2.YT=y-yp

    Psi2.segments=segments
    phi=[Psi1,Psi2]
    
    
    env_mode=None # create tunnel 
    # arguemnts
    args=(phi,alpha,beta,b) # create arguement list 
    X=[]
    Z=[]         


# In[SAVE VARIABLES]
position=True # save bot position
velocity=True# save bot velocity
forces=False  # save bot forces
control_force=False # save all contact forces
contact_positions=False

# interior particle postions, velocity and forces
if mode!='empty':
    particle_position=True
    particle_vel=True
    particle_force=True
else:
    particle_position=True
    particle_vel=True
    particle_force=True

# ball information
if control_type=='pot_field_grab' or "GRASP" or 'Verify':
    ball_data=True
else:
    ball_data=False

    


if control_type=='shape_form':
    shaped=True
    error=True
else:
    shaped=False
    error=False
    
if control_type=='image_warp':
    shaped=False
    error=True
else:
    shaped=False
    error=False
save_data=[position,velocity,forces,control_force,contact_positions,particle_position,particle_vel,particle_force,ball_data,shaped]        

