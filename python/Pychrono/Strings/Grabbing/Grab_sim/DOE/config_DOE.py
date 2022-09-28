"""
"""
import numpy as np
import grab_sim_objects_DOE as sim_obj
from scipy.interpolate import interp1d
# In[Variables for all bots]
### General parameters ###
visual="pov" # visualization
sim='ver_3'# sim number 
fixed=False # is it fixed
obj=[]
data_path="C:/Users/dmulr/OneDrive/Documents/data/" # data file for other visuals

##### control type #####
control_type='tunneling'

'''
# Control type:
"shape_form": Form a shape
'pot_field_grab': grab an object
'pot_field_grab_A': grab an object using analytical rbf
"tunneling" tunneling 
'grab_drag_A' grab and drag an object with anayltical pot fields 
'grab_drag': grab and drag an object 
'grab_drag_2': grabbing a ball and dragging it with moving pot field
"shape_formation": maintain a formation 


'''

##### interior modes #####
mode="nmax"
  
'''
# Interior generation modes:
'empty'     Nothing inside
'nonhomo'   diameter alternate
'max'       max possible number of interiors
'"nonhnmax"'      customize the ring sizes
'nonhnmax'  max number of interiors with different diamters
'homo'      all the same diameterr
'''

##### granular mode #####
granmode="homo"

'''
homo: all the same size
nonhomo: they alternate in diameter
'''

##### time #####
tstep=.001   # time step
tend=20 # time end

##### Friction #####
mu_f=.1   # friction
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
nb=30# number of robots
height=.06  # height of cylinder
diameter=.076/2 # diameter of cylinder and robots
volume=np.pi*height*(diameter)**2   # calculate volume
rowr=mr/volume # calculate density of robot
#R=(2*diameter*nb/(np.pi*2))+.3
#R=.063/(np.sin(np.pi/nb))
#R=.3
R=.65
### Geometry of robot ###
geom="cylinder"

'''
square: Robots will be in a square shape
cylinder: Robots will be a cylinder
shere: robots will be a sphere
'''

### Active bots ###
actbots=np.arange(0,nb,1)
active=np.zeros(nb)
for i  in range(len(active)):
    if any(actbots==i):
        active[i]=1

##### Interior particles #####
mp=.008 # mass 
diameter2=.076/2 # diameter of cylinder and robots
volume2=np.pi*height*(diameter2)**2   # calculate volume
rowp=mp/volume2 # density

##### Spring ##### 
k=1  # spring constant (bots)
rl=0 # resting length
rlmax=0.005 # max length
type_spring='var'

##### Floor #####
length=40 # Length of the body floor
tall=1     # height of the body floor

# In[Path follwoing]
if control_type=="path_following":
    #### Path variables ####
    paths=sim_obj.Path(0,10,'flat_line')
    #paths=Path(ax,ay)    
    pathind=0 # leader robot
    vref=2 # desired velocity
    mag_t=5 # tamgential gain
    mag_n=10 # normal gain
    kd=1 # derivative gain for follower
    
    
    poseindt=actbots[0:len(actbots)] # follower robots
    poseind=np.zeros(nb)  
    
    shapes=None # shape set to none since were not forming a desired shape 
    
    # fill the leader robots
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1

# arguements that go into 
    args=(paths,pathind,vref,poseind,mag_t,kd,mag_n)

# In[tunneling]    
if control_type=="tunneling":
    #### Path variables ####
    paths=sim_obj.Path(0,20,'flat_line')  
    pathind=0
    vref=1 # reference velocity
    poseindt=actbots[0:len(actbots)] 
    poseind=np.zeros(nb)
    #force 
    mag_t=3
    mag_n=1
    kd=1
    shapes=None
    gapw=1
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1
    env_mode="tunnel" # if we want to go into a tunnel 
    args=paths,pathind,vref,poseind,mag_t,kd,mag_n    

# In[Poit field grab]    
if control_type=="pot_field_grab": 
    tpull=100 # time to pull omn the object
    shape="circle"
    alpha=60
    beta=0
    balls=0
    p1=1.5
    p2=0
    bl=-9
    br=9
    Rd=.3
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="oval":
        nr=[0,1,2,3,4] # circle
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=.1
    Rb=.3
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p1
    zball=p2  
    shapes=sim_obj.Points_for_shape(shape,xball,zball,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,zball,xball,tpull
    mag=1
    pathind=0
    mag_tan=0
    mag_n=0
    env_mode=None
    gapw=0
    
# In[Poit field grab_2]    
if control_type=="pot_field_grab_2": 
    tpull=100 # time to pull omn the object
    shape="oval"
    alpha=300
    beta=.1
    balls=0
    p1=0
    p2=0
    bl=-9
    br=9
    Rd=R/2
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="oval":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=5
    Rb=.4
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=1.5
    zball=p2  
    shapes=sim_obj.Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,zball,xball,tpull
    mag=1
    pathind=0
    mag_tan=0
    mag_n=0
    env_mode=None
    gapw=0
# In[Potfield Grab A]
######################################################################################
if control_type=="pot_field_grab_A": 
    tpull=10
    alpha=20
    beta=0
    balls=0
    p1=0
    p2=1.5
    bl=-9
    br=9

    Rb=.2
    Rd=.65
    ##### Ball variables #####
    mb=1
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p1
    zball=p2 
    env_mode=None
    gapw=0
    shapes=sim_obj.points_shape_A(p2,p1,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,tpull
    pathind=0


# In[Potfield Grab A]
######################################################################################
if control_type=="verify": 
    tar=sim_obj.import_data()
    (TX,TY,time,X,Y)=tar.return_target()
    Tx = interp1d(time, TX, kind='nearest')
    Ty = interp1d(time, TY, kind='nearest')
    alpha=.5
    beta=0
    balls=0
    p1=TX[0]
    p2=TY[0]
    bl=-9
    br=9
    tend=90
    R=.25
    Rb=.05
    Rd=.05
    ##### Ball variables #####
    mb=1
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p1
    zball=p2 
    env_mode=None
    gapw=0
    shapes=sim_obj.points_shape_A(p2,p1,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,Tx,Ty
    pathind=0
    
# In[Grad and drag]
######################################################################################
if control_type=="grab_drag":
    tpull=1# time when to release object 
    #(ax,ay)=Path.create_line()
    paths=sim_obj.Path(0,10,'flat_line') # desired path 
    pathind=0 # leader robot
    vref=5 
    poseindt=actbots[1:len(actbots)]
    poseind=np.zeros(nb)
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1
    
    shape="circle" # desired shape in this case a circule since were grabbing a ball 
    alpha=40 # alpha gain for 
    beta=.5
    balls=0
    p1=1.8
    p2=0
    bl=-12
    br=12
    Rb=.4*R
    Rd=Rb
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=1
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p2
    zball=p1  
    mag_t=.5
    mag_n=1
    kd=0
    #env_mode='tunnel'
    env_mode=None
    gapw=1
    #env_mode=None
    shapes=sim_obj.Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=(shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,paths,pathind,vref,poseind,tpull,mag_t,kd,mag_n)
##############################################################################################################################
# In[grab and drag 2]
if control_type=="grab_drag_2":
    tpull=1# time when to release object 
    #(ax,ay)=Path.create_line()
    paths=sim_obj.Path(0,10,'flat_line') # desired path 
    pathind=0 # leader robot
    vref=5 
    poseindt=actbots[1:len(actbots)]
    poseind=np.zeros(nb)
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1
    
    shape="circle" # desired shape in this case a circule since were grabbing a ball 
    alpha=40 # alpha gain for 
    beta=1
    balls=0
    p1=1.8
    p2=0
    bl=-12
    br=12
    Rb=.4*R
    Rd=Rb
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=1
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p2
    zball=p1  
    mag_t=.5
    mag_n=1
    kd=0
    #env_mode='tunnel'
    env_mode=None
    gapw=1
    #env_mode=None
    shapes=sim_obj.Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=(shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,paths,pathind,vref,poseind,tpull,mag_t,kd,mag_n)



# In[Grab and drag A]
if control_type=="grab_drag_A":
    tpull=1.3
    #(ax,ay)=Path.create_line()
    paths=sim_obj.Path(0,4,'flat_line')
    pathind=0
    vref=3
    poseindt=actbots[1:len(actbots)]
    poseind=np.zeros(nb)
    for i  in range(len(poseind)):
        if any(poseindt==i):
            poseind[i]=1
    
    shape="circle"
    alpha=20
    beta=0
    balls=0
    p1=0
    p2=-1.2
    bl=-9
    br=9
    Rd=.3
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
    if shape=="grab":
        nr=[]
    ##### Ball variables #####
    mb=.2
    Rb=Rd
    volume3=np.pi*.25*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=p2
    zball=p1 
    mag_t=15
    mag_n=1
    kd=0
    env_mode="tunnel"
    shapes=sim_obj.points_shape_A(p1,p2,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,mb,Rb,height,volume3,rowb,xball,zball,paths,pathind,vref,poseind,tpull,mag_t,kd,mag_n

   

# In[Shape form]
if control_type=="shape_form":
    shape="oval"
    alpha=300
    beta=10
    balls=0
    p1=0
    p2=0
    bl=-9
    br=9
    Rd=R/2
    env_mode=None
    pathind=0
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="oval":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
        nr=np.asarray(nr)
    if shape=="grab":
        nr=[] # square
    gapw=0
    ##### Ball variables #####
    shapes=sim_obj.Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,Rd,nr      
# In[shape formation]
if control_type=="shape_formation":
    shape="oval"
    alpha=30
    beta=.1
    balls=0
    p1=0
    p2=0
    bl=-12
    br=12
    vref=1
    Rd=R
    env_mode=None
    pathind=0
    paths=sim_obj.Path(0,10,'parab')
    if shape=="circle":
        nr=[0,1,2,3,4] # circle
    if shape=="oval":
        nr=[0,1,2,3,4] # circle
    if shape=="Square":
        nr=[0,1,2,3] # square
        nr=np.asarray(nr)
    
    shapes=sim_obj.Points_for_shape(shape,p1,p2,nb,diameter,bl,br,R,nr,Rd)
    args=shapes,alpha,beta,p1,p2,bl,br,Rd,nr,vref,paths       


# In[save variables] 
position=True
velocity=True
forces=True
control_force=True
spring=True
contact=True
if mode!='empty':
    part_position=True
    part_vel=True
    part_force=True
else:
    part_position=False
    part_vel=False
    part_force=False

if control_type=='path_following' or control_type=='shape_form' or control_type=="tunneling" or control_type=="shape_formation":
    ball_data=False
else:
    ball_data=True    

if control_type=='shape_form' or control_type=="shape_formation" :
    desired_shape=True
else:
    desired_shape=False
    
if control_type=="grab_drag" or control_type=='path_following' or control_type=="tunneling" or control_type=="grab_drag_A" or control_type=="grab_drag_2"or control_type=="shape_formation":
    path_data=True
else:
    path_data=False

if control_type=='shape_form' or control_type=="shape_formation":
    error=True
else:
    error=False
    
save_data=[position,velocity,forces,control_force,spring,contact,part_position,part_vel,part_force,ball_data,path_data,desired_shape,error]