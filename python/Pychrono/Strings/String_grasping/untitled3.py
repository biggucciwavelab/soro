# -*- coding: utf-8 -*-
"""
Created on Wed Dec 21 12:17:00 2022

@author: Big Gucci
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

col_y = chrono.ChColorAsset(); col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
col_b = chrono.ChColorAsset(); col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
col_g = chrono.ChColorAsset(); col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
col_p = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
col_w = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(.8, .8, .8)) # Purple

print ("Example: create a system and visualize it in realtime 3D");
my_system = chrono.ChSystemNSC()

# Set the dfault outward/inward shape margins for collision detection,
# this is epecially important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

#R=1.25
#height=.25
# circle = chrono.ChBodyEasyCylinder(R,height,1000,True,True)
# circle.SetPos(chrono.ChVectorD(0,height/2,0)) 
# circle.AddAsset(col_b)
# my_system.Add(circle) 

# force = chrono.ChForce()  # create it 
# circle.AddForce(force) # apply it to bot object
# force.SetVpoint(chrono.ChVectorD(box1_x,box1_height/2+.1,box1_z))
# force.SetMode(chrono.ChForce.FORCE) # set the mode 
# force.SetDir(chrono.VECT_X) # set direction 
# force.SetMforce(float(-8000))
# botforce.SetVpoint(chrono.ChVectorD(0.5,.06,0))



height=.25
R=1.25
a=0.4
b=0.37
w1=1
w2=1.26
Rr=np.sqrt(R**2 - (w1/2)**2)
nsides=50
pt_vect = chrono.vector_ChVectorD()
#theta=np.linspace(0-np.pi/2,2.73-np.pi/2,100)
#theta2=np.linspace(3.55-np.pi/2,np.pi*2-np.pi/2,100)
theta1=0.41
theta2=2*np.pi-theta1
theta=np.linspace(theta1+np.pi/2,theta2+np.pi/2,100)
height=.25
R=1.25
a=0.4
b=0.37
w1=1
w2=1.26
Rr=R*np.cos(theta1)
#Rr=np.sqrt(R**2 - (w1/2)**2)
nsides=50
# creates bottom
for i in range(len(theta)):
    pt_vect.push_back(chrono.ChVectorD(R*np.sin(theta[i]),height/2,R*np.cos(theta[i])))
    
#for i in range(len(theta2)):
    #pt_vect.push_back(chrono.ChVectorD(R*np.sin(theta2[i]),height/2,R*np.cos(theta2[i])))    

    #create top 
for i in range(len(theta)):
    pt_vect.push_back(chrono.ChVectorD(R*np.sin(theta[i]),-height/2,R*np.cos(theta[i])))
#for i in range(len(theta2)):
   # pt_vect.push_back(chrono.ChVectorD(R*np.sin(theta2[i]),-height/2,R*np.cos(theta2[i])))

# # creates bottom
# for i in range(nsides):
#     pt_vect.push_back(chrono.ChVectorD(R*np.sin(i*2*np.pi/nsides),height/2,R*np.cos(i*2*np.pi/nsides)))
#     #create top 
# for i in range(nsides):
#     pt_vect.push_back(chrono.ChVectorD(R*np.sin(i*2*np.pi/nsides),-height/2,R*np.cos(i*2*np.pi/nsides)))

ball=chrono.ChBodyEasyConvexHull(pt_vect,1000,True,True)   
ball.SetPos(chrono.ChVectorD(0,height/2,0))
ball.SetCollide(True) # set the collision mode
ball.SetBodyFixed(False) # set if its fixed
ball.GetCollisionModel().ClearModel()
ball.GetCollisionModel().AddConvexHull(pt_vect)
ball.GetCollisionModel().BuildModel()
my_system.Add(ball) 

# force = chrono.ChForce()  # create it 
# ball.AddForce(force) # apply it to bot object
# #force.SetVpoint(chrono.ChVectorD(box1_x,box1_height/2+.1,box1_z))
# force.SetMode(chrono.ChForce.FORCE) # set the mode 
# force.SetDir(chrono.VECT_X) # set direction 
# force.SetMforce(float(-10000))
#botforce.SetVpoint(chrono.ChVectorD(0.5,.06,0))




box1_width=a
box1_length=w1
box1_height=height
box1_x=Rr + a/2 +.018
box1_y=height/2
box1_z=0


box1 = chrono.ChBody() # create ball object
box1 = chrono.ChBodyEasyBox(box1_width,box1_height,box1_length,1000,True,True) # create object # specify properties and make it a cylinder
box1.SetPos(chrono.ChVectorD(box1_x,box1_y,box1_z))
box1.SetCollide(True) # set the collision mode
box1.SetBodyFixed(False) # set if its fixed
box1.AddAsset(col_y)
my_system.Add(box1) 

glue=chrono.ChLinkMateFix() # cretae fix constraint
glue.Initialize(ball,box1) # fix object to bot
my_system.AddLink(glue) # add to system 



box2_width=b
box2_length=w2
box2_height=height
box2_x=Rr + box1_width + box2_width/2
box2_y=height/2
box2_z=0

box2 = chrono.ChBody() # create ball object
box2 = chrono.ChBodyEasyBox(box2_width,box2_height,box2_length,1000,True,True) # create object # specify properties and make it a cylinder
box2.SetPos(chrono.ChVectorD(box2_x,box2_y,box2_z))
box2.SetCollide(True) # set the collision mode
box2.SetBodyFixed(False) # set if its fixed
box2.AddAsset(col_g)
my_system.Add(box2) 

glue=chrono.ChLinkMateFix() # cretae fix constraint
glue.Initialize(box1,box2) # fix object to bot
my_system.AddLink(glue) # add to system 

# box1_width=2
# box1_length=1
# box1_height=height

# box1_x=0
# box1_y=0
# box1_z=0

# box1 = chrono.ChBody() # create ball object
# box1 = chrono.ChBodyEasyBox(box1_width,box1_height,box1_length,1000,True,True) # create object # specify properties and make it a cylinder
# box1.SetPos(chrono.ChVectorD(box1_x,box1_height/2+.1,box1_z))
# box1.SetName("ball") # give it a name
# box1.SetId(10000) 
# box1.SetCollide(True) # set the collision mode
# box1.SetBodyFixed(False) # set if its fixed
# body_floor_texture = chrono.ChTexture()
# body_floor_texture.SetTextureFilename(chrono.GetChronoDataFile('concrete.jpg'))
# box1.GetAssets().push_back(body_floor_texture)
# my_system.Add(box1) 


# box2_width=1
# box2_length=2
# box2_height=0.25

# box2_x=0
# box2_y=0
# box2_z=1.5
# box2 = chrono.ChBody() # create ball object
# box2 = chrono.ChBodyEasyBox(box2_width,box2_height,box2_length,1000,True,True) # create object # specify properties and make it a cylinder
# box2.SetPos(chrono.ChVectorD(box2_x,box2_height/2+.1,box2_z))
# box2.SetName("ball") # give it a name
# box2.SetId(10000) 
# box2.SetCollide(True) # set the collision mode
# box2.SetBodyFixed(False) # set if its fixed
# body_floor_texture = chrono.ChTexture()
# body_floor_texture.SetTextureFilename(chrono.GetChronoDataFile('concrete.jpg'))
# box2.GetAssets().push_back(body_floor_texture)
# my_system.Add(box2) 

# botforce = chrono.ChForce()  # create it 
# box2.AddForce(botforce) # apply it to bot object
# botforce.SetVpoint(chrono.ChVectorD(box1_x,box1_height/2+.1,box1_z))
# botforce.SetMode(chrono.ChForce.FORCE) # set the mode 
# botforce.SetDir(chrono.VECT_Z) # set direction 
# botforce.SetMforce(float(-6000))
# #botforce.SetVpoint(chrono.ChVectorD(0.5,.06,0))

# glue=chrono.ChLinkMateFix() # cretae fix constraint
# glue.Initialize(box1,box2) # fix object to bot
# my_system.AddLink(glue) # add to system 


floor_width=20
floord_height=1

floor = chrono.ChBody() # create ball object
floor = chrono.ChBodyEasyBox(floor_width,floord_height,floor_width,1000,True,True) # create object # specify properties and make it a cylinder
floor.SetPos(chrono.ChVectorD(0,-floord_height/2,0))
floor.SetName("ball") # give it a name
floor.SetId(10000) 
floor.SetCollide(True) # set the collision mode
floor.SetBodyFixed(True) # set if its fixed
body_floor_texture = chrono.ChTexture()
body_floor_texture.SetTextureFilename(chrono.GetChronoDataFile('concrete.jpg'))
floor.GetAssets().push_back(body_floor_texture)
my_system.Add(floor) 




myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono example', chronoirr.dimension2du(800,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo()
myapplication.AddTypicalCamera(chronoirr.vector3df(2,2,2),chronoirr.vector3df(0,0,0))
myapplication.SetSymbolscale(.002)
myapplication.SetShowInfos(True)
            #self.myapplication.SetContactsDrawMode(2)
myapplication.SetPaused(False)
myapplication.AddTypicalLights()
myapplication.DrawAll               
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()
myapplication.AddShadowAll()
myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                 chronoirr.vector3df(0,0,0),    # aimpoint
                                 9,                 # radius (power)
                                 1,9,               # near, far
                                 30)                # angle of FOV
myapplication.SetTimestep(0.005)
myapplication.SetTryRealtime(False)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()
