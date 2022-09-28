# -*- coding: utf-8 -*-
"""
Created on Thu Nov 14 17:56:34 2019

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import matplotlib.pyplot as plt
import os
import numpy as np
import timeit
from pid_controller.pid import PID
from scipy.optimize import LinearConstraint, Bounds, minimize, linprog
import random 

# In[Create material]
def Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs):
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(mu_f)
    material.SetDampingF(mu_b)
    material.SetCompliance (C)
    material.SetComplianceT(Ct)
    material.SetRollingFriction(mu_r)
    material.SetSpinningFriction(mu_s)
    material.SetComplianceRolling(Cr)
    material.SetComplianceSpinning(Cs)
    return material

# In[Create Floor]
def Floor(material,length,tall):
    body_floor = chrono.ChBody()
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVectorD(0, -tall, 0 ))
    body_floor.SetMaterialSurface(material)
    body_floor.GetCollisionModel().ClearModel()
    body_floor.GetCollisionModel().AddBox(length, tall, length) # hemi sizes
    body_floor.GetCollisionModel().BuildModel()
    body_floor.SetCollide(True)
    body_floor_shape = chrono.ChBoxShape()
    body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(length, tall, length)
    body_floor.GetAssets().push_back(body_floor_shape)
    body_floor_texture = chrono.ChTexture()
    body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    body_floor.GetAssets().push_back(body_floor_texture)
    return(body_floor)
    
# In[Bots jamming]
def BOTS_Jamming(nb,i,x,y,z,theta,diameter,height,rowr,obj,material,body_floor,my_system,Springs,k,rl):
    # create body
    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyCylinder(diameter/2, height,rowr)
    # set position
    bot.SetPos(chrono.ChVectorD(x,y,z))
    bot.SetMaterialSurface(material)
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
    bot.SetRot(rotation1)
    # collision models
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddCylinder(diameter/2,diameter/2,height/2) # hemi sizes
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    bot.SetBodyFixed(False)
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    my_system.AddLink(pt)
    
    # link springs
    if i>=1:
        p1=0
        p2=diameter/2
        p3=0
        p4=-diameter/2
        h=0
        ground=chrono.ChLinkSpring()
      # Identify points to be attatched to the springs 
        ground.SetName("ground")
        # Attatches  springs
        ground.Initialize(obj[i-1], bot,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringK(k)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground) 
    # Last spring
    if i==nb-1:  
        p1=0
        p2=diameter/2
        p3=0
        p4=-diameter/2
        h=0
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        ground.Initialize(bot, obj[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringK(k)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground) 
    my_system.Add(bot)
    obj.append(bot)
    
    return(Springs,obj,my_system)
# In[Bots with no attatchment]
def BOTS_free(nb,i,x,y,z,theta,diameter,height,rowr,force,obj,material,body_floor,my_system):
     # Create bots    
    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyCylinder(diameter/2, height,rowr)
    bot.SetPos(chrono.ChVectorD(x,y,z))
    bot.SetMaterialSurface(material)
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-random.random(), chrono.ChVectorD(0, 1, 0));  
    bot.SetRot(rotation1)
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddCylinder(diameter/2,diameter/2,height/2) # hemi sizes
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    bot.SetBodyFixed(False)
    myforcex = chrono.ChForce()
    bot.AddForce(myforcex)
    myforcex.SetMode(chrono.ChForce.FORCE)
    myforcex.SetRelDir(chrono.ChVectorD(1,0,0))
    myforcex.SetMforce(.4)
    force.append(myforcex)
    # link to the floor
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    my_system.AddLink(pt)
    body_floor_texture = chrono.ChTexture()
    body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
    bot.GetAssets().push_back(body_floor_texture)
    my_system.Add(bot)
    obj.append(bot)
    
    return(force,my_system,obj)
    
def GOAL(goal,my_system,diameter):
    gpoint=chrono.ChBody()
    gpoint=chrono.ChBodyEasySphere(diameter/2,1000)
    gpoint.SetPos(chrono.ChVectorD(goal[0],goal[1],goal[2]))
    gpoint.SetBodyFixed(True)
    col_p = chrono.ChColorAsset()
    col_p.SetColor(chrono.ChColor(0, 1, 0))
    gpoint.AddAsset(col_p)
    my_system.Add(gpoint)
    
    return(my_system)
    
    
# In[Create robots circular]
def BOTS(nb,i,x,y,z,diameter,height,theta,rowr,body_floor,obj,my_system,material,Springs,k,rl,botcall,force):
    # Create bots    
    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyCylinder(diameter/2, height,rowr)
    bot.SetPos(chrono.ChVectorD(x,y,z))
    bot.SetMaterialSurface(material)
    # rotate them
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
    bot.SetRot(rotation1)
    # give ID number
    bot.SetId(i)  
    
    # Apply forces to active bots
    if botcall[:,i]==1:
        myforcex = chrono.ChForce()
        bot.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetDir(chrono.VECT_X)
        myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
        #myforcex.SetMforce(mag)
        force.append(myforcex)
        if i==0:
            col_p = chrono.ChColorAsset()
            col_p.SetColor(chrono.ChColor(0, 1, 0))
            bot.AddAsset(col_p)
        
        else:
             
            col_p = chrono.ChColorAsset()
            col_p.SetColor(chrono.ChColor(0.44, .11, 52))
            bot.AddAsset(col_p)
    # passive robots
    else:
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))        
    # collision model
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddCylinder(diameter/2,diameter/2,height/2) # hemi sizes
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    bot.SetBodyFixed(False)
    # link to the floor
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    my_system.AddLink(pt)
    
    # link springs
    if i>=1:
        ground=chrono.ChLinkSpring()
      # Identify points to be attatched to the springs 
        ground.SetName("ground")
        p1=0
        p2=diameter/2
        p3=0
        p4=-diameter/2
        h=0
        # Attatches  springs
        ground.Initialize(obj[i-1], bot,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringF(k)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))


    # Last spring
    if i==nb-1:        
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        ground.Initialize(bot, obj[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringF(k)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
    my_system.AddLink(ground)
    Springs.append(ground) 
    my_system.Add(bot)
    obj.append(bot)
    return(obj,bot,my_system,Springs)


# In[Square Bots] these are bots for square robots
def Square_BOTS(nb,i,x,y,z,diameter,height,theta,rowr,body_floor,obj,my_system,material,Springs,k,rl,botcall,force):
     # Create bots    
    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyBox(diameter,height,diameter,rowr)   # create square robot
    bot.SetPos(chrono.ChVectorD(x,y,z))     # set position 
    bot.SetMaterialSurface(material)            #apply surface material 
    # rotate them. This is for the springs 
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
    bot.SetRot(rotation1)
    # give ID number
    bot.SetId(i)
    # collision model
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddBox(diameter/2,height/2,diameter/2) # hemi sizes
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    bot.SetBodyFixed(False) # decides if the object will be fixed. 
    

    # Apply forces to active bots
    if botcall[:,i]==1:
        myforcex = chrono.ChForce()
        bot.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetDir(chrono.VECT_X)
        myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
        #myforcex.SetMforce(mag)
        force.append(myforcex)
        # if bot is not  active apply a color
        if i==0:
            col_p = chrono.ChColorAsset()
            col_p.SetColor(chrono.ChColor(0, 1, 0))
            bot.AddAsset(col_p)
       # if bot is active apply a color
        else:
            col_p = chrono.ChColorAsset()
            col_p.SetColor(chrono.ChColor(0.44, .11, 52))
            bot.AddAsset(col_p)
    # passive robots
    else:
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))
    # link to the floor
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    my_system.AddLink(pt)
    
    # link springs
    if i>=1:
        ground=chrono.ChLinkSpring()
      # Identify points to be attatched to the springs 
        ground.SetName("ground")
        p1=0
        p2=diameter/2
        p3=0
        p4=-diameter/2
        h=0
        # Attatches  springs
        ground.Initialize(obj[i-1], bot,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringF(k)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground)

    # Last spring
    if i==nb-1:        
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        ground.Initialize(bot, obj[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringF(k)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground) 
    # add robot to system 
    my_system.Add(bot)
    obj.append(bot)
    return(obj,bot,my_system,Springs)


# In[Square Bots Spinners] for corgi model and others like it 
def BOTS_Spinners(nb,i,x,y,z,diameter,height,theta,rowr,body_floor,obj,my_system,material,Springs,k,rl,botcall,force):
    
    if botcall[:,i]==1:
        diameter2=np.sqrt(2)*diameter
    else:
        diameter2=diameter
    # Create bots    
    bot = chrono.ChBody()
    bot = chrono.ChBodyEasyCylinder(diameter2/2, height,rowr)
    bot.SetPos(chrono.ChVectorD(x,y,z))
    bot.SetMaterialSurface(material)
    # rotate them
    rotation1 = chrono.ChQuaternionD()
    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
    bot.SetRot(rotation1)
    # give ID number
    bot.SetId(i)  
    # collision model
    bot.GetCollisionModel().ClearModel()
    bot.GetCollisionModel().AddCylinder(diameter2/2,diameter2/2,height/2) # hemi sizes
    bot.GetCollisionModel().BuildModel()
    bot.SetCollide(True)
    bot.SetBodyFixed(False)
    # link to floor
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    my_system.AddLink(pt)
    col_p = chrono.ChColorAsset()
    col_p.SetColor(chrono.ChColor(0.44, .11, 52))
    if botcall[:,i]==1:  
    # apply torque instead of force 
        torque = chrono.ChForce()
        bot.AddForce(torque)
        torque.SetMode(chrono.ChForce.TORQUE)
        torque.SetDir(chrono.VECT_Y)
        force.append(torque)
        bot.AddAsset(col_p)
        body_floor_texture = chrono.ChTexture()
        body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
        bot.GetAssets().push_back(body_floor_texture)
    # passive robots
    else:
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))
        bot.AddAsset(col_g)
    # link to the floor
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    my_system.AddLink(pt)
    #color for springs
    col1=chrono.ChColorAsset()
    col1.SetColor(chrono.ChColor(0,0,1))
    # link springs
    if i>=1:
        ground=chrono.ChLinkSpring()
      # Identify points to be attatched to the springs 
        ground.SetName("ground")
        p1=0
        p2=0
        p3=0
        p4=0
        h=0
        # Attatches  springs
        ground.Initialize(obj[i-1], bot,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringF(k)
        ground.Set_SpringRestLength(rl)
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground)

    # Last spring
    if i==nb-1:        
        ground=chrono.ChLinkSpring()
        ground.SetName("ground")
        ground.Initialize(bot, obj[0], True, chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),False)
        ground.Set_SpringF(k)
        ground.Set_SpringRestLength(rl)
        col1=chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0,0,1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
        my_system.AddLink(ground)
        Springs.append(ground) 
    my_system.Add(bot)
    obj.append(bot)
    return(obj,bot,my_system,Springs,force)
# In[Interior Granulars]
def Interior(x,y,z,i,diameter,height,rowp,R2,material,obj,my_system,body_floor):
   # Create body
    gran = chrono.ChBody()
    gran = chrono.ChBodyEasyCylinder(diameter/2, height,rowp)
    gran.SetPos(chrono.ChVectorD(x,y,z))
    gran.SetMaterialSurface(material)
    gran.SetId(i)
    # Create collision model
    gran.GetCollisionModel().ClearModel()
    gran.GetCollisionModel().AddCylinder(diameter/2,diameter/2,height/2) # hemi sizes
    gran.GetCollisionModel().BuildModel()
    gran.SetCollide(True)
    gran.SetBodyFixed(False)
    # add color
    col_r = chrono.ChColorAsset()
    col_r.SetColor(chrono.ChColor(1, 0, 0))
    gran.AddAsset(col_r)
    # mate to floor
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    my_system.AddLink(pt)
    
    # add to system
    my_system.Add(gran)
    obj.append(gran)
    return my_system,obj 
    
# In[Ball]
def Ball(x,y,z,Rb,height,hhalf,rowr,material,obj,my_system,forceb,body_floor,Balls):
    
    z2x = chrono.ChQuaternionD()
    z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
    ball = chrono.ChBody()
    ball = chrono.ChBodyEasyCylinder(Rb,height,rowr)
    ball.SetPos(chrono.ChVectorD(x,y,z))
    ball.SetMaterialSurface(material)
    myforcex = chrono.ChForce()
    ball.AddForce(myforcex)
    myforcex.SetMode(chrono.ChForce.FORCE)
    myforcex.SetDir(chrono.VECT_X)
    myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
    #myforcex.SetMforce(mag)
    forceb.append(myforcex)
    # collision model
    ball.GetCollisionModel().ClearModel()
    ball.GetCollisionModel().AddCylinder(Rb,Rb,hhalf) # hemi sizes
    ball.GetCollisionModel().BuildModel()
    ball.SetCollide(True)
    ball.SetBodyFixed(False)
    col_b = chrono.ChColorAsset()
    col_b.SetColor(chrono.ChColor(0, 0, 1))
    ball.AddAsset(col_b)
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    prismatic_ground_ball = chrono.ChLinkLockPrismatic()
    prismatic_ground_ball.SetName("prismatic_ground_ball")
    prismatic_ground_ball.Initialize(body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(5.5, 0, 0), z2x))
    my_system.AddLink(prismatic_ground_ball)
    
    my_system.AddLink(pt)
    my_system.Add(ball)
    obj.append(ball)
    Balls.append(ball)
# In[Def Box] creates an object that is a box that can be pushed in 
def Box(x,y,z,Rb,RL,height,hhalf,rowr,material,obj,my_system,forceb,body_floor,ball):
    
    z2x = chrono.ChQuaternionD()
    z2x.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(0, 1, 0))
    ball = chrono.ChBody()
    ball = chrono.ChBodyEasyBox(2*Rb,height,2*RL,rowr)
    ball.SetPos(chrono.ChVectorD(x,y,z))
    ball.SetRot(chrono.Q_from_AngY(np.pi/4))
    ball.SetMaterialSurface(material)
    myforcex = chrono.ChForce()
    ball.AddForce(myforcex)
    myforcex.SetMode(chrono.ChForce.FORCE)
    myforcex.SetDir(chrono.VECT_X)
    myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
    #myforcex.SetMforce(mag)
    forceb.append(myforcex)
    # Collision shape
    ball.GetCollisionModel().ClearModel()
    ball.GetCollisionModel().AddBox(Rb,hhalf,Rb) # must set half sizes
    ball.GetCollisionModel().BuildModel()
    ball.SetCollide(True)
    ball.SetBodyFixed(False)
    col_b = chrono.ChColorAsset()
    col_b.SetColor(chrono.ChColor(0, 0, 1))
    ball.AddAsset(col_b)
    pt=chrono.ChLinkMatePlane()
    pt.Initialize(body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
    prismatic_ground_ball = chrono.ChLinkLockPrismatic()
    prismatic_ground_ball.SetName("prismatic_ground_ball")
    prismatic_ground_ball.Initialize(body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(5.5, 0, 0), z2x))
    my_system.AddLink(prismatic_ground_ball)
    my_system.AddLink(pt)
    my_system.Add(ball)
    obj.append(ball)    
    return (forceb,obj,ball)
# In[Wall] # creates a wall 
def Wall(x,y,z,rotate,length,height,width,material,my_system):
    body_brick = chrono.ChBody()
    body_brick.SetBodyFixed(True)
    # set initial position
    body_brick.SetPos(chrono.ChVectorD(x,y,z))
    body_brick.SetRot(chrono.Q_from_AngY(rotate))
    # set mass properties
    body_brick.SetMass(.5)
    body_brick.SetInertiaXX(chrono.ChVectorD(1,1,1))
    # set collision surface properties
    body_brick.SetMaterialSurface(material)
    # Collision shape
    body_brick.GetCollisionModel().ClearModel()
    body_brick.GetCollisionModel().AddBox(length/2, height/2, width/2) # must set half sizes
    body_brick.GetCollisionModel().BuildModel()
    body_brick.SetCollide(True)
    # Visualization shape, for rendering animation
    body_brick_shape = chrono.ChBoxShape()
    body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(length/2, height/2, width/2)
    col_brick = chrono.ChColorAsset()
    col_brick.SetColor(chrono.ChColor(1,0.1,0.5))     #Pink
    body_brick.AddAsset(col_brick)
    body_brick.GetAssets().push_back(body_brick_shape)
    body_brick.GetAssets().push_back(col_brick)
    my_system.Add(body_brick)    
 

# In[Call springs to be jammed]
def Jamsprings(k,rl,rlmax,Springs,t,tj,Fm,kj,rlj,rljmax,i,jamcall):
    if t>tj:
        if jamcall==1:
            k=kj
            rl=rlj
            rlmax=rljmax
        else:
            k=k
            rl=rl
            rlmax=rlmax
    else:
        k=k
        rl=rl
        rlmax=rlmax
    return(k,rlmax,rl)
# In[Set springs]   
def setSpring(k,rl,rlmax,Springs,Fm,i):
    var1=Springs[i].Get_SpringLength()
    if var1<rl:
        Springs[i].Set_SpringF(k)
        var2=Springs[i].Get_SpringF()    
    if var1>rlmax:
        Springs[i].Set_SpringF(k-10*(var1)**-1)
        var2=Springs[i].Get_SpringF()    
    else:
        Springs[i].Set_SpringF(k)
        var2=Springs[i].Get_SpringF()    
  
    Fm.append(var2)
    return(Springs,Fm)

# In[Tangent direction]
def norm_tangent(obj,nb):
# empty temporary matrices
    vtx=[]
    vtz=[]
    vnx=[]
    vnz=[]
    checkin=[]
    # tangent vector pointing in positive y direction
    vu=np.array([0,1,0])
    for i in range(nb-1):

        if i==0:
            # find tangent line of first one
            x1=obj[nb-1].GetPos().x   
            x2=obj[1].GetPos().x   
            z1=obj[nb-1].GetPos().z
            z2=obj[1].GetPos().z
            xt=x2-x1
            zt=z2-z1
            # append to empty matrix
            vtx.append(xt)
            vtz.append(zt)
        else:
            # find tangent lines for rest of them
            x1=obj[i-1].GetPos().x
            x2=obj[i+1].GetPos().x
            z1=obj[i-1].GetPos().z
            z2=obj[i+1].GetPos().z
        
            xt=x2-x1
            zt=z2-z1
            # append to empty matrix
            vtx.append(xt)
            vtz.append(zt)
    # last one
    i=nb-1
    x1=obj[i-1].GetPos().x
    x2=obj[0].GetPos().x
    z1=obj[i-1].GetPos().z
    z2=obj[0].GetPos().z
    xt=x2-x1
    zt=z2-z1
    vtx.append(xt)
    vtz.append(zt)
    # cconvert to matrix
    vtx2=np.asarray(vtx)
    vtz2=np.asarray(vtz)
    # find normal vector
    for i in range(nb):
        x=vtx2[i]
        z=vtz2[i]
        
        vt=np.array([x,0,z])
        # corss product 
        temp=np.cross(vu,vt)
        # append to empty matix
        vnx.append(temp[0])
        vnz.append(temp[2])
        checkin.append(np.cross(temp,vt))
        
        
    
    return(vtx2,vtz2,vnx,vnz,checkin)

# In[max values]
def MaxValues(R1,diameter,nb):
    Rin=R1
    ngrans1=int(Rin/(diameter))

    ri=np.zeros((1,ngrans1))
    ni=np.zeros((1,ngrans1))
    
    radii=Rin-(diameter/2)
    for i in range(ngrans1):
        remainder=((diameter))*i
        ri[:,i]=radii-remainder
        ni[:,i]=np.floor((ri[:,i]*np.pi*2)/diameter)

    ni=np.asarray(ni,dtype=int)

    return(ni)
       
# In[Extract data] 
def ExtractData(obj,nb,nt,Xpos,Ypos,Zpos,Xforce,Yforce,Zforce,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,ballp,Balls,Springs,Fm,templ,vnx,vnz,vtx,vtz,Vnx,Vnz,Vtx,Vtz):
       
    for i in range(nb):
        var1=(Springs[i].Get_SpringLength()-Springs[i].Get_SpringRestLength())
        templ.append(var1)  # append spring length
        Vnx.append(vnx[i])  # append normal x direction
        Vnz.append(vnz[i])  # append normal z direction
        Vtx.append(vtx[i])  # append tangent x direction
        Vtz.append(vtz[i])  # append tangent z direction
    for i in range(nt):
        tempx=obj[i].Get_Xforce()# temporary variables for force
        tempxx=obj[i].GetPos_dt()# temporary variables for velocity
        # Total forces (x,y,z)
        Xforce.append((tempx.x))  
        Yforce.append((tempx.y))
        Zforce.append((tempx.z))
        #positions(x,y,z)
        Xpos.append(obj[i].GetPos().x)
        Ypos.append((obj[i].GetPos().y))
        Zpos.append(obj[i].GetPos().z)
        # Rotation positions
        rott0.append(obj[i].GetRot().e0)
        rott1.append(obj[i].GetRot().e1)
        rott2.append(obj[i].GetRot().e2)
        rott3.append(obj[i].GetRot().e3)
        # velocities (x,y,z)
        Xvel.append(tempxx.x)
        Yvel.append(tempxx.y)
        Zvel.append(tempxx.z)
    # pull data for ball                      
    for i in range(len(Balls)):
        ballp.append(Balls[i].GetPos().x)   



    return (obj,Springs,Fm,ballp,Balls,templ,Xforce,Yforce,Zforce,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Vnx,Vnz,Vtx,Vtz)

# In[Extract data 2]
def ExtractData2(obj,nb,nt,Xpos,Ypos,Zpos,Xforce,Yforce,Zforce,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Springs,Fm,templ):
       
    for i in range(nb):
        var1=(Springs[i].Get_SpringLength()-Springs[i].Get_SpringRestLength())
        templ.append(var1)  # append spring length
        Fm=Springs[i].Get_SpringF()
    for i in range(nt):
        tempx=obj[i].Get_Xforce()# temporary variables for force
        tempxx=obj[i].GetPos_dt()# temporary variables for velocity
        # Total forces (x,y,z)
        Xforce.append((tempx.x))  
        Yforce.append((tempx.y))
        Zforce.append((tempx.z))
        #positions(x,y,z)
        Xpos.append(obj[i].GetPos().x)
        Ypos.append((obj[i].GetPos().y))
        Zpos.append(obj[i].GetPos().z)
        # Rotation positions
        rott0.append(obj[i].GetRot().e0)
        rott1.append(obj[i].GetRot().e1)
        rott2.append(obj[i].GetRot().e2)
        rott3.append(obj[i].GetRot().e3)
        # velocities (x,y,z)
        Xvel.append(tempxx.x)
        Yvel.append(tempxx.y)
        Zvel.append(tempxx.z)



    return (obj,Springs,Fm,templ,Xforce,Yforce,Zforce,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel) 

# In[Extract Data 3]
def ExtractData3(obj,nb,nt,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Xforce,Yforce,Zforce,Springs,Fm,sl):
    for i in range(nt):
        tempx=obj[i].Get_Xforce()# temporary variables for force
        tempxx=obj[i].GetPos_dt()# temporary variables for velocity
        # Total forces (x,y,z)
        Xforce.append((tempx.x))  
        Yforce.append((tempx.y))
        Zforce.append((tempx.z))
        #positions(x,y,z)
        Xpos.append(obj[i].GetPos().x)
        Ypos.append((obj[i].GetPos().y))
        Zpos.append(obj[i].GetPos().z)
        # Rotation positions
        rott0.append(obj[i].GetRot().e0)
        rott1.append(obj[i].GetRot().e1)
        rott2.append(obj[i].GetRot().e2)
        rott3.append(obj[i].GetRot().e3)
        # velocities (x,y,z)
        Xvel.append(tempxx.x)
        Yvel.append(tempxx.y)
        Zvel.append(tempxx.z)
    for i in range(nb):
        Fm.append(Springs[i].Get_SpringF())
        sl.append(Springs[i].Get_SpringLength())
        
    return(Xforce,Yforce,Zforce,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Fm,sl)
    
    
        
# In[Controller]
def Controller(my_system,force,botcall,obj,Springs,jamcall,normcall,forceb,mag,mag2,mag3,magf,k,kj,t,tj,tp,tset,Fm,nb,templ,rl,rlmax,rlj,rljmax):
    (vtx,vtz,vnx,vnz,checkin)=norm_tangent(obj,nb)
    for ii in range(nb):
        # Set spring lengths and initiate jamming if needed
        (k1,rlmax1,rl1)=Jamsprings(k,rl,rlmax,Springs,t,tj,Fm,kj,rlj,rljmax,ii,jamcall[0,ii])
        (Springs,Fm)=setSpring(k1,rl1,rlmax1,Springs,Fm,ii)   
        # if past settling time  
        if t > tset:
            # force on robots
            for j in range(len(force)):
                force[j].SetMforce(mag)
                force[j].SetDir(chrono.VECT_X)
            # force to push ball
            for j in range(len(forceb)):
                forceb[j].SetMforce(-mag2)
                forceb[j].SetDir(chrono.ChVectorD(1,0,0))
        # if past jamming time 
        if t > tj:
            for l in range(len(force)):
                if normcall[:,ii]==1:
                    
                    xd=-vnx[l]  # norm x vector
                    yd=0    # norm y vector
                    zd=-vnz[l]  # norm z vector
                    force[l].SetMforce(mag3)
                    force[l].SetDir(chrono.ChVectorD(xd,yd,zd))
            # force to push ball    
            for l in range(len(forceb)):
                forceb[l].SetMforce(-mag2) 
                forceb[l].SetDir(chrono.ChVectorD(1,0,0))
    
        #  past pulling time
        if t>tp:
#
#            for l in range(len(force)):
#                force[l].SetMforce(mag3)
#                force[l].SetDir(chrono.ChVectorD(xd,yd,zd))
            
            # force to pull ball
            for l in range(len(forceb)):
                forceb[l].SetMforce(1.5*magf)
                forceb[l].SetDir(chrono.ChVectorD(1,0,0))   
    return (force,Springs,templ,Fm,forceb,my_system,vnx,vnz,vtx,vtz,checkin)




# In[Controller2]
def Controller2(my_system,force,botcall,obj,Springs,jamcall,normcall,forceb,mag,mag2,mag3,magf,k,kj,t,tj,tp,tset,Fm,nb,templ,rl,rlmax,rlj,rljmax):
    (vtx,vtz,vnx,vnz,checkin)=norm_tangent(obj,nb)
    for ii in range(nb):
        # Set spring lengths and initiate jamming if needed
        (k1,rlmax1,rl1)=Jamsprings(k,rl,rlmax,Springs,t,tj,Fm,kj,rlj,rljmax,ii,jamcall[0,ii])
        (Springs,Fm)=setSpring(k1,rl1,rlmax1,Springs,Fm,ii)   
        # if past settling time  
        if t > tset:
            # force on robots
            for j in range(len(force)):
                force[j].SetMforce(mag)
                force[j].SetDir(chrono.VECT_X)
            # force to push ball
            for j in range(len(forceb)):
                forceb[j].SetMforce(-mag2)
                forceb[j].SetDir(chrono.ChVectorD(1,0,0))
        # if past jamming time 
        if t > tj:
            for l in range(len(force)):
                if normcall[:,ii]==1:
                    
                    xd=-vnx[l]  # norm x vector
                    yd=0    # norm y vector
                    zd=-vnz[l]  # norm z vector
                    force[l].SetMforce(mag3)
                    force[l].SetDir(chrono.ChVectorD(xd,yd,zd))
#                if l==0:
#                    force[l].SetMforce(-50)
#                    force[l].SetDir(chrono.ChVectorD(0,0,1))
#                if l==nb-4:
#                    force[l].SetMforce(50)
#                    force[l].SetDir(chrono.ChVectorD(0,0,1))
            # force to push ball    
            for l in range(len(forceb)):
                forceb[l].SetMforce(0) 
                forceb[l].SetDir(chrono.ChVectorD(1,0,0))
    
        #  past pulling time
        if t>tp:
#
#            for l in range(len(force)):
#                force[l].SetMforce(mag3)
#                force[l].SetDir(chrono.ChVectorD(xd,yd,zd))
            
            # force to pull ball
            for l in range(len(forceb)):
                forceb[l].SetMforce(1.5*magf)
                forceb[l].SetDir(chrono.ChVectorD(1,0,0))   
    return (force,Springs,templ,Fm,forceb,my_system,vnx,vnz,vtx,vtz,checkin)


# In[Controller 3]
    
def Controller3(my_system,force,botcall,obj,Springs,jamcall,Fm,mag,k,kj,t,tj,tp,tset,nb,templ,rl,rlmax,rlj,rljmax):
    for ii in range(nb):
        (k1,rlmax1,rl1)=Jamsprings(k,rl,rlmax,Springs,t,tj,Fm,kj,rlj,rljmax,ii,jamcall[0,ii])
        (Springs,Fm)=setSpring(k1,rl1,rlmax1,Springs,Fm,ii)  
# if past settling time       
        if t > tset:
            # force on robots
            for j in range(len(force)):
                force[j].SetMforce(mag)
                force[j].SetDir(chrono.VECT_Y)


    return (force,Springs,templ,my_system,Fm)

# In[Heading Angle]
def Heading_Angle(force,obj,goal,my_system,nb):
    alpha=[]
    for i in range(nb):
        rx=(obj[i].GetPos().x-goal[0]) # x position of r vector
        rz=(obj[i].GetPos().z-goal[2]) # z positon of r ector
        r=np.array([rx,rz])     # r vector
        rbar=np.linalg.norm(r)     # magnitude 
        
        hx=force[i].GetDir().x # heading vector x position
        hz=force[i].GetDir().z  # heading vector z position
        h=np.array([hx,hz])     # h vector
        hbar=np.linalg.norm(h)     # h magnitude

        # heading angle 
        theta=np.arccos(np.dot(h,r)/(hbar*rbar))
        alpha.append(theta)
    return(alpha)    

# In[Check if near target]
def Check_distance(obj,goal,my_system,nb,active,error):
    for i in range(nb):
        if active[i]==0:
            active[i]==0
        else:
            rx=(obj[i].GetPos().x-goal[0]) # x position of r vector
            rz=(obj[i].GetPos().z-goal[2]) # z positon of r ector
            r=np.array([rx,rz])     # r vector
            rbar=np.linalg.norm(r)     # magnitude
            if abs(rbar)<=error:
                active[i]=0
            else:
                active[i]=1
    return(active)        
# In[C active]    
def Cactive(res,active,alpha,ta,C,nb):
    for i in range(nb):
        if active[i]==1:
            if (np.pi/2+ta)<alpha[i]<(3*np.pi/2-ta):
                C[i]=-1
            elif 0<=alpha[i]<(np.pi/2-ta) or (3*np.pi/2+ta)<alpha[i]<=2*np.pi:
                C[i]=1
            else:
                C[i]=0
        else:
            C[i]=-res.x[i]*abs(alpha[i]>np.pi/2)+res.x[i]*(abs(alpha[i]<=2*np.pi))
    return C
# In[Controller 4]
def Controller4(my_system,force,obj,mag,goal,nb,active,ta,C,error):
    

    # calculate the heading angle
    alpha=Heading_Angle(force,obj,goal,my_system,nb)
    
    # check distance
    active=Check_distance(obj,goal,my_system,nb,active,error)   
    
    # run the optimization
    res = linprog(np.sin(alpha) ** 2 - np.cos(alpha) ** 2, method='simplex', bounds=[(0, 1)] * nb)
    
    # determine if it needs to go forward or backwards
    C=Cactive(res,active,alpha,ta,C,nb)
    
    for i in range(len(force)):
        p=mag*C[i]
        force[i].SetMforce(p)
        force[i].SetRelDir(chrono.ChVectorD(1,0,0))

    return(res,alpha,active,C)


# In[Export data]
def Exportdata(Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xvel,Yvel,Zvel,Fm,ballp,Vnx,Vnz,Vtx,Vtz,nb,nt,count,lengthm,nc,cx,cy,cz,Fxct,Fyct,Fzct):
    
#[Convert list to matrices]
    Xpos=np.asarray(Xpos)
    Ypos=np.asarray(Ypos)
    Zpos=np.asarray(Zpos)

    rott0=np.asarray(rott0)
    rott1=np.asarray(rott1)
    rott2=np.asarray(rott2)
    rott3=np.asarray(rott3)

    templ=np.asarray(templ)

    Xforce=np.asarray(Xforce)
    Yforce=np.asarray(Yforce)
    Zforce=np.asarray(Zforce)

    Xcontact=np.asarray(Xcontact)
    Ycontact=np.asarray(Ycontact)
    Zcontact=np.asarray(Zcontact)

    Xvel=np.asarray(Xvel)
    Yvel=np.asarray(Yvel)
    Zvel=np.asarray(Zvel)
    Fm=np.asarray(Fm)

    ballp=np.asarray(ballp)



    Vnx=np.asarray(Vnx)
    Vnz=np.asarray(Vnz)
    
    Vtx=np.asarray(Vtx)
    Vtz=np.asarray(Vtz)
    

# In[Create empty arrays]
# position
    qx=np.zeros((nt,count))
    qy=np.zeros((nt,count))
    qz=np.zeros((nt,count))

# empty toational matrices
    rot0=np.zeros((nt,count))
    rot1=np.zeros((nt,count))
    rot2=np.zeros((nt,count))
    rot3=np.zeros((nt,count))



# total forces
    Fxt=np.zeros((nt,count))
    Fyt=np.zeros((nt,count))
    Fzt=np.zeros((nt,count))
# Spring length
    SL=np.zeros((nb,count))

# Velocity empty matrices
    Xv=np.zeros((nt,count))
    Yv=np.zeros((nt,count))
    Zv=np.zeros((nt,count))

# Membrane force
    Fmem=np.zeros((nb,count))

#Create empty contact matrices
    xc=np.zeros((lengthm,count))
    yc=np.zeros((lengthm,count))
    zc=np.zeros((lengthm,count))
    # Contact forces
    Fcx=np.zeros((lengthm,count))
    Fcy=np.zeros((lengthm,count))
    Fcz=np.zeros((lengthm,count))

    VNX=np.zeros((nb,count))
    VNZ=np.zeros((nb,count))
    VTX=np.zeros((nb,count))
    VTZ=np.zeros((nb,count))
    # In[Fill the matrices]
    for i in range(count):    
        # fill the position matrices
        qx[:,i]=Xpos[nt*i:nt*i+nt]  # x position
        qy[:,i]=Ypos[nt*i:nt*i+nt]  # y position
        qz[:,i]=Zpos[nt*i:nt*i+nt]  # z position
  
        # fill the rotational matrices  
        rot0[:,i]=rott0[nt*i:nt*i+nt]    # quterion position 0
        rot1[:,i]=rott1[nt*i:nt*i+nt]    # quterion position 1
        rot2[:,i]=rott2[nt*i:nt*i+nt]    # quterion position 2
        rot3[:,i]=rott3[nt*i:nt*i+nt]    # quterion position 3
    
        # fill the total force matrices
        Fxt[:,i]=Xforce[nt*i:nt*i+nt]   # total force x
        Fyt[:,i]=Yforce[nt*i:nt*i+nt]   # total force y
        Fzt[:,i]=Zforce[nt*i:nt*i+nt]   # total force z
    

        # Fill for velocity
        Xv[:,i]=Xvel[nt*i:nt*i+nt]      # velocity x position
        Yv[:,i]=Yvel[nt*i:nt*i+nt]      # velocity y position
        Zv[:,i]=Zvel[nt*i:nt*i+nt]      # velocity z position
        # fill for spring length
        SL[:,i]=templ[nb*i:nb*i+nb]     # membrane length
        # fill for membrane force
        #Fmem[:,i]=Fm[nb*i:nb*i+nb]      # membrane force 
        # fill for normal direction
        VNX[:,i]=Vnx[nb*i:nb*i+nb]      # normal direction x
        VNZ[:,i]=Vnz[nb*i:nb*i+nb]      # normal direction z
        VTX[:,i]=Vtx[nb*i:nb*i+nb]      # tangent direction x
        VTZ[:,i]=Vtz[nb*i:nb*i+nb]      # tangent direction z
    #print(nc[0])
    for i in range(count):
        # temporary variables for contact and force chains
        ind=nc[i]
        tryme=cx[i] 
        tryme2=cy[i]
        tryme3=cz[i]
        tryme4=Fxct[i]
        tryme5=Fyct[i]
        tryme6=Fzct[i]
    
        # convert to array
        tryme=np.asarray(tryme)
        tryme2=np.asarray(tryme2)
        tryme3=np.asarray(tryme3)
        tryme4=np.asarray(tryme4)
        tryme5=np.asarray(tryme5)
        tryme6=np.asarray(tryme6)
    
        # fill array position
        xc[0:ind,i]=np.transpose(tryme)     # contact position x
        yc[0:ind,i]=np.transpose(tryme2)    # contact position y
        zc[0:ind,i]=np.transpose(tryme3)    # contact position z

        # Fill array forces
        Fcx[0:ind,i]=np.transpose(tryme4)   # Contact force x
        Fcy[0:ind,i]=np.transpose(tryme5)   # contact force y
        Fcz[0:ind,i]=np.transpose(tryme6)   # contact force z

    return(qx,qy,qz,rot0,rot1,rot2,rot3,Fxt,Fyt,Fzt,Xv,Yv,Zv,SL,VNX,VNZ,VTX,VTZ,Fcx,Fcy,Fcz,xc,yc,zc)
    
# In[Export data 2]
def Exportdata2(Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xvel,Yvel,Zvel,Fm,nb,nt,count,lengthm,nc,cx,cy,cz,Fxct,Fyct,Fzct):
    
#[Convert list to matrices]
    Xpos=np.asarray(Xpos)
    Ypos=np.asarray(Ypos)
    Zpos=np.asarray(Zpos)
# rotation positions
    rott0=np.asarray(rott0)
    rott1=np.asarray(rott1)
    rott2=np.asarray(rott2)
    rott3=np.asarray(rott3)
# spring length
    templ=np.asarray(templ)
# Force total 
    Xforce=np.asarray(Xforce)
    Yforce=np.asarray(Yforce)
    Zforce=np.asarray(Zforce)
# contact forces
    Xcontact=np.asarray(Xcontact)
    Ycontact=np.asarray(Ycontact)
    Zcontact=np.asarray(Zcontact)
# velocities
    Xvel=np.asarray(Xvel)
    Yvel=np.asarray(Yvel)
    Zvel=np.asarray(Zvel)
    # membrane force
    Fm=np.asarray(Fm)

# position
    qx=np.zeros((nt,count))
    qy=np.zeros((nt,count))
    qz=np.zeros((nt,count))

# empty toational matrices
    rot0=np.zeros((nt,count))
    rot1=np.zeros((nt,count))
    rot2=np.zeros((nt,count))
    rot3=np.zeros((nt,count))



# total forces
    Fxt=np.zeros((nt,count))
    Fyt=np.zeros((nt,count))
    Fzt=np.zeros((nt,count))
# Spring length
    SL=np.zeros((nb,count))

# Velocity empty matrices
    Xv=np.zeros((nt,count))
    Yv=np.zeros((nt,count))
    Zv=np.zeros((nt,count))

# Membrane force
    Fmem=np.zeros((nb,count))

#Create empty contact matrices
    xc=np.zeros((lengthm,count))
    yc=np.zeros((lengthm,count))
    zc=np.zeros((lengthm,count))
    # Contact forces
    Fcx=np.zeros((lengthm,count))
    Fcy=np.zeros((lengthm,count))
    Fcz=np.zeros((lengthm,count))


    # In[Fill the matrices]
    for i in range(count):    
        # fill the position matrices
        qx[:,i]=Xpos[nt*i:nt*i+nt]  # x position
        qy[:,i]=Ypos[nt*i:nt*i+nt]  # y position
        qz[:,i]=Zpos[nt*i:nt*i+nt]  # z position
  
        # fill the rotational matrices  
        rot0[:,i]=rott0[nt*i:nt*i+nt]    # quterion position 0
        rot1[:,i]=rott1[nt*i:nt*i+nt]    # quterion position 1
        rot2[:,i]=rott2[nt*i:nt*i+nt]    # quterion position 2
        rot3[:,i]=rott3[nt*i:nt*i+nt]    # quterion position 3
    
        # fill the total force matrices
        Fxt[:,i]=Xforce[nt*i:nt*i+nt]   # total force x
        Fyt[:,i]=Yforce[nt*i:nt*i+nt]   # total force y
        Fzt[:,i]=Zforce[nt*i:nt*i+nt]   # total force z
    

        # Fill for velocity
        Xv[:,i]=Xvel[nt*i:nt*i+nt]      # velocity x position
        Yv[:,i]=Yvel[nt*i:nt*i+nt]      # velocity y position
        Zv[:,i]=Zvel[nt*i:nt*i+nt]      # velocity z position
        # fill for spring length
        SL[:,i]=templ[nb*i:nb*i+nb]     # membrane length
        # fill for membrane force
        Fmem[:,i]=Fm[nb*i:nb*i+nb]      # membrane force 

    #print(nc[0])
    for i in range(count):
        # temporary variables for contact and force chains
        ind=nc[i]
        tryme=cx[i] 
        tryme2=cy[i]
        tryme3=cz[i]
        tryme4=Fxct[i]
        tryme5=Fyct[i]
        tryme6=Fzct[i]
    
        # convert to array
        tryme=np.asarray(tryme)
        tryme2=np.asarray(tryme2)
        tryme3=np.asarray(tryme3)
        tryme4=np.asarray(tryme4)
        tryme5=np.asarray(tryme5)
        tryme6=np.asarray(tryme6)
    
        # fill array position
        xc[0:ind,i]=np.transpose(tryme)     # contact position x
        yc[0:ind,i]=np.transpose(tryme2)    # contact position y
        zc[0:ind,i]=np.transpose(tryme3)    # contact position z

        # Fill array forces
        Fcx[0:ind,i]=np.transpose(tryme4)   # Contact force x
        Fcy[0:ind,i]=np.transpose(tryme5)   # contact force y
        Fcz[0:ind,i]=np.transpose(tryme6)   # contact force z

    return(qx,qy,qz,rot0,rot1,rot2,rot3,Fxt,Fyt,Fzt,Xv,Yv,Zv,SL,Fmem,Fcx,Fcy,Fcz,xc,yc,zc)


# In[Export Data 3]
    
def ExportData3(Xforce,Yforce,Zforce,Fxct,Fyct,Fzct,cx,cy,cz,nc,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,Fm,sl,nb,ni,nt,count,lengthm):
# Convert list to matrices
    Xpos=np.asarray(Xpos)
    Ypos=np.asarray(Ypos)
    Zpos=np.asarray(Zpos)
# rotation positions
    rott0=np.asarray(rott0)
    rott1=np.asarray(rott1)
    rott2=np.asarray(rott2)
    rott3=np.asarray(rott3)
# spring length
    sl=np.asarray(sl)
# Force total 
    Xforce=np.asarray(Xforce)
    Yforce=np.asarray(Yforce)
    Zforce=np.asarray(Zforce)

# velocities
    Xvel=np.asarray(Xvel)
    Yvel=np.asarray(Yvel)
    Zvel=np.asarray(Zvel)
    # membrane force
    Fm=np.asarray(Fm)
    
########    EMPTY MATRICES      ##################        

# position
    qx=np.zeros((nt,count))
    qy=np.zeros((nt,count))
    qz=np.zeros((nt,count))

# empty toational matrices
    rot0=np.zeros((nt,count))
    rot1=np.zeros((nt,count))
    rot2=np.zeros((nt,count))
    rot3=np.zeros((nt,count))
# total forces
    Fxt=np.zeros((nt,count))
    Fyt=np.zeros((nt,count))
    Fzt=np.zeros((nt,count))
# Spring length
    SL=np.zeros((nb,count))

# Velocity empty matrices
    Xv=np.zeros((nt,count))
    Yv=np.zeros((nt,count))
    Zv=np.zeros((nt,count))

# Membrane force
    Fmem=np.zeros((nb,count))

#Create empty contact matrices
    xc=np.zeros((lengthm,count))
    yc=np.zeros((lengthm,count))
    zc=np.zeros((lengthm,count))
    # Contact forces
    Fcx=np.zeros((lengthm,count))
    Fcy=np.zeros((lengthm,count))
    Fcz=np.zeros((lengthm,count))

    for i in range(count):    
        # fill the position matrices
        qx[:,i]=Xpos[nt*i:nt*i+nt]  # x position
        qy[:,i]=Ypos[nt*i:nt*i+nt]  # y position
        qz[:,i]=Zpos[nt*i:nt*i+nt]  # z position
  
        # fill the rotational matrices  
        rot0[:,i]=rott0[nt*i:nt*i+nt]    # quterion position 0
        rot1[:,i]=rott1[nt*i:nt*i+nt]    # quterion position 1
        rot2[:,i]=rott2[nt*i:nt*i+nt]    # quterion position 2
        rot3[:,i]=rott3[nt*i:nt*i+nt]    # quterion position 3
    
        # fill the total force matrices
        Fxt[:,i]=Xforce[nt*i:nt*i+nt]   # total force x
        Fyt[:,i]=Yforce[nt*i:nt*i+nt]   # total force y
        Fzt[:,i]=Zforce[nt*i:nt*i+nt]   # total force z
    

        # Fill for velocity
        Xv[:,i]=Xvel[nt*i:nt*i+nt]      # velocity x position
        Yv[:,i]=Yvel[nt*i:nt*i+nt]      # velocity y position
        Zv[:,i]=Zvel[nt*i:nt*i+nt]      # velocity z position
        # fill for spring length
        SL[:,i]=sl[nb*i:nb*i+nb]     # membrane length
        # fill for membrane force
        Fmem[:,i]=Fm[nb*i:nb*i+nb]      # membrane force 

    #print(nc[0])
    for i in range(count):
        # temporary variables for contact and force chains
        ind=nc[i]
        tryme1=cx[i]
        tryme2=cy[i]
        tryme3=cz[i]
        tryme4=Fxct[i]
        tryme5=Fyct[i]
        tryme6=Fzct[i]
    
        # convert to array
        tryme1=np.asarray(tryme1)
        tryme2=np.asarray(tryme2)
        tryme3=np.asarray(tryme3)
        tryme4=np.asarray(tryme4)
        tryme5=np.asarray(tryme5)
        tryme6=np.asarray(tryme6)
    
        # fill array position
        xc[0:ind,i]=np.transpose(tryme1)     # contact position x
        yc[0:ind,i]=np.transpose(tryme2)    # contact position y
        zc[0:ind,i]=np.transpose(tryme3)    # contact position z

        # Fill array forces
        Fcx[0:ind,i]=np.transpose(tryme4)   # Contact force x
        Fcy[0:ind,i]=np.transpose(tryme5)   # contact force y
        Fcz[0:ind,i]=np.transpose(tryme6)   # contact force z
        
    return(xc,yc,zc,Fcx,Fcy,Fcz,Xv,Yv,Zv,Fmem,rot0,rot1,rot2,rot3,qx,qy,qz,Fxt,Fyt,Fzt,SL)
# In[Packing Fraction]
def PackingFraction(script_dir,time,nb,ni,diameter,height,qx,qz,file,count,k):
    results_dir = os.path.join(script_dir, 'packing fraction/')
    
    if not os.path.isdir(results_dir):
        os.makedirs(results_dir) 
    
    A=np.zeros(count)
    Atemp=np.zeros(nb) 

    const= .25*ni*(np.pi)*(diameter)**2
 
    for j in range(count):
        for i in range(nb-1):
            Atemp[i]=.5*(qx[i+1,j]+qx[i,j])*(qz[i+1,j]-qz[i,j])
        i=nb-1
        Atemp[i]=.5*(qx[i,j]+qx[0,j])*(qz[0,j]-qz[i,j])
    
        A[j]=const/sum(Atemp)


    plt.figure(1)

    plt.plot(time,A,'b')
    plt.title('Packing Fraction vs Time[s]')
    plt.xlabel('Time')
    plt.ylabel('Packing Fraction')
    plt.grid(True)
    plt.savefig(results_dir+" Packing fraction"+file)  
    plt.close('all')
    
    return(A[-1])


# In[Number of contacts]

class MyReportContactCallback(chrono.ReportContactCallback):

    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Fcx=[]
        self.Fcy=[]
        self.Fcz=[]
        self.pointx = []
        self.pointy = []
        self.pointz = []
        #self.bodies = []
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
#        bodyUpA = chrono.CastContactableToChBody(modA)
#        nameA = bodyUpA.GetId()
#        bodyUpB = chrono.CastContactableToChBody(modB)
#        nameB = bodyUpB.GetId()
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fcx.append(force.x)
        self.Fcy.append(force.y)
        self.Fcz.append(force.z)
        #self.bodies.append([nameA,nameB])
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = [] 
        self.Fcx=[]
        self.Fcy=[]
        self.Fcz=[]
    # Get the points
    def GetList(self):
        return (self.pointx,self.pointy,self.pointz,self.Fcx,self.Fcy,self.Fcz)
    

# In[Controls] DONT CONVERT ME 
class PidControl:
    def __init__(self, nb, mag, tar, XdirForce, YdirForce,ZdirForce,XposBody,YposBody,ZposBody,Kp, Ki, Kd):
        self.nb = nb
        self.mag = mag
        self.tar = tar
        #directions
        self.XdirForce = XdirForce
        self.YdirForce = YdirForce
        self.ZdirForce = ZdirForce
        self.XposBody = XposBody
        self.YposBody = YposBody
        self.ZposBody = ZposBody
        self.angle_threshold = 2 
        self.pos_threshold  = 0.1
        # control factors
        #self.Kp=Kp
        #self.Ki=Ki
        #self.Kd=Kd
        
        # Initialize the controllers
        self.pid = PID(p=Kp, i=Ki, d=Kd)
     
        self.Tlocation = [self.tar[0],self.tar[1],self.tar[2]]
        self.dirF = [self.XdirForce,self.YdirForce,self.YdirForce]
    
        self.XcenPos = XposBody
        self.YcenPos = YposBody
        self.ZcenPos = ZposBody
        
        self.cur_angle = self.start_control_loop()
        self.intarget = self.reaching_target()
        
    def set_curangle(self):
        self.cur_angle = np.arctan(self.dirF[2]/self.dirF[0])
        self.cur_angle = np.degrees(self.cur_angle)
        return self.cur_angle
        
            
    def set_target(self):
        Zaxis = self.Tlocation[2] - self.ZcenPos
        Xaxis = self.Tlocation[0] - self.XcenPos
        tar_angle = np.degrees(np.arctan((Zaxis/Xaxis)))
        
        if Zaxis > 0.01:    
            if -0.01<= Xaxis <= 0.01:
               return 90
            if Xaxis < -0.01:
               return 180 + tar_angle
            if Xaxis > 0.01:
               return tar_angle
           
        if Zaxis < -0.01:
            if -0.01 <= Xaxis <= 0.01:
               return -90
            if Xaxis > 0.01:
               return tar_angle
            if Xaxis < -0.01:
               return tar_angle-180
           
        if -0.01 <= Zaxis <= 0.01:
            if -0.01 <= Xaxis <= 0.01:
               return 0
            if Xaxis < -0.01:
               return 180
            if Xaxis > 0.01:
               return 0  
                 
  
    def start_control_loop(self):
        while True:
            tar_angle = self.set_target()
            self.pid.target = tar_angle
            self.cur_angle = self.set_curangle()
            while True:
                self.pid(feedback=self.cur_angle)
                error = self.pid.error
                angles_are_set = (abs(error) < self.angle_threshold)  # Verify if every angles are within the tolerances
                if angles_are_set:
                    return self.cur_angle
                    break
                else:
                    if tar_angle >= 0:
                        self.cur_angle = self.cur_angle + 2   
                    
                    else:
                        self.cur_angle = self.cur_angle - 2
                        
                        
    def reaching_target(self):

        
        dis = np.sqrt((self.XposBody-self.tar[0])**2 + (self.ZposBody-self.tar[2])**2)
        
            
        if dis<self.pos_threshold:
            return 0
        else: 
            return self.mag



    

# In[Commented out codes not used]




#def direction(Xavg,Zavg,obj,ii,height):
#    xbar=Xavg-obj[ii].GetPos().x
#    zbar=Zavg-obj[ii].GetPos().z
#    magn=np.sqrt((xbar**2)+(zbar**2))
#    xd=xbar/magn
#    zd=zbar/magn
#    yd=height/2
#    
#    return(xd,yd,zd)


## In[Centroid]
#def centroid(obj,nb,Xpos,Ypos,Zpos,tj,t,tstep):
#    
#    if t>tj:
#        i=tj/tstep
#        i=int(i)
#    else:
#        i=t/tstep
#        i=int(i)
#        
#    Xpos2=np.asarray(Xpos)
#    Zpos2=np.asarray(Zpos)
#   
#    Xpostemp=Xpos2[nb*i:nb*i+nb]
#    Zpostemp=Zpos2[nb*i:nb*i+nb]
#    Xavg=np.mean(Xpostemp)
#    Zavg=np.mean(Zpostemp)
#    return (Xavg,Zavg)                   
#
#def centroid2(obj,nb):
#    for i in range(nb):
#        Xpostemp=obj[i].GetPos().x
#        Zpostemp=obj[i].GetPos().z
#        
#    Xavg=np.sum(Xpostemp)/(nb)
#    Zavg=np.sum(Zpostemp)/(nb)
#    return (Xavg,Zavg)
            
        
        
## In[Extract Data 3 this has controls in it ] DONT CONVERT ME
#def ExtractData3(my_system,obj,i,force,forceb,mag,mag2,magf,rl,rlj,
#                 rlmax,rljmax,Springs,Fm,k,ballp,Balls,templ,Xforce,
#                 Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xpos,Ypos,Zpos,
#                 rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,nb,nt,t,tj,tset,tp,
#                 jamcall,kj,XdirForce,YdirForce,ZdirForce,tarpos,Kp,Ki,Kd):
#    
#    # Apply forces to robots
#    for i in range(len(force)):
#        force[i].SetDir(chrono.VECT_X)
#
#        
#    for i in range(nb):
#        
#        # if past settling time  
#        if t > tset:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#
#
#                # Control
#                # target positon of robot i 
#                tar=tarpos[i,:]
#                x=obj[i].GetPos().x
#                y=obj[i].GetPos().y
#                z=obj[i].GetPos().z
#                control = PidControl(nb,mag,tar,XdirForce,YdirForce,ZdirForce,x,y,z,Kp,Ki,Kd)
#                forcemag = control.intarget
#                force[i].SetDir(chrono.ChVectorD(np.cos(np.radians(control.cur_angle)),0,np.sin(np.radians(control.cur_angle))))
#                force[i].SetMforce(forcemag)
#                    
#                    
#            for i in range(len(forceb)):
#                forceb[i].SetMforce(-mag2)
#                forceb[i].SetDir(chrono.ChVectorD(1,0,0))
#        # if past jamming time 
#        if t > tj:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#        
#            for i in range(len(forceb)):
#                forceb[i].SetMforce(-mag2) 
#                forceb[i].SetDir(chrono.ChVectorD(1,0,0))
#        # if past pulling time
#        if t>tp:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#            for i in range(len(forceb)):
#                forceb[i].SetMforce(1*magf)
#                forceb[i].SetDir(chrono.ChVectorD(1,0,0))   
## Jam certain robots if jam time is hit
#        if t>tj:
#            if jamcall[:,i]==1:
#                rl=rlj
#                rlmax=rljmax
#  
#                Springs[i].Set_SpringF(kj)
#            
#            else:
#                rl=rl
#                rlmax=rlmax
#            
#         #Check if springs are correct length otherwise adjust            
#        var1=Springs[i].Get_SpringLength()
#        if var1<rl:
#            Springs[i].Set_SpringF(0)
#            var2=0
#        if var1>rlmax:
#            Springs[i].Set_SpringF(2*k)
#            var2=2*k
#        else:
#            Springs[i].Set_SpringF(k)
#            var2=k
#    
#    for i in range(nb):
#        
#        var1=(Springs[i].Get_SpringLength()-Springs[i].Get_SpringRestLength())
#        var2=Springs[i].Get_SpringF()
#        Fm.append(var2)
#        templ.append(var1)
#        
#
## Pull data for plots                  
#    for i in range(nt):
#            
#        templ.append(var1)
#        temp=obj[i].GetContactForce()
#        tempx=obj[i].Get_Xforce()
#        tempxx=obj[i].GetPos_dt()
#         
#            
#                       
#        Xforce.append(tempx.x)
#        Yforce.append((tempx.y))
#        Zforce.append(tempx.z)
#            
#        Xcontact.append(temp.x)
#        Ycontact.append(temp.y)
#        Zcontact.append(temp.z)
#            
#        Xpos.append(obj[i].GetPos().x)
#        Ypos.append((obj[i].GetPos().y))
#        Zpos.append(obj[i].GetPos().z)
#            
#        rott0.append(obj[i].GetRot().e0)
#        rott1.append(obj[i].GetRot().e1)
#        rott2.append(obj[i].GetRot().e2)
#        rott3.append(obj[i].GetRot().e3)
#            
#   
#        # fill in the temporary velocity matrices
#        Xvel.append(tempxx.x)
#        Yvel.append(tempxx.y)
#        Zvel.append(tempxx.z)
#
#    # pull data for ball                      
#    for i in range(len(Balls)):
#        ballp.append(Balls[i].GetPos().x)   
#
#    return (my_system,obj,force,forceb,Springs,Fm,k,ballp,Balls,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel)


# In[Number of contacts]
    


# In[Controls]




   
# def interior_radii(nb,diameter,R1):

#     #nb=100             # number of robots
#     #diameter=.07        # diameter of cylinder and robots
#     #R1=(diameter*nb/(np.pi*2))+.1 
#     Rin=R1-diameter/2  
#     ngrans1=int(Rin/diameter)


#     ri=np.zeros((1,ngrans1))
#     ni=np.zeros((1,ngrans1))

    

#     for i in range(ngrans1):
    
#         ri[:,i]=Rin-i*diameter/2
#         ni[:,i]=np.floor((ri[:,i]*np.pi**2)/diameter)
#     ni=np.asarray(ni,dtype=int)
            
#     return ni

        

# In[Extract data] DONT CONVERT ME
#def ExtractData(my_system,obj,i,force,forceb,mag,mag2,magf,rl,rlj,rlmax,rljmax,Springs,Fm,k,ballp,Balls,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,nb,nt,t,tj,tset,tp,jamcall,kj):
#    
#    # Apply forces to robots
#    for i in range(len(force)):
#        force[i].SetDir(chrono.VECT_X)
#    
#    for i in range(nb):
#        # if past settling time  
#        if t > tset:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#            for i in range(len(forceb)):
#                forceb[i].SetMforce(-mag2)
#                forceb[i].SetDir(chrono.ChVectorD(1,0,0))
#        # if past jamming time 
#        if t > tj:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#        
#            for i in range(len(forceb)):
#                forceb[i].SetMforce(-mag2) 
#                forceb[i].SetDir(chrono.ChVectorD(1,0,0))
#        if t>tp:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#            for i in range(len(forceb)):
#                forceb[i].SetMforce(1*magf)
#                forceb[i].SetDir(chrono.ChVectorD(1,0,0))   
## Jam certain robots if jam time is hit
#        if t>tj:
#            if jamcall[:,i]==1:
#                rl=rlj
#                rlmax=rljmax
#  
#                Springs[i].Set_SpringF(kj)
#            
#            else:
#                rl=rl
#                rlmax=rlmax
#            
#         #Check if spirngs are correct length otherwise adjust            
#        var1=Springs[i].Get_SpringLength()
#        if var1<rl:
#            Springs[i].Set_SpringK(0)
#            var2=0
#        if var1>rlmax:
#            Springs[i].Set_SpringK(2*k)
#            var2=2*k
#        else:
#            Springs[i].Set_SpringK(k)
#            var2=k
#    
#    for i in range(nb):
#        
#        var1=(Springs[i].Get_SpringLength()-Springs[i].Get_SpringRestLength())
#        var2=Springs[i].Get_SpringK()
#        var3=Springs[i].Get_SpringVelocity()
#        var4=Springs[i].Get_SpringR()              
#        Fm.append(var1*var2+var3*var4)
#        templ.append(var1)
#         
#
## Pull data for plots                  
#    for i in range(nt):
#            
#        templ.append(var1)
#        temp=obj[i].GetContactForce()
#        tempx=obj[i].Get_Xforce()
#        tempxx=obj[i].GetPos_dt()
#         
#            
#                       
#        Xforce.append(tempx.x)
#        Yforce.append((tempx.y))
#        Zforce.append(tempx.z)
#            
#        Xcontact.append(temp.x)
#        Ycontact.append(temp.y)
#        Zcontact.append(temp.z)
#            
#        Xpos.append(obj[i].GetPos().x)
#        Ypos.append((obj[i].GetPos().y))
#        Zpos.append(obj[i].GetPos().z)
#        
#        XdirForce.append(dirForce.x)
#        YdirForce.append(dirForce.y)
#        ZdirForce.append(dirForce.z)
#        
#        rott0.append(obj[i].GetRot().e0)
#        rott1.append(obj[i].GetRot().e1)
#        rott2.append(obj[i].GetRot().e2)
#        rott3.append(obj[i].GetRot().e3)
#            
#   
#        # fill in the temporary velocity matrices
#        Xvel.append(tempxx.x)
#        Yvel.append(tempxx.y)
#        Zvel.append(tempxx.z)
#
#    # pull data for ball                      
#    for i in range(len(Balls)):
#        ballp.append(Balls[i].GetPos().x)   
#
#    return (my_system,obj,force,forceb,Springs,Fm,k,ballp,Balls,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel)


            
# In[Extract const force]

#def ExtractData2(my_system,obj,i,force,forceb,mag,mag2,magf,rl,rlj,rlmax,rljmax,Springs,Fm,k,ballp,Balls,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,nb,nt,t,tj,tset,tp,jamcall,kj):
#    
#    # Apply forces to robots
#    for i in range(len(force)):
#        force[i].SetDir(chrono.VECT_X)
#    
#    for i in range(nb):
#        (k1,rlmax1,rl1)=Jamsprings(k,rl,rlmax,Springs,t,tj,Fm,kj,rlj,rljmax,i,jamcall[0,i])
#         
#        
#           
#        (Springs,Fm,templ)=setSpring(k1,rl1,rlmax1,Springs,Fm,templ,i)   
#        # if past settling time  
#        if t > tset:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#            for j in range(len(forceb)):
#                forceb[j].SetMforce(-mag2)
#                forceb[j].SetDir(chrono.ChVectorD(1,0,0))
#        # if past jamming time 
#        if t > tj:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#        
#            for j in range(len(forceb)):
#                forceb[j].SetMforce(-mag2) 
#                forceb[j].SetDir(chrono.ChVectorD(1,0,0))
#        # if past pulling time
#        if t>tp:
#            for i in range(len(force)):
#                force[i].SetMforce(mag)
#            for j in range(len(forceb)):
#                forceb[j].SetMforce(1*magf)
#                forceb[j].SetDir(chrono.ChVectorD(1,0,0))   
#
#
## Pull data for plots                  
#    for i in range(nt):
#            
#        
#        temp=obj[i].GetContactForce()
#        tempx=obj[i].Get_Xforce()
#        tempxx=obj[i].GetPos_dt()
#         
#            
#                       
#        Xforce.append(tempx.x)
#        Yforce.append((tempx.y))
#        Zforce.append(tempx.z)
#            
#        Xcontact.append(temp.x)
#        Ycontact.append(temp.y)
#        Zcontact.append(temp.z)
#            
#        Xpos.append(obj[i].GetPos().x)
#        Ypos.append((obj[i].GetPos().y))
#        Zpos.append(obj[i].GetPos().z)
#            
#        rott0.append(obj[i].GetRot().e0)
#        rott1.append(obj[i].GetRot().e1)
#        rott2.append(obj[i].GetRot().e2)
#        rott3.append(obj[i].GetRot().e3)
#            
#   
#        # fill in the temporary velocity matrices
#        Xvel.append(tempxx.x)
#        Yvel.append(tempxx.y)
#        Zvel.append(tempxx.z)
#
#    # pull data for ball                      
#    for i in range(len(Balls)):
#        ballp.append(Balls[i].GetPos().x)   
#
#    return (my_system,obj,force,forceb,Springs,Fm,k,ballp,Balls,templ,Xforce,Yforce,Zforce,Xcontact,Ycontact,Zcontact,Xpos,Ypos,Zpos,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel)
#

