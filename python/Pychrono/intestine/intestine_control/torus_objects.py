# -*- coding: utf-8 -*-
"""
Created on Thu Nov 14 17:56:34 2019

@author: dmulr
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
import timeit
from pid_controller.pid import PID
from scipy.optimize import LinearConstraint, Bounds, minimize, linprog
import random
from numpy import pi


# In[Create material]
def Material(mu_f, mu_b, mu_r, mu_s, C, Ct, Cr, Cs):
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(mu_f)
    material.SetDampingF(mu_b)
    material.SetCompliance(C)
    material.SetComplianceT(Ct)
    material.SetRollingFriction(mu_r)
    material.SetSpinningFriction(mu_s)
    material.SetComplianceRolling(Cr)
    material.SetComplianceSpinning(Cs)
    return material


# In[Create Floor]
def Floor(material, length, tall):
    body_floor = chrono.ChBody()
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVectorD(0, -tall, 0))
    body_floor.SetMaterialSurface(material)
    body_floor.GetCollisionModel().ClearModel()
    body_floor.GetCollisionModel().AddBox(length, tall, length)  # hemi sizes
    body_floor.GetCollisionModel().BuildModel()
    body_floor.SetCollide(True)
    body_floor_shape = chrono.ChBoxShape()
    body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(length, tall, length)
    body_floor.GetAssets().push_back(body_floor_shape)
    body_floor_texture = chrono.ChTexture()
    body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    body_floor.GetAssets().push_back(body_floor_texture)
    return (body_floor)

 
    
# In[Bots with no attatchment]
def BOTS_free(nb, i, x, y, z, theta, diameter, height, rowr, roww,  force, obj, material, body_floor, my_system, Springs, k,rl,botcall,bots):
# if active        
    if botcall[:,i]==1:
        # Create bots
        bot = chrono.ChBody()
        bot = chrono.ChBodyEasyCylinder(diameter / 2, height, rowr)
        bot.SetPos(chrono.ChVectorD(x, y, z))
        # rotate coordinate frame
        vector=chrono.ChVectorD(x,y,z)
        my_CS = chrono.ChCoordsysD(vector,chrono.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0) ) )
        bot.SetCoord(my_CS)
        # set surface material
        bot.SetMaterialSurface(material)
       # create collision model
        bot.GetCollisionModel().ClearModel()
        bot.GetCollisionModel().AddCylinder(diameter / 2, diameter / 2, height / 2)  # hemi sizes
        bot.GetCollisionModel().BuildModel()
        bot.SetCollide(True)
        bot.SetBodyFixed(False)
        # apply force
        myforcex = chrono.ChForce()
        bot.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetRelDir(chrono.ChVectorD(0, 0, 1))
        myforcex.SetMforce(0)
        force.append(myforcex)
        
        mfunx = chrono.ChFunction_Sine(0,600,7) # phase, frequency, amplitude
        # apply vibrating force
        vforce1 = chrono.ChForce()
        bot.AddForce(vforce1)
        vforce1.SetMode(chrono.ChForce.FORCE)
        vforce1.SetRelDir(chrono.ChVectorD(0,1,0))
        vforce1.SetVrelpoint(chrono.ChVectorD(0,-height/2,diameter/4))
        vforce1.SetModulation(mfunx)
        # apply vibrating force
        vforce2 = chrono.ChForce()
        bot.AddForce(vforce2)
        vforce2.SetMode(chrono.ChForce.FORCE)
        vforce2.SetRelDir(chrono.ChVectorD(0,1,0))
        vforce2.SetVrelpoint(chrono.ChVectorD(0,-height/2,-diameter/4))
        vforce2.SetModulation(mfunx)
        # add coloring 
        body_floor_texture = chrono.ChTexture()
        body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
        bot.GetAssets().push_back(body_floor_texture)
# if passive 
    if botcall[:,i]==0:

        # Create bots
        bot = chrono.ChBody()
        bot = chrono.ChBodyEasyCylinder(diameter / 2, height, roww)
        bot.SetPos(chrono.ChVectorD(x, y, z))
        # rotate coordinate frame 
        vector=chrono.ChVectorD(x,y,z)
        my_CS = chrono.ChCoordsysD(vector,chrono.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0) ) )
        bot.SetCoord(my_CS)
        # set material 
        bot.SetMaterialSurface(material)
        # create collision model 
        bot.GetCollisionModel().ClearModel()
        bot.GetCollisionModel().AddCylinder(diameter / 2, diameter / 2, height / 2)  # hemi sizes
        bot.GetCollisionModel().BuildModel()
        bot.SetCollide(True)
        bot.SetBodyFixed(False)
        # set color 
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 1, 0))
        bot.AddAsset(col_g)   
    # link to the floor
        pt = chrono.ChLinkMatePlane()
        pt.Initialize(body_floor, bot, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0),chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, -1, 0))
        my_system.AddLink(pt)
        body_floor_texture = chrono.ChTexture()
        body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
        bot.GetAssets().push_back(body_floor_texture)

    # link springs
    if i >= 1:
        ground = chrono.ChLinkSpring()
        # Identify points to be attatched to the springs
        ground.SetName("ground")
        # set points
#        p1=-(diameter/2)*np.sin((i-1) * 2 * np.pi / (nb))
#        p2=(diameter/2)*np.cos((i-1) * 2 * np.pi / (nb))
#        p3=(diameter/2)*np.sin(theta)
#        p4=-(diameter/2)*np.cos(theta)
        h=0
        p2=(diameter/2)
        p1=0
        p4=-(diameter/2)
        p3=0
        # Attatches  springs
        ground.Initialize(obj[i - 1], bot, True, chrono.ChVectorD(p1, h, p2), chrono.ChVectorD(p3, h, p4), False)
        # set force
        ground.Set_SpringF(k)
        # set resting length
        ground.Set_SpringRestLength(rl)
        # add color
        col1 = chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0, 0, 1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
        # add to system 
        my_system.AddLink(ground)
        Springs.append(ground)

    # Last spring
    if i == nb - 1:
        # set points for last spring
#        p1=-(diameter/2)*np.sin((i) * 2 * np.pi / (nb))
#        p2=(diameter/2)*np.cos((i) * 2 * np.pi / (nb))
#        p3=(diameter/2)*np.sin(0)
#        p4=-(diameter/2)*np.cos(0)
        p2=(diameter/2)
        p1=0
        p4=-(diameter/2)
        p3=0
        # create and attath spring
        ground = chrono.ChLinkSpring()
        ground.SetName("ground")
        ground.Initialize(bot, obj[0], True, chrono.ChVectorD(p1, h, p2), chrono.ChVectorD(p3, h, p4), False)
        # set force 
        ground.Set_SpringF(k)
        # set resitng length
        ground.Set_SpringRestLength(rl)
        # show appearence
        col1 = chrono.ChColorAsset()
        col1.SetColor(chrono.ChColor(0, 0, 1))
        ground.AddAsset(col1)
        ground.AddAsset(chrono.ChPointPointSpring(.01, 80, 15))
        # add spring
        my_system.AddLink(ground)
        Springs.append(ground)

    if botcall[:,i]==1:
        bots.append(bot)
# add bot to system 
    my_system.Add(bot)
    obj.append(bot)

    return (force, my_system, obj, Springs,bots)





# In[Create Goal visual]
def GOAL(goal, my_system, diameter):
    gpoint = chrono.ChBody()
    gpoint = chrono.ChBodyEasySphere(diameter / 2, 1000)
    gpoint.SetPos(chrono.ChVectorD(goal[0], goal[1], goal[2]))
    gpoint.SetBodyFixed(True)
    col_p = chrono.ChColorAsset()
    col_p.SetColor(chrono.ChColor(0, 1, 0))
    gpoint.AddAsset(col_p)
    my_system.Add(gpoint)

    return (my_system)



# In[Ball]
def Ball(x, y, z, Rb, height, hhalf, rowr, material, obj, my_system, forceb, body_floor, Balls):
    z2x = chrono.ChQuaternionD()
    z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
    ball = chrono.ChBody()
    ball = chrono.ChBodyEasyCylinder(Rb, height, rowr)
    ball.SetPos(chrono.ChVectorD(x, y, z))
    ball.SetMaterialSurface(material)
    myforcex = chrono.ChForce()
    ball.AddForce(myforcex)
    myforcex.SetMode(chrono.ChForce.FORCE)
    myforcex.SetDir(chrono.VECT_X)
    myforcex.SetVrelpoint(chrono.ChVectorD(x, .03 * y, z))
    # myforcex.SetMforce(mag)
    forceb.append(myforcex)
    # collision model
    ball.GetCollisionModel().ClearModel()
    ball.GetCollisionModel().AddCylinder(Rb, Rb, hhalf)  # hemi sizes
    ball.GetCollisionModel().BuildModel()
    ball.SetCollide(True)
    ball.SetBodyFixed(False)
    col_b = chrono.ChColorAsset()
    col_b.SetColor(chrono.ChColor(0, 0, 1))
    ball.AddAsset(col_b)
    pt = chrono.ChLinkMatePlane()
    pt.Initialize(body_floor, ball, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0),
                  chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, -1, 0))
    prismatic_ground_ball = chrono.ChLinkLockPrismatic()
    prismatic_ground_ball.SetName("prismatic_ground_ball")
    prismatic_ground_ball.Initialize(body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(5.5, 0, 0), z2x))
    my_system.AddLink(prismatic_ground_ball)

    my_system.AddLink(pt)
    my_system.Add(ball)
    obj.append(ball)
    Balls.append(ball)


# In[Def Box] creates an object that is a box that can be pushed in
def Box(x, y, z, Rb, RL, height, hhalf, rowr, material, obj, my_system, forceb, body_floor, ball):
    z2x = chrono.ChQuaternionD()
    z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
    ball = chrono.ChBody()
    ball = chrono.ChBodyEasyBox(2 * Rb, height, 2 * RL, rowr)
    ball.SetPos(chrono.ChVectorD(x, y, z))
    ball.SetRot(chrono.Q_from_AngY(np.pi / 4))
    ball.SetMaterialSurface(material)
    myforcex = chrono.ChForce()
    ball.AddForce(myforcex)
    myforcex.SetMode(chrono.ChForce.FORCE)
    myforcex.SetDir(chrono.VECT_X)
    myforcex.SetVrelpoint(chrono.ChVectorD(x, .03 * y, z))
    # myforcex.SetMforce(mag)
    forceb.append(myforcex)
    # Collision shape
    ball.GetCollisionModel().ClearModel()
    ball.GetCollisionModel().AddBox(Rb, hhalf, Rb)  # must set half sizes
    ball.GetCollisionModel().BuildModel()
    ball.SetCollide(True)
    ball.SetBodyFixed(False)
    col_b = chrono.ChColorAsset()
    col_b.SetColor(chrono.ChColor(0, 0, 1))
    ball.AddAsset(col_b)
    pt = chrono.ChLinkMatePlane()
    pt.Initialize(body_floor, ball, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0),
                  chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, -1, 0))
    prismatic_ground_ball = chrono.ChLinkLockPrismatic()
    prismatic_ground_ball.SetName("prismatic_ground_ball")
    prismatic_ground_ball.Initialize(body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(5.5, 0, 0), z2x))
    my_system.AddLink(prismatic_ground_ball)
    my_system.AddLink(pt)
    my_system.Add(ball)
    obj.append(ball)
    return (forceb, obj, ball)


# In[Wall] # creates a wall
def Wall(x, y, z, rotate, length, height, width, material, my_system):
    body_brick = chrono.ChBody()
    body_brick.SetBodyFixed(True)
    # set initial position
    body_brick.SetPos(chrono.ChVectorD(x, y, z))
    body_brick.SetRot(chrono.Q_from_AngY(rotate))
    # set mass properties
    body_brick.SetMass(.5)
    body_brick.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
    # set collision surface properties
    body_brick.SetMaterialSurface(material)
    # Collision shape
    body_brick.GetCollisionModel().ClearModel()
    body_brick.GetCollisionModel().AddBox(length / 2, height / 2, width / 2)  # must set half sizes
    body_brick.GetCollisionModel().BuildModel()
    body_brick.SetCollide(True)
    # Visualization shape, for rendering animation
    body_brick_shape = chrono.ChBoxShape()
    body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(length / 2, height / 2, width / 2)
    col_brick = chrono.ChColorAsset()
    col_brick.SetColor(chrono.ChColor(1, 0.1, 0.5))  # Pink
    body_brick.AddAsset(col_brick)
    body_brick.GetAssets().push_back(body_brick_shape)
    body_brick.GetAssets().push_back(col_brick)
    my_system.Add(body_brick)








# In[Set springs]
def setSpring(k, rl, rlmax, Springs, Fm, i):
    var1 = Springs[i].Get_SpringLength()
    if var1 < rl:
        Springs[i].Set_SpringF(0)
        var2 = Springs[i].Get_SpringF()
    if var1 > rlmax:
        Springs[i].Set_SpringF(3*k)
        var2 = Springs[i].Get_SpringF()
    else:
        Springs[i].Set_SpringF(k)
        var2 = Springs[i].Get_SpringF()

    Fm.append(var2)
    return (Springs, Fm)


# In[Check Spring Length]
def fix_spring_lengths(obj,k,rl,rlmax,Springs,t,Fm,nb):
    
    for ii in range(nb):
    # Set spring lengths and initiate jamming if needed
        (Springs, Fm) = setSpring(k, rl, rlmax, Springs, Fm, ii)
    
    return Springs,Fm,obj

# In[Extract data 2]
def ExtractData2(obj, nb, nt, Xpos, Ypos, Zpos, Xforce, Yforce, Zforce, rott0, rott1, rott2, rott3, Xvel, Yvel, Zvel,
                 Springs, Fm, templ):
    for i in range(nb):
        var1 = (Springs[i].Get_SpringLength() - Springs[i].Get_SpringRestLength())
        templ.append(var1)  # append spring length
    for i in range(nt):
        tempx = obj[i].Get_Xforce()  # temporary variables for force
        tempxx = obj[i].GetPos_dt()  # temporary variables for velocity
        # Total forces (x,y,z)
        Xforce.append((tempx.x))
        Yforce.append((tempx.y))
        Zforce.append((tempx.z))
        # positions(x,y,z)
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

    return (
    obj, Springs, Fm, templ, Xforce, Yforce, Zforce, Xpos, Ypos, Zpos, rott0, rott1, rott2, rott3, Xvel, Yvel, Zvel)


# In[Heading Angle]
def Heading_Angle(force, obj, goal, my_system, nb,botcall,nbactive,bots):
    alpha = []
    for i in range(nbactive):

            
        rx = (bots[i].GetPos().x - goal[0])  # x position of r vector
        rz = (bots[i].GetPos().z - goal[2])  # z positon of r ector
        r = np.array([rx, rz])  # r vector
        rbar = np.linalg.norm(r)  # magnitude

        hx = force[i].GetDir().x  # heading vector x position
        hz = force[i].GetDir().z  # heading vector z position
        h = np.array([hx, hz])  # h vector
        hbar = np.linalg.norm(h)  # h magnitude

        # heading angle 
        theta = np.arccos(np.dot(h, r) / (hbar * rbar))
        alpha.append(theta)
    return (alpha)


# In[Check if near target]
def Check_distance(obj, goal, my_system, nb, active, error,bots,nbactive):
    for i in range(nbactive):
        if active[i] == 0:
            active[i] == 0
        else:
            rx = (bots[i].GetPos().x - goal[0])  # x position of r vector
            rz = (bots[i].GetPos().z - goal[2])  # z positon of r ector
            r = np.array([rx, rz])  # r vector
            rbar = np.linalg.norm(r)  # magnitude
            if abs(rbar) <= error:  # if distance is less than acceptable distance make zero
                active[i] = 0
            else:  # else still active
                active[i] = 1
    return (active)


# In[C active]    
def Cactive(res, active, alpha, ta, nb,nbactive):
    C = np.zeros(nbactive)
    for i in range(nbactive):
        if active[i] == 1:  # if active
            if (np.pi / 2 + ta) < alpha[i] < (3 * np.pi / 2 - ta):
                C[i] = 1
            elif 0 <= alpha[i] < (np.pi / 2 - ta) or (3 * np.pi / 2 + ta) < alpha[
                i] <= 2 * np.pi:  # if its within 90 degress or 270 degrees
                C[i] = -1
            else:
                C[i] = 0
        else:
            C[i] = -res.x[i] * abs(alpha[i] > np.pi / 2) + res.x[i] * (abs(alpha[i] <= np.pi/2))
    return C


# In[Controller 4]
def Controller4(my_system, force, obj, mag, goal, nb, active, ta, C, error,nbactive,bots,botcall):
    # calculate the heading angle
    alpha = Heading_Angle(force, obj, goal, my_system, nb,botcall,nbactive,bots)

    # check distance
    active = Check_distance(obj, goal, my_system, nb, active, error,bots,nbactive)

    # run the optimization
    res = linprog(np.sin(alpha) ** 2 - np.cos(alpha) ** 2, method='simplex', bounds=[(0, 1)] * nbactive)
    # determine if it needs to go forward or backwards
    C = Cactive(res, active, alpha, ta, nb,nbactive)

    for i in range(len(force)):
        p = mag * C[i]
        force[i].SetMforce(p)
        force[i].SetRelDir(chrono.ChVectorD(1, 0, 0))

    return (res, alpha, active, C)

# In[Controller 5]
def Controller5(my_system, force, obj, mag, goal, nb, active, ta, C, error):
    # calculate the heading angle
    alpha = Heading_Angle(force, obj, goal, my_system, nb)

    # check distance
    active = Check_distance(obj, goal, my_system, nb, active, error)

    # run the optimization
    bounds = Bounds(-np.ones(nb), np.ones(nb))
    A = np.array([np.cos(alpha), np.sin(alpha)])
    b = np.average(A, 1)
    theta_d = np.array([0, 0, 0, 0, 0, pi, pi, pi, pi, pi])
    linear_constraint = LinearConstraint(A, b, b)
    l = np.zeros(nb)
    for i in range(nb-1):
        rx = (obj[i].GetPos().x - obj[i+1].GetPos().x)  # x position of r vector
        rz = (obj[i].GetPos().z - obj[i].GetPos().z)  # z positon of r ector
        l[i] = 1/np.linalg.norm([rx, rz])
    l[-1] = 1/np.linalg.norm([(obj[0].GetPos().x - obj[-1].GetPos().x), (obj[0].GetPos().z - obj[-1].GetPos().z)])
    def cost(x,theta_d,alpha):

        x = np.array(x)
        return (np.linalg.norm(np.multiply(x[1:-1] - ((x[2:] + x[:-2]) / 2), l[1:-1]) - (theta_d[1:-1] - alpha[1:-1])) + abs(
            (x[-1] - (x[-2] + x[0]) / 2) * l[-1] - (theta_d[-1] - alpha[-1])) + abs(
            (x[0] - (x[1] + x[-1]) / 2) * l[0] - (theta_d[0] - alpha[0])))

    res = minimize(cost, 0.5*np.ones(nb), method='trust-constr', constraints=[linear_constraint],args=(theta_d,alpha), options={'verbose': 0},
                   bounds=bounds)
    xm = np.average(abs(res.x))
    C = np.array(1 * np.multiply((abs(res.x) > xm), res.x > 0) - 1 * np.multiply((abs(res.x) > xm), res.x < 0)).astype(float)
    # determine if it needs to go forward or backwards
    # C = Cactive(res, active, alpha, ta, nb)
    # print("C:", C)
    print(C)
    for j in range(len(force)):
        p = mag*C[j]
        force[j].SetMforce(p)
        force[j].SetRelDir(chrono.ChVectorD(1, 0, 0))

    return (res, alpha, active, C)
# In[Closest robot to target]
def closet_robot_to_target(bots,goal_c,nbactive):
    dtemp=np.zeros(nbactive)
    for i in range(nbactive):
        dtemp[i]=np.sqrt((bots[i].GetPos().x-goal_c[0])**2+(bots[i].GetPos().z-goal_c[2])**2)
    maxvalue=max(dtemp)                
    leg_closest = dtemp.argmax()
    return leg_closest
# In[Find distance]
def find_distance(nbactive,bots):
    xm = 0.0
    zm = 0.0
    d = np.zeros(nbactive)
    for i in range(nbactive):
        xm += (bots[i].GetPos().x)  # x position of r vector
        zm += (bots[i].GetPos().z)  # z positon of r ector
    # finding distance
    for i in range(0, nbactive):
        dx =  (bots[i].GetPos().x)- xm / nbactive
        dz = (bots[i].GetPos().z)- zm / nbactive
        d[i] = np.sqrt(dx**2 + dz**2)  
    
    return d
# In[Controller 6]
def Controller6(nbactive,bots,goal_d,goal_c,res,force,mag,R1):
    # find distance 
    d=find_distance(nbactive,bots)
    # if the target has moved 
    if np.sqrt((goal_d[0]-goal_c[0])**2 + (goal_d[2]-goal_c[2])**2) > R1:
        # if not in a circle
        if (d < 2*R1).any():
            res = 1 * np.ones(nbactive)
        # if in circle
        else:
            goal_d[0] = goal_c[0]
            goal_d[2] = goal_c[2]
    # if goal hasnt moved and in circle position 
    else:
        # find leg closest
        leg_closest = closet_robot_to_target(bots,goal_c,nbactive)
        i1 = leg_closest - 2
        i2 = leg_closest + 2
        a = np.multiply(1*np.multiply((np.arange(i1, i2) >= 0), (np.arange(i1, i2) <= nbactive-1)),np.arange(i1, i2)) + (nbactive*(np.arange(i1, i2) < 0) + np.multiply((np.arange(i1, i2) < 0), np.arange(i1, i2))) + (np.multiply(1*(np.arange(i1, i2) > (nbactive-1)), np.arange(i1, i2)) - nbactive*(np.arange(i1, i2) > (nbactive-1)))
        # insert value to a 
        for j in (a):
            res[j] = -1
            
        leg_closest_opposite = (leg_closest + (nbactive/2)) * (leg_closest < (nbactive/2)) + (leg_closest - (nbactive/2)) * (leg_closest > ((nbactive/2)-1))
        i1 = leg_closest_opposite - 2
        i2 = leg_closest_opposite + 2
        b = np.multiply(1*np.multiply((np.arange(i1, i2) >= 0), (np.arange(i1, i2) <= (nbactive-1))), np.arange(i1, i2)) + (nbactive*(np.arange(i1, i2) < 0) + np.multiply((np.arange(i1, i2) < 0), np.arange(i1, i2))) + (np.multiply(1*(np.arange(i1, i2) > (nbactive-1)), np.arange(i1, i2)) - nbactive*(np.arange(i1, i2) > (nbactive-1)))

        for j in (b.astype(int)):
            res[j] = -1
 
    for i in range(len(force)):
        p = mag * res[i]
        force[i].SetMforce(p)
        force[i].SetRelDir(chrono.ChVectorD(1, 0, 0))
    return res,force
# In[Export data 2]
def Exportdata2(Xpos, Ypos, Zpos, rott0, rott1, rott2, rott3, templ, Xforce, Yforce, Zforce, Xcontact, Ycontact,
                Zcontact, Xvel, Yvel, Zvel, Fm, nb, nt, count, lengthm, nc, cx, cy, cz, Fxct, Fyct, Fzct):
    # [Convert list to matrices]
    Xpos = np.asarray(Xpos)
    Ypos = np.asarray(Ypos)
    Zpos = np.asarray(Zpos)
    # rotation positions
    rott0 = np.asarray(rott0)
    rott1 = np.asarray(rott1)
    rott2 = np.asarray(rott2)
    rott3 = np.asarray(rott3)
    # spring length
    templ = np.asarray(templ)
    # Force total
    Xforce = np.asarray(Xforce)
    Yforce = np.asarray(Yforce)
    Zforce = np.asarray(Zforce)
    # contact forces
    Xcontact = np.asarray(Xcontact)
    Ycontact = np.asarray(Ycontact)
    Zcontact = np.asarray(Zcontact)
    # velocities
    Xvel = np.asarray(Xvel)
    Yvel = np.asarray(Yvel)
    Zvel = np.asarray(Zvel)
    # membrane force
    Fm = np.asarray(Fm)

    # In[Create empty arrays]
    # position
    qx = np.zeros((nt, count))
    qy = np.zeros((nt, count))
    qz = np.zeros((nt, count))

    # empty toational matrices
    rot0 = np.zeros((nt, count))
    rot1 = np.zeros((nt, count))
    rot2 = np.zeros((nt, count))
    rot3 = np.zeros((nt, count))

    # total forces
    Fxt = np.zeros((nt, count))
    Fyt = np.zeros((nt, count))
    Fzt = np.zeros((nt, count))
    # Spring length
    SL = np.zeros((nb, count))

    # Velocity empty matrices
    Xv = np.zeros((nt, count))
    Yv = np.zeros((nt, count))
    Zv = np.zeros((nt, count))

    # Membrane force
    Fmem = np.zeros((nb, count))

    # Create empty contact matrices
    xc = np.zeros((lengthm, count))
    yc = np.zeros((lengthm, count))
    zc = np.zeros((lengthm, count))
    # Contact forces
    Fcx = np.zeros((lengthm, count))
    Fcy = np.zeros((lengthm, count))
    Fcz = np.zeros((lengthm, count))

    # In[Fill the matrices]
    for i in range(count):
        # fill the position matrices
        qx[:, i] = Xpos[nt * i:nt * i + nt]  # x position
        qy[:, i] = Ypos[nt * i:nt * i + nt]  # y position
        qz[:, i] = Zpos[nt * i:nt * i + nt]  # z position

        # fill the rotational matrices  
        rot0[:, i] = rott0[nt * i:nt * i + nt]  # quterion position 0
        rot1[:, i] = rott1[nt * i:nt * i + nt]  # quterion position 1
        rot2[:, i] = rott2[nt * i:nt * i + nt]  # quterion position 2
        rot3[:, i] = rott3[nt * i:nt * i + nt]  # quterion position 3

        # fill the total force matrices
        Fxt[:, i] = Xforce[nt * i:nt * i + nt]  # total force x
        Fyt[:, i] = Yforce[nt * i:nt * i + nt]  # total force y
        Fzt[:, i] = Zforce[nt * i:nt * i + nt]  # total force z

        # Fill for velocity
        Xv[:, i] = Xvel[nt * i:nt * i + nt]  # velocity x position
        Yv[:, i] = Yvel[nt * i:nt * i + nt]  # velocity y position
        Zv[:, i] = Zvel[nt * i:nt * i + nt]  # velocity z position
        # fill for spring length
        SL[:, i] = templ[nb * i:nb * i + nb]  # membrane length
        # fill for membrane force
        Fmem[:, i] = Fm[nb * i:nb * i + nb]  # membrane force

    # print(nc[0])
    for i in range(count):
        # temporary variables for contact and force chains
        ind = nc[i]
        tryme = cx[i]
        tryme2 = cy[i]
        tryme3 = cz[i]
        tryme4 = Fxct[i]
        tryme5 = Fyct[i]
        tryme6 = Fzct[i]

        # convert to array
        tryme = np.asarray(tryme)
        tryme2 = np.asarray(tryme2)
        tryme3 = np.asarray(tryme3)
        tryme4 = np.asarray(tryme4)
        tryme5 = np.asarray(tryme5)
        tryme6 = np.asarray(tryme6)

        # fill array position
        xc[0:ind, i] = np.transpose(tryme)  # contact position x
        yc[0:ind, i] = np.transpose(tryme2)  # contact position y
        zc[0:ind, i] = np.transpose(tryme3)  # contact position z

        # Fill array forces
        Fcx[0:ind, i] = np.transpose(tryme4)  # Contact force x
        Fcy[0:ind, i] = np.transpose(tryme5)  # contact force y
        Fcz[0:ind, i] = np.transpose(tryme6)  # contact force z

    return (qx, qy, qz, rot0, rot1, rot2, rot3, Fxt, Fyt, Fzt, Xv, Yv, Zv, SL, Fmem, Fcx, Fcy, Fcz, xc, yc, zc)


# In[Number of contacts]

class MyReportContactCallback(chrono.ReportContactCallback):

    def __init__(self):
        chrono.ReportContactCallback.__init__(self)
        self.Fcx = []
        self.Fcy = []
        self.Fcz = []
        self.pointx = []
        self.pointy = []
        self.pointz = []
        # self.bodies = []

    def OnReportContact(self, vA, vB, cA, dist, rad, force, torque, modA, modB):
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
        # self.bodies.append([nameA,nameB])
        return True  # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.Fcx = []
        self.Fcy = []
        self.Fcz = []

    # Get the points
    def GetList(self):
        return (self.pointx, self.pointy, self.pointz, self.Fcx, self.Fcy, self.Fcz)


