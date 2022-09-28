# -*- coding: utf-8 -*-
"""
Created on Tue Feb  9 16:40:52 2021

@author: qiyua
"""
import pychrono as chrono
import matplotlib.pyplot as plt
import numpy as np

# Convert quaternion to euler angle about y axis
def quat2eulerCh(quat):
    return quat.Q_to_Rotv().y

def quat2eulerV(x,y,z,w):
    quat = chrono.ChQuaternionD(x,y,z,w)
    
    return 2*quat.Length()
    
# Set up the Chrono simulation system
sys = chrono.ChSystemNSC()
sys.SetSolverType(chrono.ChSolver.Type_APGD)
sys.Set_G_acc(chrono.ChVectorD(0,0, 0))

# Physical properties for rotating cylinder
radius=0.2
height=0.25
rho = 1000
torque = -100.

I = 0.5*radius*radius*(rho*height*np.pi*radius**2)
alpha = torque/I

# Create the cylinder in Chrono and apply torque
body = chrono.ChBodyEasyCylinder(radius,height,rho,True,True)
forcex = chrono.ChForce()
body.AddForce(forcex)
forcex.SetMode(chrono.ChForce.TORQUE)
forcex.SetDir(chrono.ChVectorD(0,1,0))
forcex.SetMforce(torque)
sys.Add(body)

# Time stepping 
tend = 0.5
tstep = 1e-2

# Result arrays
nsteps=int(tend/tstep)
times = np.zeros(nsteps)
ref_dt = np.zeros(nsteps)
cal_dt = np.zeros(nsteps)
ref_pos = np.zeros(nsteps)
cal_pos = np.zeros(nsteps)
cal_pos2 = np.zeros(nsteps)
conv_pos = np.zeros(nsteps)
quat_dt = np.zeros(nsteps)

# Reference quaterion components
q0s = np.zeros(nsteps)
q1s = np.zeros(nsteps)
q2s = np.zeros(nsteps)
q3s = np.zeros(nsteps)

# Time integrated quaternion components
q0p = np.zeros(nsteps)
q1p = np.zeros(nsteps)
q2p = np.zeros(nsteps)
q3p = np.zeros(nsteps)

# Counters
i = 0
ref = 0
pos2 = body.GetRot().Q_to_Rotv().y

# Run the simulation
while (sys.GetChTime() < tend):
    sys.DoStepDynamics(tstep)
    time=sys.GetChTime()
    ref+=alpha*tstep
    
    # Angular velocity and position from Chrono
    dt = body.GetWvel_par().y
    pos = body.GetRot().Q_to_Rotv().y
    pos2 += dt*tstep
    conv_pos[i]=quat2eulerCh(body.GetRot())
    quat_dt[i] =quat2eulerV(body.GetRot_dt().e0,body.GetRot_dt().e1,body.GetRot_dt().e2,body.GetRot_dt().e3)
    
    # Angular velocity and position from analytical solution
    cal_dt[i]=dt
    ref_dt[i]=ref
    
    # Store results
    cal_pos[i]=pos
    cal_pos2[i]=pos2
    ref_pos[i]=0.5*alpha*time**2
    times[i]=sys.GetChTime()
    
    # Quaternions
    q0s[i] = body.GetRot().e0
    q1s[i] = body.GetRot().e1
    q2s[i] = body.GetRot().e2
    q3s[i] = body.GetRot().e3
    
    q0p[i] = q0p[i-1] + body.GetRot_dt().e0
    q1p[i] = q1p[i-1] + body.GetRot_dt().e1
    q2p[i] = q2p[i-1] + body.GetRot_dt().e2
    q3p[i] = q3p[i-1] + body.GetRot_dt().e3
    
    i+=1

qs = [q0s,q1s,q2s,q3s]
qp = [q0p,q1p,q2p,q3p]
# Plot reference and measured angular velocities
if True:
    plt.figure()
    plt.plot(times,ref_dt)
    #plt.scatter(times,cal_dt,s=10,c='tab:orange')
    plt.scatter(times,quat_dt,s=10,c='tab:orange')
    plt.legend(('Reference','Chrono'))
    plt.show()

# Plot reference and measured positions
if False:
    plt.figure()
    plt.plot(times,ref_pos)
    plt.scatter(times,cal_pos,s=10,c='tab:orange')
    plt.scatter(times,cal_pos2, s=10, c='tab:red')
    plt.legend(('Reference','Chrono', 'Chrono Integrated'))
    plt.show()
    
# Plot quaternion components
if False:
    fig, axs = plt.subplots(4,sharex=True,dpi=150)
    fig.suptitle('Quaternion Components')
    for j in range(4):
        axs[j].set_title('q'+str(j))
        axs[j].plot(times,qs[j])
    fig.tight_layout(pad=1)
    fig.text(0.5, 0.04, 'Time[s]', ha='center')
    plt.show()
    
if False:
    fig, axs = plt.subplots(4,sharex=True,dpi=150)
    fig.suptitle('Quaternion Components')
    for j in range(4):
        axs[j].set_title('q'+str(j))
        axs[j].plot(times,qp[j])
    fig.tight_layout(pad=1)
    fig.text(0.5, 0.04, 'Time[s]', ha='center')
    plt.show()
    
if False: 
    plt.figure()
    plt.plot(times,ref_pos)
    plt.scatter(times,conv_pos,s=10,c='tab:orange')
    plt.legend(('Reference','Chrono Converted'))
    plt.show()