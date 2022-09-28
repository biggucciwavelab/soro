# In[Header]

"""
author: declan mulroy
project: JAMoEBA
email: dmulroy@hawk.iit.edu
date: 10/9/19
"""

# In[import libraries]
import numpy as np
import math as math
import matplotlib.pyplot as plt
import os

#In[Import data]
data=np.load('large_scale.npz',allow_pickle=True)

# Positions
qx=data['qx']
qy=data['qy']
qz=data['qz']

# rotations
rot0=data['rot0']
rot1=data['rot1']
rot2=data['rot2']
rot3=data['rot3']

# Spring values
SL=data['SL']

# Velocity
Xv=data['Xv']
Yv=data['Yv']
Zv=data['Zv']

# Contact forces
Fxc=data['Fxc']
Fyc=data['Fyc']
Fzc=data['Fzc']

# Total foces
Fxt=data['Fxt']
Fyt=data['Fyt']
Fzt=data['Fzt']

# Active robots
botcall=data['botcall']

# number of robots
nb=data['nb']

# time
time=data['ttemp']

sim=data['sim']

count=data['count']

# In[position]

script_dir = os.path.dirname("result_file_1_active"+str(sim)+"/")

script_dir = os.path.dirname('plots_large/')

results_dir = os.path.join(script_dir, 'XYZPosition/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)
    
    for i in range (nb):
        var=botcall[0,i]
    if var==1:
        
        txt=" (Active)"
    else:
        txt=" (passive)"
    fig=plt.figure(i)
    plt.figure(figsize=(15,10))
    
    fig.suptitle("Position  vs Time (s) for Bot " + str(i) + txt)
    ax1 = plt.subplot(3,1,1)
    ax1.grid(True)
    plt.gca().set_title('x position (mm) vs time')
    plt.plot(time, qx[i,:]*1000,'b')
    
    ax2 = plt.subplot(3,1,2)
    plt.gca().set_title('y position(mm) vs time')
    plt.plot(time,qy[i,:]*1000,'r')
    ax2.grid(True)
    ax3 = plt.subplot(3,1,3)
    plt.gca().set_title('z position (mm) vs time')
    plt.plot(time, qz[i,:]*1000,'g')
    ax3.grid(True)
    plt.subplots_adjust(hspace = 1)
    plt.xlabel('time (seconds)')
    plt.savefig(results_dir+" bot_pos " + str(i) + txt +".svg") 
    plt.close('all')

# In[Spring length plots]
    # Create directory to store data
script_dir = os.path.dirname('plots_large/')

results_dir = os.path.join(script_dir, 'spring lengths/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

for i in range(nb):

    plt.figure(i)
    plt.title('Spring length (mm) ' + str(i) + ' vs time (seconds)')
    plt.plot(time,SL[i,:]*1000,'b')
    plt.xlabel('time')
    plt.ylabel('Spring length(mm)')
    plt.grid(True)
    plt.savefig(results_dir+" Spring length " + str(i) + ".svg") 
    plt.close('all')
    
# In[Create plots for rotation]

# Create directory to store data

results_dir = os.path.join(script_dir, 'rotations/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

# Create directory to store data
script_dir = os.path.dirname('plots_large/')
results_dir = os.path.join(script_dir, 'rotations/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

yaw=np.zeros((nb,np.size(time)))
pitch=np.zeros((nb,np.size(time)))
roll=np.zeros((nb,np.size(time)))
for i in range(nb):

    for j in range(np.size(time)):
        Erot=quaternion_to_euler(rot0[i,j], rot1[i,j], rot2[i,j], rot3[i,j])   
        Erot=np.asarray(Erot)
        yaw[i,j]=Erot[0]
        pitch[i,j]=Erot[1]
        roll[i,j]=Erot[2]
        
for i in range (nb):

    var=botcall[0,i]
    if var==1:
        
        txt=" (Active)"
    else:
        txt=" (passive)"
    fig=plt.figure(i)
    plt.figure(figsize=(13,13))
    
    fig.suptitle("rotation vs Time (s) for Bot " + str(i) + txt)
    ax1 = plt.subplot(3,1,1)
    ax1.grid(True)
    plt.gca().set_title('yaw vs time')
    plt.plot(time, yaw[i,:],'b')
    
    ax2 = plt.subplot(3,1,2)
    plt.gca().set_title('pitch vs time')
    plt.plot(time,pitch[i,:],'r')
    ax2.grid(True)
    ax3 = plt.subplot(3,1,3)
    plt.gca().set_title('roll vs time')
    plt.plot(time, roll[i,:],'g')
    ax3.grid(True)
    plt.subplots_adjust(hspace = 1)
    plt.xlabel('time (seconds)')
    plt.savefig(results_dir+" bot_pos " + str(i) + txt +".svg") 
    plt.close('all')        
        
# In[Velocity of the robots]

script_dir = os.path.dirname('plots_large/')
results_dir = os.path.join(script_dir, 'XYZVelocity/')
results_dir = os.path.join(script_dir, 'XYZVelocity/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)
        
for i in range (nb):

    var=botcall[0,i]
    if var==1:
        
        txt=" (Active)"
    else:
        txt=" (passive)"
    fig=plt.figure(i)
    plt.figure(figsize=(13,13))
    
    fig.suptitle("Velocity  vs Time (s) for Bot " + str(i) + txt)
    ax1 = plt.subplot(3,1,1)
    ax1.grid(True)
    plt.gca().set_title('x Velocity (m/s) vs time')
    plt.plot(time, Xv[i,:],'b')
    
    ax2 = plt.subplot(3,1,2)
    plt.gca().set_title('y Velocity (m/s) vs time')
    plt.plot(time,Yv[i,:],'r')
    ax2.grid(True)
    ax3 = plt.subplot(3,1,3)
    plt.gca().set_title('z Velocity (m/s) vs time')
    plt.plot(time, Zv[i,:],'g')
    ax3.grid(True)
    plt.subplots_adjust(hspace = 1)
    plt.xlabel('time (seconds)')
    plt.savefig(results_dir+" bot_Velocity " + str(i) + txt +".svg") 
    plt.close('all')      

# In[Plots for external forces]

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)
        
for i in range (nb):

    var=botcall[0,i]
    if var==1:
        
        txt=" (Active)"
    else:
        txt=" (passive)"
    fig=plt.figure(i)
    plt.figure(figsize=(13,13))
    
    fig.suptitle("Velocity  vs Time (s) for Bot " + str(i) + txt)
    ax1 = plt.subplot(3,1,1)
    ax1.grid(True)
    plt.gca().set_title('x Velocity (mm/s) vs time')
    plt.plot(time, Xv[i,:]*1000,'b')
    
    ax2 = plt.subplot(3,1,2)
    plt.gca().set_title('y Velocity (mm/s) vs time')
    plt.plot(time,Yv[i,:]*1000,'r')
    ax2.grid(True)
    ax3 = plt.subplot(3,1,3)
    plt.gca().set_title('z Velocity (mm/s) vs time')
    plt.plot(time, Zv[i,:]*1000,'g')
    ax3.grid(True)
    plt.subplots_adjust(hspace = 1)
    plt.xlabel('time (seconds)')
    plt.savefig(results_dir+" bot_Velocity " + str(i) + txt +".svg") 
    plt.close('all')       

# In[Plots for external forces]
script_dir = os.path.dirname('plots_large/')

results_dir = os.path.join(script_dir, 'External Forces/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

for i in range (nb):
   
    var=botcall[0,i]
    if var==1:
        
        txt=" (Active)"
    else:
        txt=" (passive)"
        
    fig=plt.figure(i)
    plt.figure(figsize=(13,13))
    fig.suptitle("Force applied vs Time for Bot" + str(i) + txt)
    ax1 = plt.subplot(3,1,1)
    ax1.grid(True)
    plt.gca().set_title('x force (N) vs Time')
    plt.plot(time, Fxt[i,:],'b')
    ax2 = plt.subplot(3,1,2)
    plt.gca().set_title('y force (N) vs Time')
    plt.plot(time, Fyt[i,:],'r')
    ax2.grid(True)
    ax3 = plt.subplot(3,1,3)
    plt.gca().set_title('z force (N) vs Time')
    plt.plot(time, Fzt[i,:],'g')
    ax3.grid(True)
    plt.subplots_adjust(hspace = 1)
    plt.xlabel('time (seconds)')
    plt.savefig(results_dir+"Force applied for" + str(i) + txt + ".svg")
    plt.close('all')

# In[Contact forces]

results_dir = os.path.join(script_dir, 'Contact Forces/')

script_dir = os.path.dirname('plots_large/')
results_dir = os.path.join(script_dir, 'Contact Forces/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

for i in range (nb):
   
    var=botcall[0,i]
    if var==1:
        
        txt=" (Active)"
    else:
        txt=" (passive)"
        
    fig=plt.figure(i)
    plt.figure(figsize=(13,13))
    fig.suptitle("Contact forces vs Time for Bot" + str(i) + txt)
    ax1 = plt.subplot(3,1,1)
    ax1.grid(True)
    plt.gca().set_title('x force (N) vs Time')
    plt.plot(time, Fxc[i,:],'b')
    ax2 = plt.subplot(3,1,2)
    plt.gca().set_title('y force (N) vs Time')
    plt.plot(time, Fyc[i,:],'r')
    ax2.grid(True)
    ax3 = plt.subplot(3,1,3)
    plt.gca().set_title('z force (N) vs Time')
    plt.plot(time, Fzc[i,:],'g')
    ax3.grid(True)
    plt.subplots_adjust(hspace = 1)
    plt.xlabel('time (seconds)')
    plt.savefig(results_dir+"Contact Forces for" + str(i) + txt + ".svg")
    plt.close('all')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

for i in range (nb):
   
    var=botcall[0,i]
    if var==1:
        
        txt=" (Active)"
    else:
        txt=" (passive)"
        
    fig=plt.figure(i)
    plt.figure(figsize=(13,13))
    fig.suptitle("Contact forces vs Time for Bot" + str(i) + txt)
    ax1 = plt.subplot(3,1,1)
    ax1.grid(True)
    plt.gca().set_title('x force (N) vs Time')
    plt.plot(time, Fxc[i,:],'b')
    ax2 = plt.subplot(3,1,2)
    plt.gca().set_title('y force (N) vs Time')
    plt.plot(time, Fyc[i,:],'r')
    ax2.grid(True)
    ax3 = plt.subplot(3,1,3)
    plt.gca().set_title('z force (N) vs Time')
    plt.plot(time, Fzc[i,:],'g')
    ax3.grid(True)
    plt.subplots_adjust(hspace = 1)
    plt.xlabel('time (seconds)')
    plt.savefig(results_dir+"Contact Forces for" + str(i) + txt + ".svg")
    plt.close('all')

# In[Path Traveled]
    
results_dir = os.path.join(script_dir, 'Path traveled/')
cm = plt.cm.get_cmap('copper')
abs_vel = np.power(np.add(np.power(Xv,2),np.power(Yv,2),np.power(Zv,2)),.5)

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)  

plt.figure(num=1,figsize=[6,4],dpi=100)

for i in range(nb):
    
    path=plt.scatter(qx[i,:]*1000,qz[i,:]*1000,c=abs_vel[i],cmap=cm)

plt.title('Positon of the bots (unjammed)')
plt.xlabel('x position (mm)')
plt.ylim(1000*qz.min(),1000*qz.max())
plt.ylabel('z position (mm)')
plt.grid(True)

cbar=plt.colorbar(path)
cbar.set_label('Velocity (m/s)')
plt.savefig(results_dir+" Path of bots" + ".svg") 
plt.close('all')

plt.figure(num=2,figsize=[6,4],dpi=100)

for i in range(nb):
    
    path1=plt.scatter(np.average(qx,axis=0)*1000,np.average(qz,axis=0)*1000,c=np.average(abs_vel,axis=0),cmap=cm)

plt.title('Average Positon of Robot (unjammed)')
plt.xlabel('x position (mm)')
plt.xlim(0,400)
plt.ylim(-150,100)
plt.ylabel('z position (mm)')
plt.grid(True)
cbar=plt.colorbar(path1)
cbar.set_label('Velocity (m/s)')
plt.savefig(results_dir+"Average Path of bots" + ".svg") 
plt.close('all')
                
# In[Average Force]

Fxbar=np.zeros((count,1))
Fybar=np.zeros((count,1))

Fzbar=np.zeros((count,1))

for i in range(count):
    Fxbar[i]=np.sum(Fxt[:,i])
    Fybar[i]=np.sum(Fyt[:,i])
    Fzbar[i]=np.sum(Fzt[:,i])

results_dir = os.path.join(script_dir, 'Sum force/')

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)  


plt.figure(figsize=(13,13))
fig.suptitle("Sum  Force For all bots ")
ax1 = plt.subplot(3,1,1)
ax1.grid(True)
plt.gca().set_title('x force (N) vs Time')
plt.plot(time, Fxbar,'r')

ax1 = plt.subplot(3,1,2)
ax1.grid(True)
plt.gca().set_title('y force (N) vs Time')
plt.plot(time, Fybar,'b')

ax3 = plt.subplot(3,1,3)
plt.gca().set_title('z force (N) vs Time')
plt.plot(time, Fzbar,'g')
ax3.grid(True)
plt.subplots_adjust(hspace = 1)
plt.xlabel('time (seconds)')
plt.savefig(results_dir+"sum force and direction.svg")
plt.close('all')