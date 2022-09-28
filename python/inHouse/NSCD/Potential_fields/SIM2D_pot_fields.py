# -*- coding: utf-8 -*-
"""
Created on Sun Mar  1 19:42:33 2020

@author: dmulr
"""

# In[Import libraries]
import numpy as np
from bridson import poisson_disc_samples
import math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib import animation
import sys
from objects_pot_fields import * 
from config3 import *
from numpy import array, dot
#from qpsolvers import solve_qp
from scipy.optimize import minimize 
from IPython.display import Image


index=0
# In[Create Robot]
for i in range(nb):
    x=R1*np.cos(i*2*np.pi/nb)
    y=R1*np.sin(i*2*np.pi/nb)
    Q[2*i,0]=x
    Q[2*i+1,0]=y
    obj.append(Robot(x,y,phi0b,xv0b,yv0b,phiv0b,diameter/2,mb,Ib,Fx,Fy,T,i,index))
    index=index+1

#(x,y,z)=points_for_rbf_square(nb,R,p1,p2,nr)
(x,y,z)=points_for_grab(p1,p2)
rbf=create_RBF(y,x,z)
(fny,fnx,zy,zx,xx,yy)=gradient_RBF(br,bl,rbf)
ds=perimeter(x,y)
# In[Determine the number of interiors]
#n=MaxValues(R1,diameter,nb)
#n=n[0]# intern step 
#n=np.array([37,27,20])
#ni=np.sum(n)# number of interiors
ni=0
nt=ni+nb # total number

count=0


# In[Cretae Interior Particles]
#for i in range(n.size):
#    for j in range(n[i]):
#        R2=(diameter*n[i])/(np.pi*2)     # Radius of interiro ring
#        x=R2*np.cos(j*2*np.pi/n[i])     # x position
#        y=R2*np.sin(j*2*np.pi/n[i])     # z position
#        obj.append(Particle(x,y,phi0p,xv0p,yv0p,phiv0p,diameter/2,mp,Ip,count,index))  
#        count=count+1 
#        index=index+1
# In[Boundary walls]        
obj.append(verticalLine(left))
obj.append(verticalLine(right))
obj.append(horizontalLine(up))
obj.append(horizontalLine(down))




# In[Create Global Matrix]
M,q0,v0=createGlobalMatrices(nt,obj) #Creating Mass, q, and v matrices
ndof=np.size(q0) # degrees of freedom
#
# In[Gravity]
if g==1: # if gravity is on add it to forces
    Fg=gravity(obj,q0)
else: # else gravity is zero 
    Fg=np.zeros_like(q0)
# In[Empty Position and velocity matrix]
q=np.zeros((q0.size,time.size)) # empty position matrix
v=np.zeros((v0.size,time.size)) # empty velocity matrix
F=np.zeros((q0.size,time.size)) # empty force matrix
#E=np.zeros((2*nb,time.size)) # empty error matrix
Fbound=np.zeros((2*nb,time.size)) # empty BOundary force matrix
Fcontact=np.zeros((q0.size,time.size)) # empty contact force matrix
# In[Initial conditions]
q[:,0],v[:,0]=q0.reshape(q0.size,),v0.reshape(v0.size,)
# Q desired position
Qd=Q[:,0]
# In[Solving the system]
for i in range(time.size-1):

    print(i*tstep) # [print time]
    # qbar
    qbar=q[:,i]+tstep*gamma*v[:,i]
    
    if time[i]<4.0:
        l=l
    else:
        l=.75*l
    # stiffness matrix
    K,d=modelStiffness(qbar,nb,ni,l,k) 
    
    #External forces acting on the robot
    FE=-np.matmul(K,qbar)+Fg.flatten()
    #FE=Fg.flatten()
# In[Controller]
    (F,Fb)=Controller2_5(nb,qbar,i,F,fny,fnx,alpha,beta,rbf,v[:,i])
    #(F,Feb)= Controller2(nb,q,i,F,fny,fnx,alpha)
    #(F,Fb,Feb)=Controller1(M,FE,v[:,i],q[:,i],tstep,t_imp,nb,Qd,nt,F,i,mb)
    # save applied forces from robots
    Fbound[:,i]=Fb
    
    Fext=FE+F[:,i].flatten()
    
    # update coordinates
    updateCoordinates(obj,qbar,v[:,i])
    #calculated gaps and determine which ones ar gunna collide 
    gbar,ind=calculateGaps(obj) 
    # update the positions 
    updateCoordinates(obj,q[:,i],v[:,i]) 

    #Calculating vfree
    vfree=v[:,i]+tstep*np.matmul(np.linalg.inv(M),Fext)
    

# In[if theres no collisions]
    if ind.size<1:
        v[:,i+1]=vfree
# In[If there are contacts]
    else:
       (Rk1,H)=LCP_Com_Ex(obj,ind,ndof,vfree,M,mu,en,et,v[:,i],q[:,i])
        # Vk+1
       Fc=np.matmul(H,Rk1).reshape(ndof,)
       Fcontact[:,i]=Fc
       v[:,i+1]=vfree+(np.matmul(np.matmul(np.linalg.inv(M),H),Rk1)).reshape(ndof,)


    # For implicit integration of q
    vkt=tstep*(t_imp*v[:,i+1]+(1-t_imp)*v[:,i])

    #Implicit integration of q
    q[:,i+1]=q[:,i]+vkt
    E=Error_RBF(rbf,q,nb,E,i)
    # Error 
    #(EL2,E,ELAVG)=Error(q,nb,E,i,Q,EL2,ELAVG)
    #(COMA)=COM_actual(q,nb,COMA,i)
    # Desired Position
    #Qd=Next_position(EL2[:,i],nb,ni,i,tstep,Q,error)


## In[save data]
#np.savez(sim+'.npz',
#         q=q,
#         nb=nb,
#         ni=ni,
#         obj=obj,
#         left=left,
#         right=right,
#         up=up,down=down,
#         R1=R1,
#         t0=t0,
#         tend=tend,
#         time=time,
#         F=F,
#         E=E,
#         EL2=EL2,
#         Fbound=Fbound,
#         Fcontact=Fcontact,
#         COMA=COMA,
#         COM=COM,
#         ELAVG=ELAVG
#         )

# In[save data]
np.savez(sim+'.npz',
         q=q,
         nb=nb,
         ni=ni,
         obj=obj,
         left=left,
         right=right,
         up=up,down=down,
         R1=R1,
         t0=t0,
         tend=tend,
         time=time,
         F=F,
         E=E,
         R=R
         )