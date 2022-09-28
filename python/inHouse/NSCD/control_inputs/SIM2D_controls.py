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
from objects_control_sim import * 
from config3 import *
from numpy import array, dot
from qpsolvers import solve_qp
from scipy.optimize import minimize 
from IPython.display import Image


index=0
internal=1
# In[Create Robot]
for i in range(nb):
    x=R1*np.cos(i*2*np.pi/nb)
    y=R1*np.sin(i*2*np.pi/nb)
    Q[2*i,0]=x
    Q[2*i+1,0]=y
    obj.append(Robot(x,y,phi0b,xv0b,yv0b,phiv0b,diameter/2,mb,Ib,Fx,Fy,T,i,index))
    index=index+1

# In[Desired Shape]
shapes=Points_for_shape(shape,p1,p2,nb,nr,R,bl,br)
(rbf,fnx,fny)=shapes.Create_shape_gradient()
#n=MaxValues(R1,diameter,nb)
#n=n[0]# intern step 
#ni=np.sum(n)
#nt=ni+nb # total number

#n=np.array([10,7,3])
#ni=np.sum(n)# number of interiors
ni=0
nt=ni+nb # total number
count=0
#for i in range(n.size):
#    for j in range(n[i]):
#        R2=(diameter*n[i])/(np.pi*2)+.03     # Radius of interiro ring
#        x=R2*np.cos(j*2*np.pi/n[i])     # x position
#        y=R2*np.sin(j*2*np.pi/n[i])     # z position
#        obj.append(Particle(x,y,phi0p,xv0p,yv0p,phiv0p,diameter/2,mp,Ip,count,index))  
#        count=count+1 
#        index=index+1




# In[Boundary walls]        
#obj.append(verticalLine(left))
#obj.append(verticalLine(right))
#obj.append(horizontalLine(up))
#obj.append(horizontalLine(down))




# In[Create Global Matrix]
M,q0,v0=createGlobalMatrices(nt,obj) #Creating Mass, q, and v matrices
ndof=np.size(q0) # degrees of freedom
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
Qd=np.zeros((2*nb,time.size))
qall=q[:,i]
FE=np.zeros((2*nb,time.size))
#method='optim_ptp'
method='potential'

res=controls(nb,q[:,0],v[:,i],qall,Qd,Q,F,FE,fny,fnx,alpha,beta,rbf,method,path,Et,E,tstep,time,error,tbar,h,mb,0)
res.Path_desired()
pointx=[]
pointy=[]
count=0
# In[Solving the system]
for i in range(time.size-1):
    print(i*tstep) # [print time]
#    if i%40==0:
#        count=count+1
#        p2=.001*(count)**2
#        p1=p1+.01
        #p2=p2+.01
  #      shapes=Points_for_shape(shape,p1,p2,nb,nr,R,bl+p1,br+p1)
    #    (rbf,fnx,fny)=shapes.Create_shape_gradient()
    res.rbf=rbf
    res.fny=fny
    res.fnx=fnx
    #pointx.append(p1)
    #pointy.append(p2)    

    # qbar
    qbar=q[:,i]+tstep*gamma*v[:,i]
    

    # stiffness matrix
    K,d=modelStiffness(qbar,nb,ni,l,k) 
    
    #External forces acting on the robot
    FE=-np.matmul(K,qbar)+Fg.flatten()
    #FE=Fg.flatten()
# In[Controller]
    #method='potential'
    res.j=i
    res.vbar=v[:,i]
    res.qbar=qbar
    res.FE=FE
    (Fb,F)=res.Controller()
    res.timep.append(i*tstep)
    # save applied forces from robots
    Fbound[:,i]=Fb
    
    Fext=FE+F[:,i].flatten()
    (q,v)=simulate(obj,qbar,v,M,Fext,i,q,tbar,tstep,mu,en,et,ndof)
    res.qall=q[:,i+1]
    # i+1 land
    res.update_qout()
    res.Error()
    #res.Next_position()



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
         R=R,
         pointx=pointx,
         pointy=pointy
         )