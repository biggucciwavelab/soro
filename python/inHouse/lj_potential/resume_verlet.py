#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 19 12:15:04 2018

@author: asriva13
"""

import numpy as np

#imports all the data from revious file
data = np.load('resumeData.npz')
r0=data['r0']
v=data['v']
c=data['c']
nb=data['nb']
ni=data['ni']
n=data['n']
Minv=data['Minv']
lb=data['lb']
#Fg=data['Fg']
rm=data['rm']
epsilon=data['epsilon']
p=data['p']
l=data['l']
F=data['F']
k=data['k']

#g=data['g']
#mass
m=data['m']
#gravity
g=0
#Gravitational Force
Fg=np.zeros((2*n,1))
Fg[1:2*n:2,0]=-m*g


# function that returns the stiffness matrix of the robot
def modelStiffness(r,n,nb,ni,l):
    
    #define theta
    theta=r
    #reshape it
    theta=np.reshape(theta,(-1,1))
    
    #generate empty distance matrix
    d=np.zeros((nb,1))    
    
    #fill distance matrix
    for i in range(nb-1):
        d[i,0]=np.sqrt((theta[2*i,0]-theta[2*i+2,0])**2+(theta[2*i+1,0]-theta[2*i+3,0])**2)
    i=nb-1
    d[i,0]=np.sqrt((theta[2*i,0]-theta[0,0])**2+(theta[2*i+1,0]-theta[1,0])**2)

    #create empty k maatrix
    K=np.zeros((2*nb,2*nb))    

#fill matrix initial values
    K[0,0]=k*((d[nb-1,0]-l)/d[nb-1,0]+(d[0,0]-l)/d[0,0])
    K[0,2*nb-2]=-k*((d[nb-1,0]-l)/d[nb-1,0])
    K[0,2]=-k*((d[0,0]-l)/d[0,0])
    K[1,1]=K[0,0]
    K[1,2*nb-1]=K[0,2*nb-2]
    K[1,3]=K[0,2]
    
    #fill rest of the matrix
    for i in range(1,nb-1):
        K[2*i,2*i]=k*((d[i-1,0]-l)/d[i-1,0]+(d[i,0]-l)/d[i,0])
        K[2*i,2*i-2]=-k*((d[i-1,0]-l)/d[i-1,0])    
        K[2*i,2*i+2]=-k*((d[i,0]-l)/d[i,0])
        K[2*i+1,2*i+1]=K[2*i,2*i]
        K[2*i+1,2*i-1]=K[2*i,2*i-2]
        K[2*i+1,2*i+3]=K[2*i,2*i+2]        
        
    i=nb-1
    K[2*i,2*i]=k*((d[i-1,0]-l)/d[i-1,0]+(d[i,0]-l)/d[i,0])
    K[2*i,2*i-2]=-k*((d[i-1,0]-l)/d[i-1,0])    
    K[2*i,0]=-k*((d[i,0]-l)/d[i,0])
    K[2*i+1,2*i+1]=K[2*i,2*i]
    K[2*i+1,2*i-1]=K[2*i,2*i-2]
    K[2*i+1,1]=K[2*i,0]       
    
    # boundary matrix for LP equal to original K
    Kb=K
    temp=np.zeros((2*nb,2*ni))
    K=np.concatenate((Kb,temp),axis=1)
    temp=np.zeros((2*ni,2*(nb+ni)))
    K=np.concatenate((K,temp),axis=0)
    
    #
    d=np.zeros((n,n))    
    for i in range(n):
        for j in range(n):
            if i!=j:
                d[i,j]=-p*epsilon*rm**(p)*((theta[2*i,0]-theta[2*j,0])**2+(theta[2*i+1,0]-theta[2*j+1,0])**2)**(-(p/2+1))
    dii=np.sum(d,axis=1)
    
    #genrate leonard jones potential matrix KL
    KL=np.zeros((2*n,2*n))
    for i in range(n):
        for j in range(n):
            I1=2*i
            J1=2*j
            I2=2*i+1
            J2=2*j+1
            
            KL[I1,J1]=-d[i,j]
            if i==j:
                KL[I1,J1]=dii[i]
                
            KL[I2,J2]=KL[I1,J1]



    K=K+KL
    
    return K


# Function that returns thetaDot
def model(r,v,c,n,nb,ni,l,Minv,F,Fg,lb):
    
    K=modelStiffness(r,n,nb,ni,l)
    
    
       
    theta=r
    theta=np.reshape(theta,(-1,1))
    
    
    
    #create empty matrix for unit vector
    u=np.zeros((2*(nb+ni),1))
    
    #unit vector for x direction
    for i in range(nb-1):
        u[i,0]=theta[2*i,0]-theta[2*i+1,0]/np.sqrt((theta[2*i,0]-theta[2*i+2,0])**2+(theta[2*i+1,0]-theta[2*i+3,0])**2)
    i=nb-1
    u[i,0]=(theta[2*i]-theta[0,0])/np.sqrt((theta[2*i,0]-theta[0,0])**2+(theta[2*i+1,0]-theta[1,0])**2)
#unit vector for y direction
    for i in range(nb-1):
        u[i,0]=theta[2*i+1,0]-theta[2*i+2,0]/np.sqrt((theta[2*i,0]-theta[2*i+2,0])**2+(theta[2*i+1,0]-theta[2*i+3,0])**2)
    i=nb-1
    u[i,0]=(theta[2*i+1]-theta[0,0])/np.sqrt((theta[2*i,0]-theta[0,0])**2+(theta[2*i+1,0]-theta[1,0])**2)

 #forces applied to it
    F2=u
    v=np.reshape(v,(-1,1))
    F=F+F2+Fg-c*np.dot(Minv,v)

    Cb=np.zeros((2*n,1))    
    for i in range(0,2*n,2):
        Cb[i,0]=p*epsilon*((rm**p)/(theta[i,0]-lb)**(p+1)+(rm**p)/(theta[i,0]+lb)**(p+1))
    
    for i in range(1,2*n,2):
        Cb[i,0]=p*epsilon*((rm**p)/(theta[i,0]-lb)**(p+1)+(rm**p)/(theta[i,0]+lb)**(p+1))

   #unit vector for x direction
  

    F=F+Cb

#    Ci=np.zeros((2*n,1))
#    temp=p*epsilon*(rm**p)*(aa**2+bb**2)**(p/2)    
#    for i in range(0,n):
#        Ci[2*i,0]=aa*temp/((aa*theta[2*i,0]+bb*theta[2*i+1,0]+cc)**(p+1))
#        Ci[2*i+1,0]=bb*temp/((aa*theta[2*i,0]+bb*theta[2*i+1,0]+cc)**(p+1))
#
#    F=F+Ci

    #x0=4
    #y0=0
    #rd=.75
    #Co=np.zeros((2*n,1))
    #temp=p*epsilon*(rd**p)    
    #for i in range(0,n):
        #Co[2*i,0]=temp*(theta[2*i,0]-x0)/(((theta[2*i,0]-x0)**2+(theta[2*i+1,0]-y0)**2)**(p/2+1))
        #Co[2*i+1,0]=temp*(theta[2*i+1,0]-y0)/(((theta[2*i,0]-x0)**2+(theta[2*i+1,0]-y0)**2)**(p/2+1))

    #F=F+Co

    #x1=4
    #y1=-.5
    #y2=.5

    #Cb=np.zeros((2*n,1))    
    #for i in range(0,2*n,2):
        #Cb[i,0]=p*epsilon*((rm**p)/(theta[i,0]-x1)**(p+1)+(rm**p)/(theta[i,0]-x1)**(p+1))
    
    #for i in range(1,2*n,2):
        #Cb[i,0]=p*epsilon*((rm**p)/(theta[i,0]-y1)**(p+1)+(rm**p)/(theta[i,0]-y2)**(p+1))

    #Cp=np.zeros((2*n,1))
    #for i in range(0,2*n,2):
        #Cp[i,0]=(rm**p)*p*epsilon*((theta[i,0]-x1))/(((theta[i,0]-x1)**2)+((theta[i+1]-y1)**2))**(p+1)+((rm**p)*p*epsilon*((theta[i,0]-x1))/(((theta[i,0]-x1)**2)+((theta[i+1]-y2)**2))**(p+1))
 
    #for i in range(1,2*n,2):
         #Cp[i,0]=(rm**p)*p*epsilon*((theta[i,0]-y1))/(((theta[i-1,0]-x1)**2)+((theta[i,0]-y1)**2))**(p+1)+((rm**p)*p*epsilon*((theta[i,0]-y2))/(((theta[i-1,0]-x1)**2)+((theta[i]-y2)**2))**(p+1))
 
    #F=F+Cp

    d=np.zeros((nb,1))    
    for i in range(nb-1):
        d[i,0]=np.sqrt((theta[2*i,0]-theta[2*i+2,0])**2+(theta[2*i+1,0]-theta[2*i+3,0])**2)
    i=nb-1
    d[i,0]=np.sqrt((theta[2*i,0]-theta[0,0])**2+(theta[2*i+1,0]-theta[1,0])**2)

    
    rnext=np.dot(Minv,(F-np.dot(K,theta)))
    rnext=rnext.flatten()

    return rnext




#Function for calculating equilibrium equations
def modelEquilibrium(r,n,nb,ni,l):
    
    K=modelStiffness(r,n,nb,ni,l)
    r=np.reshape(r,(-1,1))
    eqs=np.dot(K,r)
    eqs=eqs.flatten()
    print(np.sum(np.absolute(eqs)))
    
    return eqs


h=1e-2
nsteps=300
r=np.zeros((2*n,nsteps))









    
    

r[:,0]=r0
r[:,1]=r[:,0]+h*v+h**2*model(r[:,0],v,c,n,nb,ni,l,Minv,F,Fg,lb)/2
v=(r[:,1]-r[:,0])/h

for i in range(1,nsteps-1):
    
  
    print(i)
    r[:,i+1]=2*r[:,i]-r[:,i-1]+h**2*model(r[:,i],v,c,n,nb,ni,l,Minv,F,Fg,lb)
    v=(r[:,i+1]-r[:,i])/h
    
#theta0[0]=theta0[0]-theta0[0]/30

theta=np.transpose(r)
x=theta[0:nsteps,0:2*n:2]
y=theta[0:nsteps,1:2*n:2]

np.savez('temp.npz',x=x,y=y,nb=nb,ni=ni,n=n)
np.savez('resumeData.npz',r0=r[:,nsteps-1],v=v,nb=nb,ni=ni,n=n,m=m,l=l,k=k,g=g,c=c,Minv=Minv,lb=lb,Fg=Fg,rm=rm,epsilon=epsilon,p=p,F=F)
