
import numpy as np
from scipy.optimize import fsolve
#from scipy.integrate import odeint
#import matplotlib.pyplot as plt
#from matplotlib import animation


#mass, length of spring, stiffness constant, acceleration due to gravity
m,l,k,g=1.,0.1,100,0
#Number of boundary robots, interior robots
nb=100
ni=50
n=nb+ni
#Damping coefficient
c=.4
#Mass Matrix
M=m*np.identity(2*n)
Minv=np.linalg.inv(M)

#External Force
F=np.zeros((2*n,1))

#Gravitational Force
Fg=np.zeros((2*n,1))
Fg[1:2*n:2,0]=-m*g

#Lennard Jones parameters
rm,epsilon,p=.2,1.,2

#Boundary -x1=x2=-y1=y2=lb
lb=8

#incline parameters
aa,bb,cc=1.,1.,5.

#circular obstruction
x0,y0,rd=2.5,-5.,2.




# function that returns the stiffness matrix of the robot
def modelStiffness(r,n,nb,ni,l):
    
    theta=r
    theta=np.reshape(theta,(-1,1))
    
    #define distance
    d=np.zeros((nb,1))    
    for i in range(nb-1):
        d[i,0]=np.sqrt((theta[2*i,0]-theta[2*i+2,0])**2+(theta[2*i+1,0]-theta[2*i+3,0])**2)
    i=nb-1
    d[i,0]=np.sqrt((theta[2*i,0]-theta[0,0])**2+(theta[2*i+1,0]-theta[1,0])**2)

    
    #create empty k matrix
    K=np.zeros((2*nb,2*nb))    

    K[0,0]=k*((d[nb-1,0]-l)/d[nb-1,0]+(d[0,0]-l)/d[0,0])
    K[0,2*nb-2]=-k*((d[nb-1,0]-l)/d[nb-1,0])
    K[0,2]=-k*((d[0,0]-l)/d[0,0])
    K[1,1]=K[0,0]
    K[1,2*nb-1]=K[0,2*nb-2]
    K[1,3]=K[0,2]
    
    
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
    
    Kb=K
    temp=np.zeros((2*nb,2*ni))
    K=np.concatenate((Kb,temp),axis=1)
    temp=np.zeros((2*ni,2*(nb+ni)))
    K=np.concatenate((K,temp),axis=0)
    
    # create another empty d matrix
    d=np.zeros((n,n))    
    for i in range(n):
        for j in range(n):
            if i!=j:
                d[i,j]=-p*epsilon*rm**(p)*((theta[2*i,0]-theta[2*j,0])**2+(theta[2*i+1,0]-theta[2*j+1,0])**2)**(-(p/2+1))
    dii=np.sum(d,axis=1)
    
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
    v=np.reshape(v,(-1,1))
    F=F+Fg-c*np.dot(Minv,v)

    #Cb=np.zeros((2*n,1))    
    #for i in range(0,2*n,2):
     #   Cb[i,0]=p*epsilon*((rm**p)/(theta[i,0]-lb)**(p+1)+(rm**p)/(theta[i,0]+lb)**(p+1))
    
    #for i in range(1,2*n,2):
     #   Cb[i,0]=p*epsilon*((rm**p)/(theta[i,0]-lb)**(p+1)+(rm**p)/(theta[i,0]+lb)**(p+1))

    #F=F+Cb

#    Ci=np.zeros((2*n,1))
#   temp=p*epsilon*(rm**p)*(aa**2+bb**2)**(p/2)    
#    for i in range(0,n):
#        Ci[2*i,0]=aa*temp/((aa*theta[2*i,0]+bb*theta[2*i+1,0]+cc)**(p+1))
#        Ci[2*i+1,0]=bb*temp/((aa*theta[2*i,0]+bb*theta[2*i+1,0]+cc)**(p+1))
#
#    F=F+Ci

#circular obstruction
    #Co=np.zeros((2*n,1))
    #temp=p*epsilon*(rd**p)    
    #for i in range(0,n):
     #   Co[2*i,0]=temp*(theta[2*i,0]-x0)/(((theta[2*i,0]-x0)**2+(theta[2*i+1,0]-y0)**2)**(p/2+1))
      #  Co[2*i+1,0]=temp*(theta[2*i+1,0]-y0)/(((theta[2*i,0]-x0)**2+(theta[2*i+1,0]-y0)**2)**(p/2+1))

    #F=F+Co

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


a=2
for i in range(nb):
    r[2*i,0]=a*np.cos(i*2*np.pi/nb)
    r[2*i+1,0]=a*np.sin(i*2*np.pi/nb)

for i in range(ni):
    j=i+nb
    rk=a*np.random.rand()
    thetak=2*np.pi*np.random.rand()
    r[2*j,0]=np.sqrt(rk)*np.cos(thetak)
    r[2*j+1,0]=np.sqrt(rk)*np.sin(thetak)


#temp=fsolve(lambda x: modelEquilibrium(x,n,nb,ni,l), r[:,0])
    
v=np.zeros((2*n))

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
#np.savez('jamming.npz',r0=r[:,nsteps-1],v=v,nb=nb,ni=ni,n=n,m=m,l=l,k=k,g=g,c=c,Minv=Minv,lb=lb,Fg=Fg,rm=rm,epsilon=epsilon,p=p,F=F)