# -*- coding: utf-8 -*-
"""
Created on Tue Dec 17 09:32:56 2019

@author: dmulr
"""

# Import libraries
import numpy as np
from bridson import poisson_disc_samples
import math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib import animation
import sys

from numpy import array, dot
from qpsolvers import solve_qp
from scipy.optimize import minimize 
from IPython.display import Image
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
# In[Robot]
class Robot:
    def __init__(self, x, y, phi, xv, yv, phiv,R,m,I,Fx,Fy,T,num,index):
        self.num=num
        self.index=index
        # tell it its ridged
        self.type='robot'
        # geometry is a circle
        self.geom='circle'
        self.q=np.array([[x,y,phi]]).T  # positons matrix
        self.x=x    # x position
        self.y=y    # y position
        self.phi=phi    # rotation
        self.v=np.array([[xv,yv,phiv]]).T   # velocity
        self.xv=xv # x velocity
        self.yv=yv # y velocity
        self.phiv=phiv # rotational velocity
        self.M=np.zeros((3,3))  # empty mass matrix
        self.M[0,0],self.M[1,1],self.M[2,2]=m,m,I   # mass matrix
        self.R=R # Radius
        self.m=m # mass
        self.I=I # inertia
        self.Fx=Fx # xForce
        self.Fy=Fy # y force
        self.T=T # torque
        self.qxt=[]
        self.qyt=[]
        self.phit=[] # empty matrix to store data
        self.xd=0 # desired x position
        self.yd=0 # desired y position
        self.phid=0                
# updates position
    def updatePosition(self,q):
        self.q=q
        self.x,self.y,self.phi=q[0,0],q[1,0],q[2,0]
# update velocity
    def updateVelocity(self,v):
        self.v=v
# get positions
    def getPosition(self):
        return(self.q[0,0],self.q[1,0],self.q[2,0])
# Update Desired Position
#    def updateDesiredPosition(self,qd):
#        self.xd=qd[0,0]
#        self.yd=qd[0,1]
# update forces applied        
    def updateForces(self,F):
        self.Fx=F[0,0]
        self.Fy=F[1,0]
        self.T=F[2,0]
    def extract_data(self):
        self.qxt.append(self.x)
        self.qyt.append(self.y)
        self.phit.append(self.phi)

            
# In[Internal particles]
class Particle:
    def __init__(self, x, y, phi, xv, yv, phiv,R,m,I,num,index):
        self.num=num
        self.index=index
        # tell it its ridged
        self.type='particle'
        # geometry is a circle
        self.geom='circle'
        self.q=np.array([[x,y,phi]]).T  # positons matrix
        self.x=x    # x position
        self.y=y    # y position
        self.phi=phi    # rotation
        self.v=np.array([[xv,yv,phiv]]).T   # velocity
        self.xv=xv # x velocity
        self.yv=yv # y velocity
        self.phiv=phiv # rotational velocity
        self.M=np.zeros((3,3))  # empty mass matrix
        self.M[0,0],self.M[1,1],self.M[2,2]=m,m,I   # mass matrix
        self.qxt=[]
        self.qyt=[]
        self.phit=[] # empty matrix to store data
        self.R=R # Radius
        self.m=m # mass
        self.I=I # inertia 
# update position
    def updatePosition(self,q):
        self.q=q
        self.x,self.y,self.phi=q[0,0],q[1,0],q[2,0]
# update velocity
    def updateVelocity(self,v):
        self.v=v
# get positions
    def getPosition(self):
        return(self.q[0,0],self.q[1,0],self.q[2,0])
# Extract data
    def extract_data(self):
        self.qxt.append(self.x)
        self.qyt.append(self.y)
        self.phit.append(self.phi)

# In[create classes for obstacles]
class obstacles:
    count=0
    def __init__(self):
        self.index=obstacles.count
        obstacles.count += 1


# In[Create line]
class line2d(obstacles):
    def __init__(self,x1,y1,x2,y2):
        obstacles.__init__(self)
        self.x1=x1
        self.x2=x2
        self.y1=y1
        self.y2=y2
        self.z1=0
        self.z2=0
        self.r1=np.array([[self.x1,self.y1,self.z1]])
        self.r2=np.array([[self.x2,self.y2,self.z2]])
        self.type='obstacle'
        self.geom='lineObstacle'

# In[Horizontal line obstacle]
class horizontalLine(obstacles):
    def __init__(self,y0):
        obstacles.__init__(self)
        self.y=y0       # postion
        self.type='obstacle'  # tell it what it is
        self.geom='horizontalLine'   # geometry

# In[Vertical line obstacle]
class verticalLine(obstacles):
    def __init__(self,x0):
        obstacles.__init__(self)
        # x intersection
        self.x=x0
        # type of object it is
        self.type='obstacle'
        # specific geometry
        self.geom='verticalLine'


# In[Create circular obstacle] 
class circleObstacle(obstacles):
    def __init__(self,x,y,R):
        obstacles.__init__(self)
        self.x=x
        self.y=y
        self.R=R
        self.z=0
        self.r=np.array([[self.x,self.y,self.z]])
        self.type='obstacle'
        self.geom='circleObstacle'


# In[Create Global Matrix for mass,velocity, position]
def createGlobalMatrices(nt,obj):
    # empy M matrix
    M=np.zeros((3*nt,3*nt))
    # empty q matrix this is x, y and phi
    q=np.zeros((3*nt,1))
    #  velocity matrix so vx, vy, phi dot.
    v=np.zeros((3*nt,1))
    # fill the matrices
    for i in range (len(obj)):
        # only fill if a ruidged body and not a wall
        if(obj[i].type=='robot' or obj[i].type=='particle'):
            ind=obj[i].index
            # Mass matrix
            M[3*ind:3*ind+3,3*ind:3*ind+3]=obj[i].M
            # create position matrix
            q[3*ind:3*ind+3,:]=obj[i].q
            # create velocity matrix
            v[3*ind:3*ind+3,:]=obj[i].v
    return (M,q,v)

# In[Max values of the interior]
def MaxValues(R1,diameter,nb):
    Rin=R1-diameter/2
    ngrans1=int(Rin/(diameter))

    ri=np.zeros((1,ngrans1))
    ni=np.zeros((1,ngrans1))
    
    radii=Rin-(diameter)
    for i in range(ngrans1):
        remainder=((diameter))*i-.2
        ri[:,i]=radii-remainder
        ni[:,i]=np.floor((ri[:,i]*np.pi*2)/diameter)

    ni=np.asarray(ni,dtype=int)

    return(ni)
    
# In[Controls class]    
class controls:
    def __init__(self,nb,qbar,vbar,qall,Qd,Q,F,FE,fny,fnx,alpha,beta,rbf,method,path,Et,E,tstep,time,error,tbar,h,m,j):
        self.nb=nb
        self.qbar=qbar
        self.qout=np.zeros(2*self.nb)
        self.qall=qall
        self.j=j
        self.fny=fny
        self.fnx=fnx
        self.alpha=alpha
        self.beta=beta
        self.vbar=vbar
        self.method=method
        self.qbar=qbar
        self.xb=0
        self.yb=0
        self.vxb=0
        self.vyb=0
        self.Fb=np.zeros(2*self.nb)
        self.F=F
        # variables specific to point to point
        self.FE=FE
        self.qkb=np.zeros(2*self.nb)
        self.vkb=np.zeros(2*self.nb)
        self.Feb=np.zeros(2*self.nb)
        self.Qd=Qd
        self.Q=Q
        self.m=m
        self.h=h
        self.tbar=tbar
        self.tstep=tstep
        self.time=time
        self.timep=[]
        self.path=path
        self.Et=Et
        self.E=E
        self.step=0
        self.error=error
        self.rbf=rbf
#path desired        
    def Path_desired(self):

        if self.path=='up_right':
            Q2=np.zeros(2*self.nb)
            Q3=np.zeros(2*self.nb)
            for i in range(self.nb):
                Q2[2*i+1]=1
                Q3[2*i]=1
            for i in range(self.time.size-1):
                self.Q[:,i+1]=(.3*self.tstep)*Q2+(.3*self.tstep)*Q3+self.Q[:,i]    
        return (self.Q)
# get positions
    def get_positions(self):
        x=[]
        y=[]
        for i in range(self.nb):
            x.append(self.qbar[3*i])
            y.append(self.qbar[3*i+1])
            self.xb=np.asarray(x)
            self.yb=np.asarray(y)
        return(self.xb,self.yb)
# get velocities
    def get_velocities(self):
        vx=[]
        vy=[]
        for i in range(self.nb):
            vx.append(self.vbar[3*i])
            vy.append(self.vbar[3*i+1])
            self.vxb=np.asarray(vx)
            self.vyb=np.asarray(vy)
        return(self.vxb,self.vyb)
        
# Get membrane force        
    def get_membrane_force(self):
        for i in range(self.nb):
            self.Feb[2*i]=self.FE[3*i]
            self.Feb[2*i+1]=self.FE[3*i+1]
        return(self.Feb)
# calculate force on bots        
    def Force_on_bots(self):
        if self.method=='potential':    

            for i in range(self.nb):
                self.Fb[2*i]=self.fnx([self.xb[i],self.yb[i]])
                self.Fb[2*i+1]=self.fny([self.xb[i],self.yb[i]])
                
        if self.method=='optim_ptp':
            for i in range(self.nb):
                self.Fb[2*i]=(self.m/self.h**2)*(self.Q[2*i,self.step]-self.xb[i])-(self.vxb[i]*self.m/self.h)-self.tbar*self.Feb[2*i]
                self.Fb[2*i+1]=(self.m/self.h**2)*(self.Q[2*i+1,self.step]-self.yb[i])-(self.vyb[i]*self.m/self.h)-self.tbar*self.Feb[2*i+1]
        
        return(self.Fb)
# Apply forces 
    def Apply_force(self):
        if self.method=='potential':
            for i in range(self.nb):
                self.F[3*i,self.j]=-(self.alpha*self.Fb[2*i+1])-self.beta*self.vbar[3*i]
                self.F[3*i+1,self.j]=-(self.alpha*self.Fb[2*i])-self.beta*self.vbar[3*i+1]
        if self.method=='optim_ptp':
            for i in range(self.nb):
                self.F[3*i,self.j]=self.Fb[2*i]
                self.F[3*i+1,self.j]=self.Fb[2*i+1]            
        return(self.F)
# update qout        
    def update_qout(self):
        for i in range(self.nb):
            self.qout[2*i]=self.qall[3*i]
            self.qout[2*i+1]=self.qall[3*i+1]
        return(self.qout)


# Next position   
    def Next_position(self):
        if self.E[0,self.j]<self.error:
            self.step=self.step+1
        else:
            self.step=self.step
            
        return(self.step)
# Error       
    def Error(self):
        if self.method=='optim_ptp':
            for i in range(self.nb):
                self.Et[2*i,self.j]=np.nan_to_num(self.qout[2*i]-self.Q[2*i,self.step]/self.Q[2*i,self.step])
                self.Et[2*i+1,self.j]=np.nan_to_num(self.qout[2*i+1]-self.Q[2*i+1,self.step]/self.Q[2*i+1,self.step])
            self.E[0,self.j]=np.nan_to_num((np.linalg.norm(self.Et[:,self.j])**2))
            
        if self.method=='potential':
    
            et=[]
            for i in range(self.nb):
                val=.5*(self.rbf(self.qout[2*i+1],self.qout[2*i]))**2
                et.append(val)
            self.E[0,self.j]=sum(et)
  
    
        return(self.E,self.Et)
# controller    
    def Controller(self):
        # potential field method
        if self.method=='potential':
            (self.xb,self.yb)=self.get_positions()
            (self.Fb)=self.Force_on_bots()
            (self.F)=self.Apply_force()
        # ptp method
        if self.method=='optim_ptp':
            (self.xb,self.yb)=self.get_positions()
            (self.vxb,self.vyb)=self.get_velocities() 
            (self.Feb)=self.get_membrane_force()
            (self.Fb)=self.Force_on_bots()
            (self.F)=self.Apply_force()
        
                        

        return(self.Fb,self.F)
# In[Points for shape]            
class Points_for_shape:
    def __init__(self,shape,p1,p2,nb,nr,R,bl,br):
        self.shape=shape
        self.p1=p1
        self.p2=p2
        self.nb=nb
        self.nr=nr
        self.R=R
        self.x=0
        self.y=0
        self.z=0
        self.rbf=0
        self.br=br
        self.bl=bl
        self.fny=0
        self.fnx=0
        
        
    
    def Create_shape_gradient(self):
        (self.x,self.y,self.z) =self.Points_for_shape()
        (self.rbf)=self.create_RBF()
        ti = np.linspace(self.bl, self.br, 100)
        xx, yy = np.meshgrid(ti, ti)
        zz = self.rbf(xx, yy)
        (zy,zx)=np.gradient(zz**2)
        # gradient of y
        self.fny = RegularGridInterpolator((ti,ti),zy)
        # gradient of x
        self.fnx = RegularGridInterpolator((ti,ti),zx)
        return(self.rbf,self.fnx,self.fny)

    def Points_for_shape(self):
        # grab
        if self.shape=='grab':
            (self.x,self.y,self.z)=self.points_grab()
            # Circle           
        if self.shape=='circle':
            (self.x,self.y,self.z)=self.Points_circle()
            # Square                           
        if self.shape=='Square':
            (self.x,self.y,self.z)=self.points_square()            
        return (self.x,self.y,self.z)  
    
# create radial basis function                 
    def create_RBF(self):
        self.rbf = Rbf(self.x,self.y,self.z,function='thin_plate')  # radial basis function interpolator instance
        return (self.rbf) 
    
# In[ points for Square]    
    def points_square(self):
        xt=np.array([1,1,0,-1,-1,-1,0,1])
        yt=np.array([0,1,1,1,0,-1,-1,-1])
    
        self.x=np.zeros(len(self.nr)*len(xt))
        self.y=np.zeros(len(self.nr)*len(xt))
        self.z=np.zeros(len(self.nr)*len(xt))

    
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                self.x[i+len(xt)*j]=self.nr[j]*xt[i]
                self.y[i+len(xt)*j]=self.nr[j]*yt[i]
                if j==0:
                    self.z[i+len(xt)*j]=0
                else:
                    self.z[i+len(xt)*j]=j             
        return (self.x,self.y,self.z)   
# In[points for grab]
    def points_grab(self):
        theta1=np.linspace(np.pi/4,7*np.pi/4,50)
        x1=.75*np.cos(theta1)+self.p1
        y1=.75*np.sin(theta1)+self.p2

        theta2=np.linspace(np.pi/2,3*np.pi/2,20)
        x2=.5*np.cos(theta2)+.6+self.p1
        y2=.4*np.sin(theta2)+self.p2   
    
        xp=np.concatenate((x1, x2), axis=None)
        yp=np.concatenate((y1, y2), axis=None)
        zp=np.zeros(len(xp))
    
        theta3=np.linspace(np.pi/2+.3,3*np.pi/2-.3,20)
        x3=.5*np.cos(theta3)+1+self.p1
        y3=.4*np.sin(theta3)+self.p2

        theta4=np.linspace(np.pi/4-.2,7*np.pi/4+.2,50)
        x4=np.cos(theta4)+self.p1
        y4=np.sin(theta4)+self.p2
    
        xr=np.concatenate((x3, x4), axis=None)
        yr=np.concatenate((y3, y4), axis=None)
        zr=np.ones(len(xr))
    
    
        self.x=np.concatenate((xp, xr), axis=None)
        self.y=np.concatenate((yp, yr), axis=None)
        self.z=np.concatenate((zp, zr), axis=None) 
        return (self.x,self.y,self.z)    
# In[ Points for circle]
    def Points_circle(self):
        theta=np.linspace(0,2*np.pi,self.nb)

    # create empty arays
        self.x=np.zeros(len(self.nr)*self.nb)
        self.y=np.zeros(len(self.nr)*self.nb)
        self.z=np.zeros(len(self.nr)*self.nb)
# create rings
        for j in range (len(self.nr)):
            for i in range(self.nb):
                nt=self.nr[j]
                self.x[i+self.nb*j]=(self.R+nt)*np.cos(theta[i])+self.p1
                self.y[i+self.nb*j]=(self.R+nt)*np.sin(theta[i])+self.p2
                if j==0:
                    self.z[i+self.nb*j]=0
                else:
                    self.z[i+self.nb*j]=nt
        return (self.x,self.y,self.z)     
                

# In[Create gravity Fg]
def gravity(obj,q0):
    Fg=np.zeros_like(q0)
    ndof=np.size(q0)
    count=int(ndof/3)

    for i in range(count):
        Fg[3*i+1,0]=-9.8*obj[i].m

    return Fg

# In[Create stiffness matrix for boundary robots]
def modelStiffness(q,nb,ni,l,k):
    # number of total particles
    n=nb+ni
    # define theta variable
    theta=q
    # reshape it
    theta=np.reshape(theta,(-1,1))
    # create empty distance matrix
    d=np.zeros((nb,1))    
    # implement distance
    for i in range(nb-1):
        d[i,0]=np.sqrt((theta[3*i,0]-theta[3*i+3,0])**2+(theta[3*i+1,0]-theta[3*i+4,0])**2)
    i=nb-1
    d[i,0]=np.sqrt((theta[3*i,0]-theta[0,0])**2+(theta[3*i+1,0]-theta[1,0])**2)
    
    # Stiffness matrix 
    K=np.zeros((3*nb,3*nb))    
    K[0,0]=k*((d[nb-1,0]-l)/d[nb-1,0]+(d[0,0]-l)/d[0,0])
    K[0,3*nb-3]=-k*((d[nb-1,0]-l)/d[nb-1,0])
    K[0,3]=-k*((d[0,0]-l)/d[0,0])
    K[1,1]=K[0,0]
    K[1,3*nb-2]=K[0,3*nb-3]
    K[1,4]=K[0,3]
    for i in range(1,nb-1):
        K[3*i,3*i]=k*((d[i-1,0]-l)/d[i-1,0]+(d[i,0]-l)/d[i,0])
        K[3*i,3*i-3]=-k*((d[i-1,0]-l)/d[i-1,0])    
        K[3*i,3*i+3]=-k*((d[i,0]-l)/d[i,0])
        K[3*i+1,3*i+1]=K[3*i,3*i]
        K[3*i+1,3*i-2]=K[3*i,3*i-3]
        K[3*i+1,3*i+4]=K[3*i,3*i+3]          
    i=nb-1
    K[3*i,3*i]=k*((d[i-1,0]-l)/d[i-1,0]+(d[i,0]-l)/d[i,0])
    K[3*i,3*i-3]=-k*((d[i-1,0]-l)/d[i-1,0])    
    K[3*i,0]=-k*((d[i,0]-l)/d[i,0])
    K[3*i+1,3*i+1]=K[3*i,3*i]
    K[3*i+1,3*i-2]=K[3*i,3*i-3]
    K[3*i+1,1]=K[3*i,0]
    Kb=K
    temp=np.zeros((3*nb,3*ni))
    K=np.concatenate((Kb,temp),axis=1)
    temp=np.zeros((3*ni,3*(nb+ni)))
    K=np.concatenate((K,temp),axis=0)   
    return K,d





def simulate(obj,qbar,v,M,Fext,j,q,tbar,tstep,mu,en,et,ndof):
    # update coordinates
    updateCoordinates(obj,qbar,v[:,j])
    #calculated gaps and determine which ones ar gunna collide 
    gbar,ind=calculateGaps(obj) 
    # update the positions 
    updateCoordinates(obj,q[:,j],v[:,j]) 

#Calculating vfree
    vfree=v[:,j]+tstep*np.matmul(np.linalg.inv(M),Fext)
    

    # In[if theres no collisions]
    if ind.size<1:
        v[:,j+1]=vfree
    # In[If there are contacts]
    else:
        (Rk1,H)=LCP_Com_Ex(obj,ind,ndof,vfree,M,mu,en,et,v[:,j],q[:,j])
        # Vk+1
      #  Fc=np.matmul(H,Rk1).reshape(ndof,)
      #  Fcontact[:,j]=Fc
        v[:,j+1]=vfree+(np.matmul(np.matmul(np.linalg.inv(M),H),Rk1)).reshape(ndof,)


    # For implicit integration of q
    vkt=tstep*(tbar*v[:,j+1]+(1-tbar)*v[:,j])

    #Implicit integration of q
    q[:,j+1]=q[:,j]+vkt
        
        
    return(q,v)





# In[Update coordinates]
def updateCoordinates(obj,qb,vb):
    for i in range (0,len(obj)):
        # classify if object is a ridged body
        if(obj[i].type=='robot' or obj[i].type=='particle'):
            ind=obj[i].index # objects index
            obj[i].updatePosition(qb[3*ind:3*ind+3].reshape(3,1))# update position
            obj[i].updateVelocity(vb[3*ind:3*ind+3].reshape(3,1))# update velocities

# In[Update Forces]      
def updateForces(obj,F):
    for i in range(0,len(obj)):
        if(obj[i].type=='robot'):
            ind=obj[i].index
            obj[i].updateForces(F[3*ind:3*ind+3].reshape(3,1))
# In[Distance]
def dist2D(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2+(y1-y2)**2)

# In[FIND GAP FUNCTION]
def calculateGaps(obj):
    g=np.zeros((len(obj),len(obj)))    # empty gap matrix
    ind=[]    # empty indices matrix
    for i in range (0,len(obj)):
        for j in range (i+1,len(obj)): # plus one is to ensure it only calculates each gap once
    # if the geometry is a circle and a circle
            if((obj[i].geom=='circle' and obj[j].geom=='circle') or (obj[j].geom=='circle' and obj[i].geom=='circle')):
                g[i,j]=dist2D(obj[i].x,obj[i].y,obj[j].x,obj[j].y)-(obj[i].R+obj[j].R)
    # if geomtery is a circle and a horizontal line
            if ((obj[i].geom=='circle' and obj[j].geom=='horizontalLine')):
                g[i,j]=dist2D(0,obj[i].y,0,obj[j].y)-obj[i].R

            if ((obj[i].geom=='horizontalLine' and obj[j].geom=='circle')):
                g[i,j]=dist2D(0,obj[i].y,0,obj[j].y)-obj[j].R
    # if geomtry is vertical line and a circle
            if ((obj[i].geom=='verticalLine' and obj[j].geom=='circle')):
                g[i,j]=dist2D(obj[i].x,0,obj[j].x,0)-obj[j].R

            if ((obj[i].geom=='circle' and obj[j].geom=='verticalLine')):
                g[i,j]=dist2D(obj[i].x,0,obj[j].x,0)-obj[i].R
    # If geometry is a circle and acircle obstacle            
            if ((obj[i].geom=='circle' and obj[j].geom=='circleObstacle') or (obj[j].geom=='circleObstacle' and obj[i].geom=='circle')):
                g[i,j]=dist2D(obj[i].x,obj[i].y,obj[j].x,obj[j].y)-(obj[i].R+obj[j].R)
    # if a collision is dedectid meaning g is less than zero it will save the indices or the object numbers that collided.
            if(g[i,j]<0):
                # add the number of the two objects colliding
                ind.append([i,j])
    # convert to array
    ind=np.array(ind)
    # flatten it
    #ind.flatten()
    # make sure its int 64. Not sure why but it wasnt liking it in any other form
    ind=np.int64(ind)

    return g,ind

           
# I[h matrix local]
def geth(theta,phi,a,b):
    #A matrix
    A=np.array([[-np.sin(phi), -np.cos(phi)],[np.cos(phi),-np.sin(phi)]])
    # f matrix
    f=np.array([1*np.cos(theta),1*np.sin(theta)])
    temp=np.dot(A,f)
    temp=np.array(temp)
    f1,f2=temp[0],temp[1]
    h=np.array([a,b,a*f1+b*f2])
    h=np.transpose(h)

    return(h)
    
# In[Get H matrix]
def getH(obj,ind,ndof):
    s2=np.shape(ind)
    alpha=s2[0]
    HN,HT=np.zeros((ndof,alpha)),np.zeros((ndof,alpha))
    #ind=np.array(ind)
    #ind.flatten()
    for g in range (alpha):
        # declare indices
        i=ind[g,0]
        j=ind[g,1]

        # if interaction is with horizontal wall and circle
        if((obj[i].geom=='circle' and obj[j].geom=='horizontalLine') or (obj[j].geom=='circle' and obj[i].geom=='horizontalLine')):
            # If the contact is between a 
            if(obj[i].geom=='circle' and obj[j].geom=='horizontalLine'):
                ii=obj[i].index
                phi=obj[i].phi
                jj=obj[j].index

            else:
                ii=obj[j].index
                phi=obj[j].phi
                jj=obj[i].index

            if((obj[i].y-obj[j].y)<0):
                theta=np.pi/2-phi
                HN[3*ii:3*ii+3,g]=geth(theta,phi,0,-1)
                HT[3*ii:3*ii+3,g]=geth(theta,phi,-1,0)
            else:
                theta=3*np.pi/2-phi
                HN[3*ii:3*ii+3,g]=geth(theta,phi,0,1)
                HT[3*ii:3*ii+3,g]=geth(theta,phi,1,0)

        # if interaction is with vertical wall and circle

        elif((obj[i].geom=='circle' and obj[j].geom=='verticalLine') or (obj[j].geom=='circle' and obj[i].geom=='verticalLine')):

            if(obj[i].geom=='circle' and obj[j].geom=='verticalLine'):
                ii=obj[i].index
                phi=obj[i].phi
                jj=obj[j].index
            else:
                ii=obj[j].index
                phi=obj[j].phi
                jj=obj[i].index

            if((obj[i].x-obj[j].x)>0):
                theta=np.pi-phi
                HN[3*ii:3*ii+3,g]=geth(theta,phi,1,0)
                HT[3*ii:3*ii+3,g]=geth(theta,phi,0,-1)
            else:
                theta=2*np.pi-phi
                HN[3*ii:3*ii+3,g]=geth(theta,phi,-1,0)
                HT[3*ii:3*ii+3,g]=geth(theta,phi,0,1)

        # if interaction is with circle and circle
        elif((obj[i].geom=='circle' and obj[j].geom=='circle')):
            ii=obj[i].index
            jj=obj[j].index
            #Vector from i to j
            x1=obj[j].q[0]-obj[i].q[0]
            y1=obj[j].q[1]-obj[i].q[1]
            #Calculate counter-clockwise angle between the two vectors beginning from [1,0]
            alpha1=math.atan2(y1,x1)
            theta1=alpha1-obj[i].phi
            a,b=-np.cos(alpha1),-np.sin(alpha1)
            #out=geth(theta1,obj[i].phi,a,b)
            HN[3*ii:3*ii+3,g]=geth(theta1,obj[i].phi,a,b)
            HT[3*ii:3*ii+3,g]=geth(theta1,obj[i].phi,b,-a)   ########Not sure of this

            alpha2=alpha1+np.pi
            theta2=alpha2-obj[j].phi
            HN[3*jj:3*jj+3,g]=-geth(theta2,obj[j].phi,a,b)
            HT[3*jj:3*jj+3,g]=-geth(theta2,obj[j].phi,b,-a)   ########Not sure of this


        elif((obj[i].geom=='circleObstacle' and obj[j].geom=='circle') or (obj[j].geom=='circleObstacle' and obj[i].geom=='circle')):
            if (obj[i].geom=='circle' and obj[j].geom=='circleObstacle'):
                
                ii=obj[i].index
                jj=obj[j].index
                phi=obj[i].phi
                
                x1=obj[j].x-obj[i].q[0]
                y1=obj[j].y-obj[i].q[1]
                
            else:
                ii=obj[j].index
                phi=obj[j].phi
                jj=obj[i].index
                x1=obj[j].q[0]-obj[i].x
                y1=obj[j].q[0]-obj[i].y
                        #Vector from i to j

            #Calculate counter-clockwise angle between the two vectors beginning from [1,0]
            alpha1=math.atan2(y1,x1)

            theta1=alpha1-obj[i].phi
            a,b=-np.cos(alpha1),-np.sin(alpha1)
            #out=geth(theta1,obj[i].phi,a,b)
            HN[3*ii:3*ii+3,g]=geth(theta1,phi,a,b)
            HT[3*ii:3*ii+3,g]=geth(theta1,phi,b,-a)   ########Not sure of this
    H=np.concatenate((HN,HT),axis=1)

    return (H,HN,HT)

# In[LCP SOLVER]
def LCPSolve(M,q, pivtol=1e-8): # pivtol = smallest allowable pivot element
    rayTerm = False
    loopcount = 0
    if (q >= 0.).all(): # Test missing in Rob Dittmar's code
        # As w - Mz = q, if q >= 0 then w = q and z = 0
        w = q
        z = np.zeros_like(q)
        retcode = 0.
    else:
        dimen = M.shape[0] # number of rows
        # Create initial tableau
        tableau = np.hstack([np.eye(dimen), -M, -np.ones((dimen, 1)), np.asarray(np.asmatrix(q).T)])
        # Let artificial variable enter the basis
        basis = list(range(dimen)) # basis contains a set of COLUMN indices in the tableau 
        locat = np.argmin(tableau[:,2*dimen+1]) # row of minimum element in column 2*dimen+1 (last of tableau)
        basis[locat] = 2*dimen # replace that choice with the row 
        cand = locat + dimen
        pivot = tableau[locat,:]/tableau[locat,2*dimen]
        tableau -= tableau[:,2*dimen:2*dimen+1]*pivot # from each column subtract the column 2*dimen, multiplied by pivot 
        tableau[locat,:] = pivot # set all elements of row locat to pivot
        # Perform complementary pivoting
        oldDivideErr = np.seterr(divide='ignore')['divide'] # suppress warnings or exceptions on zerodivide inside numpy
        while np.amax(basis) == 2*dimen:
            loopcount += 1
            eMs = tableau[:,cand]    # Note: eMs is a view, not a copy! Do not assign to it...
            missmask = eMs <= 0.
            quots = tableau[:,2*dimen+1] / eMs # sometimes eMs elements are zero, but we suppressed warnings...
            quots[missmask] = np.Inf # in any event, we set to +Inf elements of quots corresp. to eMs <= 0. 
            locat = np.argmin(quots)
            if abs(eMs[locat]) > pivtol and not missmask.all(): # and if at least one element is not missing 
                # reduce tableau
                pivot = tableau[locat,:]/tableau[locat,cand]
                tableau -= tableau[:,cand:cand+1]*pivot 
                tableau[locat,:] = pivot
                oldVar = basis[locat]
                # New variable enters the basis
                basis[locat] = cand
                # Select next candidate for entering the basis
                if oldVar >= dimen:
                    cand = oldVar - dimen
                else:
                    cand = oldVar + dimen
            else:
                rayTerm = True
                break
        np.seterr(divide=oldDivideErr) # restore original handling of zerodivide in Numpy
        # Return solution to LCP
        vars = np.zeros(2*dimen+1)
        vars[basis] = tableau[:,2*dimen+1]
        w = vars[:dimen]
        z = vars[dimen:2*dimen]    
        retcode = vars[2*dimen]
    # end if (q >= 0.).all() 
    
    if rayTerm:
        retcode = (2, retcode, loopcount)  # ray termination
    else:
        retcode = (1, retcode, loopcount)  # success
    return (w, z, retcode)

# In[Create compression matrix]:
def createCompressionLCP(HN,HT,UNA,UTA,mu,M,g_count):
    # create empty A matrix
    A=np.zeros((5*g_count,5*g_count))
    # Combine all the H matrices
    W=np.concatenate((HN,HT,-HT),axis=1)
    # Create first term of A matrix
    W=np.matmul(np.matmul(W.T,np.linalg.inv(M)),W)
    # create empty N matrix
    N=np.zeros((2*g_count,3*g_count))
    N[:g_count,:g_count],N[g_count:2*g_count,:g_count]=mu*np.eye(g_count),mu*np.eye(g_count)
    N[:g_count,g_count:2*g_count],N[g_count:2*g_count,2*g_count:3*g_count]=-np.eye(g_count),-np.eye(g_count)

    # Create empty Is matrix
    IsT=np.zeros((3*g_count,2*g_count))
    IsT[g_count:2*g_count,:g_count],IsT[2*g_count:3*g_count,g_count:2*g_count]=np.eye(g_count),np.eye(g_count)

# put the matrices together
    A=np.concatenate((W,IsT),axis=1)
    Ap=np.concatenate((N,np.zeros((2*g_count,2*g_count))),axis=1)
    A=np.concatenate((A,Ap),axis=0)

    # Create empy B column matrix
    B=np.zeros((5*g_count,1))
    B[:g_count,0],B[g_count:2*g_count,0],B[2*g_count:3*g_count,0]=UNA[:,0],UTA[:,0],-UTA[:,0]


    return A,B



# In[ create expansion matrix]
def createExpansionLCP(HN,HT,UNC,UTC,z,mu,M,en,et,g_count):
    # Create empty A matrix
    A=np.zeros((5*g_count,5*g_count))
    # COmbine the H matrices
    W=np.concatenate((HN,HT,-HT),axis=1)
    # Solve first entry of the A matrix
    W=np.matmul(np.matmul(W.T,np.linalg.inv(M)),W)
    # create empty N matrix
    N=np.zeros((2*g_count,3*g_count))
    N[:g_count,:g_count],N[g_count:2*g_count,:g_count]=mu*np.eye(g_count),mu*np.eye(g_count)
    N[:g_count,g_count:2*g_count],N[g_count:2*g_count,2*g_count:3*g_count]=-np.eye(g_count),-np.eye(g_count)

    # Create empty Is matrix
    IsT=np.zeros((3*g_count,2*g_count))
    IsT[g_count:2*g_count,:g_count],IsT[2*g_count:3*g_count,g_count:2*g_count]=np.eye(g_count),np.eye(g_count)
    #empty zero matrix
    Z=np.zeros((2*g_count,2*g_count))
    #combine A matrices
    A=np.concatenate((W,IsT),axis=1)
    Ap=np.concatenate((N,Z),axis=1)
    A=np.concatenate((A,Ap),axis=0)


    # term for B matrix
    # lambda term
    Lambda=en*z
    # little g matrix
    g=np.concatenate((UNC,UTC,-UTC),axis=0)
    # Empty B matrix
    B=np.zeros((5*g_count,1))
    
    # Define Nbar matrix
    Nbar=N
    # fill in Nbar matrix
    Nbar[:g_count,2*g_count:3*g_count],N[g_count:2*g_count,g_count:2*g_count]=(1-et)*np.eye(g_count),(1-et)*np.eye(g_count)
    # Fill in the B matrix
    B[:3*g_count]=np.matmul(W,Lambda)+g
    
    # do more matrix stuff
    B[3*g_count:5*g_count]=np.matmul(Nbar,Lambda)
    B=B.ravel()
    # Return A and B matrix
    return A,B



# In[Compression matrix and expanison matrix]
    
def LCP_Com_Ex(obj,ind,ndof,vfree,M,mu,en,et,v,qbar):
    H,HN,HT=getH(obj,ind,ndof)
    g_count=len(ind[:,0])

#Calculating normal and transverse gap velocities at A
    UA=np.matmul(H.T,v.reshape(ndof,1))
    UA=np.matmul(H.T,vfree.reshape(ndof,1))

    UNA,UTA=UA[:g_count],UA[g_count:2*g_count]

        #Creating the LCP during compression
    A1,B1=createCompressionLCP(HN,HT,UNA,UTA,mu,M,g_count)
    B1=B1.flatten()
    w,z,retcode = LCPSolve(A1,B1)
    z=z[:3*g_count].reshape(3*g_count,1)

    RNAC=z[:g_count,0]
    RTAC=z[g_count:2*g_count,0]-z[2*g_count:3*g_count,0]
    RAC=np.concatenate((RNAC,RTAC),axis=0).reshape(2*g_count,1)

        #Using the reactions to calculate the velocity at the end of compression
    UC=UA+np.matmul(np.matmul(np.matmul(H.T,np.linalg.inv(M)),H),RAC)
    UNC,UTC=UC[:g_count],UC[g_count:2*g_count]
    A2,B2=createExpansionLCP(HN,HT,UNC,UTC,z,mu,M,en,et,g_count)

#
        #Solving the second LCP to solve for the reactions during expansion
    w,z,retcode = LCPSolve(A2,B2)
    z=z[:3*g_count].reshape(3*g_count,1)


    RNP=z[:g_count,0]
    RTP=z[g_count:2*g_count,0]-z[2*g_count:3*g_count,0]
    RP=np.concatenate((RNP,RTP),axis=0).reshape(2*g_count,1)
    RCE=RP+en*RAC

    #Total reaction due to impulses during compression and expansion
    Rk1=RAC+RCE    
    return(Rk1,H)
    


           
# In[Plot]
def plotObj(obj,xf1,xf2,yf1,yf2):
    ax = plt.gca()

    # change default range so that new circles will work
    ax.set_xlim((xf1, xf2))
    ax.set_ylim((yf1, yf2))

    for i in range (0,len(obj)):
        if(obj[i].type=='particle'):
            circle1 = plt.Circle((obj[i].x,obj[i].y),obj[i].R,color='r')
            ax.add_artist(circle1)
        if(obj[i].type=='robot'):
            circle1 = plt.Circle((obj[i].x,obj[i].y),obj[i].R,color='g')
            ax.add_artist(circle1)
        if(obj[i].type=='lineObstacle'):
            l = mlines.Line2D([obj[i].x1,obj[i].x2], [obj[i].y1,obj[i].y2])
            ax.add_line(l)

        if(obj[i].type=='circleObstacle'):
            circle1 = plt.Circle((obj[i].x,obj[i].y),obj[i].R,color='b')
            ax.add_artist(circle1)

    plt.show()































