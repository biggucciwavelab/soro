# -*- coding: utf-8 -*-
"""
Created on Tue Aug 13 13:16:09 2019

@author: qiyua
"""
# Import libraries
import numpy as np
from bridson import poisson_disc_samples
import math
import matplotlib.pyplot as plt
#import matplotlib.lines as mlines
from matplotlib import animation
import cupy as cp
from numpy import *

import timeit

start_total = timeit.default_timer()

def LCPSolve(M,q, pivtol=1e-8): # pivtol = smallest allowable pivot element
    rayTerm = False
    loopcount = 0
    if (q >= 0.).all(): # Test missing in Rob Dittmar's code
        # As w - Mz = q, if q >= 0 then w = q and z = 0
        w = q
        z = cp.zeros_like(q)
        retcode = 0.
    else:
        dimen = M.shape[0] # number of rows
        # Create initial tableau
        tableau = cp.hstack([cp.eye(dimen), -M, -cp.ones((dimen, 1)), cp.asarray(cp.reshape(q,(q.size,1)))])
        # Let artificial variable enter the basis
        basis = list(range(dimen)) # basis contains a set of COLUMN indices in the tableau
        locat = int(cp.argmin(tableau[:,2*dimen+1])) # row of minimum element in column 2*dimen+1 (last of tableau)
        basis[locat] = int(2*dimen) # replace that choice with the row
        cand = locat + dimen
        pivot = tableau[locat,:]/tableau[locat,2*dimen]
        tableau -= tableau[:,2*dimen:2*dimen+1]*pivot # from each column subtract the column 2*dimen, multiplied by pivot
        tableau[locat,:] = pivot # set all elements of row locat to pivot
        # Perform complementary pivoting
        oldDivideErr = seterr(divide='ignore')['divide'] # suppress warnings or exceptions on zerodivide inside numpy
        while amax(basis) == 2*dimen:
            loopcount += 1
            eMs = tableau[:,cand]    # Note: eMs is a view, not a copy! Do not assign to it...
            missmask = eMs <= 0.
            quots = tableau[:,2*dimen+1] / eMs # sometimes eMs elements are zero, but we suppressed warnings...
            quots[missmask] = Inf # in any event, we set to +Inf elements of quots corresp. to eMs <= 0.
            locat = int(cp.argmin(quots))
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
        seterr(divide=oldDivideErr) # restore original handling of zerodivide in Numpy
        # Return solution to LCP
        vars = cp.zeros(2*dimen+1)
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

#  Create compression matrix
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


# In[]
# create classes for obstacles
class obstacles:
    count=0
    def __init__(self):
        self.index=obstacles.count
        obstacles.count += 1

# Create line
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

#Horizontal line obstacle
class horizontalLine(obstacles):
    def __init__(self,y0):
        obstacles.__init__(self)
        self.y=y0       # postion
        self.type='obstacle'  # tell it what it is
        self.geom='horizontalLine'   # geometry

# Vertical line obstacle
class verticalLine(obstacles):
    def __init__(self,x0):
        obstacles.__init__(self)
        # x intersection
        self.x=x0
        # type of object it is
        self.type='obstacle'
        # specific geometry
        self.geom='verticalLine'


# Create circular obstacle
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

# create balls
class rigidBody2D:

    count=0

    def __init__(self, x, y, phi, xv, yv, phiv, M, I):
        self.index=rigidBody2D.count
        rigidBody2D.count += 1
        self.q=np.array([[x,y,phi]]).T
        self.x,self.y,self.phi=x,y,phi
        self.v=np.array([[xv,yv,phiv]]).T
        self.M=np.zeros((3,3))
        self.M[0,0],self.M[1,1],self.M[2,2]=M,M,I


    def updatePosition(self,q):
        self.q=q
        self.x,self.y,self.phi=q[0,0],q[1,0],q[2,0]

    def updateVelocity(self,v):
        self.v=v

    def getPosition(self):
        return(self.q[0,0],self.q[1,0],self.q[2,0])

class circle(rigidBody2D):
    count=0

    def __init__(self, x, y, phi, xv, yv, phiv, R, rho):
        circle.count += 1
        # tell it its ridged
        self.type='rigidBody2D'
        # geometry is a circle
        self.geom='circle'
        # radius
        self.R=R
        # Tell it its mass
        self.M=rho*np.pi*(self.R)**2
        self.mass=self.M
        # its inertia
        self.I=rho*np.pi*(self.R)**4/2
        rigidBody2D.__init__(self, x, y, phi, xv, yv, phiv, self.M, self.I)

# In[]

#CREATE GLOBAL MATRIX
def createGlobalMatrices(obj):

    #Total number of 2D bodies
    nb=rigidBody2D.count
    # empy M matrix
    M=np.zeros((3*nb,3*nb))
    # empty q matrix this is x, y and phi
    q=np.zeros((3*nb,1))
    #  velocity matrix so vx, vy, phi dot.
    v=np.zeros((3*nb,1))

    # fill the matrices
    for i in range (0,len(obj)):
        # only fill if a ruidged body and not a wall
        if(obj[i].type=='rigidBody2D'):
            ind=obj[i].index
            # Mass matrix
            M[3*ind:3*ind+3,3*ind:3*ind+3]=obj[i].M
            # create position matrix
            q[3*ind:3*ind+3,:]=obj[i].q
            # create velocity matrix
            v[3*ind:3*ind+3,:]=obj[i].v
    return (M,q,v)
# In[]
#UPDATE COORDINATES
def updateCoordinates(obj,qb,vb):
    for i in range (0,len(obj)):
        # classify if object is a ridged body
        if(obj[i].type=='rigidBody2D'):

            ind=obj[i].index

            # update position
            obj[i].updatePosition(qb[3*ind:3*ind+3].reshape(3,1))
            # update velocities
            obj[i].updateVelocity(vb[3*ind:3*ind+3].reshape(3,1))

# In[]
#DISTANCE BETWEEN TWO BALLS #(rD) TERM
def dist2D(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2+(y1-y2)**2)


# In[] FIND GAP FUNCTION
def calculateGaps(obj):
    # empty gap matrix
    g=np.zeros((len(obj),len(obj)))
    # empty indices matrix
    ind=[]

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
    # If geometry is a circle and a circle obstacle
            if ((obj[i].geom=='circle' and obj[j].geom=='circleObstacle') or (obj[j].geom=='circleObstacle' and obj[i].geom=='circle')):
                g[i,j]=dist2D(obj[i].x,obj[i].y,obj[j].x,obj[j].y)-(obj[i].R+obj[j].R)
    #If geometry is a circle and an arbitray line
            if ((obj[i].geom=='circle' and obj[j].geom=='lineObstacle')):
                g[i,j]=dist2D(0,obj[i].y,0,obj[j].y1)-obj[i].R

            if ((obj[i].geom=='lineObstacle' and obj[j].geom=='circle')):
                g[i,j]=dist2D(0,obj[i].y,0,obj[j].y1)-obj[j].R

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

# In[]

# h MATRIX LOCAL matrix
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
# In[]
    # Big H so the global H matrix
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
# In[]
def rn():
    return np.random.uniform(-1.,1.)

# In[Create gravity Fg]
def gravity(obj,q0):
    Fg=np.zeros_like(q0)
    ndof=np.size(q0)
    count=int(ndof/3)

    for i in range(count):
        Fg[3*i+1,0]=-9.8*obj[i].mass

    return Fg

# In[Create External forces]
def Forces(obj,q0,nb,ni,t,type):
    F=np.zeros_like(q0)
    ndof=np.size(q0)
    count=int(ndof/3)

    ##### Force case 1
    if type==1:
        for i in range(nb):
            if t<500:
                F[3*i,0]=20*np.sin(t)
            else:
                F[3*i,0]=-20
#### Force type 2
    elif type==2:

        for i in range(nb):
            F[3*i,0]=20
    elif type==3:
        for i in range(nb):
            F[3*i,0]=0

# Force type 3

    elif type==3:
        for i in range(5*nb/4,5*nb/4+4):
            F[3*i+1,0]=-80

    return F
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
    for i in range(nb-1):
        d[i,0]=np.sqrt((theta[3*i,0]-theta[3*i+3,0])**2+(theta[3*i+1,0]-theta[3*i+4,0])**2)
    i=nb-1
    d[i,0]=np.sqrt((theta[3*i,0]-theta[0,0])**2+(theta[3*i+1,0]-theta[1,0])**2)


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

    return K

# In[Running the code]:

#Initializing objects
obj=[]

#Creating the bounding box with left, right, up, and down coordinates
left,right,up,down=-20,30,20,-30

# create internal and external boundary
nb=50               #number of boundary robots
R=nb/(2*3.14)     #radius of robot

#x,y,phi,\dot{x},\dot{y},\dot{phi},R,rho

for i in range(nb):
    x=R*np.cos(i*2*np.pi/nb)*.9+R/2
    y=R*np.sin(i*2*np.pi/nb)*.9+R/2


    obj.append(circle(x,y,0,0,0,0,.3*(nb/nb),1))

#for i in range(ni):
#    obj.append(circle(R*rn(),R*rn(),0.,rn(),rn(),rn(),.5*np.random.random_sample()+.5,1.))
#x, y, phi, xv, yv, phiv, self.M, self.I)

def poisson_distribution(obj):
    rLim=.3
    nb=40
    R=nb/(2*3.14)
    t=np.array(poisson_disc_samples(width=2.*R, height=2.*R, r=rLim*1.65))
    for i in range(0,np.size(t[:,0])):

        R=rLim*(np.random.rand()+.5)

        #x,y,phi,\dot{x},\dot{y},\dot{phi},R,rho

        obj.append(circle(t[i,0],t[i,1],0,rn(),rn(),rn(),R,.5))

    return obj

# Create the interior particles
poisson_distribution(obj)

# Count the number of interior objects
ni=len(obj)-nb

# add the boundaries
obj.append(verticalLine(left))
obj.append(verticalLine(right))
obj.append(horizontalLine(up))
obj.append(horizontalLine(down))

for i in range(10):

    obj.append(circleObstacle((i)-5,2-i,1))
    obj.append(circleObstacle(17-(i),-4-i,1))
    obj.append(circleObstacle((i)-4,-10-i,1))

# In[]
#Creating Mass, q, and v matrices
M,q0,v0=createGlobalMatrices(obj)
# degrees of freedom
ndof=np.size(q0)

#Gravity turns on and off gravity
# create empty matrix
g=1
if g==1:
    Fg=gravity(obj,q0)
else:
    Fg=np.zeros_like(q0)

#Time parameters
t0,tend=0.,.5
nsteps=int(100.*tend)
time=np.linspace(t0,tend,100*tend)

# time step size
h=time[1]-time[0]

#position and velocities. Making the empty matrices
q,v=np.zeros((q0.size,time.size)),np.zeros((v0.size,time.size))

#Initial conditions
q[:,0],v[:,0]=q0.reshape(q0.size,),v0.reshape(v0.size,)

#Integration parameters
t_imp,gamma=1.,.5

#Impact and Friction properties
en,et,mu=.8,.2,.5


# In[] Solving the system
for i in range(time.size-1):
    ####Predicting gap
    #P redicted q
    qbar=q[:,i]+h*gamma*v[:,i]
    K=modelStiffness(qbar,nb,ni,0.,200.)
    F=Forces(obj,q0,nb,ni,i,3)
    # Create external forces
    Fext=-np.matmul(K,qbar)+Fg.flatten()+F.flatten()

    #Predicted gap. gbar is a matrix of all gaps between all bodies and ind is the index list of those bodies for which gbar<0
    updateCoordinates(obj,qbar,v[:,i])
    gbar,ind=calculateGaps(obj)
    updateCoordinates(obj,q[:,i],v[:,i])

    #Calculating vfree
    vfree=v[:,i]+h*np.matmul(np.linalg.inv(M),Fext)

    if ind.size<1:
        v[:,i+1]=vfree
    else:
######################Using Lemke's

        H,HN,HT=getH(obj,ind,ndof)
        g_count=len(ind[:,0])

        #Calculating normal and transverse gap velocities at A
        UA=np.matmul(H.T,v[:,i].reshape(ndof,1))
        UA=np.matmul(H.T,vfree.reshape(ndof,1))

        UNA,UTA=UA[:g_count],UA[g_count:2*g_count]

        #Creating the LCP during compression

        A1,B1=createCompressionLCP(HN,HT,UNA,UTA,mu,M,g_count)
        B1=B1.flatten()
        start = timeit.default_timer()
        w,z,retcode = LCPSolve(A1,B1)
        w=cp.asnumpy(w)
        z=cp.asnumpy(z)
        z=z[:3*g_count].reshape(3*g_count,1)

        stop = timeit.default_timer()
        print('Time: ', stop - start)


        RNAC=z[:g_count,0]
        RTAC=z[g_count:2*g_count,0]-z[2*g_count:3*g_count,0]
        RAC=np.concatenate((RNAC,RTAC),axis=0).reshape(2*g_count,1)

        #Using the reactions to calculate the velocity at the end of compression
        UC=UA+np.matmul(np.matmul(np.matmul(H.T,np.linalg.inv(M)),H),RAC)
        UNC,UTC=UC[:g_count],UC[g_count:2*g_count]
        A2,B2=createExpansionLCP(HN,HT,UNC,UTC,z,mu,M,en,et,g_count)

#
        #Solving the second LCP to solve for the reactions during expansion
        start1 = timeit.default_timer()
        w,z,retcode = LCPSolve(A2,B2)
        w=cp.asnumpy(w)
        z=cp.asnumpy(z)
        z=z[:3*g_count].reshape(3*g_count,1)
        stop1 = timeit.default_timer()
        print('Time1: ', stop1 - start1)

        RNP=z[:g_count,0]
        RTP=z[g_count:2*g_count,0]-z[2*g_count:3*g_count,0]
        RP=np.concatenate((RNP,RTP),axis=0).reshape(2*g_count,1)
        RCE=RP+en*RAC

        #Total reaction due to impulses during compression and expansion
        Rk1=RAC+RCE

        #Velocity at the end of the time step
        v[:,i+1]=vfree+(np.matmul(np.matmul(np.linalg.inv(M),H),Rk1)).reshape(ndof,)


    #     For implicit integration of q
    vkt=h*(t_imp*v[:,i+1]+(1-t_imp)*v[:,i])

    #Implicit integration of q
    q[:,i+1]=q[:,i]+vkt



##Plot##
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(10, 10)
#xlimits
#Creating border

bx=[left,right,right,left,left]
by=[down,down,up,up,down]
ax = plt.axes(xlim=(left-1., right+1.), ylim=(down-1., up+1.))
lineH = plt.Line2D(bx, by, lw=1)

#lineOb = plt.Line2D([0,8],[0,-8],lw=1)


#Plotting the circles
cPatches,cLines,cInd,Obst=[],[],[],[]
for i in range (0,len(obj)):
    if(obj[i].geom=='circle'):
        ii=obj[i].index
        if i<nb:
            x0,y0,phi0,r=q[3*ii,0],q[3*ii+1,0],q[3*ii+2,0],obj[i].R
            patch = plt.Circle((x0, y0), r, fc='g')

        else:
            x0,y0,phi0,r=q[3*ii,0],q[3*ii+1,0],q[3*ii+2,0],obj[i].R
            patch = plt.Circle((x0, y0), r, fc='r')



        lineC1 = plt.Line2D((x0+r*np.cos(phi0+np.pi), x0+r*np.cos(phi0)), (y0+r*np.sin(phi0+np.pi), y0+r*np.sin(phi0)),color='k')
        lineC2 = plt.Line2D((x0+r*np.cos(phi0+np.pi+np.pi/2), x0+r*np.cos(phi0+np.pi+np.pi/2)), (y0+r*np.sin(phi0+np.pi+np.pi+np.pi/2), y0+r*np.sin(phi0+np.pi+np.pi/2)), color='k')
        cPatches.append(patch)
        cLines.append(lineC1)
        cLines.append(lineC2)
        cInd.append(ii)
    elif(obj[i].geom=='circleObstacle'):

        x0,y0,r=obj[i].x,obj[i].y,obj[i].R
        patch=plt.Circle((x0,y0),r,fc='b')

        Obst.append(patch)



# initialization function: plot the background of each frame
def init():
    ax.add_line(lineH)
    #ax.add_line(lineOb)


    for i in range (0,len(cPatches)):
        ax.add_patch(cPatches[i])
        ax.add_line(cLines[2*i])
        ax.add_line(cLines[2*i+1])
    for i in range(0,len(Obst)):
        ax.add_patch(Obst[i])
    return []

def animationManage(i,cPatches,cLines):
    animatePatches(i,cPatches,cLines)
    animateLine(i,cPatches,cLines)
    return []


def animateLine(i, cPatches, cLines):
    for j in range (0,len(cPatches)):
        x,y,phi,r=q[3*cInd[j],i],q[3*cInd[j]+1,i],q[3*cInd[j]+2,i],obj[cInd[j]].R
        cLines[2*j].set_xdata([x+r*np.cos(phi+np.pi), x+r*np.cos(phi)])
        cLines[2*j].set_ydata([y+r*np.sin(phi+np.pi), y+r*np.sin(phi)])
        cLines[2*j+1].set_xdata([x+r*np.cos(phi+np.pi+np.pi/2), x+r*np.cos(phi+np.pi/2)])
        cLines[2*j+1].set_ydata([y+r*np.sin(phi+np.pi+np.pi/2), y+r*np.sin(phi+np.pi/2)])
    return cLines,


def animatePatches(i,cPatches,cLines):
    for j in range (0,len(cPatches)):
        x,y,phi,r=q[3*cInd[j],i],q[3*cInd[j]+1,i],q[3*cInd[j]+2,i],obj[cInd[j]].R
        cPatches[j].center=(x, y)

    return cPatches,


anim = animation.FuncAnimation(fig, animationManage,
                               init_func=init,
                               frames=nsteps/2,
                               fargs=(cPatches,cLines,),
                               interval=2,
                               blit=True,
                               repeat=True)

# call the animator.  blit=True means only re-draw the parts that have changed.
#anim = animation.FuncAnimation(fig, animationManage, init_func=init,
#                               frames=nsteps, interval=1, blit=True)


#anim = animation.FuncAnimation(fig, animate, init_func=init,
#                               frames=nsteps, fargs=(q,cPatches, lineH, cLines, cInd), interval=1, blit=True)

plt.show()
np.savez('temp10.npz',q=q,nb=nb,ni=ni,obj=obj,left=left,right=right,up=up,down=down,R=R,t0=t0,tend=tend,nsteps=nsteps)

stop_total = timeit.default_timer()
print('Total Run Time: ', stop_total - start_total)
