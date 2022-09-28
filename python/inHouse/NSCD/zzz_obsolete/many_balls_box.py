# -*- coding: utf-8 -*-
"""
Created on Tue May 21 21:49:37 2019

@author: dmulr
"""
# In[]
import numpy as np
#from bridson import poisson_disc_samples
import math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib import animation
from openopt import LCP
import sys
import timeit

start = timeit.default_timer()

#supress output
class NullWriter(object):
    def write(self,arg):
        pass

# In[]c
def proj_Jacobi(M,q):
    z=np.random.rand(np.size(q[:,0]),1)
#     z=np.abs(q)
    B=np.identity(np.size(q))
    C=M-B
    tol,e=1E-6,1.

    while e>tol:
        z1=z
        qn=np.matmul(C,z)+q
        z=(-qn+np.abs(qn))/2
        e=np.linalg.norm(z-z1)/np.linalg.norm(z)
        print(e)

    return z


# LCP solver create
class lemketableau:
    def __init__(self,M,q,maxIter = 10000):
        n = len(q)
        self.T = np.hstack((np.eye(n),-M,-np.ones((n,1)),q.reshape((n,1))))
        self.n = n
        self.wPos = list(range(n))
        self.zPos = list(range(n,2*n))
        self.W = 0
        self.Z = 1
        self.Y = 2
        self.Q = 3
        TbInd = np.vstack((self.W*np.ones(n,dtype=int),
                           np.arange(n,dtype=int)))
        TnbInd = np.vstack((self.Z*np.ones(n,dtype=int),
                            np.arange(n,dtype=int)))
        DriveInd = np.array([[self.Y],[0]])
        QInd = np.array([[self.Q],[0]])
        self.Tind = np.hstack((TbInd,TnbInd,DriveInd,QInd))
        self.maxIter = maxIter


    def lemkeAlgorithm(self):
        initVal = self.initialize()
        if not initVal:
            return np.zeros(self.n),0,'Solution Found'

        for k in range(self.maxIter):
            stepVal = self.step()
#             print(stepVal)
            if self.Tind[0,-2] == self.Y:
                # Solution Found
                z = self.extractSolution()
                return z,0,'Solution Found'
            elif not stepVal:
                return None,1,'Secondary ray found'

        return None,2,'Max Iterations Exceeded'

    def initialize(self):
        q = self.T[:,-1]
        minQ = np.min(q)
        if minQ < 0:
            ind = np.argmin(q)
            self.clearDriverColumn(ind)
            self.pivot(ind)

            return True
        else:
            return False

    def step(self):
        q = self.T[:,-1]
        a = self.T[:,-2]
        ind = np.nan
        minRatio = np.inf
        for i in range(self.n):
            if a[i] > 0:
                newRatio = q[i] / a[i]
                if newRatio < minRatio:
                    ind = i
                    minRatio = newRatio

        if minRatio < np.inf:
            self.clearDriverColumn(ind)
            self.pivot(ind)
            return True
        else:
            return False

    def extractSolution(self):
        z = np.zeros(self.n)
        q = self.T[:,-1]
        for i in range(self.n):
            if self.Tind[0,i] == self.Z:
                z[self.Tind[1,i]] = q[i]
        return z
    def partnerPos(self,pos):
        v,ind = self.Tind[:,pos]
        if v == self.W:
            ppos = self.zPos[ind]
        elif v == self.Z:
            ppos = self.wPos[ind]
        else:
            ppos = None
        return ppos

    def pivot(self,pos):
        ppos = self.partnerPos(pos)
        if ppos is not None:
            self.swapColumns(pos,ppos)
            self.swapColumns(pos,-2)
            return True
        else:
            self.swapColumns(pos,-2)
            return False



    def swapMatColumns(self,M,i,j):
        Mi = np.array(M[:,i],copy=True)
        Mj = np.array(M[:,j],copy=True)
        M[:,i] = Mj
        M[:,j] = Mi
        return M

    def swapPos(self,v,ind,newPos):
        if v == self.W:
            self.wPos[ind] = newPos % (2*self.n+2)
        elif v == self.Z:
            self.zPos[ind] = newPos % (2*self.n+2)


    def swapColumns(self,i,j):
        iInd = self.Tind[:,i]
        jInd = self.Tind[:,j]

        v,ind = iInd
        self.swapPos(v,ind,j)
        v,ind = jInd
        self.swapPos(v,ind,i)

        self.Tind = self.swapMatColumns(self.Tind,i,j)
        self.T = self.swapMatColumns(self.T,i,j)


    def clearDriverColumn(self,ind):
        a = self.T[ind,-2]
        self.T[ind] /= a
        for i in range(self.n):
            if i != ind:
                b = self.T[i,-2]
                self.T[i] -= b * self.T[ind]


    def ind2str(self,indvec):
        v,pos = indvec
        if v == self.W:
            s = 'w%d' % pos
        elif v == self.Z:
            s = 'z%d' % pos
        elif v == self.Y:
            s = 'y'
        else:
            s = 'q'
        return s

    def indexStringArray(self):
        indstr = np.array([self.ind2str(indvec) for indvec in self.Tind.T],dtype=object)
        return indstr

    def indexedTableau(self):
        indstr = self.indexStringArray()
        return np.vstack((indstr,self.T))
    def __repr__(self):
        IT = self.indexedTableau()
        return IT.__repr__()
    def __str__(self):
        IT = self.indexedTableau()
        return IT.__str__()

def lemkelcp(M,q,maxIter=1000):
    """
    sol = lemkelcp(M,q,maxIter)
    Uses Lemke's algorithm to copute a solution to the
    linear complementarity problem:
    Mz + q >= 0
    z >= 0
    z'(Mz+q) = 0
    The inputs are given by:
    M - an nxn numpy array
    q - a length n numpy array
    maxIter - an optional number of pivot iterations. Set to 100 by default
    The solution is a tuple of the form:
    z,exit_code,exit_string = sol
    The entries are summaries in the table below:

    |z                | exit_code | exit_string               |
    -----------------------------------------------------------
    | solution to LCP |    0      | 'Solution Found'          |
    | None            |    1      | 'Secondary ray found'     |
    | None            |    2      | 'Max Iterations Exceeded' |
    """

    tableau = lemketableau(M,q,maxIter)
    return tableau.lemkeAlgorithm()


# In[4]:

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


#In[] create expansion matrix
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
    #empty B matrix
    Lambda=en*z
    g=np.concatenate((UNC,UTC,-UTC),axis=0)

    B=np.zeros((5*g_count,1))
    Nbar=N
    Nbar[:g_count,2*g_count:3*g_count],N[g_count:2*g_count,g_count:2*g_count]=(1-et)*np.eye(g_count),(1-et)*np.eye(g_count)
    B[:3*g_count]=np.matmul(W,Lambda)+g
    B[3*g_count:5*g_count]=np.matmul(Nbar,Lambda)
    B=B.ravel()

    return A,B


# In[]
# create classes
class obstacles:
    count=0
    def __init__(self):
        self.index=obstacles.count
        obstacles.count += 1


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
            M[3*ind:3*ind+3,3*ind:3*ind+3]=obj[i].M
            q[3*ind:3*ind+3,:]=obj[i].q
            v[3*ind:3*ind+3,:]=obj[i].v
    return (M,q,v)
# In[]
#UPDATE COORDINATES
def updateCoordinates(obj,qb,vb):
    for i in range (0,len(obj)):
        if(obj[i].type=='rigidBody2D'):
            ind=obj[i].index
            obj[i].updatePosition(qb[3*ind:3*ind+3].reshape(3,1))
            obj[i].updateVelocity(vb[3*ind:3*ind+3].reshape(3,1))

# In[]
#DISTANCE BETWEEN TWO BALLS #(rD) TERM
def dist2D(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2+(y1-y2)**2)


# In[] FIND GAP FUNCTION
def calculateGaps(obj):
    g=np.zeros((len(obj),len(obj)))
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

# H MATRIX LOCAL
def geth(theta,phi,a,b):
    A=np.array([[-np.sin(phi), -np.cos(phi)],[np.cos(phi),-np.sin(phi)]])
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
        # declar indices

        i=ind[g,0]

        j=ind[g,1]

        # if interaction is with horizontal wall and circle
        if((obj[i].geom=='circle' and obj[j].geom=='horizontalLine') or (obj[j].geom=='circle' and obj[i].geom=='horizontalLine')):

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

    H=np.concatenate((HN,HT),axis=1)

    return (H,HN,HT)



# In[]:

#Initializing objects
obj=[]

#set size of bounding box
box=18.
#set size of random ball generation
diam=.15

#Creating 4 balls of radius 1 and density 1. Initialized to phi=0 and R1,R2 given by x,y. x,y,phi,\dot{x},\dot{y},\dot{phi},R,rho
#obj.append(circle(2.,5.,0.,-3.,2.,0.,1.,1.))
#obj.append(circle(5.,5.0,0.,5.,0.,0.,1.,1.))
#obj.append(circle(8.,5.,0.,0.,0.,4.,1.,1.))
#obj.append(circle(11.,5.,0.,3.,5.,-2.,1.,1.))
#obj.append(circle(3.,7.,0.,1.,2.,-2.,1.,1.))
#obj.append(circle(6.,7.,0.,1.,2.,2.,1.,1.))
#obj.append(circle(9.,8.,0.,1.,2.,-2.,1.,1.))
#obj.append(circle(12.,9.,0.,1.,2.,-2.,1.,1.))

#
#obj.append(circle(2.,5.,0.,-3.,2.,0.,.5*np.random.random_sample()+.5,1.))
#obj.append(circle(5.,5.0,0.,5.,0.,0.,.5*np.random.random_sample()+.5,1.))
#obj.append(circle(8.,5.,0.,0.,0.,4.,.5*np.random.random_sample()+.5,1.))
#obj.append(circle(11.,5.,0.,3.,5.,-2.,.5*np.random.random_sample()+.5,1.))
#obj.append(circle(3.,7.,0.,1.,2.,-2.,.5*np.random.random_sample()+.5,1.))
#obj.append(circle(6.,7.,0.,1.,2.,2.,.5*np.random.random_sample()+.5,1.))
#obj.append(circle(9.,8.,0.,1.,2.,-2.,.5*np.random.random_sample()+.5,1.))
#obj.append(circle(12.,9.,0.,1.,2.,-2.,.5*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))
obj.append(circle(box*np.random.random_sample(),box*np.random.random_sample(),0.,1.,2.,-2.,diam*np.random.random_sample()+.5,1.))





#Creating the bounding box with left, right, up, and down coordinates
left,right,up,down=0.,20.,20.,0.
obj.append(horizontalLine(up))
obj.append(horizontalLine(down))
obj.append(verticalLine(left))
obj.append(verticalLine(right))




# In[]
#Creating Mass, q, and v matrices
M,q0,v0=createGlobalMatrices(obj)
# degrees of freedom
ndof=np.size(q0)

#Gravity
# create empty matrix
Fg=np.zeros_like(q0)
# entry for four loop so it knows how many times to go
count=ndof/3
# convert to integer. It was coming out as a float so needed to be converted
count=int(count)
# fill in empty Fg matrix
for i in range(count):
    Fg[3*i+1,0]=-9.8 #assuming all masses=1

#Time parameters
t0,tend,nsteps=0.,15.,750
time=np.linspace(t0,tend,num=nsteps)

# time step size
h=time[1]-time[0]

#position and velocities. Making the empty matrices
q,v=np.zeros((q0.size,time.size)),np.zeros((v0.size,time.size))

#Initial conditions
q[:,0],v[:,0]=q0.reshape(q0.size,),v0.reshape(v0.size,)

#Integration parameters
t_imp,gamma=1.,.5

#Impact and Friction properties
en,et,mu=.9,5.,.4

# In[] Solving the system
for i in range(time.size-1):
    ####Predicting gap
    #P redicted q
    qbar=q[:,i]+h*gamma*v[:,i]

    #Predicted gap. gbar is a matrix of all gaps between all bodies and ind is the index list of those bodies for which gbar<0
    updateCoordinates(obj,qbar,v[:,i])
    gbar,ind=calculateGaps(obj)
    updateCoordinates(obj,q[:,i],v[:,i])

    #Calculating vfree
    vfree=v[:,i]+h*np.matmul(np.linalg.inv(M),Fg.reshape(ndof,))

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
        nullwrite=NullWriter()
        oldstdout = sys.stdout
        sys.stdout=nullwrite
        #Creating the LCP during compression

        A1,B1=createCompressionLCP(HN,HT,UNA,UTA,mu,M,g_count)
        B1=B1.flatten()
        sol = LCP(A1,B1)
        r=sol.solve('lcpsolve')
        f_opt, x_opt = r.ff, r.xf
        z, w = x_opt[x_opt.size/2:], x_opt[:x_opt.size/2]
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
        sol = LCP(A2,B2)
        r=sol.solve('lcpsolve')
        f_opt, x_opt = r.ff, r.xf
        z, w = x_opt[x_opt.size/2:], x_opt[:x_opt.size/2]
        z=z[:3*g_count].reshape(3*g_count,1)


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



fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 7)
#xlimits
#Creating border
bx=[left,right,right,left,left]
by=[down,down,up,up,down]
ax = plt.axes(xlim=(left-1., right+1.), ylim=(down-1., up+1.))
lineH = plt.Line2D(bx, by, lw=1)


#Plotting the circles
cPatches,cLines,cInd=[],[],[]
for i in range (0,len(obj)):
    if(obj[i].geom=='circle'):
        ii=obj[i].index
        x0,y0,phi0,r=q[3*ii,0],q[3*ii+1,0],q[3*ii+2,0],obj[i].R
        patch = plt.Circle((x0, y0), r, fc='y')
        lineC1 = plt.Line2D((x0+r*np.cos(phi0+np.pi), x0+r*np.cos(phi0)), (y0+r*np.sin(phi0+np.pi), y0+r*np.sin(phi0)), lw=1)
        lineC2 = plt.Line2D((x0+r*np.cos(phi0+np.pi+np.pi/2), x0+r*np.cos(phi0+np.pi+np.pi/2)), (y0+r*np.sin(phi0+np.pi+np.pi+np.pi/2), y0+r*np.sin(phi0+np.pi+np.pi/2)), lw=1)
        cPatches.append(patch)
        cLines.append(lineC1)
        cLines.append(lineC2)
        cInd.append(ii)

# initialization function: plot the background of each frame
def init():
    ax.add_line(lineH)
    for i in range (0,len(cPatches)):
        ax.add_patch(cPatches[i])
        ax.add_line(cLines[2*i])
        ax.add_line(cLines[2*i+1])

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
                               frames=nsteps,
                               fargs=(cPatches,cLines,),
                               interval=1,
                               blit=True,
                               repeat=True)

# call the animator.  blit=True means only re-draw the parts that have changed.
#anim = animation.FuncAnimation(fig, animationManage, init_func=init,
#                               frames=nsteps, interval=1, blit=True)


#anim = animation.FuncAnimation(fig, animate, init_func=init,
#                               frames=nsteps, fargs=(q,cPatches, lineH, cLines, cInd), interval=1, blit=True)
sys.stdout=oldstdout
stop=timeit.default_timer()
print('Time: ', stop - start)
plt.show()
