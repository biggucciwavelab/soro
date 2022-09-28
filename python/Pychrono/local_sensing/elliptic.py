import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy import interpolate
from scipy.spatial import KDTree

def clip( x, mini, maxi) :
    if( mini > maxi ) :  return x    
    elif( x < mini ) :  return mini
    elif( x > maxi ) :  return maxi
    else :             return x
    
def generatePolygon( ctrX, ctrY, aveRadius, irregularity, spikeyness, numVerts) :
    # Generate a random polygon
    '''
    Modified from: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon
    Start with the centre of the polygon at ctrX, ctrY, 
    then creates the polygon by sampling points on a circle around the centre. 
    Randon noise is added by varying the angular spacing between sequential points,
    and by varying the radial distance of each point from the centre.
    
    Params:
    ctrX, ctrY - coordinates of the "centre" of the polygon
    aveRadius - in px, the average radius of this polygon, this roughly controls how large the polygon is, really only useful for order of magnitude.
    irregularity - [0,1] indicating how much variance there is in the angular spacing of vertices. [0,1] will map to [0, 2pi/numberOfVerts]
    spikeyness - [0,1] indicating how much variance there is in each vertex from the circle of radius aveRadius. [0,1] will map to [0, aveRadius]
    numVerts - self-explanatory
    
    Returns a list of vertices, in CCW order.
       '''
    irregularity = clip( irregularity, 0,1 ) * 2.*np.pi / numVerts
    spikeyness = clip( spikeyness, 0,1 ) * aveRadius
    
    # generate n angle steps
    angleSteps = []
    lower = (2.*np.pi / numVerts) - irregularity
    upper = (2.*np.pi / numVerts) + irregularity
    sum = 0
    for i in range(numVerts) :
        tmp = np.random.uniform(lower, upper)
        angleSteps.append( tmp )
        sum = sum + tmp
       
    # normalize the steps so that point 0 and point n+1 are the same
    k = sum / (2.*np.pi)
    for i in range(numVerts) :
        angleSteps[i] = angleSteps[i] / k
    
    # now generate the points
    xs=[]
    ys=[]
    
    angle = np.random.uniform(0, 2.*np.pi)
    for i in range(numVerts) :
        r_i = clip( np.random.normal(aveRadius, spikeyness), 0, 2.*aveRadius )
        x = ctrX + r_i*np.cos(angle)
        y = ctrY + r_i*np.sin(angle)
        angle = angle + angleSteps[i]
        xs.append(x)
        ys.append(y)
    return xs,ys
 
class elliptic_fourier:
    def __init__(self,contour,order,npoints):
        self.order=order
        self.contour=np.asarray(contour)
        if np.all(contour[0,:]!=contour[-1,:]):
            self.contour = np.vstack((self.contour,self.contour[0,:]))
        self.npoints = npoints
        self.n = 360
        
        self.dxy = np.diff(self.contour, axis=0)
        self.dt = np.sqrt((self.dxy ** 2).sum(axis=1))
        self.t = np.concatenate([([0.0]), np.cumsum(self.dt)])
        self.T = self.t[-1]
        self.phi = (2 * np.pi * self.t) / self.T
        
        self.orders = np.arange(1, self.order + 1)
        
        self.consts = self.T / (2 * self.orders * self.orders * np.pi * np.pi)
        self.phi = self.phi * self.orders.reshape((self.order, -1))
   
        self.d_cos_phi = np.cos(self.phi[:, 1:]) - np.cos(self.phi[:, :-1])
        self.d_sin_phi = np.sin(self.phi[:, 1:]) - np.sin(self.phi[:, :-1])
        
        self.cos_phi = (self.dxy[:, 0] / self.dt) * self.d_cos_phi
        
        self.a = self.consts * np.sum(self.cos_phi, axis=1)
        self.b = self.consts * np.sum((self.dxy[:, 0] / self.dt) * self.d_sin_phi, axis=1)
        self.c = self.consts * np.sum((self.dxy[:, 1] / self.dt) * self.d_cos_phi, axis=1)
        self.d = self.consts * np.sum((self.dxy[:, 1] / self.dt) * self.d_sin_phi, axis=1)
       
        self.coeffs = np.concatenate([self.a.reshape((self.order, 1)),
                                      self.b.reshape((self.order, 1)),
                                      self.c.reshape((self.order, 1)),
                                      self.d.reshape((self.order, 1)),],axis=1,)
        
        self.xi =    np.cumsum(self.dxy[:, 0]) - (self.dxy[:, 0] / self.dt) * self.t[1:]
        self.delta = np.cumsum(self.dxy[:, 1]) - (self.dxy[:, 1] / self.dt) * self.t[1:]
        
        self.A0 = (1 / self.T) * np.sum(((self.dxy[:, 0] / (2 * self.dt)) * np.diff(self.t ** 2)) + self.xi * self.dt)
        self.C0 = (1 / self.T) * np.sum(((self.dxy[:, 1] / (2 * self.dt)) * np.diff(self.t ** 2)) + self.delta * self.dt)
   
        # Cubic interpolation things
        tck, u = interpolate.splprep([self.contour[:,0], self.contour[:,1]], s=0, per=True)
        self.xi, self.yi = interpolate.splev(np.linspace(0, 1, self.n), tck)

    def plot_elliptic(self):
        # xy
        color=iter(cm.rainbow(np.linspace(0,1,(self.order))))
        self.t = np.linspace(0, 1.0, self.n)
        self.xt = np.ones((self.n,))*(self.A0+self.contour[0,0])
        self.yt = np.ones((self.n,))*(self.C0+self.contour[0,1])
        
        errors = np.zeros((self.order,len(self.contour)-1))
        
        plt.figure(figsize=(8,8))
        plt.plot(self.contour[:,0],self.contour[:,1],linewidth=3.0,c='black',label='contour')
        plt.plot(self.xi,self.yi,'r:',label='Cubic Interp')
        
        for n in range(self.coeffs.shape[0]):
            c=next(color)
            self.xt += (self.coeffs[n, 0] * np.cos(2 * (n + 1) * np.pi * self.t)) + \
                  (self.coeffs[n, 1] * np.sin(2 * (n + 1) * np.pi * self.t))
                  
            self.yt += (self.coeffs[n, 2] * np.cos(2 * (n + 1) * np.pi * self.t)) + \
                  (self.coeffs[n, 3] * np.sin(2 * (n + 1) * np.pi * self.t))
                  
            points = np.asarray([self.xt,self.yt]).T
            points = KDTree(points)
            
            dd,ii = points.query(self.contour[0:-1,:])
            errors[n,:] = dd
                  
            width = 2.0 * (n+1)/self.coeffs.shape[0]
            if n%5==0: plt.plot(self.xt, self.yt, c=c,linewidth=width,label='order= '+str(n))  
            
            plt.grid(True)
            plt.xlabel('x position')
            plt.ylabel('y position')
            plt.legend()
            
            self.errors=errors
    
npoints = 50
x,y = generatePolygon(0.0, 0.0, 1, 0.0, 0.0, npoints)

contour=np.zeros((len(x),2))
contour[:,0]=x
contour[:,1]=y
contour=np.asarray(contour)
contour[:,0]+=1.0

order=10

phi=elliptic_fourier(contour,order,npoints)
phi.plot_elliptic()

plt.figure()
plt.plot(np.linspace(0,order,order),np.mean(phi.errors,axis=1))
plt.title('Average interpolation error')
plt.xlabel('Elliptic fourier order')
plt.ylabel('Average error [m]')
plt.show()
