import random
import matplotlib.pyplot as plt
import numpy as np
import pychrono as chrono

def generatePolygon( ctrX, ctrY, aveRadius, irregularity, spikeyness, numVerts ) :
    '''Start with the centre of the polygon at ctrX, ctrY, 
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
    irregularity = clip( irregularity, 0,1 ) * 2*np.pi / numVerts
    spikeyness = clip( spikeyness, 0,1 ) * aveRadius
    
    # generate n angle steps
    angleSteps = []
    lower = (2*np.pi / numVerts) - irregularity
    upper = (2*np.pi / numVerts) + irregularity
    sum = 0
    for i in range(numVerts) :
        tmp = random.uniform(lower, upper)
        angleSteps.append( tmp )
        sum = sum + tmp
       
    # normalize the steps so that point 0 and point n+1 are the same
    k = sum / (2*np.pi)
    for i in range(numVerts) :
        angleSteps[i] = angleSteps[i] / k
    
    # now generate the points
    points = []
    angle = random.uniform(0, 2*np.pi)
    for i in range(numVerts) :
        r_i = clip( random.gauss(aveRadius, spikeyness), 0, 2*aveRadius )
        x = ctrX + r_i*np.cos(angle)
        y = ctrY + r_i*np.sin(angle)
        points.append([x,y])

        angle = angle + angleSteps[i]

    return points

def clip(x, min, max) :
     if( min > max ) :  return x    
     elif( x < min ) :  return min
     elif( x > max ) :  return max
     else :             return x
     
verts = generatePolygon( ctrX=0, ctrY=0, aveRadius=10, irregularity=0.8, spikeyness=0.0, numVerts=8 )

vertices = np.asarray(verts)

body_verts=chrono.vector_ChVectorD()
for vert in verts:
    body_verts.push_back(chrono.ChVectorD(vert[0],0,vert[1]))

body = chrono.ChBodyEasyConvexHull(body_verts,1000.0,True,True)

plt.plot(vertices[:,0],vertices[:,1])
plt.show()


        # # create points for convex hull
        # pt_vect = chrono.vector_ChVectorD()
        # # creates bottom
        # for i in range(self.nsides):
        #     pt_vect.push_back(chrono.ChVectorD((self.diameter/2)*np.cos(i*2*np.pi/self.nsides),self.height/2,(self.diameter/2)*np.sin(i*2*np.pi/self.nsides)))
        #     #create top 
        # for i in range(self.nsides):
        #     pt_vect.push_back(chrono.ChVectorD((self.diameter/2)*np.cos(i*2*np.pi/self.nsides),-self.height/2,(self.diameter/2)*np.sin(i*2*np.pi/self.nsides)))
    
        # ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rowr,True,True)   
        # ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
    
        # ball.GetCollisionModel().ClearModel()
        # ball.GetCollisionModel().AddConvexHull(pt_vect)
        # ball.GetCollisionModel().BuildModel()
# now you can save the image (im), or do whatever else you want with it.