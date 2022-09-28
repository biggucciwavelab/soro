import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import cm 
from matplotlib import colors as colors
import animatplot as amp
from mpl_toolkits.mplot3d import Axes3D
import timeit
from scipy.spatial import distance
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import scipy.constants 
import csv

# In[Robot object]
class robot:
    def __init__(self,nb,diameter,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,mag,R):
        # number of interior
        self.nb=nb 
        # diameter
        self.diameter=diameter
        #height
        self.height=height
        # density
        self.rowr=rowr
        # surface properties
        self.material=material
        # floor
        self.body_floor=body_floor
        # radius of ring
        self.R=R
        # robot position
        self.xb={}
        self.yb={}
        self.zb={}
        # velocity position
        self.xvb={}
        self.yvb={}
        self.zvb={}
        self.phib={}
        # total force
        self.Ftxb={}
        self.Ftyb={}
        self.Ftzb={}
        self.Spring_force={}
        self.spring_length={}
        self.Psi={} # heading angle
        # empty object arrays
        self.my_system=my_system
        self.bots=[]
        self.Springs=[]
        self.obj=obj
        self.force=[]
        # is object fixed
        self.fixed=fixed
        self.mag=mag
        # Geometry of robots
        self.geom="square"
        
        '''
        # geometry of robots
        square: robot is cube  
        circle: robot is cylinder
        '''
        
        self.XL=[]
        self.ZL=[]
        

        # spring rleated things
        self.k=k
        self.rl=rl
        self.type_spring=type_spring
        self.p1=0
        self.p2=self.diameter/2
        self.p3=0
        self.p4=-self.diameter/2
        self.h=0

        # data arrays of robots
        for i in range(self.nb):
            # positions
            self.xb["botx{0}".format(i)]=[]
            self.yb["boty{0}".format(i)]=[]
            self.zb["botz{0}".format(i)]=[]
            # velocities
            self.xvb["botx{0}".format(i)]=[]
            self.yvb["boty{0}".format(i)]=[]
            self.zvb["botz{0}".format(i)]=[]
            
            # forces 
            self.Ftxb["botx{0}".format(i)]=[]
            self.Ftyb["boty{0}".format(i)]=[]
            self.Ftzb["botz{0}".format(i)]=[]
            # spring lengths
            self.spring_length["spring{0}".format(i)]=[]
            self.Spring_force["spring{0}".format(i)]=[]

            # postion 
            theta=i*2*np.pi/self.nb
            x=self.R*np.cos(theta)
            y=.5*height
            z=self.R*np.sin(theta)

            # create body
            #bot = chrono.ChBody()
            
            # cylinder
            if self.geom=="cylinder":
            
                bot = chrono.ChBodyEasyCylinder(self.diameter/2, self.height,self.rowr)
                # set position
                bot.SetPos(chrono.ChVectorD(x,y,z))
                # material
                bot.SetMaterialSurface(self.material)
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
                # collision models
                bot.GetCollisionModel().ClearModel()
                bot.GetCollisionModel().AddCylinder(self.diameter,self.diameter,self.height/2) # hemi sizes
                bot.GetCollisionModel().BuildModel()
                
            # square
            if self.geom=="square":
                bot = chrono.ChBodyEasyBox(self.diameter,self.height,self.diameter,self.rowr,True,True)
                bot.SetPos(chrono.ChVectorD(x,y,z))
                bot.SetMaterialSurface(self.material)
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
                # collision model
                #bot.GetCollisionModel().ClearModel()
                #bot.GetCollisionModel().AddBox(self.diameter/2,self.height/2,self.diameter/2) # hemi sizes
                #bot.GetCollisionModel().BuildModel() 
               
            # x forces
            
            myforcex = chrono.ChForce()
            bot.AddForce(myforcex)
            myforcex.SetMode(chrono.ChForce.FORCE)
            myforcex.SetDir(chrono.VECT_X)
            #myforcex.SetVrelpoint(chrono.ChVectorD(x,.03*y,z))
            self.force.append(myforcex)
            
            # y forces    
            myforcey = chrono.ChForce()
            bot.AddForce(myforcey)
            myforcey.SetMode(chrono.ChForce.FORCE)
            myforcey.SetDir(chrono.VECT_Y)
            self.force.append(myforcey)
            
            # z forces            
            myforcez = chrono.ChForce()
            bot.AddForce(myforcez)
            myforcez.SetMode(chrono.ChForce.FORCE)
            myforcez.SetDir(chrono.VECT_Z)
            self.force.append(myforcez)
            
            # set collision
            bot.SetCollide(True)
            # set fixed
            bot.SetBodyFixed(self.fixed)
            
            # link to floor
            pt=chrono.ChLinkMatePlane()
            pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
            self.my_system.AddLink(pt)
            
            # appearance
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            bot.GetAssets().push_back(body_floor_texture)
            if i==0:
                col_g = chrono.ChColorAsset()
                col_g.SetColor(chrono.ChColor(0, 1, 0))
                bot.AddAsset(col_g)
            if i==int(25):  
                col_g = chrono.ChColorAsset()
                col_g.SetColor(chrono.ChColor(0, 0, 1))
                bot.AddAsset(col_g)
                
                
            # variable force springs 
            if self.type_spring=="var":
                
         # link springs
                if i>=1:
                    ground=chrono.ChLinkSpring()
                    # Identify points to be attatched to the springs 
                    ground.SetName("ground")
                    # Attatches  springs
                    ground.Initialize(self.bots[i-1], bot,True,chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringK(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    
                    
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    
                    # Last spring
                if i==self.nb-1:  
                    ground=chrono.ChLinkSpring()
                    ground.SetName("ground")
                    ground.Initialize(bot, self.bots[0], True, chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringK(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    
            # constant force spring 
            if self.type_spring=="const":
                
         # link springs
                if i>=1:
                    ground=chrono.ChLinkSpring()
                    # Identify points to be attatched to the springs 
                    ground.SetName("ground")
                    # Attatches  springs
                    ground.Initialize(self.bots[i-1], bot,True,chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringF(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground) 
                    
                    # Last spring
                if i==self.nb-1:  
                    ground=chrono.ChLinkSpring()
                    ground.SetName("ground")
                    ground.Initialize(bot, self.bots[0], True, chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False)
                    ground.Set_SpringF(self.k)
                    ground.Set_SpringRestLength(self.rl)
                    col1=chrono.ChColorAsset()
                    col1.SetColor(chrono.ChColor(0,0,1))
                    ground.AddAsset(col1)
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                    self.my_system.AddLink(ground)
                    self.Springs.append(ground)            
            
            # add bot to object array 
            self.my_system.Add(bot)
            self.bots.append(bot)
            self.obj.append(bot)
    # return system
    def return_system(self):
        return(self.my_system,self.Springs,self.bots,self.obj,self.force)

    # save position data
    def save_data_position(self):
        for i in range(self.nb):
            self.xb['botx'+str(i)].append(self.bots[i].GetPos().x)
            self.yb['boty'+str(i)].append(self.bots[i].GetPos().y)
            self.zb['botz'+str(i)].append(self.bots[i].GetPos().z)

    # save velocity data
    def save_data_velocity(self):
        for i in range(self.nb):
            self.xvb['botx'+str(i)].append(self.bots[i].GetPos_dt().x)
            self.yvb['boty'+str(i)].append(self.bots[i].GetPos_dt().y)
            self.zvb['botz'+str(i)].append(self.bots[i].GetPos_dt().z)    
            
    # save force data            
    def save_data_Forces(self):
        for i in range(self.nb):
            self.Ftxb["botx"+str(i)].append(self.bots[i].Get_Xforce().x)
            self.Ftyb["boty"+str(i)].append(self.bots[i].Get_Xforce().y)
            self.Ftzb["botz"+str(i)].append(self.bots[i].Get_Xforce().z)

            
     # save spring data       
    def save_data_spring_force(self):
        for i in range(self.nb):
            self.spring_length["spring"+str(i)].append(self.Springs[i].Get_SpringLength())
            if self.type_spring=="var":
                self.Spring_force["spring"+str(i)].append(self.Springs[i].Get_SpringLength()*self.Springs[i].Get_SpringK())
    
    # return spring data 
    def return_spring_data(self):
        return(self.Spring_force,self.spring_length)
        
    # return position data
    def return_position_data(self):
        return(self.xb,self.yb,self.zb)
        
    # return velocity data
    def return_velocity_data(self):
        return(self.xvb,self.yvb,self.zvb)
        
    # return force data    
    def return_force_data(self):
        return(self.Ftxb,self.Ftyb,self.Ftzb)
    
    # return last position
    def return_last_position(self):
        for i in range(self.nb):
            self.XL.append(self.xb['botx'+str(i)][-1])
            self.ZL.append(self.zb['botz'+str(i)][-1])
            
        np.savez("points.npz",allow_pickle=True,XL=self.XL,ZL=self.ZL)
        return(self.XL,self.ZL)
        
# In[Interior Particles]
class Interiors:
    def __init__(self,nb,diameter,diameter2,rowp,height,my_system,obj,body_floor,material,fixed,mode,granmode):
        
        # robots diameter
        self.diameter=diameter
        # particles diameter
        self.diameter2=diameter2
        # number of robots
        self.nb=nb    
        self.R=(self.diameter*self.nb/(np.pi*2))+.1 
        self.rowp=rowp
        self.height=height
        self.particles=[]
        self.obj=obj
        self.body_floor=body_floor
        self.my_system=my_system
        self.material=material
        self.fixed=fixed
        self.xp={}
        self.yp={}
        self.zp={}
        self.xvp={}
        self.yvp={}
        self.zvp={}
        self.Ftxp={}
        self.Ftyp={}
        self.Ftzp={}
        self.n=[]
        self.ni=0
        self.mode=mode
        self.granmode=granmode
        self.bound_force=[]
        # no interiors
        if self.mode=='empty':
            self.ni=0
            self.n=np.array([0])
        # max interiors
        if self.mode=="max":
            (n)=self.MaxValues()
            self.n=n[0]
            (self.ni)=np.sum(self.n)
        # not max interiors
        if self.mode=="nmax":
            (self.n)=self.Interior()
            (self.ni)=np.sum(self.n)
        # max non homo interios
        if self.mode=="nonhnmax":
            #self.n=2*np.array([29,32,18,18,8])
            
            self.n=np.array([49,40,39,30,29,20,19,10])
            (self.ni)=np.sum(self.n)
        
        # diamters are the same size
        if self.granmode=="homo":  
            count=0
            for i in range(self.n.size):
                for j in range(self.n[i]):
                    self.xp["gransx{0}".format(count)]=[]
                    self.yp["gransy{0}".format(count)]=[]
                    self.zp["gransz{0}".format(count)]=[]
                    self.xvp["gransx{0}".format(count)]=[]
                    self.yvp["gransy{0}".format(count)]=[]
                    self.zvp["gransz{0}".format(count)]=[]
                    self.Ftxp["gransx{0}".format(count)]=[]
                    self.Ftxp["gransy{0}".format(count)]=[]
                    self.Ftxp["gransz{0}".format(count)]=[]
                    count=count+1

                    R2=self.diameter2*self.n[i]/(np.pi*2)
                    x=R2*np.cos(j*2*np.pi/self.n[i])
                    y=.5*self.height
                    z=R2*np.sin(j*2*np.pi/self.n[i])
                    # create body
                    gran = chrono.ChBody()
                    gran = chrono.ChBodyEasyCylinder(self.diameter2/2, self.height,self.rowp)
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.material)
                    gran.SetId(i)
                    # Create collision model
                    gran.GetCollisionModel().ClearModel()
                    gran.GetCollisionModel().AddCylinder(self.diameter2/2,self.diameter2/2,self.height/2) # hemi sizes
                    gran.GetCollisionModel().BuildModel()
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    # add color
                    col_r = chrono.ChColorAsset()
                    col_r.SetColor(chrono.ChColor(1, 0, 0))
                    gran.AddAsset(col_r)
                    # mate to floor
                    pt=chrono.ChLinkMatePlane()
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    self.my_system.AddLink(pt)
                    
                    # Add constraining force object
                    forcec=chrono.ChForce()
                    gran.AddForce(forcec)
                    forcec.SetMode(chrono.ChForce.FORCE)
                    forcec.SetDir(chrono.VECT_Z)
                    self.bound_force.append(forcec)
                    
                    # add to system
                    self.my_system.Add(gran)
                    self.obj.append(gran)
                    self.particles.append(gran)
        # if diamters are not the same size            
        if self.granmode=="nonhomo": 
            count=0
            for i in range(self.n.size):
                print(i)
                if i%2==0:
                    self.diameter3=self.diameter2*(2**.5)
                else:
                    self.diameter3=self.diameter2
                for j in range(self.n[i]):
                    self.xp["gransx{0}".format(count)]=[]
                    self.yp["gransy{0}".format(count)]=[]
                    self.zp["gransz{0}".format(count)]=[]
                    self.xvp["gransx{0}".format(count)]=[]
                    self.yvp["gransy{0}".format(count)]=[]
                    self.zvp["gransz{0}".format(count)]=[]
                    self.Ftxp["gransx{0}".format(count)]=[]
                    self.Ftxp["gransy{0}".format(count)]=[]
                    self.Ftxp["gransz{0}".format(count)]=[]                  
                    count=count+1
                    R2=self.diameter3*self.n[i]/(np.pi*2)
                    x=R2*np.cos(j*2*np.pi/self.n[i])
                    y=.5*self.height
                    z=R2*np.sin(j*2*np.pi/self.n[i])
                    gran = chrono.ChBody()
                    gran = chrono.ChBodyEasyCylinder(self.diameter3/2, self.height,self.rowp)
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.material)
                    gran.SetId(i)
                    # Create collision model
                    gran.GetCollisionModel().ClearModel()
                    gran.GetCollisionModel().AddCylinder(self.diameter3/2,self.diameter3/2,self.height/2) # hemi sizes
                    gran.GetCollisionModel().BuildModel()
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    # add color
                    col_r = chrono.ChColorAsset()
                    col_r.SetColor(chrono.ChColor(1, 0, 0))
                    gran.AddAsset(col_r)
                    # mate to floor
                    pt=chrono.ChLinkMatePlane()
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    self.my_system.AddLink(pt)
                    # Add constraining force object
                    forcec=chrono.ChForce()
                    gran.AddForce(forcec)
                    forcec.SetMode(chrono.ChForce.FORCE)
                    forcec.SetDir(chrono.VECT_Z)
                    self.bound_force.append(forcec)
                    
                    # add to system
                    self.my_system.Add(gran)
                    self.obj.append(gran)
                    self.particles.append(gran)  
        
# max interior particles
    def MaxValues(self):
        Rin=self.R
        ngrans1=int(Rin/(self.diameter2))
        ri=np.zeros((1,ngrans1))
        ni=np.zeros((1,ngrans1))
        radii=Rin-(self.diameter2/2)
        for i in range(ngrans1):
            remainder=((self.diameter2))*i
            ri[:,i]=radii-remainder
            ni[:,i]=np.floor((ri[:,i]*np.pi*2)/self.diameter2)
        self.n=np.asarray(ni,dtype=int)
        return(self.n)
        
    # interior     
    def Interior(self):
        #self.n=np.arange(self.nb-15,5,-13)   # array of interior robots
        self.n=np.array([10,3])
        return(self.n)
        
    # return system
    def return_system(self):
        return(self.my_system,self.particles,self.obj,self.bound_force)
# save position data
    def save_data_position(self):
        for i in range(self.ni):
            self.xp["gransx"+str(i)].append(self.particles[i].GetPos().x)
            self.yp["gransy"+str(i)].append(self.particles[i].GetPos().y)
            self.zp["gransz"+str(i)].append(self.particles[i].GetPos().z)
        return(self.xp,self.yp,self.zp)
        
    # save velocity data
    def save_data_velocity(self):
        for i in range(self.ni):
            self.xvp['gransx'+str(i)].append(self.particles[i].GetPos_dt().x)
            self.yvp['gransy'+str(i)].append(self.particles[i].GetPos_dt().y)
            self.zvp['gransz'+str(i)].append(self.particles[i].GetPos_dt().z)    
            
    # save force data            
    def save_data_Forces(self):
        for i in range(self.nb):
            self.Ftxp["gransx"+str(i)].append(self.particles[i].Get_Xforce().x)
            self.Ftyp["gransy"+str(i)].append(self.particles[i].Get_Xforce().y)
            self.Ftzp["gransz"+str(i)].append(self.particles[i].Get_Xforce().z)  
    
  # return position data      
    def return_position_data(self):
        return(self.xp,self.yp,self.zp)
        
    # return velocity data
    def return_velocity_data(self):
        return(self.xvp,self.yvp,self.zvp)
        
    # return force data    
    def return_force_data(self):
        return(self.Ftxp,self.Ftyp,self.Ftzp)   
# In[BALL]
'''
Creates a ball for grabbing 
'''         
class Ball:
    def __init__(self,diameter,height,rowr,x,z,my_system,body_floor,obj,material):
        self.diameter=diameter
        self.height=height
        self.rowr=rowr
        self.x=x                    
        self.z=z
        self.y=self.height/2
        self.my_system=my_system
        self.obj=obj
        self.body_floor=body_floor
        self.material=material
        self.bforce=[]
        self.fixed=True
        self.balls=[]
        self.ballx=[]
        self.ballz=[]
        self.Fballx=0
        self.Fballz=0
        self.nsides=12
        self.geom="circle"
        self.path='C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/python/Pychrono/Strings/Grabbing/Grab_sim_pot_field_shape/shapes/'
        # ball circle
        if self.geom=="circle":
        #Create ball
            ball = chrono.ChBody()
            ball = chrono.ChBodyEasyCylinder(self.diameter/2, self.height,self.rowr)
            # set position
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            # material
            ball.SetMaterialSurface(self.material)

            # collision models
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddCylinder(self.diameter/2,self.diameter/2,self.height/2) # hemi sizes
            ball.GetCollisionModel().BuildModel()
            
        if self.geom=="square":
            ball = chrono.ChBodyEasyBox(self.diameter,self.height,self.diameter,self.rowr)
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetMaterialSurface(self.material)

            # collision model
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddBox(self.diameter/2,self.height/2,self.diameter/2) # hemi sizes
            ball.GetCollisionModel().BuildModel() 

        if self.geom=="polygon":
            # create points for convex hull
            pt_vect = chrono.vector_ChVectorD()
            # creates bottom
            for i in range(self.nsides):
                pt_vect.push_back(chrono.ChVectorD((self.diameter/2)*np.cos(i*2*np.pi/self.nsides),self.height/2,(self.diameter/2)*np.sin(i*2*np.pi/self.nsides)))
            #create top 
            for i in range(self.nsides):
                pt_vect.push_back(chrono.ChVectorD((self.diameter/2)*np.cos(i*2*np.pi/self.nsides),-self.height/2,(self.diameter/2)*np.sin(i*2*np.pi/self.nsides)))
            
            ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rowr,True,True)   
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddConvexHull(pt_vect)
            ball.GetCollisionModel().BuildModel()
 
        if self.geom=="star":
            ball = chrono.ChBody()
# Attach a visualization shape .
# First load a .obj from disk into a ChTriangleMeshConnected:
            mesh_for_visualization = chrono.ChTriangleMeshConnected()
            mesh_for_visualization.LoadWavefrontMesh(self.path+'star.obj')
            #mesh_for_visualization.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
            visualization_shape = chrono.ChTriangleMeshShape()
            visualization_shape.SetMesh(mesh_for_visualization)
            ball.AddAsset(visualization_shape)
            #polygon_texture = chrono.ChTexture()
            #polygon_texture.SetTextureFilename(self.path+'octB.png')
            #ball.GetAssets().push_back(polygon_texture)


            mesh_for_collision = chrono.ChTriangleMeshConnected()
            mesh_for_collision.LoadWavefrontMesh(self.path+'star.obj')
# Optionally: you can scale/shrink/rotate the mesh using this:
            #mesh_for_collision.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddTriangleMesh(
            mesh_for_collision, # the mesh 
            False,  # is it static?
            False)  # is it convex?
            # , mpos, mr,  # pos of mesh respect to REF and rotation matr.respect to REF 
            # 0.01) # 'inflating' radiust for triangles for increased robustness
            ball.GetCollisionModel().BuildModel()        
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetMass(16)
            ball.SetInertiaXX(chrono.ChVectorD(0.270,0.400,0.427))
            ball.SetInertiaXY(chrono.ChVectorD(0.057,0.037,-0.062))
            
        ball.SetCollide(True)
        ball.SetBodyFixed(self.fixed)
        pt=chrono.ChLinkMatePlane()
        pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
        self.my_system.AddLink(pt)
        body_floor_texture = chrono.ChTexture()
        body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
        ball.GetAssets().push_back(body_floor_texture)   
        
        # force x
        myforcex = chrono.ChForce()
        ball.AddForce(myforcex)
        myforcex.SetMode(chrono.ChForce.FORCE)
        myforcex.SetDir(chrono.VECT_X)
        self.bforce.append(myforcex)
        # force y    
        myforcey = chrono.ChForce()
        ball.AddForce(myforcey)
        myforcey.SetMode(chrono.ChForce.FORCE)
        myforcey.SetDir(chrono.VECT_Y)
        self.bforce.append(myforcey)
        # force z            
        myforcez = chrono.ChForce()
        ball.AddForce(myforcez)
        myforcez.SetMode(chrono.ChForce.FORCE)
        myforcez.SetDir(chrono.VECT_Z)
        self.bforce.append(myforcez)
        
        self.my_system.Add(ball)
        self.obj.append(ball)
        self.balls.append(ball)
        
    def save_data_position(self):
        self.ballx.append(self.balls[0].GetPos().x)
        self.ballz.append(self.balls[0].GetPos().z)
        
    def return_position_data(self):
        return(self.ballx,self.ballz)
        
        
    # return system    
    def return_system(self):
        return(self.obj,self.bforce,self.my_system,self.balls)   
        
# In[Simulate]
''' 
 creates the simulation 
'''
class simulate:
    def __init__(self,my_system,bots,particles,Springs,balls,obj,my_rep,shapes,sim,tstep,tend,visual,data_path,controls):   
        self.visual=visual
        self.my_system=my_system
        self.sim=sim
        self.tstep=tstep
        self.tend=tend
        self.visual=visual
        self.data_path=data_path
        self.particles=particles
        self.bots=bots
        self.obj=obj
        self.Springs=Springs
        self.controls=controls
        self.time=[]
        self.my_rep=my_rep
        self.nc=[]
        self.cx=[]
        self.cy=[]
        self.cz=[]
        self.Fxct=[]
        self.Fyct=[]
        self.Fzct=[]
        self.balls=balls
        self.shapes=shapes
        self.Trip=False
        # contact points and forces 

    # simulate the robot
    def simulate(self):
        if self.visual=="irrlecht":
    #  Create an Irrlicht application to visualize the system
            myapplication = chronoirr.ChIrrApp(self.my_system,self.sim, chronoirr.dimension2du(1600,1200))
            myapplication.AddTypicalSky()
            myapplication.AddTypicalLogo('logo_pychrono_alpha.png')
            myapplication.AddTypicalCamera(chronoirr.vector3df(0,2,.75),chronoirr.vector3df(0,0,.75))
            myapplication.SetSymbolscale(.002)
            myapplication.SetShowInfos(True)
            myapplication.SetContactsDrawMode(2)
            myapplication.SetPaused(True)
            myapplication.AddLightWithShadow(chronoirr.vector3df(2,5,2),chronoirr.vector3df(2,2,2),10,2,10,120)

            myapplication.DrawAll               
            myapplication.AssetBindAll();
            myapplication.AssetUpdateAll();
            myapplication.AddShadowAll();
            count=0
            myapplication.SetTimestep(self.tstep)
            myapplication.SetTryRealtime(False)
            cc=0
            while(myapplication.GetDevice().run()):
                self.my_rep.ResetList()
                myapplication.BeginScene()
                myapplication.DrawAll()
                print ('time=', self.my_system.GetChTime())
                
                self.time.append(self.my_system.GetChTime())
                count=count+1
                
                # run the controllers 
                self.controls.run_controller()
                self.controls.constrain_interior()
                self.controls.save_data_Forces()
                self.controls.clear_temp_forces()
                self.controls.Error()
                # save ball position
                self.balls.save_data_position()
                # save bot parameters
                self.bots.save_data_position()
                self.bots.save_data_Forces()
                self.bots.save_data_velocity()
                self.bots.save_data_spring_force()
                # save particle parameters
                self.particles.save_data_position()
                self.particles.save_data_velocity()
                # contact data
                self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
                crt_list = self.my_rep.GetList()
                self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
                self.cx.append(crt_list[0])
                self.cy.append(crt_list[1])
                self.cz.append(crt_list[2])
                self.Fxct.append(crt_list[3])
                self.Fyct.append(crt_list[4])
                self.Fzct.append(crt_list[5])
                # run step
                myapplication.DoStep()
                myapplication.EndScene()
                # save data
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.tend:
                    myapplication.GetDevice().closeDevice()
        return(self.bots,self.time,self.controls,self.cx,self.cy,self.cz,self.Fxct,self.Fyct,self.Fzct,self.nc)
        
#[material]          
def Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs):
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(mu_f)
    material.SetDampingF(mu_b)
    material.SetCompliance (C)
    material.SetComplianceT(Ct)
    material.SetRollingFriction(mu_r)
    material.SetSpinningFriction(mu_s)
    material.SetComplianceRolling(Cr)
    material.SetComplianceSpinning(Cs)
    return material

#[Create Floor]
def Floor(material,length,tall):
    body_floor = chrono.ChBody()
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVectorD(0, -tall, 0 ))
    body_floor.SetMaterialSurface(material)
    body_floor.GetCollisionModel().ClearModel()
    body_floor.GetCollisionModel().AddBox(length, tall, length) # hemi sizes
    body_floor.GetCollisionModel().BuildModel()
    body_floor.SetCollide(True)
    body_floor_shape = chrono.ChBoxShape()
    body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(length, tall, length)
    body_floor.GetAssets().push_back(body_floor_shape)
    body_floor_texture = chrono.ChTexture()
    body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'aluminum.jpg')
    body_floor.GetAssets().push_back(body_floor_texture)
    return(body_floor)
    
# In[Controller]
'''
Contains controller for actuation of exterior bots as well as logic to 
keep interior particles contained
'''    
class Controls:
    def __init__(self,forces,bots,interiors,fbound,Springs,balls,my_system,k,rl,rlmax,type_spring,control_type,mag,shapes,alpha,beta,nb):
        # objects
        self.forces=forces
        self.bots=bots
        self.interiors=interiors
        self.bound_force=fbound
        self.Springs=Springs
        self.my_system=my_system
        self.balls=balls
    
        # Springs
        self.k=k
        self.rl=rl
        self.rlmax=rlmax
        self.type_spring=type_spring
        # Control variables
        self.control_type=control_type
        self.mag=mag
        self.xb=[]
        self.zb=[]
        self.xbv=[]
        self.zbv=[]
        self.shapes=shapes
        self.alpha=alpha
        self.beta=beta
        self.fnx=shapes.fnx
        self.fny=shapes.fny
        self.rbf=shapes.rbf
        self.nb=nb
        # save controller forces
        self.Faxc={}
        self.Fayc={}
        self.Fazc={}
        # temporary arrays
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        self.E=[]
        self.jam_mode=False
        
        # ball variables
        self.ballx=0
        self.ballz=0
        self.ballxd=0
        self.ballzd=0
        self.tball=0
        self.bforce=0
        self.Fballx=0
        self.Fballz=0
        # shape variables
        self.p1=0
        self.p2=0
        self.diameter=0
        self.bl=0
        self.br=0
        self.R=0
        self.nr=0
        self.Rd=0
        self.ball_pull=False
        
        for i in range(self.nb):
            self.Faxc["botx{0}".format(i)]=[]
            self.Fayc["boty{0}".format(i)]=[]
            self.Fazc["botz{0}".format(i)]=[]
    
# save the variables        
    def save_data_Forces(self):
       for i in range(self.nb):
           self.Faxc["botx"+str(i)].append(self.fxt[i])
           self.Fayc["boty"+str(i)].append(self.fyt[i])
           self.Fazc["botz"+str(i)].append(self.fzt[i])
                
    def clear_temp_forces(self):
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
    
# return force data for controllers     
    def return_force_data(self):
        return(self.Faxc,self.Fayc,self.Fazc,self.E)        
        
    # run controller
    def run_controller(self):
        # no force applied
        if self.control_type=="nothing":
            
            (self.Springs)=self.setspring()
            
        # robot told to go right    
        if self.control_type=="force_right":
            for i in range(len(self.forces)):
                self.forces[i].SetMforce(self.mag)
                self.forces[i].SetDir(chrono.VECT_X)
                
                
       # pot field command         
        if self.control_type=="pot_field":
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            
            for i in range(self.nb):
                
                fx=-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i]
                fz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]
                
                self.fxt.append(fx[0])
                self.fyt.append(0)
                self.fzt.append(fz[0])
                
                self.forces[3*i].SetMforce(fx[0])
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(fz[0])
                self.forces[3*i+2].SetDir(chrono.VECT_Z)
                
 # pot field grab          
        if self.control_type=="pot_field_grab":
            
            if .5<self.my_system.GetChTime(): 
                self.alpha=100
                #self.shapes.shape="circle"
                self.shapes.p2=self.balls.balls[0].GetPos().x
                self.shapes.p1=self.balls.balls[0].GetPos().z
                (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
                    
            if .75<self.my_system.GetChTime():
                self.ball_pull=True
                self.balls.balls[0].SetBodyFixed(True)
            
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            self.jam_springs()
            self.Ball_controller()
            
            for i in range(self.nb):
                fx=-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i]
                fz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]
                self.fxt.append(fx[0])
                self.fyt.append(0)
                self.fzt.append(fz[0])
                self.forces[3*i].SetMforce(fx[0])
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(fz[0])
                self.forces[3*i+2].SetDir(chrono.VECT_Z)

        # have the robots drag the ball 
        if self.control_type=="pot_field_drag":
            if .5<self.my_system.GetChTime(): 
                self.alpha=4600
                self.shapes.shape="circle"
                self.shapes.p2=self.balls.balls[0].GetPos().x
                self.shapes.p1=self.balls.balls[0].GetPos().z
                (self.rbf,self.fnx,self.fny)=self.shapes.Create_shape_gradient()
                    
            if .75<self.my_system.GetChTime():
                self.ball_pull=True
                self.balls.balls[0].SetBodyFixed(False)
            
            (self.Springs)=self.setspring()
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            self.jam_springs()
            self.Ball_controller()
            
            for i in range(self.nb):
                fx=-self.alpha*self.fny([self.xb[i],self.zb[i]])-self.beta*self.xbv[i]
                fz=-self.alpha*self.fnx([self.xb[i],self.zb[i]])-self.beta*self.zbv[i]
                self.fxt.append(fx[0])
                self.fyt.append(0)
                self.fzt.append(fz[0])
                self.forces[3*i].SetMforce(fx[0])
                self.forces[3*i].SetDir(chrono.VECT_X)
                if .75<self.my_system.GetChTime(): 
                    self.forces[3*i+2].SetMforce(fz[0]+-self.mag)
                    self.forces[3*i+2].SetDir(chrono.VECT_Z)    
                else: 
                    self.forces[3*i+2].SetMforce(fz[0])
                    self.forces[3*i+2].SetDir(chrono.VECT_Z) 
# Create ball paramters                
    def set_ball_parameters(self,xball,zball,tball,bforce,Fballx,Fballz):
        self.ballxd=xball
        self.ballzd=zball
        self.tball=tball   
        self.bforce=bforce
        self.Fballx=Fballx
        self.Fballz=Fballz
        
# Ball controller                 
    def Ball_controller(self):
        if self.ball_pull==True:
            self.bforce[0].SetMforce(self.Fballz)
            self.bforce[0].SetDir(chrono.VECT_Z)
            self.bforce[1].SetMforce(self.Fballx)
            self.bforce[1].SetDir(chrono.VECT_X)   
            
    def dist(self,x1, y1, x2, y2, x3, y3): # x3,y3 is the point
        dist=np.ones([len(x3),len(x1)])
        for i in range(1,len(x1)):
            px = x2[i]-x1[i]
            py = y2[i]-y1[i]
            norm = px*px + py*py
            u =  ((x3 - x1[i]) * px + (y3 - y1[i]) * py) / float(norm)
            u[u>1]=1
            u[u<0]=0
            x = x1[i] + u * px
            y = y1[i] + u * py
            dx = x - x3
            dy = y - y3
            dist[:,i] = np.sqrt(dx*dx + dy*dy)
        # For last pair
#        px = x2[0]-x1[-1]
#        py = y2[0]-y1[-1]
#        norm = px*px + py*py
#        u =  ((x3 - x1[-1]) * px + (y3 - y1[-1]) * py) / float(norm)
#        u[u>1]=1
#        u[u<0]=0
#        x = x1[-1] + u * px
#        y = y1[-1] + u * py
#        dx = x - x3
#        dy = y - y3
#        dist[:,-1] = np.sqrt(dx*dx + dy*dy)
        return dist
    
    def constrain_interior(self):
        diam = 0.005
        max_f = 3
        
        int_pos = np.ones([3,len(self.interiors)])
        pos1 = np.ones([3,self.nb])
        pos2 = np.ones([3,self.nb])
        norms = np.ones([3,self.nb])
        
        for i in range(len(self.interiors)): # Loop over all interior particles
            int_pos[:,i] = [self.interiors[i].GetPos().x,self.interiors[i].GetPos().y,self.interiors[i].GetPos().z]
        for j in range(1,self.nb): # Loop over all bots
            # Current bot
            a = np.array([self.bots[j].GetPos().x,self.bots[j].GetPos().y,self.bots[j].GetPos().z]) 
            # Previous bot
            b = np.array([self.bots[j-1].GetPos().x,self.bots[j-1].GetPos().y,self.bots[j-1].GetPos().z])   
            # should be the inward normal assuming bots were created CCW fashion
            norm = np.cross([0,-1,0],a-b)
            pos1[:,j]=a
            pos2[:,j]=b
            norms[:,j]=norm/np.linalg.norm(norm)
        
        # Distance from each interior particle center to each line segment b/t two bots
        # Disty is [number_interior, number_bots] in size
        disty = self.dist(pos1[0,:],pos1[2,:],pos2[0,:],pos2[2,:],int_pos[0,:],int_pos[2,:])
        disty[disty>diam]=0
        
        # Loop over interior particle
        for k in range(len(disty[:,1])):
            # nonzero distance indices
            nonzero=np.nonzero(disty[k,:])[0]
            if (len(nonzero)==0):
                continue
            # average contact force and normal for each interior particle
            nz_dist=disty[k,nonzero]
            mag=np.sum(1/nz_dist[0])
            if mag>max_f: mag=max_f
            weights = (1/nz_dist[0])/np.linalg.norm(1/nz_dist[0])
            avg_x = np.dot(weights,norms[1,nonzero])[0]
            avg_z = np.dot(weights,norms[2,nonzero])[0]
            
            # Apply force in normal direction proportional to distance to line
            self.bound_force[k].SetDir(chrono.ChVectorD(avg_x,0,avg_z))
            self.bound_force[k].SetMforce(mag)
            
        # Loop over bots
        for k in range(len(disty[1,:])):
            # nonzero distance indices
            nonzero=np.nonzero(disty[:,k])[0]
            if (len(nonzero)==0):
                continue
            # average contact force and normal for each bot pair
            nz_dist=disty[nonzero,k]
            mag=np.sum(1/nz_dist[0])
            if mag>max_f: mag=max_f
            mag_x = mag*norms[0,k]
            mag_z = mag*norms[2,k]
            
            # Add equal and opposite forces to two bots so net force is zero
            fx = self.forces[3*k].GetMforce()
            fz = self.forces[3*k+2].GetMforce()
            self.forces[3*k].SetMforce(mag_x+fx)
            self.forces[3*k].SetDir(chrono.VECT_X)
            self.forces[3*k+2].SetMforce(mag_z+fz)
            self.forces[3*k+2].SetDir(chrono.VECT_Z)
            
# Keep spring lengths                
    def setspring(self):
        for i in range(len(self.Springs)):
            var1=self.Springs[i].Get_SpringLength()
            # if spring length is less then goes to zero
            if var1<self.rl:
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(0)
                    
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(0)
            # if greater then it will double
            if var1>self.rlmax:
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(2*self.k)
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(2*self.k)
                    
        return(self.Springs)
    
# jam springs    
    def jam_springs(self):
        if self.jam_mode==True:
            self.rlmax=self.rlmax/4
            for i in range(len(self.Springs)):
                if self.type_spring=="const":
                    self.Springs[i].Set_SpringF(3*self.k)
                    
                if self.type_spring=="var":
                    self.Springs[i].Set_SpringK(2*self.k) 
            
        return(self.Springs)
        
# get current position         
    def get_position(self):
        self.xb=[]        
        self.zb=[]
        for i in range(len(self.bots)):
            self.xb.append(self.bots[i].GetPos().x)
            self.zb.append(self.bots[i].GetPos().z)
        return(self.xb,self.zb)
    
# Get ball Position
    def get_ball_position(self):
        self.ballx=self.balls[0].GetPos().x
        self.ballz=self.balls[0].GetPos().z
        return(self.ballx,self.ballz)
        
 # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.zbv=[]
        for i in range(len(self.bots)):
            self.xbv.append(self.bots[i].GetPos_dt().x)
            self.zbv.append(self.bots[i].GetPos_dt().z)
        return(self.xbv,self.zbv)
        
# error of controller        
    def Error(self):
        if self.control_type=="pot_field" or self.control_type=="pot_field_grab":
            et=[]
            for i in range(self.nb):
                val=(self.rbf(self.zb[i],self.xb[i]))**2
                et.append(val)
            self.E.append(.5*sum(et))
            
# Class for creating the points for the RBF
class Points_for_shape:
    def __init__(self,shape,p1,p2,nb,diameter,bl,br,R,nr,Rd):
        self.shape=shape
        self.p1=p1
        self.p2=p2
        self.nb=nb
        self.nr=nr
        self.diameter=diameter
        self.R=R
        self.Rd=Rd
        self.x=0
        self.y=0
        self.z=0
        self.rbf=0
        self.br=br
        self.bl=bl
        self.fny=0
        self.fnx=0
        self.xp=0
        self.yp=0
        self.xp2=0
        self.yp2=0
        
        # Outershape
        self.OR=1.1*self.R
        self.theta0=1*np.pi/12
        self.theta1=23*np.pi/12
        # Inner shape
        self.IR=.4*self.R
        self.theta2=np.pi/2
        self.theta3=3*np.pi/2
        self.epsilon=.3*self.R
    # plot the RBF function
    def Plot_rbf(self):
        fsy = 6                               # Height of figure in inches
        fsx = fsy*scipy.constants.golden      # Width of figure in inches (width will be fsy * golden ratio)
        fig = plt.figure(figsize = (1.25*fsy, fsy))     # Set the figure size to square
        ti = np.linspace(self.bl, self.br, 100)
        xx, yy = np.meshgrid(ti, ti)
        zz = self.rbf(xx, yy)
        plt.pcolor(xx, yy, zz,cmap = 'jet')
        plt.colorbar()
    
        fig = plt.figure(figsize = (fsx, fsy))          # Set the figure size
        ax = fig.gca(projection='3d')                   # Include axes
        surf = ax.plot_surface(xx, yy, zz, cmap = 'jet')   # Plot the 3-D surface using the "jet" color map
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        fig.colorbar(surf)                              # Include color bar
        plt.show()    
# create the gradient ^2 function    
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
# Create the points for the RBF
    def Points_for_shape(self):
        # grab
        if self.shape=='grab':
            (self.x,self.y,self.z)=self.points_grab()
            
        # grab2   
        if self.shape=='grab2':
            (self.x,self.y,self.z)=self.points_grab2() 
                 
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
    
#[ points for Square]    
    def points_square(self):
        xt=np.array([1,1,0,-1,-1,-1,0,1])
        yt=np.array([0,1,1,1,0,-1,-1,-1])
    
        self.x=np.zeros(len(self.nr)*len(xt))
        self.y=np.zeros(len(self.nr)*len(xt))
        self.z=np.zeros(len(self.nr)*len(xt))

    
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                self.x[i+len(xt)*j]=(self.nr[j]+self.Rd)*xt[i]+self.p1
                self.y[i+len(xt)*j]=(self.nr[j]+self.Rd)*yt[i]+self.p2
                if j==0:
                    self.z[i+len(xt)*j]=0
                else:
                    self.z[i+len(xt)*j]=j             
        return (self.x,self.y,self.z)   
#[points for grab]
    def points_grab(self):
        
        #theta1=np.linspace(np.pi/4,7*np.pi/4,int(.67*self.nb))
        self.OR=self.R
        
        theta1=np.linspace(self.theta0,self.theta1,int(.67*self.nb))
        x1=self.OR*np.cos(theta1)+self.p1
        y1=self.OR*np.sin(theta1)+self.p2

        theta2=np.linspace(self.theta2,self.theta3,int(.3*self.nb))
        x2=self.IR*np.cos(theta2)+self.epsilon+self.p1
        y2=self.IR*np.sin(theta2)+self.p2 
        
        self.xp=np.concatenate((x1, x2), axis=None)
        self.yp=np.concatenate((y1, y2), axis=None)
        zp=np.zeros(len(self.xp))
        
        theta3=np.linspace(np.pi/2+.3,3*np.pi/2-.3,int(.3*self.nb))
        x3=.6*self.R*np.cos(theta3)+1.5*self.R+self.p1
        y3=.6*self.R*np.sin(theta3)+self.p2

        theta4=np.linspace(np.pi/4-.2,7*np.pi/4+.2,int(.67*self.nb))
        x4=1.5*self.R*np.cos(theta4)+self.p1
        y4=1.5*self.R*np.sin(theta4)+self.p2
        
        xr=np.concatenate((x3, x4), axis=None)
        yr=np.concatenate((y3, y4), axis=None)
        zr=2*np.ones(len(xr))
    
        self.x=np.concatenate((self.xp, xr), axis=None)
        self.y=np.concatenate((self.yp, yr), axis=None)
        self.z=np.concatenate((zp, zr), axis=None)
        
        #plt.plot(x1,y1,'bo',x2,y2,'bo',x3,y3,'go',x4,y4,'go')
        return (self.x,self.y,self.z) 
    
    # plot points used to make rbf
    def plot_points(self):
        plt.scatter(self.xp,self.yp,color='blue')

    # function for points grab 2    
    def points_grab2(self):
        path="C:/Users/dmulr/OneDrive/Documents/dm-soro_chrono/python/Pychrono/Grabbing/Grab_sim_pot_field_shape/"
        file="points.npz"
        data=np.load(path+file,allow_pickle=True)
        YL=.7*data['XL']
        XL=.7*data['ZL']
        self.xp2=np.concatenate((XL), axis=None)
        self.yp2=np.concatenate((YL), axis=None)
        XL2=3.5*XL
        YL2=3.5*YL+.2

        XL3=3*XL2
        YL3=3*YL2
        
        zp=np.zeros(len(XL))
        zr=np.ones(len(XL2))
        zr2=2*np.ones(len(XL2))
        self.x=np.concatenate((XL,XL3), axis=None)
        self.y=np.concatenate((YL,YL3), axis=None)
        self.z=np.concatenate((zp, zr2), axis=None)
        return (self.x,self.y,self.z) 

#[ Points for circle]
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
#                self.x[i+self.nb*j]=(self.R+nt)*np.cos(theta[i])+self.p1
#                self.y[i+self.nb*j]=(self.R+nt)*np.sin(theta[i])+self.p2
                self.x[i+self.nb*j]=(self.Rd+nt)*np.cos(theta[i])+self.p1
                self.y[i+self.nb*j]=(self.Rd+nt)*np.sin(theta[i])+self.p2
                if j==0:
                    self.z[i+self.nb*j]=0
                else:
                    self.z[i+self.nb*j]=nt
                    
            self.xp=self.x[0:self.nb]
            self.yp=self.y[0:self.nb]           
        return (self.x,self.y,self.z)     
    
# Contact callback    
class MyReportContactCallback(chrono.ReportContactCallback):

    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.pointx = []
        self.pointy = []
        self.pointz = []
        #self.bodies = []
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
#        bodyUpA = chrono.CastContactableToChBody(modA)
#        nameA = bodyUpA.GetId()
#        bodyUpB = chrono.CastContactableToChBody(modB)
#        nameB = bodyUpB.GetId()
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fxc.append(force.x)
        self.Fyc.append(force.y)
        self.Fzc.append(force.z)
        #self.bodies.append([nameA,nameB])
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = [] 
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
    # Get the points
    def GetList(self):
        return (self.pointx,self.pointy,self.pointz,self.Fxc,self.Fyc,self.Fzc)


                
   # export data             
class export_data():
    def __init__(self,bots,interior,ball,cx,cy,cz,Fxct,Fyct,Fzct,nc,controller,nb,sim,time,shape,save_data,mr,mp):
        
        # objects 
        self.bots=bots
        self.interior=interior
        self.ball=ball
        self.controller=controller
        # additional variables
        self.tend=time[-1]
        self.time={'time': time}
        
        self.sim=sim
        self.nb=nb
        self.nc=np.asarray(nc)
        self.lengthm=np.amax(self.nc)
        self.cx=cx
        self.cy=cy
        self.cz=cz
        self.Fxct=Fxct
        self.Fyct=Fyct
        self.Fzct=Fzct
        self.count=len(time)
        self.lengthm=np.amax(self.nc)
        self.save_data=save_data
        #Create empty contact matrices
        self.xc=np.zeros((self.lengthm,self.count))
        self.yc=np.zeros((self.lengthm,self.count))
        self.zc=np.zeros((self.lengthm,self.count))
# Contact forces
        self.Fcx=np.zeros((self.lengthm,self.count))
        self.Fcy=np.zeros((self.lengthm,self.count))
        self.Fcz=np.zeros((self.lengthm,self.count))
        self.shape=shape
        self.mp=mp
        self.mr=mr
        for i in range(self.count):
            ind=self.nc[i]
            tryme=self.cx[i]
            tryme2=self.cy[i]
            tryme3=self.cz[i]
            tryme4=self.Fxct[i]
            tryme5=self.Fyct[i]
            tryme6=self.Fzct[i]
            # convert to array
            tryme=np.asarray(tryme)
            tryme2=np.asarray(tryme2)
            tryme3=np.asarray(tryme3)
            tryme4=np.asarray(tryme4)
            tryme5=np.asarray(tryme5)
            tryme6=np.asarray(tryme6)
    
    # fill array position
            self.xc[0:ind,i]=np.transpose(tryme)
            self.yc[0:ind,i]=np.transpose(tryme2)
            self.zc[0:ind,i]=np.transpose(tryme3)

# Fill array forces
            self.Fcx[0:ind,i]=np.transpose(tryme4)
            self.Fcy[0:ind,i]=np.transpose(tryme5)
            self.Fcz[0:ind,i]=np.transpose(tryme6)    

    
        # return data robots
        (self.xb,self.yb,self.zb)=self.bots.return_position_data()
        (self.Faxb,self.Fayb,self.Fazb)=self.bots.return_force_data()
        (self.xvb,self.yvb,self.zvb)=self.bots.return_velocity_data()
        
        # spring data
        (self.Spring_force,self.Spring_length)=self.bots.return_spring_data()    
        
        # control data
        (self.Faxc,self.Fayc,self.Fazc,self.E)=self.controller.return_force_data()
        
        # Return Particle data
        (self.xp,self.yp,self.zp)=self.interior.return_position_data()
        (self.Faxp,self.Fayp,self.Fazp)=self.interior.return_force_data()
        (self.xvp,self.yvp,self.zvp)=self.interior.return_velocity_data()      
        (self.ballx,self.ballz)=self.ball.return_position_data()
        self.results_dir = os.path.join('robot_data'+self.sim+'/')     

        if not os.path.isdir(self.results_dir):
            os.makedirs(self.results_dir)
# In[position data]            
        if self.save_data[0]==True:        
        
            self.file_name0=self.results_dir+'/bot_position.csv'

            with open(self.file_name0, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xb.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yb.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zb.items():
                    w.writerow([key, *val])     
# In[bot velocity]                    
        if self.save_data[1]==True:        
        
            self.file_name1=self.results_dir+'/bot_velocity.csv'

            with open(self.file_name1, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvb.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yvb.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zvb.items():
                    w.writerow([key, *val])                      
 # In[Forces]
        if self.save_data[2]==True:
         
            self.file_name2=self.results_dir+'/bot_TotalForces.csv'

            with open(self.file_name2, 'w', newline='') as fout:
                w = csv.writer(fout)
                    # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.Faxb.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.Fayb.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.Fazb.items():
                    w.writerow([key, *val])
                
# In[Controller force]
        if self.save_data[3]==True:
            
            self.file_name3=self.results_dir+'/Force_controller.csv'

            with open(self.file_name3, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.Faxc.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.Fayc.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.Fazc.items():
                    w.writerow([key, *val])
             
# In[Error]
        if self.save_data[4]==True:
            self.file_name4=self.results_dir+'/Error.csv'    
            savetxt(self.file_name4,self.E, delimiter=',')                
            

# In[Spring force]
        if self.save_data[5]==True:
            
            self.file_name5=self.results_dir+'/Spring_properties.csv' 
       
            with open(self.file_name5, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                    
                for key, val in self.Spring_force.items():
                    w.writerow([key,*val])   
                for key, val in self.Spring_length.items():
                    w.writerow([key,*val])                       
# In[Contact points]
        if self.save_data[6]==True:
            # contact points x
            self.file_name6=self.results_dir+'/x contact points.csv' 
            savetxt(self.file_name6,self.xc, delimiter=',')
        
            # contact points y
            self.file_name7=self.results_dir+'/y contact points.csv' 
            savetxt(self.file_name7,self.yc, delimiter=',')
        
            # contact points z
            self.file_name8=self.results_dir+'/z contact points.csv' 
            savetxt(self.file_name8,self.zc, delimiter=',')
        
            # contact force x
            self.file_name9=self.results_dir+'/x contact force.csv' 
     
            savetxt(self.file_name9,self.Fcx, delimiter=',')
        
            # contact force y
            self.file_name10=self.results_dir+'/y contact force.csv' 
            savetxt(self.file_name10,self.Fcy, delimiter=',')
        
            # contact force z
            self.file_name11=self.results_dir+'/z contact force.csv' 
            savetxt(self.file_name11,self.Fcz, delimiter=',')
        
        # number of contacts
            self.file_name12=self.results_dir+'/nc.csv' 
            savetxt(self.file_name12,self.nc, delimiter=',')

# In[desired shape]            


        if self.save_data[7]==True:
            self.file_name13=self.results_dir+'/particle_position.csv'

            with open(self.file_name13, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xp.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yp.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zp.items():
                    w.writerow([key, *val])  
# In[Particle Velocity]                    
        if self.save_data[8]==True:
            self.file_name14=self.results_dir+'/particle_velocity.csv'

            with open(self.file_name14, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvp.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yvp.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zvp.items():
                    w.writerow([key, *val])
                    
        self.file_name15=self.results_dir+'/ballx.csv' 
        savetxt(self.file_name15,self.ballx, delimiter=',')
        
        self.file_name16=self.results_dir+'/ballz.csv' 
        savetxt(self.file_name16,self.ballz, delimiter=',')
# In[save Variables]                    
    def save_variables(self):
        self.file_name17=self.results_dir+'/variables.csv'
        with open(self.file_name17, 'w', newline='') as fout:
            w = csv.writer(fout)
            w.writerow(['nb', self.nb])
            w.writerow(['n', self.interior.n])
            w.writerow(['R', self.bots.R])
            w.writerow(['mr', self.mr])
            w.writerow(['ni', self.interior.ni])
            w.writerow(['mp', self.mp])
            w.writerow(['alpha', self.controller.alpha])
            w.writerow(['beta', self.controller.beta])
            w.writerow(['k', self.bots.k])
            w.writerow(['rl', self.bots.rl])
            w.writerow(['rlmax', self.controller.rlmax])
            w.writerow(['OR', self.shape.OR,self.shape.OR/self.bots.R])
            w.writerow(['IR',self.shape.IR, self.shape.IR/self.bots.R])
            w.writerow(['epsilon', self.shape.epsilon,self.shape.epsilon/self.bots.R])
            w.writerow(['theta0', self.shape.theta0])
            w.writerow(['theta1', self.shape.theta1])
            w.writerow(['theta2', self.shape.theta2])
            w.writerow(['theta3', self.shape.theta3])
            w.writerow(['tend', self.tend])
            
            
        
             