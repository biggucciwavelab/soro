""""
robots.py: 
    
This file contains the classes and defs which
define the individual robots and their controllers

Author: Qiyuan Zhou
Date (MM/DD/YY): 09/10/20
Wavelab | Illinois Institute of Technology
"""
import numpy as np
import pychrono as chrono

class robot:
    def __init__(self,system=None,xyz=None,mat=None,actu=None,sens=None,config=None):
    
        self.cf=config                      # Configuration class
        self.actu=actu                      # robot is actuated? Bool
        self.sens=sens                      # robot has sensors? Bool
        self.coord_rot=0.0                  # [rads] Rotation from local to global coordinate frame
        self.local_rot=0.0                  # [rads] Rotation from local frame to obstacle frame
        self.angles=[None,None]             # L/R angles
        self.distE=[None,None]              # L/R Estimated spring distance
        self.distR=[None,None]              # L/R Reference spring distance
        self.diam=self.cf.rob_d             # diameter
        self.height=self.cf.rob_h           # height
        self.rho=self.cf.rob_rho            # density
        
        # Sensor measurements
        self.distance=[]                    # Center Distance to each object in the object array
        self.contact=[]                     # Boolean if object is within contact radius, minimum distance to object otherwise
        self.tension=[]                     # Tension in left and right springs of robot
        self.theta=xyz[3]                   # Angle of rotation
        
        # IMU
        self.IMU=None
        self.noise = np.zeros((3,2))
        
        # Kalman Filtering object
        self.kalman=None
        
        # Controller reference things
        self.ref_dist=[]                    # Reference distance between bots
        
        # Communicated/calculated data
        self.pos_est = np.asarray([xyz[0],xyz[2]])
        self.local_pos=np.asarray([xyz[0],xyz[2]])      # Position in local coordinate system
        self.local_vel=np.zeros(2)                      # Velocity in local coordinates
        self.rel_effort=-0.1                            # Relative effort compared to system average
        self.energy=0.0                                 # Energy usage
        self.coord_diff=np.zeros(self.cf.rob_nb-1)      # Position difference from other robots
        self.normal=np.array((1,0))                     # Formation normal vectors (local coordinates)
        
        # Chrono simulation specific inits
        if config.visual!='exp':
            # Initialization parameters
            self.system=system          # Simulation system, ChSystem
            self.xyz=xyz                # Initial position and rotation, [posx,posy,posz,theta]
            self.mat=mat                # surface material, ChMaterial
            
            # Body parameters and objects
            self.body=None              # Chrono object, ChBody
            self.linkL=None             # Link 1, ChLinkLock
            self.mem=[[None,None],      # Membrane particles, ((L/R),(Close/Far))
                      [None,None]]
            self.linkR=None             # Link 2, ChLinkLock
            self.springs=[]             # List containing springs this robot can actuate
            self.forcex=None            # ChForce object to apply robot's locomotion force in local x direction
            self.forcey=None            # ChForce object to apply robot's locomotion force in local x direction
            
            self.ID=None                # ID of robot
            self.tower=False            # Is a triangulation tower for local position sensing
            self.active=None            # Is this an active (True) or passive (False) unit?
                        
            # Create the ChBody
            self.create_bot()
            
        # Experimental data specific inits
        elif config.visual=='exp':
            a=1
    
    def create_bot(self):                   # Create the ChBody and ChForce
        self.body = chrono.ChBodyEasyCylinder(self.diam/2,self.height,self.rho,True,True)
        if self.cf.rob_m>0:
            self.body.SetMass(self.cf.rob_m)
        self.body.SetPos(chrono.ChVectorD(self.xyz[0],self.xyz[1],self.xyz[2]))
        self.body.SetMaterialSurface(self.mat)
        rotation = chrono.ChQuaternionD()
        rotation.Q_from_AngAxis(self.xyz[3], chrono.ChVectorD(0, 1, 0)) 
        self.body.SetRot(rotation)
        
        # Add control forces
        self.forcex = chrono.ChForce()
        self.body.AddForce(self.forcex)
        self.forcex.SetMode(chrono.ChForce.FORCE)
        self.forcex.SetDir(chrono.ChVectorD(1,0,0))
        self.forcex.SetMforce(0)
        
        self.forcey = chrono.ChForce()
        self.body.AddForce(self.forcey)
        self.forcey.SetMode(chrono.ChForce.FORCE)
        self.forcey.SetDir(chrono.ChVectorD(0,0,1))
        self.forcey.SetMforce(0)
        
        # Add visual texture assets
        if False:
            visual=chrono.ChAssetLevel()
            self.body.AddAsset(visual)
            texture=chrono.ChTexture()
            texture.SetTextureFilename('cad/plastic.jpg')
            rob_obj='cad/chonker.obj' 
            rob_vis = chrono.ChObjShapeFile() 
            rob_vis.SetStatic(True)
            rob_vis.SetFilename(rob_obj) 
            visual.AddAsset(rob_vis)
            visual.AddAsset(texture)
        
        if(self.ID==0): self.body.AddAsset(self.cf.col_p)
        
        # Add to system
        self.system.Add(self.body)     
           
    def init_sens_kalman(self,noise,X0):  # Initialize the IMU and Kalman fitering objects
        self.IMU=imu(noise)
        self.kalman=kalman(X0,self.cf)
        
    def get_dist(self,objects,rads):   # Distance sensor
        # rads is an array containing the radius of measurement objects
        self.distance=[]
        for i in range(len(objects)):
            if i==self.ID:
                self.distance.append(0.0); self.contact.append(0)
                continue
            xdist=self.body.GetPos().x-objects[i].body.GetPos().x
            ydist=self.body.GetPos().y-objects[i].body.GetPos().y
            zdist=self.body.GetPos().z-objects[i].body.GetPos().z
            cent_dist=(xdist**2+ydist**2+zdist**2)**0.5
            self.distance.append(cent_dist)
            
    def get_dist_expr(self,xys):                # Distance and estimated position relative to neighbors from AprilTag data
        ii = self.ID + 1    
        if(self.ID==self.cf.rob_nb-1):ii = 0
        
        est1 = np.zeros(2)
        est2 = np.zeros(2)
        
        xdist1 = xys[0,self.ID] - xys[0,self.ID-1]
        xdist2 = xys[0,self.ID] - xys[0,ii]
        ydist1 = xys[1,self.ID] - xys[1,self.ID-1]
        ydist2 = xys[1,self.ID] - xys[1,ii]
        self.distance = [[xdist1, ydist1],
                         [xdist2, ydist2]]
        
        est1[0] = xys[0,self.ID-1] + xdist1
        est1[1] = xys[1,self.ID-1] + ydist1
        est2[0] = xys[0,ii] + xdist2
        est2[1] = xys[1,ii] + ydist2
        
        self.pos_est = 0.5 * (est1 + est2)
                
    def get_tension(self):                  # Tension sensor
        # left=np.linalg.norm((self.linkL.Get_react_force().x,\
        #                      self.linkL.Get_react_force().y,\
        #                      self.linkL.Get_react_force().z))
        # right=np.linalg.norm((self.linkR.Get_react_force().x,\
        #                       self.linkR.Get_react_force().y,\
        #                       self.linkR.Get_react_force().z))
        
        left=np.linalg.norm((self.linkL.Get_react_force().x,\
                             self.linkL.Get_react_force().z))
        right=np.linalg.norm((self.linkR.Get_react_force().x,\
                              self.linkR.Get_react_force().z))
        self.tension=[left, right]
        
    def get_angle(self):                    # String angle sensor
    
        # Calculate angle of string relative to global coordinates
        theta = self.theta
        
        posR_f = np.asarray([self.mem[1][1].GetPos().x,0,self.mem[1][1].GetPos().z])
        posR_n = np.asarray([self.mem[1][0].GetPos().x,0,self.mem[1][0].GetPos().z])
        posL_f = np.asarray([self.mem[0][1].GetPos().x,0,self.mem[0][1].GetPos().z])
        posL_n = np.asarray([self.mem[0][0].GetPos().x,0,self.mem[0][0].GetPos().z])
        
        left = posL_f-posL_n
        right = posR_f-posR_n
        
        left = np.arctan2(left[2],left[0])
        right = np.arctan2(right[2],right[0])
        
        self.angles = [left,right]
        
    def get_loc(self,left,right):                      # Estimate current location from neighbors
        # Left and right are the robot class object associated with neighbor bots
        
        ## Left neighbor
        # Left neighbor location
        locL = left.pos_est
        
        # Center of neighbor to far left 
        farL = np.asarray([self.mem[0][1].GetPos().x,self.mem[0][1].GetPos().z]) - locL
        
        # Far left to near left
        t_l = (self.tension[0]/(self.cf.km)) * np.asarray([np.cos(self.angles[0]),np.sin(self.angles[0])])
        # t_l = np.linalg.norm([self.mem[0][1].GetPos().x-self.mem[0][0].GetPos().x,
        #                       self.mem[0][1].GetPos().z-self.mem[0][0].GetPos().z]) \
        #                         * np.asarray([np.cos(self.angles[0]),np.sin(self.angles[0])])
        
        # near left to center of this bot
        nearL = np.asarray([self.body.GetPos().x,self.body.GetPos().z])\
              - np.asarray([self.mem[0][1].GetPos().x,self.mem[0][1].GetPos().z]) 
        
        ## Right neighbor
        locR = right.pos_est
        
        # Center of neighbor to far right 
        farR = np.asarray([self.mem[1][1].GetPos().x,self.mem[1][1].GetPos().z]) - locR
        
        # Far right to near right
        t_r = (self.tension[1]/(self.cf.km)) * np.asarray([np.cos(self.angles[1]),np.sin(self.angles[1])])
        # t_r = np.linalg.norm([self.mem[1][1].GetPos().x-self.mem[1][0].GetPos().x,
        #                       self.mem[1][1].GetPos().z-self.mem[1][0].GetPos().z]) \
        #                         * np.asarray([np.cos(self.angles[1]),np.sin(self.angles[1])])
        
        # near right to center of this bot
        nearR = np.asarray([self.body.GetPos().x,self.body.GetPos().z])\
              - np.asarray([self.mem[1][1].GetPos().x,self.mem[1][1].GetPos().z]) 
              
        # distL = np.linalg.norm([self.mem[0][1].GetPos().x-self.mem[0][0].GetPos().x,
        #                       self.mem[0][1].GetPos().z-self.mem[0][0].GetPos().z])
        # distR = np.linalg.norm([self.mem[1][1].GetPos().x-self.mem[1][0].GetPos().x,
        #                       self.mem[1][1].GetPos().z-self.mem[1][0].GetPos().z])
        # self.distE=[distL,distR]
        
        self.distE=[self.tension[0]/(self.cf.km), self.tension[1]/(self.cf.km)]
        
        # Reference Distances
        self.distR[0]=np.linalg.norm([self.mem[0][1].GetPos().x-self.mem[0][0].GetPos().x,
                                      self.mem[0][1].GetPos().z-self.mem[0][0].GetPos().z])
        self.distR[1]=np.linalg.norm([self.mem[1][1].GetPos().x-self.mem[1][0].GetPos().x,
                                      self.mem[1][1].GetPos().z-self.mem[1][0].GetPos().z])
        est1 = locL + farL + t_l + nearL
        est2 = locR + farR + t_r + nearR
        self.pos_est = 0.5*(est1+est2)
        
    def loc_chain(self,left,right):
        a=1
        
    def calc_normal(self,pos):              # Calculate skin outward normal direction in local frame
        # Pos is nb x 2 array of positions in the order of robot IDs
        
        index=self.ID-1
        vect1=pos[self.ID-1]-self.local_pos     # position vector from bot -1 to bot
        self.tangent = vect1/np.linalg.norm(vect1)
        vect2=pos[index]-self.local_pos     # position vector from bot +1 to bot
        vect1=np.cross([vect1[0],0,vect1[1]],[0,1,0])
        vect2=np.cross([vect2[0],0,vect2[1]],[0,1,0])
        self.normal = (vect1+vect2)/2
        self.normal = self.normal[0:3:2]/np.linalg.norm(self.normal)
        
    def save_sens(self):                    # Save sensor ground truth and measurement
        m = self.IMU.measurement
        x_truth = self.body.GetPos_dtdt().x
        z_truth = self.body.GetPos_dtdt().z
        t_truth = self.body.GetWvel_par().y
        x_sens = self.IMU.measurement[4]
        z_sens = self.IMU.measurement[5]
        t_sens = self.quat2eulerV(m[0],m[1],m[2],m[3])
        return x_truth,z_truth,t_truth,x_sens,z_sens,t_sens
    
    def save_pos(self):                     # Save positions
        x=self.body.GetPos().x
        y=self.body.GetPos().y
        z=self.body.GetPos().z
        return x,y,z
    
    def save_vel(self):                     # Save velocities
        x=self.body.GetPos_dt().x
        y=self.body.GetPos_dt().y
        z=self.body.GetPos_dt().z
        return x,y,z
    
    def save_control(self):                 # Save control inputs
        x1=self.forcex.GetForce().x
        z1=self.forcex.GetForce().z
        x2=self.forcey.GetForce().x
        z2=self.forcey.GetForce().z
        return x1+x2,z1+z2
    
    # Convert quaternion to euler angle about y axis
    def quat2eulerCh(self,quat):
        return quat.Q_to_Rotv().y
    
    def quat2eulerV(self,x,y,z,w):
        quat = chrono.ChQuaternionD(x,y,z,w)
        
        return quat.Q_to_Rotv().y
            
    # Low level: set the force in a spring to a certain value
    def set_spring_force(self, mag):
        for spring in self.springs:
            spring.SetActuatorForce(mag)
    
    # Low level: set the magnitude and direction of locomotion force
    def set_loco_force(self,magx,magy):
        self.forcex.SetMforce(magx)
        self.forcey.SetMforce(magy)
    
    # Move in a pre-defined direction in global frame
    def user_move(self,direction,mag):
        self.forcey.SetDir(chrono.ChVectorD(direction[0],0,direction[1]))
        self.forcey.SetMforce(mag)
        
class imu:
    def __init__(self, noise_params):
        ''''
        noise_params is a 10x2 matrix as defined:
            
        Rows:
            Gyro params:
            0:3 - Quaternion components: e0,e1,e2,e3
                
            Accel params:
            4:5 - x,z
            
            Magnetometer
            6:9 - Quaternion components: e0,e1,e2,e3
            
        Columns:
            0: STD for gaussian noise
            1: Random walk
        '''
        self.noise = noise_params
        self.prev_noise = np.zeros(10)
        self.initial = np.zeros(10)
        self.measurement = np.zeros(10)
        
    def get_measurement(self, state):
        '''
        state is a vector which contains the ground truth
        measurements to be convoluted by the virtual IMU
        '''
        # Convert over from quaternions
        stateU = np.zeros(10)
        stateU[0]=state[0].e0
        stateU[1]=state[0].e1
        stateU[2]=state[0].e2
        stateU[3]=state[0].e3
        stateU[4]=state[1]
        stateU[5]=state[2]
        stateU[6]=state[3].e0
        stateU[7]=state[3].e1
        stateU[8]=state[3].e2
        stateU[9]=state[3].e3
        state=stateU
        
        # Add gaussian noise
        state += np.random.normal(scale=self.noise[:,0], size=10)
        
        # Add random walk        
        self.prev_noise += np.random.normal(scale=self.noise[:,1], size=10)
        
        # Update state
        self.measurement = state
        
        return state
    
class kalman:
    def __init__(self,X0,config):
        # i is the number of measurements we make (# of sensors)
        self.cf=config
        self.X0 = X0                # Previous state
        self.X = X0                 # Predicted state
        self.H = self.cf.H               # Measurement transform matrix
        self.R = self.cf.R               # Measurememnt covariance matrix
        self.A = self.cf.A               # State transform matrix
        self.Q = self.cf.Q               # Prediction covariance matrix
        self.P = self.cf.P               # P matrix
        self.K = None               # Kalman gain matrix
        self.S = None               # S matrix in update step
        self.IM = None              # IM matrix in update step
        self.LH = None              # LH matrix in update step
        
    # Defs modified from https://arxiv.org/ftp/arxiv/papers/1204/1204.0375.pdf   
    # Predict mean X and covariance P at time step k
    def predict(self,U):
        self.X = np.matmul(self.A, self.X) + np.matmul(self.cf.B, U)
        #if abs(U[0])>50: self.X[1]=0
        #if abs(U[1])>50: self.X[3]=0
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q
    
    # Update mean X and covariance P returning predicted X, P, measurememnt vector Y, 
    # measurement matrix H and measurement covariance matrix R
    def kf_update(self, Y):
        self.IM = np.matmul(self.H, self.X)
        self.S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        self.K = np.matmul(self.P, np.matmul(self.H.T, np.linalg.inv(self.S)))
        self.X += np.matmul(self.K, (Y-self.IM))
        self.P -= np.matmul(self.K, np.matmul(self.S, self.K.T))
        self.LH = self.gauss_pdf(Y, self.IM, self.S)
    
    # Compute the predictive probabitilty during kf_update
    def gauss_pdf(self, X, M, S):
        DX = X - np.tile(M, X.shape[1])
        E = 0.5 * np.sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
        P = np.exp(-E)
        return (P[0],E[0])