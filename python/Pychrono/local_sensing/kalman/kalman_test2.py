import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

class imu:
    def __init__(self, initial, noise_params):
        ''''
        noise_params is a 9x2 matrix whose rows are the following:
            Gyro params:
            0: [rad] STD: Gaussian noise about ground truth
            1: [rad/rt-hr] random walk
                
            Accel params:
            2: [m/s^2] STD: Accel noise about ground truth
            3: [m/s/rt-hr] random walk
            
            Magnetometer
            4: [uT] std: mangetometer noise about ground truth
        '''
        self.noise = noise_params
        self.initial =     np.asarray([[0,0],     # rad/s^2: gyro
                                       [0,0],     # m/s^2:   accelerometer
                                       [0,0]])    # uT:      magnetometer
        self.measurement = np.asarray([[0,0],     # rad/s^2: gyro
                                       [0,0],     # m/s^2:   accelerometer
                                       [0,0]])    # uT:      magnetometer
        
    def get_measurement(self, state, prev_noise, time, tstep):
        '''
        input_state is a vector which contains the ground truth
        measurements to be convoluted by the virtual IMU
            Gyro:
            0-2: [rad/s] x,y,z angular velocities
            3-5: [m/s^2] x,y,z linear accelerations
            6-8: [uT]    x,y,z magnetometer measurements
        '''
        out = state.copy()
        walk = prev_noise
        
        # Add gaussian noise
        out[0,:] += np.random.normal(scale=self.noise[0,:],size=2)
        out[1,:] += np.random.normal(scale=self.noise[2,:],size=2)
        out[2,:] += np.random.normal(scale=self.noise[4,:],size=2)
        
        # Add random walk        
        walk[0,:] = prev_noise[0,:] + np.random.normal(scale=self.noise[1,:],size=2)
        walk[1,:] = prev_noise[1,:] + np.random.normal(scale=self.noise[3,:],size=2)
        out[0,:] += walk[0,:]
        out[1,:] += walk[1,:]
        
        self.measurement = out
        return walk, out
        
class kalman:
    def __init__(self,X0,i):
        # i is the number of measurements we make (# of sensors)
        
        j=len(X0)
        self.X0 = X0                # Previous state
        self.X = X0                 # Predicted state
        self.H = np.zeros((i,j))    # Measurement transform matrix
        self.R = np.zeros((i,i))    # Measurememnt covariance matrix
        self.A = np.zeros((j,j))    # State transform matrix
        self.Q = np.zeros((j,j))    # Prediction covariance matrix
        self.P = np.zeros((j,j))    # P matrix
        
        # Defs modified from https://arxiv.org/ftp/arxiv/papers/1204/1204.0375.pdf
        
    # Predict mean X and covariance P at time step k
    def predict(self, X, P, A, Q, B, U):
        X = np.matmul(A, X) + np.matmul(B, U)
        P = np.dot(A, np.dot(P, A.T)) + Q
        self.X=X; self.P=P
        return(X,P)
    
    # Update mean X and covariance P returning predicted X, P, measurememnt vector Y, 
    # measurement matrix H and measurement covariance matrix R
    def kf_update(self,X, P, Y, H, R):
        IM = np.dot(H, X)
        S = R + np.dot(H, np.dot(P, H.T))
        K = np.dot(P, np.dot(H.T, np.linalg.inv(S)))
        X = X + np.dot(K, (Y-IM))
        P = P - np.dot(K, np.dot(S, K.T))
        LH = self.gauss_pdf(Y, IM, S)
        return (X,P,K,IM,S,LH)
    
    # Compute the predictive probabitilty during kf_update
    def gauss_pdf(self, X, M, S):
        if M.shape[1] == 1:
            DX = X - np.tile(M, X.shape[1])
            E = 0.5 * np.sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
            P = np.exp(-E)
        elif X.shape[1] == 1:
            DX = np.tile(X, M.shape()[1])- M
            E = 0.5 * np.sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
            E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
            P = np.exp(-E)
        else:
            DX = X-M
            E = 0.5 * np.dot(DX.T, np.dot(np.linalg.inv(S), DX))
            E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
            P = np.exp(-E)
        return (P[0],E[0])
        
# %% Initial conditions and noise parameters
initial = np.asarray([[0,0],     # rad/s^2: gyro
                      [0,0],     # m/s^2:   accelerometer
                      [0,0]])    # uT:      magnetometer

noise_params = np.asarray([[0.01,0.01],
                           [0.00,0.0],
                           [0.1,0.1],
                           [0.01, 0.01],
                           [100, 100]])

imu_obj = imu(initial,noise_params)

# %% Process / Estimation Errors
dt=1.0
C=np.asarray([[1,dt],
              [0,1]])
D=np.asarray([[1,dt],
              [0,1]])

B=np.asarray([[0.5*dt*dt,0.0],
              [dt,0.0],
              [0,0.5*dt*dt],
              [0.0,dt],
              [0.0,0.0],
              [0.0,0.0]])

# %% A state transform matrix
A=np.zeros((6,6))
A[0:2,0:2]=C
A[2:4,2:4]=C
A[4:6,4:6]=D

# State vector
X=np.zeros((6,1))

# %% Observation Transformation
H=np.zeros((2,6))
H[0,0]=1
H[1,2]=1

# %% Observation errors
sig1=10
sig2=10
R=np.diag([sig1,sig2])

# %% Initial Estimation Covariance Matrix
sig1=1e-1
sig2=1e-1
sig3=1e-1
sig4=1e-1
sig5=1e-1
sig6=1e-1
covar=[sig1,sig2,sig3,sig4,sig5,sig6]
Q=np.zeros((6,6))
P=np.diag(covar)
for ii in range(6):
    for jj in range(6):
        Q[ii,jj]=covar[ii]*covar[jj]

# %% Initialize the Kalman filter object
U=np.asarray([[0],[0]])
kal_obj = kalman(X,2)

noise = np.asarray([[0.,0.],
                    [0.,0.]])

states=[[[],[]],
        [[],[]],
        [[],[]]]
outputs=[[[],[]],
         [[],[]],
         [[],[]]]

predicts=[[[],[]],
          [[],[]],
          [[],[]]]

# %% Simulate the filter
for i in range(1000):
    
    # Make the prediction
    # returns pred=(X,P)
    pred = kal_obj.predict(X, P, A, Q, B, U)
    X=pred[0]
    P=pred[1]
    # Update the physics
    if True:
        state = np.asarray([[-20*np.cos(i/20),-20*np.cos(i/20)],     # rad/s^2: gyro
                            [np.sin(i/20),np.sin(i/20)],     # m/s^2:   accelerometer
                            [-400*np.sin(i/20),-400*np.sin(i/20)]])    # position
    
    if False:
        state = np.asarray([[0.1,0.1],     # rad/s^2: gyro
                            [0.1,0.1],     # m/s^2:   accelerometer
                            [0.05*i*i,0.05*i*i]])    # position
    
    noise,output = imu_obj.get_measurement(state,noise,float(i),float(1))
    
    U=np.asarray([[output[1,0]],[output[1,1]]])
    
    # Store data
    for j in range(3):
        for k in range(2):
            states[j][k].append(state[j,k])
            outputs[j][k].append(output[j,k])
    
    # Make the measurement and update
    # returns update=(X,P,K,IM,S,LH)
    x=np.zeros((6,1))
    if True:
        x[0,0]=output[2,0] # x pos
        x[2,0]=output[2,1] # y pos
        
    if False:
        x[0,0]=X[0]+dt*state[0,0]+dt*dt*0.5*state[1,0] # x pos
        x[2,0]=X[2]+dt*state[0,1]+dt*dt*0.5*state[1,1] # y pos
        
    x[5,0]=0          # orientation
    Y=np.matmul(H,x)
    update = kal_obj.kf_update(X, P, Y, H, R)

    # Update matrices
    X=update[0]
    P=update[1]
    
    # Store data
    predicts[0][0].append(X[0]) # x pos
    predicts[0][1].append(X[2]) # y pos
    predicts[1][0].append(X[1]) # x vel
    predicts[1][1].append(X[3]) # y vel
    
# %% plot data
t=np.linspace(0.0,100,1000)
titles=['X','Y']
for i in range(2):
    plt.figure()
    plt.title(titles[i])
    
    # Accelerations
    plt.subplot(211)
    plt.plot(t,states[1][i],t,outputs[1][i])
    
    # Positions
    plt.subplot(212)
    plt.plot(t,states[2][i],t,outputs[2][i],t,predicts[0][i])
        
    
# %% Simulate the filter in open loop
if False:
    states=[[[],[]],
            [[],[]],
            [[],[]]]
    outputs=[[[],[]],
             [[],[]],
             [[],[]]]
    
    predicts=[[[],[]],
              [[],[]],
              [[],[]]]
    # X[1]=0
    # X[3]=0
    for i in range(1000):
        # Make the prediction
        # returns pred=(X,P)
        pred = kal_obj.predict(X, P, A, Q, B, U)
        X=pred[0]
        P=pred[1]
        # Update the physics
        if True:
            state = np.asarray([[-20*np.cos(i/20),-20*np.cos(i/20)],     # rad/s^2: gyro
                                [np.sin(i/20),np.sin(i/20)],     # m/s^2:   accelerometer
                                [-400*np.sin(i/20),-400*np.sin(i/20)]])    # position
        
        if False:
            state = np.asarray([[0.1,0.1],     # rad/s^2: gyro
                                [0.1,0.1],     # m/s^2:   accelerometer
                                [0.05*i*i,0.05*i*i]])    # position
        
        noise,output = imu_obj.get_measurement(state,noise,float(i),float(1))
        
        U=np.asarray([[state[1,0]],[state[1,1]]])
        
        # Store data
        for j in range(3):
            for k in range(2):
                states[j][k].append(state[j,k])
                outputs[j][k].append(output[j,k])
        
        # Make the measurement and update
        # returns update=(X,P,K,IM,S,LH)
        # x=np.zeros((6,1))
        # if True:
        #     x[0,0]=state[2,0] # x pos
        #     x[2,0]=state[2,1] # y pos
            
        # if False:
        #     x[0,0]=X[0]+dt*state[0,0]+dt*dt*0.5*state[1,0] # x pos
        #     x[2,0]=X[2]+dt*state[0,1]+dt*dt*0.5*state[1,1] # y pos
            
        # x[5,0]=0          # orientation
        # Y=np.matmul(H,x)
        # update = kal_obj.kf_update(X, P, Y, H, R)
    
        # # Update matrices
        # X=update[0]
        # P=update[1]
        
        # Store data
        predicts[0][0].append(X[0]) # x pos
        predicts[0][1].append(X[2]) # y pos
        predicts[1][0].append(X[1]) # x vel
        predicts[1][1].append(X[3]) # y vel
        
    # %% plot data
    t=np.linspace(0.0,100,1000)
    titles=['X','Y']
    for i in range(2):
        plt.figure()
        plt.title(titles[i])
        
        # Accelerations
        plt.subplot(211)
        plt.plot(t,states[1][i],t,outputs[1][i])
        
        # Positions
        plt.subplot(212)
        plt.plot(t,states[2][i],t,outputs[2][i],t,predicts[0][i])
        