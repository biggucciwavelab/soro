% Adaptive diff drive controller

%% Testing

% Drive the robot in a circle

% Gains and weights
k1=500; k2=500;
k=[k1 0;0 k2];
lambda = 0.75*eye(2);
a_hat = [0; 0; 0; 0; 0];
P = eye(5);
% [m1 m2 I1, I2, g];

% Desired location, velocity as a function of time
dt=0.001;
time=0:dt:10;
qfs=[cos(time); sin(time)];
qvs=[-sin(time); cos(time)];
qas=[-cos(time);-sin(time)];

% Physical properties of robot


% Loop over time