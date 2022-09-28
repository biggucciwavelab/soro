% Adaptive diff drive controller

%% Set up robot parameters
m = 0.1;         % mass
b = 0.07324/2;  % 1/2 wheelbase
r=0.16;          % wheel radius
Ic = 0.00008843;      % Moment of inertia of body about Pc
Iw = 0.00000051;      % Moment of inertia for wheel
Im = 0.00000765;      % Moment of inertia of wheels about com
mc = 0.5;       % Robot mass
d = 0;          % Distance between robot origin and robot COM
tau_dr = 0;     % Disturbance term right
tau_dl = 0;     % Disturbance term left
I = mc*d^2+Ic+2*mw*b^2+2*Im;       % Moment of inertia for everything

% Drive the robot in a circle
dt=0.001;
time=0:dt:10;
qfs=[cos(time); sin(time)];
qvs=[-sin(time); cos(time)];
qas=[-cos(time);-sin(time)];


% Loop over time