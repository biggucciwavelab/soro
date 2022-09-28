% Adaptive diff drive pose controller

%% Set up robot parameters
m = 0.1;         % mass
b = 0.07324/2;  % 1/2 wheelbase
r=0.16;          % wheel radius
Ic = 0.00008843;      % Moment of inertia of body about Pc
Iw = 0.00000051;      % Moment of inertia for wheel
Im = 0.00000765;      % Moment of inertia of wheels about com
mc = 0.5;       % Robot mass
mw = 0.003218;  % Wheel mass
d = 0;          % Distance between robot origin and robot COM
tau_dr = 0;     % Disturbance term right
tau_dl = 0;     % Disturbance term left
I = mc*d^2+Ic+2*mw*b^2+2*Im;       % Moment of inertia for everything

%% Kinematic and Dynamic Adaptive Pose Controller
% Initial pose
x0=0; y0=0; theta0=0;
z=[x0;y0;theta0];
z_des=z;

% Initial velocity
v0=1; omega0=0;
zdot=[v0*cos(theta0);v0*sin(theta0);omega0];
zdot_des=zdot;

% Target location
xfin=1; yfin=1; thetafin=0*pi/6;
zr=[xfin;yfin;thetafin];

% Reference velocities
vr=1;

% Time
dt=0.1;
time3=0:dt:norm([xfin-x0 yfin-y0])/(vr);
%time3=0:dt:3;

% Initialize empty matrices to store things later
pos=zeros(3,1+1/dt);
vel=zeros(2,1+1/dt);
pos_des=zeros(3,1+1/dt);
vel_des=zeros(2,1+1/dt);
poserrs3=zeros(3,1+1/dt);

% Pose Gains
k1=1;
k2=10;

% Adaptive Gains and weights
k1_a=1; k2_a=1;
k_a=[k1_a 0;0 k2_a];
lambda = 5*eye(2);
a_hat = [0; 0; 0; 0; 0];
P = eye(5);

% Desired Wheel velocities
l_wv=v0/r-theta0*b/r; r_wv=v0/r+theta0*b/r;
l_wvs=zeros(1,1+1/dt);
r_wvs=zeros(1,1+1/dt);

% Desired Wheel accelerations
l_was=zeros(1,1+1/dt);
r_was=zeros(1,1+1/dt);

% Desired Wheel positions
l_wp=0; r_wp=0;
l_wps=zeros(1,1+1/dt);
r_wps=zeros(1,1+1/dt);

% Actual Wheel velocities
l_wv_a=v0/r-theta0*b/r; r_wv_a=v0/r+theta0*b/r;
l_wvs_a=zeros(1,1+1/dt);
r_wvs_a=zeros(1,1+1/dt);
vdot=[r_wvs_a;l_wvs_a];

% Actual Wheel accelerations
l_was_a=zeros(1,1+1/dt);
r_was_a=zeros(1,1+1/dt);

% Actual Wheel positions
l_wp_a=0; r_wp_a=0;
l_wps_a=zeros(1,1+1/dt);
r_wps_a=zeros(1,1+1/dt);
v=[r_wps_a;l_wps_a];

% Iterate over time
for t = time3
    
    % Convert current position to local coordinate frame
    r = norm(zr-z); % Vector from origin of robot to target location
    delta = z(3)- atan2((zr(2)-z(2)),zr(1)-z(1));   % Angle from vector r to heading angle
    theta = zr(3)- atan2((zr(2)-z(2)),zr(1)-z(1));  % Angle from vector r to target angle
    
    % Store actual positions, velocities
    pos(:,int16(1+t/dt))=z;
    vel(:,int16(1+t/dt))=zdot(1:2);
    
    % Store desired positions, velocities
    pos_des(:,int16(1+t/dt))=z_des;
    vel_des(:,int16(1+t/dt))=zdot_des(1:2);
    
    %------------------------------------------------
    % Pose Controller
    %------------------------------------------------
    % Add pose control law
    omega = (-vr/r)*(k2*(delta-atan(-k1*theta))+(1+(k1/(1+(k1*theta)^2)))*sin(delta));
    u=[vr;omega];
    
    % Desired Wheel velocities
    l_wv_p = l_wv; r_wv_p = r_wv;   % Previous velocities
    l_wv = u(1)/r-u(2)*b/r; l_wvs(:,int16(1+t/dt))=l_wv;
    r_wv = u(1)/r+u(2)*b/r; r_wvs(:,int16(1+t/dt))=r_wv;
    
    % Desired Wheel accelerations
    l_wa = (l_wv-l_wv_p)/dt; l_was(:,int16(1+t/dt))=l_wa;
    r_wa = (r_wv-r_wv_p)/dt; r_was(:,int16(1+t/dt))=r_wa;
    
    % Desired Wheel positions
    l_wp = l_wp+l_wv*dt; l_wps(:,int16(1+t/dt))= l_wp;
    r_wp = r_wp+r_wv*dt; r_wps(:,int16(1+t/dt))= r_wp;
    
    % Calculate zdot and integrate to find new desired pose
    zdot_des=[u(1)*cos(z_des(3));u(1)*sin(z_des(3));u(2)];
    z_des=z_des+zdot_des*dt; 
    
    %------------------------------------------------
    % TODO: Adaptive controller
    %------------------------------------------------
    aaa=(r^2/4*b^2);
    % M_bar matrix terms
    M11=aaa*(m*b^2+I)+Iw;
    M12=aaa*(m*b^2+I)+Iw;
    M_bar=[M11 M12;M12 M11];

    % Vm_bar matrix terms
    bbb=mc*d*z(3);
    Vm_bar=[0 aaa*bbb;-aaa*bbb 0];

    % D_bar vector terms
    D_bar = [tau_dr 0;0 tau_dl];
    
    % Error terms
    pos_err=[r_wp_a-r_wp; l_wp_a-l_wp];
    vel_err=[r_wv_a-r_wv; l_wv_a-l_wv];
    
    % v_r, v_r_dot, v_r_dotdot
    v_rdot=[r_wv;l_wv]-lambda*pos_err;
    v_rdotdot=[r_wa;l_wa]-lambda*vel_err;

    % Calculate Y terms
    y11=v_rdotdot(1); 
    y12=v_rdotdot(2);
    y13=z(3)*v_rdot(2);
    y14=v_rdot(1);
    y15=0;
    
    y21=v_rdotdot(2); 
    y22=v_rdotdot(1); 
    y23=-z(3)*v_rdot(1);
    y24=0; 
    y25=v_rdot(2);
    
    Y=[y11 y12 y13 y14 y15;
       y21 y22 y23 y24 y25];
    
   % Sliding terms
    s=[r_wv_a;l_wv_a]-v_rdot;
    a_hatdot=-P*Y'*s;
    a_hat=a_hat+a_hatdot*dt;
    
    % Torque terms
    T=Y*a_hat-k_a*s;
    %torque1(:,int16(1+t/dt))=T;
    
    vdotdot=M_bar\(T-(Vm_bar+D_bar)*[r_wv_a;l_wv_a]);
    vdot=vdot+dt*vdotdot;
    v=v+dt*vdot;
    
    %TODO: reassign r_wa, r_wv, r_wp from vdotdot, vdot, v
    r_wa_a= vdotdot(1); l_wa_a= vdotdot(2);
    r_wv_a= vdot(1);    l_wv_a= vdot(2);
    r_wp_a= v(1);       l_wp_a= v(2);
    
    %-------------------------------------------------------
    
    % Store pose errors
    poserrs3(:,int16(1+t/dt))=[xfin-z(1);yfin-z(2);thetafin-z(3)];
    poserrs3(3,int16(1+t/dt))=poserrs3(3,int16(1+t/dt))*180/pi;
    
    
end

% Actual positions of the robot
figure
hold on
title('Actual positions and orientations')
quiver(pos(1,:),pos(2,:),vel(1,:),vel(2,:),.5)
quiver(pos(1,:),pos(2,:),cos(pos(3,:)),sin(pos(3,:)),0.5)
scatter(pos(1,:),pos(2,:),'b')
legend('velocity','heading','robot position','location','best')
axis equal
xlabel('x position (m)')
ylabel('y position (m)')

% Desired positions of the robot
figure
hold on
title('Desired positions and orientations')
quiver(pos_des(1,:),pos_des(2,:),vel_des(1,:),vel_des(2,:),.5)
quiver(pos_des(1,:),pos_des(2,:),cos(pos_des(3,:)),sin(pos_des(3,:)),0.5)
scatter(pos_des(1,:),pos_des(2,:),'b')
legend('velocity','heading','robot position','location','best')
axis equal
xlabel('x position (m)')
ylabel('y position (m)')

% Left wheel position, velocity, acceleration
figure
suptitle('Left Wheel')
hold on
subplot(3,1,1)
hold on
title('Position')
plot(time3,l_wps,'b')
subplot(3,1,2)
hold on
title('Velocity')
plot(time3,l_wvs,'r')
subplot(3,1,3)
hold on
title('Acceleration')
xlabel('time (s)')
plot(time3,l_was,'g')

% Right wheel position, velocity, acceleration
figure
suptitle('Right Wheel')
hold on
subplot(3,1,1)
hold on
title('Position')
plot(time3,r_wps,'b')
subplot(3,1,2)
hold on
title('Velocity')
plot(time3,r_wvs,'r')
subplot(3,1,3)
hold on
title('Acceleration')
xlabel('time (s)')
plot(time3,r_was,'g')

% Time response of pose errors3
figure
suptitle('Time response error')
hold on
subplot(3,1,1)
hold on
title('x position error')
plot(time3,poserrs3(1,:),'b')
plot(time3,zeros(size(time3)),'b:')
ylabel('Error (m)')
subplot(3,1,2)
hold on
title('y position error')
plot(time3,poserrs3(2,:),'r')
plot(time3,zeros(size(time3)),'b:')
ylabel('Error (m)')
subplot(3,1,3)
hold on
title('theta error')
xlabel('time (s)')
ylabel('Error (deg)')
plot(time3,poserrs3(3,:),'g')
plot(time3,zeros(size(time3)),'b:')
