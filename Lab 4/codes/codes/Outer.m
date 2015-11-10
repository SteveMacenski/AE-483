function u=Outer(t,x,dt)

% persistent K1 K2 K3
%%% MEASUREMENT
x=MoCap(x); % full state [x;y;z;vx;vy;vz;theta_3;theta_2;theta_1;p;q;r]

% Set parameters
% param.katt = 1;
% param.batt = 1;
% param.dt = dt;
% param.kdescent = 1;
% param.bdescent = 1;
% % Set goal position
% goal.q = [2; 0; -1];
% obst = {};

%%% NOMINAL CONTROL - This is the baseline control which we used when we
% linearized the system
m=.6891;
g=9.81;
u_nom=[0;0;0;m*g];

%%% FEEDBACK CONTROL
% POSITION CONTROL
delta_theta3_des=0;
delta_theta2_des=0;
delta_u4=m*g;

%if isempty(K1)
%LQR CONTROLLER X-position
% dt = 0.01;
A=[0,1;0,0]; 
B=[0;-g];
Q=[.6,0;0,1];
R=5;

A = eye(2) + dt*A;
B = dt*B;
[P,E,K1]=dare(A,B,Q,R);


%LQR CONTROLLER Y-POSITION
% dt = 0.01;
A=[0,1;0,0]; 
B=[0;g];
Q=[0.8,0;0,0.5];
R=5;

A = eye(2) + dt*A;
B = dt*B;
[P,E,K2]=dare(A,B,Q,R);

%LQR CONTROLLER Z-position
% dt = 0.01;
A=[0,1;0,0]; 
B=[0;-1/m];
Q=[3,0;0,.05];
R=5;

A = eye(2) + dt*A;
B = dt*B;
[P,E,K3]=dare(A,B,Q,R);



%K1
%K2
%K3
%end
wp = planner([0,0,-1,0],t); %waypoint

x1=[x(1)-wp(1); x(4)];
delta_theta2_des=-K1*x1;

x2=[x(2)-wp(2); x(5)];
delta_theta3_des=-K2*x2;

x3=[x(3)-wp(3); x(6)];
delta_u4=-K3*x3;

% YAW CONTROL
delta_theta1_des=0;

delta_u=[delta_theta3_des;delta_theta2_des;delta_theta1_des;delta_u4];

%%% RETURN OUTER LOOP CONTROL VECTOR
u=u_nom + delta_u;