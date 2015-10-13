function u=Outer(t,x,dt)

persistent int

% MEASUREMENT
x=MoCap(x); % full state now

% NOMINAL CONTROL  (theta_3_star, theta_2_star, theta_1_star, thrust_star)
u_nom=[0;0;0;9.81];

% FEEDBACK CONTROL
g = 9.81;
m = 1;
dt = .01;
% POSITION CONTROL TODO

A_q1 = eye(2) + dt.*[0 1;0 0]; %A matrices
A_q2 = eye(2) + dt.*[0 1;0 0];
A_q3 = eye(2) + dt.*[0 1;0 0];

B_q1 = dt*[0 ; -g]; %B matrices
B_q2 = dt*[0 ; g];
B_q3 = dt*[0 ; -1/m];

Q_q1 = eye(2);
Q_q2 = eye(2);
Q_q3 = eye(2);
R_q1 = 1;
R_q2 = 1;
R_q3 = 1;

[P,E,K_q1] = dare(A_q1,B_q1,Q_q1,R_q1); %Ks 
[P,E,K_q2] = dare(A_q2,B_q2,Q_q2,R_q2);
[P,E,K_q3] = dare(A_q3,B_q3,Q_q3,R_q3);

x_q1 = [x(1); x(4)];
x_q2 = [x(2); x(5)];
x_q3 = [x(3); x(6)];


delta_u = zeros(4,1);
delta_u(1) = -K_q1*x_q1;
delta_u(2) = -K_q2*x_q2;
delta_u(3) = 0;
delta_u(4) = -K_q3*x_q3;

% RETURN OUTER LOOP CONTROL VECTOR
u=u_nom + delta_u;
