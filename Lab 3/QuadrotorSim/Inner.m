function u=Inner(t,x,u_outer)
%
% x: state vector [x;y;z;vx;vy;vz;theta_3;theta_2;theta_1;p;q;r]
%
%         NOTE: theta_3 = roll, theta_2 = pitch, theta_1 = yaw
%
% 
% u_outer: [theta_3_star, theta_2_star, theta_1_star, u4_star] , 
%          where these values represent the "desired" or "nominal" 
%          roll, pitch, yaw, and thrust, respectively.
%
% The main purpose of the inner loop controller is to regulate attitude.
% We achieve this by linearizing about the level attitude (roll, pitch = 0),
% and design 3 independent LQR controllers for the roll, pitch, and yaw regulation.
%
% For now, we will simply pass through the nominal thrust given in u_outer.
%

% MEASUREMENT
angles=angle_meas(x);  % roll, pitch, yaw == theta_3, theta_2, theta_1
angularvelocity=gyro(x);

% CONTROL
u=zeros(4,1);
dt = .001;

J = [.004024 0 0;0 .004015 0; 0 0 .007593]; %moment of interia matrix
J1 = J(1,1);
J2 = J(2,2);
J3 = J(3,3);

A_roll = eye(2) + dt.*[0 1;0 0]; %A matrices
A_pitch = eye(2) + dt.*[0 1;0 0];
A_yaw = eye(2) + dt.*[0 1;0 0];

B_roll = dt*[0 ; 1/J1]; %B matrices
B_pitch = dt*[0 ; 1/J2];
B_yaw = dt*[0 ; 1/J3];

Q_roll = eye(2);
Q_pitch = eye(2);
Q_yaw = eye(2);
R_roll = 1;
R_pitch = 1;
R_yaw = 1;

[P,E,K_roll] = dare(A_roll,B_roll,Q_roll,R_roll); %Ks 
[P,E,K_pitch] = dare(A_pitch,B_pitch,Q_pitch,R_pitch);
[P,E,K_yaw] = dare(A_yaw,B_yaw,Q_yaw,R_yaw);

x_roll = [x(7); x(10)];
x_pitch = [x(8); x(11)];
x_yaw = [x(9); x(12)];
a = 0;
b = 0;
c = 0;
a = [a;0];
b = [b;0];
c = [c;0];
u(1) = -K_roll*(x_roll - a);
u(2) = -K_pitch*(x_pitch - b);
u(3) = -K_yaw*(x_yaw - c);
u(4) = u_outer(4);




