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

persistent Kroll Kpitch Kyaw
% CONTROL

u=zeros(4,1);

%defining desired angles
a=u_outer(3);
b=u_outer(2);
c=u_outer(1);


if isempty(Kroll)
%LQR CONTROLLER ROLL(THETA3)
dt = 0.001;
A=[0,1;0,0];
J = 0.004029; 
B=[0; 1/J];
Q=[1.5,0;0,0.1];
R=5;

A = eye(2) + dt*A;
B = dt*B;
[P,E,Kroll]=dare(A,B,Q,R);



%LQR CONTROLLER PITCH(THETA2)
dt = 0.001;
A=[0,1;0,0];
J = 0.004015; 
B=[0; 1/J];
Q=[2.0,0;0,0.1];
R=5;

A = eye(2) + dt*A;
B = dt*B;
[P,E,Kpitch]=dare(A,B,Q,R);



%LQR CONTROLLER YAW(THETA1)
dt = 0.001;
A=[0,1;0,0];
J = 0.007593; 
B=[0; 1/J];
Q=[2.0,0;0,0.1];
R=5;

A = eye(2) + dt*A;
B = dt*B;
[P,E,Kyaw]=dare(A,B,Q,R);

Kroll
Kpitch
Kyaw
end

xroll=[angles(1)-c; angularvelocity(1)];
u(1)=-Kroll*xroll;
xpitch=[angles(2)-b; angularvelocity(2)];
u(2)=-Kpitch*xpitch;
xyaw=[angles(3)-a; angularvelocity(3)];
u(3)=-Kyaw*xyaw;

u(4)=u_outer(4);




