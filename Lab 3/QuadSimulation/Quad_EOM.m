function f=Quad_EOM(x,u)
%
% x: state vector [x;y;z;vx;vy;vz;theta_3;theta_2;theta_1;p;q;r]
%
%         NOTE: theta_3 = roll, theta_2 = pitch, theta_1 = yaw
%
% u: control vector [u1, u2, u3, u4] == roll, pitch, and yaw torque, and thrust
%

% Physical Parameters
J = [.004024 0 0;0 .004015 0; 0 0 .007593]; 
J1 = J(1,1);
J2 = J(2,2);
J3 = J(3,3);
g = 9.81;  % gravity
m = 1; %arbitrary 

c1 = cos(x(9)); % defining the parameters to shorten equations of motion
c2 = cos(x(8));
c3 = cos(x(7));
s1 = sin(x(9));
s2 = sin(x(8));
s3 = sin(x(7));

f = zeros(12,1); %initialize the force matrix 

% q_dot(i)  = V(i)
f(1) = x(4);
f(2) = x(5);
f(3) = x(6);

% vdot(i) = input stuff 
f(4) = -(c1*s2*c3 + s1*s3)*u(4)/m + 0;
f(5) = -(s1*s2*c3 - c1*s3)*u(4)/m + 0;
f(6) = -(c2*c3)*u(4)/m + g;

% thetadot = omega garbage
f(9) = (1/c2)*(s3*x(11) + c3*x(12));
f(8) = (1/c2)*(c2*c3*x(11) + -c2*s3*x(12));
f(7) = (1/c2)*(c2*x(10) + s2*s3*x(11) + s2*c3*x(12));
 

% omegadot = omega, Js, other stuffz
f(10) = 1/J1 * (u(1) + (J2 - J3)*x(11)*x(12));
f(11) = 1/J2 * (u(2) + (J3 - J1)*x(10)*x(12));
f(12) = 1/J3 * (u(3) + (J1 - J2)*x(10)*x(11));

return 
%happy dance 
