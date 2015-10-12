function u = Example_policy_teststandPart6(t,x)
%
% u = Example_policy_teststand(t,x)
% inputs:
%   t -- time in seconds
%   x -- state vector [ theta; thetadot ]
% outputs:
%   u -- control vector [u], representing applied pitch moment
%x0 = [0;0];

dt = 0.001;
J = .004;
A = [0 1; 0 0];
B = [0;1/J];
C = [1 0]; 
Q = [100 0; 0 1];
R = 1000;

A = eye(2) + dt * A;
B = dt * B;

[P,E,K] = dare(A,B,Q,R);

r = 0;
A = [0 1; 0 0];
B = [0;1/J];
Kref = -1/(C*inv((A-B*K))*B);
% CREATE YOUR CONTROLLER HERE
u = -K*x + Kref*r;