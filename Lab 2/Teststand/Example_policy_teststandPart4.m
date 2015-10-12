function u = Example_policy_teststandPart4(t,x)
%
% u = Example_policy_teststand(t,x)
%
% inputs:
%   t -- time in seconds
%   x -- state vector [ theta; thetadot ]
%
% outputs:
%   u -- control vector [u], representing applied pitch moment
%
%x0 = [pi/4;0];
dt = 0.001;
J = .004;
A = [0 1; 0 0];
B = [0;1/J];
Q = [100 0; 0 1];
R = 1000;

A = eye(2) + dt * A;
B = dt * B;

[P,E,K] = dare(A,B,Q,R);


% CREATE YOUR CONTROLLER HERE
u = -K*x;
