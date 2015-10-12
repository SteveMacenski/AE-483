function u = Example_policy_teststand(t,x)
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
Kp = 5;
Kd = 6;

% CREATE YOUR CONTROLLER HERE
u = -Kp*x(1) - Kd*x(2);
