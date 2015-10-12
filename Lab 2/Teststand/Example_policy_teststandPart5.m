function u = Example_policy_teststandPart5(t,x)
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
Kp = 1.30;
Kd = 20/100;

%130 20

% CREATE YOUR CONTROLLER HERE
a = 0;

u = - Kp*(x(1) - a) - Kd*x(2);