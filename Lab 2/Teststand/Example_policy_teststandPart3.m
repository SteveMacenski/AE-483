function u = Example_policy_teststandPart3(t,x)
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
Kp = 1;
Kd = .3;
% CREATE YOUR CONTROLLER HERE
u = - Kp*x(1) - Kd*x(2);
