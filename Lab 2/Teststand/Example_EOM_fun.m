function f=Example_EOM_fun(x,u)
%
% f = EOM(x,u)
%
% inputs:
%   x -- state vector containing [theta, thetadot] (pitch angle, and angular velocity)
%   u -- control vector containing [u]  (pitch moment)
%
% outputs:
%   f -- right hand side of the state-space equations of motion
%

% MODEL PARAMETERS
J = 0.004;% 0.00401299;
 

% OUTPUT 
f=[0 1; 0 0]*x + [0; 1/J]*u;

