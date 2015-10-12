function out=wind1D(t)

% Sample wind disturbance
% Note: out must have the same dimension as the state matrix.
% For the 1D testbed, the state vector is [theta;omega;omegar1;omegar3]
out=zeros(2,1);
if t>=3 && t<=6
    % Wind disturbance is a force and affects only omega_dot.
    out(2)=normrnd(0.01,0.001);
end