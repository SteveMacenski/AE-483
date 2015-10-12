function out=vect4auto1(eom,u,disturbance,t,x)
% The name of this file is outdated, but it's just a handler file that
% plays nice between our code and ODE45.  This code applies the external
% force disturbance, and applies any state constraints (limits).

out=eom(t,x,u(t,x));

% Force Disturbance
out=out+disturbance(t,x);

% STATE CONSTRAINTS
% none right now.
