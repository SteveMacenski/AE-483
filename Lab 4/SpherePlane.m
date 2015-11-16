function [d,dgrad] = SpherePlane(q,r,p,z)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE (PROBLEM 2)
d = z'*(p-q)-r;
dgrad = z';