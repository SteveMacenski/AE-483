function [d,dgrad] = SphereSphere(q,r,p,s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE (PROBLEM 1)
d = norm(q-p)-(r+s);
dgrad = ((q-p)/norm(q-p))';
