function [d,dgrad] = SphereHotDog(q,r,p1,p2,s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE (PROBLEM 3)
gamma = (p2-p1)/(norm(p2-p1));
w = q-p1;
t = gamma'*w;

if t<0
  p_closest = p1;
  d = norm(q-p_closest)-(r+s);
  dgrad = ((q-p_closest)/norm(q-p_closest))'; 
  
elseif (t>0 && t<norm(p2-p1))
  p_closest = p1+gamma;
  d = norm(q-p_closest)-(r+s);
  dgrad = ((q-p_closest)/norm(q-p_closest))';
  
else
  p_closest = p2;
  d = norm(q-p_closest)-(r+s);
  dgrad = ((q-p_closest)/norm(q-p_closest))';

end