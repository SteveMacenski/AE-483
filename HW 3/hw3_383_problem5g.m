%ae 483 hw 3 problem 5 part g
% Splitting the control loops into inner and outer sections, complete control with
% dts different for each loop corresponding to a system with different measurement rates
% October 24, 2015

g = 9.81;
J2 = .01;

Ac = [0 1;0 0];
  
 Bcinner = [0;1/J2];
 Bcouter = [0;-g*dtouter];
 
 dtouter = .02;
 dtinner = .001;
 
 Adouter = eye(2) + dtouter*Ac;
 Adinner = eye(2) + dtinner*Ac;
 
 Bdouter = Bcouter*dtouter;
 Bdinner = Bcinner*dtinner;
 
 Q = eye(2);
 R = 1;
 
[P,E,Kouter] = dare(Adouter,Bdouter,Q,R);
[P,E,Kinner] = dare(Adinner,Bdinner,Q,R);

phi = 0;
xdesignouter = [1;0];
xdesigninner = [phi;0];
udesignouter = [0];
udesigninner = [0];

xd = [];
xd(1:2,1) = [0;0];
xd(1:2,5002) = [0;0];

udouter = -Kouter*([0;0] - xdesignouter) + udesignouter;

for i=1:5000
    if (~mod(i,.02/.001)) 
        udouter(i) = -Kouter*(xd(1:2,i) - xdesignouter) + udesignouter;
    end
    
    xdinnerdes = 
    udinnerdes = 
    
    ud(:,i) = 
    
    xd(:,i+1) = Adinner*xd(:,i) + Bd*udinnerdes;
    
end

%plot