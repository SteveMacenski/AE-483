
%ae 483 hw 3 problem 5 part e 
% with  found Ac Bc and Ad Bd for continuous and time descrete
% control, plot the the state as it converges under some time interval. October 24, 2015

clear all;clf;
x0 = [0;0;0;0];
xdes = [1;0;0;0];

udes = [0];

g = 9.81;
J2 = .01;

Ac = [0 1 0 0;
      0 0 -g 0;
      0 0 0 1;
      0 0 0 0];
  
 Bc = [0;
       0;
       0;
       1/J2];
 
 dt = .02;
 Ad = eye(4) + dt*Ac;
 Bd = Bc*dt;
 
 Q = [200 0 0 0;
      0 100 0 0;
      0 0 10 0;
      0 0 0 10];
  R = 100;

[P,E,K] = dare(Ad,Bd,Q,R);
u=[];
x = x0;
u(1) = 0;
hold on
for i = 1:250/2
    u(i) = udes - K*(x - xdes);
    x = Ad*x + Bd*u(i);
    plot(i*dt,x(1),'k.',i*dt,x(2),'b.',i*dt,x(3),'g.',i*dt,x(4),'y.')
end







