%ae 483 hw 3 problem 5 part d 
% with previously found Ac Bc and Ad Bd for continuous and time descrete
% control, plot the K(i) against Kss. October 24, 2015

clc;clear all;clf

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
 
 dt = .001;
 Ad = eye(4) + dt*Ac;
 Bd = Bc*dt;
 
 Qn = eye(4);
 Q = eye(4);
 R = 1;

K1=[];      %plotting steady state discrete value
K2=[];
K3=[];
K4=[];
[V,W,Z] = dare(Ac*dt+eye(4),Bc*dt,Qn,R);
K1 = Z(1);
K2 = Z(2);
K3 = Z(3);
K4 = Z(4);
t = 0:5;


n = 5000;
P{n+1} = Qn;


for i=n:-1:1
    P{i} = Q + Ad'*P{i+1}*Ad - Ad'*P{i+1}*Bd*inv(R+Bd'*P{i+1}*Bd)*Bd'*P{i+1}*Ad;
    K(i,:) = inv(R+Bd'*P{i+1}*Bd)*Bd'*P{i+1}*Ad;
end
time = 1:5000;
hold on
plot(t,K1*ones(6,1),'--r',t,K2*ones(6,1),'--r',t,K3*ones(6,1),'--r',t,K4*ones(6,1),'--r')
plot(time*dt,K(:,1),'-k',time*dt,K(:,2),'-k',time*dt,K(:,3),'-k',time*dt,K(:,4),'-k')












