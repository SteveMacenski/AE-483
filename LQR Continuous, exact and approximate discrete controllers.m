% hw 2 problem 6 483
%Using a differential equation, design a continuous, exact discrete, and
%approximate discrete LQR controllers and plot differences between them. X0
%= [1;0] where the state X = [theta;theta_dot] and u = J*theta_doubleDot

deltat = .01;

t = 0:deltat:5;
Q = 10^-2*eye(2);
R = 1;
x0 = [1;0];

Acon = [0 1;0 0]; Bcon = [0; 1/.01]; Kcon = lqr(Acon,Bcon,Q,R);
Aapp = [1 deltat; 0 1]; Bapp = [0; deltat/.01]; Kapp = dlqr(Aapp,Bapp,Q,R);
Aexa = [1 deltat; 0 1]; Bexa = [deltat^2/(.01*2); deltat/.01]; Kexa = dlqr(Aexa,Bexa,Q,R);


%step(ss(Acon,Bcon,eye(2),0))

%plotting
x = [];
xapp(:,1) = x0;
xexa(:,1) = x0;
xcon(:,1) = x0;

for i = 1:length(t)
    ucon(:,i) = -Kcon*xcon(:,i);
    xcon_dot(:,i) = Acon*xcon(:,i) + Bcon*ucon(:,i);
    xcon(:,i+1) = xcon(:,i) + deltat*xcon_dot(:,i);
    
    
    uapp(:,i) = -Kapp*xapp(:,i);
    xapp(:,i+1) = Aapp*xapp(:,i)+Bapp*uapp(:,i);
    
    uexa(:,i) = -Kexa*xexa(:,i);
    xexa(:,i+1) = Aexa*xexa(:,i)+Bexa*uexa(:,i);
end

%change appropriate range for each deltat (501, 51 6) for dt = (.01 .1 1)
range = 1:501;
plot(t,xapp(1,range),t,xexa(1,range),t,xcon(1,range))
title('delta t = .01')



    