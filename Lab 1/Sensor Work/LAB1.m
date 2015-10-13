%% AE 483 Lab 1 Given flight data from onboard IMU and motion capture system, give information about the data and plot the quad copter in the time domain
% Also, integrate the data to show a relationship/no relationship between
% IMU and mocap data sets. 
 
load('flight_data.mat');
%flight path figure
figure(1)
plot3(flight_data(:,8),flight_data(:,9),flight_data(:,10))
title('Flight path (m)');


% %% figure of the rate from onboard sensors
figure (2)
plot(flight_data(:,1),flight_data(:,5));
hold on
plot(flight_data(:,1),flight_data(:,6));
plot(flight_data(:,1),flight_data(:,7));
title('rate of angle (rad/s)');
hold off

%%

%figure of the angle from motion captures 
figure(3)
plot(flight_data(:,1),flight_data(:,11));
hold on
plot(flight_data(:,1),flight_data(:,12));
plot(flight_data(:,1),flight_data(:,13));
title('Angle rad');
hold off


%% integrating the X (pitch) data giving angle
rate_x =flight_data(:,5);

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=0.0035; % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
y_pitch(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = rate_x(n)*h;
    y_pitch(n+1)=y_pitch(n)+dydx; 
end



%% integrating the Y (roll) data giving angle

rate_y =flight_data(:,6);

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=0.0099; % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
y_roll(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = rate_y(n)*h;
    y_roll(n+1)=y_roll(n)+dydx; 
end

%% integrating the Z (yaw) data giving angle
rate_z =flight_data(:,7);

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=-0.0075; % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
y_yaw(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = rate_z(n)*h;
    y_yaw(n+1)=y_yaw(n)+dydx; 
end



%% plots for part A
figure(4)
plot(t,y_pitch,t,flight_data(:,11));
title('Integrated Pitch V Data Pitch');
legend('integrated pitch','data pitch','Location','Best');

figure(5)
plot(t,y_roll,t,flight_data(:,12));
title('Integrated roll V Data roll');
legend('integrated roll','data roll','Location','Best');

figure(6)
plot(t,y_yaw,t,flight_data(:,13));
title('Integrated yaw V Data yaw');
legend('integrated yaw','data yaw','Location','Best');


 %% Integrating a.x --> Vx --> x position
accelx = flight_data(:,2);

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=-0.5394; % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
Xvel(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = accelx(n)*h;
    Xvel(n+1)=Xvel(n)+dydx; 
end

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=Xvel(1); % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
Xpos(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx(n) = Xvel(n)*h;
    Xpos(n+1)=Xpos(n)+dydx(n); 
end
 figure(7)
 plot(t,Xpos,t,flight_data(:,8))
 title('X position integrated v data')

%% Integrating acc. y --> Vy --> y position

accely = flight_data(:,3);

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=-0.0588; % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
Yvel(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = accely(n)*h;
    Yvel(n+1)=Yvel(n)+dydx; 
end

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=Yvel(1); % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
Ypos(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = Yvel(n)*h;
    Ypos(n+1)=Ypos(n)+dydx; 
end
 figure(8)
 plot(t,Ypos,t,flight_data(:,9))
 title('Y position integrated v data')
 
 
 %% Integrating acc. z --> Vz --> z position

accelz = flight_data(:,4);

clear t % Clears old time steps and
clear y % y values from previous runs
clear yv
clear yx
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=-10.0616; % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
Zvel(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = accelz(n)*h;
    Zvel(n+1)=Zvel(n)+dydx; 
end

clear t % Clears old time steps and
clear y % y values from previous runs
a=0; % Initial time
b=108.0563; % Final time
N=4332; % Number of time steps
y0=Zvel(1); % Initial value y(a)
h=(b-a)/N; % Time step
t(1)=a;
Zpos(1)=y0;
for n=1:N-1 % For loop, sets next t,y values
    t(n+1)=t(n)+h;
    dydx = Zvel(n)*h;
    Zpos(n+1)=Zpos(n)+dydx; 
end
 figure(9)
 plot(t,Zpos,t,flight_data(:,10))
 title('Z position integrated v data')
 legend('integrated Z','data Z','Location','Best')
 
%% 

% p1 initial body frame  
%p1* R + (x,y,z)_pose for world frame motion 
