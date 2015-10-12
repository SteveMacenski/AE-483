% 483 lab 2 works

time_force = Lab2TestData(:,1);
force = Lab2TestData(:,2);
time_techometer = Lab2TestData(:,4);
rot_per_sec = Lab2TestData(:,6);

figure(1)
plot(time_force,force+.4);
title('Force plot');
xlabel('time');
ylabel('force');
figure(2)
plot(time_techometer,rot_per_sec);
title('rps plot');
xlabel('time');
ylabel('rps');


Force_data = [37.44	-0.64	25;
              43.91	-0.82	26;
              55.33	-1.13	30;
              61.84	-1.35	33;
              67.92	-1.56	34.5;
              73.75	-1.67	36.5;
              80.03	-2.13	38.5;
              86.19	-2.45	40.5;
              92.38	-2.8	42.5;
              97.66	-3.08	44.5];
figure(3)
RPS = (Force_data(:,1)*2*pi).^2;
plot(RPS,Force_data(:,2)*-1);
title('Force v RPS');
ylabel('Force (N)');
xlabel('RPS');

Kf = RPS\(-1*Force_data(:,2))

%%

moment_data = [28.37 -0.005917;...
               37.64 -0.009005;...
               43.72 -.0117;
               48.99 -.01356;
               55.48 -0.01714;
               61.38 -0.02043;
               67.48 -0.0242;
               74.49 -0.0291;
               79.99 -0.0343;
               85.86 -0.03895];

time_force = kMdataset(:,1);
moment = 15e-3*kMdataset(:,2);
time_techometer = kMdataset(:,4);
rot_per_sec = kMdataset(:,6);

figure(4)
plot(time_force,moment+.00454);
title('Moment plot');
xlabel('time');
ylabel('Moment');
figure(5)
plot(time_techometer,rot_per_sec);
title('rps plot');
xlabel('time');
ylabel('rps');


figure(6)
RPS = (moment_data(:,1)*2*pi).^2;
plot(RPS,moment_data(:,2)*-1);
title('moment v RPS');
ylabel('Moment (N-m)');
xlabel('RPS');

Km = RPS\(-1*moment_data(:,2))


