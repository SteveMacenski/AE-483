load('QuadTestStandData.mat');

time = DrayMacenskiFester(:,1);
theta = DrayMacenskiFester(:,2);
theta_dot = DrayMacenskiFester(:,5);

figure(1)
plot(T,X(:,1))
hold on
plotted_time = DrayMacenskiFester(1:992,1);
plotted_theta = DrayMacenskiFester(2423:3414,2)+.077;
plot(plotted_time,plotted_theta)
title('Pitch Motion of the Quad Rotor')
legend('simulation','experiment w/ hardware')
xlabel('time (s)')
ylabel('angle (rad)')
hold off


figure(2)
plot(T,X(:,2))
hold on
plotted_time2 = DrayMacenskiFester(1:992-15,1);

plotted_theta_dot = DrayMacenskiFester((2423+15):3414,5);
plot(plotted_time2,plotted_theta_dot)
hold off
title('Pitch Motion of the Quad Rotor')
legend('simulation','experiment w/ hardware')
xlabel('time (s)')
ylabel('angular rate (rad/s)')

figure(4)
plot(U_T,U)
title('Control')