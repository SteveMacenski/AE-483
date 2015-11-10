function [xdes, ydes, zdes, yawdes] = planner(x,t)
        %[x  y  z  yaw]
x_goal = [2; 0; -1];
t_goal = 3; %s

if t<t_goal
    waypoint = x(1:3)+(x_goal-x(1:3))*t/t_goal;
else
    waypoint = x_goal;
end

xdes = waypoint(1);
ydes = waypoint(2);
zdes = waypoint(3);
yawdes = 0;

