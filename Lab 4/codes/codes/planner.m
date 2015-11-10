function [ waypoint] = planner(x0,t)
        %[x  y  z  yaw]
x_goal = [2, 0, -1, 0];
x = x0;
t_goal = 3; %s

if t<t_goal
    waypoint = x+(x_goal-x)*t/t_goal;
else
    waypoint = x_goal;
end
