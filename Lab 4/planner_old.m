function [x_des, y_des, z_des, yaw_des] = planner(x,t)

% xinitial = [0;0;-1];
% xgoal = [2;0;-1];
% tgoal = 3;
% 
% if t<tgoal
%     something
% else
%     something
%     
% if (x < xgoal(1))
%     DistancepertimeX = (xgoal(1)-xinitial(1))/5;
%     DistancepertimeY = (xgoal(2)-xinitial(2))/5;
%     DistancepertimeZ = (xgoal(3)-xinitial(3))/5;
% 
% 
%    x_des = x(1) + DistancepertimeX*t;
%    y_des = x(2) + DistancepertimeY*t;
%    z_des = x(3) + DistancepertimeZ*t;
% 
%     yaw_des = 0;
% end

current = [x(1); x(2); x(3)];
goal = [10;15;20];
tgoal = 3;



for i = 1:3
    if abs(current(i)-goal(i))>0.5
        des(i) = current(i)+(goal(i)-current(i))*(t/tgoal);
    else
        des(i)= goal(i);
    end
end
x_des = des(1);
y_des = des(2);
z_des = des(3);
yaw_des = 0;

end