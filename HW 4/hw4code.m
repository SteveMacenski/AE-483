function hw4code

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE (PROBLEM 7)
%
% Set parameters
param.krep = 1;
param.brep = 1;
param.katt = 10000;
param.batt = 1;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UNCOMMENT FOR SCENARIO #1

% Set drone radius and start position
drone.r = 0.1;
drone.q = [-0.65; 0.65; -0.65];

% Set goal position
goal.q = [0.65; -0.65; 0.65];

% % Set obstacles
obst={};
obst = AddObstacle_Plane(obst,[-1;-1;-1],[1;0;0]);
obst = AddObstacle_Plane(obst,[-1;-1;-1],[0;1;0]);
obst = AddObstacle_Plane(obst,[-1;-1;-1],[0;0;1]);
obst = AddObstacle_Plane(obst,[1;1;1],[-1;0;0]);
obst = AddObstacle_Plane(obst,[1;1;1],[0;-1;0]);
obst = AddObstacle_Plane(obst,[1;1;1],[0;0;-1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % UNCOMMENT FOR SCENARIO #2
%
% % Set drone radius and start position
% drone.r = 0.1;
% drone.q = [-0.65; 0.65; -0.65];
% 
% % Set goal position
% goal.q = [0.65; -0.65; 0.65];
% 
% Set obstacles
% obst={};
% obst = AddObstacle_Plane(obst,[-1;-1;-1],[1;0;0]);
% obst = AddObstacle_Plane(obst,[-1;-1;-1],[0;1;0]);
% obst = AddObstacle_Plane(obst,[-1;-1;-1],[0;0;1]);
% obst = AddObstacle_Plane(obst,[1;1;1],[-1;0;0]);
% obst = AddObstacle_Plane(obst,[1;1;1],[0;-1;0]);
% obst = AddObstacle_Plane(obst,[1;1;1],[0;0;-1]);
% obst = AddObstacle_HotDog(obst,[0.45;-0.2;-1],[0.45;-0.2;0.85],0.1);
% obst = AddObstacle_HotDog(obst,[0.45;-0.2;.1],[-0.1;0.6;0.95],0.05);
% obst = AddObstacle_HotDog(obst,[0.45;-0.2;-.1],[0.1;-0.8;0.75],0.05);
% obst = AddObstacle_HotDog(obst,[0.45;-0.2;.5],[0.8;0.1;0.9],0.05);
% obst = AddObstacle_HotDog(obst,[-0.5;0.2;-1],[-0.5;0.2;0.65],0.1);
% obst = AddObstacle_HotDog(obst,[-0.5;0.2;.1],[-0.8;0.6;0.95],0.05);
% obst = AddObstacle_HotDog(obst,[-0.5;0.2;.3],[-0.6;-0.4;0.85],0.05);
% obst = AddObstacle_HotDog(obst,[-0.5;0.2;-.3],[0.15;0.4;0.7],0.05);
% obst = AddObstacle_MovingSphere(obst,[-0.5;0.0;0.5],0.05,[0.0;0.0;-0.95]);
% obst = AddObstacle_MovingSphere(obst,[0;0.5;1.0],0.05,[0.0;0.0;-0.85]);
% obst = AddObstacle_MovingSphere(obst,[0.3;-0.5;1.5],0.05,[0.0;0.0;-1.2]);
% obst = AddObstacle_MovingSphere(obst,[0.0;0.4;2],0.05,[0.0;0.0;-1.4]);
% obst = AddObstacle_MovingSphere(obst,[-0.7;0.9;2.5],0.05,[0.0;0.0;-1.1]);
% obst = AddObstacle_MovingSphere(obst,[0.2;-0.6;3],0.05,[0.0;0.0;-0.9]);
% obst = AddObstacle_MovingSphere(obst,[0.5;0.3;3.5],0.05,[0.0;0.0;-1.2]);
% obst = AddObstacle_MovingSphere(obst,[0.5;0.7;4.0],0.05,[0.0;0.0;-1.5]);
% obst = AddObstacle_MovingSphere(obst,[0.3;0.8;4.5],0.05,[0.0;0.0;-1.4]);
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % UNCOMMENT FOR SCENARIO #3
% 
% % Set drone radius and start position
% drone.r = 0.1;
% drone.q = [-0.65; 0.65; -0.65];
% 
% % Set goal position
% goal.q = [0.65; -0.65; 0.65];
% 
% Set obstacles
% obst = AddRandomObstacles(25,0.05,0.45,drone,goal,{},param);
% obst = AddObstacle_Plane(obst,[-1;-1;-1],[1;0;0]);
% obst = AddObstacle_Plane(obst,[-1;-1;-1],[0;1;0]);
% obst = AddObstacle_Plane(obst,[-1;-1;-1],[0;0;1]);
% obst = AddObstacle_Plane(obst,[1;1;1],[-1;0;0]);
% obst = AddObstacle_Plane(obst,[1;1;1],[0;-1;0]);
% obst = AddObstacle_Plane(obst,[1;1;1],[0;0;-1]);
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Set more parameters (don't change these)
param.maxvel = 1;
param.dt = 1e-2;
param.kdescent = 2.5e-2;
param.bdescent = param.maxvel*param.dt;
param.goaleps = 1e-2;
param.gradeps = 1e-5;

% Create the display
world = CreateWorld(drone,goal,obst);

% Pause for effect
pause;

% Do gradient descent (returning 1 for success, 0 for failure)
res = DoGradientDescent(world,drone,goal,obst,param);



function res = DoGradientDescent(world,drone,goal,obst,param)

while (1)
    
    % Update position of moving obstacles
    for i=1:length(obst)
        if (obst{i}.type==1)
            obst{i}.p = obst{i}.p+obst{i}.v*param.dt;
        end
    end
    
    % Get gradient of potential function
    gradf = GetGradient(drone,goal,obst,param);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Do gradient descent (i.e., take a step)
    dq = -param.dt*(param.kdescent*gradf);
    dqnorm = sqrt(dq'*dq);
    if (dqnorm > param.bdescent)
        dq = param.bdescent*(dq/dqnorm)
    end
    drone.q = drone.q+dq;
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Update the display
    world = UpdateWorld(world,drone,goal,obst);
    
    % Stop if we are within tolerance of the goal
    err = drone.q - goal.q;
    errnorm = sqrt(err'*err);
    if (errnorm < param.goaleps)
        fprintf(1,'SUCCESS!\n');
        res = 1;
        return;
    end
    
    % Stop if we have reached a local minimum
    dqnorm = sqrt(dq'*dq);
    if (dqnorm < param.gradeps)
        fprintf(1,'FAILURE!\n');
        res = 0;
        return;
    end
end



function gradf = GetGradient(drone,goal,obst,param)
gradf = GetAttractiveGradient(drone,goal,param)+GetRepulsiveGradient(drone,obst,param);

function gradfatt = GetAttractiveGradient(drone,goal,param)
v = drone.q - goal.q;
vnorm = sqrt(v'*v);

if (vnorm <= param.batt)
    gradfatt = param.katt * v;
else 
    gradfatt = param.katt * param.batt * (v/vnorm)
end

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [d,dgrad] = SpherePlane(q,r,p,z)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
d = z'*(p-q) - r;
dgrad = z;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [d,dgrad] = SphereHotDog(q,r,p1,p2,s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
v = (p1 - p2)/norm(p1-p2);
w = q - p1;
t = v'*w;

if t<0
    Pclosest = p1;
elseif t > norm(p2 - p1)
    Pclosest = p2;
else
    Pclosest = p1 + t*v;
end

vc = (q-Pclosest);
d = norm(vc) - (r+s);
dgrad = vc/norm(vc);

function [d,dgrad] = SphereSphere(q,r,p,s) 
v = q-p;
v_norm = sqrt(v'*v);
d = v_norm - (r+s);
dgrad = v/v_norm;


%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function gradfrep = GetRepulsiveGradient(drone,obst,param)
gradfrep = zeros(size(drone.q));
for i=1:length(obst)
    
    if (obst{i}.type == 1)
        [d,dgrad] = SphereSphere(drone.q,drone.r,obst{i}.p,obst{i}.s);
    elseif (obst{i}.type == 2)
        [d,dgrad] = SphereHotDog(drone.q,drone.r,obst{i}.p1,obst{i}.p2,obst{i}.s);
    elseif (obst{i}.type == 3)
        [d,dgrad] = SpherePlane(drone.q,drone.r,obst{i}.p,obst{i}.z);
    else
        error('bad obst{i}.type in GetRepulsiveGradient');
    end
    
    if (d <= param.brep)
        gradfrep = gradfrep + (param.krep*((1/param.brep)-(1/d))*(1/(d^2))*dgrad);
    end
    
end


function world = CreateWorld(drone,goal,obst)
clf;
axis([-1 1 -1 1 -1 1]);
axis equal;
hold on;
xlabel('x');
ylabel('y');
zlabel('z');

% Create unit sphere
[world.x,world.y,world.z]=sphere(16);
[m,n]=size(world.x);
c = ones(m,n,3);

% Create goal
c(:,:,1) = 0.25;
c(:,:,2) = 0.75;
c(:,:,3) = 0.25;
world.goal= surf(goal.q(1)+drone.r*world.x,goal.q(2)+drone.r*world.y,goal.q(3)+drone.r*world.z,c);

% Create drone
c(:,:,1) = 0.25;
c(:,:,2) = 0.25;
c(:,:,3) = 0.75;
world.drone = surf(drone.q(1)+drone.r*world.x,drone.q(2)+drone.r*world.y,drone.q(3)+drone.r*world.z,c);

% Create obstacles
for i=1:length(obst)
    if (obst{i}.type == 1)
        c(:,:,1) = 0.75;
        c(:,:,2) = 0.25;
        c(:,:,3) = 0.25;
        world.obst(i) = surf(obst{i}.p(1)+obst{i}.s*world.x,obst{i}.p(2)+obst{i}.s*world.y,obst{i}.p(3)+obst{i}.s*world.z,c);
    elseif (obst{i}.type == 2)
        [x,y,z] = MakeCylinder(obst{i}.p1,obst{i}.p2,obst{i}.s,1.5e-2,16);
        [m,n]=size(x);
        cc = ones(m,n,3);
        cc(:,:,1) = 0.75;
        cc(:,:,2) = 0.25;
        cc(:,:,3) = 0.25;
        world.obst(i) = surf(x,y,z,cc);
    elseif (obst{i}.type == 3)
        %
        % FOR NOW, DO NOTHING...
        %
    else
        error('bad obst(i).type in CreateWorld');
    end
    
end

% Make stuff look pretty
light('Position',[1 3 2]);
world.dronelight = light('position',drone.q,'style','local');
h = findobj('Type','surface');
set(h,'FaceLighting','gouraud',...
      'FaceColor','interp',...
      'EdgeColor',[.4 .4 .4],...
      'LineStyle','none',...
      'BackFaceLighting','lit',...
      'AmbientStrength',0.4,...
      'DiffuseStrength',0.6,...
      'SpecularStrength',0.5);
material default

% Make sure the axes are what we want
axis([-1 1 -1 1 -1 1]);
axis manual

% Update graphics
drawnow;

function world = UpdateWorld(world,drone,goal,obst)
% Update drone
set(world.drone,'xdata',drone.q(1)+drone.r*world.x);
set(world.drone,'ydata',drone.q(2)+drone.r*world.y);
set(world.drone,'zdata',drone.q(3)+drone.r*world.z);
set(world.dronelight,'position',drone.q);
% Update obstacles
for i=1:length(obst)
    if (obst{i}.type == 1)
        set(world.obst(i),'xdata',obst{i}.p(1)+obst{i}.s*world.x);
        set(world.obst(i),'ydata',obst{i}.p(2)+obst{i}.s*world.y);
        set(world.obst(i),'zdata',obst{i}.p(3)+obst{i}.s*world.z);
    elseif (obst{i}.type == 2)
        %
        % FOR NOW, DO NOTHING...
        %
    elseif (obst{i}.type == 3)
        %
        % FOR NOW, DO NOTHING...
        %
    else
        error('bad obst(i).type in UpdateWorld');
    end
end
% Update goal
set(world.goal,'xdata',goal.q(1)+drone.r*world.x);
set(world.goal,'ydata',goal.q(2)+drone.r*world.y);
set(world.goal,'zdata',goal.q(3)+drone.r*world.z);
% Update display
drawnow;

function [x0,y0,z0] = MakeCylinder(q1,q2,r,res,n)
v = q2-q1;
len = sqrt(v'*v);
v = v/len;
z10 = v;
x10 = (eye(3)-(v*v'))*(rand(3,1)-q1);
x10 = x10/sqrt(x10'*x10);
y10 = cross(z10,x10);
t = linspace(0,len+2*r,ceil((len+2*r)/res)+1);
t1 = t(t<=r);
t2 = t((t>r)&(t<=(len+r)));
t3 = t(t>(len+r));
s = real([sqrt(r^2-(t1-r).^2) r*ones(size(t2)) sqrt(r^2-(t3-(len+r)).^2)]);
[x1,y1,z1] = cylinder(s,n);
z1 = (z1*(len+2*r))-r;
x0 = q1(1)+(x10(1)*x1+y10(1)*y1+z10(1)*z1);
y0 = q1(2)+(x10(2)*x1+y10(2)*y1+z10(2)*z1);
z0 = q1(3)+(x10(3)*x1+y10(3)*y1+z10(3)*z1);

function obst = AddRandomObstacles(n,smin,smax,drone,goal,obst,param)
for i=1:n
    
    while(1)
        p = -1+2*rand(3,1);
        s = smin+(smax-smin)*rand;
        [dd,tmp] = SphereSphere(drone.q,drone.r,p,s);
        [dg,tmp] = SphereSphere(goal.q,drone.r,p,s);
        if ((dd>0)&&(dg>param.brep))
            obst = AddObstacle_Sphere(obst,p,s);
            break;
        end
    end
    
end

function obst = AddObstacle_Sphere(obst,p,s)
onew.type = 1;
onew.p = p;
onew.s = s;
onew.v = [0;0;0];
obst{end+1}=onew;

function obst = AddObstacle_MovingSphere(obst,p,s,v)
onew.type = 1;
onew.p = p;
onew.s = s;
onew.v = v;
obst{end+1}=onew;

function obst = AddObstacle_Plane(obst,p,z)
onew.type = 3;
onew.p = p;
onew.z = z;
obst{end+1}=onew;

function obst = AddObstacle_HotDog(obst,p1,p2,s)
onew.type = 2;
onew.p1 = p1;
onew.p2 = p2;
onew.s = s;
obst{end+1}=onew;
