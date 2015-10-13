function hw2code

%%%%%%%%% DEFINE PARAMETERS
params.g = 9.81;
params.m = 0.4;
params.J = [.01 .01 .02];
params.kF = 1e-5;
params.kM = 1e-7;
params.L = 0.5;
params.w = [0 -params.L*params.kF 0 params.L*params.kF; params.L*params.kF 0 -params.L*params.kF 0; -params.kM params.kM -params.kM params.kM; params.kF params.kF params.kF params.kF];

load('A.mat');
load('B.mat');

params.Q = eye(12);
params.R = 1000*eye(4);
params.K = lqr(A,B,params.Q,params.R);
%%% ... you can add anything you like to params ...



%%%%%%%%% DEFINE INITIAL CONDITIONS
q_1in0 = [-1;-2;2];
theta = [-pi/2;pi/6;pi/12];
w_01in1 = [0;0;0];
v0 = [0;0;0];
x = [q_1in0; v0; theta; w_01in1];



%%%%%%%%% SETUP FIGURE
% CAD model of quadrotor
[pRobot_in1,fRobot]=GetRobotModel('hw2code_quadmodel.mat');
% CAD model of room
dx = 3;
dy = 3;
dz = 1;
pRoom_in0 = [dx*[-1 1 1 -1 -1 1 1 -1]; dy*[-1 -1 1 1 -1 -1 1 1]; dz*[-1 -1 -1 -1 1 1 1 1]];
fRoom = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
% CAD model of coordinate axes
pFrame = [0 0.01 0 0; 0 0 0 0; 0 0 0 0];
pRoomFrame_in0 = pFrame;
pRobotFrame_in1 = pFrame;
% Coordinate transformations
R_1in0 = R_zyx(theta);
[pRobot_in0,pRobotFrame_in0,pRoom_in1,pRoomFrame_in1] = ApplyTransforms(q_1in0,R_1in0,pRobot_in1,pRobotFrame_in1,pRoom_in0,pRoomFrame_in0);
% Create figure
close all;
figure('position',[10 10 1250 680],'color','w');
axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on;
axis off;
% Create axis
world.axis0 = axes('position',[-0.5 -0.4 2 2]);
set(gcf,'renderer','opengl');
axis(1.5*[-dx dx -dy dy -dz dz]);
axis equal;
hold on;
axis off
set(gca,'cameraposition',[-11 -5 5]);
set(gca,'cameratarget',[0 0 0]);
camproj('perspective');
lighting gouraud
light('Position',[0 -2 1])
light('Position',[-1 1 2])
% Draw everything
world.in0 = DrawEverything(pRobot_in0,fRobot,pRobotFrame_in0,pRoom_in0,fRoom,pRoomFrame_in0);



%%%%%%%%% RUN SIMULATION
dt = 0.05;
while (1)
    % Take a step in time
    x = Simulate(x,params,dt);
    % Get the state
    theta = x(7:9,1);
    theta = AngDiff(theta,0);
    q_1in0 = x(1:3,1);
    w_01in1 = x(10:12,1);
    % Coordinate transformation
    R_1in0 = R_zyx(theta);
    [pRobot_in0,pRobotFrame_in0,pRoom_in1,pRoomFrame_in1] = ApplyTransforms(q_1in0,R_1in0,pRobot_in1,pRobotFrame_in1,pRoom_in0,pRoomFrame_in0);
    % Draw stuff
    world.in0 = UpdateEverything(world.in0,pRobot_in0,pRobotFrame_in0,pRoom_in0,pRoomFrame_in0);
    drawnow
end

function x = Simulate(x,params,dt)
[t,x] = ode45(@(t,x) fSimulate(t,x,params),[0 dt],x);
x = x(end,:)';





function dxdt = fSimulate(t,x,params)

%%%%%%%%%

q1 = 0;
q2 = 0;
q3 = 0;

a = [q1;q2;q3;zeros(9,1)];
u = -params.K*(x - a);

u(4) = (params.m*params.g)/(cos(x(8))*cos(x(9))) + u(4);
sigmasquared = inv(params.w) * u;


sigmasquared = min(max(sigmasquared,zeros(4,1)),1e6*ones(4,1));
%sigma = min(max(sigma,zeros(4,1)),1e3*ones(4,1));

u = params.w * sigmasquared;
%
%%%%%%%%%

% Dynamics
q = x(1:3,1);
v = x(4:6,1);
theta = x(7:9,1);
w = x(10:12,1);
s1 = sin(theta(1));
c1 = cos(theta(1));
s2 = sin(theta(2));
c2 = cos(theta(2));
s3 = sin(theta(3));
c3 = cos(theta(3));
dvdt = [0;0;params.g]-((u(4,1)/params.m)*[c1*s2*c3+s1*s3; s1*s2*c3-c1*s3; c2*c3]);
dwdt = diag(1./params.J)*(u(1:3,1) - wedge(w)*diag(params.J)*w);
dqdt = v;
dthetadt = (1/c2)*[0 s3 c3; 0 c2*c3 -c2*s3; c2 s2*s3 s2*c3]*w;
dxdt = [dqdt; dvdt; dthetadt; dwdt];

function ahat = wedge(a)
ahat = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%
function [pRobot_in0,pRobotFrame_in0,pRoom_in1,pRoomFrame_in1] = ApplyTransforms(q_1in0,R_1in0,pRobot_in1,pRobotFrame_in1,pRoom_in0,pRoomFrame_in0)
%%%%%%%
q_0in1 = -R_1in0'*q_1in0;
R_0in1 = R_1in0';
%%%%%%%
pRobot_in0 = Transform(q_1in0,R_1in0,pRobot_in1);
pRobotFrame_in0 = Transform(q_1in0,R_1in0,pRobotFrame_in1);
pRoom_in1 = Transform(q_0in1,R_0in1,pRoom_in0);
pRoomFrame_in1 = Transform(q_0in1,R_0in1,pRoomFrame_in0);
%%%%%%%

function p_in0 = Transform(q_1in0,R_1in0,p_in1)
p_in0 = repmat(q_1in0,1,size(p_in1,2)) + R_1in0*p_in1;
%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%
function R = R_zyx(theta)
h1 = theta(1);
h2 = theta(2);
h3 = theta(3);
R = [cos(h1)*cos(h2) cos(h1)*sin(h2)*sin(h3)-sin(h1)*cos(h3) cos(h1)*sin(h2)*cos(h3)+sin(h1)*sin(h3);
     sin(h1)*cos(h2) sin(h1)*sin(h2)*sin(h3)+cos(h1)*cos(h3) sin(h1)*sin(h2)*cos(h3)-cos(h1)*sin(h3);
     -sin(h2) cos(h2)*sin(h3) cos(h2)*cos(h3)];
%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dh=AngDiff(h2,h1)
dh=mod((h2-h1)+pi,2*pi)-pi;

function world=UpdateEverything(world,pRobot,pRobotFrame,pRoom,pRoomFrame)
q = [0;0;0];
R = [1 0 0;
     0 -1 0;
     0 0 -1];
world.room = UpdateRoom(world.room,Transform(q,R,pRoom));
world.roomframe = UpdateFrame(world.roomframe,Transform(q,R,pRoomFrame));
world.robot = UpdateQuadrotor(world.robot,Transform(q,R,pRobot));
world.robotframe = UpdateFrame(world.robotframe,Transform(q,R,pRobotFrame));

function world=DrawEverything(pRobot,fRobot,pRobotFrame,pRoom,fRoom,pRoomFrame)
q = [0;0;0];
R = [1 0 0;
     0 -1 0;
     0 0 -1];
world.room = DrawRoom(Transform(q,R,pRoom),fRoom);
world.roomframe = DrawFrame(Transform(q,R,pRoomFrame));
world.robot = DrawQuadrotor(Transform(q,R,pRobot),fRobot);
world.robotframe = DrawFrame(Transform(q,R,pRobotFrame));

function robot = UpdateQuadrotor(robot,p)
set(robot,'vertices',p');

function frame = UpdateFrame(frame,p)
set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));

function room = UpdateRoom(room,p)
set(room,'vertices',p');

function robot = DrawQuadrotor(p,f)
robot = patch('Vertices',p','Faces',f,...
          'FaceColor','y','FaceAlpha',0.6,'EdgeAlpha',0.6);

function frame = DrawFrame(p)
frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);

function room = DrawRoom(p,f)
room = patch('Vertices',p','Faces',f,...
          'FaceColor',[.9 .7 .9],'FaceAlpha',0.1,'EdgeAlpha',0.1);

function [p,f]=GetRobotModel(filename)
load(filename);
p=p1;
f=faces;



%%%%%%%%%%%%%%%