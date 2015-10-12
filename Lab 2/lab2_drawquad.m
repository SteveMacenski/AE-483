function [output] = lab2_drawquad()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('QuadTestStandData.mat'); 
%params.movie_filename = 'quad.avi';

max_t = 25.4132000000000;
t = DrayMacenskiFester(:,1);
theta = DrayMacenskiFester(:,2);
theta_dot = DrayMacenskiFester(:,5);
dt = 0:(DrayMacenskiFester(11380) - DrayMacenskiFester(11379)):max_t;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% CREATE A QUADROTOR
load('hw2code_quadmodel.mat');  % This will provide variables p1, faces, colors


% SETUP THE PLOT
clf;
set(gcf,'Renderer','zbuffer');
axis([-1 1 -1 1 -0.5 1.5]);
axis equal;
hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE TO COMPUTE p2 TO INITIALIZE THE DRAWING

p0 = rotation(theta(1))*p1;
p2 = [1 0 0;...
      0 -1 0;...
      0 0 -1]*p0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot3(0,0,0,'k.','markersize',16);
h = patch('Vertices',p2','Faces',faces,...
          'CData',colors,'FaceColor','flat');
hTitle = title(sprintf('t = %4.2f',0));
lighting flat
light('Position',[0 -2 -1])
light('Position',[0 -2 1])
xlabel('x'); ylabel('y'); zlabel('z');
drawnow;
pause(0.5);

i = 1;
% myV = VideoWriter(params.movie_filename);
% myV.Quality = 100;
% open(myV);

while (i<length(t)-1)

        i = i+1;
        p0 = rotation(theta(i))*p1;
        p2 = [1 0 0;...
              0 -1 0;...
              0 0 -1]*p0;        

        set(h,'Vertices',p2');
        set(hTitle,'string',sprintf('t = %4.2f',t(i)));
        drawnow;

        % frame = getframe(gcf);
        % writeVideo(myV,frame);
end
 %close(myV);
end %%% END lab2_drawquad()



function R = rotation(phi)

R = [cos(phi) 0 sin(phi);...
     0 1 0;...
     -sin(phi) 0 cos(phi)];


end
