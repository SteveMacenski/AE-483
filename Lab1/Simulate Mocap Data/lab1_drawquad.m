function [output] = lab1_drawquad(mocap)
%
% input: "mocap" is a n-by-N matrix, where n is the number of fields
%         in the mocap message object, and N is the number of sample 
%         measurements over time.
%
% note for steve: call function in format lab1_drawquad(mocap)
mocap = transpose(mocap);
t = mocap(1,:);                             %parsing input the for data columns
dt = mocap(1,2:end) - mocap(1,1:end-1);
 
q10 = mocap(2:4,:);
phi = mocap(5,:);
theta = mocap(6,:);
psi = mocap(7,:);

% CREATE WORLD OBJECTS
load('world.mat');  % this will load variables:   w0  wsz  wcolors
% CREATE A QUADROTOR
load('quadmodel.mat');  % this will load variables:  p1   faces   colors

%Rotation R02

R02 = [1 0 0;...
       0 -1 0;...
       0 0 -1];
        
% SETUP THE PLOT
clf;
set(gcf,'Renderer','zbuffer');
axis([-4 4 -4 4 -0.1 3.5]);
axis equal;
hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE TO COMPUTE p2 and w2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROTATE FROM BODY FRAME TO MATLAB PLOT FRAME

    
    p2 = [1 0 0;0 -1 0;0 0 -1]*rotationMatrixFromEulerAngles(phi(1),theta(1),psi(1))*p1;
    

% ROTATE FROM WORLD FRAME TO MATLAB PLOT FRAME
    w2 = [1 0 0;0 -1 0;0 0 -1]*w0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scatter3(w2(1,:), w2(2,:), w2(3,:), wsz, wcolors,'filled');
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

% ANIMATE THE RESULTS
i = 1;
tic;
while (i<length(t)-1)
    if (toc > dt(i))
        tic;
        i = i+1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % YOUR CODE HERE TO COMPUTE p0
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % TRANSFORM FROM BODY TO WORLD FRAME
        p0 = rotationMatrixFromEulerAngles(phi(i),theta(i),psi(i))*p1;
        
        p0 = p0+repmat(q10(:,i),1,294);

        % TRANSFORM FROM WORLD FRAME TO MATLAB DISPLAY FRAME
        p2 = [1 0 0;...
              0 -1 0;...
              0 0 -1]*p0;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % UPDATE GRAPHICS OBJECT VERTICES
        set(h,'Vertices',p2');
        set(hTitle,'string',sprintf('t = %4.2f',t(i)));
        drawnow;
    end
end

end  %%% END lab1_drawquad()



function [output] = rotationMatrixFromEulerAngles(phi, theta, psi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE TO COMPUTE ROTATION MATRIX FROM ZYX Euler Angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

output = [cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*cos(phi);...
          sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(phi)*sin(phi);...
          -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
