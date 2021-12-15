% Robotics 1 Course Project
clear,clc,close all
%% ***TO DO*****
%1) update picture saving path for windows camera application for functions
% https://windowsreport.com/windows-8-camera-picture-location/
%2) Update syntax for inverse kinematics functions
%3) install computer vision toolbox
%4) print out fixed position paper for dobot (see webex diagram)
%5) change camera picture positions so that grid is in view
%6) change offset between dobot zero and calibration grid zero
%7) change offset between camera and pen tip

%% Connect Dobot to Serial Communication

connectDobotSerial;

%% Robot Initialization
%load calibration parameters/robot objects and set to zero position

DobotParams;
%!!! change L5 - height of drawing effector
setJointAngles([0;5;1;0],arduinoObj);pause(1);

%% Camera Calibration

%Capture calibration images
%Make sure picture save directory matches directory in intrinsic_calc()
%Make sure that the zero position captures the calibration grid and maze
Cal_Cap(arduinoObj);

%Calculate intrinsic and extrinsic parameters
%Before running, rename photos to 'photo1, photo2, photo3.jpg ...'
%Verify that square size (edge length of square in cal grid) is correct
[cameraParams, R, t] = intrinsic_calc() %Return camera information

%Find camera location in world frame (with respect to calibration grid)
[~, cameraLocation] = extrinsicsToCameraPose(R, t);

%% Maze Solver

%Using final picture, identify maze starting point
[start_pt, outline] = processMazeImage('C:\Users\frazic\Documents\MATLAB\Robotics Project\Photos\image11.jpg');

%Using maze starting point, generate maze solution
path = solveMaze(start_pt, outline);

%Plot maze solution
figure;
plot(outline(1,:),outline(2,:))
hold on
plot(path(2,:),path(1,:))
axis square
set ( gca, 'ydir', 'reverse' )

maze_pixels = [path(2,:)' , path(1,:)'];

%Apply XYZ correction for location of calibration grid relative to dobot
x_cor = 266.7; %mm away from bot
y_cor = 38.1; %mm away from bot
z_cor = 0;

%Apply XYZ correction for location of pen relative to camera
x_pen = 29.21;
y_pen = 0;
z_pen = 50.8 + 10; %mm

for i = 1:length(maze_pixels)
    
    %Convert solution pixel coordinates to world frame coordinates
    XYZ_points = pointsToWorld(cameraParams, R, t, maze_pixels(i,:));
    
    %Apply dobot frame correction (from calibration grid to dobot base)
    %     XYZ_points = XYZ_points + [x_cor,y_cor];
    
    %Append Z coordinates
    XYZ_coords(i,:) = [XYZ_points, 0];
    
    %Adjust to pen position
    %     XYZ_points = XYZ_coords(i,:) + [x_cor, y_cor, z_cor];
    
    %Convert solution path into joint trajectory
    %     q_path(:,i) = invkin_us(XYZ_points);
    
end

%%
%Plot XYZ coordinates
figure;
plot(XYZ_coords(:,1),XYZ_coords(:,2))
axis square

for i=1:length(XYZ_coords)
    Ry=[cos(pi),0,sin(pi);0,1,0;-sin(pi),0,cos(pi)];
    Rz=[cos(-pi/2),-sin(-pi/2),0;sin(-pi/2),cos(-pi/2),0;0,0,1];
    XYZ_dobot(i,:)=(Ry*Rz*XYZ_coords(i,:)')';
    XYZ_dobot(i,3)=-3.75;
    XYZ_dobot(i,2)=XYZ_dobot(i,2)-45;
    XYZ_dobot(i,1)=XYZ_dobot(i,1)+110;
end

figure;
plot(XYZ_dobot(:,1),XYZ_dobot(:,2),'*-')
axis square

%%
pp=XYZ_dobot';
N = length(pp);
for i=1:N
    xT(:,i)=(pp(:,i))/norm(pp(:,i)); % unit vector from base to point in path
    yT(:,i) = rot(ez,pi/2)*xT(:,i); % yT = Rotation of xT about z by 90 degrees
    zT(:,i)=cross(xT(:,i),yT(:,i)); % zT = xT x yT always into the paper
end
for i = 1:length(pp)
    dobot.T = [[xT(:,i) yT(:,i) zT(:,i)] pp(:,i); [0 0 0 1]];
    dobot = invkin_us(dobot);
    dobot = anglelim(dobot);
    q_path{i}=rad2deg(dobot.q);
    q_path{i}(3)=q_path{i}(3)+q_path{i}(2);
    q_path{i}(2)=q_path{i}(2)+5;
    q_path{i}(3)=q_path{i}(3)+1;
end


%Return to zero position after maze solution
q_path{length(pp)+1} = [0;0;0;0];
q_path{length(pp)+1}(3)=q_path{length(pp)+1}(3)+q_path{length(pp)+1}(2);
q_path{length(pp)+1}(2)=q_path{length(pp)+1}(2)+5;
q_path{length(pp)+1}(3)=q_path{length(pp)+1}(3)+1;

%% Execute Maze Solution


for i = 1:length(pp)+1
    
    %Apply forward kinematics to joint solution
    setJointAngles(q_path{i},arduinoObj);pause(1);
end










