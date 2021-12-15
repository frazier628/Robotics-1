%% Setup

addpath DobotKinematics\ % forward and inverse kinematics

zz = zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
% arm lengths (m)
l1 = 101.03;
l2 = 135;
l3 = 160;
l4 = 45;
l5 = 98.5; % height pen is set to
% 
% q02 = 
% 
% R02 = rot(ey,q02);
% R03 = rot(ey,q02+q03);

p01 = l1*ez;
p12 = zz;
p23 = (l2*ez)%*R02;
p34 = (l3*ex)%*R03;
p4T = l4*ex - l5*ez;

h1 = ez;
h2 = ey; h3 = ey; h4 = ey;

dobot.P = [p01 p12 p23 p34 p4T];
dobot.H = [h1 h2 h3 h4];
dobot.joint_type = zeros(1,4);

q0 = zeros(4,1); % set initial pose
setJointAngles(q0, arduinoObj); % move arm to zero configuration
pause(1)
dobot = readDobot(arduinoObj,dobot); % read current configuration
disp(dobot.q)