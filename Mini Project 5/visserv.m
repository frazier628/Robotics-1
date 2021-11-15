clear;

% define a pinhole camera

cam_def;

% load S-shape curve in reference frame
load S_sphere_path.mat
load S_sphere_path_uniform.mat

% define ABB IRB 1200 robot

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
L1=399.1/1000;L2=448/1000;L3=42/1000;L4=451/1000;L5=82/1000;
% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;
% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;
%
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];

% translation of the robot base from the reference frame of the S-shape
% (assuming same orientation) represented in the reference frame
P_0B=[1;0;0];
% translation of the camera frame origin from end effector frame (assuming
% same orientation) represented in the end effector frame
P_TC=[0;0;0.05];
% ****
T_TC=[[0;-1;0] [0;0;1] [-1;0;0],P_TC;[0 0 0 1]];
T_0B=[eye(3,3) P_0B;[0 0 0 1]];

%target p0T and pTC
p0T=[1.8;0;0.5];
T_0T_targ=[[-1;0;0] [0;-1;0] [0;0;1],p0T;[0 0 0 1]];
T_0C_targ=T_0T_targ*T_TC;

%starting robot position
q0=[.1;1.6;3.5;3;2;0];

irb1200.q=q0;
irb1200=fwdkiniter(irb1200);
T_0C=T_0B*irb1200.T*T_TC;
%T_BT=irb1200.T;
%T_B0=inv(T_0B);
%T_BC=T_B0*T_0C;
[uv,uvw,P1]=cam_image(cam,T_0C,pS);
[T_C0_est,Zest]=pnp_general(uv,pS(:,(102-length(uv)):101),cam.K);
T_C0_est
T_C0=inv(T_0C)
disp(norm(T_C0-T_C0_est));
figure();plot(uv(1,:),uv(2,:),'x','linewidth',2);hold on
xlabel('Image Plane Horizontal Axis');
ylabel('Image Plane Vertical Axis');
p0C=T_0C(1:3,4);
title(['Camera Frame Location: '...
    '[',num2str(p0C(1)),',',num2str(p0C(2)),',',num2str(p0C(3)),']']);
axis([0 cam.uv0(1)*2 0 cam.uv0(2)*2]);
set(gca, 'xdir', 'reverse');