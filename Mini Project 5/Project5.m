%Driver code for MiniProect 5 by Chris Frazier
clear; close all
%% %Part 1

% plot the spherical S
load S_sphere_path
figure();plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
hold on;
% add a 3d sphere
surf(X,Y,Z)
% make it transparent
alpha .5
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

cam_def;

Poc=[1.2;0;0.5];
Roc=[0,0,-1;-1,0,0;0,1,0];
Toc=[Roc,Poc;0,0,0,1];
[uv1,uvw1,P11]=cam_image(cam,Toc,p_S);
figure()
plot(uv1(1,:),uv1(2,:),'x')
xlabel('Image Plane Horizontal Axis');ylabel('Image Plane Vertical Axis');
title('Camera Frame Location [1.2;0;0.5]');
xlim([0 1280]);ylim([0 1024]);
set(gca, 'XDir','reverse')

Poc=[2.5;0;0.5];
Toc=[Roc,Poc;0,0,0,1];
[uv2,uvw2,P12]=cam_image(cam,Toc,p_S);
figure()
plot(uv2(1,:),uv2(2,:),'x')
xlabel('Image Plane Horizontal Axis');ylabel('Image Plane Vertical Axis');
title('Camera Frame Location [2.5;0;0.5]');
xlim([0 1280]);ylim([0 1024]);
set(gca, 'XDir','reverse')

%% %Part 2

camcalib;

%% %Part 3 and 4

SVS;
