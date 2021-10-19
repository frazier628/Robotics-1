%% Part 1

%MiniProject 3 Code
%By Chris Frazier
%With ample use of provided code from Dr. Wen

clear all; close all;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

load S_sphere_path

% plot the spherical S
figure(1);plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
hold on;
% add a 3d sphere
surf(X,Y,Z)
% make it transparent
alpha .5
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% convert to equal path length grid
diffS=vecnorm(diff(p_S')');
ls=[0 cumsum(diffS)];
lf=sum(diffS);
N=100;
l=(0:lf/N:lf);

pS=interp1(ls,p_S',l,'spline')';
% plot it out again with equal path length
figure(2);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');title('Quaternion Based Orientation')
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% check the path length is indeed equal
dlvec=vecnorm(diff(pS')');
figure(3);plot(dlvec,'x')
dl=mean(dlvec);
disp(max(abs(dlvec-dl)));

% save it as the path file
save S_sphere_path_uniform l pS

% find the end effector frame
pc=r*ez;
N=length(pS);
xT=zeros(3,N);zT=zeros(3,N);yT=zeros(3,N);
quat=zeros(4,N);
axisangle=zeros(3,N);
euler=zeros(3,N);
for i=1:N
    xT(:,i)=(pS(:,i)-pc)/norm(pS(:,i)-pc);
    if i<N
        yT(:,i)=(pS(:,i+1)-pS(:,i));
    else
        yT(:,i)=yT(:,i-1);
    end
    yT(:,i)=yT(:,i)-yT(:,i)'*xT(:,i)*xT(:,i);
    yT(:,i)=yT(:,i)/norm(yT(:,i));
    zT(:,i)=cross(xT(:,i),yT(:,i));
    R=[xT(:,i) yT(:,i) zT(:,i)];
    quat(:,i)=R2q(R);
    [euler(1,i),euler(2,i),euler(3,i)]=R2ypr(R);
    axisangle(:,i)=R2beta(R);
end

% plot out the end effector frame
m=5;
figure(2);h=plotTransforms(pS(:,1:m:end)',quat(:,1:m:end)');
set(h,'LineWidth',.5);
figure();plot(l,quat);
xlabel('Path Length (Lambda) (m)');ylabel('Unit Quaternion')
legend('q0','q1','q2','q3');title('Unit Quaternion vs Path Length')
figure();plot(l,euler);
xlabel('Path Length (Lambda) (m)');ylabel('Euler Angles (rad)')
legend('Yaw','Pitch','Roll');title('Euler Angles vs Path Length')
figure();plot(l,axisangle);
xlabel('Path Length (Lambda) (m)');ylabel('Angle-Axis Product (rad)')
legend('Beta_x','Beta_y','Beta_z');title('Angle-Axis Product vs Path Length')

%% Part 2a

%Provided proj3example.m code.

% ABB IRB 1200 parameters
L1=399.1/1000;
L2=448/1000;
L3=42/1000;
L4=451/1000;
L5=82/1000;

% P
p01=0*ex+L1*ez;
p12=zz;
p23=L2*ez;
p34=L3*ez+L4*ex;
p45=zz;
p56=zz;
p6T=L5*ex;

% H
h1=ez;
h2=ey;
h3=ey;
h4=ex;
h5=ey;
h6=ex;

% Final transformation
R6T=[-ez ey ex];

% define abb 1200 robot using POE convention
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];
irb1200.R6T=R6T;

% define collision body for abb 1200
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);

%Standard Denavit-Hartenberg Parameters
d1=L1;
d2=0;
d3=0;
d4=L4;
d5=0;
d6=L5;

a1=0;
a2=L2;
a3=L3;
a4=0;
a5=0;
a6=0;

theta1=0;
theta2=-pi/2;
theta3=0;
theta4=0;
theta5=0;
theta6=0;

alpha1=-pi/2;
alpha2=0;
alpha3=-pi/2;
alpha4=pi/2;
alpha5=-pi/2;
alpha6=0;

%Modified Denavit-Hartenberg Parameters
d1d=L1;
d2d=0;
d3d=0;
d4d=L4;
d5d=0;
d6d=0;
d7d=L5;

a1d=0;
a2d=0;
a3d=L2;
a4d=L3;
a5d=0;
a6d=0;
a7d=0;

theta1d=0;
theta2d=-pi/2;
theta3d=0;
theta4d=0;
theta5d=0;
theta6d=0;
theta7d=0;

alpha1d=0;
alpha2d=-pi/2;
alpha3d=0;
alpha4d=-pi/2;
alpha5d=pi/2;
alpha6d=-pi/2;
alpha7d=0;

%% Part 2b

%Utilize provided fwdkiniter, fwdkinsdh and fwdkinmdh code
%POE Forward Kinematics to calculate J_T and T_0T
q=(rand(6,1)-.5)*2*pi;
irb1200.q=q;
irb1200=fwdkiniter(irb1200);
disp('T_{0T} from POE method');
irb1200.T

%Set up SDH parameters
irb12001.d=[d1;d2;d3;d4;d5;d6];
irb12001.a=[a1;a2;a3;a4;a5;a6];
irb12001.theta=q+[theta1;theta2;theta3;theta4;theta5;theta6];
irb12001.alpha=[alpha1;alpha2;alpha3;alpha4;alpha5;alpha6];
%Calculate T_0T
irb12001=fwdkinsdh(irb12001);
% additional transformation to match with POE end effector frame
T6T=[[[0 0 1 ; 0 -1 0; 1 0 0] zeros(3,1)];[zeros(1,3) 1]];
% This should match with POE's T_{0T}
disp('T_{0T} from SDH method');
T0T_SDH=irb12001.T*T6T

% check
disp('difference between POE and SDH forward kinematics');
diffSDH=irb1200.T-T0T_SDH

% set up MDH parameters
irb12002.d=[d1d;d2d;d3d;d4d;d5d;d6d;d7d];
irb12002.a=[a1d;a2d;a3d;a4d;a5d;a6d;a7d]; % this is a_{i-1}
irb12002.alpha=[alpha1d;alpha2d;alpha3d;alpha4d;alpha5d;alpha6d;alpha7d];
irb12002.theta=[q;0]+[theta1d;theta2d;theta3d;theta4d;theta5d;theta6d;theta7d];
% calculate T_{07}
irb12002=fwdkinmdh(irb12002);
% additional transformation to match with POE end effector frame
T7T=[[[0 0 1 ; 0 -1 0; 1 0 0] zeros(3,1)];[zeros(1,3) 1]];
% This should match with POE's T_{0T|
disp('T_{0T} from MDH method');
T0T_MDH=irb12002.T*T7T

% check
disp('difference between POE and MDH forward kinematics');
diffMDH=irb1200.T-T0T_MDH

%% Part 2c

%forward input with the autogenerated q from previous section
disp('Randomly generated input q vector');
q
irb1200.q=q;
irb1200=fwdkiniter(irb1200);

%inverse input with solution from above
irb1200=invkin(irb1200);
disp('Output q vector of inverse kinematics');
irb1200.q

%% Part 2d

%using same q and variables as used previously
irb12001.P=[p01 p12 p23 p34 p45 p56 p6T];
irb12001.H=[h1 h2 h3 h4 h5 h6];
irb12001.joint_type=[0 0 0 0 0 0];
irb12001.R6T=R6T;
irb12001_rbt=defineRobot(irb12001,radius);
kincheckelbow(irb12001,irb12001_rbt)

%% Part 3

% 
% choose the inverse kinematics solution
%
for i=1:N
    % specify end effector SE(3) frame
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)]*R6T' pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    %
    irb1200=invkin(irb1200); 
    %
    for k=1:8
        q(:,i,k)=irb1200.q(:,k);
    end
    % check forward kinematics to make sure the IK solution is correct
    irb1200.q=q(:,i);
    irb1200=fwdkiniter(irb1200);
    for k=1:8
        irb1200.q=q(:,i,k);
        irb1200=fwdkiniter(irb1200);
        T{i,k}=irb1200.T;
    end
    % show robot pose (ever 5 frames)
end

% choose the pose to visualize
ksol=2;

for i=1:N
    % show robot pose (ever 5 frames)
    if mod(i,5)==0
        disp(norm(T{i,ksol}-Td{i}));
        figure(2);show(irb1200_rbt,q(:,i,ksol),'collision','on');pause(.5);
        view(150,10);
    end
end

%% Part 3 Velocity Work

umax=2; %rad/s joint angular velocity limit
tdot1=zeros(6,100);
tdot2=zeros(6,100);
tdot3=zeros(6,100);
tdot4=zeros(6,100);
tdot5=zeros(6,100);
tdot6=zeros(6,100);
tdot7=zeros(6,100);
tdot8=zeros(6,100);
dq31=zeros(1,100);
dq32=zeros(1,100);
dq33=zeros(1,100);
dq34=zeros(1,100);
dq35=zeros(1,100);
dq36=zeros(1,100);
dq37=zeros(1,100);
dq38=zeros(1,100);
%calculates theta_dot for each segment of path for both solutions
for i=1:length(ls)-1 
    tdot1(:,i)=(q(:,i+1,1)-q(:,i,1))./(ls(i+1)-ls(i));
    tdot2(:,i)=(q(:,i+1,2)-q(:,i,2))./(ls(i+1)-ls(i));
    tdot3(:,i)=(q(:,i+1,3)-q(:,i,3))./(ls(i+1)-ls(i));
    tdot4(:,i)=(q(:,i+1,4)-q(:,i,4))./(ls(i+1)-ls(i));
    tdot5(:,i)=(q(:,i+1,5)-q(:,i,5))./(ls(i+1)-ls(i));
    tdot6(:,i)=(q(:,i+1,6)-q(:,i,6))./(ls(i+1)-ls(i));
    tdot7(:,i)=(q(:,i+1,7)-q(:,i,7))./(ls(i+1)-ls(i));
    tdot8(:,i)=(q(:,i+1,8)-q(:,i,8))./(ls(i+1)-ls(i));
    dq31(i)=q(3,i+1,1)-q(3,i,1);
    dq32(i)=q(3,i+1,2)-q(3,i,2);
    dq33(i)=q(3,i+1,3)-q(3,i,3);
    dq34(i)=q(3,i+1,4)-q(3,i,4);
    dq35(i)=q(3,i+1,5)-q(3,i,5);
    dq36(i)=q(3,i+1,6)-q(3,i,6);
    dq37(i)=q(3,i+1,7)-q(3,i,7);
    dq38(i)=q(3,i+1,8)-q(3,i,8);
end
%determines path velocity based off of minimum of joint velocity divided by
%theta dot
lamdotmax1=min(min(abs(umax./tdot1)));
lamdotmax2=min(min(abs(umax./tdot2)));
lamdotmax3=min(min(abs(umax./tdot3)));
lamdotmax4=min(min(abs(umax./tdot4)));
lamdotmax5=min(min(abs(umax./tdot5)));
lamdotmax6=min(min(abs(umax./tdot6)));
lamdotmax7=min(min(abs(umax./tdot7)));
lamdotmax8=min(min(abs(umax./tdot8)));
%calculates time to complete path
time1=ls(end)/lamdotmax1;
time2=ls(end)/lamdotmax2;
time3=ls(end)/lamdotmax3;
time4=ls(end)/lamdotmax4;
time5=ls(end)/lamdotmax5;
time6=ls(end)/lamdotmax6;
time7=ls(end)/lamdotmax7;
time8=ls(end)/lamdotmax8;
[tps,tp]=min([time1;time2;time3;time4;time5;time6;time7;time8]);
dq=[max(dq31)-min(dq31);max(dq32)-min(dq32);max(dq33)-min(dq33);...
    max(dq34)-min(dq34);max(dq35)-min(dq35);max(dq36)-min(dq36);...
    max(dq37)-min(dq37);max(dq38)-min(dq38)];
%it ends up that all 8 poses have the same q3 range, so this simply picks
%pose 1 every time. discussed in report
[qpd,qp]=min(dq);
fprintf('Pose %d gives the highest path speed of %d m/s\n',tp,ls(end)/tps);
fprintf('Pose %d gives the smallest variation of q3, with a variation range of %d rad\n',qp,qpd);

%% Part 4

lsdot=.01; %lambda step size
%find w and v. v is found as simple time difference of p. w is equivalent
%to vee of R_0T_dot * R_0T' divided by time step
for i=1:N-1
    dt(i) = (ls(i+1)-ls(i))/lsdot;
    for k=1:8
        qdot(:,i,k)=(q(:,i+1,k)-q(:,i,k))/dt(i);
        Ri1=T{i+1,k}(1:3,1:3);
        Ri=T{i,k}(1:3,1:3);
        w(:,i,k)=vee(Ri1*Ri'-eye(3,3))/dt(i);
        pi1=T{i+1,k}(1:3,4);
        pi=T{i,k}(1:3,4);
        v(:,i,k)=(pi1-pi)/dt(i);
    end
end

for k=1:8
   for i=1:6
       maxqdot(i,k)=max(qdot(i,:,k));
   end
   fprintf('maximum qdot for pose %d \n', k);
   disp(maxqdot(:,k)');   
end

figure();plot(ls(1:end-1),v(:,:,tp));
xlabel('Path Length (Lambda) (m)');ylabel('Linear Velocity (m/s)')
legend('vx','vy','vz');title('Linear End Effector Spatial Velocity vs Path Length')
figure();plot(ls(1:end-1),w(:,:,tp));
xlabel('Path Length (Lambda) (m)');ylabel('Angular Velocity (red/s)')
legend('wx','wy','wz');title('Angular End Effector Spatial Velocity vs Path Length')

