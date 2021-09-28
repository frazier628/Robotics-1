%% Part 1

%MiniProject 2 Code
%By Chris Frazier
%With use of provided code from Dr. Wen

clear all; close all

%load S path
load S_letter_path
%calculate xT and yT for each point of S
[xT,yT]=setR0T(Sls);
%Plot S and its outward normal
figure(1);plot(Sls(1,:),Sls(2,:),'o','linewidth',2);
hold on
for i=1:length(Sls)
    quiver(Sls(1,i),Sls(2,i),-1*xT(1,i)./2,-1*xT(2,i)./2,'r');
end
xlabel('x');ylabel('y');
axis([0 3 -1.7 1.7]);axis('square');grid;
%parameterization of S as a path
lambda=zeros(length(Sls)-1,1);
for i=2:length(Sls)-1
    lambda_f=norm(Sls(:,i)-Sls(:,i-1));
    lambda(i)=lambda_f+lambda(i-1);
end

%% Part 3
%define principal axes
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
%define robot link lengths
l1 = 1.5; l2 = 1.5; l3 = .5;
l=[l1;l2;l3];
%test input q
qin=[pi/8,-pi/8,1];
%test T and J output
[Ttest,Jtest]=forwardkin3(qin,l);
%assign qualities of robot
robot.P = [zz l1*ex l2*ex l3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];
% zero configuration 
robot.q=[0;0;0];
figure();hold on;plotarm(robot);
xlabel('x-axis');ylabel('y-axis');
title('Planar RRR arm in zero configuration (q_1=q_2=q_3=0)')
hold off
%solve q solutions based off of test T and plot
qsol=invkin3(Ttest,l);
robot.q=qin;
figure();hold on;plotarm(robot);
xlabel('x-axis');ylabel('y-axis');
axis equal
title('Planar RRR arm in forward test configuration')
hold off
%test configurations
robot.q=qsol(:,1);
figure();hold on;plotarm(robot);
robot.q=qsol(:,2);plotarm(robot);
xlabel('x-axis');ylabel('y-axis');
axis equal
title('Planar RRR arm in inverse test configuration')
hold off

%% Part 4
%initialize solution vectors and for loop
nl=length(Sls);
qsol1=zeros(3,nl-1);
qsol2=zeros(3,nl-1);
tic
for i=1:nl-1
    %assign pose T to point of S and run through inverse kinematics
    robot.T(1:3,1:4)=[xT(:,i) yT(:,i) ez [Sls(:,i);0]];
    qsol=threelink_invkin_geometric(robot);
    qsol1(:,i)=qsol(:,1);
    qsol2(:,i)=qsol(:,2);    
end
timein=toc %time counter
%plot arm following S path with both solutions
figure();plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
for i=1:3:nl-1
    robot.q=qsol1(:,i);pause(.1);
    plotarm(robot);
end
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol1')')')');

figure();plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
for i=1:3:nl-1
    robot.q=qsol2(:,i);
    plotarm(robot);pause(.1);
end
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol2')')')');

%plotting q against lambda for both solutions
figure()
plot(lambda,qsol1)
xlabel('Path Length (Lambda)')
ylabel('Arm Segment Angle (q) (rad)')
legend('q1','q2','q3')
title('Solution 1 for S-Path')
figure()
plot(lambda,qsol2)
xlabel('Path Length (Lambda)')
ylabel('Arm Segment Angle (q) (rad)')
legend('q1','q2','q3')
title('Solution 2 for S-Path')

%% Part 5
umax=1;
tdot1=zeros(3,100);
tdot2=zeros(3,100);
%calculates theta_dot for each segment of path for both solutions
for i=1:length(lambda)-1 
    tdot1(:,i)=(qsol1(:,i+1)-qsol1(:,i))./(lambda(i+1)-lambda(i));
    tdot2(:,i)=(qsol2(:,i+1)-qsol2(:,i))./(lambda(i+1)-lambda(i));
end
%determines path velocity based off of minimum of joint velocity divided by
%theta dot
lamdotmax1=min(min(abs(umax./tdot1)));
lamdotmax2=min(min(abs(umax./tdot2)));
%calculates time to complete path
time1=lambda(end)/lamdotmax1;
time2=lambda(end)/lamdotmax2;

%% Part 6
qsolj=zeros(3,nl-1);
tic
%runs through a for loop to call the jacobian inverse function to find q
%solution and then plot for 3-link
for i=1:nl-1
    robot.T(1:3,1:4)=[xT(:,i) yT(:,i) ez [Sls(:,i);0]];
    qsolj(:,i)=invjac(robot); 
end
timej=toc
figure();plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
for i=1:3:nl-1
    robot.q=qsolj(:,i);pause(.1);
    plotarm(robot);
end
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsolj')')')');

%% Part 7
%defines new 10-link robot arm
lg=ones(1,10)*.35;
robot.P = [zz .35*ex.*ones(3,10)];
robot.H = [zeros(2,10);ones(1,10)];
robot.joint_type=zeros(1,10);
qsolg=zeros(10,nl-1);
tic
%runs through a for loop to call the jacobian inverse function to find q
%solution and then plot for 10-link
for i=1:nl-1
    robot.T(1:3,1:4)=[xT(:,i) yT(:,i) ez [Sls(:,i);0]];
    qsolg(:,i)=invjac10(robot);
end
time10=toc
figure();plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
for i=1:3:nl-1
    robot.q=qsolg(:,i);pause(1);
    plotarm(robot);
end

%% %Functions
%xT and yT curve generation
%input - S curve
%output - xT and yT axes
%method - yT = unit vecotor of line between two points on curve
%xT = cross(yT and the z-vector)
function [x,y] = setR0T(curve)
    zT=[0;0;1];
    y=zeros(3,length(curve));
    x=zeros(3,length(curve));
    for i=1:(length(curve)-1)
        y(1:2,i)=[curve(1,i+1)-curve(1,i);curve(2,i+1)-curve(2,i)];
        y(1:2,i)=y(1:2,i)./norm(y(1:2,i));
        x(:,i)=cross(y(:,i),zT);
    end
end

%3link forward kinematics function
%input - q angles and l robot link lengths
%output - pose T and jacobian J
%works by fiding R0T, p0T and derivatives of xT and yT
function [T,J] = forwardkin3(q,l)
    R0T=[cos(q(1)+q(2)+q(3)),-sin(q(1)+q(2)+q(3)),0;sin(q(1)+q(2)+q(3)),...
        cos(q(1)+q(2)+q(3)),0;0,0,1];
    p0T=zeros(3,1);
    p0T(1)=l(1)*cos(q(1))+l(2)*cos(q(1)+q(2))+l(3)*cos(q(1)+q(2)+q(3));
    p0T(2)=l(1)*sin(q(1))+l(2)*sin(q(1)+q(2))+l(3)*sin(q(1)+q(2)+q(3));
    T=zeros(4,4);
    T(1:3,1:3)=R0T;
    T(1:3,4)=p0T;
    T(4,4)=1;
    J=ones(3,3);
    J(2,:)=[-1*l(1)*sin(q(1))-l(2)*sin(q(1)+q(2))-l(3)*sin(q(1)+q(2)+q(3)),...
        -1*l(2)*sin(q(1)+q(2))-l(3)*sin(q(1)+q(2)+q(3)),-1*l(3)*sin(q(1)+q(2)+q(3))];
    J(3,:)=[l(1)*cos(q(1))+l(2)*cos(q(1)+q(2))+l(3)*cos(q(1)+q(2)+q(3)),...
        l(2)*cos(q(1)+q(2))+l(3)*cos(q(1)+q(2)+q(3)),l(3)*cos(q(1)+q(2)+q(3))];
end

%3link inverse kinematics function
%input - T pose matrix and link lengths, l
%output - q link angles
%finds two q solutions based off of q2 having to solutions due to sign loss
%from trig functions so out put of acos is both + and -
function [q] = invkin3(T,l)
    R0T=T(1:3,1:3);
    p0T=T(1:3,4);
    xT=p0T(1);
    yT=p0T(2);
    q2(1)=acos(((xT^2)+(yT^2)+((l(3)^2)*(R0T(1,1)^2))+((l(3)^2)*(R0T(2,1)^2))...
        -xT*2*l(3)*R0T(1,1)-yT*2*l(3)*R0T(2,1)-(l(1)^2)-(l(2)^2))/(2*l(1)*l(2)));
    q2(2)=-1*q2(1);
    for i=1:2
        p1=[l(1)+l(2)*cos(q2(i));l(2)*sin(q2(i));0];
        p2=[xT-l(3)*R0T(1,1);yT-l(3)*R0T(2,1);0];
        q1(i)=2*sign(transpose(cross(p1,p2))*[0;0;1])*atan(norm(p1-p2)/norm(p1+p2));
        q3_c=wrapToPi(acos(R0T(1,1))*[-1;1]-q1(i)-q2(i));
        q3_s=wrapToPi([asin(R0T(2,1)),pi-asin(R0T(2,1))]-q1(i)-q2(i));
        tol=1e-5;
        q3(1,i)=nonzeros(ismembertol(q3_c,q3_s,tol)'*q3_c);
    end
    q=[q1;q2;q3];
end

%3-link S path inverse kinematics
%repeat of previous function adapted for the S path 
function q = threelink_invkin_geometric(robot)
    l=robot.P(1,2:4);
    R0T=robot.T(1:2,1:2);
    p0T=robot.T(1:2,4);
    xT=p0T(1);
    yT=p0T(2);
    q2(1)=acos(((xT^2)+(yT^2)+((l(3)^2)*(R0T(1,1)^2))+((l(3)^2)*(R0T(2,1)^2))...
        -xT*2*l(3)*R0T(1,1)-yT*2*l(3)*R0T(2,1)-(l(1)^2)-(l(2)^2))/(2*l(1)*l(2)));
    q2(2)=-1*q2(1);
    for i=1:2
        p1=[l(1)+l(2)*cos(q2(i));l(2)*sin(q2(i));0];
        p2=[p0T(1)-l(3)*R0T(1,1);p0T(2)-l(3)*R0T(2,1);0];
        q1(i)=2*sign(transpose(cross(p1,p2))*[0;0;1])*atan(norm(p1-p2)/norm(p1+p2));
        q3_c=wrapToPi(acos(R0T(1,1))*[-1;1]-q1(i)-q2(i));
        q3_s=wrapToPi([asin(R0T(2,1)),pi-asin(R0T(2,1))]-q1(i)-q2(i));
        tol=1e-10;
        q3(1,i)=nonzeros(ismembertol(q3_c,q3_s,tol)'*q3_c);
    end
    q=[q1;q2;q3];
end

%3-link S path inverse jacobian kinematics
%input - robot structure
%output - q solution
%uses Newton's method to iterate towards solution for each point of S
function q = invjac(robot)
    l=robot.P(1,2:4);
    qk=[1;1;1];
    p0Td=robot.T(1:2,4);
    error = 1;
    alpha=.01;
    k=1;
    tol=1e-10;
    qT_c=wrapToPi(acos(robot.T(1,1))*[-1;1]);
    qT_s=wrapToPi([asin(robot.T(2,1)),pi-asin(robot.T(2,1))]);
    qTd=nonzeros(ismembertol(qT_c,qT_s,tol)'*qT_c);
    xd=[qTd;p0Td];
    while max(error) > tol
        [T,J]=forwardkin3(qk,l);
        qTk_c=wrapToPi(acos(T(1,1))*[-1;1]);
        qTk_s=wrapToPi([asin(T(2,1)),pi-asin(T(2,1))]);
        qTk=nonzeros(ismembertol(qTk_c,qTk_s,tol)'*qTk_c);
        fqk=[qTk;T(1:2,4)];
        error=fqk-xd;
        qk=qk-alpha*pinv(J)*error;
        k=k+1;
    end
    q=qk;
end 

%10-link S path inverse jacobian kinematics
%adaptation of the previous code for a 10-link robot
function q = invjac10(robot)
    l=robot.P(1,2:11);
    qk=ones(10,1);
    p0Td=robot.T(1:2,4);
    error = 1;
    alpha=.01;
    k=1;
    tol=1e-10;
    qT_c=wrapToPi(acos(robot.T(1,1))*[-1;1]);
    qT_s=wrapToPi([asin(robot.T(2,1)),pi-asin(robot.T(2,1))]);
    qTd=nonzeros(ismembertol(qT_c,qT_s,tol)'*qT_c);
    xd=[qTd;p0Td];
    while abs(max(error)) > tol
        [T,J]=forwardkinn(qk,l);
        qTk_c=wrapToPi(acos(T(1,1))*[-1;1]);
        qTk_s=wrapToPi([asin(T(2,1)),pi-asin(T(2,1))]);
        qTk=nonzeros(ismembertol(qTk_c,qTk_s,tol)'*qTk_c);
        fqk=[qTk;T(1:2,4)];
        error=fqk-xd;
        qk=qk-alpha*pinv(J)*error;
        k=k+1;
    end
    q=qk;
end 

%forward kinematics for n links
%adaptation of 3-link code to use n-link solution
function [T,J] = forwardkinn(q,l)
    n=length(q);
    R0T=[cos(sum(q)),-sin(sum(q)),0;sin(sum(q)),...
        cos(sum(q)),0;0,0,1];
    p0T=zeros(3,1);
    J=ones(3,n);
    for i=1:n
        qj=0;
        for j=1:i
            qj=qj+q(j);
        end
        R0i=[cos(qj),-sin(qj),0;sin(qj),cos(qj),0;0,0,1];
        p0T=p0T+R0i*[l(i);0;0];
    end
    qs=0;
    for i=1:n
        qs=qs+q(i);
        R0i=[cos(qs),-sin(qs),0;sin(qs),cos(qs),0;0,0,1];
        piT=R0i*[l(i);0;0];
        qj=qs;
        for j=(i+1):n
            qj=qj+q(j);
            R0j=[cos(qj),-sin(qj),0;sin(qj),cos(qj),0;0,0,1];
            piT=piT+R0j*[l(j);0;0];
        end
        ezp=cross([0;0;1],piT);
        J(2:3,i)=ezp(1:2);
    end
    T=zeros(4,4);
    T(1:3,1:3)=R0T;
    T(1:3,4)=p0T;
    T(4,4)=1;
end