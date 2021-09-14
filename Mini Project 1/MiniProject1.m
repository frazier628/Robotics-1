%Robotics MiniProject 1
%Chris Frazier in conversation with Brian Deiss
%Made with utilizaiton of provided example code and trapgen function
%by Professor Wen. All credit to the code/comments utilized.
%Code based on this includes parts 3c, 3d, and the trapgen function

clear all; close all
%%
%Part 1

%defines initial position/orientation and generates room
p01=[4;.5;.15];
roomgen(p01,90,1);

%Part 2a
%defines a* position/orientation and generates room
pastar=[1.55;4;.15];
roomgen(pastar,180,1);

%Part 2b
%defines b* position/orientation and generates room
pbstar=[1;1.1;.15];
roomgen(pbstar,270,1);
%%
%Part 3b

%Number of iterative steps for each line segments
N=20;
%move north
%sets beginning and end point
P0=p01;
P1=[4;4;.15];
%defines lambda_f for line segment and then generates linear path
lambda_1=norm(P1-P0);
lambda01=0:lambda_1/N:lambda_1;
%uses the position parameterization equation to calculate position as
%a function of lambda
p_lambda1=(1-lambda01/lambda_1).*P0+(lambda01/lambda_1).*P1;
n1=length(lambda01);
theta1=ones(1,n1)*90;

%move west
P2=pastar;
lambda_2=norm(P2-P1);
lambda12=(0:lambda_2/N:lambda_2);
p_lambda2=(1-(lambda12)/lambda_2).*P1+((lambda12)/lambda_2).*P2;
n2=length(lambda12);

%face west
%defines beginning and ending theta
theta0=90;
thetaf=180;
%uses the angle parameterization equation to calculate orientation as
%a function of lambda
theta2=(1-(lambda12)/lambda_2).*theta0+((lambda12)/lambda_2).*thetaf;

%move south
P3=[1.55;3;.15];
lambda_3=norm(P3-P2);
lambda23=(0:lambda_3/N:lambda_3);
p_lambda3=(1-(lambda23)/lambda_3).*P2+((lambda23)/lambda_3).*P3;
n3=length(lambda23);

%face south
theta0=180;
thetaf=270;
theta3=(1-(lambda23)/lambda_3).*theta0+((lambda23)/lambda_3).*thetaf;

%move southwest
P4=pbstar;
lambda_4=norm(P4-P3);
lambda34=(0:lambda_4/N:lambda_4);
p_lambda4=(1-(lambda34)/lambda_4).*P3+((lambda34)/lambda_4).*P4;
n4=length(lambda34);
theta4=ones(1,n4)*270;

%collects all the lambdas, paths, and angles spaces into one large array
%for each so it can be plotted as a whole
l=[lambda01 lambda12+lambda01(end) lambda23+lambda01(end)+lambda12(end)...
    lambda34+lambda01(end)+lambda12(end)+lambda23(end)];
nl=length(l);
path=[p_lambda1(1:2,:) p_lambda2(1:2,:) p_lambda3(1:2,:) p_lambda4(1:2,:)];
angle=[theta1 theta2 theta3 theta4];
%generates the room for the combined total path
roomgen(path,angle,nl);

%plotting of robot position along path
figure()
plot(l,path,'linewidth',2);
legend('x','y');
title('Robot Position vs Path Length');
xlabel('lambda(m)');ylabel('x-y position of Robot (m)');
%plotting of robot orientation along path
figure()
plot(l,angle,'linewidth',2);
title('Robot Orientation vs Path Length');
xlabel('lambda(m)');ylabel('angle (deg)');
%%
%Part 3c

%assuming instantenous acceleration and deacceleration 
%defines maximum linear and angular velocities
umax=[2;2];
wmax=1;
%max speed for segment 1
%derivative of p(lambda) equation
p1dot=(P1(1:2)-P0(1:2))/lambda_1;
%calculating maximum lambda_dot from the derivative and umax
lamdotmax1=min(abs(umax./p1dot));
%calculating time to complete path segment
T1=lambda_1/lamdotmax1;
%max speed for segment 2
p2dot=(P2(1:2)-P1(1:2))/lambda_2;
%derivative of theta(lambda) equation
t2dot=((180-90)*(pi/180))/lambda_2;
%calculating maximum lambda_dot from the two derivatives and umax/wmax
lamdotmax2=max(min(abs(umax./p2dot),abs(wmax/t2dot)));
T2=lambda_2/lamdotmax2;
%max speed for segment 3
p3dot=(P3(1:2)-P2(1:2))/lambda_3;
t3dot=((270-180)*(pi/180))/lambda_3;
lamdotmax3=max(min(abs(umax./p3dot),abs(wmax/t3dot)));
T3=lambda_3/lamdotmax3;
%max speed for segment 4
p4dot=(P4(1:2)-P3(1:2))/lambda_4;
lamdotmax4=min(abs(umax./p4dot));
T4=lambda_4/lamdotmax4;

%segment 1
%defines time step for segment
ts=T1/N;
%creates linspace of time for segment
t1=(0:ts:T1);
%simple length determiner
NT1=length(t1);
%generate position array based on time
pt1=zeros(2,NT1);
pt1(:,1)=P0(1:2);
%for loop goes through each time step to determine position at each time t
for i=1:NT1-1
    pt1(:,i+1)=pt1(:,i)+ts*p1dot*lamdotmax1;
end
tt1=zeros(1,NT1);
tt1(:)=90;
%segment 2
ts=T2/N;
t2=(T1:ts:T1+T2);
NT2=length(t2);
pt2=zeros(2,NT2);
pt2(:,1)=pt1(:,NT1);
%similarly generating orientation array based on time
tt2=zeros(1,NT2);
tt2(1)=tt1(NT1);
%for loop goes through each time step to determine position at each time t
%as well as orientation at eaach time t
for i=1:NT2-1
    pt2(:,i+1)=pt2(:,i)+ts*p2dot*lamdotmax2;
    tt2(:,i+1)=tt2(:,i)+ts*t2dot*lamdotmax2*180/pi;
end
%segment 3
ts=T3/N;
t3=(t2(end):ts:t2(end)+T3);
NT3=length(t3);
pt3=zeros(2,NT3);
pt3(:,1)=pt2(:,NT2);
tt3=zeros(1,NT3);
tt3(1)=tt2(NT2);
for i=1:NT3-1
    pt3(:,i+1)=pt3(:,i)+ts*p3dot*lamdotmax3;
    tt3(:,i+1)=tt3(:,i)+ts*t3dot*lamdotmax3*180/pi;
end
%segment 4
ts=T4/N;
t4=(t3(end):ts:t3(end)+T4);
NT4=length(t4);
pt4=zeros(2,NT4);
pt4(:,1)=pt3(:,NT3);
for i=1:NT4-1
    pt4(:,i+1)=pt4(:,i)+ts*p4dot*lamdotmax4;
end
tt4=zeros(1,NT4);
tt4(:)=270;
%collects all the times, paths, and angles spaces into one large array
%for each so it can be plotted as a whole
t=[t1 t2(2:NT2) t3(2:NT3) t4(2:NT4)];
pt=[pt1 pt2(:,2:NT2) pt3(:,2:NT3) pt4(:,2:NT4)];
tt=[tt1 tt2(:,2:NT2) tt3(:,2:NT3) tt4(:,2:NT4)];
%calculates lambdas based on position as a function of time
lt=[0 cumsum(vecnorm(diff(pt')'))];
%plot robot position against time
figure()
plot(t,pt,'linewidth',2);
legend('x','y');
xlabel('time (sec)');
ylabel('robot position (m)');
title('Robot Position vs Time');
%plot robot orientation against time
figure()
plot(t,tt,'linewidth',2);
xlabel('time (sec)');
ylabel('robot angle (deg)');
title('Robot Angle vs Time');
%plot path position against time
figure;
plot(t,lt,'linewidth',2);
xlabel('time (sec)');
ylabel('path length (lambda) (m)');
title('Lambda vs Time');
%%
%Part 3d

%creating spline curve in the movement w/ constant velocity
% # of points on the path in the spline region from the break point
n_p = 10;
% # of points to include in the spline (n_a close to n_p means closer path
% following)
n_a = 8;
% # of segments in spline region
n_s = 50;
%spline interpolation of lambda, position and angle in order to remove
%kinks
%segment 1
l_s1 = [l(n1-n_p):(l(n1+n_p)-l(n1-n_p))/n_s:l(n1+n_p)];
p_s1 = spline([l(n1-n_p:n1-n_p+n_a) l(n1+n_p-n_a:n1+n_p)],...
    path(:,[n1-n_p:n1-n_p+n_a,n1+n_p-n_a:n1+n_p]),l_s1);
t_s1 = spline([l(n1-n_p:n1-n_p+n_a) l(n1+n_p-n_a:n1+n_p)],...
    angle(:,[n1-n_p:n1-n_p+n_a,n1+n_p-n_a:n1+n_p]),l_s1);
%segment 2
l_s2 = [l((n1+n2)-n_p):(l((n1+n2)+n_p)-l((n1+n2)-n_p))/n_s:l((n1+n2)+n_p)];
p_s2 = spline([l((n1+n2)-n_p:(n1+n2)-n_p+n_a) l((n1+n2)+n_p-n_a:(n1+n2)+n_p)],...
    path(:,[(n1+n2)-n_p:(n1+n2)-n_p+n_a,(n1+n2)+n_p-n_a:(n1+n2)+n_p]),l_s2);
t_s2 = spline([l((n1+n2)-n_p:(n1+n2)-n_p+n_a) l((n1+n2)+n_p-n_a:(n1+n2)+n_p)],...
    angle(:,[(n1+n2)-n_p:(n1+n2)-n_p+n_a,(n1+n2)+n_p-n_a:(n1+n2)+n_p]),l_s2);
%segment 3
l_s3 = [l((n1+n2+n3)-n_p):(l((n1+n2+n3)+n_p)-l((n1+n2+n3)-n_p))/n_s:l((n1+n2+n3)+n_p)];
p_s3 = spline([l((n1+n2+n3)-n_p:(n1+n2+n3)-n_p+n_a) l((n1+n2+n3)+n_p-n_a:(n1+n2+n3)+n_p)],...
    path(:,[(n1+n2+n3)-n_p:(n1+n2+n3)-n_p+n_a,(n1+n2+n3)+n_p-n_a:(n1+n2+n3)+n_p]),l_s3);
t_s3 = spline([l((n1+n2+n3)-n_p:(n1+n2+n3)-n_p+n_a) l((n1+n2+n3)+n_p-n_a:(n1+n2+n3)+n_p)],...
    angle(:,[(n1+n2+n3)-n_p:(n1+n2+n3)-n_p+n_a,(n1+n2+n3)+n_p-n_a:(n1+n2+n3)+n_p]),l_s3);
%gathers all of the segments together for plotting
l_s=[l_s1 l_s2 l_s3];
p_s=[p_s1 p_s2 p_s3];
a_s=[t_s1 t_s2 t_s3];
%plot spline vs no-spline robot positions
figure()
plot(l,path,l_s,p_s,'linewidth',2);
legend('x','y','x with spline','y with spline');
title('Robot Position vs. Path Length');
xlabel('lambda (m)');ylabel('x-y position of robot (m)')
%plot spline vs no-spline robot orientation
figure()
plot(l,angle,l_s,a_s,'linewidth',2);
legend('angle','angle with spline');
title('Robot Orientation vs. Path Length');
xlabel('lambda (m)');
ylabel('angle of robot (deg)')
%adds beginning and end of paths to splins segments in order to generate
%room with robot movement with splines
ls = [l(1:n1-n_p-1) l_s l(n1+n2+n3+n_p+1:nl)];
paths = [path(:,1:n1-n_p-1) p_s path(:,n1+n2+n3+n_p+1:nl)];
angles=[angle(:,1:n1-n_p-1) a_s angle(:,n1+n2+n3+n_p+1:nl)];
roomgen(paths,angles,length(paths));
%%
%Spline kinematics model
% first compute p' in the spline region
ps_prime=diff(p_s')'./diff(l_s);
as_prime=diff(a_s')'./diff(l_s);
% find the maximum path velocity that won't violate the velocity constraint
lsdotmax=min(min(min(abs(umax./ps_prime)')',min(abs((wmax*180/pi)./as_prime)')'));
% use a single constant velocity for the entire path 
ldotmax=min([lamdotmax1 lamdotmax2 lamdotmax3 lamdotmax4 lsdotmax]);
% calculate the time needed to travel the complete path
Ts=ls(end)/ldotmax; % faster because the path length is shorter
% calculate p' for the entire path
paths_prime=diff(paths')'./diff(ls);
angles_prime=diff(angles')'./diff(ls);
% add one more at the end because taking the difference loses one element
paths_prime=[paths_prime paths_prime(:,end)];
angles_prime=[angles_prime angles_prime(end)];
% set up time vector at regular sampling period
times=(0:ts:Ts);
NTS=length(times);
% set up storage space
pts=zeros(2,NTS);
pts(:,1)=P0(1:2);
ats=zeros(1,NTS);
ats(:,1)=90;
lts=zeros(size(times));
ptsprime=zeros(2,NTS);
atsprime=zeros(1,NTS);
% the initial path slope is just from the path itself
ptsprime(:,1)=(paths(:,2)-paths(:,1))/norm(paths(:,2)-paths(:,1));
atsprime(:,1)=angles(2)-angles(1);
% use robot kinematics to propagate, and the approximate p' and the maximum
% path velocity to set the robot velocity
us=zeros(2,NTS-1);
ws=zeros(1,NTS-1);
for i=1:NTS-1
    us(:,i)=ptsprime(:,i)*ldotmax;
    ws(:,i)=atsprime(:,i)*ldotmax;
    pts(:,i+1)=pts(:,i)+ts*us(:,i);
    ats(:,i+1)=ats(:,i)+ts*ws(:,i);
    lts(i+1)=lts(i)+ts*ldotmax;
    xprime=interp1(ls(1:end),paths_prime(1,:),lts(i+1));
    yprime=interp1(ls(1:end),paths_prime(2,:),lts(i+1));
    ptsprime(:,i+1)=[xprime;yprime];
    atsprime(:,i+1)=interp1(ls(1:end),angles_prime,lts(i+1));
end
%plotting position based on spline path vs spline kinematics
figure();
plot(lts,pts,ls,paths,'x','linewidth',2);
legend('x kinematics','y kinematics','x path','y path');
xlabel('lambda (m)');
ylabel('robot position (m)');
title('Comparison Between Robot Motion Based on Kinematics Model and Geometric Path');
%plotting angle based on spline path vs spline kinematics
figure();
plot(lts,ats,ls,angles,'x','linewidth',2);
legend('angle kinematics','angle path');
xlabel('lambda (m)');
ylabel('angles (deg)');
title('Comparison Between Robot Orientation Based on Kinematics Model and Geometric Path');
%plotting robot position from kinematics model against time
figure;
plot(times,pts,'linewidth',2);
legend('x','y');
xlabel('time (sec)');
ylabel('robot position (m)');
title('Robot Motion Based on Kinematics Model');
%plotting robot orientation from kinematics model against time
figure;
plot(times,ats,'linewidth',2);
xlabel('time (sec)');
ylabel('robot angle (deg)');
title('Robot Orientation Based on Kinematics Model');
%plotting robot linear velocities in both axes against time
figure();
plot(times(1:end-1),us,'linewidth',2);
legend('u_x','u_y');
xlabel('time (sec)');
ylabel('robot velocity (m/s)');
title('Robot Velocity')
%plotting robot angular velocities in both axes against time
figure();
plot(times(1:end-1),ws*pi/180,'linewidth',2);
xlabel('time (sec)');
ylabel('robot rotation velocity (rad/s)');
title('Robot Angular Velocity')
%%
%Trapzeoidal model
%define maximum linear and angular accelerations
amax=[.2;.2];
awmax=.1;
%calculate maximum acceleration of lambda
lddotmax=min(min(min(abs(amax./ps_prime)')',min(abs((awmax*180/pi)./as_prime)')'));
%calculate total time for trapezoidal based acceleration model
[x,v,a,ta,tb,tf]=trapgen(0,ls(end),0,0,ldotmax,lddotmax,lddotmax,0);
disp(sprintf('tf for trapezoidal profile = %g',tf));
N=100;
t_trap=(0:tf/N:tf);
for i=1:length(t_trap)
  [x(i),v(i),a(i),ta,tb,tf]=trapgen(0,ls(end),0,0,ldotmax,lddotmax,lddotmax,t_trap(i));  
end
%use interpolation to find the time index for each path point on l3
tsb=interp1(x,t_trap,ls);
%calculate linear and angular speed for each segment
usb=diff(paths')'./diff(tsb);
usb=[zeros(2,1) usb(:,1:end-1) zeros(2,1)];
wsb=diff(angles')'./diff(tsb);
wsb=[0 wsb(:,1:end-1) 0];
%plotting robot position based on trapezoidal model
figure();
plot(tsb,paths,'linewidth',2);
legend('x','y');
xlabel('time (sec)');
ylabel('robot position (m)');
title('Robot Motion Based on Trapezoidal Model');
%plotting robot orietnation based on trapezoidal model
figure();
plot(tsb,angles,'linewidth',2);
xlabel('time (sec)');
ylabel('robot angle (deg)');
title('Robot Orientation Based on Trapezoidal Model');
%plotting robot linear velocty based on trapezoidal model
figure();
plot(tsb,usb,'linewidth',2);
legend('u_x','u_y');
xlabel('time (sec)');
ylabel('robot velocity (m/s)');
title('Robot Velocity Based on Trapezoidal Model');
%plotting robot angular velocty based on trapezoidal model
figure();
plot(tsb,wsb*pi/180,'linewidth',2);
xlabel('time (sec)');
ylabel('robot angular velocity (rad/s)');
title('Robot Angular Velocity Based on Trapezoidal Model');
%%
%input:
%pos = position of robot
%theta = orientation of robot
%n = number of positions/orientations to generate
%output
%generated room/movie of robot navigating path
%Room generation function
function roomgen(pos,theta,n)
    %generates structure for movie
    M(n)=struct('cdata',[],'colormap',[]);
    figure()
    %for loop goes through each provided position/orientation of the robot
    for i=1:n
        %defines robot size
        Robot=collisionCylinder(0.3,.3);
        %inputs provided position
        Robot.Pose(1:2,4)=pos(1:2,i);
        %inputs provided orientation
        q=theta(i)*pi/180;
        Robot.Pose(1:2,1:2)=[cos(q) -sin(q); sin(q) cos(q)];
        show(Robot);
        hold on
        %generates ei and ej vectors for the robot
        quiver3(pos(1,i),pos(2,i),.3,.3*cos(q),.3*sin(q),0,'r');
        quiver3(pos(1,i),pos(2,i),.3,.3*cos(q+(90*pi/180)),.3*sin(q+(90*pi/180)),0,'g');
        if i==1
            %Define room
            westwall=collisionBox(.1,5,1);
            westwall.Pose(1:3,4)=[-.05,2.5,.5];
            show(westwall);
            eastwall=westwall;
            eastwall.Pose(1:3,4)=[5.05,2.5,.5];
            show(eastwall);
            northwall=collisionBox(5,.1,1);
            northwall.Pose(1:3,4)=[2.5,5.05,.5];
            show(northwall)
            southwall=northwall;
            southwall.Pose(1:3,4)=[2.5,-.05,.5];
            show(southwall);
            xlim([-.1,5.1]);
            ylim([-.1,5.1]);
            zlim([0,3]);
            xlabel('Room e1')
            ylabel('Room e2')
            
            %Define person
            Fred=collisionCylinder(0.2,2);
            R20=[1,0,0;0,1,0;0,0,1];
            p20=[1;.5;1];
            Fred.Pose=[R20,p20;0,0,0,1];
            show(Fred);
            quiver3(1,.5,2,.2,0,0,'r');
            quiver3(1,.5,2,0,.2,0,'g');
            
            %Define table
            Table=collisionBox(.5,.5,1);
            R30=[cosd(45),sind(45),0;-sind(45),cosd(45),0;0,0,1];
            p30=[2.5;2.5;.5];
            Table.Pose=[R30,p30;0,0,0,1];
            show(Table);
            quiver3(2.5,2.5,1,.25,.25,0,'r');
            quiver3(2.5,2.5,1,-.25,.25,0,'g');
            
            %Define shelf
            Shelf=collisionBox(.8,.3,2.5);
            R40=[0,1,0;-1,0,0;0,0,1];
            p40=[1;4;1.25];
            Shelf.Pose=[R40,p40;0,0,0,1];
            show(Shelf);
            quiver3(1,4,2.5,0,-.4,0,'r');
            quiver3(1,4,2.5,.15,0,0,'g');
            
            view(2)
        end
        %records movie frame
        M(i)= getframe;
    end
    legend('Object','e1','e2');
    %if statement to prompt to save the movie option if there is more than one frame of
    %the robot
    if n>1
        prompt='Would you like to save the movie? Y/N';
        w=input(prompt,'s');
        if w == 'Y'
            v=VideoWriter('movie.avi','Uncompressed AVI');
            v.FrameRate = 5;
            open(v);
            writeVideo(v,M);
            close(v);
            legend('Object','e1','e2');
        end
    end
end
%%
% trapezoidal velocity profile generator - Professor Wen
% input:
%       xo = initial position
%       xf = final position
%       vo = initial velocity
%       vf = final velocity
%       vmax = max velocity
%       amax = max acceleration
%       dmax = max deceleration (positive number)
%       t = current time
% output:
%       x = position at time t
%       v = velocity at time t
%       a = acceleration at time t
%       ta = first switching time (from max accel to zero accel)
%       tb = second switching time (from zero accel to max decel)
%       tf = final arrival time
function [x,v,a,ta,tb,tf]=trapgen(xo,xf,vo,vf,vmax,amax,dmax,t)
  
  % vo and vf must be less than vmax
  if (abs(vo)>=abs(vmax))|(abs(vf)>=abs(vmax))
    error('vo or vf too large');
  end  

  vmax=sign(xf-xo)*vmax;

  if xf>xo
    am1=abs(amax);
    am2=-abs(dmax);
  else
    am1=-abs(dmax);
    am2=abs(amax);
  end

  ta=abs((vmax-vo)/am1);
  tf_tb=(vf-vo-am1*ta)/am2;
  tf=(xf-xo+.5*am1*ta^2-.5*am2*tf_tb^2)/(am1*ta+vo);
  tb=tf-tf_tb;

  if ((tf<2*ta)|(tb<ta))
    tapoly=[1 2*vo/am1 ((vf^2-vo^2)/2/am2+xo-xf)*2/(1-am1/am2)/am1];
    ta = -vo/am1 + sqrt((vo/am1)^2- ... 
	(2/(1-am1/am2)/am1)*((vf-vo)^2/2/am2+xo-xf));
    tf=(vf-vo-am1*ta)/am2+ta;
    tb=ta;
  end

    if t<ta
      a=am1;
      v=am1*t+vo;
      x=.5*am1*t^2+vo*t+xo;
    elseif t<tb
      a=0;
      v=am1*ta+vo;
      x=am1*(t*ta-.5*ta^2)+vo*t+xo;
    elseif t<=tf
      a=am2;
      v=am2*(t-tb)+am1*ta+vo;
      x=am1*(-.5*ta^2+t*ta)+am2*(.5*t^2+.5*tb^2-tb*t)+vo*t+xo;
    else
      a=0;
      v=0;
      x=xf;
    end
end