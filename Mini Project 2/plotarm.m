
%
% plotarm.m
%
% plot a n-link planar arm using sticks
%
% input: robot
%           .P = link length (2x(n+1))
%           .q = joint angles
%
function plotarm(robot)

n=length(robot.q);
P=zeros(2,n+1);
P(:,1)=robot.P(1:2,1);
for i=1:n
    P(:,i+1)=rot2(sum(robot.q(1:i)))*robot.P(1:2,i+1);
end

P1=cumsum(P')';

clr='brgc';
for i=1:n
    plot(P1(1,i:i+1),P1(2,i:i+1),clr(mod(i,length(clr))+1),'linewidth',2)
    plot(P1(1,i:i+1),P1(2,i:i+1),'ko','linewidth',3)
end

end

%
% function rot2
%

function R=rot2(q)
    c=cos(q);s=sin(q);
    R=[c -s;s c];
end