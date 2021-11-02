%Mini project 4 singularity code for Part 2

%Bounded Singularity
%Singular when p23 is collinear with p34
%c3L4 + s3L3 = 0
BS=subs(J4_3,[L3 + L2*cos(q3),L2*sin(q3) - L4],[0,0]) 
q=[zeros(2,1);-1.477937762345568;zeros(3,1)];
figure()
title('Bounded Singularity Position')
show(irb1200_rbt,q,'collision','on')
zlim([0 1.5])
%%
%Interior Singularity
%Singular if L4*cos(q2 + q3) + L3*sin(q2 + q3) + L2*sin(q2) = 0 as this is
%when h1 is collinear with p14
%L4*cos(q2 + q3) + L3*sin(q2 + q3) + L2*sin(q2) = 0
IS=subs(J4_3,L4*cos(q2 + q3) + L3*sin(q2 + q3) + L2*sin(q2),0) 
%q=[zeros(2,1);-1.477937762345568;zeros(3,1)];
q=[0;-45*pi/180;0.0820;zeros(3,1)];
figure()
title('Interior Singularity Position')
show(irb1200_rbt,q,'collision','on')
zlim([0 1.5])
%%
%Wrist Singularity
%Singular if q5 = 0 as this is when h4 is collinear with h6
WS=subs(J4_3,sin(q5),0)     
q=zeros(6,1);
figure()
title('Wrist Singularity Position')
show(irb1200_rbt,q,'collision','on')
zlim([0 1.5])