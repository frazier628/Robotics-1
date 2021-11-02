N=100;
% Final transformation
R6T=[-ez ey ex];
load S_sphere_path

% plot out sphere with equal path length
figure();plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');title('End Effector Poses')
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);



for i=1:N+1
    % specify end effector SE(3) frame
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)]*R6T' pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    %
    irb1200=invkin_iterJ(irb1200); 
    %
    q(:,i)=irb1200.q;
    % check forward kinematics to make sure the IK solution is correct
    irb1200.q=q(:,i);
    irb1200=fwdkiniter(irb1200);
    irb1200.q=q(:,i);
    irb1200=fwdkiniter(irb1200);
    Ts{i}=irb1200.T;
    Js{i}=irb1200.J;
end

for i=1:N+1
    % show robot pose (ever 5 frames)
    if mod(i,5)==0
        disp(norm(Ts{i}-Td{i}));
        show(irb1200_rbt,q(:,i),'collision','on');pause(1);
        view(150,10);
    end
end

Jsingular=zeros(1,101);
for i=1:N+1
    [~,S,~]=svd(Js{i});
    Jsingular(i)=min(nonzeros(S));
end
figure();hold on;title('Minimum Jacobian Singular Values vs Path Length');
xlabel('Path Length (Lambda (m))');ylabel('Minimum Jacobian Singular Values');
plot(l,Jsingular);