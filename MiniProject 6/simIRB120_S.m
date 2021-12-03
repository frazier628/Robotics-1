
abb=loadrobot('abbIrb120','DataFormat','column');
abb.Gravity=9.81*[0;0;-1];

qinit=zeros(6,1);
qdotinit=zeros(6,1);
fext=zeros(6,8);
tau_zeros=zeros(6,1);

load('q_S_sol.mat');
kp=[1.5;1.5;1.5;1;1;1];kd=[2.5;4.5;2;.1;.1;.1];ki=[.1;.1;.1;0;0;0];
%kp=[5;5;5;5;5;5];kd=[3;3;3;5;5;5];ki=[.1;.5;.2;.1;.1;.1];

for i=1:length(q7)
    qdes=q7(:,i);
    simout=sim('robotsimulation_grav','ReturnWorkspaceOutputs', 'on');
    qinit=simout.q(end,:);
    qdotinit=simout.qdot(end,:);
    q_total{i}=simout.q;
    t_total{i}=simout.tout;
end
%%
t_whole=t_total{1};
q_whole=q_total{1};
for i=2:length(q7)
    t_whole=vertcat(t_whole,t_whole(end)+t_total{i});
    q_whole=vertcat(q_whole,q_total{i});
end

n=length(t_whole);
load S_sphere_path
figure(1);
k=1;
for i=1:n
    if mod(i,10)==0
%         hold off
%         plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
%         hold on
%         xlabel('x');ylabel('y');zlabel('z');title('End Effector Poses')
%         surf(X,Y,Z)
%         alpha 0.4
%         axis(r*[-1 1 -1 1 0 2]);axis('square');
          view(120,10);
        show(abb,q_whole(i,:)');
        M_Image(k)=getframe;
        k=k+1;
    end
end

figure(10);plot(t_whole,q_whole,'LineWidth',2);xlim([0 t_whole(end)])
legend('q_1','q_2','q_3','q_4','q_5','q_6');