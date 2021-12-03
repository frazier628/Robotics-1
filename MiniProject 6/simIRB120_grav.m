
abb=loadrobot('abbIrb120','DataFormat','column');
abb.Gravity=9.81*[0;0;-1];

qinit=zeros(6,1);
qdotinit=zeros(6,1);
fext=zeros(6,8);
tau_zeros=zeros(6,1);

qdes=[1;0;0;0;0;0];

qdes=(rand(6,1)-.5)*2*pi;

kp=[1.5;1.5;1.5;1;1;1];kd=[2.5;4.5;2;.1;.1;.1];ki=[.1;.1;.1;0;0;0];
%kp=[5;5;5;5;5;5];kd=[3;3;3;5;5;5];ki=[.1;.5;.2;.1;.1;.1];
simout=sim('robotsimulation_grav','ReturnWorkspaceOutputs', 'on');

n=length(simout.tout);
figure(1);for i=1:n;show(abb,simout.q(i,:)');pause(1);M_Image(i)=getframe;end

figure(10);plot(simout.tout,simout.q,'LineWidth',2);
legend('q_1','q_2','q_3','q_4','q_5','q_6');