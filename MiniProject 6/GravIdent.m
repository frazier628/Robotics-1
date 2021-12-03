abb=loadrobot('abbIrb120','DataFormat','column','Gravity',[0;0;-9.91]);
abb.Gravity=9.81*[0;0;-1];
M=50;
nx=12;
k=1;
tau_g=zeros(6*M,1);
A=zeros(6*M,12);
for i=1:M
    q=(rand(6,1)-.5)*2*pi;
    tau_g(k:k+5)=inverseDynamics(abb,q);
    A(2+k-1,1:5)=[sin(q(2)),sin(q(2)+q(3)),cos(q(2)+q(3)),sin(q(2)+q(3))*cos(q(4))*sin(q(5)),cos(q(2)+q(3))*cos(q(5))];
    A(3+k-1,6:9)=[sin(q(2)+q(3)),cos(q(2)+q(3)),sin(q(2)+q(3))*cos(q(4))*sin(q(5)),cos(q(2)+q(3))*cos(q(5))];
    A(4+k-1,10)=cos(q(2)+q(3))*sin(q(4))*sin(q(5));
    A(5+k-1,11:12)=[sin(q(2)+q(3))*sin(q(5)),cos(q(2)+q(3))*cos(q(4))*cos(q(5))];
    k=k+6;
end
x=pinv(A)*tau_g;
disp(x);
save('x_sol.mat','x');

% overall error
disp('overall fit error: ||Ax-tau||');
norm(A*x-tau_g)

% comparing gravity on each joint 
tau=reshape(tau_g,6,M);
tauhat=reshape(A*x,6,M);

disp('fit error for each axis: ||Ax-tau||');
for i=1:6
    %figure(i+100);
    %plot((1:M),tau(i,:),'o',(1:M),tauhat(i,:),'x','linewidth',2);
    disp(sprintf('i = %d, G_i error: %g',i,...
        norm(tau(i,:)-tauhat(i,:))));
end

% for testing
% randomly generate joint angles
n=6;
q=(rand(n,M)-.5)*2*pi;
tau_check=zeros(6,M);
tau_est=zeros(6,M);
for i=1:M
    tau_check(1:6,i)=inverseDynamics(abb,q(:,i));
    A2=zeros(6,12);
    A2(2,1:5)=[sin(q(2)),sin(q(2)+q(3)),cos(q(2)+q(3)),sin(q(2)+q(3))*cos(q(4))*sin(q(5)),cos(q(2)+q(3))*cos(q(5))];
    A2(3,6:9)=[sin(q(2)+q(3)),cos(q(2)+q(3)),sin(q(2)+q(3))*cos(q(4))*sin(q(5)),cos(q(2)+q(3))*cos(q(5))];
    A2(4,10)=cos(q(2)+q(3))*sin(q(4))*sin(q(5));
    A2(5,11:12)=[sin(q(2)+q(3))*sin(q(5)),cos(q(2)+q(3))*cos(q(4))*cos(q(5))];
    tau_est(1:6,i)=A2*x;
end

disp('testing error based on ramdom samples for each axis: ||Ax-tau||');
for i=1:6
    %figure(i+200);
    %plot((1:M),tau_check(i,:),'o',(1:M),tau_est(i,:),'x','linewidth',2);
    disp(sprintf('i = %d, G_i error: %g',i,...
        norm(tau_check(i,:)-tau_est(i,:))));
end