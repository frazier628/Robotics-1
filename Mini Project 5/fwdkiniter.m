%
% fwdkiniter.m
% 
% forward kinematics using POE approach
%
% input:    robot = robot object with
%                   .q = joint displacement vector
%                   .H = [h1, ..., hn] (3xn)
%                   .P = [p01, ..., p_{n-1,n},p_{n,T}] (3x(n+1))
%                   .joint_type = 1xn (0 for revolute, 1 for prismatic)
%
% output:   robot = robot object with 
%                   .T = homogeneous transformation T_{0T}
%                   .J = Jacobian J_T in the 0 frame
%
% usage:
%       robot = fwddiffkiniter(robot);   
%
% 

function robot=fwdkiniter(robot)

q=robot.q;
n=length(robot.q);
H=robot.H;
T=eye(4,4);
J=zeros(6,n);
joint_type=robot.joint_type;
P=robot.P;

for i=1:n
    h=H(1:3,i);
    if joint_type(i)==0
        R=expm(hat(h)*q(i));p=P(1:3,i);
    else
        R=eye(3,3);p=P(1:3,i)+q(i)*h;
    end
    J=phi(eye(3,3),T(1:3,1:3)*p)*J;
    J(:,i)=[R*h;cross(h,p)];
    if joint_type(i) == 0
        J(:,i)=[T(1:3,1:3)*h;zeros(3,1)];
    else
        J(:,i)=[zeros(3,1);T(1:3,1:3)*h];
    end
    T=T*[R p;zeros(1,3) 1];
end    
    
robot.T=T*[eye(3,3) P(:,n+1);zeros(1,3) 1];
robot.J=phi(eye(3,3),T(1:3,1:3)*P(:,n+1))*J;

end

