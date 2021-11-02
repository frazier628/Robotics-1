function robot = invkin_iterJ(robot)
    %input parameters
    N=robot.MaxIter;
    alpha=robot.StepSize;
    w=robot.Weights;
    
    %Final Target R and p
    R0Td=robot.T(1:3,1:3);
    p0Td=robot.T(1:3,4);
    
    %set up variables for use in iteration
    q0=robot.q;
    n=length(q0);
    q=zeros(n,N+1);
    q(:,1)=q0;
    p0T=zeros(3,N+1);
    
    %iterative implementation
    for i=1:N
        robot.q=q(:,i);
        robot=fwdkiniter(robot);
        p0T(:,i)=robot.T(1:3,4);
        s=R2qv(robot.T(1:3,1:3)*R0Td');
        %s
        quat=R2q(robot.T(1:3,1:3)*R0Td');
        %quat(1)*quat(2:4)
        [k,theta]=R2kth(robot.T(1:3,1:3)*R0Td');
        %k*theta
        dX=[quat(1)*quat(2:4);p0T(:,i)-p0Td];
        % Jacobian update
        qq = q(:,i) - alpha*robot.J'*inv(robot.J*robot.J' + .01*diag(1./w))*dX;
        q(:,i+1) = (qq>pi).*(-2*pi+qq) + (qq<-pi).*(2*pi+qq) + (qq<pi).*(qq>-pi).*qq;
    end
    
    %fwd kinematics with final q
    robot.q=q(:,N+1);
    robot=fwdkiniter(robot);
end 
