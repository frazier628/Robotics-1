function robot=fwddiffkiniter(robot)

    n = length(robot.joint_type);

    joint_type = robot.joint_type;
    q = robot.q;
    H = robot.H;
    P = robot.P;
    T = eye(4,4);
    J = zeros(6,n);

    for i=1:n
        h = H(1:3,i);
        if joint_type(i)==0
            R = expm(hat(h)*q(i)); 
            p = P(1:3,i);
        else
            R = eye(3,3);
            p = P(1:3,i)+q(i)*h;
        end
        J = phi(eye(3,3), T(1:3,1:3)*p)*J;
        if joint_type(i)==0
            J(:,i) = [T(1:3,1:3)*h; zeros(3,1)];
        else
            J(:,i) = [zeros(3,1);T(1:3,1:3)*h];
        end
        T = T*[R p; zeros(1,3) 1];
    end
    
    robot.J = phi(eye(3,3),T(1:3,1:3)*P(:,n+1))*J;
    robot.T = T*[eye(3,3) P(:,n+1); 0 0 0 1];

end