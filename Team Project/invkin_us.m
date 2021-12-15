function robot = invkin(robot)
    R0T = robot.T(1:3,1:3);
    P0T=robot.T(1:3,end);
    p=P0T-robot.P(:,1)-R0T*robot.P(:,end);
    
    %solution of q3
    k=robot.H(:,3);
    d=norm(p);
    p1=-1*robot.P(:,4);
    p2=robot.P(:,3);
    q3=subprob3(k,p1,p2,d);
    
    %solution of q1 and q2 for each q3
    for i=1:length(q3)
        p1=p;
        p2=robot.P(:,3)+(rot(robot.H(:,2),(q3(i)))*robot.P(:,4));
        k1=-1*robot.H(:,1);
        k2=robot.H(:,2);
        [q1(:,i),q2(:,i)]=subprob2(k1,k2,p1,p2);
    end
    
    %list of all 4 solutions of q1, q2 and q3
    q=[q1(:,1)',q1(:,2)';q2(:,1)',q2(:,2)';q3(1),q3(1),q3(2),q3(2)];
    
    q4=zeros(1,length(q));
    %find q4 for each solution
    for i=1:length(q)
        q4(i)=-(q(2,i)+q(3,i));
    end
    
    %assign q array for output
    q(4,:)=q4;
    robot.q=q;
    
end