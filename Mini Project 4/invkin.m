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
    q123=[q1(:,1)',q1(:,2)';q2(:,1)',q2(:,2)';q3(1),q3(1),q3(2),q3(2)];
    
    %solution for q4 and q5 for each q123
    for i=1:length(q123)
        R03=rot(robot.H(:,1),q123(1,i))*rot(robot.H(:,2),q123(2,i))*rot(robot.H(:,3),q123(3,i));
        p1=R03'*R0T*robot.H(:,6);
        p2=robot.H(:,6);
        k1=-1*robot.H(:,4);
        k2=robot.H(:,5);
        [q4(:,i),q5(:,i)]=subprob2(k1,k2,p1,p2);
    end
    
    %list of all 8 solutions of q1 through q5
    q=[q123,q123;q4(1,1),q4(1,2),q4(1,3),q4(1,4),...
        q4(2,1),q4(2,2),q4(2,3),q4(2,4);...
        q5(1,1),q5(1,2),q5(1,3),q5(1,4),q5(2,1),q5(2,2),q5(2,3),q5(2,4)];
    
    q6=zeros(1,length(q));
    %find q6 for each solution
    for i=1:length(q)
        R05=rot(robot.H(:,1),q(1,i))*rot(robot.H(:,2),q(2,i))*rot(robot.H(:,3),q(3,i))*rot(robot.H(:,4),q(4,i))*rot(robot.H(:,5),q(5,i));
        k=robot.H(:,6);
        p1=robot.H(:,5);
        p2=R05'*R0T*robot.H(:,5);
        q6(i)=subprob1(k,p1,p2);
    end
    
    %assign q array for output
    q(6,:)=q6;
    robot.q=q;
    
end