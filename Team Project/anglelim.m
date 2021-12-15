function robot = anglelim(robot)
%135 to -135 q1
%85 to -5 q2
%95 to -10 q3

limits = [-135 135; -5 85; -10 95];
q=rad2deg(robot.q);
q1_check=zeros(1,4);
q2_check=zeros(1,4);
q3_check=zeros(1,4);
for i=1:4
    q1_check(i) = min((q(1,i) > limits(1,1) ) & ( q(1,i) < limits(1,2) ));
    q2_check(i) = min((q(2,i) > limits(2,1) ) & ( q(2,i) < limits(2,2) ));
    q3_check(i) = min((q(3,i) > limits(3,1) ) & ( q(3,i) < limits(3,2) ));
end
qcheck = [q1_check;q2_check;q3_check];
check = sum(qcheck);
pick=find(check==3);
robot.q=q(:,pick);
robot.q=deg2rad(robot.q);
end