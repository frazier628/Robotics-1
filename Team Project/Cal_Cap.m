function Cal_Cap(ardObj)

    % paper must be 5.75" from base front edge

%     q = [ [0;0;10;0] [-5;0;10;0] [-10;0;10;0] ...
%           [0;0;0;0] [-5;0;0;0] [-10;0;0;0] ...
%           [0;3;0;0] [-5;3;0;0] [-10;3;0;0] ...
%           [0;3;10;0] [-5;3;10;0] [-10;3;10;0] ];

    q = [-4,-4,-4,-4,-4,-4, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4;...
          0, 0,13,13,25,25, 0, 0,13,13,25,25, 0, 0,13,13,25,25;...
          0, 5, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5, 0, 5;...
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    
    for i = 1:length(q)
        %Move bot to desired angles
        pause(0.25)
        setJointAngles(q(:,i), ardObj)

        %Wait to capture image
        input('Is picture saved?\n','s')
        %Move after image is captured
    end
    %return to zero
    setJointAngles(q(:,11), ardObj)


end