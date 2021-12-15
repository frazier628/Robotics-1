function setJointAngles(qin, ardObj)
% sets Dobot joint angles to qin

    assert(length(qin)==4)

    isGrab = 0; % No suction
    
    MOVE_MODE_JUMP = 0;
    MOVE_MODE_JOINTS = 1;  % joints move independent
    MOVE_MODE_LINEAR = 2;  % linear movement
    
    CARTESIAN = 3;
    JOINT = 6;
    
    % Set the float values in the data package
    line_float_cmd = zeros(1,10);
    line_float_cmd(1) = JOINT;
    line_float_cmd(3) = qin(1);
    line_float_cmd(4) = qin(2);
    line_float_cmd(5) = qin(3);
    line_float_cmd(6) = qin(4);
    line_float_cmd(7) = isGrab;
    line_float_cmd(8) = MOVE_MODE_JOINTS;

    % Combine datapackage
    header_chr = char(hex2dec('A5'));
    line_chr_cmd = char(typecast(single(line_float_cmd), 'uint8'));
    line_chr_cmd = [header_chr,line_chr_cmd]; %,tail_chr]
    line_str_cmd = convertCharsToStrings(line_chr_cmd);

    % Send command to the robot
    pause(.25);
    writeline(ardObj,line_str_cmd)
    

end