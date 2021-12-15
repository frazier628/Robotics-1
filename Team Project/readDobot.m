function robot = readDobot(ardObj,robot)
% Read and parse data package from Dobot
    flush(ardObj);
    line_str = readline(ardObj); % MATLAB returns readline as string
    line_str = readline(ardObj); % MATLAB returns readline as string
    line_chr = convertStringsToChars(line_str); % convert the string to char array

    while line_chr(1) ~= char(hex2dec('A5')) || length(line_chr) ~= 41
        disp("Did Not Read Properly, Trying Again")
        flush(ardObj);
        line_str = readline(ardObj); % MATLAB returns readline as string
        line_str = readline(ardObj); % MATLAB returns readline as string
        line_chr = convertStringsToChars(line_str); % convert the string to char array
    end

%     assert(line_chr(1) == char(hex2dec('A5'))); % Assertion for header byte
%     assert(length(line_chr) == 41); % Assertion for data length is correct (ie 42 bytes, 1 is removed with end byte while reading the line)
    
    line_float = typecast(uint8(line_chr(2:end)), 'single');
    
    robot.position.x = line_float(1); % X coordinate
    robot.position.y = line_float(2); % Y coordinate
    robot.position.z = line_float(3); % Z coordinate
    
    robot.angles.rHead = line_float(4); % Rotation value (Relative rotation angle of the end effector to the base)
    robot.angles.q1 = line_float(5); % Base Angle 
    robot.angles.q2 = line_float(6); % Rear Arm Angle 
    robot.angles.q3 = line_float(7); % Fore Arm Angle
    robot.angles.q4 = line_float(8); % Servo Angle (joint 4 angle)

    robot.q = [robot.angles.q1; robot.angles.q2; robot.angles.q3; robot.angles.q4];
end