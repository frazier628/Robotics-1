function wall = scan(dir,rc,d,maze)
% Inputs
    % dir = direction to scan in
    % rc = current position in (row; column) form
    % d = distance to scan
% Outputs
    % wall = boolean (true if wall is detected within d steps)
    i = 0;
    wall = false;
    while i<d && ~wall
        rc = move(dir,rc,1);
        if maze(rc(1),rc(2))
            wall = true;
        end
        i=i+1;
    end
end