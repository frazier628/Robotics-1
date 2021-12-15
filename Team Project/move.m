function rc_out = move(dir,rc,n)
% Inputs:
    % dir = direction to move in
    % rc = current position in (row; column) form
    % maze = array of maze walls
    % n = number of steps to move

% Outputs:
    % rc_out = new position in (row; column) form

    if dir == "up" % increment rows
        rc_out = rc + [n;0];
    elseif dir == "left" % decrement columns
        rc_out = rc + [0;-n];
    elseif dir == "down" % decrement row
        rc_out = rc + [-n;0];
    else % right - increment column
        rc_out = rc + [0;n];
    end
end