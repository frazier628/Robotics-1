function [walld,dir] = findWall(maze,rc)
% searches in the direction given for a maze wall and returns a new
% position
    % inputs:
        % maze = grid of booleans (1=wall, 0=no wall)
        % rc = [row; column] (y,x) current position
    % outputs:
        % walld = distance to closest wall
        % dir = direction of closest wall

    % search up (increment row until finding a wall)
    rc_temp = rc;
    while ~maze(rc_temp(1), rc_temp(2))
        rc_temp(1) = rc_temp(1) + 1;
    end
    dists(1) = abs(rc_temp(1) - rc(1));

    % search left (decrement column until finding a wall)
    rc_temp = rc;
    while ~maze(rc_temp(1), rc_temp(2))
        rc_temp(2) = rc_temp(2) - 1;
    end
    dists(2) = abs(rc_temp(2) - rc(2));

    % search down (decrement row until finding a wall)
    rc_temp = rc;
    while ~maze(rc_temp(1), rc_temp(2))
        rc_temp(1) = rc_temp(1) - 1;
    end
    dists(3) = abs(rc_temp(1) - rc(1));

    % search right (increment column until finding a wall)
    rc_temp = rc;
    while ~maze(rc_temp(1), rc_temp(2))
        rc_temp(2) = rc_temp(2) + 1;
    end
    dists(4) = abs(rc_temp(2) - rc(2));

    dists
    [walld,idx] = min(dists);
    dirs = ["up", "left", "down", "right"];
    dir = dirs(idx);
   
end