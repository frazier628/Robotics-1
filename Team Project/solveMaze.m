function path = solveMaze(start,walls)
% Solves a maze from image pixel coordinates
    % Inputs:
        % start: starting point of the maze
        % walls: x,y coordinates of maze walls
    % Outputs:
        % path: x,y coordinates of maze solution path corners

    xy0 = round(start);
    pos = [xy0(2);xy0(1)]; % maze starting position

    % define grid of boolean (wall/no wall) from x,y coordinates of walls
    mazegrid = zeros(max(walls(2,:)),max(walls(1,:)));
    for i = 1:length(walls)
        mazegrid(walls(2,i),walls(1,i)) = 1;
    end

    in_maze = true;

    rd = 25; % constant right wall distance
    [wall_dist,walldir] = findWall(mazegrid,pos); % find wall on right when starting the maze
    
    % determine which direction to move based on closest initial wall
    if walldir == "right"
        movedir = "up";
    elseif walldir == "up"
        movedir = "left";
    elseif walldir == "left"
        movedir = "down";
    elseif walldir == "down"
        movedir = "right"
    end
    
    % change position to be 100 from closest wall
    pathpts(:,1) = pos; % store path points as points where path changes direction, will later connect these dots and enforce lambdadot traversal speed
    pos = move(walldir, pos, (wall_dist-rd));
    pathpts(:,2) = pos;

    % go flag ensures it won't move to consecutive while loops after turning
    % before checking if it's left the maze
    n=3;
    m = 0;
    while in_maze
        go = true;
        while movedir == "right" && go
            pos = move(movedir,pos,1);
            if ~scan("down", pos, 2*rd, mazegrid) % no more wall to right of agent
                pos = move(movedir,pos,rd); % move beyond clearing
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "down"; % new direction
                pos = move(movedir, pos, rd); % move into new channel
                go = false;
            elseif scan(movedir, pos, rd, mazegrid) % wall in movement direction
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "up"; % new direction
                go = false;
            end
        end
    
        while movedir == "up" && go
            pos = move(movedir,pos,1);
            if ~scan("right", pos, 2*rd, mazegrid) % no more wall to right of agent
                pos = move(movedir,pos,rd); % move beyond clearing
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "right"; % new direction
                pos = move(movedir, pos, rd); % move into new channel
                go = false;
            elseif scan(movedir, pos, rd, mazegrid) % wall in movement direction
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "left"; % new direction
                go = false;
            end
        end
    
        while movedir == "left"
            pos = move(movedir,pos,1);
            if ~scan("up", pos, 2*rd, mazegrid) % no more wall to right of agent
                pos = move(movedir,pos,rd); % move beyond clearing
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "up"; % new direction
                pos = move(movedir, pos, rd); % move into new channel
                go = false;
            elseif scan(movedir, pos, rd, mazegrid) % wall in movement direction
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "down"; % new direction
                go = false;
            end
        end
    
        while movedir == "down"
            pos = move(movedir,pos,1);
            if ~scan("left", pos, 2*rd, mazegrid) % no more wall to right of agent
                pos = move(movedir,pos,rd); % move beyond clearing
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "left"; % new direction
                pos = move(movedir, pos, rd); % move into new channel
                go = false;
            elseif scan(movedir, pos, rd, mazegrid) % wall in movement direction
                pathpts(:,n) = pos; % place a path point
                plot(pos(2),pos(1), "*")
                n=n+1;
                movedir = "right"; % new direction
                go = false;
            end
        end
    
        if pos(1) < min(walls(2,:)) || pos(1) > max(walls(2,:))
            in_maze = false;
        elseif pos(2) < min(walls(1,:)) || pos(2) > max(walls(1,:))
            in_maze = false
        end
            
    end

    path = pathpts;

    
end