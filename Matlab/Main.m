% Main.m - Main file to run the drone navigation simulation

% Step 1: Set up grid, obstacles, start and goal positions
grid_size = 10; % 10x10 grid
start_pos = [1, 1]; % Starting point
goal_pos = [10, 10]; % Goal point
obstacles = [3, 5; 5, 7; 6, 2; 7, 4; 4, 4; 6, 5; 5, 6]; % Obstacles in the grid

% Create figure for A* path
figure(1);
% Setup environment for A* plot
setup_environment(grid_size, start_pos, goal_pos, obstacles);
disp('Running A* for real-time navigation...');
path_astar = astar(grid_size, start_pos, goal_pos, obstacles);
plot_path(path_astar, 'A* Path');
title('A* Path Navigation');

% Create separate figure for Bellman-Ford path
figure(2);
% Setup environment for Bellman-Ford plot
setup_environment(grid_size, start_pos, goal_pos, obstacles);
disp('Hazards encountered, switching to Bellman-Ford...');
distances_bellman = bellman_ford(grid_size, start_pos, goal_pos, obstacles);
path_bellman = reconstruct_path(distances_bellman, start_pos, goal_pos);
plot_path(path_bellman, 'Bellman-Ford Path');
title('Bellman-Ford Path Navigation');
disp('Bellman-Ford path plotted.');

% Helper function to reconstruct path from distances
function path = reconstruct_path(distances, start_pos, goal_pos)
    path = goal_pos;
    current = goal_pos;
    
    while ~isequal(current, start_pos)
        [x, y] = find_next_step(distances, current);
        current = [x, y];
        path = [current; path];
    end
end

function [next_x, next_y] = find_next_step(distances, current)
    [x, y] = deal(current(1), current(2));
    [grid_size, ~] = size(distances);
    min_dist = inf;
    next_x = x;
    next_y = y;
    
    % Check all neighboring cells
    for dx = -1:1
        for dy = -1:1
            if dx == 0 && dy == 0
                continue;
            end
            new_x = x + dx;
            new_y = y + dy;
            
            % Check if within grid bounds
            if new_x >= 1 && new_x <= grid_size && ...
               new_y >= 1 && new_y <= grid_size
                if distances(new_x, new_y) < min_dist
                    min_dist = distances(new_x, new_y);
                    next_x = new_x;
                    next_y = new_y;
                end
            end
        end
    end
end
