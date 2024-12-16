function path = astar(grid_size, start_pos, goal_pos, obstacles)
    % Initialize open and closed lists
    open_list = [start_pos];
    closed_list = [];
    
    % Cost tables
    g_cost = inf(grid_size);
    f_cost = inf(grid_size);
    
    % Set start position costs
    g_cost(start_pos(1), start_pos(2)) = 0;
    f_cost(start_pos(1), start_pos(2)) = heuristic(start_pos, goal_pos);
    
    % Track path
    came_from = zeros(grid_size);
    
    while ~isempty(open_list)
        % Find node with the lowest f_cost
        [~, idx] = min(f_cost(sub2ind(size(f_cost), open_list(:,1), open_list(:,2))));
        current_node = open_list(idx, :);
        
        % Check if we reached the goal
        if isequal(current_node, goal_pos)
            path = reconstruct_path(came_from, current_node);
            return;
        end
        
        % Remove from open_list, add to closed_list
        open_list(idx, :) = [];
        closed_list = [closed_list; current_node];
        
        % Explore neighbors
        neighbors = get_neighbors(current_node, grid_size, obstacles);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            
            % Skip if already in closed list
            if ismember(neighbor, closed_list, 'rows')
                continue;
            end
            
            % Tentative g_cost (consider diagonal movement cost)
            if any(neighbor - current_node == [1, 1]) || any(neighbor - current_node == [-1, -1]) || ...
               any(neighbor - current_node == [1, -1]) || any(neighbor - current_node == [-1, 1])
                tentative_g = g_cost(current_node(1), current_node(2)) + sqrt(2); % Diagonal cost
            else
                tentative_g = g_cost(current_node(1), current_node(2)) + 1; % Straight cost
            end
            
            if tentative_g < g_cost(neighbor(1), neighbor(2))
                % Update costs
                came_from(neighbor(1), neighbor(2)) = sub2ind([grid_size, grid_size], current_node(1), current_node(2));
                g_cost(neighbor(1), neighbor(2)) = tentative_g;
                f_cost(neighbor(1), neighbor(2)) = tentative_g + heuristic(neighbor, goal_pos);
                
                % Add to open_list if not already present
                if ~ismember(neighbor, open_list, 'rows')
                    open_list = [open_list; neighbor];
                end
            end
        end
    end
    
    error('No path found');
end

% Heuristic function (Euclidean Distance)
function h = heuristic(node, goal)
    h = sqrt((node(1) - goal(1))^2 + (node(2) - goal(2))^2);
end

% Get valid neighbors
function neighbors = get_neighbors(node, grid_size, obstacles)
    % Include diagonal directions
    directions = [-1, 0; 1, 0; 0, -1; 0, 1; -1, -1; -1, 1; 1, -1; 1, 1]; % Up, Down, Left, Right, Diagonals
    neighbors = [];
    for i = 1:size(directions, 1)
        neighbor = node + directions(i, :);
        if all(neighbor > 0 & neighbor <= grid_size) && ~ismember(neighbor, obstacles, 'rows')
            neighbors = [neighbors; neighbor];
        end
    end
end

% Reconstruct path
function path = reconstruct_path(came_from, current_node)
    % Initialize the path with the current node
    path = current_node;
    
    % Backtrack from the goal to the start using the came_from matrix
    while came_from(current_node(1), current_node(2)) ~= 0
        % Get the linear index from came_from and convert it back to [x, y]
        [x, y] = ind2sub(size(came_from), came_from(current_node(1), current_node(2)));
        
        % Append the new [x, y] position to the path
        current_node = [x, y];
        path = [current_node; path]; % Add current node to the beginning of the path
    end
end