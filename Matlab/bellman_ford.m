function distances = bellman_ford(grid_size, start_pos, goal_pos, obstacles)
    % Initialize distances
    distances = inf(grid_size);
    distances(start_pos(1), start_pos(2)) = 0;
    
    edges = generate_edges(grid_size, obstacles);
    V = grid_size * grid_size;
    
    % Relax edges |V|-1 times
    for i = 1:(V - 1)
        for j = 1:size(edges, 1)
            u = edges(j, 1:2);
            v = edges(j, 3:4);
            weight = 1;
            
            if distances(u(1), u(2)) + weight < distances(v(1), v(2))
                distances(v(1), v(2)) = distances(u(1), u(2)) + weight;
            end
        end
    end
    
    % Check for negative cycles
    for j = 1:size(edges, 1)
        u = edges(j, 1:2);
        v = edges(j, 3:4);
        weight = 1;
        
        if distances(u(1), u(2)) + weight < distances(v(1), v(2))
            error('Negative cycle detected');
        end
    end
end

function edges = generate_edges(grid_size, obstacles)
    edges = [];
    for x = 1:grid_size
        for y = 1:grid_size
            if ~ismember([x, y], obstacles, 'rows')
                % Right
                if x < grid_size && ~ismember([x+1, y], obstacles, 'rows')
                    edges = [edges; x, y, x+1, y];
                    edges = [edges; x+1, y, x, y];  % Add reverse edge
                end
                % Down
                if y < grid_size && ~ismember([x, y+1], obstacles, 'rows')
                    edges = [edges; x, y, x, y+1];
                    edges = [edges; x, y+1, x, y];  % Add reverse edge
                end
            end
        end
    end
end
