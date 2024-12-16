function setup_environment(grid_size, start_pos, goal_pos, obstacles)
    figure;
    hold on;
    axis([1 grid_size 1 grid_size]);
    
    % Plot grid lines
    for i = 1:grid_size
        plot([1 grid_size], [i i], 'k');
        plot([i i], [1 grid_size], 'k');
    end
    
    % Plot obstacles
    for i = 1:size(obstacles, 1)
        plot(obstacles(i,1), obstacles(i,2), 'rs', 'MarkerSize', 20, 'MarkerFaceColor', 'r');
    end
    
    % Plot start and goal positions
    plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 20, 'MarkerFaceColor', 'g');
    plot(goal_pos(1), goal_pos(2), 'bo', 'MarkerSize', 20, 'MarkerFaceColor', 'b');
    
    title('Drone Navigation Environment');
    xlabel('X-axis');
    ylabel('Y-axis');
    grid on;
end