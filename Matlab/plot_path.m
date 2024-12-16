function plot_path(path, title_str)
    hold on;
    for i = 1:size(path, 1)
        plot(path(i, 1), path(i, 2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    end
    title(title_str);
end