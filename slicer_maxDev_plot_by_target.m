function slicer_maxDev_plot_by_target(data)
    % Number of participants
    numParticipants = length(data.data.part);
    % Number of targets
    numTargets = 3;

    % Create a figure for the plot
    fig = figure('Name', 'Slicer Plot for maxDev by Target', 'NumberTitle', 'off');

    % Initial plot with all targets
    selectedTarget = 1:numTargets;
    ax = axes('Parent', fig);
    hold on;

    % Plot initial data
    plot_handles = cell(numTargets, 1);
    colors = ['r', 'g', 'b']; % Colors for different targets
    legends = {};

    for targetPos = 1:numTargets
        for p = 1:numParticipants
            maxDevByTarget = collect_maxDev(data, p, targetPos);

            if ~isempty(maxDevByTarget)
                smoothedData = smooth(maxDevByTarget, 0.2, 'rloess'); % Smooth the data
                plot_handles{targetPos}(p) = plot(ax, smoothedData, 'Color', colors(targetPos), 'LineWidth', 1.5, ...
                    'DisplayName', ['Participant ', num2str(p), ', Target ', num2str(targetPos)]);
                legends{end+1} = ['Participant ', num2str(p), ', Target ', num2str(targetPos)];
            end
        end
    end

    % Add labels, legend, and title
    xlabel('Trial Number');
    ylabel('maxDev');
    title('maxDev of Trajectories for Different Targets - All Participants');
    legend(ax, 'show');
    grid on;

    % Create a slider for selecting target
    uicontrol('Style', 'slider', 'Min', 1, 'Max', numTargets, 'Value', 1, ...
              'SliderStep', [1/(numTargets-1) , 10/(numTargets-1)], ...
              'Position', [150 20 300 20], ...
              'Callback', @(src, event) updatePlot(src, plot_handles, legends));

    % Add a text label to display the selected target
    uicontrol('Style', 'text', 'Position', [460, 20, 120, 20], 'String', 'Select Target');
    hold off;

    function updatePlot(src, plot_handles, legends)
        % Get the value of the slider (which target is selected)
        targetIdx = round(get(src, 'Value'));

        % Update plot visibility based on selected target
        for targetPos = 1:numTargets
            if targetPos == targetIdx
                set([plot_handles{targetPos}], 'Visible', 'on');
            else
                set([plot_handles{targetPos}], 'Visible', 'off');
            end
        end

        % Update the legend
        legend(ax, legends(targetIdx:numTargets:end), 'Location', 'best');
    end

    function maxDevByTarget = collect_maxDev(data, p, targetPos)
        numTrials = length(data.data.part(p).trajPos);
        maxDevByTarget = [];

        % Iterate over trials to collect maxDev values by target position
        for t = 1:numTrials
            if data.data.part(p).iv(t).position == targetPos
                maxDev = data.data.part(p).dv(t).maxDev;
                maxDevByTarget = [maxDevByTarget, maxDev];
            end
        end
    end
end
