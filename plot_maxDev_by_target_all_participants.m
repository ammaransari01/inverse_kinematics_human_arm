function plot_maxDev_by_target_all_participants(data)
    % Number of participants
    numParticipants = length(data.data.part);
    % Number of targets
    numTargets = 3;

    % Create a figure for all participants
    figure;
    hold on;
    colors = ['r', 'g', 'b']; % Colors for different targets

    for p = 1:numParticipants
        % Preallocate storage for maxDev values organized by target position
        maxDevByTarget = cell(numTargets, 1);

        % Get the number of trials for the current participant
        numTrials = length(data.data.part(p).trajPos);

        % Iterate over trials to collect maxDev values by target position
        for t = 1:numTrials
            % Get the target position for the current trial
            targetPos = data.data.part(p).iv(t).position;

            % Get the maxDev value for the current trial
            maxDev = data.data.part(p).dv(t).maxDev;

            % Collect the maxDev value for the corresponding target position
            if isempty(maxDevByTarget{targetPos})
                maxDevByTarget{targetPos} = maxDev;
            else
                maxDevByTarget{targetPos} = [maxDevByTarget{targetPos}, maxDev];
            end
        end

        % Plot smooth maxDev values for each target position for the current participant
        for targetPos = 1:numTargets
            if ~isempty(maxDevByTarget{targetPos})
                smoothedData = smooth(maxDevByTarget{targetPos}, 0.2, 'rloess'); % Smooth the data
                plot(smoothedData, 'Color', colors(targetPos), 'LineWidth', 1.5, ...
                     'DisplayName', ['Participant ', num2str(p), ', Target ', num2str(targetPos)]);
            end
        end
    end

    % Add labels, legend, and title
    xlabel('Trial Number');
    ylabel('maxDev');
    title('maxDev of Trajectories for Different Targets - All Participants');
    legend('show');
    grid on;
    hold off;
end
