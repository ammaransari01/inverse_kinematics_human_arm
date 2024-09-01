function plot_std_deviation_of_trajectories_all_participants(data)
    % Set parameters
    numTargets = 3;

    % Get the number of participants
    numParticipants = length(data.data.part);

    % Preallocate storage for trajectories organized by target position for all participants
    allTrajectoriesByTarget = cell(numTargets, 1);

    % Iterate over all participants
    for p = 1:numParticipants
        % Iterate over the trials of the current participant
        numTrials = length(data.data.part(p).trajPos);
        for t = 1:numTrials
            % Get the target position for the current trial
            targetPos = data.data.part(p).iv(t).position;

            % Collect the trajectory for the current trial and target position
            if isempty(allTrajectoriesByTarget{targetPos})
            allTrajectoriesByTarget{targetPos} = {data.data.part(p).dv(t).trajectory};
            else
            allTrajectoriesByTarget{targetPos} = [allTrajectoriesByTarget{targetPos}, data.data.part(p).dv(t).trajectory];
            end
        end
    end

    % Function to interpolate trajectories to the same length
    interpolateTrajectories = @(trajectories, maxPoints) cellfun(@(traj) interp1(1:size(traj,1), traj, linspace(1, size(traj,1), maxPoints)), trajectories, 'UniformOutput', false);

    % Prepare a figure for the standard deviation plots
    figure;
    hold on;

    % Compute and plot the standard deviation for each target position
    for targetPos = 1:numTargets
        % Get all trajectories for the current target position
        trajectories = allTrajectoriesByTarget{targetPos};

        if ~isempty(trajectories)
            % Determine the maximum length of trajectories
            maxPoints = max(cellfun(@(traj) size(traj, 1), trajectories));

            % Interpolate all trajectories to the maximum length
            interpolatedTrajectories = interpolateTrajectories(trajectories, maxPoints);

            % Convert cell array to 3D matrix (points x 3 dimensions x trials)
            trajMatrix = cell2mat(reshape(interpolatedTrajectories, 1, 1, []));

            % Calculate the standard deviation along the trajectory
            stdTrajectory = std(trajMatrix, 0, 3, 'omitnan');

            % Plot the standard deviation for each dimension (X, Y, Z) separately
            subplot(3, 1, 1);
            plot(1:maxPoints, stdTrajectory(:, 1), 'DisplayName', ['Target ' num2str(targetPos)]);
            xlabel('Trajectory Point');
            ylabel('Std Deviation (X)');
            title('Standard Deviation of X Position Across Targets');
            hold on;
            grid on;

            subplot(3, 1, 2);
            plot(1:maxPoints, stdTrajectory(:, 2), 'DisplayName', ['Target ' num2str(targetPos)]);
            xlabel('Trajectory Point');
            ylabel('Std Deviation (Y)');
            title('Standard Deviation of Y Position Across Targets');
            hold on;
            grid on;

            subplot(3, 1, 3);
            plot(1:maxPoints, stdTrajectory(:, 3), 'DisplayName', ['Target ' num2str(targetPos)]);
            xlabel('Trajectory Point');
            ylabel('Std Deviation (Z)');
            title('Standard Deviation of Z Position Across Targets');
            hold on;
            grid on;
        end
    end

    % Add legends to each subplot
    subplot(3, 1, 1);
    legend('show');
    subplot(3, 1, 2);
    legend('show');
    subplot(3, 1, 3);
    legend('show');

    hold off;
end
