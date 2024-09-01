function mean_and_variance_trajectories(data)
    % Number of participants
    numParticipants = length(data.data.part);
    % Number of targets
    numTargets = 3;

    % Preallocate storage for trajectories organized by target position
    allTrajectoriesByTarget = cell(numTargets, 1);

    % Iterate over participants and trials to collect trajectories by target position
    for p = 1:numParticipants
        numTrials = length(data.data.part(p).dv);
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

    % Create a figure for mean and variance trajectories
    figure;

    % Compute mean and variance for each target position
    for targetPos = 1:numTargets
        % Get all trajectories for the current target position
        trajectories = allTrajectoriesByTarget{targetPos};

        if ~isempty(trajectories)
            % Determine the maximum length of trajectories
            maxPoints = max(cellfun(@(traj) size(traj, 1), trajectories));

            % Interpolate all trajectories to the maximum length
            interpolatedTrajectories = interpolateTrajectories(trajectories, maxPoints);

            % Convert cell array to 3D matrix
            trajMatrix = cell2mat(reshape(interpolatedTrajectories, 1, 1, []));

            % Calculate mean and variance trajectories
            meanTrajectory = mean(trajMatrix, 3, 'omitnan');
            varTrajectory = var(trajMatrix, 0, 3, 'omitnan');

            % Plot mean trajectory in subplot
            subplot(2, numTargets, targetPos);
            plot3(meanTrajectory(:,1), meanTrajectory(:,2), meanTrajectory(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['Mean Trajectory for Target Position ', num2str(targetPos)]);
            grid on;

            % Plot variance of trajectories in subplot
            subplot(2, numTargets, numTargets + targetPos);
            plot3(varTrajectory(:,1), varTrajectory(:,2), varTrajectory(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['Variance of Trajectory for Target Position ', num2str(targetPos)]);
            grid on;
        end
    end
end