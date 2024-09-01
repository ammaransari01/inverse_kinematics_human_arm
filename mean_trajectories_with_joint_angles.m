function mean_trajectories_with_joint_angles(data)
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

    % Define limb lengths (in mm)
    upperarmLength = 320; % Upper arm length in mm
    forearmLength = 480; % Forearm length in mm

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
            figure;
            subplot(2, 1, 1);
            plot3(meanTrajectory(:,1), meanTrajectory(:,2), meanTrajectory(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['Mean Trajectory for Target Position ', num2str(targetPos)]);
            grid on;

            % Plot variance of trajectories in subplot
            subplot(2, 1, 2);
            plot3(varTrajectory(:,1), varTrajectory(:,2), varTrajectory(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['Variance of Trajectory for Target Position ', num2str(targetPos)]);
            grid on;

            % Compute joint angles over the mean trajectory using inverse kinematics
            numPoints = size(meanTrajectory, 1);
            jointAnglesOverTime = zeros(numPoints, 4);

            % Initialize angles (you may adjust the initial guess)
            initialAngles = [0; 0; 0; 0];

            for i = 1:numPoints
                % Get the hand position for the current point in the mean trajectory
                xHand = meanTrajectory(i, 1);
                yHand = meanTrajectory(i, 2);
                zHand = meanTrajectory(i, 3);

                % Calculate the joint angles using inverse kinematics
                [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, initialAngles);

                % Store the joint angles at this point
                jointAnglesOverTime(i, :) = [theta, eta, zeta, phi];
            end

            % Create a new figure for each target position's joint angles
            figure;
            plot(rad2deg(jointAnglesOverTime));
            xlabel('Trajectory Step Point');
            ylabel('Angle (degrees)');
            legend('Theta (Shoulder Elevation)', 'Eta (Shoulder Azimuth)', 'Zeta (Humeral Rotation)', 'Phi (Elbow Flexion)');
            title(['Joint Angles Over Mean Trajectory for Target Position ', num2str(targetPos)]);
            grid on;
        end
    end
end

function [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, initialAngles)
    % Extract initial angles
    theta = initialAngles(1);
    eta = initialAngles(2);
    zeta = initialAngles(3);
    phi = initialAngles(4);

    % Calculate the elbow position
    xElbow = xHand - forearmLength * cos(phi) * cos(zeta);
    yElbow = yHand - forearmLength * cos(phi) * sin(zeta);
    zElbow = zHand + forearmLength * sin(phi);

    % Calculate the shoulder elevation angle (theta)
    theta = acos(-zElbow / upperarmLength);

    % Calculate the shoulder azimuth angle (eta)
    eta = atan2(-xElbow, yElbow);

    % Calculate the elbow flexion angle (phi)
    phi = acos((xHand^2 + yHand^2 + zHand^2 - upperarmLength^2 - forearmLength^2) / (2 * upperarmLength * forearmLength));

    % Calculate the humeral rotation angle (zeta)
    zeta = atan2(upperarmLength * (xElbow * yHand - xHand * yElbow), yElbow * (yElbow * zHand - yHand * zElbow) - xElbow * (zElbow * xHand - zHand * xElbow));

    % Normalize and constrain angles
    theta = max(0, min(theta, pi)); % Shoulder elevation: 0 to 180 degrees
    eta = max(-pi/2, min(eta, pi/2)); % Shoulder azimuth: -90 to 90 degrees
    phi = max(0, min(phi, pi)); % Elbow flexion: 0 to 180 degrees
end