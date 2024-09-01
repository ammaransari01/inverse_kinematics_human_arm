function analyze_high_low_curvature_anglesTrajectories(data)
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

            % Calculate curvature for each trajectory
            curvature = calculateCurvature(trajMatrix);

            % Find high and low curvature trajectories
            [highCurvatureTrajectories, lowCurvatureTrajectories] = identifyCurvatureExtremes(curvature, interpolatedTrajectories);

            % Plot mean trajectory
            figure;
            subplot(2, 2, 1);
            plot3(meanTrajectory(:,1), meanTrajectory(:,2), meanTrajectory(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['Mean Trajectory for Target Position ', num2str(targetPos)]);
            grid on;

            % Plot variance of trajectories
            subplot(2, 2, 2);
            plot3(varTrajectory(:,1), varTrajectory(:,2), varTrajectory(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['Variance of Trajectory for Target Position ', num2str(targetPos)]);
            grid on;

            % Plot high curvature trajectory
            subplot(2, 2, 3);
            plot3(highCurvatureTrajectories{1}(:,1), highCurvatureTrajectories{1}(:,2), highCurvatureTrajectories{1}(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['High Curvature Trajectory for Target Position ', num2str(targetPos)]);
            grid on;

            % Plot low curvature trajectory
            subplot(2, 2, 4);
            plot3(lowCurvatureTrajectories{1}(:,1), lowCurvatureTrajectories{1}(:,2), lowCurvatureTrajectories{1}(:,3));
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title(['Low Curvature Trajectory for Target Position ', num2str(targetPos)]);
            grid on;

            % Compute and plot joint angles for mean trajectory
            plotJointAngles(meanTrajectory, forearmLength, upperarmLength, ...
                ['Joint Angles Over Mean Trajectory for Target Position ', num2str(targetPos)]);

            % Compute and plot joint angles for high curvature trajectory
            plotJointAngles(highCurvatureTrajectories{1}, forearmLength, upperarmLength, ...
                ['Joint Angles for High Curvature Trajectory, Target Position ', num2str(targetPos)]);

            % Compute and plot joint angles for low curvature trajectory
            plotJointAngles(lowCurvatureTrajectories{1}, forearmLength, upperarmLength, ...
                ['Joint Angles for Low Curvature Trajectory, Target Position ', num2str(targetPos)]);
        end
    end
end

function plotJointAngles(trajectory, forearmLength, upperarmLength, plotTitle)
    % Compute joint angles over the trajectory using inverse kinematics
    numPoints = size(trajectory, 1);
    jointAnglesOverTime = zeros(numPoints, 4);

    % Initialize angles (you may adjust the initial guess)
    initialAngles = [0; 0; 0; 0];

    for i = 1:numPoints
        % Get the hand position for the current point in the trajectory
        xHand = trajectory(i, 1);
        yHand = trajectory(i, 2);
        zHand = trajectory(i, 3);

        % Calculate the joint angles using inverse kinematics
        [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, initialAngles);

        % Store the joint angles at this point
        jointAnglesOverTime(i, :) = [theta, eta, zeta, phi];
    end

    % Plot joint angles
    figure;
    plot(rad2deg(jointAnglesOverTime));
    xlabel('Trajectory Point');
    ylabel('Angle (degrees)');
    legend('Theta (Shoulder Elevation)', 'Eta (Shoulder Azimuth)', 'Zeta (Humeral Rotation)', 'Phi (Elbow Flexion)');
    title(plotTitle);
    grid on;
end

function curvature = calculateCurvature(trajMatrix)
    % Number of points and trajectories
    [numPoints, ~, numTrajectories] = size(trajMatrix);
    
    % Initialize curvature matrix
    curvature = zeros(numPoints, numTrajectories);

    % Compute curvature for each trajectory
    for i = 1:numTrajectories
        trajectory = trajMatrix(:, :, i);
        curvature(:, i) = computeCurvatureForTrajectory(trajectory);
    end
end

function curvatures = computeCurvatureForTrajectory(trajectory)
    % Compute curvature using second derivative approach
    numPoints = size(trajectory, 1);
    curvatures = zeros(numPoints, 1);

    for i = 2:numPoints-1
        % Central differences to compute second derivatives
        dx1 = trajectory(i+1, 1) - trajectory(i, 1);
        dy1 = trajectory(i+1, 2) - trajectory(i, 2);
        dz1 = trajectory(i+1, 3) - trajectory(i, 3);

        dx2 = trajectory(i, 1) - trajectory(i-1, 1);
        dy2 = trajectory(i, 2) - trajectory(i-1, 2);
        dz2 = trajectory(i, 3) - trajectory(i-1, 3);

        curvature(i) = sqrt((dx1*dz2 - dx2*dz1)^2 + (dy1*dz2 - dy2*dz1)^2 + (dx1*dy2 - dx2*dy1)^2) / ...
                        ((dx1^2 + dy1^2 + dz1^2)^(3/2));
    end
end

function [highCurvatureTrajectories, lowCurvatureTrajectories] = identifyCurvatureExtremes(curvature, interpolatedTrajectories)
    % Calculate mean curvature for each trajectory
    meanCurvatures = mean(curvature, 1);

    % Identify indices of high and low curvature trajectories
    [~, sortedIndices] = sort(meanCurvatures);
    numTrajectories = length(sortedIndices);

    % Choose top 10% and bottom 10% as high and low curvature
    numHigh = max(1, floor(numTrajectories * 0.1));
    numLow = max(1, floor(numTrajectories * 0.1));

    highCurvatureIndices = sortedIndices(end-numHigh+1:end);
    lowCurvatureIndices = sortedIndices(1:numLow);

    % Extract high and low curvature trajectories
    highCurvatureTrajectories = interpolatedTrajectories(highCurvatureIndices);
    lowCurvatureTrajectories = interpolatedTrajectories(lowCurvatureIndices);
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