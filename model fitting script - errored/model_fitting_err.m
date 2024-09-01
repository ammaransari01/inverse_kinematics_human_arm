% Set the blockTrial to 3 or 4
blockTrial = 3;  % Change this to 4 for the second block

% Number of trials per block
trialsPerBlock = 96;

% Number of target positions
numTargets = 3;

% Number of trials to skip from the start and end
trialsToSkip = 13;

% Preallocate storage for trajectories organized by target position
allTrajectoriesByTarget = cell(numTargets, 1);

% Iterate over participants and trials to collect trajectories by target position
for p = 2 % numParticipants
    numTrials = length(data.data.part(p).trajPos);
    
    % Determine the start and end indices for the selected block
    if blockTrial == 3
        blockStart = 1;
        blockEnd = trialsPerBlock;
    elseif blockTrial == 4
        blockStart = trialsPerBlock + 1;
        blockEnd = 2 * trialsPerBlock;
    else
        error('blockTrial must be either 3 or 4');
    end
    
    % Determine the range of trials to consider
    middleTrialsRange = (blockStart + trialsToSkip):(blockEnd - trialsToSkip);
    
    for t = middleTrialsRange
        % Get the target position for the current trial
        targetPos = data.data.part(p).iv(t).position; % Assuming this field exists
        % Collect the trajectory for the current trial and target position
        if isempty(allTrajectoriesByTarget{targetPos})
            allTrajectoriesByTarget{targetPos} = {data.data.part(p).trajPos{t}};
        else
            allTrajectoriesByTarget{targetPos} = [allTrajectoriesByTarget{targetPos}, data.data.part(p).trajPos{t}];
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

% Save the mean trajectory for the first target position for later use
meanTrajData = meanTrajectory;

% Function to calculate forward kinematics of a human arm
function [xHand, yHand, zHand] = calcForwardKinematic(theta, eta, omicron, phi, forearmLength, upperarmLength)
    % forward kinematic according to A. Biess; D. G. Liebermann and T. Flash (2007)
    % The name of the angles follow page 13047.

    % Calculate the elbow position
    xElbow = -upperarmLength * sin(eta) * sin(theta);
    yElbow = -upperarmLength * cos(eta) * sin(theta);
    zElbow = -upperarmLength * cos(theta);

    % Calculate the hand position
    xHand = xElbow - forearmLength * (sin(phi) * (cos(omicron) * sin(eta) * cos(theta) + sin(omicron) * cos(eta)) + cos(phi) * sin(eta) * sin(theta));
    yHand = yElbow + forearmLength * (sin(phi) * (cos(omicron) * cos(eta) * cos(theta) + sin(omicron) * sin(eta)) + cos(phi) * cos(eta) * sin(theta));
    zHand = zElbow + forearmLength * (sin(phi) * cos(omicron) * cos(eta) * sin(theta) - cos(phi) * cos(theta));
end

% Function to calculate the error between the model and the data trajectory
function error = calcError(funcForwardKinematic, param, dataTraj, forearmLength, upperarmLength)
    steps = size(dataTraj, 1); % Number of steps in the trajectory

    % Preallocate arrays for storing the generated trajectory
    x = nan(steps, 1);
    y = nan(steps, 1);
    z = nan(steps, 1);

    % Generate the human arm trajectory
    for i = 1:steps
        theta = param(5) + param(1) * i;
        eta = param(6) + param(2) * i;
        omicron = param(7) + param(3) * i;
        phi = param(8) + param(4) * i;

        [x(i), y(i), z(i)] = feval(funcForwardKinematic, theta, eta, omicron, phi, forearmLength, upperarmLength);
    end

    % Calculate the error as the mean squared error between the data and the generated trajectory
    error = mean((dataTraj(:, 1) - x).^2 + (dataTraj(:, 2) - y).^2 + (dataTraj(:, 3) - z).^2);
end

% Function to fit the trajectory using the error function
function [vTheta, vEta, vOmicron, vPhi, sTheta, sEta, sOmicron, sPhi] = fitTrajectory(funcError, funcForwardKinematic, trajectory)
    % Fit model to data
    % vAngle is the velocity of the respective joint angle
    % sAngle is the starting angle of the movement

    % Set initial guesses for the parameters
    initialParams = zeros(8, 1);
    
    % Set options for fminsearch
    options = optimset('Display', 'iter', 'TolX', 1e-6, 'TolFun', 1e-6);
    % Optimize the parameters using fminsearch
    res = fminsearch(@(x) feval(funcError, funcForwardKinematic, x, trajectory, 250, 300), initialParams, options);

    % Extract the fitted parameters
    vTheta = res(1);
    vEta = res(2);
    vOmicron = res(3);
    vPhi = res(4);

    sTheta = res(5);
    sEta = res(6);
    sOmicron = res(7);
    sPhi = res(8);
end

% Function to plot the trajectory
function h = plotTrajectory(steps, vTheta, vEta, vOmicron, vPhi, sTheta, sEta, sOmicron, sPhi, dataTraj)
    % Arm segment lengths
    forearmLength = 300;
    upperarmLength = 250;

    % Generate and plot the model trajectory
    modelX = nan(steps, 1);
    modelY = nan(steps, 1);
    modelZ = nan(steps, 1);

    for i = 1:steps
        % Calculate the joint angles at each step
        theta = sTheta + vTheta * i;
        eta = sEta + vEta * i;
        omicron = sOmicron + vOmicron * i;
        phi = sPhi + vPhi * i;
        
        % Calculate the hand position
        [modelX(i), modelY(i), modelZ(i)] = calcForwardKinematic(theta, eta, omicron, phi, forearmLength, upperarmLength);
    end
    
    % Plot the model and data trajectories
    plot3(modelX, modelY, modelZ, 'b-', 'LineWidth', 2); hold on;
    plot3(dataTraj(:,1), dataTraj(:,2), dataTraj(:,3), 'r--', 'LineWidth', 1);
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    title('Model vs Data Trajectory');
    legend('Model Trajectory', 'Data Trajectory');
    grid on;
    hold off;
end


%% Loaded data and perform model fitting
trajData = meanTrajData;  % Use actual mean trajectory data here
[vTheta, vEta, vOmicron, vPhi, sTheta, sEta, sOmicron, sPhi] = fitTrajectory('calcError', 'calcForwardKinematic', trajData);
% Plot the fitted model trajectory vs data trajectory
figure;
plotTrajectory(size(trajData, 1), vTheta, vEta, vOmicron, vPhi, sTheta, sEta, sOmicron, sPhi, trajData);