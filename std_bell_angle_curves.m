%% Calculating the average and standard deviation of joint angles for each 
%% target and Plotting the bell curves for each joint angle and target

% Define the calcInverseKinematics function
function [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, alpha)
    % Define some internal parameters for the calculation
    xElbow = 160;
    yElbow = 150;
    zElbow = -300;

    % Calculate joint angles using inverse kinematics principles
    theta = acos(-zElbow / upperarmLength);
    eta = atan2(-xElbow, yElbow);
    zeta = atan2(upperarmLength * (xElbow * yHand - xHand * yElbow), ...
                 yElbow * (yElbow * zHand - yHand * zElbow) - xElbow * (zElbow * xHand - zHand * xElbow));
    phi = acos((xHand^2 + yHand^2 + zHand^2 - upperarmLength^2 - forearmLength^2)/(2 * upperarmLength * forearmLength));

    % Apply joint limits
    theta = max(0, min(theta, pi)); % Ensure theta is between 0 and 180 degrees
    eta = max(-pi/2, min(eta, pi/2)); % Ensure eta is between -90 and 90 degrees
    phi = max(0, min(phi, pi)); % Ensure phi is between 0 and 180 degrees
end

% Main Script
% Initialize parameters
lu = 320; % Upper arm length in mm
lf = 480; % Forearm length in mm
alpha = 0; % Example value

% Initialize matrices to store joint angles for all participants and targets
all_joint_angles_target1 = [];
all_joint_angles_target2 = [];
all_joint_angles_target3 = [];

% Loop through each participant
for p = 1:length(data.data.part)
    % Initialize storage for joint angles for each target for the current participant
    joint_angles_target1 = [];
    joint_angles_target2 = [];
    joint_angles_target3 = [];
    
    % Loop through each trial for the current participant
    for t = 1:length(data.data.part(p).dv)
        % Extract trajectory and target position
        trajPos = data.data.part(p).dv(t).trajectory;
        trajPos = trajPos - [392 -386 226]; % Adjust for origin

        % Determine the target from the position field
        target = data.data.part(p).iv(t).position;

        % Calculate the number of steps in the trajectory
        number_of_steps = size(trajPos, 1);

        % Initial hand position (first step in trajectory)
        xHand = trajPos(1, 1);
        yHand = trajPos(1, 2);
        zHand = trajPos(1, 3);

        % Call the inverse kinematics function for the first step
        [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, lf, lu, alpha);

        % Store initial angles
        angles = [theta, eta, zeta, phi]';

        % Initialize matrix to store joint angles for this trajectory
        joint_angles_over_time = zeros(number_of_steps, 4);
        joint_angles_over_time(1, :) = angles';

        % Loop through each step in the trajectory
        for i = 2:number_of_steps
            old_pos = trajPos(i-1, :); % Previous step position
            new_pos = trajPos(i, :);   % Current step position
            delta_hand = new_pos - old_pos; % Delta hand position

            % Compute the Jacobian matrix and its pseudo-inverse
            jh = jacobian_solver(lu, lu, angles(1), angles(2), angles(3), angles(4));
            inverse_jh = pinv(jh);

            % Calculate the change in angles
            delta_angles = inverse_jh * delta_hand';

            % Update the angles
            angles = angles + delta_angles;

            % Store the angles at this step
            joint_angles_over_time(i, :) = angles';
        end

        % Store joint angles for this target
        switch target
            case 1
                joint_angles_target1 = [joint_angles_target1; joint_angles_over_time];
            case 2
                joint_angles_target2 = [joint_angles_target2; joint_angles_over_time];
            case 3
                joint_angles_target3 = [joint_angles_target3; joint_angles_over_time];
        end
    end

    % Append this participant's data to the overall data for all participants
    all_joint_angles_target1 = [all_joint_angles_target1; joint_angles_target1];
    all_joint_angles_target2 = [all_joint_angles_target2; joint_angles_target2];
    all_joint_angles_target3 = [all_joint_angles_target3; joint_angles_target3];
end

% Calculate the average and standard deviation of joint angles for each target
mean_angles_target1 = mean(rad2deg(all_joint_angles_target1), 1);
mean_angles_target2 = mean(rad2deg(all_joint_angles_target2), 1);
mean_angles_target3 = mean(rad2deg(all_joint_angles_target3), 1);

std_angles_target1 = std(rad2deg(all_joint_angles_target1), 0, 1);
std_angles_target2 = std(rad2deg(all_joint_angles_target2), 0, 1);
std_angles_target3 = std(rad2deg(all_joint_angles_target3), 0, 1);

% Plot the bell curves for each joint angle and target
figure;

% Theta (Shoulder Elevation)
subplot(2, 2, 1);
hold on;
theta_range = linspace(min([mean_angles_target1(1), mean_angles_target2(1), mean_angles_target3(1)]) - 3*max([std_angles_target1(1), std_angles_target2(1), std_angles_target3(1)]), ...
                       max([mean_angles_target1(1), mean_angles_target2(1), mean_angles_target3(1)]) + 3*max([std_angles_target1(1), std_angles_target2(1), std_angles_target3(1)]), 100);
plot(theta_range, normpdf(theta_range, mean_angles_target1(1), std_angles_target1(1)), 'r-');
plot(theta_range, normpdf(theta_range, mean_angles_target2(1), std_angles_target2(1)), 'g-');
plot(theta_range, normpdf(theta_range, mean_angles_target3(1), std_angles_target3(1)), 'b-');
title('Theta (Shoulder Elevation)');
xlabel('Angle (degrees)');
ylabel('Probability Density');
legend('Target 1', 'Target 2', 'Target 3');
grid on;

% Eta (Shoulder Azimuth)
subplot(2, 2, 2);
hold on;
eta_range = linspace(min([mean_angles_target1(2), mean_angles_target2(2), mean_angles_target3(2)]) - 3*max([std_angles_target1(2), std_angles_target2(2), std_angles_target3(2)]), ...
                     max([mean_angles_target1(2), mean_angles_target2(2), mean_angles_target3(2)]) + 3*max([std_angles_target1(2), std_angles_target2(2), std_angles_target3(2)]), 100);
plot(eta_range, normpdf(eta_range, mean_angles_target1(2), std_angles_target1(2)), 'r-');
plot(eta_range, normpdf(eta_range, mean_angles_target2(2), std_angles_target2(2)), 'g-');
plot(eta_range, normpdf(eta_range, mean_angles_target3(2), std_angles_target3(2)), 'b-');
title('Eta (Shoulder Azimuth)');
xlabel('Angle (degrees)');
ylabel('Probability Density');
legend('Target 1', 'Target 2', 'Target 3');
grid on;

% Zeta (Humeral Angle)
subplot(2, 2, 3);
hold on;
zeta_range = linspace(min([mean_angles_target1(3), mean_angles_target2(3), mean_angles_target3(3)]) - 3*max([std_angles_target1(3), std_angles_target2(3), std_angles_target3(3)]), ...
                      max([mean_angles_target1(3), mean_angles_target2(3), mean_angles_target3(3)]) + 3*max([std_angles_target1(3), std_angles_target2(3), std_angles_target3(3)]), 100);
plot(zeta_range, normpdf(zeta_range, mean_angles_target1(3), std_angles_target1(3)), 'r-');
plot(zeta_range, normpdf(zeta_range, mean_angles_target2(3), std_angles_target2(3)), 'g-');
plot(zeta_range, normpdf(zeta_range, mean_angles_target3(3), std_angles_target3(3)), 'b-');
title('Zeta (Humeral Angle)');
xlabel('Angle (degrees)');
ylabel('Probability Density');
legend('Target 1', 'Target 2', 'Target 3');
grid on;

% Phi (Elbow Flexion)
subplot(2, 2, 4);
hold on;
phi_range = linspace(min([mean_angles_target1(4), mean_angles_target2(4), mean_angles_target3(4)]) - 3*max([std_angles_target1(4), std_angles_target2(4), std_angles_target3(4)]), ...
                     max([mean_angles_target1(4), mean_angles_target2(4), mean_angles_target3(4)]) + 3*max([std_angles_target1(4), std_angles_target2(4), std_angles_target3(4)]), 100);
plot(phi_range, normpdf(phi_range, mean_angles_target1(4), std_angles_target1(4)), 'r-');
plot(phi_range, normpdf(phi_range, mean_angles_target2(4), std_angles_target2(4)), 'g-');
plot(phi_range, normpdf(phi_range, mean_angles_target3(4), std_angles_target3(4)), 'b-');
title('Phi (Elbow Flexion)');
xlabel('Angle (degrees)');
ylabel('Probability Density');
legend('Target 1', 'Target 2', 'Target 3');
grid on;