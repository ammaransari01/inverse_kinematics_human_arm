% Radians to Degrees with Modulus

% Define the hand position
xHand = 10; % value in cm
yHand = 10; % value in cm
zHand = 10; % value in cm

% Define the arm segment lengths
lu = 32; % Upper arm length in cm
lf = 45; % Forearm length in cm

% Define the alpha parameter
alpha = 0; % Example value

% Function to calculate inverse kinematics
function [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, alpha)
    % Calculate the distance from shoulder to hand
    r = sqrt(xHand^2 + yHand^2 + zHand^2);

    % Calculate the elbow flexion angle phi
    phi = acos((r^2 - upperarmLength^2 - forearmLength^2) / (2 * upperarmLength * forearmLength));

    % Calculate the elbow position using alpha
    c = upperarmLength + forearmLength * cos(phi);
    s = forearmLength * sin(phi);

    % Define the axis of rotation
    axis = [0; 0; 1];
    crossProd = cross(axis, [xHand; yHand; zHand]);
    crossCrossProd = cross(crossProd, [xHand; yHand; zHand]);

    % Calculate elbow positions
    xElbow = (c / r) * xHand + (s / r) * (cos(alpha) * crossProd(1) + sin(alpha) * crossCrossProd(1));
    yElbow = (c / r) * yHand + (s / r) * (cos(alpha) * crossProd(2) + sin(alpha) * crossCrossProd(2));
    zElbow = (c / r) * zHand + (s / r) * (cos(alpha) * crossProd(3) + sin(alpha) * crossCrossProd(3));

    % Calculate joint angles
    theta = acos(-zElbow / upperarmLength);
    eta = atan2(-xElbow, yElbow);
    zeta = atan2(upperarmLength * (xElbow * yHand - xHand * yElbow), yElbow * (yElbow * zHand - yHand * zElbow) - xElbow * (zElbow * xHand - zHand * xElbow));

    % Apply joint limits
    theta = max(0, min(theta, pi)); % Ensure theta is between 0 and 180 degrees
    eta = max(-pi/2, min(eta, pi/2)); % Ensure eta is between -90 and 90 degrees
    phi = max(0, min(phi, pi)); % Ensure phi is between 0 and 180 degrees
end

% Call the inverse kinematics function
[theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, lf, lu, alpha);

% Convert the joint angles from radians to degrees
theta_deg = rad2deg(theta);
eta_deg = rad2deg(eta);
zeta_deg = rad2deg(zeta);
phi_deg = rad2deg(phi);
angles = [theta_deg, eta_deg, zeta_deg, phi_deg]'; % Store in degrees

% Loading the trajectory data
trajPos = data.data.part(2).trajPos{35}; % Example for participant 1, trial 35
% trajPos = trajPos - [48 386 226];
trajPos = trajPos - [392 -386 226];
number_of_steps = size(trajPos, 1);
% Matrix to store the computed joint angles at each step
joint_angles_over_time = zeros(number_of_steps, 4);

% Store initial angles in the first row of joint_angles_over_timezeta
joint_angles_over_time(1, :) = angles';

for i = 2:number_of_steps
    old_pos = trajPos(i-1, :);           % Previous and current step positions
    new_pos = trajPos(i, :);
    delta_hand = new_pos - old_pos;  % Calculate the delta hand position

    % Compute the Jacobian matrix at the current angles
    jh = jacobian_solver(lu, lu, angles(1), angles(2), angles(3), angles(4));
    inverse_jh = pinv(jh);

    % Change in angles
    delta_angles = inverse_jh * delta_hand';
    delta_angles_deg = rad2deg(delta_angles); % Convert delta_angles to degrees

    % Update the angles and constrain them
    angles = angles + delta_angles_deg;

    % Optionally, constrain angles here if needed
    angles(1) = max(0, min(angles(1), 180)); % theta in degrees
    angles(2) = max(-90, min(angles(2), 90)); % eta in degrees
    angles(3) = max(0, min(angles(3), 180)); % zeta in degrees
    angles(4) = max(0, min(angles(4), 180)); % phi in degrees

    % Store the angles at this step
    joint_angles_over_time(i, :) = angles';

    % Print the joint angles for this step in degrees
    fprintf('Step %d:\n', i);
    fprintf('Theta (Shoulder Elevation): %.2f degrees\n', angles(1));
    fprintf('Eta (Shoulder Azimuth): %.2f degrees\n', angles(2));
    fprintf('Zeta (Humeral Angle): %.2f degrees\n', angles(3));
    fprintf('Phi (Elbow Flexion): %.2f degrees\n', angles(4));
    fprintf('------------------------\n');
end

% Plot joint angles over time in degrees
figure;
plot(joint_angles_over_time);
xlabel('Step');
ylabel('Angle (degrees)');
legend('Theta (Shoulder Elevation)', 'Eta (Shoulder Azimuth)', 'Zeta (Humeral Angle)', 'Phi (Elbow Flexion)');
title('Joint Angles Over Time');
grid on;

% Display the results in degrees
fprintf('Calculated Joint Angles in Degrees:\n');
fprintf('Theta (Shoulder Elevation): %.2f degrees\n', theta_deg);
fprintf('Eta (Shoulder Azimuth): %.2f degrees\n', eta_deg);
fprintf('Zeta (Humeral Angle): %.2f degrees\n', zeta_deg);
fprintf('Phi (Elbow Flexion): %.2f degrees\n', phi_deg);

%% Radians to Degrees with Modulus and Simple Moving Average Filter
% % Define the hand position
% xHand = 10; % value in cm
% yHand = 15; % value in cm
% zHand = 13; % value in cm
% 
% % Define the arm segment lengths
% lu = 32; % Upper arm length in cm
% lf = 45; % Forearm length in cm
% 
% % Define the alpha parameter
% alpha = 0; % Example value
% 
% % Function to calculate inverse kinematics
% function [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, alpha)
%     % Calculate the distance from shoulder to hand
%     r = sqrt(xHand^2 + yHand^2 + zHand^2);
% 
%     % Calculate the elbow flexion angle phi
%     phi = acos((r^2 - upperarmLength^2 - forearmLength^2) / (2 * upperarmLength * forearmLength));
% 
%     % Calculate the elbow position using alpha
%     c = upperarmLength + forearmLength * cos(phi);
%     s = forearmLength * sin(phi);
% 
%     % Define the axis of rotation
%     axis = [0; 0; 1];
%     crossProd = cross(axis, [xHand; yHand; zHand]);
%     crossCrossProd = cross(crossProd, [xHand; yHand; zHand]);
% 
%     % Calculate elbow positions
%     xElbow = (c / r) * xHand + (s / r) * (cos(alpha) * crossProd(1) + sin(alpha) * crossCrossProd(1));
%     yElbow = (c / r) * yHand + (s / r) * (cos(alpha) * crossProd(2) + sin(alpha) * crossCrossProd(2));
%     zElbow = (c / r) * zHand + (s / r) * (cos(alpha) * crossProd(3) + sin(alpha) * crossCrossProd(3));
% 
%     % Calculate joint angles
%     theta = acos(-zElbow / upperarmLength);
%     eta = atan2(-xElbow, yElbow);
%     zeta = atan2(upperarmLength * (xElbow * yHand - xHand * yElbow), yElbow * (yElbow * zHand - yHand * zElbow) - xElbow * (zElbow * xHand - zHand * xElbow));
% 
%     % Apply joint limits
%     theta = max(0, min(theta, pi)); % Ensure theta is between 0 and 180 degrees
%     eta = max(-pi/2, min(eta, pi/2)); % Ensure eta is between -90 and 90 degrees
%     phi = max(0, min(phi, pi)); % Ensure phi is between 0 and 180 degrees
% end
% 
% % Call the inverse kinematics function
% [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, lf, lu, alpha);
% 
% % Convert the joint angles from radians to degrees
% theta_deg = rad2deg(theta);
% eta_deg = rad2deg(eta);
% zeta_deg = rad2deg(zeta);
% phi_deg = rad2deg(phi);
% angles = [theta_deg, eta_deg, zeta_deg, phi_deg]'; % Store in degrees
% 
% % Loading the trajectory data
% trajPos = data.data.part(2).trajPos{35}; % Example for participant 1, trial 1
% number_of_steps = size(trajPos, 1);
% 
% % Matrix to store the computed joint angles at each step
% joint_angles_over_time = zeros(number_of_steps, 4);
% 
% % Store initial angles in the first row of joint_angles_over_time
% joint_angles_over_time(1, :) = angles';
% 
% for i = 2:number_of_steps
%     old_pos = trajPos(i-1, :);           % Previous and current step positions
%     new_pos = trajPos(i, :);
%     delta_hand = new_pos - old_pos;  % Calculate the delta hand position
% 
%     % Compute the Jacobian matrix at the current angles
%     jh = jacobian_test(lu, lu, angles(1), angles(2), angles(3), angles(4));
%     inverse_jh = pinv(jh);
% 
%     % Change in angles
%     delta_angles = inverse_jh * delta_hand';
%     delta_angles_deg = rad2deg(delta_angles); % Convert delta_angles to degrees
% 
%     % Update the angles and constrain them
%     angles = angles + delta_angles_deg;
% 
%     % Optionally, constrain angles here if needed
%     angles(1) = max(0, min(angles(1), 180)); % theta in degrees
%     angles(2) = max(-90, min(angles(2), 90)); % eta in degrees
%     angles(3) = max(0, min(angles(3), 180)); % zeta in degrees
%     angles(4) = max(0, min(angles(4), 180)); % phi in degrees
% 
%     % Store the angles at this step
%     joint_angles_over_time(i, :) = angles';
% 
%     % Print the joint angles for this step in degrees
%     fprintf('Step %d:\n', i);
%     fprintf('Theta (Shoulder Elevation): %.2f degrees\n', angles(1));
%     fprintf('Eta (Shoulder Azimuth): %.2f degrees\n', angles(2));
%     fprintf('Zeta (Humeral Angle): %.2f degrees\n', angles(3));
%     fprintf('Phi (Elbow Flexion): %.2f degrees\n', angles(4));
%     fprintf('------------------------\n');
% end
% 
% % Apply a simple moving average filter to smooth the data
% window_size = 15; % Define the window size for the moving average filter
% smooth_joint_angles = zeros(number_of_steps, 4);
% for j = 1:4
%     smooth_joint_angles(:, j) = movmean(joint_angles_over_time(:, j), window_size);
% end
% 
% % Plot joint angles over time in degrees
% figure;
% plot(smooth_joint_angles);
% xlabel('Step');
% ylabel('Angle (degrees)');
% legend('Theta (Shoulder Elevation)', 'Eta (Shoulder Azimuth)', 'Zeta (Humeral Angle)', 'Phi (Elbow Flexion)');
% title('Smoothed Joint Angles Over Time');
% grid on;
% 
% % Display the results in degrees
% fprintf('Calculated Joint Angles in Degrees:\n');
% fprintf('Theta (Shoulder Elevation): %.2f degrees\n', theta_deg);
% fprintf('Eta (Shoulder Azimuth): %.2f degrees\n', eta_deg);
% fprintf('Zeta (Humeral Angle): %.2f degrees\n', zeta_deg);
% fprintf('Phi (Elbow Flexion): %.2f degrees\n', phi_deg);


%% Radians to Degrees without Modulus

% % Define the hand position
% xHand = 10; % value in cm
% yHand = 15; % value in cm
% zHand = 13; % value in cm
% 
% % Define the arm segment lengths
% lu = 32; % Upper arm length in cm
% lf = 45; % Forearm length in cm
% 
% % Define the alpha parameter
% alpha = 0; % Example value
% 
% % Function to calculate inverse kinematics
% function [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, alpha)
%     % Calculate the distance from shoulder to hand
%     r = sqrt(xHand^2 + yHand^2 + zHand^2);
% 
%     % Calculate the elbow flexion angle phi
%     phi = acos((r^2 - upperarmLength^2 - forearmLength^2) / (2 * upperarmLength * forearmLength));
% 
%     % Calculate the elbow position using alpha
%     c = upperarmLength + forearmLength * cos(phi);
%     s = forearmLength * sin(phi);
% 
%     % Define the axis of rotation
%     axis = [0; 0; 1];
%     crossProd = cross(axis, [xHand; yHand; zHand]);
%     crossCrossProd = cross(crossProd, [xHand; yHand; zHand]);
% 
%     % Calculate elbow positions
%     xElbow = (c / r) * xHand + (s / r) * (cos(alpha) * crossProd(1) + sin(alpha) * crossCrossProd(1));
%     yElbow = (c / r) * yHand + (s / r) * (cos(alpha) * crossProd(2) + sin(alpha) * crossCrossProd(2));
%     zElbow = (c / r) * zHand + (s / r) * (cos(alpha) * crossProd(3) + sin(alpha) * crossCrossProd(3));
% 
%     % Calculate joint angles
%     theta = acos(-zElbow / upperarmLength);
%     eta = atan2(-xElbow, yElbow);
%     zeta = atan2(upperarmLength * (xElbow * yHand - xHand * yElbow), yElbow * (yElbow * zHand - yHand * zElbow) - xElbow * (zElbow * xHand - zHand * xElbow));
% 
%     % Apply joint limits
%     theta = max(0, min(theta, pi));
%     eta = max(-3*pi/4, min(eta, pi/3));
%     phi = max(0, min(phi, pi));
% end
% 
% % Call the inverse kinematics function
% [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, lf, lu, alpha);
% 
% % Convert the joint angles from radians to degrees
% theta_deg = rad2deg(theta);
% eta_deg = rad2deg(eta);
% zeta_deg = rad2deg(zeta);
% phi_deg = rad2deg(phi);
% angles = [theta_deg, eta_deg, zeta_deg, phi_deg]'; % Store in degrees
% 
% % Loading the trajectory data
% trajPos = data.data.part(2).trajPos{35}; % Example for participant 1, trial 1
% number_of_steps = size(trajPos, 1);
% 
% % Matrix to store the computed joint angles at each step
% joint_angles_over_time = zeros(number_of_steps, 4);
% 
% % Store initial angles in the first row of joint_angles_over_time
% joint_angles_over_time(1, :) = angles';
% 
% for i = 2:number_of_steps
%     old_pos = trajPos(i-1, :);           % Previous and current step positions
%     new_pos = trajPos(i, :);
%     delta_hand = new_pos - old_pos;  % Calculate the delta hand position
% 
%     % Compute the Jacobian matrix at the current angles
%     jh = jacobian_test(lu, lu, angles(1), angles(2), angles(3), angles(4));
%     inverse_jh = pinv(jh);
% 
%     % Change in angles
%     delta_angles = inverse_jh * delta_hand';
% 
%     % Update the angles
%     angles = angles + rad2deg(delta_angles); % Ensure angles are updated in degrees
% 
%     % Store the angles at this step
%     joint_angles_over_time(i, :) = angles';
% 
%     % Print the joint angles for this step in degrees
%     fprintf('Step %d:\n', i);
%     fprintf('Theta (Shoulder Elevation): %.2f degrees\n', angles(1));
%     fprintf('Eta (Shoulder Azimuth): %.2f degrees\n', angles(2));
%     fprintf('Zeta (Humeral Angle): %.2f degrees\n', angles(3));
%     fprintf('Phi (Elbow Flexion): %.2f degrees\n', angles(4));
%     fprintf('------------------------\n');
% end
% 
% % Plot joint angles over time in degrees
% figure;
% plot(joint_angles_over_time);
% xlabel('Step');
% ylabel('Angle (degrees)');
% legend('Theta (Shoulder Elevation)', 'Eta (Shoulder Azimuth)', 'Zeta (Humeral Angle)', 'Phi (Elbow Flexion)');
% title('Joint Angles Over Time');
% grid on;
% 
% % Display the results in degrees
% fprintf('Calculated Joint Angles in Degrees:\n');
% fprintf('Theta (Shoulder Elevation): %.2f degrees\n', theta_deg);
% fprintf('Eta (Shoulder Azimuth): %.2f degrees\n', eta_deg);
% fprintf('Zeta (Humeral Angle): %.2f degrees\n', zeta_deg);
% fprintf('Phi (Elbow Flexion): %.2f degrees\n', phi_deg);
% 