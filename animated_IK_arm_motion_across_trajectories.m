function animated_IK_arm_motion_across_trajectories(data)
    % Arm lengths (in mm)
    upperarmLength = 320;
    forearmLength = 480;

    % Define time step for animation
    timeStep = 0.05; % seconds

    % Number of participants
    numParticipants = length(data.data.part);

    % Iterate over participants and trials to animate trajectories
    for p = 1:numParticipants
        numTrials = length(data.data.part(p).dv);
        for t = 1:numTrials
            % Get the target position for the current trial
            targetPos = data.data.part(p).iv(t).position;
            % Get the trajectory for the current trial
            trajectory = data.data.part(p).dv(t).trajectory;

            % Interpolate trajectory to ensure smooth animation
            numPoints = 100; % Adjust this value as needed
            interpolatedTrajectory = interp1(1:size(trajectory,1), trajectory, linspace(1, size(trajectory,1), numPoints));

            % Initialize the plot
            figure;
            hold on;
            grid on;
            axis equal;
            xlabel('X (mm)');
            ylabel('Y (mm)');
            zlabel('Z (mm)');
            title(['Arm Motion Animation for Participant ', num2str(p), ', Target Position ', num2str(targetPos)]);

            % Set the view to an oblique perspective
            view(45, 30);

            % Plot the initial trajectory for reference
            plot3(interpolatedTrajectory(:,1), interpolatedTrajectory(:,2), interpolatedTrajectory(:,3), 'r--', 'DisplayName', 'Trajectory');

            % Initialize the arm plot with colors and markers
            armPlotUpper = plot3([0, 0], [0, 0], [0, 0], 'b-', 'LineWidth', 2, 'DisplayName', 'Upper Arm');
            armPlotFore = plot3([0, 0], [0, 0], [0, 0], 'g-', 'LineWidth', 2, 'DisplayName', 'Forearm');
            shoulderPlot = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Shoulder');
            elbowPlot = plot3(0, 0, 0, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Elbow');
            handPlot = plot3(0, 0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Hand');

            % Initialize joint angle and velocity storage
            jointAngles = zeros(numPoints, 3); % Only store Theta, Eta, and Phi
            jointVelocities = zeros(numPoints, 3);

            % Initial joint angles
            initialAngles = [0; 0; 0; 0];

            % Set initial shoulder position to be above the elbow
            shoulderPos = [0, 0, 600]; % Adjust to reflect sitting posture with hand on a table

            % Animate the trajectory
            for i = 1:numPoints
                % Current hand (TCP) position
                xHand = interpolatedTrajectory(i, 1);
                yHand = interpolatedTrajectory(i, 2);
                zHand = interpolatedTrajectory(i, 3);

                % Calculate inverse kinematics
                [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, initialAngles);

                % Calculate joint angles
                jointAngles(i, :) = [theta, eta, phi]; % Exclude Zeta

                % Calculate the elbow position using forward kinematics
                [xElbow, yElbow, zElbow] = forwardKinematics(theta, eta, upperarmLength);

                % Update arm plot (from shoulder to elbow to hand) with colors
                set(armPlotUpper, 'XData', [shoulderPos(1), xElbow], ...
                                  'YData', [shoulderPos(2), yElbow], ...
                                  'ZData', [shoulderPos(3), zElbow]);
                set(armPlotFore, 'XData', [xElbow, xHand], ...
                                 'YData', [yElbow, yHand], ...
                                 'ZData', [zElbow, zHand]);

                % Update joint markers
                set(shoulderPlot, 'XData', shoulderPos(1), ...
                                  'YData', shoulderPos(2), ...
                                  'ZData', shoulderPos(3));
                set(elbowPlot, 'XData', xElbow, ...
                               'YData', yElbow, ...
                               'ZData', zElbow);
                set(handPlot, 'XData', xHand, ...
                              'YData', yHand, ...
                              'ZData', zHand);

                % Pause for animation timing
                pause(timeStep);

                % Calculate joint velocities
                if i > 1
                    jointVelocities(i, :) = (jointAngles(i, :) - jointAngles(i-1, :)) / timeStep;
                end
            end

            % Add legend to the plot
            legend('show');

            % Plot joint angles over time
            figure;
            subplot(2, 1, 1);
            plot(linspace(0, timeStep*(numPoints-1), numPoints), rad2deg(jointAngles));
            xlabel('Time (s)');
            ylabel('Joint Angles (degrees)');
            legend('Theta (Shoulder Elevation)', 'Eta (Shoulder Azimuth)', 'Phi (Elbow Flexion)');
            title('Joint Angles Over Time');
            grid on;

            % Plot joint velocities over time
            subplot(2, 1, 2);
            plot(linspace(0, timeStep*(numPoints-1), numPoints), rad2deg(jointVelocities));
            xlabel('Time (s)');
            ylabel('Joint Velocities (degrees/s)');
            legend('Theta Velocity', 'Eta Velocity', 'Phi Velocity');
            title('Joint Velocities Over Time');
            grid on;
        end
    end
end

function [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, initialAngles)
    % Inverse kinematics calculations to find joint angles

    % Calculate the shoulder elevation angle (theta)
    r = sqrt(xHand^2 + yHand^2);
    s = sqrt(r^2 + zHand^2);
    theta = acos((upperarmLength^2 + s^2 - forearmLength^2) / (2 * upperarmLength * s)) + atan2(zHand, r);

    % Calculate the shoulder azimuth angle (eta)
    eta = atan2(yHand, xHand);

    % Calculate the elbow flexion angle (phi)
    phi = acos((upperarmLength^2 + forearmLength^2 - s^2) / (2 * upperarmLength * forearmLength));

    % Calculate the humeral rotation angle (zeta)
    zeta = atan2(-xHand * sin(eta) + yHand * cos(eta), zHand);
end

function [xElbow, yElbow, zElbow] = forwardKinematics(theta, eta, upperarmLength)
    % Forward kinematics to calculate elbow position

    % Elbow joint position
    xElbow = upperarmLength * cos(theta) * cos(eta);
    yElbow = upperarmLength * cos(theta) * sin(eta);
    zElbow = upperarmLength * sin(theta);
end


%% %% Without Joint Markers and Legend
% function animated_IK_arm_motion_across_trajectories(data)
%     % Arm lengths (in mm)
%     upperarmLength = 320;
%     forearmLength = 480;
% 
%     % Define time step for animation
%     timeStep = 0.05; % seconds
% 
%     % Number of participants
%     numParticipants = length(data.data.part);
% 
%     % Iterate over participants and trials to animate trajectories
%     for p = 1:numParticipants
%         numTrials = length(data.data.part(p).dv);
%         for t = 1:numTrials
%             % Get the target position for the current trial
%             targetPos = data.data.part(p).iv(t).position;
%             % Get the trajectory for the current trial
%             trajectory = data.data.part(p).dv(t).trajectory;
% 
%             % Interpolate trajectory to ensure smooth animation
%             numPoints = 100; % Adjust this value as needed
%             interpolatedTrajectory = interp1(1:size(trajectory,1), trajectory, linspace(1, size(trajectory,1), numPoints));
% 
%             % Initialize the plot
%             figure;
%             hold on;
%             grid on;
%             axis equal;
%             xlabel('X (mm)');
%             ylabel('Y (mm)');
%             zlabel('Z (mm)');
%             title(['Arm Motion Animation for Participant ', num2str(p), ', Target Position ', num2str(targetPos)]);
% 
%             % Set the view to an oblique perspective
%             view(45, 30);
% 
%             % Plot the initial trajectory for reference
%             plot3(interpolatedTrajectory(:,1), interpolatedTrajectory(:,2), interpolatedTrajectory(:,3), 'r--', 'DisplayName', 'Trajectory');
% 
%             % Initialize the arm plot with colors and markers
%             armPlotUpper = plot3([0, 0], [0, 0], [0, 0], 'LineWidth', 2, 'DisplayName', 'Upper Arm');
%             armPlotFore = plot3([0, 0], [0, 0], [0, 0], 'LineWidth', 2, 'DisplayName', 'Forearm');
%             shoulderPlot = plot3(0, 0, 0, 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Shoulder');
%             elbowPlot = plot3(0, 0, 0, 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Elbow');
%             handPlot = plot3(0, 0, 0, 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Hand');
% 
%             % Initialize joint angle and velocity storage
%             jointAngles = zeros(numPoints, 3); % Only store Theta, Eta, and Phi
%             jointVelocities = zeros(numPoints, 3);
% 
%             % Initial joint angles
%             initialAngles = [0; 0; 0; 0];
% 
%             % Set initial shoulder position to be above the elbow
%             shoulderPos = [0, 0, 600]; % Adjust to reflect sitting posture with hand on a table
% 
%             % Animate the trajectory
%             for i = 1:numPoints
%                 % Current hand (TCP) position
%                 xHand = interpolatedTrajectory(i, 1);
%                 yHand = interpolatedTrajectory(i, 2);
%                 zHand = interpolatedTrajectory(i, 3);
% 
%                 % Calculate inverse kinematics
%                 [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, initialAngles);
% 
%                 % Calculate joint angles
%                 jointAngles(i, :) = [theta, eta, phi]; % Exclude Zeta
% 
%                 % Calculate the elbow position using forward kinematics
%                 [xElbow, yElbow, zElbow] = forwardKinematics(theta, eta, upperarmLength);
% 
%                 % Update arm plot (from shoulder to elbow to hand) with colors
%                 set(armPlotUpper, 'XData', [shoulderPos(1), xElbow], ...
%                                   'YData', [shoulderPos(2), yElbow], ...
%                                   'ZData', [shoulderPos(3), zElbow]);
%                 set(armPlotFore, 'XData', [xElbow, xHand], ...
%                                  'YData', [yElbow, yHand], ...
%                                  'ZData', [zElbow, zHand]);
% 
%                 % Update joint markers
%                 set(shoulderPlot, 'XData', shoulderPos(1), ...
%                                   'YData', shoulderPos(2), ...
%                                   'ZData', shoulderPos(3));
%                 set(elbowPlot, 'XData', xElbow, ...
%                                'YData', yElbow, ...
%                                'ZData', zElbow);
%                 set(handPlot, 'XData', xHand, ...
%                               'YData', yHand, ...
%                               'ZData', zHand);
% 
%                 % Pause for animation timing
%                 pause(timeStep);
% 
%                 % Calculate joint velocities
%                 if i > 1
%                     jointVelocities(i, :) = (jointAngles(i, :) - jointAngles(i-1, :)) / timeStep;
%                 end
%             end
% 
%             % Plot joint angles over time
%             figure;
%             subplot(2, 1, 1);
%             plot(linspace(0, timeStep*(numPoints-1), numPoints), rad2deg(jointAngles));
%             xlabel('Time (s)');
%             ylabel('Joint Angles (degrees)');
%             title('Joint Angles Over Time');
%             grid on;
% 
%             % Plot joint velocities over time
%             subplot(2, 1, 2);
%             plot(linspace(0, timeStep*(numPoints-1), numPoints), rad2deg(jointVelocities));
%             xlabel('Time (s)');
%             ylabel('Joint Velocities (degrees/s)');
%             title('Joint Velocities Over Time');
%             grid on;
%         end
%     end
% end
% 
% function [theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, initialAngles)
%     % Inverse kinematics calculations to find joint angles
% 
%     % Calculate the shoulder elevation angle (theta)
%     r = sqrt(xHand^2 + yHand^2);
%     s = sqrt(r^2 + zHand^2);
%     theta = acos((upperarmLength^2 + s^2 - forearmLength^2) / (2 * upperarmLength * s)) + atan2(zHand, r);
% 
%     % Calculate the shoulder azimuth angle (eta)
%     eta = atan2(yHand, xHand);
% 
%     % Calculate the elbow flexion angle (phi)
%     phi = acos((upperarmLength^2 + forearmLength^2 - s^2) / (2 * upperarmLength * forearmLength));
% 
%     % Calculate the humeral rotation angle (zeta)
%     zeta = atan2(-xHand * sin(eta) + yHand * cos(eta), zHand);
% end
% 
% function [xElbow, yElbow, zElbow] = forwardKinematics(theta, eta, upperarmLength)
%     % Forward kinematics to calculate elbow position
% 
%     % Elbow joint position
%     xElbow = upperarmLength * cos(theta) * cos(eta);
%     yElbow = upperarmLength * cos(theta) * sin(eta);
%     zElbow = upperarmLength * sin(theta);
% end