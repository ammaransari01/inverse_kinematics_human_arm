%% NOTE - EXECUTION TIME APPROXIMATELY 20 MINS - 
%% THE CODE PLOTS SUB-PROFILES FOR EACH TRIAL, EACH TARGET FOR ALL PARTICIPANTS

%% FIRST RUN THE FILE 'meanTrajectory_analysis_joint_angle_thresholds_and_profiles.m'
%% which Analyzes the Mean Trajectory - Joint Angles Data to understand what the code does

%% FOR DETAILED AND TRIAL SPECIFIC ANALYSIS RUN THIS FILE (EXECUTION TIME <20 mins DEPENDING ON COMPUTER SPECS)

function allTrialsTargets_analysis_joint_angle_thresholds_and_profiles(data)
    % Number of participants
    numParticipants = length(data.data.part);
    % Number of targets
    numTargets = 3;

    % Define limb lengths (in mm)
    upperarmLength = 320; % Upper arm length in mm
    forearmLength = 480; % Forearm length in mm

    % Iterate over participants and trials to collect and analyze joint angles
    for p = 1:numParticipants
        numTrials = length(data.data.part(p).dv);
        for t = 1:numTrials
            % Get the target position for the current trial
            targetPos = data.data.part(p).iv(t).position;

            % Calculate joint angles for the current trajectory
            trajectory = data.data.part(p).dv(t).trajectory;
            numPoints = size(trajectory, 1);
            jointAnglesOverTime = zeros(numPoints, 4);

            % Vectorized calculation of joint angles
            for i = 1:numPoints
                % Hand position at the current trajectory point
                xHand = trajectory(i, 1);
                yHand = trajectory(i, 2);
                zHand = trajectory(i, 3);

                % Calculate joint angles using inverse kinematics
                jointAnglesOverTime(i, :) = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength);
            end

            % Calculate the first, second, and third derivatives (velocity, acceleration, jerk)
            jointVelocities = diff(jointAnglesOverTime);
            jointAccelerations = diff(jointVelocities);
            jointJerks = diff(jointAccelerations);

            % Identify significant changes in joint angle velocities
            threshold = 0.1; % Adjustable threshold value
            significantIndices = find(any(abs(jointVelocities) > threshold, 2)) + 1;

            % Plot joint angles and their derivatives
            figure;
            subplot(4, 1, 1);
            plot(rad2deg(jointAnglesOverTime));
            xlabel('Trajectory Point');
            ylabel('Angle (degrees)');
            title(['Joint Angles for Trial ', num2str(t), ' Target Position ', num2str(targetPos)]);
            grid on;

            subplot(4, 1, 2);
            plot(rad2deg(jointVelocities));
            hold on;
            plot(significantIndices, rad2deg(jointVelocities(significantIndices, :)), 'ro');
            xlabel('Trajectory Point');
            ylabel('Velocity (degrees/point)');
            title('Joint Angle Velocities');
            grid on;

            subplot(4, 1, 3);
            plot(rad2deg(jointAccelerations));
            xlabel('Trajectory Point');
            ylabel('Acceleration (degrees/point^2)');
            title('Joint Angle Accelerations');
            grid on;

            subplot(4, 1, 4);
            plot(rad2deg(jointJerks));
            xlabel('Trajectory Point');
            ylabel('Jerk (degrees/point^3)');
            title('Joint Angle Jerks');
            grid on;

            % Highlight significant threshold points on the jerk profile
            figure;
            plot(rad2deg(jointJerks));
            hold on;
            plot(significantIndices - 2, rad2deg(jointJerks(significantIndices - 2, :)), 'ro'); % Adjust index for jerk
            xlabel('Trajectory Point');
            ylabel('Jerk (degrees/point^3)');
            title(['Jerk Profile with Threshold Points for Trial ', num2str(t), ' Target Position ', num2str(targetPos)]);
            grid on;
        end
    end
end

function jointAngles = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength)
    % Inverse kinematics calculations
    % Assume initial angles are 0 for simplicity and assume hand is in the xy plane
    % theta = shoulder elevation, eta = shoulder azimuth, zeta = humeral rotation, phi = elbow flexion

    % Elbow position relative to the shoulder
    elbowDist = sqrt(xHand^2 + yHand^2 + zHand^2);
    phi = acos((upperarmLength^2 + forearmLength^2 - elbowDist^2) / (2 * upperarmLength * forearmLength));
    
    % Shoulder elevation angle (theta)
    theta = acos(-zHand / upperarmLength);

    % Shoulder azimuth angle (eta)
    eta = atan2(yHand, xHand);

    % Humeral rotation angle (zeta)
    zeta = atan2(yHand, xHand);

    % Combine angles into a single output vector
    jointAngles = [theta, eta, zeta, phi];
end
