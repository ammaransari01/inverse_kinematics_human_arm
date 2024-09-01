%% Data Analysis of Arm Motion Data from Qualisys ProReflex MCU240, 200Hz

%% 3D Plot of Trajectories for All Participants
figure;
hold on;

% Loop through participants
for p = 1:length(data.data.part)
    % Loop through trials for each participant
    for t = 60; %:length(data.data.part(p).dv)
        % Plot the trajectory for the current trial
        traj = data.data.part(p).dv(t).trajectory;
        plot3(traj(:,1), traj(:,2), traj(:,3),'LineWidth',1.5);
    end
end

xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
title('3D Trajectories for All Participants for Trial Number ', num2str(t));
legend(arrayfun(@(x) ['Participant ', num2str(x)], 1:length(data.data.part), 'UniformOutput', false));
grid on;
hold off;


%% Initiation Latency Analysis

figure;
hold on;
for p = 1:length(data.data.part)
    initLatency = [data.data.part(p).dv.il];
    plot(initLatency);
end
xlabel('Trial Number');
ylabel('Initiation Latency (ms)');
title('Initiation Latency Across Trials for All Participants');
legend(arrayfun(@(x) ['Participant ', num2str(x)], 1:length(data.data.part), 'UniformOutput', false));
grid on;
hold off;

%% Initialization
numParticipants = length(data.data.part);
numTargets = 3; % Assuming there are 3 target positions

% Preallocate storage for velocity, acceleration, and jerk profiles
velocityProfiles = cell(numParticipants, numTargets);
accelerationProfiles = cell(numParticipants, numTargets);
jerkProfiles = cell(numParticipants, numTargets);

%% Loop through each participant and each target position
for p = 1:numParticipants
    for t = 1:length(data.data.part(p).dv) % change value for plotting for specific trial
        % Extract relevant data for the current trial
        traj = data.data.part(p).dv(t).trajectory;
        startTime = data.data.part(p).dv(t).startTime;
        endTime = data.data.part(p).dv(t).endTime;
        movePeriod = data.data.part(p).dv(t).movePeriod;
        
        % Time vector based on start and end times
        timeVector = linspace(startTime, endTime, size(traj, 1));

        % Compute the velocity profile
        velocity = diff(traj) ./ diff(timeVector'); % Calculate velocity
        velocityMagnitude = sqrt(sum(velocity.^2, 2));

        % Compute the acceleration profile
        acceleration = diff(velocity) ./ diff(timeVector(1:end-1)'); % Calculate acceleration
        accelerationMagnitude = sqrt(sum(acceleration.^2, 2));

        % Compute the jerk profile
        jerk = diff(acceleration) ./ diff(timeVector(1:end-2)'); % Calculate jerk
        jerkMagnitude = sqrt(sum(jerk.^2, 2));

        % Store the results in their respective cells
        targetPos = data.data.part(p).iv(t).position;
        velocityProfiles{p, targetPos} = [velocityProfiles{p, targetPos}; velocityMagnitude'];
        accelerationProfiles{p, targetPos} = [accelerationProfiles{p, targetPos}; accelerationMagnitude'];
        jerkProfiles{p, targetPos} = [jerkProfiles{p, targetPos}; jerkMagnitude'];
    end
end

%% Plotting the Profiles for each Target Position

for targetPos = 1:numTargets
    figure;
    sgtitle(['Velocity, Acceleration, and Jerk Profiles for Target Position ', num2str(targetPos)]);

    % Velocity Profile Plot
    subplot(3, 1, 1);
    hold on;
    for p = 1:numParticipants
        if ~isempty(velocityProfiles{p, targetPos})
            % Plot the mean velocity profile across trials for this participant and target position
            plot(mean(velocityProfiles{p, targetPos}, 1), 'LineWidth', 1.5, 'DisplayName', ['Participant ', num2str(p)]);
        end
    end
    title('Velocity Profile');
    xlabel('Time Steps');
    ylabel('Velocity (mm/s)');
    legend('show');
    grid on;

    % Acceleration Profile Plot
    subplot(3, 1, 2);
    hold on;
    for p = 1:numParticipants
        if ~isempty(accelerationProfiles{p, targetPos})
            % Plot the mean acceleration profile across trials for this participant and target position
            plot(mean(accelerationProfiles{p, targetPos}, 1), 'LineWidth', 1.5, 'DisplayName', ['Participant ', num2str(p)]);
        end
    end
    title('Acceleration Profile');
    xlabel('Time Steps');
    ylabel('Acceleration (mm/s^2)');
    legend('show');
    grid on;

    % Jerk Profile Plot
    subplot(3, 1, 3);
    hold on;
    for p = 1:numParticipants
        if ~isempty(jerkProfiles{p, targetPos})
            % Plot the mean jerk profile across trials for this participant and target position
            plot(mean(jerkProfiles{p, targetPos}, 1), 'LineWidth', 1.5, 'DisplayName', ['Participant ', num2str(p)]);
        end
    end
    title('Jerk Profile');
    xlabel('Time Steps');
    ylabel('Jerk (mm/s^3)');
    legend('show');
    grid on;
end