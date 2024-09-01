function mean_std_maxDev(data)

% Number of participants
numParticipants = length(data.data.part);
% Number of targets
numTargets = 3;

% Preallocate a cell array to store maxDev data for each target and participant
allMaxDevByTarget = cell(numTargets, 1);

% Iterate over participants and trials to collect maxDev values organized by target position
for p = 1:numParticipants
    numTrials = length(data.data.part(p).iv); % Assuming each participant has the same number of trials

    for t = 1:numTrials
        % Get the target position for the current trial
        targetPos = data.data.part(p).iv(t).position;
        
        % Get the maxDev value for the current trial
        maxDevValue = data.data.part(p).dv(t).maxDev;
        
        % Store the maxDev value corresponding to the target position
        if isempty(allMaxDevByTarget{targetPos})
            allMaxDevByTarget{targetPos} = maxDevValue;
        else
            allMaxDevByTarget{targetPos} = [allMaxDevByTarget{targetPos}; maxDevValue];
        end
    end
end

% Initialize a matrix to store the mean maxDev and standard deviation of maxDev for each target
meanMaxDev = zeros(numTargets, 1);
stdMaxDev = zeros(numTargets, 1);

% Calculate mean and standard deviation for each target
for targetPos = 1:numTargets
    meanMaxDev(targetPos) = mean(allMaxDevByTarget{targetPos});
    stdMaxDev(targetPos) = std(allMaxDevByTarget{targetPos});
end

% Plotting
figure;
hold on;
for targetPos = 1:numTargets
    % Plot the mean maxDev with error bars showing the standard deviation
    errorbar(targetPos, meanMaxDev(targetPos), stdMaxDev(targetPos), 'o-', 'LineWidth', 2);
end
hold off;

% Add labels, legend, and title
xlabel('Target');
ylabel('MaxDev (Standard Deviation)');
title('Mean and Standard Deviation of MaxDev for Each Target Across All Participants');
xticks(1:numTargets);
legend('Target 1', 'Target 2', 'Target 3');
grid on;

end