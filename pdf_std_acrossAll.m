function pdf_std_acrossAll(data)

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

% Plotting the bell curves
figure;
hold on;
colors = ['r', 'g', 'b']; % Colors for different targets

for targetPos = 1:numTargets
    % Get maxDev values for the current target
    maxDevValues = allMaxDevByTarget{targetPos};
    
    % Calculate mean and standard deviation
    mu = mean(maxDevValues);
    sigma = std(maxDevValues);
    
    % Create a range of values for x-axis
    x = linspace(mu - 4*sigma, mu + 4*sigma, 100);
    
    % Calculate the PDF (probability density function)
    y = (1/(sigma * sqrt(2 * pi))) * exp(-(x - mu).^2 / (2 * sigma^2));
    
    % Plot the bell curve
    plot(x, y, 'Color', colors(targetPos), 'LineWidth', 2);
end

% Add labels, legend, and title
xlabel('MaxDev');
ylabel('Probability Density');
title('Bell Curve of MaxDev for Each Target Across All Participants');
legend('Target 1', 'Target 2', 'Target 3');
grid on;
hold off;

end