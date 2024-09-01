function participant_mean_maxDev(data)
    % Number of participants
    numParticipants = length(data.data.part);
    
    % Initialize arrays to store average maxDev for each participant
    avgMaxDev = zeros(numParticipants, 1);
    
    % Iterate over each participant
    for p = 1:numParticipants
        % Collect maxDev values for the current participant
        maxDevValues = [];
        numTrials = length(data.data.part(p).trajPos);
        
        for t = 1:numTrials
            maxDev = data.data.part(p).dv(t).maxDev;
            maxDevValues = [maxDevValues; maxDev];
        end
        
        % Compute the average maxDev for this participant
        avgMaxDev(p) = mean(maxDevValues, 'omitnan'); % Use 'omitnan' to handle any NaNs
    end
    
    % Create a figure for the plot
    figure('Name', 'Average maxDev for Each Participant', 'NumberTitle', 'off');
    bar(avgMaxDev, 'FaceColor', [0.2 0.6 1]);
    xlabel('Participant');
    ylabel('Average maxDev');
    title('Average maxDev for Each Participant');
    grid on;
    
    % Display participant numbers on the x-axis
    xticks(1:numParticipants);
    xticklabels(arrayfun(@(x) sprintf('P%d', x), 1:numParticipants, 'UniformOutput', false));
end
