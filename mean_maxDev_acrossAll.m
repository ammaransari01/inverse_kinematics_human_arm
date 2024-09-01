function mean_maxDev_acrossAll(data)
    % Number of participants
    numParticipants = length(data.data.part);
    % Number of targets
    numTargets = 3;
    
    % Preallocate cell arrays to store maxDev values for each target
    maxDevAllTargets = cell(numTargets, 1);
    
    % Collect maxDev values for each target
    for targetPos = 1:numTargets
        allMaxDev = [];
        for p = 1:numParticipants
            % Collect maxDev for the specific target
            maxDevForTarget = collect_maxDev(data, p, targetPos);
            if ~isempty(maxDevForTarget)
                allMaxDev = [allMaxDev; maxDevForTarget];
            end
        end
        % Store all maxDev values for the target
        maxDevAllTargets{targetPos} = allMaxDev;
    end

    % Interpolate and average maxDev for each target
    maxDevAverages = cell(numTargets, 1);
    timePoints = 1:100; % Define the number of time points for interpolation

    for targetPos = 1:numTargets
        maxDevForTarget = maxDevAllTargets{targetPos};
        
        % Interpolate maxDev values to a common time vector
        interpolatedMaxDev = zeros(length(timePoints), size(maxDevForTarget, 2));
        for i = 1:size(maxDevForTarget, 2)
            % Interpolate each column of maxDev data
            interpolatedMaxDev(:, i) = interp1(linspace(1, size(maxDevForTarget, 1), size(maxDevForTarget, 1)), ...
                maxDevForTarget(:, i), timePoints, 'pchip', 'extrap');
        end
        
        % Compute average and smooth the interpolated maxDev
        avgMaxDev = mean(interpolatedMaxDev, 2);
        smoothedAvgMaxDev = smooth(avgMaxDev, 0.2, 'rloess'); % Smooth the data
        maxDevAverages{targetPos} = smoothedAvgMaxDev;
    end

    % Plot the averaged maxDev for each target
    figure;
    colors = ['r', 'g', 'b']; % Colors for different targets
    hold on;
    
    for targetPos = 1:numTargets
        plot(timePoints, maxDevAverages{targetPos}, 'Color', colors(targetPos), 'LineWidth', 1.5, ...
            'DisplayName', ['Target ', num2str(targetPos)]);
    end
    
    xlabel('Time Step');
    ylabel('Mean maxDev');
    title('Mean maxDev across all trials of all participants for Each Target');
    legend('show');
    grid on;
    hold off;
    
    function maxDevForTarget = collect_maxDev(data, p, targetPos)
        numTrials = length(data.data.part(p).trajPos);
        maxDevForTarget = [];

        % Iterate over trials to collect maxDev values by target position
        for t = 1:numTrials
            if data.data.part(p).iv(t).position == targetPos
                maxDev = data.data.part(p).dv(t).maxDev;
                maxDevForTarget = [maxDevForTarget; maxDev];
            end
        end
    end
end