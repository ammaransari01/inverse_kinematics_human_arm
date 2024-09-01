% Load data and perform model fitting
trajData = meanTrajData;  % Use actual mean trajectory data here
[vTheta, vEta, vOmicron, vPhi, sTheta, sEta, sOmicron, sPhi] = fitTrajectory('calcError', 'calcForwardKinematic', trajData);
% Plot the fitted model trajectory vs data trajectory
figure;
plotTrajectory(size(trajData, 1), vTheta, vEta, vOmicron, vPhi, sTheta, sEta, sOmicron, sPhi, trajData);
