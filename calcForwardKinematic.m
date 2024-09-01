%% Function to calculate forward kinematics of a human arm
function [xHand, yHand, zHand] = calcForwardKinematic(theta, eta, omicron, phi, forearmLength, upperarmLength)
    % forward kinematic according to A. Biess; D. G. Liebermann and T. Flash (2007)
    % The name of the angles follow page 13047.

    % Calculate the elbow position
    xElbow = -upperarmLength * sin(eta) * sin(theta);
    yElbow = -upperarmLength * cos(eta) * sin(theta);
    zElbow = -upperarmLength * cos(theta);

    % Calculate the hand position
    xHand = xElbow - forearmLength * (sin(phi) * (cos(omicron) * sin(eta) * cos(theta) + sin(omicron) * cos(eta)) + cos(phi) * sin(eta) * sin(theta));
    yHand = yElbow + forearmLength * (sin(phi) * (cos(omicron) * cos(eta) * cos(theta) + sin(omicron) * sin(eta)) + cos(phi) * cos(eta) * sin(theta));
    zHand = zElbow + forearmLength * (sin(phi) * cos(omicron) * cos(eta) * sin(theta) - cos(phi) * cos(theta));
end