% Extract trajectory data for a specific participant and trial
participant = 1; % Change this to select a different participant
trial = 1; % Change this to select a different trial
trajPos = data.data.part(participant).trajPos{trial}; % Extract the trajectory data

% Preallocate a matrix to store the joint angles
numPoints = size(trajPos, 1);
jointAngles = zeros(numPoints, 4); % 4 columns for [Theta, Eta, Zeta, Phi]

% Define the arm segment lengths
upperarmLength = 32; % Upper arm length in cm
forearmLength = 45; % Forearm length in cm

% Define the alpha parameter
alpha = 0; % Example value (can be adjusted if needed)

% Define the hand position
xHand = 10; % value in cm
yHand = 10; % value in cm
zHand = 10; % value in cm

% Define the arm segment lengths
upperarmLength = 32; % Upper arm length in cm
forearmLength = 45; % Forearm length in cm

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
    theta = max(0, min(theta, pi));
    eta = max(-3*pi/4, min(eta, pi/3));
    phi = max(0, min(phi, pi));
end

% Call the inverse kinematics function
[theta, eta, zeta, phi] = calcInverseKinematics(xHand, yHand, zHand, forearmLength, upperarmLength, alpha);

% Convert the joint angles from radians to degrees
theta_deg = rad2deg(theta);
eta_deg = rad2deg(eta);
zeta_deg = rad2deg(zeta);
phi_deg = rad2deg(phi);

% Display the results in degrees
fprintf('Calculated Joint Angles in Degrees:\n');
fprintf('Theta (Shoulder Elevation): %.2f degrees\n', theta_deg);
fprintf('Eta (Shoulder Azimuth): %.2f degrees\n', eta_deg);
fprintf('Zeta (Humeral Angle): %.2f degrees\n', zeta_deg);
fprintf('Phi (Elbow Flexion): %.2f degrees\n', phi_deg);

% Code Explanation:
% Distance Calculation: The distance r from the shoulder to the hand is calculated using the Euclidean norm of the hand position.
% Elbow Flexion Angle (ϕ): The angle ϕ is calculated using the law of cosines, considering the triangle formed by the shoulder, elbow, and hand.Elbow Position Calculation: The elbow position is calculated using the parameter αα, which resolves the redundancy in the configuration space by allowing the elbow to rotate around the axis from the shoulder to the hand.
% Joint Angle Calculation: The angles θ, η, and ζ are calculated based on the geometric relationships described in the paper.
% Joint Limits: The calculated angles are constrained to be within biomechanically feasible limits.
% The parameter α is used to handle the redundancy in the arm's configuration space, allowing for multiple valid elbow positions for a given hand position.