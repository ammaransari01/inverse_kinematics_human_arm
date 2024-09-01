function human_arm_kinematics()
    % Define limb lengths
    upperarmLength = 32; % cm
    forearmLength = 45; % cm

    % Define joint angles (initial guess)
    theta = pi/4; % elevation angle
    eta = pi/6; % azimuthal angle
    zeta = pi/3; % humeral angle
    phi = pi/4; % flexion angle

    % Calculate elbow and hand positions
    [xElbow, yElbow, zElbow] = elbow_position(upperarmLength, theta, eta);
    [xHand, yHand, zHand] = hand_position(xElbow, yElbow, zElbow, forearmLength, theta, eta, zeta, phi);

    % Display results
    fprintf('Elbow Position: (%.2f, %.2f, %.2f)\n', xElbow, yElbow, zElbow);
    fprintf('Hand Position: (%.2f, %.2f, %.2f)\n', xHand, yHand, zHand);

    % Calculate Jacobians
    Je = elbow_jacobian(upperarmLength, theta, eta);
    Jh = hand_jacobian(upperarmLength, forearmLength, theta, eta, zeta, phi);

    % Display Jacobians
    disp('Elbow Jacobian:');
    disp(Je);
    disp('Hand Jacobian:');
    disp(Jh);

    % Estimate joint angles based on hand position
    [theta_est, eta_est, zeta_est, phi_est] = inverse_kinematics(upperarmLength, forearmLength, xHand, yHand, zHand);

    % Display estimated joint angles
    fprintf('Estimated Joint Angles:\n');
    fprintf('Theta: %.2f\n', theta_est);
    fprintf('Eta: %.2f\n', eta_est);
    fprintf('Zeta: %.2f\n', zeta_est);
    fprintf('Phi: %.2f\n', phi_est);
end

function [xElbow, yElbow, zElbow] = elbow_position(lu, theta, eta)
    xElbow = -lu * sin(eta) * sin(theta);
    yElbow = lu * cos(eta) * sin(theta);
    zElbow = -lu * cos(theta);
end

function [xHand, yHand, zHand] = hand_position(xElbow, yElbow, zElbow, lf, theta, eta, zeta, phi)
    xHand = xElbow - lf * (sin(phi) * (cos(zeta) * sin(eta) * cos(theta) + sin(zeta) * cos(eta)) + cos(phi) * sin(eta) * sin(theta));
    yHand = yElbow + lf * (sin(phi) * (cos(zeta) * cos(eta) * cos(theta) - sin(zeta) * sin(eta)) + cos(phi) * cos(eta) * sin(theta));
    zHand = zElbow + lf * (sin(phi) * cos(zeta) * sin(theta) - cos(phi) * cos(theta));
end

function Je = elbow_jacobian(lu, theta, eta)
    Je = [
        -lu * sin(eta) * cos(theta), -lu * cos(eta) * sin(theta), 0, 0;
        lu * cos(eta) * cos(theta), -lu * sin(eta) * sin(theta), 0, 0;
        lu * sin(theta), 0, 0, 0
    ];
end

function Jh = hand_jacobian(lu, lf, theta, eta, zeta, phi)
    % Elbow components
    dxe_dtheta = -lu * sin(eta) * cos(theta);
    dxe_deta = -lu * cos(eta) * sin(theta);
    dye_dtheta = lu * cos(eta) * cos(theta);
    dye_deta = -lu * sin(eta) * sin(theta);
    dze_dtheta = lu * sin(theta);

    % Hand components
    dxh_dtheta = dxe_dtheta - lf * (sin(phi) * cos(zeta) * sin(eta) * (-sin(theta)) + cos(phi) * sin(eta) * cos(theta));
    dxh_deta = dxe_deta - lf * (sin(phi) * (cos(zeta) * cos(eta) * cos(theta) - sin(zeta) * sin(eta)) + cos(phi) * cos(eta) * sin(theta));
    dxh_dzeta = -lf * sin(phi) * (-sin(zeta) * sin(eta) * cos(theta) + cos(zeta) * cos(eta));
    dxh_dphi = -lf * (cos(phi) * (cos(zeta) * sin(eta) * cos(theta) + sin(zeta) * cos(eta)) - sin(phi) * sin(eta) * sin(theta));

    dyh_dtheta = dye_dtheta + lf * (sin(phi) * cos(zeta) * cos(eta) * (-sin(theta)) + cos(phi) * cos(eta) * cos(theta));
    dyh_deta = dye_deta + lf * (sin(phi) * (-sin(zeta) * cos(eta) * cos(theta) - cos(zeta) * sin(eta)) + cos(phi) * cos(eta) * sin(theta));
    dyh_dzeta = lf * sin(phi) * (cos(zeta) * cos(eta) * cos(theta) - sin(zeta) * sin(eta));
    dyh_dphi = lf * (cos(phi) * (cos(zeta) * cos(eta) * cos(theta) - sin(zeta) * sin(eta)) - sin(phi) * cos(eta) * sin(theta));

    dzh_dtheta = dze_dtheta + lf * (sin(phi) * cos(zeta) * cos(theta) + cos(phi) * sin(theta));
    dzh_deta = 0;
    dzh_dzeta = -lf * sin(phi) * sin(zeta) * sin(theta);
    dzh_dphi = lf * (cos(phi) * cos(zeta) * sin(theta) + sin(phi) * cos(theta));

    Jh = [
        dxh_dtheta, dxh_deta, dxh_dzeta, dxh_dphi;
        dyh_dtheta, dyh_deta, dyh_dzeta, dyh_dphi;
        dzh_dtheta, dzh_deta, dzh_dzeta, dzh_dphi
    ];
end

function [theta, eta, zeta, phi] = inverse_kinematics(lu, lf, xHand, yHand, zHand)
    % Estimate the elbow position
    r = sqrt(xHand^2 + yHand^2 + zHand^2);
    theta = acos(-zHand / lu);
    eta = atan2(-xHand, yHand);
    zeta = atan2(lu * (xHand * yHand), yHand * (yHand * zHand - yHand * zHand) - xHand * (zHand * xHand - zHand * xHand));
    phi = acos((xHand^2 + yHand^2 + zHand^2 - lu^2 - lf^2) / (2 * lu * lf));
end
