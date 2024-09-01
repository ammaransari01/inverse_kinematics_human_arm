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