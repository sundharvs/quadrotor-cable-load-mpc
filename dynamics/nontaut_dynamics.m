function xdot = nontaut_dynamics(~, x, Thrust, Torque, params)
    g = [0;0;-9.81]; % Gravity (m/s^2)

    % Parsing inputs
    pQ = x(1:3); % Quadrotor position in inertial frame (m)
    vQ = x(4:6); % Quadrotor velocity in inertial frame (m/s)
    pL = x(7:9); % Load position in inertial frame (m)
    vL = x(10:12); % Load velocity in inertial frame (m/s)
    quat = x(13:16); % Quaternion q_WB mapping from body to inertial
    omega = x(17:19); % Quadrotor angular velocity in body frame

    pLdot = vL; % load position
    vLdot = g / params.mL; % load velocity
    pQdot = vQ; % quadrotor position

    % Get quaternion and angular velocity derivatives
    EPKDERes = EP_torqued(params.J(1),params.J(2),params.J(3),Torque,[quat;omega]); % Quaternion KDE
    quatDot = EPKDERes(1:4);
    omegaDot = EPKDERes(5:7);

    DCM = EPtoDCM(quat);
    vQdot = DCM*Thrust/params.mQ + g; % quadrotor velocity

    xdot = [pQdot; vQdot; pLdot; vLdot; quatDot; omegaDot];
end
