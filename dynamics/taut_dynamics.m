function xdot = taut_dynamics(~, x, Thrust, Torque, params)
    g = [0;0;-9.81]; % Gravity (m/s^2)

    % Parsing inputs
    pL = x(1:3); % Load position in inertial frame (m)
    vL = x(4:6); % Load velocity in inertial frame (m/s)
    quat = x(7:10); % Quadrotor quaternion q_WB mapping from body to inertial
    omega = x(11:13); % Quadrotor angular velocity in body frame
    q = x(14:16); % Unit vector from quad to load
    d_q = x(17:19); % Derivative of unit vector from quad to load
    w = x(20:22); % Load angular velocity

    % Get quadrotor quaternion and angular velocity derivatives
    EPKDERes = EP_torqued(params.J(1),params.J(2),params.J(3),Torque,[quat;omega]); % Quaternion KDE
    quatDot = EPKDERes(1:4);
    omegaDot = EPKDERes(5:7);
        
    %load attitude dynamics
    DCM = EPtoDCM(quat);
    qdot = d_q;
    d_qdot = (1/(params.mQ*params.l)) * (  cross(q,cross(q,DCM*Thrust)) - params.mQ*params.l*dot(d_q,d_q)*q );
    
    % angular velocity of the load
    wdot = - cross(q,DCM*Thrust) / (params.mQ*params.l);
    
    % quadrotor position dynamics
    pLdot = vL;
    vLdot = ( ( dot(q,DCM*Thrust) - params.mQ*params.l*dot(d_q,d_q) )* q / (params.mQ+params.mL) ) + g;

    xdot = [pLdot; vLdot; quatDot; omegaDot; qdot; d_qdot; wdot];
end