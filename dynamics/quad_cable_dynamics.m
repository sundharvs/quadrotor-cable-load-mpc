function state = quad_cable_dynamics(x0, inputs, tspan, params)

P = [-params.dx0, -params.dx1,  params.dx2, params.dx3;
      params.dy0, -params.dy1, -params.dy2, params.dy3;
     -params.cT,   params.cT,  -params.cT,  params.cT]; % Thrust allocation matrix

% Input Computations
Torque = P*inputs; % Torque applied by rotors (Nm)
Thrust = sum(inputs); % Thrust (N)
Thrust = [0;0;Thrust];

% state order
% pQ [1:3], vQ [4:6], pL [7:9], vL [10:12], quat [13:16], omega [17:19], q [20:22], d_q [23:25], w [26:28]
state = zeros(28,1);

delta = x0(1:3) - x0(7:9); % pQ_0 - pL_0
x0_nontaut = x0(1:19); % parts of the initial state that go into nontaut dynamics
x0_taut = x0(7:28); % parts of the initial state that go into taut dynamics

if ( norm(delta) < params.l-0.01 ) %nontaut dynamics
    
    disp("nontaut dynamics")
    disp(norm(delta))
    taut_or_nontaut = 1;
    
    [~,odeOut] = ode45(@(t,x) nontaut_dynamics(t,x,Thrust,Torque, params), tspan, x0_nontaut);
    odeOut = odeOut';
    
    state(1:19, 1) = odeOut(:,end);
    % state(20:28) = nontaut_2_taut(state(7:9),state(10:12),state(1:3),state(4:6),params.l,x0(20:22),tspan(2) - tspan(1))';
    
    state(20:22, 1) = (state(7:9) - state(1:3)) / params.l;    
    state(23:25, 1) = (state(20:22) - x0(20:22)) / (tspan(2) - tspan(1));    
    state(26:28, 1) = cross((state(4:6) - state(10:12)),state(20:22)) / params.l;

elseif ( norm(delta) >= params.l-0.01 && norm(delta) < params.l+0.01 ) %taut dynamics
    
    taut_or_nontaut = 0;
    
    [~,odeOut] = ode45(@(t,x) taut_dynamics(t, x, Thrust, Torque, params), tspan, x0_taut);
    odeOut = odeOut';

    state(7:28, 1) = odeOut(:,end);
    state(1:3) = state(7:9) - params.l * state(20:22);
    state(4:6) = state(10:12) - params.l * cross(state(26:28),state(20:22));
    
else
    
    taut_or_nontaut = -1;
    warning("at "+ " delta value bigger "+norm(delta))
    
    % q = q / norm(q);
    % d_q = [0; 0; 0];
    % pQ = pL - l * q;
    
end

% state = [pQ; vQ; pL; vL; quat; omega; q; d_q; w];

%rotation matrix to euler angles
% [phi_0, theta_0, psi_0] = rot_2_eul(R_0);

end