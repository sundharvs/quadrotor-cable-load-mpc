function a = params()

% Geometry
a.dx0 = 0.19; % X distance of first rotor  (m)
a.dx1 = 0.19; % X distance of second rotor (m)
a.dx2 = 0.19; % X distance of third rotor  (m)
a.dx3 = 0.19; % X distance of fourth rotor (m)

a.dy0 = 0.19; % Y distance of first rotor  (m)
a.dy1 = 0.19; % Y distance of second rotor (m)
a.dy2 = 0.19; % Y distance of third rotor  (m)
a.dy3 = 0.19; % Y distance of fourth rotor (m)

% drag
a.cT = 3.75e-9; %0.05; % Rotor drag torque constant (idk)

% Mass parameters
a.mQ = 0.547; % mass of quadrotor (kg)
a.J = [0.0033;0.0033;0.0033];
a.maxThrust = 5; % maximum thrust (N)
a.minThrust = 0; % minimum thrust (N)
a.mL = 0.05; % mass of payload (kg)
a.l = 0.3; % length of cable [m]

% MPC Parameters
a.Q = zeros(3);
a.Q(1:3,1:3) = 40*eye(3);
% a.Q(3,3) = 10;
% a.Q(4:7,4:7) = 0.01*eye(4);
a.R = 0.01*eye(4);
a.predHorizon = 8;
a.dt = 0.05;
a.equalityTolerance = 0;

% Optimizer Parameters
tolerance = 0.1;
a.options = odeset('RelTol',tolerance, 'AbsTol',tolerance);
a.fsolveOptions = optimset('display','none','TolX',1e-6,'TolFun',1e-6,"Algorithm",'sqp','MaxFunEvals',64000,'UseParallel',false);

% Plot?
a.plot = true;
