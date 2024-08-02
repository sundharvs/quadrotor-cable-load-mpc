%% Housekeeping
clear all
close all
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');
width = 800;
height = 600;
set(groot,'DefaultFigureRenderer','painters')

% Get problem parameters and set initial state
params = params();
params.initialState = [0;0;0; % pQ - quadrotor position
                       0;0;0; % vQ - quadrotor velocity
                       -0.01;0;-0.3; % pL - load position
                       0;0;0; % vL - load velocity
                       0;0;0;1; % quat - quadrotor quaternion from body to inertial
                       0;0;0; % omega - quadrotor angular velocity in body frame
                       0;0;0; % q - unit vector from quadrotor to load
                       0;0;0; % dq - derivative of unit vector from quadrotor to load
                       0;0;0]; % w - load angular velocity

% sets initial load position to be l (cable length) away from quadrotor
if (norm(params.initialState(7:9)) ~= 0)
    params.initialState(7:9) = params.l*params.initialState(7:9) / norm(params.initialState(7:9));
end

% q (unit vector between quadrotor and load) initial condition
params.initialState(20:22) = (params.initialState(7:9) - params.initialState(1:3)) / params.l;

% Generate reference trajectory
loop_traj = generate_vertical_circle_traj([0;0;1.5],0,1.5,4.5,pi/2,-3/2*pi,1/params.dt);

% preallocate reference trajectory
params.trajectory = repmat([0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]', 1, 500);
params.trajectory(1:3,:) = repmat(loop_traj(1:3,end), 1, 500); % repeat last position in reference trajectory till end
params.trajectory(1:3,1:size(loop_traj,2)) = loop_traj(1:3,:); % insert reference trajectory

params.timeStep = 0; % time counter initialization

% Set initial guess for control input and preallocate history arrays
output = zeros(4*(params.predHorizon+1),1);
traj = params.initialState;
inputHist = zeros(4,1);
rpyHist = zeros(3,1);
%% MPC Loop
for i = 1:100
    % Set up functions
    optimFunc = @(x) MPCCost2(x,params);
    [A,b] = MPCLinearConstraints2(params);
    
    % Solve MPC
    tic
    output = fmincon(optimFunc,output,A,b,[],[],[],[],[],params.fsolveOptions);
    time = toc
    % Get next state using one-step optimal control input
    s = quad_cable_dynamics(traj(:,i), output(1:4), [0 params.dt], params);

    % Update trajectory and input history
    traj(:,i+1) = s;
    inputHist(:,i) = output(1:4);
    params.initialState = traj(:,i+1);

    DCM = EPtoDCM(traj(13:16,i)); % quaternion to DCM
    [yaw, pitch, roll] = dcm2angle(DCM');
    rpyHist(:,i) = [roll;pitch;yaw];

    % Set up for plots
    clf
    t = (1:i+1)*params.dt;
    if params.plot
        %% Plot Trajectory
        figure(1)
        subplot(3,1,1)
        hold on
        plot(t,traj(1,:),'r')
        plot(t,params.trajectory(1,1:i+1),'b')
        % plot(t,params.trajectory(1,1:i+1) - traj(1,:));
        legend("Actual Trajectory","Desired Trajectory")
        title("Trajectory")
        grid on
    
        subplot(3,1,2)
        hold on
        plot(t,traj(2,:),'r')
        plot(t,params.trajectory(2,1:i+1),'b')
        % plot(t,params.trajectory(2,1:i+1) - traj(2,:));
    
        grid on
    
        subplot(3,1,3)
        hold on
        plot(t,traj(3,:),'r')
        plot(t,params.trajectory(3,1:i+1),'b')
        % plot(t,params.trajectory(3,1:i+1) - traj(3,:));
        grid on
    
        %% Plot inputs
        figure(2)
        plot(t(1:end-1),inputHist(1,:))
        hold on
        plot(t(1:end-1),inputHist(2,:))
        plot(t(1:end-1),inputHist(3,:))
        plot(t(1:end-1),inputHist(4,:))
        grid on
        title("Input History")
        legend("$u_1$","$u_1$","$u_1$","$u_1$")
        drawnow();

        %% Plot orientation
        figure(3)
        plot(t(1:end-1),rad2deg(rpyHist(1,:)))
        hold on
        plot(t(1:end-1),rad2deg(rpyHist(2,:)))
        plot(t(1:end-1),rad2deg(rpyHist(3,:)))
        grid on
        title("Input History")
        legend("$roll$","$pitch$","$yaw$")
        drawnow();

    end

    %% Update for next timestep
    params.timeStep = params.timeStep + 1;
    output = [output(5:end);0;0;0;0];
    % output = 0*output;
end
%% Animate Drone

drone_Animation(traj(1,:),traj(2,:),traj(3,:),traj(7,:),traj(8,:),traj(9,:),1*rpyHist(1,:),1*rpyHist(2,:),rpyHist(3,:),[-5,5],[-5,5],[-5,5])

