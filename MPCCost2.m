function cost = MPCCost2(inputs,params)
% Get parameters for MPC:
% params = params();

% Parse inputs
inputs = reshape(inputs,4,params.predHorizon+1);

% Preallocate
cost = 0;
statesComputed = zeros(28,params.predHorizon+1);
statesComputed(:,1) = params.initialState;

% Compute time history and assign cost
for i = 1:params.predHorizon

    statesComputed(:,i+1) = quad_cable_dynamics(statesComputed(:,i), inputs(:,i), [0 params.dt], params);
    
    % Compute angle between load and negative quadrotor body z axis
    dcm = EPtoDCM(statesComputed(13:16,i));
    body_z_negative = -1*dcm(:,3);
    body_z_negative = body_z_negative/norm(body_z_negative);
    load_angle = acos(dot(statesComputed(20:22,i),body_z_negative));
    
    % Compute State error and cost
    stateError = params.trajectory(1:3,params.timeStep+i) - statesComputed(1:3,i);
    cost = cost + stateError'*params.Q*stateError;
    cost = cost + inputs(:,i)'*params.R*inputs(:,i);
    cost = cost + 1*load_angle;
    
end
% Add cost of terminal state
stateError = params.trajectory(1:3,params.timeStep+params.predHorizon+1) - statesComputed(1:3,params.predHorizon+1);
load_angle = acos(dot(statesComputed(20:22,params.predHorizon+1),[0;0;-1]));

cost = cost + stateError'*params.Q*stateError;
cost = cost + 1*load_angle;
end