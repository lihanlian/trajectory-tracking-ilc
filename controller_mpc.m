close all; clc; clear
load trajectory_reference.mat
%% Get nominal and actual system matrices
dt = 0.1; % Time step size
% Construct nominal system
Ac_nominal = [0 1 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 0 0];
Bc_nominal = [0 0;
      1 0;
      0 0;
      0 1];
Cc_nominal = [1 0 0 0; 0 0 1 0];
% Convert to discrete-time using zero-order hold
sys_nominal = ss(Ac_nominal, Bc_nominal, Cc_nominal, 0);
sysd_nominal = c2d(sys_nominal, dt, 'zoh');
[Ad_nominal, Bd_nominal, Cd_nominal, Dd_nominal] = ssdata(sysd_nominal);

% Construct acutal system
Ac_actual = [0 0.8 0 0;
      0 0 0 0;
      0 0 0 0.8;
      0 0 0 0];
Bc_actual = [0 0;
      1 0;
      0 0;
      0 1];
Cc_actual = [1 0 0 0; 0 0 1 0];
% Convert to discrete-time using zero-order hold
sys_actual = ss(Ac_actual, Bc_actual, Cc_actual, 0);
sysd_actual = c2d(sys_actual, dt, 'zoh');
[Ad_actual, Bd_actual, Cd_actual, Dd_actual] = ssdata(sysd_actual);

%% Set up mpc controller (using nominal system)
% Create the MPC object
mpc_controller = mpc(sysd_nominal, dt);
mpc_controller.PredictionHorizon = 10;
mpc_controller.ControlHorizon = 10;

% Define the weights
Q = diag([10, 10]);  % Higher weight on position errors than on velocity
R = diag([0.1, 0.1]);  % Weight for control effort

% Assign weights to MPC controller
mpc_controller.Weights.OV = Q;
mpc_controller.Weights.MV = R;

% Setup constraints
mpc_controller.MV(1).Min = -2;
mpc_controller.MV(1).Max = 2;
mpc_controller.MV(2).Min = -2;
mpc_controller.MV(2).Max = 2;
%% Simulation loop
x0 = [0; 0; 0; 0];  % Initial state
u0 = [0; 0];  % Initial input

% Store the results
X = zeros(4, 101);
U = zeros(2, 100);
Y = zeros(2, 101);

% set initial state
mpc_state = mpcstate(mpc_controller);
mpc_state.Plant = [0;0;0;0];
xk = x0;
uk = u0;
% The control loop
for k = 0:100
    % Get X,Y position and reference for the current timestep
    yk = Cc_actual * xk; 
    reference = [x_ref(k+1); y_ref(k+1)];
    [uk, info] = mpcmove(mpc_controller, mpc_state, yk, reference);
    
    % Update state using actual system
    xk_plus1 = Ad_actual * xk + Bd_actual * uk; 
    X(:, k + 1) = xk_plus1;
    Y(:, k + 1) = Cd_actual * xk_plus1;
    U(:, k + 1) = uk;
    % Update state for the next iteration
    xk = xk_plus1; 
end

%% Get simulated trajectory using stored control input
time = 0:0.1:10;
figure
pos_actual = lsim(sys_actual, U, time,[0;0;0;0]);
pos_x_actual = pos_actual(:,1); 
pos_y_actual = pos_actual(:,2);
subplot(2,1,1)
plot(time, x_ref, '-', 'LineWidth', 2); hold on
plot(time,pos_x_actual, '-.', 'LineWidth', 2);
xlabel('Time'); ylabel('Position X');legend("Reference", "MPC Result")
title('Trajectory X for Actual Model')

subplot(2,1,2)
plot(time, y_ref, '-', 'LineWidth', 2); hold on
plot(time,pos_y_actual, '-.', 'LineWidth', 2)
xlabel('Time'); ylabel('Position Y');legend("Reference", "MPC Result")
title('Trajectory Y for Actual Model')

figure
plot(x_ref, y_ref, 'LineWidth', 2); hold on
plot(pos_x_actual,pos_y_actual, 'LineWidth', 2)
xlabel('X'); ylabel('Y');legend("Reference", "MPC Result")
axis equal
title('MPC Tracking Result')
% Calculate the tracking error of MPC 
traj_actual = [pos_x_actual; pos_y_actual];
traj_ref = [x_ref; y_ref];
traj_error = norm(traj_ref - traj_actual);
fprintf('tracking error of MPC controller: %.4d\n',traj_error)