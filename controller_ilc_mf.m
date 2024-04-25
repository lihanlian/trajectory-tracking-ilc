close all; clear; clc
load initial_control_input.mat
load reference_trajectory.mat
load ilc_matrix_actual.mat
load ilc_matrix_nominal.mat
%% Construct necessary matrices for ILC iteration
ux = ux_initial; uy = uy_initial;
ux = ux(1:end-1);
uy = uy(1:end-1);

index_x = 1:2:199;
index_y = 2:2:200;

x_ref = x_ref(2:end);
y_ref = y_ref(2:end);

Traj_ref = zeros(200,1); % X = [x0, y0, x1, y1, ...]
U = zeros(200,1); % U = [ux0, uy0, ux1, uy1, ...]
for i = 1:100
    Traj_ref(2*i-1) = x_ref(i);
    Traj_ref(2*i) = y_ref(i);
    U(2*i-1) = ux(i);
    U(2*i) = uy(i);
end
time = 0.1:0.1:10;
N = 101; % Number of discretization points

%% Plot Result of Inital Control Input Computed Using Direct Collocation
u_prev = 0;
position = G_actual*U + d_actual;

subplot(2,1,1)
plot(time, x_ref, '-', 'LineWidth',2); hold on;
plot(time, position(index_x), '--', 'LineWidth',2)
xlabel('Time'); ylabel('Position X');legend('x\_ref', 'x\_sim')
title('Trajectory X Before ILC')
subplot(2,1,2)
plot(time, y_ref, '-', 'LineWidth',2); hold on;
plot(time, position(index_y), '--', 'LineWidth',2)
xlabel('Time'); ylabel('Position Y');legend('y\_ref', 'y\_sim')
title('Trajectory Y Before ILC')
figure
plot(Traj_ref(index_x), Traj_ref(index_y), '-', 'LineWidth',2); hold on
plot(position(index_x), position(index_y), '-.', 'LineWidth',2)
xlabel('X'); ylabel('Y');legend('reference', 'simulation'); axis equal
title('2D Trajectory Before ILC')
%% ILC Loop
num_iterations = 200; 
alpha = 1; % initialize step size
e_norm = zeros(num_iterations, 1);
% initialize G_star_e for the first time step
G_star_e_prev = 0;
tau = fliplr(eye(200));
% ILC Iteration
for i = 1:num_iterations

    % get currrent output position
    position = actual_dynamics(U, G_actual, d_actual);
    % Calculate the current error
    pos_error = Traj_ref - position;
    e_norm(i) = norm(pos_error);

    tau_e = tau*pos_error;
    % Calculate the adjoint G*e
    G_star_e = tau * actual_dynamics(tau_e, G_actual, d_actual);
   
    % Calculate Delta h_i and Delta V_i
    Delta_h_i = G_star_e - G_star_e_prev;
    Delta_u_i = U - u_prev;

    % Get step size for x
    if Delta_h_i' * Delta_u_i >= 0
        alpha = 0.01;
    else
        alpha = -Delta_h_i' * Delta_u_i / ((Delta_h_i)' * Delta_h_i);
    end

    u_prev = U;
    % Update u and G*e
    U = U + alpha * G_star_e;
    G_star_e_prev = G_star_e;
end


%% Plot the results
figure % Plot 1d trajectory vs time
subplot(2,1,1)
plot(time, x_ref, '-', 'LineWidth',2); hold on;
plot(time, position(index_x), '--', 'LineWidth',2)
legend('x\_ref', 'x\_sim')
title('Trajectory X After ILC-MF')
subplot(2,1,2)
plot(time, y_ref, '-', 'LineWidth',2); hold on;
plot(time, position(index_y), '--', 'LineWidth',2)
legend('y\_ref', 'y\_sim')
title('Trajectory Y After ILC-MF')
figure % Plot 2d trajectory
plot(x_ref, y_ref, '-', 'LineWidth',2); hold on
plot(position(index_x), position(index_y), '-.', 'LineWidth',2)
legend('Reference', 'ILC Result')
xlabel('X'); ylabel('Y'); axis equal
title('2D Trajectory After ILC-MF')
figure % Plot error vs iteration
plot(1:num_iterations, e_norm, '-x','LineWidth',2)
xlabel('Iteration'); ylabel('Error')
title('Tracking Error vs Iteration ILC-MF')
%%%%%%%%%%%%% Function definition %%%%%%%%%%%%%%%%%
function trajectory = actual_dynamics(U, G_actual, d_actual)
    trajectory = G_actual * U + d_actual;
end