close all; clear;clc
% Time and discretization setup
T = 10; % Total time
N = 101; % Number of discretization points
dt = T / (N - 1); % Time step
time = linspace(0, T, N);

% Initial guess for [position; velocity; acceleration]
x0 = [linspace(0, -1, 50), linspace(-1, 2, N-50), zeros(1, N), zeros(1, N), ...
    linspace(0, 1, 50), linspace(-1, 0, N-50), zeros(1, N), zeros(1, N)]; 

% Optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 100000);

% Solve the optimization problem
[sol,~,exitflag,output] = fmincon(@(x) objectiveFunction(x, N, dt), x0, [], [], [], [], [], [], @(x) constraints(x, N, dt), options);

% Extract solution
pos_x = sol(1:N);
vel_x = sol(N+1:2*N);
ux = sol(2*N+1:3*N); % Control input is the acceleration
pos_y = sol(3*N+1:4*N);
vel_y = sol(4*N+1:5*N);
uy = sol(5*N+1:end); % Control input is the acceleration

%%%%%%%%%%%%% Simulation %%%%%%%%%%%%%%%%%%%%

% Assuming acc is your control input vector from the optimization solution
% and time is the corresponding time vector
ux_interp = @(t) interp1(time, ux, t, 'linear', 'extrap'); % Interpolate control input ux
uy_interp = @(t) interp1(time, uy, t, 'linear', 'extrap'); % Interpolate control input uy

% Define the system dynamics function
systemDynamics = @(t, x) [x(2); ux_interp(t); x(4); uy_interp(t)];

% Initial conditions
x0 = [0; 0; 0; 0]; % Starting position 1, velocity 0

% Time span for the simulation
% tspan = [0 10];

% Solve the ODE
[t_sim, x_sim] = ode45(systemDynamics, time, x0);

% Extract the position and velocity
position_x = x_sim(:, 1);
velocity_x = x_sim(:, 2);
position_y = x_sim(:, 3);
velocity_y = x_sim(:, 4);

% Plot the results
figure
plot(position_x, position_y, 'LineWidth',2)
axis equal; xlim([-2.5 .5]); ylim([-1.5 1.5])
title('Initial Trajectory using Direct Collocation'); xlabel('X'); ylabel('Y');
ux_initial = ux'; uy_initial = uy';
save('initial_control_input', 'ux_initial', 'uy_initial')
%%%%%%%%%%%%% Function definition %%%%%%%%%%%%%%%%%
function cost = objectiveFunction(x, N, dt)
    % Objective function to minimize the control effort (sum of squared accelerations)
    ux = x(2*N+1:3*N);
    uy = x(5*N+1:end);
    cost = sum(ux.^2) * dt + sum(uy.^2) * dt;
end

function [c, ceq] = constraints(x, N, dt)
    pos_x = x(1:N);
    vel_x = x(N+1:2*N);
    ux = x(2*N+1:3*N); % Control input is the acceleration
    pos_y = x(3*N+1:4*N);
    vel_y = x(4*N+1:5*N);
    uy = x(5*N+1:end); % Control input is the acceleration
    
    % Initialize dynamics constraints (based on actual model)
    ceq = zeros(4*(N-1), 1); % Preallocate for speed, adding 1 for the mid-point constraint
    for i = 1:N-1
        ceq(i) = pos_x(i+1) - pos_x(i) - dt * vel_x(i); % Position update
        ceq(N-1+i) = vel_x(i+1) - vel_x(i) - dt * ux(i); % Velocity update based on acceleration
        ceq(2*(N-1)+i) = pos_y(i+1) - pos_y(i) - dt * vel_y(i); % Position update
        ceq(3*(N-1)+i) = vel_y(i+1) - vel_y(i) - dt * uy(i); % Velocity update based on acceleration
    end
    
    % Boundary conditions (initial and final state constraints)
    ceq = [ceq; pos_x(1) ; pos_y(1); vel_x(1); vel_y(1);...
        pos_x(N); pos_y(N); vel_x(N); vel_y(N)]; 
    
    % Additional waypoint constraint: (-1,1),(-2,0),(-1,-1)
    ceq(end+1) = pos_x(round(N/4)) + 1;
    ceq(end+1) = pos_y(round(N/4)) - 1;
    ceq(end+1) = pos_x(round(N/2)) + 2;
    ceq(end+1) = pos_y(round(N/2));
    ceq(end+1) = pos_x(round(3*N/4)) + 1;
    ceq(end+1) = pos_y(round(3*N/4)) + 1;
    
    % Control input constraints
    c = [max(ux - 2), max(-2 - ux), max(uy - 2), max(-2-uy)]; 
end
