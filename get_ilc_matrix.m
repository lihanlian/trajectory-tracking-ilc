close all; clc; clear
load initial_control_input.mat; load reference_trajectory.mat
ux = ux_initial; uy = uy_initial;

%% Get matrices for nominal model
A_nominal = [0 1 0 0; 
    0 0 0 0;
    0 0 0 1;
    0 0 0 0]; 
B_nominal = [0 0;
    1 0; 
    0 0;
    0 1]; 
C_nominal = [1 0 0 0; 0 0 1 0];
N = 100;
x0 = [0;0;0;0];
sysc = ss(A_nominal,B_nominal,C_nominal,0);
sysd = c2d(sysc,0.1,'zoh');
Ad_nominal = sysd.A; Bd_nominal = sysd.B; Cd_nominal = sysd.C;

G_nominal = zeros(200, 200); % Preallocate G
d_nominal = zeros(200,1);   % Preallocate d
% Fill in matrix G
for i = 0:N-1
    entry = Cd_nominal*Ad_nominal^i * Bd_nominal;
    pattern = diag(ones(1,100-i), -i);
    G_temp = kron(pattern, entry);
    G_nominal = G_nominal + G_temp;
end

% Fill in matrix d
for i = 1:N
    entry = Cd_nominal*Ad_nominal^i*x0;
    d_nominal(2*i-1:2*i) = entry;
end
save('ilc_matrix_nominal', 'G_nominal', 'd_nominal');
%% Get matrices for actual model
A_actual = [0 0.8 0 0; 
    0 0 0 0;
    0 0 0 0.8;
    0 0 0 0]; 
B_actual = [0 0;
    1 0; 
    0 0;
    0 1]; 
C_actual = [1 0 0 0; 0 0 1 0];
N = 100;
x0 = [0;0;0;0];
sysc_actual = ss(A_actual,B_actual,C_actual,0);
sysd_actual = c2d(sysc_actual,0.1,'zoh');
Ad_actual = sysd_actual.A; Bd_actual = sysd_actual.B; Cd_actual = sysd_actual.C;

G_actual = zeros(200, 200); % Preallocate G
d_actual = zeros(200,1);   % Preallocate d
% Fill in matrix G
for i = 0:N-1
    entry = Cd_actual*Ad_actual^i * Bd_actual;
    pattern = diag(ones(1,100-i), -i);
    G_temp = kron(pattern, entry);
    G_actual = G_actual + G_temp;
end

% Fill in matrix d
for i = 1:N
    entry = Cd_actual*Ad_actual^i*x0;
    d_actual(2*i-1:2*i) = entry;
end
save('ilc_matrix_actual', 'G_actual', 'd_actual');