close all; clc; clear
N = 101;
time = linspace(0,10,N);
fun_w = @(x) -3*pi/250*x.^2 + 3*pi/25*x;
fun_theta = @(x) -pi/250*x.^3 + 3*pi/50*x.^2;
% vx = -r*w*sin(theta)
fun_velx = @(x) -(-3*pi/250*x.^2 + 3*pi/25*x).*sin(-pi/250*x.^3 + 3*pi/50*x.^2);
% vx = r*w*cos(theta)
fun_vely = @(x) (-3*pi/250*x.^2 + 3*pi/25*x).*cos(-pi/250*x.^3 + 3*pi/50*x.^2);
w = fun_w(time);
theta = fun_theta(time);

pos = zeros(2,N);
for i = 1:N
    t = time(i);
    % pos_x = integral vx*dx
    pos(1,i) = integral(fun_velx, 0, t);
    % pos_y = integral vx*dx
    pos(2,i) = integral(fun_vely, 0, t);
end
vel_x = fun_velx(time);
vel_y = fun_vely(time);
subplot(2,1,1)
plot(time, w, 'LineWidth',2)
xlabel('time'); ylabel('velocity')
title('Angular Velocity vs Time')
subplot(2,1,2)
plot(time, theta, 'LineWidth',2)
xlabel('time'); ylabel('theta')
title('Theta vs Time')
figure
subplot(2,2,1)
plot(time, vel_x, 'LineWidth',2)
xlabel('time'); ylabel('vx')
title('Velocity X vs time')
subplot(2,2,2)
plot(time, vel_y, 'LineWidth',2)
xlabel('time'); ylabel('vy')
title('Velocity Y vs time')
subplot(2,2,3)
plot(time, pos(1,:), 'LineWidth',2)
xlabel('time'); ylabel('x')
title('Position X vs time')
subplot(2,2,4)
plot(time, pos(2,:), 'LineWidth',2)
xlabel('time'); ylabel('y')
title('Position Y vs time')
figure
plot(pos(1,:), pos(2,:),'LineWidth',2)
xlabel('x'); ylabel('y')
title('Resulting 2D Trajectory')
axis equal
x_ref = pos(1,:)';
y_ref = pos(2,:)';
vx_ref = vel_x';
vy_ref = vel_y';
save('reference_trajectory', "x_ref", "y_ref", "vx_ref", "vy_ref")