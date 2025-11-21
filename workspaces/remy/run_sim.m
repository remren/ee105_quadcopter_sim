
%clear; clc; close all;

t_start = 0;
t_end = 15;
t_span = [t_start, t_end];

% Parameteres
params.g = 9.81;    % Gravity
params.m = 1.0;     % Mass (kg)
params.Ix = 0.01;   % Moment of inertia, x-axis (kg*m^2)
params.Iy = 0.01;   % Moment of inertia, y-axis (kg*m^2)
params.Iz = 0.02;   % Moment of inertia, z-axis (kg*m^2)

% Initial Setup
% [x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r]
X0 = zeros(1, 12); 
% Value that the drone is in the air
X0(3) = 0; 

% Simple Controls
target_z = 10;
% In order to hover thrust needs to be stronger than gravity
U1_ascent = (params.m *params.g) *1.20;

% The other torques are set to 0 just to keep it vertical
U = [U1_ascent, 0, 0, 0];

% Simulation 
[t_out, X_out] = ode45(@(t, X) drone_dynamics(t, X, U, params), t_span, X0);

% Plots
figure;
sgtitle('Quadcopter 6-DOF Simulation (Open-Loop)');
subplot(2, 2, 1);
plot(t_out, X_out(:, 1), 'r', 'LineWidth', 1.5); hold on;
plot(t_out, X_out(:, 2), 'g', 'LineWidth', 1.5);
plot(t_out, X_out(:, 3), 'b', 'LineWidth', 1.5);
yline(target_z, 'k--', 'LineWidth', 1.5, 'Label', 'Target Z');
title('Position (x, y, z)');
xlabel('Time (s)'); ylabel('Position (m)');
legend('x', 'y', 'z');
grid on;

% Plot Euler Angles
subplot(2, 2, 2);
plot(t_out, rad2deg(X_out(:, 7)), 'r', 'LineWidth', 1.5); hold on;
plot(t_out, rad2deg(X_out(:, 8)), 'g', 'LineWidth', 1.5);
plot(t_out, rad2deg(X_out(:, 9)), 'b', 'LineWidth', 1.5);
title('Angles (Roll, Pitch, Yaw)');
xlabel('Time (s)'); ylabel('Angle (degrees)');
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)');
grid on;

% Plot 3D Trajectory
subplot(2, 2, 3:4);
plot3(X_out(:, 1), X_out(:, 2), X_out(:, 3), 'b', 'LineWidth', 2);
hold on;

% Make a marker of the height we want
[X_plane, Y_plane] = meshgrid(linspace(min(X_out(:,1)), max(X_out(:,1)), 2), ...
                              linspace(min(X_out(:,2)), max(X_out(:,2)), 2));
Z_plane = ones(size(X_plane)) * target_z;
if max(X_out(:,3)) > target_z % Only plot plane if we pass it
    surf(X_plane, Y_plane, Z_plane, 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    legend('Trajectory', 'Target Z Plane');
end

title('3D Trajectory');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;
axis equal;
view(30, 30);