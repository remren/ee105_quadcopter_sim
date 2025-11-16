%clear; clc; close all;

t_start = 0;
t_end = 15; 
t_span = [t_start, t_end];


params.g = 9.81;  
params.m = 1.0;  
params.Ix = 0.01;  
params.Iy = 0.01; 
params.Iz = 0.02;


% Defining the PID Controller Gains
% TBH no idea if these values are right, kinda just vibed it
% Altitude PID
params.Kp_z = 30;   
params.Ki_z = 20;   
params.Kd_z = 15;   

% Attitude (roll, pitch, yaw) PD
% Roll
params.Kp_phi = 15;
params.Kd_p = 5;
% Pitch
params.Kp_theta = 15;
params.Kd_q = 5;
% Yaw
params.Kp_psi = 5;
params.Kd_r = 2;


% Setting that we want the drone to take off at 0,0,0 and then hover 10
% meters in the air
X_desired = zeros(1, 12); 
% Sets the height
X_desired(3) = 10;

% initial conditions
X0 = zeros(1, 13); 


[t_out, X_out] = ode45(@(t, X) drone_dynamics_PID(t, X, params, X_desired), t_span, X0);

% --- 8. Plot Results ---
figure('Name', 'PID Controlled Drone Simulation');

%Altitude
subplot(2, 2, 1);
plot(t_out, X_out(:, 3), 'b', 'LineWidth', 2); hold on;
yline(X_desired(3), 'r--', 'LineWidth', 1.5);
title('Altitude (z-position)');
xlabel('Time (s)'); ylabel('Position (m)');
legend('Actual Altitude', 'Desired Altitude');
grid on;

%Euler Angles
subplot(2, 2, 2);
plot(t_out, rad2deg(X_out(:, 7)), 'r', 'LineWidth', 1.5); hold on;
plot(t_out, rad2deg(X_out(:, 8)), 'g', 'LineWidth', 1.5);
plot(t_out, rad2deg(X_out(:, 9)), 'b', 'LineWidth', 1.5);
title('Attitude (Euler Angles)');
xlabel('Time (s)'); ylabel('Angle (degrees)');
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)');
grid on;

%Position
subplot(2, 2, 3);
plot(t_out, X_out(:, 1), 'r', 'LineWidth', 1.5); hold on;
plot(t_out, X_out(:, 2), 'g', 'LineWidth', 1.5);
title('Lateral Position (x, y)');
xlabel('Time (s)'); ylabel('Position (m)');
legend('x-position', 'y-position');
grid on;

%3D Trajectory
subplot(2, 2, 4);
plot3(X_out(:, 1), X_out(:, 2), X_out(:, 3), 'b', 'LineWidth', 2);
hold on;
plot3(0, 0, 10, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Desired point
title('3D Trajectory');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Trajectory', 'Desired Setpoint');
grid on; axis equal;
view(30, 30);