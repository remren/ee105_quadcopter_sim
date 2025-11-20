%clear; clc; close all;

t_start = 0;
t_end = 15; 
t_span = [t_start, t_end];


params.g = 9.81;  
params.m = 1.0;  
params.Ix = 0.01;  
params.Iy = 0.01; 
params.Iz = 0.02;


%% Defining the PID Controller Gains

% Altitude PID values WITHOUT tuning
% params.Kp_z = 30;   
% params.Ki_z = 20;   
% params.Kd_z = 15;   

% PidTune, Hover only values
% params.Kp_z = 0.228;   
% params.Ki_z = 0.0132;   
% params.Kd_z = 0.987;   


% Pidtune (Hover + Roll)

params.Kp_z = 3.4993;   
params.Ki_z = 0.3062;   
params.Kd_z = 9.9969;   



% Values we get from root locus function
% params.Kp_z = 192.00;
% params.Ki_z = 512.00;
% params.Kd_z = 24.00;


%% Attitude (roll, pitch, yaw) PD
% Roll
% Hover Only
% params.Kp_phi = 15;
% params.Kd_p = 5;

params.Kp_phi = 3.1250;
params.Kd_p   = 0.2165;


% Pitch (Hover only)
% params.Kp_theta = 15;
% params.Kd_q = 5;

params.Kp_theta = 3.1250;
params.Kd_q     = 0.2165;


% Yaw (Hover Only)
% params.Kp_psi = 5;
% params.Kd_r = 2;

params.Kp_psi = 1.0000;
params.Kd_r   = 0.1732;

%% Hover Only
% % Setting that we want the drone to take off at 0,0,0 and then hover 10
% % meters in the air
% X_desired = zeros(1, 12); 
% % Sets the height
% X_desired(3) = 10;
% 
% % initial conditions
% X0 = zeros(1, 13); 
% 
% 
% [t_out, X_out] = ode45(@(t, X) drone_dynamics_PID(t, X, params, X_desired), t_span, X0);


%% Hover plus a bit of a roll

t_span1 = [0, 10];
X0 = zeros(1, 13); 

% Hover in a the air 
X_desired_1 = zeros(1, 12); 
X_desired_1(3) = 10; 
X_desired_1(8) = deg2rad(5); 
X_desired_1(7) = deg2rad(5); 

% Run Simulation for Segment 1
[t1, X1] = ode45(@(t, X) drone_dynamics_PID(t, X, params, X_desired_1), t_span1, X0);

t_span2 = [10, 30];

% Take the end of State 1 and start there
X0_2 = X1(end, :); 

% Desired State: Altitude 10m, LEAN Forward
% Keeping the hover + leaning 
X_desired_2 = zeros(1, 12);
X_desired_2(3) = 20;         
X_desired_2(8) = deg2rad(-6); 

% index 7 allows us to roll
                    

% Run Simulation for Segment 2
[t2, X2] = ode45(@(t, X) drone_dynamics_PID(t, X, params, X_desired_2), t_span2, X0_2);

%% Plotting

% For simulation two we need to combine the time
t_out = [t1; t2];
X_out = [X1; X2];

figure('Name', 'PID Controlled Drone Simulation');

%Altitude
subplot(2, 3, 1);
plot(t_out, X_out(:, 3), 'b', 'LineWidth', 2); hold on;
yline(10, 'r--', 'LineWidth', 1.5);
yline(20, 'b--', 'LineWidth', 1.5);
title('Altitude (z-position)');
xlabel('Time (s)'); ylabel('Position (m)');
legend('Actual Altitude', 'Desired Altitude', 'Desired Altitude 2');
grid on;

% Pitch Angle
subplot(2, 3, 2);
plot(t_out, rad2deg(X_out(:, 8)), 'g', 'LineWidth', 1.5); hold on;
yline(5, 'k--', 'LineWidth', 1.5, 'Label', 'Target Lean');
title('Pitch Angle (\theta)'); grid on; xlabel('Time (s)'); ylabel('Degrees');
% Shows when the 2nd simulation starts
xline(5, 'k:', 'LineWidth', 1);

%Euler Angles
subplot(2, 3, 3);
plot(t_out, rad2deg(X_out(:, 7)), 'r', 'LineWidth', 1.5); hold on;
plot(t_out, rad2deg(X_out(:, 8)), 'g', 'LineWidth', 1.5);
plot(t_out, rad2deg(X_out(:, 9)), 'b', 'LineWidth', 1.5);
title('Attitude (Euler Angles)');
xlabel('Time (s)'); ylabel('Angle (degrees)');
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)');
grid on;

%Position
subplot(2, 3, 4);
plot(t_out, X_out(:, 1), 'r', 'LineWidth', 1.5); hold on;
plot(t_out, X_out(:, 2), 'g', 'LineWidth', 1.5);
title('Lateral Position (x, y)');
xlabel('Time (s)'); ylabel('Position (m)');
legend('x-position', 'y-position');
grid on;


%3D Trajectory
subplot(2, 3, 5);
plot3(X_out(:, 1), X_out(:, 2), X_out(:, 3), 'b', 'LineWidth', 2);
grid on; axis equal;
title('3D Flight Path'); xlabel('X'); ylabel('Y'); zlabel('Z');
view(30, 30);


%% Getting PID values via root locus
% % Setting the poles so that we have a damped response
% p1 = -8;
% p2 = -8;
% p3 = -8;
% desired_poles = [p1,p2,p3];
% 
% desired_cfs = poly(desired_poles);
% 
% A = desired_cfs(2);
% B = desired_cfs(3);
% C = desired_cfs(4);
% 
% Kp = B *m;
% Ki = C*m;
% Kd = A*m;
% 
% fprintf('Kp = %.2f\n', Kp);
% fprintf('Ki = %.2f\n', Ki);
% fprintf('Kd = %.2f\n', Kd);
