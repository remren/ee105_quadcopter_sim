clear;
%% plot all results from Simulink

% Vertical and Roll
load("test_data_vertical_roll_2.mat")

import_throt = data{1};
import_roll  = data{2};

time = data{1}.Values.Time;
throttle = import_throt.Values.Data;
roll = import_roll.Values.Data;

%Altitude
subplot(2, 3, 1);
plot(t_out, X_out(:, 3), 'b', 'LineWidth', 2); hold on;
yline(10, 'r--', 'LineWidth', 1.5);
yline(20, 'b--', 'LineWidth', 1.5);
title('Altitude (z-position)');
xlabel('Time (s)'); ylabel('Position (m)');
legend('Actual Altitude', 'Desired Altitude', 'Desired Altitude 2');
grid on;

% % Pitch Angle
% subplot(2, 3, 2);
% plot(t_out, rad2deg(X_out(:, 8)), 'g', 'LineWidth', 1.5); hold on;
% yline(5, 'k--', 'LineWidth', 1.5, 'Label', 'Target Lean');
% title('Pitch Angle (\theta)'); grid on; xlabel('Time (s)'); ylabel('Degrees');
% % Shows when the 2nd simulation starts
% xline(5, 'k:', 'LineWidth', 1);
% 
% %Euler Angles
% subplot(2, 3, 3);
% plot(t_out, rad2deg(X_out(:, 7)), 'r', 'LineWidth', 1.5); hold on;
% plot(t_out, rad2deg(X_out(:, 8)), 'g', 'LineWidth', 1.5);
% plot(t_out, rad2deg(X_out(:, 9)), 'b', 'LineWidth', 1.5);
% title('Attitude (Euler Angles)');
% xlabel('Time (s)'); ylabel('Angle (degrees)');
% legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)');
% grid on;