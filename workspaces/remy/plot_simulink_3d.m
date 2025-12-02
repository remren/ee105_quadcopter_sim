function plot3DMotion(yaw, pitch, roll, vertical_velocity, time)
    % Convert angles from degrees to radians
    yaw_rad = deg2rad(yaw);
    pitch_rad = deg2rad(pitch);
    roll_rad = deg2rad(roll);
    
    % Initialize position arrays
    num_points = length(time);
    x = zeros(num_points, 1);
    y = zeros(num_points, 1);
    z = zeros(num_points, 1);
    
    % Calculate position based on vertical velocity and angles
    for i = 2:num_points
        dt = time(i) - time(i-1);
        % Update positions based on previous positions and velocities
        x(i) = x(i-1) + vertical_velocity(i-1) * cos(pitch_rad(i-1)) * cos(yaw_rad(i-1)) * dt + ...
             vertical_velocity(i-1) * sin(roll_rad(i-1)) * dt; % Roll affects X
        y(i) = y(i-1) + vertical_velocity(i-1) * cos(pitch_rad(i-1)) * sin(yaw_rad(i-1)) * dt + ...
             vertical_velocity(i-1) * sin(roll_rad(i-1)) * dt; % Roll affects Y
        z(i) = z(i-1) + vertical_velocity(i-1) * sin(pitch_rad(i-1)) * dt; % Positive vertical velocity indicates +Z
    end
    
    % Create 3D plot
    figure;
    plot3(x, y, z, 'LineWidth', 2);
    title('3D Motion Plot');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    grid on;
    axis equal;
end

clear;

%% Vertical only signal test
% load('test_data_vertical_only.mat');
% 
% throttle = data.Data;
% time = data.Time;
% placehold = zeros(size(time));
% 
% plot3DMotion(placehold, placehold, placehold, throttle, time)

%% General Tests
% Vertical and Roll
load("test_data_vertical_roll_2.mat")

import_throt = data{1};
import_roll  = data{2};

time = data{1}.Values.Time;
throttle = import_throt.Values.Data;
roll = import_roll.Values.Data;

placehold = zeros(size(time));

plot3DMotion(placehold, placehold, placehold, throttle, time)

% test_position = cumsum(throttle) * time/size(time);

% im getting so confused... my sim doesn't account for gravity
% ok, so i want to get position, per throttle input
% the resource said its cm/s per throttle, so 1 cm/s?
% now i need to integrate to find the positions

% i think im just confused as to what im doing to add to the project...
% like do i need to do a 3d plot of this?
% or am i more interested in comparing root locus method to the auto tune?

% Calculate the cumulative position based on throttle input
z_position = cumtrapz(time, throttle); % Integrate throttle to get z-position

% Update the z array with the calculated positions
% z(2:length(time)) = z_position(2:end);


%Altitude
subplot(2, 3, 1);
plot(time, z_position, 'b', 'LineWidth', 2); hold on;
yline(1, 'r--', 'LineWidth', 1.5);
% yline(1, 'b--', 'LineWidth', 1.5);
title('Altitude (z-position)');
xlabel('Time (s)'); ylabel('Position (m)');
legend('Actual Altitude', 'Desired Altitude');
grid on;
