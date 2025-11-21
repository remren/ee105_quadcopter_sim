% PID Tuning Test
m = 1.0; % 1 kg
Gz = tf(1/m, [1 0 0]);  % 1/(m s^2)

Kp = 30;
Ki = 20;
Kd = 15;

Cpid = pid(Kp, Ki, Kd);

rlocus(Cpid * Gz);
title('Altitude PID Root Locus');