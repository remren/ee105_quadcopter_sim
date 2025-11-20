clear; clc;

params.g = 9.81;
params.m = 1.0;
params.Ix = 0.01;
params.Iy = 0.01;
params.Iz = 0.02;

% Altitude
plant_z = tf(1, [params.m, 0, 0]);
opts_z = pidtuneOptions('DesignFocus', 'reference-tracking');


[C_z, info_z] = pidtune(plant_z, 'PID', 10, opts_z); 

CL_z = feedback(C_z * plant_z, 1);
S_z = stepinfo(CL_z);            

fprintf('--- ALTITUDE (Z) GAINS ---\n');
fprintf('params.Kp_z = %.4f;\n', C_z.Kp);
fprintf('params.Ki_z = %.4f;\n', C_z.Ki);
fprintf('params.Kd_z = %.4f;\n', C_z.Kd);
fprintf('Rise Time: %.2f s\n', S_z.RiseTime);
fprintf('Overshoot: %.2f%%\n\n', S_z.Overshoot);


% Roll + Pitch
plant_phi = tf(1, [params.Ix, 0, 0]);
opts_att = pidtuneOptions('DesignFocus', 'reference-tracking');

[C_phi, info_phi] = pidtune(plant_phi, 'PD', 25, opts_att);

CL_phi = feedback(C_phi * plant_phi, 1);
S_phi = stepinfo(CL_phi);

fprintf('--- ROLL (Phi) GAINS ---\n');
fprintf('params.Kp_phi = %.4f;\n', C_phi.Kp);
fprintf('params.Kd_p   = %.4f;\n', C_phi.Kd);
fprintf('Rise Time: %.2f s\n\n', S_phi.RiseTime);

fprintf('--- PITCH (Theta) GAINS ---\n');
fprintf('params.Kp_theta = %.4f;\n', C_phi.Kp); 
fprintf('params.Kd_q     = %.4f;\n\n', C_phi.Kd);


% Yaw
plant_psi = tf(1, [params.Iz, 0, 0]);
[C_psi, info_psi] = pidtune(plant_psi, 'PD', 10, opts_att);

fprintf('--- YAW (Psi) GAINS ---\n');
fprintf('params.Kp_psi = %.4f;\n', C_psi.Kp);
fprintf('params.Kd_r   = %.4f;\n', C_psi.Kd);