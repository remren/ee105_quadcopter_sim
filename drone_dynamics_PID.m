function X_dot = drone_dynamics_PID(t, X, params, X_desired)
    x       = X(1);
    y       = X(2);
    z       = X(3);
    x_dot   = X(4);
    y_dot   = X(5);
    z_dot   = X(6);
    phi     = X(7);  % Roll
    theta   = X(8);  % Pitch
    psi     = X(9);  % Yaw
    p       = X(10); % Roll rate
    q       = X(11); % Pitch rate
    r       = X(12); % Yaw rate
    z_int   = X(13); % Altitude integral error


    z_des = X_desired(3);
    phi_des = X_desired(7);
    theta_des = X_desired(8);
    psi_des = X_desired(9);

    p_des = 0;
    q_des = 0;
    r_des = 0;
    z_dot_des = 0;

    g = params.g; m = params.m;
    Ix = params.Ix; Iy = params.Iy; Iz = params.Iz;
    

    
    % Altitude PID
    error_z = z_des - z;
    error_z_dot = z_dot_des - z_dot;
    
    U1_correction = params.Kp_z * error_z + ...
                    params.Ki_z * z_int + ...
                    params.Kd_z * error_z_dot;
    
    hover_thrust = m * g;
    
    % Thrust must compensate for tilt
    U1 = (hover_thrust + U1_correction) / (cos(phi) * cos(theta));
    % make sure the value isn't negative
    U1 = max(0, U1); 

    % Roll, Pitch, Yaw
    error_phi = phi_des - phi;
    error_p = p_des - p;
    U2 = params.Kp_phi * error_phi + params.Kd_p * error_p; % Roll Torque
    
    error_theta = theta_des - theta;
    error_q = q_des - q;
    U3 = params.Kp_theta * error_theta + params.Kd_q * error_q; % Pitch Torque

    error_psi = psi_des - psi;
    error_r = r_des - r;
    U4 = params.Kp_psi * error_psi + params.Kd_r * error_r; % Yaw Torque
    
    % Rotational Matrix
    c_phi = cos(phi); s_phi = sin(phi);
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi); s_psi = sin(psi);

    R = [c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi;
         s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi;
         -s_theta,      c_theta*s_phi,                   c_theta*c_phi];

    % State Derivatives
    X_dot = zeros(13, 1);
    
    % Translational Derivatives
    X_dot(1) = x_dot;
    X_dot(2) = y_dot;
    X_dot(3) = z_dot;

    % Linear Accelerations
    T_body = [0; 0; U1];
    F_grav = [0; 0; -m*g];
    F_total = R * T_body + F_grav;
    accel = F_total / m;
    X_dot(4) = accel(1);
    X_dot(5) = accel(2);
    X_dot(6) = accel(3);

    % Rotational Derivatives (Kinematics)
    W = [1, s_phi*tan(theta), c_phi*tan(theta);
         0, c_phi,           -s_phi;
         0, s_phi/c_theta,  c_phi/c_theta];
    euler_rates = W * [p; q; r];
    X_dot(7) = euler_rates(1);
    X_dot(8) = euler_rates(2);
    X_dot(9) = euler_rates(3);
    
    % Angular Accelerations
    torques = [U2; U3; U4];
    omega = [p; q; r];
    I = diag([Ix, Iy, Iz]);
    ang_accel = inv(I) * (torques - cross(omega, I * omega));
    X_dot(10) = ang_accel(1);
    X_dot(11) = ang_accel(2);
    X_dot(12) = ang_accel(3);
    
    %  Derivative of the Integral Term
    X_dot(13) = error_z;
    
end