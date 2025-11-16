function X_dot = drone_dynamics(t, X, U, params)
    % Position
    x       = X(1); 
    y       = X(2);
    z       = X(3);
    % Velocity
    x_dot   = X(4); 
    y_dot   = X(5);
    z_dot   = X(6);

    phi     = X(7);  % Roll
    theta   = X(8);  % Pitch
    psi     = X(9);  % Yaw
    p       = X(10); % Roll Velocity
    q       = X(11); % Pitch Velocity
    r       = X(12); % Yaw Velocity

    U1 = U(1); % Total Thrust
    U2 = U(2); % Roll Torque
    U3 = U(3); % Pitch Torque
    U4 = U(4); % Yaw Torque
    
    % General Params
    g = params.g; % Grav
    m = params.m; % Mass of Drone

    % Moment of inertia about body
    Ix = params.Ix; 
    Iy = params.Iy;
    Iz = params.Iz;

    %Rotation Matrix 
    c_phi = cos(phi); s_phi = sin(phi);
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi); s_psi = sin(psi);

    % Making it so that the matrix is actually tied to 3D motions
    % TLDR transforming the vector from the drone body to the inertia frame
    R = [c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi;
         s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi;
         -s_theta,      c_theta*s_phi,                   c_theta*c_phi];

    % State Derivatives
    % Initialize
    X_dot = zeros(12, 1);
    
    % Derivatives of Position
    X_dot(1) = x_dot; 
    X_dot(2) = y_dot;
    X_dot(3) = z_dot; 

    % Derivatives of Velocity

    % Thrust on Frame
    % only on z axis as the thrust only moves the drone up
    T_body = [0; 0; U1]; 
    % Gravity on Frame
    F_grav = [0; 0; -m*g]; 
    
    % Total force in Frame
    F_total = R * T_body + F_grav; 
    
    % F = ma -> F/m = a 
    accel = F_total / m;
    % acceleration based on position cords
    X_dot(4) = accel(1); 
    X_dot(5) = accel(2); 
    X_dot(6) = accel(3); 

    % Rotational Derivatives
    W = [1, s_phi*tan(theta), c_phi*tan(theta);
         0, c_phi,           -s_phi;
         0, s_phi/c_theta,  c_phi/c_theta];
         
    euler_rates = W * [p; q; r];
    X_dot(7) = euler_rates(1); % \dot{phi}
    X_dot(8) = euler_rates(2); % \dot{theta}
    X_dot(9) = euler_rates(3); % \dot{psi}
    
    % Angular Acc
    torques = [U2; U3; U4];
    omega = [p; q; r];
    I = diag([Ix, Iy, Iz]);
    
    ang_accel = inv(I) * (torques - cross(omega, I * omega));
    
    X_dot(10) = ang_accel(1); % \dot{p}
    X_dot(11) = ang_accel(2); % \dot{q}
    X_dot(12) = ang_accel(3); % \dot{r}
    
end