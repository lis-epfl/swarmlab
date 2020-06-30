p_drone.mass = 0.025;      % [kg]
p_drone.Jx   = 1.657171e-5;   % [kg * m2]
p_drone.Jy   = 1.6655602e-5;   % [kg * m2]
p_drone.Jz   = 2.9261652e-5;   % [kg * m2]
p_drone.Jxz  = 0; % 0.718277e-6;    % [kg * m2]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute gammas
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_drone.gamma  = p_drone.Jx*p_drone.Jz - p_drone.Jxz^2;
p_drone.gamma1 = p_drone.Jxz*(p_drone.Jx-p_drone.Jy+p_drone.Jz)/p_drone.gamma;
p_drone.gamma2 = (p_drone.Jz*(p_drone.Jz-p_drone.Jy)+p_drone.Jxz^2)/p_drone.gamma;
p_drone.gamma3 = p_drone.Jz/p_drone.gamma;
p_drone.gamma4 = p_drone.Jxz/p_drone.gamma;
p_drone.gamma5 = (p_drone.Jz-p_drone.Jx)/p_drone.Jy;
p_drone.gamma6 = p_drone.Jxz/p_drone.Jy;
p_drone.gamma7 = ((p_drone.Jx-p_drone.Jy)*p_drone.Jx+p_drone.Jxz^2)/p_drone.gamma;
p_drone.gamma8 = p_drone.Jx/p_drone.gamma;

% Aerodynamic coefficients
p_drone.l_arm      = 0.03252691193;            % [m] length of one arm
p_drone.J_prop     = 1/12*0.0002*(0.1^2+0.01^2);   % [kg * m^2] inertia of each propeller %TODO

p_drone.k_omega       = 3052;         % [rad/s] max angular velocity for each motor
p_drone.C_prop        = 1.2819184e-8; % proportional coeff of propeller force

p_drone.Ax            = 0; % 4*9.1785e-7;         % [kg/s]
p_drone.Ay            = 0; % 4*9.1785e-7; 
p_drone.Az            = 0; % 4*9.1785e-7; 

p_drone.drag_area     = 0.0008;         % [m^2]

p_drone.CD            = p_drone.C_prop*0.005964552; % drag constant

% p_drone.pn_T    = 0;  % initial North position
% p_drone.pe_T    = 0;  % initial East position
% p_drone.pd_T    = xyu_T(3);  % initial Down position (negative altitude)
% p_drone.vx_T    = xyu_T(4); % initial velocity along body x-axis
% p_drone.vy_T    = xyu_T(5);  % initial velocity along body y-axis
% p_drone.vz_T    = xyu_T(6);  % initial velocity along body z-axis
p_drone.phi_T   = 0;  % initial roll angle
p_drone.theta_T = 0;  % initial pitch angle
p_drone.psi_T   = 0;  % initial yaw angle
% p_drone.p_T     = xyu_T(10);  % initial body frame roll rate
% p_drone.q_T     = xyu_T(11);  % initial body frame pitch rate
% p_drone.r_T     = xyu_T(12);  % initial body frame yaw rate

p_drone.Va_T = 0;
% p_drone.alpha_T = xyu_T(14);
% p_drone.beta_T = xyu_T(15);

% p_drone.d1_T = xyu_T(16);
% p_drone.d2_T = xyu_T(17);
% p_drone.d3_T = xyu_T(18);
% p_drone.d4_T = xyu_T(19);

% Altitude state machine parameters
p_drone.altitude_take_off_zone = 5;

% Values (for reference DJI Phantom 3: 15m/s)
p_drone.Va0 = 3;        % m/s


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tuning PID loops for quadrotor

% Altitude thrust hold controller
p_drone.kp_h = 1.5;
p_drone.kd_h = 2.5;
p_drone.ki_h = 0.5;

% Roll torque hold controller
omega_n_phi     = 4;
p_drone.kp_phi  = p_drone.Jx*omega_n_phi^2;
p_drone.kd_phi  = 2*KSI_OPT*omega_n_phi*p_drone.Jx;

% Pitch torque hold controller
omega_n_theta       = 4;
p_drone.kp_theta    = p_drone.Jy*omega_n_theta^2;
p_drone.kd_theta    = 2*KSI_OPT*omega_n_theta*p_drone.Jy;

% Yaw torque hold controller
omega_n_psi     = 3;
p_drone.kp_psi  = p_drone.Jz*omega_n_psi^2;
p_drone.kd_psi  = 2*KSI_OPT*omega_n_psi*p_drone.Jz;

% p_drone.kp_psi      = 1.75/100;
% p_drone.kd_psi      = 6/100;

% vd thrust hold controller
omega_n_vd      = 5;
p_drone.kp_vd   = -2*KSI_OPT*omega_n_vd*p_drone.mass;
p_drone.ki_vd   = p_drone.Az-omega_n_vd^2*p_drone.mass;

% vn pitch hold controller
omega_n_vn      = 1;
v_hover         = p_drone.mass * 9.81 / ...
                (4 * p_drone.C_prop * p_drone.k_omega^2);
uh              = p_drone.C_prop*p_drone.k_omega^2*v_hover;
p_drone.kp_vn   = (-2*KSI_OPT*omega_n_vn*p_drone.mass + p_drone.Ax)/uh;
p_drone.ki_vn   = -(omega_n_vn^2*p_drone.mass)/uh;

% ve roll hold controller
omega_n_ve      = 1;
p_drone.kp_ve   = (2*KSI_OPT*omega_n_ve*p_drone.mass - p_drone.Ay)/uh;
p_drone.ki_ve   = (omega_n_ve^2*p_drone.mass)/uh;

% ad thrust hold controller
tau_ad                = 0.1;
p_drone.kp_ad         = 0;
p_drone.ki_ad         = (p_drone.Az-p_drone.mass/tau_ad);

% an pitch hold controller
tau_an                = 0.2;
p_drone.kp_an         = 0;
p_drone.ki_an         = (tau_an*p_drone.Ax-p_drone.mass)/(tau_an*uh);

% ae roll hold controller
tau_ae                = 0.2;
p_drone.kp_ae         = 0;
p_drone.ki_ae         = (tau_ae*p_drone.Ay+p_drone.mass)/(tau_ae*uh);

% pd thrust hold controller
p_drone.kp_pd         = 0;
p_drone.ki_pd         = p_drone.mass;
p_drone.kd_pd         = 0;

% pe roll hold controller
p_drone.kp_pe         = 0;
p_drone.ki_pe         = p_drone.mass;
p_drone.kd_pe         = 0;

% pn pitch hold controller
p_drone.kp_pn         = 0;
p_drone.ki_pn         = p_drone.mass;
p_drone.kd_pn         = 0;

% pn vn hold controller
omega_n_pn           = 1.3;
p_drone.kp_pn_cas    = omega_n_pn;
p_drone.ki_pn_cas    = 0;
p_drone.kd_pn_cas    = 0;

% pe ve hold controller
omega_n_pe          = 1.3;
p_drone.kp_pe_cas   = omega_n_pe;
p_drone.ki_pe_cas   = 0;
p_drone.kd_pe_cas   = 0;

% pd ve hold controller
omega_n_pd            = 1.3;
p_drone.kp_pd_cas     = omega_n_pd;
p_drone.ki_pd_cas     = 0;
p_drone.kd_pd_cas     = 0;

% roll p hold controller
p_drone.kp_phi_cas    = omega_n_phi;
p_drone.ki_phi_cas    = 0;
p_drone.kd_phi_cas    = 0;

% pitch q hold controller
p_drone.kp_theta_cas   = omega_n_theta;
p_drone.ki_theta_cas   = 0;
p_drone.kd_theta_cas   = 0;

% yaw r hold controller
p_drone.kp_psi_cas     = omega_n_psi;
p_drone.ki_psi_cas     = 0;
p_drone.kd_psi_cas     = 0;

% p torque hold controller
omega_n_p            = 1;
Lx                   = p_drone.l_arm * p_drone.C_prop * p_drone.k_omega^2 / p_drone.Jx;
p_drone.kp_p         = omega_n_p/Lx;
p_drone.ki_p         = 0;
p_drone.kd_p         = 0;

% q torque hold controller
omega_n_q            = 1;
Ly                   = p_drone.l_arm * p_drone.C_prop * p_drone.k_omega^2 / p_drone.Jy;
p_drone.kp_q         = omega_n_q/Ly;
p_drone.ki_q         = 0;
p_drone.kd_q         = 0;

% r torque hold controller
omega_n_r            = 1;
Lz                   = p_drone.l_arm * p_drone.C_prop * p_drone.k_omega^2 / p_drone.Jz;
p_drone.kp_r         = omega_n_r/Lz;
p_drone.ki_r         = 0;
p_drone.kd_r         = 0;

%%%%%%s%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define the maximum range of actuators
p_drone.theta_max     = deg2rad(30);
p_drone.phi_max       = deg2rad(30);
p_drone.thrust_max    = 4 * p_drone.C_prop * p_drone.k_omega^2;  % [N]
p_drone.torque_max    = 100;              % [N/m] %TODO
p_drone.vd_max        = 10;               % [m/s] 
p_drone.v_max         = 5;                % [m/s]
p_drone.p_max         = 25;               % [rad/s]
p_drone.q_max         = 25;               % [rad/s]
p_drone.r_max         = 5;                % [rad/s]


% first cut at initial conditions
p_drone.pn_T    = 0;  % initial North position
p_drone.pe_T    = 0;  % initial East position
p_drone.pd_T    = -200;  % initial Down position (negative altitude)
p_drone.vx_T    = 0; % initial velocity along body x-axis
p_drone.vy_T    = 0;  % initial velocity along body y-axis
p_drone.vz_T    = 0;  % initial velocity along body z-axis
p_drone.phi_T   = 0;  % initial roll angle
p_drone.theta_T = 0;  % initial pitch angle
p_drone.psi_T   = 0;  % initial yaw angle
p_drone.p_T     = 0;  % initial body frame roll rate
p_drone.q_T     = 0;  % initial body frame pitch rate
p_drone.r_T     = 0;  % initial body frame yaw rate

p_drone.Va_T    = 0;
p_drone.alpha_T = 0;
p_drone.beta_T  = 0;

p_drone.de_T    = 0;
p_drone.da_T    = 0;
p_drone.dr_T    = 0;
p_drone.dt_T    = 0;

run('param_sensors')