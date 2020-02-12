load('table_T.mat')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for UAV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% UAV == Zagi
%     p_drone.mass = 1.56;
%     p_drone.Jx   = 0.1147;
%     p_drone.Jy   = 0.0576;
%     p_drone.Jz   = 0.1712;
%     p_drone.Jxz  = 0.0015;


%%% UAV == Aerosonde
p_drone.mass = 13.5;
p_drone.Jx   = 0.8244;
p_drone.Jy   = 1.135;
p_drone.Jz   = 1.759;
p_drone.Jxz  = .1204;

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
p_drone.S_wing          = 0.55;     % Surface of the plane
p_drone.b              = 2.8956;
p_drone.c              = 0.18994;
p_drone.S_prop         = 0.2027;    % Surface of the propeller
p_drone.k_motor        = 80;        % k motor
p_drone.k_TP           = 0;
p_drone.k_omega        = 0;
p_drone.e              = 0.9;

p_drone.kb = @(Va) 0.5*p_drone.b/Va;
p_drone.kc = @(Va) 0.5*p_drone.c/Va;

% Longitudinal dynamics:
%   lift and drag (F_L, F_D)
p_drone.CL0            = 0.28;     % C_L_0
p_drone.CL_alpha       = 3.45;     % C_L_alpha
p_drone.CL_q           = 0.0;      % C_L_q
p_drone.CL_de          = -0.36;    % C_L_delta_e
p_drone.CD0            = 0.03;     % C_D_0
p_drone.CD_alpha       = 0.30;     % C_D_alpha
p_drone.CD_p           = 0.0437;   % C_D_p
p_drone.CD_q           = 0.0;      % C_D_q
p_drone.CD_de          = 0.0;      % C_D_delta_e

%   moment around y (m)
p_drone.Cm0            = -0.02338; % C_m_0
p_drone.Cm_alpha       = -0.38;    % C_m_alpha
p_drone.Cm_q           = -3.6;     % C_m_q
p_drone.Cm_de          = -0.5;     % C_m_delta_e

% Lateral dynamics
%   F_y
p_drone.CY0            = 0.0;
p_drone.CY_beta        = -0.98;
p_drone.CY_p           = 0.0;
p_drone.CY_r           = 0.0;      % C_Y_r
p_drone.CY_da          = 0.0;      % C_Y_delta_a
p_drone.CY_dr          = -0.17;    % C_Y_delta_r

%   moment around x (l) and z (n)
p_drone.Cl0            = 0.0;      % C_l_0
p_drone.Cl_beta        = -0.12;    % Cl_beta
p_drone.Cl_p           = -0.26;    % C_l_p
p_drone.Cl_r           = 0.14;     % C_l_r
p_drone.Cl_da          = 0.08;     % C_l_delta_aileron
p_drone.Cl_dr          = 0.105;    % C_l_delta_rudder
p_drone.Cn0            = 0.0;      % C_n_0
p_drone.Cn_beta        = 0.25;     % C_n_beta
p_drone.Cn_p           = 0.022;    % C_n_p
p_drone.Cn_r           = -0.35;    % C_n_r
p_drone.Cn_da          = 0.06;     % C_n_delta_a
p_drone.Cn_dr          = -0.032;   % C_n_delta_r


p_drone.C_prop         = 1.0;      % propeller proportional coeff fx=Sprop*Cprop*dp
p_drone.M              = 50;
p_drone.epsilon        = 0.1592;
p_drone.alpha0         = 0.4712;

p_drone.Cp0     = p_drone.gamma3 * p_drone.Cl0     + p_drone.gamma4 * p_drone.Cn0;
p_drone.Cp_beta = p_drone.gamma3 * p_drone.Cl_beta + p_drone.gamma4 * p_drone.Cn_beta;
p_drone.Cp_p    = p_drone.gamma3 * p_drone.Cl_p    + p_drone.gamma4 * p_drone.Cn_p;
p_drone.Cp_r    = p_drone.gamma3 * p_drone.Cl_r    + p_drone.gamma4 * p_drone.Cn_r;
p_drone.Cp_da   = p_drone.gamma3 * p_drone.Cl_da   + p_drone.gamma4 * p_drone.Cn_da;
p_drone.Cp_dr   = p_drone.gamma3 * p_drone.Cl_dr   + p_drone.gamma4 * p_drone.Cn_dr;
p_drone.Cr0     = p_drone.gamma4 * p_drone.Cl0     + p_drone.gamma8 * p_drone.Cn0;
p_drone.Cr_b    = p_drone.gamma4 * p_drone.Cl_beta + p_drone.gamma8 * p_drone.Cn_beta;
p_drone.Cr_p    = p_drone.gamma4 * p_drone.Cl_p    + p_drone.gamma8 * p_drone.Cn_p;
p_drone.Cr_r    = p_drone.gamma4 * p_drone.Cl_r    + p_drone.gamma8 * p_drone.Cn_r;
p_drone.Cr_da   = p_drone.gamma4 * p_drone.Cl_da   + p_drone.gamma8 * p_drone.Cn_da;
p_drone.Cr_dr   = p_drone.gamma4 * p_drone.Cl_dr   + p_drone.gamma8 * p_drone.Cn_dr;


% first cut at initial conditions
% p_drone.pn_T    = xyu_T(1);  % initial North position
% p_drone.pe_T    = xyu_T(2);  % initial East position
% p_drone.pd_T    = xyu_T(3);  % initial Down position (negative altitude)
% p_drone.vx_T    = xyu_T(4); % initial velocity along body x-axis
% p_drone.vy_T    = xyu_T(5);  % initial velocity along body y-axis
% p_drone.vz_T    = xyu_T(6);  % initial velocity along body z-axis
% p_drone.phi_T   = xyu_T(7);  % initial roll angle
% p_drone.theta_T = xyu_T(8);  % initial pitch angle
% p_drone.psi_T   = xyu_T(9);  % initial yaw angle
% p_drone.p_T     = xyu_T(10);  % initial body frame roll rate
% p_drone.q_T     = xyu_T(11);  % initial body frame pitch rate
% p_drone.r_T     = xyu_T(12);  % initial body frame yaw rate

% p_drone.Va_T = xyu_T(13);
% p_drone.alpha_T = xyu_T(14);
% p_drone.beta_T = xyu_T(15);

% p_drone.de_T = xyu_T(16);
% p_drone.da_T = xyu_T(17);
% p_drone.dr_T = xyu_T(18);
% p_drone.dt_T = xyu_T(19);

% Values for the Aerosonde UAV
p_drone.Va0 = 35;        % m/s (~85 mph)

% Trim conditions used for static PID tuning
Va_Td = 35;
gamma_Td = deg2rad(0);
R_Td = Inf;

Td = [Va_Td, gamma_Td, R_Td];

% Compute PID control
% p_drone.C = compute_pid_controller(Td, P)
[Va_idx, gamma_idx, R_idx]  = Td2idx(Td);
xyu_T = table_T(:,Va_idx, gamma_idx, R_idx);
p_drone.phi_T   = 0;  % initial roll angle
p_drone.theta_T = 0;  % initial pitch angle
p_drone.psi_T   = 0;  % initial yaw angle
p_drone.Va_T = 35;
[a,TF] = compute_tf_model(xyu_T, p_drone, p_physics);
% PVa_T = xyu_T(13);  % Extract trimmed state


% Altitude state machine parameters
p_drone.altitude_take_off_zone = 40;
p_drone.h_theta_hold_zone      = 10;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tuning PID loops

% Lateral dynamics
% Roll_aileron-hold
ephi_max = deg2rad(15);
p_drone.kp_phi = p_drone.da_max / ephi_max;
ksi_phi = KSI_OPT;
wn_phi = sqrt(p_drone.kp_phi * a.phi2);
p_drone.kd_phi = (2*ksi_phi*wn_phi - a.phi1) / a.phi2;

p_drone.tau_phi = 0.05; % [seconds] <=> 1 autopilot loop

% Course_roll-hold
W_chi_phi = 10; % Bandwith separation factor
% Between 5 and 10. Safety corresponds to 10.
wn_chi = wn_phi / W_chi_phi; %[rad/sec]
ksi_chi = KSI_OPT;
p_drone.kp_chi = 2*ksi_chi*wn_chi*p_drone.Va_T/p_physics.gravity;
p_drone.ki_chi = wn_chi^2*p_drone.Va_T/p_physics.gravity;

% Sideslip_rudder-hold
%TODO

% Longitudinal dynamics
% Pitch_elevator-hold
etheta_max  = deg2rad(10);
p_drone.kp_theta  = p_drone.de_max / etheta_max * sign(a.theta3);
wn_theta = sqrt(a.theta2 + p_drone.kp_theta*a.theta3);
ksi_theta = KSI_OPT;
if isfinite(a.theta3)
    p_drone.kd_theta  = (2*ksi_theta*wn_theta - a.theta1) / a.theta3;
else
    p_drone.kd_theta  = 0;
    disp("kd_theta is null!");
end
p_drone.tau_theta = 0.1; % [seconds] <=> 10 autopilot loops

% Altitude_pitch-hold
% Scale factor between theta_c and theta
K_theta_DC = p_drone.kp_theta*a.theta3/(a.theta2+p_drone.kp_theta*a.theta3);
W_h_theta = 10; % Between 5 and 15. Safety corresponds to 15.
wn_h = wn_theta / W_h_theta; % [rad/sec]
ksi_h = 1;
p_drone.kp_h  = 2*ksi_h*wn_h/(K_theta_DC*p_drone.Va_T);
p_drone.ki_h  = wn_h^2 / (K_theta_DC*p_drone.Va_T);

% Airspeed_thrust-hold
wn_va_dt = 10; % [rad/sec] Need to be tuned.
ksi_va_dt = 1;
p_drone.kp_va_dt = (2*ksi_va_dt*wn_va_dt - a.va1)/a.va2;
p_drone.ki_va_dt = wn_va_dt^2/a.va2;

% Airspeed_pitch-hold
W_va_theta = 7; % [rad/sec] 10 by default, can be lower.
wn_va_theta = wn_theta / W_va_theta;
ksi_va_theta = KSI_OPT;
p_drone.kp_va_theta = (a.va1 - 2*ksi_va_theta*wn_va_theta)/(K_theta_DC*p_physics.gravity);
p_drone.ki_va_theta = -wn_va_theta^2/(K_theta_DC*p_physics.gravity);

% first cut at initial conditions
p_drone.pn_T    = xyu_T(1);  % initial North position
p_drone.pe_T    = xyu_T(2);  % initial East position
p_drone.pd_T    = xyu_T(3)-200;  % initial Down position (negative altitude)
p_drone.vx_T    = xyu_T(4); % initial velocity along body x-axis
p_drone.vy_T    = xyu_T(5);  % initial velocity along body y-axis
p_drone.vz_T    = xyu_T(6);  % initial velocity along body z-axis
p_drone.phi_T   = xyu_T(7);  % initial roll angle
p_drone.theta_T = xyu_T(8);  % initial pitch angle
p_drone.psi_T   = xyu_T(9);  % initial yaw angle
p_drone.p_T     = xyu_T(10);  % initial body frame roll rate
p_drone.q_T     = xyu_T(11);  % initial body frame pitch rate
p_drone.r_T     = xyu_T(12);  % initial body frame yaw rate

p_drone.Va_T    = xyu_T(13);
p_drone.alpha_T = xyu_T(14);
p_drone.beta_T  = xyu_T(15);

p_drone.de_T    = xyu_T(16);
p_drone.da_T    = xyu_T(17);
p_drone.dr_T    = xyu_T(18);
p_drone.dt_T    = xyu_T(19);

run('param_sensors')