function [ C ] = compute_pid_controller( Td, P )

% COMPUTE_PID_CONTROLLER

[Va_idx, gamma_idx, R_idx]  = Td2idx(Td);
xyu_T = table_T(:,Va_idx, gamma_idx, R_idx);
param_trim_chap5;
[a,TF] = compute_tf_model(xyu_T, P);
Va_T = xyu_T(13);  % Extract trimmed state

% Maximum performance of actuators
P.de_max = deg2rad(20);
P.de_teeth = 60;

P.da_max = deg2rad(20);
P.da_teeth = 60;

P.dr_max = deg2rad(25);
P.dr_teeth = 60;

P.dt_max = 1;
P.dt_freq = 50; % Hz

P.phi_max   =   deg2rad(60);
P.theta_max =   deg2rad(45);

% Altitude state machine parameters
P.altitude_take_off_zone = 40;
P.h_theta_hold_zone      = 10; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tuning PID loops
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lateral dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Roll_aileron-hold
ephi_max = deg2rad(15);
P.kp_phi = P.da_max / ephi_max;
ksi_phi = KSI_OPT;
wn_phi = sqrt(P.kp_phi * a.phi2);
P.kd_phi = (2*ksi_phi*wn_phi - a.phi1) / a.phi2;

P.tau_phi = 0.05; % [seconds] <=> 1 autopilot loop

% Course_roll-hold
W_chi_phi = 10; % Bandwith separation factor
                % Between 5 and 10. Safety corresponds to 10.
wn_chi = wn_phi / W_chi_phi; %[rad/sec]
ksi_chi = KSI_OPT;
P.kp_chi = 2*ksi_chi*wn_chi*Va_T/P.gravity;
P.ki_chi = wn_chi^2*Va_T/P.gravity;

% Sideslip_rudder-hold
% TODO

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longitudinal dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Pitch_elevator-hold
etheta_max  = deg2rad(10);
P.kp_theta  = P.de_max / etheta_max * sign(a.theta3);
wn_theta = sqrt(a.theta2 + P.kp_theta*a.theta3);
ksi_theta = KSI_OPT;
if isfinite(a.theta3)
    P.kd_theta  = (2*ksi_theta*wn_theta - a.theta1) / a.theta3;
else
    P.kd_theta  = 0;
    disp("kd_theta is null!");
end
P.tau_theta = 0.1; % [seconds] <=> 10 autopilot loops

% Altitude_pitch-hold
% Scale factor between theta_c and theta
K_theta_DC = P.kp_theta*a.theta3/(a.theta2+P.kp_theta*a.theta3);
W_h_theta = 10; % Between 5 and 15. Safety corresponds to 15. 
wn_h = wn_theta / W_h_theta; % [rad/sec]
ksi_h = 1;
P.kp_h  = 2*ksi_h*wn_h/(K_theta_DC*Va_T);
P.ki_h  = wn_h^2 / (K_theta_DC*Va_T);

% Airspeed_thrust-hold
wn_va_dt = 10; % [rad/sec] Need to be tuned.
ksi_va_dt = 1;
P.kp_va_dt = (2*ksi_va_dt*wn_va_dt - a.va1)/a.va2;
P.ki_va_dt = wn_va_dt^2/a.va2;

% Airspeed_pitch-hold
W_va_theta = 7; % [rad/sec] 10 by default, can be lower.
wn_va_theta = wn_theta / W_va_theta;
ksi_va_theta = KSI_OPT;
P.kp_va_theta = (a.va1 - 2*ksi_va_theta*wn_va_theta)/(K_theta_DC*P.gravity);
P.ki_va_theta = -wn_va_theta^2/(K_theta_DC*P.gravity);

end

