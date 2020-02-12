function [a,T] = compute_tf_model(xyu_trimmed, p_drone, p_physics)
% COMPUTE_TF_MODEL - Function computing the transfer function model
%
% Syntax: out = airdata(x)
%
% Inputs:
%   x_trim - desired trimmed state
%   u_trim - desired trimmed input delta
%   y_trim - desired trimmed air data
%   P      - parameters structure
%
% Outputs:
%   T_phi_da        - transfer function (tf) from delta_aileron to roll
%   T_chi_phi       - tf from chi to roll
%   T_theta_de      - tf from delta_elevator to pitch 
%   T_h_theta       - tf from pitch to altitude
%   T_h_Va          - tf from Va to altitude
%   T_Va_dt         - tf from delta_thrust to Va
%   T_Va_theta      - tf from pitch to Va
%   T_vy_dr         - tf from delta_rudder to vy

x_trim = xyu_trimmed(1:12);
y_trim = xyu_trimmed(13:15);
u_trim = xyu_trimmed(16:19);

% Relabel inputs
% Position in NED
% pn          = x_trim(1);
% pe          = x_trim(2);
% pd          = x_trim(3); % -altitude

% UAV velocity wrt inertial frame in body frame
% vx          = x_trim(4);
% vy          = x_trim(5);
% vz          = x_trim(6);
% v_xyz       = x_trim(4:6);

% Euler angles
% phi         = x_trim(7);
theta       = x_trim(8);
% psi         = x_trim(9);

% Rotation rates
% p           = x_trim(10);
% q           = x_trim(11);
% r           = x_trim(12);

% Actuators
de = u_trim(1); % delta elevator
% da = u_trim(2); % delta aileron
% dr = u_trim(3); % delta rudder
dt = u_trim(4); % delta thrust

% Wind
Va =    y_trim(1);
alpha = y_trim(2);
% beta =  y_trim(3);

% Compute parametrs
p_dyn = 0.5*p_physics.rho*Va^2;
kb = p_drone.kb(Va);
if isinf(kb)
    kb =0;
end
kc = p_drone.kc(Va);
if isinf(kc)
    kc =0;
end

a.phi1 = - p_dyn*p_drone.S_wing*p_drone.b*p_drone.Cp_p*kb;
a.phi2 =   p_dyn*p_drone.S_wing*p_drone.b*p_drone.Cp_da;

a.beta1 = - 0.5 * p_physics.rho * Va * p_drone.S_wing / p_drone.mass * p_drone.CY_beta;
a.beta2 =   0.5 * p_physics.rho * Va * p_drone.S_wing / p_drone.mass * p_drone.CY_dr;

a.theta1 = - p_dyn * p_drone.c * p_drone.S_wing / p_drone.Jy * p_drone.Cm_q * kc;
a.theta2 = - p_dyn * p_drone.c * p_drone.S_wing / p_drone.Jy * p_drone.Cm_alpha;
a.theta3 =   p_dyn * p_drone.c * p_drone.S_wing / p_drone.Jy * p_drone.Cm_de;

a.va1 = p_physics.rho * Va * p_drone.S_wing / p_drone.mass * ( p_drone.CD0 + p_drone.CD_alpha*alpha + p_drone.CD_de*de ) + ...
       p_physics.rho * p_drone.S_prop / p_drone.mass * p_drone.C_prop * Va;
a.va2 = p_physics.rho * p_drone.S_prop / p_drone.mass * p_drone.C_prop * p_drone.k_motor^2 * dt;
a.va3 = p_physics.gravity * cos(theta - alpha);

% Define transfer functions
T.phi_da   = tf([a.phi2],[1,a.phi1,0]);
T.chi_phi       = tf([p_physics.gravity/Va],[1,0]);
T.theta_de = tf(a.theta3,[1,a.theta1,a.theta2]);
T.h_theta       = tf([Va],[1,0]);
T.h_Va          = tf([theta],[1,0]);
T.Va_dt    = tf([a.va2],[1,a.va1]);
T.Va_theta      = tf([-a.va3],[1,a.va1]);
T.vy_dr     = tf([Va*a.beta2],[1,a.beta1]);

