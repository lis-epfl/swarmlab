function z_hat = estimate_states(uu, time, dt, p_drone, p_physics)
% ESTIMATE_STATES: Estimate the Drone states using gyros, accels, pressure
% sensors, and GPS.
% Inputs:
%   uu: input measurements
%   p_drone: drone parameters
%
% Outputs:
%   pnhat    - estimated North position,
%   pehat    - estimated East position,
%   hhat     - estimated altitude,
%   Vahat    - estimated airspeed,
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle,
%   thetahat - estimated pitch angel,
%   chihat   - estimated course,
%   phat     - estimated roll rate,
%   qhat     - estimated pitch rate,
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed,
%   wnhat    - estimate of North wind,
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
%

% Rename inputs
y_gyro_x      = uu(1);
y_gyro_y      = uu(2);
y_gyro_z      = uu(3);
y_accel_x     = uu(4);
y_accel_y     = uu(5);
y_accel_z     = uu(6);
y_static_pres = uu(7);
y_diff_pres   = uu(8);

y_gps_n       = uu(9);
y_gps_e       = uu(10);
y_gps_h       = uu(11);
y_gps_Vg      = uu(12);
y_gps_course  = uu(13);

% Persistent variables
persistent  kf1;
%persistent  kf2;
persistent x;
persistent kf_P;

% Initilize estimated states at t=0
if time == 0
    kf1.x_prev = [p_drone.phi_T, p_drone.theta_T]';
    kf1.P_prev = zeros(2,2);
    Vg_T = p_drone.Va_T;
    chi_T = p_drone.psi_T;
    %kf2.x_prev = [P.pn_T; P.pe_T; Vg_T; chi_T; P.wn; P.we; P.psi_T];
    %kf2.P_prev = zeros(7,7);
    phi_hat     = p_drone.phi_T;
    theta_hat   = p_drone.theta_T;
    x = [0; 0];
    kf_P = [0, 0;
        0  , 0];
end



% Not estimating these states
alphahat = 0;
betahat  = 0;
bxhat    = 0;
byhat    = 0;
bzhat    = 0;

pnhat=0;
pehat=0;
hhat=0;
Vahat=0;
alphahat=0;
betahat=0;
phihat=0;
thetahat=0;
chihat=0;
phat=0;
qhat=0;
rhat=0;
Vghat=0;
wnhat=0;
wehat=0;
psihat=0;
bxhat=0;
byhat=0;
bzhat=0;


%% Estimate states by low pass filtering

phat     = lp_filter( y_gyro_x, 0, p_drone.a_gyro, dt);
qhat     = lp_filter( y_gyro_y, 0, p_drone.a_gyro, dt);
rhat     = lp_filter( y_gyro_z, 0, p_drone.a_gyro, dt);

hhat     = (p_drone.pres0 - lp_filter( y_static_pres, 0, p_drone.a_static_pres, dt))/...
    (p_physics.rho*p_physics.gravity);
Vahat    = sqrt(2/p_physics.rho*lp_filter(y_diff_pres, 0, p_drone.a_diff_pres, dt));

%thetahat = asin(lp_filter(y_accel_x, 0, P.a_accel, P.dt)/p_physics.gravity);
%phihat   = atan(lp_filter(y_accel_y, 0, P.a_accel, P.dt)/lp_filter(y_accel_z, 0, P.a_accel, P.dt));

pnhat    = lp_filter(y_gps_n, 0, p_drone.a_gps_pos, dt);
pehat    = lp_filter(y_gps_e, 0, p_drone.a_gps_pos, dt);
chihat   = lp_filter(y_gps_course, 0, p_drone.a_gps_speed, dt);
Vghat    = lp_filter(y_gps_Vg, 0, p_drone.a_gps_speed, dt);


%% Estimated states by kalman filter

% Reconstruct theta and phi measured
% theta_m  = asin((y_accel_x)/p_physics.gravity);
% phi_m    = atan(y_accel_y/y_accel_z);
% last_measure1 = [ t       phi_m;           % not really t, but time of sensor measurement
%                   t       theta_m];        % not really t, ...
%
% input_z_hat = [phihat; thetahat; phat; qhat; rhat; Vahat];
% [F1, G1, H1, u1, Q1, R1] = ekf_phi_theta( input_z_hat, P);
% [kf1] = ext_kalman_filter(F1, G1, H1, u1, Q1, R1, t, last_measure1, kf1, P);

% Extract x to output variables
% phihat = kf1.x(1);
% thetahat = kf1.x(2);
x_prev = x;
phi_hat = x(1);
theta_hat = x(2);
kf_P_prev = kf_P;


z = [y_accel_x ; y_accel_y; y_accel_z];

% Compute f, F, h and H

cp = cos(phi_hat);
sp = sin(phi_hat);
ct = cos(theta_hat);
st = sin(theta_hat);
tt = tan(theta_hat);

p = y_gyro_x;
q = y_gyro_y;
r = y_gyro_z;
Va = Vahat;
g = p_physics.gravity;

f = [  p + q*sp*tt + r*cp*tt; ...
    q*cp - r*sp];

F = [  q*cp*tt - r*sp*tt,   (q*sp-r*cp)/(ct^2); ...
    - q*sp - r*cp,          0                ];

h = [ q*Va*st + g*st; ...
    r*Va*ct - p*Va*st - g*ct*sp; ...
    -q*Va*ct - g*ct*cp];

H = [  0,                 q*Va*ct + p_physics.gravity*ct;...
    - p_physics.gravity*cp*ct,   - r*Va*st - p*Va*ct + p_physics.gravity*sp*st;...
    p_physics.gravity*sp*ct,   (q*Va+p_physics.gravity*cp)*st               ];


%% Compute prediction (a priori)

x = x_prev+dt*f;    % a priori state estimate
kf_P = kf_P_prev + (F*kf_P_prev + kf_P_prev*F' + p_drone.Q);    % a priori estimate covariance


%% Compute update (a posteriori)

if(norm(z-p_physics.gravity) < 0.5)
    z_tilde = z - H*x;              % innovation (a priori residual)
    S = p_drone.R + H*kf_P*(H');    % innovation covariance
    K = kf_P*(H')/S;                % optimal kalman gain
    x = x + K*z_tilde;              % a posteriori state estimate
    kf_P = (eye(2) - K*H)*kf_P;     % a posteriori estimate covariance
end

z_res = z - H*x;                    % a posteriori residual

phihat      = x(1);
thetahat    = x(2);


%% Output data

z_hat = [
    pnhat;...
    pehat;...
    hhat;...
    0
    0
    0
    Vahat;...
    alphahat;...
    betahat;...
    phihat;...
    thetahat;...
    chihat;...
    phat;...
    qhat;...
    rhat;...
    Vghat;...
    wnhat;...
    wehat;...
    psihat;...
    bxhat;...
    byhat;...
    bzhat
    ];


end
