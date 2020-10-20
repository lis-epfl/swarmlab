function out = forces_moments_quad(x, delta, wind, ~, p_drone, p_sim, p_physics)

% FORCES_MOMENTS - Compute the forces and moments acting on the UAV, in 
% the body frame.
%
% Inputs:
%   x: state variables
%   delta: actuators values (angles,...)
%   wind: wind parameters
%   p_drone: drone parameters
%   p_sim: simulation parameters
%   p_physics: physics parmeters
%
% Outputs:
%   out:
%       F: forces in the body frame
%       M: moments in the body frame
%       Va: airspeed
%       alpha: angle of attack
%       beta: sideslip angle
%       wind: wind vector in the inertial frame
%


%% Variables and Inputs

% UAV velocity wrt inertial frame in body frame
v_xyz      = x(4:6);

% Euler angles
phi         = x(7);
theta       = x(8);
psi         = x(9);

% Rotation rates
p           = x(10);
q           = x(11);
r           = x(12);

% Actuators quadcopter
d1 = delta(1); % delta first propeller
d2 = delta(2); % second
d3 = delta(3); % third
d4 = delta(4); % fourth

Rbi = Rb2i(phi,theta,psi);


%% Wind computation

ws_ned  = wind(1:3);  % steady wind in NED frame
wg_xyz  = wind(4:6);  % gusts along body xyz axis

% Gust in NED frame = Rib * gust in Body frame
wg_ned = Rbi * wg_xyz;

% Wind in NED = steady in NED + gust in NED
w_ned = ws_ned + wg_ned;
wn = w_ned(1);
we = w_ned(2);
wd = w_ned(3);

%Steady wind in body (xyz) frame
Rib = Rbi';
ws_xyz = Rib * ws_ned;

% Wind in body frame
w_xyz = ws_xyz + wg_xyz;

% Compute air data wrt the inertial frame, in the body frame
va_xyz = v_xyz - w_xyz;
vax = va_xyz(1);
vay = va_xyz(2);
vaz = va_xyz(3);

% Airspeed norm (=V_inifity)
Va = norm(va_xyz);

if vax ~= 0
    alpha = atan2(vaz, vax);
else
    alpha = 0;
end

if Va ~=0
    beta = asin(vay / Va);
else
    beta = 0;
end

%   Relabel
ca = cos(alpha);
sa = sin(alpha);


%% Compute forces on quadcopter

weight = p_drone.mass * p_physics.gravity * [-sin(theta);...
    cos(theta)  * sin(phi);...
    cos(theta)  * cos(phi)];

f_thrust = [0, 0, - p_drone.C_prop*p_drone.k_omega^2* (d1^2+d2^2+d3^2+d4^2)]';

%     % linear drag 
%     A = [p_drone.Ax    0       0;
%         0        p_drone.Ay    0;
%         0        0       p_drone.Az];
%     f_drag = - A*v_xyz;

% Non-linear drag
f_drag = -0.5 * p_physics.rho * Va * p_drone.drag_area .* va_xyz;

% Total force
f_tot = weight + f_thrust + f_drag;

% Compute torques on quadcopter
% a_prop = p_drone.k_omega*(delta - delta_prev)./p_sim.dt;
a_prop = [0, 0, 0, 0]';
t_prop =  p_drone.CD*(p_drone.k_omega*delta).^2 + p_drone.J_prop*a_prop;

torque_thrust = [...
    p_drone.l_arm*p_drone.C_prop*p_drone.k_omega^2*(-d2^2+d4^2);
    p_drone.l_arm*p_drone.C_prop*p_drone.k_omega^2*(+d1^2-d3^2);
    - t_prop(1) + t_prop(2) - t_prop(3) + t_prop(4)];

torque_aerodyn = [0, 0, 0]';

% Total torque
torque_tot = torque_thrust + torque_aerodyn;


%% Create output

out = [f_tot; torque_tot; Va; alpha; beta; wn; we; wd];


end
