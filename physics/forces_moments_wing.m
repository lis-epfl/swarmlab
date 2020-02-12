function out = forces_moments_wing(x, delta, wind, ~, p_drone, p_physics)

% FORCES_MOMENTS - Computes the forces and moments acting on the UAV.
%
% Inputs:
%   x     - state variables
%   delta - actuators values (angles,...)
%   wind  - wind parameters
%   P     - general parameters
% Outputs:
%   out
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Variables and Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

% Actuators fixed-wing
de = delta(1); % delta elevator
da = delta(2); % delta aileron
dr = delta(3); % delta rudder
dt = delta(4); % delta thrust

Rbi = Rb2i(phi,theta,psi);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wind computation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Compute forces on fixed wing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%   Aerodynamic coefficients in function of alpha
%       Lift and Drag Coefficients
CL = p_drone.CL0 + p_drone.CL_alpha * alpha;
CD = p_drone.CD0 + p_drone.CD_alpha * alpha;

%       Lift and Drag expressed in body frame
Cx     = - CD * ca       + CL * sa;
Cx_q   = - p_drone.CD_q * ca   + p_drone.CL_q  * sa;
Cx_de  = - p_drone.CD_de * ca  + p_drone.CL_de * sa;
Cz     = - CD * sa       - CL * ca;
Cz_q   = - p_drone.CD_q  * sa  - p_drone.CL_q  * ca;
Cz_de  = - p_drone.CD_de * sa  - p_drone.CL_de * ca;

% Weight
weight = p_drone.mass * p_physics.gravity * [-sin(theta);...
    cos(theta) * sin(phi);...
    cos(theta) * cos(phi)];

% Aerodynamic forces
p_dyn = 0.5 * p_physics.rho * Va^2;

if Va ~= 0
    kc = 0.5*p_drone.c/Va;
    kb = 0.5*p_drone.b/Va;
else
    kc = 0;
    kb = 0;
end

f_drag = p_dyn * p_drone.S_wing * ...
    [Cx + Cx_q*kc*q + Cx_de*de; ...
    p_drone.CY0 + p_drone.CY_beta*beta + p_drone.CY_p*kb*p + p_drone.CY_r*kb*r + p_drone.CY_da*da + p_drone.CY_dr*dr;...
    Cz + Cz_q*kc*q + Cz_de*de];

% Thrust force
f_thrust = 0.5*p_physics.rho*p_drone.S_prop*p_drone.C_prop*...
    [(p_drone.k_motor*dt)^2-Va^2; 0; 0];

% Total force = Weight + Aerodynamic forces + Thrust force
f_tot = weight + f_drag + f_thrust;

% Compute torques on drone
torque_aerodyn = p_dyn * p_drone.S_wing * ...
    [p_drone.b*(p_drone.Cl0 + p_drone.Cl_beta*beta + p_drone.Cl_p*kb*p + p_drone.Cl_r*kb*r + p_drone.Cl_da*da + p_drone.Cl_dr*dr);...
    p_drone.c*(p_drone.Cm0 + p_drone.Cm_alpha*alpha + p_drone.Cm_q*kc*q + p_drone.Cm_de*de);...
    p_drone.b*(p_drone.Cn0 + p_drone.Cn_beta*beta + p_drone.Cn_p*kb*p + p_drone.Cn_r*kb*r + p_drone.Cn_da*da + p_drone.Cn_dr*dr)];

% Thrust torque
%   Relabel thrust coefficients
torque_thrust = [-p_drone.k_TP*(p_drone.k_omega*dt)^2; 0; 0];

% Total torque
torque_tot = torque_aerodyn + torque_thrust;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Create output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

out = [f_tot; torque_tot; Va; alpha; beta; wn; we; wd];

end