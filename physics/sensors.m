function y = sensors(x, forces, airdata, p_drone, p_physics)

% SENSORS - Simulate the output of rate gyros, accelerometers, and pressure
% sensors.
%
% Syntax: y = sensors(x, forces, p_drone, p_physics)
%
% Inputs:
%   x: state variables
%   p_drone, p_physics: parameters
%
% Outputs:
%   y: simulated sensor measurements, containing
%        y_gyro_x;  y_gyro_y;  y_gyro_z;
%        y_accel_x; y_accel_y; y_accel_z;
%        y_static_pres; y_diff_pres
%


%% Relabel inputs

% State: pn, pe, pd, phi, theta, psi, p, q, r
pd      = x(3);
phi     = x(7);
theta   = x(8);
p       = x(10);
q       = x(11);
r       = x(12);

% Forces: fx, fy, fz
fx      = forces(1);
fy      = forces(2);

% Air data: va, alpha, beta, wn, we, wd
Va      = airdata(1);


%% Simulate sensor measurements

% Simulate rate gyros (units are rad/sec)
y_gyro_x = p + normrnd(0, p_drone.sd_gyro) + p_drone.bias_gyro_x;
y_gyro_y = q + normrnd(0, p_drone.sd_gyro) + p_drone.bias_gyro_y;
y_gyro_z = r + normrnd(0, p_drone.sd_gyro) + p_drone.bias_gyro_z;

% Simulate accelerometers (units of g)
y_accel_x = fx/p_drone.mass + p_physics.gravity*sin(theta) + ...
    normrnd(0, p_drone.sd_accel);
y_accel_y = fy/p_drone.mass - p_physics.gravity*cos(theta)*sin(phi) + ...
    normrnd(0, p_drone.sd_accel);
y_accel_z = fx/p_drone.mass - p_physics.gravity*cos(theta)*cos(phi) + ...
    normrnd(0, p_drone.sd_accel);

% Simulate pressure sensors
y_static_pres   = p_drone.pres0 + p_physics.rho*p_physics.gravity*pd + ...
    normrnd(0,p_drone.sd_static_pres);
y_diff_pres     = p_physics.rho*Va^2/2 + normrnd(0,p_drone.sd_diff_pres);

% Construct output vector
y = [y_gyro_x;  y_gyro_y;  y_gyro_z;...
    y_accel_x; y_accel_y; y_accel_z; ...
    y_static_pres; y_diff_pres];


end



