function y = sensors(x, P)
% SENSORS - Compute the output of rate gyros, accelerometers, and pressure
% sensors.
%
% Syntax: y = sensors(x, P)
%
% Inputs:
%   x - state variables
%   P - parameters
%
% Outputs:
%   y - 
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB


% Relabel inputs
%   pn      = x(1);
%   pe      = x(2);
pd      = x(3);
%   vx       = x(4);
%   vy       = x(5);
%   vz       = x(6);
phi     = x(7);
theta   = x(8);
%   psi     = x(9);
p       = x(10);
q       = x(11);
r       = x(12);
fx      = x(13);
fy      = x(14);
fz      = x(15);
%   l       = x(16);
%   m       = x(17);
%   n       = x(18);
Va      = x(19);
%   alpha   = x(20);
%   beta    = x(21);
%   wn      = x(22);
%   we      = x(23);
%   wd      = x(24);

    % Simulate rate gyros (units are rad/sec)
    y_gyro_x = p + normrnd(0, P.sd_gyro) + P.bias_gyro_x;
    y_gyro_y = q + normrnd(0, P.sd_gyro) + P.bias_gyro_y;
    y_gyro_z = r + normrnd(0, P.sd_gyro) + P.bias_gyro_z;

    % Simulate accelerometers (units of g)
    y_accel_x = fx/P.mass + p_physics.gravity*sin(theta) + normrnd(0, P.sd_accel);
    y_accel_y = fy/P.mass - p_physics.gravity*cos(theta)*sin(phi) + normrnd(0, P.sd_accel);
    y_accel_z = fx/P.mass - p_physics.gravity*cos(theta)*cos(phi) + normrnd(0, P.sd_accel);

    % Simulate pressure sensors
    y_static_pres   = P.pres0 + p_physics.rho*p_physics.gravity*pd + normrnd(0,P.sd_static_pres);
    y_diff_pres     = p_physics.rho*Va^2/2 + normrnd(0,P.sd_diff_pres);

    % Construct output vector
    y = [y_gyro_x;  y_gyro_y;  y_gyro_z;...
         y_accel_x; y_accel_y; y_accel_z; ...
         y_static_pres; y_diff_pres];

end



