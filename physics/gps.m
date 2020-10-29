function y = gps(x, dt, p_drone)
% GPS - Compute the output of gps sensor.
%
% Inputs:
%   x: state variables
%   dt: time step of the simulation
% 
% Outputs:
%   y: gps simulated data, including
%       y_gps_pn; y_gps_pe; y_gps_ph; y_gps_Vg; y_gps_course
%


%% Relabel inputs

pn      = x(1);
pe      = x(2);
pd      = x(3);
vx      = x(4);
vy      = x(5);
vz      = x(6);
phi     = x(7);
theta   = x(8);
psi     = x(9);


%% Construct Markov Process

persistent mp_n;
persistent mp_e;
persistent mp_h;
if isempty(mp_n)
    mp_n=0;
    mp_e=0;
    mp_h=0;
end
mp_n = exp(-p_drone.k_gps*dt)*mp_n + normrnd(0, p_drone.sd_gps_pn);
mp_e = exp(-p_drone.k_gps*dt)*mp_e + normrnd(0, p_drone.sd_gps_pe);
mp_h = exp(-p_drone.k_gps*dt)*mp_h + normrnd(0, p_drone.sd_gps_ph);

% Construct North, East, and altitude GPS measurements
y_gps_pn =   pn + mp_n;
y_gps_pe =   pe + mp_e;
y_gps_ph = - pd + mp_h;

Rbi = Rb2i(phi, theta, psi);
vi = Rbi * [vx; vy; vz];
vn = vi(1);
ve = vi(2);

y_gps_vn =   vn + normrnd(0, p_drone.sd_gps_vn);
y_gps_ve =   ve + normrnd(0, p_drone.sd_gps_ve);

% Construct groundspeed and course measurements
y_gps_Vg     = sqrt(y_gps_vn^2 + y_gps_ve^2);
y_gps_course = atan2(y_gps_ve, y_gps_vn);


%% Construct total output
y = [y_gps_pn; y_gps_pe; y_gps_ph; y_gps_Vg; y_gps_course];


end
