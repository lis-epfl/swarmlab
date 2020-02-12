%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generic parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if exist('app')
    DRONE_TYPE = app.drone_type;
else
    
    if ~exist('DRONE_TYPE')
        DRONE_TYPE = input(strcat('Select a drone type  among the following',...
            'fixed-wing, \n', ...
            'quadcopter: \n'),'s');
    end
    
end

if ~exist('AUTOPILOT_VERSION', 'var')
    if DRONE_TYPE == "quadcopter" || DRONE_TYPE == "point_mass"
        AUTOPILOT_VERSION = 2;
        %             AUTOPILOT_VERSION = str2double(input(strcat('What autopilot do you select? \n',...
        %                 '1 for quadcopter attitude controller,\n', ...
        %                 '2 for quadcopter speed controller,\n', ...
        %                 '3 for quadcopter acceleration controller:\n'),'s'));
    elseif DRONE_TYPE == "fixed_wing"
        AUTOPILOT_VERSION = -1;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial conditions
p_drone.pn_T    = 0;  % initial North position [m]
p_drone.pe_T    = 0;  % initial East position [m]
p_drone.pd_T    = 0;  % initial Down position (negative altitude) [m]
p_drone.vx_T    = 0;  % initial velocity along body x-axis [m/s]
p_drone.vy_T    = 0;  % initial velocity along body y-axis [m/s]
p_drone.vz_T    = 0;  % initial velocity along body z-axis [m/s]
p_drone.phi_T   = 0;  % initial roll angle [rad]
p_drone.theta_T = 0;  % initial pitch angle [rad]
p_drone.psi_T   = 0;  % initial yaw angle [rad]
p_drone.p_T     = 0;  % initial body frame roll rate [rad/s]
p_drone.q_T     = 0;  % initial body frame pitch rate [rad/s]
p_drone.r_T     = 0;  % initial body frame yaw rate [rad/s]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the wind
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_drone.wn = 0;
p_drone.we = 0;
p_drone.wd = 0;

p_drone.Lx = 200;
p_drone.Ly = 200;
p_drone.Lz = 50;

p_drone.sigmax = 1.06;
p_drone.sigmay = 1.06;
p_drone.sigmaz = 0.7;

% p_drone.turbulence = 1;
%
% p_drone.Lx50_lt = 200;
% p_drone.Ly50_lt = 200;
% p_drone.Lz50_lt = 50;
% p_drone.sigmax_50_lt = 1.06;
% p_drone.sigmay_50_lt = 1.06;
% p_drone.sigmaz_50_lt = 0.7;
%
% p_drone.Lx600_lt = 533;
% p_drone.Ly600_lt = 533;
% p_drone.Lz600_lt = 533;
% p_drone.sigmax600_lt = 1.5;
% p_drone.sigmay600_lt = 1.5;
% p_drone.sigmaz600_lt = 1.5;
%
% p_drone.Lx50_mt = 200;
% p_drone.Ly50_mt = 200;
% p_drone.Lz50_mt = 50;
% p_drone.sigmax50_mt = 1.06;
% p_drone.sigmay50_mt = 1.06;
% p_drone.sigmaz50_mt = 0.7;
%
% p_drone.Lx600_mt = 533;
% p_drone.Ly600_mt = 533;
% p_drone.Lz600_mt = 533;
% p_drone.sigmax600_mt = 1.5;
% p_drone.sigmay600_mt = 1.5;
% p_drone.sigmaz600_mt = 1.5;

p_drone.ws_ned0 = [10; 0; 0];
p_drone.Va0 = norm(p_drone.ws_ned0);        % = [vx, vy, vz]_0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Maximum performance of actuators
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_drone.de_max = deg2rad(20);
p_drone.de_teeth = 60;

p_drone.da_max = deg2rad(20);
p_drone.da_teeth = 60;

p_drone.dr_max = deg2rad(25);
p_drone.dr_teeth = 60;

p_drone.dt_max = 1;
p_drone.dt_freq = 50; % Hz

p_drone.phi_max   =   deg2rad(60);
p_drone.theta_max =   deg2rad(45);

% V: for discretization of autopilot and motor
p_drone.freq_autopilot = 200; % Hz
p_drone.freq_motor = 50; % Hz

% Definitions
KSI_OPT = 0.7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drone-type-specific parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if DRONE_TYPE ~= "point_mass"
    str = "param_";
    run(strcat(str, DRONE_TYPE));
else
    p_drone.Va_T = 5;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planning parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_drone.k_orbit = 5; % only for fixed-wing
p_drone.k_path  = 0.01;
p_drone.chi_inf = pi/2;

% Number of waypoints in data structure
p_drone.size_wpt_array = 100;
p_drone.size_waypoint_array = p_drone.size_wpt_array;
p_drone.R_min = p_drone.Va_T^2/p_physics.gravity/tan(p_drone.phi_max);
p_drone.radius = 1.3*p_drone.R_min;