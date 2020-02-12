% Gps
p_drone.sd_gps_pn = 0.21; % standard deviation (sd) 
p_drone.sd_gps_pe = 0.21;
p_drone.sd_gps_ph = 0.4;

p_drone.sd_gps_vn = 0.05; % standard deviation (sd) 
p_drone.sd_gps_ve = 0.05;
p_drone.sd_gps_vh = 0.01;
p_drone.Ts_gps = 1; % [s]
p_drone.k_gps = 1/1100;

% Gyroscope
p_drone.sd_gyro = deg2rad(0.13); % standard deviation (sd) [deg/s]

% Acceleration
p_drone.sd_accel = 0.0025*p_physics.gravity; % standard deviation (sd) [m/s^2]

% Pressure
p_drone.pres0 = 101325; % static pressure at sea level    [Pa]
p_drone.sd_static_pres = 10; % standard deviation (sd)    [Pa]
p_drone.sd_diff_pres = 2; % standard deviation (sd)       [Pa]

p_drone.bias_gyro_x = 0.01;
p_drone.bias_gyro_y = 0.01;
p_drone.bias_gyro_z = 0.01;

p_drone.a_gyro            = 0.1;
p_drone.a_static_pres     = 0.1;
p_drone.a_diff_pres       = 0.5;
p_drone.a_accel           = 0.1;
p_drone.a_gps_speed       = 0;
p_drone.a_gps_pos         = 0;

p_drone.Q = [ p_drone.sd_gyro^2, 0;...
        0,           p_drone.sd_gyro^2];
    
p_drone.R = ([ p_drone.sd_accel, 0,          0;...
        0,           p_drone.sd_accel, 0;...
        0,           0,          p_drone.sd_accel]).^2;
    
p_drone.b_chidot  = 1.2;
p_drone.b_chi     = 1;
p_drone.b_hdot    = 1.7;
p_drone.b_h       = 1.7;
p_drone.b_Va      = 8;
p_drone.gamma_max = 10;

p_drone.bias_gyro_x = 0;
p_drone.bias_gyro_y = 0;
p_drone.bias_gyro_z = 0;
